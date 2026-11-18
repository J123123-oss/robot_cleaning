"""Regression tests for the 4GAC configuration-mode handshake."""
import importlib.util
import sys
import unittest
from pathlib import Path
from types import ModuleType
from unittest.mock import Mock, patch


def load_module():
    serial_stub = ModuleType("serial")
    serial_stub.SerialException = type("SerialException", (Exception,), {})
    serial_stub.Serial = object
    path = Path(__file__).parents[1] / "tools" / "wtrtk_ntrip_config_4gac.py"
    spec = importlib.util.spec_from_file_location("wtrtk_4gac_handshake_test", path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {"serial": serial_stub, spec.name: module}):
        spec.loader.exec_module(module)
    return module


class ConfigHandshakeTests(unittest.TestCase):
    def test_configure_waits_for_cfg_before_sending_settings(self):
        module = load_module()
        commands = []
        attempts = 0

        def send(_ser, command, _expected, _dry_run=False, **_kwargs):
            nonlocal attempts
            commands.append(command)
            if command == "AT+CFG":
                attempts += 1
                if attempts <= 4:
                    raise RuntimeError("timeout")
                return "+CFG"
            return "OK"

        with patch.object(module, "send_at", side_effect=send), \
                patch.object(module.time, "sleep"), patch.object(module, "exit_config"):
            module.configure(object(), module.Config("example.com", 8002, "user", "pass", "MP"), False)
        self.assertEqual(commands[:5], ["AT+CFG"] * 5)
        self.assertEqual(commands[5], "AT+ADDR=example.com,8002")

    def test_ctrl_c_during_cfg_never_sends_settings(self):
        module = load_module()
        with patch.object(module, "send_at", side_effect=[RuntimeError("timeout"), KeyboardInterrupt]) as send, \
                patch.object(module.time, "sleep"), patch.object(module, "exit_config") as exit_config:
            with self.assertRaises(KeyboardInterrupt):
                module.configure(object(), module.Config("example.com", 8002, "user", "pass", "MP"), False)
        self.assertTrue(all(call.args[1] == "AT+CFG" for call in send.call_args_list))
        self.assertEqual(exit_config.call_count, 2)

    def test_read_config_exits_mode_when_cfg_wait_is_interrupted(self):
        module = load_module()
        port = object()
        with patch.object(module, "send_at", side_effect=KeyboardInterrupt), \
                patch.object(module, "exit_config") as exit_config:
            with self.assertRaises(KeyboardInterrupt):
                module.read_config(port)
        self.assertEqual(exit_config.call_args_list, [
            unittest.mock.call(port, False), unittest.mock.call(port),
        ])

    def test_ctrl_c_sends_exit_on_serial_before_propagating(self):
        module = load_module()
        port = Mock()
        with patch.object(module, "send_at", side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                module.configure(port, module.Config("example.com", 8002, "user", "pass", "MP"), False)
        self.assertEqual(port.write.call_args_list, [unittest.mock.call(b"AT+EXIT\r\n")] * 2)
        self.assertEqual(port.flush.call_count, 2)

    def test_dry_run_does_not_wait_for_hardware(self):
        module = load_module()
        with patch.object(module, "send_at") as send, patch.object(module, "exit_config") as exit_config:
            module.enter_config(None, True)
        exit_config.assert_called_once_with(None, True)
        send.assert_called_once_with(None, "AT+CFG", ("+CFG",), True)

    def test_cfg_uses_short_window_and_preserves_late_responses(self):
        module = load_module()
        with patch.object(module, "send_at", side_effect=[RuntimeError("timeout"), "+CFG"]) as send, \
                patch.object(module.time, "sleep"), patch.object(module, "exit_config"):
            module.enter_config(object())
        self.assertEqual(send.call_count, 2)
        for call in send.call_args_list:
            self.assertEqual(call.kwargs["response_timeout"], 0.5)
            self.assertEqual(call.kwargs["retries"], 1)
        self.assertTrue(send.call_args_list[0].kwargs["clear_input"])
        self.assertFalse(send.call_args_list[1].kwargs["clear_input"])

    def test_startup_exit_precedes_cfg_and_waits_for_mode_switch(self):
        module = load_module()
        port = Mock()
        events = []
        port.write.side_effect = lambda packet: events.append(packet)
        with patch.object(module, "send_at", side_effect=lambda *args, **kwargs: events.append(b"AT+CFG")), \
                patch.object(module.time, "sleep", side_effect=lambda seconds: events.append(seconds)):
            module.enter_config(port)
        self.assertEqual(events, [b"AT+EXIT\r\n", 0.5, b"AT+CFG", 0.2])


if __name__ == "__main__":
    unittest.main()
