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
    def test_preparation_flag_does_not_open_hardware_in_dry_run(self):
        module = load_module()
        argv = [
            "tool", "--prepare-4ga", "--domain", "example.com",
            "--ntrip-port", "8002", "--username", "user",
            "--password", "pass", "--mountpoint", "MP",
        ]
        with patch.object(sys, "argv", argv), patch.object(module.serial, "Serial") as serial_open:
            self.assertEqual(module.main(), 0)
        serial_open.assert_not_called()

    def test_optional_4ga_preparation_closes_port_before_230400_open(self):
        module = load_module()
        preparation_port = Mock(timeout=3.0)
        preparation_port.__enter__ = Mock(return_value=preparation_port)
        preparation_port.__exit__ = Mock(return_value=False)
        configuration_port = Mock()
        events = []
        preparation_port.__exit__.side_effect = lambda *args: events.append("closed")

        def open_port(device, baud, **kwargs):
            events.append(baud)
            return preparation_port if baud == 460800 else configuration_port

        with patch.object(module.serial, "Serial", side_effect=open_port), \
                patch.object(module, "read_at_response", side_effect=RuntimeError("timeout")):
            port = module.open_serial("/dev/WTRTK", 230400, prepare_4ga=True)
        self.assertIs(port, configuration_port)
        self.assertEqual(events, [460800, "closed", 230400])
        self.assertEqual(preparation_port.write.call_args_list, [
            unittest.mock.call(b"AT+CFG\r\n"),
            unittest.mock.call(b"AT+ADDR?\r\n"),
            unittest.mock.call(b"AT+EXIT\r\n"),
        ])

    def test_default_open_never_uses_preparation_baud(self):
        module = load_module()
        with patch.object(module.serial, "Serial") as serial_open:
            module.open_serial("/dev/WTRTK", 230400)
        serial_open.assert_called_once_with(
            "/dev/WTRTK", 230400, timeout=module.DEFAULT_TIMEOUT, write_timeout=1,
        )

    def test_interrupted_preparation_sends_exit_without_opening_configuration_port(self):
        module = load_module()
        port = Mock(timeout=3.0)
        port.__enter__ = Mock(return_value=port)
        port.__exit__ = Mock(return_value=False)
        with patch.object(module.serial, "Serial", return_value=port) as serial_open, \
                patch.object(module, "read_at_response", side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                module.open_serial("/dev/WTRTK", 230400, prepare_4ga=True)
        self.assertEqual(serial_open.call_count, 1)
        self.assertEqual(port.write.call_args_list, [
            unittest.mock.call(b"AT+CFG\r\n"),
            unittest.mock.call(b"AT+EXIT\r\n"),
        ])
        port.__exit__.assert_called_once()

    def test_4ga_preparation_reads_like_original_tool(self):
        module = load_module()
        port = Mock(timeout=3.0)
        port.read.return_value = b"+CFG\r\n"
        self.assertEqual(
            module.read_at_response(port, ("+CFG",), read_size=64), "+CFG",
        )
        port.read.assert_called_once_with(64)

    def test_delayed_cfg_reply_does_not_cause_premature_resend(self):
        module = load_module()
        clock = [0.0]
        port = Mock(timeout=0.3, in_waiting=64)
        chunks = iter([b"$WTRTK,0\r\n", b"", b"", b"+CFG\r\n"])

        def read(_size):
            clock[0] += 0.3
            return next(chunks)

        port.read.side_effect = read
        with patch.object(module.time, "monotonic", side_effect=lambda: clock[0]), \
                patch.object(module.time, "sleep"):
            module.enter_config(port)
        self.assertEqual(port.write.call_args_list, [
            unittest.mock.call(b"AT+EXIT\r\n"),
            unittest.mock.call(b"AT+CFG\r\n"),
        ])
        self.assertGreaterEqual(clock[0], 1.2)

    def test_cfg_wait_timeout_cli_is_configurable(self):
        module = load_module()
        with patch.object(sys, "argv", ["tool", "--backup", "out.json", "--cfg-timeout", "8"]):
            self.assertEqual(module.parse_args().cfg_timeout, 8.0)

    def test_cfg_wait_timeout_rejects_nonpositive_and_nonfinite_values(self):
        module = load_module()
        for value in ("0", "-1", "nan", "inf"):
            with self.subTest(value=value), \
                    patch.object(sys, "argv", ["tool", "--backup", "out.json", "--cfg-timeout", value]), \
                    patch.object(sys, "stderr"):
                with self.assertRaises(SystemExit) as error:
                    module.parse_args()
                self.assertEqual(error.exception.code, 2)

    def test_standalone_error_is_reported_before_timeout(self):
        module = load_module()
        port = Mock(timeout=0.3, in_waiting=6)
        port.read.side_effect = [b"ERROR\n", AssertionError("ERROR was ignored")]
        with self.assertRaisesRegex(RuntimeError, "设备返回错误: ERROR"):
            module.read_at_response(port, ("+CFG",), timeout=0.5)

    def test_cfg_timeout_includes_actual_six_received_bytes(self):
        module = load_module()
        clock = [0.0]
        port = Mock(timeout=0.3, in_waiting=6)
        chunks = iter([b"\x01\x02\x03\x04\x05\x06", b""])

        def read(_size):
            clock[0] += 0.3
            return next(chunks)

        port.read.side_effect = read
        with patch.object(module.time, "monotonic", side_effect=lambda: clock[0]):
            with self.assertRaisesRegex(RuntimeError, "01 02 03 04 05 06"):
                module.read_at_response(port, ("+CFG",), timeout=0.5)

    def test_parameter_timeout_does_not_dump_credentials(self):
        module = load_module()
        clock = [0.0]
        port = Mock(timeout=0.3, in_waiting=64)

        def read(_size):
            clock[0] += 0.3
            return b"+CORS=user,secret"

        port.read.side_effect = read
        with patch.object(module.time, "monotonic", side_effect=lambda: clock[0]):
            with self.assertRaises(RuntimeError) as error:
                module.read_at_response(port, ("+CORS=",), timeout=0.5)
        self.assertNotIn("secret", str(error.exception))
        self.assertNotIn("73 65 63 72 65 74", str(error.exception))

    def test_cfg_reply_split_across_retry_windows_is_not_lost(self):
        module = load_module()
        clock = [0.0]
        port = Mock(timeout=0.3, in_waiting=64)
        chunks = iter([b"\xff$WTRTK,0\r\n+C", b"", b"FG\r\n"])

        def read(_size):
            clock[0] += 0.26
            try:
                return next(chunks)
            except StopIteration:
                raise AssertionError("CFG acknowledgement was lost")

        port.read.side_effect = read
        with patch.object(module.time, "monotonic", side_effect=lambda: clock[0]), \
                patch.object(module.time, "sleep"):
            module.enter_config(port, cfg_timeout=0.5)
        self.assertEqual(port.write.call_args_list, [
            unittest.mock.call(b"AT+EXIT\r\n"),
            unittest.mock.call(b"AT+CFG\r\n"),
            unittest.mock.call(b"AT+CFG\r\n"),
        ])
        # Only the startup EXIT and first CFG may discard old input.
        self.assertEqual(port.reset_input_buffer.call_count, 2)

    def test_cfg_timeout_distinguishes_missing_serial_input(self):
        module = load_module()
        clock = [0.0]
        port = Mock(timeout=0.3, in_waiting=0)

        def read(_size):
            clock[0] += 0.3
            return b""

        port.read.side_effect = read
        with patch.object(module.time, "monotonic", side_effect=lambda: clock[0]):
            with self.assertRaisesRegex(RuntimeError, "接收字节数: 0"):
                module.read_at_response(port, ("+CFG",), timeout=0.5)

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

    def test_cfg_waits_five_seconds_and_preserves_late_responses(self):
        module = load_module()
        with patch.object(module, "send_at", side_effect=[RuntimeError("timeout"), "+CFG"]) as send, \
                patch.object(module.time, "sleep"), patch.object(module, "exit_config"):
            module.enter_config(object())
        self.assertEqual(send.call_count, 2)
        for call in send.call_args_list:
            self.assertEqual(call.kwargs["response_timeout"], 5.0)
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
