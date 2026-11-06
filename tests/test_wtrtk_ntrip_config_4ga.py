import importlib.util
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from types import ModuleType

SCRIPT = Path(__file__).parents[1] / "tools" / "wtrtk_ntrip_config_4ga.py"


def load_module():
    serial = ModuleType("serial")
    serial.SerialException = type("SerialException", (Exception,), {})
    serial.Serial = object
    previous_serial = sys.modules.get("serial")
    sys.modules["serial"] = serial
    try:
        spec = importlib.util.spec_from_file_location("wtrtk_ntrip_config", SCRIPT)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return module
    finally:
        if previous_serial is None:
            sys.modules.pop("serial", None)
        else:
            sys.modules["serial"] = previous_serial


class ScriptedSerial:
    timeout = 0.01

    def __init__(self, *lines):
        self.lines = list(lines)
        self.writes = []

    def reset_input_buffer(self):
        pass

    def write(self, packet):
        self.writes.append(packet)

    def flush(self):
        pass

    def readline(self):
        return self.lines.pop(0) if self.lines else b""

    def read(self, size):
        return self.lines.pop(0) if self.lines else b""


class WtrtkNtripConfigTests(unittest.TestCase):
    def test_at_command_uses_crlf(self):
        module = load_module()

        self.assertEqual(module.at_command("AT+ADDR?"), b"AT+ADDR?\r\n")

    def test_parse_addr_response(self):
        module = load_module()

        self.assertEqual(module.parse_addr_response("+ADDR=example.com,8002"), ("example.com", 8002))

    def test_parse_cors_response_preserves_commas_in_password(self):
        module = load_module()

        self.assertEqual(module.parse_cors_response("+CORS=user,pa,ss"), ("user", "pa,ss"))

    def test_validate_config_limits(self):
        module = load_module()
        config = module.Config("a" * 32, 8002, "user", "password", "BOOT2")

        with self.assertRaises(ValueError):
            module.validate_config(config)

    def test_read_at_response_skips_unsolicited_lines(self):
        module = load_module()
        serial = ScriptedSerial(
            b"$WTRTK,0,0,0\r\n",
            b"$GNGGA,....\r\n",
            b"+ADDR=example.com,8002\r\n",
        )

        response = module.read_at_response(serial, ("+ADDR=",))

        self.assertEqual(response, "+ADDR=example.com,8002")

    def test_read_at_response_skips_binary_wtraw_stream(self):
        module = load_module()
        serial = ScriptedSerial(
            b"$WTRAW,}\xa1\x02\x00\xf2\xff!\x08\x00\x00\r\x00\x06\x00",
            b"\x82\xa1\x02\x00\xf3\xff!\x08OK\r\n",
        )

        self.assertEqual(module.read_at_response(serial, ("OK",)), "OK")

    def test_read_config_rejects_error_response(self):
        module = load_module()
        serial = ScriptedSerial(b"+CFG\r\n", b"ERROR\r\n")

        with self.assertRaises(RuntimeError):
            module.read_config(serial)

    def test_read_config_returns_current_fields(self):
        module = load_module()
        serial = ScriptedSerial(
            b"+CFG\r\n",
            b"+ADDR=example.com,8002\r\n",
            b"+CORS=user,pa,ss\r\n",
            b"+MP=BOOT2\r\n",
            b"+EXIT\r\n",
        )

        self.assertEqual(
            module.read_config(serial),
            {
                "domain": "example.com",
                "port": 8002,
                "username": "user",
                "password": "pa,ss",
                "mountpoint": "BOOT2",
            },
        )

    def test_read_config_sends_exit_without_waiting_for_response(self):
        module = load_module()
        serial = ScriptedSerial(
            b"+CFG\r\n",
            b"+ADDR=example.com,8002\r\n",
            b"+CORS=user,password\r\n",
            b"+MP=BOOT2\r\n",
            b"$WTRAW,A?\r\n",
        )

        values = module.read_config(serial)

        self.assertEqual(values["mountpoint"], "BOOT2")
        self.assertEqual(serial.writes[-1], b"AT+EXIT\r\n")

    def test_configure_sends_4gac_at_commands(self):
        module = load_module()
        serial = ScriptedSerial(
            b"OK\r\n",
            b"+CFG\r\n",
            b"+ADDR=example.com,8002\r\n",
            b"+CORS=user,password\r\n",
            b"+MP=BOOT2\r\n",
            b"OK\r\n",
            b"+NETRESET\r\n",
            b"+EXIT\r\n",
        )
        config = module.Config("example.com", 8002, "user", "password", "BOOT2")

        module.configure(serial, config, False)

        self.assertEqual(
            serial.writes,
            [
                b"AT+CFG\r\n",
                b"AT+ADDR=example.com,8002\r\n",
                b"AT+CORS=user,password\r\n",
                b"AT+MP=BOOT2\r\n",
                b"AT+SAVE\r\n",
                b"AT+NETRESET\r\n",
                b"AT+EXIT\r\n",
            ],
        )

    def test_dry_run_prints_at_commands_without_serial(self):
        module = load_module()
        output = StringIO()
        config = module.Config("example.com", 8002, "user", "password", "BOOT2")

        with redirect_stdout(output):
            module.configure(None, config, True)

        self.assertIn("TX AT+ADDR=example.com,8002", output.getvalue())

    def test_backup_writes_json_with_current_config(self):
        module = load_module()
        serial = ScriptedSerial(
            b"+CFG\r\n",
            b"+ADDR=example.com,8002\r\n",
            b"+CORS=user,password\r\n",
            b"+MP=BOOT2\r\n",
            b"+EXIT\r\n",
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "backup.json"
            module.backup_config(serial, str(path))
            self.assertEqual(
                json.loads(path.read_text(encoding="utf-8")),
                {
                    "domain": "example.com",
                    "port": 8002,
                    "username": "user",
                    "password": "password",
                    "mountpoint": "BOOT2",
                },
            )

    def test_backup_only_requires_no_configuration_arguments(self):
        module = load_module()
        previous_argv = sys.argv
        sys.argv = [str(SCRIPT), "--backup", "backup.json"]
        try:
            args = module.parse_args()
        finally:
            sys.argv = previous_argv

        self.assertEqual(args.backup, "backup.json")
        self.assertIsNone(args.domain)
        self.assertIsNone(args.ntrip_port)
        self.assertIsNone(args.username)
        self.assertIsNone(args.mountpoint)

    def test_default_baud_matches_wtrtk_4ga_device(self):
        module = load_module()
        previous_argv = sys.argv
        sys.argv = [str(SCRIPT), "--backup", "backup.json"]
        try:
            args = module.parse_args()
        finally:
            sys.argv = previous_argv

        self.assertEqual(args.baud, 460800)

    def test_configuration_still_requires_ntrip_arguments(self):
        module = load_module()
        previous_argv = sys.argv
        sys.argv = [str(SCRIPT)]
        try:
            with self.assertRaises(SystemExit) as exc_info:
                module.parse_args()
        finally:
            sys.argv = previous_argv

        self.assertEqual(exc_info.exception.code, 2)

    def test_restore_accepts_legacy_4ga_field_names(self):
        module = load_module()
        previous_argv = sys.argv
        sys.argv = [str(SCRIPT), "--restore", "legacy.json"]
        try:
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "legacy.json"
                path.write_text(json.dumps({
                    "serverDomain": "103.143.19.54",
                    "serverIp": "103.143.19.54",
                    "serverPort": "8002",
                    "account": "WT5100001603",
                    "password": "4559",
                    "mountpoint": "Auto",
                }), encoding="utf-8")
                sys.argv = [str(SCRIPT), "--restore", str(path)]
                args = module.parse_args()
                with redirect_stdout(StringIO()):
                    # main() is dry-run by default, so no serial device is opened.
                    self.assertEqual(module.main(), 0)
        finally:
            sys.argv = previous_argv


