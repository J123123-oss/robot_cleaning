"""Contract tests for the host-side grid-line RTSP streamer."""

import importlib.util
import inspect
import sys
import threading
import types
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).parents[1]
STREAMER_SOURCE_PATH = PACKAGE_ROOT / 'rtk_nav' / 'grid_line_streamer.py'
SETUP_SOURCE_PATH = PACKAGE_ROOT / 'setup.py'
RUN_LAUNCH_SOURCE_PATH = PACKAGE_ROOT / 'launch' / 'run.launch.py'
INDOOR_LAUNCH_SOURCE_PATH = (
    PACKAGE_ROOT / 'launch' / 'camera_indoor_test.launch.py'
)
README_PATH = PACKAGE_ROOT / 'README_GRID_LINE_STREAM.md'


def _source(path):
    return path.read_text(encoding='utf-8')


def _load_streamer_module():
    """Load streamer helpers without requiring a ROS installation."""
    module_name = 'grid_line_streamer_contract_module'

    class FakeNode:
        pass

    fake_rclpy = types.ModuleType('rclpy')
    fake_rclpy_node = types.ModuleType('rclpy.node')
    fake_rclpy_node.Node = FakeNode
    fake_rclpy_qos = types.ModuleType('rclpy.qos')
    fake_rclpy_qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE='volatile')
    fake_rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST='keep_last')
    fake_rclpy_qos.QoSProfile = object
    fake_rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT='best_effort')
    fake_cv_bridge = types.ModuleType('cv_bridge')
    fake_cv_bridge.CvBridge = type('CvBridge', (), {})
    fake_sensor_msgs = types.ModuleType('sensor_msgs')
    fake_sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')
    fake_sensor_msgs_msg.Image = type('Image', (), {})

    fake_modules = {
        'rclpy': fake_rclpy,
        'rclpy.node': fake_rclpy_node,
        'rclpy.qos': fake_rclpy_qos,
        'cv_bridge': fake_cv_bridge,
        'sensor_msgs': fake_sensor_msgs,
        'sensor_msgs.msg': fake_sensor_msgs_msg,
    }
    originals = {name: sys.modules.get(name) for name in fake_modules}
    sys.modules.update(fake_modules)
    try:
        spec = importlib.util.spec_from_file_location(
            module_name, STREAMER_SOURCE_PATH
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        for name, original in originals.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original


class FakeStdin:
    """Minimal stdin pipe used to test child cleanup."""

    def __init__(self):
        self.closed = False
        self.writes = []
        self.flush_count = 0

    def write(self, payload):
        self.writes.append(payload)

    def flush(self):
        self.flush_count += 1

    def close(self):
        self.closed = True


class FakeProcess:
    """Minimal child process with observable lifecycle operations."""

    def __init__(self, stdin=None):
        self.stdin = stdin or FakeStdin()
        self.returncode = None
        self.terminated = False
        self.killed = False
        self.wait_timeout = None

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminated = True
        self.returncode = 0

    def kill(self):
        self.killed = True
        self.returncode = -9

    def wait(self, timeout=None):
        self.wait_timeout = timeout
        return self.returncode


class GridLineStreamerContractTest(unittest.TestCase):
    """Run streamer contracts with the standard-library test runner."""

    def test_ffmpeg_command_is_raw_bgr_low_latency_rtsp_tcp(self):
        module = _load_streamer_module()

        self.assertEqual(
            module.validate_rtsp_url(' rtsp://server:8554/live/grid_line '),
            'rtsp://server:8554/live/grid_line',
        )
        for invalid_url in ('', 'http://server/live/grid_line', 'rtsp://'):
            with self.assertRaises(ValueError):
                module.validate_rtsp_url(invalid_url)

        command = module.build_ffmpeg_command(
            640,
            360,
            10.0,
            '800k',
            'veryfast',
            'rtsp://server:8554/live/grid_line',
            ffmpeg_binary='/usr/bin/ffmpeg',
        )

        self.assertEqual(
            command[:4],
            ['/usr/bin/ffmpeg', '-hide_banner', '-loglevel', 'warning'],
        )
        self.assertEqual(command[command.index('-f') + 1], 'rawvideo')
        self.assertEqual(command[command.index('-pix_fmt') + 1], 'bgr24')
        self.assertEqual(command[command.index('-s') + 1], '640x360')
        self.assertEqual(command[command.index('-r') + 1], '10')
        self.assertEqual(command[command.index('-c:v') + 1], 'libx264')
        self.assertEqual(command[command.index('-tune') + 1], 'zerolatency')
        self.assertEqual(
            command[-5:],
            ['-f', 'rtsp', '-rtsp_transport', 'tcp',
             'rtsp://server:8554/live/grid_line'],
        )
        self.assertNotIn('shell=True', inspect.getsource(module))

    def test_latest_frame_callback_replaces_old_pending_frame(self):
        module = _load_streamer_module()
        node = module.GridLineStreamer.__new__(module.GridLineStreamer)
        node.frame_condition = threading.Condition()
        node.latest_message = None
        node.latest_generation = 0

        node.image_callback('frame-1')
        node.image_callback('frame-2')

        self.assertEqual(node.latest_message, 'frame-2')
        self.assertEqual(node.latest_generation, 2)

    def test_start_process_uses_no_shell_and_stop_is_bounded(self):
        module = _load_streamer_module()
        node = module.GridLineStreamer.__new__(module.GridLineStreamer)
        node.fps = 10.0
        node.bitrate = '800k'
        node.preset = 'veryfast'
        node.rtsp_url = 'rtsp://server:8554/live/grid_line'
        node.ffmpeg_path = '/usr/bin/ffmpeg'
        node.stop_event = threading.Event()
        node.process_lock = threading.Lock()
        node.process = None
        process = FakeProcess()
        calls = []
        original_popen = module.subprocess.Popen

        def fake_popen(command, **kwargs):
            calls.append((command, kwargs))
            return process

        module.subprocess.Popen = fake_popen
        try:
            self.assertIs(node.start_process(640, 360), process)
        finally:
            module.subprocess.Popen = original_popen

        self.assertEqual(calls[0][0][-5:], [
            '-f', 'rtsp', '-rtsp_transport', 'tcp',
            'rtsp://server:8554/live/grid_line',
        ])
        self.assertFalse(calls[0][1]['shell'])
        node.stop_process(process)
        self.assertTrue(process.stdin.closed)
        self.assertTrue(process.terminated)
        self.assertEqual(process.wait_timeout, 1.0)

    def test_worker_restarts_after_broken_pipe_without_frame_backlog(self):
        module = _load_streamer_module()
        node = module.GridLineStreamer.__new__(module.GridLineStreamer)
        node.fps = 1000.0
        node.bitrate = '800k'
        node.preset = 'veryfast'
        node.rtsp_url = 'rtsp://server:8554/live/grid_line'
        node.ffmpeg_path = '/usr/bin/ffmpeg'
        node.reconnect_sec = 0.0
        node.stop_event = threading.Event()
        node.frame_condition = threading.Condition()
        node.process_lock = threading.Lock()
        node.process = None
        node.latest_message = object()
        node.latest_generation = 1
        node.bridge = types.SimpleNamespace(
            imgmsg_to_cv2=lambda _message, desired_encoding: np.zeros(
                (2, 3, 3), dtype=np.uint8
            )
        )
        node.get_logger = lambda: types.SimpleNamespace(
            info=lambda *_args, **_kwargs: None,
            warning=lambda *_args, **_kwargs: None,
            error=lambda *_args, **_kwargs: None,
        )

        class BrokenPipeStdin(FakeStdin):
            def write(self, _payload):
                raise BrokenPipeError('connection closed')

        first = FakeProcess(BrokenPipeStdin())

        class StopAfterWriteStdin(FakeStdin):
            def write(self, payload):
                super().write(payload)
                node.stop_event.set()

        second = FakeProcess(StopAfterWriteStdin())
        processes = [first, second]
        commands = []
        original_popen = module.subprocess.Popen

        def fake_popen(command, **_kwargs):
            commands.append(command)
            return processes.pop(0)

        module.subprocess.Popen = fake_popen
        try:
            node.stream_worker()
        finally:
            module.subprocess.Popen = original_popen

        self.assertEqual(len(commands), 2)
        self.assertTrue(first.terminated)
        self.assertEqual(second.stdin.writes, [b'\x00' * 18])

    def test_source_uses_depth_one_best_effort_and_recovers(self):
        source = _source(STREAMER_SOURCE_PATH)

        for required in (
            "'/grid_line/detected_image'",
            'latest_message',
            'latest_generation',
            'threading.Condition',
            'HistoryPolicy.KEEP_LAST',
            'depth=1',
            'ReliabilityPolicy.BEST_EFFORT',
            'reconnect_sec',
            'BrokenPipeError',
            'ValueError',
            'process.terminate()',
            'destroy_node',
        ):
            self.assertIn(required, source)

    def test_setup_and_launch_register_opt_in_streamer(self):
        setup_source = _source(SETUP_SOURCE_PATH)
        self.assertIn(
            'grid_line_streamer = rtk_nav.grid_line_streamer:main',
            setup_source,
        )

        for launch_path in (RUN_LAUNCH_SOURCE_PATH, INDOOR_LAUNCH_SOURCE_PATH):
            source = _source(launch_path)
            for argument in (
                'enable_grid_line_stream',
                'grid_line_stream_rtsp_url',
                'grid_line_stream_fps',
                'grid_line_stream_bitrate',
                'grid_line_stream_preset',
                'grid_line_stream_reconnect_sec',
                'ffmpeg_path',
            ):
                self.assertIn(argument, source)
            self.assertIn("executable='grid_line_streamer'", source)
            self.assertIn('IfCondition(LaunchConfiguration', source)
            self.assertRegex(
                source,
                r'default_value=TextSubstitution\(text=[\'\"]true[\'\"]\)',
            )
            self.assertIn('rtsp://127.0.0.1:8554/live/grid_line', source)

    def test_readme_documents_rtsp_and_http_flv_roles(self):
        readme_source = _source(README_PATH)
        for required in (
            'ros2 run rtk_nav grid_line_streamer',
            'rtsp://media-server:8554/live/grid_line',
            'http://media-server:8080/live/grid_line.live.flv',
            'publish_debug_images:=true',
            'sudo apt install ffmpeg',
            'HTTP-FLV',
            'reconnect',
        ):
            self.assertIn(required, readme_source)


if __name__ == '__main__':
    unittest.main()
