from pathlib import Path


def test_motor_control_bridges_resume_without_switching_state():
    source = Path("src/motor_control/motor_control/motor_control.py").read_text(
        encoding="utf-8"
    )
    assert "/rtk/resume_progress" in source
    assert 'command == "RESUME"' in source
    branch = source[source.index('command == "RESUME"') : source.index('command == "WAIT_RTK"')]
    assert "resume_progress_pub.publish" in branch
    assert "switch_state" not in branch
