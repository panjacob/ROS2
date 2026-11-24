"""Exercise first page load with missing dynamic elements, as in a real browser."""

from pathlib import Path
import shutil
import subprocess

import pytest


def test_first_poll_builds_joint_controls_before_updating_them():
    node = shutil.which('node')
    if node is None:
        pytest.skip('Node.js is required for page-script checks')
    script = Path(__file__).with_name('web_startup_check.js')
    subprocess.run([node, str(script)], check=True, timeout=15)
