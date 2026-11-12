"""Run actual page-script regressions in a bounded DOM fixture, without a browser."""
from pathlib import Path
import shutil
import subprocess

import pytest


def test_joint_unit_controls():
    node = shutil.which('node')
    if not node:
        pytest.skip('Node.js is required for page-script checks')
    test_dir = Path(__file__).resolve().parent
    subprocess.run([node, str(test_dir / 'web_units_check.js')], check=True, timeout=15)
