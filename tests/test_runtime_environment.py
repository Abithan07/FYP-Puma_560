"""The project interpreter must survive child processes and ROS env shebangs."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class RuntimeEnvironmentTests(unittest.TestCase):
    def select_python(self, project_runtime, override=False):
        with tempfile.TemporaryDirectory(prefix='puma python runtime ') as tmp:
            root = Path(tmp)
            scripts = root / 'arm_bot/scripts'
            scripts.mkdir(parents=True)
            shutil.copy(ROOT / 'arm_bot/scripts/runtime_environment.sh', scripts)
            fallback = root / 'system bin/python3'
            fallback.parent.mkdir()
            fallback.touch(); fallback.chmod(0o755)
            chosen = root / '.venv-ros/bin/python3'
            if project_runtime:
                chosen.parent.mkdir(parents=True)
                chosen.touch(); chosen.chmod(0o755)
            env = dict(os.environ, PATH=str(fallback.parent) + os.pathsep + os.environ['PATH'])
            env.pop('PUMA_PYTHON', None)
            if override:
                env['PUMA_PYTHON'] = str(fallback)
            result = subprocess.run(
                ['bash', '-euc', 'source "$1"; printf "%s\\n" "$PUMA_PYTHON"; command -v python3',
                 'runtime-test', str(scripts / 'runtime_environment.sh')],
                env=env, capture_output=True, text=True, timeout=10,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            expected = chosen if project_runtime and not override else fallback
            self.assertEqual(result.stdout.splitlines(), [str(expected), str(expected)])

    def test_project_runtime_is_preferred(self):
        self.select_python(project_runtime=True)

    def test_explicit_python_override_is_respected(self):
        self.select_python(project_runtime=True, override=True)

    def test_existing_python_is_used_when_project_runtime_is_absent(self):
        self.select_python(project_runtime=False)
