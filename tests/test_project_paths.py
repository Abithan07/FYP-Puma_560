"""Asset discovery must survive a renamed checkout and ROS symlink installs."""
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('project_paths', ROOT / 'arm_bot/src/scripts/shared/project_paths.py')
paths = importlib.util.module_from_spec(SPEC)
with patch.dict(os.environ, {}, clear=True):
    SPEC.loader.exec_module(paths)


class ProjectPathTests(unittest.TestCase):
    def test_real_checkout(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(paths.find_project_root(), ROOT)

    def test_renamed_checkout_and_symlink_install(self):
        with tempfile.TemporaryDirectory(prefix='puma checkout ') as tmp:
            root = Path(tmp) / 'renamed project'
            source = root / 'arm_bot/src/scripts'
            source.mkdir(parents=True)
            (source.parent / 'package.xml').touch()
            script = source / 'project_paths.py'
            script.touch()
            install = Path(tmp) / 'install/lib/arm_bot'
            install.mkdir(parents=True)
            link = install / script.name
            link.symlink_to(script)
            with patch.dict(os.environ, {}, clear=True):
                self.assertEqual(paths.find_project_root(link), root)

    def test_explicit_checkout_for_copied_install(self):
        with patch.dict(os.environ, {'PUMA_PROJECT_ROOT': str(ROOT)}, clear=True):
            self.assertEqual(paths.find_project_root('/unrelated/install/script.py'), ROOT)

    def test_bad_override_has_actionable_error(self):
        with patch.dict(os.environ, {'PUMA_PROJECT_ROOT': '/missing-checkout'}, clear=True):
            with self.assertRaisesRegex(RuntimeError, 'PUMA_PROJECT_ROOT'):
                paths.find_project_root()

    def test_missing_checkout_has_actionable_error(self):
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaisesRegex(RuntimeError, 'Set PUMA_PROJECT_ROOT'):
                paths.find_project_root('/tmp/unrelated/script.py')
