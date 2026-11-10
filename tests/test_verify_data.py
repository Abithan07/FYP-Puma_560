"""Published data checks must detect incomplete or modified clones."""
import contextlib
import csv
import hashlib
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('puma_verify_data', ROOT / 'tools/verify_data.py')
verifier = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verifier)


class DataVerificationTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        (self.root / 'data').mkdir()

    def record(self, name, content=b'original', tracked='False'):
        path = self.root / 'data' / name
        path.write_bytes(content)
        return {'current_path': f'data/{name}', 'bytes': len(content),
                'sha256': hashlib.sha256(content).hexdigest(), 'tracked': tracked}

    def manifest(self, records, name='DATASET_MANIFEST.csv'):
        path = self.root / 'data' / name
        with path.open('w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=records[0].keys())
            writer.writeheader()
            writer.writerows(records)
        return path

    def test_published_check_includes_previously_untracked_inputs(self):
        records = [self.record('tracked.csv', tracked='True'), self.record('local.csv')]
        self.manifest(records)
        (self.root / 'data/local.csv').unlink()
        with patch.object(verifier, 'ROOT', self.root), contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(verifier.main(['--datasets']), 1)

    def test_hash_size_and_missing_file_failures(self):
        records = [self.record(name) for name in ('good.csv', 'hash.csv', 'size.csv', 'missing.csv')]
        manifest = self.manifest(records)
        (self.root / 'data/hash.csv').write_bytes(b'changed!')
        (self.root / 'data/size.csv').write_bytes(b'short')
        (self.root / 'data/missing.csv').unlink()
        checked, failures = verifier.verify_manifest(self.root, manifest)
        self.assertEqual(checked, 1)
        self.assertCountEqual(failures, ['Changed: data/hash.csv', 'Changed: data/size.csv', 'Missing: data/missing.csv'])

    def test_original_manifest_selection_is_preserved(self):
        records = [self.record('tracked.csv', tracked='True'), self.record('local.csv')]
        manifest = self.manifest(records, 'MANIFEST.csv')
        (self.root / 'data/local.csv').unlink()
        self.assertEqual(verifier.verify_manifest(self.root, manifest, original_only=True), (1, []))
        self.assertEqual(verifier.verify_manifest(self.root, manifest)[1], ['Missing: data/local.csv'])

    def test_unlisted_new_outputs_do_not_change_published_snapshot(self):
        manifest = self.manifest([self.record('input.csv')])
        self.record('new_output.csv', b'unrelated output')
        self.assertEqual(verifier.verify_manifest(self.root, manifest), (1, []))


if __name__ == '__main__':
    unittest.main()
