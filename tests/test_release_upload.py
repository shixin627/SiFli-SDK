"""The discoverable release must always have an uploaded firmware archive."""
import pathlib
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] /
                       "example/get-started/dualcore/project/hcpu"))
import release_ci


class ReleaseUploadTest(unittest.TestCase):
    def setUp(self):
        for target, name, value in (
            (release_ci.rg.oss, "load_credentials", {}),
            (release_ci.rg, "read_board", "29"),
            (release_ci.rg, "read_status", ("RELEASE", "1.1.181", True)),
        ):
            self.enterContext(patch.object(target, name, return_value=value))
        self.info_key = "skaiwatch/29/info.json"
        self.zip_key = "skaiwatch/29/1.1.181/watchOS.zip"
        self.objects = {self.info_key: b'{"version":"1.1.180"}'}

    def test_archive_failure_preserves_published_metadata(self):
        before = self.objects.copy()
        with patch.object(release_ci.rg.oss, "put_object",
                          side_effect=OSError("archive upload failed")) as upload:
            with self.assertRaises(OSError):
                release_ci.step_upload()
        self.assertEqual(self.objects, before)
        self.assertEqual(upload.call_count, 1)
        self.assertEqual(upload.call_args.args[1:],
                         (self.zip_key, release_ci.rg.WATCHOS_ZIP))

    def test_metadata_is_published_only_after_archive_exists(self):
        def put_object(_creds, key, local_path):
            if key == self.info_key:
                self.assertIn(self.zip_key, self.objects)
            self.objects[key] = pathlib.Path(local_path).name.encode()
            return key

        with patch.object(release_ci.rg.oss, "put_object", side_effect=put_object) as upload:
            self.assertEqual(release_ci.step_upload(), 0)
        self.assertEqual([call.args[1] for call in upload.call_args_list],
                         [self.zip_key, self.info_key])
        self.assertEqual(self.objects[self.info_key], b"info.json")


if __name__ == "__main__":
    unittest.main()
