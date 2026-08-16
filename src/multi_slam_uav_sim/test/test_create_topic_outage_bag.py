import importlib.util
from pathlib import Path
import tempfile
import unittest


TOOL_PATH = Path(__file__).resolve().parents[3] / "tools" / "create_topic_outage_bag.py"
SPEC = importlib.util.spec_from_file_location("create_topic_outage_bag", TOOL_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class TopicOutageBagTest(unittest.TestCase):
    @staticmethod
    def write_metadata(root, compression_mode):
        (root / "metadata.yaml").write_text(
            "rosbag2_bagfile_information:\n"
            f"  compression_mode: {compression_mode!r}\n",
            encoding="utf-8",
        )

    def test_plain_bag_uses_standard_reader(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_metadata(root, "")
            reader = MODULE.make_reader(root)
            self.assertEqual(type(reader).__name__, "SequentialReader")

    def test_compressed_bag_uses_compression_reader(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_metadata(root, "FILE")
            reader = MODULE.make_reader(root)
            self.assertEqual(
                type(reader).__name__,
                "SequentialCompressionReader",
            )

    def test_none_mode_is_treated_as_plain_storage(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_metadata(root, "NONE")
            reader = MODULE.make_reader(root)
            self.assertEqual(type(reader).__name__, "SequentialReader")


if __name__ == "__main__":
    unittest.main()
