import numpy as np

from uf_map_maintenance.pcd import StreamingBinaryPcdWriter, read_binary_pcd


def test_streaming_binary_pcd_round_trip_and_cleanup(tmp_path):
    path = tmp_path / "map.pcd"
    first = np.array([[1, 2, 3, 4], [5, 6, 7, 8]], dtype=np.float64)
    second = np.array([[9, 10, 11]], dtype=np.float64)
    with StreamingBinaryPcdWriter(path) as writer:
        writer.append(first)
        writer.append(second)

    points = read_binary_pcd(path)
    np.testing.assert_allclose(
        points,
        [[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 0]],
    )
    assert b"DATA binary\n" in path.read_bytes()[:300]
    assert not list(tmp_path.glob("*.body.tmp"))


def test_streaming_writer_removes_partial_body_after_exception(tmp_path):
    path = tmp_path / "failed.pcd"
    try:
        with StreamingBinaryPcdWriter(path) as writer:
            writer.append(np.array([[1, 2, 3]]))
            raise RuntimeError("stop")
    except RuntimeError:
        pass
    assert not path.exists()
    assert not list(tmp_path.glob("*.body.tmp"))
