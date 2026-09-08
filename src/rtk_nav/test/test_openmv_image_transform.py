"""Tests for correcting the OpenMV camera mounting orientation."""

import sys
from pathlib import Path

import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

sys.path.insert(0, str(Path(__file__).parents[1]))

from rtk_nav.openmv_image_transform import (  # noqa: E402
    transform_jpeg_payload,
)


def test_transform_jpeg_payload_rotates_camera_frame_180_degrees():
    source = np.zeros((40, 40, 3), dtype=np.uint8)
    source[:20, :20] = [255, 0, 0]
    source[:20, 20:] = [0, 255, 0]
    source[20:, :20] = [0, 0, 255]
    source[20:, 20:] = [255, 255, 255]
    encoded_ok, encoded = cv2.imencode(
        ".jpg", source, [cv2.IMWRITE_JPEG_QUALITY, 100]
    )
    assert encoded_ok

    transformed = transform_jpeg_payload(encoded.tobytes(), 180)
    result = cv2.imdecode(
        np.frombuffer(transformed, dtype=np.uint8), cv2.IMREAD_COLOR
    )

    assert result.shape == source.shape
    # JPEG is lossy, so compare each corner with a small tolerance.
    np.testing.assert_allclose(result[5, 5], source[35, 35], atol=10)
    np.testing.assert_allclose(result[5, 35], source[35, 5], atol=10)
    np.testing.assert_allclose(result[35, 5], source[5, 35], atol=10)
    np.testing.assert_allclose(result[35, 35], source[5, 5], atol=10)


def test_transform_jpeg_payload_keeps_original_bytes_when_rotation_is_disabled():
    payload = b"jpeg-payload"

    assert transform_jpeg_payload(payload, 0) == payload
