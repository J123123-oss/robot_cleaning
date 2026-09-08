"""OpenCV transforms for JPEG frames received from OpenMV."""

import cv2
import numpy as np


_ROTATION_CODES = {
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}


def transform_jpeg_payload(payload, rotation_deg):
    """Rotate a JPEG payload and return encoded JPEG bytes.

    A zero-degree rotation returns the original bytes to avoid unnecessary
    decode/encode work.  ``None`` indicates that OpenCV could not decode or
    encode the frame.
    """
    if rotation_deg == 0:
        return payload

    encoded_input = np.frombuffer(payload, dtype=np.uint8)
    image = cv2.imdecode(encoded_input, cv2.IMREAD_COLOR)
    if image is None:
        return None

    image = cv2.rotate(image, _ROTATION_CODES[rotation_deg])
    encoded_ok, encoded_output = cv2.imencode(".jpg", image)
    if not encoded_ok:
        return None
    return encoded_output.tobytes()
