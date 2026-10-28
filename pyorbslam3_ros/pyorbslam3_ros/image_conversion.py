"""sensor_msgs/Image -> numpy without cv_bridge (and without cv2).

ORB-SLAM3 is fed 8-bit grayscale for every camera input; colour images are
converted here with OpenCV's fixed-point BT.601 coefficients so the result
matches what cv::cvtColor would have produced. Depth images are passed through
in their native type (uint16 millimetres or float32 metres); ORB-SLAM3 scales
them with RGBD.DepthMapFactor.
"""

import numpy as np

# encoding -> (dtype, channels)
_LAYOUTS = {
    'mono8': (np.uint8, 1),
    '8UC1': (np.uint8, 1),
    'rgb8': (np.uint8, 3),
    'bgr8': (np.uint8, 3),
    'rgba8': (np.uint8, 4),
    'bgra8': (np.uint8, 4),
    'mono16': (np.uint16, 1),
    '16UC1': (np.uint16, 1),
    '32FC1': (np.float32, 1),
}

GRAY_ENCODINGS = ('mono8', '8UC1', 'rgb8', 'bgr8', 'rgba8', 'bgra8')
DEPTH_ENCODINGS = ('mono16', '16UC1', '32FC1')

# cv::cvtColor RGB2GRAY fixed point: (R*4899 + G*9617 + B*1868 + 2^13) >> 14
_R, _G, _B = 4899, 9617, 1868


class UnsupportedEncoding(ValueError):
    pass


def image_to_array(msg) -> np.ndarray:
    """Return an (H, W) or (H, W, C) native-endian array view/copy of msg."""
    try:
        dtype, channels = _LAYOUTS[msg.encoding]
    except KeyError:
        raise UnsupportedEncoding(f'unsupported image encoding {msg.encoding!r}') from None
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    itemsize = np.dtype(dtype).itemsize
    row_bytes = w * channels * itemsize
    if step < row_bytes:
        raise ValueError(f'image step {step} < width*channels*itemsize {row_bytes}')
    buf = np.frombuffer(msg.data, dtype=np.uint8) if not isinstance(msg.data, list) \
        else np.asarray(msg.data, dtype=np.uint8)
    if buf.size < step * h:
        raise ValueError(f'image data has {buf.size} bytes, expected {step * h}')
    rows = buf[: step * h].reshape(h, step)[:, :row_bytes]
    wire = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')
    shape = (h, w, channels) if channels > 1 else (h, w)
    arr = np.ascontiguousarray(rows).view(wire).reshape(shape)
    if wire != np.dtype(dtype):
        arr = arr.astype(dtype)
    return arr


def to_gray(msg) -> np.ndarray:
    """Decode a camera image to contiguous uint8 (H, W) grayscale."""
    if msg.encoding not in GRAY_ENCODINGS:
        raise UnsupportedEncoding(
            f'camera image encoding {msg.encoding!r} not supported '
            f"(expected one of {', '.join(GRAY_ENCODINGS)})")
    arr = image_to_array(msg)
    if arr.ndim == 2:
        return np.ascontiguousarray(arr)
    if msg.encoding.startswith('rgb'):
        r, g, b = arr[..., 0], arr[..., 1], arr[..., 2]
    else:
        b, g, r = arr[..., 0], arr[..., 1], arr[..., 2]
    y = (r.astype(np.uint32) * _R + g.astype(np.uint32) * _G + b.astype(np.uint32) * _B
         + (1 << 13)) >> 14
    return y.astype(np.uint8)


def to_depth(msg) -> np.ndarray:
    """Decode a depth image to contiguous uint16 or float32 (H, W)."""
    if msg.encoding not in DEPTH_ENCODINGS:
        raise UnsupportedEncoding(
            f'depth image encoding {msg.encoding!r} not supported '
            f"(expected one of {', '.join(DEPTH_ENCODINGS)})")
    return np.ascontiguousarray(image_to_array(msg))
