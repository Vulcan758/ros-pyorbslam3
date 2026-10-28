import numpy as np
from pyorbslam3_ros.image_conversion import to_depth, to_gray, UnsupportedEncoding
import pytest
from sensor_msgs.msg import Image


def make_image(arr, encoding, pad=0, bigendian=False):
    arr = np.asarray(arr)
    h, w = arr.shape[:2]
    if bigendian:
        arr = arr.astype(arr.dtype.newbyteorder('>'))
    row = arr.reshape(h, -1).view(np.uint8)
    if pad:
        row = np.hstack([row, np.zeros((h, pad), np.uint8)])
    msg = Image()
    msg.height, msg.width = h, w
    msg.encoding = encoding
    msg.is_bigendian = int(bigendian)
    msg.step = row.shape[1]
    msg.data = row.tobytes()
    return msg


def test_mono8_roundtrip_with_padding():
    img = np.arange(5 * 7, dtype=np.uint8).reshape(5, 7)
    out = to_gray(make_image(img, 'mono8', pad=3))
    assert out.dtype == np.uint8 and out.flags.c_contiguous
    np.testing.assert_array_equal(out, img)


@pytest.mark.parametrize('encoding,order', [('rgb8', 'rgb'), ('bgr8', 'bgr'),
                                            ('rgba8', 'rgb'), ('bgra8', 'bgr')])
def test_colour_to_gray_matches_opencv_formula(encoding, order):
    rng = np.random.default_rng(0)
    rgb = rng.integers(0, 256, size=(4, 6, 3), dtype=np.uint8)
    chans = rgb if order == 'rgb' else rgb[..., ::-1]
    if encoding.endswith('a8'):
        chans = np.dstack([chans, np.full((4, 6), 255, np.uint8)])
    out = to_gray(make_image(chans, encoding, pad=2))
    r, g, b = (rgb[..., i].astype(np.uint32) for i in range(3))
    expected = ((r * 4899 + g * 9617 + b * 1868 + (1 << 13)) >> 14).astype(np.uint8)
    np.testing.assert_array_equal(out, expected)


def test_depth_16uc1_big_endian():
    d = np.array([[0, 1, 1000], [65535, 256, 2]], dtype=np.uint16)
    out = to_depth(make_image(d, '16UC1', bigendian=True))
    assert out.dtype == np.uint16
    np.testing.assert_array_equal(out, d)


def test_depth_32fc1():
    d = np.array([[0.5, 1.25], [np.nan, 3.0]], dtype=np.float32)
    out = to_depth(make_image(d, '32FC1', pad=4))
    assert out.dtype == np.float32
    np.testing.assert_array_equal(out, d)


def test_rejects_unsupported_and_mismatched():
    img = np.zeros((2, 2), np.uint8)
    with pytest.raises(UnsupportedEncoding):
        to_gray(make_image(img, 'yuv422'))
    with pytest.raises(UnsupportedEncoding):
        to_depth(make_image(img, 'mono8'))
    with pytest.raises(UnsupportedEncoding):
        to_gray(make_image(np.zeros((2, 2), np.uint16), '16UC1'))


def test_rejects_short_data_and_bad_step():
    msg = make_image(np.zeros((4, 4), np.uint8), 'mono8')
    msg.data = msg.data[:10]
    with pytest.raises(ValueError):
        to_gray(msg)
    msg = make_image(np.zeros((4, 4), np.uint8), 'mono8')
    msg.step = 3
    with pytest.raises(ValueError):
        to_gray(msg)
