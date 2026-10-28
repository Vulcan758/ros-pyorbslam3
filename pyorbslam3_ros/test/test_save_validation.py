from pyorbslam3_ros.slam_node import validate_save_request


def test_valid_requests():
    assert validate_save_request('euroc', False, 'stereo_imu') is None
    assert validate_save_request('tum', True, 'mono') is None
    assert validate_save_request('kitti', False, 'stereo') is None
    assert validate_save_request('kitti', False, 'mono_imu') is None


def test_rejections():
    assert 'monocular' in validate_save_request('kitti', False, 'mono')
    assert 'keyframes_only' in validate_save_request('kitti', True, 'stereo')
    assert 'format' in validate_save_request('csv', False, 'stereo')
