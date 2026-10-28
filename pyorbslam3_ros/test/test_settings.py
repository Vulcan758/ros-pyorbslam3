import glob
import os

import numpy as np
from pyorbslam3_ros.sensor_mode import MODES, parse_sensor
from pyorbslam3_ros.settings import (check_depth_encoding, check_settings, depth_map_factor,
                                     parse_settings_text, SettingsParseError)
import pytest

HEADER = '%YAML:1.0\nFile.version: "1.0"\n'
MATRIX = """{key}: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [{data}]
"""
IDENTITY = '1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1'
REAL_TBC = '0,-1,0,0.01, 1,0,0,0.02, 0,0,1,0.03, 0,0,0,1'
IMU_NOISE = 'IMU.NoiseGyro: 1e-4\nIMU.NoiseAcc: 1e-3\nIMU.Frequency: 200.0\n'


def levels(text, sensor):
    return [i.level for i in check_settings(parse_settings_text(text), parse_sensor(sensor))]


def test_parses_opencv_header_and_matrix():
    s = parse_settings_text(HEADER + MATRIX.format(key='IMU.T_b_c1', data=REAL_TBC))
    assert s['File.version'] == '1.0'
    assert s['IMU.T_b_c1'].shape == (4, 4)
    assert s['IMU.T_b_c1'][0, 3] == pytest.approx(0.01)


def test_parse_errors():
    with pytest.raises(SettingsParseError):
        parse_settings_text('%YAML:1.0\n: : bad: [')
    with pytest.raises(SettingsParseError):
        parse_settings_text('%YAML:1.0\n- a list\n')


def test_rectified_stereo_is_fatal():
    text = HEADER + 'Camera.type: "Rectified"\nStereo.b: 0.1\n'
    assert levels(text, 'stereo') == ['error']
    assert levels(text, 'mono') == []


def test_stereo_without_second_camera_warns():
    assert levels(HEADER + 'Camera.type: "PinHole"\n', 'stereo') == ['warn']
    assert levels(HEADER + 'Camera.type: "PinHole"\nCamera2.fx: 400\n', 'stereo') == []


def test_overlapping_typo_warns():
    text = HEADER + 'Camera.type: "KannalaBrandt8"\nCamera2.fx: 1\nCamera.overlappingBegin: 0\n'
    assert levels(text, 'stereo') == ['warn']


def test_old_bindings_with_fisheye_stereo_warn():
    text = HEADER + 'Camera.type: "KannalaBrandt8"\nCamera2.fx: 1\n'
    s = parse_settings_text(text)
    for version, expected in (('0.1.2', ['warn']), ('0.1.3', []), ('0.2.0', []),
                              ('0.1.10', []), (None, [])):
        assert [i.level for i in check_settings(s, parse_sensor('stereo'), version)] \
            == expected, version
    assert check_settings(s, parse_sensor('mono'), '0.1.2') == []
    pinhole = parse_settings_text(HEADER + 'Camera.type: "PinHole"\nCamera2.fx: 1\n')
    assert check_settings(pinhole, parse_sensor('stereo'), '0.1.2') == []


def test_inertial_requirements():
    assert levels(HEADER, 'mono_imu') == ['error']
    ok = HEADER + IMU_NOISE + MATRIX.format(key='IMU.T_b_c1', data=REAL_TBC)
    assert levels(ok, 'mono_imu') == []
    ident = HEADER + IMU_NOISE + MATRIX.format(key='IMU.T_b_c1', data=IDENTITY)
    assert levels(ident, 'mono_imu') == ['warn']


def test_depth_factor_and_encoding_check():
    s = parse_settings_text(HEADER + 'RGBD.DepthMapFactor: 1000.0\n')
    assert depth_map_factor(s) == 1000.0
    assert check_depth_encoding('16UC1', 1000.0) is None
    assert check_depth_encoding('16UC1', 1.0) is not None
    assert check_depth_encoding('32FC1', 1.0) is None
    assert check_depth_encoding('32FC1', 1000.0) is not None
    assert check_depth_encoding('32FC1', None) is None
    assert levels(HEADER, 'rgbd') == ['warn']


def test_bundled_settings_files_pass():
    root = os.path.join(os.path.dirname(__file__), '..', 'config', 'orbslam3')
    files = sorted(glob.glob(os.path.join(root, '*.yaml')))
    assert files
    for path in files:
        sensor = os.path.basename(path)[:-5].split('_', 1)[1]
        with open(path) as f:
            s = parse_settings_text(f.read())
        assert check_settings(s, parse_sensor(sensor)) == [], path


def test_sensor_modes():
    assert set(MODES) == {'mono', 'mono_imu', 'stereo', 'stereo_imu', 'rgbd', 'rgbd_imu'}
    m = parse_sensor(' Stereo_IMU ')
    assert m.inertial and m.camera == 'stereo'
    assert m.input_topics == ('left/image', 'right/image', 'imu')
    assert parse_sensor('rgbd').input_topics == ('rgb/image', 'depth/image')
    with pytest.raises(ValueError):
        parse_sensor('lidar')
    assert np.all([parse_sensor(n).pyorbslam3_sensor() is not None for n in MODES])
