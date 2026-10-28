"""Sanity checks on ORB-SLAM3 settings files, run before the SLAM system starts.

ORB-SLAM3 is the authority on its own settings; this module only catches the
mistakes that otherwise surface as a segfault, an abort deep inside Settings
parsing, or silently wrong scale. Unparseable files produce a warning, not an
error.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np
import yaml


class SettingsParseError(ValueError):
    pass


@dataclass
class Issue:
    level: str   # "error" (refuse to start) | "warn"
    message: str


class _OpenCvLoader(yaml.SafeLoader):
    pass


def _opencv_matrix(loader, node):
    m = loader.construct_mapping(node, deep=True)
    return np.asarray(m['data'], dtype=np.float64).reshape(int(m['rows']), int(m['cols']))


_OpenCvLoader.add_constructor('tag:yaml.org,2002:opencv-matrix', _opencv_matrix)


def parse_settings_text(text: str) -> Dict:
    lines = text.splitlines()
    if lines and lines[0].startswith('%YAML'):
        lines = lines[1:]  # "%YAML:1.0" is OpenCV's dialect, not valid YAML
    try:
        data = yaml.load('\n'.join(lines), Loader=_OpenCvLoader)
    except yaml.YAMLError as e:
        raise SettingsParseError(str(e)) from e
    if not isinstance(data, dict):
        raise SettingsParseError('settings file is not a YAML mapping')
    return data


def load_settings(path: str) -> Dict:
    with open(path, 'r', encoding='utf-8') as f:
        return parse_settings_text(f.read())


def depth_map_factor(settings: Dict) -> Optional[float]:
    for key in ('RGBD.DepthMapFactor', 'DepthMapFactor'):
        if key in settings:
            return float(settings[key])
    return None


# first pyorbslam3 release whose ORB-SLAM3 has the fisheye stereo out-of-bounds fixes
FISHEYE_STEREO_FIXED_VERSION = (0, 1, 3)


def _version_tuple(version: str):
    parts = []
    for p in version.split('.')[:3]:
        digits = ''.join(c for c in p if c.isdigit())
        parts.append(int(digits) if digits else 0)
    return tuple(parts)


def check_settings(settings: Dict, mode, bindings_version: Optional[str] = None) -> List[Issue]:
    """Check a parsed settings dict against a sensor_mode.SensorMode.

    bindings_version is the installed pyorbslam3 version, or None when unknown
    (e.g. a source build without package metadata).
    """
    issues: List[Issue] = []
    cam_type = str(settings.get('Camera.type', '')).strip()

    if mode.camera == 'stereo':
        if (cam_type == 'KannalaBrandt8' and bindings_version is not None
                and _version_tuple(bindings_version) < FISHEYE_STEREO_FIXED_VERSION):
            issues.append(Issue(
                'warn',
                f'pyorbslam3 {bindings_version} has ORB-SLAM3 bugs that intermittently '
                'segfault fisheye (KannalaBrandt8) stereo; upgrade to pyorbslam3 >= 0.1.3.'))
        if cam_type == 'Rectified':
            issues.append(Issue(
                'error',
                'Camera.type "Rectified" segfaults ORB-SLAM3 in stereo modes. Use '
                'Camera.type "PinHole" with Camera2.* intrinsics (zero distortion for '
                'rectified images) and Stereo.T_c1_c2.'))
        if not any(k in settings for k in ('Camera2.fx', 'Camera.bf', 'Stereo.b')):
            issues.append(Issue(
                'warn',
                'stereo sensor but no second camera found (expected Camera2.* intrinsics, '
                'Camera.bf or Stereo.b).'))
        if settings.get('File.version') is not None and any(
                k.startswith('Camera.overlapping') for k in settings):
            issues.append(Issue(
                'warn',
                'Camera.overlapping* keys found in a File.version 1.0 file; ORB-SLAM3 reads '
                'Camera1.overlapping* / Camera2.overlapping* (known typo in the upstream '
                'TUM-VI example).'))

    if mode.inertial:
        T_key = next((k for k in ('IMU.T_b_c1', 'Tbc') if k in settings), None)
        missing = [k for k in ('IMU.NoiseGyro', 'IMU.NoiseAcc', 'IMU.Frequency')
                   if k not in settings]
        if T_key is None:
            missing.insert(0, 'IMU.T_b_c1')
        if missing:
            issues.append(Issue(
                'error', f"inertial sensor but settings lack {', '.join(missing)}."))
        if T_key is not None:
            T = np.asarray(settings[T_key], dtype=np.float64)
            if T.shape == (4, 4) and np.allclose(T, np.eye(4)):
                issues.append(Issue(
                    'warn',
                    f'{T_key} is exactly identity. That is almost never the real camera-IMU '
                    'extrinsic; a wrong rotation causes tracking loss as soon as inertial '
                    'fusion engages.'))

    if mode.camera == 'rgbd' and depth_map_factor(settings) is None:
        issues.append(Issue('warn', 'RGB-D sensor but no RGBD.DepthMapFactor in settings.'))

    return issues


def check_depth_encoding(encoding: str, factor: Optional[float]) -> Optional[str]:
    """Warn when depth units and RGBD.DepthMapFactor disagree (mm vs m)."""
    if factor is None:
        return None
    if encoding in ('16UC1', 'mono16') and abs(factor - 1.0) < 1e-6:
        return ('depth is 16UC1 (millimetres by convention) but RGBD.DepthMapFactor is 1.0; '
                'expected 1000.0. Map scale will be off by 1000x.')
    if encoding == '32FC1' and abs(factor - 1.0) > 1e-6:
        return (f'depth is 32FC1 (metres by convention) but RGBD.DepthMapFactor is {factor}; '
                'expected 1.0.')
    return None
