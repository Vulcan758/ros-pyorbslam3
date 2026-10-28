"""Mapping from the `sensor` parameter to pyorbslam3 sensors and node inputs."""

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class SensorMode:
    name: str          # value of the `sensor` parameter
    camera: str        # "mono" | "stereo" | "rgbd"
    inertial: bool
    sensor_enum: str   # pyorbslam3.Sensor member name

    @property
    def image_topics(self) -> Tuple[str, ...]:
        return {
            'mono': ('image',),
            'stereo': ('left/image', 'right/image'),
            'rgbd': ('rgb/image', 'depth/image'),
        }[self.camera]

    @property
    def input_topics(self) -> Tuple[str, ...]:
        return self.image_topics + (('imu',) if self.inertial else ())

    def pyorbslam3_sensor(self):
        import pyorbslam3
        return getattr(pyorbslam3.Sensor, self.sensor_enum)


MODES = {
    m.name: m
    for m in (
        SensorMode('mono', 'mono', False, 'MONOCULAR'),
        SensorMode('mono_imu', 'mono', True, 'IMU_MONOCULAR'),
        SensorMode('stereo', 'stereo', False, 'STEREO'),
        SensorMode('stereo_imu', 'stereo', True, 'IMU_STEREO'),
        SensorMode('rgbd', 'rgbd', False, 'RGBD'),
        SensorMode('rgbd_imu', 'rgbd', True, 'IMU_RGBD'),
    )
}


def parse_sensor(value: str) -> SensorMode:
    try:
        return MODES[value.strip().lower()]
    except KeyError:
        raise ValueError(
            f"sensor must be one of {', '.join(MODES)}; got {value!r}") from None
