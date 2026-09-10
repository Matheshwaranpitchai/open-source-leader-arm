from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def _default_channel_to_joint() -> dict[int, int]:
    return {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5}


def _default_home_angle() -> dict[int, float]:
    return {0: -0.4517, 1: -3.32, 2: 3.1078, 3: 1.2338, 4: 0.1391, 5: -0.174}


def _default_direction() -> dict[int, int]:
    return {0: -1, 1: -1, 2: -1, 3: -1, 4: 1, 5: 1}


def _default_scale() -> dict[int, float]:
    return {0: 1.0, 1: 1.0, 2: 1.0, 3: 1.0, 4: 1.0, 5: 1.0}


def _default_joint_range() -> dict[int, tuple[float, float]]:
    return {
        0: (-1.92, 1.92),
        1: (-3.32, 0.174),
        2: (-0.174, 3.14),
        3: (-1.66, 1.66),
        4: (-2.79, 2.79),
        5: (-0.174, 1.75),
    }


@TeleoperatorConfig.register_subclass("encoder_leader")
@dataclass
class EncoderLeaderConfig(TeleoperatorConfig):
    port: str = "COM5"
    baudrate: int = 115200
    timeout: float = 0.05

    channel_to_joint: dict[int, int] = field(default_factory=_default_channel_to_joint)
    home_angle: dict[int, float] = field(default_factory=_default_home_angle)
    direction: dict[int, int] = field(default_factory=_default_direction)
    scale: dict[int, float] = field(default_factory=_default_scale)
    joint_range: dict[int, tuple[float, float]] = field(default_factory=_default_joint_range)

    counts_per_rev: int = 4096
