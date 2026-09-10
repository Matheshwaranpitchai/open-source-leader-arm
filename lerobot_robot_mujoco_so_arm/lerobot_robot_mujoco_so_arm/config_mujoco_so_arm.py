from dataclasses import dataclass, field

from lerobot.robots.config import RobotConfig

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

MJ_JOINTS = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]


def _default_joint_range() -> dict[int, tuple[float, float]]:
    """(lo, hi) radians per joint, from the SO-ARM100 MJCF. Fixed by the hardware."""
    return {
        0: (-1.92, 1.92),
        1: (-3.32, 0.174),
        2: (-0.174, 3.14),
        3: (-1.66, 1.66),
        4: (-2.79, 2.79),
        5: (-0.174, 1.75),
    }


def _default_home_angle() -> dict[int, float]:
    """SO-ARM rest pose in radians, same reference pose the leader is homed on."""
    return {0: -0.4517, 1: -3.32, 2: 3.1078, 3: 1.2338, 4: 0.1391, 5: -0.174}


@RobotConfig.register_subclass("mujoco_so_arm")
@dataclass
class MujocoSOArmConfig(RobotConfig):
    scene: str = "mujoco_menagerie/trs_so_arm100/scene.xml"

    viewer: bool = True

    camera_name: str = "top"
    image_width: int = 224
    image_height: int = 224

    joint_range: dict[int, tuple[float, float]] = field(default_factory=_default_joint_range)
    home_angle: dict[int, float] = field(default_factory=_default_home_angle)