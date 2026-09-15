from dataclasses import dataclass, field

from lerobot.robots.config import RobotConfig

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

MJ_JOINTS = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]


def _default_joint_range() -> dict[int, tuple[float, float]]:
    return {
        0: (-1.92, 1.92),
        1: (-3.32, 0.174),
        2: (-0.174, 3.14),
        3: (-1.66, 1.66),
        4: (-2.79, 2.79),
        5: (-0.174, 1.75),
    }


def _default_home_angle() -> dict[int, float]:
    return {0: -0.4517, 1: -3.32, 2: 3.1078, 3: 1.2338, 4: 0.1391, 5: -0.174}


@RobotConfig.register_subclass("mujoco_so_arm")
@dataclass
class MujocoSOArmConfig(RobotConfig):
    scene: str = "mujoco_menagerie/trs_so_arm100/scene.xml"

    viewer: bool = True

    camera_name: str = "top"
    image_width: int = 224
    image_height: int = 224

    wrist_camera: bool = True
    wrist_camera_name: str = "wrist"
    wrist_image_width: int = 128
    wrist_image_height: int = 128

    fast_render: bool = True

    joint_range: dict[int, tuple[float, float]] = field(default_factory=_default_joint_range)
    home_angle: dict[int, float] = field(default_factory=_default_home_angle)

    task: bool = True

    reset_delay_s: float = 1.0
    episode_time_s: float | None = None
    reset_time_s: float | None = None
    randomize: bool = True
    seed: int = 0