import logging
import time

import mujoco
import mujoco.viewer
import numpy as np

from lerobot.robots.robot import Robot
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config_mujoco_so_arm import JOINTS, MJ_JOINTS, MujocoSOArmConfig

logger = logging.getLogger(__name__)

_CAMERA_POS = (0.0, -0.45, 0.45)
_CAMERA_TARGET_BODY = "Base"


class MujocoSOArm(Robot):
    config_class = MujocoSOArmConfig
    name = "mujoco_so_arm"

    def __init__(self, config: MujocoSOArmConfig):
        super().__init__(config)
        self.config = config
        self.model = None
        self.data = None
        self._renderer = None
        self._viewer = None
        self._jnt_qposadr: list[int] = []
        self._act_id: list[int] = []
        self._wall0 = 0.0

    @property
    def observation_features(self) -> dict:
        feats: dict = {f"{j}.pos": float for j in JOINTS}
        feats[self.config.camera_name] = (
            self.config.image_height,
            self.config.image_width,
            3,
        )
        return feats

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{j}.pos": float for j in JOINTS}


    @property
    def is_connected(self) -> bool:
        return self.model is not None

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        spec = mujoco.MjSpec.from_file(self.config.scene)
        self._ensure_camera(spec)
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)

        joint_to_act = {
            int(self.model.actuator_trnid[a, 0]): a
            for a in range(self.model.nu)
            if self.model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT
        }
        for name in MJ_JOINTS:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"{self}: joint '{name}' not found in {self.config.scene}")
            if jid not in joint_to_act:
                raise ValueError(f"{self}: joint '{name}' has no position actuator")
            self._jnt_qposadr.append(int(self.model.jnt_qposadr[jid]))
            self._act_id.append(joint_to_act[jid])

        self._reset_to_home()

        if self.config.viewer:
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self._wall0 = time.perf_counter()
        logger.info(f"{self} connected ({self.config.scene})")

    @check_if_not_connected
    def disconnect(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        self._renderer = None
        self.model = None
        self.data = None
        self._jnt_qposadr = []
        self._act_id = []
        logger.info(f"{self} disconnected.")

    def configure(self) -> None:
        pass


    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass


    @check_if_not_connected
    def get_observation(self) -> dict:
        obs: dict = {}
        for i, name in enumerate(JOINTS):
            angle = float(self.data.qpos[self._jnt_qposadr[i]])
            obs[f"{name}.pos"] = self._angle_to_norm(i, angle)

        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                self.model,
                height=self.config.image_height,
                width=self.config.image_width,
            )
        self._renderer.update_scene(self.data, camera=self.config.camera_name)
        obs[self.config.camera_name] = self._renderer.render()
        return obs

    @check_if_not_connected
    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        for i, name in enumerate(JOINTS):
            key = f"{name}.pos"
            if key not in action:
                continue
            self.data.ctrl[self._act_id[i]] = self._norm_to_angle(i, float(action[key]))

        target = time.perf_counter() - self._wall0
        steps = 0
        while self.data.time < target and steps < 50:
            mujoco.mj_step(self.model, self.data)
            steps += 1

        if self._viewer is not None:
            self._viewer.sync()

        return action


    def _reset_to_home(self) -> None:
        """Settle the arm at the SO-ARM rest pose, matching the leader's reference."""
        for i in range(len(JOINTS)):
            lo, hi = self.config.joint_range[i]
            angle = min(max(self.config.home_angle[i], lo), hi)
            self.data.qpos[self._jnt_qposadr[i]] = angle
            self.data.ctrl[self._act_id[i]] = angle
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        print("home qpos:", [round(float(self.data.qpos[a]), 3) for a in self._jnt_qposadr])


    def _norm_to_angle(self, idx: int, value: float) -> float:
        lo, hi = self.config.joint_range[idx]
        unit = value / 100.0 if JOINTS[idx] == "gripper" else (value + 100.0) / 200.0
        unit = min(max(unit, 0.0), 1.0)
        return lo + unit * (hi - lo)

    def _angle_to_norm(self, idx: int, angle: float) -> float:
        lo, hi = self.config.joint_range[idx]
        unit = (angle - lo) / max(hi - lo, 1e-9)
        unit = min(max(unit, 0.0), 1.0)
        return unit * 100.0 if JOINTS[idx] == "gripper" else unit * 200.0 - 100.0


    def _ensure_camera(self, spec) -> None:
        """Add a camera aimed at the arm if the scene doesn't define one by that name."""
        if any(c.name == self.config.camera_name for c in spec.cameras):
            return
        cam = spec.worldbody.add_camera()
        cam.name = self.config.camera_name
        cam.pos = np.array(_CAMERA_POS)
        cam.mode = mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY
        cam.targetbody = _CAMERA_TARGET_BODY