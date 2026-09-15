import logging
import time

import cv2
import mujoco
import mujoco.viewer
import numpy as np

from lerobot.robots.robot import Robot
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from . import task_scene
from .config_mujoco_so_arm import JOINTS, MJ_JOINTS, MujocoSOArmConfig

logger = logging.getLogger(__name__)

_CAMERA_POS_TASK = (0.0, -0.40, 0.95)
_CAMERA_POS_BARE = (0.0, -0.45, 0.45)

_WRIST_MOUNT_BODY = "Fixed_Jaw"
_WRIST_CAM_POS = (0.0, -0.02, 0.16)
_WRIST_TARGET_POS = (0.011, -0.055, 0.0)
_WRIST_TARGET_BODY = "grasp_point"

_PHASE_DEBOUNCE_S = 0.35


class MujocoSOArm(Robot):
    config_class = MujocoSOArmConfig
    name = "mujoco_so_arm"

    def __init__(self, config: MujocoSOArmConfig):
        super().__init__(config)
        self.config = config
        self.model = None
        self.data = None
        self._renderers: dict[str, mujoco.Renderer] = {}
        self._out_size: dict[str, tuple[int, int]] = {}
        self._shared_size: tuple[int, int] | None = None
        self._viewer = None
        self._jnt_qposadr: list[int] = []
        self._act_id: list[int] = []
        self._wall0 = 0.0
        self._task: task_scene.TaskState | None = None
        self._rng = np.random.default_rng(config.seed)
        self._phase = "record"
        self._phase_start = 0.0
        self._last_phase_change = 0.0
        self._pending_reset_at: float | None = None
        self._episode = 0
        self._key_listener = None

    @property
    def observation_features(self) -> dict:
        feats: dict = {f"{j}.pos": float for j in JOINTS}
        feats[self.config.camera_name] = (
            self.config.image_height,
            self.config.image_width,
            3,
        )
        if self.config.wrist_camera:
            feats[self.config.wrist_camera_name] = (
                self.config.wrist_image_height,
                self.config.wrist_image_width,
                3,
            )
        return feats

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{j}.pos": float for j in JOINTS}

    @property
    def cameras(self) -> dict:
        names = [self.config.camera_name]
        if self.config.wrist_camera:
            names.append(self.config.wrist_camera_name)
        return {n: None for n in names}


    @property
    def is_connected(self) -> bool:
        return self.model is not None

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        spec = mujoco.MjSpec.from_file(self.config.scene)
        if self.config.task:
            task_scene.add_task(spec)
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

        self._plan_renderers()

        if self.config.task:
            self._task = task_scene.TaskState(self.model)

        self._reset_scene()

        if self.config.viewer:
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self._wall0 = time.perf_counter()
        self._phase = "record"
        self._phase_start = time.perf_counter()
        self._start_key_listener()
        if (self.config.episode_time_s is None) != (self.config.reset_time_s is None):
            logger.warning(
                f"{self}: set episode_time_s and reset_time_s together (or neither); "
                f"with only one of them a phase that times out is missed and the "
                f"scene resets on the wrong side of the next episode boundary"
            )
        logger.info(f"{self} connected ({self.config.scene})")

    @check_if_not_connected
    def disconnect(self) -> None:
        if self._key_listener is not None:
            self._key_listener.stop()
            self._key_listener = None
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        self._renderers.clear()
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

        obs[self.config.camera_name] = self._grab(self.config.camera_name)
        if self.config.wrist_camera:
            obs[self.config.wrist_camera_name] = self._grab(self.config.wrist_camera_name)

        return obs

    def _grab(self, camera: str) -> np.ndarray:
        out_h, out_w = self._out_size[camera]
        key = "shared" if self._shared_size is not None else camera
        rend = self._renderers.get(key)
        if rend is None:
            h, w = self._shared_size or (out_h, out_w)
            rend = mujoco.Renderer(self.model, height=h, width=w)
            self._renderers[key] = rend

        rend.update_scene(self.data, camera=camera)
        if self.config.fast_render:
            rend.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
            rend.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
        frame = rend.render()

        if frame.shape[0] != out_h or frame.shape[1] != out_w:
            frame = cv2.resize(frame, (out_w, out_h), interpolation=cv2.INTER_AREA)
        return frame

    def _plan_renderers(self) -> None:
        sizes = {self.config.camera_name: (self.config.image_height, self.config.image_width)}
        if self.config.wrist_camera:
            sizes[self.config.wrist_camera_name] = (
                self.config.wrist_image_height,
                self.config.wrist_image_width,
            )
        self._out_size = sizes

        if len({round(w / h, 4) for h, w in sizes.values()}) == 1:
            self._shared_size = (
                max(h for h, _ in sizes.values()),
                max(w for _, w in sizes.values()),
            )
        else:
            self._shared_size = None
            logger.info(
                f"{self}: cameras have different aspect ratios, so each needs its own "
                f"render buffer; giving them the same ratio makes every step faster"
            )

    @check_if_not_connected
    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        self._service_reset()

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

    def _start_key_listener(self) -> None:
        try:
            from pynput import keyboard
        except ImportError:
            logger.info(
                f"{self}: pynput not installed; scene resets fall back to "
                f"episode_time_s/reset_time_s"
            )
            return

        end_keys = {keyboard.Key.right, keyboard.Key.left}
        end_chars = {"n", "r"}

        def on_press(key):
            char = getattr(key, "char", None)
            if char is not None:
                char = char.lower()
            if key in end_keys or char in end_chars:
                self._advance_phase()

        self._key_listener = keyboard.Listener(on_press=on_press)
        self._key_listener.daemon = True
        self._key_listener.start()

    def _advance_phase(self) -> None:
        now = time.perf_counter()
        if now - self._last_phase_change < _PHASE_DEBOUNCE_S:
            return
        self._last_phase_change = now
        self._phase_start = now

        if self._phase == "record":
            self._phase = "reset"
            self._pending_reset_at = now + self.config.reset_delay_s
            self._report_episode()
        else:
            self._phase = "record"
            self._pending_reset_at = None

    def _check_phase_timeout(self, now: float) -> None:
        limit = (
            self.config.episode_time_s
            if self._phase == "record"
            else self.config.reset_time_s
        )
        if limit is not None and now - self._phase_start >= limit:
            self._advance_phase()

    def _service_reset(self) -> None:
        now = time.perf_counter()
        self._check_phase_timeout(now)
        if self._pending_reset_at is not None and now >= self._pending_reset_at:
            self._pending_reset_at = None
            self._reset_scene()

    def _report_episode(self) -> None:
        if self._task is None:
            return
        got = self._task.success(self.data)
        self._episode += 1
        print(f"  episode {self._episode}: {sum(got.values())}/{len(got)} sorted  {got}")

    def _reset_to_home(self) -> None:
        for i in range(len(JOINTS)):
            lo, hi = self.config.joint_range[i]
            angle = min(max(self.config.home_angle[i], lo), hi)
            self.data.qpos[self._jnt_qposadr[i]] = angle
            self.data.ctrl[self._act_id[i]] = angle
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _reset_scene(self) -> None:
        self._reset_to_home()
        if self._task is not None:
            self._task.reset(self.model, self.data, self._rng, self.config.randomize)
            print("  reset:", [
                (b["color"], [round(float(v), 3) for v in self.data.qpos[b["qadr"]:b["qadr"]+2]])
                for b in self._task.blocks
            ])

    def task_success(self) -> dict[str, bool]:
        if self._task is None:
            return {}
        return self._task.success(self.data)

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
        if not any(c.name == self.config.camera_name for c in spec.cameras):
            cam = spec.worldbody.add_camera()
            cam.name = self.config.camera_name
            cam.mode = mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY
            if self.config.task:
                cam.pos = np.array(_CAMERA_POS_TASK)
                cam.targetbody = "workspace_center"
            else:
                cam.pos = np.array(_CAMERA_POS_BARE)
                cam.targetbody = "Base"

        if self.config.wrist_camera:
            self._ensure_wrist_camera(spec)

    def _ensure_wrist_camera(self, spec) -> None:
        name = self.config.wrist_camera_name
        if any(c.name == name for c in spec.cameras):
            return
        mount = next((b for b in spec.bodies if b.name == _WRIST_MOUNT_BODY), None)
        if mount is None:
            raise ValueError(
                f"{self}: body '{_WRIST_MOUNT_BODY}' not found; cannot mount the wrist camera"
            )
        if not any(b.name == _WRIST_TARGET_BODY for b in spec.bodies):
            mount.add_body(name=_WRIST_TARGET_BODY, pos=np.array(_WRIST_TARGET_POS))
        cam = mount.add_camera()
        cam.name = name
        cam.pos = np.array(_WRIST_CAM_POS)
        cam.mode = mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY
        cam.targetbody = _WRIST_TARGET_BODY