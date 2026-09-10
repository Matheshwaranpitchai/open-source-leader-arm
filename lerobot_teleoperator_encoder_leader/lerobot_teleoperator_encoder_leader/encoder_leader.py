import logging
import math
import time

import serial

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config_encoder_leader import JOINTS, EncoderLeaderConfig

logger = logging.getLogger(__name__)


class _Unwrapper:
    def __init__(self, counts_per_rev: int):
        self.counts_per_rev = counts_per_rev
        self.last: int | None = None
        self.total = 0

    def update(self, raw: int) -> int:
        if raw < 0:
            return self.total
        if self.last is None:
            self.last = raw
        d = raw - self.last
        if d > self.counts_per_rev // 2:
            d -= self.counts_per_rev
        elif d < -self.counts_per_rev // 2:
            d += self.counts_per_rev
        self.last = raw
        self.total += d
        return self.total


class EncoderLeader(Teleoperator):
    config_class = EncoderLeaderConfig
    name = "encoder_leader"

    def __init__(self, config: EncoderLeaderConfig):
        super().__init__(config)
        self.config = config
        self.rad_per_count = 2 * math.pi / config.counts_per_rev
        self._ser: serial.Serial | None = None
        self._buf = b""
        self._unwrap = {ch: _Unwrapper(config.counts_per_rev) for ch in config.channel_to_joint}
        self._zero_count: dict[int, int] = {}


    @property
    def action_features(self) -> dict[str, type]:
        return {f"{j}.pos": float for j in JOINTS}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}


    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self._ser = serial.Serial(self.config.port, self.config.baudrate, timeout=self.config.timeout)
        time.sleep(1.5)
        self._ser.reset_input_buffer()
        self._buf = b""

        if calibrate:
            self.calibrate()

        logger.info(f"{self} connected on {self.config.port}")

    @check_if_not_connected
    def disconnect(self) -> None:
        self._ser.close()
        self._ser = None
        logger.info(f"{self} disconnected.")

    def configure(self) -> None:
        pass


    @property
    def is_calibrated(self) -> bool:
        return bool(self._zero_count)

    @check_if_not_connected
    def calibrate(self) -> None:
        input(f"Hold {self} in its reference pose, then press ENTER...")
        home = self._read_good_counts()
        self._zero_count = {
            ch: self._unwrap[ch].update(home[ch]) for ch in self.config.channel_to_joint
        }
        counts = [home[c] for c in sorted(self.config.channel_to_joint)]
        print(f"Homed on {counts}. Move the arm.")


    @check_if_not_connected
    def get_action(self) -> dict[str, float]:
        start = time.perf_counter()

        line = self._latest_line()
        if line is not None:
            counts = self._parse_counts(line)
            need = max(self.config.channel_to_joint) + 1
            if counts and len(counts) >= need:
                for ch in self.config.channel_to_joint:
                    self._unwrap[ch].update(counts[ch])

        action: dict[str, float] = {}
        for ch, jidx in self.config.channel_to_joint.items():
            delta = (self._unwrap[ch].total - self._zero_count[ch]) * self.rad_per_count
            delta *= self.config.direction[jidx] * self.config.scale[jidx]
            angle = self.config.home_angle[jidx] + delta

            lo, hi = self.config.joint_range[jidx]
            unit = (angle - lo) / max(hi - lo, 1e-9)
            unit = min(max(unit, 0.0), 1.0)

            name = JOINTS[jidx]
            if name == "gripper":
                action[f"{name}.pos"] = unit * 100.0
            else:
                action[f"{name}.pos"] = unit * 200.0 - 100.0

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        pass

    def _latest_line(self) -> bytes | None:
        self._buf += self._ser.read(self._ser.in_waiting or 1)
        if b"\n" not in self._buf:
            return None
        *complete, self._buf = self._buf.split(b"\n")
        for line in reversed(complete):
            line = line.strip()
            if line and not line.startswith(b"#"):
                return line
        return None

    @staticmethod
    def _parse_counts(line: bytes) -> list[int] | None:
        try:
            return [int(x) for x in line.split(b",")]
        except ValueError:
            return None

    def _read_good_counts(self) -> list[int]:
        need = max(self.config.channel_to_joint) + 1
        last_report = 0.0
        last_line = None
        while True:
            line = self._latest_line()
            if line is not None:
                last_line = line
                counts = self._parse_counts(line)
                if (
                    counts
                    and len(counts) >= need
                    and all(counts[c] >= 0 for c in self.config.channel_to_joint)
                ):
                    return counts
            now = time.time()
            if now - last_report > 1.0:
                last_report = now
                if last_line is None:
                    print("  waiting... no serial data yet (right port? streaming?)")
                else:
                    counts = self._parse_counts(last_line)
                    if not counts:
                        print(f"  waiting... unreadable line: {last_line!r}")
                    elif len(counts) < need:
                        print(
                            f"  waiting... only {len(counts)} values, need {need}"
                            f" -- re-flash firmware with N_CH={need}"
                        )
                    else:
                        bad = [c for c in self.config.channel_to_joint if counts[c] < 0]
                        print(
                            f"  waiting... channel(s) {bad} reading -1"
                            f" -- fix that encoder's magnet/wiring"
                        )
