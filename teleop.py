import argparse
import math
import time

import mujoco
import mujoco.viewer
import serial

SCENE = "mujoco_menagerie/trs_so_arm100/scene.xml"
COUNTS_PER_REV = 4096
RAD_PER_COUNT = 2 * math.pi / COUNTS_PER_REV

CHANNEL_TO_JOINT = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5}
HOME_ANGLE = {0: -0.4517, 1: -3.32, 2: 3.1078, 3: 1.2338, 4: 0.1391, 5: -0.174}
DIRECTION = {0: -1, 1: -1, 2: -1, 3: -1, 4: 1, 5: 1}
SCALE = {0: 1.0, 1: 1.0, 2: 1.0, 3: 1.0, 4: 1.0, 5: 1.0}


class Unwrapper:
    def __init__(self):
        self.last = None
        self.total = 0

    def update(self, raw):
        if raw < 0:
            return self.total
        if self.last is None:
            self.last = raw
        d = raw - self.last
        if d > COUNTS_PER_REV // 2:
            d -= COUNTS_PER_REV
        elif d < -COUNTS_PER_REV // 2:
            d += COUNTS_PER_REV
        self.last = raw
        self.total += d
        return self.total


def latest_line(ser, buf):
    buf += ser.read(ser.in_waiting or 1)
    if b"\n" not in buf:
        return None, buf
    *complete, buf = buf.split(b"\n")
    for line in reversed(complete):
        line = line.strip()
        if line and not line.startswith(b"#"):
            return line, buf
    return None, buf


def parse_counts(line):
    try:
        return [int(x) for x in line.split(b",")]
    except ValueError:
        return None


def read_good_counts(ser):
    buf = b""
    need = max(CHANNEL_TO_JOINT) + 1
    last_report = 0
    last_line = None
    while True:
        line, buf = latest_line(ser, buf)
        if line is not None:
            last_line = line
            counts = parse_counts(line)
            if counts and len(counts) >= need and all(
                counts[c] >= 0 for c in CHANNEL_TO_JOINT
            ):
                return counts
        now = time.time()
        if now - last_report > 1.0:
            last_report = now
            if last_line is None:
                print("  waiting... no serial data yet (right port? streaming?)")
            else:
                counts = parse_counts(last_line)
                if not counts:
                    print(f"  waiting... unreadable line: {last_line!r}")
                elif len(counts) < need:
                    print(f"  waiting... only {len(counts)} values, need {need}"
                          f" -- re-flash firmware with N_CH={need}")
                else:
                    bad = [c for c in CHANNEL_TO_JOINT if counts[c] < 0]
                    print(f"  waiting... channel(s) {bad} reading -1"
                          f" -- fix that encoder's magnet/wiring")


def build_model():
    return mujoco.MjSpec.from_file(SCENE).compile()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--list", action="store_true")
    p.add_argument("--quiet", action="store_true")
    args = p.parse_args()

    model = build_model()
    data = mujoco.MjData(model)

    # joint index -> actuator index (position actuators driving joints)
    joint_to_act = {}
    for a in range(model.nu):
        if model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT:
            joint_to_act[int(model.actuator_trnid[a, 0])] = a

    if args.list:
        for j in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
            act = joint_to_act.get(j)
            aname = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act)
                     if act is not None else "-- no actuator")
            print(f"joint {j}: {name}  -> actuator {act} ({aname})")
        return

    joints = {}
    for jidx in CHANNEL_TO_JOINT.values():
        act = joint_to_act.get(jidx)
        joints[jidx] = {
            "adr": model.jnt_qposadr[jidx],
            "act": act,
            "lo": model.jnt_range[jidx][0],
            "hi": model.jnt_range[jidx][1],
            "limited": bool(model.jnt_limited[jidx]),
        }
        if act is None:
            print(f"WARNING: joint {jidx} has no actuator; it won't be driven.")

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    unwrap = {ch: Unwrapper() for ch in CHANNEL_TO_JOINT}
    need = max(CHANNEL_TO_JOINT) + 1

    print("Hold the leader arm in its reference pose, then press ENTER...")
    input()
    home = read_good_counts(ser)
    zero_count = {ch: unwrap[ch].update(home[ch]) for ch in CHANNEL_TO_JOINT}

    # start the arm settled at home: set both state and actuator target there
    for jidx in CHANNEL_TO_JOINT.values():
        data.qpos[joints[jidx]["adr"]] = HOME_ANGLE[jidx]
        if joints[jidx]["act"] is not None:
            data.ctrl[joints[jidx]["act"]] = HOME_ANGLE[jidx]
    mujoco.mj_forward(model, data)
    print(f"Homed on {[home[c] for c in CHANNEL_TO_JOINT]}. Move the arm.")

    def clamp(v, jinfo):
        return max(jinfo["lo"], min(jinfo["hi"], v)) if jinfo["limited"] else v

    buf = b""
    wall0 = time.perf_counter()
    last_print = time.perf_counter()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # update targets from the latest reading
            line, buf = latest_line(ser, buf)
            if line is not None:
                counts = parse_counts(line)
                if counts and len(counts) >= need:
                    for ch, jidx in CHANNEL_TO_JOINT.items():
                        u = unwrap[ch].update(counts[ch])
                        delta = (u - zero_count[ch]) * RAD_PER_COUNT
                        delta *= DIRECTION[jidx] * SCALE[jidx]
                        target = clamp(HOME_ANGLE[jidx] + delta, joints[jidx])
                        act = joints[jidx]["act"]
                        if act is not None:
                            data.ctrl[act] = target

            # step physics forward to catch up with clock time
            target_time = time.perf_counter() - wall0
            steps = 0
            while data.time < target_time and steps < 50:
                mujoco.mj_step(model, data)
                steps += 1

            viewer.sync()


if __name__ == "__main__":
    main()