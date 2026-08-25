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

BLOCK_SIZE = 0.015
BLOCK_MASS = 0.03
FRICTION = [1.2, 0.05, 0.001]
GRIP_GAIN = None

_Z = BLOCK_SIZE + 0.001
FORWARD_OFFSET = -0.10

COLORS = {
    "red":   [0.85, 0.20, 0.20, 1.0],
    "green": [0.20, 0.70, 0.25, 1.0],
    "blue":  [0.20, 0.45, 0.85, 1.0],
}

BLOCK_LAYOUT = [
    (-0.12, -0.20, "green"), (0.00, -0.20, "blue"), (0.12, -0.20, "red"),
]

BINS = [
    (-0.16, -0.36, "red"),
    (0.00, -0.36, "green"),
    (0.16, -0.36, "blue"),
]
BIN_HALF = [0.05, 0.05]
BIN_WALL = 0.004
BIN_WALL_H = 0.0125


def _blocks_spec():
    out = []
    for i, (x, y, color) in enumerate(BLOCK_LAYOUT):
        out.append({
            "name": f"block_{color}_{i}",
            "pos": [x, y + FORWARD_OFFSET, _Z],
            "color": color,
            "rgba": COLORS[color],
        })
    return out


BLOCKS = _blocks_spec()
BINS_ADJ = [(x, y + FORWARD_OFFSET, color) for (x, y, color) in BINS]


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


def _add_bin(spec, cx, cy, color):
    name = f"bin_{color}"
    bhx, bhy = BIN_HALF
    wt, wh = BIN_WALL, BIN_WALL_H
    pad_rgba = COLORS[color]
    wall_rgba = [pad_rgba[0], pad_rgba[1], pad_rgba[2], 0.40]

    body = spec.worldbody.add_body(name=name, pos=[cx, cy, 0.0])

    pad = body.add_geom()
    pad.name = f"{name}_pad"
    pad.type = mujoco.mjtGeom.mjGEOM_BOX
    pad.size = [bhx + 2 * wt, bhy + 2 * wt, 0.001]
    pad.pos = [0.0, 0.0, 0.001]
    pad.rgba = pad_rgba

    walls = [
        (f"{name}_e", [bhx + wt, 0.0, wh], [wt, bhy + 2 * wt, wh]),
        (f"{name}_w", [-(bhx + wt), 0.0, wh], [wt, bhy + 2 * wt, wh]),
        (f"{name}_n", [0.0, bhy + wt, wh], [bhx + 2 * wt, wt, wh]),
        (f"{name}_s", [0.0, -(bhy + wt), wh], [bhx + 2 * wt, wt, wh]),
    ]
    for gname, gpos, gsize in walls:
        w = body.add_geom()
        w.name = gname
        w.type = mujoco.mjtGeom.mjGEOM_BOX
        w.pos = gpos
        w.size = gsize
        w.rgba = wall_rgba


def build_model():
    spec = mujoco.MjSpec.from_file(SCENE)
    for b in BLOCKS:
        body = spec.worldbody.add_body(name=b["name"], pos=b["pos"])
        body.add_freejoint()
        geom = body.add_geom()
        geom.type = mujoco.mjtGeom.mjGEOM_BOX
        geom.size = [BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE]
        geom.rgba = b["rgba"]
        geom.friction = FRICTION
        geom.mass = BLOCK_MASS
    for cx, cy, color in BINS_ADJ:
        _add_bin(spec, cx, cy, color)
    return spec.compile()


def block_registry(model):
    reg = []
    for b in BLOCKS:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b["name"])
        jid = model.body_jntadr[bid]
        reg.append({
            "name": b["name"],
            "color": b["color"],
            "bid": bid,
            "qadr": model.jnt_qposadr[jid],
            "vadr": model.jnt_dofadr[jid],
            "start": list(b["pos"]),
        })
    return reg


def reset_blocks(model, data, blocks_reg):
    for blk in blocks_reg:
        q = blk["qadr"]
        data.qpos[q:q + 3] = blk["start"]
        data.qpos[q + 3:q + 7] = [1.0, 0.0, 0.0, 0.0]
        v = blk["vadr"]
        data.qvel[v:v + 6] = 0.0
    mujoco.mj_forward(model, data)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--list", action="store_true")
    args = p.parse_args()

    model = build_model()
    data = mujoco.MjData(model)
    blocks_reg = block_registry(model)

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

    if GRIP_GAIN is not None:
        gjoint = CHANNEL_TO_JOINT[5]
        gact = joint_to_act.get(gjoint)
        if gact is not None:
            model.actuator_gainprm[gact, 0] = GRIP_GAIN

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

    for jidx in CHANNEL_TO_JOINT.values():
        data.qpos[joints[jidx]["adr"]] = HOME_ANGLE[jidx]
        if joints[jidx]["act"] is not None:
            data.ctrl[joints[jidx]["act"]] = HOME_ANGLE[jidx]
    mujoco.mj_forward(model, data)
    print("Homed. Sort the blocks into the matching-color bins. "
          "Press R in the viewer to reset the blocks.")

    def clamp(v, jinfo):
        return max(jinfo["lo"], min(jinfo["hi"], v)) if jinfo["limited"] else v

    reset_requested = [False]

    def key_callback(keycode):
        if keycode in (ord("R"), ord("r")):
            reset_requested[0] = True

    buf = b""
    wall0 = time.perf_counter()

    with mujoco.viewer.launch_passive(
        model, data, key_callback=key_callback
    ) as viewer:
        while viewer.is_running():
            if reset_requested[0]:
                reset_requested[0] = False
                reset_blocks(model, data, blocks_reg)

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

            target_time = time.perf_counter() - wall0
            steps = 0
            while data.time < target_time and steps < 50:
                mujoco.mj_step(model, data)
                steps += 1

            viewer.sync()


if __name__ == "__main__":
    main()