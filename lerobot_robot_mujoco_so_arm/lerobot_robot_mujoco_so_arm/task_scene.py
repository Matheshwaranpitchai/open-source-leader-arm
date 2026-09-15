import mujoco
import numpy as np

BLOCK_SIZE = 0.015
BLOCK_MASS = 0.03
FRICTION = [1.2, 0.05, 0.001]

_Z = BLOCK_SIZE + 0.001
FORWARD_OFFSET = -0.10

COLORS = {
    "red": [0.85, 0.20, 0.20, 1.0],
    "green": [0.20, 0.70, 0.25, 1.0],
    "blue": [0.20, 0.45, 0.85, 1.0],
}

BLOCK_SLOTS = [(-0.12, -0.20), (0.00, -0.20), (0.12, -0.20)]
BLOCK_COLORS = ["green", "blue", "red"]

BINS = [(-0.16, -0.36, "red"), (0.00, -0.36, "green"), (0.16, -0.36, "blue")]
BIN_HALF = [0.05, 0.05]
BIN_WALL = 0.004
BIN_WALL_H = 0.0125

JITTER_X = 0.035
JITTER_Y = 0.025

WORKSPACE_CENTER = [0.0, -0.33, 0.05]


def block_names() -> list[str]:
    return [f"block_{c}" for c in BLOCK_COLORS]


def bin_centers() -> dict[str, tuple[float, float]]:
    return {color: (x, y + FORWARD_OFFSET) for (x, y, color) in BINS}


def add_task(spec) -> None:
    for color, (x, y) in zip(BLOCK_COLORS, BLOCK_SLOTS):
        body = spec.worldbody.add_body(
            name=f"block_{color}", pos=[x, y + FORWARD_OFFSET, _Z]
        )
        body.add_freejoint()
        geom = body.add_geom()
        geom.type = mujoco.mjtGeom.mjGEOM_BOX
        geom.size = [BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE]
        geom.rgba = COLORS[color]
        geom.friction = FRICTION
        geom.mass = BLOCK_MASS

    for cx, cy, color in BINS:
        _add_bin(spec, cx, cy + FORWARD_OFFSET, color)

    if not any(b.name == "workspace_center" for b in spec.bodies):
        spec.worldbody.add_body(name="workspace_center", pos=WORKSPACE_CENTER)


def _add_bin(spec, cx, cy, color) -> None:
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


class TaskState:

    def __init__(self, model):
        self.blocks = []
        for color in BLOCK_COLORS:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"block_{color}")
            if bid < 0:
                raise ValueError(f"block_{color} missing from the model")
            jid = model.body_jntadr[bid]
            self.blocks.append(
                {
                    "color": color,
                    "bid": bid,
                    "qadr": int(model.jnt_qposadr[jid]),
                    "vadr": int(model.jnt_dofadr[jid]),
                }
            )
        self.bins = bin_centers()

    def reset(self, model, data, rng: np.random.Generator, randomize: bool = True) -> None:
        slots = list(BLOCK_SLOTS)
        if randomize:
            rng.shuffle(slots)

        for blk, (sx, sy) in zip(self.blocks, slots):
            x, y = sx, sy + FORWARD_OFFSET
            if randomize:
                x += float(rng.uniform(-JITTER_X, JITTER_X))
                y += float(rng.uniform(-JITTER_Y, JITTER_Y))
            q, v = blk["qadr"], blk["vadr"]
            data.qpos[q : q + 3] = [x, y, _Z]
            data.qpos[q + 3 : q + 7] = [1.0, 0.0, 0.0, 0.0]
            data.qvel[v : v + 6] = 0.0
        mujoco.mj_forward(model, data)

    def success(self, data) -> dict[str, bool]:
        out = {}
        for blk in self.blocks:
            pos = data.xpos[blk["bid"]]
            cx, cy = self.bins[blk["color"]]
            inside = (
                abs(float(pos[0]) - cx) < BIN_HALF[0]
                and abs(float(pos[1]) - cy) < BIN_HALF[1]
                and float(pos[2]) < BIN_WALL_H * 2 + BLOCK_SIZE
            )
            out[blk["color"]] = inside
        return out