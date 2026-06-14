"""002_battery_compartment — parametric snap-fit removable battery drawer.

Open C-channel pocket (base + two side walls + back stop) that a rectangular
battery slides into along its long axis. A side cantilever finger near the mouth
snaps over the battery's trailing edge to retain it; press the finger outward and
grip the battery through a thumb scallop to remove it. Open top, short walls and
side-wall windows keep it ventilated (the battery runs warm); a base bolt pattern
mounts it to the robot.

Designed for FDM (PETG), printed flat (base down) with no supports. The latch
finger flexes in +Y (in-plane) so bending stress runs along the layer roads, not
across the weak Z interlayer bonds.

Coordinates: base bottom on Z=0, centered in X/Y. X = long/slide-in axis (back
stop at -X, open mouth at +X). Y = width (side walls at +/-Y). Z = up (open top).

Run: python battery_compartment.py
"""

import math
from pathlib import Path

import cadquery as cq

# Battery envelope (mm) -- PLACEHOLDER, set to the real cell.
BATTERY_LENGTH = 70.0   # long axis = slide-in (X) direction
BATTERY_WIDTH = 35.0    # across the pocket (Y)
BATTERY_HEIGHT = 20.0   # vertical (Z)

# Slide fit.
CLEARANCE = 0.30        # per-side gap between battery and pocket walls (PETG)

# Frame.
BASE_THICK = 4.0
WALL_THICK = 3.0
BACK_THICK = 3.0
WALL_HEIGHT = 14.0      # < BATTERY_HEIGHT so the battery top protrudes (grip + airflow)

# Cantilever snap latch (top-band finger on the +Y wall, flexes outward in +Y).
LATCH_LENGTH = 22.0     # free finger length, root -> tip (tune for stiffness)
LATCH_HEIGHT = 10.0     # finger height in Z (must leave wall below the relief slot)
LATCH_HOOK_DEPTH = 1.2  # how far the hook protrudes into the pocket (engagement)
LATCH_LEADIN_DEG = 35.0  # lead-in ramp angle from the X axis (insertion camming)
LATCH_RELIEF_GAP = 1.5  # relief slot width that frees the finger from the wall below

# Thumb scallop (finger access cut into the -Y wall near the mouth).
SCALLOP_RADIUS = 12.0
SCALLOP_DEPTH = 9.0     # how far the scoop reaches below the wall top
SCALLOP_OFFSET_X = 10.0  # scoop center, measured -X from the mouth face

# Ventilation windows in the side walls.
VENT_MARGIN = 4.0       # solid border kept around the window region
VENT_RIB = 5.0          # rib width between adjacent windows
VENT_COLS = 2           # number of windows along X per side wall

# Mounting bolt pattern through the base -- PLACEHOLDER, match the robot.
# NOTE: holes sit in the pocket floor; use countersunk flat-head screws (head
# flush with the floor) or move to external ears so heads don't foul the battery.
MOUNT_HOLE_DIA = 4.5    # M4 clearance
MOUNT_PATTERN_X = 56.0  # bolt spacing along X
MOUNT_PATTERN_Y = 24.0  # bolt spacing along Y
MOUNT_HOLE_COLS = 2     # holes along X
MOUNT_HOLE_ROWS = 2     # holes along Y

EPS = 0.5               # overshoot so cutters break cleanly through faces


def _box(length: float, width: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _grid_coords(span: float, count: int) -> list[float]:
    if count <= 1:
        return [0.0]
    step = span / (count - 1)
    return [-span / 2 + i * step for i in range(count)]


def make_battery_compartment(
    battery_length: float = BATTERY_LENGTH,
    battery_width: float = BATTERY_WIDTH,
    battery_height: float = BATTERY_HEIGHT,
    clearance: float = CLEARANCE,
) -> cq.Workplane:
    pocket_w = battery_width + 2 * clearance
    pocket_l = battery_length + 2 * clearance
    outer_l = BACK_THICK + pocket_l
    outer_w = 2 * WALL_THICK + pocket_w
    wall_top = BASE_THICK + WALL_HEIGHT

    x_min, x_max = -outer_l / 2, outer_l / 2
    mouth_x = x_max
    y_inner = pocket_w / 2          # pocket-side face of each side wall
    y_outer = outer_w / 2           # outside face of each side wall

    ramp_run = LATCH_HOOK_DEPTH / math.tan(math.radians(LATCH_LEADIN_DEG))

    # Geometry sanity (fail loud).
    assert WALL_HEIGHT - LATCH_HEIGHT - LATCH_RELIEF_GAP > 0, "latch finger + relief slot exceed wall height"
    assert ramp_run + 2.0 < LATCH_LENGTH, "hook does not fit within the finger length"
    assert VENT_MARGIN > BACK_THICK, "vent region would eat into the back stop wall"

    # 1. Outer block (base + walls), base bottom on Z=0.
    frame = cq.Workplane("XY").box(outer_l, outer_w, wall_top, centered=(True, True, False))

    # 2. Cut the open-top pocket -> base floor + back stop + two side walls.
    pocket_x0 = x_min + BACK_THICK
    frame = frame.cut(_box(
        length=(mouth_x + EPS) - pocket_x0,
        width=pocket_w,
        height=(wall_top + EPS) - BASE_THICK,
        center=((pocket_x0 + mouth_x + EPS) / 2, 0.0, (BASE_THICK + wall_top + EPS) / 2),
    ))

    # 3. Latch finger on the +Y wall: relief slot frees a top band, hook on its tip.
    x_root = mouth_x - LATCH_LENGTH
    finger_z0 = wall_top - LATCH_HEIGHT
    frame = frame.cut(_box(
        length=(mouth_x + EPS) - x_root,
        width=WALL_THICK + 2 * EPS,
        height=LATCH_RELIEF_GAP,
        center=((x_root + mouth_x + EPS) / 2, (y_inner + y_outer) / 2, finger_z0 - LATCH_RELIEF_GAP / 2),
    ))
    x_block = mouth_x - ramp_run - 2.0
    hook = (
        cq.Workplane("XY")
        .workplane(offset=finger_z0)
        .polyline([
            (x_block, y_inner),                          # top of blocking face
            (x_block, y_inner - LATCH_HOOK_DEPTH),       # peak (into the pocket)
            (x_block + ramp_run, y_inner),               # ramp back to the wall
        ])
        .close()
        .extrude(LATCH_HEIGHT)
    )
    frame = frame.union(hook)

    # 4. Thumb scallop: scoop a sphere out of the -Y wall top near the mouth.
    frame = frame.cut(
        cq.Workplane("XY")
        .sphere(SCALLOP_RADIUS)
        .translate((mouth_x - SCALLOP_OFFSET_X, -y_inner, wall_top - SCALLOP_DEPTH + SCALLOP_RADIUS))
    )

    # 5. Ventilation windows through both side walls (one cutter spans both).
    vent_x0 = x_min + VENT_MARGIN
    vent_x1 = x_root - VENT_RIB                # stay clear of the latch finger
    region_len = vent_x1 - vent_x0
    window_w = (region_len - (VENT_COLS - 1) * VENT_RIB) / VENT_COLS
    assert window_w > 0, "vent windows do not fit; reduce VENT_COLS/VENT_RIB/VENT_MARGIN"
    vent_z0 = BASE_THICK + VENT_MARGIN
    vent_z1 = wall_top - VENT_MARGIN
    for i in range(VENT_COLS):
        win_cx = vent_x0 + window_w / 2 + i * (window_w + VENT_RIB)
        frame = frame.cut(_box(
            length=window_w,
            width=outer_w + 2 * EPS,
            height=vent_z1 - vent_z0,
            center=(win_cx, 0.0, (vent_z0 + vent_z1) / 2),
        ))

    # 6. Mounting holes through the base.
    for hx in _grid_coords(MOUNT_PATTERN_X, MOUNT_HOLE_COLS):
        for hy in _grid_coords(MOUNT_PATTERN_Y, MOUNT_HOLE_ROWS):
            frame = frame.cut(
                cq.Workplane("XY")
                .circle(MOUNT_HOLE_DIA / 2)
                .extrude(BASE_THICK + 2 * EPS)
                .translate((hx, hy, -EPS))
            )

    return frame


if __name__ == "__main__":
    result = make_battery_compartment()
    solid = result.val()
    bb = solid.BoundingBox()
    print(f"Bounding box: {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm")
    print(f"Volume: {solid.Volume():.0f} mm^3, solids: {len(result.solids().vals())}")

    out = Path(__file__).with_name("battery_compartment.stl")
    cq.exporters.export(result, str(out))
    print(f"Wrote {out}")
