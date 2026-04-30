#!/usr/bin/env python3
"""
Wall_generator_optimized.py

Path-style wall generator for Gazebo / Ignition.

This version is optimized for smaller model.sdf files:
    - Repeated "windows" blocks are NOT expanded into every individual lower/upper piece.
    - A repeated window block creates:
        1 lower strip
        1 upper strip
        posts between windows
        optional glass panes

It also supports angle turns:
    ("turn", "right")    -> -90 deg
    ("turn", "left")     -> +90 deg
    ("turn", -45)        -> clockwise 45 deg
    ("turn", 45)         -> counterclockwise 45 deg
    ("heading", 135)     -> absolute heading in degrees

Angle convention:
    E = 0 deg
    N = 90 deg
    W = 180 deg
    S = 270 deg

Run:
    python Wall_generator_optimized.py

Creates:
    models/fourth_floor_walls/model.config
    models/fourth_floor_walls/model.sdf
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math


# ============================================================
# Global dimensions
# ============================================================

INCH = 0.0254

# Match the old room world dimensions approximately:
# 2.75 m = 108.27 in
# 0.12 m = 4.72 in
WALL_H_IN = 108.27
WALL_T_IN = 4.72

# Old door panels were about 2.43 m = 95.67 in
DEFAULT_DOOR_H_IN = 96

# For large floor models, keep this False unless you specifically want glass visuals.
ADD_GLASS_PANES = False

# Keep false if your world already has a floor.
ADD_FLOOR_SLAB = False
FLOOR_THICKNESS_IN = 1.0
FLOOR_MARGIN_IN = 24.0

WALL_COLOR = "0.82 0.82 0.82 1"
GLASS_COLOR = "0.35 0.65 0.95 0.35"
FLOOR_COLOR = "0.55 0.55 0.55 1"


# ============================================================
# EDIT THIS SECTION
# ============================================================

WALL_PATHS = [
    {
        "name": "exteriors",
        "x0": 0,
        "y0": 0,
        "heading": "W",
        "commands": [
            ("solid", 10),
            ("windows", 6, 93.25, 26.75, 43, 53, 0),
            ("solid", 10),

            ("turn", "right"),
            ("solid", 249),

            ("turn", "left"),
            ("solid", 12),

            ("turn", "right"),
            ("solid", 10),
            ("windows", 6, 93.25, 26.75, 43, 53, 0),
            ("solid", 10),

            ("turn", "right"),
            ("solid", 12),

            ("turn", "left"),
            ("solid", 260),

            # North wall
            ("turn", "right"),
            ("solid", 20),
            ("windows", 14, 93.25, 26.75, 43, 53, 0),
            ("solid", 20),

            ("turn", "right"),
            ("solid", 260),

            #corridor
            ("turn", "left"),
            ("windows", 1, 60, 0, 20, 80, 0),
            ("turn", "left"),
            ("solid", 85),
            ("turn", "right"),
            ("solid", 123),
            ("turn", "right"),
            ("solid", 85),
            ("turn", 45),

            ("solid", 270),
            ("turn", "right"),
            ("solid", 100),
            ("turn", "right"),
            ("solid", 214),
            ("turn", 45),
            ("solid", 90),
            ("windows", 1, 60, 0, 20, 80, 0),
            ("turn", "left"),

            #south running wall
            ("solid", 164),
            ("turn", 45),
            ("solid", 276),
            ("turn", 45),
            ("solid", 42),
            
            ("turn", 'right'),
            ("solid", 13),
            ("windows", 2, 93.25, 26.75, 43, 53, 0),
            ("solid", 13),
            ("turn", 'right'),
            ("solid", 33),
            
            ("turn", 'left'),
            ("solid", 254),
            ("turn", 'right'),
            ("solid", 10),
            ("windows", 2, 93.25, 26.75, 43, 53, 0),
            ("solid", 10)
        ],
    },
    {
        "name": "hallways",
        "x0": 0,
        "y0": 141,
        "heading": "W",
        "commands": [
            # room415
            ("solid", 83),
            ("door", 36, 96),
            ("solid", 23),
            ("door", 36, 96),
            ("solid", 133),
            ("door", 36, 96),
            ("solid", 23),
            # room412
            ("door", 36, 96),
            ("solid", 85.5),
            ("door", 36, 96),
            ("solid", 63),
            
            ("turn", 'right'),
            ("solid", 2),
            ("door", 36, 96),
            ("solid", 81),
            ("door", 36, 96),
            ("solid", 142),
            ("door", 36, 96),
            ("solid", 87.2),
            ("door", 36, 96),
            
            # room 403
            ("solid", 26),
            ("door", 36, 96),
            ("solid", 142),
            ("door", 36, 96),
            ("solid", 26),
            
            #room 400
            ("door", 36, 96),
            ("solid", 63),
            
            ('turn', 'right'),
            ("solid", 272),
            ("door", 72, 96),
            
            #room 437
            ("solid", 208),
            ("door", 36, 96),
            
            ("solid", 202),
            ("door", 36, 96),
            
            #room 435
            ("solid", 25),
            ("door", 72, 96),
            ("solid", 312),
            ("door", 72, 96),
            ("solid", 24),
            ("door", 72, 96),
            ("solid", 48),
            ("door", 72, 96),
            ("solid", 48),
            
            # passageway
            ("windows", 1, 60, 0, 20, 80, 0),
            ("solid", 78),
            ("door", 36, 96),
            ("solid", 8),
            ("turn", -45),
            ("solid", 56),
            ("turn", "right"),
            ("door", 36, 96),

            ("solid", 64),
            ("turn", -45),
            ("solid", 90),

            ("windows", 1, 60, 0, 20, 80, 0),
            ("solid", 59),
            ("windows", 1, 44, 0, 20, 80, 0),
            ("solid", 62),
            
            ("turn", "left"),
            ("solid", 277),
            ("door", 72, 96),
            ("solid", 168),
            ("door", 36, 96),
            ("solid", 44),
            

        ],
    },
    {
        "name": "inner walls 1",
        "x0": 0,
        "y0": 335,
        "heading": "E",
        "commands": [
            # room423
            ("solid", 83.75),
            ("door", 36, 96),
            ("solid", 22),
            ("door", 36, 96),
            ("solid", 87.5),
            ("door", 36, 96),
            ("solid", 203),
            # room426
            ("door", 72, 96),
            ("solid", 161),

            ("turn", 'left'),
            ("solid", 442),
            ("door", 36, 96),
            ("solid", 41),

            ("turn", 'left'),
            ("solid", 205),
            ("door", 36, 96),
            ("solid", 167),
            ("door", 72, 96),
            ("solid", 21),
            ("door", 36, 96),
            ("solid", 200),
            
            ("turn", 'left'),
            ("solid", 48),
            ("door", 36, 96),
            ("solid", 206),
            ("door", 36, 96),
            ("solid", 193),
        ],
    },
    {
        "name": "inner walls 2",
        "x0": -77.3,
        "y0": 237,
        "heading": "W",
        "commands": [
            ("solid", 408),

            ("turn", 'right'),
            ("solid", 57),
            ("door", 36, 96),
            ("solid", 111),
            ("door", 36, 96),
            ("solid", 39),
            ("door", 36, 96),
            ("solid", 96),
            ("door", 36, 96),
            ("solid", 52),
            ("turn", 'right'),
            ("solid", 18),
            ("turn", 'left'),
            ("door", 52, 96),
            ("solid", 61),
            
            ("turn", 'right'),
            ("solid", 182),
            ("door", 36, 96),
            ("solid", 172),
            
            ("turn", 'right'),
            ("solid", 76),
            ("door", 36, 96),
            ("solid", 19),
            ("door", 36, 96),
            ("solid", 147),
            ("door", 36, 96),
            ("solid", 87),
            ("door", 36, 96),
            ("solid", 67),
            ("door", 36, 96),
            ("solid", 36)

        ],
    },
]

# Optional extra boxes: columns, protrusions, pilasters, etc.
# Coordinates are center coordinates in inches.
# yaw_deg is optional.
EXTRA_BOXES = [
    # {
    #     "name": "column_01",
    #     "x_center": 300,
    #     "y_center": 120,
    #     "sx": 12,
    #     "sy": 12,
    #     "height": WALL_H_IN,
    #     "yaw_deg": 0,
    # },
]


# ============================================================
# Internal data
# ============================================================

def m(inches: float) -> float:
    return inches * INCH


@dataclass
class BoxPiece:
    name: str
    sx: float
    sy: float
    sz: float
    px: float
    py: float
    pz: float
    yaw: float
    color: str
    collision: bool = True
    transparency: float | None = None


pieces: list[BoxPiece] = []


# ============================================================
# Heading helpers
# ============================================================

def normalize_deg(angle: float) -> float:
    return angle % 360.0


def heading_to_deg(heading) -> float:
    """
    Accept:
        "E", "N", "W", "S"
        numeric degrees like 45, 90, 135
        numeric strings like "45"
    """
    if isinstance(heading, (int, float)):
        return normalize_deg(float(heading))

    h = str(heading).strip().upper()

    mapping = {
        "E": 0.0,
        "N": 90.0,
        "W": 180.0,
        "S": 270.0,
    }

    if h in mapping:
        return mapping[h]

    return normalize_deg(float(h))


def heading_vector(heading_deg: float) -> tuple[float, float]:
    theta = math.radians(heading_deg)
    return math.cos(theta), math.sin(theta)


def turn_heading(heading_deg: float, turn) -> float:
    """
    Positive angle = left / counterclockwise.
    Negative angle = right / clockwise.
    """
    if isinstance(turn, (int, float)):
        return normalize_deg(heading_deg + float(turn))

    t = str(turn).strip().lower()

    if t in ("left", "l"):
        return normalize_deg(heading_deg + 90.0)

    if t in ("right", "r"):
        return normalize_deg(heading_deg - 90.0)

    if t in ("around", "back"):
        return normalize_deg(heading_deg + 180.0)

    # Allows strings like "45" or "-45"
    return normalize_deg(heading_deg + float(t))


# ============================================================
# Geometry helpers
# ============================================================

def add_box(
    *,
    name: str,
    sx_in: float,
    sy_in: float,
    sz_in: float,
    px_in: float,
    py_in: float,
    pz_in: float,
    yaw_deg: float = 0.0,
    color: str = WALL_COLOR,
    collision: bool = True,
    transparency: float | None = None,
) -> None:
    pieces.append(
        BoxPiece(
            name=name.replace(" ", "_"),
            sx=m(sx_in),
            sy=m(sy_in),
            sz=m(sz_in),
            px=m(px_in),
            py=m(py_in),
            pz=m(pz_in),
            yaw=math.radians(yaw_deg),
            color=color,
            collision=collision,
            transparency=transparency,
        )
    )


def segment_pose_from_start(
    *,
    x: float,
    y: float,
    heading_deg: float,
    length: float,
) -> tuple[float, float, float, float, float, float, float]:
    """
    Return:
        px, py, sx, sy, next_x, next_y, yaw_deg

    All values are inches except yaw_deg.
    The box is locally length x wall_thickness, then rotated by yaw.
    """
    dx, dy = heading_vector(heading_deg)

    next_x = x + dx * length
    next_y = y + dy * length

    px = (x + next_x) / 2.0
    py = (y + next_y) / 2.0

    sx = length
    sy = WALL_T_IN
    yaw_deg = heading_deg

    return px, py, sx, sy, next_x, next_y, yaw_deg


def point_forward(
    *,
    x: float,
    y: float,
    heading_deg: float,
    distance: float,
) -> tuple[float, float]:
    dx, dy = heading_vector(heading_deg)
    return x + dx * distance, y + dy * distance


def add_full_height_segment(
    *,
    name: str,
    x: float,
    y: float,
    heading_deg: float,
    length: float,
) -> tuple[float, float]:
    px, py, sx, sy, next_x, next_y, yaw_deg = segment_pose_from_start(
        x=x,
        y=y,
        heading_deg=heading_deg,
        length=length,
    )

    add_box(
        name=name,
        sx_in=sx,
        sy_in=sy,
        sz_in=WALL_H_IN,
        px_in=px,
        py_in=py,
        pz_in=WALL_H_IN / 2.0,
        yaw_deg=yaw_deg,
    )

    return next_x, next_y


def add_door_segment(
    *,
    name: str,
    x: float,
    y: float,
    heading_deg: float,
    width: float,
    door_h: float,
) -> tuple[float, float]:
    """
    Door opening below, lintel above.
    """
    lintel_h = WALL_H_IN - door_h

    px, py, sx, sy, next_x, next_y, yaw_deg = segment_pose_from_start(
        x=x,
        y=y,
        heading_deg=heading_deg,
        length=width,
    )

    if lintel_h > 0:
        add_box(
            name=name,
            sx_in=sx,
            sy_in=sy,
            sz_in=lintel_h,
            px_in=px,
            py_in=py,
            pz_in=door_h + lintel_h / 2.0,
            yaw_deg=yaw_deg,
        )

    return next_x, next_y


def add_window_segment(
    *,
    name: str,
    x: float,
    y: float,
    heading_deg: float,
    width: float,
    sill_h: float,
    window_h: float,
) -> tuple[float, float]:
    """
    Single window segment.
    For repeated windows, use add_windows_block instead.
    """
    upper_h = WALL_H_IN - sill_h - window_h

    if sill_h <= 0:
        raise ValueError(f"{name}: sill height must be positive.")
    if window_h <= 0:
        raise ValueError(f"{name}: window height must be positive.")
    if upper_h <= 0:
        raise ValueError(f"{name}: sill + window height exceeds wall height.")

    px, py, sx, sy, next_x, next_y, yaw_deg = segment_pose_from_start(
        x=x,
        y=y,
        heading_deg=heading_deg,
        length=width,
    )

    add_box(
        name=f"{name}_lower",
        sx_in=sx,
        sy_in=sy,
        sz_in=sill_h,
        px_in=px,
        py_in=py,
        pz_in=sill_h / 2.0,
        yaw_deg=yaw_deg,
    )

    add_box(
        name=f"{name}_upper",
        sx_in=sx,
        sy_in=sy,
        sz_in=upper_h,
        px_in=px,
        py_in=py,
        pz_in=sill_h + window_h + upper_h / 2.0,
        yaw_deg=yaw_deg,
    )

    if ADD_GLASS_PANES:
        glass_t = min(1.0, WALL_T_IN * 0.20)

        add_box(
            name=f"{name}_glass_visual",
            sx_in=width,
            sy_in=glass_t,
            sz_in=window_h,
            px_in=px,
            py_in=py,
            pz_in=sill_h + window_h / 2.0,
            yaw_deg=yaw_deg,
            color=GLASS_COLOR,
            collision=False,
            transparency=0.55,
        )

    return next_x, next_y


def add_windows_block(
    *,
    name: str,
    x: float,
    y: float,
    heading_deg: float,
    count: int,
    window_w: float,
    gap: float,
    sill_h: float,
    window_h: float,
    end_cap: float,
) -> tuple[float, float, float]:
    """
    Optimized repeated-window wall block.

    Creates:
        1 continuous lower strip
        1 continuous upper strip
        full-height posts between windows
        optional glass panes

    Returns:
        next_x, next_y, total_length
    """
    count = int(count)
    window_w = float(window_w)
    gap = float(gap)
    sill_h = float(sill_h)
    window_h = float(window_h)
    end_cap = float(end_cap)

    if count <= 0:
        return x, y, 0.0

    upper_h = WALL_H_IN - sill_h - window_h

    if sill_h <= 0:
        raise ValueError(f"{name}: sill height must be positive.")
    if window_h <= 0:
        raise ValueError(f"{name}: window height must be positive.")
    if upper_h <= 0:
        raise ValueError(f"{name}: sill + window height exceeds wall height.")

    total_len = 2 * end_cap + count * window_w + (count - 1) * gap

    # Entire block pose
    px, py, sx, sy, next_x, next_y, yaw_deg = segment_pose_from_start(
        x=x,
        y=y,
        heading_deg=heading_deg,
        length=total_len,
    )

    # One lower strip for whole window block
    add_box(
        name=f"{name}_lower_strip",
        sx_in=sx,
        sy_in=sy,
        sz_in=sill_h,
        px_in=px,
        py_in=py,
        pz_in=sill_h / 2.0,
        yaw_deg=yaw_deg,
    )

    # One upper strip for whole window block
    add_box(
        name=f"{name}_upper_strip",
        sx_in=sx,
        sy_in=sy,
        sz_in=upper_h,
        px_in=px,
        py_in=py,
        pz_in=sill_h + window_h + upper_h / 2.0,
        yaw_deg=yaw_deg,
    )

    # Full-height posts between windows
    cursor = end_cap + window_w

    for i in range(count - 1):
        post_center_distance = cursor + gap / 2.0
        post_px, post_py = point_forward(
            x=x,
            y=y,
            heading_deg=heading_deg,
            distance=post_center_distance,
        )

        add_box(
            name=f"{name}_post_{i + 1}",
            sx_in=gap,
            sy_in=WALL_T_IN,
            sz_in=WALL_H_IN,
            px_in=post_px,
            py_in=post_py,
            pz_in=WALL_H_IN / 2.0,
            yaw_deg=heading_deg,
        )

        cursor += gap + window_w

    # Optional visual-only glass panes
    if ADD_GLASS_PANES:
        glass_t = min(1.0, WALL_T_IN * 0.20)
        cursor = end_cap

        for i in range(count):
            glass_center_distance = cursor + window_w / 2.0
            glass_px, glass_py = point_forward(
                x=x,
                y=y,
                heading_deg=heading_deg,
                distance=glass_center_distance,
            )

            add_box(
                name=f"{name}_glass_{i + 1}",
                sx_in=window_w,
                sy_in=glass_t,
                sz_in=window_h,
                px_in=glass_px,
                py_in=glass_py,
                pz_in=sill_h + window_h / 2.0,
                yaw_deg=heading_deg,
                color=GLASS_COLOR,
                collision=False,
                transparency=0.55,
            )

            cursor += window_w + gap

    return next_x, next_y, total_len


def run_wall_path(path: dict) -> tuple[float, float, float, float]:
    """
    Generate one continuous path.

    Returns:
        final_x, final_y, final_heading_deg, total_forward_length
    """
    name = path["name"].replace(" ", "_")
    x = float(path["x0"])
    y = float(path["y0"])
    heading_deg = heading_to_deg(path["heading"])

    total_forward_length = 0.0
    command_index = 0

    for raw_cmd in path["commands"]:
        kind = raw_cmd[0]

        if kind == "windows":
            _, count, window_w, gap, sill_h, window_h, end_cap = raw_cmd

            command_index += 1

            x, y, block_len = add_windows_block(
                name=f"{name}_{command_index:03d}_windows_block",
                x=x,
                y=y,
                heading_deg=heading_deg,
                count=count,
                window_w=window_w,
                gap=gap,
                sill_h=sill_h,
                window_h=window_h,
                end_cap=end_cap,
            )

            total_forward_length += block_len
            continue

        command_index += 1

        if kind in ("solid", "wall", "post"):
            length = float(raw_cmd[1])
            x, y = add_full_height_segment(
                name=f"{name}_{command_index:03d}_{kind}",
                x=x,
                y=y,
                heading_deg=heading_deg,
                length=length,
            )
            total_forward_length += length

        elif kind == "door":
            width = float(raw_cmd[1])
            door_h = float(raw_cmd[2]) if len(raw_cmd) >= 3 else DEFAULT_DOOR_H_IN
            x, y = add_door_segment(
                name=f"{name}_{command_index:03d}_door_lintel",
                x=x,
                y=y,
                heading_deg=heading_deg,
                width=width,
                door_h=door_h,
            )
            total_forward_length += width

        elif kind == "window":
            width = float(raw_cmd[1])
            sill_h = float(raw_cmd[2])
            window_h = float(raw_cmd[3])
            x, y = add_window_segment(
                name=f"{name}_{command_index:03d}_window",
                x=x,
                y=y,
                heading_deg=heading_deg,
                width=width,
                sill_h=sill_h,
                window_h=window_h,
            )
            total_forward_length += width

        elif kind == "gap":
            width = float(raw_cmd[1])
            dx, dy = heading_vector(heading_deg)
            x += dx * width
            y += dy * width
            total_forward_length += width

        elif kind == "move":
            width = float(raw_cmd[1])
            dx, dy = heading_vector(heading_deg)
            x += dx * width
            y += dy * width

        elif kind == "turn":
            heading_deg = turn_heading(heading_deg, raw_cmd[1])

        elif kind == "heading":
            heading_deg = heading_to_deg(raw_cmd[1])

        else:
            raise ValueError(f"{name}: unknown command {kind!r}")

    return x, y, heading_deg, total_forward_length


def add_extra_boxes() -> None:
    for box in EXTRA_BOXES:
        height = float(box.get("height", WALL_H_IN))

        add_box(
            name=box["name"],
            sx_in=float(box["sx"]),
            sy_in=float(box["sy"]),
            sz_in=height,
            px_in=float(box["x_center"]),
            py_in=float(box["y_center"]),
            pz_in=height / 2.0,
            yaw_deg=float(box.get("yaw_deg", 0.0)),
            color=box.get("color", WALL_COLOR),
            collision=box.get("collision", True),
            transparency=box.get("transparency", None),
        )


def rotated_box_corners_2d(p: BoxPiece) -> list[tuple[float, float]]:
    """
    Return plan-view corners in inches. Used for better floor bounds.
    """
    cx = p.px / INCH
    cy = p.py / INCH
    sx = p.sx / INCH
    sy = p.sy / INCH

    c = math.cos(p.yaw)
    s = math.sin(p.yaw)

    corners_local = [
        (-sx / 2, -sy / 2),
        ( sx / 2, -sy / 2),
        ( sx / 2,  sy / 2),
        (-sx / 2,  sy / 2),
    ]

    corners_world = []
    for lx, ly in corners_local:
        wx = cx + lx * c - ly * s
        wy = cy + lx * s + ly * c
        corners_world.append((wx, wy))

    return corners_world


def compute_plan_bounds_in() -> tuple[float, float, float, float]:
    if not pieces:
        return 0.0, 0.0, 0.0, 0.0

    all_corners = []
    for p in pieces:
        all_corners.extend(rotated_box_corners_2d(p))

    min_x = min(x for x, _ in all_corners)
    max_x = max(x for x, _ in all_corners)
    min_y = min(y for _, y in all_corners)
    max_y = max(y for _, y in all_corners)

    return min_x, max_x, min_y, max_y


def add_floor_slab_from_bounds() -> None:
    min_x, max_x, min_y, max_y = compute_plan_bounds_in()

    min_x -= FLOOR_MARGIN_IN
    max_x += FLOOR_MARGIN_IN
    min_y -= FLOOR_MARGIN_IN
    max_y += FLOOR_MARGIN_IN

    add_box(
        name="floor_slab",
        sx_in=max_x - min_x,
        sy_in=max_y - min_y,
        sz_in=FLOOR_THICKNESS_IN,
        px_in=(min_x + max_x) / 2.0,
        py_in=(min_y + max_y) / 2.0,
        pz_in=-FLOOR_THICKNESS_IN / 2.0,
        yaw_deg=0.0,
        color=FLOOR_COLOR,
        collision=True,
    )


def build_model() -> None:
    pieces.clear()

    print("Wall path checks:")
    for path in WALL_PATHS:
        final_x, final_y, final_heading, total_len = run_wall_path(path)
        print(
            f"  {path['name']}: total drawn/moved length = {total_len:.3f} in, "
            f"end = ({final_x:.3f}, {final_y:.3f}), heading = {final_heading:.3f} deg"
        )

    add_extra_boxes()

    if ADD_FLOOR_SLAB:
        add_floor_slab_from_bounds()


# ============================================================
# SDF writing
# ============================================================

def box_xml(p: BoxPiece) -> str:
    collision_xml = ""
    if p.collision:
        collision_xml = f"""
      <collision name="{p.name}_collision">
        <geometry>
          <box>
            <size>{p.sx:.4f} {p.sy:.4f} {p.sz:.4f}</size>
          </box>
        </geometry>
      </collision>
"""

    transparency_xml = ""
    if p.transparency is not None:
        transparency_xml = f"""
          <transparency>{p.transparency:.3f}</transparency>
"""

    return f"""
    <link name="{p.name}">
      <pose>{p.px:.4f} {p.py:.4f} {p.pz:.4f} 0 0 {p.yaw:.6f}</pose>
{collision_xml}
      <visual name="{p.name}_visual">
        <geometry>
          <box>
            <size>{p.sx:.4f} {p.sy:.4f} {p.sz:.4f}</size>
          </box>
        </geometry>
        <material>
          <ambient>{p.color}</ambient>
          <diffuse>{p.color}</diffuse>
{transparency_xml}
        </material>
      </visual>
    </link>
"""


def make_model_config() -> str:
    return """<?xml version="1.0"?>
<model>
  <name>fourth_floor_walls</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>

  <author>
    <name>Abhijeet Kadam</name>
    <email>akadam3@buffalo.edu</email>
  </author>

  <description>
    Fourth-floor wall layout generated from optimized continuous wall paths.
  </description>
</model>
"""


def make_model_sdf() -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="fourth_floor_walls">
    <static>true</static>
{chr(10).join(box_xml(p) for p in pieces)}
  </model>
</sdf>
"""


def write_model(output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    (output_dir / "model.config").write_text(make_model_config(), encoding="utf-8")
    (output_dir / "model.sdf").write_text(make_model_sdf(), encoding="utf-8")

    print()
    print("Created:")
    print(f"  {output_dir / 'model.config'}")
    print(f"  {output_dir / 'model.sdf'}")
    print()
    print(f"Pieces generated: {len(pieces)}")
    print()
    print("World include:")
    print("""
    <include>
      <uri>model://fourth_floor_walls</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
""")


def main() -> None:
    build_model()
    write_model(Path("models/fourth_floor_walls"))


if __name__ == "__main__":
    main()
