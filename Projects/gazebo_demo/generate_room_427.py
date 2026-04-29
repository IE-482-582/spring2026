from pathlib import Path

INCH = 0.0254

# ----------------------------
# Dimensions from your notes
# ----------------------------
wall_h_in = 113
wall_t_in = 12

room_len_in = 941     # top / bottom inside length
room_dep_in = 261     # left / right inside depth

door_h_in = 100

# Top-wall protrusions
protrusion_w_in = 6
protrusion_d_in = 56
protrusion_starts_in = [258, 316, 376, 433]   # left edge of each protrusion, from inside left wall

# Top wall, from top-left moving right
top_seq = [
    ("solid", 480),
    ("door", 72),
    ("solid", 164),
    ("door", 72),
    ("solid", 153),
]

# Left wall, from bottom-left moving up
left_seq = [
    ("solid", 174),
    ("door", 36),
    ("solid", 51),
]

# Bottom wall inferred from:
# total = 941, all windows identical, right end solid = 10
# 5 + 8*93 + 7*26 + 10 = 941
bottom_left_end_in = 5
bottom_right_end_in = 10
bottom_window_w_in = 93
bottom_window_h_in = 38
bottom_window_sill_in = 27
bottom_window_gap_in = 26
bottom_window_count = 8

# Exact bottom-window wall geometry supplied separately, in meters.
# This replaces the inferred lower/upper/gap segmentation above.
bottom_wall_total_len_m = 24.2062
bottom_wall_thickness_m = 0.2
bottom_strip_h_m = 1.1176
top_strip_h_m = 0.3048
top_strip_z_m = 2.5908
section_z_m = 1.778
section_h_m = 1.3208
section_end_w_m = 0.254
section_inner_w_m = 0.6785429
window_w_m = 2.36855
window_h_m = 1.3208
window_y_offset_m = 0.11
glass_color = "0.3 0.6 0.9 0.35"

# ----------------------------
# Convert to meters
# ----------------------------
wall_h = wall_h_in * INCH
wall_t = wall_t_in * INCH
room_len = room_len_in * INCH
room_dep = room_dep_in * INCH
door_h = door_h_in * INCH

full_z = wall_h / 2.0
lintel_h = wall_h - door_h
lintel_z = door_h + lintel_h / 2.0

bottom_lower_h = bottom_window_sill_in * INCH
bottom_lower_z = bottom_lower_h / 2.0

bottom_upper_h = (wall_h_in - bottom_window_sill_in - bottom_window_h_in) * INCH
bottom_upper_z = (bottom_window_sill_in + bottom_window_h_in) * INCH + bottom_upper_h / 2.0

pieces = []

def m(v_in: float) -> float:
    return v_in * INCH

def add_box(name, sx, sy, sz, px, py, pz, color="0.82 0.82 0.82 1"):
    pieces.append({
        "name": name,
        "sx": sx, "sy": sy, "sz": sz,
        "px": px, "py": py, "pz": pz,
        "color": color,
        "collision": True,
    })

def add_visual_box(name, sx, sy, sz, px, py, pz, color):
    pieces.append({
        "name": name,
        "sx": sx, "sy": sy, "sz": sz,
        "px": px, "py": py, "pz": pz,
        "color": color,
        "collision": False,
    })

def box_xml(p):
    collision_xml = ""
    if p["collision"]:
        collision_xml = f"""      <collision name="{p['name']}_collision">
        <pose>{p['px']:.4f} {p['py']:.4f} {p['pz']:.4f} 0 0 0</pose>
        <geometry>
          <box>
            <size>{p['sx']:.4f} {p['sy']:.4f} {p['sz']:.4f}</size>
          </box>
        </geometry>
      </collision>
"""
    return collision_xml + f"""      <visual name="{p['name']}_visual">
        <pose>{p['px']:.4f} {p['py']:.4f} {p['pz']:.4f} 0 0 0</pose>
        <geometry>
          <box>
            <size>{p['sx']:.4f} {p['sy']:.4f} {p['sz']:.4f}</size>
          </box>
        </geometry>
        <material>
          <ambient>{p['color']}</ambient>
          <diffuse>{p['color']}</diffuse>
        </material>
      </visual>"""

# -------------------------------------------------
# Model origin = inside bottom-left corner of room
# x grows right, y grows up, z grows upward
# -------------------------------------------------

# Top wall
y_top = room_dep + wall_t / 2.0
cursor = 0.0
door_idx = 1
solid_idx = 1

for kind, w_in in top_seq:
    cx = m(cursor + w_in / 2.0)
    sx = m(w_in)

    if kind == "solid":
        add_box(
            name=f"top_solid_{solid_idx}",
            sx=sx, sy=wall_t, sz=wall_h,
            px=cx, py=y_top, pz=full_z,
        )
        solid_idx += 1
    elif kind == "door":
        add_box(
            name=f"top_door_{door_idx}_lintel",
            sx=sx, sy=wall_t, sz=lintel_h,
            px=cx, py=y_top, pz=lintel_z,
        )
        door_idx += 1

    cursor += w_in

# Top-wall protrusions
for i, start_in in enumerate(protrusion_starts_in):
    add_box(
        name=f"top_protrusion_{i+1}",
        sx=m(protrusion_w_in),
        sy=m(protrusion_d_in),
        sz=wall_h,
        px=m(start_in + protrusion_w_in / 2.0),
        py=room_dep - m(protrusion_d_in) / 2.0,
        pz=full_z,
    )

# Bottom wall
# Room origin is the inside bottom-left corner; map the supplied wall model so
# its inner face stays at y = 0 and its full outer length spans the room width.
bottom_wall_center_x = room_len / 2.0
bottom_wall_center_y = -bottom_wall_thickness_m / 2.0

add_box(
    name="bottom_strip",
    sx=bottom_wall_total_len_m, sy=bottom_wall_thickness_m, sz=bottom_strip_h_m,
    px=bottom_wall_center_x, py=bottom_wall_center_y, pz=bottom_strip_h_m / 2.0,
    color="0.85 0.85 0.85 1",
)
add_box(
    name="top_strip",
    sx=bottom_wall_total_len_m, sy=bottom_wall_thickness_m, sz=top_strip_h_m,
    px=bottom_wall_center_x, py=bottom_wall_center_y, pz=top_strip_z_m,
    color="0.85 0.85 0.85 1",
)

section_centers_x = [
    -11.9761,
    -9.1412786,
    -6.0941857,
    -3.0470929,
    0.0,
    3.0470929,
    6.0941857,
    9.1412786,
    11.9761,
]

for i, local_x in enumerate(section_centers_x, start=1):
    add_box(
        name=f"bottom_section_{i}",
        sx=section_end_w_m if i in (1, 9) else section_inner_w_m,
        sy=bottom_wall_thickness_m,
        sz=section_h_m,
        px=bottom_wall_center_x + local_x,
        py=bottom_wall_center_y,
        pz=section_z_m,
        color="0.85 0.85 0.85 1",
    )

window_centers_x = [
    -10.664825,
    -7.6177321,
    -4.5706393,
    -1.5235464,
    1.5235464,
    4.5706393,
    7.6177321,
    10.664825,
]

for i, local_x in enumerate(window_centers_x, start=1):
    add_visual_box(
        name=f"bottom_window_{i}_glass",
        sx=window_w_m,
        sy=0.02,
        sz=window_h_m,
        px=bottom_wall_center_x + local_x,
        py=bottom_wall_center_y + window_y_offset_m,
        pz=section_z_m,
        color=glass_color,
    )

# Left wall
x_left = -wall_t / 2.0
cursor = 0.0
door_idx = 1
solid_idx = 1

for kind, w_in in left_seq:
    cy = m(cursor + w_in / 2.0)
    sy = m(w_in)

    if kind == "solid":
        add_box(
            name=f"left_solid_{solid_idx}",
            sx=wall_t, sy=sy, sz=wall_h,
            px=x_left, py=cy, pz=full_z,
        )
        solid_idx += 1
    elif kind == "door":
        add_box(
            name=f"left_door_{door_idx}_lintel",
            sx=wall_t, sy=sy, sz=lintel_h,
            px=x_left, py=cy, pz=lintel_z,
        )
        door_idx += 1

    cursor += w_in

# Right wall
x_right = room_len + wall_t / 2.0
add_box(
    name="right_wall_full",
    sx=wall_t, sy=room_dep, sz=wall_h,
    px=x_right, py=room_dep / 2.0, pz=full_z,
)

model_config = """<?xml version="1.0" ?>
<model>
  <name>room_427_walls</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>Greg</name>
    <email>your_email@example.com</email>
  </author>
  <description>
    Room 427 walls generated from inside-face dimensions.
  </description>
</model>
"""

model_sdf = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="room_427_walls">
    <static>true</static>
    <link name="walls">
""" + "\n\n".join(box_xml(p) for p in pieces) + """
    </link>
  </model>
</sdf>
"""

out_dir = Path("models/room_427_walls")
out_dir.mkdir(parents=True, exist_ok=True)
(out_dir / "model.config").write_text(model_config, encoding="utf-8")
(out_dir / "model.sdf").write_text(model_sdf, encoding="utf-8")

print("Created:")
print(out_dir / "model.config")
print(out_dir / "model.sdf")
print(f"Pieces: {len(pieces)}")
print(f"Room inside size: {room_len:.4f} m x {room_dep:.4f} m")
print("Bottom wall used:")
print("Integrated exact 8-window wall geometry with bottom strip, top strip, 9 sections, and 8 glass panels")
print("Top protrusions used:")
print("4 protrusions, each 6 in wide x 56 in deep, starts at 258, 316, 376, 433 in")
