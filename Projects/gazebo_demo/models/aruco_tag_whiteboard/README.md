# Aruco Tag Whiteboard Model

This reusable Gazebo model includes:
- a small whiteboard base
- an Aruco / AprilTag image attached to the front face
- marker texture support for IDs 1–15

## Files
- `aruco_tag_whiteboard.sdf`
- `model.config`
- `materials/textures/apriltag_36h11_id1.png` to `apriltag_36h11_id15.png`

## How to use
Add this model to a world file with:

```xml
<include>
  <uri>model://aruco_tag_whiteboard</uri>
  <pose>2 0 1.5 0 0 0</pose>
</include>

How to adjust position
Modify:
<pose>x y z roll pitch yaw</pose>
How to add more markers
Repeat the <include> block with different poses.
How to change marker ID
Edit this line in aruco_tag_whiteboard.sdf:
<albedo_map>materials/textures/apriltag_36h11_id1.png</albedo_map>
Change id1 to any ID from 1 to 15.