# eyes_display

ROS2 Humble package to render full-screen 3D eyes on a Raspberry Pi 5 display and track a person using a tracking topic.

## What it does
- Full-screen OpenGL rendering (pyglet)
- Subscribes to a `geometry_msgs/Point` tracking topic
- Smooth gaze movement with configurable parameters

## Parameters
- `tracking_topic` (string): topic name for tracking (default `/person/target`)
- `input_normalized` (bool): true if x/y are in $[-1, 1]$
- `frame_width` / `frame_height` (float): used when `input_normalized` is false
- `fullscreen` (bool)
- `screen_width` / `screen_height` (int)
- `render_fps` (float)
- `smoothing_alpha` (float)

## Usage
- Build with your ROS2 workspace tools.
- Run the display node:
  - `ros2 run eyes_display eyes_node`
- Run the mock publisher for testing:
  - `ros2 run eyes_display mock_publisher`
- Or run both:
  - `ros2 launch eyes_display eyes_display.launch.py`

## Tracking message format
Publish `geometry_msgs/Point`:
- `x`: horizontal gaze, $[-1, 1]$ if normalized
- `y`: vertical gaze, $[-1, 1]$ if normalized
- `z`: unused

## Notes
- If your OAK-D node publishes pixel coordinates, set `input_normalized` to false and provide frame size.
- The display uses a single topic. Adjust `tracking_topic` to match your pipeline.
