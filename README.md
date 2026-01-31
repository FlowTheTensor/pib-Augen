# eyes_display

ROS2‑Humble‑Paket zum Vollbild‑Rendern von 3D‑Augen auf einem Raspberry Pi 5 und zum Verfolgen einer Person über ein Tracking‑Topic.

## Was es macht
- Vollbild‑OpenGL‑Rendering (pyglet)
- Abonniert ein `geometry_msgs/Point`‑Tracking‑Topic
- Weiche Blickbewegung über Glättung

## Parameter
- `tracking_topic` (string): Topic für Tracking (Standard: `/person/target`)
- `input_normalized` (bool): true, wenn x/y im Bereich $[-1, 1]$ liegen
- `frame_width` / `frame_height` (float): genutzt, wenn `input_normalized` false ist
- `fullscreen` (bool)
- `screen_width` / `screen_height` (int)
- `render_fps` (float)
- `smoothing_alpha` (float)

Die Parameter liegen in [config/eyes_display.yaml](config/eyes_display.yaml).

## Ausführen auf dem Raspberry Pi (ohne VS Code)

### Automatisiertes Setup (Host)
- Script ausführen (installiert python3-pyglet, kopiert Paket in den Container, baut es und zeigt Startbefehl):
  - `bash scripts/setup_eyes_display.sh`
- Optional mit Container‑Name und Workspace:
  - `bash scripts/setup_eyes_display.sh multirepo-ros-display-1 /app/ros2_ws`

### Start direkt im Container (Host)
- Start‑Script ausführen:
  - `bash scripts/run_eyes_display.sh`
- Optional mit Container‑Name und Workspace:
  - `bash scripts/run_eyes_display.sh multirepo-ros-display-1 /app/ros2_ws`

### 1) Projekt holen
- Per Git:
  - `git clone <repo-url> ~/pib-Augen`
- Oder per scp in ein Arbeitsverzeichnis kopieren.

### 2) ROS2‑Umgebung laden
- `source /opt/ros/humble/setup.bash`

### 3) Abhängigkeiten installieren (pyglet)
- `sudo apt update`
- `sudo apt install -y python3-pyglet`

### 4) Bauen
- `cd ~/pib-Augen`
- `colcon build --symlink-install`
- `source install/setup.bash`

### 5) Starten
- Nur Anzeige starten:
  - `ros2 launch eyes_display eyes_display.launch.py`
- Direkt per Node:
  - `ros2 run eyes_display eyes_node`
- Mock‑Publisher zum Testen:
  - `ros2 run eyes_display mock_publisher`

## Tracking‑Nachrichtenformat
Publish `geometry_msgs/Point`:
- `x`: horizontale Blickrichtung, $[-1, 1]$ bei normalisiertem Input
- `y`: vertikale Blickrichtung, $[-1, 1]$ bei normalisiertem Input
- `z`: ungenutzt

## Hinweise
- Wenn der OAK‑D‑Node Pixel‑Koordinaten liefert, setze `input_normalized: false` und trage `frame_width`/`frame_height` ein.
- Das Display nutzt ein einzelnes Topic. Passe `tracking_topic` an deine Pipeline an.

## OAK‑D Lite: Face‑Tracking → Blickrichtung
Es gibt einen einfachen Face‑Tracker, der Bilddaten von der OAK‑D abonniert und das Blick‑Topic füttert.

### Node
- `ros2 run eyes_display face_tracker`

### Parameter (config)
- `image_topic`: ROS‑Bildtopic der OAK‑D (z. B. `/oakd/rgb/image`)
- `tracking_topic`: Ziel‑Topic für Blickrichtung (Standard `/person/target`)
- `scale_factor`, `min_neighbors`, `min_size`: OpenCV‑Haar‑Parameter

### Wichtig
Stelle sicher, dass dein Kamera‑Container das Bildtopic publiziert und `image_topic` in [config/eyes_display.yaml](config/eyes_display.yaml) dazu passt.
