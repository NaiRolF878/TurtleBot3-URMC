# TurtleBot3 – Upper Rhine Mobile Robotic Challenge

## Wer wir sind

Wir sind Studierende des Masterstudiengangs **Robotik und künstliche Intelligenz in der Produktion (RKIM)** an der [Hochschule Karlsruhe (HKA)](https://www.h-ka.de/rkim).

Die Begeisterung für autonome Roboter entstand in der Vorlesung Robogistic, in der wir mit Duckiebots gearbeitet haben. Das hat so viel Spaß gemacht, dass wir auf Nachfrage unseres Professors Lust hatten, an der **Upper Rhine Mobile Robotic Challenge (URMC) 2026** in Mulhouse teilzunehmen.

## Über den Wettbewerb

Bei der URMC programmieren Teams standardisierte TurtleBot3 Burger Roboter, die sich autonom und sicher durch eine simulierte urbane Testumgebung bewegen müssen.

## Tech-Stack

- **Roboter:** TurtleBot3 Burger (LiDAR, 2D-Kamera, IMU, Rad-Encoder)
- **Software:** ROS2 Jazzy, Gazebo Harmonic, OpenCV
- **Betriebssystem:** Ubuntu 24.04 (im Docker-Container)
- **Sprache:** Python 3

## Projekt starten

### Voraussetzungen

- Docker Desktop (Windows) oder Docker Engine (Linux)
- Mindestens 4 CPU-Kerne, 8 GB RAM, 20 GB Speicher

### Quickstart (Linux)

```bash
# X11-Zugriff erlauben
xhost +local:docker

# Container bauen & starten
docker compose up -d --build

# In den Container einsteigen
docker exec -it tb3_urmc bash

# Simulation testen
ros2 launch urmc simulation.launch.py
```

## Ordnerstruktur

```
.
├── docker/                  ← Docker-Dateien
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── entrypoint.sh
│   └── novnc_startup.sh
│
└── workspace/               ← ROS2-Workspace (wird in den Container gemountet)
    └── src/
        └── urmc/            ← Unser Package
            ├── urmc/        ← Nodes (Python)
            ├── config/      ← Parameter (JSON)
            └── launch/      ← Startdateien
```
