# TurtleBot3 – Upper Rhine Mobile Robotic Challenge

## Wer wir sind

Wir sind Studierende des Masterstudiengangs **Robotik und künstliche Intelligenz in der Produktion (RKIM)** an der [Hochschule Karlsruhe (HKA)](https://www.h-ka.de/rkim).

Die Begeisterung für autonome Roboter entstand in der Vorlesung Robogistic, in der wir mit Duckiebots gearbeitet haben. Das hat so viel Spaß gemacht, dass wir auf Nachfrage unseres Professors Lust hatten, an der **Upper Rhine Mobile Robotic Challenge (URMC) 2026** in Mulhouse teilzunehmen.

## Über den Wettbewerb

Bei der URMC programmieren Teams standardisierte TurtleBot3 Burger Roboter, die sich autonom und sicher durch eine simulierte urbane Testumgebung bewegen müssen.

## Tech-Stack

- **Roboter:** TurtleBot3 Burger (LiDAR, 2D-Kamera, IMU, Rad-Encoder)
- **Software:** ROS2 Humble, Gazebo Classic, OpenCV
- **Betriebssystem:** Ubuntu 22.04 (im Docker-Container)
- **Sprache:** Python 3

## Projekt starten

### Voraussetzungen

- Docker Desktop (Windows) oder Docker Engine (Linux)
- VS Code mit der Erweiterung „Dev Containers“

### Quickstart

1. Repo in VS Code öffnen und **Reopen in Container** wählen (`.devcontainer/`,
   ROS 2 Humble). Beim ersten Start werden die TurtleBot3-Pakete gebaut und die
   Kurzbefehle (`tb`, `botup`, `lanup` …) angelegt.
2. Workspace bauen:
   ```bash
   cd workspace && colcon build --symlink-install
   ```
3. Bot starten und Pipeline laufen lassen: `docs/Befehle.md`.

Das Konzept des ausgecheckten Branches steht in `docs/Konzept_*.md`.

## Ordnerstruktur

```
.
├── .devcontainer/           ← Container (Dockerfile, devcontainer.json, DDS-Profil)
├── docs/                    ← Konzept, Befehle, Kalibrierung, Nav2-Doku
├── scripts/tb/              ← Bot-Start, Netz, Kalibrierwerkzeuge, Launcher (Menü `tb`)
└── workspace/               ← ROS2-Workspace
    └── src/
        ├── Follow_Lane/     ← Unser Package
        │   ├── Follow_Lane/src/        ← Nodes (Python)
        │   ├── Follow_Lane/Parameter/  ← Parameter (YAML)
        │   ├── launch/                 ← Startdateien
        │   └── rviz/                   ← RViz-Konfiguration
        └── follow_lane_msg/ ← eigene Messages
```
