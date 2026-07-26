# TurtleBot3 URMC – Docker Entwicklungsumgebung

Minimaler Docker-Container für die **Upper Rhine Mobile Robotic Challenge**.

## Inhalt

```
turtlebot3_docker/
├── Dockerfile            # Container-Definition
├── docker-compose.yml    # Start-Konfiguration
├── entrypoint.sh         # ROS2 Autostart-Script
├── novnc_startup.sh      # noVNC Script (nur Windows)
├── workspace/            # Dein Code (wird gemountet)
│   └── .bash_aliases     # Schnellbefehle
└── README.md
```

## Schnellstart (Linux mit X11)

```bash
# 1. X11-Zugriff erlauben
xhost +local:docker

# 2. Container bauen & starten
docker compose up -d --build

# 3. In den Container einsteigen
docker exec -it tb3_urmc bash

# 4. Simulation starten (im Container)
tb3_empty        # Leere Welt
tb3_world        # Welt mit Hindernissen
tb3_teleop       # Tastatursteuerung (neues Terminal)
```

## Windows (noVNC) aktivieren

1. Im **Dockerfile**: noVNC-Block einkommentieren
2. In **docker-compose.yml**: X11-Zeilen auskommentieren, noVNC-Zeilen einkommentieren
3. `docker compose up -d --build`
4. Browser öffnen: `http://localhost:6080` (Passwort: `ros`)

## YOLO aktivieren

Im **Dockerfile** den YOLO-Block einkommentieren, dann:
```bash
docker compose up -d --build
```

## Nützliche Befehle

| Befehl       | Beschreibung                      |
|------------- |-----------------------------------|
| `tb3_empty`  | Gazebo: leere Welt                |
| `tb3_world`  | Gazebo: Welt mit Hindernissen     |
| `tb3_house`  | Gazebo: Haus-Umgebung             |
| `tb3_teleop` | Tastatursteuerung                 |
| `tb3_slam`   | SLAM mit Cartographer             |
| `tb3_nav`    | Navigation2 starten               |
| `cb`         | Workspace bauen                   |
| `sb`         | Workspace sourcen                 |
| `topics`     | Alle ROS2 Topics anzeigen         |
| `nodes`      | Alle ROS2 Nodes anzeigen          |

## Systemanforderungen

- **Minimum**: 4 CPU-Kerne, 8 GB RAM, 20 GB Speicher
- **Empfohlen**: 6+ Kerne, 16 GB RAM, 50 GB Speicher
