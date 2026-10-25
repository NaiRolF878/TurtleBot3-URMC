std_msgs/Header header

# Kleinste gueltige LiDAR-Distanz je Zone (m), inf = frei. Zonengrenzen in Avoid_Node.yaml.
float32 front
float32 front_left
float32 front_right
float32 left
float32 right

# Offset auf den Spurfehler fuer den Lane-Modus, gleiche Konvention wie /detect/lane
# (positiv = nach links). Waechst stetig mit der Naehe des Hindernisses.
float32 offset

# Ausweichrichtung fuer den Obstacle-Modus: +1 links, -1 rechts, 0 geradeaus.
int8 direction
# false: Front zu nah oder kein freier Sektor -> nur auf der Stelle drehen.
bool forward_allowed
# Gewaehlter Sektor (Name aus Avoid_Node.yaml), nur zur Diagnose.
string sector
