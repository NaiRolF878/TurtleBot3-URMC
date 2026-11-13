# Grundbewegung: switch_control gibt sie vor, control_driving faehrt sie in odom ab und meldet
# das Ende auf /control/maneuver_status (ManeuverStatus).

uint8 ROTATE = 1     # auf der Stelle auf den Gierwinkel yaw (rad, odom) drehen
uint8 STRAIGHT = 2   # distance (m) geradeaus, < 0 = rueckwaerts, Richtung beim Start halten
uint8 PATH = 3       # die Punkte von path nacheinander anfahren, ohne an Zwischenpunkten zu halten
uint8 UNWEDGE = 4    # freifahren: control_driving waehlt die freieste kleine Bewegung

std_msgs/Header header

# Laufende Nummer; eine neue id startet eine neue Bewegung.
uint32 id
uint8 type

float64 yaw
float64 distance
nav_msgs/Path path
# Nur PATH: rueckwaerts fahren (Heck voraus).
bool reverse

# Hoechstgeschwindigkeit (m/s) fuer diese Bewegung; 0 = Parameter von control_driving.
float32 max_vel
