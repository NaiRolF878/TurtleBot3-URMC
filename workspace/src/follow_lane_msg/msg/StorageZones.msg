# Zonenmodell der Storage-Erkundung (switch_control), fuer die Anzeige.
std_msgs/Header header          # odom
# Querlinien L0..Ln: Punkt auf der Spur, Richtung in die Zone.
geometry_msgs/Pose[] lines
# false = aus dem gleichen Abstand ergaenzt.
bool[] measured
# Ziffer je Zone (len(lines) - 1 Eintraege), 0 = unbekannt.
uint8[] digits
