# Bitte an switch_control um einen Ablauf aus Grundbewegungen (Obstacle und Tunnel).

uint8 PASSAGE = 1    # zum Start der Geraden, auf ihr Ende drehen, langsam gerade bis zum Ende
uint8 UNWEDGE = 2    # freifahren: control_driving waehlt die freieste kleine Bewegung

uint8 type
# Nur PASSAGE: Start und Ende der Geraden (odom).
nav_msgs/Path path
