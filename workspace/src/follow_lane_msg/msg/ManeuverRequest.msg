# Bitte an switch_control um einen Ablauf aus Grundbewegungen (Obstacle und Tunnel).

uint8 PASSAGE = 1    # zum Start der Geraden, auf ihr Ende drehen, langsam gerade bis zum Ende
uint8 UNWEDGE = 2    # freifahren: control_driving waehlt die freieste kleine Bewegung
uint8 FOLLOW = 3     # Weg genau abfahren (statt RPP): auf seine Richtung drehen, Punkten folgen

uint8 type
# PASSAGE: Start und Ende der Geraden (odom). UNWEDGE: zuletzt gefahrener Weg (Richtung zum
# Drehen, darf leer sein). FOLLOW: die abzufahrenden Wegpunkte (odom).
nav_msgs/Path path
