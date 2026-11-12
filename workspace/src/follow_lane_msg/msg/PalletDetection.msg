# Zone und Palette in der Vogelperspektive (storage_zone_node), base_footprint zum Bildzeitpunkt.
std_msgs/Header header

# Anteil der Zonenflaeche, die weder dunkler Boden noch weisse Linie ist (0..1); -1 = nicht gemessen.
float32 fill

# Vorderkante der Palette (Bodenkante der Stuetze) gefunden. Die Felder darunter gelten nur dann.
bool found
# Radachse -> Vorderkante in Fahrtrichtung (m).
float32 distance
# Mitte der Vorderkante, links positiv (m).
float32 lateral
# So weit muesste der Bot drehen, um rechtwinklig zur Vorderkante zu stehen, links positiv (rad).
float32 yaw
