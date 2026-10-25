std_msgs/Header header

# Breite/Hoehe des Bilds, aus dem die Koordinaten stammen: die metrische
# Vogelperspektive /lokal/bev. Konsumenten pruefen damit, ob ihre Umrechnung
# (Parameter/bev.yaml) fuer genau dieses Bild gilt.
int32 image_width
int32 image_height

# Pixelkoordinaten aller erkannten Linienpunkte in /lokal/bev
# (0,0 = oben links, x nach rechts, y nach unten).
# yellow_x/yellow_y sowie white_x/white_y sind jeweils parallele Arrays gleicher Laenge.
int32[] yellow_x
int32[] yellow_y
int32[] white_x
int32[] white_y
