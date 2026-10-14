std_msgs/Header header

# Breite/Hoehe des entzerrten Kamera-Vollbilds, aus dem die Koordinaten stammen.
# Konsumenten pruefen damit, ob ihre Kalibrierung (z.B. die Bodenhomographie in
# lane_costmap_node) fuer genau diese Aufloesung gilt.
int32 image_width
int32 image_height

# Pixelkoordinaten aller erkannten Linienpunkte im entzerrten Kamerabild, NICHT im
# Bird's-Eye-Bild (0,0 = oben links, x nach rechts, y nach unten).
# yellow_x/yellow_y sowie white_x/white_y sind jeweils parallele Arrays gleicher Laenge.
int32[] yellow_x
int32[] yellow_y
int32[] white_x
int32[] white_y
