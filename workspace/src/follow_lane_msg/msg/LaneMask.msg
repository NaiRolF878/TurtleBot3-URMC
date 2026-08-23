std_msgs/Header header

# Breite/Hoehe des Bird's-Eye-Bildes (quadratisch), aus dem die Koordinaten stammen.
# Identisch zu FollowLaneDebug.crop_im_size, hier redundant mitgeschickt, damit
# Konsumenten dieser Message nicht zusaetzlich /detect/lane_debug abonnieren muessen.
int32 crop_im_size

# Pixelkoordinaten aller erkannten Linienpunkte im entzerrten Bird's-Eye-Bild
# (0,0 = oben links, x nach rechts, y nach unten). yellow_x/yellow_y sowie
# white_x/white_y sind jeweils parallele Arrays gleicher Laenge.
int32[] yellow_x
int32[] yellow_y
int32[] white_x
int32[] white_y
