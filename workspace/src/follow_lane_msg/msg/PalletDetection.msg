std_msgs/Header header

# Palette im entzerrten Kamerabild gefunden. Alle anderen Felder gelten nur dann.
bool found

# Seitlicher Versatz Palettenmitte gegen Magnetmitte, geteilt durch die halbe
# Bildbreite: > 0 = Palette links vom Magneten (REP 103), etwa -1..1.
float32 lateral

# Senkrechter Abstand in Pixeln von der Unterkante der Palette bis zur Oberkante des
# Magneten; <= 0 = beruehrt oder ueberlappt.
float32 gap_px

# Breite der Palette im Bild (px).
float32 width_px
