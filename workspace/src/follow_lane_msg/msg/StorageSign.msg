std_msgs/Header header

# Ziffer des Zonenschilds (blauer Kreis, weisse Ziffer 1-3); 0 = Schild gefunden,
# Ziffer aber nicht sicher genug. Ohne Schild im Bild wird nichts gesendet.
uint8 digit

# Sicherheit 0..1 (Vorlagenvergleich: Korrelation, Modell: Konfidenz).
float32 confidence

# Waagerechte Lage der Schildmitte im Bild, -1 = linker Rand, 1 = rechter Rand.
float32 image_x

# Wer erkannt hat: "vorlage" oder "modell".
string source
