# Ergänzungen der Organisatoren zum Reglement 2026

Mail der Organisatoren an alle Teams, ergänzt `R_glement_Robohub_v_2026.pdf`. Im Team
weitergegeben am 25.10.2026. In der Mail waren die Antworten blau markiert, hier stehen sie
als **Antwort**. Fragen und Antworten sind im Originalwortlaut.

## Was vom Reglement abweicht oder es ergänzt

| Thema | Reglement | Mail |
|---|---|---|
| Palette | 15 × 15 × 5 cm auf 10 cm hoher Stütze (S. 5) | 10 × 2 cm, ca. 3 cm tief, ca. 5 cm über dem Boden auf einer Stütze, Metallstreifen vorne, keine Markierung |
| Ablage | von Zone ① nach Zone ② (S. 5) | Position bestimmt die Jury; Schilder blau mit weißen Zeichen |
| Schranke | rot-weiß in Abbildung 3 (S. 4) | blau, Unterkante ca. 7 cm, Balken ca. 3 cm hoch; waagerecht = anhalten, senkrecht = weiterfahren |
| Kreuzung | Linienfarben nur in Abbildung 3 (S. 4) | bei freier Kreuzung geradeaus; Linien unterbrochen und vertauscht |
| Enclosure | Hindernisse zufällig (S. 4–5) | Ein- und Ausgang fest; ca. 20–30 min Zugang, Karte oder Trajektorie vorab erlaubt; Hindernisse können zwischen zwei Versuchen versetzt werden |

## Originaltext

> Dear all,
>
> Here are some questions relative to the challenge rules (thanks to Loïc for all comments) and in blue the answers.

### 4.1.2 Track following with barrier detection

- Dimensions and sensors: What are the approximate height and dimensions of the barrier? (Since if it is located below the LiDAR line of sight, detection should be performed exclusively via camera.)
- Visual appearance: What color is it (solid red, red/white striped, etc.)?
- Opening mechanism: Is it an obstacle manually removed from the field of view, or a barrier consisting of a horizontal bar mounted on a pivot support? If so, does vertical tilting during opening leave the barrier within the camera's field of view?

**Antwort:** [Bild der Schranke](https://e-partage.uha.fr/service/home/~/?auth=co&loc=fr&id=369605&part=2)

Here is a picture of the barrier. Its color will be in blue. The height from the ground is around 7 cm and the height of the barrier is around 3 cm.
The jury manually moves up/down the barrier. The barrier stay in the field of the camera.
Robot should detect the horizontal position to stop and the vertical position to go on.

### 4.1.3 Track following with intersection and the Right-of-Way rule

- Diagram compliance: Are the track border colors in Fig. 3 accurate to reality? As it stands, turning left (after giving way to Robot 2) appears to be the only natural trajectory that does not involve any color inversion.
- Crossing: How should the robot cross the intersection? Going straight or turning right implies handling track border color inversions.

**Antwort:** When the intersection is free from the right, the robot goes straight on. The white and yellow lines are disrupted, and inverted.

### 4.2.1 Navigating and exiting an enclosure

- Does the position of the entrance and exit remain fixed throughout the event? If so, this allows for blind navigation using odometry alone.
- Prior mapping: Is it permitted to use test sessions to build a static map (with and without obstacles) for subsequent navigation (as opposed to real-time SLAM)? Are all obstacles randomly placed for each attempt (which would render the prior mapping strategy obsolete)?

**Antwort:** The entrance and exit remain fixed throughout the task.
For the challenge, you can access to the experimental erea (the track + closed area) for a fixed time (around 20~30 miniute). You can manage your time as you want to build a map, define a trajectory, etc.
But the obstacles can be moved between two attemps.

### 4.2.2 Storage / retrieval maneuver

- Geometric inconsistency (Palette & Support):
  - A 10 cm support combined with a 5 cm pallet reaches a height of 15 cm, which is level with the TurtleBot's top plate. This places the front camera beneath the pallet level (risk of collision) and implies positioning the electromagnet at a height of 12.5 cm since the pallet starts at 10 cm, which contradicts Robot configuration in Fig. 1.
  - Could this instead be a 10 mm (1 cm) support?
- Signage: Do the "rear traffic signs" for zones ① to ③ correspond to those in Fig. 6 (stop, tunnel, turn right)? If not, what are their formats, dimensions, and heights? Does sign ① extend above the pallet height?
- Pallet identification: What is the intended method of identification (QR/ArUco code, specific color like the blue of the parallelepiped shape)?
- Wording error in the french document: Reporting a redundancy : "Elle peut se décomposer en 4 parties : Cette mission se décompose en 3 partie :".

**Antwort:** The palett is 10 cm x 2 cm. Its depth is around 3 cm (not 10 cm). The palett is placed about 5 cm from the ground on asupport. A metal strip is placed in front alllowing gripping by the magnet. There is no palett identification. The robot detects the palett, grips it and put in a position determined by the jury. The different positions are identified by a sign in color blue and characters in white.
