#!/usr/bin/env python3
"""Wertet zwei Brettmessungen (vor/nach gerader Fahrt) aus -> Winkel Kamera <-> Fahrtrichtung.

Ablauf:
  1. python3 docs/bev-tests/board-position.py --save docs/bev-tests/yaw_vorher.json
  2. python3 docs/bev-tests/drive-straight.py                 (15 cm gerade rueckwaerts)
  3. python3 docs/bev-tests/board-position.py --save docs/bev-tests/yaw_nachher.json
  4. python3 docs/bev-tests/yaw-auswertung.py

Das Brett liegt fest, der Bot faehrt gerade. Im Kamera-Bodensystem (vor/links ab Linse, so
wie die Kamera "geradeaus" sieht) wandert das Brett dann exakt entlang der ECHTEN
Fahrtrichtung. Wandert es dabei seitlich, schaut die Kamera schraeg.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    # Optional: python3 yaw-auswertung.py <vorher.json> <nachher.json>
    fa, fb = (sys.argv[1], sys.argv[2]) if len(sys.argv) == 3 else \
        (os.path.join(HERE, 'yaw_vorher.json'), os.path.join(HERE, 'yaw_nachher.json'))
    a = json.load(open(fa))['median']
    b = json.load(open(fb))['median']
    df, dl = b['cf'] - a['cf'], b['lat'] - a['lat']
    dist = math.hypot(df, dl)
    ang = math.degrees(math.atan2(dl, df))          # Fahrtrichtung im Kamerasystem
    drot = b['yaw'] - a['yaw']
    print(f'Brettmitte vorher : {a["cf"]*100:6.2f} cm vor, {a["lat"]*100:+6.2f} cm links')
    print(f'Brettmitte nachher: {b["cf"]*100:6.2f} cm vor, {b["lat"]*100:+6.2f} cm links')
    print(f'Verschiebung      : {df*100:+6.2f} cm vor, {dl*100:+6.2f} cm seitlich  (Betrag {dist*100:.2f} cm)')
    print(f'Brettdrehung      : {drot:+.2f} Grad (muss ~0 sein, sonst ist der Bot nicht gerade gefahren)')
    print()
    if abs(drot) > 0.5:
        print('WARNUNG: Bot hat sich gedreht -- Ergebnis unbrauchbar, Fahrt wiederholen.')
    side = 'links' if ang > 0 else 'rechts'
    # Faehrt der Bot zurueck, wandert das Brett "nach vorn" in Richtung der echten Fahrtrichtung.
    # ang = Winkel der echten Bot-Laengsachse im Kamerasystem.
    print(f'Echte Fahrtrichtung liegt {abs(ang):.2f} Grad {side} von der Kamera-Blickrichtung.')
    print(f'=> Kamera schaut {abs(ang):.2f} Grad nach {"rechts" if ang > 0 else "links"} '
          f'gegen die Fahrtrichtung (yaw_deg = {-ang:+.2f}, positiv = nach links).')
    print(f'   Bei 50 cm voraus entspricht das {math.tan(math.radians(abs(ang)))*50:.1f} cm seitlich.')


if __name__ == '__main__':
    sys.exit(main())
