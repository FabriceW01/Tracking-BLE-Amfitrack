"""
Geschwindigkeitsprofil und Deckung gegen Geschwindigkeit
=========================================================

Eigenständiges Werkzeug für Test 7 (Geschwindigkeitslimit).

Zwei Aufgaben:

**7a — Grenzgeschwindigkeit aus einem einzigen Druck.** Du druckst eine
Vollfläche und beschleunigst dabei absichtlich von sehr langsam bis schnell.
Auf dem Papier suchst du die Stelle, ab der die Deckung einbricht, und willst
wissen, wie schnell du dort warst. Genau das macht ``--bei-u``: es sucht die
Stelle in der Profil-CSV und gibt die Geschwindigkeit aus. Der Plot zeigt
zusätzlich das ganze Profil, damit du siehst, ob du überhaupt den nötigen
Bereich abgedeckt hast.

**7b — Deckung gegen Geschwindigkeit.** Mehrere Durchgänge mit
unterschiedlichem Tempo. Je Durchgang holt sich das Werkzeug die mittlere
Geschwindigkeit aus der CSV; die Deckung in Prozent gibst du dazu (die Zahl
„Covered N/M" aus der Programmausgabe). Daraus wird die Geschwindigkeit
interpoliert, bei der die Deckung unter eine Schwelle fällt.

Datenquelle
-----------
Die ``--profile-csv`` aus dem Druckdurchgang:

    python main.py --pattern solid --pattern-length-mm 200 --mode page \\
        --page-calibration page_calibration.json \\
        --profile --profile-csv geschw.csv --record geschw.png

Beide CSV-Formate werden gelesen:
  * Seiten-Modus: ``t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,x,y,z,qx..qw``
    -- Position ist ``u_mm``.
  * Line-Modus: ``t_s,column,advance_mm,write_latency_ms,speed_mm_s,x,y,z``
    -- Position ist ``advance_mm``.

⚠️ Im Seiten-Modus wird nur bei **Musterwechseln** geschrieben. Steht der
Wagen oder ändert sich nichts, entstehen Lücken — das ist normal und für diese
Auswertung unschädlich, weil hier über die Position und nicht über die Zeit
ausgewertet wird.

Diese Datei ist bewusst UNABHÄNGIG vom printhead-Paket, damit sie allein
kopierbar bleibt.

Benutzung
---------
    python funktionen/geschwindigkeit_profil.py geschw.csv --png profil.png
    python funktionen/geschwindigkeit_profil.py geschw.csv --bei-u 137
    python funktionen/geschwindigkeit_profil.py lauf*.csv --deckung 99,96,73,44
"""

import argparse
import csv
import glob
import math
import os
import sys

# Standardschwelle für "die Deckung bricht ein" (Prozent).
DECKUNGS_SCHWELLE = 95.0


# ===========================================================================
# Einlesen
# ===========================================================================
def lies_profil_csv(pfad):
    """
    Liest eine ``--profile-csv`` und gibt ``(positionen, geschwindigkeiten,
    modus)`` zurück.

    Der Modus wird an den Spalten erkannt: ``u_mm`` -> Seiten-Modus,
    ``advance_mm`` -> Line-Modus. Beide tragen ``speed_mm_s``.
    """
    with open(pfad, newline="", encoding="utf-8") as datei:
        leser = csv.DictReader(datei)
        felder = leser.fieldnames or []
        if "speed_mm_s" not in felder:
            raise ValueError(
                f"{pfad!r} hat keine Spalte speed_mm_s (gefunden: "
                f"{','.join(felder) or '<keine>'}). Das sieht nicht nach einer "
                f"--profile-csv aus.")
        if "u_mm" in felder:
            spalte, modus = "u_mm", "page"
        elif "advance_mm" in felder:
            spalte, modus = "advance_mm", "line"
        else:
            raise ValueError(
                f"{pfad!r} hat weder u_mm (Seiten-Modus) noch advance_mm "
                f"(Line-Modus) — keine Positionsspalte zum Auswerten.")

        positionen, tempi = [], []
        for zeile in leser:
            try:
                p = float(zeile[spalte])
                v = float(zeile["speed_mm_s"])
            except (TypeError, ValueError, KeyError):
                continue
            if math.isnan(p) or math.isnan(v):
                continue
            positionen.append(p)
            tempi.append(v)
    return positionen, tempi, modus


# ===========================================================================
# Rechnung
# ===========================================================================
def _mittel(werte):
    return sum(werte) / len(werte) if werte else 0.0


def geschwindigkeit_bei(positionen, tempi, ziel_u, fenster_mm=1.0):
    """
    Mittlere Geschwindigkeit in der Umgebung von ``ziel_u``.

    Gemittelt über ein Fenster statt ein einzelner Messwert: die Rohwerte
    schwanken von Sample zu Sample (die Geschwindigkeit ist eine Differenz
    aufeinanderfolgender Positionen und verstärkt damit Rauschen), ein
    einzelner Wert wäre also zufälliger als die Größe, die man wissen will.

    Rückgabe ``(mittel, minimum, maximum, anzahl)`` oder ``None``, wenn im
    Fenster nichts liegt.
    """
    treffer = [v for p, v in zip(positionen, tempi)
               if abs(p - ziel_u) <= fenster_mm]
    if not treffer:
        # Fenster leer -- den nächstgelegenen Punkt nehmen, aber die Distanz
        # mitgeben, damit der Aufrufer sagen kann, wie weit daneben er lag.
        if not positionen:
            return None
        index = min(range(len(positionen)),
                    key=lambda i: abs(positionen[i] - ziel_u))
        return {"mittel": tempi[index], "min": tempi[index],
                "max": tempi[index], "anzahl": 1,
                "abstand_mm": abs(positionen[index] - ziel_u)}
    return {"mittel": _mittel(treffer), "min": min(treffer),
            "max": max(treffer), "anzahl": len(treffer), "abstand_mm": 0.0}


def profil_kennzahlen(positionen, tempi):
    """Überblick über einen Durchgang."""
    return {
        "punkte": len(positionen),
        "u_min": min(positionen) if positionen else 0.0,
        "u_max": max(positionen) if positionen else 0.0,
        "v_mittel": _mittel(tempi),
        "v_min": min(tempi) if tempi else 0.0,
        "v_max": max(tempi) if tempi else 0.0,
        "v_median": _median(tempi),
    }


def _median(werte):
    if not werte:
        return 0.0
    s = sorted(werte)
    m = len(s) // 2
    return s[m] if len(s) % 2 else (s[m - 1] + s[m]) / 2.0


def deckung_gegen_tempo(laeufe, schwelle=DECKUNGS_SCHWELLE):
    """
    Test 7b: interpoliert die Geschwindigkeit, bei der die Deckung unter
    ``schwelle`` fällt.

    ``laeufe`` ist eine Liste von dicts mit ``v_mittel`` und ``deckung``.

    Rückgabe ``(grenze, art)``; ``art`` ist ``"interpoliert"``,
    ``"unterhalb"`` (schon der langsamste Lauf liegt unter der Schwelle) oder
    ``"oberhalb"`` (kein Lauf fällt darunter).
    """
    sortiert = sorted(laeufe, key=lambda l: l["v_mittel"])
    if not sortiert:
        return None, "oberhalb"
    if sortiert[0]["deckung"] < schwelle:
        return sortiert[0]["v_mittel"], "unterhalb"
    for langsam, schnell in zip(sortiert, sortiert[1:]):
        if schnell["deckung"] < schwelle:
            spanne = langsam["deckung"] - schnell["deckung"]
            if spanne <= 0:
                return schnell["v_mittel"], "interpoliert"
            anteil = (langsam["deckung"] - schwelle) / spanne
            return (langsam["v_mittel"]
                    + anteil * (schnell["v_mittel"] - langsam["v_mittel"])), \
                   "interpoliert"
    return None, "oberhalb"


# ===========================================================================
# Bericht
# ===========================================================================
def bericht(laeufe, bei_u=None, schwelle=DECKUNGS_SCHWELLE):
    zeilen = ["---- Geschwindigkeitsprofil ----"]
    zeilen.append("  Datei                          Modus  Punkte   "
                  "u von .. bis     v mittel  v median   v max")
    for lauf in laeufe:
        name = os.path.basename(lauf["name"])
        if len(name) > 28:
            name = "..." + name[-25:]
        k = lauf["kennzahlen"]
        zeilen.append(
            f"  {name:<30} {lauf['modus']:<5} {k['punkte']:>7} "
            f"{k['u_min']:>7.1f} ..{k['u_max']:>7.1f} "
            f"{k['v_mittel']:>9.1f} {k['v_median']:>9.1f} {k['v_max']:>7.1f}")
    zeilen.append("  (u in mm, v in mm/s)")

    if bei_u is not None:
        zeilen.append("")
        zeilen.append(f"  Geschwindigkeit bei u = {bei_u:.1f} mm:")
        for lauf in laeufe:
            treffer = lauf.get("bei_u")
            name = os.path.basename(lauf["name"])
            if treffer is None:
                zeilen.append(f"    {name}: keine Daten")
                continue
            if treffer["abstand_mm"] > 0:
                zeilen.append(
                    f"    {name}: {treffer['mittel']:.1f} mm/s  "
                    f"(nichts im Fenster — nächster Punkt liegt "
                    f"{treffer['abstand_mm']:.1f} mm daneben)")
            else:
                zeilen.append(
                    f"    {name}: {treffer['mittel']:.1f} mm/s   "
                    f"(Bereich {treffer['min']:.1f}..{treffer['max']:.1f} "
                    f"aus {treffer['anzahl']} Werten)")

    mit_deckung = [l for l in laeufe if l.get("deckung") is not None]
    if mit_deckung:
        zeilen.append("")
        zeilen.append("  Deckung gegen Geschwindigkeit (Test 7b):")
        zeilen.append("    v mittel (mm/s)   Deckung (%)   Datei")
        for lauf in sorted(mit_deckung, key=lambda l: l["kennzahlen"]["v_mittel"]):
            zeilen.append(f"    {lauf['kennzahlen']['v_mittel']:>15.1f}   "
                          f"{lauf['deckung']:>11.1f}   "
                          f"{os.path.basename(lauf['name'])}")
        grenze, art = deckung_gegen_tempo(
            [{"v_mittel": l["kennzahlen"]["v_mittel"], "deckung": l["deckung"]}
             for l in mit_deckung], schwelle)
        zeilen.append("")
        if art == "interpoliert":
            zeilen.append(f"    Die Deckung fällt bei etwa "
                          f"**{grenze:.1f} mm/s** unter {schwelle:.0f} %.")
        elif art == "unterhalb":
            zeilen.append(f"    Schon der langsamste Durchgang "
                          f"({grenze:.1f} mm/s) liegt unter {schwelle:.0f} % — "
                          f"die Grenze liegt darunter und ist nicht erfasst. "
                          f"Langsamer nachmessen.")
        else:
            schnellster = max(l["kennzahlen"]["v_mittel"] for l in mit_deckung)
            zeilen.append(f"    Bis {schnellster:.1f} mm/s bleibt die Deckung "
                          f"über {schwelle:.0f} % — die Grenze liegt höher und "
                          f"ist nicht erfasst. Schneller nachmessen.")
        if len(mit_deckung) < 3:
            zeilen.append(f"    (nur {len(mit_deckung)} Durchgänge — für eine "
                          f"belastbare Kurve besser 4-5)")

    zeilen.append("")
    zeilen.append("  Abgleich mit der Vorhersage: seit der Umstellung auf das "
                  "Tropfenmodell haengt die Tinte am zurueckgelegten Weg, "
                  "nicht mehr an der Verweildauer — die Deckung soll also "
                  "flach bei 100 % bleiben, bis die Abtastrate nicht mehr "
                  "mitkommt. Diese Kante liegt bei mm_per_column * poll_hz "
                  "(0,087 * 500 = 43,5 mm/s); simuliert: 100 % bis 43,5, "
                  "95 % bei 46, 86,7 % bei 50, 72,5 % bei 60. Faellt die "
                  "Messung deutlich frueher ab, liegt es nicht am "
                  "Dosiermodell — dann zuerst --poll-hz und die BLE-Rate "
                  "(--profile) pruefen. Zum Vergleich das alte "
                  "Verweildauer-Modell: 100 % bei <=17,3 mm/s, 60 % bei 25, "
                  "14 % bei 35.")
    return "\n".join(zeilen)


# ===========================================================================
# Grafik
# ===========================================================================
_ACHSEN = (60, 60, 60)
_GITTER = (216, 216, 216)
_FARBEN = [(70, 130, 200), (220, 120, 40), (90, 170, 90), (180, 90, 180),
           (200, 170, 40), (100, 190, 190)]
_WARN_FARBE = (200, 60, 60)

# Warnschwelle des Clients (controller.DEFAULT_SPEED_WARNING_MM_S).
WARNSCHWELLE_MM_S = 25.0


def zeichne_plot(laeufe, pfad_png, bei_u=None, breite=1100, hoehe=620):
    """
    Geschwindigkeit gegen Position, eine Kurve je Durchgang.

    Eingezeichnet ist zusätzlich die Warnschwelle von 25 mm/s (der Wert, ab
    dem der Client die Firmware-LED ansteuert) — damit sofort sichtbar ist,
    welcher Teil der Fahrt überhaupt im kritischen Bereich lag.
    """
    from PIL import Image, ImageDraw

    if not laeufe:
        return False

    rand_l, rand_r, rand_o, rand_u = 80, 185, 50, 66
    pl_b, pl_h = breite - rand_l - rand_r, hoehe - rand_o - rand_u

    alle_p = [p for l in laeufe for p in l["positionen"]]
    alle_v = [v for l in laeufe for v in l["tempi"]]
    if not alle_p:
        return False
    x_min, x_max = min(alle_p), max(alle_p)
    if x_max - x_min < 1e-9:
        x_max = x_min + 1.0
    # Die Warnschwelle soll immer im Bild sein, auch bei durchweg langsamen
    # Fahrten -- sonst fehlt der Maßstab, gegen den die Kurve gelesen wird.
    y_max = max(max(alle_v), WARNSCHWELLE_MM_S * 1.2) * 1.12

    def px(p):
        return rand_l + (p - x_min) / (x_max - x_min) * pl_b

    def py(v):
        return rand_o + (1.0 - v / y_max) * pl_h

    bild = Image.new("RGB", (breite, hoehe), (255, 255, 255))
    z = ImageDraw.Draw(bild)
    schrift, klein = _schrift(13), _schrift(11)

    for anteil in [i / 6.0 for i in range(7)]:
        x = rand_l + anteil * pl_b
        z.line([(x, rand_o), (x, rand_o + pl_h)], fill=_GITTER)
        z.text((x - 14, rand_o + pl_h + 8),
               f"{x_min + anteil * (x_max - x_min):.0f}", fill=_ACHSEN, font=klein)
        y = rand_o + anteil * pl_h
        z.line([(rand_l, y), (rand_l + pl_b, y)], fill=_GITTER)
        z.text((10, y - 6), f"{y_max * (1.0 - anteil):.0f}", fill=_ACHSEN,
               font=klein)

    # Warnschwelle
    y_warn = py(WARNSCHWELLE_MM_S)
    if rand_o <= y_warn <= rand_o + pl_h:
        _gestrichelt(z, rand_l, y_warn, rand_l + pl_b, _WARN_FARBE)
        z.text((rand_l + 4, y_warn - 14),
               f"Warnschwelle {WARNSCHWELLE_MM_S:.0f} mm/s",
               fill=_WARN_FARBE, font=klein)

    # Abgefragte Position
    if bei_u is not None and x_min <= bei_u <= x_max:
        _gestrichelt_v(z, px(bei_u), rand_o, rand_o + pl_h, (110, 110, 180))
        z.text((px(bei_u) + 4, rand_o + 4), f"u = {bei_u:.0f} mm",
               fill=(80, 80, 160), font=klein)

    for index, lauf in enumerate(laeufe):
        farbe = _FARBEN[index % len(_FARBEN)]
        # Nach Position sortiert zeichnen: die CSV steht in Zeitreihenfolge,
        # bei einer Hin- und Rückfahrt läge die Kurve sonst doppelt und
        # sähe wie ein Zickzack aus.
        paare = sorted(zip(lauf["positionen"], lauf["tempi"]))
        punkte = [(px(p), py(v)) for p, v in paare]
        if len(punkte) >= 2:
            z.line(punkte, fill=farbe, width=1)

    z.rectangle([rand_l, rand_o, rand_l + pl_b, rand_o + pl_h], outline=_ACHSEN)
    z.text((rand_l, 16), "Geschwindigkeit über die Strecke",
           fill=(20, 20, 20), font=schrift)
    z.text((rand_l + pl_b / 2 - 45, hoehe - 24), "Position (mm)",
           fill=_ACHSEN, font=klein)
    z.text((8, rand_o - 22), "v (mm/s)", fill=_ACHSEN, font=klein)

    lx, ly = rand_l + pl_b + 14, rand_o + 4
    for index, lauf in enumerate(laeufe):
        farbe = _FARBEN[index % len(_FARBEN)]
        z.line([(lx, ly + 6), (lx + 20, ly + 6)], fill=farbe, width=2)
        name = os.path.basename(lauf["name"])
        if len(name) > 19:
            name = name[:16] + "..."
        z.text((lx + 26, ly), name, fill=(40, 40, 40), font=klein)
        if lauf.get("deckung") is not None:
            ly += 14
            z.text((lx + 26, ly), f"  {lauf['deckung']:.0f} % Deckung",
                   fill=(110, 110, 110), font=klein)
        ly += 18

    bild.save(pfad_png)
    return True


def _gestrichelt(z, x1, y, x2, farbe, strich=6, luecke=5):
    x = x1
    while x < x2:
        z.line([(x, y), (min(x + strich, x2), y)], fill=farbe)
        x += strich + luecke


def _gestrichelt_v(z, x, y1, y2, farbe, strich=6, luecke=5):
    y = y1
    while y < y2:
        z.line([(x, y), (x, min(y + strich, y2))], fill=farbe)
        y += strich + luecke


def _schrift(groesse):
    from PIL import ImageFont
    for pfad in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                 "C:/Windows/Fonts/segoeui.ttf", "C:/Windows/Fonts/arial.ttf",
                 "/System/Library/Fonts/Helvetica.ttc"):
        try:
            return ImageFont.truetype(pfad, groesse)
        except (OSError, ImportError):
            continue
    try:
        return ImageFont.load_default(groesse)
    except TypeError:
        return ImageFont.load_default()


# ===========================================================================
# Kommandozeile
# ===========================================================================
def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="geschwindigkeit_profil",
        description="Geschwindigkeitsprofil aus einer --profile-csv; "
                    "optional Deckung gegen Geschwindigkeit über mehrere "
                    "Durchgänge.")
    ap.add_argument("dateien", nargs="+", help="Eine oder mehrere --profile-csv")
    ap.add_argument("--png", default="geschwindigkeit.png",
                    help="Dateiname der Grafik (Default geschwindigkeit.png)")
    ap.add_argument("--bei-u", type=float, default=None,
                    help="Position in mm, für die die Geschwindigkeit "
                         "ausgegeben werden soll (Test 7a)")
    ap.add_argument("--fenster-mm", type=float, default=1.0,
                    help="Mittelungsfenster um --bei-u (Default 1.0 mm)")
    ap.add_argument("--deckung", default=None,
                    help="Deckung in Prozent je Datei, kommagetrennt, in "
                         "Reihenfolge der Dateien (Test 7b), z.B. "
                         "'99,96,73,44'")
    ap.add_argument("--schwelle", type=float, default=DECKUNGS_SCHWELLE,
                    help=f"Deckungsschwelle in Prozent "
                         f"(Default {DECKUNGS_SCHWELLE:g})")
    ap.add_argument("--kein-plot", action="store_true")
    args = ap.parse_args(sys.argv[1:] if argv is None else argv)

    pfade = []
    for muster in args.dateien:
        treffer = sorted(glob.glob(muster))
        pfade.extend(treffer if treffer else [muster])

    deckungen = None
    if args.deckung:
        try:
            deckungen = [float(t.strip().replace(",", "."))
                         for t in args.deckung.split(",") if t.strip()]
        except ValueError as fehler:
            print(f"[geschwindigkeit] --deckung: {fehler}")
            return 2

    laeufe = []
    for pfad in pfade:
        try:
            positionen, tempi, modus = lies_profil_csv(pfad)
        except (OSError, ValueError) as fehler:
            print(f"[geschwindigkeit] {fehler}")
            continue
        if len(positionen) < 2:
            print(f"[geschwindigkeit] {pfad}: keine verwertbaren Zeilen.")
            continue
        lauf = {"name": pfad, "modus": modus, "positionen": positionen,
                "tempi": tempi,
                "kennzahlen": profil_kennzahlen(positionen, tempi),
                "deckung": None}
        if args.bei_u is not None:
            lauf["bei_u"] = geschwindigkeit_bei(positionen, tempi, args.bei_u,
                                               args.fenster_mm)
        laeufe.append(lauf)

    if not laeufe:
        print("[geschwindigkeit] Keine auswertbare Datei.")
        return 2

    if deckungen is not None:
        if len(deckungen) != len(laeufe):
            print(f"[geschwindigkeit] --deckung hat {len(deckungen)} Werte, es "
                  f"sind aber {len(laeufe)} auswertbare Dateien.")
            return 2
        for lauf, wert in zip(laeufe, deckungen):
            lauf["deckung"] = wert

    print(bericht(laeufe, bei_u=args.bei_u, schwelle=args.schwelle))

    if not args.kein_plot:
        try:
            if zeichne_plot(laeufe, args.png, bei_u=args.bei_u):
                print(f"\n  Grafik geschrieben: {args.png}")
        except ImportError:
            print("\n[geschwindigkeit] Pillow (PIL) fehlt — der Textbericht "
                  "oben ist vollständig.")
        except OSError as fehler:
            print(f"\n[geschwindigkeit] Grafik konnte nicht geschrieben "
                  f"werden: {fehler}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
