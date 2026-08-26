"""
Sensorrauschen über die Entfernung zum Sender
==============================================

Eigenständiges Werkzeug. Wertet Aufzeichnungen aus, die bei **stillstehendem,
mechanisch fixiertem** Wagen in verschiedenen Abständen zum Amfitrack-Sender
gemacht wurden, und beantwortet die Kernfrage von Test 2a:

    Ab welchem Abstand überschreitet das Rauschen eine Düsenreihe (0,087 mm)?

Jenseits dieses Abstands begrenzt der Sensor die Druckqualität, unabhängig von
allem anderen — das ist die härteste Zahl der ganzen Messreihe, weil sie den
nutzbaren Arbeitsbereich festlegt.

Aufnahme
--------
    python main.py --pos --pos-json > rausch_d10.jsonl     # 10 cm Abstand
    python main.py --pos --pos-json > rausch_d20.jsonl     # 20 cm
    ...

Der Wagen muss **festgeklemmt oder festgeklebt** sein. In der Hand gehalten
misst man Handzittern, nicht den Sensor.

Rauschen gegen Drift
--------------------
Die Standardabweichung über die ganze Aufzeichnung enthält beides: schnelles
Rauschen **und** langsames Wegdriften. Das sind verschiedene Fehler mit
verschiedenen Folgen — Rauschen mittelt sich über eine Dosis teilweise heraus,
Drift nicht. Deshalb wird zusätzlich die Streuung **innerhalb kurzer Fenster**
berechnet: liegt sie deutlich unter der Gesamtstreuung, driftet der Sensor,
statt nur zu rauschen.

Zeitangaben
-----------
``--pos --pos-json`` schreibt **keinen Zeitstempel**. Die Abtastrate von
``--pos`` liegt fest bei 15 Hz (``diagnostics.monitor_position``, Parameter
``hz``), Sekundenangaben hier sind also daraus abgeleitet und keine Messung.
Mit ``--hz`` anpassbar, falls sich das je ändert.

Diese Datei ist bewusst UNABHÄNGIG vom printhead-Paket, damit sie allein
kopierbar bleibt. Ein Test im Repo hält die Düsenteilung gegen
``printhead.geometry.NOZZLE_PITCH_MM``.

Benutzung
---------
    python funktionen/rauschen_entfernung.py rausch_d*.jsonl --png rauschen.png

Der Abstand wird aus dem Dateinamen gelesen (erste Zahl darin), oder explizit:

    python funktionen/rauschen_entfernung.py a.jsonl b.jsonl --abstaende 10,20

Zusätzlich Test 2b (Maßstabsfehler über Entfernung):

    ... --massstab 10=99.4,20=99.1,30=98.2 --referenz 100
"""

import argparse
import glob
import json
import math
import os
import re
import sys

# Siehe Modul-Docstring: bewusst lokal, per Test gegen die Geometrie gehalten.
DUESENTEILUNG_MM = 13.2 / 152

# Abtastrate von --pos (diagnostics.monitor_position, hz=15.0).
STANDARD_HZ = 15.0


# ===========================================================================
# Einlesen
# ===========================================================================
def lies_pos_json(pfad):
    """
    Liest eine ``--pos --pos-json``-Datei und gibt ``(xs, ys, zs)`` zurück.

    Zeilen ohne x/y/z (etwa ``{"event":"connected"}``) und kaputte Zeilen
    werden übersprungen — eine abgebrochene Aufzeichnung endet oft mit einer
    halben Zeile und soll trotzdem auswertbar bleiben.
    """
    xs, ys, zs = [], [], []
    with open(pfad, encoding="utf-8") as datei:
        for zeile in datei:
            zeile = zeile.strip()
            if not zeile:
                continue
            try:
                obj = json.loads(zeile)
            except ValueError:
                continue
            if not isinstance(obj, dict):
                continue
            if "x" in obj and "y" in obj and "z" in obj:
                try:
                    xs.append(float(obj["x"]))
                    ys.append(float(obj["y"]))
                    zs.append(float(obj["z"]))
                except (TypeError, ValueError):
                    continue
    return xs, ys, zs


def abstand_aus_name(pfad):
    """
    Liest den Abstand aus dem Dateinamen: die erste Zahl darin.

    ``rausch_d20.jsonl`` -> 20.0. Gibt ``None`` zurück, wenn keine Zahl im
    Namen steht — dann muss der Aufrufer ``--abstaende`` angeben, statt dass
    hier geraten wird.
    """
    name = os.path.basename(pfad)
    treffer = re.search(r"(\d+(?:[.,]\d+)?)", name)
    if not treffer:
        return None
    return float(treffer.group(1).replace(",", "."))


# ===========================================================================
# Rechnung
# ===========================================================================
def _mittel(werte):
    return sum(werte) / len(werte) if werte else 0.0


def _stdabw(werte):
    """Stichproben-Standardabweichung (n-1), 0.0 bei weniger als 2 Werten."""
    if len(werte) < 2:
        return 0.0
    m = _mittel(werte)
    return math.sqrt(sum((w - m) ** 2 for w in werte) / (len(werte) - 1))


def _spitze(werte):
    return (max(werte) - min(werte)) if werte else 0.0


def fenster_streuung(werte, fenster):
    """
    Mittlere Streuung **innerhalb** kurzer Fenster.

    Trennt schnelles Rauschen von langsamer Drift: ein Sensor, der über 20 s
    langsam wegläuft, hat eine große Gesamtstreuung, aber eine kleine
    Fensterstreuung. Ohne diese Unterscheidung sähe Drift wie Rauschen aus,
    obwohl sie sich anders auswirkt (Rauschen mittelt sich über eine Dosis
    teilweise heraus, Drift nicht).

    Gemittelt werden Varianzen, nicht Standardabweichungen: die Varianz ist
    die additive Größe, der Mittelwert von Standardabweichungen unterschätzt
    die Gesamtstreuung systematisch.
    """
    fenster = max(2, int(fenster))
    if len(werte) < fenster:
        return _stdabw(werte)
    varianzen = []
    for start in range(0, len(werte) - fenster + 1, fenster):
        stueck = werte[start:start + fenster]
        varianzen.append(_stdabw(stueck) ** 2)
    if not varianzen:
        return _stdabw(werte)
    return math.sqrt(sum(varianzen) / len(varianzen))


def werte_einer_datei(xs, ys, zs, fenster):
    """Kennzahlen einer einzelnen Aufzeichnung."""
    mx, my, mz = _mittel(xs), _mittel(ys), _mittel(zs)
    # 3D-Streuung: RMS-Abstand vom Mittelpunkt. Eine einzige Zahl, die alle
    # drei Achsen zusammenfasst -- das ist die Größe, die mit der
    # Düsenteilung verglichen wird.
    rms3d = math.sqrt(_mittel([(x - mx) ** 2 + (y - my) ** 2 + (z - mz) ** 2
                               for x, y, z in zip(xs, ys, zs)])) if xs else 0.0
    return {
        "punkte": len(xs),
        "mittel": (mx, my, mz),
        "sigma": (_stdabw(xs), _stdabw(ys), _stdabw(zs)),
        "spitze": (_spitze(xs), _spitze(ys), _spitze(zs)),
        "rms3d": rms3d,
        "fenster": (fenster_streuung(xs, fenster), fenster_streuung(ys, fenster),
                    fenster_streuung(zs, fenster)),
    }


def grenzabstand(punkte, schwelle=DUESENTEILUNG_MM):
    """
    Abstand, bei dem die 3D-Streuung ``schwelle`` überschreitet — linear
    zwischen den beiden Messpunkten interpoliert, die sie einrahmen.

    Rückgabe ``(abstand, art)`` mit ``art``:
      * ``"interpoliert"`` -- die Schwelle liegt zwischen zwei Messpunkten,
      * ``"unterhalb"``    -- schon der nächste Messpunkt liegt darüber,
      * ``"oberhalb"``     -- kein Messpunkt erreicht die Schwelle.
    """
    sortiert = sorted(punkte, key=lambda p: p["abstand"])
    if not sortiert:
        return None, "oberhalb"
    if sortiert[0]["rms3d"] >= schwelle:
        return sortiert[0]["abstand"], "unterhalb"
    for vorher, nachher in zip(sortiert, sortiert[1:]):
        if nachher["rms3d"] >= schwelle:
            spanne = nachher["rms3d"] - vorher["rms3d"]
            if spanne <= 0:
                return nachher["abstand"], "interpoliert"
            anteil = (schwelle - vorher["rms3d"]) / spanne
            return (vorher["abstand"]
                    + anteil * (nachher["abstand"] - vorher["abstand"])), "interpoliert"
    return None, "oberhalb"


def auswerten(messungen, fenster=15):
    """
    ``messungen`` ist eine Liste von ``(abstand_cm, name, xs, ys, zs)``.

    Rückgabe: dict mit ``fehler`` oder mit ``punkte`` (je Abstand ein Eintrag,
    nach Abstand sortiert) und dem Grenzabstand.
    """
    punkte = []
    for abstand, name, xs, ys, zs in messungen:
        if len(xs) < 2:
            continue
        eintrag = werte_einer_datei(xs, ys, zs, fenster)
        eintrag["abstand"] = float(abstand)
        eintrag["name"] = name
        punkte.append(eintrag)
    if not punkte:
        return {"fehler": "Keine Aufzeichnung mit mindestens zwei Punkten."}

    punkte.sort(key=lambda p: p["abstand"])
    grenze, art = grenzabstand(punkte)
    return {"punkte": punkte, "fenster": fenster,
            "grenzabstand": grenze, "grenzart": art}


def massstab_auswerten(paare, referenz_mm):
    """
    Test 2b: Maßstabsfehler über die Entfernung.

    ``paare`` ist eine Liste ``(abstand_cm, gemessene_strecke_mm)``,
    ``referenz_mm`` die mit dem Messschieber geprüfte wahre Strecke.
    """
    if referenz_mm <= 0:
        return {"fehler": "Referenzstrecke muss größer als 0 sein."}
    zeilen = []
    for abstand, gemessen in sorted(paare):
        fehler_mm = gemessen - referenz_mm
        zeilen.append({"abstand": float(abstand), "gemessen": float(gemessen),
                       "fehler_mm": fehler_mm,
                       "fehler_prozent": 100.0 * fehler_mm / referenz_mm})
    return {"referenz_mm": referenz_mm, "zeilen": zeilen}


# ===========================================================================
# Bericht
# ===========================================================================
def _reihen(mm):
    return mm / DUESENTEILUNG_MM


def bericht(ergebnis, hz=STANDARD_HZ, massstab=None):
    if "fehler" in ergebnis:
        return f"[rauschen] {ergebnis['fehler']}"

    zeilen = ["---- Sensorrauschen über die Entfernung ----"]
    zeilen.append(f"  Fenster für die Kurzzeit-Streuung: "
                  f"{ergebnis['fenster']} Samples "
                  f"(~{ergebnis['fenster'] / hz:.1f} s bei {hz:g} Hz)")
    zeilen.append("")
    zeilen.append("  Abstand  Punkte    ~Dauer   sigma_x  sigma_y  sigma_z   "
                  "3D-RMS  Spitze-Sp.  kurzfr.")
    zeilen.append("     (cm)                (s)      (mm)     (mm)     (mm)     "
                  "(mm)        (mm)     (mm)")
    for p in ergebnis["punkte"]:
        sx, sy, sz = p["sigma"]
        fx, fy, fz = p["fenster"]
        kurz = math.sqrt((fx ** 2 + fy ** 2 + fz ** 2))
        zeilen.append(
            f"  {p['abstand']:7.1f} {p['punkte']:7d} {p['punkte'] / hz:9.1f} "
            f"{sx:9.4f} {sy:8.4f} {sz:8.4f} {p['rms3d']:8.4f} "
            f"{max(p['spitze']):11.4f} {kurz:8.4f}")

    zeilen.append("")
    zeilen.append("  3D-RMS in Düsenreihen (0,087 mm je Reihe):")
    for p in ergebnis["punkte"]:
        marke = "  <-- über einer Düsenreihe" if p["rms3d"] >= DUESENTEILUNG_MM else ""
        zeilen.append(f"  {p['abstand']:7.1f} cm : {_reihen(p['rms3d']):6.2f} "
                      f"Reihen{marke}")

    zeilen.append("")
    zeilen.extend(_urteil(ergebnis))

    if massstab is not None:
        zeilen.append("")
        zeilen.extend(_massstab_zeilen(massstab))
    return "\n".join(zeilen)


def _urteil(ergebnis):
    zeilen = []
    grenze, art = ergebnis["grenzabstand"], ergebnis["grenzart"]
    schwelle = DUESENTEILUNG_MM

    if art == "oberhalb":
        groesster = max(p["abstand"] for p in ergebnis["punkte"])
        zeilen.append(f"  FAZIT: Bis {groesster:.0f} cm bleibt das Rauschen unter "
                      f"einer Düsenreihe ({schwelle:.4f} mm) — in diesem ganzen "
                      f"Bereich begrenzt der Sensor die Druckqualität nicht.")
        zeilen.append("         Wo die Grenze wirklich liegt, ist damit noch "
                      "offen: dafür bei größeren Abständen weitermessen.")
    elif art == "unterhalb":
        kleinster = min(p["abstand"] for p in ergebnis["punkte"])
        zeilen.append(f"  FAZIT: Schon bei {kleinster:.0f} cm liegt das Rauschen "
                      f"über einer Düsenreihe ({schwelle:.4f} mm). Die brauchbare "
                      f"Grenze liegt darunter und ist mit dieser Messreihe nicht "
                      f"erfasst — näher am Sender nachmessen.")
    else:
        zeilen.append(f"  FAZIT: Das Rauschen erreicht eine Düsenreihe "
                      f"({schwelle:.4f} mm) bei etwa **{grenze:.0f} cm**.")
        zeilen.append(f"         Darunter arbeiten. Jenseits davon begrenzt der "
                      f"Sensor die Druckqualität, unabhängig von Dosierung, "
                      f"BLE und Kalibrierung.")

    # Drift getrennt melden -- nur dort, wo die Kurzzeit-Streuung deutlich
    # unter der Gesamtstreuung liegt, ist wirklich Drift im Spiel.
    driftend = []
    for p in ergebnis["punkte"]:
        kurz = math.sqrt(sum(f ** 2 for f in p["fenster"]))
        if kurz > 0 and p["rms3d"] > 2.0 * kurz:
            driftend.append(p["abstand"])
    if driftend:
        liste = ", ".join(f"{a:.0f}" for a in driftend)
        zeilen.append(f"         DRIFT bei {liste} cm: die Gesamtstreuung ist "
                      f"dort mehr als doppelt so groß wie die Streuung "
                      f"innerhalb kurzer Fenster. Der Sensor läuft langsam weg, "
                      f"statt nur zu rauschen — das mittelt sich NICHT heraus. "
                      f"Prüfen, ob der Wagen wirklich fest saß und ob sich in "
                      f"der Nähe etwas bewegt hat.")

    zeilen.append("         Einschränkung: gemessen wird alles, was den Sensor "
                  "zappeln lässt — auch ein nicht ganz fest sitzender Wagen "
                  "oder Metall in Bewegung. Ein guter Wert beweist einen ruhigen "
                  "Sensor; ein schlechter beweist noch nicht, dass der Sensor "
                  "schuld ist.")
    return zeilen


def _massstab_zeilen(massstab):
    if "fehler" in massstab:
        return [f"  [massstab] {massstab['fehler']}"]
    zeilen = ["---- Maßstabsfehler über die Entfernung (Test 2b) ----",
              f"  Referenzstrecke: {massstab['referenz_mm']:.2f} mm",
              "  Abstand   gemessen    Fehler    Fehler",
              "     (cm)       (mm)      (mm)       (%)"]
    for z in massstab["zeilen"]:
        zeilen.append(f"  {z['abstand']:7.1f} {z['gemessen']:10.2f} "
                      f"{z['fehler_mm']:+9.3f} {z['fehler_prozent']:+9.2f}")
    schlimmster = max(massstab["zeilen"], key=lambda z: abs(z["fehler_prozent"]))
    if abs(schlimmster["fehler_prozent"]) < 1.0:
        zeilen.append(f"  Alle Abweichungen unter 1 % (größte: "
                      f"{schlimmster['fehler_prozent']:+.2f} % bei "
                      f"{schlimmster['abstand']:.0f} cm).")
    else:
        zeilen.append(f"  Größte Abweichung {schlimmster['fehler_prozent']:+.2f} % "
                      f"bei {schlimmster['abstand']:.0f} cm. Wächst der Fehler mit "
                      f"dem Abstand, ist es Feldverzerrung — nicht "
                      f"wegkalibrierbar, nur vermeiden.")
    return zeilen


# ===========================================================================
# Grafik
# ===========================================================================
_ACHSEN = (60, 60, 60)
_GITTER = (216, 216, 216)
_FARBE_3D = (200, 30, 30)
_FARBEN_ACHSE = [(70, 130, 200), (220, 140, 40), (90, 170, 90)]
_REIHE_FARBE = (150, 150, 150)


def zeichne_plot(ergebnis, pfad_png, breite=1000, hoehe=620):
    """
    Rauschen gegen Entfernung als PNG.

    Zeichnet die drei Achsen einzeln und die 3D-Streuung fett, dazu eine
    gestrichelte Linie bei einer Düsenreihe — die Marke, gegen die das
    Ergebnis gelesen wird.
    """
    from PIL import Image, ImageDraw

    if "fehler" in ergebnis:
        return False

    punkte = ergebnis["punkte"]
    rand_l, rand_r, rand_o, rand_u = 85, 175, 50, 66
    pl_b, pl_h = breite - rand_l - rand_r, hoehe - rand_o - rand_u

    x_min = min(p["abstand"] for p in punkte)
    x_max = max(p["abstand"] for p in punkte)
    if x_max - x_min < 1e-9:
        x_min, x_max = x_min - 1.0, x_max + 1.0
    y_max = max(max(p["rms3d"], max(p["sigma"])) for p in punkte)
    y_max = max(y_max, DUESENTEILUNG_MM * 1.4) * 1.12

    def px(a):
        return rand_l + (a - x_min) / (x_max - x_min) * pl_b

    def py(v):
        return rand_o + (1.0 - v / y_max) * pl_h

    bild = Image.new("RGB", (breite, hoehe), (255, 255, 255))
    z = ImageDraw.Draw(bild)
    schrift = _schrift(13)
    klein = _schrift(11)

    for anteil in [i / 6.0 for i in range(7)]:
        x = rand_l + anteil * pl_b
        z.line([(x, rand_o), (x, rand_o + pl_h)], fill=_GITTER)
        z.text((x - 12, rand_o + pl_h + 8),
               f"{x_min + anteil * (x_max - x_min):.0f}", fill=_ACHSEN, font=klein)
        y = rand_o + anteil * pl_h
        z.line([(rand_l, y), (rand_l + pl_b, y)], fill=_GITTER)
        z.text((8, y - 6), f"{y_max * (1.0 - anteil):.4f}", fill=_ACHSEN,
               font=klein)

    # Düsenreihen-Marke
    y_reihe = py(DUESENTEILUNG_MM)
    if rand_o <= y_reihe <= rand_o + pl_h:
        _gestrichelt(z, rand_l, y_reihe, rand_l + pl_b, _REIHE_FARBE)

    # Achsenkurven
    for index, name in enumerate(("x", "y", "z")):
        farbe = _FARBEN_ACHSE[index]
        pts = [(px(p["abstand"]), py(p["sigma"][index])) for p in punkte]
        if len(pts) >= 2:
            z.line(pts, fill=farbe, width=1)
        for pt in pts:
            z.ellipse([pt[0] - 2, pt[1] - 2, pt[0] + 2, pt[1] + 2], fill=farbe)

    # 3D-Streuung fett
    pts = [(px(p["abstand"]), py(p["rms3d"])) for p in punkte]
    if len(pts) >= 2:
        z.line(pts, fill=_FARBE_3D, width=3)
    for pt in pts:
        z.ellipse([pt[0] - 4, pt[1] - 4, pt[0] + 4, pt[1] + 4],
                  fill=_FARBE_3D, outline=(255, 255, 255))

    # Grenzabstand markieren
    if ergebnis["grenzart"] == "interpoliert" and ergebnis["grenzabstand"]:
        gx = px(ergebnis["grenzabstand"])
        if rand_l <= gx <= rand_l + pl_b:
            _gestrichelt_v(z, gx, rand_o, rand_o + pl_h, (120, 120, 190))
            z.text((gx + 4, rand_o + 4),
                   f"{ergebnis['grenzabstand']:.0f} cm", fill=(80, 80, 160),
                   font=klein)

    z.rectangle([rand_l, rand_o, rand_l + pl_b, rand_o + pl_h], outline=_ACHSEN)
    z.text((rand_l, 16), "Sensorrauschen über die Entfernung zum Sender",
           fill=(20, 20, 20), font=schrift)
    z.text((rand_l + pl_b / 2 - 55, hoehe - 24), "Abstand zum Sender (cm)",
           fill=_ACHSEN, font=klein)
    z.text((8, rand_o - 22), "Streuung (mm)", fill=_ACHSEN, font=klein)

    lx, ly = rand_l + pl_b + 14, rand_o + 4
    for index, name in enumerate(("sigma x", "sigma y", "sigma z")):
        z.line([(lx, ly + 6), (lx + 20, ly + 6)], fill=_FARBEN_ACHSE[index],
               width=2)
        z.text((lx + 26, ly), name, fill=(40, 40, 40), font=klein)
        ly += 18
    ly += 4
    z.line([(lx, ly + 6), (lx + 20, ly + 6)], fill=_FARBE_3D, width=3)
    z.text((lx + 26, ly), "3D-RMS", fill=(40, 40, 40), font=klein)
    ly += 22
    _gestrichelt(z, lx, ly + 6, lx + 20, _REIHE_FARBE)
    z.text((lx + 26, ly), "1 Düsenreihe", fill=(40, 40, 40), font=klein)

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
def _paare(text):
    """'10=99.4,20=99.1' -> [(10.0, 99.4), (20.0, 99.1)]"""
    paare = []
    for teil in text.split(","):
        teil = teil.strip()
        if not teil:
            continue
        if "=" not in teil:
            raise ValueError(f"'{teil}' ist kein Paar der Form ABSTAND=WERT")
        links, rechts = teil.split("=", 1)
        paare.append((float(links.replace(",", ".")),
                      float(rechts.replace(",", "."))))
    return paare


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="rauschen_entfernung",
        description="Wertet --pos-json-Aufzeichnungen bei stillstehendem Wagen "
                    "aus: Rauschen über die Entfernung zum Sender.")
    ap.add_argument("dateien", nargs="+",
                    help="Aufzeichnungen (--pos --pos-json). Der Abstand wird "
                         "aus dem Dateinamen gelesen (erste Zahl darin), sonst "
                         "--abstaende benutzen.")
    ap.add_argument("--abstaende", default=None,
                    help="Abstände in cm, kommagetrennt, in Reihenfolge der "
                         "Dateien. Hat Vorrang vor dem Dateinamen.")
    ap.add_argument("--png", default="rauschen.png",
                    help="Dateiname der Grafik (Default rauschen.png)")
    ap.add_argument("--fenster", type=int, default=15,
                    help="Fensterlänge in Samples für die Kurzzeit-Streuung "
                         "(Default 15 = ~1 s bei 15 Hz)")
    ap.add_argument("--hz", type=float, default=STANDARD_HZ,
                    help=f"Abtastrate von --pos, nur für die Sekundenangaben "
                         f"(Default {STANDARD_HZ:g})")
    ap.add_argument("--massstab", default=None,
                    help="Test 2b: ABSTAND=GEMESSEN-Paare, z.B. "
                         "'10=99.4,20=99.1'")
    ap.add_argument("--referenz", type=float, default=100.0,
                    help="Wahre Referenzstrecke in mm für --massstab "
                         "(Default 100)")
    ap.add_argument("--kein-plot", action="store_true")
    args = ap.parse_args(sys.argv[1:] if argv is None else argv)

    pfade = []
    for muster in args.dateien:
        treffer = sorted(glob.glob(muster))
        pfade.extend(treffer if treffer else [muster])

    explizit = None
    if args.abstaende:
        try:
            explizit = [float(t.strip().replace(",", "."))
                        for t in args.abstaende.split(",") if t.strip()]
        except ValueError as fehler:
            print(f"[rauschen] --abstaende: {fehler}")
            return 2
        if len(explizit) != len(pfade):
            print(f"[rauschen] --abstaende hat {len(explizit)} Werte, es sind "
                  f"aber {len(pfade)} Dateien.")
            return 2

    messungen = []
    for index, pfad in enumerate(pfade):
        try:
            xs, ys, zs = lies_pos_json(pfad)
        except OSError as fehler:
            print(f"[rauschen] {pfad}: {fehler}")
            continue
        if len(xs) < 2:
            print(f"[rauschen] {pfad}: keine verwertbaren Punkte gefunden.")
            continue
        abstand = explizit[index] if explizit else abstand_aus_name(pfad)
        if abstand is None:
            print(f"[rauschen] {pfad}: kein Abstand im Dateinamen erkennbar — "
                  f"mit --abstaende angeben.")
            continue
        messungen.append((abstand, pfad, xs, ys, zs))

    if not messungen:
        print("[rauschen] Keine auswertbare Datei.")
        return 2

    ergebnis = auswerten(messungen, fenster=args.fenster)
    massstab = None
    if args.massstab:
        try:
            massstab = massstab_auswerten(_paare(args.massstab), args.referenz)
        except ValueError as fehler:
            print(f"[rauschen] --massstab: {fehler}")
            return 2

    print(bericht(ergebnis, hz=args.hz, massstab=massstab))

    if not args.kein_plot and "fehler" not in ergebnis:
        try:
            if zeichne_plot(ergebnis, args.png):
                print(f"\n  Grafik geschrieben: {args.png}")
        except ImportError:
            print("\n[rauschen] Pillow (PIL) fehlt — der Textbericht oben ist "
                  "vollständig.")
        except OSError as fehler:
            print(f"\n[rauschen] Grafik konnte nicht geschrieben werden: {fehler}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
