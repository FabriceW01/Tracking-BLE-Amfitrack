"""
Geradheit einer Führungsschiene über mehrere Messfahrten
=========================================================

Eigenständiges Werkzeug. Wertet Messfahrten aus, bei denen der Sensor an einem
geraden Balken entlang der x-Achse geführt wurde, und beantwortet:

  * Wie stark weicht die gemessene Bahn quer zur Fahrtrichtung ab?
  * Wo entlang der Strecke ist die Abweichung groß?
  * Wie viel davon **wiederholt sich** über mehrere Fahrten (systematisch:
    Feldverzerrung des Trackers oder ein krummer Balken) und wie viel
    **streut** (zufällig: Sensorrauschen)?

Die letzte Frage ist der eigentliche Grund für mehrere Messreihen. Eine
einzelne Fahrt kann nicht zwischen „der Tracker ist hier dauerhaft schief" und
„das war Rauschen" unterscheiden; mehrere Fahrten über denselben Balken schon.

Datenquelle
-----------
**Empfohlen: ``--pos --pos-json``** — rohe Sensorkoordinaten:

    python main.py --pos --pos-json > fahrt1.jsonl

Die **Profil-CSV** (``--profile-csv``) geht auch, ist aber für diese Messung
die schlechtere Quelle und wird nur mit Warnung akzeptiert:

  * ``u_mm``/``v_mm`` sind **Seitenebenen**-Koordinaten. Kalibrierung,
    Sensor-zu-Düsenleisten-Versatz und die Drehung um den Gierwinkel stecken
    bereits darin — eine dort gemessene Abweichung enthält also
    Kalibrierfehler und Wagendrehung, nicht nur den Tracker.
  * Es wird nur geschrieben, wenn sich das Düsenmuster **ändert**, also
    unregelmäßig und mit Lücken.
  * Es braucht einen echten Druckdurchgang.

Verfahren
---------
Der Balken liegt nie exakt auf der x-Achse. Diese Schiefstellung ist **keine**
Tracker-Abweichung, sondern Aufbau — sie wird herausgerechnet, indem eine
Ausgleichsgerade durch die Punkte gelegt und nur der **senkrechte Abstand**
davon ausgewertet wird.

Die Gerade wird per **Total Least Squares** (Hauptachse) angepasst, nicht per
gewöhnlicher y-auf-x-Regression: der Messfehler steckt in beiden Achsen, und
der senkrechte Abstand ist die Größe, die interessiert.

Bei mehreren Fahrten wird **eine gemeinsame** Gerade durch alle Punkte gelegt,
nicht je Fahrt eine eigene. Sonst bekäme jede Fahrt ihr eigenes Bezugssystem
und die Fahrten wären untereinander nicht mehr vergleichbar — ein Versatz
zwischen zwei Fahrten würde wegdefiniert statt sichtbar zu werden.

Der Balken darf zwischen den Fahrten **nicht bewegt** werden: die absoluten
Sensorkoordinaten sind der gemeinsame Bezug, über den die Fahrten überhaupt
erst übereinandergelegt werden können.

Benutzung
---------
    python funktionen/geradheit_messreihe.py fahrt1.jsonl fahrt2.jsonl \\
        --png geradheit.png

    python funktionen/geradheit_messreihe.py *.jsonl --png out.png --bins 40

Ausgabe: ein Textbericht auf der Konsole und eine PNG-Grafik.
"""

import argparse
import csv
import glob
import json
import math
import os
import sys

# Düsenteilung in mm -- die Einheit, in der eine Abweichung für den Druck
# überhaupt zählt. Bewusst hier hart hinterlegt statt aus printhead.geometry
# importiert, damit diese Datei allein kopierbar bleibt (siehe Modul-Docstring).
# Ein Test im Repo (tests/test_geradheit_messreihe.py) hält den Wert gegen
# printhead.geometry.NOZZLE_PITCH_MM, damit er nicht auseinanderläuft.
DUESENTEILUNG_MM = 13.2 / 152


# ===========================================================================
# Einlesen
# ===========================================================================
def lies_pos_json(pfad):
    """
    Liest eine ``--pos --pos-json``-Datei (NDJSON, ein Objekt je Zeile).

    Nimmt die rohen Sensorwerte ``x``/``y``. Zeilen ohne beide Felder (z.B.
    ``{"event":"connected"}``) werden übersprungen, kaputte Zeilen ebenfalls --
    eine abgebrochene Aufzeichnung soll auswertbar bleiben statt an der
    letzten, halb geschriebenen Zeile zu scheitern.
    """
    xs, ys = [], []
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
            if "x" in obj and "y" in obj:
                try:
                    xs.append(float(obj["x"]))
                    ys.append(float(obj["y"]))
                except (TypeError, ValueError):
                    continue
    return xs, ys


def lies_profile_csv(pfad):
    """
    Liest eine Seiten-Modus-``--profile-csv`` und nimmt ``u_mm``/``v_mm``.

    Siehe Modul-Docstring: das sind Seitenebenen-Koordinaten, keine rohen
    Sensorwerte. Der Aufrufer warnt davor; hier wird nur gelesen.
    """
    xs, ys = [], []
    with open(pfad, newline="", encoding="utf-8") as datei:
        leser = csv.DictReader(datei)
        felder = leser.fieldnames or []
        if "u_mm" not in felder or "v_mm" not in felder:
            raise ValueError(
                f"{pfad!r} hat keine Spalten u_mm/v_mm (gefunden: "
                f"{','.join(felder) or '<keine>'}). Eine Line-Modus-CSV "
                f"enthält keine zweite Achse und ist hier nicht auswertbar.")
        for zeile in leser:
            try:
                xs.append(float(zeile["u_mm"]))
                ys.append(float(zeile["v_mm"]))
            except (TypeError, ValueError):
                continue
    return xs, ys


def lies_messreihe(pfad):
    """
    Liest eine Datei und rät das Format an der Endung bzw. am Inhalt.

    Rückgabe: ``(xs, ys, quelle)`` mit ``quelle`` = ``"pos-json"`` oder
    ``"profile-csv"``.
    """
    endung = os.path.splitext(pfad)[1].lower()
    if endung == ".csv":
        xs, ys = lies_profile_csv(pfad)
        return xs, ys, "profile-csv"
    xs, ys = lies_pos_json(pfad)
    if not xs:
        # Endung sagt nichts -- vielleicht doch eine CSV ohne .csv-Endung.
        try:
            xs, ys = lies_profile_csv(pfad)
            return xs, ys, "profile-csv"
        except (ValueError, UnicodeDecodeError):
            pass
    return xs, ys, "pos-json"


# ===========================================================================
# Rechnung
# ===========================================================================
def passe_gerade_an(xs, ys):
    """
    Total-Least-Squares-Gerade (Hauptachse) durch alle Punkte.

    Rückgabe: dict mit ``mittelpunkt`` (x, y), ``richtung`` (Einheitsvektor
    entlang der Geraden) und ``normale`` (senkrecht dazu), oder ``None``, wenn
    sich nichts anpassen lässt (weniger als 2 Punkte, oder alle Punkte an
    derselben Stelle).

    Ohne numpy gerechnet, damit die Datei allein lauffähig bleibt: bei zwei
    Dimensionen ist die Hauptachse geschlossen lösbar über den größeren
    Eigenwert der 2x2-Kovarianzmatrix.
    """
    punkte = [(x, y) for x, y in zip(xs, ys)
              if _endlich(x) and _endlich(y)]
    if len(punkte) < 2:
        return None

    n = len(punkte)
    mx = sum(p[0] for p in punkte) / n
    my = sum(p[1] for p in punkte) / n

    sxx = sum((p[0] - mx) ** 2 for p in punkte) / n
    syy = sum((p[1] - my) ** 2 for p in punkte) / n
    sxy = sum((p[0] - mx) * (p[1] - my) for p in punkte) / n

    if max(sxx, syy, abs(sxy)) < 1e-18:
        return None

    # Größerer Eigenwert der Kovarianzmatrix [[sxx,sxy],[sxy,syy]].
    spur = sxx + syy
    wurzel = math.sqrt(max(0.0, (sxx - syy) ** 2 + 4.0 * sxy * sxy))
    lam = (spur + wurzel) / 2.0

    # Zugehöriger Eigenvektor; der stabilere der beiden Ausdrücke gewinnt.
    if abs(lam - syy) > abs(lam - sxx):
        rx, ry = lam - syy, sxy
    else:
        rx, ry = sxy, lam - sxx
    laenge = math.hypot(rx, ry)
    if laenge < 1e-18:
        rx, ry, laenge = 1.0, 0.0, 1.0
    rx, ry = rx / laenge, ry / laenge

    # Vorzeichen festnageln: Gerade ist ungerichtet, damit Hin- und Rückfahrt
    # dieselbe Beschreibung bekommen.
    if rx < 0 or (rx == 0.0 and ry < 0):
        rx, ry = -rx, -ry

    return {"mittelpunkt": (mx, my), "richtung": (rx, ry),
            "normale": (-ry, rx)}


def projiziere(xs, ys, fit):
    """
    Rechnet Punkte in das Bezugssystem der Geraden um.

    Rückgabe ``(entlang, abweichung)``:
      * ``entlang``    -- Weg entlang der Geraden ab deren Mittelpunkt (mm).
      * ``abweichung`` -- **vorzeichenbehafteter** senkrechter Abstand (mm).
        Das Vorzeichen bleibt erhalten: eine Bahn, die erst +0,2 und dann
        −0,2 abweicht, ist krumm; eine, die zufällig um ±0,2 springt,
        rauscht. Als Betrag wäre dieser Unterschied weg.
    """
    (mx, my) = fit["mittelpunkt"]
    (rx, ry) = fit["richtung"]
    (nx, ny) = fit["normale"]
    entlang, abweichung = [], []
    for x, y in zip(xs, ys):
        if not (_endlich(x) and _endlich(y)):
            continue
        dx, dy = x - mx, y - my
        entlang.append(dx * rx + dy * ry)
        abweichung.append(dx * nx + dy * ny)
    return entlang, abweichung


def _endlich(wert):
    try:
        return not (math.isnan(wert) or math.isinf(wert))
    except TypeError:
        return False


def kennzahlen(werte):
    """RMS, größter Betrag und Spanne einer Werteliste."""
    if not werte:
        return {"rms": 0.0, "max_abs": 0.0, "spanne": 0.0}
    rms = math.sqrt(sum(w * w for w in werte) / len(werte))
    return {"rms": rms, "max_abs": max(abs(w) for w in werte),
            "spanne": max(werte) - min(werte)}


def binne(entlang, abweichung, kanten):
    """
    Mittelt die Abweichung in vorgegebene Bins entlang der Strecke.

    Rückgabe: Liste gleicher Länge wie ``kanten`` minus 1, mit dem Mittelwert
    je Bin oder ``None`` für leere Bins. ``None`` statt 0.0, damit eine Lücke
    in der Fahrt eine Lücke bleibt und nicht als „hier war die Abweichung 0"
    gelesen wird.
    """
    n = len(kanten) - 1
    summen = [0.0] * n
    zaehler = [0] * n
    for e, a in zip(entlang, abweichung):
        index = _bin_index(e, kanten)
        if index is not None:
            summen[index] += a
            zaehler[index] += 1
    return [(summen[i] / zaehler[i]) if zaehler[i] else None for i in range(n)]


def _bin_index(wert, kanten):
    if wert < kanten[0] or wert > kanten[-1]:
        return None
    # Rechter Rand gehört in den letzten Bin.
    n = len(kanten) - 1
    if wert == kanten[-1]:
        return n - 1
    lo, hi = 0, n
    while lo < hi - 1:
        mitte = (lo + hi) // 2
        if wert < kanten[mitte]:
            hi = mitte
        else:
            lo = mitte
    return lo


def auswerten(messreihen, anzahl_bins=30):
    """
    Gesamtauswertung über eine oder mehrere Messreihen.

    ``messreihen`` ist eine Liste von ``(name, xs, ys)``.

    Eine **gemeinsame** Ausgleichsgerade über alle Punkte (siehe
    Modul-Docstring) sorgt dafür, dass die Fahrten im selben Bezugssystem
    liegen und ein Versatz zwischen ihnen sichtbar bleibt.

    Rückgabe: dict mit ``fehler``, oder mit den Ergebnissen je Fahrt
    (``fahrten``), dem gemeinsamen Raster (``bin_mitten``), der
    Mittelwertkurve (``mittel``), der Streuung zwischen den Fahrten
    (``streuung``) und den zusammenfassenden Kennzahlen.
    """
    brauchbar = [(name, xs, ys) for name, xs, ys in messreihen if len(xs) >= 2]
    if not brauchbar:
        return {"fehler": "Keine Messreihe mit mindestens zwei Punkten."}

    alle_x = [x for _, xs, _ in brauchbar for x in xs]
    alle_y = [y for _, _, ys in brauchbar for y in ys]
    fit = passe_gerade_an(alle_x, alle_y)
    if fit is None:
        return {"fehler": "Punkte liegen alle an derselben Stelle -- "
                          "keine Gerade bestimmbar."}

    fahrten = []
    for name, xs, ys in brauchbar:
        entlang, abweichung = projiziere(xs, ys, fit)
        if len(entlang) < 2:
            continue
        werte = kennzahlen(abweichung)
        fahrten.append({
            "name": name,
            "punkte": len(entlang),
            "entlang": entlang,
            "abweichung": abweichung,
            "strecke_mm": max(entlang) - min(entlang),
            "rms_mm": werte["rms"],
            "max_abs_mm": werte["max_abs"],
            "spanne_mm": werte["spanne"],
            "versatz_mm": sum(abweichung) / len(abweichung),
        })
    if not fahrten:
        return {"fehler": "Keine auswertbare Fahrt nach dem Filtern."}

    # Gemeinsames Raster über den Bereich, den ALLE Fahrten abdecken --
    # sonst würde der Mittelwert am Rand aus unterschiedlich vielen Fahrten
    # gebildet und dort einen Sprung zeigen, der nur vom Raster kommt.
    start = max(min(f["entlang"]) for f in fahrten)
    ende = min(max(f["entlang"]) for f in fahrten)
    if ende <= start:
        # Fahrten überlappen sich nicht -- dann jede für sich über ihren
        # eigenen Bereich, ohne Mittelwertkurve.
        start = min(min(f["entlang"]) for f in fahrten)
        ende = max(max(f["entlang"]) for f in fahrten)
        ueberlappung = False
    else:
        ueberlappung = True

    anzahl_bins = max(1, int(anzahl_bins))
    if ende - start < 1e-12:
        ende = start + 1e-12
    schritt = (ende - start) / anzahl_bins
    kanten = [start + i * schritt for i in range(anzahl_bins + 1)]
    mitten = [(kanten[i] + kanten[i + 1]) / 2.0 for i in range(anzahl_bins)]

    for fahrt in fahrten:
        fahrt["binned"] = binne(fahrt["entlang"], fahrt["abweichung"], kanten)

    mittel, streuung, belegung = [], [], []
    for i in range(anzahl_bins):
        werte = [f["binned"][i] for f in fahrten if f["binned"][i] is not None]
        belegung.append(len(werte))
        if not werte:
            mittel.append(None)
            streuung.append(None)
            continue
        m = sum(werte) / len(werte)
        mittel.append(m)
        if len(werte) >= 2:
            streuung.append(math.sqrt(sum((w - m) ** 2 for w in werte)
                                      / (len(werte) - 1)))
        else:
            streuung.append(None)

    mittel_vorhanden = [m for m in mittel if m is not None]
    streu_vorhanden = [s for s in streuung if s is not None]

    # Rauschen je EINZELMESSWERT: jeder Punkt gegen die Mittelwertkurve seines
    # Abschnitts. Bewusst getrennt von `streuung` oben, denn das ist die
    # Streuung der Bin-MITTELWERTE und damit um rund sqrt(Punkte je Bin)
    # kleiner als das tatsächliche Sensorrauschen -- wer die Zahl als
    # "Rauschen" liest, hielte den Sensor für deutlich ruhiger als er ist.
    # Diese hier ist die Zahl, die mit einer Rauschmessung am stehenden
    # Wagen vergleichbar ist.
    reste = []
    for fahrt in fahrten:
        fahrt_reste = []
        for e, a in zip(fahrt["entlang"], fahrt["abweichung"]):
            index = _bin_index(e, kanten)
            if index is None or mittel[index] is None:
                continue
            fahrt_reste.append(a - mittel[index])
        fahrt["rausch_rms_mm"] = kennzahlen(fahrt_reste)["rms"]
        reste.extend(fahrt_reste)

    return {
        "fahrten": fahrten,
        "anzahl_fahrten": len(fahrten),
        "ueberlappung": ueberlappung,
        "bin_mitten": mitten,
        "bin_kanten": kanten,
        "mittel": mittel,
        "streuung": streuung,
        "belegung": belegung,
        "winkel_grad": math.degrees(math.atan2(fit["richtung"][1],
                                               fit["richtung"][0])),
        "systematisch_rms_mm": kennzahlen(mittel_vorhanden)["rms"],
        "systematisch_spanne_mm": kennzahlen(mittel_vorhanden)["spanne"],
        "streuung_mittel_mm": (sum(streu_vorhanden) / len(streu_vorhanden)
                               if streu_vorhanden else None),
        "rausch_rms_mm": kennzahlen(reste)["rms"] if reste else None,
    }


# ===========================================================================
# Bericht
# ===========================================================================
def _reihen(mm):
    return mm / DUESENTEILUNG_MM


def bericht(ergebnis):
    """Formatiert das Ergebnis von :func:`auswerten` als Text."""
    if "fehler" in ergebnis:
        return f"[geradheit] {ergebnis['fehler']}"

    zeilen = ["---- Geradheit über die Messreihe ----"]
    zeilen.append(f"  Fahrten              : {ergebnis['anzahl_fahrten']}")
    zeilen.append(f"  Winkel der Ausgleichsgeraden gegen +x: "
                  f"{ergebnis['winkel_grad']:+.3f} deg")
    zeilen.append("   (Schiefstellung des Balkens -- herausgerechnet, "
                  "keine Tracker-Abweichung)")
    zeilen.append("")
    zeilen.append("  Je Fahrt:")
    zeilen.append("    Name                          Punkte  Strecke   RMS      max     Versatz")
    for fahrt in ergebnis["fahrten"]:
        name = fahrt["name"]
        if len(name) > 28:
            name = "..." + name[-25:]
        zeilen.append(f"    {name:<28} {fahrt['punkte']:>6} "
                      f"{fahrt['strecke_mm']:>8.1f} {fahrt['rms_mm']:>7.4f} "
                      f"{fahrt['max_abs_mm']:>8.4f} {fahrt['versatz_mm']:>+9.4f}")
    zeilen.append("    (alle Werte in mm)")

    if ergebnis["anzahl_fahrten"] >= 2:
        zeilen.append("")
        if not ergebnis["ueberlappung"]:
            zeilen.append("  ACHTUNG: die Fahrten überlappen sich nicht -- der "
                          "Mittelwert ist damit nicht aussagekräftig.")
        zeilen.append("  Über die Fahrten hinweg:")
        sys_rms = ergebnis["systematisch_rms_mm"]
        zeilen.append(f"    systematisch (RMS der Mittelwertkurve): "
                      f"{sys_rms:.4f} mm ({_reihen(sys_rms):.1f} Düsenreihen)")
        zeilen.append(f"    systematische Spanne                  : "
                      f"{ergebnis['systematisch_spanne_mm']:.4f} mm")
        if ergebnis["rausch_rms_mm"] is not None:
            rausch = ergebnis["rausch_rms_mm"]
            zeilen.append(f"    zufällig (Rauschen je Messwert)        : "
                          f"{rausch:.4f} mm ({_reihen(rausch):.1f} Düsenreihen)")
        if ergebnis["streuung_mittel_mm"] is not None:
            streu = ergebnis["streuung_mittel_mm"]
            zeilen.append(f"    Streuung der Abschnitts-Mittelwerte   : "
                          f"{streu:.4f} mm")
            zeilen.append("     (kleiner als das Rauschen -- über die Punkte "
                          "je Abschnitt gemittelt; nicht als Sensorrauschen "
                          "lesen)")

    zeilen.append("")
    zeilen.extend(_urteil(ergebnis))
    return "\n".join(zeilen)


def _urteil(ergebnis):
    zeilen = []
    n = ergebnis["anzahl_fahrten"]

    if n < 2:
        rms = ergebnis["fahrten"][0]["rms_mm"]
        zeilen.append(f"  FAZIT: eine einzelne Fahrt, RMS {rms:.4f} mm "
                      f"({_reihen(rms):.1f} Düsenreihen).")
        zeilen.append("         Mit nur einer Fahrt lässt sich NICHT trennen, "
                      "ob die Abweichung sich wiederholt (Feldverzerrung oder "
                      "krummer Balken) oder ob es Rauschen war. Dafür "
                      "mindestens zwei, besser drei Fahrten über denselben "
                      "Balken aufnehmen.")
        return zeilen

    sys_rms = ergebnis["systematisch_rms_mm"]
    # Gegen das Rauschen JE MESSWERT verglichen, nicht gegen die Streuung der
    # Abschnitts-Mittelwerte: Letztere schrumpft mit der Punktzahl je
    # Abschnitt, ein Vergleich damit würde jede Messung mit genügend Punkten
    # automatisch "systematisch" nennen.
    streu = ergebnis["rausch_rms_mm"]

    if streu is None:
        zeilen.append(f"  FAZIT: systematischer Anteil {sys_rms:.4f} mm RMS; "
                      f"das Rauschen ließ sich nicht bestimmen.")
        return zeilen

    if sys_rms > 2.0 * streu:
        zeilen.append(f"  FAZIT: überwiegend SYSTEMATISCH "
                      f"({sys_rms:.4f} mm gegen {streu:.4f} mm Rauschen).")
        zeilen.append("         Die Abweichung wiederholt sich über die "
                      "Fahrten, ist also keine Zufallsschwankung. Zwei "
                      "Ursachen kommen infrage und lassen sich mit dem "
                      "vorhandenen Aufbau unterscheiden: ein krummer Balken "
                      "oder Feldverzerrung des Trackers. Balken um 180 Grad "
                      "drehen und erneut messen -- wandert die Kurve mit, "
                      "war es der Balken; bleibt sie liegen, der Tracker.")
    elif streu > 2.0 * sys_rms:
        zeilen.append(f"  FAZIT: überwiegend ZUFÄLLIG "
                      f"({streu:.4f} mm Rauschen gegen {sys_rms:.4f} mm "
                      f"systematisch).")
        zeilen.append("         Sensorrauschen dominiert. Das mittelt sich "
                      "über eine Dosis teilweise heraus und lässt sich mit "
                      "--smooth-ms dämpfen (auf Kosten von Nachlauf).")
    else:
        zeilen.append(f"  FAZIT: systematischer Anteil {sys_rms:.4f} mm und "
                      f"Rauschen {streu:.4f} mm sind ähnlich groß -- beide "
                      f"Effekte tragen bei.")

    grenze = DUESENTEILUNG_MM
    gesamt = max(sys_rms, streu)
    if gesamt < grenze:
        zeilen.append(f"         Beide liegen unter einer Düsenreihe "
                      f"({grenze:.4f} mm) und können im Druck nicht sichtbar "
                      f"werden.")
    else:
        zeilen.append(f"         Das entspricht {_reihen(gesamt):.1f} "
                      f"Düsenreihen -- im Druck sichtbar.")

    zeilen.append("         Einschränkung: gemessen wird die Summe aus "
                  "Tracker-Fehler, Geradheit des Balkens und der Führung von "
                  "Hand. Ein guter Wert beweist einen guten Tracker; ein "
                  "schlechter beweist noch nicht, dass der Tracker schuld ist.")
    return zeilen


# ===========================================================================
# Grafik (PIL, damit keine zusätzliche Abhängigkeit nötig ist)
# ===========================================================================
_FARBEN = [(70, 130, 200), (220, 120, 40), (90, 170, 90), (180, 90, 180),
           (200, 170, 40), (100, 190, 190)]
_MITTEL_FARBE = (200, 30, 30)
_ACHSEN = (60, 60, 60)
_GITTER = (216, 216, 216)
_REIHE_FARBE = (150, 150, 150)


def zeichne_plot(ergebnis, pfad_png, breite=1200, hoehe=700):
    """
    Zeichnet Abweichung gegen Strecke und schreibt eine PNG-Datei.

    Enthält: eine dünne Kurve je Fahrt, die Mittelwertkurve fett, ein Band für
    die Streuung zwischen den Fahrten, die Nulllinie und gestrichelte Linien
    bei ±1 Düsenreihe -- Letztere, damit auf einen Blick erkennbar ist, ob die
    Abweichung für den Druck überhaupt eine Rolle spielt.

    PIL statt matplotlib, weil PIL im Projekt ohnehin gebraucht wird und diese
    Datei allein lauffähig bleiben soll.
    """
    from PIL import Image, ImageDraw

    if "fehler" in ergebnis:
        return False

    rand_l, rand_r, rand_o, rand_u = 90, 210, 50, 70
    pl_b = breite - rand_l - rand_r
    pl_h = hoehe - rand_o - rand_u

    alle_e = [e for f in ergebnis["fahrten"] for e in f["entlang"]]
    alle_a = [a for f in ergebnis["fahrten"] for a in f["abweichung"]]
    x_min, x_max = min(alle_e), max(alle_e)
    y_min, y_max = min(alle_a), max(alle_a)
    # Düsenreihen-Marken sollen immer sichtbar sein, auch wenn die Kurve
    # flacher verläuft -- sonst fehlt der Maßstab, gegen den man liest.
    y_min = min(y_min, -DUESENTEILUNG_MM * 1.3)
    y_max = max(y_max, DUESENTEILUNG_MM * 1.3)
    if x_max - x_min < 1e-9:
        x_max = x_min + 1.0
    if y_max - y_min < 1e-9:
        y_max = y_min + 1.0
    spanne_y = y_max - y_min
    y_min -= spanne_y * 0.08
    y_max += spanne_y * 0.08

    def px(e):
        return rand_l + (e - x_min) / (x_max - x_min) * pl_b

    def py(a):
        return rand_o + (y_max - a) / (y_max - y_min) * pl_h

    bild = Image.new("RGB", (breite, hoehe), (255, 255, 255))
    zeichnung = ImageDraw.Draw(bild)
    schrift = _schrift(13)
    schrift_klein = _schrift(11)

    # --- Gitter und Achsenbeschriftung ---
    for anteil in [i / 8.0 for i in range(9)]:
        x = rand_l + anteil * pl_b
        zeichnung.line([(x, rand_o), (x, rand_o + pl_h)], fill=_GITTER)
        wert = x_min + anteil * (x_max - x_min)
        zeichnung.text((x - 18, rand_o + pl_h + 8), f"{wert:.0f}",
                       fill=_ACHSEN, font=schrift_klein)
    for anteil in [i / 6.0 for i in range(7)]:
        y = rand_o + anteil * pl_h
        zeichnung.line([(rand_l, y), (rand_l + pl_b, y)], fill=_GITTER)
        wert = y_max - anteil * (y_max - y_min)
        zeichnung.text((8, y - 6), f"{wert:+.3f}", fill=_ACHSEN,
                       font=schrift_klein)

    # --- Streuungsband (nur wo mindestens zwei Fahrten beitragen) ---
    if ergebnis["anzahl_fahrten"] >= 2:
        band = []
        unten = []
        for mitte, m, s in zip(ergebnis["bin_mitten"], ergebnis["mittel"],
                               ergebnis["streuung"]):
            if m is None or s is None:
                continue
            band.append((px(mitte), py(m + s)))
            unten.append((px(mitte), py(m - s)))
        if len(band) >= 2:
            zeichnung.polygon(band + list(reversed(unten)),
                              fill=(255, 225, 225))

    # --- Nulllinie und Düsenreihen-Marken ---
    zeichnung.line([(rand_l, py(0.0)), (rand_l + pl_b, py(0.0))],
                   fill=(120, 120, 120), width=2)
    for vorzeichen in (1, -1):
        y = py(vorzeichen * DUESENTEILUNG_MM)
        if rand_o <= y <= rand_o + pl_h:
            _gestrichelt(zeichnung, rand_l, y, rand_l + pl_b, _REIHE_FARBE)

    # --- eine Kurve je Fahrt ---
    for index, fahrt in enumerate(ergebnis["fahrten"]):
        farbe = _FARBEN[index % len(_FARBEN)]
        punkte = [(px(e), py(a)) for e, a in zip(fahrt["entlang"],
                                                 fahrt["abweichung"])]
        if len(punkte) >= 2:
            zeichnung.line(punkte, fill=farbe, width=1)

    # --- Mittelwertkurve ---
    if ergebnis["anzahl_fahrten"] >= 2:
        mittelpunkte = [(px(mitte), py(m)) for mitte, m
                        in zip(ergebnis["bin_mitten"], ergebnis["mittel"])
                        if m is not None]
        if len(mittelpunkte) >= 2:
            zeichnung.line(mittelpunkte, fill=_MITTEL_FARBE, width=3)

    # --- Achsenrahmen ---
    zeichnung.rectangle([rand_l, rand_o, rand_l + pl_b, rand_o + pl_h],
                        outline=_ACHSEN)

    # --- Titel und Achsentitel ---
    zeichnung.text((rand_l, 16),
                   f"Abweichung quer zur Fahrt  ({ergebnis['anzahl_fahrten']} "
                   f"Fahrt(en), Balken {ergebnis['winkel_grad']:+.2f} deg "
                   f"gegen +x)", fill=(20, 20, 20), font=schrift)
    zeichnung.text((rand_l + pl_b / 2 - 60, hoehe - 26),
                   "Strecke entlang des Balkens (mm)", fill=_ACHSEN,
                   font=schrift_klein)
    zeichnung.text((8, rand_o - 22), "Abweichung (mm)", fill=_ACHSEN,
                   font=schrift_klein)

    # --- Legende ---
    lx = rand_l + pl_b + 16
    ly = rand_o + 4
    for index, fahrt in enumerate(ergebnis["fahrten"]):
        farbe = _FARBEN[index % len(_FARBEN)]
        zeichnung.line([(lx, ly + 6), (lx + 22, ly + 6)], fill=farbe, width=2)
        name = os.path.basename(fahrt["name"])
        if len(name) > 20:
            name = name[:17] + "..."
        zeichnung.text((lx + 28, ly), name, fill=(40, 40, 40),
                       font=schrift_klein)
        ly += 18
    if ergebnis["anzahl_fahrten"] >= 2:
        ly += 6
        zeichnung.line([(lx, ly + 6), (lx + 22, ly + 6)],
                       fill=_MITTEL_FARBE, width=3)
        zeichnung.text((lx + 28, ly), "Mittelwert", fill=(40, 40, 40),
                       font=schrift_klein)
        ly += 18
        zeichnung.rectangle([lx, ly + 2, lx + 22, ly + 10],
                            fill=(255, 225, 225), outline=(230, 190, 190))
        zeichnung.text((lx + 28, ly), "Streuung", fill=(40, 40, 40),
                       font=schrift_klein)
        ly += 18
    ly += 6
    _gestrichelt(zeichnung, lx, ly + 6, lx + 22, _REIHE_FARBE)
    zeichnung.text((lx + 28, ly), "1 Düsenreihe", fill=(40, 40, 40),
                   font=schrift_klein)

    bild.save(pfad_png)
    return True


def _gestrichelt(zeichnung, x1, y, x2, farbe, strich=6, luecke=5):
    x = x1
    while x < x2:
        zeichnung.line([(x, y), (min(x + strich, x2), y)], fill=farbe)
        x += strich + luecke


def _schrift(groesse):
    """Truetype wenn auffindbar, sonst PILs Standardschrift."""
    from PIL import ImageFont
    for pfad in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                 "C:/Windows/Fonts/segoeui.ttf",
                 "C:/Windows/Fonts/arial.ttf",
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
        prog="geradheit_messreihe",
        description="Wertet Messfahrten entlang eines geraden Balkens aus: "
                    "Abweichung quer zur Fahrtrichtung über die Strecke, "
                    "gemittelt über mehrere Fahrten.")
    ap.add_argument("dateien", nargs="+",
                    help="Eine oder mehrere Messreihen. Empfohlen: "
                         "--pos --pos-json (rohe Sensorwerte). Eine "
                         "--profile-csv wird auch gelesen, misst aber die "
                         "Seitenebene statt des rohen Sensors.")
    ap.add_argument("--png", default="geradheit.png",
                    help="Dateiname der Grafik (Default geradheit.png)")
    ap.add_argument("--bins", type=int, default=30,
                    help="Anzahl der Abschnitte für Mittelwert und Streuung "
                         "(Default 30)")
    ap.add_argument("--breite", type=int, default=1200)
    ap.add_argument("--hoehe", type=int, default=700)
    ap.add_argument("--kein-plot", action="store_true",
                    help="Nur den Textbericht ausgeben, keine PNG schreiben")
    args = ap.parse_args(sys.argv[1:] if argv is None else argv)

    # Platzhalter selbst auflösen: die Windows-Eingabeaufforderung expandiert
    # *.jsonl nicht, anders als eine Unix-Shell.
    pfade = []
    for muster in args.dateien:
        treffer = sorted(glob.glob(muster))
        pfade.extend(treffer if treffer else [muster])

    messreihen = []
    for pfad in pfade:
        try:
            xs, ys, quelle = lies_messreihe(pfad)
        except (OSError, ValueError) as fehler:
            print(f"[geradheit] {pfad}: {fehler}")
            continue
        if len(xs) < 2:
            print(f"[geradheit] {pfad}: keine verwertbaren Punkte gefunden.")
            continue
        if quelle == "profile-csv":
            print(f"[geradheit] {pfad}: Profil-CSV erkannt -- u_mm/v_mm sind "
                  f"SEITENEBENEN-Koordinaten (Kalibrierung, Düsenversatz und "
                  f"Gierwinkel sind eingerechnet) und werden nur bei "
                  f"Musterwechseln geschrieben. Für die reine Sensor-Präzision "
                  f"besser mit --pos --pos-json aufzeichnen.")
        messreihen.append((pfad, xs, ys))

    if not messreihen:
        print("[geradheit] Keine auswertbare Datei.")
        return 2

    ergebnis = auswerten(messreihen, anzahl_bins=args.bins)
    print(bericht(ergebnis))

    if not args.kein_plot and "fehler" not in ergebnis:
        try:
            if zeichne_plot(ergebnis, args.png, args.breite, args.hoehe):
                print(f"\n  Grafik geschrieben: {args.png}")
        except ImportError:
            print("\n[geradheit] Pillow (PIL) fehlt -- ohne es kann keine "
                  "Grafik erzeugt werden. Der Textbericht oben ist "
                  "vollständig.")
        except OSError as fehler:
            print(f"\n[geradheit] Grafik konnte nicht geschrieben werden: "
                  f"{fehler}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
