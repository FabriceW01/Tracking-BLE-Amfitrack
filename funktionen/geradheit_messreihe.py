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

Die **Profil-CSV** (``--profile-csv``) geht auch. Enthält sie die rohen
Sensorkoordinaten ``x``/``y`` (seit deren x/y/z-Erweiterung, Seiten- UND
Line-Modus), werden GENAU DIESE benutzt -- dieselbe Quelle wie oben, nur
schon während des Druckdurchgangs mitgeschrieben, kein separater Lauf
nötig. Fehlen sie (ältere Aufzeichnung), wird für eine Seiten-Modus-Datei
ersatzweise auf ``u_mm``/``v_mm`` zurückgegriffen -- dann mit Warnung,
denn das ist die schlechtere Quelle:

  * ``u_mm``/``v_mm`` sind **Seitenebenen**-Koordinaten. Kalibrierung,
    Sensor-zu-Düsenleisten-Versatz und die Drehung um den Gierwinkel stecken
    bereits darin — eine dort gemessene Abweichung enthält also
    Kalibrierfehler und Wagendrehung, nicht nur den Tracker.
  * Es wird nur geschrieben, wenn sich das Düsenmuster **ändert**, also
    unregelmäßig und mit Lücken.
  * Es braucht einen echten Druckdurchgang.

Unabhängig von der Quelle gilt weiterhin: geschrieben wird in einer
Profil-CSV nur, wenn tatsächlich Spalten rausgehen -- keine gleichmäßige
Zeitreihe, und deshalb für Rauschmessung kein Ersatz für ``--pos --pos-json``.

Y-Bereich
---------
Nur Punkte mit einem y-Wert in ``[--y-min, --y-max]`` (Default -90..90 mm)
gehen in die Auswertung ein -- Punkte außerhalb werden VOR der
Geradenanpassung verworfen, mit Zähler im Bericht. Gedacht, um Messwerte
außerhalb des vertrauenswürdigen Trackingbereichs auszuschließen, statt sie
unbemerkt die Ausgleichsgerade verzerren zu lassen. Fällt eine ganze Fahrt
komplett heraus, wird sie im Bericht als übersprungen genannt, nicht
stillschweigend weggelassen.

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

Die **Position entlang der Geraden** (``entlang == 0`` in Bericht/Plot) wird
NICHT auf den gewichteten Mittelwert der Punkte gelegt (der wandert bei
ungleichmäßiger Abtastung), sondern -- sofern die Fahrt überwiegend entlang
der y-Achse verläuft -- auf die Stelle, an der die absolute y-Koordinate 0
ist (``_verankere_bei_y_null``). Das allein macht die Achse aber noch nicht
symmetrisch, es legt nur fest, was 0 bedeutet -- die Plot-x-Achse selbst
steht deshalb zusätzlich, wenn diese Verankerung gegriffen hat, IMMER genau
auf ``--y-min``/``--y-max`` (Default -90/90) statt auf dem tatsächlich
aufgezeichneten Ausschnitt (``zeichne_plot``): eine kürzere oder einseitige
Fahrt bekäme sonst trotz korrekter Verankerung eine schiefe Achse.

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

# Default-Grenzen des y-Bereichs, der in die Auswertung eingeht (mm) --
# über --y-min/--y-max änderbar. Siehe Modul-Docstring "Y-Bereich".
Y_BEREICH_MIN_MM = -90.0
Y_BEREICH_MAX_MM = 90.0


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
    Liest eine ``--profile-csv`` (Seiten- ODER Line-Modus).

    Bevorzugt die ROHEN Sensorkoordinaten ``x``/``y``, falls die Datei sie
    enthält (seit deren x/y/z-Erweiterung, in beiden Modi -- siehe
    README) -- dieselbe Quelle wie ``--pos --pos-json``, nur schon während
    des Druckdurchgangs mitgeschrieben. Zeilen ohne Position (leere
    x/y-Felder; siehe README: "leer statt 0,0,0", wenn kein Tracking-Fix
    vorlag) werden übersprungen, nicht als (0, 0) gezählt.

    Fehlen x/y (ältere Aufzeichnung von vor dieser Erweiterung) ODER sind
    sie zwar als Spalte vorhanden, aber in JEDER Zeile leer (z.B. eine
    Datei ohne Tracking während der Aufzeichnung), wird für eine
    Seiten-Modus-Datei ersatzweise ``u_mm``/``v_mm`` gelesen --
    SEITENEBENEN-Koordinaten, siehe Modul-Docstring für die Einschränkung.
    Eine Line-Modus-Datei ohne brauchbare x/y hat keine zweite Achse
    (``advance_mm`` ist 1-D) und ist dann nicht auswertbar.

    Rückgabe: ``(xs, ys, quelle)`` mit ``quelle`` = ``"profile-csv-xy"``
    (rohe Koordinaten) oder ``"profile-csv-uv"`` (Seitenebene, Fallback).
    """
    with open(pfad, newline="", encoding="utf-8") as datei:
        leser = csv.DictReader(datei)
        felder = leser.fieldnames or []
        zeilen = list(leser)

    hat_xy = "x" in felder and "y" in felder
    hat_uv = "u_mm" in felder and "v_mm" in felder
    if not hat_xy and not hat_uv:
        raise ValueError(
            f"{pfad!r} hat weder x/y (rohe Sensorkoordinaten) noch "
            f"u_mm/v_mm (Seitenebenen-Koordinaten) -- keine "
            f"Positionsspalten zum Auswerten (gefunden: "
            f"{','.join(felder) or '<keine>'}).")

    if hat_xy:
        xs, ys = [], []
        for zeile in zeilen:
            try:
                # Leeres Feld (kein Tracking-Fix, siehe README: "leer statt
                # 0,0,0") ist hier ein leerer String -> float("") wirft
                # ValueError; eine fehlende Spalte ergibt None -> TypeError.
                # Beides faellt hier durch, kein eigener Leer-Check noetig.
                # Beide Werte werden erst in lokale Variablen geparst und
                # nur GEMEINSAM angehängt -- sonst könnte ein gültiges x
                # neben einem ungültigen y landen und xs/ys liefen
                # auseinander.
                x_wert = float(zeile.get("x"))
                y_wert = float(zeile.get("y"))
            except (TypeError, ValueError):
                continue
            xs.append(x_wert)
            ys.append(y_wert)
        if xs:
            return xs, ys, "profile-csv-xy"
        # x/y-Spalten vorhanden, aber in jeder Zeile leer -- nicht als
        # "leer = 0 Punkte" liegen lassen, wenn u_mm/v_mm als Fallback
        # noch etwas hergeben.
        if not hat_uv:
            return xs, ys, "profile-csv-xy"

    xs, ys = [], []
    for zeile in zeilen:
        try:
            x_wert = float(zeile["u_mm"])
            y_wert = float(zeile["v_mm"])
        except (TypeError, ValueError):
            continue
        xs.append(x_wert)
        ys.append(y_wert)
    return xs, ys, "profile-csv-uv"


def lies_messreihe(pfad):
    """
    Liest eine Datei und rät das Format an der Endung bzw. am Inhalt.

    Rückgabe: ``(xs, ys, quelle)`` mit ``quelle`` = ``"pos-json"``,
    ``"profile-csv-xy"`` oder ``"profile-csv-uv"`` (siehe
    lies_profile_csv).
    """
    endung = os.path.splitext(pfad)[1].lower()
    if endung == ".csv":
        return lies_profile_csv(pfad)
    xs, ys = lies_pos_json(pfad)
    if not xs:
        # Endung sagt nichts -- vielleicht doch eine CSV ohne .csv-Endung.
        try:
            return lies_profile_csv(pfad)
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


def filtere_y_bereich(xs, ys, y_min, y_max):
    """
    Behält nur Punkte, deren y-Wert in ``[y_min, y_max]`` liegt.

    Rückgabe: ``(xs_gefiltert, ys_gefiltert, entfernt)`` -- ``entfernt``
    ist die Anzahl der herausgefallenen Punkte, für eine ehrliche Meldung,
    wie viel vom Rohsignal tatsächlich benutzt wurde (siehe Modul-Docstring
    "Y-Bereich").
    """
    xs_neu, ys_neu = [], []
    for x, y in zip(xs, ys):
        if y_min <= y <= y_max:
            xs_neu.append(x)
            ys_neu.append(y)
    return xs_neu, ys_neu, len(xs) - len(xs_neu)


# Unterhalb dieser Steigung (Betrag von ry, der y-Komponente der
# Fahrtrichtung) gilt eine Fahrt als praktisch waagerecht -- dort gibt es
# keine stabile Kreuzung mit absolutem y=0 (bei ry=0 exakt sogar gar
# keine), und _verankere_bei_y_null lässt den ursprünglichen, datenbasierten
# Bezugspunkt unangetastet.
_MINDEST_RY_FUER_VERANKERUNG = 1e-6


def _verankere_bei_y_null(fit):
    """
    Verschiebt ``fit["mittelpunkt"]`` entlang der (unveränderten) Geraden
    zu der Stelle, an der die ABSOLUTE y-Koordinate 0 ist.

    ``entlang`` (siehe projiziere()) ist als Abstand vom Mittelpunkt der
    Geraden definiert -- vor dieser Verankerung war das der GEWICHTETE
    MITTELWERT aller einbezogenen Punkte. Bei ungleichmäßiger Abtastung
    entlang der Strecke (z.B. wechselnde Handgeschwindigkeit) liegt dieser
    Mittelwert nicht zwangsläufig in der geometrischen Mitte des
    Y-Bereichs -- genau das beobachtete Symptom (Plot-Mittelpunkt bei -3
    mm statt 0, obwohl --y-min/--y-max symmetrisch bei -90/90 lagen).
    Mit dieser Verankerung bedeutet ``entlang == 0`` stattdessen immer
    "absolutes y == 0", unabhängig von der Punktedichte.

    Geometrisch unbedenklich: der neue Mittelpunkt liegt per Konstruktion
    auf derselben Geraden (nur um ein Vielfaches von ``richtung``
    verschoben), richtung/normale bleiben unverändert. ``abweichung``
    (der Normalenanteil) ist davon nachweislich unberührt, weil richtung
    und normale zueinander orthogonal sind -- nur ``entlang`` verschiebt
    sich um eine Konstante. RMS, Spanne, Überlappung zwischen Fahrten
    usw. bleiben also exakt gleich; nur der Nullpunkt der Positionsangabe
    ändert sich.

    WICHTIG, und der Teil, der beim ersten Anlauf hier fehlte: diese
    Funktion allein macht die geplottete Achse noch NICHT symmetrisch --
    sie legt nur fest, was ``entlang == 0`` bedeutet. Ob der tatsächlich
    AUFGEZEICHNETE Ausschnitt der Fahrt (``min(entlang)..max(entlang)``)
    danach auch wirklich [-90, 90] trifft, hängt davon ab, ob die Fahrt
    diesen Bereich vollständig UND einigermaßen gleichmäßig abgedeckt hat
    -- bei einer kürzeren oder einseitigen Fahrt bliebe die Achse trotz
    korrekter Verankerung schief. Die eigentlich feste Achse setzt erst
    zeichne_plot() (siehe dort), indem es --y-min/--y-max direkt als
    Achsengrenzen benutzt, statt sie aus den Daten abzuleiten.

    Rückgabe: ``(neuer_fit, verankert)`` -- ``verankert`` ist ``False``
    (und ``neuer_fit is fit``), wenn die Fahrt zu waagerecht ist, um eine
    stabile y=0-Kreuzung zu haben.
    """
    (mx, my) = fit["mittelpunkt"]
    (rx, ry) = fit["richtung"]
    if abs(ry) < _MINDEST_RY_FUER_VERANKERUNG:
        return fit, False
    t0 = -my / ry
    neuer_mittelpunkt = (mx + t0 * rx, my + t0 * ry)
    return {**fit, "mittelpunkt": neuer_mittelpunkt}, True


def auswerten(messreihen, anzahl_bins=30, y_min=Y_BEREICH_MIN_MM,
             y_max=Y_BEREICH_MAX_MM):
    """
    Gesamtauswertung über eine oder mehrere Messreihen.

    ``messreihen`` ist eine Liste von ``(name, xs, ys)``. ``y_min``/
    ``y_max`` grenzen den benutzten y-Bereich ein (mm, Default -90..90,
    siehe Modul-Docstring "Y-Bereich") -- Punkte außerhalb werden VOR der
    Geradenanpassung verworfen. Das betrifft die rohen y-Werte aus
    ``messreihen``, nicht die spätere, geraden-bezogene ``abweichung``.

    Eine **gemeinsame** Ausgleichsgerade über alle (gefilterten) Punkte
    (siehe Modul-Docstring) sorgt dafür, dass die Fahrten im selben
    Bezugssystem liegen und ein Versatz zwischen ihnen sichtbar bleibt.

    Rückgabe: dict mit ``fehler``, oder mit den Ergebnissen je Fahrt
    (``fahrten``), den komplett aus dem Y-Bereich herausgefallenen Fahrten
    (``uebersprungen``), dem gemeinsamen Raster (``bin_mitten``), der
    Mittelwertkurve (``mittel``), der Streuung zwischen den Fahrten
    (``streuung``) und den zusammenfassenden Kennzahlen.
    """
    vorbereitet = []
    uebersprungen = []
    for name, xs, ys in messreihen:
        xs_f, ys_f, entfernt = filtere_y_bereich(xs, ys, y_min, y_max)
        if len(xs_f) < 2:
            uebersprungen.append({"name": name, "punkte_roh": len(xs),
                                  "im_bereich": len(xs_f)})
            continue
        vorbereitet.append((name, xs_f, ys_f, len(xs), entfernt))

    if not vorbereitet:
        if uebersprungen:
            return {"fehler": f"Keine Messreihe mit mindestens zwei Punkten "
                              f"im Y-Bereich [{y_min:g}, {y_max:g}] mm "
                              f"(--y-min/--y-max)."}
        return {"fehler": "Keine Messreihe mit mindestens zwei Punkten."}

    alle_x = [x for _, xs, _, _, _ in vorbereitet for x in xs]
    alle_y = [y for _, _, ys, _, _ in vorbereitet for y in ys]
    fit = passe_gerade_an(alle_x, alle_y)
    if fit is None:
        return {"fehler": "Punkte liegen alle an derselben Stelle -- "
                          "keine Gerade bestimmbar."}
    fit, y_verankert = _verankere_bei_y_null(fit)

    fahrten = []
    for name, xs, ys, punkte_roh, entfernt in vorbereitet:
        entlang, abweichung = projiziere(xs, ys, fit)
        if len(entlang) < 2:
            continue
        werte = kennzahlen(abweichung)
        fahrten.append({
            "name": name,
            "punkte": len(entlang),
            "punkte_roh": punkte_roh,
            "y_bereich_entfernt": entfernt,
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
        "uebersprungen": uebersprungen,
        "y_min": y_min,
        "y_max": y_max,
        "y_verankert": y_verankert,
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
    zeilen.append(f"  Y-Bereich (benutzt)  : {ergebnis['y_min']:.1f} .. "
                  f"{ergebnis['y_max']:.1f} mm  (--y-min/--y-max)")
    for u in ergebnis["uebersprungen"]:
        zeilen.append(f"    ACHTUNG: {u['name']} übersprungen -- nur "
                      f"{u['im_bereich']} von {u['punkte_roh']} Punkten "
                      f"im Y-Bereich.")
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
        if fahrt["y_bereich_entfernt"]:
            zeilen.append(f"      ({fahrt['y_bereich_entfernt']} von "
                          f"{fahrt['punkte_roh']} Punkten außerhalb des "
                          f"Y-Bereichs entfernt)")
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

# --slide-show: Schriftgrad für die Projektion. Auf einer Folie wird die PNG
# auf eine feste Breite skaliert, also zählt allein das VERHÄLTNIS von
# Schrifthöhe zu Bildbreite -- ein größeres Bild mit proportional größerer
# Schrift sähe an der Wand exakt gleich aus. Deshalb wächst hier die Schrift
# gegenüber der Datenfläche, nicht mit ihr.
#
# Der Wert ist der DEFAULT von --slide-show; ein Faktor darf direkt dahinter
# angegeben werden (--slide-show 3).
SLIDE_SHOW_SKALA = 2.2


def loese_skala(wert):
    """``--slide-show`` in einen Skalierungsfaktor übersetzen.

    ``None`` heißt "Flag nicht angegeben" -> 1.0, also unverändertes
    Aussehen. Ohne Zahl dahinter setzt argparse ``SLIDE_SHOW_SKALA`` ein;
    mit Zahl kommt genau die an. Ein Faktor <= 0 wird abgelehnt statt still
    zu einem leeren oder gespiegelten Bild zu führen: 0 macht jede Schrift
    und jede Linie unsichtbar, negativ dreht sämtliche Ränder nach innen.
    """
    if wert is None:
        return 1.0
    wert = float(wert)
    if wert <= 0:
        raise ValueError(
            f"--slide-show braucht einen Faktor groesser 0, nicht {wert:g}")
    return wert

# Grundmaße der Ränder bei SKALA 1. Sie halten ausschließlich Text
# (Achsenbeschriftung und Achsentitel) und wachsen deshalb mit der Schrift.
_RAND_L, _RAND_R, _RAND_O, _RAND_U = 90, 20, 50, 70


def zeichne_plot(ergebnis, pfad_png, breite=1200, hoehe=700, skala=1.0):
    """
    Zeichnet Abweichung gegen Strecke und schreibt eine PNG-Datei.

    Enthält: eine dünne Kurve je Fahrt, die Mittelwertkurve fett, ein Band für
    die Streuung zwischen den Fahrten, die Nulllinie und gestrichelte Linien
    bei ±1 Düsenreihe -- Letztere, damit auf einen Blick erkennbar ist, ob die
    Abweichung für den Druck überhaupt eine Rolle spielt.

    BEIDE Achsen sind um 0 zentriert, jede aus einem eigenen Grund (siehe
    die Kommentare unten): waagerecht steht der konfigurierte
    ``--y-min``/``--y-max``-Bereich, senkrecht ein symmetrischer
    Abweichungsbereich ``[-grenze, +grenze]``. Die Nulllinie ist damit
    tatsächlich die Bildmitte und nicht nur zufällig in ihrer Nähe.

    PIL statt matplotlib, weil PIL im Projekt ohnehin gebraucht wird und diese
    Datei allein lauffähig bleiben soll.

    ``skala`` vergrößert die Schrift und alle Maße, die an ihr hängen (Ränder,
    Beschriftungsabstände, Legendenraster) -- ``SLIDE_SHOW_SKALA`` für die
    Projektion, ``1.0`` (Default) für das bisherige Aussehen. Die
    **Datenfläche behält dabei ihre Größe**: die Leinwand wächst um genau die
    Pixel, die die größeren Ränder zusätzlich brauchen. Sonst würde die
    Zeichenfläche mit jeder Schriftvergrößerung schrumpfen, statt dass nur die
    Beschriftung wächst.
    """
    from PIL import Image, ImageDraw

    if "fehler" in ergebnis:
        return False

    def skal(mass):
        """Ein an der Schrift hängendes Maß auf die gewählte Skalierung."""
        return mass * skala

    def strich(breite_px):
        """Eine Linienstärke auf die gewählte Skalierung, mindestens 1 Pixel.

        Eine 1-Pixel-Kurve verschwindet auf einer Projektionsfläche neben
        24-Punkt-Schrift; die Linien müssen mitwachsen, sonst wird der Plot
        durch das größere Bild sogar schlechter lesbar als vorher. Gerundet
        auf ganze Pixel, weil PIL nur ganzzahlige Stärken zeichnet."""
        return max(1, int(round(breite_px * skala)))

    # rand_r braucht keinen Platz mehr für eine Legende daneben -- die
    # Legende sitzt jetzt als eigener Kasten INNERHALB der Plotfläche
    # (siehe unten).
    rand_l, rand_r = skal(_RAND_L), skal(_RAND_R)
    rand_o, rand_u = skal(_RAND_O), skal(_RAND_U)
    # Leinwand um den Zuwachs der Ränder verbreitern, damit pl_b/pl_h
    # unabhängig von `skala` exakt das bleiben, was der Aufrufer über
    # breite/hoehe angefordert hat.
    breite = int(round(breite + (_RAND_L + _RAND_R) * (skala - 1.0)))
    hoehe = int(round(hoehe + (_RAND_O + _RAND_U) * (skala - 1.0)))
    pl_b = breite - rand_l - rand_r
    pl_h = hoehe - rand_o - rand_u

    alle_e = [e for f in ergebnis["fahrten"] for e in f["entlang"]]
    alle_a = [a for f in ergebnis["fahrten"] for a in f["abweichung"]]

    # Die Achsengrenzen heißen bewusst NICHT x_min/y_min: "y" ist in dieser
    # Datei doppelt belegt -- ergebnis["y_min"]/["y_max"] sind der
    # --y-min/--y-max-FILTER auf der Sensor-y-Koordinate, und die wird hier
    # WAAGERECHT aufgetragen, während die SENKRECHTE Achse die Abweichung
    # zeigt. Genau diese Doppelbedeutung hat schon zu einem Fix an der
    # falschen Achse geführt; die Namen sagen ab jetzt, was sie zeigen.

    # --- Waagerecht: Position entlang des Balkens ---
    # Ist entlang bei y=0 verankert (siehe _verankere_bei_y_null), zeigt die
    # Achse immer genau den konfigurierten Y-Bereich -- nicht bloß den
    # tatsächlich aufgezeichneten Ausschnitt, sonst wäre sie nur zufällig
    # symmetrisch (nämlich nur bei lückenloser Abdeckung). Ohne Verankerung
    # (überwiegend waagerechte Fahrt) bleibt es beim datenbasierten Bereich,
    # weil --y-min/--y-max dort keine sinnvolle Achse wäre.
    if ergebnis.get("y_verankert") and ergebnis["y_max"] > ergebnis["y_min"]:
        pos_min, pos_max = ergebnis["y_min"], ergebnis["y_max"]
    else:
        pos_min, pos_max = min(alle_e), max(alle_e)
    if pos_max - pos_min < 1e-9:
        pos_max = pos_min + 1.0

    # --- Senkrecht: Abweichung, SYMMETRISCH um 0 ---
    # Die Nulllinie ist der Bezug, gegen den diese Kurve gelesen wird, also
    # muss sie die MITTE der Achse sein. Ein datenabhängiger Bereich
    # (min..max der Abweichung) legt sie irgendwohin -- an den echten Daten
    # hier auf 52.8 % der Höhe --, und eine Abweichung nach oben wäre dann
    # anders skaliert als eine gleich große nach unten. Nebeneffekt der
    # Symmetrie: bei 7 Gitterlinien fällt die mittlere exakt auf 0.000.
    # Untergrenze DUESENTEILUNG_MM * 1.3, damit die Düsenreihen-Marken auch
    # bei sehr flacher Kurve im Bild bleiben -- sonst fehlt der Maßstab,
    # gegen den man liest.
    grenze = max(abs(min(alle_a)), abs(max(alle_a)), DUESENTEILUNG_MM * 1.3)
    if grenze < 1e-9:
        grenze = 1.0
    grenze *= 1.08                       # etwas Luft über und unter der Kurve
    abw_min, abw_max = -grenze, grenze

    def px(e):
        return rand_l + (e - pos_min) / (pos_max - pos_min) * pl_b

    def py(a):
        return rand_o + (abw_max - a) / (abw_max - abw_min) * pl_h

    bild = Image.new("RGB", (breite, hoehe), (255, 255, 255))
    zeichnung = ImageDraw.Draw(bild)
    schrift_klein = _schrift(max(1, int(round(skal(11)))))

    # --- Gitter und Achsenbeschriftung ---
    for anteil in [i / 8.0 for i in range(9)]:
        x = rand_l + anteil * pl_b
        zeichnung.line([(x, rand_o), (x, rand_o + pl_h)], fill=_GITTER)
        wert = pos_min + anteil * (pos_max - pos_min)
        zeichnung.text((x - skal(18), rand_o + pl_h + skal(8)), f"{wert:.0f}",
                       fill=_ACHSEN, font=schrift_klein)
    for anteil in [i / 6.0 for i in range(7)]:
        y = rand_o + anteil * pl_h
        zeichnung.line([(rand_l, y), (rand_l + pl_b, y)], fill=_GITTER)
        wert = abw_max - anteil * (abw_max - abw_min)
        zeichnung.text((skal(8), y - skal(6)), f"{wert:+.3f}", fill=_ACHSEN,
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
                   fill=(120, 120, 120), width=strich(2))
    for vorzeichen in (1, -1):
        y = py(vorzeichen * DUESENTEILUNG_MM)
        if rand_o <= y <= rand_o + pl_h:
            _gestrichelt(zeichnung, rand_l, y, rand_l + pl_b, _REIHE_FARBE,
                         strich=skal(6), luecke=skal(5), breite=strich(1))

    # --- eine Kurve je Fahrt ---
    for index, fahrt in enumerate(ergebnis["fahrten"]):
        farbe = _FARBEN[index % len(_FARBEN)]
        punkte = [(px(e), py(a)) for e, a in zip(fahrt["entlang"],
                                                 fahrt["abweichung"])]
        if len(punkte) >= 2:
            zeichnung.line(punkte, fill=farbe, width=strich(1))

    # --- Mittelwertkurve ---
    if ergebnis["anzahl_fahrten"] >= 2:
        mittelpunkte = [(px(mitte), py(m)) for mitte, m
                        in zip(ergebnis["bin_mitten"], ergebnis["mittel"])
                        if m is not None]
        if len(mittelpunkte) >= 2:
            zeichnung.line(mittelpunkte, fill=_MITTEL_FARBE, width=strich(3))

    # --- Achsenrahmen ---
    zeichnung.rectangle([rand_l, rand_o, rand_l + pl_b, rand_o + pl_h],
                        outline=_ACHSEN)

    # --- Achsentitel (kein Bildtitel mehr -- siehe Moduldocstring/Anfrage:
    #     der Kopf "Abweichung quer zur Fahrt ..." ist bewusst weg) ---
    zeichnung.text((rand_l + pl_b / 2 - skal(60), hoehe - skal(26)),
                   "entlang der y-Achse (mm)", fill=_ACHSEN,
                   font=schrift_klein)
    zeichnung.text((skal(8), rand_o - skal(22)), "Abweichung (mm)", fill=_ACHSEN,
                   font=schrift_klein)

    # --- Legende -- als eigener Kasten INNERHALB der Plotfläche, nicht
    # mehr daneben (dafür ist rand_r oben auf ein schmales Randmaß
    # geschrumpft). Jede Fahrt heißt hier "Messwert" (bzw. "Messwert N"
    # bei mehreren) statt ihres Dateinamens -- der ist für die Grafik
    # selbst nicht relevant und steht ohnehin schon im Textbericht.
    eintraege = []
    for index in range(ergebnis["anzahl_fahrten"]):
        farbe = _FARBEN[index % len(_FARBEN)]
        label = ("Messwert" if ergebnis["anzahl_fahrten"] == 1
                else f"Messwert {index + 1}")
        eintraege.append(("linie", farbe, label))
    if ergebnis["anzahl_fahrten"] >= 2:
        eintraege.append(("linie_dick", _MITTEL_FARBE, "Mittelwert"))
        eintraege.append(("kasten", (255, 225, 225), "Streuung"))
    eintraege.append(("gestrichelt", _REIHE_FARBE, "1 Düsenreihe"))

    zeilenhoehe = skal(18)
    swatch_breite = skal(22)
    innen_abstand = skal(8)
    text_breite = max(
        zeichnung.textbbox((0, 0), text, font=schrift_klein)[2]
        for _, _, text in eintraege)
    legende_b = innen_abstand * 2 + swatch_breite + skal(6) + text_breite
    legende_h = innen_abstand * 2 + len(eintraege) * zeilenhoehe - skal(4)

    lx0 = rand_l + pl_b - legende_b - skal(10)
    ly0 = rand_o + skal(10)
    zeichnung.rectangle([lx0, ly0, lx0 + legende_b, ly0 + legende_h],
                        fill=(255, 255, 255), outline=_ACHSEN)

    lx = lx0 + innen_abstand
    ly = ly0 + innen_abstand
    for art, farbe, text in eintraege:
        if art == "linie":
            zeichnung.line([(lx, ly + skal(6)), (lx + swatch_breite, ly + skal(6))],
                           fill=farbe, width=strich(2))
        elif art == "linie_dick":
            zeichnung.line([(lx, ly + skal(6)), (lx + swatch_breite, ly + skal(6))],
                           fill=farbe, width=strich(3))
        elif art == "kasten":
            zeichnung.rectangle(
                [lx, ly + skal(2), lx + swatch_breite, ly + skal(10)],
                fill=farbe, outline=(230, 190, 190))
        elif art == "gestrichelt":
            _gestrichelt(zeichnung, lx, ly + skal(6), lx + swatch_breite, farbe,
                         strich=skal(6), luecke=skal(5), breite=strich(1))
        zeichnung.text((lx + swatch_breite + skal(6), ly), text,
                       fill=(40, 40, 40), font=schrift_klein)
        ly += zeilenhoehe

    bild.save(pfad_png)
    return True


def _gestrichelt(zeichnung, x1, y, x2, farbe, strich=6, luecke=5, breite=1):
    """Waagerechte gestrichelte Linie.

    Strich-/Lückenlänge sind Parameter, damit ein vergrößerter Plot sie
    mitskalieren kann: bliebe das 6/5-Raster fest, während die Linie dicker
    wird, verschmölzen die Striche optisch zu einer durchgezogenen Linie und
    die Marke wäre nicht mehr von den Datenkurven zu unterscheiden."""
    x = x1
    while x < x2:
        zeichnung.line([(x, y), (min(x + strich, x2), y)], fill=farbe,
                       width=breite)
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
                         "--profile-csv wird auch gelesen -- bevorzugt "
                         "deren rohe x/y-Spalten, falls vorhanden, sonst "
                         "ersatzweise die Seitenebene (u_mm/v_mm).")
    ap.add_argument("--png", default="geradheit.png",
                    help="Dateiname der Grafik (Default geradheit.png)")
    ap.add_argument("--bins", type=int, default=30,
                    help="Anzahl der Abschnitte für Mittelwert und Streuung "
                         "(Default 30)")
    ap.add_argument("--breite", type=int, default=1200)
    ap.add_argument("--hoehe", type=int, default=700)
    ap.add_argument("--y-min", type=float, default=Y_BEREICH_MIN_MM,
                    help=f"Untere Grenze des benutzten y-Bereichs in mm "
                        f"(Default {Y_BEREICH_MIN_MM:g})")
    ap.add_argument("--y-max", type=float, default=Y_BEREICH_MAX_MM,
                    help=f"Obere Grenze des benutzten y-Bereichs in mm "
                        f"(Default {Y_BEREICH_MAX_MM:g})")
    ap.add_argument("--slide-show", nargs="?", type=float, metavar="FAKTOR",
                    const=SLIDE_SHOW_SKALA, default=None,
                    help=f"Schrift und Linien für die Projektion vergrößern. "
                         f"Ohne Zahl Faktor {SLIDE_SHOW_SKALA:g}, mit Zahl "
                         f"genau diese (z.B. --slide-show 3). Betrifft Achsen- "
                         f"und Legendentext, die Ränder und Abstände, die "
                         f"daran hängen, und die Strichstärke aller "
                         f"gezeichneten Linien. Die Datenfläche behält die "
                         f"über --breite/--hoehe angeforderte Größe; die "
                         f"Leinwand wächst um den Zuwachs der Ränder.")
    ap.add_argument("--kein-plot", action="store_true",
                    help="Nur den Textbericht ausgeben, keine PNG schreiben")
    args = ap.parse_args(sys.argv[1:] if argv is None else argv)

    try:
        skala = loese_skala(args.slide_show)
    except ValueError as fehler:
        ap.error(str(fehler))

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
        if quelle == "profile-csv-uv":
            print(f"[geradheit] {pfad}: Profil-CSV erkannt -- keine rohen "
                  f"x/y-Spalten gefunden, ausgewertet werden ersatzweise "
                  f"u_mm/v_mm, und das sind SEITENEBENEN-Koordinaten "
                  f"(Kalibrierung, Düsenversatz und Gierwinkel sind "
                  f"eingerechnet). Und: geschrieben wird nur, wenn "
                  f"tatsächlich Spalten rausgehen -- die Datei ist also "
                  f"KEINE gleichmäßige Zeitreihe und für Rauschmessung "
                  f"weiterhin kein Ersatz für --pos --pos-json.")
        elif quelle == "profile-csv-xy":
            print(f"[geradheit] {pfad}: Profil-CSV erkannt -- rohe "
                  f"Sensorkoordinaten x/y verwendet. Trotzdem: geschrieben "
                  f"wird nur, wenn tatsächlich Spalten rausgehen -- die "
                  f"Datei ist also KEINE gleichmäßige Zeitreihe und für "
                  f"Rauschmessung weiterhin kein Ersatz für "
                  f"--pos --pos-json.")
        messreihen.append((pfad, xs, ys))

    if not messreihen:
        print("[geradheit] Keine auswertbare Datei.")
        return 2

    ergebnis = auswerten(messreihen, anzahl_bins=args.bins,
                         y_min=args.y_min, y_max=args.y_max)
    print(bericht(ergebnis))

    if not args.kein_plot and "fehler" not in ergebnis:
        try:
            if zeichne_plot(ergebnis, args.png, args.breite, args.hoehe,
                            skala):
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
