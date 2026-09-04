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

Tabelle und Grafik zeigen genau EINE Achse (``--achse x``/``y``/``z``,
Default x) -- die anderen beiden werden nicht mit eingerechnet. Statt einer
Standardabweichung zeigt die gewählte Achse drei Kennzahlen: Durchschnitt,
p95 und p99 der Abweichung vom Mittelwert je Sample, wie bei
Latenzmessungen üblich. Der Grenzabstand im FAZIT basiert auf dem
**p95-Wert der gewählten Achse** -- ``--achse`` bestimmt also nicht nur die
Anzeige, sondern auch, wogegen die Düsenreihen-Schwelle geprüft wird.

    python funktionen/rauschen_entfernung.py rausch_d*.jsonl --achse y
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

# Von auswerten() (welche Achse ausgewertet wird), bericht() und
# zeichne_plot() (welcher Tupel-Index zu welcher Achse gehört) gemeinsam
# benutzt -- deshalb hier oben statt in einem der drei Abschnitte.
_ACHS_INDEX = {"x": 0, "y": 1, "z": 2}


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


def _perzentil(werte, p):
    """
    p-tes Perzentil (0..100) per linearer Interpolation zwischen den
    beiden umgebenden Rängen -- dieselbe Konvention wie NumPy's Default
    (``interpolation="linear"``), von Hand nachrechenbar ohne NumPy als
    Abhängigkeit (siehe Moduldocstring: dieses Werkzeug bleibt bewusst
    eigenständig).
    """
    if not werte:
        return 0.0
    s = sorted(werte)
    if len(s) == 1:
        return s[0]
    rang = (p / 100.0) * (len(s) - 1)
    unten = int(math.floor(rang))
    oben = int(math.ceil(rang))
    if unten == oben:
        return s[unten]
    anteil = rang - unten
    return s[unten] + anteil * (s[oben] - s[unten])


def rauschen_kennzahlen(werte):
    """
    Rauschen EINER Achse als drei Zahlen statt einer: Durchschnitt, p95, p99
    der absoluten Abweichung jedes einzelnen Samples vom Mittelwert der
    Aufzeichnung (``|wert_i - mittel|``) -- dieselbe Art Kennzahl wie bei
    Latenzen üblich (avg/p95/p99), hier auf die Sensorposition angewandt.

    Anders als ``_stdabw`` (die klassische Standardabweichung, quadratisch
    gewichtet und damit von einzelnen Ausreißern überproportional
    beeinflusst) zeigen p95/p99 direkt, wie schlecht der SCHLECHTESTE
    typische Moment ist -- die Zahl, die für "reicht die Genauigkeit noch"
    eigentlich zählt, nicht nur der Durchschnitt.
    """
    if not werte:
        return {"avg": 0.0, "p95": 0.0, "p99": 0.0}
    m = _mittel(werte)
    abweichungen = [abs(w - m) for w in werte]
    return {
        "avg": _mittel(abweichungen),
        "p95": _perzentil(abweichungen, 95),
        "p99": _perzentil(abweichungen, 99),
    }


def werte_einer_datei(xs, ys, zs, fenster):
    """
    Kennzahlen einer einzelnen Aufzeichnung.

    Bewusst KEIN 3D-kombinierter Wert mehr (das frühere ``rms3d``, RMS-Abstand
    vom 3D-Mittelpunkt über alle drei Achsen): dieses Werkzeug wertet genau
    EINE gewählte Achse aus (siehe ``--achse``), die anderen beiden sind für
    diese Auswertung nicht relevant und sollen auch keine Zahl mehr
    beeinflussen, die diese eine Achse betrifft.
    """
    mx, my, mz = _mittel(xs), _mittel(ys), _mittel(zs)
    return {
        "punkte": len(xs),
        "mittel": (mx, my, mz),
        "sigma": (_stdabw(xs), _stdabw(ys), _stdabw(zs)),
        "spitze": (_spitze(xs), _spitze(ys), _spitze(zs)),
        "fenster": (fenster_streuung(xs, fenster), fenster_streuung(ys, fenster),
                    fenster_streuung(zs, fenster)),
        # Je Achse avg/p95/p99 (siehe rauschen_kennzahlen) -- das, was
        # --achse im Bericht/Plot tatsächlich zeigt, UND die Grundlage des
        # Grenzabstands (siehe auswerten(): p95 der gewählten Achse). Für
        # alle drei Achsen berechnet, nicht nur die gewählte: einmalige
        # Kosten beim Einlesen, damit ein späterer Aufruf mit anderer
        # --achse nicht neu einlesen muss (siehe main()).
        "rauschen": {"x": rauschen_kennzahlen(xs), "y": rauschen_kennzahlen(ys),
                     "z": rauschen_kennzahlen(zs)},
    }


def grenzabstand(punkte, schwelle=DUESENTEILUNG_MM, wert_key="wert"):
    """
    Abstand, bei dem ``punkte[i][wert_key]`` die ``schwelle`` überschreitet —
    linear zwischen den beiden Messpunkten interpoliert, die sie einrahmen.

    Bewusst allgemein über ``wert_key`` statt fest auf eine bestimmte
    Kennzahl verdrahtet: welcher Wert das ist (früher die 3D-Streuung,
    heute ``auswerten()``s p95 der gewählten Achse, siehe dort), ist eine
    Entscheidung des Aufrufers, nicht dieser Funktion.

    Rückgabe ``(abstand, art)`` mit ``art``:
      * ``"interpoliert"`` -- die Schwelle liegt zwischen zwei Messpunkten,
      * ``"unterhalb"``    -- schon der nächste Messpunkt liegt darüber,
      * ``"oberhalb"``     -- kein Messpunkt erreicht die Schwelle.
    """
    sortiert = sorted(punkte, key=lambda p: p["abstand"])
    if not sortiert:
        return None, "oberhalb"
    if sortiert[0][wert_key] >= schwelle:
        return sortiert[0]["abstand"], "unterhalb"
    for vorher, nachher in zip(sortiert, sortiert[1:]):
        if nachher[wert_key] >= schwelle:
            spanne = nachher[wert_key] - vorher[wert_key]
            if spanne <= 0:
                return nachher["abstand"], "interpoliert"
            anteil = (schwelle - vorher[wert_key]) / spanne
            return (vorher["abstand"]
                    + anteil * (nachher["abstand"] - vorher["abstand"])), "interpoliert"
    return None, "oberhalb"


def auswerten(messungen, fenster=15, achse="x"):
    """
    ``messungen`` ist eine Liste von ``(abstand_cm, name, xs, ys, zs)``.

    ``achse`` (``"x"``/``"y"``/``"z"``, Default ``"x"``) legt fest, WELCHE
    Achse ausgewertet wird -- nicht nur für Tabelle/Grafik (siehe
    ``bericht``/``zeichne_plot``, die beide ``ergebnis["achse"]`` lesen
    statt eine eigene Achse entgegenzunehmen, damit Anzeige und Grenzwert
    niemals auseinanderlaufen können), sondern auch für den Grenzabstand
    selbst: der basiert auf dem **p95-Wert dieser einen Achse**
    (``rauschen_kennzahlen``), nicht mehr auf einer alle drei Achsen
    kombinierenden 3D-Größe. Die anderen beiden Achsen fließen in dieses
    Ergebnis an keiner Stelle mehr ein.

    Rückgabe: dict mit ``fehler`` oder mit ``punkte`` (je Abstand ein
    Eintrag, nach Abstand sortiert, mit ``punkt["wert"]`` = p95 der
    gewählten Achse), der gewählten ``achse`` und dem Grenzabstand.
    """
    if achse not in _ACHS_INDEX:
        raise ValueError(f"achse muss x/y/z sein, nicht {achse!r}")
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
    for p in punkte:
        # p95, nicht avg oder p99: auf Wunsch des Anlagenbesitzers die Basis
        # des Grenzabstands. avg wäre zu optimistisch (die Hälfte der
        # Samples liegt bereits darüber), p99 zu konservativ für diesen
        # Zweck -- p95 ist der Mittelweg, den auch die Latenz-Konvention
        # dafür üblicherweise nimmt.
        p["wert"] = p["rauschen"][achse]["p95"]
    grenze, art = grenzabstand(punkte, wert_key="wert")
    return {"punkte": punkte, "fenster": fenster, "achse": achse,
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
    """
    Liest die auszuwertende Achse aus ``ergebnis["achse"]`` (gesetzt von
    ``auswerten()``) statt sie hier ein zweites Mal entgegenzunehmen --
    sonst könnten Anzeige und Grenzabstand versehentlich zu verschiedenen
    Achsen gehören. Um eine andere Achse zu sehen, ``auswerten(...,
    achse=...)`` mit der gewünschten Achse aufrufen, nicht diese Funktion.

    Kein 3D-kombinierter Wert mehr in der Ausgabe (siehe
    ``werte_einer_datei``s Docstring): jede gezeigte Zahl gehört zur EINEN
    gewählten Achse, inklusive des Grenzabstands im FAZIT.
    """
    if "fehler" in ergebnis:
        return f"[rauschen] {ergebnis['fehler']}"
    achse = ergebnis["achse"]
    idx = _ACHS_INDEX[achse]

    zeilen = ["---- Sensorrauschen über die Entfernung ----"]
    zeilen.append(f"  Achse: {achse}  (mit --achse x/y/z wählbar; bestimmt "
                  f"auch den Grenzabstand im FAZIT unten)")
    zeilen.append(f"  Fenster für die Kurzzeit-Streuung: "
                  f"{ergebnis['fenster']} Samples "
                  f"(~{ergebnis['fenster'] / hz:.1f} s bei {hz:g} Hz)")
    zeilen.append("")
    zeilen.append(f"  Abstand  Punkte    ~Dauer   {achse}-avg   {achse}-p95   "
                  f"{achse}-p99  Spitze-Sp.  kurzfr.")
    zeilen.append("     (cm)                (s)      (mm)     (mm)     (mm)        "
                  "(mm)     (mm)")
    for p in ergebnis["punkte"]:
        r = p["rauschen"][achse]
        zeilen.append(
            f"  {p['abstand']:7.1f} {p['punkte']:7d} {p['punkte'] / hz:9.1f} "
            f"{r['avg']:9.4f} {r['p95']:8.4f} {r['p99']:8.4f} "
            f"{p['spitze'][idx]:11.4f} {p['fenster'][idx]:8.4f}")

    zeilen.append("")
    zeilen.append(f"  {achse}-Rauschen in Düsenreihen (0,087 mm je Reihe) -- "
                  f"p95 ist die Grundlage des FAZITs unten:")
    for p in ergebnis["punkte"]:
        p95, p99 = p["rauschen"][achse]["p95"], p["rauschen"][achse]["p99"]
        marke = "  <-- über einer Düsenreihe" if p95 >= DUESENTEILUNG_MM else ""
        zeilen.append(f"  {p['abstand']:7.1f} cm : p95 {_reihen(p95):6.2f} Reihen"
                      f"{marke}   ·   p99 {_reihen(p99):6.2f} Reihen")

    zeilen.append("")
    zeilen.extend(_urteil(ergebnis))

    if massstab is not None:
        zeilen.append("")
        zeilen.extend(_massstab_zeilen(massstab))
    return "\n".join(zeilen)


def _urteil(ergebnis):
    zeilen = []
    achse = ergebnis["achse"]
    idx = _ACHS_INDEX[achse]
    grenze, art = ergebnis["grenzabstand"], ergebnis["grenzart"]
    schwelle = DUESENTEILUNG_MM

    if art == "oberhalb":
        groesster = max(p["abstand"] for p in ergebnis["punkte"])
        zeilen.append(f"  FAZIT: Bis {groesster:.0f} cm bleibt das {achse}-p95-"
                      f"Rauschen unter einer Düsenreihe ({schwelle:.4f} mm) — in "
                      f"diesem ganzen Bereich begrenzt der Sensor die "
                      f"Druckqualität auf dieser Achse nicht.")
        zeilen.append("         Wo die Grenze wirklich liegt, ist damit noch "
                      "offen: dafür bei größeren Abständen weitermessen.")
    elif art == "unterhalb":
        kleinster = min(p["abstand"] for p in ergebnis["punkte"])
        zeilen.append(f"  FAZIT: Schon bei {kleinster:.0f} cm liegt das "
                      f"{achse}-p95-Rauschen über einer Düsenreihe "
                      f"({schwelle:.4f} mm). Die brauchbare Grenze liegt "
                      f"darunter und ist mit dieser Messreihe nicht erfasst "
                      f"— näher am Sender nachmessen.")
    else:
        zeilen.append(f"  FAZIT: Das {achse}-p95-Rauschen erreicht eine "
                      f"Düsenreihe ({schwelle:.4f} mm) bei etwa "
                      f"**{grenze:.0f} cm**.")
        zeilen.append(f"         Darunter arbeiten. Jenseits davon begrenzt der "
                      f"Sensor die Druckqualität auf der Achse {achse}, "
                      f"unabhängig von Dosierung, BLE und Kalibrierung.")

    # Drift getrennt melden -- nur dort, wo die Kurzzeit-Streuung deutlich
    # unter der Gesamtstreuung liegt, ist wirklich Drift im Spiel. Beide
    # Größen nur noch von der gewählten Achse, nicht mehr 3D-kombiniert
    # (siehe werte_einer_datei()s Docstring): die anderen Achsen sind für
    # diese Auswertung nicht relevant, also auch nicht für die Drift-Frage.
    driftend = []
    for p in ergebnis["punkte"]:
        kurz = p["fenster"][idx]
        if kurz > 0 and p["sigma"][idx] > 2.0 * kurz:
            driftend.append(p["abstand"])
    if driftend:
        liste = ", ".join(f"{a:.0f}" for a in driftend)
        zeilen.append(f"         DRIFT bei {liste} cm (Achse {achse}): die "
                      f"Gesamtstreuung ist dort mehr als doppelt so groß wie "
                      f"die Streuung innerhalb kurzer Fenster. Der Sensor "
                      f"läuft langsam weg, statt nur zu rauschen — das "
                      f"mittelt sich NICHT heraus. Prüfen, ob der Wagen "
                      f"wirklich fest saß und ob sich in der Nähe etwas "
                      f"bewegt hat.")

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
# Drei Linien EINER Achse (avg/p95/p99), nicht mehr drei Achsen -- siehe
# zeichne_plot()'s Docstring. Dieselben drei Farben wie vorher, jetzt mit
# neuer Bedeutung, damit sich an Legende/Konstanten sonst nichts ändern
# musste.
_FARBEN_KENNZAHL = [(70, 130, 200), (220, 140, 40), (90, 170, 90)]
_REIHE_FARBE = (150, 150, 150)

# --slide-show: Schriftgrad für die Projektion. Auf einer Folie wird die PNG
# auf eine feste Breite skaliert, also zählt allein das VERHÄLTNIS von
# Schrifthöhe zu Bildbreite -- ein größeres Bild mit proportional größerer
# Schrift sähe an der Wand exakt gleich aus. Deshalb wächst hier die Schrift
# gegenüber der Datenfläche, nicht mit ihr. Derselbe Faktor wie in
# geradheit_messreihe.py, damit zwei Grafiken auf derselben Folie
# zusammenpassen.
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

# Grundmaße der Ränder bei SKALA 1. Alle vier halten ausschließlich Text
# (rand_r die Legende neben der Plotfläche) und wachsen mit der Schrift.
_RAND_L, _RAND_R, _RAND_O, _RAND_U = 85, 175, 50, 66


def zeichne_plot(ergebnis, pfad_png, breite=1000, hoehe=620, skala=1.0):
    """
    Rauschen gegen Entfernung als PNG, für die in ``ergebnis["achse"]``
    gewählte Achse (siehe ``auswerten``).

    Zeichnet drei Linien -- Durchschnitt, p95 und p99 der absoluten
    Abweichung dieser Achse (siehe ``rauschen_kennzahlen``) -- plus eine
    gestrichelte Linie bei einer Düsenreihe, die Marke, gegen die das
    Ergebnis gelesen wird. KEINE 3D-kombinierte Linie mehr: die anderen
    beiden Achsen fließen nirgends mehr ein, auch nicht als Referenz (siehe
    ``werte_einer_datei``s Docstring). Der Grenzabstand-Marker gehört zur
    p95-Linie -- das ist die Kennzahl, die ``auswerten()`` dafür benutzt.

    ``skala`` vergrößert die Schrift und alle Maße, die an ihr hängen (Ränder,
    Beschriftungsabstände, Legendenraster) -- ``SLIDE_SHOW_SKALA`` für die
    Projektion, ``1.0`` (Default) für das bisherige Aussehen. Die
    **Datenfläche behält dabei ihre Größe**: die Leinwand wächst um genau die
    Pixel, die die größeren Ränder zusätzlich brauchen. Hier zählt das
    besonders, weil ``rand_r`` die Legende trägt und bei doppelter Schrift
    sonst gut ein Drittel der Bildbreite aus der Plotfläche herausschneiden
    würde.
    """
    from PIL import Image, ImageDraw

    if "fehler" in ergebnis:
        return False
    achse = ergebnis["achse"]

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

    punkte = ergebnis["punkte"]
    rand_l, rand_r = skal(_RAND_L), skal(_RAND_R)
    rand_o, rand_u = skal(_RAND_O), skal(_RAND_U)
    breite = int(round(breite + (_RAND_L + _RAND_R) * (skala - 1.0)))
    hoehe = int(round(hoehe + (_RAND_O + _RAND_U) * (skala - 1.0)))
    pl_b, pl_h = breite - rand_l - rand_r, hoehe - rand_o - rand_u

    x_min = min(p["abstand"] for p in punkte)
    x_max = max(p["abstand"] for p in punkte)
    if x_max - x_min < 1e-9:
        x_min, x_max = x_min - 1.0, x_max + 1.0
    # p99 ist immer die höchste der drei Linien (per Konstruktion, siehe
    # test_rauschen_kennzahlen_p99_ist_nie_kleiner_als_avg), also reicht sie
    # allein für die y-Achsen-Obergrenze.
    y_max = max(p["rauschen"][achse]["p99"] for p in punkte)
    y_max = max(y_max, DUESENTEILUNG_MM * 1.4) * 1.12

    def px(a):
        return rand_l + (a - x_min) / (x_max - x_min) * pl_b

    def py(v):
        return rand_o + (1.0 - v / y_max) * pl_h

    bild = Image.new("RGB", (breite, hoehe), (255, 255, 255))
    z = ImageDraw.Draw(bild)
    schrift = _schrift(max(1, int(round(skal(13)))))
    klein = _schrift(max(1, int(round(skal(11)))))

    for anteil in [i / 6.0 for i in range(7)]:
        x = rand_l + anteil * pl_b
        z.line([(x, rand_o), (x, rand_o + pl_h)], fill=_GITTER)
        z.text((x - skal(12), rand_o + pl_h + skal(8)),
               f"{x_min + anteil * (x_max - x_min):.0f}", fill=_ACHSEN, font=klein)
        y = rand_o + anteil * pl_h
        z.line([(rand_l, y), (rand_l + pl_b, y)], fill=_GITTER)
        z.text((skal(8), y - skal(6)), f"{y_max * (1.0 - anteil):.4f}", fill=_ACHSEN,
               font=klein)

    # Düsenreihen-Marke
    y_reihe = py(DUESENTEILUNG_MM)
    if rand_o <= y_reihe <= rand_o + pl_h:
        _gestrichelt(z, rand_l, y_reihe, rand_l + pl_b, _REIHE_FARBE,
                     strich=skal(6), luecke=skal(5), breite=strich(1))

    # avg/p95/p99 der gewählten Achse -- steigende Linienstärke, weil p99
    # der Wert ist, der am meisten zählt (der seltene schlechte Moment).
    for kennzahl, breite_linie in (("avg", 1), ("p95", 2), ("p99", 3)):
        index = ("avg", "p95", "p99").index(kennzahl)
        farbe = _FARBEN_KENNZAHL[index]
        pts = [(px(p["abstand"]), py(p["rauschen"][achse][kennzahl])) for p in punkte]
        if len(pts) >= 2:
            z.line(pts, fill=farbe, width=strich(breite_linie))
        radius = skal(2)
        for pt in pts:
            z.ellipse([pt[0] - radius, pt[1] - radius,
                       pt[0] + radius, pt[1] + radius], fill=farbe)

    # Grenzabstand markieren -- gehört zur p95-Linie, siehe auswerten()
    if ergebnis["grenzart"] == "interpoliert" and ergebnis["grenzabstand"]:
        gx = px(ergebnis["grenzabstand"])
        if rand_l <= gx <= rand_l + pl_b:
            _gestrichelt_v(z, gx, rand_o, rand_o + pl_h, (120, 120, 190),
                           strich=skal(6), luecke=skal(5), breite=strich(1))
            z.text((gx + skal(4), rand_o + skal(4)),
                   f"{ergebnis['grenzabstand']:.0f} cm", fill=(80, 80, 160),
                   font=klein)

    z.rectangle([rand_l, rand_o, rand_l + pl_b, rand_o + pl_h], outline=_ACHSEN)
    z.text((rand_l, skal(16)),
           f"Sensorrauschen über die Entfernung zum Sender (Achse {achse})",
           fill=(20, 20, 20), font=schrift)
    z.text((rand_l + pl_b / 2 - skal(55), hoehe - skal(24)),
           "Abstand zum Sender (cm)", fill=_ACHSEN, font=klein)
    z.text((skal(8), rand_o - skal(22)), "Streuung (mm)", fill=_ACHSEN, font=klein)

    lx, ly = rand_l + pl_b + skal(14), rand_o + skal(4)
    namen = (f"{achse}-avg", f"{achse}-p95 (Grenzwert)", f"{achse}-p99")
    for index, name in enumerate(namen):
        z.line([(lx, ly + skal(6)), (lx + skal(20), ly + skal(6))],
               fill=_FARBEN_KENNZAHL[index], width=strich(index + 1))
        z.text((lx + skal(26), ly), name, fill=(40, 40, 40), font=klein)
        ly += skal(18)
    ly += skal(4)
    _gestrichelt(z, lx, ly + skal(6), lx + skal(20), _REIHE_FARBE,
                 strich=skal(6), luecke=skal(5), breite=strich(1))
    z.text((lx + skal(26), ly), "1 Düsenreihe", fill=(40, 40, 40), font=klein)

    bild.save(pfad_png)
    return True


def _gestrichelt(z, x1, y, x2, farbe, strich=6, luecke=5, breite=1):
    """Waagerechte gestrichelte Linie.

    Strich-/Lückenlänge sind Parameter, damit ein vergrößerter Plot sie
    mitskalieren kann: bliebe das 6/5-Raster fest, während die Linie dicker
    wird, verschmölzen die Striche optisch zu einer durchgezogenen Linie und
    die Marke wäre nicht mehr von den Datenkurven zu unterscheiden."""
    x = x1
    while x < x2:
        z.line([(x, y), (min(x + strich, x2), y)], fill=farbe, width=breite)
        x += strich + luecke


def _gestrichelt_v(z, x, y1, y2, farbe, strich=6, luecke=5, breite=1):
    """Senkrechte Entsprechung zu :func:`_gestrichelt`."""
    y = y1
    while y < y2:
        z.line([(x, y), (x, min(y + strich, y2))], fill=farbe, width=breite)
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
    ap.add_argument("--achse", choices=("x", "y", "z"), default="x",
                    help="Welche Achse ausgewertet wird (Default x) -- Tabelle, "
                         "Grafik UND der Grenzabstand im FAZIT. Zeigt drei "
                         "Kennzahlen dieser einen Achse: Durchschnitt, p95 und "
                         "p99 der Abweichung vom Mittelwert (siehe "
                         "rauschen_kennzahlen). Der Grenzabstand basiert auf "
                         "dem p95-Wert dieser Achse; die anderen beiden Achsen "
                         "fließen an keiner Stelle mehr ein (kein 3D-RMS mehr).")
    ap.add_argument("--hz", type=float, default=STANDARD_HZ,
                    help=f"Abtastrate von --pos, nur für die Sekundenangaben "
                         f"(Default {STANDARD_HZ:g})")
    ap.add_argument("--massstab", default=None,
                    help="Test 2b: ABSTAND=GEMESSEN-Paare, z.B. "
                         "'10=99.4,20=99.1'")
    ap.add_argument("--referenz", type=float, default=100.0,
                    help="Wahre Referenzstrecke in mm für --massstab "
                         "(Default 100)")
    ap.add_argument("--slide-show", nargs="?", type=float, metavar="FAKTOR",
                    const=SLIDE_SHOW_SKALA, default=None,
                    help=f"Schrift und Linien für die Projektion vergrößern. "
                         f"Ohne Zahl Faktor {SLIDE_SHOW_SKALA:g}, mit Zahl "
                         f"genau diese (z.B. --slide-show 3). Betrifft Titel, "
                         f"Achsen- und Legendentext, die Ränder und Abstände, "
                         f"die daran hängen, und die Strichstärke aller "
                         f"gezeichneten Linien. Die Datenfläche behält ihre "
                         f"Größe; die Leinwand wächst um den Zuwachs der "
                         f"Ränder.")
    ap.add_argument("--kein-plot", action="store_true")
    args = ap.parse_args(sys.argv[1:] if argv is None else argv)

    try:
        skala = loese_skala(args.slide_show)
    except ValueError as fehler:
        ap.error(str(fehler))

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

    ergebnis = auswerten(messungen, fenster=args.fenster, achse=args.achse)
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
            if zeichne_plot(ergebnis, args.png, skala=skala):
                print(f"\n  Grafik geschrieben: {args.png}")
        except ImportError:
            print("\n[rauschen] Pillow (PIL) fehlt — der Textbericht oben ist "
                  "vollständig.")
        except OSError as fehler:
            print(f"\n[rauschen] Grafik konnte nicht geschrieben werden: {fehler}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
