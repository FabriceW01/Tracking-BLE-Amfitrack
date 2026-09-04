"""
Tests für funktionen/geradheit_messreihe.py (keine Hardware, keine GUI).

Das Werkzeug ist eigenständig und hält die Düsenteilung selbst vor, statt
printhead.geometry zu importieren. Der wichtigste Test hier ist deshalb der
DRIFT-Test: der lokale Wert muss mit printhead.geometry.NOZZLE_PITCH_MM
übereinstimmen. Läuft er weg, rechnet das Werkzeug Abweichungen in
"Düsenreihen" um, die es an dieser Anlage nicht gibt.

Die übrigen Tests bevorzugen Fälle mit bekannter analytischer Antwort: eine
konstruierte Neigung, ein konstruierter Bogen und ein konstruiertes Rauschen
müssen als genau diese Werte zurückkommen.

Aufruf:  python tests/test_geradheit_messreihe.py
"""

import json
import math
import os
import random
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "funktionen"))

import geradheit_messreihe as G                              # noqa: E402
from printhead.geometry import NOZZLE_PITCH_MM               # noqa: E402

WERKZEUG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "funktionen", "geradheit_messreihe.py")


# ================================================== Drift gegen die Anlage
def test_duesenteilung_stimmt_mit_der_geometrie_ueberein():
    # DER zentrale Test: der eigenständig vorgehaltene Wert muss dem
    # entsprechen, mit dem der Drucker tatsächlich arbeitet.
    assert abs(G.DUESENTEILUNG_MM - NOZZLE_PITCH_MM) < 1e-12


# ========================================================= Geradenanpassung
def test_gerade_findet_bekannte_neigung():
    for grad in (0.0, 1.5, 30.0, 89.0):
        rad = math.radians(grad)
        xs = [s * math.cos(rad) for s in range(0, 100)]
        ys = [s * math.sin(rad) for s in range(0, 100)]
        fit = G.passe_gerade_an(xs, ys)
        gemessen = math.degrees(math.atan2(fit["richtung"][1],
                                           fit["richtung"][0]))
        assert abs(gemessen - grad) < 1e-6, (grad, gemessen)


def test_gerade_kommt_mit_senkrechter_bahn_zurecht():
    # Eine y-auf-x-Regression hätte hier unendliche Steigung; die Hauptachse
    # kennt keine bevorzugte Achse. Fahrten überwiegend entlang y sind ein
    # völlig normaler Fall und dürfen keinen Sonderweg brauchen.
    xs = [3.0] * 50
    ys = [s * 0.5 for s in range(50)]
    fit = G.passe_gerade_an(xs, ys)
    assert fit is not None
    entlang, abweichung = G.projiziere(xs, ys, fit)
    assert max(abs(a) for a in abweichung) < 1e-9
    assert abs((max(entlang) - min(entlang)) - 24.5) < 1e-9


def test_perfekt_gerade_bahn_hat_keine_abweichung():
    xs = [s * 0.5 for s in range(200)]
    ys = [7.0 + s * 0.5 * math.tan(math.radians(2.0)) for s in range(200)]
    fit = G.passe_gerade_an(xs, ys)
    _, abweichung = G.projiziere(xs, ys, fit)
    assert max(abs(a) for a in abweichung) < 1e-9


def test_gerade_ist_richtungsunabhaengig():
    xs = [s * 1.0 for s in range(50)]
    ys = [s * 0.3 for s in range(50)]
    f1 = G.passe_gerade_an(xs, ys)
    f2 = G.passe_gerade_an(xs[::-1], ys[::-1])
    assert abs(f1["richtung"][0] - f2["richtung"][0]) < 1e-12
    assert abs(f1["richtung"][1] - f2["richtung"][1]) < 1e-12


def test_gerade_gibt_none_bei_entartetem_eingang():
    assert G.passe_gerade_an([], []) is None
    assert G.passe_gerade_an([1.0], [2.0]) is None
    assert G.passe_gerade_an([5.0] * 10, [5.0] * 10) is None


def test_abweichung_behaelt_das_vorzeichen():
    # +0.2 dann -0.2 ist krumm, ±0.2 zufällig ist Rauschen -- als Betrag
    # wäre der Unterschied weg.
    xs = list(range(20))
    ys = [0.0] * 10 + [1.0] * 10
    fit = G.passe_gerade_an(xs, ys)
    _, abweichung = G.projiziere(xs, ys, fit)
    assert min(abweichung) < 0 < max(abweichung)


# ====================================================== Verankerung bei y=0
def test_verankere_bei_y_null_schiebt_mittelpunkt_auf_die_gerade():
    # Rein vertikale Richtung: der neue Mittelpunkt muss exakt bei y=0
    # landen, x bleibt unveraendert (die Gerade ist ja x=5 konstant).
    fit = {"mittelpunkt": (5.0, 10.0), "richtung": (0.0, 1.0),
          "normale": (-1.0, 0.0)}
    neu, verankert = G._verankere_bei_y_null(fit)
    assert verankert is True
    assert abs(neu["mittelpunkt"][0] - 5.0) < 1e-9
    assert abs(neu["mittelpunkt"][1] - 0.0) < 1e-9
    assert neu["richtung"] == fit["richtung"]
    assert neu["normale"] == fit["normale"]


def test_verankere_bei_y_null_mit_schraeger_richtung():
    # Von Hand nachgerechnet: richtung=(0.6,0.8) (Einheitsvektor,
    # 0.36+0.64=1), mittelpunkt=(2,4). t0 = -4/0.8 = -5.
    # neuer Punkt = (2 + (-5)*0.6, 4 + (-5)*0.8) = (-1, 0).
    fit = {"mittelpunkt": (2.0, 4.0), "richtung": (0.6, 0.8),
          "normale": (-0.8, 0.6)}
    neu, verankert = G._verankere_bei_y_null(fit)
    assert verankert is True
    assert abs(neu["mittelpunkt"][0] - (-1.0)) < 1e-9
    assert abs(neu["mittelpunkt"][1] - 0.0) < 1e-9


def test_verankere_bei_y_null_laesst_waagerechte_fahrt_unveraendert():
    # ry praktisch 0 -- keine stabile y=0-Kreuzung, der urspruengliche
    # (datenbasierte) Bezugspunkt bleibt stehen, verankert=False.
    fit = {"mittelpunkt": (5.0, 10.0), "richtung": (1.0, 0.0),
          "normale": (0.0, 1.0)}
    neu, verankert = G._verankere_bei_y_null(fit)
    assert verankert is False
    assert neu["mittelpunkt"] == fit["mittelpunkt"]
    assert neu is fit


def test_auswerten_verankert_entlang_bei_absolutem_y_null_fuer_vertikale_fahrt():
    # Rein vertikale Fahrt (xs konstant): richtung ist exakt (0, 1), also
    # muss NACH der Verankerung entlang == y sein, Punkt fuer Punkt --
    # nicht nur im Mittel. Weiter Y-Bereich, damit hier nichts gefiltert
    # wird.
    xs = [3.0] * 50
    ys = [10.0 + s * 0.5 for s in range(50)]
    e = G.auswerten([("f1", xs, ys)], y_min=-1000.0, y_max=1000.0)
    entlang = e["fahrten"][0]["entlang"]
    for entlang_wert, y_wert in zip(entlang, ys):
        assert abs(entlang_wert - y_wert) < 1e-9


def test_dichte_verzerrt_den_gewichteten_mittelwert_aber_nicht_die_verankerung():
    # Zeigt das gemeldete Symptom ursaechlich: bei ungleichmaessiger
    # Punktedichte entlang der Fahrt (hier: duenn von -90 bis -45, dicht
    # von -40 bis 80) liegt der schlicht gemittelte y-Wert -- die vor
    # dieser Änderung als Bezugspunkt diente -- spuerbar abseits von 0.
    # Die Verankerung ignoriert diese Dichte und haengt trotzdem exakt
    # bei y=0.
    xs = [3.0] * 200
    ys = [-90.0 + i * (45.0 / 40) for i in range(40)]          # duenn
    ys += [-40.0 + i * (120.0 / 160) for i in range(160)]      # dicht
    mittelwert_roh = sum(ys) / len(ys)
    assert abs(mittelwert_roh) > 1.0, mittelwert_roh   # die alte Bezugsgroesse waere spuerbar daneben

    e = G.auswerten([("f1", xs, ys)], y_min=-1000.0, y_max=1000.0)
    entlang = e["fahrten"][0]["entlang"]
    for entlang_wert, y_wert in zip(entlang, ys):
        assert abs(entlang_wert - y_wert) < 1e-9


def test_auswerten_setzt_y_verankert_flag():
    xs = [3.0] * 50
    ys = [10.0 + s * 0.5 for s in range(50)]           # ueberwiegend vertikal
    assert G.auswerten([("f1", xs, ys)], y_min=-1000, y_max=1000)["y_verankert"] is True

    xs2 = [s * 0.5 for s in range(50)]
    ys2 = [10.0] * 50                                   # rein waagerecht
    assert G.auswerten([("f1", xs2, ys2)], y_min=-1000, y_max=1000)["y_verankert"] is False


def test_zeichne_plot_achse_bleibt_bei_verankerung_am_konfigurierten_bereich():
    # Regressionstest gegen genau das gemeldete Problem: eine Fahrt, die
    # den Y-Bereich NICHT vollständig/symmetrisch abdeckt (hier nur y von
    # 0 bis ~89, nicht -90 bis 90). _verankere_bei_y_null allein macht
    # nur "entlang == 0" zu "y == 0" -- das reicht nicht, wenn die
    # Plot-Achse trotzdem aus den (einseitigen) Daten abgeleitet würde:
    # dann läge der Punkt bei y=0 am LINKEN Rand des Plots statt in der
    # Mitte. Mit der jetzt fest auf --y-min/--y-max stehenden Achse muss
    # er in der Mitte landen -- direkt an den tatsächlich gezeichneten
    # Pixeln geprüft, nicht nur an der internen Formel.
    xs = [3.0] * 100
    ys = [i * 0.9 for i in range(100)]      # 0 .. ~89, NICHT -90..90
    e = G.auswerten([("f1", xs, ys)])
    assert e["y_verankert"] is True

    from PIL import Image
    breite, hoehe = 1200, 700
    rand_l, rand_r = 90, 20
    pl_b = breite - rand_l - rand_r
    mitte_px = rand_l + 0.5 * pl_b
    farbe = (70, 130, 200)   # G._FARBEN[0] -- die erste (und einzige) Fahrt

    ziel = tempfile.mktemp(suffix=".png")
    try:
        assert G.zeichne_plot(e, ziel, breite=breite, hoehe=hoehe) is True
        with Image.open(ziel) as bild:
            # Kurve muss nahe der PLOT-MITTE auftauchen (y=0-Punkt bei
            # fester -90..90-Achse) ...
            mitte_treffer = any(
                bild.getpixel((int(mitte_px) + dx, y)) == farbe
                for dx in range(-3, 4) for y in range(50, hoehe - 50))
            assert mitte_treffer, "Kurve fehlt an der erwarteten Plot-Mitte"

            # ... und NICHT mehr am linken Rand (wo sie ohne die feste
            # Achse gelandet wäre, weil die Daten selbst bei entlang≈0
            # anfangen).
            rand_treffer = any(
                bild.getpixel((rand_l + dx, y)) == farbe
                for dx in range(0, 8) for y in range(50, hoehe - 50))
            assert not rand_treffer, "Kurve haengt noch am linken Rand"
    finally:
        if os.path.exists(ziel):
            os.unlink(ziel)


def test_zeichne_plot_ohne_verankerung_bleibt_beim_datenbereich():
    # Ohne Verankerung (ueberwiegend waagerechte Fahrt) muss die Achse
    # weiterhin aus den Daten selbst kommen -- --y-min/--y-max waeren
    # hier keine sinnvolle x-Achse (entlang hat nichts mit y zu tun).
    xs = [s * 5.0 for s in range(101)]      # 0 .. 500 mm Fahrtstrecke
    ys = [10.0] * 101
    e = G.auswerten([("f1", xs, ys)], y_min=-1000.0, y_max=1000.0)
    assert e["y_verankert"] is False

    from PIL import Image
    breite, hoehe = 1200, 700
    rand_l, rand_r = 90, 20
    pl_b = breite - rand_l - rand_r
    farbe = (70, 130, 200)

    ziel = tempfile.mktemp(suffix=".png")
    try:
        assert G.zeichne_plot(e, ziel, breite=breite, hoehe=hoehe) is True
        with Image.open(ziel) as bild:
            # Kurve muss nahe dem RECHTEN Rand auftauchen -- nur
            # möglich, wenn die Achse den vollen ~500mm-Datenbereich
            # abdeckt statt der festen 180mm-Spanne von --y-min/--y-max.
            rechter_rand_x = rand_l + pl_b
            treffer = any(
                bild.getpixel((rechter_rand_x - dx, y)) == farbe
                for dx in range(0, 8) for y in range(50, hoehe - 50))
            assert treffer, "Kurve deckt nicht den vollen Datenbereich ab"
    finally:
        if os.path.exists(ziel):
            os.unlink(ziel)


def test_verankerung_aendert_abweichung_und_spannen_nicht():
    # Dieselbe Form, einmal um y verschoben -- Winkel/Bogen/Rauschen
    # muessen komplett unveraendert bleiben (siehe Docstring von
    # _verankere_bei_y_null: richtung/normale sind orthogonal, die
    # Verschiebung wirkt nur auf "entlang").
    a = _fahrt(bogen_mm=0.10, rausch_mm=0.02, seed=1)
    b_xs, b_ys = _fahrt(bogen_mm=0.10, rausch_mm=0.02, seed=1)
    b = (b_xs, [y + 500.0 for y in b_ys])
    ea = G.auswerten([("a", *a)], y_min=-1e6, y_max=1e6)
    eb = G.auswerten([("b", *b)], y_min=-1e6, y_max=1e6)
    fa, fb = ea["fahrten"][0], eb["fahrten"][0]
    assert abs(fa["rms_mm"] - fb["rms_mm"]) < 1e-9
    assert abs(fa["spanne_mm"] - fb["spanne_mm"]) < 1e-9
    assert abs(fa["strecke_mm"] - fb["strecke_mm"]) < 1e-9
    assert abs(fa["max_abs_mm"] - fb["max_abs_mm"]) < 1e-9


# ================================================================= Binning
def test_binne_mittelt_je_abschnitt():
    kanten = [0.0, 10.0, 20.0]
    entlang = [1.0, 2.0, 3.0, 11.0, 12.0]
    abweichung = [1.0, 2.0, 3.0, 10.0, 20.0]
    assert G.binne(entlang, abweichung, kanten) == [2.0, 15.0]


def test_binne_laesst_leere_abschnitte_als_none():
    # None statt 0.0: eine Lücke in der Fahrt muss eine Lücke bleiben und
    # darf nicht als "hier war die Abweichung null" gelesen werden.
    kanten = [0.0, 10.0, 20.0, 30.0]
    ergebnis = G.binne([1.0, 25.0], [5.0, 7.0], kanten)
    assert ergebnis[0] == 5.0 and ergebnis[1] is None and ergebnis[2] == 7.0


def test_binne_nimmt_den_rechten_rand_in_den_letzten_abschnitt():
    kanten = [0.0, 10.0, 20.0]
    assert G.binne([20.0], [3.0], kanten) == [None, 3.0]


# ========================================================== Gesamtauswertung
def _fahrt(neigung_grad=1.5, bogen_mm=0.10, rausch_mm=0.0, n=300,
           laenge_mm=150.0, versatz_mm=0.0, seed=1):
    """Konstruiert eine Fahrt mit exakt bekannten Eigenschaften."""
    rng = random.Random(seed)
    rad = math.radians(neigung_grad)
    xs, ys = [], []
    for i in range(n):
        s = laenge_mm * i / (n - 1)
        quer = bogen_mm * (1.0 - (2.0 * s / laenge_mm - 1.0) ** 2) + versatz_mm
        if rausch_mm:
            quer += rng.gauss(0.0, rausch_mm)
        xs.append(s * math.cos(rad) - quer * math.sin(rad))
        ys.append(s * math.sin(rad) + quer * math.cos(rad) + 20.0)
    return xs, ys


def test_auswerten_findet_neigung_und_bogen_zurueck():
    xs, ys = _fahrt(neigung_grad=1.5, bogen_mm=0.10, rausch_mm=0.0)
    e = G.auswerten([("f1", xs, ys)], anzahl_bins=30)
    assert abs(e["winkel_grad"] - 1.5) < 1e-3, e["winkel_grad"]
    # Die Bahn ist rauschfrei, die Spanne der Abweichung ist also die
    # Bogenhöhe -- abzüglich eines Abtastfehlers, der hier hergeleitet und
    # nicht großzügig weggerundet wird: bei n=300 Punkten über [0, L] liegt
    # KEIN Sample exakt auf dem Scheitel (der läge bei Index 149,5). Das
    # nächstgelegene ist um L/(2(n-1)) daneben, der Scheitelwert dort also um
    #   bogen * (1/(n-1))^2 = 0.10 / 299^2 = 1.12e-6 mm
    # zu klein. Die Spanne muss demnach 0.10 minus genau diesen Betrag sein.
    n = 300
    erwartet = 0.10 - 0.10 / (n - 1) ** 2
    assert abs(e["fahrten"][0]["spanne_mm"] - erwartet) < 1e-9, (
        e["fahrten"][0]["spanne_mm"], erwartet)


def test_auswerten_trennt_systematisch_von_rauschen():
    # Drei Fahrten, gleicher Bogen, unabhängiges Rauschen. Der Bogen muss im
    # systematischen Anteil landen, das Rauschen im zufälligen.
    reihen = [(f"f{i}", *_fahrt(bogen_mm=0.10, rausch_mm=0.02, seed=i))
              for i in range(3)]
    e = G.auswerten(reihen, anzahl_bins=30)
    assert abs(e["systematisch_spanne_mm"] - 0.10) < 0.01, e
    assert abs(e["rausch_rms_mm"] - 0.02) < 0.004, e["rausch_rms_mm"]


def test_rauschen_ist_je_messwert_nicht_je_abschnitt():
    # Die Streuung der Abschnitts-MITTELWERTE ist um rund sqrt(Punkte je
    # Abschnitt) kleiner als das tatsächliche Rauschen. Würde sie als
    # "Rauschen" ausgewiesen, hielte man den Sensor für viel ruhiger als er
    # ist -- und jede Messung mit genug Punkten fiele automatisch als
    # "systematisch" aus.
    reihen = [(f"f{i}", *_fahrt(bogen_mm=0.0, rausch_mm=0.02, seed=i))
              for i in range(3)]
    e = G.auswerten(reihen, anzahl_bins=30)
    assert abs(e["rausch_rms_mm"] - 0.02) < 0.004
    assert e["streuung_mittel_mm"] < e["rausch_rms_mm"] / 2.0


def test_urteil_nennt_reines_rauschen_zufaellig():
    reihen = [(f"f{i}", *_fahrt(bogen_mm=0.0, rausch_mm=0.03, seed=i))
              for i in range(3)]
    text = G.bericht(G.auswerten(reihen, anzahl_bins=30))
    assert "überwiegend ZUFÄLLIG" in text


def test_urteil_nennt_reinen_bogen_systematisch():
    reihen = [(f"f{i}", *_fahrt(bogen_mm=0.30, rausch_mm=0.005, seed=i))
              for i in range(3)]
    text = G.bericht(G.auswerten(reihen, anzahl_bins=30))
    assert "überwiegend SYSTEMATISCH" in text
    # Und der Hinweis, wie man Balken von Tracker unterscheidet.
    assert "180 Grad" in text


def test_gemeinsame_gerade_macht_einen_versatz_sichtbar():
    # Je Fahrt eine eigene Gerade würde einen Höhenversatz zwischen zwei
    # Fahrten wegdefinieren. Eine gemeinsame Gerade zeigt ihn.
    a = _fahrt(bogen_mm=0.0, versatz_mm=0.0, seed=1)
    b = _fahrt(bogen_mm=0.0, versatz_mm=0.5, seed=2)
    e = G.auswerten([("a", *a), ("b", *b)], anzahl_bins=20)
    versatz = [f["versatz_mm"] for f in e["fahrten"]]
    assert abs((max(versatz) - min(versatz)) - 0.5) < 0.01, versatz


# ===================================================================== Y-Bereich
def test_filtere_y_bereich_behaelt_nur_werte_im_intervall():
    xs = [1.0, 2.0, 3.0, 4.0]
    ys = [-100.0, -5.0, 5.0, 100.0]
    xs_f, ys_f, entfernt = G.filtere_y_bereich(xs, ys, -90.0, 90.0)
    assert xs_f == [2.0, 3.0] and ys_f == [-5.0, 5.0]
    assert entfernt == 2


def test_filtere_y_bereich_grenzen_sind_inklusiv():
    xs_f, ys_f, entfernt = G.filtere_y_bereich([1.0, 2.0], [-90.0, 90.0],
                                               -90.0, 90.0)
    assert xs_f == [1.0, 2.0] and entfernt == 0


def test_auswerten_filtert_default_y_bereich():
    # Ohne --y-min/--y-max muessen Punkte weit ausserhalb [-90, 90]
    # verworfen werden.
    xs = list(range(20))
    ys = [0.0] * 10 + [500.0] * 10
    e = G.auswerten([("f1", xs, ys)])
    assert e["fahrten"][0]["punkte"] == 10
    assert e["fahrten"][0]["punkte_roh"] == 20
    assert e["fahrten"][0]["y_bereich_entfernt"] == 10
    assert e["y_min"] == -90.0 and e["y_max"] == 90.0


def test_auswerten_y_bereich_ist_einstellbar():
    xs = list(range(20))
    ys = [0.0] * 10 + [500.0] * 10
    e = G.auswerten([("f1", xs, ys)], y_min=-1000.0, y_max=1000.0)
    assert e["fahrten"][0]["punkte"] == 20
    assert e["fahrten"][0]["y_bereich_entfernt"] == 0


def test_auswerten_meldet_fehler_wenn_y_bereich_alles_entfernt():
    xs = [0.0, 1.0, 2.0]
    ys = [500.0, 500.0, 500.0]
    e = G.auswerten([("f1", xs, ys)])
    assert "fehler" in e
    assert "Y-Bereich" in e["fehler"]


def test_auswerten_meldet_teilweise_ausserhalb_liegende_fahrt_als_uebersprungen():
    gut = _fahrt(rausch_mm=0.01)
    schlecht = ([0.0, 1.0, 2.0], [500.0, 500.0, 500.0])
    e = G.auswerten([("gut", *gut), ("schlecht", *schlecht)])
    assert e["anzahl_fahrten"] == 1
    assert len(e["uebersprungen"]) == 1
    assert e["uebersprungen"][0]["name"] == "schlecht"
    assert e["uebersprungen"][0]["im_bereich"] == 0
    assert e["uebersprungen"][0]["punkte_roh"] == 3


def test_bericht_zeigt_y_bereich_und_entfernte_punkte_je_fahrt():
    xs = list(range(20))
    ys = [0.0] * 15 + [500.0] * 5     # 5 von 20 draussen, Rest bleibt gueltig
    text = G.bericht(G.auswerten([("f1", xs, ys)]))
    assert "-90.0" in text and "90.0" in text
    assert "5 von 20" in text


def test_bericht_meldet_uebersprungene_fahrt():
    gut = _fahrt(rausch_mm=0.01)
    schlecht = ([0.0, 1.0, 2.0], [500.0, 500.0, 500.0])
    text = G.bericht(G.auswerten([("gut", *gut), ("schlecht", *schlecht)]))
    assert "schlecht" in text
    assert "übersprungen" in text


def test_cli_y_min_max_flags_aendern_das_ergebnis():
    pfad = _schreib("\n".join(
        json.dumps({"event": "position", "x": float(i), "y": 500.0})
        for i in range(5)) + "\n")
    try:
        # Default-Bereich [-90, 90] -- alle Punkte liegen bei y=500,
        # also ausserhalb -> keine auswertbare Fahrt.
        p_default = _cli(pfad, "--kein-plot")
        assert p_default.returncode == 0, p_default.stderr
        assert "Keine Messreihe" in p_default.stdout

        # Mit weiterem Bereich funktioniert dieselbe Datei.
        p_weiter = _cli(pfad, "--kein-plot", "--y-min", "0", "--y-max", "1000")
        assert p_weiter.returncode == 0, p_weiter.stderr
        assert "Geradheit über die Messreihe" in p_weiter.stdout
        assert "Fahrten              : 1" in p_weiter.stdout
    finally:
        os.unlink(pfad)


def test_auswerten_meldet_fehler_statt_zu_raten():
    assert "fehler" in G.auswerten([])
    assert "fehler" in G.auswerten([("leer", [], [])])
    assert "fehler" in G.auswerten([("punkt", [1.0] * 5, [2.0] * 5)])


def test_einzelne_fahrt_sagt_dass_sie_nichts_trennen_kann():
    xs, ys = _fahrt(rausch_mm=0.02)
    text = G.bericht(G.auswerten([("f1", xs, ys)]))
    assert "eine einzelne Fahrt" in text
    assert "NICHT trennen" in text


def test_nicht_ueberlappende_fahrten_werden_gemeldet():
    a = _fahrt(bogen_mm=0.0, laenge_mm=50.0, n=100)
    b_xs, b_ys = _fahrt(bogen_mm=0.0, laenge_mm=50.0, n=100)
    b = ([x + 500.0 for x in b_xs], b_ys)          # ganz woanders
    e = G.auswerten([("a", *a), ("b", b[0], b[1])], anzahl_bins=20)
    assert e["ueberlappung"] is False
    assert "überlappen sich nicht" in G.bericht(e)


# ================================================================= Einlesen
def _schreib(text, endung=".jsonl"):
    fh = tempfile.NamedTemporaryFile("w", suffix=endung, delete=False,
                                     encoding="utf-8")
    fh.write(text)
    fh.close()
    return fh.name


def test_pos_json_liest_x_und_y():
    pfad = _schreib(
        '{"event":"connected"}\n'
        '{"event":"position","x":1.0,"y":2.0,"z":3.0}\n'
        '{"event":"position","x":4.0,"y":5.0,"z":6.0}\n')
    try:
        xs, ys = G.lies_pos_json(pfad)
        assert xs == [1.0, 4.0] and ys == [2.0, 5.0]
    finally:
        os.unlink(pfad)


def test_pos_json_ueberspringt_kaputte_und_fremde_zeilen():
    # Eine abgebrochene Aufzeichnung endet oft mit einer halben Zeile -- die
    # darf die Auswertung der übrigen Daten nicht verhindern.
    pfad = _schreib(
        '{"event":"position","x":1.0,"y":2.0}\n'
        'kein json\n'
        '{"event":"stopped"}\n'
        '[1,2,3]\n'
        '{"event":"position","x":3.0,"y":4.0}\n'
        '{"event":"position","x":9.0\n')
    try:
        xs, ys = G.lies_pos_json(pfad)
        assert xs == [1.0, 3.0] and ys == [2.0, 4.0]
    finally:
        os.unlink(pfad)


def _csv_zeile(werte):
    """Baut eine CSV-Zeile aus einer Werteliste -- fehlende Felder als
    ECHT LEERE Strings (nicht als fehlende Spalten am Zeilenende), damit
    Testdaten dasselbe Format haben wie eine echte --profile-csv (siehe
    README: "leer statt 0,0,0", wenn kein Tracking-Fix vorlag)."""
    return ",".join("" if w is None else str(w) for w in werte)


def test_profile_csv_bevorzugt_rohe_x_y_wenn_vorhanden():
    # u_mm/v_mm absichtlich auf einen unmöglichen Wert (999) gesetzt --
    # kommen die im Ergebnis zurück, würde fälschlich die Seitenebene
    # statt der rohen Koordinaten gelesen.
    kopf = "t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,x,y,z,qx,qy,qz,qw"
    zeilen = [kopf,
             _csv_zeile([0.1, 0, 0, 999.0, 999.0, 10.0, 50.0,
                        10.0, 20.0, 0.0, 0, 0, 0, 1]),
             _csv_zeile([0.2, 0, 1, 999.0, 999.0, 10.0, 50.0,
                        30.0, 40.0, 0.0, 0, 0, 0, 1])]
    pfad = _schreib("\n".join(zeilen) + "\n", endung=".csv")
    try:
        xs, ys, quelle = G.lies_profile_csv(pfad)
        assert xs == [10.0, 30.0] and ys == [20.0, 40.0]
        assert quelle == "profile-csv-xy"
    finally:
        os.unlink(pfad)


def test_profile_csv_faellt_auf_u_v_zurueck_wenn_x_y_leer_sind():
    kopf = "t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,x,y,z,qx,qy,qz,qw"
    zeilen = [kopf,
             _csv_zeile([0.1, 0, 0, 1.000, 2.000, 10.0, 50.0,
                        None, None, None, None, None, None, None]),
             _csv_zeile([0.2, 0, 1, 3.000, 4.000, 10.0, 50.0,
                        None, None, None, None, None, None, None])]
    pfad = _schreib("\n".join(zeilen) + "\n", endung=".csv")
    try:
        xs, ys, quelle = G.lies_profile_csv(pfad)
        assert xs == [1.0, 3.0] and ys == [2.0, 4.0]
        assert quelle == "profile-csv-uv"
    finally:
        os.unlink(pfad)


def test_profile_csv_ueberspringt_zeilen_ohne_position():
    # Roh-x/y vorhanden, aber in einer Zeile leer (kein Tracking-Fix in
    # diesem Moment) -- muss übersprungen werden, nicht als (0, 0)
    # gezählt (siehe README: "leer statt 0,0,0").
    kopf = "t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,x,y,z,qx,qy,qz,qw"
    zeilen = [kopf,
             _csv_zeile([0.1, 0, 0, 0.0, 0.0, 10.0, 50.0,
                        10.0, 20.0, 0.0, 0, 0, 0, 1]),
             _csv_zeile([0.2, 0, 1, 0.0, 0.0, 10.0, 50.0,
                        None, None, None, None, None, None, None]),
             _csv_zeile([0.3, 0, 2, 0.0, 0.0, 10.0, 50.0,
                        30.0, 40.0, 0.0, 0, 0, 0, 1])]
    pfad = _schreib("\n".join(zeilen) + "\n", endung=".csv")
    try:
        xs, ys, quelle = G.lies_profile_csv(pfad)
        assert xs == [10.0, 30.0] and ys == [20.0, 40.0]
    finally:
        os.unlink(pfad)


def test_profile_csv_haelt_xs_und_ys_ausgerichtet_bei_teilkaputter_zeile():
    # x gueltig, y kaputt (oder umgekehrt) darf NICHT dazu fuehren, dass
    # xs einen Eintrag mehr hat als ys (oder umgekehrt) -- sonst
    # verschieben sich alle folgenden Punkte gegeneinander.
    kopf = "t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,x,y,z,qx,qy,qz,qw"
    zeilen = [kopf,
             _csv_zeile([0.1, 0, 0, 0.0, 0.0, 10.0, 50.0,
                        10.0, 20.0, 0.0, 0, 0, 0, 1]),
             _csv_zeile([0.2, 0, 1, 0.0, 0.0, 10.0, 50.0,
                        "abc", 40.0, 0.0, 0, 0, 0, 1]),   # x kaputt
             _csv_zeile([0.3, 0, 2, 0.0, 0.0, 10.0, 50.0,
                        50.0, "abc", 0.0, 0, 0, 0, 1]),   # y kaputt
             _csv_zeile([0.4, 0, 3, 0.0, 0.0, 10.0, 50.0,
                        70.0, 80.0, 0.0, 0, 0, 0, 1])]
    pfad = _schreib("\n".join(zeilen) + "\n", endung=".csv")
    try:
        xs, ys, quelle = G.lies_profile_csv(pfad)
        assert len(xs) == len(ys)
        assert xs == [10.0, 70.0] and ys == [20.0, 80.0]
    finally:
        os.unlink(pfad)


def test_profile_csv_liest_auch_line_modus_ueber_x_y():
    # Line-Modus hat kein u_mm/v_mm (advance_mm ist nur 1-D) -- war vor
    # der x/y/z-Erweiterung von --profile-csv nicht auswertbar. Jetzt
    # reicht x/y, dieselben Spalten wie im Seiten-Modus.
    kopf = "t_s,column,advance_mm,write_latency_ms,speed_mm_s,x,y,z"
    zeilen = [kopf,
             _csv_zeile([0.1, 0, 0.0, 3.1, 10.0, 10.0, 20.0, 0.0]),
             _csv_zeile([0.2, 1, 0.087, 3.0, 10.0, 30.0, 40.0, 0.0])]
    pfad = _schreib("\n".join(zeilen) + "\n", endung=".csv")
    try:
        xs, ys, quelle = G.lies_profile_csv(pfad)
        assert xs == [10.0, 30.0] and ys == [20.0, 40.0]
        assert quelle == "profile-csv-xy"
    finally:
        os.unlink(pfad)


def test_profile_csv_ohne_positionsspalten_ist_ein_fehler():
    pfad = _schreib("t_s,irgendwas\n0.1,42\n", endung=".csv")
    try:
        G.lies_profile_csv(pfad)
    except ValueError as fehler:
        assert "x/y" in str(fehler) and "u_mm/v_mm" in str(fehler)
        return
    finally:
        os.unlink(pfad)
    raise AssertionError("erwartet: ValueError ohne jede Positionsspalte")


def test_lies_messreihe_erkennt_die_quelle():
    kopf = "t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,x,y,z,qx,qy,qz,qw"
    j = _schreib('{"event":"position","x":1.0,"y":2.0}\n')
    c_xy = _schreib(kopf + "\n" + _csv_zeile(
        [0.1, 0, 0, 1.0, 2.0, 10.0, 50.0, 1.0, 2.0, 0, 0, 0, 0, 1]) + "\n",
        endung=".csv")
    c_uv = _schreib(kopf + "\n" + _csv_zeile(
        [0.1, 0, 0, 1.0, 2.0, 10.0, 50.0, None, None, None, None, None,
         None, None]) + "\n", endung=".csv")
    try:
        assert G.lies_messreihe(j)[2] == "pos-json"
        assert G.lies_messreihe(c_xy)[2] == "profile-csv-xy"
        assert G.lies_messreihe(c_uv)[2] == "profile-csv-uv"
    finally:
        os.unlink(j)
        os.unlink(c_xy)
        os.unlink(c_uv)


# ==================================================================== Grafik
def test_plot_schreibt_ein_lesbares_png():
    from PIL import Image
    reihen = [(f"f{i}", *_fahrt(rausch_mm=0.02, seed=i)) for i in range(3)]
    e = G.auswerten(reihen, anzahl_bins=30)
    ziel = tempfile.mktemp(suffix=".png")
    try:
        assert G.zeichne_plot(e, ziel, breite=900, hoehe=520) is True
        with Image.open(ziel) as bild:
            assert bild.size == (900, 520)
            assert bild.mode == "RGB"
            # Nicht nur weiß: es muss tatsächlich etwas gezeichnet sein.
            assert len(bild.getcolors(maxcolors=100000)) > 5
    finally:
        if os.path.exists(ziel):
            os.unlink(ziel)


def test_plot_geht_auch_mit_einer_einzigen_fahrt():
    ziel = tempfile.mktemp(suffix=".png")
    try:
        e = G.auswerten([("f1", *_fahrt(rausch_mm=0.01))], anzahl_bins=20)
        assert G.zeichne_plot(e, ziel) is True
    finally:
        if os.path.exists(ziel):
            os.unlink(ziel)


def _bogen_fahrt(bogen_mm):
    """
    Fahrt überwiegend entlang y, mit parabelförmigem Bogen quer dazu.

    Die Ausgleichsgerade geht durch den Schwerpunkt, bei einer Parabel
    liegt der aber NICHT in der Mitte zwischen Scheitel und Enden -- die
    Abweichungen werden dadurch deutlich asymmetrisch (rund 0.67 zu 1.33
    mm, Faktor ~2). Genau das braucht es, um eine datenabhängige von einer
    um 0 symmetrischen senkrechten Achse unterscheiden zu können: bei
    symmetrischen Daten sähen beide gleich aus.

    Die leichte NEIGUNG (0.05) ist nicht kosmetisch: bei einer exakt
    senkrechten Geraden ist ``rx == 0``, und dann entscheidet in
    passe_gerade_an der Vorzeichen-Tie-Break über ``ry`` -- der kippt mit
    dem Bogen mit, die Normale ebenfalls, und beide Bogenrichtungen
    liefern am Ende dieselben Abweichungen. Mit Neigung ist ``rx > 0``,
    die Richtung damit eindeutig festgelegt, und das Vorzeichen von
    ``bogen_mm`` steuert tatsächlich, ob das Minimum oder das Maximum
    betragsmäßig überwiegt.

    ``bogen_mm > 0`` -> |min| überwiegt, ``< 0`` -> |max| überwiegt.
    """
    n = 201
    ys = [-90.0 + 180.0 * i / (n - 1) for i in range(n)]
    xs = [3.0 + 0.05 * y + bogen_mm * (y / 90.0) ** 2 for y in ys]
    return xs, ys


def _nulllinien_pixel(bild, rand_l, rand_o, pl_h, spalten=range(2, 40)):
    """y-Pixel der Nulllinie (_MITTEL-Grau (120,120,120), 2px breit).

    Sie hat eine eigene Farbe: Gitter ist (216,216,216), die
    Düsenreihen-Marken sind (150,150,150), die Kurve (70,130,200) --
    ein Treffer auf (120,120,120) kann also nur die Nulllinie sein.
    """
    treffer = set()
    for dx in spalten:
        for y in range(rand_o, rand_o + pl_h + 1):
            if bild.getpixel((rand_l + dx, y)) == (120, 120, 120):
                treffer.add(y)
    return sorted(treffer)


def test_zeichne_plot_nulllinie_liegt_senkrecht_genau_in_der_mitte():
    # Der gemeldete Fehler: der Nullpunkt der senkrechten Achse war nicht
    # die Mitte der Achse, sondern dorthin verschoben, wo die Daten
    # zufällig lagen. Mit den asymmetrischen Abweichungen unten läge die
    # Nulllinie bei einer datenabhängigen Achse auf rund 64 % der Höhe --
    # geprüft wird an den tatsächlichen Pixeln im gespeicherten PNG, nicht
    # an derselben Formel, die die Implementierung benutzt.
    xs, ys = _bogen_fahrt(-2.0)          # |max| ueberwiegt
    e = G.auswerten([("f1", xs, ys)])

    alle_a = [v for f in e["fahrten"] for v in f["abweichung"]]
    # Absicherung, dass der Fixture überhaupt etwas zeigen kann.
    assert abs(max(alle_a)) > 1.5 * abs(min(alle_a)), (min(alle_a), max(alle_a))

    from PIL import Image
    breite, hoehe = 1200, 700
    rand_l, rand_r, rand_o, rand_u = 90, 20, 50, 70
    pl_h = hoehe - rand_o - rand_u
    mitte = rand_o + pl_h / 2.0

    ziel = tempfile.mktemp(suffix=".png")
    try:
        assert G.zeichne_plot(e, ziel, breite=breite, hoehe=hoehe) is True
        with Image.open(ziel) as bild:
            null_y = _nulllinien_pixel(bild, rand_l, rand_o, pl_h)
            assert null_y, "Nulllinie nicht gefunden"
            # Die Linie ist 2px breit, liegt also auf mitte und mitte+1.
            assert max(abs(y - mitte) for y in null_y) <= 1.5, (null_y, mitte)
    finally:
        if os.path.exists(ziel):
            os.unlink(ziel)


def test_zeichne_plot_senkrechte_achse_reicht_oben_und_unten_gleich_weit():
    # Dieselbe Aussage von der anderen Seite: der Abstand der Nulllinie zum
    # oberen und zum unteren Rahmen muss gleich sein. Ohne symmetrische
    # Achse wären das bei diesem Fixture rund 64 % zu 36 %.
    xs, ys = _bogen_fahrt(-2.0)
    e = G.auswerten([("f1", xs, ys)])

    from PIL import Image
    breite, hoehe = 1200, 700
    rand_l, rand_r, rand_o, rand_u = 90, 20, 50, 70
    pl_h = hoehe - rand_o - rand_u

    ziel = tempfile.mktemp(suffix=".png")
    try:
        G.zeichne_plot(e, ziel, breite=breite, hoehe=hoehe)
        with Image.open(ziel) as bild:
            null_y = _nulllinien_pixel(bild, rand_l, rand_o, pl_h)
            assert null_y
            oben = min(null_y) - rand_o
            unten = (rand_o + pl_h) - max(null_y)
            assert abs(oben - unten) <= 2, (oben, unten)
    finally:
        if os.path.exists(ziel):
            os.unlink(ziel)


def test_zeichne_plot_zeigt_duesenreihen_marken_auch_bei_flacher_kurve():
    # Die symmetrische Achse darf die Untergrenze nicht wegoptimieren: bei
    # einer sehr geraden Fahrt muss der Maßstab ±1 Düsenreihe weiterhin im
    # Bild sein, sonst fehlt der Bezug, gegen den die Kurve gelesen wird.
    #
    # Die Abweichung ist hier absichtlich winzig, aber NICHT exakt 0: bei
    # exakt 0 greift der Entartungs-Schutz (grenze = 1.0) und die
    # Untergrenze wäre gar nicht beteiligt -- der Test würde dann auch
    # bestehen, wenn man sie entfernt (genau so ist die Mutation zuerst
    # durchgerutscht).
    ys = [-90.0 + 180.0 * i / 199 for i in range(200)]
    xs = [3.0 + 0.002 * (y / 90.0) ** 2 for y in ys]
    e = G.auswerten([("f1", xs, ys)])
    alle_a = [v for f in e["fahrten"] for v in f["abweichung"]]
    groesste = max(abs(a) for a in alle_a)
    assert 0.0 < groesste < G.DUESENTEILUNG_MM / 10.0, groesste

    from PIL import Image
    breite, hoehe = 1200, 700
    rand_l, rand_o, rand_u = 90, 50, 70
    pl_h = hoehe - rand_o - rand_u
    mitte = rand_o + pl_h / 2.0

    ziel = tempfile.mktemp(suffix=".png")
    try:
        G.zeichne_plot(e, ziel, breite=breite, hoehe=hoehe)
        with Image.open(ziel) as bild:
            marken = set()
            for dx in range(2, 60):
                for y in range(rand_o, rand_o + pl_h + 1):
                    if bild.getpixel((rand_l + dx, y)) == (150, 150, 150):
                        marken.add(y)
            assert marken, "Düsenreihen-Marken fehlen"
            oberhalb = [y for y in marken if y < mitte - 2]
            unterhalb = [y for y in marken if y > mitte + 2]
            assert oberhalb and unterhalb, sorted(marken)
            # ... und beide gleich weit von der Nulllinie entfernt.
            assert abs((mitte - max(oberhalb)) - (min(unterhalb) - mitte)) <= 2
    finally:
        if os.path.exists(ziel):
            os.unlink(ziel)


def test_zeichne_plot_schneidet_die_kurve_in_keiner_richtung_ab():
    # Die Achsengrenze muss aus BEIDEN Extremen gebildet werden. Deshalb
    # beide Bogenrichtungen: einmal überwiegt betragsmäßig das Minimum,
    # einmal das Maximum. Würde nur eines von beiden benutzt, wäre die
    # Achse im jeweils anderen Fall rund halb so groß wie nötig und die
    # Kurve liefe aus dem Rahmen -- mit nur einer Richtung im Test bliebe
    # genau die Hälfte davon unbemerkt.
    from PIL import Image
    breite, hoehe = 1200, 700
    rand_l, rand_r, rand_o, rand_u = 90, 20, 50, 70
    pl_b = breite - rand_l - rand_r
    pl_h = hoehe - rand_o - rand_u
    farbe = (70, 130, 200)

    for bogen, wer in ((2.0, "min"), (-2.0, "max")):
        xs, ys = _bogen_fahrt(bogen)
        e = G.auswerten([("f1", xs, ys)])
        alle_a = [v for f in e["fahrten"] for v in f["abweichung"]]

        # Absicherung, dass der Fixture wirklich das prüft, was er soll.
        if wer == "min":
            assert abs(min(alle_a)) > 1.5 * abs(max(alle_a)), alle_a[:3]
        else:
            assert abs(max(alle_a)) > 1.5 * abs(min(alle_a)), alle_a[:3]

        ziel = tempfile.mktemp(suffix=".png")
        try:
            G.zeichne_plot(e, ziel, breite=breite, hoehe=hoehe)
            with Image.open(ziel) as bild:
                innen = ausserhalb = 0
                for x in range(rand_l + 1, rand_l + pl_b):
                    for y in range(0, hoehe):
                        if bild.getpixel((x, y)) == farbe:
                            if rand_o <= y <= rand_o + pl_h:
                                innen += 1
                            else:
                                ausserhalb += 1
                assert innen > 0, f"Kurve gar nicht gezeichnet (Bogen {bogen})"
                assert ausserhalb == 0, (
                    f"Bogen {bogen}: {ausserhalb} Kurvenpixel ausserhalb des "
                    f"Rahmens -- die senkrechte Achse ist zu klein (|{wer}| "
                    f"nicht beruecksichtigt?)")
        finally:
            if os.path.exists(ziel):
                os.unlink(ziel)


def test_zeichne_plot_beschriftet_die_senkrechte_achse_mit_der_abweichung():
    # Die senkrechte Achse muss mit der ABWEICHUNG beschriftet sein, nicht
    # versehentlich mit der Positionsachse -- genau diese Verwechslung der
    # beiden Achsen ist der Kern des ursprünglichen Fehlers gewesen, und
    # sie wäre an den Kurvenpixeln allein nicht zu sehen (die Beschriftung
    # ändert kein einziges davon).
    #
    # Geprüft wird ohne Texterkennung: dieselbe Zeichenkette wird mit
    # derselben Schrift frisch gerendert und der Bildausschnitt bitweise
    # verglichen. Das ist deterministisch und hängt an keiner Schätzung.
    xs, ys = _bogen_fahrt(-2.0)
    e = G.auswerten([("f1", xs, ys)])
    alle_a = [v for f in e["fahrten"] for v in f["abweichung"]]
    grenze = max(abs(min(alle_a)), abs(max(alle_a)),
                 G.DUESENTEILUNG_MM * 1.3) * 1.08
    erwartet = f"{grenze:+.3f}"

    # Gegenprobe: die Positionsachse trüge hier eine völlig andere Zahl --
    # ohne das könnte der Test versehentlich beide gleich bewerten.
    assert erwartet != f"{e['y_max']:+.3f}"

    from PIL import Image, ImageDraw
    rand_o = 50                       # oberste Gitterlinie, Text bei y-6
    kasten = (8, rand_o - 6, 78, rand_o + 10)

    ziel = tempfile.mktemp(suffix=".png")
    try:
        G.zeichne_plot(e, ziel, breite=1200, hoehe=700)
        with Image.open(ziel) as bild:
            ist = bild.convert("RGB").crop(kasten)
        soll = Image.new("RGB", (kasten[2] - kasten[0],
                                 kasten[3] - kasten[1]), (255, 255, 255))
        ImageDraw.Draw(soll).text((0, 0), erwartet, fill=(60, 60, 60),
                                  font=G._schrift(11))
        assert ist.tobytes() == soll.tobytes(), (
            "oberste Beschriftung der senkrechten Achse ist nicht "
            f"{erwartet!r}")
    finally:
        if os.path.exists(ziel):
            os.unlink(ziel)


# ============================================== Optische Anpassungen (Plot)
def test_zeichne_plot_hat_keine_ueberschrift_mehr():
    # "Abweichung quer zur Fahrt (...)" soll komplett weg sein -- geprüft
    # per Byte-Vergleich an genau der Stelle, an der sie früher stand
    # (nicht per Leerflächen-Annahme: direkt daneben lebt weiterhin die
    # unveränderte "Abweichung (mm)"-Achsenbeschriftung, die zufällig bis
    # in denselben Bereich hineinreicht).
    xs, ys = _fahrt(rausch_mm=0.01)
    e = G.auswerten([("f1", xs, ys)])

    from PIL import Image, ImageDraw
    alter_titel = (f"Abweichung quer zur Fahrt  ({e['anzahl_fahrten']} "
                  f"Fahrt(en), Balken {e['winkel_grad']:+.2f} deg gegen +x)")
    schrift13 = G._schrift(13)
    dummy = Image.new("RGB", (10, 10))
    bbox = ImageDraw.Draw(dummy).textbbox((90, 16), alter_titel, font=schrift13)
    soll = Image.new("RGB", (bbox[2] - bbox[0], bbox[3] - bbox[1]),
                     (255, 255, 255))
    ImageDraw.Draw(soll).text((90 - bbox[0], 16 - bbox[1]), alter_titel,
                              fill=(20, 20, 20), font=schrift13)

    ziel = tempfile.mktemp(suffix=".png")
    try:
        G.zeichne_plot(e, ziel, breite=1200, hoehe=700)
        with Image.open(ziel) as bild:
            crop = bild.convert("RGB").crop(bbox)
        ist, soll_bytes = crop.tobytes(), soll.tobytes()
        # Toleranz statt strikter Ungleichheit: selbst wenn der alte Titel
        # unverändert dort steht, weichen ein paar Bytes an den
        # Buchstaben-Rändern minimal ab (Kantenglättung reagiert leicht
        # anders, je nachdem, worauf gerade gezeichnet wird) -- hier
        # gemessen: 15 von 19194 Bytes. Fehlt der Titel dagegen wirklich
        # (die Fläche ist weiß statt beschriftet), weichen tausende Bytes
        # ab. Die Schwelle liegt weit dazwischen.
        diff = sum(1 for a, b in zip(ist, soll_bytes) if a != b)
        assert diff > 500, (
            f"die alte Überschrift steht (fast) unverändert dort "
            f"({diff} von {len(ist)} Bytes weichen ab)")
    finally:
        if os.path.exists(ziel):
            os.unlink(ziel)


def test_zeichne_plot_legende_liegt_innerhalb_der_plotflaeche():
    # Die Legende stand früher rechts NEBEN der Plotfläche (im breiten
    # rand_r-Streifen) -- der ist jetzt schmal, und der Streifen muss
    # komplett leer (weiß) sein, weil die Legende in einen Kasten
    # INNERHALB der Plotfläche gewandert ist.
    xs, ys = _fahrt(rausch_mm=0.01)
    e = G.auswerten([("f1", xs, ys)])

    from PIL import Image
    breite, hoehe = 1200, 700
    rand_l, rand_r = 90, 20
    pl_b = breite - rand_l - rand_r

    ziel = tempfile.mktemp(suffix=".png")
    try:
        G.zeichne_plot(e, ziel, breite=breite, hoehe=hoehe)
        with Image.open(ziel) as bild:
            bild = bild.convert("RGB")
            rand_frei = all(
                bild.getpixel((x, y)) == (255, 255, 255)
                for x in range(rand_l + pl_b + 1, breite)
                for y in range(0, hoehe, 5))
        assert rand_frei, "im rechten Rand steht noch etwas -- alte Legende?"
    finally:
        if os.path.exists(ziel):
            os.unlink(ziel)


def test_zeichne_plot_legende_hat_einen_eigenen_hintergrundkasten():
    # Innerhalb der Plotfläche sitzend braucht die Legende einen sichtbar
    # abgesetzten Kasten -- sonst verschwimmt ihr Text mit der Kurve
    # dahinter. Geprüft an der oberen Kante: die muss über ihre volle
    # Breite in der Rahmenfarbe gezeichnet sein.
    xs, ys = _fahrt(rausch_mm=0.01)
    e = G.auswerten([("f1", xs, ys)])

    from PIL import Image, ImageDraw
    breite, hoehe = 1200, 700
    rand_l, rand_o = 90, 50
    pl_b = breite - rand_l - 20
    schrift = G._schrift(11)
    dummy = Image.new("RGB", (10, 10))
    draw = ImageDraw.Draw(dummy)
    text_breite = max(
        draw.textbbox((0, 0), t, font=schrift)[2]
        for t in ("Messwert", "1 Düsenreihe"))
    legende_b = 8 * 2 + 22 + 6 + text_breite
    lx0 = rand_l + pl_b - legende_b - 10
    ly0 = rand_o + 10

    ziel = tempfile.mktemp(suffix=".png")
    try:
        G.zeichne_plot(e, ziel, breite=breite, hoehe=hoehe)
        with Image.open(ziel) as bild:
            bild = bild.convert("RGB")
            obere_kante = sum(
                1 for x in range(lx0, lx0 + legende_b)
                if bild.getpixel((x, ly0)) == (60, 60, 60))
        assert obere_kante == legende_b, (
            f"nur {obere_kante} von {legende_b} Randpixeln gesetzt -- "
            f"Legendenkasten fehlt oder ist verschoben")
    finally:
        if os.path.exists(ziel):
            os.unlink(ziel)


def test_zeichne_plot_legende_zeigt_messwert_nicht_den_dateinamen():
    # Statt des (potenziell langen/kryptischen) Dateinamens soll die
    # Legende schlicht "Messwert" zeigen -- der Dateiname steht ohnehin
    # schon im Textbericht. Byte-Vergleich wie beim Achsentitel-Test.
    xs, ys = _fahrt(rausch_mm=0.01)
    dateiname = "ein_sehr_langer_dateiname_der_so_nicht_erscheinen_darf.csv"
    e = G.auswerten([(dateiname, xs, ys)])

    from PIL import Image, ImageDraw
    breite, hoehe = 1200, 700
    rand_l, rand_o, rand_u = 90, 50, 70
    pl_b = breite - rand_l - 20
    schrift = G._schrift(11)

    dummy = Image.new("RGB", (10, 10))
    draw = ImageDraw.Draw(dummy)
    text_breite = max(
        draw.textbbox((0, 0), t, font=schrift)[2]
        for t in ("Messwert", "1 Düsenreihe"))
    legende_b = 8 * 2 + 22 + 6 + text_breite
    lx0 = rand_l + pl_b - legende_b - 10
    ly0 = rand_o + 10
    text_pos = (lx0 + 8 + 22 + 6, ly0 + 8)

    soll = Image.new("RGB", (text_breite, 16), (255, 255, 255))
    ImageDraw.Draw(soll).text((0, 0), "Messwert", fill=(40, 40, 40),
                              font=schrift)

    ziel = tempfile.mktemp(suffix=".png")
    try:
        G.zeichne_plot(e, ziel, breite=breite, hoehe=hoehe)
        with Image.open(ziel) as bild:
            crop = bild.convert("RGB").crop(
                (text_pos[0], text_pos[1],
                 text_pos[0] + text_breite, text_pos[1] + 16))
        assert crop.tobytes() == soll.tobytes(), (
            "Legende zeigt nicht 'Messwert' an der erwarteten Stelle")
    finally:
        if os.path.exists(ziel):
            os.unlink(ziel)


def test_zeichne_plot_achsenbeschriftung_lautet_entlang_der_y_achse():
    xs, ys = _fahrt(rausch_mm=0.01)
    e = G.auswerten([("f1", xs, ys)])

    from PIL import Image, ImageDraw
    breite, hoehe = 1200, 700
    rand_l, rand_r = 90, 20
    pl_b = breite - rand_l - rand_r
    schrift = G._schrift(11)
    text = "entlang der y-Achse (mm)"
    pos = (int(rand_l + pl_b / 2 - 60), hoehe - 26)

    dummy = Image.new("RGB", (10, 10))
    bbox = ImageDraw.Draw(dummy).textbbox(pos, text, font=schrift)
    soll = Image.new("RGB", (bbox[2] - bbox[0], bbox[3] - bbox[1]),
                     (255, 255, 255))
    ImageDraw.Draw(soll).text((pos[0] - bbox[0], pos[1] - bbox[1]), text,
                              fill=G._ACHSEN, font=schrift)

    ziel = tempfile.mktemp(suffix=".png")
    try:
        G.zeichne_plot(e, ziel, breite=breite, hoehe=hoehe)
        with Image.open(ziel) as bild:
            crop = bild.convert("RGB").crop(bbox)
        assert crop.tobytes() == soll.tobytes()
    finally:
        if os.path.exists(ziel):
            os.unlink(ziel)


def test_plot_zeichnet_nichts_bei_fehlerhaftem_ergebnis():
    ziel = tempfile.mktemp(suffix=".png")
    assert G.zeichne_plot({"fehler": "x"}, ziel) is False
    assert not os.path.exists(ziel)


# ============================================================ Kommandozeile
def _cli(*argumente):
    return subprocess.run([sys.executable, WERKZEUG, *argumente],
                          capture_output=True, text=True, timeout=120)


def test_cli_laeuft_eigenstaendig_und_schreibt_das_png():
    verzeichnis = tempfile.mkdtemp()
    dateien = []
    for i in range(3):
        xs, ys = _fahrt(rausch_mm=0.02, seed=i)
        pfad = os.path.join(verzeichnis, f"fahrt{i}.jsonl")
        with open(pfad, "w", encoding="utf-8") as datei:
            for x, y in zip(xs, ys):
                datei.write(json.dumps({"event": "position", "x": x,
                                        "y": y, "z": 0.0}) + "\n")
        dateien.append(pfad)
    ziel = os.path.join(verzeichnis, "out.png")
    p = _cli(*dateien, "--png", ziel)
    assert p.returncode == 0, p.stderr
    assert "Geradheit über die Messreihe" in p.stdout
    assert "Fahrten              : 3" in p.stdout
    assert os.path.exists(ziel) and os.path.getsize(ziel) > 1000


def test_cli_warnt_bei_einer_profile_csv_ohne_x_y():
    kopf = "t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,x,y,z,qx,qy,qz,qw"
    zeilen = [kopf] + [
        _csv_zeile([f"0.{i}", 0, 0, f"{i}.0", 0.5, 10.0, 50.0,
                   None, None, None, None, None, None, None])
        for i in range(1, 20)]
    pfad = _schreib("\n".join(zeilen) + "\n", endung=".csv")
    try:
        p = _cli(pfad, "--kein-plot")
        assert p.returncode == 0, p.stderr
        assert "SEITENEBENEN-Koordinaten" in p.stdout
    finally:
        os.unlink(pfad)


def test_cli_verwendet_x_y_ohne_seitenebenen_warnung():
    kopf = "t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,x,y,z,qx,qy,qz,qw"
    zeilen = [kopf] + [
        _csv_zeile([f"0.{i}", 0, 0, 999.0, 999.0, 10.0, 50.0,
                   float(i), 0.5, 0, 0, 0, 0, 1])
        for i in range(1, 20)]
    pfad = _schreib("\n".join(zeilen) + "\n", endung=".csv")
    try:
        p = _cli(pfad, "--kein-plot")
        assert p.returncode == 0, p.stderr
        assert "SEITENEBENEN-Koordinaten" not in p.stdout
        assert "rohe Sensorkoordinaten x/y verwendet" in p.stdout
    finally:
        os.unlink(pfad)


def test_cli_meldet_fehlende_datei_ohne_traceback():
    p = _cli("/gibt/es/nicht.jsonl")
    assert p.returncode == 2
    assert "Traceback" not in p.stdout + p.stderr


def test_werkzeug_importiert_kein_printhead():
    # Eigenständigkeit ist eine Zusage: die Datei soll allein kopierbar sein.
    with open(WERKZEUG, encoding="utf-8") as datei:
        quelltext = datei.read()
    assert "import printhead" not in quelltext
    assert "from printhead" not in quelltext


def test_rechnung_braucht_kein_pil():
    # PIL wird nur zum Zeichnen gebraucht und erst dort importiert, damit der
    # Textbericht auch ohne Pillow funktioniert.
    with open(WERKZEUG, encoding="utf-8") as datei:
        kopf = datei.read().split("def zeichne_plot(")[0]
    assert "from PIL" not in kopf and "import PIL" not in kopf


# ============================================================ --slide-show
def _beispielfahrten(anzahl=2):
    """Zwei kurze, leicht verrauschte Fahrten entlang y -- genug für einen
    vollständigen Plot samt Mittelwertkurve, Streuungsband und Legende."""
    rng = random.Random(11)
    fahrten = []
    for nummer in range(anzahl):
        xs, ys = [], []
        for i in range(120):
            y = -90.0 + i * 180.0 / 119.0
            xs.append(3.0 + rng.gauss(0, 0.02))
            ys.append(y)
        fahrten.append((f"f{nummer}", xs, ys))
    return fahrten


def _text_tinte(bild, kasten):
    """Nicht-weiße Pixel in einem Ausschnitt -- ein Maß für die Textmenge.

    Über das Graustufen-Histogramm gezählt statt Pixel für Pixel: das ist
    schnell und kommt ohne das inzwischen veraltete ``getdata()`` aus."""
    x0, y0, x1, y1 = kasten
    grau = bild.crop((int(x0), int(y0), int(x1), int(y1))).convert("L")
    return sum(grau.histogram()[:250])


def _gemessener_plotrahmen(bild):
    """Der TATSÄCHLICH gezeichnete Achsenrahmen, aus den Pixeln gelesen.

    Bewusst gemessen statt aus _RAND_* nachgerechnet: eine Nachrechnung
    würde die Layoutformel des Werkzeugs im Test wiederholen und wäre damit
    blind dafür, dass ein einzelner Rand nicht mitskaliert -- die Leinwand
    stimmte, der Rahmen säße trotzdem falsch."""
    px = bild.load()
    w, h = bild.size
    zeilen = [y for y in range(h)
              if _laengster_lauf([px[x, y] for x in range(w)]) >= w // 2]
    spalten = [x for x in range(w)
               if _laengster_lauf([px[x, y] for y in range(h)]) >= h // 2]
    assert len(zeilen) >= 2 and len(spalten) >= 2, (zeilen, spalten)
    return (min(spalten), min(zeilen), max(spalten), max(zeilen))


def _laengster_lauf(pixel):
    lauf = best = 0
    for p in pixel:
        if p == (60, 60, 60):                   # G._ACHSEN
            lauf += 1
            best = max(best, lauf)
        else:
            lauf = 0
    return best


def test_slide_show_skala_laesst_die_datenflaeche_unveraendert():
    # Der Sinn der Option ist ein anderes VERHÄLTNIS von Schrift zu
    # Zeichenfläche, nicht ein insgesamt größeres Bild. Deshalb wächst die
    # Leinwand um genau den Zuwachs der Ränder: die Plotfläche muss danach
    # exakt dieselbe Pixelgröße haben wie ohne die Option. Ohne diese
    # Kompensation würde die Datenfläche mit jeder Schriftvergrößerung
    # schrumpfen -- bei Faktor 2.2 um rund ein Achtel der Breite.
    from PIL import Image
    e = G.auswerten(_beispielfahrten())
    breite, hoehe = 1200, 700
    ziele = []
    try:
        for skala in (1.0, G.SLIDE_SHOW_SKALA):
            ziel = tempfile.mktemp(suffix=".png")
            ziele.append(ziel)
            assert G.zeichne_plot(e, ziel, breite=breite, hoehe=hoehe,
                                  skala=skala) is True
        with Image.open(ziele[0]) as klein, Image.open(ziele[1]) as gross:
            assert klein.size == (breite, hoehe)
            # Leinwand größer ...
            assert gross.size[0] > klein.size[0]
            assert gross.size[1] > klein.size[1]
            # ... aber der gezeichnete Rahmen exakt gleich groß.
            x0k, y0k, x1k, y1k = _gemessener_plotrahmen(klein)
            x0g, y0g, x1g, y1g = _gemessener_plotrahmen(gross)
            # 1 Pixel Toleranz: die skalierten Ränder sind Fließkomma,
            # ihre Rasterung kann den Rahmen um einen Pixel verschieben.
            assert abs((x1g - x0g) - (x1k - x0k)) <= 1
            assert abs((y1g - y0g) - (y1k - y0k)) <= 1
            # und liegt an der Stelle, die dem skalierten Rand entspricht
            assert abs(x0g - G._RAND_L * G.SLIDE_SHOW_SKALA) <= 2, x0g
            assert abs(y0g - G._RAND_O * G.SLIDE_SHOW_SKALA) <= 2, y0g
    finally:
        for ziel in ziele:
            if os.path.exists(ziel):
                os.unlink(ziel)


def test_slide_show_vergroessert_wirklich_die_schrift():
    # Die eigentliche Zusage der Option. Gemessen an der Tintenmenge im
    # LINKEN Rand -- dort steht ausschließlich Text (y-Achsenbeschriftung
    # und Achsentitel), keine Kurve, kein Gitter. Eine Mutation, die nur die
    # Ränder skaliert und die Schriftgröße bei 11 stehen lässt, macht das
    # Bild zwar größer, füllt diesen Streifen aber mit derselben Tintenmenge
    # wie zuvor und faellt hier durch.
    from PIL import Image
    e = G.auswerten(_beispielfahrten())
    ziele = []
    try:
        for skala in (1.0, G.SLIDE_SHOW_SKALA):
            ziel = tempfile.mktemp(suffix=".png")
            ziele.append(ziel)
            assert G.zeichne_plot(e, ziel, breite=1200, hoehe=700,
                                  skala=skala) is True
        with Image.open(ziele[0]) as klein, Image.open(ziele[1]) as gross:
            tinte_klein = _text_tinte(
                klein, (0, 0, G._RAND_L, klein.size[1]))
            tinte_gross = _text_tinte(
                gross, (0, 0, G._RAND_L * G.SLIDE_SHOW_SKALA, gross.size[1]))
        # Textfläche wächst etwa quadratisch mit der Schrifthöhe; bei 2.2
        # wären das rund 4.8x. Die Schranke liegt bewusst deutlich darunter,
        # damit sie nicht an der Rasterung einzelner Glyphen hängt.
        assert tinte_klein > 0
        assert tinte_gross > tinte_klein * 2.5, (tinte_gross, tinte_klein)
    finally:
        for ziel in ziele:
            if os.path.exists(ziel):
                os.unlink(ziel)


def test_ohne_slide_show_aendert_sich_das_bild_nicht():
    # Regressionsschutz: skala=1.0 (der Default) muss Pixel für Pixel das
    # liefern, was die Funktion vor der Option geliefert hat. Die Option
    # darf nichts kosten, solange sie nicht benutzt wird.
    import hashlib
    e = G.auswerten(_beispielfahrten())
    ziele = []
    try:
        for kwargs in ({}, {"skala": 1.0}):
            ziel = tempfile.mktemp(suffix=".png")
            ziele.append(ziel)
            assert G.zeichne_plot(e, ziel, breite=900, hoehe=520,
                                  **kwargs) is True
        pruef = [hashlib.sha256(open(z, "rb").read()).hexdigest()
                 for z in ziele]
        assert pruef[0] == pruef[1]
    finally:
        for ziel in ziele:
            if os.path.exists(ziel):
                os.unlink(ziel)


def _lange_achsenlinien(bild, mindestlaenge):
    """Zeilen, die einen waagerechten Lauf der Achsenfarbe dieser Mindest-
    länge enthalten -- damit lassen sich Plotrahmen (sehr lang) und
    Legendenkasten (kürzer) im fertigen Bild wiederfinden, ohne die
    Layoutformeln aus dem Werkzeug im Test zu wiederholen."""
    px = bild.load()
    w, h = bild.size
    treffer = []
    for y in range(h):
        lauf = best = 0
        for x in range(w):
            if px[x, y] == (60, 60, 60):        # G._ACHSEN
                lauf += 1
                best = max(best, lauf)
            else:
                lauf = 0
        if best >= mindestlaenge:
            treffer.append(y)
    return treffer


def test_slide_show_skaliert_auch_das_legendenraster():
    # Die Schrift allein zu vergrößern reicht nicht: bleibt die Zeilenhöhe
    # der Legende bei ihren 18 Pixeln, überlappen sich die Einträge, sobald
    # die Schrift größer ist als der Zeilenabstand. Gemessen an der HÖHE des
    # Legendenkastens, den das Werkzeug selbst zeichnet -- der folgt direkt
    # aus Zeilenhöhe mal Anzahl Einträge.
    from PIL import Image
    e = G.auswerten(_beispielfahrten())
    hoehen = {}
    ziele = []
    try:
        for skala in (1.0, G.SLIDE_SHOW_SKALA):
            ziel = tempfile.mktemp(suffix=".png")
            ziele.append(ziel)
            assert G.zeichne_plot(e, ziel, breite=1200, hoehe=700,
                                  skala=skala) is True
            with Image.open(ziel) as bild:
                rahmen = _lange_achsenlinien(bild, 600)
                kasten = [y for y in _lange_achsenlinien(bild, 60)
                          if y not in rahmen]
                assert len(kasten) >= 2, kasten
                hoehen[skala] = max(kasten) - min(kasten)
        verhaeltnis = hoehen[G.SLIDE_SHOW_SKALA] / hoehen[1.0]
        assert abs(verhaeltnis - G.SLIDE_SHOW_SKALA) < 0.25, (hoehen,
                                                              verhaeltnis)
    finally:
        for ziel in ziele:
            if os.path.exists(ziel):
                os.unlink(ziel)


def test_slide_show_haelt_den_achsentitel_von_der_beschriftung_frei():
    # Der y-Achsentitel steht bei rand_o - 22 über dem Plotrahmen, die
    # oberste Achsenbeschriftung bei rand_o - 6. Skaliert der Titelabstand
    # nicht mit, rutscht der Titel bei größerer Schrift in die Beschriftung
    # hinein. Geprüft wird deshalb nicht seine Position, sondern die Folge:
    # zwischen beiden muss im linken Rand mindestens eine komplett leere
    # Bildzeile bleiben.
    from PIL import Image
    e = G.auswerten(_beispielfahrten())
    ziel = tempfile.mktemp(suffix=".png")
    try:
        assert G.zeichne_plot(e, ziel, breite=1200, hoehe=700,
                              skala=G.SLIDE_SHOW_SKALA) is True
        with Image.open(ziel) as bild:
            px = bild.load()
            rand_l = int(G._RAND_L * G.SLIDE_SHOW_SKALA)
            rahmen_oben = _lange_achsenlinien(bild, 600)[0]
            spalten = range(0, rand_l)
            tinte = [y for y in range(rahmen_oben + 1)
                     if any(px[x, y] != (255, 255, 255) for x in spalten)]
            assert tinte, "kein Achsentitel im linken Rand gefunden"
            leer = [y for y in range(min(tinte), rahmen_oben)
                    if all(px[x, y] == (255, 255, 255) for x in spalten)]
            assert leer, ("Achsentitel und Achsenbeschriftung berühren sich "
                          "-- der Titelabstand skaliert nicht mit")
    finally:
        if os.path.exists(ziel):
            os.unlink(ziel)


def _linien_dicke(bild, farbe, schrittweite=7):
    """Groesster senkrechter Lauf dieser Farbe -- die Strichstaerke einer
    WAAGERECHTEN Linie, direkt am fertigen Bild gemessen. Fuer die Nulllinie
    (120,120,120) eindeutig: keine andere Farbe im Plot ist dieselbe."""
    px = bild.load()
    w, h = bild.size
    best = 0
    for x in range(0, w, schrittweite):
        lauf = 0
        for y in range(h):
            if px[x, y] == farbe:
                lauf += 1
                best = max(best, lauf)
            else:
                lauf = 0
    return best


def test_loese_skala_uebersetzt_das_flag():
    # None = Flag gar nicht angegeben -> aus. argparse setzt ohne Zahl den
    # const-Wert ein, mit Zahl genau diese.
    assert G.loese_skala(None) == 1.0
    assert G.loese_skala(G.SLIDE_SHOW_SKALA) == G.SLIDE_SHOW_SKALA
    assert G.loese_skala(3) == 3.0
    assert G.loese_skala(0.5) == 0.5


def test_loese_skala_lehnt_null_und_negativ_ab():
    # Nicht kosmetisch: 0 macht jede Schrift und jede Linie unsichtbar,
    # negativ dreht saemtliche Raender nach innen. Beides wuerde ein Bild
    # erzeugen statt zu scheitern -- der schlechteste Ausgang.
    for schlecht in (0, 0.0, -1, -2.5):
        try:
            G.loese_skala(schlecht)
        except ValueError:
            continue
        raise AssertionError(f"{schlecht!r} haette abgelehnt werden muessen")


def test_slide_show_verdickt_auch_die_linien():
    # Eine 1-Pixel-Kurve neben 24-Punkt-Schrift verschwindet auf einer
    # Projektionsflaeche -- das groessere Bild waere sonst schlechter lesbar
    # als das kleine. Gemessen an der Nulllinie (width=2 bei Skala 1), deren
    # Farbe im Plot sonst nirgends vorkommt.
    from PIL import Image
    e = G.auswerten(_beispielfahrten())
    dicken = {}
    ziele = []
    try:
        for skala in (1.0, G.SLIDE_SHOW_SKALA, 3.0):
            ziel = tempfile.mktemp(suffix=".png")
            ziele.append(ziel)
            assert G.zeichne_plot(e, ziel, breite=1200, hoehe=700,
                                  skala=skala) is True
            with Image.open(ziel) as bild:
                dicken[skala] = _linien_dicke(bild, (120, 120, 120))
        assert dicken[1.0] == 2, dicken
        assert dicken[G.SLIDE_SHOW_SKALA] == round(2 * G.SLIDE_SHOW_SKALA), dicken
        assert dicken[3.0] == 6, dicken
    finally:
        for ziel in ziele:
            if os.path.exists(ziel):
                os.unlink(ziel)


def test_slide_show_verdickt_auch_die_kurven_nicht_nur_die_nulllinie():
    # Die Nulllinie allein zu messen reicht nicht: sie ist EINE von fuenf
    # Stellen mit einer Strichstaerke, und eine vergessene davon faellt sonst
    # nicht auf. Hier die Mittelwertkurve (width=3), gezaehlt in Pixeln ihrer
    # eigenen Farbe -- die Datenflaeche bleibt gleich gross, der Kurvenweg
    # also gleich lang, ein Zuwachs kann nur aus der Strichstaerke kommen.
    from PIL import Image
    e = G.auswerten(_beispielfahrten(anzahl=3))
    pixel = {}
    ziele = []
    try:
        for skala in (1.0, G.SLIDE_SHOW_SKALA):
            ziel = tempfile.mktemp(suffix=".png")
            ziele.append(ziel)
            assert G.zeichne_plot(e, ziel, breite=1200, hoehe=700,
                                  skala=skala) is True
            with Image.open(ziel) as bild:
                pixel[skala] = _text_tinte_farbe(bild, G._MITTEL_FARBE)
        assert pixel[1.0] > 0
        # width 3 -> round(3 * 2.2) = 7. Schranke 1.8 statt 7/3, damit sie
        # nicht an der Rasterung einzelner Kurvenstuecke haengt; eine nicht
        # mitskalierte Kurve laege bei ~1.0.
        verhaeltnis = pixel[G.SLIDE_SHOW_SKALA] / pixel[1.0]
        assert verhaeltnis > 1.8, (pixel, verhaeltnis)
    finally:
        for ziel in ziele:
            if os.path.exists(ziel):
                os.unlink(ziel)


def _text_tinte_farbe(bild, farbe):
    """Wie viele Pixel genau diese Farbe tragen."""
    px = bild.load()
    w, h = bild.size
    return sum(1 for y in range(h) for x in range(w) if px[x, y] == farbe)


def test_slide_show_faktor_ist_frei_waehlbar():
    # Der Faktor muss wirklich durchschlagen und nicht auf den Default
    # zurueckfallen: ein groesserer Faktor heisst groessere Leinwand UND
    # dickere Linien.
    from PIL import Image
    e = G.auswerten(_beispielfahrten())
    ziele = []
    try:
        werte = {}
        for skala in (2.0, 4.0):
            ziel = tempfile.mktemp(suffix=".png")
            ziele.append(ziel)
            assert G.zeichne_plot(e, ziel, breite=1200, hoehe=700,
                                  skala=skala) is True
            with Image.open(ziel) as bild:
                werte[skala] = (bild.size[0],
                                _linien_dicke(bild, (120, 120, 120)))
        assert werte[4.0][0] > werte[2.0][0], werte      # Leinwand
        assert werte[4.0][1] > werte[2.0][1], werte      # Strichstaerke
    finally:
        for ziel in ziele:
            if os.path.exists(ziel):
                os.unlink(ziel)


def test_cli_slide_show_nimmt_einen_faktor_entgegen():
    from PIL import Image
    verzeichnis = tempfile.mkdtemp()
    rng = random.Random(5)
    pfad = os.path.join(verzeichnis, "fahrt.jsonl")
    with open(pfad, "w", encoding="utf-8") as datei:
        for i in range(150):
            datei.write(json.dumps({
                "event": "position",
                "x": 3.0 + rng.gauss(0, 0.02),
                "y": -90.0 + i * 180.0 / 149.0}) + "\n")

    groessen = {}
    for marke, zusatz in (("default", ["--slide-show"]),
                          ("faktor4", ["--slide-show", "4"])):
        ziel = os.path.join(verzeichnis, f"{marke}.png")
        p = _cli(pfad, "--png", ziel, *zusatz)
        assert p.returncode == 0, p.stderr
        with Image.open(ziel) as bild:
            groessen[marke] = bild.size
    assert groessen["faktor4"][0] > groessen["default"][0], groessen


def test_cli_slide_show_lehnt_einen_faktor_von_null_ab():
    verzeichnis = tempfile.mkdtemp()
    pfad = os.path.join(verzeichnis, "fahrt.jsonl")
    with open(pfad, "w", encoding="utf-8") as datei:
        for i in range(50):
            datei.write(json.dumps({"event": "position", "x": 3.0,
                                    "y": float(i)}) + "\n")
    for schlecht in ("0", "-2"):
        p = _cli(pfad, "--slide-show", schlecht, "--kein-plot")
        assert p.returncode != 0, f"--slide-show {schlecht} wurde angenommen"
        assert "slide-show" in (p.stderr + p.stdout)


def test_cli_slide_show_schreibt_ein_groesseres_bild():
    from PIL import Image
    verzeichnis = tempfile.mkdtemp()
    rng = random.Random(5)
    pfad = os.path.join(verzeichnis, "fahrt.jsonl")
    with open(pfad, "w", encoding="utf-8") as datei:
        for i in range(150):
            datei.write(json.dumps({
                "event": "position",
                "x": 3.0 + rng.gauss(0, 0.02),
                "y": -90.0 + i * 180.0 / 149.0}) + "\n")

    groessen = {}
    for marke, zusatz in (("normal", []), ("slide", ["--slide-show"])):
        ziel = os.path.join(verzeichnis, f"{marke}.png")
        p = _cli(pfad, "--png", ziel, *zusatz)
        assert p.returncode == 0, p.stderr
        with Image.open(ziel) as bild:
            groessen[marke] = bild.size
    assert groessen["slide"][0] > groessen["normal"][0]
    assert groessen["slide"][1] > groessen["normal"][1]


if __name__ == "__main__":
    tests = [v for k, v in list(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"OK: {t.__name__}")
    print(f"Alle {len(tests)} Geradheits-Tests bestanden.")
