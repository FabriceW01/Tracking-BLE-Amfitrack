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


def test_profile_csv_liest_u_und_v():
    pfad = _schreib(
        "t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,x,y,z,qx,qy,qz,qw\n"
        "0.1,0,0,1.000,2.000,10.00,50.0,0,0,0,1\n"
        "0.2,0,1,3.000,4.000,10.00,50.0,0,0,0,1\n", endung=".csv")
    try:
        xs, ys = G.lies_profile_csv(pfad)
        assert xs == [1.0, 3.0] and ys == [2.0, 4.0]
    finally:
        os.unlink(pfad)


def test_profile_csv_weist_line_modus_mit_begruendung_ab():
    pfad = _schreib("t_s,column,advance_mm,write_latency_ms,speed_mm_s,x,y,z\n"
                    "0.1,0,0.0,3.1,10.0\n", endung=".csv")
    try:
        G.lies_profile_csv(pfad)
    except ValueError as fehler:
        assert "u_mm/v_mm" in str(fehler)
        return
    finally:
        os.unlink(pfad)
    raise AssertionError("erwartet: ValueError für eine Line-Modus-CSV")


def test_lies_messreihe_erkennt_die_quelle():
    j = _schreib('{"event":"position","x":1.0,"y":2.0}\n')
    c = _schreib("t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,x,y,z,qx,qy,qz,qw\n"
                 "0.1,0,0,1.0,2.0,10.0,50.0,0,0,0,1\n", endung=".csv")
    try:
        assert G.lies_messreihe(j)[2] == "pos-json"
        assert G.lies_messreihe(c)[2] == "profile-csv"
    finally:
        os.unlink(j)
        os.unlink(c)


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


def test_cli_warnt_bei_einer_profile_csv():
    pfad = _schreib("t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,x,y,z,qx,qy,qz,qw\n"
                    + "".join(f"0.{i},0,0,{i}.0,0.5,10.0,50.0,0,0,0,1\n"
                              for i in range(1, 20)), endung=".csv")
    try:
        p = _cli(pfad, "--kein-plot")
        assert p.returncode == 0, p.stderr
        assert "SEITENEBENEN-Koordinaten" in p.stdout
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


if __name__ == "__main__":
    tests = [v for k, v in list(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"OK: {t.__name__}")
    print(f"Alle {len(tests)} Geradheits-Tests bestanden.")
