"""
Tests für funktionen/rauschen_entfernung.py und
funktionen/geschwindigkeit_profil.py (keine Hardware, keine GUI).

Beide Werkzeuge sind eigenständig. rauschen_entfernung hält die Düsenteilung
selbst vor -- der DRIFT-Test dagegen ist der wichtigste Test hier, denn ein
weggelaufener Wert würde Abweichungen in "Düsenreihen" umrechnen, die es an
dieser Anlage nicht gibt.

Bevorzugt werden Fälle mit bekannter analytischer Antwort: ein konstruiertes
Rauschen, eine konstruierte Grenze und eine von Hand nachrechenbare
Interpolation müssen als genau diese Werte zurückkommen.

Aufruf:  python tests/test_rauschen_und_tempo.py
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

import geschwindigkeit_profil as T                           # noqa: E402
import rauschen_entfernung as R                              # noqa: E402
from printhead.controller import DEFAULT_SPEED_WARNING_MM_S  # noqa: E402
from printhead.geometry import NOZZLE_PITCH_MM               # noqa: E402

FUNK = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    "funktionen")
W_RAUSCH = os.path.join(FUNK, "rauschen_entfernung.py")
W_TEMPO = os.path.join(FUNK, "geschwindigkeit_profil.py")


# ================================================ Drift gegen die Anlage
def test_duesenteilung_stimmt_mit_der_geometrie_ueberein():
    assert abs(R.DUESENTEILUNG_MM - NOZZLE_PITCH_MM) < 1e-12


def test_warnschwelle_stimmt_mit_dem_controller_ueberein():
    # Der Plot zeichnet diese Linie als Maßstab ein -- läuft sie vom echten
    # Default weg, liest man die Kurve gegen eine Schwelle, die es nicht gibt.
    assert abs(T.WARNSCHWELLE_MM_S - DEFAULT_SPEED_WARNING_MM_S) < 1e-12


# =========================================================== Rauschen: Statistik
def test_stdabw_gegen_handrechnung():
    # Für [1,2,3,4] ist die Stichproben-Standardabweichung sqrt(5/3).
    assert abs(R._stdabw([1.0, 2.0, 3.0, 4.0]) - math.sqrt(5.0 / 3.0)) < 1e-12
    assert R._stdabw([]) == 0.0
    assert R._stdabw([7.0]) == 0.0


def test_konstruiertes_rauschen_kommt_zurueck():
    rng = random.Random(11)
    sigma = 0.05
    xs = [rng.gauss(0.0, sigma) for _ in range(4000)]
    assert abs(R._stdabw(xs) - sigma) < 0.003


def test_rms3d_fasst_die_drei_achsen_zusammen():
    # Drei Achsen mit je sigma -> 3D-RMS ist sqrt(3)*sigma.
    rng = random.Random(5)
    sigma = 0.04
    n = 6000
    xs = [rng.gauss(0, sigma) for _ in range(n)]
    ys = [rng.gauss(0, sigma) for _ in range(n)]
    zs = [rng.gauss(0, sigma) for _ in range(n)]
    w = R.werte_einer_datei(xs, ys, zs, 15)
    assert abs(w["rms3d"] - math.sqrt(3) * sigma) < 0.004


def test_fenster_streuung_trennt_drift_von_rauschen():
    # Gleiche Gesamtstreuung, einmal als Rauschen, einmal als reine Drift.
    # Die Fensterstreuung muss nur im ersten Fall groß sein -- sonst sähe
    # ein langsam weglaufender Sensor wie ein rauschender aus.
    rng = random.Random(3)
    rauschen = [rng.gauss(0.0, 0.05) for _ in range(600)]
    drift = [0.6 * i / 599.0 for i in range(600)]
    assert abs(R.fenster_streuung(rauschen, 15) - R._stdabw(rauschen)) < 0.01
    assert R.fenster_streuung(drift, 15) < R._stdabw(drift) / 5.0


def test_fenster_streuung_mittelt_varianzen_nicht_standardabweichungen():
    # Bei gleichmäßigem Rauschen liefern beide Formeln fast dasselbe -- der
    # Unterschied zeigt sich erst, wenn die Fenster UNTERSCHIEDLICH stark
    # rauschen (Jensen: Mittel der Sigmas < Wurzel aus dem Mittel der
    # Varianzen). Ohne diesen Fall bliebe ein Austausch der Formel unbemerkt;
    # genau das ist beim Mutationstest zunächst passiert.
    #
    # Zwei Fenster à 100 Punkte, eines fast ruhig, eines laut:
    #   Mittel der Sigmas         = (0.01 + 0.50) / 2      = 0.255   (falsch)
    #   Wurzel(Mittel der Var.)   = sqrt((0.0001+0.25)/2)  = 0.3536  (richtig)
    rng = random.Random(21)
    leise = [rng.gauss(0.0, 0.01) for _ in range(100)]
    laut = [rng.gauss(0.0, 0.50) for _ in range(100)]
    got = R.fenster_streuung(leise + laut, 100)

    richtig = math.sqrt((R._stdabw(leise) ** 2 + R._stdabw(laut) ** 2) / 2.0)
    falsch = (R._stdabw(leise) + R._stdabw(laut)) / 2.0
    assert abs(got - richtig) < 1e-12, (got, richtig)
    # Und die beiden liegen wirklich weit auseinander -- sonst würde der Test
    # nichts absichern.
    assert abs(richtig - falsch) > 0.05, (richtig, falsch)


def test_fenster_streuung_faellt_auf_gesamt_zurueck_wenn_zu_wenig_punkte():
    werte = [1.0, 2.0, 3.0]
    assert abs(R.fenster_streuung(werte, 50) - R._stdabw(werte)) < 1e-12


# ========================================================= Rauschen: Grenzwert
def _punkt(abstand, rms):
    return {"abstand": abstand, "rms3d": rms}


def test_grenzabstand_interpoliert_von_hand_nachrechenbar():
    # Schwelle 0.10 liegt zwischen 20cm (0.08) und 30cm (0.12):
    #   20 + (0.10-0.08)/(0.12-0.08) * 10 = 25.0
    punkte = [_punkt(10, 0.04), _punkt(20, 0.08), _punkt(30, 0.12)]
    grenze, art = R.grenzabstand(punkte, schwelle=0.10)
    assert art == "interpoliert"
    assert abs(grenze - 25.0) < 1e-12


def test_grenzabstand_meldet_wenn_schon_der_naechste_punkt_darueber_liegt():
    punkte = [_punkt(10, 0.5), _punkt(20, 0.8)]
    grenze, art = R.grenzabstand(punkte, schwelle=0.1)
    assert art == "unterhalb" and grenze == 10


def test_grenzabstand_meldet_wenn_keiner_die_schwelle_erreicht():
    punkte = [_punkt(10, 0.01), _punkt(20, 0.02)]
    grenze, art = R.grenzabstand(punkte, schwelle=0.1)
    assert art == "oberhalb" and grenze is None


def test_grenzabstand_sortiert_unsortierte_eingabe():
    punkte = [_punkt(30, 0.12), _punkt(10, 0.04), _punkt(20, 0.08)]
    grenze, art = R.grenzabstand(punkte, schwelle=0.10)
    assert art == "interpoliert" and abs(grenze - 25.0) < 1e-12


# ========================================================= Rauschen: Einlesen
def _schreib(text, endung=".jsonl"):
    fh = tempfile.NamedTemporaryFile("w", suffix=endung, delete=False,
                                     encoding="utf-8")
    fh.write(text)
    fh.close()
    return fh.name


def test_pos_json_braucht_alle_drei_achsen():
    pfad = _schreib('{"event":"connected"}\n'
                    '{"event":"position","x":1.0,"y":2.0}\n'      # kein z
                    '{"event":"position","x":1.0,"y":2.0,"z":3.0}\n'
                    'kaputt\n'
                    '{"event":"position","x":4.0,"y":5.0,"z":6.0}\n')
    try:
        xs, ys, zs = R.lies_pos_json(pfad)
        assert xs == [1.0, 4.0] and ys == [2.0, 5.0] and zs == [3.0, 6.0]
    finally:
        os.unlink(pfad)


def test_abstand_aus_dateiname():
    assert R.abstand_aus_name("rausch_d20.jsonl") == 20.0
    assert R.abstand_aus_name("/pfad/zu/30cm.jsonl") == 30.0
    assert R.abstand_aus_name("12.5cm.jsonl") == 12.5
    # Keine Zahl -> None, damit der Aufrufer nachfragt statt zu raten.
    assert R.abstand_aus_name("messung.jsonl") is None


# ==================================================== Rauschen: Gesamtauswertung
def _messung(abstand, sigma, n=400, seed=1, drift=0.0):
    rng = random.Random(seed)
    xs, ys, zs = [], [], []
    for i in range(n):
        t = i / (n - 1)
        xs.append(rng.gauss(0, sigma) + drift * t)
        ys.append(rng.gauss(0, sigma))
        zs.append(rng.gauss(0, sigma))
    return (abstand, f"d{abstand}", xs, ys, zs)


def test_auswerten_findet_die_konstruierte_grenze():
    # Rauschen so gewählt, dass der 3D-RMS bei 30cm genau eine Düsenreihe ist.
    je_achse = R.DUESENTEILUNG_MM / math.sqrt(3)
    messungen = [_messung(10, je_achse * 0.25, seed=1),
                 _messung(20, je_achse * 0.55, seed=2),
                 _messung(30, je_achse, seed=3),
                 _messung(40, je_achse * 1.8, seed=4)]
    e = R.auswerten(messungen)
    assert e["grenzart"] == "interpoliert"
    assert 27.0 < e["grenzabstand"] < 33.0, e["grenzabstand"]


def test_auswerten_meldet_drift_im_bericht():
    je_achse = 0.005
    messungen = [_messung(10, je_achse, seed=1),
                 _messung(20, je_achse, seed=2, drift=0.6)]
    text = R.bericht(R.auswerten(messungen))
    assert "DRIFT bei 20 cm" in text


def test_auswerten_meldet_keine_drift_bei_reinem_rauschen():
    messungen = [_messung(10, 0.05, seed=1), _messung(20, 0.05, seed=2)]
    text = R.bericht(R.auswerten(messungen))
    assert "DRIFT" not in text


def test_auswerten_meldet_fehler_statt_zu_raten():
    assert "fehler" in R.auswerten([])
    assert "fehler" in R.auswerten([(10, "leer", [], [], [])])


def test_massstab_rechnet_prozent_richtig():
    e = R.massstab_auswerten([(10, 99.0), (20, 97.5)], 100.0)
    assert abs(e["zeilen"][0]["fehler_mm"] + 1.0) < 1e-12
    assert abs(e["zeilen"][0]["fehler_prozent"] + 1.0) < 1e-12
    assert abs(e["zeilen"][1]["fehler_prozent"] + 2.5) < 1e-12


def test_massstab_weist_unsinnige_referenz_ab():
    assert "fehler" in R.massstab_auswerten([(10, 99.0)], 0.0)


def test_rausch_plot_schreibt_ein_lesbares_png():
    from PIL import Image
    e = R.auswerten([_messung(10, 0.01, seed=1), _messung(20, 0.05, seed=2),
                     _messung(30, 0.12, seed=3)])
    ziel = tempfile.mktemp(suffix=".png")
    try:
        assert R.zeichne_plot(e, ziel, breite=800, hoehe=500) is True
        with Image.open(ziel) as bild:
            assert bild.size == (800, 500) and bild.mode == "RGB"
            assert len(bild.getcolors(maxcolors=100000)) > 5
    finally:
        if os.path.exists(ziel):
            os.unlink(ziel)


# ===================================================== Tempo: Einlesen
def _csv_page(paare):
    kopf = "t_s,row,col,u_mm,v_mm,speed_mm_s,writes_per_s,qx,qy,qz,qw\n"
    zeilen = "".join(f"{i * 0.01:.4f},0,0,{u:.3f},0.5,{v:.2f},50.0,0,0,0,1\n"
                     for i, (u, v) in enumerate(paare))
    return _schreib(kopf + zeilen, endung=".csv")


def test_tempo_liest_seiten_modus():
    pfad = _csv_page([(0.0, 10.0), (1.0, 12.0)])
    try:
        positionen, tempi, modus = T.lies_profil_csv(pfad)
        assert modus == "page"
        assert positionen == [0.0, 1.0] and tempi == [10.0, 12.0]
    finally:
        os.unlink(pfad)


def test_tempo_liest_line_modus_ueber_advance_mm():
    pfad = _schreib("t_s,column,advance_mm,write_latency_ms,speed_mm_s\n"
                    "0.1,0,0.000,3.1,10.00\n"
                    "0.2,1,0.200,3.2,11.00\n", endung=".csv")
    try:
        positionen, tempi, modus = T.lies_profil_csv(pfad)
        assert modus == "line"
        assert positionen == [0.0, 0.2] and tempi == [10.0, 11.0]
    finally:
        os.unlink(pfad)


def test_tempo_weist_csv_ohne_speed_ab():
    pfad = _schreib("a,b\n1,2\n", endung=".csv")
    try:
        T.lies_profil_csv(pfad)
    except ValueError as fehler:
        assert "speed_mm_s" in str(fehler)
        return
    finally:
        os.unlink(pfad)
    raise AssertionError("erwartet: ValueError ohne speed_mm_s")


def test_tempo_weist_csv_ohne_position_ab():
    pfad = _schreib("t_s,speed_mm_s\n0.1,10.0\n", endung=".csv")
    try:
        T.lies_profil_csv(pfad)
    except ValueError as fehler:
        assert "u_mm" in str(fehler) and "advance_mm" in str(fehler)
        return
    finally:
        os.unlink(pfad)
    raise AssertionError("erwartet: ValueError ohne Positionsspalte")


# ===================================================== Tempo: Rechnung
def test_geschwindigkeit_bei_mittelt_ueber_das_fenster():
    # Ein einzelner Messwert wäre zufälliger als die gesuchte Größe, weil die
    # Geschwindigkeit selbst eine verrauschte Differenz ist.
    positionen = [10.0, 10.4, 10.8, 11.2, 20.0]
    tempi = [10.0, 12.0, 14.0, 16.0, 99.0]
    t = T.geschwindigkeit_bei(positionen, tempi, 10.6, fenster_mm=1.0)
    assert abs(t["mittel"] - 13.0) < 1e-12       # (10+12+14+16)/4
    assert t["anzahl"] == 4 and t["abstand_mm"] == 0.0


def test_geschwindigkeit_bei_nimmt_den_naechsten_punkt_wenn_das_fenster_leer_ist():
    t = T.geschwindigkeit_bei([0.0, 50.0], [10.0, 20.0], 45.0, fenster_mm=1.0)
    assert t["mittel"] == 20.0
    assert abs(t["abstand_mm"] - 5.0) < 1e-12    # Distanz wird gemeldet


def test_geschwindigkeit_bei_ohne_daten_ist_none():
    assert T.geschwindigkeit_bei([], [], 10.0) is None


def test_deckungsgrenze_von_hand_nachrechenbar():
    # Schwelle 95 liegt zwischen 18mm/s (96%) und 26mm/s (73%):
    #   18 + (96-95)/(96-73) * (26-18) = 18.3478...
    laeufe = [{"v_mittel": 10.0, "deckung": 99.0},
              {"v_mittel": 18.0, "deckung": 96.0},
              {"v_mittel": 26.0, "deckung": 73.0},
              {"v_mittel": 36.0, "deckung": 44.0}]
    grenze, art = T.deckung_gegen_tempo(laeufe, schwelle=95.0)
    assert art == "interpoliert"
    assert abs(grenze - (18.0 + (96 - 95) / (96 - 73) * 8.0)) < 1e-12


def test_deckungsgrenze_meldet_beide_randfaelle():
    zu_langsam = [{"v_mittel": 10.0, "deckung": 50.0}]
    assert T.deckung_gegen_tempo(zu_langsam)[1] == "unterhalb"
    zu_schnell = [{"v_mittel": 10.0, "deckung": 99.0},
                  {"v_mittel": 40.0, "deckung": 98.0}]
    assert T.deckung_gegen_tempo(zu_schnell)[1] == "oberhalb"


def test_kennzahlen_liefern_median_und_spanne():
    k = T.profil_kennzahlen([0.0, 1.0, 2.0, 3.0], [10.0, 20.0, 30.0, 40.0])
    assert k["u_min"] == 0.0 and k["u_max"] == 3.0
    assert k["v_mittel"] == 25.0 and k["v_median"] == 25.0
    assert k["v_max"] == 40.0


def test_tempo_plot_schreibt_ein_lesbares_png():
    from PIL import Image
    laeufe = [{"name": "a.csv", "modus": "page",
               "positionen": [i * 1.0 for i in range(50)],
               "tempi": [5.0 + i * 0.8 for i in range(50)],
               "kennzahlen": T.profil_kennzahlen(
                   [i * 1.0 for i in range(50)],
                   [5.0 + i * 0.8 for i in range(50)]),
               "deckung": 88.0}]
    ziel = tempfile.mktemp(suffix=".png")
    try:
        assert T.zeichne_plot(laeufe, ziel, bei_u=20.0, breite=800,
                              hoehe=500) is True
        with Image.open(ziel) as bild:
            assert bild.size == (800, 500) and bild.mode == "RGB"
    finally:
        if os.path.exists(ziel):
            os.unlink(ziel)


# ============================================================ Kommandozeile
def _lauf(werkzeug, *argumente):
    return subprocess.run([sys.executable, werkzeug, *argumente],
                          capture_output=True, text=True, timeout=120)


def test_cli_rauschen_laeuft_eigenstaendig():
    verzeichnis = tempfile.mkdtemp()
    dateien = []
    for abstand, sigma in ((10, 0.01), (20, 0.05), (30, 0.15)):
        pfad = os.path.join(verzeichnis, f"rausch_d{abstand}.jsonl")
        rng = random.Random(abstand)
        with open(pfad, "w", encoding="utf-8") as datei:
            for _ in range(300):
                datei.write(json.dumps({
                    "event": "position", "x": rng.gauss(0, sigma),
                    "y": rng.gauss(0, sigma), "z": rng.gauss(0, sigma)}) + "\n")
        dateien.append(pfad)
    ziel = os.path.join(verzeichnis, "out.png")
    p = _lauf(W_RAUSCH, *dateien, "--png", ziel)
    assert p.returncode == 0, p.stderr
    assert "Sensorrauschen über die Entfernung" in p.stdout
    assert os.path.exists(ziel) and os.path.getsize(ziel) > 1000


def test_cli_rauschen_meldet_fehlenden_abstand():
    pfad = _schreib('{"event":"position","x":1,"y":2,"z":3}\n'
                    '{"event":"position","x":1.1,"y":2.1,"z":3.1}\n')
    ohne_zahl = os.path.join(tempfile.mkdtemp(), "messung.jsonl")
    os.rename(pfad, ohne_zahl)
    p = _lauf(W_RAUSCH, ohne_zahl, "--kein-plot")
    assert "kein Abstand im Dateinamen" in p.stdout
    assert "Traceback" not in p.stdout + p.stderr


def test_cli_rauschen_prueft_die_anzahl_der_abstaende():
    pfad = _schreib('{"event":"position","x":1,"y":2,"z":3}\n'
                    '{"event":"position","x":1.1,"y":2.1,"z":3.1}\n')
    try:
        p = _lauf(W_RAUSCH, pfad, "--abstaende", "10,20", "--kein-plot")
        assert p.returncode == 2
        assert "2 Werte" in p.stdout and "1 Dateien" in p.stdout
    finally:
        os.unlink(pfad)


def test_cli_tempo_gibt_die_geschwindigkeit_an_der_stelle_aus():
    # v = 5 + 40*(u/200): bei u=100 also genau 25 mm/s.
    paare = []
    u = 0.0
    while u < 200:
        v = 5.0 + 40.0 * (u / 200.0)
        paare.append((u, v))
        u += v * 0.01
    pfad = _csv_page(paare)
    try:
        p = _lauf(W_TEMPO, pfad, "--bei-u", "100", "--kein-plot")
        assert p.returncode == 0, p.stderr
        assert "25.0 mm/s" in p.stdout, p.stdout
    finally:
        os.unlink(pfad)


def test_cli_tempo_prueft_die_anzahl_der_deckungswerte():
    pfad = _csv_page([(0.0, 10.0), (1.0, 10.0)])
    try:
        p = _lauf(W_TEMPO, pfad, "--deckung", "99,88", "--kein-plot")
        assert p.returncode == 2
        assert "2 Werte" in p.stdout
    finally:
        os.unlink(pfad)


def test_cli_tempo_meldet_fehlende_datei_ohne_traceback():
    p = _lauf(W_TEMPO, "/gibt/es/nicht.csv")
    assert p.returncode == 2
    assert "Traceback" not in p.stdout + p.stderr


def test_werkzeuge_importieren_kein_printhead():
    for werkzeug in (W_RAUSCH, W_TEMPO):
        with open(werkzeug, encoding="utf-8") as datei:
            quelltext = datei.read()
        assert "import printhead" not in quelltext, werkzeug
        assert "from printhead" not in quelltext, werkzeug


def test_rechnung_braucht_kein_pil():
    for werkzeug, marke in ((W_RAUSCH, "def zeichne_plot("),
                            (W_TEMPO, "def zeichne_plot(")):
        with open(werkzeug, encoding="utf-8") as datei:
            kopf = datei.read().split(marke)[0]
        assert "from PIL" not in kopf and "import PIL" not in kopf, werkzeug


if __name__ == "__main__":
    tests = [v for k, v in list(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"OK: {t.__name__}")
    print(f"Alle {len(tests)} Rausch-/Tempo-Tests bestanden.")
