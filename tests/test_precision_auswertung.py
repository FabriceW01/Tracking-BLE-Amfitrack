"""
Tests für funktionen/precision_check_auswertung.py (keine Hardware, keine GUI).

Das Werkzeug ist bewusst eigenständig und rechnet das Muster-Layout selbst
nach, statt printhead.patterns zu importieren. Der wichtigste Test hier ist
deshalb der DRIFT-Test: die eigenständige Berechnung muss Spalte für Spalte
mit printhead.patterns.precision_check_layout übereinstimmen. Läuft eine der
beiden Seiten weg, wertet das Werkzeug gegen ein Muster aus, das so nie
gedruckt wurde -- und jedes Ergebnis wäre stillschweigend falsch.

Die übrigen Tests bevorzugen Fälle mit bekannter analytischer Antwort: ein
konstruierter Maßstabsfehler muss exakt zurückkommen, eine perfekte Messung
muss Faktor 1 und Residuum 0 ergeben.

Die grafische Oberfläche wird hier NICHT getestet -- sie braucht tkinter und
einen Bildschirm. Genau dafür liegt die gesamte Rechnung in reinen
Funktionen, die ohne beides auskommen.

Aufruf:  python tests/test_precision_auswertung.py
"""

import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "funktionen"))

import precision_check_auswertung as A                       # noqa: E402
from printhead import patterns                               # noqa: E402

WERKZEUG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "funktionen", "precision_check_auswertung.py")


# ====================================================== Drift gegen das Muster
def test_layout_stimmt_mit_dem_echten_muster_ueberein():
    # DER zentrale Test dieser Datei: das eigenständig nachgerechnete Layout
    # muss dem entsprechen, was --pattern precision-check wirklich druckt.
    for line_cols in (1, 2, 3):
        for gap_start in (1, 2, 4):
            echt = patterns.precision_check_layout(5000, line_cols, gap_start)
            eigen = A.soll_layout(len(echt), line_cols, gap_start)
            assert len(eigen) == len(echt), (line_cols, gap_start)
            for e, m in zip(eigen, echt):
                assert e["start_spalte"] == m["start"], (line_cols, gap_start, e)
                assert e["luecke_davor_spalten"] == m["gap_before"], e


def test_soll_abstaende_sind_start_spalte_mal_spaltenbreite():
    soll = A.soll_abstaende_mm(6, 0.087, line_cols=1, gap_start=1)
    layout = A.soll_layout(6, 1, 1)
    for wert, linie in zip(soll, layout):
        assert abs(wert - linie["start_spalte"] * 0.087) < 1e-12
    assert soll[0] == 0.0                    # Linie 0 ist der Bezugspunkt


def test_luecken_verdoppeln_sich_ab_gap_start():
    for gap_start, erwartet in ((1, [0, 1, 2, 4, 8]),
                                (2, [0, 2, 4, 8, 16]),
                                (4, [0, 4, 8, 16, 32])):
        layout = A.soll_layout(5, 1, gap_start)
        assert [l["luecke_davor_spalten"] for l in layout] == erwartet


# ============================================================ Maßstabsanpassung
def test_perfekte_messung_gibt_faktor_eins_und_null_residuum():
    soll = A.soll_abstaende_mm(8, 0.087)
    k, residuen = A.massstab_fit(soll, soll)
    assert abs(k - 1.0) < 1e-12
    assert max(abs(r) for r in residuen) < 1e-12


def test_konstruierter_massstabsfehler_kommt_exakt_zurueck():
    # 3 % zu lang gedruckt -> der Fit muss exakt 1.03 finden.
    soll = A.soll_abstaende_mm(8, 0.087)
    gemessen = [s * 1.03 for s in soll]
    k, residuen = A.massstab_fit(soll, gemessen)
    assert abs(k - 1.03) < 1e-12
    assert max(abs(r) for r in residuen) < 1e-12


def test_fit_ist_kleinste_quadrate_und_kein_summenverhaeltnis():
    # Wenn die Messung NICHT exakt auf einem Faktor liegt, unterscheiden
    # sich die Schätzer -- und nur dann. Alle anderen Tests hier benutzen
    # exakte Vielfache, bei denen jeder vernünftige Schätzer dasselbe
    # liefert; ohne diesen Fall bliebe ein Austausch der Formel unbemerkt
    # (genau das ist beim Mutationstest zunächst passiert).
    #
    # Von Hand nachgerechnet für soll=[0,1,10], gemessen=[0,2,10]:
    #   kleinste Quadrate durch Ursprung: (1*2 + 10*10)/(1 + 100) = 102/101
    #   Summenverhältnis (falsch):        (0+2+10)/(0+1+10)      =  12/11
    soll = [0.0, 1.0, 10.0]
    gemessen = [0.0, 2.0, 10.0]
    k, _ = A.massstab_fit(soll, gemessen)
    assert abs(k - 102.0 / 101.0) < 1e-12, k
    assert abs(k - 12.0 / 11.0) > 0.05, k

    # Und die Gewichtung stimmt in die richtige Richtung: der lange,
    # relativ genauere Abstand zieht das Ergebnis stärker als der kurze.
    assert k < 1.5, "der kurze Ausreißer darf den Faktor nicht dominieren"


def test_fit_geht_durch_den_ursprung():
    # Ein konstanter Versatz auf ALLEN Werten darf NICHT als Maßstab
    # weggerechnet werden -- er muss als Residuum sichtbar bleiben, sonst
    # verschwindet ein falsch angelegter Messnullpunkt unbemerkt.
    soll = A.soll_abstaende_mm(6, 0.087)
    gemessen = [s + 0.5 for s in soll]
    k, residuen = A.massstab_fit(soll, gemessen)
    assert max(abs(r) for r in residuen) > 0.1, residuen


def test_fit_ueberspringt_nicht_gemessene_linien():
    soll = A.soll_abstaende_mm(8, 0.087)
    gemessen = [s * 1.02 for s in soll]
    gemessen[3] = None
    gemessen[5] = None
    k, residuen = A.massstab_fit(soll, gemessen)
    assert abs(k - 1.02) < 1e-12
    assert len(residuen) == 6            # zwei Lücken ausgelassen


def test_fit_gibt_none_bei_zu_wenig_daten():
    assert A.massstab_fit([], []) == (None, None)
    assert A.massstab_fit([0.0], [0.0]) == (None, None)
    # Nur Linie 0: Nenner ist 0, kein Maßstab bestimmbar.
    assert A.massstab_fit([0.0, 0.0], [0.0, 0.0]) == (None, None)


# =========================================================== Tintenausbreitung
def test_ausbreitung_ist_gemessene_minus_soll_breite():
    # 1 Spalte à 0.087mm soll, 0.207mm gemessen -> 0.12mm Ausbreitung.
    assert abs(A.tintenausbreitung_mm(0.207, 1, 0.087) - 0.12) < 1e-12
    # 3 Spalten à 0.087 = 0.261 soll, 0.301 gemessen -> 0.04mm.
    assert abs(A.tintenausbreitung_mm(0.301, 3, 0.087) - 0.04) < 1e-12


def test_ausbreitung_bleibt_negativ_wenn_zu_duenn_gedruckt():
    # Nicht auf 0 klemmen: eine zu schmale Linie ist ein echtes Ergebnis
    # (zu schwache Dosierung), das sichtbar bleiben muss.
    assert A.tintenausbreitung_mm(0.05, 1, 0.087) < 0


def test_ausbreitung_ohne_messung_ist_none():
    assert A.tintenausbreitung_mm(None, 1, 0.087) is None


# ============================================================== Auflösung
def test_aufloesung_erkennt_die_offene_und_die_geschlossene_luecke():
    luecken = [0.0, 0.087, 0.174, 0.348, 0.696]
    a = A.aufloesung_bewerten(luecken, erste_getrennte_index=3,
                              ausbreitung_mm=0.12)
    assert abs(a["beobachtet_offen_mm"] - 0.348) < 1e-12
    assert abs(a["beobachtet_geschlossen_mm"] - 0.174) < 1e-12
    # Aus 0.12mm Ausbreitung allein wäre 0.174 die kleinste noch offene.
    assert abs(a["erwartet_offen_mm"] - 0.174) < 1e-12
    # Rest = beobachtete Zuwachsung minus Ausbreitung.
    assert abs(a["rest_min_mm"] - (0.174 - 0.12)) < 1e-12
    assert abs(a["rest_max_mm"] - (0.348 - 0.12)) < 1e-12


def test_aufloesung_ohne_rest_wenn_ausbreitung_alles_erklaert():
    # Ausbreitung so groß, dass sie die beobachtete Grenze voll erklärt.
    luecken = [0.0, 0.087, 0.174, 0.348]
    a = A.aufloesung_bewerten(luecken, 2, ausbreitung_mm=0.5)
    assert a["rest_min_mm"] == 0.0 and a["rest_max_mm"] == 0.0
    assert a["erwartet_offen_mm"] is None      # keine Lücke überlebt 0.5mm


def test_aufloesung_erste_luecke_offen_hat_keine_geschlossene():
    luecken = [0.0, 0.087, 0.174]
    a = A.aufloesung_bewerten(luecken, 1, ausbreitung_mm=0.01)
    assert a["beobachtet_offen_mm"] is not None
    assert a["beobachtet_geschlossen_mm"] is None


def test_aufloesung_weist_unsinnige_indizes_ab():
    luecken = [0.0, 0.087, 0.174]
    for index in (0, -1, 99):                # 0 hat keine Lücke davor
        a = A.aufloesung_bewerten(luecken, index, 0.05)
        assert a["beobachtet_offen_mm"] is None, index
    assert A.aufloesung_bewerten(luecken, None, 0.05)["beobachtet_offen_mm"] is None


# ============================================================ Gesamtauswertung
def test_auswerten_findet_massstab_und_ausbreitung_zurueck():
    soll = A.soll_abstaende_mm(8, 0.087)
    gemessen = [s * 1.015 for s in soll]
    e = A.auswerten(gemessen, 0.087, linienbreite_mm=0.207,
                    erste_getrennte_index=3)
    assert abs(e["massstab"] - 1.015) < 1e-12
    assert abs(e["abweichung_prozent"] - 1.5) < 1e-9
    assert abs(e["mm_per_column_korrigiert"] - 0.087 * 1.015) < 1e-12
    assert abs(e["ausbreitung_mm"] - 0.12) < 1e-12


def test_auswerten_trennt_faktor_von_positionsabhaengigem_fehler():
    soll = A.soll_abstaende_mm(8, 0.087)
    # Reiner Faktor -> Residuum praktisch 0.
    nur_faktor = A.auswerten([s * 1.02 for s in soll], 0.087)
    assert nur_faktor["residuum_rms_mm"] < 1e-12
    # Faktor plus wachsende Verzerrung -> Residuum deutlich > 0. Der Fit
    # durch den Ursprung schluckt einen Teil der Verzerrung (er verschiebt
    # dafür den Faktor), der REST bleibt aber stehen -- genau das ist die
    # Trennung, um die es hier geht.
    verzerrt = A.auswerten([s * 1.02 + 0.02 * (s ** 1.5) for s in soll], 0.087)
    assert verzerrt["residuum_rms_mm"] > 100 * nur_faktor["residuum_rms_mm"]
    assert verzerrt["residuum_rms_mm"] > 0.01


def test_auswerten_meldet_fehler_statt_zu_raten():
    assert "fehler" in A.auswerten([0.0], 0.087)
    assert "fehler" in A.auswerten([0.0, 1.0], 0.0)
    assert "fehler" in A.auswerten([0.0, 1.0], -0.5)
    # Nur Linie 0 gemessen -> kein Maßstab bestimmbar.
    assert "fehler" in A.auswerten([0.0, None, None], 0.087)


def test_auswerten_kommt_ohne_optionale_messungen_aus():
    soll = A.soll_abstaende_mm(6, 0.087)
    e = A.auswerten(soll, 0.087)
    assert e["ausbreitung_mm"] is None
    assert e["aufloesung"]["beobachtet_offen_mm"] is None
    text = A.bericht(e)
    assert "keine Linienbreite gemessen" in text
    assert "nicht angegeben" in text


# ==================================================================== Bericht
def test_bericht_nennt_die_korrigierte_spaltenbreite():
    soll = A.soll_abstaende_mm(8, 0.087)
    text = A.bericht(A.auswerten([s * 1.03 for s in soll], 0.087))
    assert "0.08961" in text                 # 0.087 * 1.03
    assert "+3.00 %" in text


def test_bericht_sagt_bei_stimmendem_massstab_dass_nichts_zu_tun_ist():
    soll = A.soll_abstaende_mm(8, 0.087)
    text = A.bericht(A.auswerten(soll, 0.087))
    assert "Maßstab stimmt" in text
    assert "keine Korrektur" in text


def test_bericht_warnt_bei_positionsabhaengigem_rest():
    # Die Warnschwelle liegt bei 2 Spalten RMS (bei 0.087 mm/Spalte also
    # 0.174 mm). Von BEIDEN Seiten eingegrenzt statt nur einmal darüber:
    # ein Test, der nur die laute Seite prüft, übersieht eine Schwelle, die
    # versehentlich immer feuert -- und eine Dauerwarnung wäre genauso
    # wertlos wie gar keine.
    soll = A.soll_abstaende_mm(8, 0.087)

    # Auf branch-EINDEUTIGE Phrasen geprüft: der Entwarnungszweig enthält
    # das Wort "positionsabhängig" ebenfalls ("kein positionsabhängiger
    # Verzug"), ein Teilstring-Test darauf würde also beide Seiten gleich
    # bewerten und nichts absichern.
    leicht = A.auswerten([s * 1.02 + 0.01 * (s ** 1.5) for s in soll], 0.087)
    assert leicht["residuum_rms_mm"] < 2 * 0.087
    text_leicht = A.bericht(leicht)
    assert "nicht wegzubekommen" not in text_leicht
    assert "im Wesentlichen ein Faktor" in text_leicht

    stark = A.auswerten([s * 1.02 + 0.2 * (s ** 1.5) for s in soll], 0.087)
    assert stark["residuum_rms_mm"] > 2 * 0.087
    text_stark = A.bericht(stark)
    assert "nicht wegzubekommen" in text_stark
    assert "im Wesentlichen ein Faktor" not in text_stark


def test_bericht_traegt_immer_die_einschraenkungen():
    # Die Faktor-2-Grenze und "das ist eine Summe" dürfen nie fehlen --
    # ohne sie liest sich jede Zahl genauer, als sie ist.
    soll = A.soll_abstaende_mm(6, 0.087)
    text = A.bericht(A.auswerten(soll, 0.087))
    assert "Faktor 2 genau" in text
    assert "Summe aus" in text


def test_bericht_meldet_fehler_lesbar():
    assert A.bericht({"fehler": "x"}).startswith("[precision-check]")


# ================================================================ Kommandozeile
def _cli(*argumente):
    return subprocess.run([sys.executable, WERKZEUG, "--cli", *argumente],
                          capture_output=True, text=True, timeout=60)


def test_cli_laeuft_eigenstaendig_durch():
    # Eigenständigkeit: als eigenes Skript gestartet, nicht als Import.
    p = _cli("--mm-per-column", "0.087",
             "--gemessen", "0,0.177,0.442,0.883,1.677")
    assert p.returncode == 0, p.stderr
    assert "precision-check: Auswertung" in p.stdout
    assert "Maßstab" in p.stdout


def test_cli_nimmt_leere_felder_als_nicht_gemessen():
    p = _cli("--gemessen", "0,0.177,,0.883")
    assert p.returncode == 0, p.stderr
    assert "(davon gemessen: 3)" in p.stdout


def test_cli_reicht_optionale_messungen_durch():
    p = _cli("--gemessen", "0,0.177,0.442,0.883,1.677",
             "--linienbreite", "0.207", "--erste-getrennte", "3")
    assert p.returncode == 0, p.stderr
    assert "Ausbreitung" in p.stdout
    assert "Auflösungsgrenze" in p.stdout


def test_cli_meldet_unlesbare_zahlen_ohne_traceback():
    p = _cli("--gemessen", "0,abc,1.0")
    assert p.returncode == 2
    assert "Traceback" not in p.stderr + p.stdout
    assert "konnte nicht gelesen werden" in p.stdout


def test_werkzeug_importiert_kein_printhead():
    # Eigenständigkeit ist eine Zusage an den Nutzer (die Datei soll allein
    # kopierbar sein) -- hier festgenagelt, damit sie nicht versehentlich
    # durch einen bequemen Import verlorengeht.
    with open(WERKZEUG, encoding="utf-8") as datei:
        quelltext = datei.read()
    assert "import printhead" not in quelltext
    assert "from printhead" not in quelltext


def test_werkzeug_braucht_kein_tkinter_fuer_die_rechnung():
    # tkinter wird erst in _gui() importiert. Der Rechenweg muss ohne
    # auskommen, sonst ist das Werkzeug auf Installationen ohne tkinter
    # (wie dieser Testumgebung) komplett unbenutzbar.
    with open(WERKZEUG, encoding="utf-8") as datei:
        kopf = datei.read().split("def _gui(")[0]
    assert "import tkinter" not in kopf


if __name__ == "__main__":
    tests = [v for k, v in list(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"OK: {t.__name__}")
    print(f"Alle {len(tests)} Auswertungs-Tests bestanden.")
