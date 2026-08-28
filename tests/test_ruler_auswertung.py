"""
Tests für funktionen/ruler_auswertung.py (keine Hardware, keine GUI).

Wie bei precision_check_auswertung.py ist das Werkzeug bewusst eigenständig
und rechnet das Soll-Raster selbst nach, statt printhead.patterns zu
importieren. Der wichtigste Test hier ist deshalb wieder der DRIFT-Test:
soll_schritte() muss Spalte für Spalte mit dem übereinstimmen, was
printhead.patterns.ruler_ticks_pattern WIRKLICH druckt. Ermittelt wird das
Soll dabei nicht durch Nachrechnen der gleichen Formel (das wäre nur ein
Vergleich mit sich selbst), sondern durch Abtasten des echten Tinten-Arrays.

Der zweite Schwerpunkt ist neu gegenüber precision_check_auswertung.py: die
Standardfehler-Signifikanzprüfung des Major/Minor-Verhältnisses (siehe
Moduldocstring von ruler_auswertung.py, Punkt 2). Ein Test pinnt den
tatsächlichen Fund an echten Messdaten fest (Kante-zu-Kante-Messung,
2026-08-28, ~7.4 Standardfehler Abweichung von 10,0) als Regressionstest,
zwei weitere prüfen die Guard-Bedingung an den Rändern (zu wenig Werte,
exakt null Streuung -> sigmas bleibt None statt durch 0 zu teilen).

Die grafische Oberfläche wird hier NICHT getestet -- siehe
test_precision_auswertung.py für die Begründung, die hier identisch gilt.

Aufruf:  python tests/test_ruler_auswertung.py
"""

import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "funktionen"))

import ruler_auswertung as R                                  # noqa: E402
from printhead import patterns                                # noqa: E402

WERKZEUG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "funktionen", "ruler_auswertung.py")

# Die reale zweite Messreihe des Anlagenbesitzers (linke Kante zu linker
# Kante, --mm-per-column 0.087, 2026-08-28) -- derselbe Datensatz, an dem
# das ~7.4-Sigma-Ergebnis im Moduldocstring von ruler_auswertung.py
# dokumentiert ist. Als Regressionstest festgenagelt: läuft die
# Signifikanzrechnung künftig auseinander, soll genau DIESER reale Fund
# das zuerst zeigen, nicht nur ein synthetischer Fall.
ECHTE_MAJOR_MM = [9.79, 9.92, 9.81, 9.82, 9.85, 9.90, 9.96, 9.79]
ECHTE_MINOR_MM = [0.95, 0.92, 0.91, 0.95, 0.93, 0.96, 0.94, 0.92,
                  0.98, 0.99, 0.97, 0.93, 0.96, 0.91, 0.91, 0.86,
                  0.97, 0.93, 0.88, 0.91, 0.91, 0.93, 0.93, 0.96,
                  0.93, 0.81, 0.95, 0.97, 0.91, 0.96, 0.97, 0.91]


# ====================================================== Drift gegen das Muster
def test_soll_schritte_stimmt_mit_dem_echten_muster_ueberein():
    # DER zentrale Test dieser Datei: das eigenständig nachgerechnete
    # Soll-Raster muss dem entsprechen, was --pattern ruler wirklich
    # druckt. Die Tick-Spalten werden aus dem Tinten-Array selbst
    # abgetastet (nicht aus der internen Formel von ruler_ticks_pattern
    # kopiert): jede Spalte mit mehr als einem True-Pixel (die
    # durchgehende Grundlinie liefert an JEDER Spalte genau ein Pixel) ist
    # eine Tick-Spalte; die erste davon nach Spalte 0 ist per Konstruktion
    # minor_step. Unter den Tick-Spalten ist eine Major-Spalte daran zu
    # erkennen, dass sie bis Zeile 0/rows-1 reicht -- der Major-Tick-Band
    # ist laut ruler_ticks_pattern's Docstring bei jeder rows/mm_per_column
    # -Kombination ein STRIKTES Superset des Minor-Bands, und bei
    # rows=IMAGE_HEIGHT (default) klemmt der 20mm-Major-Tick auf die volle
    # Höhe (152 Zeilen entsprechen ~13,2mm Balkenhöhe).
    for mm_per_column in (0.087, 0.0868421, 0.1, 0.2, 0.05):
        ink = patterns.ruler_ticks_pattern(60.0, mm_per_column)
        width = ink.shape[1]

        tick_cols = [c for c in range(1, width) if ink[:, c].sum() > 1]
        assert tick_cols, mm_per_column
        minor_step_echt = tick_cols[0]

        major_cols = [c for c in tick_cols if ink[0, c] or ink[-1, c]]
        assert major_cols, mm_per_column
        major_step_echt = major_cols[0]

        minor_step_eigen, major_step_eigen = R.soll_schritte(mm_per_column)
        assert minor_step_eigen == minor_step_echt, mm_per_column
        assert major_step_eigen == major_step_echt, mm_per_column
        # Die zentrale Garantie der Musterlogik, hier gegen das echte
        # Array bestätigt statt nur gegen die eigene Formel: exaktes
        # Vielfaches, kein unabhängiges Runden.
        assert major_step_echt == minor_step_echt * 10, mm_per_column


def test_soll_schritte_kennt_den_realen_riemenwert():
    # Handnachrechnung für den auf der Anlage tatsächlich benutzten Wert.
    assert R.soll_schritte(0.087) == (11, 110)
    minor_mm, major_mm = R.soll_mm(0.087)
    assert abs(minor_mm - 0.957) < 1e-12
    assert abs(major_mm - 9.57) < 1e-9


def test_soll_schritte_bei_glatten_werten():
    # Bei mm_per_column=0.1 rundet 1/0.1 exakt auf 10 -- keine
    # Rasterungsabweichung vom nominellen Maßband.
    assert R.soll_schritte(0.1) == (10, 100)
    minor_mm, major_mm = R.soll_mm(0.1)
    assert abs(minor_mm - 1.0) < 1e-12
    assert abs(major_mm - 10.0) < 1e-12


# ==================================================================== Stdabw
def test_stdabw_stimmt_mit_der_lehrbuchformel_ueberein():
    # Bekannter Fall (Stichproben-Stdabw, n-1): [2,4,4,4,5,5,7,9] -> 2.13809...
    werte = [2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0]
    assert abs(R._stdabw(werte) - 2.138089935299395) < 1e-9


def test_stdabw_bei_weniger_als_zwei_werten_ist_null():
    assert R._stdabw([]) == 0.0
    assert R._stdabw([5.0]) == 0.0


def test_stdabw_bei_identischen_werten_ist_exakt_null():
    # Wichtig für die sem_verhaeltnis-Guard-Bedingung unten: exakt 0.0,
    # nicht nur "sehr klein".
    assert R._stdabw([9.79, 9.79, 9.79, 9.79]) == 0.0


# ============================================================ Maßstabsanpassung
def test_massstab_fit_ist_kleinste_quadrate_und_kein_summenverhaeltnis():
    # Identischer Fall wie in test_precision_auswertung.py, von Hand
    # nachgerechnet: soll=[1,10], gemessen=[2,10] ->
    #   kleinste Quadrate durch Ursprung: (1*2 + 10*10)/(1 + 100) = 102/101
    k = R.massstab_fit([(1.0, 2.0), (10.0, 10.0)])
    assert abs(k - 102.0 / 101.0) < 1e-12, k


def test_massstab_fit_gibt_none_bei_zu_wenig_daten():
    assert R.massstab_fit([]) is None
    assert R.massstab_fit([(0.0, 0.0)]) is None       # Nenner 0


def test_massstab_fit_gewichtet_groessere_soll_werte_staerker():
    # Major (soll=10) und Minor (soll=1) gemischt, wie in auswerten(): der
    # Major-Anteil muss den Faktor dominieren.
    paare = [(1.0, 1.0)] * 20 + [(10.0, 11.0)] * 5
    k = R.massstab_fit(paare)
    assert k > 1.05, k                # naeher an den Major-Werten (Faktor 1.1)


# ============================================================ Gesamtauswertung
def test_auswerten_filtert_none_werte_heraus():
    minor_soll, major_soll = R.soll_mm(0.087)
    e = R.auswerten([major_soll, None, major_soll * 1.02],
                    [minor_soll, None, minor_soll, minor_soll * 0.9],
                    0.087)
    assert e["major_n"] == 2
    assert e["minor_n"] == 3


def test_auswerten_meldet_fehler_statt_zu_raten():
    assert "fehler" in R.auswerten([1.0], [1.0], 0.0)
    assert "fehler" in R.auswerten([1.0], [1.0], -0.5)
    assert "fehler" in R.auswerten([], [1.0, 2.0], 0.087)
    assert "fehler" in R.auswerten([1.0, 2.0], [], 0.087)
    # Nach dem Filtern bleibt eine Kategorie leer.
    assert "fehler" in R.auswerten([None], [1.0], 0.087)


def test_auswerten_verhaeltnis_ist_major_avg_durch_minor_avg():
    e = R.auswerten([10.0, 12.0], [1.0, 1.0, 1.0], 0.087)
    assert abs(e["major_avg_mm"] - 11.0) < 1e-12
    assert abs(e["minor_avg_mm"] - 1.0) < 1e-12
    assert abs(e["verhaeltnis_major_minor"] - 11.0) < 1e-12


def test_auswerten_sigma_guard_bei_konstanten_messwerten_ist_none():
    # Exakt null Streuung in beiden Kategorien -> sem_verhaeltnis ist exakt
    # 0.0 -> die Guard-Bedingung ("sem_verhaeltnis > 0") muss das abfangen,
    # sonst gäbe es hier eine ZeroDivisionError statt eines sauberen None.
    e = R.auswerten([9.79] * 8, [0.95] * 10, 0.087)
    assert e["verhaeltnis_sem"] == 0.0
    assert e["verhaeltnis_abweichung_sigmas"] is None


def test_auswerten_sigma_ist_none_bei_zu_wenig_werten():
    # Nur 1 Wert je Kategorie -- _stdabw ist da schon 0.0, aber die
    # Signifikanzprüfung selbst braucht mindestens 2 je Kategorie.
    minor_soll, major_soll = R.soll_mm(0.087)
    e = R.auswerten([major_soll], [minor_soll], 0.087)
    assert e["verhaeltnis_abweichung_sigmas"] is None


def test_auswerten_echte_messreihe_zeigt_die_dokumentierte_abweichung():
    # Regressionstest gegen den realen Fund (siehe Moduldocstring von
    # ruler_auswertung.py und die Konstanten oben in dieser Datei): mit
    # unabhängiger Fehlerfortpflanzung von Hand nachgerechnet.
    e = R.auswerten(ECHTE_MAJOR_MM, ECHTE_MINOR_MM, 0.087)
    assert e["major_n"] == 8
    assert e["minor_n"] == 32
    assert abs(e["major_avg_mm"] - 9.855) < 1e-9
    assert abs(e["minor_avg_mm"] - 0.931875) < 1e-9
    assert abs(e["verhaeltnis_major_minor"] - 10.575452716297788) < 1e-6

    # Fehlerfortpflanzung unabhängig von auswerten() nachgerechnet (nicht
    # einfach dieselbe Formel zurückgespiegelt): SEM je Kategorie aus der
    # Stichproben-Stdabw von Hand über die statistics-Bibliothek.
    import statistics as st
    sem_major = st.stdev(ECHTE_MAJOR_MM) / math.sqrt(len(ECHTE_MAJOR_MM))
    sem_minor = st.stdev(ECHTE_MINOR_MM) / math.sqrt(len(ECHTE_MINOR_MM))
    verhaeltnis = st.mean(ECHTE_MAJOR_MM) / st.mean(ECHTE_MINOR_MM)
    sem_verhaeltnis = verhaeltnis * math.sqrt(
        (sem_major / st.mean(ECHTE_MAJOR_MM)) ** 2
        + (sem_minor / st.mean(ECHTE_MINOR_MM)) ** 2)
    sigmas_erwartet = abs(verhaeltnis - 10.0) / sem_verhaeltnis

    assert abs(e["verhaeltnis_abweichung_sigmas"] - sigmas_erwartet) < 1e-9
    assert e["verhaeltnis_abweichung_sigmas"] > 7.0
    assert e["verhaeltnis_abweichung_sigmas"] < 7.8


def test_auswerten_geringes_messrauschen_bleibt_unauffaellig():
    # Gegenprobe zum vorigen Test: bei Messrauschen, das tatsächlich nur
    # zufällig streut (hier künstlich mit geringer, gleichmäßiger Sigma
    # erzeugt), darf die Signifikanzprüfung NICHT anschlagen -- sonst wäre
    # sie bei jeder realen Messung falsch-positiv.
    import random
    minor_soll, major_soll = R.soll_mm(0.087)
    rng = random.Random(3)
    major = [major_soll + rng.gauss(0, 0.01) for _ in range(6)]
    minor = [minor_soll + rng.gauss(0, 0.01) for _ in range(20)]
    e = R.auswerten(major, minor, 0.087)
    assert e["verhaeltnis_abweichung_sigmas"] is not None
    assert e["verhaeltnis_abweichung_sigmas"] < R._SIGNIFIKANZ_SCHWELLE_SIGMA


# ==================================================================== Bericht
def test_bericht_meldet_fehler_lesbar():
    assert R.bericht({"fehler": "x"}).startswith("[ruler]")


def test_bericht_sagt_bei_stimmendem_massstab_dass_er_stimmt():
    minor_soll, major_soll = R.soll_mm(0.087)
    text = R.bericht(R.auswerten([major_soll] * 4, [minor_soll] * 10, 0.087))
    assert "Maßstab stimmt mit dem Soll-Raster überein" in text


def test_bericht_nennt_die_korrigierte_spaltenbreite_bei_massstabsfehler():
    minor_soll, major_soll = R.soll_mm(0.087)
    text = R.bericht(R.auswerten([major_soll * 1.03] * 4,
                                 [minor_soll * 1.03] * 10, 0.087))
    assert "+3.00 %" in text
    assert "0.08961" in text                 # 0.087 * 1.03


def test_bericht_warnt_signifikant_bei_der_echten_messreihe():
    # Branch-eindeutig geprüft (wie in test_precision_auswertung.py): die
    # "unauffällig"-Seite enthält "Rahmen dessen", die "signifikant"-Seite
    # "MEHR, als" -- ein Teilstring-Test müsste beide Formulierungen
    # unterscheiden, sonst wäre die Prüfung wertlos.
    text = R.bericht(R.auswerten(ECHTE_MAJOR_MM, ECHTE_MINOR_MM, 0.087))
    assert "MEHR, als die eigene" in text
    assert "im Rahmen dessen" not in text
    assert "10.575" in text


def test_bericht_bleibt_unauffaellig_bei_geringem_messrauschen():
    import random
    minor_soll, major_soll = R.soll_mm(0.087)
    rng = random.Random(3)
    major = [major_soll + rng.gauss(0, 0.01) for _ in range(6)]
    minor = [minor_soll + rng.gauss(0, 0.01) for _ in range(20)]
    text = R.bericht(R.auswerten(major, minor, 0.087))
    assert "im Rahmen dessen" in text
    assert "MEHR, als die eigene" not in text


def test_bericht_meldet_massstab_nicht_bestimmbar_wenn_none():
    # Defensiver Zweig: in der Praxis liefert auswerten() (bei gültigen,
    # nicht-leeren Eingaben) immer einen Maßstab, weil soll_mm() nie 0
    # ist. Direkt gegen _urteil()/bericht() mit einem von Hand gebauten
    # Ergebnis-Dict geprüft, damit der Zweig trotzdem abgesichert ist --
    # gleiche Vorgehensweise wie test_urteil_drift_... in
    # test_rauschen_und_tempo.py.
    ergebnis = {
        "mm_per_column": 0.087, "minor_step_spalten": 11,
        "major_step_spalten": 110, "minor_soll_mm": 0.957,
        "major_soll_mm": 9.57, "minor_n": 2, "major_n": 2,
        "minor_avg_mm": 0.95, "major_avg_mm": 9.5,
        "minor_std_mm": 0.0, "major_std_mm": 0.0,
        "minor_min_mm": 0.95, "minor_max_mm": 0.95,
        "major_min_mm": 9.5, "major_max_mm": 9.5,
        "massstab": None, "mm_per_column_korrigiert": None,
        "abweichung_prozent": None, "verhaeltnis_major_minor": 10.0,
        "verhaeltnis_sem": None, "verhaeltnis_abweichung_sigmas": None,
    }
    text = R.bericht(ergebnis)
    assert "nicht bestimmbar" in text


def test_bericht_traegt_immer_die_einschraenkungen():
    minor_soll, major_soll = R.soll_mm(0.087)
    text = R.bericht(R.auswerten([major_soll] * 4, [minor_soll] * 10, 0.087))
    assert "Summe aus" in text
    assert "immer dieselbe Kante gegen" in text


# ================================================================ Kommandozeile
def _cli(*argumente):
    return subprocess.run([sys.executable, WERKZEUG, "--cli", *argumente],
                          capture_output=True, text=True, timeout=60)


def test_cli_laeuft_eigenstaendig_mit_der_echten_messreihe_durch():
    p = _cli("--mm-per-column", "0.087",
             "--major", ",".join(str(v) for v in ECHTE_MAJOR_MM),
             "--minor", ",".join(str(v) for v in ECHTE_MINOR_MM))
    assert p.returncode == 0, p.stderr
    assert "ruler: Auswertung" in p.stdout
    assert "Major/Minor" in p.stdout
    assert "10.575" in p.stdout


def test_cli_nimmt_leere_felder_als_nicht_gemessen():
    p = _cli("--major", "9.79,,9.81,9.82", "--minor", "0.95,0.92,0.91")
    assert p.returncode == 0, p.stderr
    assert "n= 3" in p.stdout               # Major: 4 Felder, 1 leer -> 3


def test_cli_benutzt_den_default_mm_per_column():
    p = _cli("--major", "9.79,9.92,9.81,9.82",
             "--minor", "0.95,0.92,0.91,0.95")
    assert p.returncode == 0, p.stderr
    assert "--mm-per-column 0.087" in p.stdout


def test_cli_meldet_unlesbare_zahlen_ohne_traceback():
    p = _cli("--major", "abc,9.9", "--minor", "0.9,0.9")
    assert p.returncode == 2
    assert "Traceback" not in p.stderr + p.stdout
    assert "konnten nicht gelesen werden" in p.stdout


def test_cli_verlangt_major_und_minor():
    # argparse selbst (required=True) -- kein try/except drumherum, muss
    # also mit dem argparse-typischen Exitcode 2 und OHNE Traceback
    # scheitern.
    p = _cli("--minor", "0.9,0.9")
    assert p.returncode == 2
    assert "Traceback" not in p.stderr + p.stdout


def test_werkzeug_importiert_kein_printhead():
    with open(WERKZEUG, encoding="utf-8") as datei:
        quelltext = datei.read()
    assert "import printhead" not in quelltext
    assert "from printhead" not in quelltext


def test_werkzeug_braucht_kein_tkinter_fuer_die_rechnung():
    with open(WERKZEUG, encoding="utf-8") as datei:
        kopf = datei.read().split("def _gui(")[0]
    assert "import tkinter" not in kopf


if __name__ == "__main__":
    tests = [v for k, v in list(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"OK: {t.__name__}")
    print(f"Alle {len(tests)} Auswertungs-Tests bestanden.")
