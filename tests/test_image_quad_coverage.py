"""
Tests für funktionen/image_quad_coverage.py (keine GUI).

Anders als image_line_to_angle.py ist dieses Werkzeug bewusst in CLI- und
GUI-Teil getrennt (siehe dessen Moduldocstring): tkinter wird erst in
_gui() importiert. Das macht die reine Rechnung (berechne_deckung,
render_export_bild, ...) hier -- anders als beim Winkel-Werkzeug --
automatisiert testbar, unabhängig davon, ob auf der jeweiligen Maschine
überhaupt tkinter installiert ist.

Der wichtigste Test dieser Datei ist deshalb der Import-Seiteneffekt-Test:
er beweist, dass "import image_quad_coverage" tkinter tatsächlich NICHT
mitzieht, statt das nur als Docstring-Behauptung stehen zu lassen.

Zweiter Schwerpunkt: berechne_deckung() nimmt EIN beliebiges Viereck
(nicht nur ein achsparalleles Rechteck) und rastert es über
PIL Image.histogram(mask=...). Um zu beweisen, dass hier wirklich die
Polygonform ausgewertet wird und nicht bloß die Bounding-Box, wird das
Ergebnis für ein schräges (gedrehtes) Viereck gegen eine komplett
UNABHÄNGIGE, in dieser Testdatei selbst geschriebene Punkt-in-Polygon-
Rasterung (Ray-Casting, even-odd-Regel) gegengeprüft -- keine der beiden
Implementierungen wird von der anderen kopiert.

Aufruf:  python tests/test_image_quad_coverage.py
"""

import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "funktionen"))

import image_quad_coverage as Q                               # noqa: E402
from PIL import Image                                         # noqa: E402

WERKZEUG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "funktionen", "image_quad_coverage.py")

# Direkt nach dem obigen Import geprüft: der reine Modul-Import darf
# tkinter (und damit PIL.ImageTk, das intern tkinter braucht) nicht in
# sys.modules ziehen -- unabhängig davon, ob tkinter auf dieser Maschine
# überhaupt installiert ist. Das muss HIER, unmittelbar nach dem Import
# oben, festgehalten werden, nicht erst in der Testfunktion weiter unten
# (da wäre das Modul längst importiert und die Aussage wertlos).
_TKINTER_NICHT_GELADEN_NACH_IMPORT = "tkinter" not in sys.modules


# ======================================================= Unabhängige Referenz
def _punkt_in_polygon(x, y, ecken):
    """Ray-Casting, even-odd-Regel -- Standardalgorithmus, hier von Hand
    geschrieben statt aus PIL/Q übernommen (siehe Dateidocstring)."""
    innen = False
    x1, y1 = ecken[-1]
    for x2, y2 in ecken:
        if (y1 > y) != (y2 > y):
            schnitt_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < schnitt_x:
                innen = not innen
        x1, y1 = x2, y2
    return innen


def _referenz_deckung(image, ecken, schwelle):
    """Zählt Pixel-Mittelpunkte innerhalb des Vierecks per eigener
    Punkt-in-Polygon-Prüfung -- komplett unabhängig von
    Q.berechne_deckung()."""
    grau = image.convert("L")
    breite, hoehe = grau.size
    px = grau.load()
    gesamt = schwarz = 0
    for y in range(hoehe):
        for x in range(breite):
            if _punkt_in_polygon(x + 0.5, y + 0.5, ecken):
                gesamt += 1
                if px[x, y] < schwelle:
                    schwarz += 1
    return gesamt, schwarz


# ============================================================= berechne_deckung
def test_deckung_bei_achsparallelem_viereck_ist_exakt_halb_halb():
    bild = Image.new("L", (10, 10), 255)
    for x in range(5):
        for y in range(10):
            bild.putpixel((x, y), 10)

    e = Q.berechne_deckung(bild, [(0, 0), (10, 0), (10, 10), (0, 10)], 128)
    assert e["pixel_gesamt"] == 100
    assert e["pixel_schwarz"] == 50 and e["pixel_weiss"] == 50
    assert abs(e["schwarz_prozent"] - 50.0) < 1e-9
    assert abs(e["weiss_prozent"] - 50.0) < 1e-9


def test_deckung_bei_ganz_schwarzem_und_ganz_weissem_bild():
    schwarz_bild = Image.new("L", (5, 5), 0)
    e = Q.berechne_deckung(schwarz_bild, [(0, 0), (5, 0), (5, 5), (0, 5)], 128)
    assert e["schwarz_prozent"] == 100.0
    assert e["pixel_weiss"] == 0

    weiss_bild = Image.new("L", (5, 5), 255)
    e2 = Q.berechne_deckung(weiss_bild, [(0, 0), (5, 0), (5, 5), (0, 5)], 128)
    assert e2["schwarz_prozent"] == 0.0
    assert e2["pixel_schwarz"] == 0


def test_schwelle_ist_exklusiv_gleich_der_schwelle_zaehlt_als_weiss():
    # Grauwert < Schwelle = schwarz -- ein Grauwert GENAU an der Schwelle
    # zählt also schon als weiß, nicht mehr als schwarz.
    genau_an_schwelle = Image.new("L", (4, 4), 128)
    e = Q.berechne_deckung(genau_an_schwelle, [(0, 0), (4, 0), (4, 4), (0, 4)],
                           schwelle=128)
    assert e["schwarz_prozent"] == 0.0
    assert e["weiss_prozent"] == 100.0

    knapp_darunter = Image.new("L", (4, 4), 127)
    e2 = Q.berechne_deckung(knapp_darunter, [(0, 0), (4, 0), (4, 4), (0, 4)],
                            schwelle=128)
    assert e2["schwarz_prozent"] == 100.0


def test_schwelle_wird_auf_0_bis_255_geklemmt():
    bild = Image.new("L", (4, 4), 200)
    ecken = [(0, 0), (4, 0), (4, 4), (0, 4)]
    assert Q.berechne_deckung(bild, ecken, schwelle=999)["schwelle"] == 255
    assert Q.berechne_deckung(bild, ecken, schwelle=-50)["schwelle"] == 0


def test_deckung_verlangt_genau_vier_eckpunkte():
    bild = Image.new("L", (4, 4), 128)
    assert "fehler" in Q.berechne_deckung(bild, [(0, 0), (1, 1), (2, 2)], 128)
    assert "fehler" in Q.berechne_deckung(
        bild, [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)], 128)
    assert "fehler" in Q.berechne_deckung(bild, [], 128)


def test_deckung_ausserhalb_des_bildes_ist_ein_fehler():
    bild = Image.new("L", (10, 10), 128)
    ecken = [(-50, -50), (-40, -50), (-40, -40), (-50, -40)]
    assert "fehler" in Q.berechne_deckung(bild, ecken, 128)


def test_entartetes_viereck_alle_ecken_gleich_ist_kein_fehler():
    # Absichtlich dokumentiertes Randverhalten, kein Bug: PIL zeichnet
    # für ein Polygon mit vier identischen Punkten trotzdem mindestens
    # einen Pixel -- also ein winziges, aber gültiges Ergebnis statt
    # eines Fehlers.
    bild = Image.new("L", (10, 10), 128)
    e = Q.berechne_deckung(bild, [(3, 3)] * 4, 128)
    assert "fehler" not in e
    assert e["pixel_gesamt"] == 1


def test_deckung_bei_schraegem_viereck_stimmt_mit_unabhaengiger_rasterung_ueberein():
    # DER zentrale Test dieser Datei: beweist, dass wirklich die
    # Polygonform ausgewertet wird (nicht nur deren Bounding-Box) --
    # gegen eine komplett eigenständige Punkt-in-Polygon-Implementierung
    # geprüft (siehe Dateidocstring).
    bild = Image.new("L", (100, 100), 255)
    for x in range(50):
        for y in range(100):
            bild.putpixel((x, y), 0)

    vierecke = [
        # Raute mittig ueber der Schwarz/Weiss-Grenze (symmetrisch).
        [(50, 20), (80, 50), (50, 80), (20, 50)],
        # Raute NUR mit einer Spitze im weissen Bereich -- Bounding-Box
        # und tatsaechliche Flaeche muessten hier deutlich auseinanderlaufen,
        # wenn die Implementierung faelschlich nur die Bounding-Box naehme.
        [(40, 20), (70, 50), (40, 80), (10, 50)],
        # Unregelmaessiges (kein Rechteck, kein Parallelogramm) Trapez.
        [(5, 5), (95, 15), (85, 90), (15, 70)],
    ]

    for ecken in vierecke:
        e = Q.berechne_deckung(bild, ecken, schwelle=128)
        referenz_gesamt, referenz_schwarz = _referenz_deckung(bild, ecken, 128)

        # Toleranz an den UMFANG gekoppelt, nicht an die Fläche: die
        # Differenz zwischen PIL-Polygonfüllung und der hier von Hand
        # geschriebenen Punkt-in-Polygon-Prüfung entsteht ausschließlich
        # an der Kontur (unterschiedliche Randpixel-Konvention), nicht
        # in der Fläche -- empirisch an diesen drei Vierecken liegt die
        # Differenz bei ~0.2-0.4 Pixel je Umfangseinheit, Faktor 0.5 gibt
        # deutlich Luft, ohne eine grob falsche Fläche durchzulassen.
        umfang = sum(
            math.hypot(ecken[i][0] - ecken[i - 1][0], ecken[i][1] - ecken[i - 1][1])
            for i in range(4))
        toleranz = max(10, round(umfang * 0.5))

        assert abs(e["pixel_gesamt"] - referenz_gesamt) <= toleranz, (
            ecken, e["pixel_gesamt"], referenz_gesamt, toleranz)
        assert abs(e["pixel_schwarz"] - referenz_schwarz) <= toleranz, (
            ecken, e["pixel_schwarz"], referenz_schwarz, toleranz)

    # Die zweite Raute (nur eine Spitze im Weissen) muss einen deutlich
    # kleineren Weiss-Anteil haben als ihre Bounding-Box (10..70 in x,
    # davon 50..70 = ein Drittel im Weissen) -- sonst wuerde faelschlich
    # nur rechteckig gerastert statt entlang der echten Kanten.
    spitze = Q.berechne_deckung(bild, vierecke[1], schwelle=128)
    bounding_box_weiss_anteil = (70 - 50) / (70 - 10) * 100.0  # ~33 %
    assert spitze["weiss_prozent"] < bounding_box_weiss_anteil - 5, spitze


# ==================================================================== Bericht
def test_bericht_zeigt_beide_prozentwerte():
    bild = Image.new("L", (10, 10), 255)
    for x in range(5):
        for y in range(10):
            bild.putpixel((x, y), 10)
    text = Q.bericht(Q.berechne_deckung(bild, [(0, 0), (10, 0), (10, 10), (0, 10)], 128))
    assert "50.00 %" in text
    assert "Schwarz" in text and "Weiß" in text


def test_bericht_meldet_fehler_lesbar():
    assert Q.bericht({"fehler": "x"}).startswith("[quad-deckung]")


# ============================================================ Export (PIL only)
def test_render_export_bild_behaelt_groesse_und_modus():
    original = Image.new("RGBA", (60, 40), (255, 255, 255, 255))
    ecken = [(5, 5), (55, 5), (55, 35), (5, 35)]
    ergebnis = Q.berechne_deckung(original, ecken, 128)
    export = Q.render_export_bild(original, ecken, ergebnis)
    assert export.size == (60, 40)
    assert export.mode == "RGB"


def test_render_export_bild_zeichnet_den_umriss():
    # Großes Bild, damit das Prozent-Label (dessen Kasten-Rahmen dieselbe
    # Farbe wie der Viereck-Umriss benutzt) klein gegenüber dem Bild
    # bleibt und die Prüfung unten nicht zufällig auf das Label statt
    # auf die Kante trifft.
    original = Image.new("L", (600, 400), 255).convert("RGB")
    ecken = [(50, 200), (300, 50), (550, 200), (300, 350)]
    ergebnis = Q.berechne_deckung(original.convert("L"), ecken, 128)
    export = Q.render_export_bild(original, ecken, ergebnis)

    zielfarbe = tuple(int(Q.QUAD_FARBE[i:i + 2], 16) for i in (1, 3, 5))

    # Exakter Mittelpunkt der Kante (50,200)->(300,50) muss (mit kleiner
    # Toleranz für die Linienbreite) in Zielfarbe gezeichnet sein.
    mx, my = (ecken[0][0] + ecken[1][0]) / 2, (ecken[0][1] + ecken[1][1]) / 2
    treffer = any(
        export.getpixel((int(mx) + dx, int(my) + dy)) == zielfarbe
        for dx in range(-3, 4) for dy in range(-3, 4)
    )
    assert treffer


def test_render_export_bild_zeigt_prozente_nur_ohne_fehler():
    original = Image.new("L", (200, 200), 255).convert("RGB")
    ecken = [(10, 10), (190, 10), (190, 190), (10, 190)]

    mit_erfolg = Q.berechne_deckung(original.convert("L"), ecken, 128)
    export_ok = Q.render_export_bild(original, ecken, mit_erfolg)

    dunkler_kasten_vorhanden = any(
        export_ok.getpixel((x, y)) == (0x20, 0x20, 0x20)
        for x in range(0, 250) for y in range(0, 60)
        if x < export_ok.width and y < export_ok.height
    )
    assert dunkler_kasten_vorhanden

    export_fehler = Q.render_export_bild(original, ecken, {"fehler": "x"})
    dunkler_kasten_bei_fehler = any(
        export_fehler.getpixel((x, y)) == (0x20, 0x20, 0x20)
        for x in range(0, 250) for y in range(0, 60)
        if x < export_fehler.width and y < export_fehler.height
    )
    assert not dunkler_kasten_bei_fehler


def test_suggest_export_dateiname():
    assert Q.suggest_export_dateiname(None) == "deckung_export.png"
    from pathlib import Path
    assert Q.suggest_export_dateiname(Path("/x/y/testbild.png")) == "testbild_deckung.png"


def test_lade_export_schrift_wirft_bei_keiner_groesse():
    for groesse in (20, 200, 2000):
        schrift = Q.lade_export_schrift(groesse)
        assert schrift is not None


# =================================================================== _parse_ecken
def test_parse_ecken_liest_vier_punkte():
    ecken = Q._parse_ecken("40,40;860,60;840,760;60,740")
    assert ecken == [(40.0, 40.0), (860.0, 60.0), (840.0, 760.0), (60.0, 740.0)]


def test_parse_ecken_wirft_bei_falschem_format():
    for text in ("40,40;abc", "40,40;40", "40,40,40;1,1", ""):
        try:
            ecken = Q._parse_ecken(text)
        except ValueError:
            continue
        # Leerer Text ergibt eine leere Liste (kein Fehler) -- das ist
        # okay, berechne_deckung() faengt die fehlende Anzahl ab.
        assert text == "" and ecken == []


# ================================================================ Kommandozeile
def _cli(*argumente):
    return subprocess.run([sys.executable, WERKZEUG, "--cli", *argumente],
                          capture_output=True, text=True, timeout=60)


def test_cli_laeuft_eigenstaendig_durch(tmp_bild=None):
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        bildpfad = os.path.join(tmp, "test.png")
        bild = Image.new("RGB", (100, 100), "white")
        px = bild.load()
        for x in range(50):
            for y in range(100):
                px[x, y] = (0, 0, 0)
        bild.save(bildpfad)

        p = _cli("--bild", bildpfad, "--ecken", "0,0;100,0;100,100;0,100")
        assert p.returncode == 0, p.stderr
        assert "Schwarz/Weiß-Deckung" in p.stdout
        assert "50.00 %" in p.stdout


def test_cli_export_schreibt_eine_echte_datei():
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        bildpfad = os.path.join(tmp, "test.png")
        Image.new("RGB", (50, 50), "black").save(bildpfad)
        exportpfad = os.path.join(tmp, "export.png")

        p = _cli("--bild", bildpfad, "--ecken", "0,0;50,0;50,50;0,50",
                 "--export", exportpfad)
        assert p.returncode == 0, p.stderr
        assert os.path.isfile(exportpfad)
        exportiert = Image.open(exportpfad)
        assert exportiert.size == (50, 50)


def test_cli_meldet_unlesbare_ecken_ohne_traceback():
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        bildpfad = os.path.join(tmp, "test.png")
        Image.new("RGB", (10, 10), "white").save(bildpfad)

        p = _cli("--bild", bildpfad, "--ecken", "abc;1,1;2,2;3,3")
        assert p.returncode == 2
        assert "Traceback" not in p.stderr + p.stdout
        assert "konnte nicht gelesen werden" in p.stdout


def test_cli_meldet_falsche_eckenanzahl_ohne_traceback():
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        bildpfad = os.path.join(tmp, "test.png")
        Image.new("RGB", (10, 10), "white").save(bildpfad)

        p = _cli("--bild", bildpfad, "--ecken", "0,0;1,1;2,2")
        assert p.returncode == 2
        assert "Traceback" not in p.stderr + p.stdout
        assert "4 Eckpunkte" in p.stdout


def test_cli_meldet_fehlendes_bild_ohne_traceback():
    p = _cli("--bild", "/pfad/der/nicht/existiert.png",
             "--ecken", "0,0;1,0;1,1;0,1")
    assert p.returncode == 2
    assert "Traceback" not in p.stderr + p.stdout


# ============================================================== Eigenständigkeit
def test_modul_importiert_ohne_tkinter_zu_laden():
    # DER wichtigste Test dieser Datei -- siehe Dateidocstring. Beweist
    # die im Moduldocstring von image_quad_coverage.py behauptete
    # Trennung, statt sie nur zu glauben.
    assert _TKINTER_NICHT_GELADEN_NACH_IMPORT, (
        "import image_quad_coverage hat tkinter in sys.modules gezogen -- "
        "die GUI/CLI-Trennung ist damit nicht mehr gegeben.")


def test_werkzeug_hat_kein_tkinter_import_vor_gui():
    with open(WERKZEUG, encoding="utf-8") as datei:
        inhalt = datei.read()
    kopf = inhalt.split("def _gui(")[0]
    assert "import tkinter" not in kopf


def test_werkzeug_importiert_kein_printhead():
    with open(WERKZEUG, encoding="utf-8") as datei:
        quelltext = datei.read()
    assert "import printhead" not in quelltext
    assert "from printhead" not in quelltext


if __name__ == "__main__":
    tests = [v for k, v in list(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"OK: {t.__name__}")
    print(f"Alle {len(tests)} Auswertungs-Tests bestanden.")
