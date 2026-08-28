"""
Auswertung des Testmusters "precision-check"
=============================================

Eigenständiges Werkzeug. Es wertet die am gedruckten Muster GEMESSENEN Werte
aus und beantwortet damit drei Fragen, die eine einzelne Messung nicht
trennen kann:

  1. Stimmt der Maßstab? (--mm-per-column richtig kalibriert?)
  2. Ist der Fehler ein gleichmäßiger Faktor oder positionsabhängig?
  3. Wie groß ist die Tintenausbreitung -- und erklärt sie die beobachtete
     Auflösungsgrenze, oder bleibt ein Rest, der auf Tracking/Timing geht?

Erzeugt wird das Muster mit:

    python main.py --pattern precision-check --mode line \\
        --pattern-gap-start 1 --pattern-line-cols 1

Das Muster druckt Linien PARALLEL ZUR DÜSENLEISTE, deren Abstände sich
entlang der Fahrtrichtung verdoppeln. Die Soll-Abstände sind exakt bekannt
(gap_start * 2^n Spalten), also lässt sich alles Gemessene dagegen halten.

Diese Datei ist bewusst UNABHÄNGIG vom printhead-Paket: sie rechnet das
Layout selbst nach, statt es zu importieren, damit sie auch für sich allein
kopiert und benutzt werden kann. Ein Test im Repo
(tests/test_precision_auswertung.py) vergleicht die hiesige Berechnung gegen
printhead.patterns.precision_check_layout, damit die beiden nicht
auseinanderlaufen.

Benutzung
---------
Mit grafischer Oberfläche (benötigt tkinter):

    python funktionen/precision_check_auswertung.py

Ohne GUI, direkt auf der Kommandozeile:

    python funktionen/precision_check_auswertung.py --cli \\
        --mm-per-column 0.087 --gap-start 1 --line-cols 1 \\
        --gemessen 0,0.18,0.44,0.87,1.66,3.14,6.01,11.7 \\
        --linienbreite 0.21 --erste-getrennte 3
"""

import argparse
import math
import sys


# ===========================================================================
# Reine Berechnung -- ohne GUI, damit direkt testbar
# ===========================================================================
def soll_layout(anzahl_linien, line_cols=1, gap_start=1):
    """
    Rechnet das Soll-Layout des Musters nach: Startspalte und davorliegende
    Lücke jeder Linie.

    Muss mit printhead.patterns.precision_check_layout übereinstimmen (siehe
    Modul-Docstring). Die Lücke verdoppelt sich nach jeder Linie, die erste
    Linie hat keine Lücke vor sich.

    Rückgabe: Liste von dicts mit index, start_spalte, luecke_davor_spalten.
    """
    line_cols = max(1, int(line_cols))
    gap = max(1, int(gap_start))

    linien = []
    spalte = 0
    luecke_davor = 0
    for index in range(max(0, int(anzahl_linien))):
        linien.append({
            "index": index,
            "start_spalte": spalte,
            "luecke_davor_spalten": luecke_davor,
        })
        luecke_davor = gap
        spalte += line_cols + gap
        gap *= 2
    return linien


def soll_abstaende_mm(anzahl_linien, mm_per_column, line_cols=1, gap_start=1):
    """
    Soll-Abstand jeder Linie von der ERSTEN Linie, in mm.

    Bewusst kumulativ von Linie 0 aus und nicht Lücke für Lücke: mit einem
    Messschieber misst man vom selben Bezugspunkt aus, und dabei summieren
    sich Einzelfehler nicht auf.
    """
    return [linie["start_spalte"] * mm_per_column
            for linie in soll_layout(anzahl_linien, line_cols, gap_start)]


def massstab_fit(soll_mm, gemessen_mm):
    """
    Kleinste-Quadrate-Anpassung ``gemessen = k * soll`` DURCH DEN URSPRUNG.

    Durch den Ursprung, weil Linie 0 der Bezugspunkt ist: ihr Abstand zu sich
    selbst ist per Definition exakt 0, also gibt es hier keinen freien
    Achsenabschnitt zu schätzen. Ein systematischer Versatz (falsch
    angelegter Messnullpunkt) fällt dadurch nicht unter den Tisch, sondern
    zeigt sich deutlich in den Residuen.

    Die langen Abstände bekommen dabei automatisch mehr Gewicht -- richtig
    so, denn ihre relative Messgenauigkeit ist besser.

    Rückgabe: (k, residuen) oder (None, None), wenn zu wenig verwertbare
    Paare vorliegen.
    """
    paare = [(s, m) for s, m in zip(soll_mm, gemessen_mm)
             if s is not None and m is not None]
    nenner = sum(s * s for s, _ in paare)
    if len(paare) < 2 or nenner <= 0.0:
        return None, None

    k = sum(s * m for s, m in paare) / nenner
    residuen = [m - k * s for s, m in paare]
    return k, residuen


def kennzahlen(werte):
    """RMS und größter Betrag einer Residuenliste (leere Liste -> 0.0)."""
    if not werte:
        return 0.0, 0.0
    rms = math.sqrt(sum(w * w for w in werte) / len(werte))
    return rms, max(abs(w) for w in werte)


def tintenausbreitung_mm(linienbreite_gemessen_mm, line_cols, mm_per_column):
    """
    Tintenausbreitung = gemessene Linienbreite minus Soll-Breite.

    Soll-Breite ist ``line_cols * mm_per_column``. Der Überschuss ist die
    Verbreiterung durch die Tropfenausbreitung auf dem Papier -- verteilt
    auf beide Seiten der Linie, also je die Hälfte pro Seite.

    Negative Werte werden NICHT auf 0 geklemmt: eine gemessene Breite unter
    Soll ist ein echtes Ergebnis (zu schwache Dosierung, fehlende Düsen)
    und soll sichtbar bleiben, nicht stillschweigend verschwinden.
    """
    if linienbreite_gemessen_mm is None:
        return None
    return linienbreite_gemessen_mm - max(1, int(line_cols)) * mm_per_column


def aufloesung_bewerten(soll_luecken_mm, erste_getrennte_index, ausbreitung_mm):
    """
    Bewertet die beobachtete Auflösungsgrenze gegen das, was allein die
    Tintenausbreitung erklären würde.

    ``erste_getrennte_index`` ist der Index der ersten Linie, deren Lücke
    DAVOR im Druck noch als Weiß durchkommt (Linie 0 hat keine Lücke davor,
    der kleinste sinnvolle Index ist also 1).

    Modell: zwei benachbarte Linien wachsen um je ``ausbreitung/2``
    aufeinander zu, eine Soll-Lücke G schrumpft also auf G - ausbreitung und
    schließt sich, sobald G <= ausbreitung. Die kleinste noch offene Lücke
    ist demnach die kleinste Soll-Lücke oberhalb der Ausbreitung.

    Bleibt die beobachtete Grenze GRÖBER als das, steckt der Rest nicht in
    der Tinte, sondern in Position/Timing.

    Wichtige Einschränkung, die auch im Bericht steht: die Lücken
    verdoppeln sich, das Ergebnis ist also höchstens auf den Faktor 2 genau.
    Die Zahlen unten sind eine EINGRENZUNG, keine Punktmessung.
    """
    ergebnis = {
        "beobachtet_offen_mm": None,
        "beobachtet_geschlossen_mm": None,
        "erwartet_offen_mm": None,
        "rest_min_mm": None,
        "rest_max_mm": None,
    }
    if erste_getrennte_index is None:
        return ergebnis

    index = int(erste_getrennte_index)
    if not 1 <= index < len(soll_luecken_mm):
        return ergebnis

    ergebnis["beobachtet_offen_mm"] = soll_luecken_mm[index]
    # Die nächstkleinere Lücke der Folge -- die letzte, die sich noch
    # geschlossen hat. Bei index == 1 gibt es keine kleinere.
    if index >= 2:
        ergebnis["beobachtet_geschlossen_mm"] = soll_luecken_mm[index - 1]

    if ausbreitung_mm is None:
        return ergebnis

    # Kleinste Soll-Lücke, die die Ausbreitung allein noch offen ließe.
    for luecke in soll_luecken_mm[1:]:
        if luecke > ausbreitung_mm:
            ergebnis["erwartet_offen_mm"] = luecke
            break

    # Gesamte wirksame Zuwachsung liegt zwischen der größten geschlossenen
    # und der kleinsten offenen Lücke. Davon geht die Ausbreitung ab; was
    # bleibt, ist Position/Timing.
    unten = ergebnis["beobachtet_geschlossen_mm"] or 0.0
    ergebnis["rest_min_mm"] = max(0.0, unten - ausbreitung_mm)
    ergebnis["rest_max_mm"] = max(0.0, ergebnis["beobachtet_offen_mm"]
                                  - ausbreitung_mm)
    return ergebnis


def auswerten(gemessen_mm, mm_per_column, line_cols=1, gap_start=1,
              linienbreite_mm=None, erste_getrennte_index=None):
    """
    Gesamtauswertung. ``gemessen_mm[i]`` ist der gemessene Abstand von
    Linie 0 zu Linie i (``None`` für nicht gemessene Linien; der erste Wert
    ist per Definition 0).

    Rückgabe: dict mit allen Ergebnissen, oder mit ``fehler``, wenn sich
    nichts auswerten lässt.
    """
    if mm_per_column <= 0:
        return {"fehler": "mm-per-column muss größer als 0 sein."}

    anzahl = len(gemessen_mm)
    if anzahl < 2:
        return {"fehler": "Mindestens zwei Linien werden benötigt."}

    soll = soll_abstaende_mm(anzahl, mm_per_column, line_cols, gap_start)
    layout = soll_layout(anzahl, line_cols, gap_start)
    soll_luecken = [linie["luecke_davor_spalten"] * mm_per_column
                    for linie in layout]

    k, residuen = massstab_fit(soll, gemessen_mm)
    if k is None:
        return {"fehler": ("Zu wenige gemessene Werte für eine Anpassung "
                           "(mindestens zwei Linien mit Abstand > 0).")}

    # Welcher Linien-INDEX zu residuen[j] gehört: massstab_fit lässt jede
    # Linie mit gemessen_mm[i] is None einfach aus, residuen ist also KÜRZER
    # als soll/gemessen_mm, sobald irgendwo mitten in der Liste eine Lücke
    # steht (nicht nur am Ende). residuen selbst trägt diesen Index nicht --
    # ohne ihn hier separat mitzuführen, würde bericht()'s Zeilentabelle
    # residuen[j] gegen soll_mm[j] statt gegen soll_mm[den richtigen Index]
    # anzeigen, also ab der ersten Lücke systematisch falsche Soll-Werte
    # (und damit falsche "Linie N"-Beschriftungen) neben jeden Rest stellen.
    # soll[i] ist nie None (soll_abstaende_mm liefert immer eine Zahl), der
    # Filter hier ist also exakt derselbe wie in massstab_fit.
    residuen_index = [i for i, m in enumerate(gemessen_mm) if m is not None]

    rms, maxabs = kennzahlen(residuen)
    ausbreitung = tintenausbreitung_mm(linienbreite_mm, line_cols,
                                       mm_per_column)

    return {
        "anzahl_linien": anzahl,
        "anzahl_gemessen": sum(1 for m in gemessen_mm if m is not None),
        "mm_per_column": mm_per_column,
        "line_cols": max(1, int(line_cols)),
        "gap_start": max(1, int(gap_start)),
        "soll_mm": soll,
        "soll_luecken_mm": soll_luecken,
        "massstab": k,
        "mm_per_column_korrigiert": mm_per_column * k,
        "abweichung_prozent": (k - 1.0) * 100.0,
        "residuen": residuen,
        "residuen_index": residuen_index,
        "residuum_rms_mm": rms,
        "residuum_max_mm": maxabs,
        "linienbreite_soll_mm": max(1, int(line_cols)) * mm_per_column,
        "linienbreite_gemessen_mm": linienbreite_mm,
        "ausbreitung_mm": ausbreitung,
        "aufloesung": aufloesung_bewerten(soll_luecken, erste_getrennte_index,
                                          ausbreitung),
    }


# ===========================================================================
# Bericht
# ===========================================================================
def bericht(ergebnis):
    """Formatiert das Ergebnis von :func:`auswerten` als lesbaren Text."""
    if "fehler" in ergebnis:
        return f"[precision-check] {ergebnis['fehler']}"

    spalten = ergebnis["mm_per_column"]
    zeilen = []
    zeilen.append("---- precision-check: Auswertung ----")
    zeilen.append(f"  Linien im Muster    : {ergebnis['anzahl_linien']} "
                  f"(davon gemessen: {ergebnis['anzahl_gemessen']})")
    zeilen.append(f"  Musterparameter     : --pattern-line-cols "
                  f"{ergebnis['line_cols']}  --pattern-gap-start "
                  f"{ergebnis['gap_start']}  --mm-per-column {spalten:g}")
    zeilen.append("")

    # --- 1) Maßstab -------------------------------------------------------
    zeilen.append("  1) Maßstab")
    zeilen.append(f"     Faktor gemessen/soll : {ergebnis['massstab']:.5f}  "
                  f"({ergebnis['abweichung_prozent']:+.2f} %)")
    zeilen.append(f"     --mm-per-column      : {spalten:.5f} gesetzt  ->  "
                  f"{ergebnis['mm_per_column_korrigiert']:.5f} laut Messung")

    # --- 2) Linearität ----------------------------------------------------
    rms_spalten = ergebnis["residuum_rms_mm"] / spalten
    zeilen.append("")
    zeilen.append("  2) Linearität (Rest nach Herausrechnen des Maßstabs)")
    zeilen.append(f"     Residuum RMS         : "
                  f"{ergebnis['residuum_rms_mm']:.4f} mm "
                  f"({rms_spalten:.2f} Spalten)")
    zeilen.append(f"     Residuum max         : "
                  f"{ergebnis['residuum_max_mm']:.4f} mm")
    zeilen.append("     je Linie (soll -> gemessen-soll*k):")
    # Über residuen_index gehen, NICHT blind über soll_mm in Reihenfolge:
    # residuen ist kürzer als soll_mm, sobald irgendwo mitten in der Liste
    # eine nicht gemessene Linie steht (nicht nur am Ende) -- ein
    # positionsgleiches zip() würde ab dort jede folgende Zeile mit der
    # falschen Soll-Distanz und der falschen Linien-Nummer beschriften,
    # obwohl "Rest" selbst korrekt berechnet ist. Siehe auswerten()s
    # residuen_index-Kommentar.
    for linie, rest in zip(ergebnis["residuen_index"], ergebnis["residuen"]):
        soll = ergebnis["soll_mm"][linie]
        zeilen.append(f"       Linie {linie:>2}  soll {soll:8.3f} mm   "
                      f"Rest {rest:+8.4f} mm")

    # --- 3) Tintenausbreitung --------------------------------------------
    zeilen.append("")
    zeilen.append("  3) Tintenausbreitung")
    if ergebnis["ausbreitung_mm"] is None:
        zeilen.append("     keine Linienbreite gemessen -- ohne sie lässt "
                      "sich Tinte nicht von Tracking trennen.")
    else:
        zeilen.append(f"     Linienbreite soll    : "
                      f"{ergebnis['linienbreite_soll_mm']:.4f} mm "
                      f"({ergebnis['line_cols']} Spalte(n))")
        zeilen.append(f"     Linienbreite gemessen: "
                      f"{ergebnis['linienbreite_gemessen_mm']:.4f} mm")
        zeilen.append(f"     Ausbreitung          : "
                      f"{ergebnis['ausbreitung_mm']:+.4f} mm "
                      f"({ergebnis['ausbreitung_mm'] / spalten:+.2f} Spalten), "
                      f"davon je Seite "
                      f"{ergebnis['ausbreitung_mm'] / 2:+.4f} mm")

    # --- 4) Auflösung -----------------------------------------------------
    aufl = ergebnis["aufloesung"]
    zeilen.append("")
    zeilen.append("  4) Auflösungsgrenze")
    if aufl["beobachtet_offen_mm"] is None:
        zeilen.append("     nicht angegeben (erste noch getrennte Linie "
                      "fehlt) -- ohne sie keine Aussage zur Auflösung.")
    else:
        geschlossen = aufl["beobachtet_geschlossen_mm"]
        zeilen.append(f"     kleinste offene Lücke: "
                      f"{aufl['beobachtet_offen_mm']:.4f} mm")
        if geschlossen is not None:
            zeilen.append(f"     größte geschlossene  : {geschlossen:.4f} mm")
        else:
            zeilen.append("     größte geschlossene  : keine -- schon die "
                          "kleinste Lücke war offen, die echte Grenze liegt "
                          "darunter und ist mit diesem Muster nicht erfasst.")
        if aufl["erwartet_offen_mm"] is not None:
            zeilen.append(f"     aus Ausbreitung allein zu erwarten: "
                          f"{aufl['erwartet_offen_mm']:.4f} mm")
        if aufl["rest_max_mm"] is not None:
            zeilen.append(f"     Rest auf Position/Timing: "
                          f"{aufl['rest_min_mm']:.4f} .. "
                          f"{aufl['rest_max_mm']:.4f} mm")

    zeilen.append("")
    zeilen.extend(_urteil(ergebnis))
    return "\n".join(zeilen)


def _urteil(ergebnis):
    """Kurzes Fazit mit den Einschränkungen, die dazugehören."""
    zeilen = []
    spalten = ergebnis["mm_per_column"]
    abw = ergebnis["abweichung_prozent"]

    if abs(abw) < 0.5:
        zeilen.append(f"  FAZIT: Maßstab stimmt ({abw:+.2f} %). "
                      f"--mm-per-column braucht keine Korrektur.")
    else:
        zeilen.append(f"  FAZIT: Maßstab weicht um {abw:+.2f} % ab. "
                      f"--mm-per-column "
                      f"{ergebnis['mm_per_column_korrigiert']:.5f} setzen; "
                      f"das ist ein reiner Faktor und vollständig "
                      f"korrigierbar.")

    rest = ergebnis["residuum_rms_mm"]
    if rest > 2.0 * spalten:
        zeilen.append(f"         Nach der Maßstabskorrektur bleiben "
                      f"{rest:.4f} mm RMS ({rest / spalten:.1f} Spalten) "
                      f"übrig. Ein reiner Faktor erklärt die Messung also "
                      f"NICHT -- das ist positionsabhängig (Feldverzerrung, "
                      f"nichtlineares Tracking) und durch Kalibrieren nicht "
                      f"wegzubekommen.")
    else:
        zeilen.append(f"         Der Rest nach der Maßstabskorrektur ist "
                      f"klein ({rest:.4f} mm RMS): die Abweichung ist im "
                      f"Wesentlichen ein Faktor, kein positionsabhängiger "
                      f"Verzug.")

    aufl = ergebnis["aufloesung"]
    if aufl["rest_max_mm"] is not None:
        if aufl["rest_max_mm"] <= 0.0:
            zeilen.append("         Die Auflösungsgrenze wird von der "
                          "Tintenausbreitung allein erklärt -- Tracking und "
                          "Timing sind hier nicht der begrenzende Faktor.")
        else:
            zeilen.append(f"         Über die Tintenausbreitung hinaus "
                          f"bleiben {aufl['rest_min_mm']:.4f} .. "
                          f"{aufl['rest_max_mm']:.4f} mm, die auf "
                          f"Position/Timing gehen.")

    zeilen.append("         Einschränkung: die Lücken verdoppeln sich, die "
                  "Auflösungsgrenze ist damit höchstens auf den Faktor 2 "
                  "genau -- eine Eingrenzung, keine Punktmessung. Für einen "
                  "engeren Wert das Muster mit einem anderen "
                  "--pattern-gap-start erneut drucken.")
    zeilen.append("         Einschränkung: gemessen wird immer die Summe aus "
                  "Tracking, Dosier-Timing, Tintenausbreitung und der "
                  "Genauigkeit der Messung selbst. Ein guter Wert beweist "
                  "gutes Tracking; ein schlechter beweist noch nicht, dass "
                  "das Tracking schuld ist.")
    return zeilen


# ===========================================================================
# Kommandozeile
# ===========================================================================
def _zahlenliste(text):
    """'0,0.18,,0.87' -> [0.0, 0.18, None, 0.87] (leer = nicht gemessen)."""
    werte = []
    for teil in text.split(","):
        teil = teil.strip().replace(",", ".")
        if teil == "":
            werte.append(None)
            continue
        werte.append(float(teil))
    return werte


def _cli(argv):
    ap = argparse.ArgumentParser(
        prog="precision_check_auswertung",
        description="Wertet die am precision-check-Muster gemessenen Werte aus.")
    ap.add_argument("--cli", action="store_true",
                    help="Ohne grafische Oberfläche rechnen (dieser Modus)")
    ap.add_argument("--mm-per-column", type=float, default=0.087,
                    help="Beim Drucken benutzter Wert (Default 0.087)")
    ap.add_argument("--line-cols", type=int, default=1,
                    help="Beim Drucken benutztes --pattern-line-cols")
    ap.add_argument("--gap-start", type=int, default=1,
                    help="Beim Drucken benutztes --pattern-gap-start")
    ap.add_argument("--gemessen", required=True,
                    help="Gemessene Abstände von Linie 0 zu Linie i, in mm, "
                         "durch Komma getrennt. Der erste Wert ist 0. Nicht "
                         "gemessene Linien leer lassen, z.B. '0,0.18,,0.87'")
    ap.add_argument("--linienbreite", type=float, default=None,
                    help="Gemessene Breite EINER gedruckten Linie in mm "
                         "(optional, für die Tintenausbreitung)")
    ap.add_argument("--erste-getrennte", type=int, default=None,
                    help="Index der ersten Linie, deren Lücke davor noch als "
                         "Weiß durchkommt (optional, für die Auflösung)")
    args = ap.parse_args(argv)

    try:
        gemessen = _zahlenliste(args.gemessen)
    except ValueError as fehler:
        print(f"[precision-check] --gemessen konnte nicht gelesen werden: "
              f"{fehler}")
        return 2

    print(bericht(auswerten(
        gemessen, args.mm_per_column, line_cols=args.line_cols,
        gap_start=args.gap_start, linienbreite_mm=args.linienbreite,
        erste_getrennte_index=args.erste_getrennte)))
    return 0


# ===========================================================================
# Grafische Oberfläche
# ===========================================================================
def _gui():
    """
    Startet die grafische Oberfläche. tkinter wird erst hier importiert,
    damit die Rechenfunktionen und der --cli-Modus auch ohne tkinter
    benutzbar bleiben (manche Python-Installationen bringen es nicht mit).
    """
    try:
        import tkinter as tk
        from tkinter import messagebox, ttk
    except ImportError:
        print("[precision-check] tkinter ist nicht verfügbar. Die "
              "Auswertung geht auch ohne grafische Oberfläche:\n"
              "  python funktionen/precision_check_auswertung.py --cli "
              "--gemessen 0,0.18,0.44,... \n"
              "  (python funktionen/precision_check_auswertung.py --help "
              "zeigt alle Optionen)")
        return 1

    ANZAHL_ZEILEN = 12

    class App:
        def __init__(self, root):
            self.root = root
            root.title("precision-check: Auswertung")
            root.geometry("1150x780")
            root.minsize(950, 640)

            self.mm_per_column = tk.StringVar(value="0.087")
            self.line_cols = tk.StringVar(value="1")
            self.gap_start = tk.StringVar(value="1")
            self.linienbreite = tk.StringVar(value="")
            self.erste_getrennte = tk.StringVar(value="")
            self.gemessen_vars = [tk.StringVar() for _ in range(ANZAHL_ZEILEN)]
            self.soll_labels = []

            self._aufbauen(tk, ttk)
            self._messagebox = messagebox
            self.soll_aktualisieren()

        # -------------------------------------------------- Oberfläche
        def _aufbauen(self, tk, ttk):
            self.root.columnconfigure(1, weight=1)
            self.root.rowconfigure(0, weight=1)

            links = ttk.Frame(self.root, padding=12)
            links.grid(row=0, column=0, sticky="ns")

            ttk.Label(links, text="Musterparameter",
                      font=("Segoe UI", 13, "bold")).grid(
                          row=0, column=0, columnspan=2, sticky="w",
                          pady=(0, 8))

            for zeile, (text, var) in enumerate((
                    ("--mm-per-column", self.mm_per_column),
                    ("--pattern-line-cols", self.line_cols),
                    ("--pattern-gap-start", self.gap_start)), start=1):
                ttk.Label(links, text=text).grid(row=zeile, column=0,
                                                 sticky="w", pady=3)
                eingabe = ttk.Entry(links, textvariable=var, width=12)
                eingabe.grid(row=zeile, column=1, sticky="w", pady=3)
                var.trace_add("write", lambda *_: self.soll_aktualisieren())

            ttk.Separator(links, orient="horizontal").grid(
                row=4, column=0, columnspan=2, sticky="ew", pady=12)

            ttk.Label(links, text="Gemessene Abstände von Linie 0 (mm)",
                      font=("Segoe UI", 11, "bold")).grid(
                          row=5, column=0, columnspan=2, sticky="w")
            ttk.Label(links, text="Nicht gemessene Zeilen leer lassen.",
                      wraplength=260).grid(row=6, column=0, columnspan=2,
                                           sticky="w", pady=(0, 6))

            tabelle = ttk.Frame(links)
            tabelle.grid(row=7, column=0, columnspan=2, sticky="ew")
            ttk.Label(tabelle, text="Linie", width=6).grid(row=0, column=0)
            ttk.Label(tabelle, text="Soll (mm)", width=12).grid(row=0, column=1)
            ttk.Label(tabelle, text="Gemessen", width=12).grid(row=0, column=2)

            for i in range(ANZAHL_ZEILEN):
                ttk.Label(tabelle, text=str(i)).grid(row=i + 1, column=0)
                soll = ttk.Label(tabelle, text="-", width=12)
                soll.grid(row=i + 1, column=1)
                self.soll_labels.append(soll)
                eingabe = ttk.Entry(tabelle, textvariable=self.gemessen_vars[i],
                                    width=12)
                eingabe.grid(row=i + 1, column=2, pady=1)
                if i == 0:
                    self.gemessen_vars[i].set("0")

            ttk.Separator(links, orient="horizontal").grid(
                row=8, column=0, columnspan=2, sticky="ew", pady=12)

            ttk.Label(links, text="Linienbreite (mm, optional)").grid(
                row=9, column=0, sticky="w", pady=3)
            ttk.Entry(links, textvariable=self.linienbreite, width=12).grid(
                row=9, column=1, sticky="w", pady=3)

            ttk.Label(links, text="Erste getrennte Linie (optional)").grid(
                row=10, column=0, sticky="w", pady=3)
            ttk.Entry(links, textvariable=self.erste_getrennte, width=12).grid(
                row=10, column=1, sticky="w", pady=3)

            ttk.Button(links, text="Auswerten",
                       command=self.auswerten_klick).grid(
                           row=11, column=0, columnspan=2, sticky="ew",
                           pady=(16, 0))

            rechts = ttk.Frame(self.root, padding=(0, 12, 12, 12))
            rechts.grid(row=0, column=1, sticky="nsew")
            rechts.columnconfigure(0, weight=1)
            rechts.rowconfigure(1, weight=1)

            ttk.Label(rechts, text="Ergebnis",
                      font=("Segoe UI", 13, "bold")).grid(row=0, column=0,
                                                          sticky="w")
            self.ausgabe = tk.Text(rechts, wrap="none",
                                   font=("Consolas", 10))
            self.ausgabe.grid(row=1, column=0, sticky="nsew", pady=(6, 0))
            leiste = ttk.Scrollbar(rechts, orient="vertical",
                                   command=self.ausgabe.yview)
            leiste.grid(row=1, column=1, sticky="ns", pady=(6, 0))
            self.ausgabe.configure(yscrollcommand=leiste.set)

        # -------------------------------------------------- Logik
        def _parameter(self):
            return (float(self.mm_per_column.get().replace(",", ".")),
                    int(float(self.line_cols.get())),
                    int(float(self.gap_start.get())))

        def soll_aktualisieren(self):
            """Soll-Spalte live nachführen, sobald ein Parameter sich ändert."""
            try:
                spalten, line_cols, gap_start = self._parameter()
                if spalten <= 0:
                    raise ValueError
            except (ValueError, TypeError):
                for label in self.soll_labels:
                    label.configure(text="-")
                return
            soll = soll_abstaende_mm(len(self.soll_labels), spalten,
                                     line_cols, gap_start)
            for label, wert in zip(self.soll_labels, soll):
                label.configure(text=f"{wert:.3f}")

        def auswerten_klick(self):
            try:
                spalten, line_cols, gap_start = self._parameter()
            except (ValueError, TypeError):
                self._messagebox.showerror(
                    "Ungültige Eingabe",
                    "Musterparameter müssen Zahlen sein.")
                return

            gemessen = []
            for i, var in enumerate(self.gemessen_vars):
                text = var.get().strip().replace(",", ".")
                if text == "":
                    gemessen.append(None)
                    continue
                try:
                    gemessen.append(float(text))
                except ValueError:
                    self._messagebox.showerror(
                        "Ungültige Eingabe",
                        f"Zeile {i}: '{var.get()}' ist keine Zahl.")
                    return

            # Nach der letzten gemessenen Linie abschneiden: leere Zeilen am
            # Ende sind "nicht gedruckt/nicht gemessen", keine Messlücke.
            while gemessen and gemessen[-1] is None:
                gemessen.pop()

            def optional(var, wandler):
                text = var.get().strip().replace(",", ".")
                if text == "":
                    return None
                try:
                    return wandler(text)
                except ValueError:
                    return None

            ergebnis = auswerten(
                gemessen, spalten, line_cols=line_cols, gap_start=gap_start,
                linienbreite_mm=optional(self.linienbreite, float),
                erste_getrennte_index=optional(self.erste_getrennte,
                                               lambda t: int(float(t))))

            self.ausgabe.delete("1.0", "end")
            self.ausgabe.insert("1.0", bericht(ergebnis))

    root = tk.Tk()
    try:
        stil = ttk.Style()
        themen = stil.theme_names()
        if "vista" in themen:
            stil.theme_use("vista")
        elif "clam" in themen:
            stil.theme_use("clam")
    except tk.TclError:
        pass

    App(root)
    root.mainloop()
    return 0


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv:
        return _cli(argv)
    return _gui()


if __name__ == "__main__":
    sys.exit(main())
