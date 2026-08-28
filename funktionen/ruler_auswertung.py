"""
Auswertung des Testmusters "--pattern ruler"
=============================================

Eigenständiges Werkzeug, analog zu :mod:`precision_check_auswertung`. Wertet
am gedruckten 1/10mm-Maßband GEMESSENE Abstände aus (Major-Ticks, nominell
10mm, Minor-Ticks, nominell 1mm -- siehe ``printhead.patterns.
ruler_ticks_pattern``) und beantwortet drei Fragen:

  1. **Stimmt der Maßstab?** Gemessen gegen das SOLL, das der Code
     tatsächlich erzeugt -- NICHT gegen die nominellen 10mm/1mm direkt: die
     Spaltenrasterung (``minor_step = round(1 / mm_per_column)``) macht
     schon rein rechnerisch eine kleine, unvermeidliche Abweichung, die
     kein Mess- oder Druckfehler ist (siehe ``ruler_ticks_pattern``s
     eigener Docstring für die Herleitung).
  2. **Stimmen Major und Minor UNTEREINANDER überein?** Der Code erzwingt
     ``major_step = minor_step * 10`` EXAKT (kein unabhängiges Runden,
     siehe dort) -- das Verhältnis der gemessenen Mittelwerte muss also
     bei 10,0 liegen, bis auf das, was Messrauschen erklärt. Weicht es
     MEHR ab, als die eigene Streuung der Wiederholmessungen erwarten
     lässt, steckt der Rest nicht in der Musterlogik (die erzwingt das
     Verhältnis exakt), sondern entweder in der Messmethode selbst oder in
     etwas außerhalb dieses Musters (z.B. im Tracking/Timing bei sehr
     kurzen Fahrstrecken).
  3. **Wie präzise ist das Ergebnis?** Standardabweichung je Kategorie.

Erzeugt wird das Muster mit:

    python main.py --pattern ruler --mode line --pattern-length-mm <LAENGE>

Miss auf dem Ausdruck IMMER dieselbe Seite zweier Striche gegeneinander
(z.B. linke Kante zu linker Kante), NICHT die weiße Lücke dazwischen --
sonst schlägt die Tintenbreite der Striche systematisch (und bei den
1mm-Abständen überproportional stark) in die Messung durch. Ein Vergleich
beider Methoden an echten Daten: dieselbe Anlage, dieselbe Fahrt, einmal
Lücke gemessen (Major 6,3% kurz, Minor 11,3% kurz) und einmal Kante-zu-
Kante (Major 1,5% kurz, Minor 6,8% kurz) -- die Kante-zu-Kante-Messung ist
näher am Soll, aber der Rest (Punkt 2 oben) blieb in beiden Fällen fast
identisch, was gegen reine Tintenausbreitung als alleinige Erklärung
spricht.

Diese Datei ist bewusst UNABHÄNGIG vom printhead-Paket (siehe
precision_check_auswertung.py für dieselbe Begründung): sie rechnet das
Soll-Raster selbst nach, statt es zu importieren, damit sie auch für sich
allein kopiert und benutzt werden kann. Ein Test im Repo
(tests/test_ruler_auswertung.py) vergleicht die hiesige Berechnung gegen
printhead.patterns.ruler_ticks_pattern, damit die beiden nicht
auseinanderlaufen.

Benutzung
---------
Mit grafischer Oberfläche (benötigt tkinter):

    python funktionen/ruler_auswertung.py

Ohne GUI, direkt auf der Kommandozeile:

    python funktionen/ruler_auswertung.py --cli --mm-per-column 0.087 \\
        --major 9.79,9.92,9.81,9.82,9.85,9.90,9.96,9.79 \\
        --minor 0.95,0.92,0.91,0.95,0.93,0.96,0.94,0.92,0.98,0.99,...
"""

import argparse
import math
import sys


# ===========================================================================
# Reine Berechnung -- ohne GUI, damit direkt testbar
# ===========================================================================
# Siehe printhead.patterns.py's gleichnamige Konstanten -- muss mit denen
# übereinstimmen, gehalten von test_ruler_auswertung.py.
_MAJOR_EVERY_MM = 10.0
_MINOR_EVERY_MM = 1.0


def soll_schritte(mm_per_column):
    """
    ``(minor_step, major_step)`` in Spalten -- exakt dieselbe Rechnung wie
    ``printhead.patterns.ruler_ticks_pattern``: ``major_step`` ist ein
    EXAKTES Vielfaches von ``minor_step`` (10x), nicht unabhängig
    gerundet, sonst laufen die beiden Raster mit dem Druck auseinander
    (siehe dessen Docstring für das an echter Hardware beobachtete
    Symptom, das diese Konstruktion behebt).
    """
    minor_step = max(1, round(_MINOR_EVERY_MM / mm_per_column))
    major_step = minor_step * round(_MAJOR_EVERY_MM / _MINOR_EVERY_MM)
    return minor_step, major_step


def soll_mm(mm_per_column):
    """``(minor_soll_mm, major_soll_mm)`` -- die tatsächlich vom Code
    erzeugte Distanz, nicht die nominellen 1mm/10mm."""
    minor_step, major_step = soll_schritte(mm_per_column)
    return minor_step * mm_per_column, major_step * mm_per_column


def _mittel(werte):
    return sum(werte) / len(werte) if werte else 0.0


def _stdabw(werte):
    """Stichproben-Standardabweichung (n-1), 0.0 bei weniger als 2 Werten."""
    if len(werte) < 2:
        return 0.0
    m = _mittel(werte)
    return math.sqrt(sum((w - m) ** 2 for w in werte) / (len(werte) - 1))


def massstab_fit(soll_und_gemessen):
    """
    Kleinste-Quadrate-Anpassung ``gemessen = k * soll`` DURCH DEN URSPRUNG,
    über eine gemischte Liste ``[(soll, gemessen), ...]`` -- hier Major- UND
    Minor-Messungen zusammen (siehe ``auswerten``). Größere Soll-Distanzen
    (die Major-Messungen) wiegen dabei automatisch mehr, weil ihre relative
    Messgenauigkeit besser ist -- dieselbe Überlegung wie in
    ``precision_check_auswertung.massstab_fit``.

    Rückgabe: ``k`` oder ``None`` bei zu wenig verwertbaren Paaren.
    """
    nenner = sum(s * s for s, _ in soll_und_gemessen)
    if len(soll_und_gemessen) < 1 or nenner <= 0.0:
        return None
    return sum(s * m for s, m in soll_und_gemessen) / nenner


def auswerten(major_mm, minor_mm, mm_per_column):
    """
    Gesamtauswertung eines ``--pattern ruler``-Ausdrucks.

    ``major_mm``/``minor_mm`` sind Listen gemessener Abstände in mm,
    gleiche Kante zu gleicher Kante (siehe Moduldocstring) -- NICHT die
    weiße Lücke. ``None``-Einträge (nicht gemessen) werden übersprungen.

    Rückgabe: dict mit allen Ergebnissen, oder mit ``fehler``, wenn sich
    nichts auswerten lässt.
    """
    if mm_per_column <= 0:
        return {"fehler": "mm-per-column muss größer als 0 sein."}

    major_mm = [m for m in major_mm if m is not None]
    minor_mm = [m for m in minor_mm if m is not None]
    if not major_mm or not minor_mm:
        return {"fehler": "Mindestens eine Major- und eine Minor-Messung "
                          "werden benötigt."}

    minor_step, major_step = soll_schritte(mm_per_column)
    minor_soll, major_soll = soll_mm(mm_per_column)

    minor_avg, major_avg = _mittel(minor_mm), _mittel(major_mm)
    minor_std, major_std = _stdabw(minor_mm), _stdabw(major_mm)

    # Kombinierter Maßstabsfaktor ueber BEIDE Kategorien zusammen -- die
    # Major-Messungen wiegen automatisch mehr (siehe massstab_fit).
    paare = ([(minor_soll, m) for m in minor_mm]
            + [(major_soll, m) for m in major_mm])
    k = massstab_fit(paare)

    verhaeltnis = major_avg / minor_avg if minor_avg > 0 else None

    # Wie viele Standardfehler liegt das gemessene Verhaeltnis von den
    # exakt geforderten 10,0 entfernt? Fehlerfortpflanzung aus den
    # Standardfehlern der beiden Mittelwerte (unabhaengige Messreihen).
    # Erst ab je 2 Werten pro Kategorie ueberhaupt definiert (_stdabw
    # braucht das schon).
    sem_verhaeltnis = None
    abweichung_sigmas = None
    if (len(minor_mm) >= 2 and len(major_mm) >= 2
            and minor_avg > 0 and major_avg > 0 and verhaeltnis is not None):
        sem_minor = minor_std / math.sqrt(len(minor_mm))
        sem_major = major_std / math.sqrt(len(major_mm))
        sem_verhaeltnis = verhaeltnis * math.sqrt(
            (sem_major / major_avg) ** 2 + (sem_minor / minor_avg) ** 2)
        if sem_verhaeltnis > 0:
            abweichung_sigmas = abs(verhaeltnis - 10.0) / sem_verhaeltnis

    return {
        "mm_per_column": mm_per_column,
        "minor_step_spalten": minor_step,
        "major_step_spalten": major_step,
        "minor_soll_mm": minor_soll,
        "major_soll_mm": major_soll,
        "minor_n": len(minor_mm),
        "major_n": len(major_mm),
        "minor_avg_mm": minor_avg,
        "major_avg_mm": major_avg,
        "minor_std_mm": minor_std,
        "major_std_mm": major_std,
        "minor_min_mm": min(minor_mm),
        "minor_max_mm": max(minor_mm),
        "major_min_mm": min(major_mm),
        "major_max_mm": max(major_mm),
        "massstab": k,
        "mm_per_column_korrigiert": mm_per_column * k if k else None,
        "abweichung_prozent": (k - 1.0) * 100.0 if k else None,
        "verhaeltnis_major_minor": verhaeltnis,
        "verhaeltnis_sem": sem_verhaeltnis,
        "verhaeltnis_abweichung_sigmas": abweichung_sigmas,
    }


# ===========================================================================
# Bericht
# ===========================================================================
# Schwelle für "das Verhältnis weicht signifikant ab" -- 3 Standardfehler
# ist die in der Messtechnik übliche Faustregel für "kein Zufall mehr"
# (bei normalverteiltem Rauschen liegt eine echte Punktmessung nur zu
# ~0,3% der Fälle zufällig so weit weg).
_SIGNIFIKANZ_SCHWELLE_SIGMA = 3.0


def bericht(ergebnis):
    """Formatiert das Ergebnis von :func:`auswerten` als lesbaren Text."""
    if "fehler" in ergebnis:
        return f"[ruler] {ergebnis['fehler']}"

    spalten = ergebnis["mm_per_column"]
    zeilen = []
    zeilen.append("---- --pattern ruler: Auswertung ----")
    zeilen.append(f"  --mm-per-column {spalten:g}  ->  Soll-Raster: "
                  f"Minor {ergebnis['minor_step_spalten']} Spalten "
                  f"({ergebnis['minor_soll_mm']:.4f} mm), "
                  f"Major {ergebnis['major_step_spalten']} Spalten "
                  f"({ergebnis['major_soll_mm']:.4f} mm)")
    zeilen.append("")

    # --- 1) Maßstab -------------------------------------------------------
    zeilen.append("  1) Maßstab (gegen das Soll-Raster, nicht gegen 10mm/1mm direkt)")
    if ergebnis["massstab"] is None:
        zeilen.append("     nicht bestimmbar.")
    else:
        zeilen.append(f"     Faktor gemessen/soll : {ergebnis['massstab']:.5f}  "
                      f"({ergebnis['abweichung_prozent']:+.2f} %)")
        zeilen.append(f"     --mm-per-column      : {spalten:.5f} gesetzt  ->  "
                      f"{ergebnis['mm_per_column_korrigiert']:.5f} laut Messung")

    # --- 2) Major/Minor-Konsistenz -----------------------------------------
    zeilen.append("")
    zeilen.append("  2) Major/Minor-Verhältnis (Soll: exakt 10,0 -- siehe "
                  "ruler_ticks_pattern)")
    zeilen.append(f"     Minor: n={ergebnis['minor_n']:2d}  "
                  f"{ergebnis['minor_avg_mm']:.4f} mm  "
                  f"± {ergebnis['minor_std_mm']:.4f} mm  "
                  f"[{ergebnis['minor_min_mm']:.4f} .. {ergebnis['minor_max_mm']:.4f}]")
    zeilen.append(f"     Major: n={ergebnis['major_n']:2d}  "
                  f"{ergebnis['major_avg_mm']:.4f} mm  "
                  f"± {ergebnis['major_std_mm']:.4f} mm  "
                  f"[{ergebnis['major_min_mm']:.4f} .. {ergebnis['major_max_mm']:.4f}]")
    if ergebnis["verhaeltnis_major_minor"] is not None:
        zeile = f"     Verhältnis Major/Minor: {ergebnis['verhaeltnis_major_minor']:.3f}"
        if ergebnis["verhaeltnis_abweichung_sigmas"] is not None:
            zeile += (f"  (weicht um {ergebnis['verhaeltnis_abweichung_sigmas']:.1f} "
                      f"Standardfehler von 10,0 ab)")
        zeilen.append(zeile)

    zeilen.append("")
    zeilen.extend(_urteil(ergebnis))
    return "\n".join(zeilen)


def _urteil(ergebnis):
    """Kurzes Fazit mit den Einschränkungen, die dazugehören."""
    zeilen = []

    if ergebnis["massstab"] is not None:
        abw = ergebnis["abweichung_prozent"]
        if abs(abw) < 1.0:
            zeilen.append(f"  FAZIT: Maßstab stimmt mit dem Soll-Raster überein "
                          f"({abw:+.2f} %).")
        else:
            zeilen.append(f"  FAZIT: Maßstab weicht um {abw:+.2f} % vom "
                          f"Soll-Raster ab. --mm-per-column "
                          f"{ergebnis['mm_per_column_korrigiert']:.5f} setzen, "
                          f"falls das kein Einzelausreißer ist.")

    sigmas = ergebnis["verhaeltnis_abweichung_sigmas"]
    if sigmas is not None:
        if sigmas >= _SIGNIFIKANZ_SCHWELLE_SIGMA:
            zeilen.append(
                f"         Das Major/Minor-Verhältnis "
                f"({ergebnis['verhaeltnis_major_minor']:.3f} statt 10,0) weicht "
                f"um {sigmas:.1f} Standardfehler ab -- MEHR, als die eigene "
                f"Streuung der Wiederholmessungen erwarten lässt "
                f"(Schwelle: {_SIGNIFIKANZ_SCHWELLE_SIGMA:g}). Die Musterlogik "
                f"selbst erzwingt das Verhältnis exakt (major_step = "
                f"minor_step * 10, keine unabhängige Rundung) -- der Rest "
                f"liegt also entweder an einem Effekt, der spezifisch für "
                f"die Messmethode ist (z.B. Anlegedruck/Ablesefehler, der "
                f"nicht rein zufällig zwischen Wiederholungen streut), oder "
                f"an etwas außerhalb dieses Musters. Zum Eingrenzen: "
                f"denselben Ausdruck mit einer unabhängigen Methode "
                f"nachmessen (kalibriertes Foto, Mikroskop) -- zeigt die "
                f"auch ein Verhältnis nahe 10,5-10,6 statt 10,0, ist es kein "
                f"Messschieber-Artefakt mehr.")
        else:
            zeilen.append(
                f"         Das Major/Minor-Verhältnis "
                f"({ergebnis['verhaeltnis_major_minor']:.3f}) liegt im Rahmen "
                f"dessen, was die beobachtete Messstreuung erwarten lässt -- "
                f"kein Hinweis auf einen Effekt jenseits von Messrauschen.")
    else:
        zeilen.append("         Für die Signifikanzprüfung des Verhältnisses "
                      "werden mindestens 2 Werte je Kategorie gebraucht.")

    zeilen.append("         Einschränkung: gemessen wird immer die Summe aus "
                  "Tracking, Dosier-Timing, Tintenausbreitung und der "
                  "Genauigkeit der Messung selbst. Ein guter Wert beweist "
                  "gutes Tracking; ein schlechter beweist noch nicht, dass "
                  "das Tracking schuld ist.")
    zeilen.append("         Einschränkung: immer dieselbe Kante gegen "
                  "dieselbe Kante messen (nicht die weiße Lücke) -- sonst "
                  "schlägt die Tintenbreite der Striche in die Messung "
                  "durch, bei den 1mm-Abständen überproportional stark.")
    return zeilen


# ===========================================================================
# Kommandozeile
# ===========================================================================
def _zahlenliste(text):
    """'9.79,9.92,,9.81' -> [9.79, 9.92, None, 9.81] (leer = nicht gemessen)."""
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
        prog="ruler_auswertung",
        description="Wertet die am --pattern ruler-Ausdruck gemessenen "
                    "Major-/Minor-Abstände aus.")
    ap.add_argument("--cli", action="store_true",
                    help="Ohne grafische Oberfläche rechnen (dieser Modus)")
    ap.add_argument("--mm-per-column", type=float, default=0.087,
                    help="Beim Drucken benutzter Wert (Default 0.087)")
    ap.add_argument("--major", required=True,
                    help="Gemessene Major-Abstände (nominell 10mm), in mm, "
                        "durch Komma getrennt, gleiche Kante zu gleicher "
                        "Kante gemessen -- nicht die weiße Lücke")
    ap.add_argument("--minor", required=True,
                    help="Gemessene Minor-Abstände (nominell 1mm), in mm, "
                        "durch Komma getrennt, gleiche Kante zu gleicher "
                        "Kante gemessen")
    args = ap.parse_args(argv)

    try:
        major = _zahlenliste(args.major)
        minor = _zahlenliste(args.minor)
    except ValueError as fehler:
        print(f"[ruler] --major/--minor konnten nicht gelesen werden: {fehler}")
        return 2

    print(bericht(auswerten(major, minor, args.mm_per_column)))
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
        print("[ruler] tkinter ist nicht verfügbar. Die Auswertung geht "
              "auch ohne grafische Oberfläche:\n"
              "  python funktionen/ruler_auswertung.py --cli "
              "--major 9.79,9.92,... --minor 0.95,0.92,...\n"
              "  (python funktionen/ruler_auswertung.py --help zeigt alle "
              "Optionen)")
        return 1

    class App:
        def __init__(self, root):
            self.root = root
            root.title("--pattern ruler: Auswertung")
            root.geometry("900x680")
            root.minsize(760, 560)

            self.mm_per_column = tk.StringVar(value="0.087")

            self._aufbauen(tk, ttk)
            self._messagebox = messagebox

        def _aufbauen(self, tk, ttk):
            self.root.columnconfigure(0, weight=1)
            self.root.rowconfigure(3, weight=1)

            kopf = ttk.Frame(self.root, padding=12)
            kopf.grid(row=0, column=0, sticky="ew")
            ttk.Label(kopf, text="--mm-per-column").grid(row=0, column=0,
                                                          sticky="w")
            ttk.Entry(kopf, textvariable=self.mm_per_column, width=12).grid(
                row=0, column=1, sticky="w", padx=(6, 0))

            eingabe = ttk.Frame(self.root, padding=(12, 0, 12, 12))
            eingabe.grid(row=1, column=0, sticky="ew")
            eingabe.columnconfigure(0, weight=1)

            ttk.Label(eingabe, text="Major-Abstände (nominell 10mm), "
                                    "komma-getrennt, gleiche Kante zu "
                                    "gleicher Kante").grid(row=0, column=0,
                                                           sticky="w")
            self.major_text = tk.Text(eingabe, height=3, font=("Consolas", 10))
            self.major_text.grid(row=1, column=0, sticky="ew", pady=(2, 10))

            ttk.Label(eingabe, text="Minor-Abstände (nominell 1mm), "
                                    "komma-getrennt").grid(row=2, column=0,
                                                           sticky="w")
            self.minor_text = tk.Text(eingabe, height=4, font=("Consolas", 10))
            self.minor_text.grid(row=3, column=0, sticky="ew", pady=(2, 0))

            ttk.Button(self.root, text="Auswerten",
                      command=self.auswerten_klick).grid(
                          row=2, column=0, sticky="w", padx=12, pady=(0, 8))

            ergebnis = ttk.Frame(self.root, padding=(12, 0, 12, 12))
            ergebnis.grid(row=3, column=0, sticky="nsew")
            ergebnis.columnconfigure(0, weight=1)
            ergebnis.rowconfigure(0, weight=1)
            self.ausgabe = tk.Text(ergebnis, wrap="none", font=("Consolas", 10))
            self.ausgabe.grid(row=0, column=0, sticky="nsew")
            leiste = ttk.Scrollbar(ergebnis, orient="vertical",
                                   command=self.ausgabe.yview)
            leiste.grid(row=0, column=1, sticky="ns")
            self.ausgabe.configure(yscrollcommand=leiste.set)

        def auswerten_klick(self):
            try:
                spalten = float(self.mm_per_column.get().replace(",", "."))
            except ValueError:
                self._messagebox.showerror("Ungültige Eingabe",
                                           "--mm-per-column muss eine Zahl sein.")
                return
            try:
                major = _zahlenliste(self.major_text.get("1.0", "end"))
                minor = _zahlenliste(self.minor_text.get("1.0", "end"))
            except ValueError as fehler:
                self._messagebox.showerror("Ungültige Eingabe", str(fehler))
                return

            ergebnis = auswerten(major, minor, spalten)
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
