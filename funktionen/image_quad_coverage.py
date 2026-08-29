"""
Flächendeckung in einem frei gezogenen Viereck
================================================

GUI-Teil analog zu image_line_to_angle.py: Eckpunkte werden mit der Maus
gesetzt und lassen sich danach frei verschieben -- "flexible Ecken" statt
eines starren, achsenparallelen Rechtecks, damit auch ein in der
Fotografie perspektivisch verzerrtes Quadrat (Kamerawinkel, Drehung)
sauber umrissen werden kann.

Beantwortet EINE Frage: wie viel Prozent der Fläche innerhalb des
Vierecks sind schwarz, wie viel weiß? Das Bild wird nach Graustufen
konvertiert (PIL, Modus "L") und jedes Pixel gegen einen Schwellenwert
verglichen (Default 128): Grauwert < Schwelle zählt als schwarz, sonst
als weiß -- eine binäre Aufteilung, kein dritter Grau-Eimer, wie
angefragt.

Die vier Eckpunkte müssen der Reihe nach entlang des Randes gesetzt
werden (im oder gegen den Uhrzeigersinn) -- NICHT über Kreuz, sonst
füllt die Polygon-Rasterung eine Schmetterlingsform statt der gemeinten
Fläche.

Wie bei precision_check_auswertung.py/ruler_auswertung.py ist diese
Datei bewusst in CLI- und GUI-Teil getrennt: tkinter (und PIL.ImageTk,
das intern tkinter braucht) wird erst in _gui() importiert, damit die
reine Rechnung (berechne_deckung, render_export_bild, ...) auch ohne
tkinter/Bildschirm lauffähig und automatisiert testbar bleibt --
image_line_to_angle.py selbst hat diese Trennung nicht (dort wird
tkinter oben importiert), diese Datei folgt stattdessen der Konvention
der *_auswertung.py-Werkzeuge, damit sie eine echte Testdatei bekommen
kann.

Benutzung
---------
Mit grafischer Oberfläche (benötigt tkinter):

    python funktionen/image_quad_coverage.py

Ohne GUI, direkt auf der Kommandozeile:

    python funktionen/image_quad_coverage.py --cli --bild foto.png \\
        --ecken "40,40;860,60;840,760;60,740" --schwelle 128 \\
        --export ergebnis.png
"""

import argparse
import math
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


# ===========================================================================
# Reine Berechnung -- ohne GUI, damit direkt testbar
# ===========================================================================
DEFAULT_SCHWELLE = 128
QUAD_FARBE = "#00c853"


def berechne_deckung(image, ecken, schwelle=DEFAULT_SCHWELLE):
    """
    Anteil schwarzer/weißer Pixel innerhalb des Vierecks ``ecken`` (vier
    ``(x, y)``-Punkte in Bildkoordinaten, der Reihe nach entlang des
    Randes -- siehe Moduldocstring).

    Grauwert < ``schwelle`` zählt als schwarz, sonst als weiß.
    ``schwelle`` wird auf [0, 255] geklemmt (bei 128 zählen also die
    Grauwerte 0..127 als schwarz, 128..255 als weiß).

    Rückgabe: dict mit ``pixel_gesamt``/``pixel_schwarz``/``pixel_weiss``/
    ``schwarz_prozent``/``weiss_prozent``/``schwelle``, oder mit
    ``fehler`` bei ungültiger Eingabe.
    """
    if len(ecken) != 4:
        return {"fehler": f"Es werden genau 4 Eckpunkte benötigt "
                          f"(erhalten: {len(ecken)})."}

    schwelle = max(0, min(255, int(round(schwelle))))

    maske = Image.new("1", image.size, 0)
    ImageDraw.Draw(maske).polygon([tuple(p) for p in ecken], fill=1)

    grau = image.convert("L")
    histogramm = grau.histogram(mask=maske)

    pixel_gesamt = sum(histogramm)
    if pixel_gesamt == 0:
        return {"fehler": "Das Viereck umschließt keine Bildfläche "
                          "(zu klein, außerhalb des Bildes oder "
                          "entartet)."}

    pixel_schwarz = sum(histogramm[:schwelle])
    pixel_weiss = pixel_gesamt - pixel_schwarz

    return {
        "pixel_gesamt": pixel_gesamt,
        "pixel_schwarz": pixel_schwarz,
        "pixel_weiss": pixel_weiss,
        "schwelle": schwelle,
        "schwarz_prozent": pixel_schwarz / pixel_gesamt * 100.0,
        "weiss_prozent": pixel_weiss / pixel_gesamt * 100.0,
    }


def bericht(ergebnis):
    """Formatiert das Ergebnis von :func:`berechne_deckung` als Text."""
    if "fehler" in ergebnis:
        return f"[quad-deckung] {ergebnis['fehler']}"

    zeilen = ["---- Viereck: Schwarz/Weiß-Deckung ----"]
    zeilen.append(f"  Schwellenwert        : {ergebnis['schwelle']} "
                  f"(Grauwert < Schwelle = schwarz, sonst weiß; 0..255)")
    zeilen.append(f"  Pixel im Viereck      : {ergebnis['pixel_gesamt']}")
    zeilen.append(f"  Schwarz               : {ergebnis['pixel_schwarz']} "
                  f"Pixel  ({ergebnis['schwarz_prozent']:.2f} %)")
    zeilen.append(f"  Weiß                  : {ergebnis['pixel_weiss']} "
                  f"Pixel  ({ergebnis['weiss_prozent']:.2f} %)")
    return "\n".join(zeilen)


# ===========================================================================
# Export -- ebenfalls reine Bildbearbeitung, kein tkinter
# ===========================================================================
def suggest_export_dateiname(bildpfad):
    if bildpfad is None:
        return "deckung_export.png"
    return f"{Path(bildpfad).stem}_deckung.png"


def lade_export_schrift(referenzgroesse):
    schriftgroesse = max(16, round(referenzgroesse * 0.03))

    # Versucht zuerst gebräuchliche, auf dem System vorhandene
    # Schriftarten -- funktioniert unter Windows (arialbd.ttf/Arial
    # Bold.ttf) genauso wie unter Linux (DejaVuSans-Bold.ttf) ohne
    # zusätzliche Abhängigkeiten.
    for name in ("DejaVuSans-Bold.ttf", "arialbd.ttf", "Arial Bold.ttf",
                "Arial.ttf"):
        try:
            return ImageFont.truetype(name, schriftgroesse)
        except OSError:
            continue

    # Fallback: Pillows eigene Standardschrift. Das size-Argument gibt es
    # erst ab Pillow 10.1 -- ohne es bleibt die Schrift klein, aber
    # lesbar, statt den Export scheitern zu lassen.
    try:
        return ImageFont.load_default(size=schriftgroesse)
    except TypeError:
        return ImageFont.load_default()


def zeichne_text_mit_hintergrund(draw, position, text, schrift):
    x, y = position
    box = draw.textbbox((x, y), text, font=schrift)
    polster = 8
    draw.rectangle(
        [box[0] - polster, box[1] - polster,
         box[2] + polster, box[3] + polster],
        fill="#202020", outline=QUAD_FARBE, width=2)
    draw.text(position, text, fill="white", font=schrift)


def render_export_bild(image, ecken, ergebnis):
    """
    Zeichnet den Umriss des Vierecks (geschlossen, sobald 4 Eckpunkte
    vorliegen) auf eine RGB-Kopie von ``image``. Ist ``ergebnis`` eine
    erfolgreiche Auswertung (kein ``fehler``-Schlüssel), werden
    zusätzlich die Schwarz-/Weiß-Prozente oben links eingeblendet.
    """
    export = image.convert("RGB").copy()
    draw = ImageDraw.Draw(export)

    referenz = min(export.width, export.height)
    linienbreite = max(2, round(referenz * 0.004))
    punktradius = max(4, round(referenz * 0.006))

    if len(ecken) >= 2:
        umriss = list(ecken)
        if len(ecken) == 4:
            umriss = umriss + [umriss[0]]
        draw.line([tuple(p) for p in umriss], fill=QUAD_FARBE,
                  width=linienbreite, joint="curve")

    for punkt in ecken:
        x, y = punkt
        draw.ellipse(
            [x - punktradius, y - punktradius,
             x + punktradius, y + punktradius],
            fill=QUAD_FARBE, outline="white",
            width=max(1, linienbreite // 2))

    if ergebnis and "fehler" not in ergebnis:
        text = (f"Schwarz: {ergebnis['schwarz_prozent']:.1f} %   "
               f"Weiß: {ergebnis['weiss_prozent']:.1f} %   "
               f"(Schwelle {ergebnis['schwelle']})")
        schrift = lade_export_schrift(referenz)
        rand = round(referenz * 0.02)
        zeichne_text_mit_hintergrund(draw, (rand, rand), text, schrift)

    return export


# ===========================================================================
# Kommandozeile
# ===========================================================================
def _parse_ecken(text):
    """
    ``'40,40;860,60;840,760;60,740'`` -> ``[(40.0, 40.0), (860.0, 60.0),
    ...]``.

    Punkte werden durch ``;`` getrennt, x und y je Punkt durch ``,``.
    Anders als bei den zahlenlisten-Feldern der anderen Werkzeuge dieses
    Repos ist das Dezimaltrennzeichen hier bewusst NUR der Punkt (kein
    Komma-als-Dezimaltrennzeichen), weil das Komma hier schon x von y
    trennt.
    """
    punkte = []
    for teil in text.split(";"):
        teil = teil.strip()
        if not teil:
            continue
        stuecke = teil.split(",")
        if len(stuecke) != 2:
            raise ValueError(f"Eckpunkt {teil!r} ist nicht im Format 'x,y'.")
        punkte.append((float(stuecke[0]), float(stuecke[1])))
    return punkte


def _cli(argv):
    ap = argparse.ArgumentParser(
        prog="image_quad_coverage",
        description="Schwarz/Weiß-Deckung innerhalb eines frei gezogenen "
                    "Vierecks auf einem Bild.")
    ap.add_argument("--cli", action="store_true",
                    help="Ohne grafische Oberfläche rechnen (dieser Modus)")
    ap.add_argument("--bild", required=True, help="Pfad zum Bild")
    ap.add_argument("--ecken", required=True,
                    help="Vier Eckpunkte, der Reihe nach entlang des "
                        "Randes, z.B. '40,40;860,60;840,760;60,740' "
                        "(';' zwischen Punkten, ',' zwischen x und y, "
                        "Dezimalpunkt)")
    ap.add_argument("--schwelle", type=int, default=DEFAULT_SCHWELLE,
                    help=f"Schwellenwert 0..255 (Default {DEFAULT_SCHWELLE})")
    ap.add_argument("--export", default=None,
                    help="Optional: Ergebnisbild (Viereck + Prozente) "
                        "unter diesem Pfad speichern")
    args = ap.parse_args(argv)

    try:
        image = Image.open(args.bild)
        image.load()
    except OSError as fehler:
        print(f"[quad-deckung] Bild konnte nicht geladen werden: {fehler}")
        return 2

    try:
        ecken = _parse_ecken(args.ecken)
    except ValueError as fehler:
        print(f"[quad-deckung] --ecken konnte nicht gelesen werden: {fehler}")
        return 2

    ergebnis = berechne_deckung(image, ecken, args.schwelle)
    print(bericht(ergebnis))

    if args.export:
        try:
            export = render_export_bild(image, ecken, ergebnis)
            export.save(args.export)
            print(f"\n  Export gespeichert: {args.export}")
        except OSError as fehler:
            print(f"\n[quad-deckung] Export konnte nicht gespeichert "
                  f"werden: {fehler}")
            return 2

    return 0 if "fehler" not in ergebnis else 2


# ===========================================================================
# Grafische Oberfläche
# ===========================================================================
def _gui():
    """
    Startet die grafische Oberfläche. tkinter (und PIL.ImageTk) wird
    erst hier importiert, damit die Rechenfunktionen und der --cli-Modus
    auch ohne tkinter benutzbar und testbar bleiben (siehe
    Moduldocstring).
    """
    try:
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk
        from PIL import ImageTk
    except ImportError:
        print("[quad-deckung] tkinter ist nicht verfügbar. Die Auswertung "
              "geht auch ohne grafische Oberfläche:\n"
              "  python funktionen/image_quad_coverage.py --cli --bild "
              "foto.png --ecken \"40,40;860,60;840,760;60,740\"\n"
              "  (python funktionen/image_quad_coverage.py --help zeigt "
              "alle Optionen)")
        return 1

    class App:
        POINT_RADIUS = 7
        POINT_HIT_RADIUS = 14

        def __init__(self, root):
            self.root = root
            root.title("Bildmessung: Flächendeckung (Viereck)")
            root.geometry("1250x800")
            root.minsize(900, 600)

            self.original_image = None
            self.display_image = None
            self.photo_image = None
            self.image_path = None

            self.scale = 1.0
            self.image_offset_x = 0.0
            self.image_offset_y = 0.0

            # Bis zu 4 Eckpunkte, in Koordinaten des Originalbildes, der
            # Reihe nach entlang des Randes.
            self.corners = []
            self.dragged_point = None
            self._letztes_ergebnis = None

            self.schwelle_var = tk.StringVar(value=str(DEFAULT_SCHWELLE))
            self.schwarz_var = tk.StringVar(value="Nicht definiert")
            self.weiss_var = tk.StringVar(value="Nicht definiert")
            self.pixel_var = tk.StringVar(value="Nicht definiert")
            self.status_var = tk.StringVar(
                value="Laden Sie ein Bild, um mit der Messung zu beginnen.")
            self.instruction_var = tk.StringVar(value="Bild laden")

            self._aufbauen()
            self._ereignisse_binden()

        # ---- Aufbau --------------------------------------------------
        def _aufbauen(self):
            self.root.columnconfigure(0, weight=1)
            self.root.rowconfigure(1, weight=1)

            toolbar = ttk.Frame(self.root, padding=(10, 8))
            toolbar.grid(row=0, column=0, sticky="ew")
            toolbar.columnconfigure(9, weight=1)

            ttk.Button(toolbar, text="Bild laden",
                      command=self.bild_laden).grid(row=0, column=0, padx=(0, 6))
            ttk.Button(toolbar, text="Letzten Punkt löschen",
                      command=self.letzten_punkt_loeschen).grid(row=0, column=1, padx=6)
            ttk.Button(toolbar, text="Viereck zurücksetzen",
                      command=self.viereck_zuruecksetzen).grid(row=0, column=2, padx=6)

            ttk.Separator(toolbar, orient="vertical").grid(
                row=0, column=3, sticky="ns", padx=10)

            ttk.Button(toolbar, text="Ansicht einpassen",
                      command=self.ansicht_einpassen).grid(row=0, column=4, padx=6)
            ttk.Button(toolbar, text="Vergrößern",
                      command=lambda: self.zoom_aendern(1.2)).grid(row=0, column=5, padx=6)
            ttk.Button(toolbar, text="Verkleinern",
                      command=lambda: self.zoom_aendern(1 / 1.2)).grid(row=0, column=6, padx=6)

            ttk.Separator(toolbar, orient="vertical").grid(
                row=0, column=7, sticky="ns", padx=10)

            ttk.Button(toolbar, text="Export",
                      command=self.export_ergebnisbild).grid(
                          row=0, column=8, sticky="w", padx=6)

            main_frame = ttk.Frame(self.root, padding=(10, 0, 10, 10))
            main_frame.grid(row=1, column=0, sticky="nsew")
            main_frame.columnconfigure(0, weight=1)
            main_frame.rowconfigure(0, weight=1)

            canvas_frame = ttk.Frame(main_frame)
            canvas_frame.grid(row=0, column=0, sticky="nsew")
            canvas_frame.columnconfigure(0, weight=1)
            canvas_frame.rowconfigure(0, weight=1)

            self.canvas = tk.Canvas(
                canvas_frame, background="#2b2b2b", highlightthickness=1,
                highlightbackground="#707070", cursor="crosshair")
            self.canvas.grid(row=0, column=0, sticky="nsew")

            hbar = ttk.Scrollbar(canvas_frame, orient="horizontal",
                                 command=self.canvas.xview)
            hbar.grid(row=1, column=0, sticky="ew")
            vbar = ttk.Scrollbar(canvas_frame, orient="vertical",
                                 command=self.canvas.yview)
            vbar.grid(row=0, column=1, sticky="ns")
            self.canvas.configure(xscrollcommand=hbar.set, yscrollcommand=vbar.set)

            side = ttk.Frame(main_frame, padding=(15, 5))
            side.grid(row=0, column=1, sticky="ns")

            ttk.Label(side, text="Messwerte", font=("Segoe UI", 16, "bold")).grid(
                row=0, column=0, sticky="w", pady=(0, 15))

            self._ergebnis_box(side, row=1, titel="Schwarz",
                              variable=self.schwarz_var, farbe="#202020")
            self._ergebnis_box(side, row=2, titel="Weiß",
                              variable=self.weiss_var, farbe="#e0e0e0")
            self._ergebnis_box(side, row=3, titel="Pixel im Viereck",
                              variable=self.pixel_var, farbe=QUAD_FARBE)

            ttk.Separator(side, orient="horizontal").grid(
                row=4, column=0, sticky="ew", pady=15)

            ttk.Label(side, text="Schwellenwert (0..255)",
                     font=("Segoe UI", 10, "bold")).grid(row=5, column=0, sticky="w")
            schwelle_zeile = ttk.Frame(side)
            schwelle_zeile.grid(row=6, column=0, sticky="w", pady=(3, 15))
            schwelle_eingabe = ttk.Entry(schwelle_zeile, textvariable=self.schwelle_var,
                                        width=8)
            schwelle_eingabe.grid(row=0, column=0)
            schwelle_eingabe.bind("<Return>", lambda e: self.messwerte_aktualisieren())
            schwelle_eingabe.bind("<FocusOut>", lambda e: self.messwerte_aktualisieren())
            ttk.Label(schwelle_zeile,
                     text=" Grauwert < Schwelle = schwarz").grid(
                         row=0, column=1, padx=(6, 0))

            ttk.Label(side, text="Aktueller Schritt",
                     font=("Segoe UI", 10, "bold")).grid(row=7, column=0, sticky="w")
            ttk.Label(side, textvariable=self.instruction_var, wraplength=250,
                     foreground="#0067c0").grid(row=8, column=0, sticky="w", pady=(3, 15))

            ttk.Label(side, text=(
                "Bedienung\n\n"
                "• Linksklick setzt der Reihe nach die vier Eckpunkte "
                "des Vierecks (entlang des Randes, nicht über Kreuz).\n"
                "• Ziehen eines Punktes verschiebt ihn.\n"
                "• Mausrad ändert die Vergrößerung.\n"
                "• Grauwert < Schwellenwert zählt als schwarz, sonst "
                "als weiß.\n"
                "• Export speichert Viereck + Prozente auf einer Kopie "
                "des Originalbildes -- ohne weitere Werte."
            ), justify="left", wraplength=270).grid(row=9, column=0, sticky="nw")

            status = ttk.Label(self.root, textvariable=self.status_var, anchor="w",
                               relief="sunken", padding=(8, 4))
            status.grid(row=2, column=0, sticky="ew")

        def _ergebnis_box(self, parent, row, titel, variable, farbe):
            frame = ttk.LabelFrame(parent, text=titel, padding=10)
            frame.grid(row=row, column=0, sticky="ew", pady=5)
            marker = tk.Canvas(frame, width=18, height=18, highlightthickness=0)
            marker.grid(row=0, column=0, padx=(0, 8))
            marker.create_oval(2, 2, 16, 16, fill=farbe, outline=farbe)
            ttk.Label(frame, textvariable=variable,
                     font=("Segoe UI", 12, "bold")).grid(row=0, column=1, sticky="w")

        def _ereignisse_binden(self):
            self.canvas.bind("<Button-1>", self._maus_runter)
            self.canvas.bind("<B1-Motion>", self._maus_ziehen)
            self.canvas.bind("<ButtonRelease-1>", self._maus_los)
            self.canvas.bind("<Configure>", self._canvas_groesse_geaendert)
            self.canvas.bind("<MouseWheel>", self._mausrad)
            self.canvas.bind("<Button-4>", lambda e: self.zoom_aendern(1.1))
            self.canvas.bind("<Button-5>", lambda e: self.zoom_aendern(1 / 1.1))
            self.root.bind("<Control-o>", lambda e: self.bild_laden())
            self.root.bind("<Control-z>", lambda e: self.letzten_punkt_loeschen())
            self.root.bind("<Escape>", lambda e: self.viereck_zuruecksetzen())

        # ---- Bild laden / Ansicht -------------------------------------
        def bild_laden(self):
            pfad = filedialog.askopenfilename(
                title="Bild auswählen",
                filetypes=[("Bilddateien",
                          "*.png *.jpg *.jpeg *.bmp *.tif *.tiff *.webp"),
                          ("PNG-Dateien", "*.png"), ("JPEG-Dateien", "*.jpg *.jpeg"),
                          ("Alle Dateien", "*.*")])
            if not pfad:
                return
            try:
                bild = Image.open(pfad)
                bild.load()
                if bild.mode not in ("RGB", "RGBA"):
                    bild = bild.convert("RGB")

                self.original_image = bild
                self.image_path = Path(pfad)
                self.viereck_zuruecksetzen(neu_zeichnen=False)

                self.root.update_idletasks()
                self.ansicht_einpassen()

                self.status_var.set(
                    f"Bild geladen: {self.image_path.name} "
                    f"({bild.width} × {bild.height} Pixel)")
                self._anleitung_aktualisieren()

            except Exception as fehler:
                messagebox.showerror(
                    "Fehler beim Laden",
                    f"Das Bild konnte nicht geladen werden.\n\n{fehler}")

        def ansicht_einpassen(self):
            if self.original_image is None:
                return
            self.root.update_idletasks()
            cw = max(self.canvas.winfo_width(), 100)
            ch = max(self.canvas.winfo_height(), 100)
            rand = 30
            sx = (cw - 2 * rand) / self.original_image.width
            sy = (ch - 2 * rand) / self.original_image.height
            self.scale = max(min(min(sx, sy), 10.0), 0.02)
            dw = self.original_image.width * self.scale
            dh = self.original_image.height * self.scale
            self.image_offset_x = max((cw - dw) / 2, 0)
            self.image_offset_y = max((ch - dh) / 2, 0)
            self._neu_zeichnen()

        def zoom_aendern(self, faktor):
            if self.original_image is None:
                return
            alt = self.scale
            neu = max(0.02, min(alt * faktor, 20.0))
            if abs(alt - neu) < 1e-9:
                return
            mx = self.canvas.canvasx(
                self.canvas.winfo_pointerx() - self.canvas.winfo_rootx())
            my = self.canvas.canvasy(
                self.canvas.winfo_pointery() - self.canvas.winfo_rooty())
            ix = (mx - self.image_offset_x) / alt
            iy = (my - self.image_offset_y) / alt
            self.scale = neu
            self.image_offset_x = mx - ix * neu
            self.image_offset_y = my - iy * neu
            self._neu_zeichnen()

        def _mausrad(self, event):
            if event.delta > 0:
                self.zoom_aendern(1.1)
            elif event.delta < 0:
                self.zoom_aendern(1 / 1.1)

        def _canvas_groesse_geaendert(self, event):
            if self.original_image is not None and self.photo_image is None:
                self.ansicht_einpassen()

        # ---- Zeichnen ---------------------------------------------------
        def _neu_zeichnen(self):
            self.canvas.delete("all")

            if self.original_image is None:
                self.canvas.create_text(
                    max(self.canvas.winfo_width() / 2, 100),
                    max(self.canvas.winfo_height() / 2, 100),
                    text="Bild über „Bild laden“ öffnen", fill="#d0d0d0",
                    font=("Segoe UI", 16))
                return

            dw = max(1, round(self.original_image.width * self.scale))
            dh = max(1, round(self.original_image.height * self.scale))
            self.display_image = self.original_image.resize(
                (dw, dh), Image.Resampling.LANCZOS)
            self.photo_image = ImageTk.PhotoImage(self.display_image)
            self.canvas.create_image(
                self.image_offset_x, self.image_offset_y,
                image=self.photo_image, anchor="nw", tags="image")

            self._viereck_zeichnen()

            links = min(0, self.image_offset_x)
            oben = min(0, self.image_offset_y)
            rechts = max(self.canvas.winfo_width(), self.image_offset_x + dw)
            unten = max(self.canvas.winfo_height(), self.image_offset_y + dh)
            self.canvas.configure(scrollregion=(links, oben, rechts, unten))

        def _viereck_zeichnen(self):
            if len(self.corners) >= 2:
                punkte = [self._bild_zu_canvas(*p) for p in self.corners]
                if len(self.corners) == 4:
                    punkte = punkte + [punkte[0]]
                flach = [wert for punkt in punkte for wert in punkt]
                self.canvas.create_line(*flach, fill=QUAD_FARBE, width=3)

            for index, punkt in enumerate(self.corners):
                cx, cy = self._bild_zu_canvas(*punkt)
                self.canvas.create_oval(
                    cx - self.POINT_RADIUS, cy - self.POINT_RADIUS,
                    cx + self.POINT_RADIUS, cy + self.POINT_RADIUS,
                    fill=QUAD_FARBE, outline="white", width=2)
                self.canvas.create_text(
                    cx, cy, text=str(index + 1), fill="white",
                    font=("Segoe UI", 8, "bold"))

        # ---- Maus ---------------------------------------------------------
        def _maus_runter(self, event):
            if self.original_image is None:
                messagebox.showinfo("Kein Bild geladen",
                                    "Laden Sie zuerst ein Bild.")
                return

            cx = self.canvas.canvasx(event.x)
            cy = self.canvas.canvasy(event.y)

            naechster = self._naechster_punkt(cx, cy)
            if naechster is not None:
                self.dragged_point = naechster
                self.canvas.configure(cursor="hand2")
                return

            bild_punkt = self._canvas_zu_bild(cx, cy)
            if bild_punkt is None:
                self.status_var.set("Der Punkt muss innerhalb des Bildes liegen.")
                return

            if len(self.corners) >= 4:
                # Alle vier Ecken gesetzt -- anders als beim
                # Zwei-Linien-Werkzeug gibt es hier nur EIN Viereck: ein
                # Klick daneben setzt keine fünfte Ecke. Nur Ziehen oder
                # "Viereck zurücksetzen" ändert die Form noch.
                return

            self.corners.append(bild_punkt)
            self.messwerte_aktualisieren()
            self._anleitung_aktualisieren()
            self._neu_zeichnen()

        def _maus_ziehen(self, event):
            if self.dragged_point is None or self.original_image is None:
                return

            cx = self.canvas.canvasx(event.x)
            cy = self.canvas.canvasy(event.y)
            ix = (cx - self.image_offset_x) / self.scale
            iy = (cy - self.image_offset_y) / self.scale
            ix = min(max(ix, 0), self.original_image.width - 1)
            iy = min(max(iy, 0), self.original_image.height - 1)

            self.corners[self.dragged_point] = (ix, iy)
            self.messwerte_aktualisieren()
            self._neu_zeichnen()

        def _maus_los(self, event):
            self.dragged_point = None
            self.canvas.configure(cursor="crosshair")

        def _naechster_punkt(self, cx, cy):
            naechster, distanz = None, float("inf")
            for index, punkt in enumerate(self.corners):
                px, py = self._bild_zu_canvas(*punkt)
                d = math.hypot(cx - px, cy - py)
                if d <= self.POINT_HIT_RADIUS and d < distanz:
                    distanz, naechster = d, index
            return naechster

        # ---- Ergebnisse -----------------------------------------------
        def messwerte_aktualisieren(self):
            if len(self.corners) != 4 or self.original_image is None:
                self.schwarz_var.set("Nicht definiert")
                self.weiss_var.set("Nicht definiert")
                self.pixel_var.set("Nicht definiert")
                self._letztes_ergebnis = None
                return

            # Läuft bei jedem Ziehen mit -- ein ungültiger Schwellenwert
            # fällt hier still auf den Default zurück statt bei jedem
            # Mauszug einen Dialog zu zeigen; Return/FocusOut lösen
            # denselben Pfad aus, falls der Nutzer die Eingabe bewusst
            # abschließt.
            try:
                schwelle = int(float(self.schwelle_var.get().replace(",", ".")))
            except ValueError:
                schwelle = DEFAULT_SCHWELLE

            ergebnis = berechne_deckung(self.original_image, self.corners, schwelle)
            self._letztes_ergebnis = ergebnis

            if "fehler" in ergebnis:
                self.schwarz_var.set(ergebnis["fehler"])
                self.weiss_var.set("Nicht definiert")
                self.pixel_var.set("Nicht definiert")
                return

            self.schwarz_var.set(
                f"{ergebnis['schwarz_prozent']:.2f} %  "
                f"({ergebnis['pixel_schwarz']} Pixel)")
            self.weiss_var.set(
                f"{ergebnis['weiss_prozent']:.2f} %  "
                f"({ergebnis['pixel_weiss']} Pixel)")
            self.pixel_var.set(str(ergebnis["pixel_gesamt"]))

        def letzten_punkt_loeschen(self):
            if not self.corners:
                return
            self.corners.pop()
            self.messwerte_aktualisieren()
            self._anleitung_aktualisieren()
            self._neu_zeichnen()

        def viereck_zuruecksetzen(self, neu_zeichnen=True):
            self.corners = []
            self.dragged_point = None
            self._letztes_ergebnis = None
            self.messwerte_aktualisieren()
            self._anleitung_aktualisieren()
            if neu_zeichnen:
                self._neu_zeichnen()

        def _anleitung_aktualisieren(self):
            if self.original_image is None:
                self.instruction_var.set("Bild laden")
            elif len(self.corners) < 4:
                self.instruction_var.set(
                    f"Eckpunkt {len(self.corners) + 1} von 4 setzen")
            else:
                self.instruction_var.set(
                    "Viereck vollständig. Punkte können verschoben werden.")

        # ---- Export -----------------------------------------------------
        def export_ergebnisbild(self):
            if self.original_image is None:
                messagebox.showinfo("Kein Bild geladen",
                                    "Laden Sie zuerst ein Bild.")
                return
            if len(self.corners) != 4:
                messagebox.showinfo("Viereck unvollständig",
                                    "Setzen Sie zuerst alle vier Eckpunkte.")
                return

            pfad = filedialog.asksaveasfilename(
                title="Export speichern", defaultextension=".png",
                initialfile=suggest_export_dateiname(self.image_path),
                filetypes=[("PNG-Bild", "*.png"), ("JPEG-Bild", "*.jpg *.jpeg"),
                          ("Alle Dateien", "*.*")])
            if not pfad:
                return

            try:
                ergebnis = self._letztes_ergebnis
                if ergebnis is None:
                    ergebnis = berechne_deckung(
                        self.original_image, self.corners,
                        int(float(self.schwelle_var.get().replace(",", "."))))
                export = render_export_bild(self.original_image, self.corners,
                                           ergebnis)
                export.save(pfad)
                self.status_var.set(f"Export gespeichert: {Path(pfad).name}")

            except Exception as fehler:
                messagebox.showerror(
                    "Fehler beim Export",
                    f"Der Export konnte nicht gespeichert werden.\n\n{fehler}")

        # ---- Koordinaten --------------------------------------------------
        def _bild_zu_canvas(self, x, y):
            return (self.image_offset_x + x * self.scale,
                   self.image_offset_y + y * self.scale)

        def _canvas_zu_bild(self, cx, cy):
            if self.original_image is None:
                return None
            x = (cx - self.image_offset_x) / self.scale
            y = (cy - self.image_offset_y) / self.scale
            if (0 <= x < self.original_image.width
                    and 0 <= y < self.original_image.height):
                return (x, y)
            return None

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
