import math
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageTk


class ImageMeasurementApp:
    """
    Grafische Anwendung zum Messen von zwei Linien in einem Bild.

    Bedienung:
    1. Bild laden
    2. Start- und Endpunkt der ersten Linie anklicken
    3. Start- und Endpunkt der zweiten Linie anklicken
    4. Endpunkte bei Bedarf mit der Maus verschieben

    Die Längen werden zunächst in Pixeln angezeigt.
    Optional kann eine Pixel-zu-Millimeter-Kalibrierung gesetzt werden.
    """

    LINE_COLORS = ("#ff3030", "#00a8ff")
    POINT_RADIUS = 7
    POINT_HIT_RADIUS = 14

    def __init__(self, root):
        self.root = root
        self.root.title("Bildmessung: Linienlänge und Winkel")
        self.root.geometry("1250x800")
        self.root.minsize(900, 600)

        # Originalbild und angezeigtes Bild
        self.original_image = None
        self.display_image = None
        self.photo_image = None
        self.image_path = None

        # Transformation zwischen Bildkoordinaten und Canvas-Koordinaten
        self.scale = 1.0
        self.image_offset_x = 0.0
        self.image_offset_y = 0.0

        # Linienpunkte liegen immer in Koordinaten des Originalbildes vor.
        # Aufbau:
        # [
        #     [(x1, y1), (x2, y2)],
        #     [(x1, y1), (x2, y2)]
        # ]
        self.lines = [[], []]

        self.active_line_index = 0
        self.dragged_point = None

        # Kalibrierung
        self.mm_per_pixel = None

        self.status_var = tk.StringVar(
            value="Laden Sie ein Bild, um mit der Messung zu beginnen."
        )
        self.line_1_length_var = tk.StringVar(value="Nicht definiert")
        self.line_2_length_var = tk.StringVar(value="Nicht definiert")
        self.angle_var = tk.StringVar(value="Nicht definiert")
        self.scale_var = tk.StringVar(value="Keine Kalibrierung")
        self.instruction_var = tk.StringVar(value="Bild laden")

        self.create_user_interface()
        self.bind_events()

    def create_user_interface(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(self.root, padding=(10, 8))
        toolbar.grid(row=0, column=0, sticky="ew")
        toolbar.columnconfigure(11, weight=1)

        ttk.Button(
            toolbar,
            text="Bild laden",
            command=self.load_image
        ).grid(row=0, column=0, padx=(0, 6))

        ttk.Button(
            toolbar,
            text="Letzten Punkt löschen",
            command=self.undo_last_point
        ).grid(row=0, column=1, padx=6)

        ttk.Button(
            toolbar,
            text="Linien zurücksetzen",
            command=self.reset_lines
        ).grid(row=0, column=2, padx=6)

        ttk.Separator(
            toolbar,
            orient="vertical"
        ).grid(row=0, column=3, sticky="ns", padx=10)

        ttk.Button(
            toolbar,
            text="Ansicht einpassen",
            command=self.fit_image_to_canvas
        ).grid(row=0, column=4, padx=6)

        ttk.Button(
            toolbar,
            text="Vergrößern",
            command=lambda: self.change_zoom(1.2)
        ).grid(row=0, column=5, padx=6)

        ttk.Button(
            toolbar,
            text="Verkleinern",
            command=lambda: self.change_zoom(1 / 1.2)
        ).grid(row=0, column=6, padx=6)

        ttk.Separator(
            toolbar,
            orient="vertical"
        ).grid(row=0, column=7, sticky="ns", padx=10)

        ttk.Button(
            toolbar,
            text="Kalibrierung",
            command=self.open_calibration_dialog
        ).grid(row=0, column=8, sticky="w", padx=6)

        ttk.Separator(
            toolbar,
            orient="vertical"
        ).grid(row=0, column=9, sticky="ns", padx=10)

        ttk.Button(
            toolbar,
            text="Export",
            command=self.export_measurement_image
        ).grid(row=0, column=10, sticky="w", padx=6)

        main_frame = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        main_frame.grid(row=1, column=0, sticky="nsew")
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(0, weight=1)

        canvas_frame = ttk.Frame(main_frame)
        canvas_frame.grid(row=0, column=0, sticky="nsew")
        canvas_frame.columnconfigure(0, weight=1)
        canvas_frame.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(
            canvas_frame,
            background="#2b2b2b",
            highlightthickness=1,
            highlightbackground="#707070",
            cursor="crosshair"
        )
        self.canvas.grid(row=0, column=0, sticky="nsew")

        horizontal_scrollbar = ttk.Scrollbar(
            canvas_frame,
            orient="horizontal",
            command=self.canvas.xview
        )
        horizontal_scrollbar.grid(row=1, column=0, sticky="ew")

        vertical_scrollbar = ttk.Scrollbar(
            canvas_frame,
            orient="vertical",
            command=self.canvas.yview
        )
        vertical_scrollbar.grid(row=0, column=1, sticky="ns")

        self.canvas.configure(
            xscrollcommand=horizontal_scrollbar.set,
            yscrollcommand=vertical_scrollbar.set
        )

        side_panel = ttk.Frame(main_frame, padding=(15, 5))
        side_panel.grid(row=0, column=1, sticky="ns")

        ttk.Label(
            side_panel,
            text="Messwerte",
            font=("Segoe UI", 16, "bold")
        ).grid(row=0, column=0, sticky="w", pady=(0, 15))

        self.create_result_box(
            side_panel,
            row=1,
            title="Linie 1",
            variable=self.line_1_length_var,
            color=self.LINE_COLORS[0]
        )

        self.create_result_box(
            side_panel,
            row=2,
            title="Linie 2",
            variable=self.line_2_length_var,
            color=self.LINE_COLORS[1]
        )

        self.create_result_box(
            side_panel,
            row=3,
            title="Winkel",
            variable=self.angle_var,
            color="#f2c94c"
        )

        ttk.Separator(
            side_panel,
            orient="horizontal"
        ).grid(row=4, column=0, sticky="ew", pady=15)

        ttk.Label(
            side_panel,
            text="Maßstab",
            font=("Segoe UI", 10, "bold")
        ).grid(row=5, column=0, sticky="w")

        ttk.Label(
            side_panel,
            textvariable=self.scale_var,
            wraplength=250
        ).grid(row=6, column=0, sticky="w", pady=(3, 15))

        ttk.Label(
            side_panel,
            text="Aktueller Schritt",
            font=("Segoe UI", 10, "bold")
        ).grid(row=7, column=0, sticky="w")

        ttk.Label(
            side_panel,
            textvariable=self.instruction_var,
            wraplength=250,
            foreground="#0067c0"
        ).grid(row=8, column=0, sticky="w", pady=(3, 15))

        ttk.Label(
            side_panel,
            text=(
                "Bedienung\n\n"
                "• Linksklick setzt einen Linienpunkt.\n"
                "• Ziehen eines Punktes verschiebt ihn.\n"
                "• Mausrad ändert die Vergrößerung.\n"
                "• Die Linienlänge wird standardmäßig in Pixeln angegeben.\n"
                "• Über Kalibrierung kann zusätzlich in Millimetern gemessen werden."
            ),
            justify="left",
            wraplength=270
        ).grid(row=9, column=0, sticky="nw")

        status_bar = ttk.Label(
            self.root,
            textvariable=self.status_var,
            anchor="w",
            relief="sunken",
            padding=(8, 4)
        )
        status_bar.grid(row=2, column=0, sticky="ew")

    def create_result_box(self, parent, row, title, variable, color):
        frame = ttk.LabelFrame(parent, text=title, padding=10)
        frame.grid(row=row, column=0, sticky="ew", pady=5)

        color_marker = tk.Canvas(
            frame,
            width=18,
            height=18,
            highlightthickness=0
        )
        color_marker.grid(row=0, column=0, padx=(0, 8))
        color_marker.create_oval(2, 2, 16, 16, fill=color, outline=color)

        ttk.Label(
            frame,
            textvariable=variable,
            font=("Segoe UI", 12, "bold")
        ).grid(row=0, column=1, sticky="w")

    def bind_events(self):
        self.canvas.bind("<Button-1>", self.on_left_mouse_down)
        self.canvas.bind("<B1-Motion>", self.on_left_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_left_mouse_up)
        self.canvas.bind("<Configure>", self.on_canvas_resize)

        # Windows und macOS
        self.canvas.bind("<MouseWheel>", self.on_mouse_wheel)

        # Linux
        self.canvas.bind("<Button-4>", lambda event: self.change_zoom(1.1))
        self.canvas.bind("<Button-5>", lambda event: self.change_zoom(1 / 1.1))

        self.root.bind("<Control-o>", lambda event: self.load_image())
        self.root.bind("<Control-z>", lambda event: self.undo_last_point())
        self.root.bind("<Escape>", lambda event: self.reset_lines())

    def load_image(self):
        file_path = filedialog.askopenfilename(
            title="Bild auswählen",
            filetypes=[
                ("Bilddateien", "*.png *.jpg *.jpeg *.bmp *.tif *.tiff *.webp"),
                ("PNG-Dateien", "*.png"),
                ("JPEG-Dateien", "*.jpg *.jpeg"),
                ("Alle Dateien", "*.*")
            ]
        )

        if not file_path:
            return

        try:
            image = Image.open(file_path)
            image.load()

            # Einheitliches RGB-Format verhindert Probleme bei bestimmten Dateitypen.
            if image.mode not in ("RGB", "RGBA"):
                image = image.convert("RGB")

            self.original_image = image
            self.image_path = Path(file_path)
            self.mm_per_pixel = None

            self.reset_lines(redraw=False)

            self.root.update_idletasks()
            self.fit_image_to_canvas()

            self.status_var.set(
                f"Bild geladen: {self.image_path.name} "
                f"({image.width} × {image.height} Pixel)"
            )
            self.update_instruction()

        except Exception as error:
            messagebox.showerror(
                "Fehler beim Laden",
                f"Das Bild konnte nicht geladen werden.\n\n{error}"
            )

    def fit_image_to_canvas(self):
        if self.original_image is None:
            return

        self.root.update_idletasks()

        canvas_width = max(self.canvas.winfo_width(), 100)
        canvas_height = max(self.canvas.winfo_height(), 100)

        margin = 30

        scale_x = (canvas_width - 2 * margin) / self.original_image.width
        scale_y = (canvas_height - 2 * margin) / self.original_image.height

        self.scale = min(scale_x, scale_y)
        self.scale = max(min(self.scale, 10.0), 0.02)

        displayed_width = self.original_image.width * self.scale
        displayed_height = self.original_image.height * self.scale

        self.image_offset_x = max((canvas_width - displayed_width) / 2, 0)
        self.image_offset_y = max((canvas_height - displayed_height) / 2, 0)

        self.redraw_canvas()

    def change_zoom(self, zoom_factor):
        if self.original_image is None:
            return

        old_scale = self.scale
        new_scale = max(0.02, min(old_scale * zoom_factor, 20.0))

        if math.isclose(old_scale, new_scale):
            return

        # Zoomzentrum ist die aktuelle Mausposition.
        mouse_canvas_x = self.canvas.canvasx(
            self.canvas.winfo_pointerx() - self.canvas.winfo_rootx()
        )
        mouse_canvas_y = self.canvas.canvasy(
            self.canvas.winfo_pointery() - self.canvas.winfo_rooty()
        )

        image_x = (mouse_canvas_x - self.image_offset_x) / old_scale
        image_y = (mouse_canvas_y - self.image_offset_y) / old_scale

        self.scale = new_scale

        self.image_offset_x = mouse_canvas_x - image_x * new_scale
        self.image_offset_y = mouse_canvas_y - image_y * new_scale

        self.redraw_canvas()

    def on_mouse_wheel(self, event):
        if event.delta > 0:
            self.change_zoom(1.1)
        elif event.delta < 0:
            self.change_zoom(1 / 1.1)

    def on_canvas_resize(self, event):
        if self.original_image is not None and self.photo_image is None:
            self.fit_image_to_canvas()

    def redraw_canvas(self):
        self.canvas.delete("all")

        if self.original_image is None:
            self.canvas.create_text(
                max(self.canvas.winfo_width() / 2, 100),
                max(self.canvas.winfo_height() / 2, 100),
                text="Bild über „Bild laden“ öffnen",
                fill="#d0d0d0",
                font=("Segoe UI", 16)
            )
            return

        displayed_width = max(1, round(self.original_image.width * self.scale))
        displayed_height = max(1, round(self.original_image.height * self.scale))

        self.display_image = self.original_image.resize(
            (displayed_width, displayed_height),
            Image.Resampling.LANCZOS
        )
        self.photo_image = ImageTk.PhotoImage(self.display_image)

        self.canvas.create_image(
            self.image_offset_x,
            self.image_offset_y,
            image=self.photo_image,
            anchor="nw",
            tags="image"
        )

        self.draw_measurement_lines()

        content_left = min(0, self.image_offset_x)
        content_top = min(0, self.image_offset_y)
        content_right = max(
            self.canvas.winfo_width(),
            self.image_offset_x + displayed_width
        )
        content_bottom = max(
            self.canvas.winfo_height(),
            self.image_offset_y + displayed_height
        )

        self.canvas.configure(
            scrollregion=(
                content_left,
                content_top,
                content_right,
                content_bottom
            )
        )

    def draw_measurement_lines(self):
        for line_index, points in enumerate(self.lines):
            color = self.LINE_COLORS[line_index]

            if len(points) == 2:
                x1, y1 = self.image_to_canvas(*points[0])
                x2, y2 = self.image_to_canvas(*points[1])

                self.canvas.create_line(
                    x1,
                    y1,
                    x2,
                    y2,
                    fill=color,
                    width=4,
                    tags=f"line_{line_index}"
                )

                middle_x = (x1 + x2) / 2
                middle_y = (y1 + y2) / 2

                length_text = self.get_formatted_length(
                    self.calculate_line_length(points)
                )

                self.draw_text_with_background(
                    middle_x,
                    middle_y - 18,
                    length_text,
                    color
                )

            for point_index, point in enumerate(points):
                canvas_x, canvas_y = self.image_to_canvas(*point)

                self.canvas.create_oval(
                    canvas_x - self.POINT_RADIUS,
                    canvas_y - self.POINT_RADIUS,
                    canvas_x + self.POINT_RADIUS,
                    canvas_y + self.POINT_RADIUS,
                    fill=color,
                    outline="white",
                    width=2,
                    tags=f"point_{line_index}_{point_index}"
                )

                self.canvas.create_text(
                    canvas_x,
                    canvas_y,
                    text=str(point_index + 1),
                    fill="white",
                    font=("Segoe UI", 8, "bold")
                )

    def draw_text_with_background(self, x, y, text, color):
        text_id = self.canvas.create_text(
            x,
            y,
            text=text,
            fill="white",
            font=("Segoe UI", 10, "bold")
        )

        bounding_box = self.canvas.bbox(text_id)

        if bounding_box is not None:
            rectangle_id = self.canvas.create_rectangle(
                bounding_box[0] - 5,
                bounding_box[1] - 3,
                bounding_box[2] + 5,
                bounding_box[3] + 3,
                fill="#202020",
                outline=color,
                width=2
            )
            self.canvas.tag_raise(text_id, rectangle_id)

    def on_left_mouse_down(self, event):
        if self.original_image is None:
            messagebox.showinfo(
                "Kein Bild geladen",
                "Laden Sie zuerst ein Bild."
            )
            return

        canvas_x = self.canvas.canvasx(event.x)
        canvas_y = self.canvas.canvasy(event.y)

        nearest_point = self.find_nearest_point(canvas_x, canvas_y)

        if nearest_point is not None:
            self.dragged_point = nearest_point
            self.canvas.configure(cursor="hand2")
            return

        image_point = self.canvas_to_image(canvas_x, canvas_y)

        if image_point is None:
            self.status_var.set("Der Punkt muss innerhalb des Bildes liegen.")
            return

        # Wenn beide Linien vollständig sind, wird durch einen neuen Klick
        # die nächstgelegene Linie neu begonnen.
        if all(len(line) == 2 for line in self.lines):
            self.active_line_index = self.find_nearest_line_index(image_point)
            self.lines[self.active_line_index] = [image_point]
        else:
            self.active_line_index = self.get_next_incomplete_line()
            self.lines[self.active_line_index].append(image_point)

        self.update_measurements()
        self.update_instruction()
        self.redraw_canvas()

    def on_left_mouse_drag(self, event):
        if self.dragged_point is None or self.original_image is None:
            return

        canvas_x = self.canvas.canvasx(event.x)
        canvas_y = self.canvas.canvasy(event.y)

        image_x = (canvas_x - self.image_offset_x) / self.scale
        image_y = (canvas_y - self.image_offset_y) / self.scale

        # Punkt beim Ziehen auf den Bildbereich begrenzen.
        image_x = min(max(image_x, 0), self.original_image.width - 1)
        image_y = min(max(image_y, 0), self.original_image.height - 1)

        line_index, point_index = self.dragged_point
        self.lines[line_index][point_index] = (image_x, image_y)

        self.update_measurements()
        self.redraw_canvas()

    def on_left_mouse_up(self, event):
        self.dragged_point = None
        self.canvas.configure(cursor="crosshair")

    def find_nearest_point(self, canvas_x, canvas_y):
        nearest_point = None
        nearest_distance = float("inf")

        for line_index, points in enumerate(self.lines):
            for point_index, point in enumerate(points):
                point_canvas_x, point_canvas_y = self.image_to_canvas(*point)

                distance = math.hypot(
                    canvas_x - point_canvas_x,
                    canvas_y - point_canvas_y
                )

                if (
                    distance <= self.POINT_HIT_RADIUS
                    and distance < nearest_distance
                ):
                    nearest_distance = distance
                    nearest_point = (line_index, point_index)

        return nearest_point

    def find_nearest_line_index(self, image_point):
        distances = []

        for line in self.lines:
            if len(line) != 2:
                distances.append(float("inf"))
                continue

            distances.append(
                self.distance_point_to_line_segment(
                    image_point,
                    line[0],
                    line[1]
                )
            )

        return distances.index(min(distances))

    @staticmethod
    def distance_point_to_line_segment(point, start, end):
        px, py = point
        x1, y1 = start
        x2, y2 = end

        dx = x2 - x1
        dy = y2 - y1

        line_length_squared = dx * dx + dy * dy

        if line_length_squared == 0:
            return math.hypot(px - x1, py - y1)

        factor = (
            ((px - x1) * dx + (py - y1) * dy)
            / line_length_squared
        )
        factor = min(max(factor, 0), 1)

        nearest_x = x1 + factor * dx
        nearest_y = y1 + factor * dy

        return math.hypot(px - nearest_x, py - nearest_y)

    def get_next_incomplete_line(self):
        if len(self.lines[0]) < 2:
            return 0

        return 1

    def undo_last_point(self):
        if len(self.lines[1]) > 0:
            self.lines[1].pop()
        elif len(self.lines[0]) > 0:
            self.lines[0].pop()
        else:
            return

        self.update_measurements()
        self.update_instruction()
        self.redraw_canvas()

    def reset_lines(self, redraw=True):
        self.lines = [[], []]
        self.active_line_index = 0
        self.dragged_point = None

        self.update_measurements()
        self.update_instruction()

        if redraw:
            self.redraw_canvas()

    def update_measurements(self):
        if len(self.lines[0]) == 2:
            length_1 = self.calculate_line_length(self.lines[0])
            self.line_1_length_var.set(self.get_formatted_length(length_1))
        else:
            self.line_1_length_var.set("Nicht definiert")

        if len(self.lines[1]) == 2:
            length_2 = self.calculate_line_length(self.lines[1])
            self.line_2_length_var.set(self.get_formatted_length(length_2))
        else:
            self.line_2_length_var.set("Nicht definiert")

        if len(self.lines[0]) == 2 and len(self.lines[1]) == 2:
            angle = self.calculate_angle_between_lines(
                self.lines[0],
                self.lines[1]
            )
            self.angle_var.set(f"{angle:.2f}°")
        else:
            self.angle_var.set("Nicht definiert")

    @staticmethod
    def calculate_line_length(line):
        start, end = line

        return math.hypot(
            end[0] - start[0],
            end[1] - start[1]
        )

    @staticmethod
    def calculate_angle_between_lines(line_1, line_2):
        """
        Berechnet den kleineren Winkel zwischen zwei Linien.

        Ergebnisbereich:
        0° bis 90°

        Die Richtung, in der eine Linie eingezeichnet wurde,
        beeinflusst das Ergebnis dadurch nicht.
        """
        vector_1 = (
            line_1[1][0] - line_1[0][0],
            line_1[1][1] - line_1[0][1]
        )
        vector_2 = (
            line_2[1][0] - line_2[0][0],
            line_2[1][1] - line_2[0][1]
        )

        length_1 = math.hypot(*vector_1)
        length_2 = math.hypot(*vector_2)

        if length_1 == 0 or length_2 == 0:
            return 0.0

        scalar_product = (
            vector_1[0] * vector_2[0]
            + vector_1[1] * vector_2[1]
        )

        cosine = scalar_product / (length_1 * length_2)
        cosine = max(-1.0, min(1.0, cosine))

        angle = math.degrees(math.acos(cosine))

        # Der kleinere, richtungsunabhängige Winkel wird ausgegeben.
        if angle > 90:
            angle = 180 - angle

        return angle

    def get_formatted_length(self, pixel_length):
        if self.mm_per_pixel is None:
            return f"{pixel_length:.2f} px"

        millimeter_length = pixel_length * self.mm_per_pixel

        return f"{pixel_length:.2f} px  |  {millimeter_length:.3f} mm"

    def export_measurement_image(self):
        """
        Speichert eine Kopie des Originalbildes (volle Auflösung, unabhängig
        vom aktuellen Zoom) mit den vollständig gezeichneten Linien und --
        sobald beide Linien vorliegen -- dem Winkel zwischen ihnen.

        Die Linienlängen werden hier BEWUSST nicht mit ausgegeben (anders
        als in der Live-Ansicht über draw_measurement_lines): der Export
        soll nur Linien und Winkel zeigen.
        """
        if self.original_image is None:
            messagebox.showinfo(
                "Kein Bild geladen",
                "Laden Sie zuerst ein Bild."
            )
            return

        lines_to_draw = [
            (line_index, points)
            for line_index, points in enumerate(self.lines)
            if len(points) == 2
        ]

        if not lines_to_draw:
            messagebox.showinfo(
                "Keine Linie definiert",
                "Zeichnen Sie mindestens eine vollständige Linie, "
                "bevor Sie exportieren."
            )
            return

        file_path = filedialog.asksaveasfilename(
            title="Export speichern",
            defaultextension=".png",
            initialfile=self.suggest_export_filename(),
            filetypes=[
                ("PNG-Bild", "*.png"),
                ("JPEG-Bild", "*.jpg *.jpeg"),
                ("Alle Dateien", "*.*")
            ]
        )

        if not file_path:
            return

        try:
            export_image = self.render_export_image(lines_to_draw)
            export_image.save(file_path)

            self.status_var.set(
                f"Export gespeichert: {Path(file_path).name}"
            )

        except Exception as error:
            messagebox.showerror(
                "Fehler beim Export",
                f"Der Export konnte nicht gespeichert werden.\n\n{error}"
            )

    def suggest_export_filename(self):
        if self.image_path is None:
            return "messung_export.png"

        return f"{self.image_path.stem}_winkel.png"

    def render_export_image(self, lines_to_draw):
        """
        Zeichnet die übergebenen Linien (Liste aus (line_index, points))
        auf eine RGB-Kopie des Originalbildes. Ist für BEIDE Linien ein
        vollständiges Punktepaar vorhanden, wird zusätzlich der Winkel
        zwischen ihnen als Textlabel eingeblendet -- keine Längenangaben.
        """
        export_image = self.original_image.convert("RGB").copy()
        draw = ImageDraw.Draw(export_image)

        reference_size = min(export_image.width, export_image.height)
        line_width = max(2, round(reference_size * 0.004))
        point_radius = max(4, round(reference_size * 0.006))

        for line_index, points in lines_to_draw:
            color = self.LINE_COLORS[line_index]
            start, end = points

            draw.line([start, end], fill=color, width=line_width)

            for point in (start, end):
                point_x, point_y = point
                draw.ellipse(
                    [
                        point_x - point_radius,
                        point_y - point_radius,
                        point_x + point_radius,
                        point_y + point_radius
                    ],
                    fill=color,
                    outline="white",
                    width=max(1, line_width // 2)
                )

        if len(self.lines[0]) == 2 and len(self.lines[1]) == 2:
            angle = self.calculate_angle_between_lines(
                self.lines[0],
                self.lines[1]
            )
            angle_text = f"Winkel: {angle:.2f}°"

            font = self.load_export_font(reference_size)
            margin = round(reference_size * 0.02)

            self.draw_export_text_with_background(
                draw,
                (margin, margin),
                angle_text,
                font
            )

        return export_image

    @staticmethod
    def load_export_font(reference_size):
        font_size = max(16, round(reference_size * 0.03))

        # Versucht zuerst gebräuchliche, auf dem System vorhandene
        # Schriftarten -- funktioniert unter Windows (arialbd.ttf/Arial
        # Bold.ttf) genauso wie unter Linux (DejaVuSans-Bold.ttf) ohne
        # zusätzliche Abhängigkeiten.
        for font_name in (
            "DejaVuSans-Bold.ttf",
            "arialbd.ttf",
            "Arial Bold.ttf",
            "Arial.ttf"
        ):
            try:
                return ImageFont.truetype(font_name, font_size)
            except OSError:
                continue

        # Fallback: Pillows eigene Standardschrift. Das size-Argument gibt
        # es erst ab Pillow 10.1 -- ohne es bleibt die Schrift klein, aber
        # lesbar, statt den Export scheitern zu lassen.
        try:
            return ImageFont.load_default(size=font_size)
        except TypeError:
            return ImageFont.load_default()

    @staticmethod
    def draw_export_text_with_background(draw, position, text, font):
        x, y = position
        bounding_box = draw.textbbox((x, y), text, font=font)

        padding = 8
        draw.rectangle(
            [
                bounding_box[0] - padding,
                bounding_box[1] - padding,
                bounding_box[2] + padding,
                bounding_box[3] + padding
            ],
            fill="#202020",
            outline="#f2c94c",
            width=2
        )
        draw.text(position, text, fill="white", font=font)

    def open_calibration_dialog(self):
        if self.original_image is None:
            messagebox.showinfo(
                "Kein Bild geladen",
                "Laden Sie zuerst ein Bild."
            )
            return

        dialog = tk.Toplevel(self.root)
        dialog.title("Kalibrierung")
        dialog.transient(self.root)
        dialog.grab_set()
        dialog.resizable(False, False)

        frame = ttk.Frame(dialog, padding=20)
        frame.grid(row=0, column=0, sticky="nsew")

        ttk.Label(
            frame,
            text="Pixelmaßstab festlegen",
            font=("Segoe UI", 13, "bold")
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 12))

        ttk.Label(
            frame,
            text=(
                "Geben Sie die reale Länge und die zugehörige Länge "
                "im Bild an. Als Pixellänge kann beispielsweise die "
                "aktuelle erste Linie verwendet werden."
            ),
            wraplength=420,
            justify="left"
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(0, 15))

        pixel_value = tk.StringVar()
        millimeter_value = tk.StringVar()

        if len(self.lines[0]) == 2:
            line_1_length = self.calculate_line_length(self.lines[0])
            pixel_value.set(f"{line_1_length:.3f}")

        ttk.Label(
            frame,
            text="Länge im Bild:"
        ).grid(row=2, column=0, sticky="w", pady=5)

        pixel_entry = ttk.Entry(
            frame,
            textvariable=pixel_value,
            width=20
        )
        pixel_entry.grid(row=2, column=1, sticky="ew", pady=5)

        ttk.Label(
            frame,
            text="Pixel"
        ).grid(row=2, column=2, sticky="w", padx=(5, 0))

        ttk.Label(
            frame,
            text="Reale Länge:"
        ).grid(row=3, column=0, sticky="w", pady=5)

        millimeter_entry = ttk.Entry(
            frame,
            textvariable=millimeter_value,
            width=20
        )
        millimeter_entry.grid(row=3, column=1, sticky="ew", pady=5)

        ttk.Label(
            frame,
            text="mm"
        ).grid(row=3, column=2, sticky="w", padx=(5, 0))

        button_frame = ttk.Frame(frame)
        button_frame.grid(
            row=4,
            column=0,
            columnspan=3,
            sticky="e",
            pady=(18, 0)
        )

        def apply_calibration():
            try:
                pixel_length = float(
                    pixel_value.get().replace(",", ".")
                )
                millimeter_length = float(
                    millimeter_value.get().replace(",", ".")
                )

                if pixel_length <= 0 or millimeter_length <= 0:
                    raise ValueError

                self.mm_per_pixel = millimeter_length / pixel_length

                self.scale_var.set(
                    f"1 px = {self.mm_per_pixel:.6f} mm"
                )

                self.update_measurements()
                self.redraw_canvas()
                dialog.destroy()

            except ValueError:
                messagebox.showerror(
                    "Ungültige Eingabe",
                    "Beide Werte müssen positive Zahlen sein.",
                    parent=dialog
                )

        def remove_calibration():
            self.mm_per_pixel = None
            self.scale_var.set("Keine Kalibrierung")
            self.update_measurements()
            self.redraw_canvas()
            dialog.destroy()

        ttk.Button(
            button_frame,
            text="Kalibrierung entfernen",
            command=remove_calibration
        ).grid(row=0, column=0, padx=5)

        ttk.Button(
            button_frame,
            text="Abbrechen",
            command=dialog.destroy
        ).grid(row=0, column=1, padx=5)

        ttk.Button(
            button_frame,
            text="Übernehmen",
            command=apply_calibration
        ).grid(row=0, column=2, padx=5)

        pixel_entry.focus_set()
        dialog.bind("<Return>", lambda event: apply_calibration())
        dialog.bind("<Escape>", lambda event: dialog.destroy())

    def update_instruction(self):
        if self.original_image is None:
            self.instruction_var.set("Bild laden")
        elif len(self.lines[0]) == 0:
            self.instruction_var.set("Startpunkt von Linie 1 setzen")
        elif len(self.lines[0]) == 1:
            self.instruction_var.set("Endpunkt von Linie 1 setzen")
        elif len(self.lines[1]) == 0:
            self.instruction_var.set("Startpunkt von Linie 2 setzen")
        elif len(self.lines[1]) == 1:
            self.instruction_var.set("Endpunkt von Linie 2 setzen")
        else:
            self.instruction_var.set(
                "Messung vollständig. Punkte können verschoben werden."
            )

    def image_to_canvas(self, image_x, image_y):
        canvas_x = self.image_offset_x + image_x * self.scale
        canvas_y = self.image_offset_y + image_y * self.scale

        return canvas_x, canvas_y

    def canvas_to_image(self, canvas_x, canvas_y):
        if self.original_image is None:
            return None

        image_x = (canvas_x - self.image_offset_x) / self.scale
        image_y = (canvas_y - self.image_offset_y) / self.scale

        if (
            0 <= image_x < self.original_image.width
            and 0 <= image_y < self.original_image.height
        ):
            return image_x, image_y

        return None


def main():
    root = tk.Tk()

    try:
        style = ttk.Style()
        available_themes = style.theme_names()

        if "vista" in available_themes:
            style.theme_use("vista")
        elif "clam" in available_themes:
            style.theme_use("clam")

    except tk.TclError:
        pass

    app = ImageMeasurementApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()