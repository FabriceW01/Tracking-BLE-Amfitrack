# Tracking-BLE-Amfitrack

Ansteuerung der Düsen einer **HP302-Druckerpatrone** über einen ESP32-BLE-Server
(„PrintheadBLE"), erweitert um eine **Positionserkennung mit dem Amfitrack-System**.
Text wird in ein 152 px hohes Schwarz/Weiß-Bild gerendert und spaltenweise gedruckt.

Statt rein zeitgesteuert (eine Spalte pro `--period`) kann der Druck jetzt der
**real gemessenen Position** des Druckkopfs folgen (Closed-Loop). Damit ist die
horizontale Skalierung unabhängig von der Verfahrgeschwindigkeit.

---

## Projektstruktur

```
printhead/
├── geometry.py     BLE-UUIDs + Druckkopf-Geometrie (Nozzles 8..159, 152 px, 19-Byte-Frames)
├── config.py       Einstellungen als dataclasses (RenderSettings, BleSettings, TrackingSettings)
├── rendering.py    Text → 152-px-Ink-Maske → 19-Byte-Frames (vektorisiert via numpy.packbits)
├── ble_client.py   Async-BLE-Transport (bleak): Connect, Notify, Spalten/Blank schreiben
├── tracking.py     Amfitrack-Tracker + Achsen-Remapping/Projektion + Simulator
├── controller.py   Orchestriert Positions- und Zeit-Modus
├── cli.py          Kommandozeile → Einstellungen → Controller
└── __main__.py     python -m printhead
main.py             dünner Einstiegspunkt (== python -m printhead)
tests/              hardwarefreie Tests (Protokoll-Äquivalenz)
```

## Installation

```bash
pip install -r requirements.txt
```

`amfiprot` / `amfiprot-amfitrack` werden nur für den echten Positionsbetrieb
gebraucht. Rendering, `--dry-run` und `--simulate` laufen ohne sie (und ohne `bleak`).

## Schnellstart

```bash
# Vorschau erzeugen, nichts senden:
python main.py "Hallo" --dry-run --preview vorschau.png

# Positionsbasiert drucken (Standard), auf START-Taster warten:
python main.py "Hallo"

# Positions-Loop ohne Hardware testen:
python main.py "Hallo" --simulate --mode line --dry-run

# Klassisch zeitbasiert (wie das Ursprungsskript):
python main.py "Hallo" --mode time --period 0.03
```

**Experimentell: `--mode page`** — freihändiges 2D-Drucken (Wagen frei über die
Seite bewegen, nicht nur eine Richtung). Braucht eine vorher erstellte
`PageCalibration` (`printhead/calibration.py`, `--page-calibration PATH`) — dafür
im **Calibration**-Tab der Web-UI zwei angrenzende Seitenkanten mit dem
Sensor abfahren, "Compute calibration" berechnen lassen und speichern; die
gespeicherte Datei dann per `--page-calibration PATH` laden. Details:
`README_BLE_INTERFACE.md` im Firmware-Repo (Abschnitt "Page Mode").

⚠️ **Wichtig:** Beim ersten echten Hardware-Bring-up ist der Modus ohne
Fehlermeldung leer durchgelaufen (`active=0` die ganze Zeit, Exit-Code 0,
nichts auf dem Papier) — der Grund war genau der folgende Punkt: Das
gerenderte Zielbild ist mit 152 Düsenreihen nur ca. 15 mm
hoch (`NOZZLE_PITCH_MM * 151`). Damit überhaupt eine Düse zündet, muss der
Wagen in `v`-Richtung auf ca. ±15 mm um die abgefahrene Spaltenkante herum
bleiben — außerhalb dieses schmalen Streifens ist für *jede* Düse `active=0`,
und der Pass läuft klaglos (Exit-Code 0) durch, ohne dass etwas gedruckt wird.
Vor dem eigentlichen Druck mit `--pos --page-calibration PATH` das live
`(u, v)` gegen eine bekannte Handbewegung prüfen, um sicherzustellen, dass der
Wagen tatsächlich innerhalb der Seite (und nicht z. B. am falschen Rand oder
mit vertauschten Achsen) unterwegs ist.

**Sensor-Düsen-Versatz (`--sensor-offset-row-mm` / `--sensor-offset-col-mm`):**
Der getrackte Amfitrack-Sensor sitzt **nicht** physisch am Druckkopf — er ist
an einer anderen Stelle des Wagens montiert als die 152-Düsen-Leiste, mit
einem festen Versatz dazwischen. `PageMapper` (`printhead/tracking.py`)
korrigiert das automatisch, bevor `(u, v)` an die Coverage-Engine geht.
Die Default-Werte sind eine echte Messung, kein Schätzwert:

| Option | Bedeutung | Default |
|---|---|---|
| `--sensor-offset-row-mm MM` | Abstand Sensor → **Mitte** der Düsenleiste entlang der Zeilenachse (entlang der Düsenreihe, senkrecht zur Fahrtrichtung) | `62.36` mm (gemessen: "Die Mitte der Nozzle-Reihe ist 62,36 mm verschoben von der Y-Koordinate des Amfitrack") |
| `--sensor-offset-col-mm MM` | Dasselbe entlang der Spaltenachse (Fahrtrichtung) | `0.0` mm (bisher keine gegenteilige Messung; explizit als eigener, überschreibbarer Wert geführt, falls sich das noch ändert) |

Beide Defaults stecken als `SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM` /
`SENSOR_TO_NOZZLE_COL_MM` in `printhead/geometry.py` — als feste mechanische
Eigenschaft des Wagens, unabhängig von jeder einzelnen `PageCalibration`
(eine neue Seite kalibrieren erfordert diesen Wert also nie erneut).

⚠️ **Falsche Richtung nach dem Testdruck?** Einfach den Flag-Wert **negieren**
(z. B. `--sensor-offset-row-mm -62.36`), sonst muss nichts geändert werden.

**Verifikation:** Nach dieser Änderung `--pos --page-calibration PATH`
starten und den Wagen so halten, dass **die Düsenleiste** (nicht der Sensor!)
exakt auf der zuvor abgefahrenen Seitenecke steht. Das live angezeigte `v`
sollte jetzt nahe **0** liegen — vor diesem Fix hätte es (je nach
Zentrum-vs.-Düse-0-Bezug) eher um **-62.36 mm** oder einen ähnlich
verschobenen Wert gelegen.

**Größeres, richtig proportioniertes Testmuster in `--mode page`:** Genau weil
`--mode page` nicht auf die ~15 mm der 152 Düsen begrenzt ist, lohnt sich für
den Bring-up ein deutlich größeres `--calibrate`/`--pattern`-Bild als die
sonst übliche `IMAGE_HEIGHT`-Zeilenzahl:

| Option | Bedeutung |
|---|---|
| `--pattern-height-mm MM` | Physische Gesamthöhe von `--calibrate`/`--pattern` in mm (`rows = height_mm / NOZZLE_PITCH_MM`). Nur mit `--mode page` gültig — im Zeilen-/Zeit-Modus packt `frames_from_ink()` feste Frames mit genau `IMAGE_HEIGHT` Zeilen, eine andere Höhe wird dort mit einem klaren Fehler abgelehnt. Ohne diese Option bleibt das Muster bei `IMAGE_HEIGHT` Zeilen (~15 mm) gedeckelt. |
| `--pattern-square-height-mm MM` | Zeilenperiode in mm für checkerboard/h-stripes, überschreibt `--pattern-square-rows` (`square_rows = v / NOZZLE_PITCH_MM`). |

⚠️ **Seitenverhältnis-Falle:** Eine Bildzeile ist nur **~0.0993 mm** hoch
(`NOZZLE_PITCH_MM`). `--pattern-square-rows 20` (der Default) ist damit nur
knapp **2 mm** hoch, während `--pattern-square-mm 10` (der Default) **10 mm**
breit ist — ein 5:1-Streifen statt eines Quadrats. Für tatsächlich quadratische
Kacheln `--pattern-square-height-mm` statt `--pattern-square-rows` verwenden.

```bash
# Großes Schachbrett in Seiten-Modus: 200mm x 100mm Gesamtfläche, 10mm-Quadrate.
python main.py --pattern checkerboard --mode page --page-calibration page_calibration.json \
    --pattern-length-mm 200 --pattern-height-mm 100 \
    --pattern-square-mm 10 --pattern-square-height-mm 10
```

**Dosierung in `--mode page` (`--dose-hold-s`):** Ein Pixel gilt erst als
gedruckt, wenn eine Düse ununterbrochen `--dose-hold-s` Sekunden darüber
steht — Default `coverage.DEFAULT_DOSE_HOLD_S = 0.00405` s (4.05 ms), gemessen
an einem echten 200×100 mm Schachbrett-Druck bei median 17.3 mm/s
Handgeschwindigkeit. Bei diesem Wert bekommt ein Pixel ca. **3 Tropfen**
(wie `BLE_DROPS_PER_COLUMN` im Zeilen-Modus), bevor es als fertig gilt. Der
alte Default (0.05 s = 50 ms) verlangte unter 4 mm/s, um überhaupt ein Pixel
zu markieren; gemessen wurden dabei nur 0.044 % Coverage über einen
fertigen Druck, mit sichtbarem Geisterbild/Doppeldruck als Folge (jeder
Revisit hat dieselben Pixel an leicht anderer Handposition erneut gefeuert,
weil `printed` fast nirgends True wurde).

⚠️ **Quantisierungs-Klippe (korrigiert):** Ein Pixel wird nur in dem
Sample als fertig markiert, in dem die Verweildauer seit dem *ersten*
Sample auf diesem Pixel `>= dose_hold_s` erreicht — Fertigstellung kostet
also ganze Poll-Intervalle (`1 / --poll-hz`), keine stetige Zeit. Eine erste
Korrekturrunde setzte `dose_hold_s = 0.0054` s (5.4 ms) bei
`PATTERN_STRIDE = 4` — knapp **über** dem 5.00 ms Poll-Intervall von
`--poll-hz 200`. Das erzwingt ein *drittes* Sample auf derselben Spalte,
weil ein zweites Sample bei +5.00 ms die 5.4 ms noch nicht erreicht. Direkt
gemessen (poll_hz=200, realistischer Handgeschwindigkeits-Durchlauf):

```
dose_hold  4.90 ms -> 100.0 % Coverage
dose_hold  5.40 ms ->  31.0 %   <-- der zuvor ausgelieferte Wert
dose_hold  7.00 ms ->  31.0 %
dose_hold 10.00 ms ->   6.5 %
```

Das Überschreiten des Poll-Intervalls degradiert die Coverage also nicht
sanft, sondern lässt sie einbrechen. Zusätzlich zum 3-Tropfen-Ziel gilt
deshalb: `dose_hold_s` **muss unter** dem Poll-Intervall (`1 / poll_hz`)
bleiben, damit zwei aufeinanderfolgende Samples immer für eine
Fertigstellung reichen. Der neue Default 4.05 ms liegt 19 % unter dem
5.00 ms Poll-Intervall (statt knapp darüber). `PrintController` warnt zur
Laufzeit, falls `dose_hold_s >= 1 / poll_hz` doch wieder zutrifft.

Bei Handgeschwindigkeiten über ca. **20 mm/s** reicht die Verweildauer über
einer Spalte nicht mehr für die vollen 4.05 ms — das Pixel bleibt
absichtlich offen für einen späteren Durchgang, statt halb dosiert zu
gelten; das ist gewolltes Verhalten, kein Fehler, aber auch kein Hinweis,
dass dieser Default bei jeder Handgeschwindigkeit ausreicht: gemessen
(poll_hz=200) sind es 100 % bei ≤17.3 mm/s, 60 % bei 25 mm/s, 14 % bei
35 mm/s und 0 % bei 46 mm/s — abgestimmt auf die gemessene Median-, nicht
die Spitzengeschwindigkeit.

⚠️ **Firmware-Kopplung:** `--dose-hold-s` muss zum Firmware-`PATTERN_STRIDE`
(`src/ble_dose.h`, Firmware-Repo) passen: `DEFAULT_DOSE_HOLD_S ≈ 3 ×
PATTERN_STRIDE × 450 µs` (jetzt `PATTERN_STRIDE = 3`). Wird nur eine Seite geändert, stimmt die
Tropfenzahl pro Pixel nicht mehr — die Firmware muss bei einer Änderung
**neu geflasht** werden.

**Geschwindigkeitswarnung in `--mode page` (`--speed-warning-mm-s`):**
Während des Freihand-Durchlaufs schreibt der Client zusätzlich zur
Nozzle-Charakteristik eine neue BLE-Charakteristik (`SPEED_WARN_UUID =
58c05253-945f-48fc-a26c-989c785d6678`, Read/Write, 1 Byte, `0` = ok /
`1` = zu schnell), sobald die gemessene Handgeschwindigkeit
`--speed-warning-mm-s` überschreitet — Default
`controller.DEFAULT_SPEED_WARNING_MM_S = 25.0` mm/s. Dieser Wert stammt aus
derselben Messreihe wie oben: bei 25 mm/s war die Coverage bereits auf **~60
%** gefallen (siehe Tabelle oben), also der Punkt, ab dem ein spürbarer Teil
des Durchlaufs ungedruckt bleibt und eine Warnung an den Bediener sinnvoll
wird. Die Firmware nutzt den Wert nur, um die (zu diesem Zweck
umgewidmete) HEALTH-LED anzusteuern — auf die Dosierung hat er keinen
Einfluss.

Um an der Schwelle nicht bei jedem Sample umzuschalten, hat das Ein-/
Ausschalten eine **Hysterese**: EIN ab `speed_warning_mm_s`, AUS erst wieder
20 % darunter (Totband 20–25 mm/s beim Default). Die Charakteristik wird nur
bei einem tatsächlichen Zustandswechsel beschrieben, nicht bei jedem
Sample, und bei Durchlaufende immer auf `0` zurückgesetzt (auch wenn der
Durchlauf durch einen Fehler abbricht). Der Schreibvorgang ist bewusst
*fail-soft*: anders als der Print-Mode-Wechsel (`--dose-hold-s` o.ä.) darf
ein verlorenes BLE-Write hier niemals den Druckvorgang abbrechen — ein
Fehler wird nur geloggt.

⚠️ **Firmware-Kopplung:** Erfordert eine Firmware mit der neuen Speed-Warning-
Charakteristik geflasht (siehe `README_BLE_INTERFACE.md`, Abschnitt "3) Speed
Warning Characteristic", im Firmware-Repo `Printhead_Original_V2`, Branch
`claude/speed-warning-led`). Ohne diese Firmware schlägt das BLE-Write
fehl — das wird abgefangen und geloggt, bricht den Druckvorgang aber nicht
ab (siehe oben).

---

## Web-UI

Statt der Kommandozeile gibt es eine grafische Oberfläche im Browser, die **alle
CLI-Funktionen** bedient und – sobald verbunden – **die Sensorposition dauerhaft
live anzeigt** (X/Y/Z, Advance, Spalte, Geschwindigkeit + Sparkline).

```bash
pip install -r requirements-ui.txt
python -m printhead.ui            # öffnet http://127.0.0.1:8000 im Browser
```

Die UI ist ein kleiner lokaler Server (FastAPI): sie baut aus den Formularfeldern
den passenden `main.py`-Befehl (mit Live-Vorschau des Befehls), führt ihn aus und
streamt die Ausgabe live in eine Konsole. Alles ist in Tabs organisiert – **Print**
(Text/Kalibrier-Lineal/Testmuster + Render-Optionen), **Tracking & Scale**,
**Nozzle map**, **BLE & Profiling** und **Diagnostics** (list-nodes, scan-ble,
nozzle-test, ble-benchmark). Die Schalter **Simulate** und **Dry-run** oben gelten
global – so lässt sich die komplette UI auch **ohne Hardware** ausprobieren
(„Connect sensor" bei aktivem Simulate zeigt eine simulierte Live-Position).

Optionen: `python -m printhead.ui --host 0.0.0.0 --port 8080 --no-browser`.

---

## Texteinstellungen

Alle Optionen des ursprünglichen Skripts bleiben erhalten:

| Option | Wirkung |
|---|---|
| `--render-size N` | Font-Pixelgröße für das erste Rendering (Default 220) |
| `--font PFAD` | eigene `.ttf`-Datei |
| `--threshold 0..255` | Schwarz/Weiß-Schwelle (Default 128) |
| `--margin N` | vertikaler Rand oben+unten in px |
| `--invert` | weiße Schrift auf schwarz |
| `--flip-y` | vertikal spiegeln (falls kopfüber) |
| `--mirror-x` | Spaltenreihenfolge umkehren (falls gespiegelt) |

Die Höhe ist immer exakt 152 px (Nozzles 8..159); die Breite ergibt sich aus dem Text.

## Positionserkennung (Amfitrack)

Im Positions-Modus liest der Controller die Sensorposition und wählt daraus die
zu druckende Spalte:

```
Spalte = round((Position_entlang_Verfahrachse − Nullpunkt) / mm_pro_spalte)
```

Der Nullpunkt wird beim Start gesetzt (START-Taster oder `--origin startpoint`).
Bei schneller Bewegung übersprungene Spalten werden automatisch nachgefüllt,
damit keine vertikalen Streifen der Schrift verloren gehen. Steht der Kopf still,
wird ein Blank-Frame gesendet (kein Ink-Blob).

**Rückwärts-Schutz:** Der Controller merkt sich mit einer „Frontier" die höchste
bereits gedruckte Spalte. Gedruckt wird nur beim Vorfahren über diese Front hinaus.
Wird der Druckkopf **zurückbewegt**, werden die schon übertragenen Spalten **nicht
erneut gedruckt** (es wird ein Blank-Frame gesendet); erst wenn er wieder über die
bisherige Front hinausfährt, kommen neue Spalten dazu.

**Startpoint-Taster = Reset:** Ein Druck auf den Startpoint-Taster setzt **während des
Drucks jederzeit** den Nullpunkt auf die **aktuelle Position** und setzt die Frontier
zurück – der Druck beginnt also wieder bei Spalte 0, ohne dass ein neuer START-Druck
nötig ist.

### Verfahrachse / verdreht eingebauter Sensor

Die tatsächliche Verfahrachse dieses Aufbaus ist **X** (Default). Ist der Sensor
auf einem Aufbau verdreht verbaut, gibt es zwei Wege, das zu behandeln:

**1. Feste Achse (Standard)** – die Verfahrrichtung ist eine wählbare Achse:

```bash
python main.py "Text" --advance-axis x          # Default (Bewegung entlang X)
python main.py "Text" --advance-axis z --axis-sign -1
```

**2. Auto-Kalibrierung** – die tatsächliche Bewegungsrichtung wird beim Start aus
den ersten Millimetern Bewegung gemessen und die Position darauf projiziert.
Robust gegen **beliebige** Verdrehung, ohne eine feste Achse zu wählen:

```bash
python main.py "Text" --auto-calibrate --calib-distance 5
```

> Hinweis: Während der Kalibrierstrecke (`--calib-distance`, Default 5 mm) wird noch
> nicht gedruckt. Kleiner wählen = früher drucken, aber empfindlicher gegen Rauschen.

### Horizontale Skalierung

```bash
python main.py "Text" --mm-per-column 0.2     # Breite einer Spalte in mm
python main.py "Text" --dpi 96                # alternativ über Auflösung (25.4/DPI)
```

### Weitere Positions-Optionen

| Option | Bedeutung |
|---|---|
| `--origin button\|startpoint` | Was den Nullpunkt setzt (START-Taster oder Startpoint-Charakteristik) |
| `--smooth-ms MS` | Tiefpass-Zeitkonstante (ms) gegen das verrauschte Amfitrack-Signal; `0` = aus, größer = glatter aber mehr Nachlauf (Default 12). Reduziert unregelmäßige Linien/Lücken. |
| `--min-move MM` | Deadband; darunter gilt der Kopf als stehend (Default 0.05) |
| `--poll-hz HZ` | Abtastrate der Position (Default 200). Ein Spaltenübergang kann nur einmal pro Abtastung bemerkt werden, das begrenzt also, wie genau eine Column platziert wird: bei 200 Hz und 20 mm/s sind das 0.1 mm = eine halbe Column. Mit `--profile-csv` messbar — die Abstände zwischen den Schreibzeitpunkten sind auf die effektive Schleifenperiode quantisiert. |
| `--timeout S` | Abbruch eines Durchlaufs nach S Sekunden (Default 30) |
| `--vendor-id` / `--product-id` | USB-IDs des Amfitrack-Dongles (Default `0x0C17` / `0x0D12`) |
| `--sensor-id` | optionaler `tx_id`-Filter unter den „Sensor"-Nodes (Default: alle) |
| `--simulate` | Fake-Tracker (keine Hardware) zum Testen des Loops |

## Kalibrierung & Testmuster

`--calibrate` und `--pattern` sind Alternativen zu `text`: Statt Schrift wird ein
generiertes Muster gedruckt. Beides läuft durch **dieselbe** Pipeline wie normaler
Text – Positions- oder Zeit-Modus, Tracking, `--simulate`, `--dry-run` und
`--preview` funktionieren identisch.

### `--calibrate` – Kalibrier-Lineal

Druckt eine durchgängige Basislinie mit Strichen über die **volle Höhe** alle 1 cm
und **kurzen** Strichen alle 1 mm – wie ein Lineal. Damit lässt sich `mm_per_column`
bzw. `--dpi` exakt einstellen: Muster drucken, echten Abstand zwischen zwei
1-cm-Strichen nachmessen, `--mm-per-column` entsprechend korrigieren.

```bash
python main.py --calibrate --pattern-length-mm 200 --mm-per-column 0.2 --preview lineal.png
```

| Option | Bedeutung |
|---|---|
| `--pattern-length-mm` | Physische Länge des Musters in mm (Default 200) |
| `--calib-major-mm` | Abstand der vollen Striche (Default 10 = 1 cm) |
| `--calib-minor-mm` | Abstand der kurzen Striche (Default 1 = 1 mm) |

### `--pattern NAME` – Testmuster-Presets

| Preset | Zweck |
|---|---|
| `checkerboard` | Schachbrett – deckt Zeilen-/Spalten-Vertauschungen und Ausrichtungsfehler auf |
| `h-stripes` | Volle Zeilenbänder – jede Düse feuert durchgängig über die ganze Länge, eine tote Düse zeigt sich als durchgehende Lücke |
| `v-stripes` | Volle Spaltenbänder – prüft Spalten-/Trackingtiming; ungleiche Streifenbreite = ungleichmäßiger Vorschub |
| `diagonal` | Wiederkehrende Diagonale – eine vertauschte Düsenzeile zeigt sich sofort als Knick (siehe Düsen-Mapping unten) |
| `solid` | Vollfläche – prüft Ink-Deckung/Banding |

```bash
python main.py --pattern checkerboard --pattern-square-mm 10 --pattern-square-rows 20
python main.py --pattern diagonal --mode line --preview diag.png
```

| Option | Bedeutung |
|---|---|
| `--pattern-length-mm` | Physische Länge des Musters in mm (Default 200) |
| `--pattern-square-mm` | Kachel-/Streifenbreite in mm (checkerboard, v-stripes, diagonal-Periode) |
| `--pattern-square-rows` | Kachel-/Streifenhöhe in Zeilen (checkerboard, h-stripes) — Achtung Seitenverhältnis, siehe `--pattern-square-height-mm` im `--mode page`-Abschnitt oben |

## Düsen-Mapping

Falls die physischen Düsen in Blöcken fester Größe verdrahtet sind, deren
Reihenfolge nicht der tatsächlichen (vertikalen) Position entspricht, korrigiert
`--nozzle-block-size` + `--nozzle-order` das vor dem Senden: Die Bildzeilen werden
in Blöcken der angegebenen Größe gemäß der neuen Reihenfolge umsortiert.

`--nozzle-order` ist **1-indiziert** und gibt pro Block-Slot an, welche
ursprüngliche Position dort landen soll. Beispiel: Block-Standardreihenfolge
`1,2,3,4,5` wird zu `2,3,4,1,5` → Slot 1 bekommt, was ursprünglich Düse 2 war,
Slot 2 bekommt Düse 3, Slot 3 bekommt Düse 4, Slot 4 bekommt Düse 1, Slot 5 bleibt.
Das Muster wiederholt sich für alle 152 Zeilen; passt die Blockgröße nicht exakt
(z. B. 152 nicht durch 5 teilbar), bleibt der letzte unvollständige Block
unverändert (eine Meldung weist darauf hin).

```bash
python main.py "Test" --nozzle-block-size 5 --nozzle-order 2,3,4,1,5
```

**Verifikation ohne echten Druck:** `--nozzle-test` wendet dasselbe Mapping auf den
Düsen-Sweep an, sodass man die korrigierte Reihenfolge direkt an der Patrone sehen
kann:

```bash
python main.py --nozzle-test --nozzle-block-size 5 --nozzle-order 2,3,4,1,5
```

## Debug / Diagnose

Jedes dieser Flags führt eine eigenständige Prüfung aus und beendet sich danach —
unabhängig vom Druck. Fehlt Hardware oder eine Bibliothek, kommt eine klare Meldung
statt eines Tracebacks.

| Flag | Wirkung |
|---|---|
| `--pos` | Gibt die **Live-Position** vom Amfitrack aus: `x/y/z` (mm) + Verfahr-Wert entlang `--advance-axis` + Spaltenindex. Zugleich Kalibrierhilfe für Achse und `--mm-per-column`. Ctrl+C beendet. |
| `--list-nodes` | Verbindet zum USB-Dongle und listet alle Nodes (`name`/`uuid`/`tx_id`), markiert die als „Sensor" erkannten. |
| `--scan-ble` | Scannt BLE und listet Geräte (`address` + `name`) – zum Finden der PrintheadBLE-Adresse (nutzbar mit `--address`). |
| `--nozzle-test` | Feuert per BLE ein Testmuster (alle 152 Düsen kurz an → Einzeldüse über alle Zeilen → Blank), um die Patrone zu prüfen. Berücksichtigt `--nozzle-block-size`/`--nozzle-order`, falls gesetzt. |
| `--ble-benchmark` | Misst den **BLE-Durchsatz** (Frames/s ohne Response) und die **Round-Trip-Latenz** (Frames mit Response) – die Obergrenze, ab der der Druck geschwindigkeitsabhängig wird. |

```bash
# Live-Position anschauen (Achse/Skalierung kalibrieren):
python main.py --pos --advance-axis x --mm-per-column 0.2
python main.py --pos --simulate                  # ohne Hardware

# Amfitrack-Nodes / BLE-Geräte auflisten:
python main.py --list-nodes
python main.py --scan-ble

# Düsen der Patrone testen:
python main.py --nozzle-test
```

## Echtzeit / Timing debuggen

Im Positions-Modus wird zwar die *richtige* Spalte aus der Position gewählt, aber
jede Spalte muss noch über BLE **gesendet und von der Firmware verarbeitet** werden.
Genau das ist begrenzt (BLE-Connection-Intervall ~7,5–30 ms, gepufferte
Writes-ohne-Response). Bewegt sich der Kopf schneller, als Spalten geliefert werden
können, **hinken die Spalten der realen Position hinterher** → der Druck wird
geschwindigkeitsabhängig. Zwei Werkzeuge machen das messbar:

**1. `--ble-benchmark`** – misst die Obergrenze der BLE-Strecke:

```bash
python main.py --ble-benchmark --mm-per-column 0.2
```

Ausgabe: erreichter Durchsatz (Spalten/s), Round-Trip-Latenz (avg/p95/max) und
daraus die **maximale Kopfgeschwindigkeit**, bis zu der Spalten noch mithalten
(`Durchsatz × mm_per_column`). Darüber verzerrt der Druck geschwindigkeitsabhängig.

**2. `--profile`** – instrumentiert einen echten Positions-Durchlauf:

```bash
python main.py "Test" --profile
python main.py "Test" --profile --profile-csv timing.csv   # zusätzlich CSV-Log
```

Live werden Kopfgeschwindigkeit, **geforderte** vs. **erreichte** Spaltenrate und die
BLE-Write-Latenz ausgegeben (`load > 1.0` = BLE kommt nicht hinterher). Am Ende ein
Fazit inkl. „bis ~X mm/s halten die Spalten mit". Das `--profile-csv` schreibt pro
Spalte `t, column, advance, write_latency, speed` für die Offline-Analyse.

Im Seitenmodus (`--mode page`) enthält dieselbe `--profile-csv`-Datei zusätzlich
`qx,qy,qz,qw` — das rohe Orientierungs-Quaternion des Sensors, sofern die Hardware es
gerade geliefert hat (sonst leer, nicht `0,0,0,0`). Reine Diagnosedaten für die
nachträgliche, manuelle Auswertung eines echten Druckdurchlaufs: die Hypothese ist,
dass eine Rotation des Wagens zusammen mit dem festen Hebelarm Sensor→Düsenleiste
(`--sensor-offset-row-mm`, s. o.) die beobachteten Verzerrungen (nicht-parallele
Linien, Versatz bei mehreren Durchgängen) erklären könnte. Aktuell fließt das
Quaternion in nichts Live ein — `PageMapper.project()` korrigiert nach wie vor nur
einen festen Versatz, keine Rotation.

**3. `--record BILD.png`** – rekonstruiert, was **tatsächlich aufs Papier geht**:
jeder gesendete Frame wird mit der Kopfposition aufgezeichnet und danach als Bild
gespeichert, das die Frames auf ihre reale Position mappt (die Firmware druckt den
zuletzt empfangenen Frame, bis der nächste kommt – ein Frame belegt also die Strecke
von seiner Sende-Position bis zur nächsten). Oben das beabsichtigte Bild, unten das
gesendete – so werden Stauchung/Verlust sichtbar, wenn bei schneller Bewegung mehrere
Spalten an *einer* Position zusammenfallen.

```bash
python main.py "Test" --record recon.png
python main.py "Test" --simulate --mode line --dry-run --record recon.png  # ohne Hardware
```

In der Web-UI gibt es dafür den **🎞 Record**-Button (zeigt das Vergleichsbild direkt an).
Hinweis: `--record` erfasst, was der Client sendet und *wo* – nicht, was auf dem Funkweg
evtl. verloren geht. Ist die Rekonstruktion sauber, liegt ein verbleibendes Problem an
BLE-Paketverlust/Firmware; ist sie schon gestaucht, liegt es am Sende-/Positionstiming.

> Hinweis: Ohne per-Frame-Rückmeldung der Firmware lässt sich nicht *beweisen*, dass
> eine Spalte physisch rechtzeitig gedruckt wurde; Write-Latenz (`--profile`) und
> Round-Trip mit Response (`--ble-benchmark`) sind die bestmöglichen Proxys. Wenn der
> Druck weiterhin von der Geschwindigkeit abhängt, zeigen diese Werte, ob die
> BLE-Strecke das Nadelöhr ist – dann helfen kürzeres Connection-Intervall, größere
> MTU/mehr Nozzle-Bytes pro Write, größeres `--mm-per-column` oder langsamer verfahren.

## BLE-Protokoll (aus README_BLE_INTERFACE.md / Firmware)

| | |
|---|---|
| Device name | `PrintheadBLE` |
| Service | `d0567401-5a22-c59f-5243-8c0fa18e257b` |
| Nozzle char | `41a9348e-2f6b-8db1-934d-743c6f17649a` (Write/WriteNoRsp, Vielfaches von 19 Bytes) |
| Start btn | `b473a21f-6e58-6380-2647-abd7cd4a904e` (Read/Notify, 1 Byte 0/1) |
| Startpoint | `cc1087f5-1d92-6ca4-b84f-3e5880e6713d` (Read/Notify, 1 Byte 0/1) |

Eine Spalte = 19 Bytes = 152 Nozzle-Bits, LSB-first: Bit `j` (Byte `j//8`, Bit `j%8`).
Die Firmware paddt oben und unten je ein Nullbyte auf das alte 21-Byte-Layout, d. h.
Frame-Bit `j` feuert physisch Nozzle `j + 8`; Bildzeile `y` ↦ Bit `j = y`.

**Mehrere Spalten pro Write:** ein Write darf beliebig viele Spalten hintereinander
tragen (jedes Vielfache von 19 Bytes, max. 32). Die Firmware stellt sie in eine
Warteschlange und druckt **jede genau einmal, in Reihenfolge**, für eine begrenzte
Anzahl Schüsse. Wie viele Spalten pro Write gehen, ergibt sich aus der ausgehandelten
MTU (`--batch-cols`, Default `0` = automatisch; die Firmware fordert MTU 247 an → 12
Spalten). `--batch-cols 1` erzwingt eine Spalte pro Write für ältere Firmware **ohne**
Spalten-Queue — diese verwirft längere Writes kommentarlos.

## Amfitrack-Anbindung / Hinweis zum Payload

Der Zugriff erfolgt über die USB-Pakete `amfiprot` und `amfiprot_amfitrack`
(6-DOF-Ausgabe: Position X/Y/Z + Orientierung). `AmfitrackTracker` in
`printhead/tracking.py` bildet das erprobte Verbindungsverhalten ab:

- **Verbindung**: erst `USBConnection(vendor_id, product_id)` (Sensor-PID `0x0D12`),
  bei Fehler Fallback auf die Source-PID `0x0D01`.
- **Node-Auswahl**: alle Nodes, deren `node.name` „Sensor" enthält, werden als
  `Device` angebunden (optional per `--sensor-id` auf eine `tx_id` eingegrenzt);
  `conn.start()` erst danach.
- **Position**: gelesen aus `payload.emf.pos_x / pos_y / pos_z` (in **mm**). Diese
  bestätigten Namen sind in `_extract_position()` primär; einige Alternativlayouts
  (`.position.x/y/z`, flach `.x/.y/.z`, `position_x_in_m`) bleiben als Fallback für
  abweichende SDK-Versionen. Falls deine SDK die Position anders liefert, dort anpassen.

## Tests / Verifikation ohne Hardware

```bash
python tests/test_frames.py          # Protokoll-Äquivalenz der Frame-Erzeugung
python tests/test_batching.py        # Spalten-Batching (Bytestrom bleibt identisch)
python main.py "Hi" --simulate --mode line --dry-run   # Positions-Loop
python -m printhead --help
```

Alle Tests am Stück:

```bash
for t in tests/test_*.py; do python "$t" >/dev/null && echo "PASS $t" || echo "FAIL $t"; done
```

## Abhängigkeiten

`bleak`, `pillow`, `numpy`, `amfiprot`, `amfiprot-amfitrack` (siehe `requirements.txt`).
