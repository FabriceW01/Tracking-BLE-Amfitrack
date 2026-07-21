# Tracking-BLE-Amfitrack

Ansteuerung der Düsen einer **HP302-Druckerpatrone** über einen ESP32-BLE-Server
(„PrintheadBLE"), erweitert um eine **Positionserkennung mit dem Amfitrack-System**.
Text wird in ein 164 px hohes Schwarz/Weiß-Bild gerendert und spaltenweise gedruckt.

Statt rein zeitgesteuert (eine Spalte pro `--period`) kann der Druck jetzt der
**real gemessenen Position** des Druckkopfs folgen (Closed-Loop). Damit ist die
horizontale Skalierung unabhängig von der Verfahrgeschwindigkeit.

---

## Projektstruktur

```
printhead/
├── geometry.py     BLE-UUIDs + Druckkopf-Geometrie (Nozzles 2..165, 164 px, 21-Byte-Frames)
├── config.py       Einstellungen als dataclasses (RenderSettings, BleSettings, TrackingSettings)
├── rendering.py    Text → 164-px-Ink-Maske → 21-Byte-Frames (vektorisiert via numpy.packbits)
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
python main.py "Hallo" --simulate --mode position --dry-run

# Klassisch zeitbasiert (wie das Ursprungsskript):
python main.py "Hallo" --mode time --period 0.03
```

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

Die Höhe ist immer exakt 164 px (Nozzles 2..165); die Breite ergibt sich aus dem Text.

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

### Verdreht eingebauter Sensor

Der Sensor ist so verbaut, dass die Bewegung in **Y/Z statt X/Y** stattfindet.
Es gibt zwei Wege, das zu behandeln:

**1. Feste Achse (Standard)** – die Verfahrrichtung ist eine wählbare Achse:

```bash
python main.py "Text" --advance-axis y          # Default (Bewegung entlang Y)
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
| `--min-move MM` | Deadband; darunter gilt der Kopf als stehend (Default 0.05) |
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
python main.py --pattern diagonal --mode position --preview diag.png
```

| Option | Bedeutung |
|---|---|
| `--pattern-length-mm` | Physische Länge des Musters in mm (Default 200) |
| `--pattern-square-mm` | Kachel-/Streifenbreite in mm (checkerboard, v-stripes, diagonal-Periode) |
| `--pattern-square-rows` | Kachel-/Streifenhöhe in Zeilen (checkerboard, h-stripes) |

## Düsen-Mapping

Falls die physischen Düsen in Blöcken fester Größe verdrahtet sind, deren
Reihenfolge nicht der tatsächlichen (vertikalen) Position entspricht, korrigiert
`--nozzle-block-size` + `--nozzle-order` das vor dem Senden: Die Bildzeilen werden
in Blöcken der angegebenen Größe gemäß der neuen Reihenfolge umsortiert.

`--nozzle-order` ist **1-indiziert** und gibt pro Block-Slot an, welche
ursprüngliche Position dort landen soll. Beispiel: Block-Standardreihenfolge
`1,2,3,4,5` wird zu `2,3,4,1,5` → Slot 1 bekommt, was ursprünglich Düse 2 war,
Slot 2 bekommt Düse 3, Slot 3 bekommt Düse 4, Slot 4 bekommt Düse 1, Slot 5 bleibt.
Das Muster wiederholt sich für alle 164 Zeilen; passt die Blockgröße nicht exakt
(z. B. 164 nicht durch 5 teilbar), bleibt der letzte unvollständige Block
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
| `--nozzle-test` | Feuert per BLE ein Testmuster (alle 164 Düsen kurz an → Einzeldüse über alle Zeilen → Blank), um die Patrone zu prüfen. Berücksichtigt `--nozzle-block-size`/`--nozzle-order`, falls gesetzt. |

```bash
# Live-Position anschauen (Achse/Skalierung kalibrieren):
python main.py --pos --advance-axis y --mm-per-column 0.2
python main.py --pos --simulate                  # ohne Hardware

# Amfitrack-Nodes / BLE-Geräte auflisten:
python main.py --list-nodes
python main.py --scan-ble

# Düsen der Patrone testen:
python main.py --nozzle-test
```

## BLE-Protokoll (aus README_BLE_INTERFACE.md / Firmware)

| | |
|---|---|
| Device name | `PrintheadBLE` |
| Service | `d0567401-5a22-c59f-5243-8c0fa18e257b` |
| Nozzle char | `41a9348e-2f6b-8db1-934d-743c6f17649a` (Write/WriteNoRsp, 21 Bytes) |
| Start btn | `b473a21f-6e58-6380-2647-abd7cd4a904e` (Read/Notify, 1 Byte 0/1) |
| Startpoint | `cc1087f5-1d92-6ca4-b84f-3e5880e6713d` (Read/Notify, 1 Byte 0/1) |

Jeder Frame = 21 Bytes = 168 Nozzle-Bits, LSB-first: Bit `p` (Byte `p//8`, Bit `p%8`)
feuert Nozzle `p`. Physisch verbunden sind nur Nozzles 2..165 → 164 Zeilen. Bildzeile
`y` ↦ Nozzle `p = 2 + y`. Die Firmware druckt stets den *zuletzt* empfangenen Frame,
bis der nächste ihn überschreibt.

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
python main.py "Hi" --simulate --mode position --dry-run   # Positions-Loop
python -m printhead --help
```

## Abhängigkeiten

`bleak`, `pillow`, `numpy`, `amfiprot`, `amfiprot-amfitrack` (siehe `requirements.txt`).
