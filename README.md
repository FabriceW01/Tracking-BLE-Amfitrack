# Tracking-BLE-Amfitrack

Ansteuerung der Düsen einer **HP302-Druckerpatrone** über einen ESP32-BLE-Server
(„PrintheadBLE"), erweitert um eine **Positionserkennung mit dem Amfitrack-System**.
Ein Bild oder ein Text wird in ein 152 px hohes Schwarz/Weiß-Raster gerendert und
spaltenweise gedruckt — nicht zeitgesteuert, sondern der **real gemessenen Position**
des Druckkopfs folgend. Damit ist das Druckbild unabhängig von der
Verfahrgeschwindigkeit.

Dieses Repo ist die **Client-Seite** (Python, läuft auf dem Laptop). Die Firmware
liegt im Repo `Printhead_Original_V2`, Branch
`claude/ble-i2s-nozzle-frequency-axpot1`.

---

## Inhaltsverzeichnis

- [1. Überblick](#1-überblick)
  - [1.1 Wie das System arbeitet](#11-wie-das-system-arbeitet)
  - [1.2 Projektstruktur](#12-projektstruktur)
  - [1.3 Installation](#13-installation)
  - [1.4 Schnellstart](#14-schnellstart)
- [2. Betriebsarten](#2-betriebsarten)
  - [2.1 Seiten-Modus (Standard)](#21-seiten-modus-standard)
  - [2.2 Seitenrahmen: kalibriert](#22-seitenrahmen-kalibriert)
  - [2.3 Seitenrahmen: einfach](#23-seitenrahmen-einfach)
  - [2.4 Zeilen-Modus](#24-zeilen-modus)
  - [2.5 Zeit-Modus](#25-zeit-modus)
- [3. Bedienung an der Anlage](#3-bedienung-an-der-anlage)
  - [3.1 START-Taster](#31-start-taster)
  - [3.2 Startpoint-Taster](#32-startpoint-taster)
  - [3.3 Web-UI](#33-web-ui)
  - [3.4 Druckansicht /view](#34-druckansicht-view)
- [4. Geometrie und Kalibrierung](#4-geometrie-und-kalibrierung)
  - [4.1 Feste Maße des Druckkopfs](#41-feste-maße-des-druckkopfs)
  - [4.2 Sensor-Düsen-Versatz](#42-sensor-düsen-versatz)
  - [4.3 Gierwinkel und Boresight](#43-gierwinkel-und-boresight)
  - [4.4 Kalibrierungsqualität](#44-kalibrierungsqualität)
- [5. Dosierung, Tempo und Tinte](#5-dosierung-tempo-und-tinte)
  - [5.1 Dosierung: --drops-per-pixel](#51-dosierung---drops-per-pixel)
  - [5.2 Was die Geschwindigkeit begrenzt](#52-was-die-geschwindigkeit-begrenzt)
  - [5.3 Geschwindigkeitswarnung und Ampel](#53-geschwindigkeitswarnung-und-ampel)
  - [5.4 Latenz-Kompensation](#54-latenz-kompensation)
  - [5.5 Tintenausbreitung](#55-tintenausbreitung)
  - [5.6 Düsengruppierung](#56-düsengruppierung)
- [6. Muster und Text drucken](#6-muster-und-text-drucken)
  - [6.1 Text](#61-text)
  - [6.2 Mustergröße im Seiten-Modus](#62-mustergröße-im-seiten-modus)
  - [6.3 Kalibrier-Lineal: --calibrate](#63-kalibrier-lineal---calibrate)
  - [6.4 Testmuster-Presets: --pattern](#64-testmuster-presets---pattern)
  - [6.5 precision-check](#65-precision-check)
  - [6.6 ruler](#66-ruler)
  - [6.7 drill_pattern](#67-drill_pattern)
- [7. Diagnose und Messwerkzeuge](#7-diagnose-und-messwerkzeuge)
  - [7.1 Übersicht der Diagnose-Flags](#71-übersicht-der-diagnose-flags)
  - [7.2 --straightness: Tracking-Präzision am Lineal](#72---straightness-tracking-präzision-am-lineal)
  - [7.3 --calibration-check: Gierwinkel-Drift](#73---calibration-check-gierwinkel-drift)
  - [7.4 --profile und --ble-benchmark: Echtzeit und Timing](#74---profile-und---ble-benchmark-echtzeit-und-timing)
  - [7.5 --record: was tatsächlich aufs Papier geht](#75---record-was-tatsächlich-aufs-papier-geht)
- [8. Referenz](#8-referenz)
  - [8.1 Positions- und Tracking-Optionen](#81-positions--und-tracking-optionen)
  - [8.2 Textoptionen](#82-textoptionen)
  - [8.3 Düsen-Mapping](#83-düsen-mapping)
  - [8.4 BLE-Protokoll](#84-ble-protokoll)
  - [8.5 Amfitrack-Anbindung](#85-amfitrack-anbindung)
  - [8.6 Abhängigkeiten](#86-abhängigkeiten)
- [9. Tests und Messreihen](#9-tests-und-messreihen)
- [10. Anhang: behobene Fehler und Verifikationen](#10-anhang-behobene-fehler-und-verifikationen)

---

## 1. Überblick

### 1.1 Wie das System arbeitet

Drei Teile arbeiten zusammen:

| Teil | Aufgabe |
|---|---|
| **Amfitrack** (USB am Laptop) | liefert 6-DOF-Pose des Wagens: Position `x/y/z` in mm und Orientierung als Quaternion |
| **Python-Client** (dieses Repo) | rechnet die Pose in Seitenkoordinaten um, entscheidet **welche Düse wann feuert**, und schickt fertige 19-Byte-Spalten über BLE |
| **ESP32-Firmware** (`Printhead_Original_V2`) | puffert die empfangenen Spalten und feuert jede **genau einmal** über I2S auf die Patrone |

Die Arbeitsteilung ist bewusst einseitig: **die Firmware trifft keine inhaltliche
Entscheidung.** Sie feuert, was ankommt, genau einmal, in Eingangsreihenfolge.
Wie viel Tinte wohin kommt, wird vollständig auf dieser Seite entschieden
(siehe [5.1](#51-dosierung---drops-per-pixel)). Die Puffer in der Firmware sind
reine Transportpuffer.

Der Datenweg pro Abtastung (Default 500 Hz):

```
tracker.read_pose()          rohe Pose (x,y,z) + Quaternion
   -> PositionFilter          Tiefpass, NUR auf die Position
   -> PageMapper.project()    Kalibrierung + Gierwinkel + Sensor->Düsen-Versatz
                              => Seitenkoordinaten u/v in mm
   -> Geschwindigkeit         Rückwärtsdifferenz auf u/v
   -> Tintenbudget            wie viele Spalten ist der gefahrene Weg wert
   -> CoverageEngine.step()   pro Düse: feuern oder nicht  => 19 Byte
   -> PatternSender           Warteschlange, bündelt bis zu 12 Spalten je Write
   -> BLE Write-Without-Response
```

### 1.2 Projektstruktur

```
printhead/
├── geometry.py       BLE-UUIDs + Druckkopf-Geometrie (152 Düsen, 19-Byte-Frames)
├── config.py         Einstellungen als dataclasses (Render/Ble/TrackingSettings)
├── cli.py            Kommandozeile -> Einstellungen -> Controller
├── controller.py     Orchestriert Seiten-, Zeilen- und Zeit-Modus
│
├── tracking.py       Amfitrack-Tracker, PositionFilter, PageMapper, Simulator
├── calibration.py    Seitenebenen-Kalibrierung (Kanten fitten, Fit-Metriken)
├── rotation.py       Gierwinkel aus dem Quaternion (Swing-Twist)
│
├── rendering.py      Text -> 152-px-Ink-Maske -> 19-Byte-Frames (numpy.packbits)
├── patterns.py       Testmuster (checkerboard, ruler, precision-check, ...)
├── nozzle_map.py     Düsen-Blockpermutation (--nozzle-block-size/--nozzle-order)
├── coverage.py       CoverageEngine: Feuerentscheidung + Dosis-Buchführung
│
├── ble_client.py     Async-BLE-Transport (bleak), MTU-Aushandlung
├── pattern_sender.py Warteschlange + Bündelung der Spalten-Writes
│
├── diagnostics.py    --pos, --list-nodes, --scan-ble, --nozzle-test, ...
├── profiling.py      --profile / --profile-csv
├── recording.py      --record: Rekonstruktions-PNG (INTENDED/COVERED/MISSED/PATH)
├── straightness.py   --straightness: Offline-Geradheitsauswertung
│
└── ui/               Web-UI (FastAPI-Server + statisches HTML/JS)
    ├── server.py         HTTP + WebSocket-Hub
    ├── runner.py         startet echte main.py-Unterprozesse
    └── static/           index.html (Steuerseite), view.html (/view),
                          coverage_view.js (geteilte Canvas-Logik)

main.py               dünner Einstiegspunkt (== python -m printhead)
funktionen/           eigenständige Auswerteskripte für die Messreihen aus TESTS.md
tests/                25 hardwarefreie Testdateien (siehe Abschnitt 9)
TESTS.md              Testprotokolle für die Messreihe an der Hardware
```

### 1.3 Installation

```bash
pip install -r requirements.txt
```

`amfiprot` / `amfiprot-amfitrack` werden nur für den echten Positionsbetrieb
gebraucht. Rendering, `--dry-run` und `--simulate` laufen ohne sie (und ohne `bleak`).

Für die Web-UI zusätzlich:

```bash
pip install -r requirements-ui.txt
```

### 1.4 Schnellstart

**Wichtig:** `--mode page` (freihändiges 2D-Drucken) ist der Standard-Modus.
Er kennt **zwei Seiten-Rahmen** (`--page-frame`): den kalibrierten (Standard,
braucht `--page-calibration PATH`) und den **einfachen** (`--page-frame simple`,
braucht gar keine Kalibrierung). Ohne `--mode`, ohne `--page-calibration` *und*
ohne `--page-frame simple` bricht das Programm mit einer klaren Fehlermeldung
ab, statt stillschweigend das falsche Verhalten zu zeigen.

```bash
# Vorschau erzeugen, nichts senden (kein Hardware-/Kalibrierungs-Zugriff nötig):
python main.py "Hallo" --dry-run --preview vorschau.png --mode line

# Freihändig drucken OHNE Kalibrierung (einfachster Einstieg):
python main.py "Hallo" --page-frame simple

# Freihändig drucken MIT kalibrierter Seite (genauer), auf START-Taster warten:
python main.py "Hallo" --page-calibration page_calibration.json

# Positions-Loop (1D, eine Richtung) ohne Hardware testen:
python main.py "Hallo" --simulate --mode line --dry-run

# Klassisch zeitbasiert (wie das Ursprungsskript):
python main.py "Hallo" --mode time --period 0.03

# Web-UI statt Kommandozeile:
python -m printhead.ui            # öffnet http://127.0.0.1:8000 im Browser
```

---

## 2. Betriebsarten

| Modus | Bewegung | Kalibrierung | Wofür |
|---|---|---|---|
| `page` (**Default**) | frei über die Seite, Drehung erlaubt | kalibriert **oder** `simple` | der eigentliche Betrieb |
| `line` | nur eine Richtung, 1D-Closed-Loop | keine | Auflösungs- und Timingtests |
| `time` | keine Positionsmessung, feste Spaltenperiode | keine | Rückfall auf das Ursprungsskript |

### 2.1 Seiten-Modus (Standard)

Der Wagen wird frei über die Seite geführt, auch gedreht. Der Client führt
laufend Buch, welches Pixel schon Tinte hat, und feuert jede Düse einzeln,
entsprechend ihrer aktuellen, **gierwinkel-gedrehten** Position auf der Seite.

Der Seiten-Rahmen — wo die Seite liegt und wie ihre Achsen zeigen — kommt
entweder aus einer abgefahrenen Kalibrierung ([2.2](#22-seitenrahmen-kalibriert))
oder direkt aus dem Tracker-Koordinatensystem
([2.3](#23-seitenrahmen-einfach)).

⚠️ **Beim Bring-up ist dieser Modus schon einmal ohne Fehlermeldung leer
durchgelaufen** (`active=0` die ganze Zeit, Exit-Code 0, nichts auf dem
Papier). Der Grund: Die Düsenleiste ist nur **13,11 mm** hoch
([4.1](#41-feste-maße-des-druckkopfs)). Damit überhaupt eine Düse zündet, muss
der Wagen in `v`-Richtung innerhalb dieses schmalen Streifens um das Zielbild
bleiben — außerhalb ist für *jede* Düse `active=0`, und der Pass läuft klaglos
durch, ohne dass etwas gedruckt wird. Vor dem eigentlichen Druck deshalb mit
`--pos --page-calibration PATH` das live angezeigte `(u, v)` gegen eine
bekannte Handbewegung prüfen.

### 2.2 Seitenrahmen: kalibriert

Im **Kalibrierung**-Tab der Web-UI zwei angrenzende Seitenkanten mit dem Sensor
abfahren, den Boresight erfassen ([4.3](#43-gierwinkel-und-boresight)),
"Compute calibration" berechnen lassen und speichern; die gespeicherte Datei
dann per `--page-calibration PATH` laden.

Vorteile gegenüber dem einfachen Rahmen: Das Blatt darf schräg zum Tracker
liegen, und eine systematische Skalenabweichung des Trackers kann über eine
bekannte Blattgröße ausgeglichen werden.

### 2.3 Seitenrahmen: einfach

`--page-frame simple` überspringt die Seiten-Kalibrierung komplett und nimmt
direkt das Amfitrack-Koordinatensystem als Seiten-Rahmen:

| | einfach (`--page-frame simple`) | kalibriert (Standard) |
|---|---|---|
| Spaltenachse `u` | Tracker-**x** | abgefahrene Spaltenkante |
| Zeilenachse `v` | Tracker-**y** | abgefahrene Zeilenkante |
| Gierwinkel | absoluter Twist um die z-Achse | um die Seitennormale, relativ zur abgefahrenen Boresight-Pose |
| Nullpunkt | wo der Wagen beim **START**-Druck steht | abgefahrene Seitenecke |
| Skalenkorrektur | keine (Tracker-mm = echte mm) | optional aus bekannter Blattgröße |
| Vorbereitung | keine | zwei Kanten abfahren + Boresight aufnehmen |

Der Preis ist explizit: Das Blatt muss **achsparallel zum Tracker** liegen (der
einfache Modus kennt die Seitenlage nicht und kann sie nicht korrigieren), und
eine systematische Skalenabweichung des Trackers wird nicht ausgeglichen.

Dafür kann er auch keine *schlechte* Kalibrierung erben — was der Grund ist,
warum es ihn gibt. Eine Winkelmessreihe (0…90° in 15°-Schritten gegen physisch
angezeichnete Winkellinien) hat gezeigt, dass der Gierwinkel selbst sauber
misst (Steigung 1,012, konstanter Versatz +0,89°, Reststreuung RMS 0,82° /
max. 1,36°) und die Boresight-Aufnahme über fünf Versuche bis auf 0,001 in
einer Quaternion-Komponente reproduzierbar ist. Schlechte Druckergebnisse
kamen also nicht aus der Winkelrechnung, sondern aus dem kalibrierten Rahmen
selbst — genau den umgeht dieser Modus.

**Bedienung:** Wagen dorthin stellen, wo die **linke obere Ecke des Drucks**
liegen soll, dann START drücken. Der Nullpunkt wird auf die **Düsenleiste**
gelegt, nicht auf den Sensor — die beiden liegen rund 50 mm auseinander
([4.2](#42-sensor-düsen-versatz)), und ohne diese Korrektur läge die Leiste
weit neben der Seite, sodass gar nichts gedruckt würde
(siehe `PageMapper.zero_at_nozzle`).

In der Web-UI steht dafür im **Einstellungen**-Tab das Feld **Page frame**
(`advanced (traced calibration)` / `simple (no calibration)`); bei `simple`
verschwindet das Kalibrierungs-Feld und der Start-Check verlangt keine
Kalibrierungsdatei mehr. `--page-frame simple` und `--page-calibration`
schließen sich gegenseitig aus und werden zusammen abgelehnt.

Live prüfen lässt sich der Rahmen ohne Druck mit:

```bash
python main.py --pos --page-frame simple
```

— das meldet fortlaufend `page_u`/`page_v` und `yaw_deg`, sodass eine bekannte
Handbewegung bzw. ein angezeichneter Winkel direkt gegengeprüft werden kann.
Hinweis: Der *Nullpunkt* wird im Diagnosemodus bewusst **nicht** neu gesetzt,
`page_u`/`page_v` sind dort also absolute Tracker-Koordinaten (plus
Sensor→Düsen-Versatz).

#### Gierwinkel im einfachen Modus: absoluter Twist um die z-Achse

Der Gierwinkel kommt hier aus `rotation.twist_about_axis` — der vom
Hardware-Betreiber selbst erprobten Berechnung aus dessen
`amfitrack_live_pose.py` (`quaternion_twist_angle_deg` mit Achse `(0, 0, 1)`),
wortwörtlich portiert und über 20 000 Zufallsquaternionen × 4 Achsen gegen die
Vorlage geprüft (größte Abweichung 2,3 · 10⁻¹³ Grad).

Der Unterschied zur Rechnung des kalibrierten Modus (`yaw_about_normal`):
**keine Referenzpose nötig.** Es ist der absolute Twist des Wagens um die
Achse, nicht die Drehung relativ zu einem aufgenommenen Boresight. Genau das
war hier wiederholt die Schwachstelle — blinde Erfassung beim ersten Sample
greift irgendeine Pose ab (BLE noch nicht eingeschwungen, Hand noch nicht
ruhig), und der gespeicherte Boresight der Anlage lag ~110° neben „flach".
Eine absolute Ablesung hat dieses Fehlerbild nicht: dieselbe physische
Orientierung liefert lauf für lauf denselben Wert. Zusätzlich ist der Wert auf
±180° gewickelt und damit immun gegen die Quaternion-Doppelüberdeckung
(`q` und `−q` lesen gleich).

An einem echten 360°-Handdreh-Datensatz nachgemessen: Verstärkung (gemeldeter
Gierwinkel pro Grad echter Drehung, endpunktgemessen) **0,994 bis 1,006**,
effektiv monoton (ein einziger „Rückschritt" über 242 Samples, und der beträgt
exakt 0,0°), Gesamtsumme 366,9° für eine 360°-Handdrehung.

**Nicht** umgestellt wurden Roll/Nick: Drei unabhängige Einzelachsen-Twists
sind keine orthogonale Zerlegung einer Drehung — so gelesen meldet eine *reine*
Drehung um die Normale bereits Ausschläge auf den anderen beiden Achsen
(gemessen: 15° flache Drehung aus der realen Montagepose ergibt 15° „Roll").
Für die Live-Anzeige des Betreibers, wo jede Achse für sich betrachtet wird,
ist das in Ordnung; hier sind Roll/Nick aber gerade der „steht der Wagen
schief?"-Indikator und wären damit unbrauchbar. Sie behalten deshalb die
boresight-relative Swing-Twist-Rechnung. Der **kalibrierte** Modus bleibt
davon vollständig unberührt.

#### `--simple-boresight`: Referenzpose anpinnen statt blind erfassen

Die automatische Erfassung beim START hat einen echten Schwachpunkt: Sie nimmt
einfach die Pose, in der der Wagen in genau diesem Moment zufällig steht — BLE
noch nicht eingeschwungen, Hand noch nicht ganz ruhig — und es gibt **keine
Möglichkeit, das zu kontrollieren**, außer den ganzen Durchlauf neu zu starten
und zu hoffen. Genau das ist auf der echten Anlage passiert: Eine tatsächlich
flach und mit 0° Gier gehaltene Pose (`quat=[-0.50 -0.50 -0.51 +0.49]`) wurde
als `yaw=-71.23° roll=-70.29° pitch=-69.34°` gemeldet, weil die automatische
Erfassung an einer anderen, ungeprüften Pose hängengeblieben war.

Der robuste Weg: Referenzpose **erst separat erfassen und verifizieren**, dann
**anpinnen**:

```bash
python main.py --pos --page-frame simple
```

— Wagen wirklich flach mit 0° Gier hinlegen, den rohen `quat=[...]`-Wert aus
der Konsole ablesen, dann exakt diesen Wert anpinnen (**vier
leerzeichengetrennte Zahlen, NICHT kommagetrennt** — ein einzelner
kommagetrennter Wert wie `-0.5,-0.5,-0.51,0.49` fällt durch argparses
Erkennung negativer Zahlen und wird als unbekannte Option abgelehnt):

```bash
python main.py --pos --page-frame simple --simple-boresight -0.50 -0.50 -0.51 0.49
```

Jetzt in derselben Pose bleiben — Yaw/Roll/Nick müssen ~0° zeigen. Erst wenn
das stimmt, denselben `--simple-boresight`-Wert in den echten Druck übernehmen;
die automatische Erfassung beim START greift dann **nicht** mehr ein (ein
gepinnter Wert wird nie überschrieben, siehe `tests/test_freehand_pass.py`).

In der Web-UI übernimmt der Button **Capture yaw reference** (im
Einstellungen-Tab, nur bei `page_frame = simple` sichtbar) genau diesen
Schritt: er liest den zuletzt empfangenen Quaternion-Wert aus dem
Live-Sensor-Panel, zeigt ihn an, und hängt ihn automatisch als
`--simple-boresight` an sowohl den Live-`--pos`-Verifikationsbefehl als auch
den eigentlichen Druckbefehl an.

Ein aufgenommener oder per `--simple-boresight` gesetzter Boresight verschiebt
nur den **Nullpunkt** (sein eigener Twist um dieselbe Achse wird abgezogen),
damit die Pose, an der er erfasst wurde, weiterhin 0° liest.

### 2.4 Zeilen-Modus

`--mode line` ist der 1D-Closed-Loop: Der Wagen bewegt sich nur in eine
Richtung, es wird keine Kalibrierung gebraucht. Der Controller liest die
Sensorposition und wählt daraus die zu druckende Spalte:

```
Spalte = round((Position_entlang_Verfahrachse − Nullpunkt) / mm_pro_spalte)
```

Der Nullpunkt wird beim Start gesetzt (START-Taster oder `--origin startpoint`).
Bei schneller Bewegung übersprungene Spalten werden automatisch nachgefüllt,
damit keine vertikalen Streifen der Schrift verloren gehen. Steht der Kopf
still, wird ein Blank-Frame gesendet (kein Ink-Blob).

**Rückwärts-Schutz:** Der Controller merkt sich mit einer „Frontier" die
höchste bereits gedruckte Spalte. Gedruckt wird nur beim Vorfahren über diese
Front hinaus. Wird der Druckkopf **zurückbewegt**, werden die schon
übertragenen Spalten **nicht erneut gedruckt** (es wird ein Blank-Frame
gesendet); erst wenn er wieder über die bisherige Front hinausfährt, kommen
neue Spalten dazu.

#### Verfahrachse / verdreht eingebauter Sensor

Die tatsächliche Verfahrachse dieses Aufbaus ist **X** (Default). Ist der
Sensor auf einem Aufbau verdreht verbaut, gibt es zwei Wege:

**1. Feste Achse (Standard)** – die Verfahrrichtung ist eine wählbare Achse:

```bash
python main.py "Text" --advance-axis x          # Default (Bewegung entlang X)
python main.py "Text" --advance-axis z --axis-sign -1
```

**2. Auto-Kalibrierung** – die tatsächliche Bewegungsrichtung wird beim Start
aus den ersten Millimetern Bewegung gemessen und die Position darauf
projiziert. Robust gegen **beliebige** Verdrehung, ohne eine feste Achse zu
wählen:

```bash
python main.py "Text" --auto-calibrate --calib-distance 5
```

> Hinweis: Während der Kalibrierstrecke (`--calib-distance`, Default 5 mm) wird
> noch nicht gedruckt. Kleiner wählen = früher drucken, aber empfindlicher
> gegen Rauschen.

#### Horizontale Skalierung

```bash
python main.py "Text" --mm-per-column 0.087   # Breite einer Spalte in mm (Default)
python main.py "Text" --dpi 96                # alternativ über Auflösung (25.4/DPI)
```

### 2.5 Zeit-Modus

`--mode time --period 0.03` sendet eine Spalte pro `--period` Sekunden, ganz
ohne Positionsmessung — das Verhalten des Ursprungsskripts. Die horizontale
Skalierung hängt damit direkt an der Verfahrgeschwindigkeit.

---

## 3. Bedienung an der Anlage

Auf der Platine sitzen zwei Taster und vier LEDs. Die LED-Bedeutungen stehen im
Firmware-Repo (`README_DEBUG_LEDS.md`); hier geht es um die beiden Taster und
die Web-UI.

### 3.1 START-Taster

Startet und beendet einen Durchgang. Der Client wartet zwischen zwei
Durchgängen mit `Waiting for next START press ...`.

Der Taster ist auf der Firmware ein reiner Hardware-Toggle. Damit der
Firmware-Zustand nicht aus dem Tritt gerät, wenn ein Durchgang **von selbst**
endet (Timeout, volle Deckung, Abbruch per Startpoint-Taster), schreibt der
Client nach **jedem** Pass die `PROCESS_STOP`-Charakteristik — unabhängig vom
Modus und unabhängig davon, ob der Pass normal endete oder mit einem Fehler
abbrach. Ohne dieses Signal musste man den START-Taster gelegentlich zweimal
drücken; die vollständige Fehlerbeschreibung steht in
[10.1](#101-start-taster-musste-manchmal-zweimal-gedrückt-werden).

⚠️ Erfordert eine Firmware mit dieser Charakteristik. Ältere Flash-Stände
verhalten sich wie zuvor — kein neuer Fehler, aber der alte Bug bleibt.

### 3.2 Startpoint-Taster

Im **Seiten-Modus** hat der Taster **zwei** Bedeutungen, je nachdem, ob gerade
gedruckt wird:

| Situation | Tastendruck bewirkt |
|---|---|
| **Kein Druck aktiv** (`Waiting for next START press ...`) | Setzt den **Startpunkt**: ein bestimmter Punkt des zu druckenden Musters (per `--startpoint-anchor`, Default die **Mitte**) landet dort, wo die **Düsenleiste** gerade steht. Ausgabe: `[startpoint] page origin placed -- pattern CENTRE now at the nozzle bar's current position ...` Beliebig oft wiederholbar — der zuletzt gesetzte Punkt gilt. Danach START drücken. |
| **Druck läuft** | **STOP**: der Pass endet sofort, es wird ein Blank-Frame gesendet, `--record`/`--profile-csv` werden noch sauber geschrieben, und die Ausgabe kehrt zu `Waiting for next START press ...` zurück. Ausgabe: `[startpoint] pass stopped by button press.` |

**Welcher Punkt des Musters gesetzt wird (`--startpoint-anchor`):**

| Wert | Punkt des Musters am Taster-Druck | Wann sinnvoll |
|---|---|---|
| `center` (Default) | Mitte des Musters | Normalfall — man zeigt auf die Stelle, wo das Bild mittig sitzen soll, ohne eine Ecke abschätzen zu müssen. |
| `left-middle` | linke Kante, vertikal mittig | Druck soll bündig an einer bekannten linken Kante starten (Lineal, Blattkante). |
| `top-left` | die tatsächliche obere linke Ecke | Exakte Ecken-Platzierung, z. B. gegen eine Ecke des Papiers. |

`top`/`left` beziehen sich auf Zeile 0 / Spalte 0 des Zielbild-Arrays selbst —
dieselbe Richtung, die `--flip-y`/`--mirror-x` korrigieren, falls der reale
Druck auf dieser Kalibrierung gespiegelt herauskommt. Nur der Ursprung ändert
sich mit der Wahl; die Ausgabe nennt den Anker beim Namen (`CENTRE` /
`LEFT EDGE, vertically centred` / `TOP-LEFT CORNER`), sodass der gesetzte Punkt
auch ohne Blick in den Code nachvollziehbar ist.

Wichtig: Nur der **Ursprung** wird verschoben. Die abgefahrene Ebene aus
`page_calibration.json` (Achsen `e_col`/`e_row`, Skalen) bleibt komplett
unangetastet — die Kalibrierungsdatei definiert weiterhin *wo die Ebene liegt*,
der Taster nur *wo auf dem Blatt die Mitte des Bildes liegt*.

Der Ursprung bleibt über mehrere Pässe hinweg gesetzt (wie eine Kalibrierung),
bis er erneut per Taster verschoben wird. Im einfachen Modus
(`--page-frame simple`) hat ein so gesetzter Ursprung **Vorrang** vor dem sonst
automatischen Nullen beim START — sonst würde die bewusste Platzierung still
überschrieben. Ohne Tastendruck bleibt dort alles beim Nullpunkt zum
START-Druck.

Im **Zeilen-Modus** (`--mode line`) hat derselbe Taster eine andere Bedeutung:
Ein Druck setzt **während des Drucks jederzeit** den Nullpunkt auf die
**aktuelle Position** und setzt die Frontier zurück — der Druck beginnt also
wieder bei Spalte 0, ohne dass ein neuer START-Druck nötig ist.

### 3.3 Web-UI

Grafische Oberfläche im Browser, gebaut um die zwei Dinge, für die die Anlage
benutzt wird: **Bilder drucken** und **die Messreihe aus `TESTS.md` fahren**.

```bash
pip install -r requirements-ui.txt
python -m printhead.ui            # öffnet http://127.0.0.1:8000 im Browser
```

Optionen: `python -m printhead.ui --host 0.0.0.0 --port 8080 --no-browser`.

**Vier Reiter:** **Drucken** (Bild, Testmuster oder Text, mit Größen und den
Ablauf-Schaltern sofort starten / ein Durchgang / Trockenlauf), **Tests** (die
Protokolle aus `TESTS.md` als Ein-Klick-Aktionen, jeweils mit der Nummer des
Tests und einem Satz dazu, was er misst), **Kalibrierung** (beide Blattkanten
abfahren, Boresight erfassen, berechnen, speichern) und **Einstellungen**
(Modus, Seitenrahmen, Dosierung, Glättung, Spray, Latenzkompensation).

Der gebaute Befehl steht immer im Klartext unter den Druckknöpfen — die UI
führt echte `main.py`-Unterprozesse aus und kann deshalb nicht davon abweichen,
was die CLI tut.

**Zwei Anzeigen sind immer sichtbar**, egal in welchem Reiter man gerade ist:

- **Live-Position** mit denselben Größen, die `--verbose` ausgibt: rohes
  x/y/z, Seiten-u/v, Zeile/Spalte, Gier/Roll/Nick — und während eines
  Durchgangs zusätzlich Geschwindigkeit und Deckung mit Fortschrittsbalken.
  Bleiben die Werte aus, färben sie sich nach zwei Sekunden grau und die
  Quelle springt auf „veraltet", statt eine tote Zahl weiter anzuzeigen. Das
  Feld „v mm/s" ist zusätzlich als Ampel eingefärbt
  ([5.3](#53-geschwindigkeitswarnung-und-ampel)).
- **Deckung (live)** — während eines Durchgangs wächst hier mit, was
  tatsächlich schon Tinte bekommen hat. Das Zielbild liegt blass darunter,
  also ist auf einen Blick sichtbar, was noch **fehlt**. Klick schaltet auf
  1:1-Pixel um (die Seitenspalte verkleinert ein 2299 Spalten breites Ziel
  sonst 6-fach, wobei einzelne Spaltenstriche untergehen).

  **Wo der Druckkopf gerade steht**, zeigt eine rote Linie: die Düsenleiste in
  ihrer aktuellen Gierlage, mit einem Punkt am Ende von Düse 0, damit die
  Leiste eine erkennbare Richtung hat. Ist der Kopf **komplett außerhalb** des
  Druckbilds, entfällt die Linie und stattdessen sitzt ein oranger Punkt am
  Bildrand in seiner Richtung — man weiß dann, wohin zurückzufahren ist. Die
  Unterscheidung läuft über die Balkenmitte: ragt bei Schräglage ein Ende ins
  Bild, bleibt die Linie, weil sie dann die bessere Auskunft ist.

  Die beiden Endpunkte kommen fertig aus `controller._coverage_event`, mit
  derselben Formel gerechnet (`coverage.bar_offset_uv`), mit der
  `CoverageEngine.step()` jede einzelne Düse platziert — die Linie liegt also
  da, wo auch wirklich Tinte landet. Bewusst nicht im Browser nachgerechnet:
  das wären eine zweite Kopie der Formel plus Kopien von
  `NOZZLE_PITCH_MM`/`NOZZLE_BAR_SPAN_MM`/`NUM_NOZZLES`, die beim nächsten
  Neuvermessen der Leiste still auseinanderlaufen würden.

  Ein Klotz-Pixel = eine Zelle des Zielbilds, dieselbe Konvention wie
  `record.png`. Reißt die WebSocket-Verbindung mitten im Durchgang, fehlen die
  in dieser Zeit gedruckten Zellen auf dieser Leinwand dauerhaft — es gibt
  keine Nachlieferung. Das Panel sagt das dann auch. Maßgeblich ist in dem Fall
  das am Pass-Ende geschriebene `--record`-PNG
  ([7.5](#75---record-was-tatsächlich-aufs-papier-geht)), das zusätzlich MISSED
  und die Fahrspur zeigt.

**Sensor-Übergabe:** Der Amfitrack ist ein einzelnes USB-Gerät und lässt sich
nicht zweimal öffnen. Startet man eine Aktion, während der Leerlauf-Strom
läuft, tritt dieser automatisch ab und kommt danach von selbst zurück;
währenddessen speist der Durchgang selbst die Live-Anzeige. Ein ausdrücklich
gestoppter Strom wird **nicht** wieder aufgeweckt.

Oben rechts liegen **Aktion stoppen** und **Herunterfahren** — Letzteres
beendet laufende Aktion und Sensorstrom sauber (SIGINT, damit der Druckkopf
noch geleert und der Tracker geschlossen wird) und fährt dann den Server
herunter.

> Die Steuerseite hatte früher zusätzlich ein statisches **Druckvorschau**-Panel
> und ein **Deckung (letzter Durchgang)**-Panel. Beide wurden auf Wunsch
> entfernt; die Vorschau wird weiterhin serverseitig erzeugt (`/api/preview.png`
> bleibt aktuell), nur nicht mehr auf der Steuerseite angezeigt.

### 3.4 Druckansicht /view

Kopfzeilen-Knopf **„Druckansicht ↗"** öffnet `/view` in einem eigenen
Tab/Fenster — eine schlanke, reine Beobachtungsseite ohne Druck-Formular,
Konsole oder Kalibrierung, gedacht dafür, sie neben (oder auf einem zweiten
Bildschirm über) der Anlage offen zu lassen, während ein Durchgang läuft:

- **Hauptfokus die Deckungsansicht** — dieselbe live wachsende Ansicht wie auf
  der Steuerseite (Zielbild blass darunter, rote Kopflinie/orange Randmarke),
  nur deutlich größer statt in einer schmalen Seitenspalte.
- **Position** — dieselben Felder wie im Live-Positions-Panel der Steuerseite
  (x/y/z, Seite u/v, Spalte/Zeile, Gier, Geschwindigkeit), inklusive Ampel.
- **Deckung mit Prozent** — die Steuerseite zeigt „7483 / 9939 Pixel" plus
  Balken; hier steht zusätzlich eine große Prozentzahl davor, auf einen Blick
  aus der Entfernung lesbar.

Läuft über **denselben** `/ws`, den auch die Steuerseite benutzt — der Hub
sendet an alle verbundenen Clients dasselbe, ganz ohne Extra-Serverlogik für
ein zweites Fenster (`server.py`'s `Hub.broadcast`). Die Canvas-Zeichenlogik
(`covStart`/`covCells`/`covHead`) liegt in einer geteilten `coverage_view.js`
statt zweimal in beiden HTML-Dateien — dieselbe Begründung wie für die
serverseitige `bar`-Berechnung oben: zwei Kopien derselben
Skalierungs-/Geometrierechnung würden beim nächsten Umbau still
auseinanderlaufen, und hier geht es nicht nur um Optik, sondern um die Stelle,
an der die Kopfmarke dem Bediener das Papier zeigt.

**Mitten im Durchgang geöffnet oder neu verbunden?** Der Hub merkt sich den
letzten `coverage_start` einer noch laufenden Aktion und schickt ihn beim
Verbinden sofort nach (`replay: true`), statt das Fenster bis zum NÄCHSTEN
Durchgang leer zu lassen — bei einem einzelnen `--once`-Lauf käme der nie.
Zellen, die vor dieser Verbindung schon gedruckt wurden, fehlen auf dieser
einen Leinwand trotzdem für immer (der Hub puffert sie nicht, ein Druck kann
Millionen Pixel haben) — die Ansicht sagt das dann auch, statt eine
augenscheinlich vollständige, aber lückenhafte Deckung zu zeigen. Dieselbe
Reparatur kommt der Steuerseite zugute: ein Neuladen mitten im Durchgang zeigte
vorher ebenfalls nur eine leere Leinwand bis zum nächsten Pass.

---

## 4. Geometrie und Kalibrierung

### 4.1 Feste Maße des Druckkopfs

Alle folgenden Werte stehen als benannte Konstanten in `printhead/geometry.py`
und sind die **einzige** Quelle dafür — im Code wird nirgends eine Zahl davon
kopiert. Wird die Leiste neu vermessen, ist diese Datei die einzige Stelle, die
sich ändert.

| Konstante | Wert | Bedeutung |
|---|---|---|
| `ROW_BYTES` | 19 | Bytes je BLE-Spalte |
| `NUM_NOZZLES` / `IMAGE_HEIGHT` | 152 | Düsen bzw. Bildzeilen je Spalte |
| `NOZZLE_OFFSET` | 8 | Frame-Bit `j` → physische Düse `j + 8` |
| `NOZZLE_PITCH_MM` | **0,0868 mm** | Düsenabstand (13,2 mm / 152) |
| `NOZZLE_BAR_WIDTH_MM` | **13,2 mm** | äußere Breite der 152 Düsenzellen |
| `NOZZLE_BAR_SPAN_MM` | **13,11 mm** | Mitte-zu-Mitte, Düse 0 → Düse 151 (151 Abstände) |
| `SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM` | **−50,25 mm** | Sensor → Mitte der Düsenleiste, entlang der Zeilenachse |
| `SENSOR_TO_NOZZLE_COL_MM` | 0 mm | dasselbe entlang der Spaltenachse |

⚠️ **`WIDTH` und `SPAN` sind nicht dasselbe** und liegen nur 0,09 mm
auseinander — eine leicht zu übersehende Falle. `WIDTH` ist die äußere
Ausdehnung (152 Zellen), `SPAN` der Abstand der beiden äußersten
Düsen*mitten* (151 Abstände). **Nur `SPAN` darf halbiert werden**, um von
Düse 0 auf die Leistenmitte zu kommen; `WIDTH / 2` liegt um eine halbe
Teilung daneben. Die einzige Stelle, an der diese Umrechnung passiert, ist
`tracking.PageMapper.__init__`.

Ein früherer Messdurchgang hatte die Leiste mit 15,2 mm über dieselben 152
Zellen vermessen, was `NOZZLE_PITCH_MM` auf exakt 0,1 mm brachte — eine
verdächtig runde Zahl, die damals selbst als Beleg für die Zell-Interpretation
herangezogen wurde. Eine spätere, sorgfältigere Nachmessung direkt an der
physischen Leiste ergibt 13,2 mm. Dieser Wert gilt heute; ältere Zahlen in
Notizen oder Auswertungen, die auf 0,1 mm/Düse oder 15,1 mm Spannweite
beruhen, sind überholt.

`--mm-per-column` (Breite einer gedruckten Spalte, Default **0,087 mm**) ist
davon unabhängig: Es ist eine frei wählbare Rastergröße entlang der
Fahrtrichtung, nicht durch die Hardware festgelegt. Eine Druckzelle ist also
`mm_per_column` **breit** und `NOZZLE_PITCH_MM` **hoch** — zwei verschiedene
physische Maße, was an mehreren Stellen relevant wird (Seitenverhältnis von
Mustern, Spray-Radius, Skalierung von `drill_pattern`).

### 4.2 Sensor-Düsen-Versatz

Der getrackte Amfitrack-Sensor sitzt **nicht** physisch am Druckkopf — er ist
an einer anderen Stelle des Wagens montiert als die 152-Düsen-Leiste, mit einem
festen Versatz dazwischen. `PageMapper` (`printhead/tracking.py`) korrigiert das
automatisch, bevor `(u, v)` an die Coverage-Engine geht.

| Option | Bedeutung | Default |
|---|---|---|
| `--sensor-offset-row-mm MM` | Abstand Sensor → **Mitte** der Düsenleiste entlang der Zeilenachse (entlang der Düsenreihe, senkrecht zur Fahrtrichtung) | `-50.25` mm |
| `--sensor-offset-col-mm MM` | Dasselbe entlang der Spaltenachse (Fahrtrichtung) | `0.0` mm (bisher keine gegenteilige Messung; explizit als eigener, überschreibbarer Wert geführt) |

Beide Defaults stecken als `SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM` /
`SENSOR_TO_NOZZLE_COL_MM` in `printhead/geometry.py` — als feste mechanische
Eigenschaft des Wagens, unabhängig von jeder einzelnen `PageCalibration` (eine
neue Seite kalibrieren erfordert diesen Wert also nie erneut).

**Vorzeichen:** Positiv heißt, die Düsenleiste sitzt weiter in +Zeilenrichtung
als der Sensor. Kommt ein Testdruck in die falsche Richtung versetzt heraus,
ist der Fix, den Wert zu **negieren** (z. B. `--sensor-offset-row-mm 50.25`) —
sonst muss nichts geändert werden.

**Verifikation:** `--pos --page-calibration PATH` starten und den Wagen so
halten, dass **die Düsenleiste** (nicht der Sensor!) exakt auf der zuvor
abgefahrenen Seitenecke steht. Das live angezeigte `v` sollte dann nahe **0**
liegen — bei falschem Vorzeichen liegt es stattdessen um **±50,25 mm** (bzw.
einen ähnlich verschobenen Wert, je nach Zentrum-vs.-Düse-0-Bezug) daneben.

### 4.3 Gierwinkel und Boresight

Der Wagen wird beim freihändigen Drucken nicht nur verschoben, sondern auch
gedreht. An einem echten Druck gemessen (`pass5.csv`, Achsen-Winkel-Zerlegung
der relativen Rotation gegen die Seitennormale) spannt die Gier-Rotation um die
Seitennormale **75,6°** über einen normalen Durchlauf, während Neigung
(Pitch/Roll) klein bleibt (Median 2,7°, Maximum 7,8°) — deshalb wird bewusst
**nur Yaw** korrigiert, Pitch/Roll nicht.

Unkorrigiert bedeutet das zwei konkrete Fehler:

- Der ~50 mm lange Hebelarm Sensor→Düsenleiste
  ([4.2](#42-sensor-düsen-versatz)) ist ein Vektor im
  Wagen-Koordinatensystem, dreht sich also mit dem Wagen mit. Als fester
  Seiten-Versatz behandelt, erzeugt das bei 75,6° Gierspanne bis zu
  **~62 mm** Positionsfehler.
- Die 13,11 mm lange Spanne der Düsenleiste selbst (`NOZZLE_BAR_SPAN_MM`)
  fächert bei Drehung über mehrere Spalten auf: bei 75° sind das ca.
  **12,7 mm** entlang der Fahrtrichtung (≈146 Spalten bei 0,087 mm/Spalte)
  statt einer einzigen Spalte für alle 152 Düsen.

`PageMapper.project()` dreht den Hebelarm-Versatz deshalb mit dem gemessenen
Gierwinkel mit, und `CoverageEngine.step()` platziert jede der 152 Düsen
einzeln entsprechend verteilt.

**Boresight-Aufnahme** (nur im kalibrierten Rahmen): die Referenz-Orientierung,
bei der die Düsenleiste exakt entlang der abgefahrenen Zeilenkante (Kante 2)
liegt, gegen die der aktuelle Gierwinkel während des Drucks gemessen wird.
Aufnahme im **Kalibrierung**-Tab der Web-UI: Sensor verbinden, beide Kanten wie
gewohnt abfahren, den Wagen dann **flach auf das Papier legen**, Düsenleiste
entlang Kante 2 ausrichten, still halten und "Capture boresight" drücken —
übernimmt automatisch das zuletzt vom laufenden `--pos-json`-Stream gelieferte
Quaternion. Die Statuszeile zeigt vorher deutlich "not captured" an; erst danach
"Compute calibration" drücken, damit das Quaternion mit gespeichert wird.

⚠️ **Bestehende Kalibrierungen ohne Boresight** funktionieren unverändert
weiter — **ohne** Rotationskorrektur. Das wird beim Druckstart laut gemeldet
(`[warn] page calibration has no boresight ...`), nie still übernommen: ein
Druck darf nicht davon abhängen, wie der Wagen zufällig zu Beginn gehalten
wurde. Neu kalibrieren mit Boresight-Aufnahme schaltet die Korrektur ein.

`--boresight-deg GRAD` feintunt die aufgenommene Boresight-Rotation additiv,
ohne neu zu kalibrieren — kommt ein Druck verdreht heraus, lässt sich das
direkt per Flag nachjustieren, dasselbe „anpassen statt neu bauen"-Prinzip wie
`--sensor-offset-row-mm`. Hat keine Wirkung, wenn keine Kalibrierung einen
Boresight trägt.

**Verifikation:** `--pos --page-calibration PATH` zeigt das live `yaw` in Grad
an. Wird der Wagen exakt in der Referenzpose gehalten (Düsenleiste entlang der
abgefahrenen Zeilenkante), sollte `yaw` nahe **0°** liegen — weicht es deutlich
ab, per `--boresight-deg` nachjustieren oder neu kalibrieren.

Wie der Gierwinkel aus dem Quaternion gerechnet wird (Swing-Twist statt
Rotationsvektor, und warum), steht in
[10.2](#102-gierwinkel-singularität-bei-180).

### 4.4 Kalibrierungsqualität

`calibrate_page()` prüft, ob die beiden abgefahrenen Kanten nahe genug an 90°
zueinander liegen (`CalibrationAngleWarning`, Toleranz
`MAX_ANGLE_ERROR_DEG = 15°`) — und zusätzlich, wie GUT der Linien-Fit einer
einzelnen Kante für sich selbst ist. Jede Kante liefert dafür drei
Fit-Kennzahlen (`fit_axis_quality()`):

- **Länge** (mm) entlang der gefitteten Richtung,
- **RMS-Residuum** (mm) senkrecht zur gefitteten Linie,
- **Sample-Anzahl**.

Dazu die Neigung der gefitteten Seitennormale gegen die Tracker-z-Achse
(`normal_tilt_deg`). Alle vier Werte landen auf `PageCalibration` (optionale
Felder — bestehende gespeicherte Kalibrierungen ohne diese Felder laden
weiterhin klaglos, mit `None` statt erfundenen Werten).

Gewarnt wird (`CalibrationQualityWarning`, eigene Warnklasse neben
`CalibrationAngleWarning`, beide unabhängig voneinander) auf einer Kante, die
kürzer als **50 mm** ist, ein RMS-Residuum über **1 mm** hat, oder aus weniger
als **20 Samples** besteht. Diese Schwellen stammen aus einer Messreihe
(synthetische gerade Kante, Länge/Rauschen/Sample-Anzahl variiert →
resultierender Fehler in der gefitteten Seitennormale):

```
 Kantenlänge  Rauschen  Samples | resultierender Seitennormalen-Fehler
    210 mm    0.05mm      200   |   0.00° (max 0.01°)
    100 mm    0.5 mm      100   |   0.12° (max 0.37°)
     50 mm    1.0 mm       50   |   0.65° (max 1.40°)
     30 mm    2.0 mm       30   |   3.16° (max 6.25°)
     20 mm    3.0 mm       20   |   7.23° (max 18.63°)
```

und dem separat gemessenen Zusammenhang
`Gierwinkel-Fehler ≈ Neigungswinkel × sin(Seitennormalen-Fehler)` — der Grund,
warum eine schlechte Seitennormale überhaupt etwas ausmacht, obwohl nirgends
direkt Roll/Pitch korrigiert wird: Der Gierwinkel wird relativ zur gefitteten
Normale gemessen, eine falsche Normale verwandelt also gewöhnliches
Tracker-Rauschen in Neigung (Median 2,7°, Max 7,8° auf dieser Anlage) in
**scheinbaren Gierwinkel-Fehler**.

⚠️ **Die Kalibrierung des Betreibers selbst ist GUT** (0,63° Normalen-Neigung,
0,92° Orthogonalitätsfehler — weit im grünen Bereich). Diese Warnungen erklären
also **nicht** das früher beobachtete Gierwinkel-Problem
([10.2](#102-gierwinkel-singularität-bei-180)) — sie fangen künftig schlechte
Kalibrierungen im Allgemeinen ab.

Sichtbar im **Kalibrierung**-Tab der Web-UI direkt neben dem Winkelfehler
(Kantenlänge/Samples/RMS pro Kante, Normalen-Neigung; „n/a" bei einer geladenen
Datei ohne diese Metriken), und als Konsolenzeile, sobald eine Kalibrierung
berechnet wird — dort läuft `calibrate_page()` als Teil des
Web-UI-Serverprozesses, der einzige Ort in diesem Projekt, an dem eine
Kalibrierung überhaupt berechnet wird.

Wie das RMS-Residuum anfangs vom falschen Bezugspunkt gemessen wurde, steht in
[10.3](#103-rms-residuum-wurde-vom-falschen-bezugspunkt-gemessen).

---

## 5. Dosierung, Tempo und Tinte

Alles in diesem Abschnitt gilt für `--mode page`.

### 5.1 Dosierung: --drops-per-pixel

Ein Pixel gilt als gedruckt, sobald es `--drops-per-pixel` Tropfen bekommen hat
— Default `coverage.DEFAULT_DROPS_PER_PIXEL = 2`. Wie viele Kopien einer Spalte
dafür rausgehen, entscheidet **allein der zurückgelegte Weg**:

```
Kopien für dieses Sample = --drops-per-pixel × gefahrener Weg / --mm-per-column
```

Der Bruchteil wird in einem Akkumulator über die Samples mitgeschleppt, damit
nichts durch Abschneiden verlorengeht. Gebrochene Werte sind erlaubt — der
Regler muss nach unten feiner sein als „ganz aus".

Das ist **geschwindigkeitsunabhängig per Konstruktion**: doppeltes Tempo heißt
doppelter Weg je Sample, also doppelt so viele Kopien in der halben Zeit — die
gleiche Tintenmenge pro Spalte. Ein *stehender* Wagen ist entsprechend gar
nichts schuldig und feuert nicht.

⚠️ **Der Wert ist eine Dichte, keine Tropfenzahl.** Aufs Papier geht
`--drops-per-pixel ÷ --mm-per-column`:

```
2 / 0,087 = 23,0 Tropfen/mm   <- Default, heutige Spaltenbreite
1 / 0,087 = 11,5 Tropfen/mm   <- Dichte, an der der Default verankert ist
3 / 0,200 = 15,0 Tropfen/mm   <- Zeilen-Modus, wofür die 3 validiert wurde
3 / 0,087 = 34,5 Tropfen/mm   <- dieselbe 3 bei heutiger Spaltenbreite
```

Wird `--mm-per-column` geändert, ändert sich die Tintenmenge mit, auch wenn
diese Zahl gleich bleibt. Das war ein echter Fehler in der ersten Fassung der
Umstellung: der Default stand auf 3, kopiert aus der Firmware-Konstante
`BLE_DROPS_PER_COLUMN` des Zeilen-Modus, ohne zu bemerken, dass die 3 zu einer
**0,2 mm** breiten Spalte gehört. Bei 0,087 mm ergibt das gut die dreifache
Tintenmenge — von der Hardware zurückgemeldet als „jetzt kommt zu viel raus",
gegenüber einem vorherigen Druck, der heller **und schärfer** war. Die 11,5
Tropfen/mm sind genau das, was der Client **vor** der Umstellung bei langsamer
Fahrt geliefert hat (simuliert: 120 Spalten Tinte auf 120 Spalten Fahrweg),
also die Dichte, die auf echtem Papier beurteilt wurde — daran ist dieser Wert
verankert.

Der Default steht auf **2** (23,0 Tropfen/mm), also bewusst auf dem Doppelten
dieser Dichte: nach einer Reihe echter Drucke mit explizitem
`--drops-per-pixel 2` auf der Kommandozeile vom Anlagenbesitzer so festgelegt.
Auf Papier entschieden, was der einzige Ort ist, an dem sich diese Frage
entscheiden lässt. Kommt der Druck zu dunkel oder verlaufen, ist das der
Regler; kommt er blass heraus, hochsetzen (z. B. `0.7` nach unten).

Gegenprobe aus der Physik: ein Tropfen läuft auf ~60–120 µm aus, eine Spalte
ist 87 µm breit — **ein** Tropfen deckt sie also bereits ab.

⚠️ **Firmware-Kopplung:** Erfordert die Firmware mit dem
Feuer-einmal-Seitenmodus (Branch `claude/ble-i2s-nozzle-frequency-axpot1` im
Repo `Printhead_Original_V2`). Gegen eine ältere, das Muster *haltende*
Firmware würde dieser Client massiv überdrucken, weil er bei jedem Sample mit
offener Tintenschuld sendet. `--drops-per-pixel` selbst ist dagegen **nicht**
an eine Firmware-Konstante gekoppelt — es ist ein reiner Client-Wert und kann
ohne Neu-Flashen verändert werden.

Was die Umstellung vom früheren Verweildauer-Modell (`--dose-hold-s`) im
Einzelnen verändert hat, steht in
[10.4](#104-umstellung-vom-verweildauer-modell-auf-tropfen).

### 5.2 Was die Geschwindigkeit begrenzt

**Die Abtastrate, nicht die Dosis.** Eine Spalte, die der Tracker nie
abgetastet hat, wird nie gefeuert; diese Kante liegt bei

```
--mm-per-column × --poll-hz  =  0,087 × 500  =  43,5 mm/s
```

Simuliert über einen 120-Spalten-Vollblock:

```
Geschwindigkeit   Samples/Spalte   printed   fired
      17,3 mm/s             2,51    100,0 %   100,0 %
      30,0 mm/s             1,45    100,0 %   100,0 %
      43,5 mm/s             1,00    100,0 %   100,0 %
      50,0 mm/s             0,87     86,7 %    86,7 %
      60,0 mm/s             0,72     72,5 %    72,5 %
```

Man beachte, was die beiden Spalten **zusammen** sagen: unterhalb von
43,5 mm/s sind beide 100 %; darüber fallen sie **gemeinsam**, weil der Fehler
keine Unterdosierung mehr ist, sondern komplett übersprungene Spalten.
`fired`, das `printed` nach unten folgt, ist die Signatur davon — Tinte, die
auf dem Papier fehlt, nicht bloß in der Buchhaltung.

**BLE ist dabei nicht die Grenze:** jeder Tropfen ist eine gesendete Spalte,
also `--drops-per-pixel × v / --mm-per-column` Spalten/s — selbst 43,5 mm/s
verlangen beim Default nur 500 Spalten/s = 42 Schreibvorgänge/s bei 12 Spalten
je Vorgang, weit unter den gemessenen ~270/s. (Mit `--drops-per-pixel 3` wären
es 1500 Spalten/s bzw. 125 Schreibvorgänge/s — immer noch drin, aber bei
`--batch-cols 1` bereits über der Decke.)

⚠️ **Startwarnung:** Liegt die Spalten-Kante (`--mm-per-column × --poll-hz`)
auf oder unter der Geschwindigkeitswarnung (`--speed-warning-mm-s`), kann die
Warnung den Schaden nicht mehr ankündigen — dann meldet sich der Client beim
Start. Bei den Defaults (43,5 gegen 25 mm/s) feuert sie nicht; bei
`--poll-hz 200` läge die Kante bei 17,4 mm/s und sie feuert.

Dass die Buchführung pro Sample selbst einmal das eigentliche Tempolimit war,
steht in
[10.6](#106-die-buchführung-pro-sample-war-das-eigentliche-tempolimit).

### 5.3 Geschwindigkeitswarnung und Ampel

Während des Freihand-Durchlaufs schreibt der Client zusätzlich zur
Nozzle-Charakteristik die Speed-Warning-Charakteristik
(`58c05253-945f-48fc-a26c-989c785d6678`, Read/Write, 1 Byte, `0` = ok /
`1` = zu schnell), sobald die gemessene Handgeschwindigkeit
`--speed-warning-mm-s` überschreitet — Default
`controller.DEFAULT_SPEED_WARNING_MM_S = 25.0` mm/s.

Der Wert stammte ursprünglich aus dem Verweildauer-Modell, wo die Deckung bei
25 mm/s bereits auf ~60 % gefallen war. Unter dem Tropfenmodell ist er
**bewusster Sicherheitsabstand** statt Klippenkante: die erste Geschwindigkeit,
bei der real etwas verlorengeht, ist `--mm-per-column × --poll-hz` = 43,5 mm/s
([5.2](#52-was-die-geschwindigkeit-begrenzt)), 25 mm/s liegt ~40 % darunter.
Das lässt Luft für Übersteuern der Hand zwischen zwei Samples und für ein
kleineres `--poll-hz`.

Um an der Schwelle nicht bei jedem Sample umzuschalten, hat das Ein-/Ausschalten
eine **Hysterese**: EIN ab `speed_warning_mm_s`, AUS erst wieder 20 % darunter
(Totband 20–25 mm/s beim Default). Die Charakteristik wird nur bei einem
tatsächlichen Zustandswechsel beschrieben, nicht bei jedem Sample, und bei
Durchlaufende immer auf `0` zurückgesetzt (auch wenn der Durchlauf durch einen
Fehler abbricht). Der Schreibvorgang ist bewusst *fail-soft*: anders als der
Print-Mode-Wechsel darf ein verlorenes BLE-Write hier niemals den Druckvorgang
abbrechen — ein Fehler wird nur geloggt.

Die Firmware nutzt den Wert ausschließlich, um die dafür umgewidmete LED an
GPIO26 anzusteuern — auf die Dosierung hat er **keinen** Einfluss.

⚠️ **Firmware-Kopplung:** Erfordert eine Firmware mit dieser Charakteristik
(siehe `README_BLE_INTERFACE.md`, Abschnitt „3) Speed Warning Characteristic",
im Firmware-Repo). Ohne sie schlägt das BLE-Write fehl — das wird abgefangen
und geloggt, bricht den Druckvorgang aber nicht ab.

**Dieselbe Warnung als Ampel im Browser:** Sowohl die Steuerseite als auch
`/view` färben „v mm/s" im Positionspanel nach dem Ampel-Prinzip ein:

| Farbe | Bereich | Bedeutung |
|---|---|---|
| **grün** | unter `--speed-warning-mm-s` | unbedenklich |
| **gelb** | ab `--speed-warning-mm-s` | grenzwertig — dieselbe Schwelle, die auch die Firmware-LED ansteuert |
| **rot** | ab `--mm-per-column × --poll-hz` | zu schnell — ab hier werden nachweislich Spalten übersprungen |

Beide Zahlen kommen mit jedem `coverage_start` vom Server
(`speed_warn_mm_s`/`speed_stop_mm_s`) statt im Browser fest codiert zu sein —
ein anderes `--dpi`/`--mm-per-column`, `--poll-hz` oder `--speed-warning-mm-s`
verschiebt die Ampel also automatisch mit, ohne eine zweite, driftende Kopie
dieser Einstellungen im JavaScript. Ohne Firmware-Kopplung nutzbar: die
Browser-Ampel hängt nicht an der BLE-Charakteristik, nur an denselben Zahlen.

### 5.4 Latenz-Kompensation

Zwischen „Position gelesen" und „Tinte tatsächlich platziert" liegt eine
messbare Pipeline-Verzögerung: das ausgehandelte BLE-Verbindungsintervall (auf
echter Hardware gemessen: durchgängig 15,00 ms, `itvl=12`) und die
Firmware-Warteschlange. Zusammen ergibt das grob **5 ms bestenfalls, ~13 ms
typisch, ~21 ms im ungünstigsten Fall** — bei 20 mm/s sind das ca. 0,26 mm bzw.
3 Spalten systematischer Nachlauf, der bei einem Richtungswechsel das Vorzeichen
wechselt.

> ⚠️ Die Firmware-Anteile darin stammen noch aus dem alten Seiten-Modus
> (6-Slot-Queue × 450 µs plus `PATTERN_STRIDE` × 450 µs Feuer-Takt). Der
> aktuelle Seiten-Modus fährt einen 128-Spalten-FIFO, eine 3-Slot-Queue und
> 300 µs Takt und feuert jede Spalte genau einmal; die Größenordnung bleibt,
> die genaue Aufteilung ist **nicht neu vermessen** — der Wert oben ist als
> Startpunkt zu lesen, nicht als aktuelle Messung.

`--latency-compensate-s SEKUNDEN` extrapoliert **nur** die an
`CoverageEngine.step()` übergebene Position linear entlang der aktuell
gemessenen Geschwindigkeit nach vorn (`u/v + Geschwindigkeit × Sekunden`) —
alles andere (die `--record`-Pfad-Panels, die Out-of-Page-Prüfung, der Profiler,
die Speed-Warnung) bleibt auf der echten, unkompensierten Position, da diese
zeigen sollen, wo der Wagen wirklich war. Default `0.0` = aus.

⚠️ Das ist eine **Heuristik gegen einen geschätzten Wert**, kein
Allzweck-Glättungsregler: ein zu hoher Wert schießt vor allem beim Abbremsen
oder Richtungswechsel kurz übers Ziel hinaus (die Extrapolation nutzt noch die
Geschwindigkeit von kurz davor). Klein anfangen und gegen einen echten Druck
prüfen, bevor man sich darauf verlässt. Die Geschwindigkeitsschätzung selbst
(Differenz aufeinanderfolgender Positionen) verstärkt außerdem Rauschen — je
größer der gewählte Wert, desto empfindlicher.

### 5.5 Tintenausbreitung

Ein echter Tropfen landet nicht exakt in *einer* Rasterzelle, er benetzt eine
kleine Fläche drumherum. Ohne dieses Modell passiert Folgendes: Bei einer
Rückfahrt sitzt der Wagen ein paar Zehntel-mm versetzt, die Düsen adressieren
dadurch **andere Zeilen-Indizes**, diese gelten als „noch nicht gedruckt" — und
es wird erneut über Papier gedruckt, auf dem längst Tinte ist.

| Option | Bedeutung |
|---|---|
| `--spray-radius-mm MM` | Physischer Radius um ein fertiges Pixel, der eine Teildosis abbekommt. **In Millimetern, nicht in Pixeln** — eine Zelle ist 0,087 mm hoch und `--mm-per-column` breit; bei gleichen Werten ist ein runder Tropfen im Raster also rund, bei abweichendem `--mm-per-column` elliptisch. Default `0` = aus. |
| `--spray-strength F` | Dosis, die ein **direkt angrenzendes** Pixel abbekommt (0.0–1.0), linear abfallend bis 0 am Radius. Ein Pixel gilt ab Gesamtdosis 1.0 als gedruckt: bei `1.0` markiert ein einzelner Tropfen die Nachbarzelle sofort mit, bei `0.5` sind zwei Tropfen nötig. Default `0` = aus. |

Beide müssen `> 0` sein, damit das Modell greift; sonst verhält sich die Engine
exakt wie ohne.

Gemessen an simulierten Mehrfach-Überfahrten mit 0,05 mm Versatz pro Durchgang
(40 × 30 mm Vollfläche, 500 Hz; gemessen noch unter dem alten
Verweildauer-Modell — die *relative* Wirkung des Spray-Modells hängt nicht
daran, die absoluten Feuerungszahlen schon):

```
Einstellung   | Düsen-Feuerungen | Deckung im überfahrenen Band
aus (heute)   |           62.400 |                       100,0 %
r=.15 s=0.5   |           62.400 |                       100,0 %
r=.15 s=1.0   |           46.400 |                       100,0 %
r=.25 s=1.0   |           46.400 |                       100,0 %
r=.40 s=1.0   |           23.501 |                       100,0 %
```

Bei `strength 0.5` passiert nichts, weil eine einzelne Nachbar-Teildosis von 0,5
allein nie die 1,0 erreicht und die Leiste im nächsten Durchgang ohnehin selbst
darüberfährt. Erst ab `strength 1.0` fällt das Nachdrucken messbar
(−25 % Feuerungen gesamt, −33 % in den Wiederholungs-Durchgängen).

⚠️ **Die Deckungszahl kann das nicht validieren:** Sie bleibt per Konstruktion
100 %, weil das Modell die Nachbarpixel ja selbst als gedruckt markiert. Ob real
Lücken bleiben, sagt **nur das Papier**. Deshalb schrittweise erhöhen und jedes
Mal den echten Ausdruck prüfen — ein zu großer Radius unterdruckt still, statt
sichtbar zu scheitern. Startpunkt: `--spray-radius-mm 0.15 --spray-strength 1.0`
(entspricht ±1 Zeile bei Standardgeometrie).

### 5.6 Düsengruppierung

Standardmäßig wird jede der 152 Düsen einzeln angesteuert (`--nozzle-group 1`,
Default). Mit `--nozzle-group 2` werden je zwei benachbarte Düsen zu einer
gemeinsam adressierbaren Einheit zusammengefasst, die immer nur gemeinsam feuert
oder gar nicht. **Gilt nur in `--mode page`** (`CoverageEngine`) —
Zeilen-/Zeit-Modus packt feste Frames über einen anderen Pfad
(`rendering.frames_from_ink`), den diese Option nicht berührt;
`--nozzle-group 2` außerhalb von `--mode page` wird deshalb beim Parsen
abgelehnt.

Der physische Düsenabstand (`NOZZLE_PITCH_MM`, 0,087 mm) ändert sich dadurch
**nicht** — nur die kleinste noch einzeln ansprechbare vertikale Einheit wird
doppelt so groß: aus 0,087 mm pro Düse werden 0,174 mm pro adressierbarer
Einheit.

**Feuerregel (OR):** Eine Gruppe feuert, sobald **mindestens eine** ihrer beiden
Düsen ihr Pixel noch braucht (angefordert und noch nicht gedruckt) — so geht nie
ein gewolltes Pixel verloren, weil die Gruppe es nicht anfeuert. Der Preis:
Liegt eine Gruppe genau auf der Grenze zwischen einer Tinte- und einer
Nicht-Tinte-Zeile, wird beim Fertigwerden auch die Nicht-Tinte-Zeile
mitgedruckt (Kantenverbreiterung um bis zu eine Zeile) — die Gruppe kann nicht
nur zur Hälfte feuern.

⚠️ Diese Option ist **kein Fix** für wiederholtes Überdrucken (dafür ist
[5.5](#55-tintenausbreitung) da) und senkt auch nicht spürbar die CPU-Last —
gemessen kostet `CoverageEngine.step()` ~46,9 µs pro Aufruf (2,3 % eines Kerns
bei 500 Hz), unabhängig von `--nozzle-group`, weil weiterhin alle 152 Düsen pro
Sample durchlaufen werden, nur gruppiert. Sie existiert ausschließlich, weil
eine gröbere vertikale Adressierung gewünscht war.

**Nicht zu verwechseln mit `--nozzle-block-size`/`--nozzle-order`**
([8.3](#83-düsen-mapping)): Das korrigiert eine **Vertauschung** in der
Verdrahtung — eine Zeilen-*Permutation* — ändert aber nichts daran, dass jede
Düse einzeln feuert, und ist nur *außerhalb* von `--mode page` erlaubt (die
Blockpermutation ist nach Bildzeile indiziert, aber die Zuordnung Düse↔Zeile
verschiebt sich in `--mode page` mit jeder vertikalen Bewegung).
`--nozzle-group` vertauscht nichts, sondern bindet benachbarte Düsen fest
zusammen, und ist nur *innerhalb* von `--mode page` erlaubt. Die beiden
Optionen lösen unterschiedliche Probleme und schließen sich schon durch den
jeweils erforderlichen Modus gegenseitig aus.

---

## 6. Muster und Text drucken

`--calibrate` und `--pattern` sind Alternativen zu `text`: Statt Schrift wird
ein generiertes Muster gedruckt. Beides läuft durch **dieselbe** Pipeline wie
normaler Text — Positions- oder Zeit-Modus, Tracking, `--simulate`, `--dry-run`
und `--preview` funktionieren identisch.

### 6.1 Text

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

Die Höhe ist immer exakt 152 px (physische Düsen 8..159); die Breite ergibt
sich aus dem Text.

### 6.2 Mustergröße im Seiten-Modus

Genau weil `--mode page` nicht auf die 13,2 mm der 152 Düsen begrenzt ist,
lohnt sich für den Bring-up ein deutlich größeres `--calibrate`/`--pattern`-Bild
als die sonst übliche `IMAGE_HEIGHT`-Zeilenzahl:

| Option | Bedeutung |
|---|---|
| `--pattern-height-mm MM` | Physische Gesamthöhe von `--calibrate`/`--pattern` in mm (`rows = height_mm / NOZZLE_PITCH_MM`). Nur mit `--mode page` gültig — im Zeilen-/Zeit-Modus packt `frames_from_ink()` feste Frames mit genau `IMAGE_HEIGHT` Zeilen, eine andere Höhe wird dort mit einem klaren Fehler abgelehnt. Ohne diese Option bleibt das Muster bei `IMAGE_HEIGHT` Zeilen (13,2 mm, = `NOZZLE_BAR_WIDTH_MM`) gedeckelt. |
| `--pattern-square-height-mm MM` | Zeilenperiode in mm für checkerboard/h-stripes, überschreibt `--pattern-square-rows` (`square_rows = v / NOZZLE_PITCH_MM`). |

⚠️ **Seitenverhältnis-Falle:** Eine Bildzeile ist nur **0,087 mm** hoch
(`NOZZLE_PITCH_MM`). `--pattern-square-rows 20` (der Default) ist damit nur ca.
**1,74 mm** hoch, während `--pattern-square-mm 10` (der Default) **10 mm** breit
ist — ein ~5,8:1-Streifen statt eines Quadrats. Für tatsächlich quadratische
Kacheln `--pattern-square-height-mm` statt `--pattern-square-rows` verwenden.

```bash
# Großes Schachbrett in Seiten-Modus: 200mm x 100mm Gesamtfläche, 10mm-Quadrate.
python main.py --pattern checkerboard --mode page --page-calibration page_calibration.json \
    --pattern-length-mm 200 --pattern-height-mm 100 \
    --pattern-square-mm 10 --pattern-square-height-mm 10
```

### 6.3 Kalibrier-Lineal: --calibrate

Druckt eine durchgängige Basislinie mit Strichen über die **volle Höhe** alle
1 cm und **kurzen** Strichen alle 1 mm — wie ein Lineal. Damit lässt sich
`mm_per_column` bzw. `--dpi` exakt einstellen: Muster drucken, echten Abstand
zwischen zwei 1-cm-Strichen nachmessen, `--mm-per-column` entsprechend
korrigieren.

```bash
python main.py --calibrate --pattern-length-mm 200 --mm-per-column 0.087 --preview lineal.png
```

| Option | Bedeutung |
|---|---|
| `--pattern-length-mm` | Physische Länge des Musters in mm (Default 200) |
| `--calib-major-mm` | Abstand der vollen Striche (Default 10 = 1 cm) |
| `--calib-minor-mm` | Abstand der kurzen Striche (Default 1 = 1 mm) |

### 6.4 Testmuster-Presets: --pattern

| Preset | Zweck |
|---|---|
| `checkerboard` | Schachbrett — deckt Zeilen-/Spalten-Vertauschungen und Ausrichtungsfehler auf |
| `h-stripes` | Volle Zeilenbänder — jede Düse feuert durchgängig über die ganze Länge, eine tote Düse zeigt sich als durchgehende Lücke |
| `v-stripes` | Volle Spaltenbänder — prüft Spalten-/Trackingtiming; ungleiche Streifenbreite = ungleichmäßiger Vorschub |
| `diagonal` | Wiederkehrende Diagonale — eine vertauschte Düsenzeile zeigt sich sofort als Knick (siehe [8.3](#83-düsen-mapping)) |
| `solid` | Vollfläche — prüft Ink-Deckung/Banding |
| `precision-check` | Linien **parallel zur Düsenleiste** mit **verdoppelnden** Abständen entlang der Fahrtrichtung — siehe [6.5](#65-precision-check) |
| `drill_pattern` | Rastert eine externe Bilddatei auf die gewünschte physische Größe — siehe [6.7](#67-drill_pattern) |
| `ruler` | 1/10mm-Maßband — siehe [6.6](#66-ruler) |

```bash
python main.py --pattern checkerboard --pattern-square-mm 10 --pattern-square-rows 20
python main.py --pattern diagonal --mode line --preview diag.png
```

| Option | Bedeutung |
|---|---|
| `--pattern-length-mm` | Physische Länge des Musters in mm (Default 200) |
| `--pattern-square-mm` | Kachel-/Streifenbreite in mm (checkerboard, v-stripes, diagonal-Periode) |
| `--pattern-square-rows` | Kachel-/Streifenhöhe in Zeilen (checkerboard, h-stripes) — Achtung Seitenverhältnis, siehe [6.2](#62-mustergröße-im-seiten-modus) |
| `--pattern-line-cols` | Liniendicke in Spalten (`precision-check`, Default 1) |
| `--pattern-gap-start` | Erster Abstand in Spalten, verdoppelt sich danach (`precision-check`, Default 1) |
| `--pattern-image PATH` | Bilddatei für `--pattern drill_pattern` (jedes von PIL lesbare Format) |

### 6.5 precision-check

Druckt Linien **parallel zur Düsenleiste** über die volle Leistenhöhe, deren
Abstände **entlang der Fahrtrichtung** von Linie zu Linie **verdoppeln**. Eine
Linie ist dabei ein kurzer Moment, in dem alle 152 Düsen gleichzeitig feuern,
während der Wagen diese Spalte passiert.

```bash
# Abstände 1,2,4,8,16,32,64 Spalten, Linien 1 Spalte dick
python main.py --pattern precision-check --mode line

# Abstände 4,8,16,32,64 -- gröber, falls 1-2 Spalten ohnehin verschmieren
python main.py --pattern precision-check --pattern-gap-start 4

# Linien 3 Spalten dick: kräftiger, falls die dünnsten zu blass werden
python main.py --pattern precision-check --pattern-line-cols 3 --pattern-gap-start 2
```

`--pattern-gap-start` wählt die ganze Reihe: `1` → 1,2,4,8,16…, `2` →
2,4,8,16…, `4` → 4,8,16,32…

**Warum quer zur Fahrtrichtung und nicht längs:** Eine Linie *längs* der
Fahrtrichtung wäre eine einzelne durchgehend feuernde Düse — das misst den
Reihenabstand der Leiste selbst und sagt wenig über die bewegten Teile. *Quer*
dazu ist jede Linie ein Timing-/Positionsereignis, also genau die Achse, auf der
Positions-Nachlauf und Dosier-Intervall wirken. Erst diese Ausrichtung belastet
das Tracking wirklich.

**Auswertung:** Vom engen Ende her schauen und den ersten Abstand suchen, der
noch als Weiß durchkommt. Dieser Abstand ist die praktische Auflösung des
**gesamten** Systems **entlang der Fahrtrichtung** bei der gefahrenen
Geschwindigkeit — Tracking-Genauigkeit, Dosier-Timing und Tintenausbreitung
zusammen. Diese Kombination liefert keine Einzelmessung; deshalb ist das Muster
ein Ergänzungswerkzeug zu `--straightness` (das nur die Tracking-Seite isoliert
betrachtet) und nicht dessen Ersatz.

Beide Parameter zählen **Spalten, nicht Millimeter** — entlang der
Fahrtrichtung ist das Raster auf `--mm-per-column` quantisiert, und darauf
landet das Ergebnis. Damit sich das Gedruckte trotzdem mit einem Lineal
nachmessen lässt, gibt die CLI beim Erzeugen eine Tabelle mit beiden Einheiten
aus (hier mit `--mm-per-column 0.2` und `--pattern-length-mm 60`):

```
[precision-check] 9 lines parallel to the nozzle bar, 1 column(s) thick (0.200 mm):
  line   gap before (cols)   gap before (mm)   at col
     0                   -                 -        0
     1                   1             0.200        2
     2                   2             0.400        5
     3                   4             0.800       10
     4                   8             1.600       19
     5                  16             3.200       36
     6                  32             6.400       69
     7                  64            12.800      134
     8                 128            25.600      263
```

Die mm-Spalte skaliert mit `--mm-per-column`/`--dpi` mit. Die Tabelle erscheint
auch bei `--dry-run`/`--preview`, also bevor Tinte fließt. Passt bei der
gewählten Länge keine einzige Linie mehr, sagt sie das ausdrücklich, statt still
ein leeres Muster zu drucken. Eine Linie wird nie angeschnitten: passt die
letzte nicht mehr vollständig, entfällt sie — eine halb gedruckte Linie sähe wie
eine dünnere aus und würde als Auflösungsergebnis fehlgedeutet.

### 6.6 ruler

```bash
python main.py --pattern ruler --pattern-length-mm 100 --mode line --preview lineal.png
```

Druckt eine durchgehende Grundlinie mit festem Strichraster: alle 10 mm ein
langer Strich (20 mm), jeden Millimeter ein kurzer Strich (6 mm) — quer zur
Grundlinie gemessen, wie beim `--calibrate`-Lineal. Anders als dort ist hier
nichts weiter einstellbar: kein `--calib-major-mm`/`--calib-minor-mm`-
Äquivalent, nur `--pattern-length-mm`. Vorteil gegenüber `--calibrate`: läuft
durch dieselbe `--pattern`-Pipeline wie jedes andere Preset, also auch mit
`--mode page`, `--record` oder `--dry-run`/`--preview`.

Die Strichlänge wird auf die verfügbaren Zeilen begrenzt: im `--mode line`/`time`
(152 Düsen, 13,11 mm Leistenspannweite) passt ein 20-mm-Strich nicht hinein und
wird auf volle Leistenhöhe begrenzt — genau wie beim `--calibrate`-Lineal, dessen
langer Strich aus demselben physischen Grund immer volle Höhe hat. Erst im
`--mode page` mit `--pattern-height-mm` über ~13,1 mm hinaus erscheinen 20 mm /
6 mm in tatsächlicher Länge.

### 6.7 drill_pattern

⚠️ **`drill_pattern` liefert kein Bild mit.** Anders als die übrigen Presets
berechnet es nichts selbst, sondern liest eine Bilddatei ein. Diese Datei ist
**nicht** Teil dieses Repos — sie muss vom Anlagenbesitzer selbst bereitgestellt
werden, entweder am Default-Pfad `assets/drill_pattern.png` (relativ zum
`printhead/`-Paket, unabhängig vom aktuellen Arbeitsverzeichnis) oder über
`--pattern-image PATH`. Fehlt die Datei an beiden Stellen, bricht der Befehl mit
einer klaren Fehlermeldung ab (kein Traceback), die den exakt gesuchten Pfad
nennt:

```bash
$ python main.py --pattern drill_pattern --dry-run --mode line
printhead: error: --pattern drill_pattern needs an image, but none was found
at '/pfad/zum/repo/assets/drill_pattern.png'. Place an image there (any
PIL-readable format: PNG, JPG, BMP, ...), or point at a different one with
--pattern-image PATH.

$ python main.py --pattern drill_pattern --pattern-image mein_muster.png \
    --dry-run --mode line --preview drill.png
```

Das Bild wird unabhängig für Breite (`length_mm / mm_per_column` Spalten) und
Höhe (`rows`, standardmäßig `IMAGE_HEIGHT`) skaliert — **nicht**
seitenverhältnis-erhaltend. Das sieht auf den ersten Blick wie ein Bug aus, ist
aber richtig: eine Druckzelle ist `mm_per_column` breit, aber `NOZZLE_PITCH_MM`
hoch, also zwei unterschiedliche physische Maße — nur die unabhängige
Skalierung auf die angeforderte Spalten-/Zeilenzahl ergibt auf dem Papier die
korrekten Proportionen.

---

## 7. Diagnose und Messwerkzeuge

### 7.1 Übersicht der Diagnose-Flags

Jedes dieser Flags führt eine eigenständige Prüfung aus und beendet sich danach
— unabhängig vom Druck. Fehlt Hardware oder eine Bibliothek, kommt eine klare
Meldung statt eines Tracebacks.

| Flag | Wirkung |
|---|---|
| `--pos` | Gibt die **Live-Position** vom Amfitrack aus: `x/y/z` (mm) + Verfahr-Wert entlang `--advance-axis` + Spaltenindex; mit einem Seitenrahmen zusätzlich `page_u`/`page_v` und `yaw_deg`. Zugleich Kalibrierhilfe für Achse und `--mm-per-column`. Ctrl+C beendet. |
| `--pos-json` | Wie `--pos`, aber als NDJSON-Strom — was die Web-UI liest. |
| `--calibration-check` | Kalibrierungs-Gesundheitscheck: Wagen flach über die Seite schieben, **ohne zu drehen** — misst, wie stark der Gierwinkel trotzdem driftet. Braucht `--page-calibration PATH` oder `--page-frame simple`. Siehe [7.3](#73---calibration-check-gierwinkel-drift). |
| `--straightness CSV` | Offline-Auswertung eines `--profile-csv`-Laufs am Lineal. Braucht keine Hardware. Siehe [7.2](#72---straightness-tracking-präzision-am-lineal). |
| `--list-nodes` | Verbindet zum USB-Dongle und listet alle Nodes (`name`/`uuid`/`tx_id`), markiert die als „Sensor" erkannten. |
| `--scan-ble` | Scannt BLE und listet Geräte (`address` + `name`) — zum Finden der PrintheadBLE-Adresse (nutzbar mit `--address`). |
| `--nozzle-test` | Feuert per BLE ein Testmuster (alle 152 Düsen kurz an → Einzeldüse über alle Zeilen → Blank), um die Patrone zu prüfen. Berücksichtigt `--nozzle-block-size`/`--nozzle-order`, falls gesetzt. |
| `--ble-benchmark` | Misst den **BLE-Durchsatz** (Frames/s ohne Response) und die **Round-Trip-Latenz** (Frames mit Response). Siehe [7.4](#74---profile-und---ble-benchmark-echtzeit-und-timing). |

```bash
# Live-Position anschauen (Achse/Skalierung kalibrieren):
python main.py --pos --advance-axis x --mm-per-column 0.087
python main.py --pos --simulate                  # ohne Hardware

# Amfitrack-Nodes / BLE-Geräte auflisten:
python main.py --list-nodes
python main.py --scan-ble

# Düsen der Patrone testen:
python main.py --nozzle-test
```

`--verbose` ist die Alternative, die sich **mit** einem echten Druck
kombinieren lässt: eine live überschreibende Statuszeile (Position, bei `page`
zusätzlich `page u/v`, Gierwinkel/Roll/Pitch, `covered N/M`) während des
laufenden Drucks. `--pos` selbst ist ein eigenständiger Diagnosemodus und lässt
sich nicht mit einem Druck kombinieren.

### 7.2 --straightness: Tracking-Präzision am Lineal

Auswertung eines mit `--mode page --profile-csv` aufgezeichneten Laufs, bei dem
der Wagen an einer **geraden Kante (Lineal)** entlanggefahren wurde. Alle
geloggten `(u_mm, v_mm)` müssten dann auf einer Geraden liegen — wie weit sie
davon abweichen, ist das Maß für die Präzision.

```bash
# 1) Lauf aufzeichnen (Wagen am Lineal entlangfahren)
python main.py --mode page --page-calibration page_calibration.json \
    --pattern solid --profile --profile-csv lineal.csv

# 2) Offline auswerten -- braucht keine Hardware
python main.py --straightness lineal.csv
python main.py --straightness lineal.csv --straightness-bins 20
```

`--straightness-bins N` teilt die Strecke in N Abschnitte für die
positionsabhängige Auswertung (letzter Punkt der Liste unten).

Ausgegeben wird:

- **Linienwinkel** und Streckenlänge (Plausibilitätscheck, ob überhaupt genug
  gefahren wurde — unter 50 mm gibt es bewusst kein Urteil, weil über so wenig
  Weg fast alles gerade ist),
- **Abweichung** senkrecht zur Ausgleichsgeraden: RMS / p95 / max, jeweils
  zusätzlich **in Düsenreihen** (0,0868 mm) — das ist die Einheit, die
  entscheidet, ob eine Abweichung im Druck überhaupt sichtbar werden kann,
- **Aufteilung systematisch ↔ zufällig**: ein glatter Bogen (quadratischer Fit)
  gegen den Rest. 0,3 mm gleichmäßiger Verzug ist ein völlig anderes Problem
  als 0,3 mm Zittern — Ersteres mittelt sich nicht weg und ist typisch für
  Feldverzerrung, Letzteres dämpft `--smooth-ms`,
- **Abweichung nach Position** entlang der Linie (Bins mit Mittelwert / RMS /
  max) — beantwortet direkt „wo genau ist es krumm",
- **Wagen-Drehung und ihr Hebelarm-Effekt** (siehe Warnung unten).

**Warum die Gerade per Total-Least-Squares gefittet wird, nicht per
`v = m·u + c`:** Zum einen ist der Fehler zweidimensional — der Tracker ist in
beide Seitenachsen gleichermaßen ungenau —, also muss der **senkrechte**
Abstand minimiert werden, nicht der vertikale. Zum anderen läuft die
gewöhnliche Regression bei einer senkrechten Linie (unendliche Steigung) ins
Leere; ein Lauf überwiegend entlang `v` ist aber völlig normal und darf keinen
Sonderfall brauchen. TLS hat gar keine bevorzugte Achse.

⚠️ **Der mit Abstand größte Störterm ist die Wagen-Drehung, nicht der Tracker.**
Die geloggten `u_mm`/`v_mm` sind **düsenleisten-bezogen**: `PageMapper` addiert
den festen Sensor→Düsenleisten-Versatz
(`SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM`, 50,25 mm) *gedreht um den aktuellen
Gierwinkel*. Eine Drehung um **1° verschiebt den geloggten Punkt damit um
0,88 mm**, während der Sensor völlig stillsteht. Eine Hand, die beim
Entlangfahren leicht mitdreht, erzeugt also Millimeter an scheinbarer
Abweichung, die kein Tracking-Fehler sind. Genau dafür loggt
`PassProfiler.record_page_sample` das rohe Quaternion mit — dieses Tool ist der
Offline-Leser, der diese Spalten auswertet: es meldet die Drehspanne, die daraus
rechnerisch folgende scheinbare Abweichung und die Korrelation zwischen beiden.
Ist die Korrelation hoch, ist die Drehung die Ursache, nicht das Tracking.

Die Drehung wird als **3D-Gesamtwinkel** gegen das erste Sample gemessen, ist
also eine **Obergrenze**: Roll und Pitch stecken mit drin, schwenken die
Düsenleiste aber nicht so über die Seite wie der Gierwinkel. Für die saubere
Zerlegung bräuchte es `e_col`/`e_row`/Boresight aus der Kalibrierung, die ein
CSV allein nicht enthält.

⚠️ **Der Zahlenwert ist grundsätzlich eine OBERGRENZE für den Tracking-Fehler.**
Vier Dinge addieren sich darin und lassen sich aus dem CSV allein nicht trennen:
echter Tracker-Fehler (Rauschen + Feldverzerrung), die Handführung (Wagen nicht
durchgehend bündig am Lineal), die Geradheit des Lineals selbst, und die eben
beschriebene Wagen-Drehung. Ein guter Wert beweist also gutes Tracking; ein
schlechter Wert beweist noch nicht, dass der Tracker schuld ist.

Ein `--mode line`-CSV wird bewusst mit klarer Meldung abgewiesen: es enthält
nur einen 1D-Spaltenindex und eine Vorschubstrecke, also gar keine zweite
Seitenachse, gegen die sich Geradheit prüfen ließe.

### 7.3 --calibration-check: Gierwinkel-Drift

Das gemeldete Symptom war ein driftender Gierwinkel, obwohl der Wagen nur
verschoben, nie gedreht wurde. `--calibration-check` macht genau das messbar:
Live-Stream wie `--pos` (identische `position`-NDJSON-Events — die Web-UI kann
sie unverändert weiterverwenden), dazu am Ende (Ctrl+C) eine Zusammenfassung:

- verfahrene Strecke in `u`/`v` (mm) — Plausibilitätscheck, ob überhaupt genug
  bewegt wurde,
- Gierwinkel min/max/**Spanne** (°) — die Kopfzahl: ohne Drehung sollte das
  nahe 0 bleiben,
- Roll-/Pitch-Spanne (°) — die Größe, die über eine unsaubere Seitennormale in
  den Gierwinkel durchsickert (siehe [4.4](#44-kalibrierungsqualität)),
- **Korrelation** des Gierwinkels mit `u` bzw. `v` getrennt — trennt
  gewöhnliches Rauschen von **systematischer** Drift mit der Position: auf
  echten Daten dieser Anlage korrelierte die gemessene Neigung mit **+0,69**
  gegen `v`, bei nachweislich flachem Wagen.

**Verdikt-Schwellen:** Spanne bis **~2°** = unauffällig, bis **~4°** (nahe an
den 2–3°, die der Betreiber an seiner aktuellen Kalibrierung schon akzeptiert)
= grenzwertig, darüber = echtes Problem — entweder eine schlechte
Kalibrierungs-Seitennormale oder eine Feldverzerrung des Trackers.

**Unterscheidung:** dieselbe Stelle mit einer frisch, sorgfältig neu
abgefahrenen Kalibrierung wiederholen (verschwindet die Drift → war es die
Kalibrierung); bleibt sie trotz einer nachweislich guten Kalibrierung bestehen,
denselben Sweep an einer **anderen** Position/Höhe über der Basisstation
wiederholen — wandert die Drift mit der absoluten Trackerposition statt mit der
Kalibrierung, ist es Feldverzerrung, die kein Neu-Kalibrieren beheben kann.

⚠️ Vor den Gierwinkel-Schwellen wird geprüft, ob überhaupt genug gemessen wurde:
mindestens **20 Samples** und **50 mm** Weg. Darunter lautet das Verdikt
`INCONCLUSIVE` — ausdrücklich **kein Bestehen**. Warum das nötig war, steht in
[10.5](#105-kein-freispruch-ohne-messung).

```bash
python main.py --calibration-check --page-calibration page_calibration.json
python main.py --calibration-check --page-frame simple --simulate   # ohne Hardware
```

**Beispielausgabe eines simulierten Laufs** (Boustrophedon-Sweep über eine
A4-große Fläche, Wagen dabei durchgehend flach — aber mit künstlich injiziertem
Gierwinkel proportional zu `v`, stellvertretend für eine Seitennormale, deren
Fehler positionsabhängige Neigung in scheinbaren Gierwinkel verwandelt):

```
Calibration health check: slide the cart FLAT over the page, WITHOUT rotating it. Ctrl+C to stop and print the summary.
page u=  199.20  v=  210.42 mm  |  yaw= +5.60  roll= +0.00  pitch= +0.00 deg

---- calibration health check summary ----
  samples: 579
  travelled: u=199.2mm  v=280.3mm
  yaw: min=+0.00  max=+5.60  span=5.60 deg
  roll span: 0.00 deg   pitch span: 0.00 deg
  yaw correlation: vs u = -0.25  vs v = +1.00
  verdict: BAD: yaw span 5.60 deg is well beyond what a flat, non-rotating sweep should show. ...
Stopped calibration check.
```

Und zum Vergleich derselbe Sweep ganz ohne injizierten Fehler (Wagen bleibt die
ganze Zeit exakt in derselben Orientierung — die Korrelation ist dann
`None`/„n/a", nicht 0: bei einer Gierwinkel-Reihe ohne jede Streuung ist ein
Korrelationskoeffizient mathematisch undefiniert):

```
---- calibration health check summary ----
  samples: 553
  travelled: u=193.1mm  v=280.0mm
  yaw: min=+0.00  max=+0.00  span=0.00 deg
  roll span: 0.00 deg   pitch span: 0.00 deg
  yaw correlation: n/a (not enough motion/variation collected)
  verdict: OK: yaw span 0.00 deg is at or under the ~2 deg 'fine' mark for a flat, non-rotating sweep -- consistent with a good calibration.
Stopped calibration check.
```

### 7.4 --profile und --ble-benchmark: Echtzeit und Timing

Im Positions-Modus wird zwar die *richtige* Spalte aus der Position gewählt,
aber jede Spalte muss noch über BLE **gesendet und von der Firmware
verarbeitet** werden. Genau das ist begrenzt (die Firmware fordert ein
Verbindungsintervall von 7,5–15 ms an; gepufferte Writes ohne Response). Bewegt
sich der Kopf schneller, als Spalten geliefert werden können, hinken die
Spalten der realen Position hinterher.

**`--ble-benchmark`** misst die Obergrenze der BLE-Strecke:

```bash
python main.py --ble-benchmark --mm-per-column 0.087
```

Ausgabe: erreichter Durchsatz (Spalten/s), Round-Trip-Latenz (avg/p95/max) und
daraus die **maximale Kopfgeschwindigkeit**, bis zu der Spalten noch mithalten
(`Durchsatz × mm_per_column`).

**`--profile`** instrumentiert einen echten Positions-Durchlauf:

```bash
python main.py "Test" --profile
python main.py "Test" --profile --profile-csv timing.csv   # zusätzlich CSV-Log
```

Live werden Kopfgeschwindigkeit, **geforderte** vs. **erreichte** Spaltenrate
und die BLE-Write-Latenz ausgegeben (`load > 1.0` = BLE kommt nicht hinterher).
Am Ende ein Fazit inkl. „bis ~X mm/s halten die Spalten mit".

Im **Seiten-Modus** misst `--profile` **Spalten pro Sekunde**, nicht
Musterwechsel pro Sekunde: da die Firmware jede empfangene Spalte genau einmal
feuert, ist eine Spalte ein Tintentropfen, ein „Musterwechsel" dagegen nur eine
Abtastung, bei der etwas fällig war. Verglichen wird gegen
`--ble-write-ceiling × Spalten pro Write` (Default 270 × 12 bei MTU 247 ≈
3200 Spalten/s). Wird die Decke überschritten, geht **Tinte verloren**, nicht
bloß Aktualität — `PatternSender` verwirft dann die ältesten Spalten und zählt
sie mit.

**CSV-Spalten:**

```
Seiten-Modus: t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,x,y,z,qx,qy,qz,qw
Zeilen-Modus: t_s,column,advance_mm,write_latency_ms,speed_mm_s,x,y,z
```

`x,y,z` ist die **rohe** Sensorposition, `u_mm`/`v_mm` die
Seitenebenen-Projektion davon (Kalibrierung, Düsenversatz und Gierwinkel
eingerechnet), `advance_mm` ein 1-D-Vorschub. Die Spalten heißen bewusst wie
die NDJSON-Felder aus `--pos-json` (drei Nachkommastellen), damit dieselbe
Auswertung beide Quellen lesen kann. Fehlt die Position, bleiben die Felder
**leer**, nicht `0,0,0` — anders als beim Quaternion wäre eine Null hier ein
plausibler Messwert (direkt am Sender-Ursprung) und würde als echte Angabe
gelesen. Im Seiten-Modus stehen sie **vor** der Quaternion-Gruppe, damit die
Orientierung das Zeilenende bleibt.

`qx,qy,qz,qw` (nur Seiten-Modus) ist das rohe Orientierungs-Quaternion des
Sensors, sofern die Hardware es gerade geliefert hat (sonst leer, nicht
`0,0,0,0`). Ursprünglich reine Diagnosedaten für die Hypothese, dass eine
Rotation des Wagens zusammen mit dem festen Hebelarm Sensor→Düsenleiste die
beobachteten Verzerrungen erklärt. Die Hypothese hat sich an echten Daten
bestätigt (`pass5.csv`) und ist inzwischen live korrigiert
([4.3](#43-gierwinkel-und-boresight)); das Quaternion bleibt als Rohdaten
erhalten.

⚠️ Trotz der Rohwerte ersetzt die Profil-CSV `--pos --pos-json` **nicht** für
Rauschmessungen: geschrieben wird nur, wenn tatsächlich Spalten rausgehen
(Seiten-Modus nur bei fälliger Tinte und nicht-leerem Muster, Zeilen-Modus nur
beim Spaltenwechsel). Sie ist damit keine gleichmäßige Zeitreihe. Im
Zeilen-Modus kommt hinzu, dass ein ganzer Spalten-Batch in einem BLE-Vorgang
rausgeht und die Zeilen dieses Batches sich dieselbe Position teilen — die
Wiederholung ist die Bündelung, kein eingefrorenes Tracking.

### 7.5 --record: was tatsächlich aufs Papier geht

```bash
python main.py "Test" --record recon.png
python main.py "Test" --simulate --mode line --dry-run --record recon.png  # ohne Hardware
```

In der Web-UI gibt es dafür den **🎞 Record**-Button (zeigt das Vergleichsbild
direkt an).

**Im Zeilen-/Zeit-Modus** rekonstruiert `--record`, was aufs Papier geht: jeder
gesendete Frame wird mit der Kopfposition aufgezeichnet und danach als Bild
gespeichert, das die Frames auf ihre reale Position mappt. Oben das
beabsichtigte Bild, unten das gesendete — so werden Stauchung/Verlust sichtbar,
wenn bei schneller Bewegung mehrere Spalten an *einer* Position zusammenfallen.

Hinweis: `--record` erfasst, was der Client sendet und *wo* — nicht, was auf dem
Funkweg evtl. verloren geht. Ist die Rekonstruktion sauber, liegt ein
verbleibendes Problem an BLE-Paketverlust/Firmware; ist sie schon gestaucht,
liegt es am Sende-/Positionstiming.

> Ohne per-Frame-Rückmeldung der Firmware lässt sich nicht *beweisen*, dass eine
> Spalte physisch rechtzeitig gedruckt wurde; Write-Latenz (`--profile`) und
> Round-Trip mit Response (`--ble-benchmark`) sind die bestmöglichen Proxys.

**Im Seiten-Modus** gibt es nichts nachzubilden — `CoverageEngine` führt schon
während des Drucks live Buch, welches Pixel getroffen wurde. Das PNG zeigt
gestapelte Panels:

| Panel | Inhalt |
|---|---|
| INTENDED | das Zielbild |
| COVERED | tatsächlich getroffen (aus `fired`) |
| MISSED | gewollt, aber nie getroffen |
| THIN | Tinte da, Dosis unvollständig — erscheint nur, wenn nicht leer. Bedeutet „das kam hell heraus", nicht „Stelle verpasst"; im Wesentlichen die letzte Spalte vor einem Richtungswechsel oder dem Seitenrand |
| PATH | farbig: blau = **Sensor-Mittelpunkt**, orange = **Düsenleisten-Mitte** (nicht Düse 0) |

Damit lässt sich eine MISSED-Stelle direkt gegen die Fahrspur prüfen — ist der
Wagen dort nie vorbeigekommen, oder war er so schnell, dass der Tracker diese
Spalten nie abgetastet hat?

Das ganze PNG wird standardmäßig **3-fach vergrößert**
(`recording.DEFAULT_RECORD_SCALE`) — INTENDED/COVERED/MISSED blockig (jeder
Block bleibt exakt eine reale Düsenzeile/-spalte, kein Weichzeichnen, das eine
falsche Sub-Pixel-Genauigkeit vortäuschen würde), das PATH-Panel direkt in
voller Zielauflösung gezeichnet (keine verpixelten Linien/Zahlen).

Auf beiden Spuren wird zusätzlich alle **2 Sekunden**
(`recording.DEFAULT_MARKER_INTERVAL_S`) ein größerer, durchnummerierter Punkt
gesetzt — 1 beim allerersten Sample, dann 2, 3, 4 … im 2-Sekunden-Takt, auf
Sensor- und Düsenleisten-Spur jeweils zur **exakt gleichen** Pass-Zeit. Damit
lässt sich ablesen, wo Sensor und Düsenleiste zu welchem Zeitpunkt standen —
z. B. um eine MISSED-Stelle einem bestimmten Moment im Pass zuzuordnen.

```bash
python main.py --pattern checkerboard --mode page --page-frame simple \
    --pattern-length-mm 60 --pattern-height-mm 100 --record coverage.png
```

⚠️ **Damit der Sensor-Pfad überhaupt sichtbar wird:** Sensor und Düsenleiste
sitzen ~50 mm auseinander ([4.2](#42-sensor-düsen-versatz)). Bei konstanter
Ausrichtung (kein Gieren während des Passes) liegt die blaue Sensor-Spur deshalb
**komplett außerhalb** eines nur ~13–20 mm hohen Zielbilds — sie ist real
vorhanden, nur schlicht nie im sichtbaren Canvas. Erst bei einem ausreichend
hohen Zielbild (`--pattern-height-mm` deutlich über ~50 mm) oder bei einem Pass
mit echtem Gieren laufen beide Spuren im selben Bildausschnitt zusammen.

In der Web-UI erscheint das PATH-Panel automatisch im **🎞 Record**-Vergleichsbild,
sobald `--mode page` aktiv ist — keine zusätzliche Option nötig.

---

## 8. Referenz

Die vollständige Optionsliste liefert `python main.py --help`. Hier stehen die
Optionen, die eine Erklärung brauchen.

### 8.1 Positions- und Tracking-Optionen

| Option | Bedeutung |
|---|---|
| `--mode page\|line\|time` | Betriebsart (Default `page`, siehe [Abschnitt 2](#2-betriebsarten)) |
| `--page-frame calibrated\|simple` | Welchen 2D-Rahmen `--mode page` benutzt (Default `calibrated`). `simple` braucht keine Kalibrierung und schließt `--page-calibration` aus. Siehe [2.3](#23-seitenrahmen-einfach). |
| `--page-calibration PATH` | Gespeicherte `page_calibration.json` laden |
| `--simple-boresight QX QY QZ QW` | Nur mit `--page-frame simple`: pinnt die Gier-Referenzpose fest, statt sie beim START automatisch (und ungeprüft) zu erfassen. Vier **leerzeichengetrennte** Zahlen. |
| `--boresight-deg GRAD` | Additive Feinjustage der aufgenommenen Boresight-Rotation |
| `--sensor-offset-row-mm MM` | Sensor → Mitte der Düsenleiste, Zeilenachse (Default `-50.25`) |
| `--sensor-offset-col-mm MM` | Dasselbe, Spaltenachse (Default `0.0`) |
| `--startpoint-anchor center\|left-middle\|top-left` | Welcher Punkt des Musters beim Startpoint-Taster gesetzt wird (Default `center`) |
| `--origin button\|startpoint` | Was den Nullpunkt setzt (START-Taster oder Startpoint-Charakteristik) |
| `--mm-per-column MM` | Breite einer gedruckten Spalte (Default `0.087`) |
| `--dpi N` | Alternativ zu `--mm-per-column`: setzt `mm/Spalte = 25.4/DPI` |
| `--drops-per-pixel F` | Tintendosis, siehe [5.1](#51-dosierung---drops-per-pixel) (Default 2) |
| `--spray-radius-mm MM` / `--spray-strength F` | Tintenausbreitungsmodell, siehe [5.5](#55-tintenausbreitung) (Default beide `0` = aus) |
| `--nozzle-group 1\|2` | Vertikale Adressierungs-Granularität, nur in `--mode page` (Default 1) |
| `--speed-warning-mm-s F` | Schwelle für Warn-Charakteristik und Ampel (Default 25.0) |
| `--latency-compensate-s S` | Extrapoliert nur die an die Coverage-Engine übergebene Position. Default `0.0` = aus. Siehe [5.4](#54-latenz-kompensation). |
| `--smooth-ms MS` | Tiefpass-Zeitkonstante (ms) gegen das verrauschte Amfitrack-Signal; `0` = aus, größer = glatter aber mehr Nachlauf. **Default `0` (aus).** Siehe die Anmerkung unten. |
| `--min-move MM` | Deadband; darunter gilt der Kopf als stehend (Default 0.05) |
| `--poll-hz HZ` | Abtastrate der Position (Default 500). Ein Spaltenübergang kann nur einmal pro Abtastung bemerkt werden — bei 500 Hz und 20 mm/s sind das 0,04 mm = ein Fünftel Spalte. Bestimmt zusammen mit `--mm-per-column` die Geschwindigkeitsgrenze ([5.2](#52-was-die-geschwindigkeit-begrenzt)). |
| `--progress-hz HZ` | Rate des `coverage`-Ereignisses im `--progress-json`-Strom (Default 25). Es geht dabei kein Tropfen verloren. |
| `--timeout S` | Abbruch eines Durchlaufs nach S Sekunden (Default 180) |
| `--batch-cols N` | Spalten je BLE-Write; `0` = automatisch aus der MTU (Default) |
| `--ble-write-ceiling N` | Angenommene Write-Decke für `--profile` (Default 270/s) |
| `--advance-axis x\|y\|z` / `--axis-sign ±1` | Verfahrachse im Zeilen-Modus |
| `--auto-calibrate` / `--calib-distance MM` | Verfahrrichtung aus der ersten Bewegung messen (Zeilen-Modus) |
| `--vendor-id` / `--product-id` | USB-IDs des Amfitrack-Dongles (Default `0x0C17` / `0x0D12`) |
| `--sensor-id` | optionaler `tx_id`-Filter unter den „Sensor"-Nodes (Default: alle) |
| `--simulate` | Fake-Tracker (keine Hardware) zum Testen des Loops |
| `--dry-run` | Nichts über BLE senden |
| `--verbose` | Live-Statuszeile während des Drucks (siehe [7.1](#71-übersicht-der-diagnose-flags)) |

> ⚠️ **Anmerkung zu `--smooth-ms`:** Der wirksame Default ist **`0.0` (aus)** —
> das ist der argparse-Default in `cli.py`, und die CLI reicht ihn immer an
> `TrackingSettings` durch. Der Dataclass-Default in `config.py` steht dagegen
> auf `12.0` und greift nur, wenn `TrackingSettings` direkt im Code gebaut wird
> (Tests, Web-UI-Pfade). **Über die Kommandozeile ist die Glättung also aus,
> solange sie nicht ausdrücklich angefordert wird.** Ob der beabsichtigte
> Default 0 oder 12 ist, ist eine offene Frage — beides ist vertretbar, und die
> Entscheidung gehört auf die Hardware, nicht in eine Codeänderung nebenbei.
> Bis dahin beschreibt der Hilfetext der Option das tatsächliche Verhalten und
> nennt die Abweichung ausdrücklich.

**Verbindung und Ablauf:**

| Option | Bedeutung |
|---|---|
| `--device-name NAME` | BLE-Gerätename, nach dem gescannt wird (Default `PrintheadBLE`) |
| `--address ADDR` | Direkt zu einer BLE-MAC/UUID verbinden und den Scan überspringen (Adresse per `--scan-ble` ermitteln) |
| `--scan-timeout S` | Wie lange gescannt wird (Default 10) |
| `--auto-start` | Sofort loslegen, ohne auf den START-Taster zu warten |
| `--once` | Nach einem Durchgang beenden (Default: weiter auf den nächsten START warten) |
| `--no-track` | Tracking abschalten — erzwingt `--mode time` |
| `--period S` | Sekunden pro Spalte im Zeit-Modus (Default 0.03) |
| `--preview PFAD` | Zielbild als PNG schreiben, statt/zusätzlich zum Drucken |
| `--record PFAD` | Rekonstruktions-PNG schreiben, siehe [7.5](#75---record-was-tatsächlich-aufs-papier-geht) |
| `--profile` / `--profile-csv PFAD` | Timing instrumentieren, siehe [7.4](#74---profile-und---ble-benchmark-echtzeit-und-timing) |
| `--progress-json` | NDJSON-Fortschrittsstrom (was die Web-UI liest); unterdrückt `--verbose` |

### 8.2 Textoptionen

Siehe [6.1](#61-text).

### 8.3 Düsen-Mapping

Falls die physischen Düsen in Blöcken fester Größe verdrahtet sind, deren
Reihenfolge nicht der tatsächlichen (vertikalen) Position entspricht, korrigiert
`--nozzle-block-size` + `--nozzle-order` das vor dem Senden: Die Bildzeilen
werden in Blöcken der angegebenen Größe gemäß der neuen Reihenfolge umsortiert.

`--nozzle-order` ist **1-indiziert** und gibt pro Block-Slot an, welche
ursprüngliche Position dort landen soll. Beispiel: Block-Standardreihenfolge
`1,2,3,4,5` wird zu `2,3,4,1,5` → Slot 1 bekommt, was ursprünglich Düse 2 war,
Slot 2 bekommt Düse 3, Slot 3 bekommt Düse 4, Slot 4 bekommt Düse 1, Slot 5
bleibt. Das Muster wiederholt sich für alle 152 Zeilen; passt die Blockgröße
nicht exakt (z. B. 152 nicht durch 5 teilbar), bleibt der letzte unvollständige
Block unverändert (eine Meldung weist darauf hin).

```bash
python main.py "Test" --nozzle-block-size 5 --nozzle-order 2,3,4,1,5
```

**Verifikation ohne echten Druck:** `--nozzle-test` wendet dasselbe Mapping auf
den Düsen-Sweep an, sodass man die korrigierte Reihenfolge direkt an der Patrone
sehen kann:

```bash
python main.py --nozzle-test --nozzle-block-size 5 --nozzle-order 2,3,4,1,5
```

Nur außerhalb von `--mode page` erlaubt — nicht zu verwechseln mit
`--nozzle-group` ([5.6](#56-düsengruppierung)).

### 8.4 BLE-Protokoll

Maßgeblich ist `README_BLE_INTERFACE.md` im Firmware-Repo.

| | |
|---|---|
| Device name | `PrintheadBLE` |
| Service | `d0567401-5a22-c59f-5243-8c0fa18e257b` |
| Nozzle char | `41a9348e-2f6b-8db1-934d-743c6f17649a` (Write/WriteNoRsp, Vielfaches von 19 Bytes) |
| Start btn | `b473a21f-6e58-6380-2647-abd7cd4a904e` (Read/Notify, 1 Byte 0/1) |
| Startpoint | `cc1087f5-1d92-6ca4-b84f-3e5880e6713d` (Read/Notify, 1 Byte 0/1) |
| Mode | `f5ad7c1f-f6e1-4dd7-bbb7-d8b9286a88c6` (Read/Write, 1 Byte 0=line/1=page) |
| Speed warning | `58c05253-945f-48fc-a26c-989c785d6678` (Read/Write, 1 Byte 0/1) |
| Process stop | `a2e1c9d4-7f3b-4a8e-9c1d-5b6f8e2a0d47` (Write, 1 Byte, nur `1` gültig) |

**Eine Spalte = 19 Bytes = 152 Nozzle-Bits, LSB-first:** Bit `j` (Byte `j//8`,
Bit `j%8`). Die Firmware paddt oben und unten je ein Nullbyte auf das alte
21-Byte-Layout, d. h. Frame-Bit `j` feuert physisch Nozzle `j + 8`; Bildzeile
`y` ↦ Bit `j = y`.

Die 19 Bytes sind so gewählt, dass **eine Spalte in die Standard-Nutzdatengröße
von 20 Byte passt** (ATT-MTU 23 − 3 Byte Header). Mit dem früheren 21-Byte-Frame
ging ein Write-Without-Response nicht mehr in ein Standardpaket, was den Druck
still auf ~21 grobe Blöcke reduzierte.

**Mehrere Spalten pro Write:** ein Write darf beliebig viele Spalten
hintereinander tragen (jedes Vielfache von 19 Bytes, max. 32
= `BLE_NOZZLE_MAX_COLS_PER_WRITE`). Die Firmware stellt sie in eine
Warteschlange und druckt **jede genau einmal, in Reihenfolge**. Wie viele
Spalten pro Write gehen, ergibt sich aus der ausgehandelten MTU: Die Firmware
fordert MTU 247 an, davon 244 nutzbar → `244 // 19 = 12` Spalten
(`--batch-cols`, Default `0` = automatisch).

⚠️ **12 ist eine Obergrenze, kein Ziel.** Der Sender bündelt
`min(batch_cols, 32, Warteschlangenlänge)` — es gibt keinen Timer und nichts
wird zurückgehalten. Liegt eine Spalte an, geht eine Spalte raus.

`--batch-cols 1` erzwingt eine Spalte pro Write für ältere Firmware **ohne**
Spalten-Queue — diese verwirft längere Writes kommentarlos.

### 8.5 Amfitrack-Anbindung

Der Zugriff erfolgt über die USB-Pakete `amfiprot` und `amfiprot_amfitrack`
(6-DOF-Ausgabe: Position X/Y/Z + Orientierung). `AmfitrackTracker` in
`printhead/tracking.py` bildet das erprobte Verbindungsverhalten ab:

- **Verbindung**: erst `USBConnection(vendor_id, product_id)` (Sensor-PID
  `0x0D12`), bei Fehler Fallback auf die Source-PID `0x0D01`.
- **Node-Auswahl**: alle Nodes, deren `node.name` „Sensor" enthält, werden als
  `Device` angebunden (optional per `--sensor-id` auf eine `tx_id` eingegrenzt);
  `conn.start()` erst danach.
- **Position**: gelesen aus `payload.emf.pos_x / pos_y / pos_z` (in **mm**).
  Diese bestätigten Namen sind in `_extract_position()` primär; einige
  Alternativlayouts (`.position.x/y/z`, flach `.x/.y/.z`, `position_x_in_m`)
  bleiben als Fallback für abweichende SDK-Versionen. Falls die SDK die Position
  anders liefert, dort anpassen.

### 8.6 Abhängigkeiten

`bleak`, `pillow`, `numpy`, `amfiprot`, `amfiprot-amfitrack` (siehe
`requirements.txt`). Für die Web-UI zusätzlich `requirements-ui.txt`.

---

## 9. Tests und Messreihen

### Hardwarefreie Tests

25 Testdateien in `tests/`, alle ohne Hardware und ohne pytest lauffähig:

```bash
python tests/test_frames.py          # Protokoll-Äquivalenz der Frame-Erzeugung
python tests/test_coverage.py        # Feuerentscheidung + Dosis-Buchführung
python tests/test_rotation.py        # Gierwinkel aus dem Quaternion
python tests/test_calibration.py     # Seitenkalibrierung + Fit-Metriken
python tests/test_ui_live.py         # Web-UI-Livepanels und Ampel
```

Alle Tests am Stück:

```bash
for t in tests/test_*.py; do python "$t" >/dev/null && echo "PASS $t" || echo "FAIL $t"; done
```

Zusätzlich als Rauchtest ohne Hardware:

```bash
python main.py "Hi" --simulate --mode line --dry-run
python -m printhead --help
```

### Messreihe an der Hardware

Ausführbare Testprotokolle für die acht Eigenschaften der Anlage
(Kantenqualität, Tracker-Genauigkeit über die Entfernung, Auflösung,
Bildqualität, Wiederholbarkeit, Rechtwinkligkeit, Geschwindigkeitslimit,
Blattausrichtung) stehen in **[`TESTS.md`](TESTS.md)** — jeweils mit
Durchführung, auszufüllenden Messtabellen und Bewertungskriterium.
Vorangestellt ist ein Vorflug-Check (tote Düsen, BLE-Grenzwerte,
Versatz-Vorzeichen, Kalibrierungsgesundheit), ohne den mehrere Tests etwas
anderes messen als gedacht.

Die zugehörigen Auswerteskripte liegen in `funktionen/`:

| Skript | Wertet aus |
|---|---|
| `geradheit_messreihe.py` | Geradheits-Messreihe über mehrere Abstände |
| `geschwindigkeit_profil.py` | Geschwindigkeitsprofil aus einer `--profile-csv` |
| `image_line_to_angle.py` | Linienwinkel aus einem fotografierten Druck |
| `image_quad_coverage.py` | Deckung eines Vierecks aus einem Foto |
| `precision_check_auswertung.py` | `precision-check`-Muster auswerten |
| `rauschen_entfernung.py` | Tracker-Rauschen über die Entfernung |
| `ruler_auswertung.py` | `ruler`-Muster nachmessen |

---

## 10. Anhang: behobene Fehler und Verifikationen

Dieser Abschnitt hält die Fehlerbilder fest, die während der Entwicklung
auftraten, samt Messreihen und Gegenproben. Für die Bedienung ist er nicht
nötig — er beantwortet die Frage „warum ist das so gebaut" und dient als Beleg
dafür, dass die Zahlen im Hauptteil gemessen und nicht geschätzt sind.

### 10.1 START-Taster musste manchmal zweimal gedrückt werden

**Symptom:** Der START-Taster wurde immer korrekt erkannt und angezeigt, der
Pässe-Zähler (`../..  gedruckt`) stieg beim ersten Druck normal hoch — aber es
floss weder Strom noch Tinte. Erst ein zweiter Tastendruck brachte den Druck
wirklich in Gang.

**Ursache:** Der physische START-Taster auf der Firmware ist ein reiner
Hardware-Toggle (`mainloop()` in `main.c`): Druck = Start, wenn gerade gestoppt;
Druck = Stop, wenn gerade am Laufen. Die Firmware hat aber keine andere
Möglichkeit zu erfahren, dass ein vom Client gesteuerter Pass **von selbst** zu
Ende gegangen ist (Timeout, volle Deckung, oder ein per Startpoint-Taster
ausgelöster Abbruch) — sie bleibt intern „running", obwohl der Client den Pass
bereits als beendet betrachtet. Der **nächste** physische Tastendruck des
Bedieners ist damit aus Firmware-Sicht bereits der **zweite** Druck seit dem
letzten Start und wird als STOP gelesen, während der Client denselben
Tastendruck als Beginn eines **neuen** Passes interpretiert: der Deckungszähler
springt normal hoch, aber der Druckkopf schaltet währenddessen ab — kein
Fehler, keine Tinte, kein Strom. Erst der Druck danach startet wieder wirklich.
Ein reiner Zwei-Repo-Bug: keine der beiden Seiten allein sieht genug, um den
Zustand des jeweils anderen zu rekonstruieren.

**Fix:** Neue GATT-Characteristic `PROCESS_STOP_UUID` (write-only, 1 Byte), die
der Client nach **jedem** Pass schreibt — unabhängig vom Modus
(`line`/`page`/`time`) und unabhängig davon, ob der Pass normal zu Ende ging
oder mit einem Fehler abbrach (`_run_ble()`'s `finally`-Block, läuft auf jedem
Ausstiegspfad). Die Firmware konsumiert das Signal genau einmal und beendet
I2S-Ausgabe/`process_running` selbst, statt auf einen zweiten physischen
Tastendruck zu warten. Der Schreibversuch selbst wirft nie eine Exception
(`request_process_stop()` fängt jeden Fehler ab und gibt nur `False` zurück) —
ein Fehlschlag hier darf niemals den Rücksprung zu
`Waiting for next START press ...` verhindern.

Betrifft beide Repos: die Firmware-Seite (`ble_server.c`/`main.c`/
`README_BLE_INTERFACE.md`, dort „6) Process Stop Characteristic") und diese
Client-Seite (`geometry.py`, `ble_client.py`, `controller.py`). Beide Seiten
müssen den passenden Stand tragen, damit der Fix greift — mit alter Firmware
bei neuem Client (oder umgekehrt) bleibt das ursprüngliche Verhalten (Taster
funktioniert, Characteristic wird ignoriert bzw. nie geschrieben).

### 10.2 Gierwinkel-Singularität bei 180°

Die relative Rotation (aktuelle Pose gegen Boresight) wurde zunächst als
Rotationsmatrix gebaut und über deren Achsen-Winkel-„Rotationsvektor" (Log-Map
von SO(3)) auf die Seitennormale projiziert. Mit der echten Kalibrierung des
Betreibers (`e_col`/`e_row`/Boresight aus dessen `page_calibration.json`) und
einer synthetischen, reinen Drehung um genau diese Seitennormale gemessen:

```
 wahr | alte Methode | swing-twist
   0° |         0,0° |        0,0°
  45° |        45,0° |       45,0°
  90° |        90,0° |       90,0°
 135° |       135,0° |      135,0°
 180° |       934,2° |      180,0°   <-- numerischer Ausreißer an der Singularität
 225° |      -135,0° |      225,0°
 270° |       -90,0° |      270,0°
```

Bis 135° stimmt die alte Methode, genau bei 180° liefert sie Unsinn (die
Division durch `sin(angle)` im Rotationsvektor geht dort gegen 0), danach ist
sie um exakt 360° vorzeichenverkehrt — eine Rotations**matrix**, anders als ein
Quaternion, „weiß" nicht mehr, in welche Richtung eine Drehung über 180° hinaus
ging. Auf echter Hardware zeigte sich genau diese Fehlerform schon **vor** der
sauberen 180°-Grenze: ein Sprung von **-109° auf +109°** bei einer echten
180°-Drehung.

`printhead/rotation.py` (`yaw_about_normal`) berechnet den Gierwinkel jetzt über
eine **Swing-Twist-Zerlegung direkt auf dem Rotations-Quaternion**
(`quat * conj(boresight_quat)`, über `_quat_multiply`/`_quat_conjugate` —
bewusst *nicht* über eine Matrix: die Rückrichtung Matrix→Quaternion hätte
dieselbe Art Instabilität nahe 180° wieder eingeschleppt):

```
v, w = Vektor-/Skalarteil von quat_rel
twist_rad = 2 * atan2(dot(v, n_hat), w)
```

Kein Ausdruck darin geht bei 180° gegen 0/0 — keine Singularität. Ein gewisser
Umschlag ist für **jede** Ein-Zahl-Winkeldarstellung mathematisch unvermeidbar
(dieselbe physische Orientierung ist ab einer vollen Umdrehung über zwei
unterschiedlich vorzeichenbehaftete Quaternionen erreichbar), landet hier aber
erst bei einer **vollen** Drehung (±360°) statt schon bei 180° — weit jenseits
des größten je auf dieser Anlage gemessenen Gierwinkels (75,6° über einen ganzen
Durchlauf) — und ist für die Druckkorrektur ohnehin folgenlos, weil dort nur
`sin`/`cos` des Gierwinkels verwendet werden (`PageMapper.project`), beide
360°-periodisch.

Die neue Methode ist zusätzlich nicht nur singularitätsfrei, sondern **exakt
statt nur näherungsweise**, sobald Neigung (Roll/Pitch) mit dabei ist: Bei 75°
Neigung um eine Diagonalachse plus 40° injiziertem Gierwinkel lieferte die alte
Rotationsvektor-Methode 34,0° statt 40° — die neue exakt 40,0°.
`cart_rotation_angles` (Roll/Pitch, rein diagnostisch) liest Roll/Pitch jetzt
aus dem um den Twist bereinigten „Swing"-Quaternion statt direkt aus der vollen
Relativ-Rotation — sonst kippt Roll/Pitch fälschlich um, sobald allein der
Gierwinkel über 180° steigt.

#### Nachtrag: was der Wertebereich ±360° kostet

Die Methode liefert den Gierwinkel im Bereich **(−360°, +360°]** — absichtlich
**nicht** auf ±180° geklemmt, weil eine Klemmung den beobachteten Sprung nur
von seiner jetzigen Stelle auf 180° zurückverlegen würde, statt ihn zu
beseitigen.

Der Preis dafür, gemessen: Die Methode ist **nicht mehr invariant gegen die
Quaternion-Doppelüberdeckung**. `q` und `−q` sind dieselbe physische
Orientierung; die alte Matrix-Methode war konstruktionsbedingt immun
(`R(q) == R(−q)`), die neue liest die Quaternion-Komponenten direkt. Mit der
echten Kalibrierung gemessen: `−q` statt `q` verschiebt die Ausgabe um **exakt
360°**, und zwar bei **jedem** getesteten Winkel (0/30/75/90/135/179/180/225/
270/315°) — nicht nur nahe einer vollen Umdrehung. Dasselbe gilt für ein
vorzeichenverkehrtes `boresight_quat`.

Bewusst akzeptiert, aus diesen Gründen:

- **Der Druck kann davon nicht betroffen sein.** `PageMapper.project()` und
  `CoverageEngine` verwenden nur `sin`/`cos` dieses Winkels, beide exakt
  360°-periodisch — numerisch über einen vollen Sweep nachgemessen, nicht nur
  behauptet: **0 von 52** abgetasteten Winkeln zeigten irgendeinen
  `sin`/`cos`-Unterschied zwischen beiden Vorzeichen. Roll/Pitch sind ebenfalls
  unbetroffen (sie werden am Swing-Quaternion abgelesen, nachdem der Twist
  herausgerechnet wurde).
- **Das beobachtete Symptom passt nicht zu einem Vorzeichenwechsel.** Der
  Sprung trat reproduzierbar bei realen 180° auf — der tatsächlichen
  Singularität der alten Methode — nie zu zufälligen Zeitpunkten. Ein Tracker,
  der das Vorzeichen springen lässt, hätte zufällige Sprünge erzeugt.

Falls je ein **360°-Sprung im Stillstand** auftritt: Das ist die Diagnose, und
der Fix ist eine Zeile (`quat_rel` auf `qw >= 0` normalisieren, dokumentiert im
Docstring von `yaw_about_normal`). Das stellt die Invarianz her und kostet genau
den weiten Wertebereich — deshalb erst dann, nicht vorsorglich.

⚠️ **Zwei Bestandstests mussten inhaltlich angepasst werden**, weil sie
nachweislich nur das Verhalten der ALTEN Methode gemessen hatten:

- Der kombinierte Neigung+Gier-Test in `test_rotation.py` verglich gegen eine
  frisch nachgerechnete Rotationsvektor-Formel — die driftet aber selbst
  (34,0° statt 40°). Umgestellt auf eine unabhängige swing-twist-Nachrechnung;
  die alte Methode bleibt als Kontrastwert im selben Test erhalten
  (`old_deg` muss weiterhin deutlich abweichen).
- `test_simple_frame_identity_boresight_would_be_wrong` in
  `test_calibration.py` maß bisher die **Differenz** zweier
  Gierwinkel-Ablesungen bei einer flachen 90°-Drehung — diese Differenz ist mit
  swing-twist exakt boresight-unabhängig richtig. Der Test prüft jetzt
  stattdessen, was Identitäts-Boresight nach wie vor falsch macht: die
  **absolute** Gierwinkel-Ablesung an der Referenzpose (statt 0° kommen -91,4°
  heraus) und Roll/Pitch (statt ~0° kommen ~88,8° heraus, weil die reale
  ~120°-Montageverdrehung des Sensors ungefiltert als „Neigung" erscheint).

**Warum überhaupt eine Referenzpose nötig ist (und keine feste Tracker-Achse):**
Der erste Wurf des einfachen Modus hat den Gierwinkel direkt gegen die
Tracker-z-Achse gemessen (Identitäts-Boresight). Das ist falsch, sobald der
Sensor verdreht am Wagen sitzt — auf dieser Anlage gemessen 120,1° um
[0,553 0,589 −0,590], also Sensor-x → Welt-−z, Sensor-y → Welt-+x, Sensor-z →
Welt-−y. Diese Montagedrehung steckt dann dauerhaft im gemeldeten Winkel: Eine
**flache** Wagendrehung von 90° wurde als nur ~70° Gieränderung gemeldet, stark
nichtlinear, während Roll und Nick um Dutzende Grad mitwanderten. Mit einer
Referenzpose kommt exakt 0/15/30/45/60/90° heraus und Roll/Nick bleiben bei
0,000°. Das war kein reines Anzeigeproblem: Derselbe Gierwinkel dreht den
Sensor→Düsen-Versatz, die Identitäts-Variante hat also auch die Tinte falsch
platziert.

**Tests:**

```
tests/test_rotation.py
  test_yaw_about_normal_combined_tilt_and_yaw_recovers_exactly_where_the_old_method_drifted
  test_yaw_about_normal_pure_rotation_recovers_exactly_through_a_full_sweep
  test_yaw_about_normal_MUTATION_check_old_method_sign_flips_past_180
  test_yaw_about_normal_normalises_non_unit_quat_and_boresight
  test_yaw_about_normal_rejects_a_zero_norm_quat
  test_cart_rotation_angles_roll_pitch_stay_small_when_yaw_exceeds_180
  test_yaw_about_normal_double_cover_shifts_the_READOUT_by_exactly_360
  test_yaw_about_normal_double_cover_cannot_affect_the_PRINT_correction
  test_cart_rotation_angles_roll_pitch_are_double_cover_INVARIANT

tests/test_calibration.py
  test_simple_frame_identity_boresight_would_be_wrong
```

### 10.3 RMS-Residuum wurde vom falschen Bezugspunkt gemessen

Bei der Überprüfung der Fit-Metriken ([4.4](#44-kalibrierungsqualität)) fiel ein
echter Fehler in `fit_axis_quality()` auf: Das RMS-Residuum wurde relativ zu
`samples[0]` gemessen statt relativ zur **gefitteten Linie**. `fit_axis()` legt
seinen PCA-Fit durch den **Schwerpunkt** der Samples — `samples[0]` ist dagegen
einfach ein weiteres verrauschtes Sample und liegt gar nicht auf der Linie. (In
`trace_length_mm()` ist derselbe Bezugspunkt harmlos, weil dort `max − min`
gebildet wird und sich der Bezugspunkt herauskürzt; hier nicht.)

Gemessene Auswirkungen — alle drei machten die Kennzahl als Schwellwert
(`MAX_RMS_RESIDUAL_MM = 1 mm`) unbrauchbar:

| Effekt | gemessen |
|---|---|
| systematische Überhöhung | **1,33×** (400 Durchläufe je Rauschpegel, σ 0,1–0,8 mm) → eine reale Streuung von 0,71 mm löste bereits die 1-mm-Warnung aus |
| Abhängigkeit von der Sample-**Reihenfolge** | dieselben 60 Punkte, nur rotiert: **0,49 bis 1,04 mm** — der Wert lag mal unter, mal über seiner eigenen Schwelle |
| ein einzelner Ausreißer an **erster** Stelle | **4,97 mm** statt real 0,81 mm — ein 6-facher Fehlalarm auf einer sauberen Kante |

Behoben durch Messung vom Schwerpunkt aus. Danach exakt Faktor **1,000**
gegenüber dem echten Wert (dieselben 400 Durchläufe je Pegel), und die
Reihenfolge-Abhängigkeit ist konstruktionsbedingt weg.

Der mitgelieferte Test hatte den Fehler nicht gefangen: Sein Toleranzband
(0,7–1,6 mm um einen Erwartungswert von 1,13 mm) war weit genug, dass der um
1,33× überhöhte Wert (1,47 mm) noch hineinpasste. Das Band ist jetzt eng
(±0,1 mm), plus zwei neue Regressionstests für Reihenfolge und Ausreißer.
Mutationsprobe bestätigt: Setzt man den Bezugspunkt auf `samples[0]` zurück,
schlägt der Reihenfolge-Test fehl (0,49 vs. 1,04).

**Tests:**

```
tests/test_calibration.py
  test_fit_axis_quality_reports_length_count_and_near_zero_rms_when_clean
  test_fit_axis_quality_rms_residual_reflects_injected_noise
  test_fit_axis_quality_rms_residual_is_independent_of_sample_ORDER
  test_fit_axis_quality_rms_residual_is_not_dominated_by_one_outlier
  test_fit_axis_quality_does_not_change_with_more_noise_free_samples
  test_calibrate_page_populates_quality_metrics
  test_calibrate_page_warns_on_short_trace
  test_calibrate_page_warns_on_noisy_trace
  test_calibrate_page_warns_on_few_samples
  test_calibrate_page_normal_tilt_reflects_a_tilted_page
  test_save_and_load_roundtrip_includes_quality_metrics
  test_quality_metrics_default_to_none_when_built_directly
  test_to_dict_omits_absent_quality_fields
  test_from_dict_loads_a_pre_feature_json_with_no_quality_fields

tests/test_ui_calibration.py
  test_compute_calibration_returns_quality_metrics
  test_compute_calibration_flags_low_quality_separately_from_angle
  test_load_calibration_reports_none_quality_for_a_pre_feature_file
```

### 10.4 Umstellung vom Verweildauer-Modell auf Tropfen

Die alte Firmware **hielt** das zuletzt geschriebene Muster und feuerte es alle
`PATTERN_STRIDE` Ticks erneut; die Tinte hing also daran, wie *lange* der Client
ein Düsenbit auf 1 hielt, und beide Seiten waren über
`DOSE_HOLD_S ≈ 3 × PATTERN_STRIDE × Tick` gekoppelt. Die aktuelle Firmware
feuert jede empfangene Spalte **genau einmal** und wiederholt sie nie
(`PATTERN_STRIDE` und `pattern_dose_should_fire()` sind aus `ble_dose.h`
verschwunden). Damit gibt es keine Wiederholrate mehr, gegen die man halten
könnte — die Tinte wird vollständig auf der Client-Seite entschieden.
`--dose-hold-s` gibt es nicht mehr.

Zwei Konsequenzen, die beide Verhalten ändern:

1. **Der Client sendet jetzt bei jedem Sample mit offener Tintenschuld, nicht
   nur bei Musterwechsel.** Unter einer Feuer-einmal-Firmware bedeutet „nur bei
   Änderung senden", dass eine gleichmäßige Fläche (ein gefüllter Block, eine
   breite Linie) *gar nichts* sendet: die Menge der gewollten Düsen ändert sich
   dort von Sample zu Sample nicht. Gemessen an einem 120 Spalten breiten
   Vollblock bei 30 mm/s: **2 Schreibvorgänge für den ganzen Durchlauf** statt
   der ~360, die das Tintenbudget verlangt — während die Deckung 100 % meldete.
2. **`PatternSender` ist keine „latest wins"-Mailbox mehr, sondern eine
   begrenzte Warteschlange.** Eine überholte Spalte war früher wertlos (die
   gehaltene färbte ohnehin weiter); heute ist jede verworfene Spalte verlorene
   *Tinte*. Läuft die Warteschlange über, fliegt die **älteste** Spalte (die,
   deren Position der Wagen am sichersten schon verlassen hat) und wird in
   `PatternSender.dropped` gezählt statt still geschluckt.

Zum Vergleich das alte Verweildauer-Modell an derselben Strecke wie in
[5.2](#52-was-die-geschwindigkeit-begrenzt): 73,3 % `printed` gegen 99,3 %
`fired` bei 25 mm/s, 44,2 % gegen 99,3 % bei 30 — eine vollständig eingefärbte
Seite, die als stark gestreift gemeldet wurde. Genau das war die Rückmeldung von
der Hardware („die Füllung des echten Drucks ist perfekt, das Coverage-Bild
sieht völlig anders aus").

Die alte Startwarnung („`dose_hold_s` muss unter dem Poll-Intervall bleiben")
ist ersatzlos weg — eine Tropfenzahl hat mit der Poll-Rate nichts zu tun. An
ihre Stelle tritt die Warnung aus [5.2](#52-was-die-geschwindigkeit-begrenzt).

**Tests** (`tests/test_coverage.py`, `tests/test_freehand_pass.py`,
`tests/test_pattern_sender.py`):

```
test_coverage_is_speed_independent_up_to_the_poll_rate_limit
test_past_the_poll_rate_limit_whole_columns_are_skipped_not_thinned
test_the_delivered_column_count_matches_the_drops_per_pixel_budget
test_nozzle_keeps_firing_through_the_last_drop_after_being_reported
test_a_stationary_cart_owes_no_drops_and_never_completes
test_completion_depends_on_drops_delivered_not_on_elapsed_time
test_a_dose_summed_from_fractions_completes_without_an_extra_sample
test_MUTATION_check_crediting_integer_columns_reintroduces_speed_stripes
test_a_uniform_region_keeps_sending_instead_of_going_quiet
test_columns_sent_track_the_drops_per_pixel_budget
test_fractional_drops_are_accumulated_rather_than_truncated
test_copies_are_queued_once_each_and_never_merged
test_sends_made_while_a_write_is_in_flight_are_NOT_coalesced
test_overflow_drops_the_OLDEST_columns_and_counts_them
```

### 10.5 Kein Freispruch ohne Messung

Das Verdikt von `--calibration-check` ([7.3](#73---calibration-check-gierwinkel-drift))
hing ausschließlich am Gierwinkel-Span. Ein Lauf, der **gar nichts** gesammelt
hat — Tracker liefert keine Pose, oder Strg+C kommt sofort — meldete damit:

```
  samples: 0
  verdict: OK: yaw span 0.00 deg ... consistent with a good calibration.
```

Also ein Freispruch für genau die Frage, wegen der man das Werkzeug startet.
Dasselbe galt für ein 2-cm-Wackeln: Der Gierwinkel bleibt dabei nahe null, weil
sich der Wagen kaum bewegt hat, nicht weil die Kalibrierung gut ist.
`_calibration_check_summary` berechnete `u_travel_mm`/`v_travel_mm` bereits und
nannte sie im eigenen Docstring den „headline sanity check" — das Verdikt hat
sie nur nie ausgewertet.

Jetzt wird vor den Gierwinkel-Schwellen geprüft, ob überhaupt genug gemessen
wurde: mindestens **20 Samples** und **50 mm** Weg (Diagonale der
u/v-Bounding-Box). Darunter lautet das Verdikt `INCONCLUSIVE` mit dem
ausdrücklichen Zusatz, dass das **kein Bestehen** ist. Die Schwellen sind
bewusst dieselben Konstanten wie `MIN_TRACE_LENGTH_MM`/`MIN_SAMPLE_COUNT` aus
`calibration.py` (importiert, nicht kopiert) — es ist dieselbe Frage, mit
derselben Messreihe belegt.

Mutationsproben bestätigt: Entfernt man die Sample-Bedingung, die
Weg-Bedingung, oder lässt man den Guard alles verschlucken, schlägt jeweils ein
Test fehl. Die beiden in [7.3](#73---calibration-check-gierwinkel-drift)
abgedruckten Beispielläufe (579 bzw. 553 Samples über ~200/280 mm) liegen weit
über beiden Schwellen — ihre Verdikte sind unverändert.

**Tests:**

```
tests/test_page_mapper.py
  test_calibration_check_ndjson_event_shape
  test_calibration_check_reports_an_error_without_a_page_frame
  test_calibration_check_pure_translation_gives_near_zero_yaw_span
  test_calibration_check_injected_yaw_ramp_gives_large_span_and_high_correlation
  test_calibration_check_summary_pure_function_matches_the_live_run
  test_calibration_check_zero_samples_is_INCONCLUSIVE_not_a_pass
  test_calibration_check_short_wiggle_is_INCONCLUSIVE_not_a_pass
  test_calibration_check_too_few_samples_is_INCONCLUSIVE_even_over_a_long_sweep
  test_calibration_check_a_real_sweep_still_reaches_a_real_verdict

tests/test_patterns_and_mapping.py
  test_cli_calibration_check_is_a_debug_mode
  test_cli_calibration_check_requires_a_page_frame
  test_cli_calibration_check_accepts_page_calibration
  test_cli_calibration_check_accepts_simple_frame
  test_cli_calibration_check_conflicts_with_pos
  test_cli_calibration_check_pos_json_flag_reused
```

### 10.6 Die Buchführung pro Sample war das eigentliche Tempolimit

Die UI fährt jeden Druck mit `--progress-json`. Dieser Stream schickte ein
Ereignis **pro Poll-Sample** (bis zu 500/s), und dahinter liefen mehrere
Vollbild-numpy-Durchläufe. Gemessen an einem großen Muster
(`--pattern-length-mm 200 --pattern-height-mm 100` = 2299×1152 = 2,65 M Pixel),
gegen ein Budget von 2000 µs je Sample:

| Operation | Kosten/Sample | lief |
|---|---|---|
| `coverage.done` → `np.all(printed[ink])` | 1279 µs | **immer**, auch ohne `--progress-json` |
| `(ink & fired).sum()` | 1536 µs | nur `--progress-json` |
| `ink.sum()` | 977 µs | nur `--progress-json` — eine **Konstante** |
| `fired & ~prev` + `fired.copy()` | 723 µs | nur `--progress-json` |

Weil die Spalten-Kante des Tintenmodells `--mm-per-column × --poll-hz` ist, war
das kein reines CPU-Thema, sondern ein **Tempolimit auf dem Druck**:

| | erreicht | Spalten-Kante |
|---|---|---|
| Soll | 500 Hz | 43,5 mm/s |
| vorher, ohne `--progress-json` | ~208 Hz | 18,1 mm/s |
| **vorher, mit `--progress-json` (= die UI)** | **~71 Hz** | **6,2 mm/s** |
| **jetzt** | **~330 Hz** | **28,7 mm/s** |

Behoben, indem jede dieser Größen inkrementell mitgeführt wird statt pro Sample
neu aus dem Vollbild berechnet: `printed` wird ausschließlich in `_deposit`
gesetzt, `fired` ausschließlich in `step()`, also sind exakte Zähler an genau
diesen Stellen möglich (`CoverageEngine.ink_total` / `ink_fired` / `ink_printed`,
und `done` als Zählervergleich). Neu gefeuerte Zellen schreibt die Engine gleich
mit (`drain_new_cells()`), womit der Masken-Diff ganz entfällt.

Zusätzlich geht das `coverage`-Ereignis nur noch mit `--progress-hz` (Default
25) raus statt pro Sample. **Dabei geht kein Tropfen verloren:** zwischen zwei
Ereignissen gesammelte Zellen kommen im nächsten mit, und ein zwingender Flush
am Pass-Ende trägt den Rest — auch beim Abbruch per STARTPOINT-Taster oder
SIGINT. Gegengeprüft an einem echten Durchlauf: 130.720 gemeldete Zellen gegen
130.720 gezählte bedeckte Pixel.

Die verbleibende Lücke zu 500 Hz ist der nicht deadline-korrigierte `sleep` am
Schleifenende (die Periode ist immer `Arbeit + 2 ms`) plus `step()` selbst.
Beides ist eine eigene Baustelle.

### 10.7 --mm-per-column wurde komplett ignoriert

`build_tracking()` in `cli.py` hat `TrackingSettings` gebaut, ohne
`mm_per_column` überhaupt zu übergeben — dadurch griff immer der eigene
Dataclass-Default, egal was auf der Kommandozeile stand. Nur `--dpi` hatte je
einen echten Effekt (über `resolve_mm_per_column`). Direkt bestätigt:

```bash
python main.py --pattern checkerboard --pattern-length-mm 200 --mm-per-column 0.1 ...
# vorher: "-> 1000 columns x 2000 rows"   (der ignorierte Default)
# jetzt:  "-> 2000 columns x 2000 rows"   (0.1mm/Spalte, wie angefordert)
```

Genau das war die Ursache, wenn ein eigentlich quadratisch gedachtes
Schachbrett (`--pattern-square-mm` == `--pattern-square-height-mm`,
`--mm-per-column` == `NOZZLE_PITCH_MM`) in `coverage.png` trotzdem doppelt so
hoch wie breit aussah: jede Spalte war heimlich doppelt so breit wie
angefordert. Betraf **jeden** Aufruf mit einem von der Voreinstellung
abweichenden `--mm-per-column` — Musterbreite, Coverage-Engine-Spaltenadressierung,
alles, was `tracking.mm_per_column` liest.

**Tests** (`tests/test_patterns_and_mapping.py`):

```
test_cli_mm_per_column_reaches_build_tracking
test_cli_mm_per_column_default_still_matches_the_dataclass_default
test_cli_dpi_still_overrides_mm_per_column
test_cli_mm_per_column_MUTATION_check_omitting_it_reintroduces_the_bug
```

Mutationsgeprüft gegen die reale, alte Konstruktion (nicht nur eine Nachbildung
im Test).

### 10.8 coverage.png zeigte senkrechte Streifen

COVERED/MISSED wurden aus `printed` gezeichnet — das ist aber die
**Dosis-Abschluss**-Buchhaltung, nicht die tatsächlich gelandete Tinte. Eine
Düse feuert, sobald ihr Pixel gewollt ist; `printed` wird dagegen erst gesetzt,
wenn die Dosis voll ist. Unter dem damaligen Verweildauer-Modell brauchte eine
Spalte dafür **mindestens zwei Samples**, gemessen an den echten Einstellungen
der Anlage (`--mm-per-column 0.087`, `--poll-hz 500`):

| Wagen-Geschwindigkeit | Samples/Spalte | `printed` | tatsächlich gefeuert |
|---|---|---|---|
| 17,3 mm/s | 2,51 | 99,3 % | 99,3 % |
| 25 mm/s | 1,74 | **73,3 %** | 99,3 % |
| 30 mm/s | 1,45 | **44,2 %** | 99,3 % |

Unterhalb von zwei Samples pro Spalte schloss keine Dosis mehr ab, die Düse
hatte aber auf dem einen Sample gefeuert — das Papier war voll, das Bild zeigte
Lücken. `CoverageEngine.fired` hält jetzt zusätzlich fest, wo Tinte
**physisch** gelandet ist (gegen eine unabhängige Rekonstruktion der gesendeten
BLE-Patterns bit-genau geprüft), und COVERED/MISSED sowie die
`Covered N/M`-Zeile stammen daraus. `printed` behält seine Dosis-Rolle (steuert
Nachfeuern und `coverage.done`).

Mit dem Tropfenmodell ist diese Schere weitgehend zu: eine überfahrene Spalte
bekommt ihre volle Dosis bei jedem Tempo, das der Tracker noch abtasten kann,
und jenseits davon fallen `fired` und `printed` gemeinsam.

### 10.9 Dosis ging bei Zeilen-Flapping verloren

`NOZZLE_PITCH_MM` (0,087 mm) ist feiner als reales Tracker-Rauschen. Steht eine
Düse nahe an einer Zeilengrenze, kippt die gerundete Zeile von Sample zu Sample
zwischen zwei Nachbarn — die Engine hat den Dosis-Zähler bisher bei **jedem**
Wechsel auf 0 zurückgesetzt. Die Düse feuert dabei trotzdem (`active[p]` wird
gesetzt, sobald ein Pixel gewollt ist), aber die volle Dosis wurde nie erreicht,
weil der Zähler nie über einen Sample-Wechsel hinweg überlebt hat.

Reproduziert: Düse (fast) still auf einer Zeilengrenze, nur ±0,001 mm Rauschen
(zwei Größenordnungen unter realem Sensorrauschen) — **200 von 200 Samples
feuern real, aber 0 Pixel werden als gedruckt verbucht.** Eine Rausch-Messreihe
zeigt zusätzlich: Mit mehr (realistischerem) Rauschen feuert die Düse öfter,
aber die verbuchte Fläche geht **runter statt rauf** — das genaue Gegenteil
dessen, was man erwarten würde:

```
Rauschen (mm)   Proben mit Feuern   verbuchte Pixel
         0.00           164/1000                192
         0.05           491/1000                191
         0.20           963/1000                173
```

Behoben, indem die Dosis **pro Pixel** akkumuliert wird
(`CoverageEngine._pixel_drops`, ein Dict, keyed auf `(row, col)`) statt pro
Düsen-/Gruppen-Slot mit Reset bei jedem Zeilenwechsel. Ein Wechsel weg von einem
Pixel — sei es Flapping zur Nachbarzeile oder ein längerer Ausflug, weil der
Wagen woanders hin fährt — lässt die bereits angesammelte Dosis unangetastet;
sie läuft beim nächsten Besuch einfach weiter. Der Eintrag wird erst beim
Fertigstellen des Pixels aus dem Dict entfernt, der Speicherbedarf bleibt also
auf „gerade angefangene, noch nicht fertige Pixel" begrenzt.

**Zwei Schwellen auf einem Konto.** `printed` (der Report, `coverage.done` und
die Zahl am Durchlaufende) wird **eine Probe früher** gesetzt als die Düse
freigegeben wird. Grund: die Gutschrift kommt in ganzen Poll-Samples an. Ist
eine Spaltenüberquerung `m` Samples wert, landen `floor(m)` oder `ceil(m)` davon
*innerhalb* der Spalte — eine vollständig überfahrene Spalte kann also allein
dadurch eine Probe zu kurz kommen, wie das Sample-Raster gerade zum
Spalten-Raster steht. Ohne diese eine Probe Spiel meldet der Report regelmäßige
Streifen, deren Dichte sinnlos mit dem Tempo schwankt (gemessen: 70,0 % der
Spalten bei 5 mm/s, 35,0 % bei 10, 51,7 % bei 17,3, 9,2 % bei 40 — gegen
durchgehend 100 % `fired`).

Die Düse **freizugeben** darf dagegen erst die strenge Schwelle: wer beides auf
der lockeren Schwelle macht, kürzt jede Überquerung um bis zu eine Probe echte
Tinte — gemessen beim Default-Dose 142 statt 199 Spalten/s bei 17,3 mm/s, 74
statt 285 bei 25 und 189 statt 343 bei 30, also 30–75 % zu wenig, während die
Deckung weiter 100 % meldet.
