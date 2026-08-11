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

**Wichtig:** Seit diesem Release ist `--mode page` (freihändiges 2D-Drucken)
der Standard-Modus, nicht mehr `--mode line` — siehe Abschnitt
`--mode page` unten. Page-Mode kennt **zwei Seiten-Rahmen** (`--page-frame`):
den kalibrierten (Standard, braucht `--page-calibration PATH`) und den
**einfachen** (`--page-frame simple`, braucht gar keine Kalibrierung, siehe
Abschnitt „Einfacher Modus" unten). Ohne `--mode`, ohne `--page-calibration`
*und* ohne `--page-frame simple` bricht das Programm mit einer klaren
Fehlermeldung ab, statt stillschweigend das falsche Verhalten zu zeigen.

```bash
# Vorschau erzeugen, nichts senden (kein Hardware-/Kalibrierungs-Zugriff nötig):
python main.py "Hallo" --dry-run --preview vorschau.png --mode line

# Freihändig drucken OHNE Kalibrierung (einfachster Einstieg): Seitenachsen
# = Tracker-x/y, Gierwinkel relativ zur Pose beim START-Druck:
python main.py "Hallo" --page-frame simple

# Freihändig drucken MIT kalibrierter Seite (genauer), auf START-Taster
# warten -- braucht eine vorher erstellte PageCalibration:
python main.py "Hallo" --page-calibration page_calibration.json

# Positions-Loop (1D, eine Richtung) ohne Hardware testen:
python main.py "Hallo" --simulate --mode line --dry-run

# Klassisch zeitbasiert (wie das Ursprungsskript):
python main.py "Hallo" --mode time --period 0.03
```

**`--mode page`** (seit diesem Release der Standard) — freihändiges 2D-Drucken
(Wagen frei über die Seite bewegen, nicht nur eine Richtung). Der Seiten-Rahmen
kommt entweder aus einer kalibrierten Seite (Standard) oder aus dem einfachen
Modus (`--page-frame simple`, siehe direkt unten). Für den kalibrierten Rahmen
im **Calibration**-Tab der Web-UI zwei angrenzende Seitenkanten mit dem Sensor
abfahren, "Compute calibration" berechnen lassen und speichern; die
gespeicherte Datei dann per `--page-calibration PATH` laden. Details:
`README_BLE_INTERFACE.md` im Firmware-Repo (Abschnitt "Page Mode"). Für den
alten 1D-Closed-Loop (Wagen bewegt sich nur in eine Richtung, keine
Kalibrierung nötig) weiterhin `--mode line` verwenden.

### Einfacher Modus: `--page-frame simple` (ohne Kalibrierung)

`--page-frame simple` überspringt die Seiten-Kalibrierung komplett und nimmt
direkt das Amfitrack-Koordinatensystem als Seiten-Rahmen:

| | einfach (`--page-frame simple`) | kalibriert (Standard) |
|---|---|---|
| Spaltenachse `u` | Tracker-**x** | abgefahrene Spaltenkante |
| Zeilenachse `v` | Tracker-**y** | abgefahrene Zeilenkante |
| Gierwinkel | um die Seitennormale, relativ zur Pose beim **START** | um die Seitennormale, relativ zur abgefahrenen Boresight-Pose |
| Nullpunkt | wo der Wagen beim **START**-Druck steht | abgefahrene Seitenecke |
| Skalenkorrektur | keine (Tracker-mm = echte mm) | optional aus bekannter Blattgröße |
| Vorbereitung | keine | zwei Kanten abfahren + Boresight aufnehmen |

Der Preis dafür ist explizit: Das Blatt muss **achsparallel zum Tracker**
liegen (der einfache Modus kennt die Seitenlage nicht und kann sie nicht
korrigieren), und eine systematische Skalenabweichung des Trackers wird nicht
ausgeglichen. Dafür kann er auch keine *schlechte* Kalibrierung erben — was
der Grund ist, warum es ihn gibt: Eine Winkelmessreihe (0…90° in 15°-Schritten
gegen physisch angezeichnete Winkellinien) hat gezeigt, dass der Gierwinkel
selbst sauber misst (Steigung 1,012, konstanter Versatz +0,89°, Reststreuung
RMS 0,82° / max. 1,36°) und die Boresight-Aufnahme über fünf Versuche bis auf
0,001 in einer Quaternion-Komponente reproduzierbar ist. Schlechte
Druckergebnisse kamen also nicht aus der Winkelrechnung, sondern aus dem
kalibrierten Rahmen selbst — genau den umgeht dieser Modus.

**Bedienung:** Wagen dorthin stellen, wo die **linke obere Ecke des Drucks**
liegen soll, dann START drücken. Der Nullpunkt wird auf die **Düsenleiste**
gelegt, nicht auf den Sensor — die beiden liegen
`SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM` ≈ 62 mm auseinander, und ohne diese
Korrektur läge die Leiste rund 70 mm neben der Seite, sodass gar nichts
gedruckt würde (siehe `PageMapper.zero_at_nozzle`).

In der Web-UI steht dafür im **Tracking & scale**-Tab das Feld **Page frame**
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
Die Gier-Referenz wird dabei aus der **ersten** Orientierungsmessung
übernommen: `yaw_deg` startet also bei 0° für die Pose, in der der Wagen beim
Start von `--pos` steht, und zeigt danach die Drehung relativ dazu. Hinweis:
Der *Nullpunkt* wird im Diagnosemodus bewusst **nicht** neu gesetzt,
`page_u`/`page_v` sind dort also absolute Tracker-Koordinaten (plus
Sensor→Düsen-Versatz).

**Warum eine Referenzpose nötig ist (und keine feste Tracker-Achse):** Der
erste Wurf dieses Modus hat den Gierwinkel direkt gegen die Tracker-z-Achse
gemessen (Identitäts-Boresight). Das ist falsch, sobald der Sensor verdreht am
Wagen sitzt — auf dieser Anlage gemessen 120,1° um [0,553 0,589 −0,590], also
Sensor-x → Welt-−z, Sensor-y → Welt-+x, Sensor-z → Welt-−y. Diese
Montagedrehung steckt dann dauerhaft im gemeldeten Winkel: Eine **flache**
Wagendrehung von 90° wurde als nur ~70° Gieränderung gemeldet, stark
nichtlinear, während Roll und Nick um Dutzende Grad mitwanderten. Mit der beim
START aufgenommenen Referenzpose kommt exakt 0/15/30/45/60/90° heraus und
Roll/Nick bleiben bei 0,000°. Das war kein reines Anzeigeproblem: Derselbe
Gierwinkel dreht den Sensor→Düsen-Versatz, die Identitäts-Variante hat also
auch die Tinte falsch platziert. Festgehalten in
`tests/test_calibration.py` / `tests/test_page_mapper.py`.

**`--simple-boresight QX QY QZ QW`: Referenzpose anpinnen statt blind
automatisch erfassen.** Die automatische Erfassung beim START (siehe oben)
hat einen echten Schwachpunkt: Sie nimmt einfach die Pose, in der der Wagen
in genau diesem Moment zufällig steht — BLE noch nicht eingeschwungen, Hand
noch nicht ganz ruhig — und es gibt **keine Möglichkeit, das zu
kontrollieren**, außer den ganzen Durchlauf neu zu starten und zu hoffen.
Genau das ist auf der echten Anlage passiert: Eine tatsächlich flach und mit
0° Gier gehaltene Pose (`quat=[-0.50 -0.50 -0.51 +0.49]`) wurde als
`yaw=-71.23° roll=-70.29° pitch=-69.34°` gemeldet, weil die automatische
Erfassung an einer anderen, ungeprüften Pose hängengeblieben war.

Der robuste Weg: Referenzpose **erst separat erfassen und verifizieren**,
dann **anpinnen** — genau der Capture-Workflow, den es für den kalibrierten
Modus mit seinem Boresight-Button schon gibt.

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
das stimmt, denselben `--simple-boresight`-Wert in den echten Druck
übernehmen; die automatische Erfassung beim START greift dann **nicht** mehr
ein (ein gepinnter Wert wird nie überschrieben, siehe
`tests/test_freehand_pass.py`s Regressionstest dazu).

In der Web-UI übernimmt der Button **Capture yaw reference** (im Tracking-Tab,
nur bei `page_frame = simple` sichtbar) genau diesen Schritt: er liest den
zuletzt empfangenen Quaternion-Wert aus dem Live-Sensor-Panel oben, zeigt ihn
an, und hängt ihn automatisch als `--simple-boresight` an sowohl den
Live-`--pos`-Verifikationsbefehl als auch den eigentlichen Druckbefehl an.

⚠️ **Wichtig:** Beim ersten echten Hardware-Bring-up ist der Modus ohne
Fehlermeldung leer durchgelaufen (`active=0` die ganze Zeit, Exit-Code 0,
nichts auf dem Papier) — der Grund war genau der folgende Punkt: Das
gerenderte Zielbild ist mit 152 Düsenreihen nur ca. 15,1 mm
hoch (`NOZZLE_PITCH_MM * 151`, = `NOZZLE_BAR_SPAN_MM`). Damit überhaupt eine Düse zündet, muss der
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
| `--sensor-offset-row-mm MM` | Abstand Sensor → **Mitte** der Düsenleiste entlang der Zeilenachse (entlang der Düsenreihe, senkrecht zur Fahrtrichtung) | `-62.36` mm (Betrag gemessen: "Die Mitte der Nozzle-Reihe ist 62,36 mm verschoben von der Y-Koordinate des Amfitrack"; Vorzeichen an echtem Testdruck auf dieser Anlage bestätigt — siehe Warnhinweis direkt darunter) |
| `--sensor-offset-col-mm MM` | Dasselbe entlang der Spaltenachse (Fahrtrichtung) | `0.0` mm (bisher keine gegenteilige Messung; explizit als eigener, überschreibbarer Wert geführt, falls sich das noch ändert) |

Beide Defaults stecken als `SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM` /
`SENSOR_TO_NOZZLE_COL_MM` in `printhead/geometry.py` — als feste mechanische
Eigenschaft des Wagens, unabhängig von jeder einzelnen `PageCalibration`
(eine neue Seite kalibrieren erfordert diesen Wert also nie erneut).

⚠️ **Falsche Richtung nach dem Testdruck?** Einfach den aktuellen Default-Wert
**negieren** (z. B. `--sensor-offset-row-mm 62.36`, um testweise zum
ursprünglich gemessenen Vorzeichen zurückzukehren), sonst muss nichts
geändert werden — siehe der Kommentar direkt über der Konstante in
`geometry.py`.

**Verifikation:** Nach dieser Änderung `--pos --page-calibration PATH`
starten und den Wagen so halten, dass **die Düsenleiste** (nicht der Sensor!)
exakt auf der zuvor abgefahrenen Seitenecke steht. Das live angezeigte `v`
sollte jetzt nahe **0** liegen — bei falschem Vorzeichen liegt es (je nach
Zentrum-vs.-Düse-0-Bezug) stattdessen um **±62.36 mm** oder einen ähnlich
verschobenen Wert daneben.

**Wagen-Rotation / Boresight (`--boresight-deg`):** Der Wagen wird beim
freihändigen Drucken nicht nur verschoben, sondern auch gedreht. An einem
echten Druck gemessen (`pass5.csv`, Achsen-Winkel-Zerlegung der relativen
Rotation gegen die Seitennormale) spannt die Gier-Rotation (Yaw) um die
Seitennormale **75,6°** über einen normalen Durchlauf, während Neigung
(Pitch/Roll) klein bleibt (Median 2,7°, Maximum 7,8°) — deshalb wird bewusst
**nur Yaw** korrigiert, Pitch/Roll nicht. Unkorrigiert bedeutet das zwei
konkrete Fehler:

- Der 62,36 mm Hebelarm Sensor→Düsenleiste (`--sensor-offset-row-mm`, s. o.)
  ist ein Vektor im Wagen-Koordinatensystem, dreht sich also mit dem Wagen
  mit. Als fester Seiten-Versatz behandelt (wie bisher), erzeugt das bei
  75,6° Yaw bis zu **76 mm** Positionsfehler.
- Die 15,1 mm lange Spanne der Düsenleiste selbst (Düse 0 → Düse 151,
  `NOZZLE_BAR_SPAN_MM`) fächert bei Drehung über mehrere
  Spalten auf: bei 75° sind das ca. **14,6 mm** (≈73 Spalten bei
  0,2 mm/Spalte) statt einer einzigen Spalte für alle 152 Düsen.

Voraussetzung für die Korrektur ist eine **Boresight-Aufnahme** — die
Referenz-Orientierung, bei der die Düsenleiste exakt entlang der
abgefahrenen Zeilenkante (Kante 2) liegt, gegen die der aktuelle Yaw während
des Drucks gemessen wird. Aufnahme im **Calibration**-Tab der Web-UI: Sensor
verbinden, beide Kanten wie gewohnt abfahren, den Wagen dann **flach auf das
Papier legen**, Düsenleiste entlang Kante 2 ausrichten, still halten und
"Capture boresight" drücken — übernimmt automatisch das zuletzt vom
laufenden `--pos-json`-Stream gelieferte Quaternion. Die Statuszeile zeigt
vorher deutlich "not captured" an; erst danach "Compute calibration"
drücken, damit das Quaternion mit gespeichert wird.

⚠️ **Bestehende Kalibrierungen ohne Boresight** (alles, was vor dieser
Funktion gespeichert wurde) funktionieren unverändert weiter — **ohne**
Rotationskorrektur, exakt wie bisher. Das wird beim Druckstart laut gemeldet
(`[warn] page calibration has no boresight ...`), nie still übernommen: ein
Druck darf nicht davon abhängen, wie der Wagen zufällig zu Beginn gehalten
wurde (genau die Art unsichtbarer Fehler, die dieses Projekt schon einmal
gebissen hat). Neu kalibrieren mit Boresight-Aufnahme schaltet die Korrektur
ein.

`--boresight-deg GRAD` feintunt die aufgenommene Boresight-Rotation additiv,
ohne neu zu kalibrieren — kommt ein Druck verdreht heraus, lässt sich das
direkt per Flag nachjustieren, dasselbe "anpassen statt neu bauen"-Prinzip
wie `--sensor-offset-row-mm` oben. Hat keine Wirkung, wenn keine Kalibrierung
einen Boresight trägt.

**Verifikation:** `--pos --page-calibration PATH` zeigt jetzt zusätzlich das
live `yaw` in Grad an. Wird der Wagen exakt in der Referenzpose gehalten
(Düsenleiste entlang der abgefahrenen Zeilenkante), sollte `yaw` nahe **0°**
liegen — weicht es deutlich ab, per `--boresight-deg` nachjustieren oder neu
kalibrieren (Boresight neu aufnehmen).

**Größeres, richtig proportioniertes Testmuster in `--mode page`:** Genau weil
`--mode page` nicht auf die 15,2 mm der 152 Düsen begrenzt ist, lohnt sich für
den Bring-up ein deutlich größeres `--calibrate`/`--pattern`-Bild als die
sonst übliche `IMAGE_HEIGHT`-Zeilenzahl:

| Option | Bedeutung |
|---|---|
| `--pattern-height-mm MM` | Physische Gesamthöhe von `--calibrate`/`--pattern` in mm (`rows = height_mm / NOZZLE_PITCH_MM`). Nur mit `--mode page` gültig — im Zeilen-/Zeit-Modus packt `frames_from_ink()` feste Frames mit genau `IMAGE_HEIGHT` Zeilen, eine andere Höhe wird dort mit einem klaren Fehler abgelehnt. Ohne diese Option bleibt das Muster bei `IMAGE_HEIGHT` Zeilen (15,2 mm, = `NOZZLE_BAR_WIDTH_MM`) gedeckelt. |
| `--pattern-square-height-mm MM` | Zeilenperiode in mm für checkerboard/h-stripes, überschreibt `--pattern-square-rows` (`square_rows = v / NOZZLE_PITCH_MM`). |

⚠️ **Seitenverhältnis-Falle:** Eine Bildzeile ist nur **0,1 mm** hoch
(`NOZZLE_PITCH_MM`, exakt seit der Neuvermessung). `--pattern-square-rows 20` (der Default) ist damit
genau **2 mm** hoch, während `--pattern-square-mm 10` (der Default) **10 mm**
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
Fertigstellung reichen. Der Default 4.05 ms liegt 19 % unter dem
5.00 ms Poll-Intervall von `--poll-hz 200` (statt knapp darüber).
`PrintController` warnt zur Laufzeit, falls `dose_hold_s >= 1 / poll_hz`
doch wieder zutrifft.

⚠️ **`--poll-hz` ist jetzt standardmäßig 500, nicht mehr 200:** Das
Poll-Intervall schrumpft dadurch von 5.00 ms auf **2.00 ms** — kürzer als
das unveränderte `dose_hold_s`-Default (4.05 ms). Die obige
Laufzeit-Warnung feuert also seit diesem Release **out of the box** bei
jedem Seiten-Druck mit Default-Einstellungen (ein Pixel braucht jetzt 3
Samples × 2 ms = 6 ms reale Zeit für eine Dosis, minimal schlechter als die
alten 2 Samples × 5 ms = 10 ms bei 200 Hz — die höhere Poll-Rate verbessert
nur die Spalten-Platzierungsgenauigkeit, nicht von sich aus die
Dosis-Fertigstellungszeit). Bis eine an Hardware neu vermessene Paarung
vorliegt, entweder explizit ein kürzeres `--dose-hold-s` setzen (und die
Firmware entsprechend auf ein neues `PATTERN_STRIDE` umstellen und neu
flashen, siehe "Firmware-Kopplung" unten) oder für die alte, zueinander
passende Kombination explizit `--poll-hz 200` übergeben.
`coverage.DEFAULT_DOSE_HOLD_S` selbst wurde in diesem Release **nicht**
geändert — das bleibt für eine spätere, hardwareverifizierte Runde offen.

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

### Tintenausbreitung: `--spray-radius-mm` / `--spray-strength`

Ein echter Tropfen landet nicht exakt in *einer* Rasterzelle, er benetzt eine
kleine Fläche drumherum. Ohne dieses Modell passiert Folgendes: Bei einer
Rückfahrt sitzt der Wagen ein paar Zehntel-mm versetzt, die Düsen adressieren
dadurch **andere Zeilen-Indizes**, diese gelten als „noch nicht gedruckt" — und
es wird erneut über Papier gedruckt, auf dem längst Tinte ist.

| Option | Bedeutung |
|---|---|
| `--spray-radius-mm MM` | Physischer Radius um ein fertiges Pixel, der eine Teildosis abbekommt. **In Millimetern, nicht in Pixeln** — eine Zelle ist 0,1 mm hoch, aber `--mm-per-column` (Default 0.2 mm) breit, ein runder Tropfen ist im Raster also ~2:1 elliptisch. Default `0` = aus. |
| `--spray-strength F` | Dosis, die ein **direkt angrenzendes** Pixel abbekommt (0.0–1.0), linear abfallend bis 0 am Radius. Ein Pixel gilt ab Gesamtdosis 1.0 als gedruckt: bei `1.0` markiert ein einzelner Tropfen die Nachbarzelle sofort mit, bei `0.5` sind zwei Tropfen nötig. Default `0` = aus. |

Beide müssen `> 0` sein, damit das Modell greift; sonst verhält sich die Engine
exakt wie zuvor (Default-Verhalten unverändert).

Gemessen an simulierten Mehrfach-Überfahrten mit 0,05 mm Versatz pro Durchgang
(40 × 30 mm Vollfläche, `--dose-hold-s 0.0018`, 500 Hz):

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

### Düsengruppierung: `--nozzle-group`

Standardmäßig wird jede der 152 Düsen einzeln angesteuert (`--nozzle-group 1`,
heutiges Verhalten, Default). Mit `--nozzle-group 2` werden je zwei
benachbarte Düsen zu einer gemeinsam adressierbaren Einheit zusammengefasst,
die immer nur gemeinsam feuert oder gar nicht. **Gilt nur in `--mode page`**
(`CoverageEngine`) — Line-/Time-Modus packt feste Frames über einen anderen
Pfad (`rendering.frames_from_ink`), den diese Option nicht berührt;
`--nozzle-group 2` außerhalb von `--mode page` wird deshalb beim Parsen
abgelehnt.

Der physische Düsenabstand (`NOZZLE_PITCH_MM`, 0,1 mm) ändert sich dadurch
**nicht** — nur die kleinste noch einzeln ansprechbare vertikale Einheit wird
doppelt so groß: aus 0,1 mm pro Düse werden bei `--nozzle-group 2`
0,2 mm pro adressierbarer Einheit.

**Feuerregel (OR):** Eine Gruppe feuert, sobald **mindestens eine** ihrer
beiden Düsen ihr Pixel noch braucht (angefordert und noch nicht gedruckt) —
so geht nie ein gewolltes Pixel verloren, weil die Gruppe es nicht anfeuert.
Der Preis: Liegt eine Gruppe genau auf der Grenze zwischen einer Tinte- und
einer Nicht-Tinte-Zeile, wird beim Fertigwerden auch die Nicht-Tinte-Zeile
mitgedruckt (Kantenverbreiterung um bis zu eine Zeile) — die Gruppe kann
nicht nur zur Hälfte feuern.

⚠️ Diese Option ist **kein Fix** für wiederholtes Überdrucken (siehe
Tintenausbreitung oben, `--spray-radius-mm`/`--spray-strength`) und senkt
auch nicht spürbar die CPU-Last — gemessen kostet `CoverageEngine.step()`
~46,9 µs pro Aufruf (2,3 % eines Kerns bei 500 Hz), unabhängig von
`--nozzle-group`, weil weiterhin alle 152 Düsen pro Sample durchlaufen
werden, nur gruppiert. Sie existiert ausschließlich, weil eine gröbere
vertikale Adressierung gewünscht war.

**Nicht zu verwechseln mit `--nozzle-block-size`/`--nozzle-order`**
(Düsen-Mapping, siehe unten): Das korrigiert eine **Vertauschung** in der
Verdrahtung — eine Zeilen-*Permutation* — ändert aber nichts daran, dass
jede Düse einzeln feuert, und ist nur außerhalb von `--mode page` erlaubt
(die Blockpermutation ist nach Bildzeile indiziert, aber die Zuordnung
Düse↔Zeile verschiebt sich in `--mode page` mit jeder vertikalen Bewegung —
siehe den entsprechenden Fehlertext unten). `--nozzle-group` vertauscht
nichts, sondern bindet benachbarte Düsen fest zusammen, und ist nur
*innerhalb* von `--mode page` erlaubt. Die beiden Optionen lösen
unterschiedliche Probleme und schließen sich schon durch den jeweils
erforderlichen Modus gegenseitig aus.

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
| `--page-frame calibrated\|simple` | Welchen 2D-Rahmen `--mode page` benutzt (Default `calibrated`). `simple` braucht keine Kalibrierung: Seitenachsen = Tracker-x/y, Nullpunkt beim START-Druck auf der Düsenleiste, Gierwinkel relativ zur beim START gehaltenen Pose. Schließt `--page-calibration` aus. Siehe Abschnitt „Einfacher Modus" oben. |
| `--simple-boresight QX QY QZ QW` | Nur mit `--page-frame simple`: pinnt die Gier-Referenzpose fest, statt sie beim START automatisch (und ungeprüft) zu erfassen. Vier leerzeichengetrennte Zahlen, nicht kommagetrennt. Siehe Abschnitt „Einfacher Modus" oben. |
| `--origin button\|startpoint` | Was den Nullpunkt setzt (START-Taster oder Startpoint-Charakteristik) |
| `--smooth-ms MS` | Tiefpass-Zeitkonstante (ms) gegen das verrauschte Amfitrack-Signal; `0` = aus, größer = glatter aber mehr Nachlauf (Default 12). Reduziert unregelmäßige Linien/Lücken. |
| `--min-move MM` | Deadband; darunter gilt der Kopf als stehend (Default 0.05) |
| `--poll-hz HZ` | Abtastrate der Position (Default 500). Ein Spaltenübergang kann nur einmal pro Abtastung bemerkt werden, das begrenzt also, wie genau eine Column platziert wird: bei 500 Hz und 20 mm/s sind das 0.04 mm = ein Fünftel Column. Mit `--profile-csv` messbar — die Abstände zwischen den Schreibzeitpunkten sind auf die effektive Schleifenperiode quantisiert. |
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
| `drill_pattern` | Rastert eine externe Bilddatei (z. B. ein Bohr-/Fadenkreuz-Justiermuster) auf die gewünschte physische Größe, statt ein Muster zu berechnen – siehe `--pattern-image` unten |

```bash
python main.py --pattern checkerboard --pattern-square-mm 10 --pattern-square-rows 20
python main.py --pattern diagonal --mode line --preview diag.png
```

| Option | Bedeutung |
|---|---|
| `--pattern-length-mm` | Physische Länge des Musters in mm (Default 200) |
| `--pattern-square-mm` | Kachel-/Streifenbreite in mm (checkerboard, v-stripes, diagonal-Periode) |
| `--pattern-square-rows` | Kachel-/Streifenhöhe in Zeilen (checkerboard, h-stripes) — Achtung Seitenverhältnis, siehe `--pattern-square-height-mm` im `--mode page`-Abschnitt oben |
| `--pattern-image PATH` | Bilddatei für `--pattern drill_pattern` (jedes von PIL lesbare Format: PNG, JPG, BMP, …) |

⚠️ **`drill_pattern` liefert kein Bild mit.** Anders als die übrigen Presets
berechnet `drill_pattern` nichts selbst, sondern liest eine Bilddatei ein.
Diese Datei ist **nicht** Teil dieses Repos — sie muss vom Anlagenbesitzer
selbst bereitgestellt werden, entweder am Default-Pfad
`assets/drill_pattern.png` (relativ zum `printhead/`-Paket, unabhängig vom
aktuellen Arbeitsverzeichnis) oder über `--pattern-image PATH` an einer
beliebigen anderen Stelle. Fehlt die Datei an beiden Stellen, bricht der
Befehl mit einer klaren Fehlermeldung ab (kein Traceback), die den exakt
gesuchten Pfad nennt:

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
Höhe (`rows`, standardmäßig `IMAGE_HEIGHT`) skaliert — **nicht** seitenverhältnis-
erhaltend. Das sieht auf den ersten Blick wie ein Bug aus, ist aber richtig: eine
Druck-Zelle ist `mm_per_column` breit, aber `NOZZLE_PITCH_MM` hoch, also zwei
unterschiedliche physische Maße — nur die unabhängige Skalierung auf die
angeforderte Spalten-/Zeilenzahl ergibt auf dem Papier die korrekten Proportionen.

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
gerade geliefert hat (sonst leer, nicht `0,0,0,0`). Ursprünglich reine Diagnosedaten
für die nachträgliche, manuelle Auswertung eines echten Druckdurchlaufs: die Hypothese
war, dass eine Rotation des Wagens zusammen mit dem festen Hebelarm Sensor→Düsenleiste
(`--sensor-offset-row-mm`, s. o.) die beobachteten Verzerrungen (nicht-parallele
Linien, Versatz bei mehreren Durchgängen) erklären könnte. Diese Hypothese hat sich an
echten Daten bestätigt (`pass5.csv`, siehe Abschnitt "Wagen-Rotation / Boresight"
oben) und ist inzwischen live korrigiert: `PageMapper.project()` rotiert den
Hebelarm-Versatz mit dem gemessenen Yaw, `CoverageEngine.step()` platziert jede der
152 Düsen einzeln entsprechend verteilt. Das Quaternion bleibt zusätzlich als
Rohdaten in `--profile-csv` erhalten, unabhängig davon, ob eine Boresight-Kalibrierung
vorliegt.

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

**`--record BILD.png` in `--mode page`** rekonstruiert anders als im Zeilen-Modus
oben: Es gibt hier nichts nachzubilden, `CoverageEngine` führt schon während des
Drucks live Buch, welches Pixel getroffen wurde. Das PNG zeigt drei übereinander
gestapelte Panels — INTENDED (Zielbild), COVERED (tatsächlich getroffen) und MISSED
(gewollt, aber nie getroffen) — plus ein viertes, farbiges **PATH**-Panel: die blau
gezeichnete Spur ist der **Sensor-Mittelpunkt**, orange die **Düsenleisten-Mitte**
(nicht Düse 0). Damit lässt sich eine MISSED-Stelle direkt gegen die Fahrspur
prüfen — ist der Wagen dort nie vorbeigekommen, oder war er zu schnell für
`--dose-hold-s`?

Das ganze PNG wird dabei standardmäßig **3-fach vergrößert** (`recording.
DEFAULT_RECORD_SCALE`) — INTENDED/COVERED/MISSED blockig (jeder Block bleibt exakt
eine reale Düsenzeile/-spalte, kein Weichzeichnen, das eine falsche
Sub-Pixel-Genauigkeit vortäuschen würde), das PATH-Panel direkt in voller
Zielauflösung gezeichnet (keine verpixelten Linien/Zahlen).

Auf beiden Spuren wird zusätzlich alle **2 Sekunden** (`recording.
DEFAULT_MARKER_INTERVAL_S`) ein größerer, durchnummerierter Punkt gesetzt — 1 beim
allerersten Sample, dann 2, 3, 4 … im 2-Sekunden-Takt, auf Sensor- und
Düsenleisten-Spur jeweils zur **exakt gleichen** Pass-Zeit. Damit lässt sich direkt
ablesen, wo Sensor und Düsenleiste zu welchem Zeitpunkt standen — z. B. um eine
MISSED-Stelle einem bestimmten Moment im Pass zuzuordnen. Ohne Zeitstempel (Aufrufer
ohne `sample_times`) bleibt es beim einfachen grünen Start-/dunklen End-Punkt wie
zuvor.

```bash
python main.py --pattern checkerboard --mode page --page-frame simple \
    --pattern-length-mm 60 --pattern-height-mm 100 --record coverage.png
```

⚠️ **Wichtig, damit der Sensor-Pfad überhaupt sichtbar wird:** Sensor und
Düsenleiste sitzen ~62 mm auseinander (`SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM`). Bei
konstanter Ausrichtung (kein Gieren während des Passes) liegt die blaue Sensor-Spur
deshalb **komplett außerhalb** eines nur ~15–20 mm hohen Zielbilds — sie ist real
vorhanden, nur schlicht nie im sichtbaren Canvas. Erst bei einem ausreichend hohen
Zielbild (`--pattern-height-mm` deutlich über ~62 mm) oder bei einem Pass mit
echtem Gieren (Wagen dreht sich während der Bewegung) laufen beide Spuren im
selben Bildausschnitt zusammen.

In der Web-UI erscheint das PATH-Panel automatisch im **🎞 Record**-Vergleichsbild,
sobald `--mode page` aktiv ist — keine zusätzliche Option nötig.

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
