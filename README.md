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

### Startpoint-Taster im Seiten-Modus: Startpunkt setzen / Druck abbrechen

Im Seiten-Modus hat der Startpoint-Taster **zwei** Bedeutungen, je nachdem, ob
gerade gedruckt wird:

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
sich mit der Wahl; die Zuordnung „Ausgabe nennt den Anker beim Namen"
(`CENTRE` / `LEFT EDGE, vertically centred` / `TOP-LEFT CORNER`) macht den
gesetzten Punkt auch ohne Blick in den Code nachvollziehbar.

Wichtig: Nur der **Ursprung** wird verschoben. Die abgefahrene Ebene aus
`page_calibration.json` (Achsen `e_col`/`e_row`, Skalen) bleibt komplett
unangetastet — die Kalibrierungsdatei definiert weiterhin *wo die Ebene liegt*,
der Taster nur *wo auf dem Blatt die Mitte des Bildes liegt*.

Der Ursprung bleibt über mehrere Pässe hinweg gesetzt (wie eine Kalibrierung),
bis er erneut per Taster verschoben wird. Im einfachen Modus
(`--page-frame simple`) hat ein so gesetzter Ursprung **Vorrang** vor dem
sonst automatischen Nullen beim START — sonst würde die bewusste Platzierung
still überschrieben. Ohne Tastendruck bleibt dort alles wie bisher (Nullpunkt
beim START-Druck).

Der Line-Modus (`--mode line`) behält seine bisherige, andere Taster-Bedeutung
(Nullpunkt-Reset mitten im Druck, Neubeginn bei Spalte 0) unverändert.

### START-Taster: manchmal zweimal drücken nötig behoben (Process-Stop-Fix)

**Symptom:** Der START-Taster wurde immer korrekt erkannt und angezeigt, der
Pässe-Zähler (`../..  gedruckt`) stieg beim ersten Druck normal hoch — aber es
floss weder Strom noch Tinte. Erst ein zweiter Tastendruck brachte den Druck
wirklich in Gang.

**Ursache:** Der physische START-Taster auf der Firmware ist ein reiner
Hardware-Toggle (`mainloop()` in `main.c`): Druck = Start, wenn gerade gestoppt;
Druck = Stop, wenn gerade am Laufen. Die Firmware hat aber keine andere
Möglichkeit zu erfahren, dass ein vom Client gesteuerter Pass **von selbst**
zu Ende gegangen ist (Timeout, volle Coverage, oder ein per Startpoint-Taster
ausgelöster Abbruch) — sie bleibt intern "running", obwohl der Client den Pass
bereits als beendet betrachtet. Der **nächste** physische Tastendruck des
Bedieners ist damit aus Firmware-Sicht bereits der **zweite** Druck seit dem
letzten Start und wird als STOP gelesen, während der Client denselben
Tastendruck als Beginn eines **neuen** Passes interpretiert: der Coverage-
Zähler springt normal hoch, aber der Druckkopf schaltet währenddessen ab —
kein Fehler, keine Tinte, kein Strom. Erst der Druck danach startet wieder
wirklich. Ein reiner Zwei-Repo-Bug: keine der beiden Seiten allein sieht
genug, um den Zustand des jeweils anderen zu rekonstruieren.

**Fix:** Neue GATT-Characteristic `PROCESS_STOP_UUID` (write-only, 1 Byte),
die der Client nach **jedem** Pass schreibt — unabhängig vom Modus
(`line`/`page`/`time`) und unabhängig davon, ob der Pass normal zu Ende ging
oder mit einem Fehler abbrach (`_run_ble()`'s `finally`-Block, läuft auf jedem
Ausstiegspfad). Die Firmware konsumiert das Signal genau einmal und beendet
I2S-Ausgabe/`process_running` selbst, statt auf einen zweiten physischen
Tastendruck zu warten. Der Schreibversuch selbst wirft nie eine Exception
(`request_process_stop()` fängt jeden Fehler ab und gibt nur `False` zurück) —
ein Fehlschlag hier darf niemals den Rücksprung zu
`Waiting for next START press ...` verhindern. Firmware ohne diese
Characteristic (älterer Flash-Stand) verhält sich wie bisher — kein neuer
Fehler, nur der alte Bug bleibt für diesen Fall bestehen.

Betrifft beide Repos: die Firmware-Seite (`ble_server.c`/`main.c`/
`README_BLE_INTERFACE.md`, siehe „6) Process Stop Characteristic" dort) und
diese Client-Seite (`geometry.py`, `ble_client.py`, `controller.py`). Beide
Seiten müssen den passenden Firmware-/Client-Stand tragen, damit der Fix
greift — mit alter Firmware bei neuem Client (oder umgekehrt) bleibt das
ursprüngliche Verhalten (Taster funktioniert, Characteristic wird ignoriert
bzw. nie geschrieben).

### Einfacher Modus: `--page-frame simple` (ohne Kalibrierung)

`--page-frame simple` überspringt die Seiten-Kalibrierung komplett und nimmt
direkt das Amfitrack-Koordinatensystem als Seiten-Rahmen:

> **Gierwinkel im einfachen Modus: absoluter Twist um die z-Achse.**
> Der Gierwinkel kommt hier aus `rotation.twist_about_axis` — der vom
> Hardware-Betreiber selbst erprobten Berechnung aus dessen
> `amfitrack_live_pose.py` (`quaternion_twist_angle_deg` mit Achse
> `(0, 0, 1)`), wortwörtlich portiert und über 20 000 Zufallsquaternionen ×
> 4 Achsen gegen die Vorlage geprüft (größte Abweichung 2,3 · 10⁻¹³ Grad).
>
> Der Unterschied zur vorherigen Rechnung (`yaw_about_normal`): **keine
> Referenzpose nötig.** Es ist der absolute Twist des Wagens um die Achse,
> nicht die Drehung relativ zu einem aufgenommenen Boresight. Genau das war
> hier wiederholt die Schwachstelle — blinde Erfassung beim ersten Sample
> greift irgendeine Pose ab (BLE noch nicht eingeschwungen, Hand noch nicht
> ruhig), und der gespeicherte Boresight der Anlage lag ~110° neben „flach".
> Eine absolute Ablesung hat dieses Fehlerbild nicht: dieselbe physische
> Orientierung liefert lauf für lauf denselben Wert. Zusätzlich ist der Wert
> auf ±180° gewickelt und damit immun gegen die Quaternion-Doppel­überdeckung
> (`q` und `−q` lesen gleich).
>
> An deinem echten 360°-Handdreh-Datensatz nachgemessen: Verstärkung
> (gemeldeter Gierwinkel pro Grad echter Drehung, endpunktgemessen) **0,994
> bis 1,006**, effektiv monoton (ein einziger „Rückschritt" über 242 Samples,
> und der beträgt exakt 0,0°), Gesamtsumme 366,9° für eine 360°-Handdrehung.
>
> Ein aufgenommener oder per `--simple-boresight` gesetzter Boresight
> verschiebt weiterhin nur den **Nullpunkt** (sein eigener Twist um dieselbe
> Achse wird abgezogen), damit die Pose, an der er erfasst wurde, weiterhin
> 0° liest.
>
> **Nicht** umgestellt wurden Roll/Nick: Drei unabhängige Einzelachsen-Twists
> sind keine orthogonale Zerlegung einer Drehung — so gelesen meldet eine
> *reine* Drehung um die Normale bereits Ausschläge auf den anderen beiden
> Achsen (gemessen: 15° flache Drehung aus der realen Montagepose ergibt 15°
> „Roll"). Für die Live-Anzeige des Betreibers, wo jede Achse für sich
> betrachtet wird, ist das in Ordnung; hier sind Roll/Nick aber gerade der
> „steht der Wagen schief?"-Indikator und wären damit unbrauchbar. Sie
> behalten deshalb die boresight-relative Swing-Twist-Rechnung. Der
> **kalibrierte** Modus bleibt vollständig unverändert.

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
gerenderte Zielbild ist mit 152 Düsenreihen nur ca. 13,1 mm
hoch (`NOZZLE_PITCH_MM * 151`, = `NOZZLE_BAR_SPAN_MM`). Damit überhaupt eine Düse zündet, muss der
Wagen in `v`-Richtung auf ca. ±13 mm um die abgefahrene Spaltenkante herum
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

### Latenz-Kompensation (`--latency-compensate-s`)

Zwischen "Position gelesen" und "Tinte tatsächlich platziert" liegt eine
messbare Pipeline-Verzögerung: das ausgehandelte BLE-Verbindungsintervall
(auf echter Hardware gemessen: durchgängig 15,00 ms, `itvl=12`) und die
Firmware-Warteschlange. Zusammen ergibt das grob **5 ms bestenfalls, ~13 ms
typisch, ~21 ms im ungünstigsten Fall** — bei 20 mm/s sind das ca. 0,26 mm
bzw. 3 Spalten systematischer Nachlauf, der bei einem Richtungswechsel das
Vorzeichen wechselt.

> Die Firmware-Anteile darin stammen noch aus dem alten Seiten-Modus
> (6-Slot-Queue × 450 µs plus `PATTERN_STRIDE` × 450 µs Feuer-Takt). Der neue
> Seiten-Modus fährt einen 128-Spalten-FIFO mit 300 µs Takt und feuert jede
> Spalte genau einmal; die Größenordnung bleibt, die genaue Aufteilung ist
> **nicht neu vermessen** — der Wert oben ist als Startpunkt zu lesen, nicht
> als aktuelle Messung.

`--latency-compensate-s SEKUNDEN` extrapoliert **nur** die an
`CoverageEngine.step()` übergebene Position linear entlang der aktuell
gemessenen Geschwindigkeit nach vorn (`u/v + Geschwindigkeit × Sekunden`) —
alles andere (die `--record`-Pfad-Panels, die Out-of-Page-Prüfung, der
Profiler, die Speed-Warnung) bleibt auf der echten, unkompensierten Position,
da diese zeigen sollen, wo der Wagen wirklich war. Default `0.0` = aus,
heutiges Verhalten.

⚠️ Das ist eine **Heuristik gegen einen geschätzten Wert**, kein
Allzweck-Glättungsregler: ein zu hoher Wert schießt vor allem beim Abbremsen
oder Richtungswechsel kurz übers Ziel hinaus (die Extrapolation nutzt noch
die Geschwindigkeit von kurz davor). Klein anfangen und gegen einen echten
Druck prüfen, bevor man sich darauf verlässt. Die Geschwindigkeitsschätzung
selbst (Differenz aufeinanderfolgender Positionen) verstärkt außerdem
Rauschen — je größer der gewählte Wert, desto empfindlicher.

### Gierwinkel-Singularität behoben: Swing-Twist statt Rotationsvektor

Die relative Rotation (aktuelle Pose gegen Boresight) wurde bisher als
Rotationsmatrix gebaut und über deren Achsen-Winkel-„Rotationsvektor"
(Log-Map von SO(3)) auf die Seitennormale projiziert. Mit der echten
Kalibrierung des Betreibers (`e_col`/`e_row`/Boresight aus dessen
`page_calibration.json`) und einer synthetischen, reinen Drehung um genau
diese Seitennormale gemessen:

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
Division durch `sin(angle)` im Rotationsvektor geht dort gegen 0), danach
ist sie um exakt 360° vorzeichenverkehrt — eine Rotations**matrix**, anders
als ein Quaternion, „weiß" nicht mehr, in welche Richtung eine Drehung über
180° hinausging. Auf echter Hardware zeigte sich genau diese Fehlerform
schon **vor** der sauberen 180°-Grenze: ein Sprung von **-109° auf +109°**
bei einer echten 180°-Drehung.

`printhead/rotation.py` (`yaw_about_normal`) berechnet den Gierwinkel jetzt
über eine **Swing-Twist-Zerlegung direkt auf dem Rotations-Quaternion**
(`quat * conj(boresight_quat)`, über neue `_quat_multiply`/`_quat_conjugate`-
Helfer — bewusst *nicht* über eine Matrix: die Rückrichtung
Matrix→Quaternion hätte dieselbe Art Instabilität nahe 180° wieder
eingeschleppt):

```
v, w = Vektor-/Skalarteil von quat_rel
twist_rad = 2 * atan2(dot(v, n_hat), w)
```

Kein Ausdruck darin geht bei 180° gegen 0/0 — keine Singularität. Ein
gewisser Umschlag ist für **jede** Ein-Zahl-Winkeldarstellung mathematisch
unvermeidbar (dieselbe physische Orientierung ist ab einer vollen Umdrehung
über zwei unterschiedlich vorzeichenbehaftete Quaternionen erreichbar),
landet hier aber erst bei einer **vollen** Drehung (±360°) statt schon bei
180° — weit jenseits des größten je auf dieser Anlage gemessenen
Gierwinkels (75,6° über einen ganzen Durchlauf, s. o.) — und ist für die
Druckkorrektur ohnehin folgenlos, weil dort nur `sin`/`cos` des Gierwinkels
verwendet werden (`PageMapper.project`), beide 360°-periodisch.

Die neue Methode ist zusätzlich nicht nur singularitätsfrei, sondern
**exakt statt nur näherungsweise**, sobald Neigung (Roll/Pitch) mit dabei
ist: Bei 75° Neigung um eine Diagonalachse plus 40° injiziertem Gierwinkel
lieferte die alte Rotationsvektor-Methode 34,0° statt 40° — die neue exakt
40,0°. `cart_rotation_angles` (Roll/Pitch, weiterhin rein diagnostisch,
siehe oben) liest Roll/Pitch jetzt aus dem um den Twist bereinigten
„Swing"-Quaternion statt direkt aus der vollen Relativ-Rotation — sonst
kippt Roll/Pitch fälschlich um, sobald allein der Gierwinkel über 180°
steigt (derselbe Skalarteil wird sonst von beidem gleichzeitig
„verschmutzt").

**Neue/aktualisierte Tests:**

```
tests/test_rotation.py
  test_yaw_about_normal_combined_tilt_and_yaw_recovers_exactly_where_the_old_method_drifted
    (ersetzt test_yaw_about_normal_combined_tilt_and_yaw_where_naive_projection_disagrees_with_itself:
     die alte Methode selbst driftet bei Neigung -- der Vergleichswert musste
     auf die swing-twist-Formel umgestellt werden, siehe Kommentar im Test)
  test_yaw_about_normal_pure_rotation_recovers_exactly_through_a_full_sweep   (0/45/90/135/179/180/225/270/315°)
  test_yaw_about_normal_MUTATION_check_old_method_sign_flips_past_180        (baut die entfernte alte Methode nach)
  test_yaw_about_normal_normalises_non_unit_quat_and_boresight               (Norm 1.00002 wie beim echten Boresight)
  test_yaw_about_normal_rejects_a_zero_norm_quat
  test_cart_rotation_angles_roll_pitch_stay_small_when_yaw_exceeds_180
  test_yaw_about_normal_double_cover_shifts_the_READOUT_by_exactly_360   (siehe Nachtrag unten)
  test_yaw_about_normal_double_cover_cannot_affect_the_PRINT_correction
  test_cart_rotation_angles_roll_pitch_are_double_cover_INVARIANT

tests/test_calibration.py
  test_simple_frame_identity_boresight_would_be_wrong   (aktualisiert -- siehe unten)
```

#### Nachtrag (Verifikation): was der Wertebereich ±360° kostet

Die neue Methode liefert den Gierwinkel im Bereich **(−360°, +360°]** —
absichtlich **nicht** auf ±180° geklemmt, weil eine Klemmung den von dir
beobachteten Sprung nur von seiner jetzigen Stelle auf 180° zurückverlegen
würde, statt ihn zu beseitigen.

Der Preis dafür, gemessen und vorher nicht dokumentiert: Die Methode ist
**nicht mehr invariant gegen die Quaternion-Doppelüberdeckung**. `q` und
`−q` sind dieselbe physische Orientierung; die alte Matrix-Methode war
dagegen konstruktionsbedingt immun (`R(q) == R(−q)`), die neue liest die
Quaternion-Komponenten direkt. Mit deiner echten Kalibrierung gemessen:
`−q` statt `q` verschiebt die Ausgabe um **exakt 360°**, und zwar bei
**jedem** getesteten Winkel (0/30/75/90/135/179/180/225/270/315°) — nicht
nur nahe einer vollen Umdrehung. Dasselbe gilt für ein vorzeichenverkehrtes
`boresight_quat`.

Bewusst akzeptiert, aus diesen Gründen:

- **Der Druck kann davon nicht betroffen sein.** `PageMapper.project()` und
  `CoverageEngine` verwenden nur `sin`/`cos` dieses Winkels, beide exakt
  360°-periodisch — numerisch über einen vollen Sweep nachgemessen, nicht
  nur behauptet: **0 von 52** abgetasteten Winkeln zeigten irgendeinen
  `sin`/`cos`-Unterschied zwischen beiden Vorzeichen. Roll/Pitch sind
  ebenfalls unbetroffen (sie werden am Swing-Quaternion abgelesen, nachdem
  der Twist herausgerechnet wurde) — auch das ist jetzt festgenagelt.
- **Dein beobachtetes Symptom passt nicht zu einem Vorzeichenwechsel.** Der
  Sprung trat reproduzierbar bei realen 180° auf — der tatsächlichen
  Singularität der alten Methode — nie zu zufälligen Zeitpunkten. Ein
  Tracker, der das Vorzeichen springen lässt, hätte zufällige Sprünge
  erzeugt.

Falls du je einen **360°-Sprung im Stillstand** siehst: Das ist die
Diagnose, und der Fix ist eine Zeile (`quat_rel` auf `qw >= 0`
normalisieren, dokumentiert im Docstring von `yaw_about_normal`). Das stellt
die Invarianz her und kostet genau den weiten Wertebereich — deshalb erst
dann, nicht vorsorglich.

⚠️ **Zwei Bestandstests mussten inhaltlich angepasst werden**, weil sie
nachweislich nur das Verhalten der ALTEN Methode gemessen hatten, nicht
etwas, das unabhängig von der Implementierung gelten muss:

- Der kombinierte Neigung+Gier-Test in `test_rotation.py` verglich bisher
  gegen eine frisch nachgerechnete Rotationsvektor-Formel — die driftet
  aber selbst (34,0° statt 40°, s. o.). Umgestellt auf eine unabhängige
  swing-twist-Nachrechnung; die alte Methode bleibt als Kontrastwert im
  selben Test erhalten (`old_deg` muss weiterhin deutlich abweichen).
- `test_simple_frame_identity_boresight_would_be_wrong` in
  `test_calibration.py` maß bisher die **Differenz** zweier Gierwinkel-
  Ablesungen bei einer flachen 90°-Drehung — diese Differenz ist mit
  swing-twist jetzt exakt boresight-unabhängig richtig (ein allgemeiner,
  im selben Test dokumentierter Effekt: eine zusätzliche, weltfeste
  Zusatzdrehung addiert sich exakt zum Ausgangswert, egal welche Neigung
  die Ausgangspose sonst hatte). Der Test selbst hatte das mit einem
  Kommentar vorgesehen ("if this now holds, revisit the design note").
  Er prüft jetzt stattdessen, was Identitäts-Boresight nach wie vor falsch
  macht: die **absolute** Gierwinkel-Ablesung an der Referenzpose (statt 0°
  kommen -91,4° heraus) und Roll/Pitch (statt ~0° kommen ~88,8° heraus, weil
  die reale ~120°-Montageverdrehung des Sensors ungefiltert als „Neigung"
  erscheint).

### Kalibrierungsqualität: Fit-Metriken und Warnungen

`calibrate_page()` prüfte bisher nur, ob die beiden abgefahrenen Kanten nahe
genug an 90° zueinander liegen (`CalibrationAngleWarning`, Toleranz
`MAX_ANGLE_ERROR_DEG = 15°`) — wie GUT der Linien-Fit einer einzelnen Kante
für sich selbst ist (kurz? verrauscht? wenige Samples?), wurde nirgends
gemessen. Jetzt liefert jede Kante zusätzlich drei Fit-Kennzahlen
(`fit_axis_quality()`, ein neuer, eigenständiger Helfer — `fit_axis()`s
bisherige 2-Werte-Rückgabe bleibt unverändert, jeder bestehende Aufrufer
funktioniert unverändert weiter):

- **Länge** (mm) entlang der gefitteten Richtung,
- **RMS-Residuum** (mm) senkrecht zur gefitteten Linie,
- **Sample-Anzahl**.

Dazu die Neigung der gefitteten Seitennormale gegen die Tracker-z-Achse
(`normal_tilt_deg`). Alle vier Werte landen auf `PageCalibration` (neue,
**optionale** Felder — bestehende gespeicherte Kalibrierungen ohne diese
Felder laden weiterhin klaglos, mit `None` statt erfundenen Werten) und
werden mitgespeichert/-geladen.

`calibrate_page()` warnt jetzt zusätzlich (`CalibrationQualityWarning`,
eigene Warnklasse neben `CalibrationAngleWarning`, beide können unabhängig
voneinander auftreten) auf einer Kante, die kürzer als **50 mm** ist, ein
RMS-Residuum über **1 mm** hat, oder aus weniger als **20 Samples**
besteht. Diese Schwellen stammen aus einer Messreihe (synthetische gerade
Kante, Länge/Rauschen/Sample-Anzahl variiert → resultierender Fehler in der
gefitteten Seitennormale):

```
 Kantenlänge  Rauschen  Samples | resultierender Seitennormalen-Fehler
    210 mm    0.05mm      200   |   0.00° (max 0.01°)
    100 mm    0.5 mm      100   |   0.12° (max 0.37°)
     50 mm    1.0 mm       50   |   0.65° (max 1.40°)
     30 mm    2.0 mm       30   |   3.16° (max 6.25°)
     20 mm    3.0 mm       20   |   7.23° (max 18.63°)
```

und dem separat gemessenen Zusammenhang `Gierwinkel-Fehler ≈
Neigungswinkel * sin(Seitennormalen-Fehler)` — der Grund, warum eine
schlechte Seitennormale überhaupt etwas ausmacht, obwohl nirgends direkt
Roll/Pitch korrigiert wird (siehe oben): Der Gierwinkel wird relativ zur
gefitteten Normale gemessen, eine falsche Normale verwandelt also
gewöhnliches Tracker-Rauschen in Neigung (Median 2,7°, Max 7,8° auf dieser
Anlage) in **scheinbaren Gierwinkel-Fehler**.

⚠️ **Wichtig:** Die Kalibrierung des Betreibers selbst ist GUT (0,63°
Normalen-Neigung, 0,92° Orthogonalitätsfehler — weit im grünen Bereich).
Diese Warnungen erklären also **nicht** das Gierwinkel-Problem, das mit der
180°-Singularität oben behoben wurde — sie fangen künftig schlechte
Kalibrierungen im Allgemeinen ab.

Sichtbar im **Calibration**-Tab der Web-UI direkt neben dem bisherigen
Winkelfehler (Kantenlänge/Samples/RMS pro Kante, Normalen-Neigung; „n/a" bei
einer geladenen Datei ohne diese Metriken), und als Konsolenzeile, sobald
eine Kalibrierung berechnet wird — dort läuft `calibrate_page()` als Teil
des Web-UI-Serverprozesses, der einzige Ort in diesem Projekt, an dem eine
Kalibrierung überhaupt berechnet wird.

#### Nachtrag (Verifikation): RMS-Residuum wurde vom falschen Bezugspunkt gemessen

Bei der Überprüfung der obigen Implementierung ist ein echter Fehler in
`fit_axis_quality()` aufgefallen: Das RMS-Residuum wurde relativ zu
`samples[0]` gemessen statt relativ zur **gefitteten Linie**. `fit_axis()`
legt seinen PCA-Fit durch den **Schwerpunkt** der Samples — `samples[0]` ist
dagegen einfach ein weiteres verrauschtes Sample und liegt gar nicht auf der
Linie. (In `trace_length_mm()` ist derselbe Bezugspunkt harmlos, weil dort
`max − min` gebildet wird und sich der Bezugspunkt herauskürzt; hier nicht.)

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

**Neue Tests:**

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

**Größeres, richtig proportioniertes Testmuster in `--mode page`:** Genau weil
`--mode page` nicht auf die 15,2 mm der 152 Düsen begrenzt ist, lohnt sich für
den Bring-up ein deutlich größeres `--calibrate`/`--pattern`-Bild als die
sonst übliche `IMAGE_HEIGHT`-Zeilenzahl:

| Option | Bedeutung |
|---|---|
| `--pattern-height-mm MM` | Physische Gesamthöhe von `--calibrate`/`--pattern` in mm (`rows = height_mm / NOZZLE_PITCH_MM`). Nur mit `--mode page` gültig — im Zeilen-/Zeit-Modus packt `frames_from_ink()` feste Frames mit genau `IMAGE_HEIGHT` Zeilen, eine andere Höhe wird dort mit einem klaren Fehler abgelehnt. Ohne diese Option bleibt das Muster bei `IMAGE_HEIGHT` Zeilen (13,2 mm, = `NOZZLE_BAR_WIDTH_MM`) gedeckelt. |
| `--pattern-square-height-mm MM` | Zeilenperiode in mm für checkerboard/h-stripes, überschreibt `--pattern-square-rows` (`square_rows = v / NOZZLE_PITCH_MM`). |

⚠️ **Seitenverhältnis-Falle:** Eine Bildzeile ist nur ca. **0,087 mm** hoch
(`NOZZLE_PITCH_MM`, aus der 13,2mm/152-Neuvermessung). `--pattern-square-rows 20` (der Default) ist damit
nur ca. **1,74 mm** hoch, während `--pattern-square-mm 10` (der Default) **10 mm**
breit ist — ein ~5,8:1-Streifen statt eines Quadrats. Für tatsächlich quadratische
Kacheln `--pattern-square-height-mm` statt `--pattern-square-rows` verwenden.

```bash
# Großes Schachbrett in Seiten-Modus: 200mm x 100mm Gesamtfläche, 10mm-Quadrate.
python main.py --pattern checkerboard --mode page --page-calibration page_calibration.json \
    --pattern-length-mm 200 --pattern-height-mm 100 \
    --pattern-square-mm 10 --pattern-square-height-mm 10
```

**Dosierung in `--mode page` (`--drops-per-pixel`):** Ein Pixel gilt als
gedruckt, sobald es `--drops-per-pixel` Tropfen bekommen hat — Default
`coverage.DEFAULT_DROPS_PER_PIXEL = 2`. Wie viele Kopien einer Spalte dafür
rausgehen, entscheidet **allein der zurückgelegte Weg**:

```
Kopien für dieses Sample = --drops-per-pixel × gefahrener Weg / --mm-per-column
```

Der Bruchteil wird in einem Akkumulator über die Samples mitgeschleppt, damit
nichts durch Abschneiden verlorengeht. Gebrochene Werte sind erlaubt — der
Regler muss nach unten feiner sein als „ganz aus".

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
Umstellung: der Default stand auf 3, kopiert aus dem Firmware-Konstanten
`BLE_DROPS_PER_COLUMN` des Zeilen-Modus, ohne zu bemerken, dass die 3 zu einer
**0,2 mm** breiten Spalte gehört. Bei 0,087 mm ergibt das gut die dreifache
Tintenmenge — von der Hardware zurückgemeldet als „jetzt kommt zu viel raus",
gegenüber einem vorherigen Druck, der heller **und schärfer** war. Die 11,5
Tropfen/mm sind genau das, was der Client **vor** der Umstellung bei langsamer
Fahrt geliefert hat (simuliert: 120 Spalten Tinte auf 120 Spalten Fahrweg),
also die Dichte, die auf echtem Papier beurteilt wurde — daran ist dieser
Wert verankert.

Der Default steht inzwischen auf **2** (23,0 Tropfen/mm), also bewusst auf
dem Doppelten dieser Dichte: nach einer Reihe echter Drucke mit explizitem
`--drops-per-pixel 2` auf der Kommandozeile vom Anlagenbesitzer so
festgelegt. Auf Papier entschieden, was der einzige Ort ist, an dem sich
diese Frage entscheiden lässt. Kommt der Druck zu dunkel oder verlaufen,
ist das der Regler.

Gegenprobe aus der Physik: ein Tropfen läuft auf ~60–120 µm aus, eine Spalte
ist 87 µm breit — **ein** Tropfen deckt sie also bereits ab.

Der Wert ist eine erste Kalibrierung, keine fertige: kommt ein Druck blass
heraus, hochsetzen; kommt er verlaufen heraus, runter (z. B. `0.7`).

Das ist **geschwindigkeitsunabhängig per Konstruktion**: doppeltes Tempo heißt
doppelter Weg je Sample, also doppelt so viele Kopien in der halben Zeit — die
gleiche Tintenmenge pro Spalte. Ein *stehender* Wagen ist entsprechend gar
nichts schuldig und feuert nicht (das ersetzt die alte Stillstands-Logik gegen
Tintenklekse).

⚠️ **Umstellung vom Verweildauer-Modell (`--dose-hold-s` gibt es nicht mehr).**
Die alte Firmware **hielt** das zuletzt geschriebene Muster und feuerte es alle
`PATTERN_STRIDE` Ticks erneut; die Tinte hing also daran, wie *lange* der
Client ein Düsenbit auf 1 hielt, und beide Seiten waren über
`DOSE_HOLD_S ≈ 3 × PATTERN_STRIDE × Tick` gekoppelt. Die neue Firmware feuert
jede empfangene Spalte **genau einmal** und wiederholt sie nie
(`PATTERN_STRIDE` und `pattern_dose_should_fire()` sind aus `ble_dose.h`
verschwunden). Damit gibt es keine Wiederholrate mehr, gegen die man halten
könnte — die Tinte wird vollständig hier entschieden.

Zwei Konsequenzen, die beide Verhalten ändern:

1. **Der Client sendet jetzt bei jedem Sample mit offener Tintenschuld, nicht
   nur bei Musterwechsel.** Unter einer Feuer-einmal-Firmware bedeutet „nur
   bei Änderung senden“, dass eine gleichmäßige Fläche (ein gefüllter Block,
   eine breite Linie) *gar nichts* sendet: die Menge der gewollten Düsen
   ändert sich dort von Sample zu Sample nicht. Gemessen an einem 120 Spalten
   breiten Vollblock bei 30 mm/s: **2 Schreibvorgänge für den ganzen
   Durchlauf** statt der ~360, die das Tintenbudget verlangt — während die
   Deckung 100 % meldete.
2. **`PatternSender` ist keine „latest wins“-Mailbox mehr, sondern eine
   begrenzte Warteschlange.** Eine überholte Spalte war früher wertlos (die
   gehaltene färbte ohnehin weiter); heute ist jede verworfene Spalte
   verlorene *Tinte*. Mehrere Spalten gehen gebündelt in einem Schreibvorgang
   raus (bis zu `BLE_NOZZLE_MAX_COLS_PER_WRITE = 32`, in der Praxis 12 bei
   MTU 247). Läuft die Warteschlange über, fliegt die **älteste** Spalte
   (die, deren Position der Wagen am sichersten schon verlassen hat) und wird
   in `PatternSender.dropped` gezählt statt still geschluckt.

**Was die Geschwindigkeit jetzt begrenzt, ist die Abtastrate, nicht die
Dosis.** Eine Spalte, die der Tracker nie abgetastet hat, wird nie gefeuert;
diese Kante liegt bei `--mm-per-column × --poll-hz` = 0,087 × 500 =
**43,5 mm/s**. Simuliert über einen 120-Spalten-Vollblock:

```
Geschwindigkeit   Samples/Spalte   printed   fired
      17,3 mm/s             2,51    100,0 %   100,0 %
      30,0 mm/s             1,45    100,0 %   100,0 %
      43,5 mm/s             1,00    100,0 %   100,0 %
      50,0 mm/s             0,87     86,7 %    86,7 %
      60,0 mm/s             0,72     72,5 %    72,5 %
```

Zum Vergleich das alte Verweildauer-Modell an derselben Strecke: 73,3 %
`printed` gegen 99,3 % `fired` bei 25 mm/s, 44,2 % gegen 99,3 % bei 30 — eine
vollständig eingefärbte Seite, die als stark gestreift gemeldet wurde. Genau
das war die Rückmeldung von der Hardware („die Füllung des echten Drucks ist
perfekt, das Coverage-Bild sieht völlig anders aus“).

Man beachte, was die beiden Spalten oben **zusammen** sagen: unterhalb von
43,5 mm/s sind beide 100 %; darüber fallen sie **gemeinsam**, weil der Fehler
keine Unterdosierung mehr ist, sondern komplett übersprungene Spalten. `fired`,
das `printed` nach unten folgt, ist die Signatur davon — Tinte, die auf dem
Papier fehlt, nicht bloß in der Buchhaltung.

BLE ist dabei nicht die Grenze: jeder Tropfen ist eine gesendete Spalte, also
`--drops-per-pixel × v / --mm-per-column` Spalten/s — selbst 43,5 mm/s
verlangen beim Default nur 500 Spalten/s = 42 Schreibvorgänge/s bei 12 Spalten
je Vorgang, weit unter den gemessenen ~270/s. (Mit `--drops-per-pixel 3` wären
es 1500 Spalten/s bzw. 125 Schreibvorgänge/s — immer noch drin, aber bei
`--batch-cols 1` bereits über der Decke.)

⚠️ **Firmware-Kopplung:** Erfordert die Firmware mit dem
Feuer-einmal-Seitenmodus (Branch `claude/ble-i2s-nozzle-frequency-axpot1` im
Repo `Printhead_Original_V2`). Gegen die **alte**, das Muster haltende
Firmware würde dieser Client massiv überdrucken, weil er jetzt bei jedem
Sample sendet. `--drops-per-pixel` ist dagegen **nicht** mehr an eine
Firmware-Konstante gekoppelt — es ist ein reiner Client-Wert und kann ohne
Neu-Flashen verändert werden.

⚠️ **Neue Startwarnung statt der Quantisierungs-Klippe.** Die alte Warnung
(„`dose_hold_s` muss unter dem Poll-Intervall bleiben“) ist ersatzlos weg —
eine Tropfenzahl hat mit der Poll-Rate nichts zu tun. An ihre Stelle tritt
eine Warnung, wenn die Spalten-Kante (`--mm-per-column × --poll-hz`) auf oder
unter der Geschwindigkeitswarnung (`--speed-warning-mm-s`, Default 25 mm/s)
liegt: dann kann die Warnung den Schaden nicht mehr ankündigen. Bei den
Defaults (43,5 gegen 25 mm/s) feuert sie nicht; bei `--poll-hz 200` läge die
Kante bei 17,4 mm/s und sie feuert.

⚠️ **Fehler behoben: Verweildauer ging bei Zeilen-Flapping komplett verloren,
`coverage.png` zeigte deutlich weniger als real gedruckt wurde.**
`NOZZLE_PITCH_MM` (ca. 0,087 mm) ist feiner als reales Tracker-Rauschen. Steht eine
Düse nahe an einer Zeilengrenze, kippt die gerundete Zeile von Sample zu
Sample zwischen zwei Nachbarn — die Engine hat den Dosis-Zähler bisher
bei **jedem** Wechsel auf 0 zurückgesetzt. Die Düse feuert dabei trotzdem
(`active[p]` wird gesetzt, sobald ein Pixel gewollt ist — unabhängig davon,
ob die Dosis schon voll ist), aber die volle Dosis wurde nie
erreicht, weil der Zähler nie über einen Sample-Wechsel hinweg überlebt hat.
Reproduziert: Düse (fast) still auf einer Zeilengrenze, nur ±0,001 mm
Rauschen (zwei Größenordnungen unter realem Sensorrauschen) — **200 von 200
Samples feuern real, aber 0 Pixel werden als gedruckt verbucht.** Eine
Rausch-Messreihe zeigt zusätzlich: Mit mehr (realistischerem) Rauschen
feuert die Düse öfter, aber die verbuchte Fläche geht **runter statt
rauf** — das genaue Gegenteil dessen, was man erwarten würde:

```
Rauschen (mm)   Proben mit Feuern   verbuchte Pixel
         0.00           164/1000                192
         0.05           491/1000                191
         0.20           963/1000                173
```

Behoben, indem die Dosis jetzt **pro Pixel** akkumuliert wird
(`CoverageEngine._pixel_drops`, ein Dict, keyed auf `(row, col)`) statt pro
Düsen-/Gruppen-Slot mit Reset bei jedem Zeilenwechsel. Ein Wechsel weg von
einem Pixel — sei es Flapping zur Nachbarzeile oder ein längerer Ausflug,
weil der Wagen woanders hin fährt — lässt die bereits angesammelte Dosis
unangetastet; sie läuft beim nächsten Besuch einfach weiter, statt bei 0 neu
zu beginnen. Der Eintrag wird erst beim Fertigstellen des Pixels aus dem Dict
entfernt, der Speicherbedarf bleibt also auf „gerade angefangene, noch nicht
fertige Pixel" begrenzt, nicht auf die Bildgröße.

**Zwei Schwellen auf einem Konto.** `printed` (der Report, `coverage.done` und
die Zahl am Durchlaufende) wird **eine Probe früher** gesetzt als die Düse
freigegeben wird. Grund: die Gutschrift kommt in ganzen Poll-Samples an. Ist
eine Spaltenüberquerung `m` Samples wert, landen `floor(m)` oder `ceil(m)`
davon *innerhalb* der Spalte — eine vollständig überfahrene Spalte kann also
allein dadurch eine Probe zu kurz kommen, wie das Sample-Raster gerade zum
Spalten-Raster steht. Ohne diese eine Probe Spiel meldet der Report regelmäßige
Streifen, deren Dichte sinnlos mit dem Tempo schwankt (gemessen: 70,0 % der
Spalten bei 5 mm/s, 35,0 % bei 10, 51,7 % bei 17,3, 9,2 % bei 40 — gegen
durchgehend 100 % `fired`). Die Düse **freizugeben** darf dagegen erst die
strenge Schwelle: wer beides auf der lockeren Schwelle macht, kürzt jede
Überquerung um bis zu eine Probe echte Tinte — gemessen beim Default-Dose
142 statt 199 Spalten/s bei 17,3 mm/s, 74 statt 285 bei 25 und 189 statt 343
bei 30, also 30–75 % zu wenig, während die Deckung weiter 100 % meldet.

**Neue Tests** (`tests/test_coverage.py`, `tests/test_freehand_pass.py`,
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

### Tintenausbreitung: `--spray-radius-mm` / `--spray-strength`

Ein echter Tropfen landet nicht exakt in *einer* Rasterzelle, er benetzt eine
kleine Fläche drumherum. Ohne dieses Modell passiert Folgendes: Bei einer
Rückfahrt sitzt der Wagen ein paar Zehntel-mm versetzt, die Düsen adressieren
dadurch **andere Zeilen-Indizes**, diese gelten als „noch nicht gedruckt" — und
es wird erneut über Papier gedruckt, auf dem längst Tinte ist.

| Option | Bedeutung |
|---|---|
| `--spray-radius-mm MM` | Physischer Radius um ein fertiges Pixel, der eine Teildosis abbekommt. **In Millimetern, nicht in Pixeln** — eine Zelle ist ca. 0,087 mm hoch, aber `--mm-per-column` (Default 0.2 mm) breit, ein runder Tropfen ist im Raster also ~2,3:1 elliptisch. Default `0` = aus. |
| `--spray-strength F` | Dosis, die ein **direkt angrenzendes** Pixel abbekommt (0.0–1.0), linear abfallend bis 0 am Radius. Ein Pixel gilt ab Gesamtdosis 1.0 als gedruckt: bei `1.0` markiert ein einzelner Tropfen die Nachbarzelle sofort mit, bei `0.5` sind zwei Tropfen nötig. Default `0` = aus. |

Beide müssen `> 0` sein, damit das Modell greift; sonst verhält sich die Engine
exakt wie zuvor (Default-Verhalten unverändert).

Gemessen an simulierten Mehrfach-Überfahrten mit 0,05 mm Versatz pro Durchgang
(40 × 30 mm Vollfläche, 500 Hz; gemessen noch unter dem alten
Verweildauer-Modell mit `dose_hold_s = 0.0018` — die *relative* Wirkung des
Spray-Modells hängt nicht daran, die absoluten Feuerungszahlen schon):

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
`controller.DEFAULT_SPEED_WARNING_MM_S = 25.0` mm/s. Der Wert stammte
ursprünglich aus dem Verweildauer-Modell, wo die Coverage bei 25 mm/s bereits
auf ~60 % gefallen war. Unter dem Tropfenmodell ist er **bewusster
Sicherheitsabstand** statt Klippenkante: die erste Geschwindigkeit, bei der
real etwas verlorengeht, ist `--mm-per-column × --poll-hz` = 43,5 mm/s (siehe
Dosierungs-Abschnitt), 25 mm/s liegt ~40 % darunter. Das lässt Luft für
Übersteuern der Hand zwischen zwei Samples und für ein kleineres `--poll-hz`
(bei `--poll-hz 200` läge die Kante bei 17,4 mm/s — dann warnt der Client
beim Start, siehe dort). Die Firmware nutzt den Wert nur, um die (zu diesem
Zweck umgewidmete) HEALTH-LED anzusteuern — auf die Dosierung hat er keinen
Einfluss.

Um an der Schwelle nicht bei jedem Sample umzuschalten, hat das Ein-/
Ausschalten eine **Hysterese**: EIN ab `speed_warning_mm_s`, AUS erst wieder
20 % darunter (Totband 20–25 mm/s beim Default). Die Charakteristik wird nur
bei einem tatsächlichen Zustandswechsel beschrieben, nicht bei jedem
Sample, und bei Durchlaufende immer auf `0` zurückgesetzt (auch wenn der
Durchlauf durch einen Fehler abbricht). Der Schreibvorgang ist bewusst
*fail-soft*: anders als der Print-Mode-Wechsel darf
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

Der physische Düsenabstand (`NOZZLE_PITCH_MM`, ca. 0,087 mm) ändert sich dadurch
**nicht** — nur die kleinste noch einzeln ansprechbare vertikale Einheit wird
doppelt so groß: aus ca. 0,087 mm pro Düse werden bei `--nozzle-group 2`
ca. 0,174 mm pro adressierbarer Einheit.

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

Grafische Oberfläche im Browser, gebaut um die zwei Dinge, für die die Anlage
benutzt wird: **Bilder drucken** und **die Messreihe aus `TESTS.md` fahren**.

```bash
pip install -r requirements-ui.txt
python -m printhead.ui            # öffnet http://127.0.0.1:8000 im Browser
```

Zwei Anzeigen sind **immer** sichtbar, egal was man gerade tut:

- **Live-Position** mit denselben Größen, die `--verbose` ausgibt: rohes
  x/y/z, Seiten-u/v, Zeile/Spalte, Gier/Roll/Nick — und während eines
  Durchgangs zusätzlich Geschwindigkeit und Deckung mit Fortschrittsbalken.
  Bleiben die Werte aus, färben sie sich nach zwei Sekunden grau und die
  Quelle springt auf „veraltet", statt eine tote Zahl weiter anzuzeigen.
- **Druckvorschau**, die sich nach jeder Änderung an einem Feld neu rendert
  (entprellt, damit nicht jede Tasteneingabe einen Unterprozess startet).
- **Deckung (live)** — während eines Durchgangs wächst hier mit, was
  tatsächlich schon Tinte bekommen hat. Das Zielbild liegt blass darunter,
  also ist auf einen Blick sichtbar, was noch **fehlt**. Klick schaltet auf
  1:1-Pixel um (die Seitenspalte verkleinert ein 2299 Spalten breites Ziel
  sonst 6-fach, wobei einzelne Spaltenstriche untergehen).

  **Wo der Druckkopf gerade steht**, zeigt eine rote Linie: die Düsenleiste
  in ihrer aktuellen Gierlage, mit einem Punkt am Ende von Düse 0, damit die
  Leiste eine erkennbare Richtung hat. Ohne die sieht man zwar, was schon
  Tinte hat, aber nicht, wo man sich befindet. Ist der Kopf **komplett
  außerhalb** des Druckbilds, entfällt die Linie und stattdessen sitzt ein
  oranger Punkt am Bildrand in seiner Richtung — man weiß dann, wohin
  zurückzufahren ist. Die Unterscheidung läuft über die Balkenmitte: ragt
  bei Schräglage ein Ende ins Bild, bleibt die Linie, weil sie dann die
  bessere Auskunft ist.

  Die beiden Endpunkte kommen fertig aus `controller._coverage_event`, mit
  derselben Formel gerechnet (`coverage.bar_offset_uv`), mit der
  `CoverageEngine.step()` jede einzelne Düse platziert — die Linie liegt
  also da, wo auch wirklich Tinte landet. Bewusst nicht im Browser
  nachgerechnet: das wären eine zweite Kopie der Formel plus Kopien von
  `NOZZLE_PITCH_MM`/`NOZZLE_BAR_SPAN_MM`/`NUM_NOZZLES`, die beim nächsten
  Neuvermessen der Leiste still auseinanderlaufen würden.

  Ein Klotz-Pixel = eine Zelle des Zielbilds, dieselbe Konvention wie
  `record.png`. Reißt die WebSocket-Verbindung mitten im Durchgang, fehlen
  die in dieser Zeit gedruckten Zellen dauerhaft — es gibt keine
  Nachlieferung. Das Panel sagt das dann auch; maßgeblich ist in dem Fall
  **Deckung (letzter Durchgang)** darunter, das am Pass-Ende aus
  `record.png` kommt und zusätzlich MISSED und die Fahrspur zeigt.

Vier Reiter: **Drucken** (Bild, Testmuster oder Text, mit Größen und den
Ablauf-Schaltern sofort starten / ein Durchgang / Trockenlauf), **Tests** (die
Protokolle aus `TESTS.md` als Ein-Klick-Aktionen, jeweils mit der Nummer des
Tests und einem Satz dazu, was er misst), **Kalibrierung** (beide Blattkanten
abfahren, Boresight erfassen, berechnen, speichern) und **Einstellungen**
(Modus, Seitenrahmen, Dosierung, Glättung, Spray, Latenzkompensation).

Der gebaute Befehl steht immer im Klartext unter den Druckknöpfen — die UI
führt echte `main.py`-Unterprozesse aus und kann deshalb nicht davon
abweichen, was die CLI tut.

### Druckansicht: `/view`

Kopfzeilen-Knopf **„Druckansicht ↗"** öffnet `/view` in einem eigenen
Tab/Fenster — eine schlanke, reine Beobachtungsseite ohne Druck-Formular,
Konsole oder Kalibrierung, gedacht dafür, sie neben (oder auf einem zweiten
Bildschirm über) der Anlage offen zu lassen, während ein Durchgang läuft:

- **Hauptfokus die Druckpreview** — dieselbe live wachsende Deckungsansicht
  wie auf der Steuerseite (Zielbild blass darunter, rote Kopflinie/orange
  Randmarke), nur deutlich größer statt in einer schmalen Seitenspalte.
- **Position** — dieselben Felder wie im Live-Positions-Panel der
  Steuerseite (x/y/z, Seite u/v, Spalte/Zeile, Gier, Geschwindigkeit).
- **Deckung mit Prozent** — die Steuerseite zeigt „7483 / 9939 Pixel" plus
  Balken; hier steht zusätzlich eine große Prozentzahl davor, auf einen
  Blick aus der Entfernung lesbar, ohne die beiden Zahlen erst dividieren
  zu müssen.

Läuft über **denselben** `/ws`, den auch die Steuerseite benutzt — der Hub
sendet an alle verbundenen Clients dasselbe, ganz ohne Extra-Serverlogik für
ein zweites Fenster (`server.py`'s `Hub.broadcast`). Die Canvas-Zeichenlogik
(`covStart`/`covCells`/`covHead`) liegt seit dieser Seite in einer geteilten
`coverage_view.js` statt zweimal in beiden HTML-Dateien — dieselbe
Begründung wie für die serverseitige `bar`-Berechnung weiter oben: zwei
Kopien derselben Skalierungs-/Geometrierechnung würden beim nächsten Umbau
still auseinanderlaufen, und hier geht es nicht nur um Optik, sondern um die
Stelle, an der die Kopfmarke dem Bediener das Papier zeigt.

**Mitten im Durchgang geöffnet oder neu verbunden?** Der Hub merkt sich den
letzten `coverage_start` einer noch laufenden Aktion und schickt ihn beim
Verbinden sofort nach (`replay: true`), statt das Fenster bis zum
NÄCHSTEN Durchgang leer zu lassen — bei einem einzelnen `--once`-Lauf käme
der nie. Zellen, die vor dieser Verbindung schon gedruckt wurden, fehlen auf
dieser einen Leinwand trotzdem für immer (der Hub puffert sie nicht,
ein Druck kann Millionen Pixel haben) — die Ansicht sagt das dann auch,
statt eine augenscheinlich vollständige, aber lückenhafte Deckung zu zeigen.
Dieselbe Reparatur kommt der Steuerseite selbst zugute: ein Neuladen
mitten im Durchgang zeigte vorher ebenfalls nur eine leere Leinwand bis zum
nächsten Pass.

### ⚠️ Behoben: die Buchführung pro Sample war das eigentliche Tempolimit

Die UI fährt jeden Druck mit `--progress-json`. Dieser Stream schickte ein
Ereignis **pro Poll-Sample** (bis zu 500/s), und dahinter liefen mehrere
Vollbild-numpy-Durchläufe. Gemessen am Beispiel aus diesem README
(`--pattern-length-mm 200 --pattern-height-mm 100` = 2299×1152 = 2,65 M Pixel),
gegen ein Budget von 2000 µs je Sample:

| Operation | Kosten/Sample | lief |
|---|---|---|
| `coverage.done` → `np.all(printed[ink])` | 1279 µs | **immer**, auch ohne `--progress-json` |
| `(ink & fired).sum()` | 1536 µs | nur `--progress-json` |
| `ink.sum()` | 977 µs | nur `--progress-json` — eine **Konstante** |
| `fired & ~prev` + `fired.copy()` | 723 µs | nur `--progress-json` |

Weil die Spalten-Kante des Tintenmodells `--mm-per-column × --poll-hz` ist,
war das kein reines CPU-Thema, sondern ein **Tempolimit auf dem Druck**:

| | erreicht | Spalten-Kante |
|---|---|---|
| Soll | 500 Hz | 43,5 mm/s |
| vorher, ohne `--progress-json` | ~208 Hz | 18,1 mm/s |
| **vorher, mit `--progress-json` (= die UI)** | **~71 Hz** | **6,2 mm/s** |
| **jetzt** | **~330 Hz** | **28,7 mm/s** |

Behoben, indem jede dieser Größen inkrementell mitgeführt wird statt pro
Sample neu aus dem Vollbild berechnet: `printed` wird ausschließlich in
`_deposit` gesetzt, `fired` ausschließlich in `step()`, also sind exakte
Zähler an genau diesen Stellen möglich (`CoverageEngine.ink_total` /
`ink_fired` / `ink_printed`, und `done` als Zählervergleich). Neu gefeuerte
Zellen schreibt die Engine gleich mit (`drain_new_cells()`), womit der
Masken-Diff ganz entfällt.

Zusätzlich geht das `coverage`-Ereignis nur noch mit `--progress-hz`
(Default 25) raus statt pro Sample. **Dabei geht kein Tropfen verloren:**
zwischen zwei Ereignissen gesammelte Zellen kommen im nächsten mit, und ein
zwingender Flush am Pass-Ende trägt den Rest — auch beim Abbruch per
STARTPOINT-Taster oder SIGINT. Gegengeprüft an einem echten Durchlauf:
130.720 gemeldete Zellen gegen 130.720 gezählte bedeckte Pixel.

Die verbleibende Lücke zu 500 Hz ist der nicht deadline-korrigierte
`sleep` am Schleifenende (die Periode ist immer `Arbeit + 2 ms`) plus
`step()` selbst. Beides ist eine eigene Baustelle.

**Sensor-Übergabe:** Der Amfitrack ist ein einzelnes USB-Gerät und lässt sich
nicht zweimal öffnen. Startet man eine Aktion, während der Leerlauf-Strom
läuft, tritt dieser automatisch ab und kommt danach von selbst zurück;
währenddessen speist der Durchgang selbst die Live-Anzeige. Ein ausdrücklich
gestoppter Strom wird **nicht** wieder aufgeweckt.

Oben rechts liegen **Aktion stoppen** und **Herunterfahren** — Letzteres
beendet laufende Aktion und Sensorstrom sauber (SIGINT, damit der Druckkopf
noch geleert und der Tracker geschlossen wird) und fährt dann den Server
herunter.

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

> Im **Seiten-Modus** (`--mode page`) hat derselbe Taster eine andere, dort
> passendere Bedeutung – siehe „Startpoint-Taster im Seiten-Modus" weiter oben.

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
| `--verbose` | Bei `--mode line`/`page`: druckt eine live überschreibende Statuszeile (Position, bei `page` zusätzlich `page u/v`, Gierwinkel/Roll/Pitch, `covered N/M`) **während des laufenden Drucks** — das `--pos`-Äquivalent, aber nutzbar im echten Druck, da `--pos` selbst einer der eigenständigen Diagnose-Checks ist und sich nicht mit einem echten Druck kombinieren lässt (siehe unten). Wird bei `--progress-json` unterdrückt, damit dieser Stream reines NDJSON bleibt. Bei `--mode time` unverändert: loggt jeden 50. Spaltenschreibvorgang. |
| `--latency-compensate-s S` | Seiten-Modus: extrapoliert nur die an die Coverage-Engine übergebene Position um `S` Sekunden entlang der gemessenen Geschwindigkeit nach vorn, gegen die gemessene BLE/Firmware-Pipeline-Verzögerung (~13ms typisch). Default `0.0` = aus. Siehe Abschnitt „Latenz-Kompensation" oben. |

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
| `precision-check` | Linien **parallel zur Düsenleiste** mit **verdoppelnden** Abständen entlang der Fahrtrichtung – Auflösungstest: ab welchem Abstand verschmieren zwei Linien zu einer? Siehe eigenen Abschnitt unten |
| `drill_pattern` | Rastert eine externe Bilddatei (z. B. ein Bohr-/Fadenkreuz-Justiermuster) auf die gewünschte physische Größe, statt ein Muster zu berechnen – siehe `--pattern-image` unten |
| `ruler` | 1/10mm-Maßband: durchgehende Grundlinie, alle 10mm ein langer Strich (20mm), jeden Millimeter ein kurzer Strich (6mm). Anders als `--calibrate` (konfigurierbarer Strichabstand/-länge) ist hier nur `--pattern-length-mm` einstellbar – siehe eigenen Abschnitt unten |

```bash
python main.py --pattern checkerboard --pattern-square-mm 10 --pattern-square-rows 20
python main.py --pattern diagonal --mode line --preview diag.png
```

| Option | Bedeutung |
|---|---|
| `--pattern-length-mm` | Physische Länge des Musters in mm (Default 200) |
| `--pattern-square-mm` | Kachel-/Streifenbreite in mm (checkerboard, v-stripes, diagonal-Periode) |
| `--pattern-square-rows` | Kachel-/Streifenhöhe in Zeilen (checkerboard, h-stripes) — Achtung Seitenverhältnis, siehe `--pattern-square-height-mm` im `--mode page`-Abschnitt oben |
| `--pattern-line-cols` | Liniendicke in Spalten (`precision-check`, Default 1) |
| `--pattern-gap-start` | Erster Abstand in Spalten, verdoppelt sich danach (`precision-check`, Default 1) |
| `--pattern-image PATH` | Bilddatei für `--pattern drill_pattern` (jedes von PIL lesbare Format: PNG, JPG, BMP, …) |

#### `precision-check`: ab welchem Abstand trennen sich zwei Linien noch?

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
dazu ist jede Linie ein Timing-/Positionsereignis, also genau die Achse, auf
der Positions-Nachlauf und Dosier-Intervall wirken. Erst diese Ausrichtung
belastet das Tracking wirklich.

**Auswertung:** Vom engen Ende her schauen und den ersten Abstand suchen, der
noch als Weiß durchkommt. Dieser Abstand ist die praktische Auflösung des
**gesamten** Systems **entlang der Fahrtrichtung** bei der gefahrenen
Geschwindigkeit — Tracking-Genauigkeit, Dosier-Timing und Tintenausbreitung
zusammen. Diese Kombination liefert keine Einzelmessung; deshalb ist das
Muster ein Ergänzungswerkzeug zu `--straightness` (das nur die Tracking-Seite
isoliert betrachtet) und nicht dessen Ersatz.

Beide Parameter zählen **Spalten, nicht Millimeter** — entlang der
Fahrtrichtung ist das Raster auf `--mm-per-column` quantisiert (Default
0,2 mm), und darauf landet das Ergebnis. Damit sich das Gedruckte trotzdem mit
einem Lineal nachmessen lässt, gibt die CLI beim Erzeugen eine Tabelle mit
beiden Einheiten aus (hier `--pattern-length-mm 60`):

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

Die mm-Spalte skaliert mit `--mm-per-column`/`--dpi` mit. Die Tabelle
erscheint auch bei `--dry-run`/`--preview`, also bevor Tinte fließt. Passt bei
der gewählten Länge keine einzige Linie mehr, sagt sie das ausdrücklich, statt
still ein leeres Muster zu drucken. Eine Linie wird nie angeschnitten: passt
die letzte nicht mehr vollständig, entfällt sie — eine halb gedruckte Linie
sähe wie eine dünnere aus und würde als Auflösungsergebnis fehlgedeutet.

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

⚠️ **Fehler behoben: `--mm-per-column` wurde komplett ignoriert.**
`build_tracking()` in `cli.py` hat `TrackingSettings` gebaut, ohne
`mm_per_column` überhaupt zu übergeben — dadurch griff immer der eigene
Dataclass-Default (`0.2`), egal was auf der Kommandozeile stand. Nur `--dpi`
hatte je einen echten Effekt (über `resolve_mm_per_column`). Direkt bestätigt:

```bash
python main.py --pattern checkerboard --pattern-length-mm 200 --mm-per-column 0.1 ...
# vorher: "-> 1000 columns x 2000 rows"   (0.2mm/Spalte, der ignorierte Default)
# jetzt:  "-> 2000 columns x 2000 rows"   (0.1mm/Spalte, wie angefordert)
```

Genau das war die Ursache, wenn ein eigentlich quadratisch gedachtes
Schachbrett (`--pattern-square-mm` == `--pattern-square-height-mm`,
`--mm-per-column` == `NOZZLE_PITCH_MM`, damals 0,1 mm — seither auf ca.
0,087 mm neu vermessen) in `coverage.png` trotzdem
doppelt so hoch wie breit aussah: jede Spalte war heimlich doppelt so breit
wie angefordert. Betrifft **jeden** Aufruf mit `--mm-per-column ≠ 0.2` —
Musterbreite, Coverage-Engine-Spaltenadressierung, alles, was
`tracking.mm_per_column` liest.

**Neue Tests** (`tests/test_patterns_and_mapping.py`):

```
test_cli_mm_per_column_reaches_build_tracking
test_cli_mm_per_column_default_still_matches_the_dataclass_default
test_cli_dpi_still_overrides_mm_per_column
test_cli_mm_per_column_MUTATION_check_omitting_it_reintroduces_the_bug
```

Mutationsgeprüft gegen die reale, alte Konstruktion (nicht nur eine
Nachbildung im Test).

#### `ruler`: 1/10mm-Maßband

```bash
python main.py --pattern ruler --pattern-length-mm 100 --mode line --preview lineal.png
```

Druckt eine durchgehende Grundlinie mit festem Strichraster: alle 10mm ein
langer Strich (20mm), jeden Millimeter ein kurzer Strich (6mm) — quer zur
Grundlinie gemessen, wie bei `--calibrate`s Lineal oben. Anders als dort ist
hier nichts weiter einstellbar: kein `--calib-major-mm`/`--calib-minor-mm`-
Äquivalent, nur `--pattern-length-mm`. Vorteil gegenüber `--calibrate`: läuft
durch dieselbe `--pattern`-Pipeline wie jedes andere Preset, also auch mit
`--mode page`, `--record` oder `--dry-run`/`--preview`, statt an das eigene
`--calibrate`-Flag gebunden zu sein.

Die Strichlänge ist quer zur Grundlinie gemessen und wird auf die
verfügbaren Zeilen begrenzt: im `--mode line`/`time` (152 Düsen, ~13,1mm
Leistenspannweite) passt ein 20mm-Strich nicht hinein und wird auf volle
Leistenhöhe begrenzt — genau wie beim `--calibrate`-Lineal, dessen langer
Strich aus demselben physischen Grund immer volle Höhe hat. Erst im
`--mode page` mit `--pattern-height-mm` über ~13,1mm hinaus erscheinen
20mm/6mm in tatsächlicher Länge.

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
| `--calibration-check` | Kalibrierungs-Gesundheitscheck: Wagen flach über die Seite schieben, **ohne zu drehen** — misst, wie stark der Gierwinkel trotzdem driftet. Braucht `--page-calibration PATH` oder `--page-frame simple`. Ctrl+C beendet und druckt eine Zusammenfassung. Details unten. |
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

### `--straightness`: Tracking-Präzision am Lineal messen (offline)

Auswertung eines mit `--mode page --profile-csv` aufgezeichneten Laufs, bei
dem der Wagen an einer **geraden Kante (Lineal)** entlanggefahren wurde. Alle
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

Ausgegeben wird:

- **Linienwinkel** und Streckenlänge (Plausibilitätscheck, ob überhaupt genug
  gefahren wurde — unter 50 mm gibt es bewusst kein Urteil, weil über so
  wenig Weg fast alles gerade ist),
- **Abweichung** senkrecht zur Ausgleichsgeraden: RMS / p95 / max, jeweils
  zusätzlich **in Düsenreihen** (0,0868 mm) — das ist die Einheit, die
  entscheidet, ob eine Abweichung im Druck überhaupt sichtbar werden kann,
- **Aufteilung systematisch ↔ zufällig**: ein glatter Bogen (quadratischer
  Fit) gegen den Rest. 0,3 mm gleichmäßiger Verzug ist ein völlig anderes
  Problem als 0,3 mm Zittern — Ersteres mittelt sich nicht weg und ist
  typisch für Feldverzerrung, Letzteres dämpft `--smooth-ms`,
- **Abweichung nach Position** entlang der Linie (Bins mit Mittelwert / RMS /
  max) — beantwortet direkt „wo genau ist es krumm",
- **Wagen-Drehung und ihr Hebelarm-Effekt** (siehe Warnung unten).

**Warum die Gerade per Total-Least-Squares gefittet wird, nicht per
`v = m·u + c`:** Zum einen ist der Fehler zweidimensional — der Tracker ist
in beide Seitenachsen gleichermaßen ungenau —, also muss der **senkrechte**
Abstand minimiert werden, nicht der vertikale. Zum anderen läuft die
gewöhnliche Regression bei einer senkrechten Linie (unendliche Steigung) ins
Leere; ein Lauf überwiegend entlang `v` ist aber völlig normal und darf
keinen Sonderfall brauchen. TLS hat gar keine bevorzugte Achse.

⚠️ **Der mit Abstand größte Störterm ist die Wagen-Drehung, nicht der
Tracker.** Die geloggten `u_mm`/`v_mm` sind **düsenleisten-bezogen**:
`PageMapper` addiert den festen Sensor→Düsenleisten-Versatz
(`SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM`, 62,36 mm) *gedreht um den aktuellen
Gierwinkel*. Eine Drehung um **1° verschiebt den geloggten Punkt damit um
1,09 mm**, während der Sensor völlig stillsteht. Eine Hand, die beim
Entlangfahren leicht mitdreht, erzeugt also Millimeter an scheinbarer
Abweichung, die kein Tracking-Fehler sind. Genau dafür loggt
`PassProfiler.record_page_sample` das rohe Quaternion mit — dieses Tool ist
der Offline-Leser, der diese Spalten endlich auswertet: es meldet die
Drehspanne, die daraus rechnerisch folgende scheinbare Abweichung und die
Korrelation zwischen beiden. Ist die Korrelation hoch, ist die Drehung die
Ursache, nicht das Tracking.

Die Drehung wird als **3D-Gesamtwinkel** gegen das erste Sample gemessen, ist
also eine **Obergrenze**: Roll und Pitch stecken mit drin, schwenken die
Düsenleiste aber nicht so über die Seite wie der Gierwinkel. Für die saubere
Zerlegung bräuchte es `e_col`/`e_row`/Boresight aus der Kalibrierung, die ein
CSV allein nicht enthält.

⚠️ **Der Zahlenwert ist grundsätzlich eine OBERGRENZE für den
Tracking-Fehler.** Vier Dinge addieren sich darin und lassen sich aus dem CSV
allein nicht trennen: echter Tracker-Fehler (Rauschen + Feldverzerrung), die
Handführung (Wagen nicht durchgehend bündig am Lineal), die Geradheit des
Lineals selbst, und die eben beschriebene Wagen-Drehung. Ein guter Wert
beweist also gutes Tracking; ein schlechter Wert beweist noch nicht, dass der
Tracker schuld ist.

Ein `--mode line`-CSV wird bewusst mit klarer Meldung abgewiesen: es enthält
nur einen 1D-Spaltenindex und eine Vorschubstrecke, also gar keine zweite
Seitenachse, gegen die sich Geradheit prüfen ließe.

### `--calibration-check`: Gierwinkel-Drift bei reiner Verschiebung messen

Das gemeldete Symptom war ein driftender Gierwinkel, obwohl der Wagen nur
verschoben, nie gedreht wurde. `--calibration-check` macht genau das messbar:
Live-Stream wie `--pos` (identische `position`-NDJSON-Events — die Web-UI
kann sie unverändert weiterverwenden, ohne eigene Behandlung dieses neuen
Diagnosemodus), dazu am Ende (Ctrl+C) eine Zusammenfassung:

- verfahrene Strecke in `u`/`v` (mm) — Plausibilitätscheck, ob überhaupt
  genug bewegt wurde,
- Gierwinkel min/max/**Spanne** (°) — die Kopfzahl: ohne Drehung sollte das
  nahe 0 bleiben,
- Roll-/Pitch-Spanne (°) — die Größe, die über eine unsaubere Seitennormale
  in den Gierwinkel durchsickert (siehe „Kalibrierungsqualität" oben),
- **Korrelation** des Gierwinkels mit `u` bzw. `v` getrennt — trennt
  gewöhnliches Rauschen von **systematischer** Drift mit der Position: auf
  echten Daten dieser Anlage korrelierte die gemessene Neigung mit **+0,69**
  gegen `v`, bei nachweislich flachem Wagen.

Verdikt-Schwellen: Spanne bis **~2°** = unauffällig, bis **~4°** (nahe an
den 2-3°, die der Betreiber an seiner aktuellen Kalibrierung schon
akzeptiert) = grenzwertig, darüber = echtes Problem — entweder eine
schlechte Kalibrierungs-Seitennormale oder eine Feldverzerrung des
Trackers. Unterscheidung: dieselbe Stelle mit einer frisch, sorgfältig neu
abgefahrenen Kalibrierung wiederholen (verschwindet die Drift → war es die
Kalibrierung); bleibt sie trotz einer nachweislich guten Kalibrierung
bestehen, denselben Sweep an einer **anderen** Position/Höhe über der
Basisstation wiederholen — wandert die Drift mit der absoluten
Trackerposition statt mit der Kalibrierung, ist es Feldverzerrung, die
kein Neu-Kalibrieren beheben kann.

```bash
python main.py --calibration-check --page-calibration page_calibration.json
python main.py --calibration-check --page-frame simple --simulate   # ohne Hardware
```

**Beispielausgabe eines simulierten Laufs** (Boustrophedon-Sweep über eine
A4-große Fläche, Wagen dabei durchgehend flach — aber mit künstlich
injiziertem Gierwinkel proportional zu `v`, stellvertretend für eine
Seitennormale, deren Fehler positionsabhängige Neigung in scheinbaren
Gierwinkel verwandelt):

```
Calibration health check: slide the cart FLAT over the page, WITHOUT rotating it. Ctrl+C to stop and print the summary.
page u=  199.20  v=  210.42 mm  |  yaw= +5.60  roll= +0.00  pitch= +0.00 deg

---- calibration health check summary ----
  samples: 579
  travelled: u=199.2mm  v=280.3mm
  yaw: min=+0.00  max=+5.60  span=5.60 deg
  roll span: 0.00 deg   pitch span: 0.00 deg
  yaw correlation: vs u = -0.25  vs v = +1.00
  verdict: BAD: yaw span 5.60 deg is well beyond what a flat, non-rotating sweep should show. Likely either (a) a bad calibration page-normal (retrace the edges -- see calibration.py's CalibrationQualityWarning for whether the trace itself was short/noisy/sparse), or (b) tracker field distortion (a real physical effect, independent of calibration). To tell them apart: re-run this check at the SAME physical spot with a freshly, carefully re-traced calibration -- if the drift disappears, it was the calibration; if it persists even with a known-good one, repeat the same sweep at a DIFFERENT position/height over the tracker base station -- a drift pattern that moves with absolute tracker position rather than with the calibration is field distortion, not something re-tracing can fix.
Stopped calibration check.
```

Und zum Vergleich derselbe Sweep ganz ohne injizierten Fehler (Wagen bleibt
die ganze Zeit exakt in derselben Orientierung — die Korrelation ist dann
`None`/„n/a", nicht 0: bei einer Gierwinkel-Reihe ohne jede Streuung ist ein
Korrelationskoeffizient mathematisch undefiniert, siehe
`_calibration_check_summary`s Docstring):

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

**Neue Tests:**

#### Nachtrag (Verifikation): kein Freispruch ohne Messung

Das Verdikt hing ausschließlich am Gierwinkel-Span. Ein Lauf, der **gar
nichts** gesammelt hat — Tracker liefert keine Pose, oder Strg+C kommt
sofort — meldete damit:

```
  samples: 0
  verdict: OK: yaw span 0.00 deg ... consistent with a good calibration.
```

Also ein Freispruch für genau die Frage, wegen der man das Werkzeug startet.
Dasselbe galt für ein 2-cm-Wackeln: Der Gierwinkel bleibt dabei nahe null,
weil sich der Wagen kaum bewegt hat, nicht weil die Kalibrierung gut ist.
`_calibration_check_summary` berechnete `u_travel_mm`/`v_travel_mm` bereits
und nannte sie im eigenen Docstring den „headline sanity check" — das
Verdikt hat sie nur nie ausgewertet.

Jetzt wird vor den Gierwinkel-Schwellen geprüft, ob überhaupt genug gemessen
wurde: mindestens **20 Samples** und **50 mm** Weg (Diagonale der u/v-
Bounding-Box). Darunter lautet das Verdikt `INCONCLUSIVE` mit dem
ausdrücklichen Zusatz, dass das **kein Bestehen** ist. Die Schwellen sind
bewusst dieselben Konstanten wie `MIN_TRACE_LENGTH_MM`/`MIN_SAMPLE_COUNT`
aus `calibration.py` (importiert, nicht kopiert) — es ist dieselbe Frage,
mit derselben Messreihe belegt.

Mutationsproben bestätigt: Entfernt man die Sample-Bedingung, die Weg-
Bedingung, oder lässt man den Guard alles verschlucken, schlägt jeweils ein
Test fehl. Die beiden oben abgedruckten Beispielläufe (579 bzw. 553 Samples
über ~200/280 mm) liegen weit über beiden Schwellen — ihre Verdikte sind
unverändert.

**Neue Tests:**

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
Spalte `t_s, column, advance_mm, write_latency_ms, speed_mm_s, x, y, z` für die
Offline-Analyse.

Im **Seiten-Modus** misst `--profile` **Spalten pro Sekunde**, nicht
Musterwechsel pro Sekunde: seit die Firmware jede empfangene Spalte genau
einmal feuert, ist eine Spalte ein Tintentropfen, ein „Musterwechsel"
dagegen nur eine Abtastung, bei der etwas fällig war. Verglichen wird gegen
`--ble-write-ceiling × Spalten pro Write` (Default 270 × 12 bei MTU 247 ≈
3200 Spalten/s). Wird die Decke überschritten, geht **Tinte verloren**, nicht
bloß Aktualität — `PatternSender` verwirft dann die ältesten Spalten und
zählt sie mit. Die CSV-Spalte heißt entsprechend `cols_per_s` (früher
`writes_per_s`).

**Rohe Sensorposition (`x,y,z`), in beiden Modi.** Bis dahin protokollierte
kein Modus eine absolute Position: im Seiten-Modus stehen mit `u_mm`/`v_mm`
nur Seitenebenen-Koordinaten (Kalibrierung, Düsenversatz und Gierwinkel sind
eingerechnet), im Line-Modus mit `advance_mm` nur ein 1-D-Vorschub. Wer die
Tracking-Rohdaten **während eines Drucks** brauchte, musste einen zweiten,
getrennten `--pos --pos-json`-Lauf fahren — was mit einem echten Druck gar
nicht kombinierbar ist.

Die Spalten heißen bewusst wie die NDJSON-Felder aus `--pos-json` (`x`, `y`,
`z`, drei Nachkommastellen), damit dieselbe Auswertung beide Quellen lesen
kann. Fehlt die Position, bleiben die Felder **leer**, nicht `0,0,0` — anders
als beim Quaternion wäre eine Null hier ein plausibler Messwert (direkt am
Sender-Ursprung) und würde als echte Angabe gelesen. Im Seiten-Modus stehen
sie **vor** der Quaternion-Gruppe, damit die Orientierung das Zeilenende
bleibt.

⚠️ Trotz der Rohwerte ersetzt die Profil-CSV `--pos --pos-json` **nicht** für
Rauschmessungen: geschrieben wird nur, wenn tatsächlich Spalten rausgehen
(Seiten-Modus nur bei fälliger Tinte und nicht-leerem Muster, Line-Modus nur
beim Spaltenwechsel). Sie ist damit keine gleichmäßige Zeitreihe. Im
Line-Modus kommt hinzu, dass ein ganzer Spalten-Batch in einem BLE-Vorgang
rausgeht und die Zeilen dieses Batches sich dieselbe Position teilen — die
Wiederholung ist die Bündelung, kein eingefrorenes Tracking.

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
prüfen — ist der Wagen dort nie vorbeigekommen, oder war er so schnell, dass
der Tracker diese Spalten nie abgetastet hat?

⚠️ **Fehler behoben: `coverage.png` zeigte senkrechte Streifen, obwohl der echte
Druck vollflächig war.** COVERED/MISSED wurden aus `printed` gezeichnet — das ist
aber die **Dosis-Abschluss**-Buchhaltung, nicht die tatsächlich gelandete Tinte.
Eine Düse feuert, sobald ihr Pixel gewollt ist; `printed` wird dagegen erst
gesetzt, wenn die Dosis voll ist. Unter dem damaligen Verweildauer-Modell
brauchte eine Spalte dafür **mindestens zwei Samples**, gemessen an den echten
Einstellungen der Anlage (`--mm-per-column 0.087`, `--poll-hz 500`):

| Wagen-Geschwindigkeit | Samples/Spalte | `printed` | tatsächlich gefeuert |
|---|---|---|---|
| 17,3 mm/s | 2,51 | 99,3 % | 99,3 % |
| 25 mm/s | 1,74 | **73,3 %** | 99,3 % |
| 30 mm/s | 1,45 | **44,2 %** | 99,3 % |

Unterhalb von zwei Samples pro Spalte schloss keine Dosis mehr ab, die Düse hatte
aber auf dem einen Sample gefeuert — das Papier war voll, das Bild zeigte Lücken.
`CoverageEngine.fired` hält jetzt zusätzlich fest, wo Tinte **physisch** gelandet
ist (gegen eine unabhängige Rekonstruktion der gesendeten BLE-Patterns bit-genau
geprüft), und COVERED/MISSED sowie die `Covered N/M`-Zeile stammen daraus.
`printed` behält seine Dosis-Rolle (steuert Nachfeuern und `coverage.done`).

Mit dem Tropfenmodell ist diese Schere weitgehend zu: eine überfahrene Spalte
bekommt ihre volle Dosis bei jedem Tempo, das der Tracker noch abtasten kann
(siehe Dosierungs-Abschnitt), und jenseits davon fallen `fired` und `printed`
gemeinsam. Das **THIN**-Panel (Tinte da, Dosis unvollständig) bleibt trotzdem —
es zeigt jetzt echte Teil-Dosierung, im Wesentlichen die letzte Spalte vor einem
Richtungswechsel oder dem Seitenrand — und erscheint nur, wenn es nicht leer
ist. Es bedeutet „das kam hell heraus", nicht „Stelle verpasst".

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
| Mode | `f5ad7c1f-f6e1-4dd7-bbb7-d8b9286a88c6` (Read/Write, 1 Byte 0=line/1=page) |
| Speed warning | `58c05253-945f-48fc-a26c-989c785d6678` (Read/Write, 1 Byte 0/1) |
| Process stop | `a2e1c9d4-7f3b-4a8e-9c1d-5b6f8e2a0d47` (Write, 1 Byte, nur `1` gültig) — siehe „START-Taster: manchmal zweimal drücken nötig behoben" oben |

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

## Messreihe an der Hardware

Ausführbare Testprotokolle für die acht Eigenschaften der Anlage (Kantenqualität,
Tracker-Genauigkeit über die Entfernung, Auflösung, Bildqualität,
Wiederholbarkeit, Rechtwinkligkeit, Geschwindigkeitslimit, Blattausrichtung)
stehen in **[`TESTS.md`](TESTS.md)** — jeweils mit Durchführung, auszufüllenden
Messtabellen und Bewertungskriterium. Vorangestellt ist ein Vorflug-Check (tote
Düsen, BLE-Grenzwerte, Versatz-Vorzeichen, Kalibrierungsgesundheit), ohne den
mehrere Tests etwas anderes messen als gedacht.

## Tests / Verifikation ohne Hardware

```bash
python tests/test_frames.py          # Protokoll-Äquivalenz der Frame-Erzeugung
python tests/test_batching.py        # Spalten-Batching (Bytestrom bleibt identisch)
python tests/test_straightness.py    # Geradheits-/Präzisionsauswertung (--straightness)
python main.py "Hi" --simulate --mode line --dry-run   # Positions-Loop
python -m printhead --help
```

Alle Tests am Stück:

```bash
for t in tests/test_*.py; do python "$t" >/dev/null && echo "PASS $t" || echo "FAIL $t"; done
```

## Abhängigkeiten

`bleak`, `pillow`, `numpy`, `amfiprot`, `amfiprot-amfitrack` (siehe `requirements.txt`).
