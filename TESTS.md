# Messreihe: acht Tests am Freihand-Druckkopf

Ausführbare Testprotokolle. Jeder Test beantwortet **eine** Frage, liefert eine
**Zahl** (nicht „sieht gut aus") und nennt, was das Ergebnis verfälschen kann.

Die Tests bauen aufeinander auf — die [empfohlene Reihenfolge](#empfohlene-reihenfolge)
steht am Ende. Wer nur einen einzelnen Test fährt, liest zumindest die drei
Grundsätze und den Vorflug-Check.

---

## Drei Grundsätze

### 1. `coverage.png` ist die Kontrollgruppe

Jeder Druck läuft mit `--record`. Das Bild zeigt, was der Client zu drucken
**glaubte**; das Papier zeigt, was **passiert ist**.

- Unterschied zwischen beiden → der Fehler liegt **hinter** der Coverage-Engine
  (BLE, Firmware, Mechanik, Tinte).
- In beiden gleich falsch → der Fehler liegt **davor** (Kalibrierung, Tracking,
  Muster).

Diese Trennung ist bei fast jedem Test die halbe Diagnose und kostet nichts.

> ⚠️ Die PATH-Panels zeigen die Sensorspur nur bei ausreichend hohem Ziel. Bei
> 45,5 mm Sensor-Düsen-Versatz liegt sie außerhalb einer 13 mm hohen Fläche. Für
> Wegverfolgung `--pattern-height-mm` groß wählen.

### 2. Jeder Druck mit `--profile --profile-csv`

Protokolliert `t_s, row, col, u_mm, v_mm, speed_mm_s, writes_per_s, qx..qw`.
Kostet nichts, ist für Test 7 zwingend und beantwortet nachträglich bei jedem
Test die Frage „wie schnell war ich an dieser Stelle eigentlich".

### 3. Protokollblatt

Auf **jedes** Blatt die tatsächlich benutzten Werte schreiben. Die Defaults haben
sich in diesem Projekt mehrfach geändert — ein Blatt ohne Einstellungen ist
später nicht mehr auswertbar.

| Einstellung | Default (Stand dieser Datei) | benutzt |
|---|---|---|
| `--mm-per-column` | 0.087 | |
| `--dose-hold-s` | 0.001 | |
| `--spray-radius-mm` | 0.15 | |
| `--spray-strength` | 0.5 | |
| `--poll-hz` | 500 | |
| `--smooth-ms` | 0 | |
| `--speed-warning-mm-s` | 25 | |
| `--mode` | page | |
| Kalibrierdatei | — | |
| Abstand zum Sender | — | |
| Datum / Blattnummer | — | |

Feste Anlagenwerte zum Nachrechnen: Düsenteilung **0,0868 mm**, Leistenhöhe
**13,2 mm** (152 Düsen), Sensor→Leistenmitte **−45,5 mm** (Zeile), **+5,5 mm**
(Spalte).

---

## Vorflug-Check (einmalig, vor der ganzen Reihe)

Ohne diese vier Punkte messen mehrere Tests etwas anderes als gedacht.

| # | Prüfung | Vorgehen | Warum |
|---|---|---|---|
| **V1** | Tote Düsen | `python main.py --nozzle-test` | Eine tote Düse erzeugt in Test 1 eine Dauer-Scharte und in Test 4 einen Streifen — beides würde sonst dem Tracking angelastet. |
| **V2** | BLE-Grenzwerte | `python main.py --ble-benchmark --mm-per-column 0.087` | Liefert Durchsatz, Round-Trip-Latenz (avg/p95/max) und die abgeleitete Maximalgeschwindigkeit. **Diese Zahlen sind bisher nirgends festgehalten** — sie sind die unabhängige Gegenprobe zu Test 7. |
| **V3** | Versatz-Vorzeichen | `--pos --page-calibration PATH`, dann die **Düsenleiste** (nicht den Sensor!) auf die Kalibrier-Ecke halten. `v` muss ≈ 0 sein. | ⚠️ Prüft den **tatsächlichen** Wert aus `geometry.py` (−45,5 / +5,5). Die README nennt an mehreren Stellen noch die alten 62,36 / 0,0 — nicht danach gehen. |
| **V4** | Kalibrierung gesund | `--calibration-check`, Wagen flach **ohne Drehung** ≥ 50 mm schieben, Ctrl+C | Gierwinkel-Spanne + Korrelation gegen u/v. Ist die schon > 4°, sind Test 6 und 8 nicht sinnvoll interpretierbar. Siehe README „`--calibration-check`". |

**V2-Ergebnis festhalten:**

| Größe | Wert |
|---|---|
| Durchsatz (Spalten/s) | |
| Latenz avg / p95 / max (ms) | |
| max. Kopfgeschwindigkeit (mm/s) | |

---

## Test 0: Wie genau misst dein Foto überhaupt?

**Muss vor jeder Fotomessung laufen — sonst misst du deine Kamera.**

Perspektive verzerrt Winkel: ein schräg fotografiertes Rechteck wird zum
allgemeinen Viereck, aus 90° werden leicht 85°. Genau diese Größenordnung wurde
am Schachbrett beobachtet — das ist hier also kein theoretisches Bedenken.

**Durchführung**

1. Bekannt rechtwinklige Referenz nehmen: Millimeterpapier oder ein mit einem
   normalen Bürodrucker gedrucktes Quadrat.
2. Mit **exakt dem Aufbau** fotografieren, der später für die Drucke gilt
   (gleicher Abstand, gleiche Kamera, gleiche Beleuchtung).
3. Mit `python funktionen/image_line_to_angle.py` Innenwinkel und beide
   Seitenlängen messen.

**Auswertung**

| Messgröße | Soll | gemessen | = Unsicherheit |
|---|---|---|---|
| Innenwinkel der Referenz | 90,00° | | |
| Seitenlänge 1 | | | |
| Seitenlänge 2 | | | |

**Kriterium:** Liest die Referenz schon 85–87°, ist die **Kamera** die
Fehlerquelle, nicht der Drucker. Dann erst den Aufbau korrigieren: Kameraachse
senkrecht aufs Blatt, weit weg mit Zoom (lange Brennweite minimiert
Perspektive), Blatt plan aufliegend.

> ⚠️ **Eigenheit des Werkzeugs:** `image_line_to_angle.py` faltet den Winkel auf
> **0–90°**. Ein angezeigter Wert von 85,6° kann ein echter 85,6°- **oder** ein
> 94,4°-Winkel sein. Ohne die Seitenlängen ist nicht entscheidbar, ob das Quadrat
> gestaucht oder gestreckt ist — bei Test 6 deshalb immer beide mitmessen.

**Alles unterhalb dieser Unsicherheit ist mit dem Foto nicht messbar** — dafür
Messschieber oder Mikroskop nehmen.

---

## Test 1: Kantenqualität

**Frage:** Wie scharf und gerade sind Kanten — und unterscheiden sich Kanten quer
zur Fahrtrichtung von Kanten längs dazu?

Der Kern ist der **Vergleich zweier Kantenarten**, weil sie völlig verschieden
entstehen:

| Kante | Muster | Wodurch bestimmt |
|---|---|---|
| **längs** zur Fahrt | `h-stripes` | rein geometrisch: welche Düse ist die letzte. **Kein Timing beteiligt.** |
| **quer** zur Fahrt | `v-stripes` | Position/Timing: wann schalten alle Düsen gemeinsam um. |

**Durchführung** — beide bei gleicher, möglichst gleichmäßiger Geschwindigkeit:

```bash
python main.py --pattern h-stripes --pattern-length-mm 80 \
    --mode page --page-calibration page_calibration.json \
    --record h.png --profile --profile-csv h.csv

python main.py --pattern v-stripes --pattern-square-mm 5 --pattern-length-mm 80 \
    --mode page --page-calibration page_calibration.json \
    --record v.png --profile --profile-csv v.csv
```

**Messung** (Mikroskop)

1. **Kantenrauheit:** an ~10 Punkten über 20 mm Kantenlänge die Lage der Kante
   messen, Gerade durchlegen, RMS-Abweichung bilden.
2. **Kantenschärfe:** Breite der Übergangszone von voller Tinte zu Weiß.

| Messgröße | h-stripes (längs) | v-stripes (quer) |
|---|---|---|
| Rauheit RMS (mm) | | |
| Rauheit max (mm) | | |
| Übergangsbreite (mm) | | |

**Auswertung — der eigentliche Trick**

- Rauheit `h-stripes` = **Bodenwert** der Anlage (Düsen, Tinte, Papier).
- Rauheit `v-stripes` = derselbe Bodenwert **plus** Tracking/Timing.
- **Die Differenz ist der Tracking-Anteil.**

Die Tintenausbreitung betrifft beide Kanten gleich und fällt in der Differenz
heraus — deshalb ist der Vergleich aussagekräftiger als jede Einzelmessung.

**Kriterium**

- Querkante deutlich rauer → Tracking/Timing dominiert.
- Beide etwa gleich → Tinte und Düsen begrenzen; am Tracking ist nichts zu holen.

---

## Test 2: Amfitrack-Genauigkeit über die Entfernung

**Kein Druck nötig** — reine Sensor-Charakterisierung.

### 2a) Rauschen über Entfernung (ohne Referenzmaß, deshalb zuerst)

1. Wagen **mechanisch fixieren** (kleben/klemmen) — nicht in der Hand halten,
   Handzittern würde alles überdecken.
2. Je Abstand `d` etwa 20 s aufzeichnen:
   ```bash
   python main.py --pos --pos-json > rausch_d20.jsonl
   ```
3. `d` = 10, 20, 30, 40, 50, 60 cm.

**Auswertung:** je Datei und Achse Standardabweichung und Spitze-Spitze aus den
`x`/`y`/`z`-Feldern.

| Abstand (cm) | σx (mm) | σy (mm) | σz (mm) | Spitze-Spitze max (mm) |
|---|---|---|---|---|
| 10 | | | | |
| 20 | | | | |
| 30 | | | | |
| 40 | | | | |
| 50 | | | | |
| 60 | | | | |

**Auswertung**

```bash
python funktionen/rauschen_entfernung.py rausch_d*.jsonl --png rauschen.png
```

Der Abstand wird aus dem Dateinamen gelesen (erste Zahl darin), sonst
`--abstaende 10,20,30,...` angeben. Ausgegeben werden die Tabelle oben, eine
Grafik Rauschen gegen Entfernung und der **interpolierte Grenzabstand**.

**Kriterium:** Ab wo überschreitet das Rauschen **eine Düsenreihe (0,087 mm)**?
Jenseits dieses Abstands begrenzt der Sensor die Druckqualität, unabhängig von
allem anderen. Das ist die härteste Zahl der ganzen Reihe — sie legt den
nutzbaren Arbeitsbereich fest.

> Das Werkzeug meldet zusätzlich **Drift**: liegt die Streuung innerhalb kurzer
> Fenster deutlich unter der Gesamtstreuung, läuft der Sensor langsam weg statt
> nur zu rauschen. Das ist ein anderer Fehler mit anderen Folgen — Rauschen
> mittelt sich über eine Dosis teilweise heraus, Drift nicht. Tritt Drift auf,
> zuerst prüfen, ob der Wagen wirklich fest saß.

### 2b) Maßstabsfehler über Entfernung (mit Referenzmaß)

1. Zwei Marken im mit dem Messschieber geprüften Abstand von **exakt 100 mm**.
2. Wagen bei Abstand `d` auf Marke A stellen, `--pos` einige Sekunden mitteln,
   notieren. Dasselbe auf Marke B.
3. Für jedes `d` aus 2a wiederholen.

| Abstand (cm) | gemeldete Differenz (mm) | Fehler (mm) | Fehler (%) |
|---|---|---|---|
| 10 | | | |
| … | | | |

Auswertbar im selben Werkzeug:

```bash
python funktionen/rauschen_entfernung.py rausch_d*.jsonl \
    --massstab 10=99.4,20=99.1,30=98.2 --referenz 100
```

**Kriterium:** Nutzbarer Arbeitsbereich = Rauschen < 0,087 mm **und**
Maßstabsfehler < 1 %. Wächst der Fehler mit `d`, ist es Feldverzerrung — die
lässt sich nicht wegkalibrieren, nur vermeiden (näher am Sender arbeiten).

### 2c) Optional: Verzerrung über das Volumen

Dieselben 100 mm bei **gleichem** Abstand, aber an verschiedenen Stellen des
Feldes (links/rechts/vorn/hinten). Trennt „Fehler wächst mit Entfernung" von
„Fehler hängt vom Ort ab" — Letzteres deutet auf Metall in der Nähe.

### 2d) Geradheit entlang einer Führung (mehrere Fahrten)

**Frage:** Wie stark weicht die gemessene Bahn quer zur Fahrtrichtung ab — und
wiederholt sich diese Abweichung?

Der Sensor wird an einem **geraden Balken** entlang der x-Achse geführt.
Ideal bliebe y konstant; jede Änderung ist Abweichung.

```bash
# Drei Fahrten über denselben Balken, Balken dazwischen NICHT bewegen
python main.py --pos --pos-json > fahrt1.jsonl
python main.py --pos --pos-json > fahrt2.jsonl
python main.py --pos --pos-json > fahrt3.jsonl

python funktionen/geradheit_messreihe.py fahrt1.jsonl fahrt2.jsonl fahrt3.jsonl \
    --png geradheit.png
```

Das Werkzeug legt eine **gemeinsame** Ausgleichsgerade durch alle Fahrten
(rechnet damit die Schiefstellung des Balkens heraus, die kein Trackerfehler
ist) und trennt:

| Größe | Bedeutung | gemessen |
|---|---|---|
| systematisch (RMS der Mittelwertkurve) | wiederholt sich → Feldverzerrung **oder** krummer Balken | |
| zufällig (Rauschen je Messwert) | Sensorrauschen | |
| Versatz je Fahrt | Wiederholbarkeit der Führung | |

**Warum mehrere Fahrten:** Eine einzelne Fahrt kann nicht unterscheiden, ob der
Tracker an einer Stelle dauerhaft schief misst oder ob es Rauschen war. Erst
der Vergleich mehrerer Fahrten über denselben Balken trennt das.

**Wenn „überwiegend systematisch" herauskommt:** Balken um **180° drehen** und
erneut messen. Wandert die Kurve mit, war es der Balken; bleibt sie liegen, der
Tracker. Das ist die entscheidende Gegenprobe — ohne sie ist nicht entschieden,
welches von beiden es war.

> ⚠️ **Nicht die Profil-CSV benutzen.** Deren `u_mm`/`v_mm` sind
> Seitenebenen-Koordinaten: Kalibrierung, Sensor-zu-Düsenleisten-Versatz und
> Gierwinkel-Drehung sind bereits eingerechnet, und geschrieben wird nur bei
> Musterwechseln. Für die reine Sensor-Präzision `--pos --pos-json` nehmen. Das
> Werkzeug liest eine Profil-CSV zwar, warnt dann aber ausdrücklich.

---

## Test 3: Auflösung/Präzision

**Frage:** Ab welchem Abstand trennen sich zwei Linien noch — und woran liegt die
Grenze?

```bash
python main.py --pattern precision-check --pattern-gap-start 1 \
    --pattern-line-cols 1 --pattern-length-mm 60 \
    --mode page --page-calibration page_calibration.json \
    --record pc.png --profile --profile-csv pc.csv
```

Die beim Erzeugen ausgegebene **Soll-Tabelle aufheben** — sie ordnet jeder Lücke
ihren Sollwert zu. Ohne sie ist der Ausdruck nicht auswertbar.

**Messung**

| Größe | Messmittel | gemessen |
|---|---|---|
| Index der ersten noch getrennten Linie | Mikroskop | |
| Breite **einer** gedruckten Linie (mm) | Mikroskop | |
| Abstand Linie 0 → 1 (mm) | Foto | |
| Abstand Linie 0 → 2 (mm) | Foto | |
| Abstand Linie 0 → 3 (mm) | Foto/Messschieber | |
| … bis zur letzten Linie | Messschieber | |

**Auswertung**

```bash
python funktionen/precision_check_auswertung.py --cli \
    --mm-per-column 0.087 --gap-start 1 --line-cols 1 \
    --gemessen 0,<L1>,<L2>,<L3>,... \
    --linienbreite <Breite> --erste-getrennte <Index>
```

Liefert Maßstabsfaktor, Linearitätsrest, Tintenausbreitung und die Aufteilung
**Tinte ↔ Tracking/Timing**.

**Bei 2–3 Geschwindigkeiten wiederholen** — die Auflösungsgrenze ist
geschwindigkeitsabhängig. Ein Wert ohne Geschwindigkeitsangabe ist wertlos.

**Nebenertrag:** Die gemessene Linienbreite ist die **erste echte
Tintenausbreitung dieser Anlage**. Die vorhandenen `--spray-*`-Defaults
(0,15 / 0,5) stammen ausschließlich aus Simulation — erst dieser Wert erlaubt es,
sie gegen die Realität zu prüfen.

---

## Test 4: Bild drucken und Qualität beurteilen

**Kein Foto als Testbild.** „Qualität beurteilen" braucht messbare Merkmale.

**Testbild bauen** (beliebiges Zeichenprogramm, als PNG) mit:

- **konzentrischen Kreisen** bekannten Durchmessers
- **Fadenkreuz** über die volle Bildbreite/-höhe
- **Linienpaaren** abnehmenden Abstands (Gegenprobe zu Test 3)
- **Flächen verschiedener Deckung**

```bash
python main.py --pattern drill_pattern --pattern-image testbild.png \
    --pattern-length-mm 100 --pattern-height-mm 100 \
    --mode page --page-calibration page_calibration.json \
    --record bild.png --profile --profile-csv bild.csv
```

**Messung und was sie verrät**

| Merkmal | Messmittel | Aussage | gemessen |
|---|---|---|---|
| Kreisdurchmesser | Messschieber | Maßstab absolut | |
| **Kreis-Rundheit** (größter − kleinster Durchmesser) | Messschieber | **Anisotropie** — ein Kreis wird zur Ellipse, wenn `--mm-per-column` gegenüber der Düsenteilung falsch steht. Fängt beide Achsen in *einem* Merkmal. | |
| Fadenkreuz-Winkel | `image_line_to_angle` | Rechtwinkligkeit (Gegenprobe Test 6) | |
| feinstes getrenntes Linienpaar | Mikroskop | Auflösung (Gegenprobe Test 3) | |
| Bänderung, Deckungsunterschiede | Auge | Dosierung/Nachschub | |

**Gegenprobe:** dieselben Merkmale in `bild.png` ausmessen. Was dort schon falsch
ist, liegt vor der Coverage-Engine.

---

## Test 5: Wiederholbarkeit

**Zwingende Voraussetzung:** feste `--page-calibration` benutzen, **nicht**
`--page-frame simple`. Simple nullt den Ursprung bei jedem START neu — dann misst
du die Wiederholbarkeit des Nullens, nicht die des Trackings.

### 5a) Zwei Durchgänge auf dasselbe Blatt (die genaueste Variante)

1. Muster mit gut getrennten Linien drucken:
   ```bash
   python main.py --pattern precision-check --pattern-gap-start 32 \
       --pattern-line-cols 1 --mode page \
       --page-calibration page_calibration.json --record w1.png
   ```
2. **Ohne irgendetwas zu bewegen** denselben Befehl erneut auf **dasselbe Blatt**
   laufen lassen.

**Messung** (Mikroskop): Breite derselben Linie vorher und nachher.

| Linie | Breite nach Durchgang 1 (mm) | Breite nach Durchgang 2 (mm) | Zuwachs (mm) |
|---|---|---|---|
| 1 | | | |
| 3 | | | |
| 5 | | | |

**Auswertung**

- Linie bleibt gleich breit → Wiederholbarkeit **besser als die
  Tintenausbreitung** aus Test 3, also besser als hier messbar.
- Linie wird um `X` breiter oder doppelt sich → **`X` ist die Wiederholbarkeit.**

Das Papier führt den Vergleich selbst durch — kein Ausmessen zweier Blätter, kein
gemeinsamer Bezugspunkt nötig. Deshalb ist das die genaueste Variante.

### 5b) Nach Unterbrechung

Wie 5a, aber zwischen den Durchgängen: BLE trennen und neu verbinden, Wagen
wegnehmen und zurückstellen, ggf. Firmware neu starten. Fängt Drift, den 5a nicht
sieht.

### 5c) Knapp daneben

Zweiter Durchgang mit **absichtlich um bekannten Betrag** versetztem Ursprung
(Startpunkt-Taster oder zweite Kalibrierdatei), z. B. 0,5 mm.

**Messen:** tatsächlicher Abstand der beiden Linien. Weicht er vom befohlenen
Versatz ab, ist der Fehler **systematisch**, nicht zufällig.

---

## Test 6: Rechtwinkligkeit der Schachbrett-Quadrate

**Test 0 und Vorflug V4 müssen gelaufen sein.**

```bash
python main.py --pattern checkerboard \
    --pattern-square-mm 10 --pattern-square-height-mm 10 \
    --pattern-length-mm 120 --pattern-height-mm 80 \
    --mode page --page-calibration page_calibration.json \
    --record schach.png --profile --profile-csv schach.csv
```

> `--pattern-square-mm` und `--pattern-square-height-mm` **gleich** setzen —
> sonst sind die Kacheln schon im Soll keine Quadrate. Eine Zeile ist 0,087 mm,
> `--pattern-square-rows` führt hier in die Irre.

**Messung:** Innenwinkel **mehrerer** Quadrate an verschiedenen Stellen — vier
Ecken **und** Mitte, nicht nur an einem. Je Quadrat **beide Seitenlängen**
mitmessen (wegen der 0–90°-Faltung, siehe Test 0).

| Position | Winkel (°) | Abw. von 90° | Seite quer (mm) | Seite längs (mm) |
|---|---|---|---|---|
| oben links | | | | |
| oben rechts | | | | |
| Mitte | | | | |
| unten links | | | | |
| unten rechts | | | | |

**Auswertung — die Ortsabhängigkeit ist die eigentliche Information**

| Befund | Bedeutung | Behebbar? |
|---|---|---|
| Winkelfehler überall **gleich** | Scherung/Verdrehung im Bezugssystem | **ja**, über Kalibrierung |
| Winkelfehler **variiert** mit dem Ort | Feldverzerrung, nichtlinear | **nein**, nur vermeiden |
| Seitenlängen richtungsabhängig verschieden | anisotroper Maßstab | **ja**, `--mm-per-column` |

**Zwei Gegenproben, die die Ursache eingrenzen**

1. Dieselben Winkel in `schach.png` messen — dort müssen sie **exakt 90°** sein.
   Sind sie es nicht, liegt der Fehler schon vor dem Druck.
2. Das in der README unter „`--calibration-check`" beschriebene Verfahren fahren:
   am **selben Ort** mit frisch und sorgfältig neu abgefahrener Kalibrierung
   (Drift weg → es war die Kalibrierung); bleibt er, denselben Sweep an einer
   **anderen Stelle** über der Basisstation (Drift folgt der absoluten
   Trackerposition → Feldverzerrung).

---

## Test 7: Geschwindigkeitslimit

### 7a) Der ganze Verlauf aus einem einzigen Druck (empfohlen)

```bash
python main.py --pattern solid --pattern-length-mm 200 \
    --mode page --page-calibration page_calibration.json \
    --record geschw.png --profile --profile-csv geschw.csv
```

Dabei **absichtlich von sehr langsam bis schnell beschleunigen** — links
kriechen, nach rechts hin immer schneller.

**Auswertung**

1. Auf dem Papier die Stelle `u` suchen, ab der die Deckung sichtbar einbricht.
2. Diese Stelle dem Werkzeug geben:

```bash
python funktionen/geschwindigkeit_profil.py geschw.csv --bei-u 137 --png profil.png
```

Es mittelt über ein Fenster um `u` (ein einzelner Messwert wäre zufälliger als
die gesuchte Größe, weil `speed_mm_s` selbst eine verrauschte Differenz ist) und
zeigt im Plot das ganze Geschwindigkeitsprofil samt der 25-mm/s-Warnschwelle —
damit siehst du auch, ob du überhaupt den nötigen Bereich abgedeckt hast.

Das ist die Grenzgeschwindigkeit — direkt abgelesen, ohne die Geschwindigkeit von
Hand konstant halten zu müssen (was ohnehin kaum gelingt).

| Größe | Wert |
|---|---|
| `u` beim Einbruch (mm) | |
| zugehörige `speed_mm_s` | |

### 7b) Gestufte Gegenprobe

3–5 getrennte Durchgänge mit bewusst unterschiedlichem Tempo.

| Durchgang | „Covered N/M" | Deckung (%) | mittlere `speed_mm_s` |
|---|---|---|---|
| sehr langsam | | | |
| langsam | | | |
| mittel | | | |
| schnell | | | |
| sehr schnell | | | |

```bash
python funktionen/geschwindigkeit_profil.py lauf*.csv --deckung 99,96,73,44
```

Die mittlere Geschwindigkeit je Lauf holt sich das Werkzeug aus der CSV; die
Deckung in Prozent gibst du in Reihenfolge der Dateien dazu. Ausgegeben wird die
interpolierte Geschwindigkeit, bei der die Deckung unter die Schwelle fällt.

**Kriterium:** Geschwindigkeit, bei der die Deckung unter ~95 % fällt.

**Drei Abgleiche, die diesen Test aussagekräftig machen**

1. **Gegen die Vorhersage:** dokumentiert sind 100 % bei ≤ 17,3 mm/s, 60 % bei
   25 mm/s, 14 % bei 35 mm/s (simuliert, `poll_hz=200`). Weicht die Messung stark
   ab, stimmt das Dosiermodell nicht — Stellschrauben sind `--dose-hold-s` und das
   Firmware-`PATTERN_STRIDE`, **die zusammen bewegt werden müssen** (siehe README
   „Firmware-Kopplung"; die Firmware muss dann neu geflasht werden).
2. **Gegen die BLE-Seite:** die Maximalgeschwindigkeit aus Vorflug V2. Liegt die
   gemessene Grenze darunter, begrenzt die **Dosierung**; liegt sie bei V2,
   begrenzt **BLE**.
3. **Gegen die Warnschwelle:** der Default steht bei 25 mm/s
   (`--speed-warning-mm-s`). Der Test zeigt, ob der richtig gesetzt ist.

---

## Test 8: Papier parallel zum Sender ausrichten

**Frage:** Verbessert es die Geometrie, wenn die Blattkanten parallel zu den
Senderachsen liegen?

Zweimal dasselbe Schachbrett wie in Test 6, **alles identisch außer der
Blattausrichtung**:

- **A** — Blattkanten parallel zum Sendergehäuse
- **B** — Blatt um ~45° gedreht

> ⚠️ **Entscheidend:** Für **jede** Ausrichtung die Seitenkalibrierung **neu
> abfahren**. Eine `page_calibration.json` beschreibt, wo *ein bestimmtes* Blatt
> liegt — die alte auf ein gedrehtes Blatt anzuwenden misst nur die falsche
> Kalibrierung. Abstand zum Sender und Geschwindigkeit gleich halten.

**Messung:** dieselben Winkel und Seitenlängen wie in Test 6, an denselben
Stellen. Zusätzlich je Ausrichtung ein `--calibration-check`-Sweep — dessen
Gierwinkel-Spanne ist ein zweiter, vom Druck unabhängiger Indikator.

| Position | A: Winkel (°) | B: Winkel (°) |
|---|---|---|
| oben links | | |
| Mitte | | |
| unten rechts | | |
| **Gierwinkel-Spanne (`--calibration-check`)** | | |

**Auswertung**

- **A deutlich besser als B** → der Fehler hängt von der Ausrichtung im Feld ab.
  Praktische Konsequenz: Blatt immer ausrichten. Zugleich ein starkes Indiz, dass
  **Feldgeometrie** und nicht die Mechanik hinter dem Rechtwinkligkeitsproblem
  steckt.
- **A ≈ B** → die Ausrichtung ist unerheblich; die Ursache liegt woanders
  (Verdrehung des Wagens während der Fahrt, Kalibrierverfahren, Kameramessung).

---

## Empfohlene Reihenfolge

Jeder Test liefert eine Zahl, die der nächste braucht:

| # | Test | Warum an dieser Stelle |
|---|---|---|
| 1 | **Vorflug V1–V4** | ohne tote Düsen, BLE-Grenzwerte und gesunde Kalibrierung ist alles Weitere fragwürdig |
| 2 | **Test 0** Foto-Kontrolle | Torwächter für jede Fotomessung |
| 3 | **Test 2a** Rauschen | ohne Druck, schnell; großes Rauschen erklärt alles Weitere |
| 4 | **Test 2b** Maßstab über Entfernung | legt den Arbeitsabstand für alle Drucke fest |
| 5 | **Test 3** Auflösung | liefert die Tintenausbreitung, die Test 1 und 5 zum Interpretieren brauchen |
| 6 | **Test 7** Geschwindigkeit | legt die Geschwindigkeit für alle folgenden Drucke fest |
| 7 | **Test 1** Kanten | braucht Ausbreitung (T3) und Geschwindigkeit (T7) |
| 8 | **Test 5** Wiederholbarkeit | braucht die Ausbreitung als Vergleichsmaßstab |
| 9 | **Test 6** Rechtwinkligkeit | braucht Test 0 und V4 |
| 10 | **Test 8** Ausrichtung | verfeinert Test 6 |
| 11 | **Test 4** Bilddruck | integrierender Abschlusstest |

---

## Was diese Tests nicht können

- **Jede Messung ist eine Summe.** Tracking, Dosier-Timing, Tintenausbreitung und
  die Genauigkeit der Messung selbst gehen immer gemeinsam ein. Ein guter Wert
  beweist gutes Tracking; ein schlechter beweist noch nicht, dass das Tracking
  schuld ist. Deshalb arbeiten Test 1 und Test 3 mit **Differenzen** — dort
  kürzen sich gemeinsame Anteile heraus.
- **Test 3 ist auf Faktor 2 genau.** Die Lücken verdoppeln sich, die
  Auflösungsgrenze ist damit eine Eingrenzung, keine Punktmessung. Für einen
  engeren Wert mit anderem `--pattern-gap-start` erneut drucken.
- **Ohne Scanner** ist die Fotomessung auf die in Test 0 ermittelte Unsicherheit
  begrenzt. Feine Größen (Kantenrauheit, kleinste Lücken, Linienbreite) gehören
  unters Mikroskop, große (Abstände über 20 mm) an den Messschieber.
