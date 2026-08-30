"""
UI: Live-Anzeige, Sensor-Übergabe und Testkatalog (kein Browser, keine Hardware).

Deckt ab, was beim UI-Neubau dazugekommen ist. Die reine Kalibrier-Logik und
der Coverage-Relay bleiben in tests/test_ui_calibration.py.

Drei Dinge, die hier festgenagelt werden:

  * **Die --verbose-Felder im coverage-Ereignis.** Die Live-Anzeige soll
    während eines Durchgangs dasselbe zeigen wie --verbose. --verbose selbst
    taugt dafür nicht: es beendet jede Zeile mit ``\\r`` statt ``\\n``, damit
    es sich im Terminal selbst überschreibt — ein zeilenweise lesender
    Konsument sieht also bis zum Passende gar nichts. Die Felder hängen
    deshalb am --progress-json-Ereignis, und dieser Test prüft, dass sie
    wirklich ankommen.

  * **Die Sensor-Übergabe.** Der Amfitrack ist ein einzelnes USB-Gerät und
    lässt sich nicht zweimal öffnen. Eine Aktion muss den Leerlauf-Strom
    also verdrängen und ihn danach zurückbringen — sonst wäre "die Position
    ist immer live" nach dem ersten Druck gebrochen.

  * **Der Testkatalog.** Er steht als Daten im Server statt als Knöpfe im
    HTML, damit UI und TESTS.md nicht auseinanderlaufen.

Aufruf:  python tests/test_ui_live.py
"""

import asyncio
import pathlib
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead.ui.server import TEST_ACTIONS, Hub                    # noqa: E402


class _FakeWS:
    """Sammelt, was der Hub gesendet hätte."""

    def __init__(self):
        self.messages = []

    async def accept(self):
        pass          # echte Registrierung braucht das (Hub.register), Tests bisher nicht

    async def send_json(self, msg):
        self.messages.append(msg)


async def _bis_fertig(hub, grenze=400):
    for _ in range(grenze):
        if hub.action is not None and not hub.action.running:
            return True
        await asyncio.sleep(0.05)
    return False


# ================================================= --verbose-Felder im Ereignis
def test_coverage_event_traegt_die_verbose_felder():
    """Die Live-Anzeige braucht während eines Durchgangs dieselben Größen wie
    --verbose. Ohne diese Felder bliebe die Anzeige im Druck halb leer."""
    async def run():
        hub = Hub()
        fake = _FakeWS()
        hub.clients.add(fake)
        r = await hub.run_action([
            "--pattern", "solid", "--pattern-length-mm", "4",
            "--pattern-height-mm", "4", "--mode", "page",
            "--page-frame", "simple", "--dry-run", "--simulate",
            "--progress-json", "--auto-start", "--once", "--timeout", "2",
        ])
        assert r["ok"], r
        assert await _bis_fertig(hub), "Aktion lief nicht zu Ende"
        return fake.messages

    messages = asyncio.run(run())
    coverage = [m for m in messages
                if m.get("type") == "coverage_event" and m.get("event") == "coverage"]
    assert coverage, [m.get("type") for m in messages]

    ereignis = coverage[0]
    for feld in ("u", "v", "row", "col", "x", "y", "z",
                 "yaw_deg", "roll_deg", "pitch_deg", "covered", "total"):
        assert feld in ereignis, (feld, ereignis)
    assert "speed_mm_s" in ereignis          # darf None sein, muss aber da sein
    assert ereignis["total"] > 0
    assert 0 <= ereignis["covered"] <= ereignis["total"]
    # Die rote Druckkopf-Linie der Live-Ansicht: zwei Endpunkte im
    # Pixelraster des Zielbilds.
    assert "bar" in ereignis, ereignis
    assert len(ereignis["bar"]) == 2 and len(ereignis["bar"][0]) == 2


def test_bar_endpunkte_treffen_die_echte_duesenplatzierung():
    """Die rote Linie muss dort liegen, wo auch Tinte landet.

    ``_coverage_event`` rechnet die Endpunkte mit ``bar_offset_uv`` --
    derselben Formel, mit der ``CoverageEngine.step()`` jede einzelne Düse
    platziert. Hier gegen genau diese Platzierung nachgerechnet, bei
    mehreren Gierwinkeln, statt nur die Formel gegen sich selbst zu
    prüfen. Ein in JavaScript nachgebauter Zweitrechenweg (der Grund,
    warum das serverseitig passiert) würde hier auffallen.
    """
    import math
    import numpy as np
    from printhead.controller import _coverage_event
    from printhead.geometry import NOZZLE_PITCH_MM, NUM_NOZZLES

    mm_per_column = 0.087
    u_mm, v_mm = 10.0, 5.0
    for yaw_deg in (0.0, 45.0, 90.0, -45.0, -90.0, 137.5):
        yaw = math.radians(yaw_deg)
        ereignis = _coverage_event(u_mm, v_mm, np.zeros(3), yaw, 0.0, 0.0,
                                   mm_per_column, 1.0, 0, 1, [])
        sin_y, cos_y = math.sin(yaw), math.cos(yaw)
        for duese, (row, col) in zip((0, NUM_NOZZLES - 1), ereignis["bar"]):
            versatz = duese * NOZZLE_PITCH_MM
            soll_row = (v_mm + versatz * cos_y) / NOZZLE_PITCH_MM
            soll_col = (u_mm - versatz * sin_y) / mm_per_column
            assert abs(row - soll_row) < 0.01, (yaw_deg, duese, row, soll_row)
            assert abs(col - soll_col) < 0.01, (yaw_deg, duese, col, soll_col)


def test_bar_endpunkte_fallen_bei_null_gier_auf_row_col_zusammen():
    """Bei yaw = 0 steht die Leiste senkrecht: beide Endpunkte teilen sich
    die Spalte, und der erste ist genau das gemeldete row/col -- die
    Verankerung, ohne die die Linie um eine halbe Leiste versetzt läge."""
    import numpy as np
    from printhead.controller import _coverage_event
    from printhead.geometry import NUM_NOZZLES

    ereignis = _coverage_event(10.0, 5.0, np.zeros(3), 0.0, 0.0, 0.0,
                               0.087, None, 0, 1, [])
    (row0, col0), (row1, col1) = ereignis["bar"]
    assert abs(col0 - col1) < 0.01, ereignis["bar"]
    assert round(row0) == ereignis["row"]
    assert round(col0) == ereignis["col"]
    assert row1 > row0                      # Leiste laeuft in +v / +Zeile
    assert round(row1 - row0) == NUM_NOZZLES - 1


def test_deckung_waechst_ueber_den_durchgang():
    """covered/total ist nur brauchbar, wenn es sich auch bewegt — ein
    konstanter Wert wäre als Fortschrittsanzeige wertlos."""
    async def run():
        hub = Hub()
        fake = _FakeWS()
        hub.clients.add(fake)
        await hub.run_action([
            "--pattern", "solid", "--pattern-length-mm", "6",
            "--pattern-height-mm", "6", "--mode", "page",
            "--page-frame", "simple", "--dry-run", "--simulate",
            "--progress-json", "--auto-start", "--once", "--timeout", "3",
        ])
        await _bis_fertig(hub)
        return fake.messages

    werte = [m["covered"] for m in asyncio.run(run())
             if m.get("event") == "coverage" and "covered" in m]
    assert len(werte) > 2, werte
    assert werte[-1] > werte[0], werte[:5]
    assert werte == sorted(werte), "Deckung darf nie zurückgehen"


def test_zellen_erreichen_den_client_genau_einmal():
    """Die Live-Ansicht zeichnet Deltas: eine doppelt gemeldete Zelle wäre
    harmlos, eine verlorene bliebe für den Rest des Durchgangs ein Loch.
    Derselbe Exactly-once-Vertrag wie in test_freehand_pass.py, hier aber
    hinter Drosselung, WS-Relay und JSON-Serialisierung geprüft — genau die
    Strecke, auf der eine künftige Optimierung Zellen fallen lassen würde."""
    async def run():
        hub = Hub()
        fake = _FakeWS()
        hub.clients.add(fake)
        await hub.run_action([
            "--pattern", "solid", "--pattern-length-mm", "6",
            "--pattern-height-mm", "6", "--mode", "page",
            "--page-frame", "simple", "--dry-run", "--simulate",
            "--progress-json", "--auto-start", "--once", "--timeout", "3",
        ])
        await _bis_fertig(hub)
        return fake.messages

    nachrichten = asyncio.run(run())

    start = [m for m in nachrichten if m.get("event") == "coverage_start"]
    assert start, "coverage_start muss den Client erreichen"
    assert start[0]["width"] > 0 and start[0]["height"] > 0

    zellen = [tuple(c) for m in nachrichten
              if m.get("event") == "coverage" for c in m.get("new_cells", [])]
    assert zellen, "ohne new_cells kann die Live-Ansicht nichts zeichnen"
    assert len(zellen) == len(set(zellen)), (
        f"{len(zellen) - len(set(zellen))} Zellen doppelt gemeldet")

    fertig = [m for m in nachrichten if m.get("event") == "coverage_done"]
    assert fertig, nachrichten[-3:]
    assert len(zellen) >= fertig[-1]["covered"], (
        "weniger Zellen gemeldet als am Ende als bedeckt gezählt — der "
        "finale Flush fehlt oder die Drosselung verwirft")


_STATIC = pathlib.Path(__file__).resolve().parent.parent / "printhead" / "ui" / "static"


def test_ui_zeichnet_die_zellen_auch_wirklich():
    """Schwacher, aber gezielter Verdrahtungstest: die Zellen wurden lange
    berechnet, serialisiert und gesendet — und im Browser verworfen, weil es
    gar kein Canvas gab. Genau diese Lücke sichert das hier ab.

    Die eigentliche Canvas-Logik (getContext, das Canvas-Markup) liegt seit
    der Druckansicht in coverage_view.js, geteilt zwischen index.html und
    view.html -- siehe test_beide_seiten_laden_coverage_view_js für die
    Einbindung und test_coverage_view_js_zeichnet_die_zellen_und_die_marke
    für den Inhalt. Hier bleibt nur, was WIRKLICH noch in index.html steht:
    die AUFRUFSTELLEN in dessen eigenem Inline-Skript."""
    quelle = (_STATIC / "index.html").read_text()
    # Auf die AUFRUFSTELLEN geprüft, nicht auf das blosse Vorkommen der
    # Wörter: ein Kommentar, der "new_cells" erwähnt, erfüllt eine
    # Vorkommensprüfung auch dann noch, wenn die Verdrahtung entfernt wurde
    # (genau so ist diese Prüfung beim Mutationstest zuerst durchgerutscht).
    for aufruf in ("covStart(m.width, m.height)",
                   "covCells(m.new_cells)",
                   "/api/preview.png"):
        assert aufruf in quelle, f"index.html ruft {aufruf!r} nicht auf"


def test_ui_zeichnet_die_druckkopf_marke():
    """Dieselbe Verdrahtungsprüfung für die Kopfanzeige, auf index.html's
    eigene AUFRUFSTELLEN beschränkt -- das Zeichnen selbst prüft
    test_coverage_view_js_zeichnet_die_zellen_und_die_marke."""
    quelle = (_STATIC / "index.html").read_text()
    for aufruf in ("covHead(m.bar)",          # Ereignis -> Zustand
                   "covHead(null)"):          # am Durchgangsende geloescht
        assert aufruf in quelle, f"index.html ruft {aufruf!r} nicht auf"


def test_beide_seiten_laden_coverage_view_js():
    """Die eine Zeile, die index.html und view.html überhaupt erst an die
    geteilte Canvas-Logik anschließt. Ohne sie rufen beide Seiten covStart/
    covCells/covHead auf undefinierte Funktionen auf -- ein Fehler, den
    keiner der Aufrufstellen-Tests oben oder unten für sich allein fängt,
    weil jeder nur seine eigene Datei liest."""
    for name in ("index.html", "view.html"):
        quelle = (_STATIC / name).read_text()
        assert '<script src="/coverage_view.js">' in quelle, name


def test_coverage_view_js_zeichnet_die_zellen_und_die_marke():
    """Der Inhalt der ausgelagerten Datei: dieselben Aufrufstellen-Prüfungen
    wie zuvor direkt in index.html, jetzt hier, weil hier die eigentliche
    Zeichenarbeit passiert."""
    quelle = (_STATIC / "coverage_view.js").read_text()
    for aufruf in ("getContext(\"2d\")",
                   "covDrawHead()",           # je Bild neu gezeichnet
                   'id="cov-ov"'):            # eigenes Overlay-Canvas
        assert aufruf in quelle, f"coverage_view.js enthaelt {aufruf!r} nicht"
    # Das Overlay MUSS ein zweites Canvas sein: das Deckungs-Canvas wird
    # nur ergaenzt und nie geloescht, eine dort gezeichnete Kopflinie
    # bliebe als Schleifspur stehen.
    assert quelle.count("<canvas") >= 2, "Kopfmarke braucht ein eigenes Canvas"
    assert "clearRect" in quelle, "Overlay wird nie geleert"


def test_view_html_ruft_dieselben_funktionen_auf():
    """Die Druckansicht-Seite muss dieselben drei Einstiegspunkte in
    coverage_view.js benutzen wie index.html -- sonst bekäme sie zwar die
    Ereignisse über denselben /ws, würde aber nichts zeichnen."""
    quelle = (_STATIC / "view.html").read_text()
    for aufruf in ("covStart(m.width, m.height)",
                   "covCells(m.new_cells)",
                   "covHead(m.bar)",
                   "covHead(null)"):
        assert aufruf in quelle, f"view.html ruft {aufruf!r} nicht auf"


# ==================================== Startseite: Vorschau-Panels entfernt
def test_index_druckvorschau_panel_ist_entfernt():
    """Auf Wunsch entfernt: das sichtbare Vorschaubild-Panel samt seinem
    "Vorschau neu"-Knopf. NICHT gemeint war der "Druckansicht ↗"-Knopf, der
    öffnet eine ganz andere Seite (/view) -- siehe
    test_index_deckung_live_und_druckansicht_bleiben unten."""
    quelle = (_STATIC / "index.html").read_text()
    assert "<h2>Druckvorschau</h2>" not in quelle
    assert 'id="pv-wrap"' not in quelle
    assert 'id="b-preview"' not in quelle
    assert "Vorschau neu" not in quelle


def test_index_letzter_durchgang_panel_ist_entfernt():
    """Auf Wunsch entfernt: die Anzeige des letzten Druckergebnisses
    (record.png, ein Vier-Panel-Diagnosebild). Die dafuer zustaendige
    loadRecord()-Funktion muss mitentfernt sein, sonst bliebe totes,
    nirgends mehr aufgerufenes Fetch-Fragment zurueck."""
    quelle = (_STATIC / "index.html").read_text()
    assert "<h2>Deckung (letzter Durchgang)</h2>" not in quelle
    assert 'id="rec-wrap"' not in quelle
    assert "loadRecord" not in quelle


def test_index_deckung_live_und_druckansicht_bleiben():
    """Ausdruecklich NICHT Teil der Anfrage: das Live-Deckungspanel und der
    Knopf zur separaten Druckansicht (/view) bleiben bestehen."""
    quelle = (_STATIC / "index.html").read_text()
    assert '<h2>Deckung (live)</h2>' in quelle
    assert 'id="cov-wrap"' in quelle
    assert '<a href="/view" target="_blank"' in quelle
    assert "Druckansicht ↗" in quelle


def test_index_hintergrund_vorschau_bleibt_verdrahtet():
    """Ohne sichtbares Panel gibt es keinen Knopf mehr, der /api/preview
    manuell antriggert -- der automatische Hintergrund-Refresh (bei jeder
    Feldaenderung und beim Laden) MUSS aber weiterlaufen, sonst bekaeme die
    Geisterebene in covStart() (geteilt mit /view, siehe coverage_view.js)
    beim naechsten Durchgang ein veraltetes Zielbild. Siehe die ausfuehrliche
    Begruendung direkt im Quelltext ueber refreshPreview()."""
    quelle = (_STATIC / "index.html").read_text()
    assert "async function refreshPreview(){" in quelle
    assert 'await post("/api/preview", {args: previewArgs()});' in quelle
    # An den tatsaechlichen Aufrufstellen geprueft, nicht nur am Vorhandensein
    # der Funktion: jede Feldaenderung (input/change) und der initiale Aufruf
    # beim Laden muessen sie weiterhin auslösen.
    assert ('el.addEventListener("input", () => { refreshCmd(); schedulePreview(); });'
            in quelle)
    assert "connect();\nrefreshPreview();" in quelle


# ============================================== Standardwerte Dosis/Spray
def test_ui_standardwerte_fuer_dosis_und_spray():
    """Vom Anlagenbesitzer festgelegt: Dosis 2, Spray aus. Die Felder der
    Oberfläche müssen dieselben Werte tragen wie die CLI-Defaults, sonst
    druckt ein Klick in der UI anders als derselbe Lauf im Terminal."""
    quelle = (_STATIC / "index.html").read_text()
    for feld, wert in (("dose", "2"), ("spray_r", "0"), ("spray_s", "0")):
        muster = f'id="{feld}" type="number" value="{wert}"'
        assert muster in quelle, f"Feld {feld!r} steht nicht auf {wert!r}"


def test_cli_standardwerte_fuer_dosis_und_spray():
    """Die andere Hälfte desselben Vertrags, auf der Python-Seite.

    --spray-radius-mm/--spray-strength standen auf 0.15/0.5, obwohl BEIDE
    Hilfetexte schon immer 'Default 0 (off)' behaupteten -- Spray war also
    entgegen der eigenen Dokumentation standardmäßig an. Hier festgenagelt.
    """
    from printhead import cli
    from printhead.coverage import DEFAULT_DROPS_PER_PIXEL

    assert DEFAULT_DROPS_PER_PIXEL == 2.0
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line"])
    assert args.spray_radius_mm == 0.0
    assert args.spray_strength == 0.0
    # Und der Weg bis zur Engine, nicht nur der argparse-Wert: build_ink/
    # build_controller reichen die Werte nur weiter, wenn sie nicht None
    # sind -- 0.0 ist nicht None, muss also ankommen.
    ctrl = cli.build_controller(args)
    assert ctrl.spray_radius_mm == 0.0
    assert ctrl.spray_strength == 0.0
    assert ctrl.drops_per_pixel == DEFAULT_DROPS_PER_PIXEL


# ====================================================== Sensor-Übergabe
def test_aktion_uebernimmt_den_sensor_und_gibt_ihn_zurueck():
    """Der Tracker lässt sich nicht zweimal öffnen. Eine Aktion verdrängt den
    Leerlauf-Strom und startet ihn danach von selbst wieder."""
    async def run():
        hub = Hub()
        hub.clients.add(_FakeWS())
        s = await hub.start_sensor(["--simulate"])
        assert s["ok"], s
        await asyncio.sleep(0.8)
        assert hub.status()["sensor_running"], "Sensorstrom lief nicht an"

        await hub.run_action(["--pattern", "solid", "--pattern-length-mm", "3",
                              "--dry-run", "--simulate", "--mode", "line"])
        # Während der Aktion ist der Strom abgetreten ...
        waehrend = hub.status()["sensor_running"]
        assert await _bis_fertig(hub)
        # ... und kommt danach von selbst zurück.
        zurueck = False
        for _ in range(60):
            await asyncio.sleep(0.1)
            if hub.status()["sensor_running"]:
                zurueck = True
                break
        await hub.stop_sensor()
        return waehrend, zurueck

    waehrend, zurueck = asyncio.run(run())
    assert waehrend is False, "der Sensor hätte für die Aktion weichen müssen"
    assert zurueck is True, "der Sensor kam nach der Aktion nicht zurück"


def test_aktion_schaltet_den_sensor_nicht_von_selbst_ein():
    """Lief kein Strom, darf eine Aktion den Tracker auch nicht anschalten —
    sonst greift sie ungefragt auf Hardware zu."""
    async def run():
        hub = Hub()
        hub.clients.add(_FakeWS())
        assert not hub.status()["sensor_running"]
        await hub.run_action(["--pattern", "solid", "--pattern-length-mm", "3",
                              "--dry-run", "--simulate", "--mode", "line"])
        assert await _bis_fertig(hub)
        for _ in range(20):
            await asyncio.sleep(0.05)
        return hub.status()["sensor_running"]

    assert asyncio.run(run()) is False


def test_sensor_stopp_vor_einer_aktion_bleibt_gestoppt():
    """Vorher gestoppt heißt: die Aktion findet nichts zum Zurückbringen."""
    async def run():
        hub = Hub()
        hub.clients.add(_FakeWS())
        await hub.start_sensor(["--simulate"])
        await asyncio.sleep(0.5)
        await hub.stop_sensor()
        await hub.run_action(["--pattern", "solid", "--pattern-length-mm", "3",
                              "--dry-run", "--simulate", "--mode", "line"])
        assert await _bis_fertig(hub)
        for _ in range(20):
            await asyncio.sleep(0.05)
        return hub.status()["sensor_running"]

    assert asyncio.run(run()) is False


def test_sensor_stopp_WAEHREND_einer_aktion_hebt_die_wiederaufnahme_auf():
    """
    Der Fall, auf den es wirklich ankommt: läuft der Strom beim Start einer
    Aktion, wird seine Wiederaufnahme vorgemerkt. Stoppt der Bediener ihn
    dann WÄHREND der Aktion, ist das eine Ansage — er darf nicht
    zurückkommen, bloß weil die Aktion später endet.

    (Vorher gestoppt reicht als Prüfung NICHT: dort setzt run_action die
    Vormerkung ohnehin auf None, der Test liefe also auch mit kaputtem
    stop_sensor durch — beim Mutationstest genau so passiert.)
    """
    async def run():
        hub = Hub()
        hub.clients.add(_FakeWS())
        await hub.start_sensor(["--simulate"])
        await asyncio.sleep(0.8)
        assert hub.status()["sensor_running"]

        await hub.run_action(["--pattern", "solid", "--pattern-length-mm", "40",
                              "--dry-run", "--simulate", "--mode", "line"])
        assert hub._sensor_resume is not None, "Wiederaufnahme nicht vorgemerkt"

        await hub.stop_sensor()          # mitten in der laufenden Aktion
        await hub.stop_action()
        assert await _bis_fertig(hub)
        for _ in range(30):
            await asyncio.sleep(0.05)
        laeuft = hub.status()["sensor_running"]
        await hub.stop_sensor()
        return laeuft

    assert asyncio.run(run()) is False, \
        "der Sensor kam trotz ausdrücklichem Stopp zurück"


def test_zwei_aktionen_gleichzeitig_werden_abgelehnt():
    async def run():
        hub = Hub()
        hub.clients.add(_FakeWS())
        a = await hub.run_action(["--pattern", "solid", "--pattern-length-mm", "40",
                                  "--dry-run", "--simulate", "--mode", "line"])
        b = await hub.run_action(["--pattern", "solid", "--dry-run", "--simulate"])
        await hub.stop_action()
        await _bis_fertig(hub)
        return a, b

    a, b = asyncio.run(run())
    assert a["ok"] is True
    assert b["ok"] is False and "bereits" in b["error"]


# ============================================================ Vorschau
def test_vorschau_liefert_die_echte_fehlermeldung_zurueck():
    """Scheitert die Vorschau, gehört die Ursache ins Vorschaufeld — im Log
    ginge sie zwischen der laufenden Aktion unter."""
    async def run():
        hub = Hub()
        hub.clients.add(_FakeWS())
        return await hub.run_preview([
            "--pattern", "drill_pattern",
            "--pattern-image", "/gibt/es/nicht.png", "--mode", "line"])

    r = asyncio.run(run())
    assert r["ok"] is False
    assert "gibt/es/nicht.png" in r["error"] or "image" in r["error"].lower(), r


def test_vorschau_laeuft_ohne_den_sensor_zu_stoeren():
    """Die Vorschau ist keine Aktion: sie darf den Sensorstrom nicht
    verdrängen, sonst flackert die Live-Anzeige bei jeder Feldänderung."""
    async def run():
        hub = Hub()
        hub.clients.add(_FakeWS())
        await hub.start_sensor(["--simulate"])
        await asyncio.sleep(0.8)
        vorher = hub.status()["sensor_running"]
        await hub.run_preview(["--pattern", "solid", "--pattern-length-mm", "10",
                               "--mode", "line"])
        nachher = hub.status()["sensor_running"]
        await hub.stop_sensor()
        return vorher, nachher

    vorher, nachher = asyncio.run(run())
    assert vorher is True and nachher is True


# ============================================================ Testkatalog
def test_testkatalog_ist_vollstaendig_und_wohlgeformt():
    assert TEST_ACTIONS, "Katalog ist leer"
    ids = [t["id"] for t in TEST_ACTIONS]
    assert len(ids) == len(set(ids)), ids
    for t in TEST_ACTIONS:
        for feld in ("id", "test", "label", "help", "args",
                     "needs_calibration", "dry"):
            assert feld in t, (feld, t)
        assert t["args"], t["id"]
        assert all(isinstance(a, str) for a in t["args"]), t["id"]
        assert t["help"].strip(), t["id"]


def test_testkatalog_nennt_nur_echte_cli_optionen():
    """
    Ein Knopf, der eine Option schickt, die es nicht gibt, scheitert sonst
    erst beim Klicken auf der Anlage. Hier scheitert er im Test.

    Nachgebaut wird, was die Oberfläche wirklich zusammensetzt: Einträge mit
    ``needs_calibration`` bekommen die Grundeinstellungen angehängt (Modus und
    Seitenrahmen), die übrigen laufen für sich. Ohne diese Nachbildung würde
    der Test Knöpfe bemängeln, die in der UI einwandfrei funktionieren.
    """
    from printhead import cli

    for t in TEST_ACTIONS:
        argumente = list(t["args"])
        if t["needs_calibration"]:
            argumente += ["--mode", "page", "--page-frame", "simple"]
        try:
            cli.parse_args(argumente + ["--dry-run"])
        except SystemExit as exc:
            raise AssertionError(
                f"Testknopf {t['id']!r} benutzt Argumente, die die CLI "
                f"ablehnt: {argumente} ({exc})") from None


def test_testknoepfe_ohne_kalibrierung_laufen_auch_allein():
    """Gegenprobe: Einträge mit needs_calibration=False dürfen KEINEN
    Seitenrahmen brauchen — sonst ist das Flag falsch gesetzt und der Knopf
    scheitert auf der Anlage."""
    from printhead import cli

    for t in TEST_ACTIONS:
        if t["needs_calibration"]:
            continue
        try:
            cli.parse_args(list(t["args"]) + ["--dry-run"])
        except SystemExit as exc:
            raise AssertionError(
                f"Testknopf {t['id']!r} ist als needs_calibration=False "
                f"markiert, braucht aber doch einen Seitenrahmen ({exc})"
            ) from None


# ==================================================== Druckansicht (/view)
def test_view_route_liefert_view_html():
    """Dieselbe Herangehensweise wie beim bestehenden `/`-Test wäre (dieses
    Projekt hat kein httpx/TestClient, siehe test_ui_calibration.py's
    Modul-Doku) -- der async-Routenhandler wird direkt aufgerufen, nicht
    über einen echten HTTP-Request."""
    from printhead.ui.server import view

    html = asyncio.run(view())
    assert "Druckansicht" in html
    assert '<script src="/coverage_view.js">' in html
    # Muss vom selben Server-Text stammen wie die Datei auf der Platte,
    # nicht eine eingebettete Kopie -- sonst laufen Route und Datei
    # irgendwann auseinander.
    assert html == (_STATIC / "view.html").read_text(encoding="utf-8")


def test_coverage_view_js_route_liefert_die_datei_mit_dem_richtigen_typ():
    from printhead.ui.server import coverage_view_js

    resp = asyncio.run(coverage_view_js())
    assert resp.media_type == "application/javascript"
    assert str(_STATIC / "coverage_view.js") in str(resp.path)


# ========================================== Replay eines laufenden Durchgangs
def test_registrierung_ohne_laufenden_durchgang_bekommt_keinen_replay():
    """Der Normalfall: nichts läuft, ein neu verbundener Client bekommt nur
    den Status, keinen erfundenen coverage_event."""
    async def run():
        hub = Hub()
        fake = _FakeWS()
        await hub.register(fake)
        return fake.messages

    nachrichten = asyncio.run(run())
    assert len(nachrichten) == 1
    assert nachrichten[0]["type"] == "status"


def test_registrierung_waehrend_eines_durchgangs_bekommt_den_replay():
    """Der Vertrag, den /view überhaupt erst nutzbar macht: ein Fenster, das
    MITTEN in einem laufenden Durchgang verbunden wird, muss sofort wissen,
    wie groß das Zielbild ist -- sonst bliebe seine Leinwand bis zum
    NÄCHSTEN Durchgang leer, der bei einem einzelnen `--once`-Lauf nie
    kommt. Deterministisch geprüft, ohne echten Durchgang/Timing-Rennen:
    _last_coverage_start wird direkt gesetzt, wie es on_line während eines
    echten Laufs auch täte (siehe die End-zu-Ende-Gegenprobe unten)."""
    async def run():
        hub = Hub()
        hub._last_coverage_start = {"event": "coverage_start",
                                    "width": 42, "height": 7}
        fake = _FakeWS()
        await hub.register(fake)
        return fake.messages

    nachrichten = asyncio.run(run())
    assert len(nachrichten) == 2
    assert nachrichten[0]["type"] == "status"
    replay = nachrichten[1]
    assert replay["type"] == "coverage_event"
    assert replay["event"] == "coverage_start"
    assert replay["width"] == 42 and replay["height"] == 7
    assert replay["replay"] is True


def test_ein_echter_durchgang_setzt_und_loescht_last_coverage_start():
    """Die End-zu-Ende-Gegenprobe zum Test oben: dass on_line
    _last_coverage_start während eines ECHTEN Durchgangs wirklich pflegt,
    nicht nur, dass register() es korrekt weiterreicht, wenn es gesetzt
    ist. Gepollt statt auf ein exaktes Zeitfenster gewettet -- run_action
    läuft asynchron neben der Poll-Schleife, und ein `--once`-Durchgang mit
    mehreren Metern Fahrweg braucht durchweg mehr als einen 20ms-Tick."""
    async def run():
        hub = Hub()
        fake = _FakeWS()
        hub.clients.add(fake)
        gesehen_waehrend_aktiv = []
        await hub.run_action([
            "--pattern", "solid", "--pattern-length-mm", "40",
            "--pattern-height-mm", "40", "--mode", "page",
            "--page-frame", "simple", "--dry-run", "--simulate",
            "--progress-json", "--auto-start", "--once", "--timeout", "5",
        ])
        for _ in range(500):
            if hub.action is None or not hub.action.running:
                break
            if hub._last_coverage_start is not None:
                gesehen_waehrend_aktiv.append(dict(hub._last_coverage_start))
            await asyncio.sleep(0.02)
        await _bis_fertig(hub)
        return gesehen_waehrend_aktiv, hub._last_coverage_start, fake.messages

    gesehen, danach, nachrichten = asyncio.run(run())
    echter_start = next(m for m in nachrichten if m.get("event") == "coverage_start")
    assert gesehen, "on_line hat _last_coverage_start waehrend des Durchgangs nie gesetzt"
    assert gesehen[0]["width"] == echter_start["width"]
    assert gesehen[0]["height"] == echter_start["height"]
    assert danach is None, "coverage_done/on_exit muessen _last_coverage_start wieder loeschen"


def test_neue_aktion_verwirft_last_coverage_start_der_vorigen():
    """Ohne das würde eine Aktion, die selbst gar keine Deckung meldet (z.B.
    line/time-Modus, oder eine, die vor dem ersten coverage_start endet),
    einem mitten in ihr verbindenden Client fälschlich die Geometrie DES
    VORIGEN Durchgangs als angeblich laufend unterschieben."""
    async def run():
        hub = Hub()
        hub._last_coverage_start = {"event": "coverage_start",
                                    "width": 999, "height": 999}
        fake = _FakeWS()
        hub.clients.add(fake)
        await hub.run_action(["Hi", "--dry-run", "--mode", "line",
                              "--simulate", "--once", "--timeout", "1"])
        return hub._last_coverage_start

    assert asyncio.run(run()) is None


if __name__ == "__main__":
    tests = [v for k, v in list(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"OK: {t.__name__}")
    print(f"Alle {len(tests)} UI-Live-Tests bestanden.")
