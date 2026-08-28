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


def test_ui_zeichnet_die_zellen_auch_wirklich():
    """Schwacher, aber gezielter Verdrahtungstest: die Zellen wurden lange
    berechnet, serialisiert und gesendet — und im Browser verworfen, weil es
    gar kein Canvas gab. Genau diese Lücke sichert das hier ab."""
    quelle = (pathlib.Path(__file__).resolve().parent.parent
              / "printhead" / "ui" / "static" / "index.html").read_text()
    # Auf die AUFRUFSTELLEN geprüft, nicht auf das blosse Vorkommen der
    # Wörter: ein Kommentar, der "new_cells" erwähnt, erfüllt eine
    # Vorkommensprüfung auch dann noch, wenn die Verdrahtung entfernt wurde
    # (genau so ist diese Prüfung beim Mutationstest zuerst durchgerutscht).
    for aufruf in ("covStart(m.width, m.height)",
                   "covCells(m.new_cells)",
                   "getContext(\"2d\")",
                   "/api/preview.png"):
        assert aufruf in quelle, f"index.html ruft {aufruf!r} nicht auf"
    assert "<canvas" in quelle


def test_ui_zeichnet_die_druckkopf_marke():
    """Dieselbe Verdrahtungsprüfung für die Kopfanzeige: die Endpunkte
    werden serverseitig mitgeschickt, das Zeichnen kann trotzdem fehlen.
    Wieder auf AUFRUFSTELLEN geprüft, nicht auf Wortvorkommen."""
    quelle = (pathlib.Path(__file__).resolve().parent.parent
              / "printhead" / "ui" / "static" / "index.html").read_text()
    for aufruf in ("covHead(m.bar)",          # Ereignis -> Zustand
                   "covHead(null)",           # am Durchgangsende geloescht
                   "covDrawHead()",           # je Bild neu gezeichnet
                   'id="cov-ov"'):            # eigenes Overlay-Canvas
        assert aufruf in quelle, f"index.html ruft {aufruf!r} nicht auf"
    # Das Overlay MUSS ein zweites Canvas sein: das Deckungs-Canvas wird
    # nur ergaenzt und nie geloescht, eine dort gezeichnete Kopflinie
    # bliebe als Schleifspur stehen.
    assert quelle.count("<canvas") >= 2, "Kopfmarke braucht ein eigenes Canvas"
    assert "clearRect" in quelle, "Overlay wird nie geleert"


# ============================================== Standardwerte Dosis/Spray
def test_ui_standardwerte_fuer_dosis_und_spray():
    """Vom Anlagenbesitzer festgelegt: Dosis 2, Spray aus. Die Felder der
    Oberfläche müssen dieselben Werte tragen wie die CLI-Defaults, sonst
    druckt ein Klick in der UI anders als derselbe Lauf im Terminal."""
    quelle = (pathlib.Path(__file__).resolve().parent.parent
              / "printhead" / "ui" / "static" / "index.html").read_text()
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


if __name__ == "__main__":
    tests = [v for k, v in list(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"OK: {t.__name__}")
    print(f"Alle {len(tests)} UI-Live-Tests bestanden.")
