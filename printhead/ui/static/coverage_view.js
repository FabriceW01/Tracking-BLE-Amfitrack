/*
 * Live-Deckungsansicht -- Canvas-Logik, geteilt zwischen index.html (Panel
 * "Deckung (live)") und view.html (der große Druckansicht-Modus, siehe
 * README).
 *
 * Eigene Datei statt in beiden Seiten dupliziert: dieselbe Begründung wie
 * für _coverage_event's serverseitige `bar`-Berechnung (siehe
 * controller.py) -- zwei Kopien derselben Canvas-/Skalierungs-Rechnung
 * würden beim nächsten Umbau still auseinanderlaufen, und hier geht es
 * nicht nur um Optik: eine falsch skalierte Kopfmarke zeigt dem Bediener
 * die falsche Stelle auf dem Papier.
 *
 * Klassisches Script (kein <script type="module">), absichtlich: beide
 * Seiten laden diese Datei VOR ihrem eigenen Inline-<script>-Block und
 * teilen sich denselben globalen Scope, genau wie zuvor innerhalb einer
 * einzigen Datei -- keine Import/Export-Umstellung nötig, keine neue
 * Bundling-Infrastruktur für ein Projekt ohne Build-Schritt.
 *
 * Erwartet vom HTML der einbindenden Seite:
 *   #cov-wrap   -- .imgwrap-Container, der das Canvas-Paar aufnimmt
 *   #cov-note   -- Textzeile für Warnungen ("unvollständig", ...)
 * covStart() baut #cov/#cov-ov/#cov-stack selbst hinein.
 */

/* Eine Canvas-Zelle = ein Pixel des Zielbilds, dieselbe Konvention wie
   record.png. Bei --mm-per-column 0.087 gegen NOZZLE_PITCH_MM 0.0868 sind
   die Zellen auf 0,2% quadratisch; bei einem abweichenden --mm-per-column
   ist das Bild horizontal gestreckt -- record.png hat exakt dieselbe
   Eigenschaft, es ist also dieselbe Vereinfachung, keine neue. */
const COV_MAX_PX = 1600;      // Deckel fuer den Canvas-Speicher
let covCtx = null, covScale = 1, covActive = false, covStale = false;
let covQueue = [], covPending = false;
/* Druckkopf-Overlay: covBar sind die beiden Balken-Endpunkte
   [[row,col],[row,col]] aus dem coverage-Event (Duese 0 und die letzte),
   im Pixelraster des Zielbilds -- serverseitig mit derselben Formel
   gerechnet, mit der CoverageEngine.step() die Duesen platziert, siehe
   controller._coverage_event. */
let covOvCtx = null, covBar = null;

function covNote(text, warn){
  const el = document.getElementById("cov-note");
  if (!el) return;
  el.textContent = text || "";
  el.className = warn ? "warn" : "";
}

function covStart(width, height){
  const wrap = document.getElementById("cov-wrap");
  covScale = Math.min(1, COV_MAX_PX / width, COV_MAX_PX / height);
  let cv = document.getElementById("cov");
  if (!cv){
    wrap.classList.remove("empty");
    wrap.innerHTML = '<div id="cov-stack"><canvas id="cov"></canvas>'
                   + '<canvas id="cov-ov"></canvas></div>';
    cv = document.getElementById("cov");
    cv.addEventListener("click", () => wrap.classList.toggle("zoom"));
  }
  cv.width  = Math.max(1, Math.round(width  * covScale));
  cv.height = Math.max(1, Math.round(height * covScale));
  covCtx = cv.getContext("2d");
  covCtx.fillStyle = "#fff";
  covCtx.fillRect(0, 0, cv.width, cv.height);
  /* Overlay teilt sich das Geraetepixel-Raster mit dem Deckungs-Canvas,
     liegt per CSS aber auf dessen DARGESTELLTER Groesse -- damit stimmen
     Linie und Zellen in jedem Zoomzustand ueberein. */
  const ov = document.getElementById("cov-ov");
  ov.width = cv.width; ov.height = cv.height;
  covOvCtx = ov.getContext("2d");
  covBar = null;
  covQueue = [];
  covActive = true; covStale = false;
  covNote("");

  /* Zielbild blass darunter: save_preview() schreibt die Ink-Maske exakt
     1:1, also dasselbe (H,W)-Raster, das coverage_start meldet. Nur
     verwenden, wenn die Groesse wirklich passt -- die Vorschau kann fehlen
     oder zu anderen Argumenten gehoeren. */
  const bg = new Image();
  bg.onload = () => {
    if (!covCtx || bg.naturalWidth !== width || bg.naturalHeight !== height) return;
    covCtx.save();
    covCtx.globalAlpha = 0.18;
    covCtx.drawImage(bg, 0, 0, cv.width, cv.height);
    covCtx.restore();
  };
  bg.src = "/api/preview.png?t=" + Date.now();
}

function covDraw(){
  covPending = false;
  if (!covCtx) { covQueue = []; return; }
  covCtx.fillStyle = "#1a1f27";
  for (const batch of covQueue){
    for (const cell of batch){
      /* floor + immer 1 Geraetepixel: bei covScale<1 fallen Nachbarzellen
         auf dasselbe Pixel, was ein idempotentes Nachfuellen ist --
         verschwinden kann dadurch nichts. */
      covCtx.fillRect(Math.floor(cell[1] * covScale),
                      Math.floor(cell[0] * covScale), 1, 1);
    }
  }
  covQueue = [];
  covDrawHead();
}

function covCells(cells){
  if (!covCtx || !cells || !cells.length) return;
  covQueue.push(cells);
  if (!covPending){ covPending = true; requestAnimationFrame(covDraw); }
}

/* ---------------------------------------------- Druckkopf im Deckungsbild
   Zwei getrennte Anzeigen, die sich gegenseitig ausschliessen:

     * Der Kopf ist (wenigstens teilweise) IM Bild -> rote Linie entlang der
       Duesenleiste, an der aktuellen Gierlage. Ohne die sieht man zwar was
       schon Tinte hat, aber nicht, wo man gerade steht.
     * Der Kopf ist KOMPLETT AUSSERHALB -> keine Linie (es gaebe nichts zu
       sehen), stattdessen ein farbiger Punkt am Bildrand in seiner
       Richtung, damit man weiss, wohin man zurueckfahren muss.

   Die Fallunterscheidung laeuft ueber die Balkenmitte, nicht ueber die
   Endpunkte: bei starker Schraeglage kann ein Ende drin und das andere
   draussen sein, und dann ist die Linie das Richtige -- ein Randpunkt
   waere dann sogar irrefuehrend, weil er auf etwas zeigt, das man ohnehin
   schon sieht. */
function covHead(bar){
  covBar = bar || null;
  if (!covPending){ covPending = true; requestAnimationFrame(covDraw); }
}

function covDrawHead(){
  const ctx = covOvCtx;
  if (!ctx) return;
  const w = ctx.canvas.width, h = ctx.canvas.height;
  ctx.clearRect(0, 0, w, h);
  if (!covBar) return;

  const x0 = covBar[0][1] * covScale, y0 = covBar[0][0] * covScale;
  const x1 = covBar[1][1] * covScale, y1 = covBar[1][0] * covScale;
  const inside = (x, y) => x >= 0 && x < w && y >= 0 && y < h;

  if (inside(x0, y0) || inside(x1, y1)){
    ctx.strokeStyle = "#e0402f";
    /* Mindestens 1,5 Geraetepixel, sonst verschwindet die Linie im
       6-fach verkleinerten Uebersichtszustand voellig. */
    ctx.lineWidth = Math.max(1.5, 2 * covScale);
    ctx.lineCap = "round";
    ctx.beginPath();
    ctx.moveTo(x0, y0);
    ctx.lineTo(x1, y1);
    ctx.stroke();
    /* Duese 0 markiert, damit die Leiste eine erkennbare Richtung hat --
       sonst sieht man den Balken, weiss aber nicht, welches Ende welches
       ist (relevant, sobald man den Wagen dreht). */
    ctx.fillStyle = "#e0402f";
    ctx.beginPath();
    ctx.arc(x0, y0, Math.max(2, 3 * covScale), 0, 2 * Math.PI);
    ctx.fill();
    return;
  }

  /* Komplett draussen: Balkenmitte auf den Bildrand projizieren und dort
     einen Punkt setzen. Geklemmt statt weggelassen, damit der Punkt auch
     bei weit entferntem Kopf am Rand sichtbar bleibt statt aus dem Canvas
     zu wandern. */
  const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
  const r = Math.max(4, 6 * covScale);
  const px = Math.min(w - r, Math.max(r, cx));
  const py = Math.min(h - r, Math.max(r, cy));
  ctx.fillStyle = "#f0a020";
  ctx.strokeStyle = "#2a1c05";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(px, py, r, 0, 2 * Math.PI);
  ctx.fill();
  ctx.stroke();
}
