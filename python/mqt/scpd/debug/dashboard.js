// Copyright (c) 2026 Chair for Design Automation, TUM
// Copyright (c) 2026 Munich Quantum Software Company GmbH
// All rights reserved.
//
// SPDX-License-Identifier: MIT
//
// Licensed under the MIT License

// The dashboard of one routing phase, in two views of one file.
//
// The table: a table per pass, a row per wire, the columns grouped by round, and in every round a
// column for the plain search ("·") and one per relaxation level. A cell is one search; it links to
// "#search-<n>", the second view, and opens it in a tab of its own.
//
// The search ("#search-<n>"): the picture of that search with layers that switch off, a zoom, and
// what the records say of why it failed. The picture is what the search saw, as its record
// carries it: the box it was confined to, the cells its corridor left open and those it closed, the
// guards in front of the target ports and behind the source ports' stubs, the wire price and the
// obstacle halo, the crossing rule, the lane, the fence, the ring neighbours, the couplers, the
// ways standing and the way it found. The cells of the box are not in this file but beside it
// (`<page>-ground/<k>.js`), loaded as a script when the search is opened, so that the page stays
// small and still opens from the file system. Records without the ways standing (written before
// they were recorded) are replayed: every wire stands on the last way it found, and a search that
// found its way takes the ways of the wires it let go of. Every link is relative, so the page holds
// wherever it is served from.

const grid = JSON.parse(document.getElementById("grid-data").textContent);
const searches = JSON.parse(document.getElementById("search-data").textContent);

// How a cell is shown, in the order of severity.
const LOOK = {
  "gave-up": ["#D55E00", "no way, not even after the relaxation"],
  "no-way": ["#E69F00", "this search found nothing"],
  short: ["#CC79A7", "a way found, but not to be made its length"],
  relaxed: ["#56B4E9", "only after a neighbour was let go of"],
  found: ["#009E73", "a way found"],
};
const esc = (t) =>
  String(t).replace(
    /[&<>"]/g,
    (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c],
  );
const ends = new Map(grid.wires.map((w) => [w.wire, w]));
const isFeedline = (wire) => /^f\d+$/.test(wire);
function sortOf(wire) {
  if (isFeedline(wire)) return "feedline";
  const end = ends.get(wire);
  return end && end.resonator ? "resonator" : "plain";
}
const isShort = (s) =>
  s.found && /no room for a meander|too long for its target|short/.test(s.note);
const count = (n, one, more = `${one}s`) => `${n} ${n === 1 ? one : more}`;
const arrow = (forward) => (forward ? "→" : "←");
const where = (s) =>
  `${s.pass} · round ${s.round} ${arrow(s.forward)} · ${s.kind}`;

// ------------------------------------------------------------- the columns
// A kind of search is a column of its round: the plain search first, then the
// relaxation levels, then any other kind (the refinement's) in the order it came.
function rank(kind) {
  if (kind === "normal") return [0, 0];
  const m = /^relax (\d+)$/.exec(kind);
  return m ? [1, Number(m[1])] : [2, 0];
}
function label(kind) {
  if (kind === "normal") return "·";
  const m = /^relax (\d+)$/.exec(kind);
  return m ? m[1] : kind.slice(0, 1);
}

// The passes in the order they ran, each with its rounds, columns and wires.
const passes = new Map();
searches.forEach((s, at) => {
  if (!passes.has(s.pass))
    passes.set(s.pass, {
      name: s.pass,
      rounds: new Map(),
      kinds: [],
      wires: [],
      cells: new Map(),
    });
  const pass = passes.get(s.pass);
  if (!pass.rounds.has(s.round))
    pass.rounds.set(s.round, { round: s.round, forward: s.forward, count: 0 });
  pass.rounds.get(s.round).count += 1;
  if (!pass.kinds.includes(s.kind)) pass.kinds.push(s.kind);
  if (!pass.wires.includes(s.wire)) pass.wires.push(s.wire);
  const key = `${s.round}|${s.kind}|${s.wire}`;
  if (!pass.cells.has(key)) pass.cells.set(key, []);
  pass.cells.get(key).push(at);
});
for (const pass of passes.values()) {
  const first = new Map(pass.kinds.map((k, at) => [k, at]));
  pass.kinds.sort((a, b) => {
    const [ra, na] = rank(a);
    const [rb, nb] = rank(b);
    return ra - rb || na - nb || first.get(a) - first.get(b);
  });
  pass.roundList = [...pass.rounds.values()].sort((a, b) => a.round - b.round);
}

// What a search came to. A round in which no search of a wire found a way gave
// up: its cells are darker than those a later level of the same round made good.
const roundFound = new Set();
for (const s of searches)
  if (s.found) roundFound.add(`${s.pass}|${s.round}|${s.wire}`);
function outcome(s) {
  if (!s.found)
    return roundFound.has(`${s.pass}|${s.round}|${s.wire}`)
      ? "no-way"
      : "gave-up";
  if (isShort(s)) return "short";
  return s.kind.startsWith("relax") ? "relaxed" : "found";
}
function said(s) {
  return (
    (s.found ? `found ${s.cells} cells` : "no way") +
    (s.note ? ` · ${s.note}` : "") +
    (s.ripped.length ? ` · let go of ${s.ripped.join(", ")}` : "")
  );
}

// -------------------------------------------------------------- the replay
// The ways standing right before search `until`: every wire on the last way it
// found; a search that found its way took the ways of the wires it let go of.
function standingBefore(until) {
  const standing = new Map();
  for (let at = 0; at < until; at++) {
    const s = searches[at];
    if (!s.found) continue;
    for (const r of s.ripped) standing.delete(r);
    standing.set(s.wire, { way: s.way, at });
  }
  return standing;
}
const atTheEnd = standingBefore(searches.length);
const lastFound = new Map();
searches.forEach((s, at) => {
  if (s.found) lastFound.set(s.wire, at);
});
// A wire the phase routes from scratch (a plain search) and that stands on no way at
// the end has none; one the phase only refines keeps a way the records do not hold.
const routed = new Set(
  searches.filter((s) => rank(s.kind)[0] < 2).map((s) => s.wire),
);
function fails(wire) {
  if (!atTheEnd.has(wire))
    return [routed.has(wire) ? "no way" : "no way found here"];
  return isShort(searches[atTheEnd.get(wire).at]) ? ["short"] : [];
}

// --------------------------------------------------------------- the table
function table(pass) {
  const kinds = pass.kinds;
  let h =
    '<div class="wide"><table class="routes"><thead><tr><th class="w"></th>';
  for (const r of pass.roundList)
    h +=
      `<th class="r" colspan="${kinds.length}" title="${r.count} searches, ${r.forward ? "forward" : "backward"}">` +
      `round ${r.round} ${arrow(r.forward)}</th><th class="sep"></th>`;
  h += '<th></th></tr><tr class="levels"><th class="w"></th>';
  for (const _ of pass.roundList) {
    for (const k of kinds)
      h += `<th class="l" title="${esc(k)}">${esc(label(k))}</th>`;
    h += '<th class="sep"></th>';
  }
  h += "<th></th></tr></thead><tbody>";
  for (const wire of pass.wires) {
    const bad = fails(wire);
    let missed = false;
    let row = "";
    for (const r of pass.roundList) {
      for (const k of kinds) {
        const found = pass.cells.get(`${r.round}|${k}|${wire}`);
        if (!found) {
          row += '<td><span class="c"></span></td>';
          continue;
        }
        const at = found[found.length - 1];
        const s = searches[at];
        if (!s.found) missed = true;
        const more =
          found.length > 1
            ? ` · ${found.length} searches here, the last one shown`
            : "";
        const tip = `wire ${wire} · ${where(s)} · ${said(s)}${more}`;
        row +=
          `<td><a class="c${found.length > 1 ? " many" : ""}" href="#search-${at}" target="_blank" ` +
          `style="background:${LOOK[outcome(s)][0]}" title="${esc(tip)}"></a></td>`;
      }
      row += '<td class="sep"></td>';
    }
    const badges = bad
      .map((b) => `<span class="badge">${esc(b)}</span>`)
      .join("");
    h +=
      `<tr data-fail="${bad.length ? 1 : 0}" data-miss="${missed ? 1 : 0}">` +
      `<td class="w ${sortOf(wire)}">${esc(wire)}</td>${row}<td class="sum">${badges}</td></tr>`;
  }
  return h + "</tbody></table></div>";
}

function board() {
  let key = '<div class="key">';
  for (const k in LOOK)
    key += `<span><i style="background:${LOOK[k][0]}"></i>${esc(LOOK[k][1])}</span>`;
  key +=
    '<label class="filt"><input type="checkbox" id="only-fail"> only wires that fail at the end</label>' +
    '<label class="filt"><input type="checkbox" id="only-miss"> only wires with a search that found nothing</label></div>';
  const failing = [...new Set(searches.map((s) => s.wire))].filter(
    (w) => fails(w).length,
  ).length;
  let h =
    key +
    "<p class=\"sub\">The '·' column of a round is its plain search, the numbers are the relaxation levels; " +
    "→ is a forward round, ← a backward one. Blue is a resonator, orange a feedline edge. " +
    "A cell opens its search in a tab of its own; a dot in a cell says the wire was searched there more than once. " +
    `${failing} wires have no way or are short at the end.</p>`;
  for (const pass of passes.values()) {
    const n = pass.roundList.reduce((a, r) => a + r.count, 0);
    h +=
      `<h2>${esc(pass.name)} pass · ${count(pass.wires.length, "wire")} · ${count(pass.roundList.length, "round")} · ${count(n, "search", "searches")}</h2>` +
      table(pass);
  }
  return h;
}

const boardElement = document.getElementById("board");
boardElement.innerHTML = board();
boardElement.addEventListener("change", () => {
  const fail = document.getElementById("only-fail").checked;
  const miss = document.getElementById("only-miss").checked;
  for (const row of boardElement.querySelectorAll("tr[data-fail]"))
    row.hidden =
      (fail && row.dataset.fail === "0") || (miss && row.dataset.miss === "0");
});

// -------------------------------------------------- the picture of a search
// Every layer the picture can hold, grouped the way a reader thinks about them, with the name the
// panel shows. The classes and colours are those of the Final stage's old debug pictures
// (`DEBUG_STYLE`, before 5805c40) where they had one: `co` the band, `ob` the artwork, `s1`–`s8`
// the obstacle halo (here `s`), `p1`–`p8` the wire price (here `p`), `lane`, `fz` the fence, `nb`
// the ring neighbours, `rp` let go of, `ow` the way it had, `fd` the way it found, `fw` and `fwg`
// the feedlines, `ocp` the couplers, `cbx` their box, `src`, `tgt`, `ar`, `prt`. New are the cells
// closed to this search (`cz`), the box (`bx`), the two guards (`tg`, `sg`), the crossing rule
// (`xr`, `xc`), the other wires standing (`sd`) and their clearance (`cl`). The panel lists the
// layers the search holds; one in `OFF` starts switched off, and a layer switched by hand stays so
// from one search to the next.
const GROUPS = [
  [
    "The ground: what the search could enter",
    [
      ["co", "band: the cells the corridor left open, all the search may enter"],
      ["cz", "closed to this search: past the band, the fences, the guards, the length bands, the coupler bodies"],
      ["ob", "artwork and keepout: closed to every search"],
      ["bx", "the box the search was confined to; everything outside it is closed"],
    ],
  ],
  [
    "Guards",
    [
      ["tg", "target guard: in front of every other wire's target port (feedline_target_stub_cells), closed by the clearance around it"],
      ["sg", "source guard: behind every other wire's source port stub (feedline_source_guard_cells), closed by the clearance around it"],
    ],
  ],
  [
    "Prices",
    [
      ["p", "wire price p1–p8: what the search paid a cell, light to dark over the values it holds"],
      ["s", "obstacle halo s1–s8: the price of running beside the artwork, darker is dearer"],
      ["lane", "the lane: the polygon the wire price is measured against; outside it is priced"],
    ],
  ],
  [
    "Crossing rule",
    [
      ["xr", "feedline: to be crossed at a right angle only"],
      ["xc", "feedline curve or end: not to be entered at all"],
    ],
  ],
  [
    "Wires",
    [
      ["fd", "the way it found"],
      ["ow", "the way it had"],
      ["rp", "let go of: crossable unless it is the fence"],
      ["nb", "ring neighbours"],
      ["fz", "fence: the clearance around the fence wires, closed"],
      ["fw", "feedline edges standing"],
      ["fwg", "what the greedy chose for an edge that has no way: never built"],
      ["sd", "the other wires standing"],
      ["cl", "the clearance around the wires standing, to each side"],
      ["wl", "wire names"],
    ],
  ],
  [
    "Couplers",
    [
      ["ocp", "the couplers as they stand: pad and resonator lead"],
      ["cbx", "where a coupler may sit: what the launcher stubs leave open"],
    ],
  ],
  [
    "Ends and ports",
    [
      ["src", "source"],
      ["tgt", "target"],
      ["ar", "the headings they leave and arrive on, a clearance long"],
      ["oe", "the other wires' ends"],
      ["prt", "ports: the cell"],
      ["prx", "ports: the exact position and the orientation"],
      ["prl", "port names"],
    ],
  ],
];
const OFF = new Set(["cl"]);
const switched = new Map();

// The width and height of the grid, and every closed cell as a bit.
const W = grid.width;
const H = grid.height;
const tuning = grid.tuning || {};
const clearance = grid.clearance;
const blockedBits = new Uint8Array(W * H);
for (const [y, x0, n] of grid.blocked)
  blockedBits.fill(1, y * W + x0, y * W + x0 + n);
const inGrid = (x, y) => x >= 0 && y >= 0 && x < W && y < H;
const isBlocked = ([x, y]) => inGrid(x, y) && blockedBits[y * W + x] === 1;

// The runs of closed cells by row, for the walls.
const blockedRows = new Map();
for (const [y, x0, n] of grid.blocked) {
  if (!blockedRows.has(y)) blockedRows.set(y, []);
  blockedRows.get(y).push([x0, n]);
}

// The closed cells as rectangles: a run of a row that repeats exactly on the
// rows above grows over them, so the picture holds few and seamless shapes.
const walls = (() => {
  const open = new Map();
  let out = "";
  const flush = (key, run) => {
    const [x0, n] = key.split("|").map(Number);
    out += `<rect x="${x0}" y="${H - 1 - run.top}" width="${n}" height="${run.top - run.bottom + 1}"/>`;
  };
  for (const y of [...blockedRows.keys()].sort((a, b) => a - b))
    for (const [x0, n] of blockedRows.get(y)) {
      const key = `${x0}|${n}`;
      const run = open.get(key);
      if (run && run.top === y - 1) run.top = y;
      else {
        if (run) flush(key, run);
        open.set(key, { bottom: y, top: y });
      }
    }
  for (const [key, run] of open) flush(key, run);
  return out;
})();

// ------------------------------------------------- the ways standing
// The records of the Final stage carry the ways standing at every search as what changed since
// the search before (`ways`: a wire's `[drawn, bends]`, or null for none; `fresh` where they start
// over). Records without them are replayed instead: every wire stands on the last way it found.
const recorded = searches.some((s) => s.ways);
function standingAt(at) {
  if (!recorded) {
    const standing = standingBefore(at);
    for (const v of standing.values()) v.drawn = true;
    return standing;
  }
  const standing = new Map();
  for (let j = 0; j <= at; j++) {
    const s = searches[j];
    if (s.fresh) standing.clear();
    if (!s.ways) continue;
    for (const [w, v] of Object.entries(s.ways))
      if (v) standing.set(w, { way: v[1], drawn: !!v[0], at: j });
      else standing.delete(w);
  }
  return standing;
}
// What a record says only when it changed — the couplers, the greedy's ways, the crossing rule —
// as it stands at search `at`: the last record at or before it that says it.
function lastSaid(at, key) {
  for (let j = at; j >= 0; j--) if (searches[j][key] !== undefined) return [searches[j][key], j];
  return [undefined, -1];
}

// The two ends of a wire: as its record carries them (with the headings), else as the grid
// record does, else (a feedline edge, made after the grid was recorded) the ends of a way it found.
function endsOf(s) {
  if (s.source && s.target)
    return { source: s.source, target: s.target, guessed: false };
  const end = ends.get(s.wire);
  if (end) return { source: end.source, target: end.target, guessed: false };
  const at = lastFound.get(s.wire);
  if (at === undefined) return null;
  const way = searches[at].way;
  return { source: way[0], target: way[way.length - 1], guessed: true };
}

// ------------------------------------------------------- the ground
// The cells each search found open and the price it read are not in this file: they lie beside
// it, a script per `chunk` searches, loaded when one of them is opened.
const groundMeta = (() => {
  try {
    return JSON.parse(document.getElementById("ground-data").textContent);
  } catch {
    return { dir: null, chunk: 16 };
  }
})();
const host = typeof window !== "undefined" ? window : globalThis;
const chunks = new Map();
const waiting = new Map();
host.__ground = (k, list) => {
  const done = waiting.get(k);
  if (done) done(list);
};
function groundOf(at) {
  if (!groundMeta.dir || !document.createElement) return Promise.resolve(null);
  const k = Math.floor(at / groundMeta.chunk);
  if (!chunks.has(k))
    chunks.set(
      k,
      new Promise((resolve) => {
        waiting.set(k, resolve);
        const script = document.createElement("script");
        script.onerror = () => resolve(null);
        script.src = `${groundMeta.dir}/${k}.js`;
        (document.head || document.body).appendChild(script);
      }),
    );
  return chunks.get(k).then((list) => (list ? list[at % groundMeta.chunk] || null : null));
}

// The open cells, in the form of the stage's `contourOf`: per row of the box, the columns at which
// the cells flip, as decimals, or as letters for how far each flip moved from the row below.
function flipsOf(text) {
  let below = [];
  return text.split(";").map((row) => {
    let flips;
    if (row === "") flips = [];
    else if (/^[a-z]+$/.test(row))
      flips = Array.from(row, (c, j) => below[j] + c.charCodeAt(0) - 110);
    else flips = row.split(",").map(Number);
    below = flips;
    return flips;
  });
}
// The price, in the form of the stage's `priceOf`: per block row, the columns at which the price
// changes and the price from there on, or letters for how far each change moved.
function changesOf(text) {
  let below = [];
  return text.split(";").map((row) => {
    let changes;
    if (row === "") changes = [];
    else if (/^[a-z]+$/.test(row))
      changes = below.map(([x, v], j) => [x + row.charCodeAt(j) - 110, v]);
    else changes = row.split(",").map((t) => t.split(":").map(Number));
    below = changes;
    return changes;
  });
}
// The ground of one search, read: whether a cell was open and what it cost.
function readGround(s, raw) {
  if (!raw || !s.box) return null;
  const [minX, minY, maxX, maxY] = s.box;
  const block = s.priceBlock || 1;
  const open = typeof raw.open === "string" ? flipsOf(raw.open) : null;
  const price = typeof raw.price === "string" ? changesOf(raw.price) : null;
  const values = new Set();
  if (price) for (const row of price) for (const [, v] of row) if (v) values.add(v);
  const shades = [...values].sort((a, b) => a - b);
  return {
    minX,
    minY,
    maxX,
    maxY,
    block,
    open,
    price,
    shades,
    isOpen(x, y) {
      if (!open || x < minX || x > maxX || y < minY || y > maxY) return false;
      const rx = x - minX;
      let inside = false;
      for (const f of open[y - minY]) {
        if (f > rx) break;
        inside = !inside;
      }
      return inside;
    },
    priceAt(x, y) {
      if (!price || x < minX || x > maxX || y < minY || y > maxY) return 0;
      const row = price[Math.floor((y - minY) / block)] || [];
      const bx = Math.floor((x - minX) / block);
      let v = 0;
      for (const [c, value] of row) {
        if (c > bx) break;
        v = value;
      }
      return v;
    },
  };
}

// A cell layer as a picture: one pixel a cell (or a block of cells), drawn into a canvas and
// placed over the cells it covers, crisp at every zoom. `paint(x, y)` returns the colour of a pixel
// as [r, g, b, a] or nothing; `y` counts from the top of the image.
const ALPHA = [0, 0.1, 0.15, 0.2, 0.25, 0.31, 0.37, 0.44, 0.52];
const rgb = (hex) => [1, 3, 5].map((at) => parseInt(hex.slice(at, at + 2), 16));
function image(cls, w, h, at, paint, tip) {
  if (!document.createElement || w <= 0 || h <= 0) return "";
  const canvas = document.createElement("canvas");
  if (!canvas || !canvas.getContext) return "";
  canvas.width = w;
  canvas.height = h;
  const context = canvas.getContext("2d");
  if (!context) return "";
  const pixels = context.createImageData(w, h);
  const data = pixels.data;
  for (let y = 0; y < h; y++)
    for (let x = 0; x < w; x++) {
      const colour = paint(x, y);
      if (!colour) continue;
      const i = (y * w + x) * 4;
      data[i] = colour[0];
      data[i + 1] = colour[1];
      data[i + 2] = colour[2];
      data[i + 3] = colour[3];
    }
  context.putImageData(pixels, 0, 0);
  const [x, top, width, height] = at;
  return (
    `<image class="${cls}" x="${x}" y="${top}" width="${width}" height="${height}" ` +
    `preserveAspectRatio="none" href="${canvas.toDataURL()}">${titled(tip)}</image>`
  );
}
const shade = (hex, level) => [...rgb(hex), Math.round(255 * ALPHA[level])];

// The obstacle halo of the whole grid, once: the router's static price, as eight levels of the
// most it reaches (the old `levelOf`).
const haloValues = (() => {
  const values = new Uint8Array(W * H);
  for (const row of grid.halo || [])
    for (let j = 1; j < row.length; j += 3)
      values.fill(row[j + 2], row[0] * W + row[j], row[0] * W + row[j] + row[j + 1]);
  return values;
})();
const haloMost = Math.max(1, tuning.obstaclePrice || 0);
const levelOf = (value, most) =>
  value <= 0 ? 0 : 1 + Math.min(7, Math.floor((7 * value) / Math.max(1, most)));
let haloImage = null;
function halo() {
  if (haloImage === null) {
    const colours = ALPHA.map((_, level) => shade("#E69F00", level));
    haloImage = (grid.halo || []).length
      ? image(
          "s",
          W,
          H,
          [0, 0, W, H],
          (x, y) => {
            const v = haloValues[(H - 1 - y) * W + x];
            return v ? colours[levelOf(v, haloMost)] : null;
          },
          `the obstacle halo: ${tuning.obstaclePrice} at the artwork, falling off over ${tuning.obstacleReach} cells`,
        )
      : "";
  }
  return haloImage;
}

// The crossing rule of the feedline phase, by the search that carried it.
const crossingImages = new Map();
function crossing(j, rule) {
  if (!crossingImages.has(j)) {
    const values = new Uint8Array(W * H);
    for (const row of rule)
      for (let k = 1; k < row.length; k += 3)
        values.fill(row[k + 2], row[0] * W + row[k], row[0] * W + row[k] + row[k + 1]);
    const right = [...rgb("#F0E442"), 110];
    const none = [...rgb("#D55E00"), 80];
    crossingImages.set(
      j,
      image("xr", W, H, [0, 0, W, H], (x, y) => (values[(H - 1 - y) * W + x] === 1 ? right : null), "a feedline: crossed at a right angle only") +
        image("xc", W, H, [0, 0, W, H], (x, y) => (values[(H - 1 - y) * W + x] === 2 ? none : null), "a feedline's curve or end: not to be entered"),
    );
  }
  return crossingImages.get(j);
}

// The cell layers of one search: the band, the cells closed to it and the price it read.
function groundImages(g) {
  const w = g.maxX - g.minX + 1;
  const h = g.maxY - g.minY + 1;
  const at = [g.minX, H - 1 - g.maxY, w, h];
  let out = "";
  if (g.open) {
    const open = new Uint8Array(w * h);
    g.open.forEach((flips, row) => {
      const top = h - 1 - row;
      for (let k = 0; k + 1 < flips.length; k += 2) open.fill(1, top * w + flips[k], top * w + flips[k + 1]);
    });
    const band = [...rgb("#56B4E9"), Math.round(255 * 0.26)];
    const shut = [...rgb("#333a40"), Math.round(255 * 0.2)];
    out += image("co", w, h, at, (x, y) => (open[y * w + x] ? band : null), "band: open to the search");
    out += image(
      "cz",
      w,
      h,
      at,
      (x, y) => (!open[y * w + x] && !blockedBits[(g.maxY - y) * W + g.minX + x] ? shut : null),
      "closed to this search",
    );
  }
  if (g.price && g.shades.length) {
    const across = Math.floor((g.maxX - g.minX) / g.block) + 1;
    const down = g.price.length;
    const values = new Uint16Array(across * down);
    g.price.forEach((changes, row) => {
      const top = down - 1 - row;
      changes.forEach(([c, v], k) => {
        const end = k + 1 < changes.length ? changes[k + 1][0] : across;
        if (v) values.fill(v, top * across + c, top * across + Math.min(end, across));
      });
    });
    const rank = new Map(g.shades.map((v, k) => [v, 1 + Math.min(7, k)]));
    const colours = ALPHA.map((_, level) => shade("#CC79A7", level));
    out += image(
      "p",
      across,
      down,
      [g.minX, H - (g.minY + down * g.block), across * g.block, down * g.block],
      (x, y) => {
        const v = values[y * across + x];
        return v ? colours[rank.get(v)] : null;
      },
      `the wire price, over the values ${g.shades.join(", ")}`,
    );
  }
  return out;
}

// The grid's y grows upward and an SVG's downward: every y is flipped, so the
// picture stands as `mqt-scpd plot` draws the chip.
const X = (x) => x + 0.5;
const Y = (y) => H - 0.5 - y;
const titled = (tip) => (tip ? `<title>${esc(tip)}</title>` : "");
const dot = ([x, y], cls, tip) =>
  `<path class="${cls}" d="M${X(x)} ${Y(y)}h0">${titled(tip)}</path>`;
const pts = (way) => way.map(([x, y]) => `${X(x)},${Y(y)}`).join(" ");
const line = (way, cls, tip, more = "") =>
  way && way.length > 1
    ? `<polyline class="${cls}" points="${pts(way)}"${more}>${titled(tip)}</polyline>`
    : "";
const words = ([x, y], what, cls) =>
  `<text class="${cls}" x="${X(x) + 1}" y="${Y(y) - 1}">${esc(what)}</text>`;
const rect = (box, cls, tip) =>
  `<rect class="${cls}" x="${box[0]}" y="${H - 1 - box[3]}" width="${box[2] - box[0] + 1}" height="${box[3] - box[1] + 1}">${titled(tip)}</rect>`;

// The distance in cells from a cell to a way, through its bends.
function distance([px, py], way) {
  if (way.length === 1) return Math.hypot(px - way[0][0], py - way[0][1]);
  let best = Infinity;
  for (let at = 1; at < way.length; at++) {
    const [ax, ay] = way[at - 1];
    const [bx, by] = way[at];
    const dx = bx - ax;
    const dy = by - ay;
    const len = dx * dx + dy * dy;
    const t = len
      ? Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / len))
      : 0;
    best = Math.min(best, Math.hypot(px - (ax + t * dx), py - (ay + t * dy)));
  }
  return best;
}

// The guards that hold a cell: those whose centre line lies within the clearance of it.
function guardsAt(s, cell) {
  const out = [];
  for (const [kind, name] of [
    ["target", "target guard"],
    ["source", "source guard"],
  ])
    for (const g of (s.guards && s.guards[kind]) || [])
      if (distance(cell, [g.slice(0, 2), g.slice(2, 4)]) <= clearance + 0.5)
        out.push(`the ${name} of ${g[4] === null ? "a wire" : g[4]}`);
  return out;
}

// What the records say of why a search came to what it came to, as HTML items.
function reasons(at, s, standing, own, others, g) {
  const out = [];
  if (!s.found)
    out.push(
      "<b>no way</b>: the search found no way from its source to its target past the closed cells and the wires standing",
    );
  else
    out.push(
      `it found a way of ${s.cells} cells in ${Math.max(0, s.way.length - 1)} straight runs`,
    );
  if (s.note) out.push(`the lengthening: ${esc(s.note)}`);
  if (s.ripped.length) {
    const none = s.ripped.filter((w) => !standing.has(w));
    out.push(
      `it let go of ${s.ripped.map(esc).join(", ")} (long-dashed)` +
        (none.length
          ? `; ${none.map(esc).join(", ")} had no way standing`
          : ""),
    );
  }
  if (own) {
    for (const [what, cell] of [
      ["source", own.source],
      ["target", own.target],
    ]) {
      if (isBlocked(cell)) {
        const alike = grid.wires.filter(
          (w) => w.wire !== s.wire && isBlocked(w[what]),
        );
        const routed = alike.filter((w) => atTheEnd.has(w.wire)).length;
        out.push(
          `its ${what} (${cell.slice(0, 2).join(", ")}) lies on a closed cell of the grid; so does the ${what} of ` +
            `${alike.length} other wires, ${routed} of which stand at the end of the phase`,
        );
      }
      if (g && !g.isOpen(cell[0], cell[1]))
        out.push(`<b>its ${what} (${cell.slice(0, 2).join(", ")}) is closed to this search</b>`);
      const guarded = guardsAt(s, cell);
      if (guarded.length)
        out.push(`<b>its ${what} lies in ${guarded.map(esc).join(", ")}</b>`);
      const near = others
        .map(([w, v]) => [w, distance(cell, v.way)])
        .sort((a, b) => a[1] - b[1]);
      const inside = near.filter(([, d]) => d <= clearance);
      if (inside.length)
        out.push(
          `<b>its ${what} lies within the clearance (${clearance} cells) of ` +
            inside
              .map(([w, d]) => `${esc(w)} (${d.toFixed(0)} cells)`)
              .join(", ") +
            "</b>",
        );
      else if (near.length)
        out.push(
          `the wire standing nearest to its ${what}: ${esc(near[0][0])}, ${near[0][1].toFixed(0)} cells away`,
        );
    }
    if (own.guessed)
      out.push(
        "its ends are those of a way it found: the grid record holds no feedline edge",
      );
  } else
    out.push(
      "its ends are not recorded: the grid record holds no feedline edge, and this one never found a way",
    );
  // The round around it: what the other levels of the same round came to.
  const round = [];
  searches.forEach((t, j) => {
    if (t.wire === s.wire && t.pass === s.pass && t.round === s.round)
      round.push([t, j]);
  });
  const good = round.find(([t, j]) => t.found && j > at);
  if (!s.found && good)
    out.push(
      `the <a href="#search-${good[1]}">${esc(good[0].kind)}</a> search of this round found a way` +
        (good[0].ripped.length
          ? ` after letting go of ${good[0].ripped.map(esc).join(", ")}`
          : ""),
    );
  else if (!round.some(([t]) => t.found))
    out.push(
      `<b>this round gave up: none of its ${round.length} searches of ${esc(s.wire)} found a way</b>`,
    );
  return out;
}

// What the search was set: its box and band, the stubs and the bend, what fenced it and what it
// let go of, the guards and the rules' figures.
function setting(s) {
  const out = [];
  const name = (w) => (w === null || w === undefined ? "-" : esc(w));
  if (s.box)
    out.push(
      `box x ${s.box[0]}..${s.box[2]}, y ${s.box[1]}..${s.box[3]}` +
        (s.reach ? ` · band ${s.reach} cells around the way it had` : ""),
    );
  if (s.params)
    out.push(
      `stubs: ${s.params.start} cells off its source, ${s.params.end} into its target · bend ${s.params.bend}`,
    );
  if (s.fence)
    out.push(
      `fence ${s.fence.length ? s.fence.map(esc).join(", ") : "-"} · ring neighbours ${name(s.before)} and ${name(s.after)}` +
        (s.bridge ? ` · it has to cross ${esc(s.bridge)}` : ""),
    );
  if (s.guards)
    out.push(
      `guards closed here: ${s.guards.target.length} in front of target ports (${tuning.targetStub} cells), ` +
        `${s.guards.source.length} behind source port stubs (${tuning.sourceGuard} cells)`,
    );
  if (s.crossing) out.push("it obeys the crossing rule of the feedlines");
  if (grid.tuning)
    out.push(
      `clearance ${clearance} cells · wire price ${tuning.wirePrice} · obstacle price ${tuning.obstaclePrice} ` +
        `over ${tuning.obstacleReach} cells · halo of a wire let go of ${tuning.haloReach} clearance · bend ${tuning.bendPenalty}`,
    );
  return out;
}

// The vector layers of a search, bottom up; the cell layers go into `#g-under` once its ground is
// loaded. `have` collects the layers drawn, for the panel.
function picture(at, s, standing, own, others, have) {
  const add = (cls, text) => {
    if (text) have.add(cls);
    return text;
  };
  const c2 = 2 * clearance + 1;
  let svg = `<rect class="bg" x="0" y="0" width="${W}" height="${H}"/><g id="g-under"></g>`;
  svg += add("ob", walls && `<g class="ob">${walls}</g>`);
  if ((grid.halo || []).length) have.add("s");
  if (s.box) {
    have.add("co").add("cz");
    if (s.price) have.add("p");
  }
  const [rule] = lastSaid(at, "crossingRule");
  if (s.crossing && rule && rule.length) have.add("xr").add("xc");
  // The guards, inflated by the clearance as the corridor closes them.
  for (const [kind, cls] of [
    ["target", "tg"],
    ["source", "sg"],
  ]) {
    const list = (s.guards && s.guards[kind]) || [];
    let g = "";
    for (const [x0, y0, x1, y1, owner] of list) {
      const tip = `the ${kind} guard of ${owner === null ? "a wire" : owner}: (${x0}, ${y0}) to (${x1}, ${y1})`;
      g +=
        `<polyline class="band" points="${X(x0)},${Y(y0)} ${X(x1)},${Y(y1)}" stroke-width="${c2}">${titled(tip)}</polyline>` +
        `<polyline class="mid" points="${X(x0)},${Y(y0)} ${X(x1)},${Y(y1)}"/>`;
    }
    svg += add(cls, g && `<g class="${cls}">${g}</g>`);
  }
  // The fences: the clearance around each fence wire's way.
  const fence = (s.fence || []).filter((w) => standing.has(w));
  let fz = "";
  for (const w of fence) {
    const v = standing.get(w);
    fz += line(v.way, "band", `fence ${w}`, ` stroke-width="${c2}"`) + line(v.way, "mid", "");
  }
  svg += add("fz", fz && `<g class="fz">${fz}</g>`);
  let cl = "";
  for (const [w, v] of others) cl += line(v.way, "", `the clearance of ${w}`);
  svg += add("cl", cl && `<g class="cl" stroke-width="${c2}">${cl}</g>`);
  if (s.lane && s.lane.length > 2)
    svg += add(
      "lane",
      `<polygon class="lane" points="${pts(s.lane)}">${titled(`the lane between ${s.before} and ${s.after}`)}</polygon>`,
    );
  if (grid.couplerBox) svg += add("cbx", rect(grid.couplerBox, "cbx", "where a coupler may sit"));
  if (s.box) svg += add("bx", rect(s.box, "bx", `the box of the search: x ${s.box[0]}..${s.box[2]}, y ${s.box[1]}..${s.box[3]}`));
  const [couplers] = lastSaid(at, "couplers");
  let ocp = "";
  for (const c of couplers || []) {
    ocp += `<polygon points="${pts(c.pad)}">${titled(`the coupler of ${c.wire}`)}</polygon>`;
    ocp += line(c.lead, "", `the lead of ${c.wire}`);
  }
  svg += add("ocp", ocp && `<g class="ocp">${ocp}</g>`);
  const [greedy] = lastSaid(at, "greedy");
  for (const way of greedy || []) svg += add("fwg", line(way, "fwg", "what the greedy chose: never built"));
  const ring = new Set([s.before, s.after]);
  for (const [w, v] of others) {
    const cls = isFeedline(w) ? "fw" : ring.has(w) ? "nb" : "sd";
    const when = v.at === undefined ? "" : ` (as it stood at search ${v.at}, ${where(searches[v.at])})`;
    svg += add(cls, line(v.way, cls, `${w}${when}`));
  }
  const ripped = s.ripped
    .filter((w) => standing.has(w))
    .map((w) => [w, standing.get(w)]);
  for (const [w, v] of ripped) svg += add("rp", line(v.way, "rp", `${w}, let go of`));
  const had = s.had && s.had.length ? s.had : (standing.get(s.wire) || {}).way;
  svg += add("ow", line(had, "ow", `the way ${s.wire} had`));
  svg += add("fd", line(s.way, "fd", `the way it found: ${s.cells} cells`));
  // The other wires' ends: as the grid record carries them, and the feedline edges' as their ways.
  for (const w of grid.wires)
    if (w.wire !== s.wire)
      svg += add("oe", dot(w.source, "oe", `${w.wire}: source`) + dot(w.target, "oe", `${w.wire}: target`));
  for (const [w, v] of standing)
    if (isFeedline(w) && w !== s.wire && v.way.length)
      svg += add("oe", dot(v.way[0], "oe", `${w}: source`) + dot(v.way[v.way.length - 1], "oe", `${w}: target`));
  // The ports: the cell, the exact position and the orientation, the name.
  for (const p of grid.ports) svg += add("prt", dot([p.x, p.y], "prt", p.label));
  let prx = "";
  for (const p of grid.ports)
    if (p.fx !== undefined) {
      const x = p.fx + 0.5;
      const y = H - 0.5 - p.fy;
      prx +=
        `<path d="M${x} ${y}h0"><title>${esc(`${p.label}: port ${p.index}, exactly at (${p.fx.toFixed(2)}, ${p.fy.toFixed(2)})`)}</title></path>` +
        (p.dx || p.dy ? `<line x1="${x}" y1="${y}" x2="${x + p.dx * clearance}" y2="${y - p.dy * clearance}"/>` : "");
    }
  svg += add("prx", prx && `<g class="prx">${prx}</g>`);
  for (const p of grid.ports) svg += add("prl", words([p.x, p.y], p.label.replace(/^Chip\./, ""), "prl"));
  for (const [w, v] of [...others, ...ripped]) {
    const mid = v.way[Math.floor(v.way.length / 2)];
    if (mid) svg += add("wl", words(mid, w, "wl"));
  }
  if (own) {
    const [sx, sy, sdx, sdy] = own.source;
    const [tx, ty, tdx, tdy] = own.target;
    if (sdx !== undefined) {
      svg += add(
        "ar",
        `<g class="ar">${line([[sx, sy], [sx + sdx * clearance, sy + sdy * clearance]], "", "the heading it leaves on")}` +
          `${line([[tx - tdx * clearance, ty - tdy * clearance], [tx, ty]], "", "the heading it arrives on")}</g>`,
      );
    }
    svg += add("src", dot([sx, sy], "src", `source of ${s.wire}: ${sx}, ${sy}`));
    svg += add("tgt", dot([tx, ty], "tgt", `target of ${s.wire}: ${tx}, ${ty}`));
    svg += add("wl", words([tx, ty], s.wire, "wl"));
  }
  return svg;
}

// The box a search is looked at in: the box it was confined to, else its ends, the way it found
// and the way it had, with room around them; as an SVG viewBox.
function focusBox(s, own, had) {
  if (s.box) {
    const pad = 2 * clearance;
    const minX = Math.max(0, s.box[0] - pad);
    const maxX = Math.min(W - 1, s.box[2] + pad);
    const minY = Math.max(0, s.box[1] - pad);
    const maxY = Math.min(H - 1, s.box[3] + pad);
    return [minX, H - 1 - maxY, maxX - minX + 1, maxY - minY + 1];
  }
  const cells = [...s.way, ...(had ? had.way : [])];
  if (own) cells.push(own.source, own.target);
  if (!cells.length) return null;
  const xs = cells.map((p) => p[0]);
  const ys = cells.map((p) => p[1]);
  const pad = Math.max(
    4 * clearance,
    0.15 *
      Math.max(
        Math.max(...xs) - Math.min(...xs),
        Math.max(...ys) - Math.min(...ys),
      ),
  );
  const minX = Math.max(0, Math.min(...xs) - pad);
  const maxX = Math.min(W - 1, Math.max(...xs) + pad);
  const minY = Math.max(0, Math.min(...ys) - pad);
  const maxY = Math.min(H - 1, Math.max(...ys) + pad);
  return [minX, H - 1 - maxY, maxX - minX + 1, maxY - minY + 1];
}

const detailElement = document.getElementById("detail");
const baseTitle = document.title;
const view = { box: null, focus: null, at: -1, ground: null };

function panel(have) {
  let groups = "";
  for (const [group, entries] of GROUPS) {
    const rows = entries.filter(([cls]) => have.has(cls));
    if (!rows.length) continue;
    const key = group.replace(/\W+/g, "");
    groups +=
      `<fieldset><legend>${esc(group)}</legend><div class="bulk">` +
      `<button type="button" data-group="${key}" data-state="1">on</button>` +
      `<button type="button" data-group="${key}" data-state="0">off</button></div>` +
      rows
        .map(([cls, what]) => {
          const on = switched.has(cls) ? switched.get(cls) : !OFF.has(cls);
          return (
            `<label><input type="checkbox"${on ? " checked" : ""} data-cls="${cls}" data-group="${key}">` +
            `<i class="sw ${cls}"></i><span>${esc(what)} <code>${cls}</code></span></label>`
          );
        })
        .join("") +
      "</fieldset>";
  }
  return groups;
}

function detail(at) {
  const s = searches[at];
  const standing = standingAt(at);
  const own = endsOf(s);
  const others = [...standing].filter(
    ([w, v]) => w !== s.wire && !s.ripped.includes(w) && v.drawn,
  );
  const look = outcome(s);
  const have = new Set();
  const svg = picture(at, s, standing, own, others, have);
  // The searches of this wire in the phase, the open one marked.
  const mine = [];
  searches.forEach((t, j) => {
    if (t.wire === s.wire) mine.push([t, j]);
  });
  const index = mine.findIndex(([, j]) => j === at);
  const step = (by, name) => {
    const other = mine[index + by];
    return other
      ? `<a href="#search-${other[1]}">${name}</a>`
      : `<span class="off">${name}</span>`;
  };
  let list = '<table class="mine">';
  for (const [t, j] of mine)
    list +=
      `<tr${j === at ? ' class="here"' : ""}><td><i class="chip" style="background:${LOOK[outcome(t)][0]}"></i></td>` +
      `<td><a href="#search-${j}">${esc(where(t))}</a></td><td>${esc(t.found ? `${t.cells} cells` : "no way")}</td></tr>`;
  list += "</table>";
  const why = reasons(at, s, standing, own, others, null)
    .map((r) => `<li>${r}</li>`)
    .join("");
  const set = setting(s)
    .map((r) => `<li>${r}</li>`)
    .join("");
  const standingNote = recorded
    ? "The wires standing are those the record of this search carries: the ways as they stood when it ran."
    : "The wires standing are replayed from the searches of this phase: a wire stands on the last way it " +
      "found, and a search that found its way took the ways of the wires it let go of. A wire the phase has not " +
      "searched yet is not drawn.";
  view.focus = focusBox(s, own, standing.get(s.wire));
  view.at = at;
  view.ground = null;
  detailElement.innerHTML =
    `<div id="dpanel"><nav class="links"><a href="#table">← the table</a>` +
    `${step(-1, "‹ the search before")}${step(1, "the search after ›")}</nav>` +
    `<h1>wire ${esc(s.wire)} · search ${at}</h1><p class="sub">${esc(where(s))}</p>` +
    `<p><span class="badge" style="background:${LOOK[look][0]}">${esc(LOOK[look][1])}</span></p>` +
    `<h2>What the records say</h2><ul class="why" id="why">${why}</ul>` +
    (set ? `<h2>What the search was set</h2><ul class="why">${set}</ul>` : "") +
    `<p class="sub">${standingNote}</p>` +
    `<p class="sub" id="ground-state">${s.box ? "loading the cells of this search…" : ""}</p>` +
    '<div class="bulk"><button type="button" data-all="1">all on</button><button type="button" data-all="0">all off</button>' +
    `<button type="button" data-view="focus"${view.focus ? "" : " disabled"}>the search</button>` +
    '<button type="button" data-view="chip">the whole chip</button></div>' +
    '<label><span>Zoom</span></label><input id="zoom" type="range" min="300" max="16000" step="50" value="1200">' +
    `<p class="sub" id="at">the cell under the pointer</p>${panel(have)}` +
    `<h2>The searches of wire ${esc(s.wire)}</h2>${list}</div>` +
    `<div id="dstage"><svg id="picture" xmlns="http://www.w3.org/2000/svg">${svg}</svg></div>`;
  document.title = `wire ${s.wire} · ${where(s)} — ${baseTitle}`;
  show(view.focus || [0, 0, W, H]);
  apply();
  // The cell layers: the halo and the crossing rule at once, the band, the closed cells and the
  // price once the ground of this search is loaded.
  const under = () => document.getElementById("g-under");
  const [crossingRule, j] = lastSaid(at, "crossingRule");
  const shared = halo() + (s.crossing && crossingRule && crossingRule.length ? crossing(j, crossingRule) : "");
  if (under()) under().innerHTML = shared;
  if (!s.box) return Promise.resolve();
  return groundOf(at).then((raw) => {
    if (view.at !== at) return;
    const g = readGround(s, raw);
    const state = document.getElementById("ground-state");
    if (!g) {
      if (state) state.textContent = "the cells of this search are not beside this page";
      return;
    }
    view.ground = g;
    if (under()) under().innerHTML = shared + groundImages(g);
    if (state)
      state.textContent =
        `the cells of the box: the band, the closed cells and the price` +
        (g.shades.length ? ` (the wire price over the values ${g.shades.join(", ")}` + `${g.block > 1 ? `, in blocks of ${g.block} cells` : ""})` : "");
    const list = document.getElementById("why");
    if (list)
      list.innerHTML = reasons(at, s, standing, own, others, g)
        .map((r) => `<li>${r}</li>`)
        .join("");
  });
}

// The box the picture shows, its width on the screen and the size of its labels.
function show(box) {
  view.box = box;
  const svg = document.getElementById("picture");
  const stage = document.getElementById("dstage");
  const zoom = document.getElementById("zoom");
  if (!svg || !zoom) return;
  svg.setAttribute("viewBox", box.join(" "));
  svg.style.setProperty("--fs", `${Math.max(2, box[2] / 90)}px`);
  // The whole box in the stage at first; the zoom makes it larger.
  const w = ((stage && stage.clientWidth) || 1200) - 4;
  const h = ((stage && stage.clientHeight) || 900) - 4;
  const width = Math.max(300, Math.round(Math.min(w, (h * box[2]) / box[3])));
  zoom.value = String(width);
  size(width);
}
function size(width) {
  const svg = document.getElementById("picture");
  if (!svg || !view.box) return;
  svg.setAttribute("width", String(width));
  svg.setAttribute(
    "height",
    String(Math.round((width * view.box[3]) / view.box[2])),
  );
}
function apply() {
  const boxes = [...detailElement.querySelectorAll("input[data-cls]")];
  document.getElementById("hidden").textContent = boxes
    .filter((b) => !b.checked)
    .map((b) => `#picture .${b.dataset.cls}{display:none!important}`)
    .join("");
}
detailElement.addEventListener("change", (e) => {
  if (e.target.dataset && e.target.dataset.cls) {
    switched.set(e.target.dataset.cls, e.target.checked);
    apply();
  }
});
detailElement.addEventListener("input", (e) => {
  if (e.target.id === "zoom") size(Number(e.target.value));
});
detailElement.addEventListener("click", (e) => {
  const b = e.target.closest("button");
  if (!b) return;
  const boxes = [...detailElement.querySelectorAll("input[data-cls]")];
  const set = (x, on) => {
    x.checked = on;
    switched.set(x.dataset.cls, on);
  };
  if (b.dataset.all !== undefined)
    for (const x of boxes) set(x, b.dataset.all === "1");
  else if (b.dataset.group !== undefined)
    for (const x of boxes)
      if (x.dataset.group === b.dataset.group) set(x, b.dataset.state === "1");
  if (b.dataset.view === "focus" && view.focus) show(view.focus);
  if (b.dataset.view === "chip") show([0, 0, W, H]);
  apply();
});
detailElement.addEventListener("mousemove", (e) => {
  const svg = e.target.closest && e.target.closest("#picture");
  const out = document.getElementById("at");
  if (!svg || !out || !svg.getScreenCTM) return;
  const point = svg.createSVGPoint();
  point.x = e.clientX;
  point.y = e.clientY;
  const p = point.matrixTransform(svg.getScreenCTM().inverse());
  const x = Math.floor(p.x);
  const y = H - 1 - Math.floor(p.y);
  const said = [`cell ${x}, ${y}`];
  if (isBlocked([x, y])) said.push("artwork or keepout");
  const g = view.ground;
  if (g) {
    said.push(g.isOpen(x, y) ? "open to the search" : "closed to the search");
    const price = g.priceAt(x, y);
    if (price) said.push(`wire price ${price}`);
  }
  if (inGrid(x, y) && haloValues[y * W + x]) said.push(`obstacle price ${haloValues[y * W + x]}`);
  const s = searches[view.at];
  if (s) said.push(...guardsAt(s, [x, y]));
  out.textContent = said.join(" · ");
});

// ------------------------------------------------------------ the two views
function route() {
  const m = /^#search-(\d+)$/.exec(location.hash);
  if (m && searches[Number(m[1])]) {
    document.body.classList.add("detail");
    detailElement.hidden = false;
    detail(Number(m[1]));
  } else {
    document.body.classList.remove("detail");
    detailElement.hidden = true;
    detailElement.innerHTML = "";
    document.title = baseTitle;
  }
}
addEventListener("hashchange", route);
route();
