const grid = JSON.parse(document.getElementById("grid-data").textContent);
const searches = JSON.parse(document.getElementById("search-data").textContent);
const LOOK = {"gave-up": ["#D55E00", "no way, not even after the relaxation"],
              "no-way": ["#E69F00", "this search found nothing"],
              "short": ["#CC79A7", "a way found, but not to be made its length"],
              "relaxed": ["#56B4E9", "only after a neighbour was let go of"],
              "found": ["#009E73", "a way found"]};
function classify(s) {
  if (!s.found) return "no-way";
  if (/no room for a meander|too long for its target|short/.test(s.note)) return "short";
  return s.kind.startsWith("relax") ? "relaxed" : "found";
}
// Columns: one per pass, round, direction and kind of search, in the order they were made.
const columns = [], index = new Map(), wires = [], seen = new Map();
for (const s of searches) {
  const key = `${s.pass}|${s.round}|${s.forward ? "f" : "b"}|${s.kind}`;
  if (!index.has(key)) { index.set(key, columns.length); columns.push({key, pass: s.pass, round: s.round, forward: s.forward, kind: s.kind}); }
  if (!seen.has(s.wire)) { seen.set(s.wire, wires.length); wires.push(s.wire); }
}
const cells = new Map();
searches.forEach((s, at) => {
  const key = `${s.wire}|${index.get(`${s.pass}|${s.round}|${s.forward ? "f" : "b"}|${s.kind}`)}`;
  cells.set(key, at);  // the last search of a wire in a column stands
});
const esc = t => String(t).replace(/[&<>"]/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));
let h = "<table><tr><th>wire</th>";
for (const c of columns) h += `<th>${esc(c.pass)} r${c.round}${c.forward ? "" : "b"}<br>${esc(c.kind)}</th>`;
h += "</tr>";
for (const w of wires) {
  h += `<tr><td><code>${esc(w)}</code></td>`;
  columns.forEach((c, at) => {
    const found = cells.get(`${w}|${at}`);
    if (found === undefined) { h += "<td></td>"; return; }
    const s = searches[found], cls = classify(s);
    h += `<td><a href="#" data-at="${found}" title="${esc(LOOK[cls][1])}" style="display:block;width:16px;height:14px;border-radius:2px;background:${LOOK[cls][0]}"></a></td>`;
  });
  h += "</tr>";
}
h += "</table>";
let key = '<p class="sub">';
for (const k in LOOK) key += `<span style="margin-right:14px"><i style="display:inline-block;width:12px;height:10px;border-radius:2px;background:${LOOK[k][0]};margin-right:4px"></i>${esc(LOOK[k][1])}</span>`;
document.getElementById("board").innerHTML = key + "</p>" + h;
document.getElementById("board").addEventListener("click", e => {
  const a = e.target.closest("a[data-at]");
  if (!a) return;
  e.preventDefault();
  show(searches[Number(a.dataset.at)]);
});
function show(s) {
  const panel = document.getElementById("panel");
  panel.hidden = false;
  document.getElementById("panel-title").textContent = `wire ${s.wire} · ${s.pass} round ${s.round} ${s.forward ? "forward" : "backward"} · ${s.kind}`;
  document.getElementById("panel-text").textContent = (s.found ? `found ${s.cells} cells` : "no way") + (s.note ? ` · ${s.note}` : "") + (s.ripped.length ? ` · let go of ${s.ripped.join(", ")}` : "");
  const svg = document.getElementById("picture");
  const xs = s.way.map(p => p[0]), ys = s.way.map(p => p[1]);
  const pad = Math.max(10, grid.clearance * 2);
  const minX = Math.max(0, Math.min(...xs, Infinity) - pad), maxX = Math.min(grid.width - 1, Math.max(...xs, -Infinity) + pad);
  const minY = Math.max(0, Math.min(...ys, Infinity) - pad), maxY = Math.min(grid.height - 1, Math.max(...ys, -Infinity) + pad);
  const box = s.way.length ? [minX, minY, maxX - minX + 1, maxY - minY + 1] : [0, 0, grid.width, grid.height];
  svg.setAttribute("viewBox", box.join(" "));
  let out = `<rect x="${box[0]}" y="${box[1]}" width="${box[2]}" height="${box[3]}" fill="#f6f6f6"/>`;
  for (const [y, x0, n] of grid.blocked) {
    if (y < box[1] || y > box[1] + box[3] || x0 + n < box[0] || x0 > box[0] + box[2]) continue;
    out += `<rect x="${x0}" y="${y}" width="${n}" height="1" fill="#8c8780"/>`;
  }
  for (const w of grid.wires) {
    if (w.wire === s.wire) {
      out += `<circle cx="${w.source[0] + .5}" cy="${w.source[1] + .5}" r="1.5" fill="#0072B2"/><rect x="${w.target[0] - 1}" y="${w.target[1] - 1}" width="3" height="3" fill="#D55E00"/>`;
    }
  }
  if (s.way.length) out += `<polyline points="${s.way.map(p => `${p[0] + .5},${p[1] + .5}`).join(" ")}" fill="none" stroke="#009E73" stroke-width="1" vector-effect="non-scaling-stroke"/>`;
  svg.innerHTML = out;
}
