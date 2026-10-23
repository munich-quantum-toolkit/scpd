# Übergabe: Stub-Schranke in der Coupler Insertion — Messung, Befunde, nächste Schritte

Repository `munich-quantum-toolkit/scpd`, Branch `phase-4-routing-stages`,
Basis-Commit `b097c70`. Die Experiment-Änderung ist **nicht committet**; der
vollständige Diff steht in Anhang A.

## 1. Kurzfassung

- Die Stub-Schranke (`SCPD_CHAIN_BOUND=4`) zieht von der analytischen
  Schranke (`AnalyticDubins::minTurns`) die zwei Geradeausläufe ab, die
  `runsOfEdge` an den Kantenenden erzwingt. Sie bleibt eine untere Schranke
  und ist schärfer.
- 17q und 33q: rund 60–68 % weniger expandierte Präfixe und geroutete Kanten.
  Kettenkosten und Fails sind unter beiden Schranken gleich.
- Insertionszeit (Median von 3): 17q −57 %, 33q −8 %, 45q −37 %.
- 45q: Mit dem Standard-Zeitlimit (20 s pro Kette) löst nur b4 die Kette 3.
  Dadurch sinkt die Zahl der Fails der Final-Stage von 2 auf 1. Die zwei
  Korrektheits-Gates halten auf 45q deshalb nicht, beide zugunsten von b4.
- In keinem Lauf hat b4 höhere Kettenkosten als b1.
- **Wichtigster Befund für die weitere Arbeit:** Auf 33q und 45q verbraucht je
  eine Kette rund 100 s, weil die Suche mit Capacity-Regel alle Präfixe
  aufzählt und jeder an der **letzten Kante zum Launcher** scheitert
  (Abschnitt 6). Daran ändert die Schranke nichts. Eine Endkanten-Probe oder
  Nogood-Lernen (Abschnitt 7, Idee A) sollte diese Zeit fast ganz sparen.

## 2. Begriffe

| Begriff | Bedeutung |
|---|---|
| b1 | `SCPD_CHAIN_BOUND=1`, analytische Schranke (Standard) |
| b4 | `SCPD_CHAIN_BOUND=4`, Stub-Schranke (dieses Experiment) |
| P | Summe der expandierten Präfixe aus den Zeilen `[Coupler Insertion] chain N: optimum\|best … prefixes expanded` |
| S | Summe der gerouteten Kanten aus denselben Zeilen (`… steps routed`) |
| T-S | Kanten in Suchen, die mit `out of time` enden. Diese Suchen haben keine Zusammenfassungszeile; ihre Kanten fehlen in S. |
| Insertionszeit | Zeitstempel `coupler insertion: the layer graph took …` minus Zeitstempel `[Coupler Insertion] settings: …`. Das Fenster enthält die Capacity-Steps. |
| Gate Kosten | Kettenkosten je Kette unter b1 und b4 identisch |
| Gate Fails | `final routing: … \| Fails: N` unter b1 und b4 identisch |

## 3. Code-Änderung (Experiment)

Alle Änderungen liegen in `src/pipeline/FinalRouter.cpp`:

1. Neu vor `familyPathBudget()`:
   - `forcedRunOf(chain, at, option, leaving)` liefert den erzwungenen
     Geradeauslauf am Port einer Option, so wie `runsOfEdge` ihn für die
     gewählte Option liest.
   - `stubbedTurns(from, to, first, last)` verschiebt das Ziel um beide
     Läufe zurück und fragt `analytic_.minTurns`. Beide Punkte werden um
     `1 << 20` angehoben, damit die unsignierten Koordinaten nicht
     unterlaufen.
2. Die drei Stellen `turns[...] = 10000ULL * boundTurns(leaving, arriving);`
   (Trellis, Prefix-A*, Segment-Re-Search mit `lo + e`) nutzen bei
   `chainBound() == 4` die Funktion `stubbedTurns`.
3. Der Audit läuft nur noch bei `chainBound() == 2 || chainBound() == 3`.
4. Die Log-Zeile meldet bei Modus 4 `analytic with the end runs`.

## 4. Build und Messung reproduzieren

### Build

```
uv sync --inexact --only-group build --only-group test
uv sync --inexact --no-dev --no-build-isolation-package mqt-scpd
# nach jeder C++-Änderung: dieselbe Zeile mit --reinstall-package mqt-scpd
```

**Problem in der Cloud-Umgebung:** Der Proxy lehnt die GitHub-Tarballs der
CMake-Abhängigkeiten ab (HTTP 403). Git-Klone von github.com funktionieren.
Lösung:

```
cd /home/user
git clone --depth 1 --branch v25.12.19 https://github.com/google/flatbuffers google/flatbuffers
git clone --depth 1 --branch v3.12.0  https://github.com/nlohmann/json nlohmann/json
git clone --depth 1 --branch v1.11.0  https://github.com/ERGO-Code/highs ergo-code/highs
git clone --depth 1 --branch boost-1.89.0 --recurse-submodules --shallow-submodules -j 2 \
    https://github.com/boostorg/boost boostorg/boost
export SKBUILD_CMAKE_DEFINE="FETCHCONTENT_SOURCE_DIR_FLATBUFFERS=/home/user/google/flatbuffers;FETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=/home/user/nlohmann/json;FETCHCONTENT_SOURCE_DIR_HIGHS=/home/user/ergo-code/highs;FETCHCONTENT_SOURCE_DIR_BOOST=/home/user/boostorg/boost"
# danach die zwei uv-sync-Zeilen oben
```

Prüfen, dass die neue Extension läuft:
`grep -c "analytic with the end runs" .venv/lib/python3.13/site-packages/mqt/scpd/pyscpd.abi3.so`
muss 1 liefern.

### Messung

```
# einmal pro Chip: voller Lauf. -c auf die Benchmark-Config, NICHT auf eine
# vorher nach runs/<chip> kopierte Config: die CLI kopiert die Config selbst
# und bricht sonst mit "are the same file" ab.
.venv/bin/mqt-scpd plan -c benchmarks/<chip>/config.toml -o runs/<chip> -v 1 > runs/<chip>-full.log 2>&1

# dann nur die Final-Stage, b1 und b4 je dreimal abwechselnd, strikt nacheinander
for rep in 1 2 3; do for b in 1 4; do
  SCPD_CHAIN_BOUND=$b .venv/bin/mqt-scpd plan -c runs/<chip>/config.toml -o runs/<chip> \
      --stage final -v 1 > runs/<chip>-b$b-$rep.log 2>&1
done; done
```

Das Auswerteskript steht in Anhang B (`RUNS=<dir> python3 evaluate.py 17q 33q 45q`).

Laufzeiten einer Final-Stage auf 4 Cloud-Kernen: 17q rund 3,5 min, 33q rund
8 min, 45q rund 15 min.

## 5. Ergebnisse

### 5.1 Gesamt

| Chip | P b1 → b4 | S b1 → b4 | T-S b1 → b4 | Insertion b1 → b4 (Median von 3) | Final-Stage gesamt b1 → b4 | Gate Kosten | Gate Fails |
|---|---|---|---|---|---|---|---|
| 17q | 101 → **32** (−68 %) | 295 → **95** (−68 %) | 0 → 0 | 14,7 s → **6,3 s** (−57 %) | 220 s → 207 s | ja | ja (5 / 5) |
| 33q | 103 → **41** (−60 %) | 108 → **40** (−63 %) | ~740 → ~740 | 120,7 s → 110,6 s (−8 %) | 501 s → 483 s | ja | ja (1 / 1) |
| 45q | 97 → 116¹ | 363 → 366¹ | 1051 → **650** | 288,7 s → **182,7 s** (−37 %) | 916 s → 931 s | **nein²** | **nein²** (2 → 1) |

¹ Nur unter b4 zählt Kette 3 mit (19 / 28), weil b1 sie nicht löst. Ohne
Kette 3 kommt b4 auf P 97 und S 338.
² Abschnitt 5.4.

Die Kontrolle hält in allen Logs: b4 meldet `bound analytic with the end runs`,
b1 meldet `bound analytic`. Die Schranke selbst kostet 3–5 µs pro Optionspaar,
also wenige Millisekunden pro Chip.

### 5.2 17q (Zähler deterministisch, lokale Referenz exakt reproduziert)

| Kette | b1 | b4 | Zeit b1 → b4 |
|---|---|---|---|
| 0 | best found³ 160000, 58 P / 149 S | **optimum** 160000, 9 / 29 | 8,2 → 2,0 s |
| 1 | optimum 100000, 9 / 20 | optimum 100000, 7 / 10 | 1,1 → 0,5 s |
| 2 | best found³ 160000, 30 / 122 | **optimum** 160000, 12 / 52 | 3,1 → 1,4 s |
| 3 | optimum 70000, 4 / 4 | optimum 70000, 4 / 4 | 0,3 → 0,3 s |

³ `best found, an end pair laid the other way round`. b4 weist dieselben
Kosten als Optimum nach.

### 5.3 33q (Zähler deterministisch)

| Kette | b1 | b4 | Zeit b1 → b4 |
|---|---|---|---|
| 0 | optimum 100000, 6 / 7 | 6 / 6 | 2,4 → 2,3 s |
| 1 | optimum 120000, 6 / 4, vorher Capacity-Timeout (~740 Kanten) | gleich | **98 → 99 s** |
| 2 | optimum 120000, 29 / 30 | **6 / 6** | 6,0 → 0,7 s |
| 3 | optimum 120000, 5 / 7 | 5 / 5 | 1,0 → 0,7 s |
| 4 | optimum 80000, 45 / 48 | **6 / 7** | 7,0 → 0,9 s |
| 5 | optimum 80000, 6 / 6 | gleich | 0,7 → 0,7 s |
| 6 | optimum 140000, 6 / 6 | gleich | 1,1 → 1,0 s |

Ohne Kette 1 sinkt die Zeit der Chain-Suche von 18,2 s auf 6,3 s.

### 5.4 45q (Kosten und Fails deterministisch, Zähler schwanken)

| Kette | b1 | b4 | Zeit b1 → b4 |
|---|---|---|---|
| 0 | optimum 100000, 7 / 8 | 7 / 7 | 5,0 → 4,7 s |
| 1 | optimum 180000, 19 / 36 | 19 / 27 | 5,4 → 3,9 s |
| 2 | optimum 140000, 7 / ~180 | 6 / ~166 | 34 → 32 s |
| 3 | **zwei Timeouts (~280 + ~145 Kanten), `leaving it as it stands`** | **optimum 150000, 19 / 28** | **109 → 3,8 s** |
| 4 | optimum 120000, 6 / 6 | gleich | 1,1 → 1,1 s |
| 5 | optimum 100000, 6 / 7 | gleich | 1,5 → 1,4 s |
| 6 | best in the time given 180000, ~45 / ~120 | gleich | 26,5 → 26,2 s |
| 7 | optimum 100000, 7 / 5, vorher Capacity-Timeout (~630 Kanten) | gleich, ~650 Kanten | **99 → 102 s** |

- Final-Stage unter b1: `208 of 209 drawn, 1 unrouted (1 feedline edges) … Fails: 2`.
- Final-Stage unter b4: `209 of 209 drawn … Fails: 1`.
- Die Zähler schwanken, weil Ketten gegen das Zeitlimit laufen. Kette 6:
  P 44–50 unter b1, 40–47 unter b4. Kette 2: S 179–182 unter b1, 159–173
  unter b4.

### 5.5 Kontrolllauf 45q mit `SCPD_CHAIN_ASTAR_SECONDS=300` (je 1 Lauf)

| Kette | b1 | b4 |
|---|---|---|
| 0 | optimum 100000, 7 / 8 | 7 / 7 |
| 1 | optimum 180000, 19 / 36 | 19 / 27 |
| 2 | optimum **160000**, 523 / 1470 | optimum **160000**, 523 / 1192 |
| 3 | optimum **150000**, 152 / 178 (57 s) | optimum **150000**, 19 / 28 (4 s) |
| 4 | optimum 120000, 6 / 6 | gleich |
| 5 | optimum 100000, 6 / 7 | gleich |
| 6 | best in the time given 180000, 618 / 1754 | best in the time given 180000, 634 / 1747 |
| 7 | optimum **120000**, 289 / 470 | optimum **120000**, 248 / 394 |
| Summe | P 1620, S 3929 | P 1462, S 3408 |
| Insertion | 825 s | 693 s (−16 %) |
| Final-Stage | 2 open, 1 short, Fails 3 | 2 open, Fails 2 |

- Mit 300 s kommt auch b1 bei Kette 3 auf 150000. Das bestätigt das Optimum
  von b4.
- Alle Kettenkosten sind gleich. Die Fails unterscheiden sich trotzdem (3
  gegen 2). Vermutlich wählen die Ketten bei gleichen Kosten verschiedene
  Optionen. **Das ist nicht untersucht.**
- Das Zeitlimit verändert die Optima unter beiden Schranken: Kette 2 kommt
  auf 160000 statt 140000, Kette 7 auf 120000 statt 100000. Bei 20 s öffnet
  Kette 2 die Second-Dogleg-Optionen, und Kette 7 sucht ohne Capacity-Regel
  neu. `optimum` gilt also nur für die Optionsmenge und den Capacity-Ausgang,
  die das Zeitlimit zulässt.

## 6. Befund: die letzte Kante zum Launcher kostet die Zeit

Auf 33q/Kette 1 und 45q/Kette 7 läuft die Suche mit Capacity-Regel
(`SCPD_CAPACITY_RULE`) ins Zeitlimit. Erst danach sucht sie ohne Regel neu.
Die Capacity-Checks verteilen sich so:

**33q, Kette 1 (b4, Lauf 1):**

| Kante | SAT | UNSAT |
|---|---|---|
| f6 (0 → 1) | 3 | 0 |
| f7 (1 → 2) | 3 | 0 |
| f8 (2 → 3) | 12 | 0 |
| f9 (3 → 4) | 37 | 0 |
| f10 (4 → 5) | 364 | 0 |
| **f11 (5 → Launcher)** | **0** | **362** |

Die 362 UNSAT-Fälle schließen immer die Drähte 21 23 24, in 116 Fällen dazu
28 29 30. Die Layer-5-Optionen 28, 31, 34, 37, 40 und 43 kommen je etwa
36-mal vor. Wahrscheinlich sind das Verschiebungen derselben Orientierung
(`shiftX`/`shiftY` in `CouplerOption`).

Gesamt für Kette 1: 781 Capacity-Checks in 58,9 s (0,075 s pro Check), 362
davon schließen einen Draht. Vorher meldet die Suche für Waypoint '31':
`no prefix reaches past it on plain options — opening the second-dogleg options`.

**45q, Kette 7 (b4, Lauf 1):**

| Kante | SAT | UNSAT |
|---|---|---|
| f45 … f48 | 4 / 4 / 4 / 11 | 0 |
| f49 (4 → 5) | 52 | 0 |
| f50 (5 → 6) | 311 | 0 |
| **f51 (6 → Launcher)** | **0** | **311** |

Die 311 UNSAT-Fälle schließen immer 132 und 133, meist dazu 136.

**Folgerung:** Die Suche zählt alle Kombinationen der inneren Layer auf, und
jede scheitert an derselben letzten Kante. Diese Klasse von Schritten ist
unabhängig vom Präfix. Ein einziger Check könnte sie erkennen.

Hinweis zum Code: Die Zeit der Capacity-Checks wird dem Budget gutgeschrieben
(`problem.budget += spent` in `solveChainAStar`). Das Zeitlimit von 20 s gilt
also nur für das Routing; die Wanduhr-Zeit pro Kette ist viel größer.

## 7. Ideen zur weiteren Beschleunigung

| # | Idee | Art der Klasse | Erwarteter Nutzen | Exakt? | Aufwand |
|---|---|---|---|---|---|
| A | Endkanten-Probe und Nogoods für die Capacity-Regel | Schritt (Layer, Option, Option) | ~100 s auf 33q, ~100 s auf 45q | ja, mit Bedingung | klein |
| B | Präfix-Dominanz statt Baumsuche | Präfix (Layer, Endoption, Zaun im Zukunftsbereich, `lost`) | groß bei Ketten am Zeitlimit (45q/6, 45q/2) | ja, mit Bedingung | mittel |
| C | Options-Klassen (Verschiebungen, Dogleg-Varianten) | Option | weniger Verzweigung | mit Schranke pro Klasse | mittel |
| D | Capacity-Verdikt nach Kanal-Signatur cachen | Weg (Homotopie) | weniger Checks à 0,075 s | nein (Vorfilter) | mittel bis groß |
| E | Kanten-Memo mit Zaun-Signatur | Kante | weniger Edge-Routing | ja | klein bis mittel |
| F | Schärfere Schranke (Freiraum-Tabelle, Modus 3+4) | – | weniger Präfixe | ja | mittel |
| G | Kinder eines Präfixes parallel expandieren | – | bis ~3× auf 4 Kernen | ja bei deterministischer Reihenfolge | mittel |

### A — Endkanten-Probe und Nogoods (Priorität 1)

- **Variante 1, Endkanten-Probe vor der Suche:** Für jede mögliche letzte
  Kante (Option an Layer L−2 → Launcher) einmal prüfen, nur mit dieser Kante
  und ihrem Coupler. Schließt jede Kombination einen Draht, ist die
  Capacity-Regel für die Kette unerfüllbar. Die Suche geht dann sofort ohne
  Regel los.
- **Variante 2, Nogoods während der Suche lernen:** Nach dem ersten UNSAT an
  (k, c, d) einen zweiten Check ohne die übrigen Wege des Präfixes. Schließt
  die Kante dort auch, gilt der Nogood für alle Präfixe.
- **Warum das korrekt sein kann:** Der Check scheint monoton zu sein; mehr
  verlegte Geometrie heißt weniger freier Raum und mehr verlorene Drähte. In
  beiden Fällen war der Elternschritt SAT, also ist `closed = lost(step)`.
  Dann gilt `lost(Präfix + Kante) ⊇ lost(Kante allein)`.
  **Die Monotonie muss am Capacity-Check nachgewiesen werden.**
- **Haken:** Der Weg der letzten Kante hängt vom Zaun des Präfixes ab. Exakt
  ist ein Nogood nur auf dem, was immer steht: dem Coupler-Körper der Option
  und dem Stub am Launcher. Ein Nogood auf einen geprobten Weg ist eine
  Heuristik. Er taugt zum Ordnen, als harte Sperre braucht er einen Fallback.
- **Erwartung:** 33q-Insertion etwa 110 s → 15 s, 45q etwa 183 s → 85 s.

### B — Präfix-Dominanz

- **Heute:** Ein A*-Knoten ist ein ganzer Präfix, und nichts wird
  zusammengeführt („no prefix comes round twice“, Kommentar über `struct
  Laid`). Die Suche ist ein Baum und wächst exponentiell.
- **Klasse:** Zwei Präfixe mit gleichem Layer und gleicher Endoption sind für
  den Rest der Kette gleichwertig, wenn ihre Wege im Zukunftsbereich (Union
  von `corridorOfEdge` über die verbleibenden Kanten) gleich sind und ihre
  `lost`-Mengen gleich sind.
- **Dominanz:** Präfix p1 schlägt p2, wenn p1 nicht teurer ist, sein Zaun im
  Zukunftsbereich eine Teilmenge des Zauns von p2 ist und `lost(p1) ⊆
  lost(p2)` gilt.
- **Bedingung für Exaktheit:** Der Edge-Router muss monoton im Zaun sein
  (mehr Hindernisse → nicht billiger) und darf nur im Korridor suchen.

### C — Options-Klassen

- Optionen mit gleicher Orientierung, die sich nur um `shift` unterscheiden,
  bilden eine Klasse. Die Suche expandiert erst einen Vertreter. Ein weiteres
  Mitglied öffnet sie erst, wenn dessen Schranke unter dem besten gefundenen
  Wert liegt.
- Für die Stub-Schranke ist das fast gratis: Sie hängt nur von der Pose nach
  dem Stub ab.

### D — Capacity-Cache nach Kanal-Signatur

- Signatur = Folge der Kammern und Sättel des Capacity-Graphen, die ein Weg
  passiert, plus Breite. Gleiche Signatur heißt fast immer gleiches Verdikt.
- „short by“ hängt an der Breite, also ist der Cache nicht exakt. Er taugt
  als Vorfilter mit Stichproben-Nachprüfung.

### E — Kanten-Memo mit Zaun-Signatur

- `edgeMemo_` speichert heute pro (Kette, Layer, Option, Option)
  (`edgeMemoKey`) und testet jeden gemerkten Weg gegen den Zaun.
- Ein Hash des Zauns im Korridor der Kante als Teil des Schlüssels macht die
  Wiederverwendung exakt und spart Neusuchen.

### F — Schärfere Schranken

- Freiraum-Tabelle: Pro Optionspaar einmal die echten Kosten auf dem Grid nur
  mit der festen Artwork berechnen. Hindernisse der Kette können die Kosten
  nur erhöhen, also bleibt die Schranke zulässig. Die Tabelle lässt sich lazy
  füllen und cachen.
- Modus 3 (`boundTurnsAround`) und Modus 4 kombinieren.
- Hilft bei Ketten mit vielen Präfixen, nicht gegen den Befund aus Abschnitt 6.

### G — Parallelität

- Die Kinder eines Präfixes (Edge-Routing plus Capacity-Check) sind
  voneinander unabhängig.
- Mit 4 Kernen ist bis etwa 3× drin, ohne die Suche zu ändern.

### Vorgeschlagene Reihenfolge

1. A (Endkanten-Probe) hinter einem Schalter, zum Beispiel
   `SCPD_CHAIN_END_PROBE`. Messen auf 33q und 45q gegen b4 mit dem
   Messaufbau aus Abschnitt 4 und den Gates aus Abschnitt 2.
2. C.
3. B, wenn die Ketten am Zeitlimit danach noch dominieren.
4. D, E, F und G je nach dem nächsten Profil.

## 8. Offene Punkte

- 57q und 69q sind nicht gemessen. 57q bräuchte grob 2–3 h für einen vollen
  Lauf plus 6 Final-Läufe.
- Ungeklärt: Warum unterscheiden sich die Fails im 45q-Kontrolllauf bei
  gleichen Kettenkosten?
- Ungeklärt: Ist der Capacity-Check monoton in der verlegten Geometrie?
  Idee A und B hängen davon ab.
- Ungeklärt: Ist der Edge-Router monoton im Zaun? Idee B hängt davon ab.
- Die Experiment-Änderung ist nicht committet und hat weder Tests noch einen
  Lint-Lauf.

## Anhang A — Diff der Experiment-Änderung

```diff
diff --git a/src/pipeline/FinalRouter.cpp b/src/pipeline/FinalRouter.cpp
index 06be66e..9f4d883 100644
--- a/src/pipeline/FinalRouter.cpp
+++ b/src/pipeline/FinalRouter.cpp
@@ -4444,6 +4444,7 @@ public:
               static_cast<double>(boundPairs_),
           chainBound() == 0   ? "turnBound"
           : chainBound() == 3 ? "analytic around the artwork"
+          : chainBound() == 4 ? "analytic with the end runs"
                               : "analytic"));
     }
     if (chainBound() == 3) {
@@ -6436,6 +6437,47 @@ public:
     return analytic_.minTurns(from, to);
   }
 
+  /// EXPERIMENT: the forced straight run at a waypoint's port for a named
+  /// option, as `runsOfEdge` reads it off the chosen one.
+  [[nodiscard]] std::uint32_t forcedRunOf(const std::uint32_t chain,
+                                          const std::size_t at,
+                                          const std::size_t option,
+                                          const bool leaving) const {
+    const auto& point = chains_[chain][at];
+    if (point.fixed || (leaving && !couplerStubs())) {
+      return tuning_.straightStart;
+    }
+    return couplers_[point.coupler].options[option].run;
+  }
+
+  /// EXPERIMENT: the analytic bound with the forced end runs folded in
+  /// (`SCPD_CHAIN_BOUND=4`). Every way has to run `first` cells straight out
+  /// of the source and `last` straight into the target, so the rest of it
+  /// covers the displacement less those two runs. Both points are lifted by
+  /// a constant so the shifted target stays inside the unsigned
+  /// coordinates; the bound reads differences only.
+  [[nodiscard]] std::uint32_t stubbedTurns(const PathPoint& from,
+                                           const PathPoint& to,
+                                           const std::uint32_t first,
+                                           const std::uint32_t last) const {
+    constexpr std::int64_t LIFT = 1LL << 20U;
+    const auto vs = routing::headingVector(from.heading & 7U);
+    const auto vt = routing::headingVector(to.heading & 7U);
+    auto a = from;
+    auto b = to;
+    a.x = static_cast<std::uint32_t>(static_cast<std::int64_t>(a.x) + LIFT);
+    a.y = static_cast<std::uint32_t>(static_cast<std::int64_t>(a.y) + LIFT);
+    b.x = static_cast<std::uint32_t>(
+        static_cast<std::int64_t>(b.x) + LIFT -
+        ((static_cast<std::int64_t>(first) * vs.dx) +
+         (static_cast<std::int64_t>(last) * vt.dx)));
+    b.y = static_cast<std::uint32_t>(
+        static_cast<std::int64_t>(b.y) + LIFT -
+        ((static_cast<std::int64_t>(first) * vs.dy) +
+         (static_cast<std::int64_t>(last) * vt.dy)));
+    return analytic_.minTurns(a, b);
+  }
+
   [[nodiscard]] static std::uint64_t familyPathBudget() {
     static const auto paths = static_cast<std::uint64_t>(
         std::clamp(envWhole("SCPD_CHAIN_FAMILY_PATHS", 20000), 100, 100000000));
@@ -13073,7 +13115,13 @@ public:
             for (std::size_t d = 0; d < m; ++d) {
               const auto arriving =
                   portOf(at + 1, static_cast<std::uint32_t>(d), false);
-              turns[at][(c * m) + d] = 10000ULL * boundTurns(leaving, arriving);
+              turns[at][(c * m) + d] =
+                  10000ULL *
+                  (chainBound() == 4
+                       ? stubbedTurns(leaving, arriving,
+                                      forcedRunOf(chain, at, open[at][c], true),
+                                      forcedRunOf(chain, at + 1, open[at + 1][d], false))
+                       : boundTurns(leaving, arriving));
             }
           }
           boundPairs_ += n * m;
@@ -13130,7 +13178,7 @@ public:
           const auto real =
               edgeCost(wires, chain, at, open[at][c], open[at + 1][d],
                        aheadCode, ahead.empty() ? nullptr : &laid);
-          if (chainBound() >= 2) {
+          if (chainBound() == 2 || chainBound() == 3) {
             audit(chain, at, portOf(at, static_cast<std::uint32_t>(c), true),
                   portOf(at + 1, static_cast<std::uint32_t>(d), false), real);
           }
@@ -13373,7 +13421,13 @@ public:
           for (std::size_t d = 0; d < m; ++d) {
             const auto arriving =
                 portOfOption(chain, at + 1, open[at + 1][d], false);
-            turns[at][(c * m) + d] = 10000ULL * boundTurns(leaving, arriving);
+            turns[at][(c * m) + d] =
+                10000ULL *
+                (chainBound() == 4
+                     ? stubbedTurns(leaving, arriving,
+                                    forcedRunOf(chain, at, open[at][c], true),
+                                    forcedRunOf(chain, at + 1, open[at + 1][d], false))
+                     : boundTurns(leaving, arriving));
           }
         }
         boundPairs_ += n * m;
@@ -13502,7 +13556,7 @@ public:
             edgeCost(wires, chain, at, open[at][prefix[at]], open[at + 1][j], 0,
                      fence.empty() ? nullptr : &fence, &way, !fresh);
         stepBudget_ = routing::TRELLIS_UNREACHABLE;
-        if (chainBound() >= 2) {
+        if (chainBound() == 2 || chainBound() == 3) {
           audit(chain, at, portOfOption(chain, at, open[at][prefix[at]], true),
                 portOfOption(chain, at + 1, open[at + 1][j], false), real);
         }
@@ -16649,7 +16703,13 @@ public:
         for (std::size_t d = 0; d < m; ++d) {
           const auto arriving =
               portOfOption(chain, lo + e + 1, open[e + 1][d], false);
-          turns[e][(c * m) + d] = 10000ULL * boundTurns(leaving, arriving);
+          turns[e][(c * m) + d] =
+              10000ULL *
+              (chainBound() == 4
+                   ? stubbedTurns(leaving, arriving,
+                                  forcedRunOf(chain, lo + e, open[e][c], true),
+                                  forcedRunOf(chain, lo + e + 1, open[e + 1][d], false))
+                   : boundTurns(leaving, arriving));
         }
       }
       pairs += n * m;
```

## Anhang B — Auswerteskript `evaluate.py`

Aufruf: `RUNS=/home/user/scpd/runs python3 evaluate.py 17q 33q 45q`. Es
wertet nur fertige Läufe aus (Logs mit `Fails`-Zeile).

```python
"""Summarise b1/b4 final-stage logs per chip."""
import re, statistics, sys
from pathlib import Path

import os
RUNS = Path(os.environ.get("RUNS", "/home/user/scpd/runs"))
STAMP = re.compile(r"^\[final\]\s+([\d.]+)s")
CHAIN = re.compile(r"\[Coupler Insertion\] chain (\d+): (optimum|best found|best in the time given)(.*?) (\d+) over (\d+) layers, (\d+) prefixes expanded, (\d+) steps routed")

def parse(path):
    t_set = t_end = None
    bound = fails = None
    chains = {}
    other = []
    timeouts = {}
    for line in path.read_text(errors="replace").splitlines():
        m = STAMP.match(line)
        ts = float(m.group(1)) if m else None
        if "[Coupler Insertion] settings:" in line and t_set is None:
            t_set = ts
        if "coupler insertion: the layer graph took" in line:
            t_end = ts
            bound = line.split("bound ", 1)[-1].strip()
        c = CHAIN.search(line)
        if c:
            chains[int(c.group(1))] = dict(kind=c.group(2), note=c.group(3).strip(", "), cost=int(c.group(4)),
                                           layers=int(c.group(5)), P=int(c.group(6)), S=int(c.group(7)))
        o = re.search(r"\[Coupler Insertion\] chain (\d+): out of time before a run of options joined the chain without closing a wire \((\d+) steps routed\)", line)
        if o:
            timeouts[int(o.group(1))] = int(o.group(2))
        l = re.search(r"\[Coupler Insertion\] chain (\d+): out of time before any run of options joined the chain — leaving it as it stands \((\d+) steps routed\)", line)
        if l:
            chains[int(l.group(1))] = dict(kind="left as it stands", note="", cost=None, layers=None, P=0, S=0)
            timeouts[int(l.group(1)) + 0.5] = int(l.group(2))
        f = re.search(r"\] +[\d.]+s +final routing: .*\| Fails: (\d+)", line)
        if f:
            fails = int(f.group(1))
        if re.search(r"timeout|timed out|budget|ran out", line, re.I) and "[Coupler Insertion]" in line and "settings" not in line:
            other.append(line.strip())
    total = re.search(r"^\[final\]\s+([\d.]+)s", [l for l in path.read_text().splitlines() if l.startswith("[final]")][-1]).group(1)
    return dict(t=None if t_set is None or t_end is None else t_end - t_set, bound=bound, fails=fails,
                chains=chains, other=other, timeouts=timeouts, total=float(total))

for chip in sys.argv[1:]:
    # Only finished runs count: a run is finished once it printed its Fails line.
    res = {b: [r for r in (parse(p) for p in sorted(RUNS.glob(f"{chip}-b{b}-*.log"))) if r["fails"] is not None] for b in (1, 4)}
    print(f"== {chip}: {len(res[1])} b1 runs, {len(res[4])} b4 runs")
    for b in (1, 4):
        for i, r in enumerate(res[b]):
            P = sum(c["P"] for c in r["chains"].values()); S = sum(c["S"] for c in r["chains"].values())
            TO = sum(r["timeouts"].values())
            print(f"  b{b}#{i+1}: bound={r['bound']!r} P={P} S={S} timeout-steps={TO} {r['timeouts']} insertion={r['t']:.2f}s final={r['total']:.1f}s fails={r['fails']} chains={len(r['chains'])}")
        # determinism across reps
        sigs = {tuple(sorted((k, v['cost'], v['P'], v['S'], v['kind']) for k, v in r['chains'].items())) for r in res[b]}
        print(f"  b{b}: chain results identical across reps: {len(sigs) == 1}")
        if res[b]:
            print(f"  b{b}: median insertion {statistics.median(r['t'] for r in res[b]):.2f}s, median final {statistics.median(r['total'] for r in res[b]):.1f}s")
    if res[1] and res[4]:
        a, z = res[1][0], res[4][0]
        print("  chain | b1 kind cost P S | b4 kind cost P S")
        for k in sorted(set(a["chains"]) | set(z["chains"])):
            x, y = a["chains"].get(k), z["chains"].get(k)
            fx = f"{x['kind']}{' ('+x['note']+')' if x['note'] else ''} {x['cost']} {x['P']} {x['S']}" if x else "-"
            fy = f"{y['kind']}{' ('+y['note']+')' if y['note'] else ''} {y['cost']} {y['P']} {y['S']}" if y else "-"
            print(f"  {k:5d} | {fx} | {fy}")
        g1 = all(a["chains"].get(k, {}).get("cost") == z["chains"].get(k, {}).get("cost") for k in set(a["chains"]) | set(z["chains"]))
        g2 = {r['fails'] for r in res[1]} == {r['fails'] for r in res[4]} and len({r['fails'] for r in res[1]}) == 1
        print(f"  gate costs: {g1}, gate fails: {g2} ({sorted({r['fails'] for r in res[1]})} vs {sorted({r['fails'] for r in res[4]})})")
        for b in (1, 4):
            for r in res[b]:
                for o in r["other"]:
                    print(f"  b{b} note: {o}")
```
