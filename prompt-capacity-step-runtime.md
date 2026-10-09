# Auftrag: Die Kapazitätsprüfung pro Schritt der Kettensuche schnell machen

Die Coupler-Insertion prüft seit dem 2026-10-09 nach **jedem** Schritt der
Kettensuche den Kapazitätsgraphen des Chips und verwirft einen Schritt, dessen
Feedline-Kante einen Draht einschließt (`SCPD_CAPACITY_RULE`, an). Auf 17q
macht das den Unterschied: Der Graph ist SAT, und der Feedline-Pass lässt
nichts ungeroutet, offen oder kreuzend (ohne die Regel: 6 schlechte Drähte).

Die Regel ist aber zu teuer für die großen Chips. Eine Prüfung ist heute eine
**ganze Analyse des Chips**: etwa 0,1 s auf 17q, 0,5 s auf 21q, 1,4–3 s auf
33q bis 69q. Die Suche verwirft auf den großen Chips Hunderte Schritte, und
weil die Prüfzeit dem Zeitlimit der Kette gutgeschrieben wird, dauert eine
Kette dort 20–40 Minuten. 45q, 57q und 69q wurden deshalb abgebrochen.

**Deine Aufgabe:** Mach die Prüfung pro Schritt so schnell, dass die Regel auf
allen acht Chips in einer Laufzeit läuft, die man täglich laufen lassen kann,
und **miss** das Ergebnis auf allen acht Chips. Das Urteil der Prüfung soll
dabei gleich bleiben; wo ein schnelleres Verfahren anders urteilt, miss die
Abweichung und leg sie dem Nutzer vor, bevor du sie zum Standard machst.

## Lies das zuerst

- `handover-cpw-coupler-insertion.md`, Abschnitt *How the insertion uses the
  capacity check* – wie die Insertion die Prüfung nutzt: der Chip eines
  Schritts (`stateOf` mit Präfix), die Demands (vorgeschriebene Kreuzungen,
  Längenregel der Resonatoren), `checkInTurn`, das Verwerfen, der Rückfall,
  die Uhr, das Log, die Messungen auf allen Chips.
- `handover-capacity-check.md` – die Analyse selbst (Wände, dünne Port-Wände
  mit Schlitz, Schnitte, Kammern, Stretches), was eine Prüfung kostet (§2),
  was lokal geht (§2, *What can be done incrementally*), die Schalter.
- `artifacts/logs/cap-fast/capfast.cpp` – ein Prototyp außerhalb des Pakets:
  statische Distanz- und Feature-Transformation einmal pro Chip, Profile
  entlang der dynamischen Wände, Kammern auf gröberem Raster. Er ist schnell
  (17q: 7 ms für alle Wände, Kammern bei 1/4 in 1,2 ms), trifft aber die
  Schnitte des Stage nur zu 93–129 von 144. Lies, warum (Abschnitt in
  `handover-capacity-check.md` und die Notizen dort), bevor du ihn weiterbaust.
- `AGENTS.md` – die Regeln des Repos.

Checkout `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`, Branch
`phase-4-routing-stages`. **Alles ist unkommittiert**, und es bleibt so: Der
Nutzer committet selbst, pro Phase. Leg keine Commits an.

## Was gemessen ist

Pro Prüfung, in den Kettensuchen gemessen (beide Margins 5):

| Chip | Sekunden pro Prüfung | Beobachtung |
| --- | --- | --- |
| 17q | 0,11–0,18 | alle Ketten in Minuten |
| 21q | 0,5 | Kette 0: 336 Schritte, 251 verworfen, 179 s |
| 33q | 0,8–1,4 | Insertion 1486 s statt 15 s |
| 45q | 1,5–1,8 | Kette 2: 680 Schritte, jede Option von f16 schließt 38, 39, 40 |
| 57q | 2–3 | Kette 0: 420 Schritte, jede Option von f4 schließt 9, 10, 12, 13 |
| 69q | 1,9–2,1 | Kette 2: 308 Schritte, 646 s |

Wohin die Zeit einer Prüfung geht (17q, nach der linearen
Distanztransformation): Wände 8 ms, Distanz 17 ms, Voronoi 42 ms, Achse 6 ms,
Schnitte 21 ms, Kammern 21 ms, Stretches und Ports 2 ms, `checkInTurn` 0,1 ms.
Auf 69q (5400 × 5400 Zellen) skaliert das mit der Fläche.

## Ansätze, die du prüfen solltest

Prüfe sie, miss sie, und nimm, was sich lohnt. Du musst nicht alle bauen.


1. **Lokale Aktualisierung.** Ein Schritt ändert den Chip nur um eine Kante,
   ein Pad und einen Lead. Distanz, Achse und Schnitte nur in einem Fenster
   um diese Änderung neu rechnen (gemessen: 120 Zellen Reichweite 7,7 ms auf
   17q und 14 ms auf 69q, 240 Zellen 17,5 und 43 ms; die Schnitte am
   Fensterrand stimmen nicht ganz), die Kammern nur dort neu fluten, mit
   stabilen Kammernummern.
2. **Wiederverwendung über Präfixe.** Der Chip einer Kette ohne Präfix ist für
   die ganze Suche dieser Kette derselbe. Ein Präfix unterscheidet sich vom
   Präfix davor um genau eine Kante. Die Analyse des Elternpräfixes als Basis
   zu nehmen, statt den Chip jedes Mal neu zu bauen, ist der direkte Weg zu 1.
3. **Früher urteilen.** Auf 45q und 57q schließt *jede* Option einer Ebene
   dieselben Drähte. Eine Prüfung, die das einmal erkennt (z. B. ob die
   geschlossenen Drähte schon durch die Kanten davor oder die Pads der
   Ebene festliegen), spart Hunderte Prüfungen. Prüfe auch, ob es dort
   wirklich keinen Lauf gibt oder ob die Prüfung zu streng ist.
4. **Billige Vorprüfung.** Nur prüfen, wenn die neue Kante eine Kammer
   berührt, durch die ein Weg der vorigen Prüfung lief, oder einen Schnitt
   verändert; sonst das Urteil des Elternpräfixes übernehmen.
5. **Parallelität.** Nutze keine PRalleliserung! Das kommt später erstr

## Wie du misst

Das Protokoll der Messung vom 2026-10-09 (`artifacts/logs/allchips/`):

- Eingaben: `artifacts/<chip>/00-…05-*`, Config `benchmarks/<chip>/config.toml`
  mit `stop_after = "feedlines"` und `repair_trials = 0`.
- `artifacts/logs/allchips/run-one.sh <chip> rule|norule` (die Regel an oder
  `SCPD_CAPACITY_RULE=0`), Auswertung mit `artifacts/logs/allchips/summary.py`.
- Zahl der Bewertung: `bad = unrouted + open + crossing` des finalen
  Routings. `short` und `long` zählen nicht.
- Das Binding für die Läufe kommt aus dem Overlay `artifacts/dev/py-cg`; nach
  einer Änderung am C++ das Binding bauen und in `artifacts/dev/{py,py-cg,py-cap}`
  kopieren (nicht, während ein `dev-mqt-scpd`-Prozess läuft).

Berichte pro Chip: Ketten gesetzt, wie oft der Rückfall ohne Regel nötig war,
Urteil des Graphen, `bad`, Laufzeit der Insertion und des ganzen Laufs, und
die Zeit pro Prüfung vorher und nachher.

## Was am Ende da sein muss

- Die Regel läuft auf allen acht Chips durch; die Laufzeit pro Chip steht im
  Bericht.
- Tests für jede Änderung (AGENTS.md), `ctest -E EveryChip` grün bis auf den
  bekannten `FinalRouter.OnlyUnsettledRedrawsOneWire`.
- Die Handover (`handover-cpw-coupler-insertion.md`,
  `handover-capacity-check.md`) beschreiben den neuen Stand und die
  Messungen; CHANGELOG nachgetragen.
- In `artifacts/<chip>/` für jeden Chip `final-layout.svg` und
  `final-capacity-graph.html` aus einem Lauf mit `-d` (für 4q, 9q, 17q, 21q
  und 33q liegen sie schon dort, für 45q, 57q und 69q fehlen sie).
