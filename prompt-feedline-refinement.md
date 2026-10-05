# Auftrag: das Feedline-Refinement und die abschließende Meander-Einfügung

Die Final-Stufe endet heute nach der vierten Phase, dem Feedline-Routing
(`stop_after = "feedlines"` in allen acht `benchmarks/*/config.toml`). Die
fünfte Phase, `refined`, ist im Prototyp
`run_final_routing_feedline_refinement_parallel` und bei uns nur ein Rumpf:
`Driver::refine` mit `feedline_refinement_rounds = 0`. **Deine Aufgabe ist,
diese Phase zu bauen und zu messen** — und in ihr geschieht zum ersten Mal
endgültig, was bisher ausgeschaltet war: jeder Resonator wird auf seine
Ziellänge gebracht (`SCPD_FEEDLINE_MEANDER` ist aus, die Längen wurden in
Phase 4 bewusst nicht gelesen). Nach dieser Phase werden sie gelesen.

## Wo die Stufe steht (2026-10-05, Arm `artifacts/logs/squeeze-reject2`)

| Chip | bad | unrouted / open / crossing | short | long | Feedline-Kanten |
|---|---|---|---|---|---|
| 4q | 0 | 0 / 0 / 0 | 4 | 0 | 5 / 5 |
| 9q | 0 | 0 / 0 / 0 | 2 | 0 | 10 / 10 |
| 17q | 4 | 0 / 0 / 4 | 7 | 0 | 20 / 20 |
| 21q | 1 | 0 / 0 / 1 | 3 | 0 | 26 / 26 |
| 33q | 0 | 0 / 0 / 0 | 10 | 0 | 40 / 40 |
| 45q | 0 | 0 / 0 / 0 | 13 | 0 | 52 / 52 |
| 57q | 2 | 0 / 0 / 2 | 32 | 0 | 66 / 66 |
| 69q | 1 | 0 / 0 / 1 | 31 | 0 | 81 / 81 |

Die acht Crossings sind ein Fall: ein konventioneller Wire kreuzt eine Kante
"10 cells from edge", der Halo-Rand der Crossing-Rule, bei dem Suche und
Zähler nicht übereinstimmen, solange `SCPD_CROSSING_EXIT_HEADING` aus ist.
Das ist **nicht** dein Auftrag; wenn das Refinement sie verschiebt, berichte
es. Die 102 short sind dein Auftrag: Resonatoren, die nach dem Schnitt am
Coupler kürzer sind als `target_resonator_length` minus Toleranz. long gibt
es heute keine.

Checkout `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`, Branch
`phase-4-routing-stages`, HEAD `454df86` plus der unkommittierte Nachmittag
und Abend des 2026-10-05 (Squeeze-Regel, Schrittbudget, drei gemessene
Negativarme). Der Prototyp ist
`/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD/include/fiction/layout/FinalGrid.cpp`,
die Funktion `run_final_routing_feedline_refinement_parallel` ab Zeile
12391, ihr Kopf in `FinalGrid.hpp` ab Zeile 989.

## Was es schon gibt

- **`Driver::refine`** (`src/pipeline/FinalRouter.cpp`, Suche nach
  `void refine(`): eine Runde zieht jeden Wire einmal neu, Korridor
  `pass.reach`, Fence der beiden Ringnachbarn, `closeLengthBands` nur außerhalb
  der Feedline-Pässe, `priceRoom` (der Chamfer-Abstand zur Kanalmitte, der
  Prototyp-Begriff `compute_corridor_proximity_decay`), `constrainByFeedlines`
  unter `pass.feedlines`. Ein gefundener Weg wird nur genommen, wenn er nicht
  mehr Konflikte hat als der alte und keine Feedline kreuzt; ein Resonator
  wird dabei verlängert (`lengthen`) und behält den alten Weg, wenn der
  breitere keinen Platz für die Schleife hat. Die Pipeline ruft es als
  `feedline refinement` mit `tuning.feedlineRefinementRounds` (0) und
  `.feedlines = true` auf, hinter `runs(4)`; der Snapshot heißt `refined`.
- **Die Meander-Einfügung**: `routing::insertMeander`
  (`include/mqt-scpd/routing/MeanderInsertion.hpp`) und `Driver::lengthen`;
  eine rechteckige Schleife pro Resonator, Paare von Geradenzellen vom Qubit
  her kleinstes Spannmaß zuerst (die Korrektur des Nutzers vom 2026-09-15),
  nur im Korridor der Suche, Selbstschnitt geprüft, gerenderte Länge
  verifiziert. `lengthensIn(wire, pass)` entscheidet, ob ein Pass verlängert:
  außerhalb der Feedline-Pässe immer, in ihnen nur mit
  `SCPD_FEEDLINE_MEANDER`. `needsLength`, `requiredLength` und
  `Wire::anchorGap` sind die Größen dazu; `exact_` und
  `tuning_.lengthTolerance` die Toleranz.
- **Die Verdikte**: `failsOf` zählt short und long pro Wire, `FinalWire.verdict`
  trägt die Bits `Short` und `Long` im Artefakt, `FinalWire.length` die
  gerenderte Länge. Der Plot markiert heute nur unrouted/open/crossing rot und
  nennt short/long im Tooltip (`python/mqt/scpd/planning.py`,
  `BAD_VERDICTS`); ob er in dieser Phase auch die Längenverdikte markiert, ist
  deine Entscheidung, aber dann als eigene Ebene und Farbe.
- **Die Konfiguration**: `feedline_refinement_rounds` in `schemas/config.fbs`
  (Default 0), `stop_after` in den Benchmark-Configs auf `"feedlines"`. Für
  deine Läufe setzt du `stop_after = "refined"` in der Kopie im
  Laufverzeichnis (`STOP_AFTER=refined artifacts/logs/run-arm.sh <arm>`); ob
  die Benchmark-Configs folgen, entscheidet der Nutzer.
- **Die Werkzeuge**: `artifacts/logs/run-arm.sh <arm> [ENV=…]`,
  `bad-table.py <arm> …` (erweitere es um die Spalten short und long),
  `keylines.sh`, `dev-sync.sh`/`dev-run.sh`/`dev-mqt-scpd.py` für ein Binding
  aus dem Build-Baum, die Dashboards mit `-v 1 -d`, und
  `.venv/bin/mqt-scpd plot -c benchmarks/<c>/config.toml --stage final --run-dir artifacts/<c> -o … --phase refined`.
  Das venv-Binding wird nur durch
  `uv pip install --python .venv/bin/python --no-build-isolation --no-deps --reinstall-package mqt-scpd -e .`
  erneuert, nicht durch `uv sync`.

## Was der Prototyp tut, und was du vergleichst

Sein Refinement läuft **nach** dem Feedline-Routing auf dessen fertigem
Zustand, mit denselben Zielen, Korridoren und harten Regeln (Bridge-Ersetzung,
Resonator-Startbogen, `forbidden_feedline_paths`, `bridge_terminal_nodes`,
`coupler_source_paths`, `route` gegen `route_orthogonal`). Pro Wire **ein**
Versuch und kein Rip-up:

- der Korridor um den eigenen Weg wird um `ref_corridor_expansion` geweitet;
- die Ringnachbarn **i ± 4** stehen als harte Hindernisse, mit der engeren
  Clearance `ref_min_clearance`;
- bepreist wird mit dem Zerfallsfeld `compute_corridor_proximity_decay`
  (`ref_proximity_penalty_cost`), das den Weg in die Mitte seines Kanals
  zieht;
- der neue Weg ersetzt den alten, sonst Rollback auf das Backup
  (`backup_dubin_path`), über `ref_refinement_rounds` (2) Runden;
- `outer_use_meander` / `outer_meander_length`: die Meander-Einfügung
  (`meander_insertion_proximity_strict`) mit den Rändern
  `outer_meander_insertion_boundary`, nach dem Routing des Resonators.

Die Parameter stehen in `FinalGridParams` (`FinalGrid.hpp` ab Zeile 752),
die Werte je Chip in den Treibern des Prototyps. Vergleiche Punkt für Punkt
mit `Driver::refine`: Was von den fünf Punkten haben wir, was fehlt, was ist
bei uns absichtlich anders (die Handover-Dokumente nennen jede Abweichung
mit Grund). **Die Meander-Einfügung ist bei uns bewusst anders** als im
Prototyp — vom Qubit her, kleinstes Spannmaß zuerst — und bleibt so.

## Wie gemessen wird

Die Figur dieser Phase ist zweiteilig und beide Teile stehen in jeder
Tabelle: `bad = unrouted + open + crossing` **darf nicht steigen**, und
`short + long` ist das, was du senkst. Ziel ist 0 short und 0 long bei
unverändertem oder besserem bad, Chip für Chip gegen `squeeze-reject2`.
Dazu die Laufzeit der Phase in Sekunden und, wie bisher, die Zahl der
Wires, die in der letzten Runde keinen Weg fanden. Nie `Fails:` zitieren.

Jeder Arm läuft über `run-arm.sh`, die großen Chips einer nach dem anderen,
Logs unter `artifacts/logs/<arm>/`. Für jede neue Regel ein Schalter, eine
Kontrollmessung ohne sie, und die Zahlen im Kommentar des Schalters, auch
die negativen.

## Fallen, die schon bekannt sind

- Eine Schleife, die der Korridor nicht fasst, lässt den Resonator short;
  `refine` behält dann den alten Weg. Die Frage ist, ob der Korridor der
  Phase weit genug ist (der Prototyp weitet ihn) und ob die Schleife nahe am
  Qubit Platz findet, wo die Coupler-Kante und die Feedline nicht liegen.
- Ein verlängerter Resonator ist länger im Feld: er kann Nachbarn, die heute
  sauber sind, in die Clearance drücken. `bad` ist deshalb Teil der Figur.
- Die Terminal-Kanten der Chains sind seit heute an ihrem Coupler-Ende Wand
  (`SCPD_TERMINAL_STUB_GUARD`), die Squeeze-Regel (`SCPD_SQUEEZE_REJECT`)
  greift in jeder Kantensuche der Insertion; beides ist in Phase 5 nicht
  aktiv, weil dort keine Kante neu gesucht wird — es sei denn, du baust das.
- Die Längen-Bänder (`closeLengthBands`) sind in Feedline-Pässen aus und
  bleiben es; ihr Zweck war die Position des Couplers, die hier feststeht.
- `plan --stage final` liest die Konfiguration aus dem Laufverzeichnis, nicht
  von der Kommandozeile; ein neuer Quelltext braucht in der
  `pipeline/CMakeLists.txt` einen Touch, damit der Glob ihn sieht.

## Arbeitsweise

Zuerst lesen und berichten: `Driver::refine` gegen den Prototyp, die
Zahlen von oben, ein Lauf auf 17q mit `feedline_refinement_rounds = 2`,
`SCPD_FEEDLINE_MEANDER=1` und `stop_after = "refined"` im Laufverzeichnis,
um zu sehen, was der Rumpf schon tut. Dann die Richtung mit dem Nutzer
festlegen. Am Ende ein `handover-feedline-refinement.md` nach dem Muster
der anderen Handover-Dokumente: Stand, Schalter, Messungen, Negativergebnisse,
wo die Teile liegen, was nicht getan ist, Tests. CHANGELOG-Eintrag unter
`[Unreleased]`, keine deutschen Texte im Repository außer den Prompts.
