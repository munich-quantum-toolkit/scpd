# Auftrag: gezielte Neusuche der Coupler-Optionen nach dem Feedline-Pass

Deine Aufgabe ist es, die **verbliebenen offenen Drähte** des Feedline-Routings
zu beseitigen, indem du den blinden `repair` (Coupler neben einem Fail auf
eine seiner zwei billigsten anderen Optionen drehen, ganzen Ring neu
zeichnen, bei streng weniger Fails behalten) durch eine **gezielte
Neusuche** ersetzt: Fails werden auf die Chain-Segmente zurückgeführt, die
sie verursachen, und genau diese Segmente werden mit der Präfix-Suche
(A\* über Coupler-Optionen) neu gesucht — beide Enden fest, der Rest des
Chips eingefroren — wobei der A\* über seine erste Antwort hinaus läuft und
jede vollständige Optionsfolge mit einem **lokalen Rip-up-and-Reroute** auf
Legalität geprüft wird, unter der Orthogonalregel.

Der vollständige Plan mit allen Codestellen, Mechanismen, Schaltern, dem
Messprotokoll, den Risiken und den Tests steht in
`plan-coupler-repair-search.md` (Englisch, am 2026-10-04 freigegeben). Dieses
Dokument sagt, worum es geht und woran du dich misst.

## Lies das zuerst

- `plan-coupler-repair-search.md` — der Plan. Jede Codestelle wurde am
  2026-10-04 geprüft; die Zeilennummern driften, sobald du Code einfügst, also
  grep die Bezeichner.
- `handover-feedline-routing.md` — der Stand des Feedline-Passes, die beiden
  Zählweisen für „offen", die Fallen, die Negativergebnisse.
- `handover-cpw-coupler-insertion.md`, Abschnitt *The room rules* — was die
  Platzregel-Kalibrierung gemessen hat und warum dieser Plan keine Heuristik
  mehr ist; dann `handover-chain-astar.md` für die Präfix-Suche, auf der die
  Neusuche aufsetzt.

Checkout `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`, Branch
`phase-4-routing-stages`, HEAD `63851cf` **plus die unkommittierte
Platzregel-Arbeit vom 2026-10-04** (RoomRules-Bibliothek, R0, Kalibrierzeilen,
`CHECK bridges`, `CHECK chain channels`). **Alles bleibt unkommittiert** — der
Nutzer committet selbst, pro Phase.

## Der Befund, von dem du ausgehst

Ohne Umgebungsvariable lässt der Feedline-Pass 7 Drähte in der letzten Runde
offen (45q 2, 57q 1, 69q 4), 18 am Ende. Mit `SCPD_ORTHO_CROSSING=1`, dem
Regime, das jetzt Default wird:

| | 4q | 9q | 17q | 21q | 33q | 45q | 57q | 69q | alle |
|---|---|---|---|---|---|---|---|---|---|
| offen, letzte Runde | 0 | 0 | 4 | 0 | 3 | 2 | 9 | 10 | 28 |
| offen, Ende | 0 | 0 | 4 | 0 | 4 | 6 | 17 | 24 | 55 |
| kreuzt Feedline | 0 | 0 | 4 | 0 | 1 | 2 | 8 | 7 | 22 |

Die Platzregeln (R1–R4) haben am 2026-10-04 gezeigt, dass keine lokale
Geometrieregel am Coupler die offenen Paare von gesunden Couplern trennt.
Zwei weitere Befunde trägt der Plan: der heutige `repair` nominiert für einen
Draht, der nur eine Feedline kreuzt, gar keinen Coupler; und auf 69q kreuzen
sich die terminalen Kanten von vier Paaren benachbarter Chains, ohne dass
eine CHECK-Zeile es sagt.

## Die Entscheidungen des Nutzers (2026-10-04)

1. **Regime:** `SCPD_ORTHO_CROSSING=1` wird Default. Zielgröße ist offen in
   der letzten Runde plus kreuzende Drähte (plus unrouted). Die
   Default-aus-Zahlen werden einmal als Kontrollarm berichtet, nichts wird
   darauf abgestimmt.
2. **Einbau:** anstelle von `Driver::repair` in Phase 4, im selben Lauf nach
   den beiden Sweeps; `repair_trials` wird zum Budget (Legalitätstests pro
   Chip). Der Suchzustand bleibt im Speicher.
3. **Kriterium:** eine Kandidaten-Optionsfolge wird genommen, wenn ihr
   lokales Fenster mit 0 offen, 0 kreuzend, 0 unrouted endet und die Zahl
   (offen + kreuzend + unrouted) über alle Drähte streng unter der vor dem
   Segment liegt. Längen sind kein Kriterium. Ein voller Pass danach ist ein
   gemessener Arm, kein Teil des Kriteriums.
4. **k:** Schalter, Startwert 2, Sweep 1 / 2 / 3.

## Woran du dich misst

Jeder Arm über **alle acht Chips**, einer nach dem anderen, ohne `-d`, Regel
an:

- `==> coupler insertion: 0 feedline edges NOT drawn`, die **vier**
  `CHECK`-Zeilen grün (die neue heißt `CHECK feedline crossings`),
  `0 unrouted` in der Endzeile.
- `bad = offen + kreuzend + unrouted` pro Chip nicht über `base-ortho`, die
  Summe darunter; 4q / 9q / 21q bleiben bei 0 — eine Änderung dort ist ein
  Fehler der Neusuche, kein Ergebnis.
- Jeder neue Schalter aus → `base-ortho` byteidentisch in den `==>`-, `CHECK`-,
  `feedline routing`- und `final routing`-Zeilen (`artifacts/logs/keylines.sh`).
- Sekunden der Einfügung, des Repairs und der Stufe daneben; Short und Long
  sind **nicht** die Zielgröße.

Melde vor dem Sweep (Schritt 8 des Plans) den Stand nach Schritt 5: welche
Segmente die Triage auf 45q / 57q / 69q baut, ob sie Coupler 64 (45q), 8
(57q), 9 (69q) und die kreuzenden terminalen Paare von 69q enthalten, und wie
breit sie sind. Der Nutzer entscheidet dann die Richtung des Sweeps.

## Werkzeug, das schon da ist

```bash
# Bauen und installieren nach jeder Quelltextänderung (nie während ein Lauf läuft):
cmake --build build/cp311-abi3-macosx_15_0_arm64/Release --parallel
cp build/cp311-abi3-macosx_15_0_arm64/Release/bindings/pyscpd.abi3.so \
   .venv/lib/python3.13/site-packages/mqt/scpd/pyscpd.abi3.so
codesign -s - --force .venv/lib/python3.13/site-packages/mqt/scpd/pyscpd.abi3.so

# Ein Chip, nur die Final-Stufe:
cp benchmarks/45q/config.toml artifacts/45q/config.toml
.venv/bin/mqt-scpd plan -c benchmarks/45q/config.toml -o artifacts/45q --stage final -v 1

# Alle acht Chips eines Arms, nacheinander, mit Spotlight-Anzeige und CPU-Zeit:
./artifacts/logs/run-arm.sh <arm> [ENV=WERT ...]      # -> artifacts/logs/<arm>/<chip>.log
./artifacts/logs/keylines.sh artifacts/logs/<arm>/45q.log   # die Zeilen, nach denen ein Arm beurteilt wird
python3 artifacts/logs/calibration.py artifacts/logs/<arm>/45q.log
.venv/bin/python artifacts/logs/fcross.py artifacts/69q     # Feedline-Kanten, die sich Zellen teilen
.venv/bin/python artifacts/logs/pinch.py artifacts/45q feedlines 2806 1332 22 23 65 66

# Tests (die mehrstündigen Final-Suiten ausgenommen; vier Fehlschläge sind Vorbestand):
cmake --build --preset release && ctest --test-dir build/release -E 'EveryChip/(Final|SpacedFinal)' -j 6
```

Die Baselines vom 2026-10-03/04 liegen unter `artifacts/logs/base`
(Defaults), `artifacts/logs/base-ortho` (Regel an), `control`, `calib`,
`calib2` (Platzregel-Report). Vor einem `-d`-Lauf `artifacts/<chip>/debug`
löschen, die CLI leert es nicht. Spotlight vor jedem großen Chip prüfen:
`ps -A -o %cpu,comm -r | awk 'NR>1 && ($2 ~ /mds_stores|mediaanalysisd/) {s+=$1} END {print s"%"}'`.

## Arbeitsweise

Jede neue Regel und jeder Mechanismus hinter einen `SCPD_*`-Schalter mit
dokumentiertem Default und der gemessenen Begründung im Kommentar, auch wenn
die Messung negativ ausfällt, und in der `settings:`-Zeile. Erst die
Identität (alle neuen Schalter aus) muss `base-ortho` byteidentisch
reproduzieren, dann die Arme. Melde Zwischenstände und rate nicht, was der
Nutzer als Nächstes will — er entscheidet die Richtung. Handover-Dokumente am
Ende aktualisieren (`handover-feedline-routing.md`, neu
`handover-targeted-repair.md`).
