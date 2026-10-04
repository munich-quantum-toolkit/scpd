# Auftrag: Platzregeln in der Coupler-Einfügung (harter Ausschluss)

Deine Aufgabe ist es, die **nächsten offenen Wires** des Feedline-Routings zu
beseitigen, indem die Coupler-Einfügung Optionen **hart ausschließt**, die den
Feedlines und den Ringdrähten zu wenig Platz lassen. Keine Kostenfunktion:
eine abgelehnte Option oder Kante existiert für die Suche nicht.

Der vollständige Plan mit allen Codestellen, Formeln, Schaltern, der
Stand-down-Leiter, dem Messprotokoll und den Tests steht in
`plan-coupler-room-rules.md` (Englisch, am 2026-10-03 freigegeben). Dieses
Dokument sagt, worum es geht und woran du dich misst.

## Lies das zuerst

- `plan-coupler-room-rules.md` — der Plan. Jede Zeile darin wurde am Code
  geprüft; die Zeilennummern driften, sobald du Code einfügst, also grep die
  Bezeichner.
- `handover-feedline-routing.md` — der Stand des Feedline-Passes, beide
  Zählweisen für "offen", die Fallen, die Negativergebnisse, die Dashboards.
- `handover-cpw-coupler-insertion.md` und `handover-chain-astar.md` — die
  Coupler-Einfügung und die Präfix-Suche, auf denen die Regeln aufsetzen.

Checkout `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`, Branch
`phase-4-routing-stages`, HEAD `63851cf`. **Alles bleibt unkommittiert** —
der Nutzer committet selbst, pro Phase.

## Der Befund, von dem du ausgehst

Nach den Fixes vom 2026-10-03 (`SCPD_RESONATOR_STUB` aus, `SCPD_FENCE_FIXED`
an) bleiben, ohne Umgebungsvariable, ein Chip nach dem anderen:

| offen | 45q | 57q | 69q | kleine Chips |
|---|---|---|---|---|
| in der letzten Runde (**die Zielgröße**) | 2 (116, 65) | 1 (10) | 4 (176, 19, 9, f0) | 0 |
| am Ende der Stufe (Paare) | 6 | 3 | 9 | 0 |

Vier der neun Konflikte sind Engstellen, die die Chain-Geometrie **am
Coupler** erzeugt, und die Einfügung schaut dort nie hin:

- 45q 65/66: zwischen f22 und f23, den beiden Kanten an Coupler 64 (je 22
  Zellen), die hinter dem Pad auf dieselbe Seite zurückbiegen — ein U mit
  ~44 Zellen Rinne, wo zwei Drähte 3 × 19 brauchen.
- 57q 9/10: dieselbe Form an Coupler 8 (f2 bei 20, f3 bei 30).
- 69q 9/10/f1: der Lead von Resonator 9 liegt eine Zelle neben Draht 10.
- 69q 19/20: zwei Drähte zwischen den Terminal-Kanten f5 und f6 zweier
  Chains (chain-übergreifend; in diesem Schritt nur messen).

Die anderen fünf liegen am Rand oder am Gitter ohne Chain-Kante in 80 Zellen
Umkreis; sie sind nicht Sache der Einfügung.

Zwei Fakten tragen den Plan: Die Einfügung routet ihre Kanten gegen Artwork,
Pads, Leads und Resonatorschwänze, aber **nie gegen die Ringdrähte**. Und der
Feedline-Pass zeichnet **nur die Terminal-Kanten** neu; die Kanten zwischen
Couplern behalten die Wege der Einfügung bis zum Ende der Stufe. Eine
Engstelle zwischen zwei solchen Kanten lässt sich nur hier beheben. Der
Prototyp hat keine solche Regel; sein einziger Ansatz ist ein auskommentierter
Strafterm (`FinalGrid.cpp:18378`), dessen Kommentar genau unseren Fall
beschreibt.

## Die Regeln, kurz

- **R0** — je Kante die Ringdrähte, die sie kreuzen müssen (der Lauf von
  `assignBridges`, in die Einfügung vorgezogen, mit Richtungsprüfung).
- **R1** — Kreuzungskapazität: genug gerade Segmente auf der Kante für die
  Zahl der kreuzenden Drähte (Geradendefinition der Kreuzungsregel,
  Pitch `SCPD_ROOM_PITCH`, Rand `SCPD_ROOM_MARGIN`). Ablehnung in `edgeCost`.
- **R2** — die Rinne am Coupler: engster Abstand zwischen In- und Out-Kante
  hinter dem Pad gegen die Drähte, die dort hindurch müssen. Ablehnung in
  `step`, wo beide Kanten bekannt sind.
- **R3** — der eigene Platz der Option: Pad, Feedline-Run und Lead dürfen
  nicht im Kupfer (oder innerhalb R Zellen) eines Plain-Ringdrahts liegen.
  Ablehnung in `makeOption` über `free()`; lehnt den Platz ab, der Lauf
  über die Plätze geht weiter.
- **R4** — nur Bericht: eine CHECK-Zeile für Rinnen zwischen Kanten
  verschiedener Chains (der 69q-19/20-Fall).

Jede Regel hinter einem eigenen `SCPD_ROOM_*`-Schalter, jede Ablehnung mit
Zahlen im Log, Zähler in einer Summenzeile vor `==>`, alle Schalter in der
`settings:`-Zeile der Einfügung.

## Die Entscheidungen des Nutzers

1. Harter Ausschluss, keine Kostenfunktion.
2. **Stand-down-Leiter**: solange eine Chain joinbar bleibt, sind die Regeln
   hart; findet keine Optionsfolge mehr zusammen (auch nicht mit geöffneten
   Doglegs), werden die Regeln **nur für diese Chain** stufenweise aufgehoben
   (R2, dann R1, dann R3), jede Stufe laut geloggt. "Alle Feedline-Kanten
   gezeichnet" bleibt hartes Kriterium.
3. Abnahme gegen die heutigen Defaults (`SCPD_ORTHO_CROSSING` aus); jeder
   Arm zusätzlich mit `SCPD_ORTHO_CROSSING=1` gemessen und berichtet; der
   Default bleibt aus, bis der Nutzer entscheidet.
4. Erst Regeln innerhalb einer Chain; chain-übergreifend nur messen.

## Woran du dich misst

Jeder Arm über **alle acht Chips**, einer nach dem anderen, ohne `-d`:

- `==> coupler insertion: 0 feedline edges NOT drawn`, die drei
  `CHECK`-Zeilen grün, `0 unrouted` in der Endzeile.
- Offen in der **letzten Runde** ≤ 2 / 1 / 4 (45q / 57q / 69q), offen am
  Ende ≤ 6 / 3 / 9, die fünf kleinen Chips bei 0 nach beiden Zählweisen mit
  allen Kanten — eine Änderung auf einem kleinen Chip ist ein Fehler der
  Regel, kein Ergebnis.
- Fails und Einfügungszeit (heute 36 / 61 / 117 s) werden daneben
  berichtet; Short und Long sind **nicht** die Zielgröße.

Vor dem Scharfschalten: die Kalibrierzeilen aus Schritt 3 des Plans (R1 und
R2 an den gewählten Optionen ausrechnen, nichts ablehnen) — die vier bekannten
Fälle müssen rot sein und möglichst nichts sonst. Das kostet drei Läufe, ein
scharfer Sweep kostet Stunden.

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

# Tests (die mehrstündigen Final-Suiten ausgenommen; vier Fehlschläge sind Vorbestand):
cmake --build --preset release && ctest --test-dir build/release -E 'EveryChip/(Final|SpacedFinal)' -j 6
```

Die Dashboards aller acht Chips vom 2026-10-03 liegen unter
`artifacts/<chip>/debug/dashboard-{feedline,outer,inner}.html`; vor einem
neuen `-d`-Lauf das Verzeichnis löschen, die CLI leert es nicht. Bei einem
Wire, das überall scheitert, zuerst die `dead on arrival`-Zeile lesen; die
`in the way`-Zeile zäunt das zweite Nachbarpaar nie.

## Arbeitsweise

Jede neue Regel hinter einen `SCPD_*`-Schalter mit dokumentiertem Default und
der gemessenen Begründung im Kommentar, auch wenn die Messung negativ
ausfällt. Erst der Kontrollarm (alle Regeln aus) muss die heutigen Zahlen
byte-identisch reproduzieren, dann die Arme. Melde Zwischenstände und rate
nicht, was der Nutzer als Nächstes will — er entscheidet die Richtung.
Handover-Dokumente am Ende aktualisieren (`handover-feedline-routing.md`,
`handover-cpw-coupler-insertion.md`).
