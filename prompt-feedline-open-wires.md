# Auftrag: die letzten offenen Wires im Feedline-Routing

Deine Aufgabe ist es, die **51 offenen Wires** zu beseitigen, die auf 45q, 57q
und 69q nach dem Feedline-Routing übrig bleiben. Alle anderen Chips sind
sauber, alle 305 Feedline-Kanten werden gezeichnet, und die drei Prüfungen am
Ende der Coupler-Einfügung sind ohne Befund.

**Der Verdacht, von dem du ausgehst:** Im Bild sind etliche dieser Wires
offensichtlich routbar — es ist sichtbar Platz. Sie werden trotzdem als
offen gemeldet. Es liegt also nahe, dass nicht die Geometrie das Problem ist,
sondern was das Routing sich selbst verbietet. **Der Weg dorthin führt über
den Vergleich mit dem Prototyp.**

## Lies das zuerst

- `handover-feedline-routing.md` — Stand, die drei Prüfungen, die vollständige
  Hinderniskonstruktion, alle Schalter, fünf bereits gefundene Fehler und die
  Negativergebnisse. **Die Negativergebnisse sind wichtig: zahle nicht
  zweimal dafür.**
- `handover-cpw-coupler-insertion.md` und `handover-chain-astar.md` — der
  Boden, auf dem das steht.
- Die Bilder mit markierten offenen Wires:
  <https://claude.ai/artifact/W5TBNrQKX7fxJS7T21g3Jo> (zoombar bis 16×).

Checkout `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`, Branch
`phase-2-grid-router`, HEAD `435ca78`. **Alles ist unkommittiert** — der
Nutzer committet selbst, pro Phase. Lass die Arbeit im Arbeitsbaum.

Der Prototyp ist
`/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD/include/fiction/layout/FinalGrid.cpp`,
die Funktion heißt `run_final_routing_feedline_parallel`.

## Der stärkste Hinweis

**49 der 51 offenen Wires liegen in zusammenhängenden Läufen benachbarter
Ring-Indizes**, nur zwei stehen allein:

```
45q  12 offen:   4–5 · 10–11 · 65–66 · 116–117 · 131–133 · 136
57q  19 offen:   9–12 · 86–87 · 91–92 · 132–133 · 147–153 · 162–163
69q  20 offen:   8–11 · 19–20 · 99–102 · 106–107 · 179–180 · 185–186 · 201–203 · 206
```

Benachbarte Ring-Indizes sind benachbarte Resonatoren. Das sieht nach
**gegenseitiger Blockade** aus: jeder hält dem anderen den Platz besetzt, und
keiner von beiden weicht, weil der Sweep sie einzeln anfasst. Die Diagnose
dafür ist schon im Code — mit `-v 1` schreibt jede Runde

```
45 is open in the room of 46 at (210,1109), 170 from its source and 159 from its target, stub 11
```

und sagt damit, **wer** blockiert, **wo**, wie weit von den eigenen Enden und
ob die Zelle im eigenen Zwangslauf liegt (`IN ITS OWN STUB`). Fang damit an:
Sind die Läufe wirklich gegenseitige Paare, oder blockiert sie alle derselbe
Dritte?

## Wie du einen Engpass von einem Fehler unterscheidest

Das ist das schärfste Werkzeug, das du hast, und das Dashboard ist genau
dafür gebaut: Zeilen sind Wires, Spalten sind Runde × Relaxationsstufe.

**Was normal ist und wahrscheinlich *kein* Fehler ist:** In Runde 1 scheitert
Wire 1 und Wire 2 kommt durch, in Runde 2 andersherum. So sieht es aus, wenn
die Geometrie wirklich zu wenig Luft hat — zwei Wires teilen sich einen Platz,
der nur für einen reicht, und wer ihn bekommt, hängt daran, wer zuerst dran
ist. Dagegen hilft nur mehr Platz oder eine andere Platzierung.

**Was verdächtig ist:** Ein Wire, das in **jeder** Runde und auf **jeder**
Relaxationsstufe scheitert. Wenn fünf Stufen nacheinander immer mehr Nachbarn
loslassen und sich am Ergebnis nichts ändert, dann liegt es nicht an den
Nachbarn. Dann steht etwas im Weg, das durch keine Relaxation verschwindet —
eine Wand, die eigentlich ein Preis sein sollte, ein Zaun, der zu früh oder
zu spät gebaut wird, ein Feld, das nicht angehängt ist. **Genau so sah der
abgehängte Proximity-Zeiger aus**, der zuvor gefunden wurde.

Diese Kandidaten liefert der jetzige Stand frei Haus (sechs Runden im
Feedline-Pass, gezählt sind die Suchen ohne Weg je Wire):

```
45q    6× Wire 4     6× Wire 131    3× 65, 66, 116, 120
57q    7× Wire 133   6× Wire 151    6× 147    6× 10    5× 86
69q    4× Wire 185   4× 107         4× 102    3× f61, f6, 99, 9, 8
```

**Fang bei Wire 133 auf 57q und Wire 4 und 131 auf 45q an.** Sieben
beziehungsweise sechs vergebliche Suchen, bei fünf Relaxationsstufen und
wachsendem Band — das ist kein Platzmangel, der sich ausgehen sollte. Schau
dir im Dashboard an, ob überhaupt je eine Stufe erreicht wird, in der der
Blockierer losgelassen ist, und wenn ja, was dann noch im Weg steht.

## Was zu vergleichen ist

Der Prototyp löst dieselbe Aufgabe. Die Unterschiede, die du prüfen solltest,
sind nicht alle schon gemessen:

1. **Wie viele Nachbarn gezäunt werden.** Der Prototyp zäunt `i ± 1` und
   `i ± 2`, also vier. Prüfe, ob das bei uns wirklich dasselbe ist — und was
   passiert, wenn ein Paar sich gegenseitig blockiert, bei dem beide im Zaun
   des anderen stehen.
2. **Die Reihenfolge, in der gerippt wird.** Der Sweep geht abwechselnd
   vorwärts und rückwärts über den Ring. Ein gegenseitiges Paar braucht, dass
   **beide gleichzeitig** losgelassen werden; wenn der Sweep immer nur einen
   anfasst, kann er so ein Paar nie auflösen. Sieht der Prototyp das anders?
3. **Die Relaxationsstufen.** Fünf Stufen, und in jeder wird ein weiterer
   Nachbar losgelassen. Prüfe am Dashboard, ob die offenen Wires überhaupt je
   eine Stufe erreichen, in der ihr Blockierer losgelassen ist.
4. **Die Bandbreite je Runde.** Unter `SCPD_FEEDLINE_PROTOTYPE` wächst das
   Band mit `(Runde + 1) × expansion`. Reicht das, oder stehen die offenen
   Wires an der Bandgrenze statt am Nachbarn?
5. **Der Preis.** Die Proximity war lange an einem Feld aus Nullen gemessen —
   das ist behoben, aber **alle Preis-Sweeps davor sind damit wertlos**. Eine
   Wiederholung kann sich lohnen.

## Werkzeug, das schon da ist

```bash
# Ein Chip, nur die Final-Stufe, mit Zeile je Wire und Suche:
.venv/bin/mqt-scpd plan -c benchmarks/45q/config.toml -o artifacts/45q --stage final -v 1

# Dasselbe mit Debug-Bildern und den drei Dashboards (45q: ~5 GB, ~8 min):
.venv/bin/mqt-scpd plan -c benchmarks/45q/config.toml -o artifacts/45q --stage final -v 1 -d
```

`--stage final` setzt auf den vorhandenen Artefakten auf und rechnet die
früheren Stufen **nicht** neu; es schlägt fehl, wenn sie fehlen. Die
Dashboards landen in `<run>/debug/dashboard-{feedline,outer,inner}.html` und
zeigen als Tabelle, welcher Wire in welcher Runde und Relaxationsstufe
scheitert, mit Klick auf den jeweiligen Einzel-Viewer.

Bauen und installieren nach einer Quelltextänderung:

```bash
cmake --build build/cp311-abi3-macosx_15_0_arm64/Release --parallel
cp build/cp311-abi3-macosx_15_0_arm64/Release/bindings/pyscpd.abi3.so \
   .venv/lib/python3.13/site-packages/mqt/scpd/pyscpd.abi3.so
codesign -s - --force .venv/lib/python3.13/site-packages/mqt/scpd/pyscpd.abi3.so
```

**Tausche die Bibliothek nie aus, während ein Lauf läuft.**

## Wie gemessen wird

Jede Änderung geht über **alle acht Chips**, nicht über einen. Die Zahlen, an
denen du dich misst, stehen in der letzten Zeile des Laufs:

```
final routing: 209 of 209 drawn, 0 unrouted, 12 open, 8 short, 6 long, 0 crossing a feedline | Fails: 24
```

Der Ausgangsstand, gegen den verglichen wird:

| | unrouted | offen | Fails |
|---|---|---|---|
| alle acht Chips | 0 | **51** | 146 |

**Offen ist die Zielgröße.** Die 108 Längenfails sind der abgeschaltete
Meander (`SCPD_FEEDLINE_MEANDER=0`) und nicht deine Aufgabe — aber lass sie
nicht schlechter werden. Und die drei Prüfungen müssen grün bleiben:

```
CHECK feedline room — 0 … GREEN
CHECK coupler crossings — 0 … GREEN
CHECK resonator crossings — 0 … GREEN
```

## Arbeitsweise

Jede neue Regel kommt hinter einen `SCPD_*`-Schalter mit dokumentiertem
Default, damit sie einzeln messbar und einzeln zurücknehmbar ist. Schreib die
Messung in den Kommentar — auch die negative. Der Nutzer entscheidet die
Richtung, also melde Zwischenstände und rate nicht, was er als Nächstes will.

Zwei Dinge, die vor einem Commit zu tun sind und noch offen stehen:
`ctest --test-dir build/release` ist seit dem 2. Oktober nicht mehr
vollständig gelaufen, und `SCPD_ORTHO_CROSSING` steht auf aus — die
orthogonale Kreuzungsregel ist die richtige Regel und soll zurückkommen,
sobald das Rip-up steht.
