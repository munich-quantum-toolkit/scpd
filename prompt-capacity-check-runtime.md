# Auftrag: Kapazitätsprüfung in der Coupler-Insertion – Laufzeit und schnellere Verfahren

Nach der Coupler-Insertion gibt es eine Bottleneck- und Kapazitätsanalyse
(`SCPD_BOTTLENECKS=1`). Sie schneidet den freien Raum an seinen Engstellen in
Partitionen und prüft mit einem ganzzahligen Multi-Commodity-Fluss, ob jeder
äußere Draht von der Partition seines Quell-Terminals zur Partition seines
Ziel-Terminals kommt. Heute läuft sie nur als Bericht, einmal am Ende der
Insertion.

Der Nutzer will sie als Nächstes **in** die Coupler-Insertion einbauen: Eine
Feedline-Kante, nach der der Graph nicht mehr erfüllbar ist, soll als ungültig
ausgeschlossen werden, bevor der Feedline-Pass an ihr scheitert.

Deine Aufgabe hat drei Teile:

0. Mach dich mit der Analyse vertraut.
1. Bewerte die Laufzeit eines solchen Einbaus.
2. Untersuche Verfahren für die Erfüllbarkeitsprüfung, die schneller sind als
   der heutige Fluss mit HiGHS.

Du **misst und schlägst vor**. Du baust den Einbau nicht. Was eingebaut wird,
entscheidet der Nutzer nach deinem Bericht.

## Lies das zuerst

- `handover-bottleneck-analysis.md` – die ganze Analyse: Wände, Port-Läufe mit
  Schlitz, Schnitte, Partitionen, Kapazitätsgraph, Fluss, JSON und HTML-Seite,
  alle Schalter und Umgebungsvariablen, der Stand auf 17q, die offenen Punkte.
- `handover-cpw-coupler-insertion.md` und `handover-chain-astar.md` – die
  Insertion und die Präfix-Suche (A\* über Coupler-Optionen pro Chain). Dort
  findest du die Stellen, an denen eine Kante gesucht, bewertet und
  festgeschrieben wird. Das sind die Kandidaten für einen Einbau.
- `AGENTS.md` – die Regeln des Repos. Halte dich daran.

Checkout `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`, Branch
`phase-4-routing-stages`, HEAD `712cf7a`
**plus unkommittierte Arbeit vom 2026-10-07/08**: die Analyse mit Schlitzen, die
Schlitz-Regeln in `src/grid/Bottlenecks.cpp`, die JSON-Daten und die HTML-Seite.
**Alles bleibt unkommittiert** – der Nutzer committet selbst, pro Phase. Leg
keine Commits an.

## Schritt 0: die Analyse ansehen

Lass sie einmal auf 17q laufen und öffne die Seite:

```zsh
cd /Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4
cmake --build build/cp311-abi3-macosx_15_0_arm64/Release --target mqt-scpd-bindings
OVERLAY=py-cg zsh artifacts/logs/dev-sync.sh
SCPD_DEV_OVERLAY=artifacts/dev/py-cg \
SCPD_BOTTLENECKS=1 SCPD_LAUNCHER_FENCE_TURN=1 SCPD_SEARCH_PICTURES=0 \
.venv/bin/python artifacts/logs/dev-mqt-scpd.py plan \
  -c benchmarks/17q/config.toml -o artifacts/dev/17q-cg --stage final -v 1 -d
```

Die Seite liegt in `artifacts/dev/17q-cg/debug/final-capacity-graph.html`. Die
Log-Zeilen `coupler insertion: BOTTLENECKS —` und `CAPACITY GRAPH …` nennen am
Ende die Zeit jedes Schritts.

Lies den Code der Kette nach:

- `reportBottlenecks` und `wallsAfterInsertion` in
  `src/pipeline/FinalRouter.cpp`;
- `grid::findBottlenecks` in `src/grid/Bottlenecks.cpp`;
- `grid::chambersOf` in `src/grid/Chambers.cpp`;
- `reportCapacityGraph`, `crossingStretches`, `portsOf` in `FinalRouter.cpp`;
- `pipeline::checkCapacity` in `src/pipeline/CapacityFlow.cpp`.

Ändere an der Bedeutung der Analyse nichts. Der Nutzer hat eine offene Frage
dazu (die Kapazität eines Schnitts am eigenen Stub, siehe *What is open* im
Handover). Die entscheidet er, nicht du.

## Der Befund, von dem du ausgehst

17q, letzter Lauf (`artifacts/logs/cg/17q-arms.log`):

| Schritt                                    | Zeit            |
| ------------------------------------------ | --------------- |
| Wände                                      | 0,011 s         |
| Abstandstransformation                     | 0,065 s         |
| Voronoi / Mittelachse                      | 0,043 + 0,006 s |
| Schnitte                                   | 0,010 s         |
| Partitionen                                | 0,022 s         |
| Querungsstrecken und Ports                 | 0,002 s         |
| Fluss (HiGHS)                              | 0,304 s         |
| **Analyse gesamt**                         | **etwa 0,47 s** |
| zum Vergleich: die ganze Coupler-Insertion | 41,7 s          |

Der Graph hat 321 Partitionen, 214 Schnitte und 67 Querungsstrecken. Es gibt 58
Bedarfe; das Ergebnis ist UNSAT mit 19 überlaufenden Kanten. Das Raster hat 1425
× 1425 Zellen.

Die Insertion bewertet auf 17q in der Größenordnung Tausende Optionen (die
Log-Zeile `… of … option costs` am Ende der Insertion) und schreibt rund 20
Kanten fest. Eine volle Neuberechnung pro Kandidat kostet also grob Tausende ×
0,47 s. Das ist der Grund für diesen Auftrag.

## Teil 1: Laufzeit eines Einbaus bewerten

1. **Wo würde geprüft?** Benenne die Stellen der Insertion, an denen eine
   Feedline-Kante als ungültig ausgeschlossen werden könnte, mit Funktionen und
   Datei. Zum Beispiel: beim Festschreiben einer Kante, beim Bewerten einer
   Option in der Präfix-Suche, nach jeder fertigen Chain. Zähle auf 17q, wie oft
   jede Stelle durchlaufen wird.
2. **Was kostet eine Prüfung?** Miss die Schritte einzeln, mehrfach, und trenne
   Kaltstart von Wiederholung. Die Zeitmessung pro Schritt ist schon im Log;
   ergänze bei Bedarf eigene Zähler hinter einem Schalter mit Standard aus.
3. **Was lässt sich inkrementell machen?** Eine neue Feedline-Kante ändert die
   Wände nur in einem Band um ihren Weg. Prüfe und schätze mit Messungen ab:
   - lokale Aktualisierung der Abstandstransformation und der Mittelachse in
     einem Fenster;
   - nur die Schnitte und Partitionen neu, die das Fenster berühren;
   - den Fluss warm starten oder nur die betroffenen Bedarfe neu lösen.

   Sag jeweils, was es spart und was es an Genauigkeit kostet. Die Partitionen
   hängen an Schnitten, die weit reichen können.
4. **Skalierung:** Miss die einmalige Analyse auch auf 69q, dem größten Chip, um
   die Größenordnung dort zu kennen. Alles Weitere misst du auf 17q.
5. **Ergebnis:** eine Tabelle Stelle × Aufrufe × Kosten pro Aufruf (voll /
   inkrementell) × geschätzte Mehrzeit der Insertion, und deine Empfehlung, wo
   und wie geprüft werden sollte.

## Teil 2: schnellere Verfahren für die Erfüllbarkeit

Heute löst `checkCapacity` ein ganzzahliges Programm mit HiGHS:

- jede Kante ist ein eigener Knoten;
- je Kante gibt es eine ganzzahlige Überlaufvariable;
- das Ziel ist der kleinste Gesamtüberlauf, danach die wenigsten Bögen.

Für den Ausschluss einer Kante reicht oft die Frage „erfüllbar ja/nein“.
Untersuche und **miss** auf dem 17q-Graphen mindestens:

- **die reine Entscheidungsfrage:** Kapazität als harte Grenze, kein Überlauf,
  kein Optimieren, Abbruch bei der ersten zulässigen Lösung;
- **die LP-Relaxation als schneller Vorfilter:** Ist schon der gebrochene Fluss
  unzulässig, ist es der ganzzahlige auch. Wie oft entscheidet sie allein?
- **Schnittbedingungen als Vorfilter:** Für jede Engstelle bzw. jeden Schnitt
  des Graphen dürfen die Bedarfe, die ihn queren müssen, seine Kapazität nicht
  übersteigen. Das ist notwendig und billig über Max-Flow/Min-Cut. Wie viele der
  19 Überläufe findet das?
- **Zerlegung:** Zweifach zusammenhängende Komponenten, Serienreduktion von
  Partitionen ohne Port, eine Teilaufgabe pro Chain oder Bereich. Der Graph ist
  fast ein Ring.
- **andere Löser:** CP-SAT (OR-Tools), ein SAT- oder Pseudo-Boolean-Löser, oder
  eine Heuristik (gierig plus Reparatur), die nur im Zweifel den exakten Löser
  ruft. Neue Abhängigkeiten nur nach Rücksprache mit dem Nutzer; Abhängigkeiten
  werden über `cmake/ExternalDependencies.cmake` geholt.
- **inkrementelles Lösen:** Nach einer kleinen Änderung des Graphen die alte
  Lösung als Start nutzen.

Für jedes Verfahren gilt:

- Gleiche das Ergebnis mit dem heutigen Fluss ab (SAT/UNSAT und welche Kanten
  überlaufen). Ein Verfahren, das ein anderes Urteil gibt, sagt das deutlich und
  sagt, ob es notwendig oder hinreichend ist.
- Miss die Zeit pro Aufruf, auf 17q und einmal auf 69q.

## Messregeln

- Miss auf 17q mit
  `SCPD_BOTTLENECKS=1 SCPD_LAUNCHER_FENCE_TURN=1 SCPD_SEARCH_PICTURES=0`. Für
  reine Zeitmessungen lass `-d` und `-v` weg, weil die Bilder und Logzeilen
  selbst Zeit kosten.
- Wiederhole jede Zeitmessung mindestens dreimal und nenne Median und
  Spannweite.
- Lege Messcode hinter einen Schalter `SCPD_…` mit Standard aus. Die Analyse
  ohne Schalter muss Wort für Wort dieselben Log-Zeilen geben wie vorher.
- Dev-Läufe nur über das Overlay (`artifacts/logs/dev-sync.sh`,
  `SCPD_DEV_OVERLAY`). Installiere nicht ins `.venv`, solange ein Lauf darauf
  läuft.
- Logs nach `artifacts/logs/<name>/`, Laufordner nach `artifacts/dev/<name>/`.

## Regeln des Repos, die hier zählen

- Tests für jede Codeänderung, im passenden Testbaum. Die Tests des
  Kapazitätsflusses liegen in `test/pipeline/test_capacity_flow.cpp`, die der
  Engstellen in `test/grid/test_bottlenecks.cpp`.
- `uvx nox -s lint` schreibt mit `--all-files` rund 130 fremde Dateien um. Setze
  die zurück und prüfe nur deine Dateien:
  - `SKIP=clang-format .nox/lint/bin/prek run --files <deine Dateien>`;
  - clang-format nur auf deine geänderten Zeilen, mit `--lines` als **Array** in
    zsh. Eine Variable mit mehreren Flags formatiert sonst die ganze Datei.
- Bekannter Fehlschlag, nicht von dir:
  `FinalRouter.OnlyUnsettledRedrawsOneWire`.
- Schreib Code, Kommentare und Dokumente auf Englisch. Antworte dem Nutzer auf
  Deutsch.

## Was du ablieferst

1. **`plan-capacity-check-in-insertion.md` (Englisch):**
   - die Einbaustellen mit Aufrufzahlen;
   - die Kosten voll und inkrementell;
   - die gemessenen Verfahren aus Teil 2 mit Zeiten und Urteilsabgleich;
   - deine Empfehlung: welches Verfahren, an welcher Stelle, mit welcher
     erwarteten Mehrzeit;
   - was dabei an Genauigkeit verloren geht.
2. Eine kurze deutsche Zusammenfassung an den Nutzer mit den Zahlen und den
   Entscheidungen, die er treffen muss.
3. Den Arbeitsbaum **unkommittiert**, mit dem Stand der Tests.
