# Einstiegs-Prompt

Zum Kopieren in eine frische Sitzung. Er macht nichts als hinzeigen — alles
Weitere steht in den Dokumenten.

---

Du übernimmst die Coupler-Einfügung der Final-Stufe in
`/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4` (Branch
`phase-4-routing-stages`, HEAD `63851cf`). Ziel sind harte Platzregeln in der
Einfügung, die Coupler-Optionen ausschließen, welche den Feedlines und den
Ringdrähten zu wenig Raum lassen — keine Kostenfunktion.

Lies dazu **in dieser Reihenfolge**:

1. `prompt-coupler-room-rules.md` — dein Auftrag: der Befund (vier der neun
   verbliebenen Konflikte sind Engstellen, die die Chain-Geometrie am Coupler
   erzeugt), die Regeln R0–R4 in Kurzform, meine Entscheidungen und die
   Zahlen, an denen du dich misst.
2. `plan-coupler-room-rules.md` — der freigegebene Plan mit allen
   Codestellen, Formeln, Schaltern, der Stand-down-Leiter, dem Messprotokoll,
   den Risiken und den Tests. Die Zeilennummern wurden am HEAD geprüft und
   driften, sobald du Code einfügst: grep die Bezeichner.
3. `handover-feedline-routing.md` — der Stand des Feedline-Passes, die beiden
   Zählweisen für "offen", die Fallen und die Negativergebnisse; danach
   `handover-cpw-coupler-insertion.md` und `handover-chain-astar.md` für die
   Einfügung und die Präfix-Suche.

Dann arbeite den Plan in seiner Reihenfolge ab: Schritt 1 (Baseline-Logs
aller acht Chips, auch mit `SCPD_ORTHO_CROSSING=1`), Schritt 2 (die
router-freie Geometrie-Bibliothek `RoomRules` mit Unit-Tests), Schritt 3 (R0
und die Kalibrierzeilen an den gewählten Optionen, ohne dass eine Regel
ablehnt). **Melde mir die Kalibrierung, bevor du eine Regel scharf
schaltest**: die vier bekannten Fälle (45q Coupler 64 / f22–f23, 57q Coupler
8 / f2–f3, 69q Coupler 9 / Draht 10, 69q f5/f6 als R4) müssen rot sein, und du
sagst, wie viele gesunde Coupler und Kanten mit den Startwerten ebenfalls rot
wären. Die Richtung entscheide ich.

Woran du dich misst, je Arm über alle acht Chips, ein Chip nach dem anderen:
alle Feedline-Kanten gezeichnet, die drei `CHECK`-Zeilen grün, offen in der
**letzten Runde** ≤ 2 / 1 / 4 (45q / 57q / 69q) und am Ende ≤ 6 / 3 / 9, die
fünf kleinen Chips bei 0 nach beiden Zählweisen. Der Kontrollarm mit allen
Regeln aus muss die heutigen Zahlen zuerst byte-identisch reproduzieren.

Drei Dinge, die immer gelten: Alles liegt **unkommittiert** im Arbeitsbaum
und bleibt dort — ich committe selbst, pro Phase. Jede neue Regel kommt
hinter einen `SCPD_*`-Schalter mit dokumentiertem Default und der gemessenen
Begründung im Kommentar, auch wenn die Messung negativ ausfällt, und erscheint
in der `settings:`-Zeile der Einfügung. Und Short- und Long-Fails sind nicht
die Zielgröße; gezählt wird "offen in der letzten Runde".
