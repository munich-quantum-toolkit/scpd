# Einstiegs-Prompt

Zum Kopieren in eine frische Sitzung. Er macht nichts als hinzeigen — alles
Weitere steht in den Dokumenten.

---

Du übernimmst den Repair der Final-Stufe in
`/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4` (Branch
`phase-4-routing-stages`, HEAD `63851cf` plus die unkommittierte
Platzregel-Arbeit vom 2026-10-04). Ziel ist eine gezielte Neusuche der
Coupler-Optionen anstelle des blinden `repair`: Fails werden auf
Chain-Segmente zurückgeführt, diese Segmente werden mit der Präfix-Suche neu
gesucht, und jede vollständige Optionsfolge wird mit einem lokalen
Rip-up-and-Reroute (Segment ± k Ringnachbarn) unter der Orthogonalregel auf
Legalität geprüft.

Lies dazu **in dieser Reihenfolge**:

1. `prompt-coupler-repair-search.md` — dein Auftrag: der Befund, die
   Entscheidungen des Nutzers und die Zahlen, an denen du dich misst.
2. `plan-coupler-repair-search.md` — der freigegebene Plan mit allen
   Codestellen, den vier Mechanismen (Accept-Haken im A\*, Sub-Chain-Problem
   mit Zaun, `Pass::onlyUnsettled`, Triage), den Schaltern, dem Messprotokoll,
   den Risiken und den Tests. Die Zeilennummern wurden am 2026-10-04 geprüft
   und driften, sobald du Code einfügst: grep die Bezeichner.
3. `handover-feedline-routing.md`, dann *The room rules* in
   `handover-cpw-coupler-insertion.md` und `handover-chain-astar.md`.

Dann arbeite den Plan in seiner Reihenfolge ab: Schritt 1 (Regel als
Default, DRC-Abgleich, Baseline `base-ortho2` und der Arm mit dem alten
`repair` bei 20 Trials), Schritt 2 (`checkFeedlineCrossings`), Schritt 3
(`Pass::onlyUnsettled`), Schritt 4 (`accept` im A\* mit Tests), Schritt 5
(die Triage `blameFails`). **Melde mir die Triage, bevor du die Neusuche
baust**: welche Segmente sie auf 45q / 57q / 69q bildet, ob Coupler 64, 8, 9
und die kreuzenden terminalen Paare von 69q darin liegen, und wie breit sie
sind. Dann Schritte 6 und 7, und vor dem Sweep (Schritt 8) wieder ein
Zwischenstand auf 17q. Die Richtung entscheide ich.

Woran du dich misst, je Arm über alle acht Chips, ein Chip nach dem anderen,
Regel an: alle Feedline-Kanten gezeichnet, die vier `CHECK`-Zeilen grün,
`bad = offen + kreuzend + unrouted` pro Chip nicht über `base-ortho` und in
der Summe darunter, 4q / 9q / 21q bei 0. Alle neuen Schalter aus muss
`base-ortho` byteidentisch reproduzieren.

Drei Dinge, die immer gelten: Alles liegt **unkommittiert** im Arbeitsbaum
und bleibt dort — ich committe selbst, pro Phase. Jeder neue Mechanismus kommt
hinter einen `SCPD_*`-Schalter mit dokumentiertem Default und der gemessenen
Begründung im Kommentar und erscheint in einer `settings:`-Zeile. Und Short-
und Long-Fails sind nicht die Zielgröße.
