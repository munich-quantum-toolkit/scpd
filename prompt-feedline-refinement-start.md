# Einstiegs-Prompt: das Feedline-Refinement

Zum Kopieren in eine frische Sitzung. Er macht nichts als hinzeigen — alles
Weitere steht in den Dokumenten.

---

Du übernimmst die fünfte Phase der Final-Stufe, das **Feedline-Refinement
mit der abschließenden Meander-Einfügung**, in
`/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`.

Lies dazu **in dieser Reihenfolge**:

1. `handover-feedline-routing.md` — wo die Stufe steht (bad 8 über die acht
   Chips, kein Wire unrouted oder offen, alle 305 Feedline-Kanten gezeichnet),
   die Hindernisse, alle `SCPD_*`-Schalter mit Defaults, die Negativergebnisse
   und der Abschnitt *The squeeze report*. Die Negativergebnisse sind kein
   Beiwerk: sie sagen dir, wofür du nicht ein zweites Mal bezahlst.
2. `handover-cpw-coupler-insertion.md` und `handover-chain-astar.md` — der
   Boden, auf dem die Chains stehen, und warum vier Chains auf 69q in die Uhr
   laufen.
3. `summary-final-routing.md`, Abschnitt *The meander* — wie die
   Meander-Einfügung gebaut ist und was der Nutzer am 2026-09-15 daran
   korrigiert hat.
4. `prompt-feedline-refinement.md` — dein Auftrag: Ziel, was es schon gibt,
   was der Prototyp tut, die Befehle und die Zahlen, an denen du dich misst.

Dann fang an und melde mir, was du vorfindest, bevor du etwas änderst. Die
Richtung entscheide ich.

Drei Dinge, die immer gelten: Alles liegt **unkommittiert** im Arbeitsbaum und
bleibt dort — ich committe selbst, pro Phase. Jede neue Regel kommt hinter
einen `SCPD_*`-Schalter mit dokumentiertem Default und der gemessenen
Begründung im Kommentar, auch wenn die Messung negativ ausfällt. Und berichtet
wird `bad = unrouted + open + crossing` **plus**, neu in dieser Phase, die
Längenverdikte short und long — nie die `Fails:`-Summe der Logzeile.
