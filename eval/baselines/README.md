# Eval baselines

`*.graph.json` are the AssemblyGraphs scored by `scripts/eval_generation.py`;
`scores.json` + `report.md` are their frozen pre-refactor scorecards. The
per-workstream regression gate is:

```
python scripts/eval_generation.py --replay eval/baselines --out eval/runs/<name> \
    --compare eval/baselines/scores.json
```

**Provenance: SYNTHETIC.** These graphs were hand-authored by
`eval/make_synthetic_baselines.py` (deterministic, re-runnable) because the
`ANTHROPIC_API_KEY` in `.env` was rejected (401) at capture time. They are
shaped like real `design_robot` output — same component ids, attachment
idioms, attach_rpy crouch pairs, link_geometry shells — and intentionally
include known-bad baseline behaviors the refactor targets:

- `dog`: crouch leaves shin tips below the foot pads (ground metric 0).
- `hexapod`/`sculpture`: standard-mode multi-child distribution piles legs at
  face-center slots → interpenetration metric 0.
- `sculpture`: tail servo placed via the baseplate's 5mm bbox is swallowed by
  the link_geometry carapace shell (buried).
- `humanoid`: arms clip the pelvis; link_geometry torso bounds ignored.

Replace with real captures once a valid API key is set:

```
python scripts/eval_generation.py --live --out eval/baselines
```
