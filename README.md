[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/maxwin96/sokoban-solver-01-run-20261121-entry/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/maxwin96/sokoban-solver-01-run-20261121-entry/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

# Push-Optimal Sokoban Solver

A Sokoban solver for make.fun's
[Build the best Sokoban solver, 21 November 2026](https://make.fun/bounty/sokoban-solver-01-run-20261121)
bounty ($100 in USDC). One program, `main.ts` (Node 22, no dependencies,
no network), answers all 5 instances of a set in a single run.

## The idea

Every level is one 1-wide hallway with 1–4 rooms off it, each room holding
3–4 boxes and as many goals — and a box pushed out of a room's door into
the hallway can never come back. So a level's push-optimal count is the
sum of its rooms' optima, and the solver:

1. **Splits the level** into rooms (connected floor components off the
   hallway row, one door each).
2. **Solves each room push-optimally** with A\* over macro-push states
   (sorted box cells + canonical player-reachability region), using a
   minimum-assignment heuristic over shortest box-path distances, and
   deadlock pruning (static dead cells on closed wall runs, corners,
   2×2 blocks).
3. **Walks between doors** with BFS (walking never counts as a push) and
   concatenates everything into one `LURD` move string.

Because every room is solved in its optimal push count, the whole level
is solved in P\* pushes — full points. A whole-level best-first search is
kept as a fallback for levels that do not decompose the expected way.

Determinism is by construction: fixed neighbour order, heap tie-breaks by
insertion sequence, and all budgets are node counts — the program never
reads the wall clock, the environment, or anything outside `/input`.

## Practice score

| instance | pushes | P\* | points |
|----------|--------|-----|--------|
| i001 | 13 | 13 | 1,000 |
| i002 | 16 | 16 | 1,000 |
| i003 | 29 | 29 | 1,000 |
| i004 | 47 | 47 | 1,000 |
| i005 | 60 | 60 | 1,000 |

**Total: 1,000,000 / 1,000,000**, all 5 instances answered, byte-identical
output across two runs, whole set solved in ~0.5 s locally (limit: 60 s).

The same solver also scores 1,000,000 on three further sets drawn from
the kit's own generator with fresh seeds (20/20 instances at P\*).

## Layout

- `main.ts` — the whole solver (entry point: reads `/input/<id>/level.txt`,
  writes `/output/<id>/solution.txt`).
- `LICENSE` — MIT.

## License

MIT — see [LICENSE](LICENSE).
