// Push-optimal Sokoban solver.
//
// Strategy (from the competition rules): every level is one 1-wide hallway
// with 1-4 rooms off it, each room holding 3-4 boxes and as many goals, and
// a box pushed out of a room's door into the hallway can never come back.
// So a level's push-optimal count is the sum of its rooms' optima: solve
// each room independently with a push-optimal A* search (macro pushes,
// player-reachability canonicalisation, deadlock pruning), walk between
// the rooms' doors with BFS, and concatenate everything. Walking never
// counts as a push, so per-room optimal solutions give a level-optimal
// answer. A whole-level best-first search is kept as a fallback for
// levels that do not decompose the expected way.
//
// Determinism: every order is fixed (neighbour order u/d/l/r, heap
// tie-break by insertion sequence), all budgets are node counts, never
// the wall clock. No network, no environment, no files outside
// /input and /output are read.

import { mkdirSync, readdirSync, readFileSync, writeFileSync } from "node:fs";
import { join } from "node:path";

// ---------------------------------------------------------------------------
// Small utilities
// ---------------------------------------------------------------------------

const DIRS = [
  { dx: 0, dy: -1, ch: "u" },
  { dx: 0, dy: 1, ch: "d" },
  { dx: -1, dy: 0, ch: "l" },
  { dx: 1, dy: 0, ch: "r" },
] as const;

function permutations<T>(items: T[]): T[][] {
  if (items.length === 0) return [[]];
  const out: T[][] = [];
  for (let i = 0; i < items.length; i++) {
    const rest = items.slice(0, i).concat(items.slice(i + 1));
    for (const p of permutations(rest)) out.push([items[i], ...p]);
  }
  return out;
}

// ---------------------------------------------------------------------------
// Level parsing (XSB)
// ---------------------------------------------------------------------------

interface Level {
  w: number;
  h: number;
  wall: Uint8Array; // 1 = wall (includes out-of-board padding)
  goal: Uint8Array;
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text
    .split("\n")
    .map((r) => r.replace(/\r$/, ""))
    .filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const size = w * h;
  const wall = new Uint8Array(size).fill(1);
  const goal = new Uint8Array(size);
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === "#") continue;
      wall[i] = 0;
      if (ch === "." || ch === "*" || ch === "+") goal[i] = 1;
      if (ch === "$" || ch === "*") boxes.push(i);
      if (ch === "@" || ch === "+") player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goal, boxes, player };
}

// ---------------------------------------------------------------------------
// Breadth-first walk (boxes are walls). Fixed neighbour order => deterministic.
// Returns the move string from `from` to `to`, or null when unreachable.
// `walkable[i]` must be 1 for every cell the player may stand on.
// ---------------------------------------------------------------------------

function walkPath(
  walkable: Uint8Array,
  boxSet: Uint8Array,
  w: number,
  from: number,
  to: number,
): string | null {
  if (from === to) return "";
  const n = walkable.length;
  const prev = new Int32Array(n).fill(-1);
  const prevCh = new Array<string>(n).fill("");
  const queue: number[] = [from];
  prev[from] = from;
  let head = 0;
  while (head < queue.length) {
    const cur = queue[head++];
    const cx = cur % w;
    const cy = (cur / w) | 0;
    for (const d of DIRS) {
      const nx = cx + d.dx;
      const ny = cy + d.dy;
      if (nx < 0 || ny < 0) continue;
      const ni = ny * w + nx;
      if (ni < 0 || ni >= n) continue;
      if (!walkable[ni] || boxSet[ni] || prev[ni] !== -1) continue;
      prev[ni] = cur;
      prevCh[ni] = d.ch;
      if (ni === to) {
        let out = "";
        let c = ni;
        while (c !== from) {
          out = prevCh[c] + out;
          c = prev[c];
        }
        return out;
      }
      queue.push(ni);
    }
  }
  return null;
}

// ---------------------------------------------------------------------------
// Push-optimal A* over macro-push states.
//
// A state is (sorted box cells, canonical player cell). The canonical
// player cell is the smallest cell of the player's reachable region, so
// all player positions that can walk to each other for free are one state.
// Cost g = pushes so far; heuristic h = cheapest assignment of boxes to
// goals by shortest box-path distance (admissible: one push moves one box
// one step). Deadlock pruning: static dead cells and 2x2 blocks. The search
// is cut off only by a node-count budget, never by time.
// ---------------------------------------------------------------------------

interface PushProblem {
  w: number;
  h: number;
  walkable: Uint8Array; // cells the player may stand on
  boxCells: Uint8Array; // cells a box may occupy
  goals: number[];
  boxes: number[];
  startPlayer: number;
  exitTarget: number; // -1: no exit needed; else goal needs a walk to here
  budget: number; // max expansions
}

interface PushSolution {
  moves: string;
  pushes: number;
  endPlayer: number;
  endBoxes: number[];
}

interface StateRec {
  boxes: number[];
  player: number; // actual player cell
  g: number;
  parent: string | number | null;
  pushBox: number; // box cell before the push that made this state (-1: root)
  pushDir: number; // index into DIRS of that push
}

function solvePushOptimal(p: PushProblem): PushSolution | null {
  const { w, h, walkable, boxCells, goals } = p;
  const n = w * h;
  const goalSet = new Uint8Array(n);
  for (const g of goals) goalSet[g] = 1;

  // A cell the player cannot stand on. (For a room this includes the
  // hallway except the door cell: a box can be pushed from there, so the
  // door cell itself is NOT solid, but a box can never rest on it.)
  const solid = new Uint8Array(n);
  for (let i = 0; i < n; i++) solid[i] = walkable[i] ? 0 : 1;

  // --- static dead cells -------------------------------------------------
  // Corner: solid on two perpendicular sides => the box can never move.
  // Wall run: a box against a wall can only slide along the wall (it can
  // never be pushed away from it: the player cannot stand on the wall
  // side, and towards the wall is blocked). It is confined to the maximal
  // run of wall-adjacent cells ONLY when both ends of the run are closed
  // by cells no box can occupy -- otherwise it slides out of the run.
  // A confined run with no goal can never hold a box that reaches a goal.
  const dead = new Uint8Array(n);
  for (let i = 0; i < n; i++) {
    if (!boxCells[i] || goalSet[i]) continue;
    const x = i % w;
    const y = (i / w) | 0;
    const sN = y > 0 && solid[i - w] === 1;
    const sS = y + 1 < h && solid[i + w] === 1;
    const sW = x > 0 && solid[i - 1] === 1;
    const sE = x + 1 < w && solid[i + 1] === 1;
    if ((sN || sS) && (sW || sE)) {
      dead[i] = 1; // corner
      continue;
    }
    // North wall run (cells below a north wall, sliding horizontally).
    if (sN) {
      let lo = i;
      while (lo - 1 >= 0 && (((lo - 1) / w) | 0) === y && boxCells[lo - 1] && solid[lo - 1 - w]) lo--;
      let hi = i;
      while (hi + 1 < n && (((hi + 1) / w) | 0) === y && boxCells[hi + 1] && solid[hi + 1 - w]) hi++;
      const closedLo = lo - 1 < 0 || (((lo - 1) / w) | 0) !== y || !boxCells[lo - 1];
      const closedHi = hi + 1 >= n || (((hi + 1) / w) | 0) !== y || !boxCells[hi + 1];
      if (closedLo && closedHi) {
        let hasGoal = false;
        for (let j = lo; j <= hi; j++) if (goalSet[j]) hasGoal = true;
        if (!hasGoal) dead[i] = 1;
      }
    }
    // South wall run.
    if (sS) {
      let lo = i;
      while (lo - 1 >= 0 && (((lo - 1) / w) | 0) === y && boxCells[lo - 1] && solid[lo - 1 + w]) lo--;
      let hi = i;
      while (hi + 1 < n && (((hi + 1) / w) | 0) === y && boxCells[hi + 1] && solid[hi + 1 + w]) hi++;
      const closedLo = lo - 1 < 0 || (((lo - 1) / w) | 0) !== y || !boxCells[lo - 1];
      const closedHi = hi + 1 >= n || (((hi + 1) / w) | 0) !== y || !boxCells[hi + 1];
      if (closedLo && closedHi) {
        let hasGoal = false;
        for (let j = lo; j <= hi; j++) if (goalSet[j]) hasGoal = true;
        if (!hasGoal) dead[i] = 1;
      }
    }
    // West wall run (cells right of a west wall, sliding vertically).
    if (sW) {
      let lo = i;
      while (lo - w >= 0 && (lo - w) % w === x && boxCells[lo - w] && solid[lo - w - 1]) lo -= w;
      let hi = i;
      while (hi + w < n && (hi + w) % w === x && boxCells[hi + w] && solid[hi + w - 1]) hi += w;
      const closedLo = lo - w < 0 || (lo - w) % w !== x || !boxCells[lo - w];
      const closedHi = hi + w >= n || (hi + w) % w !== x || !boxCells[hi + w];
      if (closedLo && closedHi) {
        let hasGoal = false;
        for (let j = lo; j <= hi; j += w) if (goalSet[j]) hasGoal = true;
        if (!hasGoal) dead[i] = 1;
      }
    }
    // East wall run.
    if (sE) {
      let lo = i;
      while (lo - w >= 0 && (lo - w) % w === x && boxCells[lo - w] && solid[lo - w + 1]) lo -= w;
      let hi = i;
      while (hi + w < n && (hi + w) % w === x && boxCells[hi + w] && solid[hi + w + 1]) hi += w;
      const closedLo = lo - w < 0 || (lo - w) % w !== x || !boxCells[lo - w];
      const closedHi = hi + w >= n || (hi + w) % w !== x || !boxCells[hi + w];
      if (closedLo && closedHi) {
        let hasGoal = false;
        for (let j = lo; j <= hi; j += w) if (goalSet[j]) hasGoal = true;
        if (!hasGoal) dead[i] = 1;
      }
    }
  }

  // --- shortest box-path distances from each goal, through boxCells only.
  const dist: Int32Array[] = goals.map(() => {
    const d = new Int32Array(n).fill(-1);
    return d;
  });
  for (let gi = 0; gi < goals.length; gi++) {
    const d = dist[gi];
    const q: number[] = [goals[gi]];
    d[goals[gi]] = 0;
    let head = 0;
    while (head < q.length) {
      const cur = q[head++];
      const cx = cur % w;
      const cy = (cur / w) | 0;
      for (const dir of DIRS) {
        const nx = cx + dir.dx;
        const ny = cy + dir.dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const ni = ny * w + nx;
        if (!boxCells[ni] || d[ni] !== -1) continue;
        d[ni] = d[cur] + 1;
        q.push(ni);
      }
    }
  }

  const goalPerms = permutations(goals.map((_, i) => i));

  function heuristic(boxes: number[]): number {
    if (boxes.length === 0) return 0;
    if (boxes.length === goals.length) {
      let best = Infinity;
      for (const perm of goalPerms) {
        let s = 0;
        for (let b = 0; b < boxes.length; b++) {
          const d = dist[perm[b]][boxes[b]];
          if (d < 0) {
            s = Infinity;
            break;
          }
          s += d;
        }
        if (s < best) best = s;
      }
      return best;
    }
    let s = 0;
    for (const b of boxes) {
      let m = Infinity;
      for (let gi = 0; gi < goals.length; gi++) {
        const d = dist[gi][b];
        if (d >= 0 && d < m) m = d;
      }
      if (m === Infinity) return Infinity;
      s += m;
    }
    return s;
  }

  // --- shared scratch: stamp-marked sets, no per-call allocation --------
  // `seenStamp` is scratch for canonical-player BFS; `reachSeen` holds the
  // current expansion's reachable set separately, so a successor's
  // canonical BFS cannot clobber it.
  const seenStamp = new Int32Array(n);
  const reachSeen = new Int32Array(n);
  const boxStamp = new Int32Array(n);
  const bfsQueue = new Int32Array(n);
  let stampCounter = 1;

  // Mark boxes of `boxes` with a fresh stamp; returns the stamp value.
  function markBoxes(boxes: number[]): number {
    const s = stampCounter++;
    for (const b of boxes) boxStamp[b] = s;
    return s;
  }

  // BFS from `from` over walkable cells avoiding cells marked with
  // `boxMark`. Stamps the visited cells into `markArr` with `markVal`
  // and returns the smallest reachable cell (canonical player).
  function bfsCanonical(
    from: number,
    boxMark: number,
    markArr: Int32Array,
    markVal: number,
  ): number {
    let head = 0;
    let tail = 0;
    bfsQueue[tail++] = from;
    markArr[from] = markVal;
    let min = from;
    while (head < tail) {
      const cur = bfsQueue[head++];
      if (cur < min) min = cur;
      const cx = cur % w;
      const cy = (cur / w) | 0;
      for (const d of DIRS) {
        const nx = cx + d.dx;
        const ny = cy + d.dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const ni = ny * w + nx;
        if (!walkable[ni] || boxStamp[ni] === boxMark || markArr[ni] === markVal) continue;
        markArr[ni] = markVal;
        bfsQueue[tail++] = ni;
      }
    }
    return min;
  }

  // 2x2 blocks, precomputed: only blocks whose four cells are all
  // boxCells or solid can ever be all-blocked. Indexed per cell.
  const blocksByCell: number[][] = Array.from({ length: n }, () => []);
  const blocks: number[][] = [];
  for (let y = 0; y + 1 < h; y++) {
    for (let x = 0; x + 1 < w; x++) {
      const a = y * w + x;
      const cells = [a, a + 1, a + w, a + w + 1];
      if (cells.every((c) => boxCells[c] || solid[c])) {
        const bi = blocks.length;
        blocks.push(cells);
        for (const c of cells) blocksByCell[c].push(bi);
      }
    }
  }

  // Deadlock test for one box that just moved to `cell` (or any box of a
  // fresh state when `cell` is -1: then every box is tested).
  // `boxMark` stamps the current box set.
  function boxDead(cell: number, boxMark: number): boolean {
    if (cell >= 0) {
      if (dead[cell]) return true; // (a box on a goal is never dead)
      // NOTE: no single-box "frozen" test here. A box blocked on both
      // sides by other boxes is not permanently stuck -- those boxes can
      // move away -- so treating neighbours as walls over-prunes and
      // loses real solutions. Genuine 2x2 deadlocks are caught below,
      // and wall runs by the static dead cells above.
      // 2x2 blocks containing this box.
      for (const bi of blocksByCell[cell]) {
        const cs = blocks[bi];
        let allBlocked = true;
        let hasOffGoalBox = false;
        for (const c of cs) {
          if (boxStamp[c] === boxMark) {
            if (!goalSet[c]) hasOffGoalBox = true;
          } else if (!solid[c]) {
            allBlocked = false;
            break;
          }
        }
        if (allBlocked && hasOffGoalBox) return true;
      }
      return false;
    }
    return false;
  }

  function stateDead(boxes: number[], boxMark: number): boolean {
    for (const b of boxes) {
      if (boxDead(b, boxMark)) return true;
    }
    return false;
  }

  function isGoal(boxes: number[], player: number, boxMark: number): boolean {
    for (const b of boxes) if (!goalSet[b]) return false;
    if (p.exitTarget < 0) return true;
    const reachMark = stampCounter++;
    bfsCanonical(player, boxMark, seenStamp, reachMark);
    return seenStamp[p.exitTarget] === reachMark;
  }

  // State keys: numeric packing when it fits (rooms: <=4 boxes, <256
  // cells), otherwise a string. Both are deterministic.
  const numericKeys = p.boxes.length <= 4 && n <= 255;
  function stateKey(boxes: number[], canon: number): string | number {
    if (!numericKeys) return boxes.join(",") + "|" + canon;
    let k = canon;
    for (let i = boxes.length - 1; i >= 0; i--) k = k * 256 + boxes[i];
    for (let i = boxes.length; i < 4; i++) k = k * 256 + 255;
    return k;
  }

  // --- binary heap ordered by (f, h, insertion sequence) ----------------
  interface HeapEntry {
    f: number;
    h: number;
    seq: number;
    key: string | number;
    g: number;
  }
  const heap: HeapEntry[] = [];
  let seqCounter = 0;
  function heapLess(a: HeapEntry, b: HeapEntry): boolean {
    if (a.f !== b.f) return a.f < b.f;
    if (a.h !== b.h) return a.h < b.h;
    return a.seq < b.seq;
  }
  function heapPush(e: HeapEntry): void {
    heap.push(e);
    let i = heap.length - 1;
    while (i > 0) {
      const parent = (i - 1) >> 1;
      if (!heapLess(heap[i], heap[parent])) break;
      const t = heap[i];
      heap[i] = heap[parent];
      heap[parent] = t;
      i = parent;
    }
  }
  function heapPop(): HeapEntry | undefined {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length > 0 && last !== undefined) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1;
        const r = l + 1;
        let m = i;
        if (l < heap.length && heapLess(heap[l], heap[m])) m = l;
        if (r < heap.length && heapLess(heap[r], heap[m])) m = r;
        if (m === i) break;
        const t = heap[i];
        heap[i] = heap[m];
        heap[m] = t;
        i = m;
      }
    }
    return top;
  }

  const states = new Map<string | number, StateRec>();
  const startBoxes = [...p.boxes].sort((a, b) => a - b);
  const startH = heuristic(startBoxes);
  const startBoxMark = markBoxes(startBoxes);
  if (startH === Infinity || stateDead(startBoxes, startBoxMark)) return null;
  const startReach = stampCounter++;
  const startCanon = bfsCanonical(p.startPlayer, startBoxMark, seenStamp, startReach);
  const startKey = stateKey(startBoxes, startCanon);
  states.set(startKey, {
    boxes: startBoxes,
    player: p.startPlayer,
    g: 0,
    parent: null,
    pushBox: -1,
    pushDir: -1,
  });
  heapPush({ f: startH, h: startH, seq: seqCounter++, key: startKey, g: 0 });

  let expansions = 0;
  let goalKey: string | number | null = null;

  while (heap.length > 0) {
    const entry = heapPop();
    if (entry === undefined) break;
    const rec = states.get(entry.key);
    if (rec === undefined || rec.g !== entry.g) continue; // stale entry
    const recBoxMark = markBoxes(rec.boxes);
    if (isGoal(rec.boxes, rec.player, recBoxMark)) {
      goalKey = entry.key;
      break;
    }
    if (expansions >= p.budget) break;
    expansions++;

    const reachMark = stampCounter++;
    bfsCanonical(rec.player, recBoxMark, reachSeen, reachMark);

    for (let bi = 0; bi < rec.boxes.length; bi++) {
      const b = rec.boxes[bi];
      const bx = b % w;
      const by = (b / w) | 0;
      for (let di = 0; di < DIRS.length; di++) {
        const d = DIRS[di];
        const nx = bx + d.dx;
        const ny = by + d.dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const dest = ny * w + nx;
        // Occupancy must be tested against this state's own box list:
        // the stamp array is re-stamped for every successor below, so it
        // cannot be trusted for the current state's boxes here.
        if (!boxCells[dest] || rec.boxes.includes(dest)) continue;
        const behindX = bx - d.dx;
        const behindY = by - d.dy;
        if (behindX < 0 || behindY < 0 || behindX >= w || behindY >= h) continue;
        const behind = behindY * w + behindX;
        if (!walkable[behind] || reachSeen[behind] !== reachMark) continue;
        const nextBoxes = [...rec.boxes];
        nextBoxes[bi] = dest;
        nextBoxes.sort((a, b2) => a - b2);
        const nextBoxMark = markBoxes(nextBoxes);
        if (boxDead(dest, nextBoxMark)) continue;
        const hNext = heuristic(nextBoxes);
        if (hNext === Infinity) continue;
        const gNext = rec.g + 1;
        const canonMark = stampCounter++;
        const canon = bfsCanonical(b, nextBoxMark, seenStamp, canonMark);
        const key = stateKey(nextBoxes, canon);
        const old = states.get(key);
        if (old !== undefined && old.g <= gNext) continue;
        states.set(key, {
          boxes: nextBoxes,
          player: b,
          g: gNext,
          parent: entry.key,
          pushBox: b,
          pushDir: di,
        });
        heapPush({ f: gNext + hNext, h: hNext, seq: seqCounter++, key, g: gNext });
      }
    }
  }

  if (goalKey === null) return null;
  const goalRec = states.get(goalKey);
  if (goalRec === undefined) return null;

  // Reconstruct: walk back, recomputing each walk leg with the same
  // deterministic BFS the search relied on.
  const legs: string[] = [];
  let curKey: string | number | null = goalKey;
  while (curKey !== null) {
    const rec = states.get(curKey);
    if (rec === undefined || rec.parent === null) break;
    const parent = states.get(rec.parent);
    if (parent === undefined) break;
    const parentBoxSet = new Uint8Array(n);
    for (const b of parent.boxes) parentBoxSet[b] = 1;
    const d = DIRS[rec.pushDir];
    const behind = rec.pushBox - d.dy * w - d.dx;
    const walk = walkPath(walkable, parentBoxSet, w, parent.player, behind);
    if (walk === null) return null; // cannot happen: search checked reach
    legs.push(walk + d.ch);
    curKey = rec.parent;
  }
  legs.reverse();
  return {
    moves: legs.join(""),
    pushes: goalRec.g,
    endPlayer: goalRec.player,
    endBoxes: goalRec.boxes,
  };
}

// ---------------------------------------------------------------------------
// Room decomposition
// ---------------------------------------------------------------------------

interface Room {
  cells: number[];
  cellSet: Uint8Array;
  boxes: number[];
  goals: number[];
  door: number; // room cell next to the hallway
  doorHall: number; // hallway cell next to the door
}

function findRooms(level: Level): Room[] | null {
  const { w, h, wall, goal, boxes, player } = level;
  if (player < 0) return null;
  const hallwayRow = (player / w) | 0;
  const n = w * h;
  const seen = new Uint8Array(n);
  const rooms: Room[] = [];
  let boxesSeen = 0;
  for (let i = 0; i < n; i++) {
    const y = (i / w) | 0;
    if (y === hallwayRow || wall[i] || seen[i]) continue;
    // Flood fill one component (all rows except the hallway row).
    const cells: number[] = [];
    const q: number[] = [i];
    seen[i] = 1;
    let head = 0;
    while (head < q.length) {
      const cur = q[head++];
      cells.push(cur);
      const cx = cur % w;
      const cy = (cur / w) | 0;
      for (const d of DIRS) {
        const nx = cx + d.dx;
        const ny = cy + d.dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        if (ny === hallwayRow) continue;
        const ni = ny * w + nx;
        if (wall[ni] || seen[ni]) continue;
        seen[ni] = 1;
        q.push(ni);
      }
    }
    const cellSet = new Uint8Array(n);
    for (const c of cells) cellSet[c] = 1;
    const roomBoxes = boxes.filter((b) => cellSet[b]);
    const roomGoals = cells.filter((c) => goal[c]);
    boxesSeen += roomBoxes.length;
    // The door: the room cell next to the hallway row.
    const doors = cells.filter((c) => {
      const cy = (c / w) | 0;
      if (Math.abs(cy - hallwayRow) !== 1) return false;
      const hall = hallwayRow * w + (c % w);
      return !wall[hall];
    });
    if (doors.length !== 1) return null;
    const door = doors[0];
    const doorHall = hallwayRow * w + (door % w);
    rooms.push({ cells, cellSet, boxes: roomBoxes, goals: roomGoals, door, doorHall });
  }
  if (boxesSeen !== boxes.length) return null; // a box sits in the hallway
  // Deterministic order: by door column, then by side (above first).
  rooms.sort((a, b) => a.doorHall - b.doorHall);
  return rooms;
}

// ---------------------------------------------------------------------------
// Whole-level solver
// ---------------------------------------------------------------------------

const ROOM_BUDGET = 800000;
const GLOBAL_BUDGET = 1500000;

function solveLevel(level: Level): string {
  const { w, h, wall, goal } = level;
  const n = w * h;
  const floor = new Uint8Array(n);
  for (let i = 0; i < n; i++) floor[i] = wall[i] ? 0 : 1;

  const rooms = findRooms(level);
  if (rooms !== null && rooms.every((r) => r.boxes.length === r.goals.length)) {
    // Solve each room push-optimally; rooms that need their final player
    // position trapped inside (no exit) go last.
    const solved = new Map<Room, PushSolution>();
    const noExit: Room[] = [];
    let ok = true;
    for (const room of rooms) {
      const walkable = new Uint8Array(n);
      for (const c of room.cells) walkable[c] = 1;
      walkable[room.doorHall] = 1;
      let sol = solvePushOptimal({
        w,
        h,
        walkable,
        boxCells: room.cellSet,
        goals: room.goals,
        boxes: room.boxes,
        startPlayer: room.doorHall,
        exitTarget: room.doorHall,
        budget: ROOM_BUDGET,
      });
      if (sol === null) {
        sol = solvePushOptimal({
          w,
          h,
          walkable,
          boxCells: room.cellSet,
          goals: room.goals,
          boxes: room.boxes,
          startPlayer: room.doorHall,
          exitTarget: -1,
          budget: ROOM_BUDGET,
        });
        if (sol === null) {
          ok = false;
          break;
        }
        noExit.push(room);
      }
      solved.set(room, sol);
    }
    if (ok) {
      const order = rooms.filter((r) => !noExit.includes(r)).concat(noExit);
      const globalBoxes = new Uint8Array(n);
      for (const b of level.boxes) globalBoxes[b] = 1;
      let moves = "";
      let player = level.player;
      for (const room of order) {
        const sol = solved.get(room);
        if (sol === undefined) {
          ok = false;
          break;
        }
        const walkIn = walkPath(floor, globalBoxes, w, player, room.doorHall);
        if (walkIn === null) {
          ok = false;
          break;
        }
        moves += walkIn + sol.moves;
        player = sol.endPlayer;
        for (const b of room.boxes) globalBoxes[b] = 0;
        for (const b of sol.endBoxes) globalBoxes[b] = 1;
      }
      if (ok) return moves;
    }
  }

  // Fallback: whole-level best-first push search (not necessarily optimal,
  // but it can still solve a level the room split could not handle).
  const allGoals: number[] = [];
  for (let i = 0; i < n; i++) if (goal[i]) allGoals.push(i);
  const sol = solvePushOptimal({
    w,
    h,
    walkable: floor,
    boxCells: floor,
    goals: allGoals,
    boxes: level.boxes,
    startPlayer: level.player,
    exitTarget: -1,
    budget: GLOBAL_BUDGET,
  });
  return sol === null ? "" : sol.moves;
}

// ---------------------------------------------------------------------------
// Entry point: answer every instance in /input, write /output/<id>/solution.txt
// ---------------------------------------------------------------------------

function main(): void {
  const inputDir = "/input";
  const outputDir = "/output";
  const entries = readdirSync(inputDir, { withFileTypes: true });
  const ids = entries
    .filter((e) => e.isDirectory())
    .map((e) => e.name)
    .sort();
  for (const id of ids) {
    let moves = "";
    try {
      const text = readFileSync(join(inputDir, id, "level.txt"), "utf8");
      moves = solveLevel(parseLevel(text));
    } catch {
      moves = "";
    }
    mkdirSync(join(outputDir, id), { recursive: true });
    writeFileSync(join(outputDir, id, "solution.txt"), moves + "\n", "utf8");
  }
}

main();
