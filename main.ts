// A push-optimal Sokoban solver for make.fun's sokoban-solver-01-run-20261121.
//
// Strategy: a level is a 1-wide hallway plus independent rooms (a box pushed
// out of a door into the hallway can never come back, so every box stays in
// its room, and the push-optimal count P* is the sum of the rooms' optima).
// Solve each room alone with a push-based A* (min-cost matching heuristic,
// deadlock pruning, transposition table), then stitch the rooms together
// with hallway walks. All budgets are fixed node counts, never the clock,
// so two runs produce byte-identical output.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Fixed work budgets (node expansions). A whole-level search would meet the
// product of the rooms' positions; a room holds 3-4 boxes, so these caps are
// far above what a room needs, and the global cap keeps the whole 5-instance
// run comfortably inside the 60 s wall.
const ASTAR_BUDGET = 200000;
const WEIGHTED_BUDGET = 200000;
const GREEDY_BUDGET = 300000;
const GLOBAL_BUDGET = 1500000;
let spent = 0;

const DIRS = [
  [0, -1, 'u'],
  [0, 1, 'd'],
  [-1, 0, 'l'],
  [1, 0, 'r'],
];
function dirLetter(delta, w) {
  if (delta === -w) return 'U';
  if (delta === w) return 'D';
  if (delta === -1) return 'L';
  return 'R';
}

// ---------------------------------------------------------------- parsing

function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const wall = new Uint8Array(w * h).fill(1);
  const goal = new Uint8Array(w * h);
  const boxes = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goal[i] = 1;
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goal, boxes, player };
}

// ------------------------------------------------------------- BFS helpers

// Shortest walk from `from` to `to` over open squares (wall[i] false) avoiding
// `blocked` (a Set of box squares). Deterministic: neighbours in u,d,l,r
// order, first visit wins. Returns the move letters, '' when from == to,
// null when unreachable.
function walkPath(w, h, wall, blocked, from, to) {
  if (from === to) return '';
  const prev = new Int32Array(w * h).fill(-2);
  const prevDir = new Array(w * h).fill('');
  const q = [from];
  prev[from] = -1;
  for (let head = 0; head < q.length; head++) {
    const c = q[head];
    const cx = c % w;
    const cy = (c / w) | 0;
    for (const [dx, dy, letter] of DIRS) {
      const nx = cx + dx;
      const ny = cy + dy;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const n = ny * w + nx;
      if (wall[n] || blocked.has(n) || prev[n] !== -2) continue;
      prev[n] = c;
      prevDir[n] = letter;
      if (n === to) {
        let s = '';
        for (let t = n; t !== from; t = prev[t]) s = prevDir[t] + s;
        return s;
      }
      q.push(n);
    }
  }
  return null;
}

// Squares reachable from `from` over board squares (inBoard[i] true) avoiding
// boxes. Returns { seen, rep } where rep is the smallest square index of the
// region (a canonical representative: two regions sharing a square are one).
function regionOf(w, h, inBoard, boxSet, from) {
  const seen = new Uint8Array(w * h);
  const q = [from];
  seen[from] = 1;
  let rep = from;
  for (let head = 0; head < q.length; head++) {
    const c = q[head];
    if (c < rep) rep = c;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (const [dx, dy] of DIRS) {
      const nx = cx + dx;
      const ny = cy + dy;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const n = ny * w + nx;
      if (!inBoard[n] || boxSet.has(n) || seen[n]) continue;
      seen[n] = 1;
      q.push(n);
    }
  }
  return { seen, rep };
}

// BFS distances from `src` over squares where open[i] is true.
function bfsDist(w, h, open, src) {
  const dist = new Int32Array(w * h).fill(-1);
  const q = [src];
  dist[src] = 0;
  for (let head = 0; head < q.length; head++) {
    const c = q[head];
    const cx = c % w;
    const cy = (c / w) | 0;
    for (const [dx, dy] of DIRS) {
      const nx = cx + dx;
      const ny = cy + dy;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const n = ny * w + nx;
      if (!open[n] || dist[n] !== -1) continue;
      dist[n] = dist[c] + 1;
      q.push(n);
    }
  }
  return dist;
}

// ------------------------------------------------------------ decomposition

// The hallway is the player's row. Rooms are the connected floor components
// off that row. Each room opens onto the hallway through doors; entries are
// the hallway squares beside them.
function decompose(level) {
  const { w, h, wall, boxes, goal } = level;
  const hy = Math.floor(level.player / w);
  const seen = new Uint8Array(w * h);
  const rooms = [];
  for (let i = 0; i < w * h; i++) {
    if (wall[i] || Math.floor(i / w) === hy || seen[i]) continue;
    const comp = [];
    const q = [i];
    seen[i] = 1;
    while (q.length > 0) {
      const c = q.pop();
      comp.push(c);
      const cx = c % w;
      const cy = (c / w) | 0;
      for (const [dx, dy] of DIRS) {
        const nx = cx + dx;
        const ny = cy + dy;
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
        const n = ny * w + nx;
        if (wall[n] || Math.floor(n / w) === hy || seen[n]) continue;
        seen[n] = 1;
        q.push(n);
      }
    }
    const inComp = new Set(comp);
    const entries = [];
    let door = -1;
    for (const c of comp) {
      const cx = c % w;
      const cy = (c / w) | 0;
      for (const [dx, dy] of DIRS) {
        const nx = cx + dx;
        const ny = cy + dy;
        if (ny !== hy || nx < 0 || nx >= w) continue;
        const n = ny * w + nx;
        if (wall[n]) continue;
        if (door === -1) door = c;
        if (!entries.includes(n)) entries.push(n);
      }
    }
    const rBoxes = boxes.filter((b) => inComp.has(b));
    const rGoals = comp.filter((c) => goal[c]);
    rooms.push({ squares: comp, inComp, door, entries, boxes: rBoxes, goals: rGoals });
  }
  return rooms;
}

// --------------------------------------------------------------- room solve

// All permutations of [0..n-1] in lexicographic order (n <= 4: at most 24).
function permutations(n) {
  const out = [];
  const a = Array.from({ length: n }, (_, i) => i);
  const gen = (k) => {
    if (k === n) {
      out.push(a.slice());
      return;
    }
    for (let i = k; i < n; i++) {
      [a[k], a[i]] = [a[i], a[k]];
      gen(k + 1);
      [a[k], a[i]] = [a[i], a[k]];
    }
  };
  gen(0);
  return out;
}

// Push-based A* over one room. Returns the list of pushes
// [{from, to}] from the entry, or null when the budget runs out.
// weight 1 is A* (push-optimal); higher weights trade optimality for speed.
function searchRoom(level, room, startEntry, weight, budget) {
  const { w, h, wall, goal } = level;
  const n = room.boxes.length;
  if (n === 0) return [];
  const inBoard = new Uint8Array(w * h);
  for (const s of room.squares) inBoard[s] = 1;
  for (const e of room.entries) inBoard[e] = 1;
  const isRoomSq = new Uint8Array(w * h);
  for (const s of room.squares) isRoomSq[s] = 1;
  const goalSet = new Set(room.goals);

  // BFS distance from each goal over ROOM squares only (a box can never
  // stand on an entry square, so its path to a goal stays inside the room).
  const distToGoal = room.goals.map((g) => bfsDist(w, h, isRoomSq, g));
  const perms = permutations(n);

  // Dead squares: a room square no goal reaches, or a corner that is not a
  // goal (two perpendicular neighbours off the room). A box there is lost.
  const dead = new Uint8Array(w * h);
  const reachAll = new Int32Array(w * h).fill(-1);
  {
    const q = [];
    for (const g of room.goals) {
      reachAll[g] = 0;
      q.push(g);
    }
    for (let head = 0; head < q.length; head++) {
      const c = q[head];
      const cx = c % w;
      const cy = (c / w) | 0;
      for (const [dx, dy] of DIRS) {
        const nx = cx + dx;
        const ny = cy + dy;
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
        const nn = ny * w + nx;
        if (!isRoomSq[nn] || reachAll[nn] !== -1) continue;
        reachAll[nn] = 0;
        q.push(nn);
      }
    }
  }
  for (const s of room.squares) {
    if (goalSet.has(s)) continue;
    if (reachAll[s] === -1) {
      dead[s] = 1;
      continue;
    }
    const x = s % w;
    const y = (s / w) | 0;
    const up = y === 0 || !isRoomSq[s - w];
    const down = y === h - 1 || !isRoomSq[s + w];
    const left = x === 0 || !isRoomSq[s - 1];
    const right = x === w - 1 || !isRoomSq[s + 1];
    if ((up || down) && (left || right)) dead[s] = 1;
  }

  // 2x2 blocks indexed by square: a block fully occupied by walls/boxes with
  // a box off its goal can never be undone. Only blocks holding the box that
  // just moved can newly freeze, so only those are checked.
  const blocksBySquare = new Map();
  for (let y = 0; y < h - 1; y++) {
    for (let x = 0; x < w - 1; x++) {
      const cells = [y * w + x, y * w + x + 1, (y + 1) * w + x, (y + 1) * w + x + 1];
      if (!cells.some((c) => isRoomSq[c])) continue;
      for (const c of cells) {
        if (!blocksBySquare.has(c)) blocksBySquare.set(c, []);
        blocksBySquare.get(c).push(cells);
      }
    }
  }
  const frozenAt = (boxSet, sq) => {
    for (const cells of blocksBySquare.get(sq) ?? []) {
      let offGoalBox = false;
      let full = true;
      for (const c of cells) {
        if (boxSet.has(c)) {
          if (!goalSet.has(c)) offGoalBox = true;
        } else if (isRoomSq[c]) {
          full = false;
          break;
        }
      }
      if (full && offGoalBox) return true;
    }
    return false;
  };

  // Min-cost matching of boxes to goals over BFS distances: each box needs
  // at least its matched distance in pushes, so the sum is admissible.
  const heuristic = (boxes) => {
    let best = Infinity;
    for (const p of perms) {
      let sum = 0;
      let ok = true;
      for (let i = 0; i < n; i++) {
        const d = distToGoal[p[i]][boxes[i]];
        if (d === -1) {
          ok = false;
          break;
        }
        sum += d;
      }
      if (ok && sum < best) best = sum;
    }
    return best;
  };

  // Binary heap on [f, -g, counter, stateId]: lowest f first, deeper first
  // on ties, insertion order last (fully deterministic).
  const heap = [];
  const less = (a, b) =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e) => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      [heap[i], heap[p]] = [heap[p], heap[i]];
      i = p;
    }
  };
  const heapPop = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
        if (!less(heap[c], heap[i])) break;
        [heap[i], heap[c]] = [heap[c], heap[i]];
        i = c;
      }
    }
    return top;
  };

  const startBoxes = room.boxes.slice().sort((a, b) => a - b);
  const startBoxSet = new Set(startBoxes);
  const startRegion = regionOf(w, h, inBoard, startBoxSet, startEntry);
  const sBoxes = [startBoxes];
  const sPlayer = [startEntry];
  const sParent = [-1];
  const sPushFrom = [-1];
  const sPushTo = [-1];
  const sG = [0];
  const sCanExit = [room.entries.some((e) => startRegion.seen[e])];
  const best = new Map();
  const keyOf = (boxes, rep) => boxes.join(',') + '|' + rep;
  best.set(keyOf(startBoxes, startRegion.rep), 0);
  const h0 = heuristic(startBoxes);
  if (h0 === Infinity) return null;
  let counter = 0;
  heapPush([weight * h0, 0, counter++, 0]);

  let expanded = 0;
  while (heap.length > 0 && expanded < budget && spent < GLOBAL_BUDGET) {
    const [, , , s] = heapPop();
    const boxes = sBoxes[s];
    const g = sG[s];
    // Stale entry (a better path to this state was found since)? Skip it.
    const boxSet = new Set(boxes);
    const region = regionOf(w, h, inBoard, boxSet, sPlayer[s]);
    if (best.get(keyOf(boxes, region.rep)) !== g) continue;
    if (boxes.every((b) => goalSet.has(b)) && sCanExit[s]) {
      const pushes = [];
      for (let t = s; t > 0; t = sParent[t]) {
        pushes.push({ from: sPushFrom[t], to: sPushTo[t] });
      }
      pushes.reverse();
      return pushes;
    }
    expanded++;
    spent++;
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi];
      const bx = b % w;
      const by = (b / w) | 0;
      for (const [dx, dy] of DIRS) {
        const tx = bx + dx;
        const ty = by + dy;
        if (tx < 0 || tx >= w || ty < 0 || ty >= h) continue;
        const to = ty * w + tx;
        if (!isRoomSq[to] || boxSet.has(to)) continue; // boxes stay in the room
        const px = bx - dx;
        const py = by - dy;
        if (px < 0 || px >= w || py < 0 || py >= h) continue;
        const pushSq = py * w + px;
        if (!region.seen[pushSq]) continue; // player cannot reach the pushing square
        if (dead[to]) continue;
        const nb = boxes.slice();
        nb[bi] = to;
        nb.sort((a, b) => a - b);
        const nbSet = new Set(nb);
        if (frozenAt(nbSet, to)) continue;
        const nRegion = regionOf(w, h, inBoard, nbSet, b);
        const k = keyOf(nb, nRegion.rep);
        const seenG = best.get(k);
        if (seenG !== undefined && seenG <= g + 1) continue;
        const hh = heuristic(nb);
        if (hh === Infinity) continue;
        const t = sBoxes.length;
        sBoxes.push(nb);
        sPlayer.push(b);
        sParent.push(s);
        sPushFrom.push(b);
        sPushTo.push(to);
        sG.push(g + 1);
        sCanExit.push(room.entries.some((e) => nRegion.seen[e]));
        best.set(k, g + 1);
        heapPush([g + 1 + weight * hh, -(g + 1), counter++, t]);
      }
    }
  }
  return null;
}

// Turn a room's push list into the player's move letters: BFS walks between
// pushes (over the room board, avoiding the current boxes), the push step
// itself, then a walk back out to an entry square.
function roomMoves(level, room, startEntry, pushes) {
  const { w, h } = level;
  const boardWall = new Uint8Array(w * h).fill(1);
  for (const s of room.squares) boardWall[s] = 0;
  for (const e of room.entries) boardWall[e] = 0;
  const boxes = new Set(room.boxes);
  let moves = '';
  let player = startEntry;
  for (const { from, to } of pushes) {
    const d = to - from;
    const pushSq = from - d;
    const walk = walkPath(w, h, boardWall, boxes, player, pushSq);
    if (walk === null) return null;
    moves += walk;
    moves += dirLetter(d, w);
    boxes.delete(from);
    boxes.add(to);
    player = from;
  }
  // Walk back out to an entry square (any entry the player can reach).
  let out = null;
  let endEntry = startEntry;
  for (const e of room.entries) {
    const p = walkPath(w, h, boardWall, boxes, player, e);
    if (p !== null) {
      out = p;
      endEntry = e;
      break;
    }
  }
  if (out === null) return null;
  moves += out;
  return { moves, endEntry };
}

// Solve one room: push-optimal A* first, then weighted A*, then greedy.
// Returns { moves, endEntry, pushes } or null.
function solveRoom(level, room, startEntry) {
  const attempts = [
    [1, ASTAR_BUDGET],
    [2, WEIGHTED_BUDGET],
    [0, GREEDY_BUDGET], // weight 0: f = g, i.e. breadth-first on pushes... use h-only below
  ];
  for (let a = 0; a < attempts.length; a++) {
    const [weight, budget] = attempts[a];
    // Greedy pass: rank by h alone (weight 0 with g dropped is not expressible
    // in f = g + w*h, so emulate it with a very large weight).
    const w = a === 2 ? 1000 : weight;
    const pushes = searchRoom(level, room, startEntry, w, budget);
    if (pushes === null) continue;
    const built = roomMoves(level, room, startEntry, pushes);
    if (built === null) continue;
    return { moves: built.moves, endEntry: built.endEntry, pushes: pushes.length };
  }
  return null;
}

// ------------------------------------------------------------------- driver

function solve(text) {
  const level = parseLevel(text);
  const rooms = decompose(level);
  const boxSet = new Set(level.boxes);
  let moves = '';
  let player = level.player;
  // Rooms with boxes, in door order (deterministic; order does not matter
  // for the score, since pushes add up the same either way).
  const todo = rooms
    .filter((r) => r.boxes.length > 0)
    .sort((a, b) => a.door - b.door);
  for (const room of todo) {
    if (room.boxes.length !== room.goals.length) return '';
    if (room.entries.length === 0) return '';
    const entry = room.entries[0];
    const walk = walkPath(level.w, level.h, level.wall, boxSet, player, entry);
    if (walk === null) return '';
    moves += walk;
    player = entry;
    const sol = solveRoom(level, room, entry);
    if (sol === null) return '';
    moves += sol.moves;
    for (const b of room.boxes) boxSet.delete(b);
    for (const g of room.goals) boxSet.add(g);
    player = sol.endEntry;
  }
  return moves;
}

for (const id of readdirSync(I).sort()) {
  const moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
