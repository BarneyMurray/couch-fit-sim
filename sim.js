// Couch clearance simulator core: scene geometry, collision, and path planning.
// Units are centimetres. World axes: x to the right, y away from you, z up.
// Every obstacle is a vertical prism (convex floor-plan polygon extruded between z0 and z1);
// the object being moved is a union of boxes in its own frame.
(function (global) {
  'use strict';

  // ---------- random ----------
  function mulberry32(seed) {
    return function () {
      seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
      let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  // ---------- quaternions [w, x, y, z] ----------
  const Q = {
    identity: () => [1, 0, 0, 0],
    mul(a, b) {
      return [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
      ];
    },
    axisAngle(ax, ang) {
      const s = Math.sin(ang / 2);
      return [Math.cos(ang / 2), ax[0] * s, ax[1] * s, ax[2] * s];
    },
    // Yaw about z, then pitch about the new y, then roll about the new x.
    euler(yaw, pitch, roll) {
      return Q.mul(Q.mul(Q.axisAngle([0, 0, 1], yaw), Q.axisAngle([0, 1, 0], pitch)), Q.axisAngle([1, 0, 0], roll));
    },
    toEuler(q) {
      const [w, x, y, z] = q;
      const s = Math.max(-1, Math.min(1, 2 * (w * y - z * x)));
      return {
        yaw: Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
        pitch: Math.asin(s),
        roll: Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
      };
    },
    // Row-major 3x3; column k is the object's local axis k in world space.
    toMat(q) {
      const [w, x, y, z] = q;
      return [
        1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
        2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
        2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
      ];
    },
    random(rng) {
      const u1 = rng(), u2 = rng(), u3 = rng();
      const a = Math.sqrt(1 - u1), b = Math.sqrt(u1);
      return [b * Math.cos(2 * Math.PI * u3), a * Math.sin(2 * Math.PI * u2), a * Math.cos(2 * Math.PI * u2), b * Math.sin(2 * Math.PI * u3)];
    },
    dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3],
    angle(a, b) { return 2 * Math.acos(Math.min(1, Math.abs(Q.dot(a, b)))); },
    slerp(a, b, t) {
      let d = Q.dot(a, b);
      let bb = b;
      if (d < 0) { d = -d; bb = [-b[0], -b[1], -b[2], -b[3]]; }
      let s0, s1;
      if (d > 0.9995) { s0 = 1 - t; s1 = t; } else {
        const th = Math.acos(d), sn = Math.sin(th);
        s0 = Math.sin((1 - t) * th) / sn; s1 = Math.sin(t * th) / sn;
      }
      const r = [s0 * a[0] + s1 * bb[0], s0 * a[1] + s1 * bb[1], s0 * a[2] + s1 * bb[2], s0 * a[3] + s1 * bb[3]];
      const n = Math.hypot(r[0], r[1], r[2], r[3]);
      return [r[0] / n, r[1] / n, r[2] / n, r[3] / n];
    },
  };

  // The 24 rotations that map the object's axes onto the world axes.
  const AXIS_ROTATIONS = (function () {
    const out = [], seen = new Set();
    for (let a = 0; a < 4; a++) for (let b = 0; b < 4; b++) for (let c = 0; c < 4; c++) {
      const q = Q.euler(a * Math.PI / 2, b * Math.PI / 2, c * Math.PI / 2);
      const key = Q.toMat(q).map((v) => Math.round(v)).join(',');
      if (!seen.has(key)) { seen.add(key); out.push(q); }
    }
    return out;
  })();

  // ---------- obstacles ----------
  function prism(poly, z0, z1, kind, label) {
    let area = 0;
    for (let i = 0; i < poly.length; i++) {
      const a = poly[i], b = poly[(i + 1) % poly.length];
      area += a[0] * b[1] - b[0] * a[1];
    }
    if (area < 0) poly = poly.slice().reverse();
    const verts = [], edges = [], normals = [];
    const min = [Infinity, Infinity, z0], max = [-Infinity, -Infinity, z1];
    for (const [x, y] of poly) {
      verts.push([x, y, z0], [x, y, z1]);
      min[0] = Math.min(min[0], x); min[1] = Math.min(min[1], y);
      max[0] = Math.max(max[0], x); max[1] = Math.max(max[1], y);
    }
    for (let i = 0; i < poly.length; i++) {
      const a = poly[i], b = poly[(i + 1) % poly.length];
      const ex = b[0] - a[0], ey = b[1] - a[1], len = Math.hypot(ex, ey);
      edges.push([ex / len, ey / len]);
      normals.push([ey / len, -ex / len]);
    }
    return { poly, z0, z1, kind, label, verts, edges, normals, min, max };
  }

  function box(x0, x1, y0, y1, z0, z1, kind, label) {
    return prism([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], z0, z1, kind, label);
  }

  // Polygon of a wedge from the origin between angles a0..a1 (<= 90deg), clipped to [0,w]x[0,d].
  function wedgeInRect(a0, a1, w, d) {
    const hit = (a) => {
      const c = Math.cos(a), s = Math.sin(a);
      const tx = c > 1e-9 ? w / c : Infinity, ty = s > 1e-9 ? d / s : Infinity;
      const t = Math.min(tx, ty);
      return [c * t, s * t];
    };
    const pts = [[0, 0], hit(a0)];
    const corner = Math.atan2(d, w);
    if (corner > a0 + 1e-9 && corner < a1 - 1e-9) pts.push([w, d]);
    pts.push(hit(a1));
    return pts;
  }

  // ---------- object ----------
  function makeObject(spec) {
    const m = Math.max(0, +spec.margin || 0);
    const L = +spec.length, D = +spec.depth, H = +spec.height;
    let boxes;
    if (spec.shape === 'sofa') {
      const armH = Math.min(Math.max(+spec.armHeight || H, 1), H);
      const backD = Math.min(Math.max(+spec.backDepth || D, 1), D);
      boxes = [
        { c: [0, 0, -H / 2 + armH / 2], h: [L / 2, D / 2, armH / 2] },
        { c: [0, D / 2 - backD / 2, 0], h: [L / 2, backD / 2, H / 2] },
      ];
    } else {
      boxes = [{ c: [0, 0, 0], h: [L / 2, D / 2, H / 2] }];
    }
    boxes = boxes.map((b) => ({ c: b.c, h: b.h.map((v) => v + m) }));
    const half = [L / 2 + m, D / 2 + m, H / 2 + m];
    const corners = [];
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) corners.push([sx * half[0], sy * half[1], sz * half[2]]);
    return { spec, boxes, half, corners, radius: Math.hypot(half[0], half[1], half[2]) };
  }

  function worldCorners(obj, p, q) {
    const m = Q.toMat(q);
    return obj.corners.map((c) => [
      p[0] + m[0] * c[0] + m[1] * c[1] + m[2] * c[2],
      p[1] + m[3] * c[0] + m[4] * c[1] + m[5] * c[2],
      p[2] + m[6] * c[0] + m[7] * c[1] + m[8] * c[2],
    ]);
  }

  // ---------- collision (separating axis test, box vs vertical prism) ----------
  function separated(lx, ly, lz, C, A, h, P) {
    const l2 = lx * lx + ly * ly + lz * lz;
    if (l2 < 1e-12) return false;
    const c = C[0] * lx + C[1] * ly + C[2] * lz;
    const r = h[0] * Math.abs(A[0][0] * lx + A[0][1] * ly + A[0][2] * lz)
      + h[1] * Math.abs(A[1][0] * lx + A[1][1] * ly + A[1][2] * lz)
      + h[2] * Math.abs(A[2][0] * lx + A[2][1] * ly + A[2][2] * lz);
    let mn = Infinity, mx = -Infinity;
    const vs = P.verts;
    for (let i = 0; i < vs.length; i++) {
      const v = vs[i], d = v[0] * lx + v[1] * ly + v[2] * lz;
      if (d < mn) mn = d;
      if (d > mx) mx = d;
    }
    const tol = 1e-4 * Math.sqrt(l2);
    return c + r <= mn + tol || c - r >= mx - tol;
  }

  function boxPrismOverlap(C, A, h, P) {
    for (let k = 0; k < 3; k++) if (separated(A[k][0], A[k][1], A[k][2], C, A, h, P)) return false;
    if (separated(0, 0, 1, C, A, h, P)) return false;
    for (const n of P.normals) if (separated(n[0], n[1], 0, C, A, h, P)) return false;
    for (let k = 0; k < 3; k++) {
      const a = A[k];
      if (separated(a[1], -a[0], 0, C, A, h, P)) return false;
      for (const e of P.edges) {
        if (separated(-a[2] * e[1], a[2] * e[0], a[0] * e[1] - a[1] * e[0], C, A, h, P)) return false;
      }
    }
    return true;
  }

  // Returns true/false, or with `all` the list of prism indices touched.
  function collide(scene, obj, p, q, all) {
    const m = Q.toMat(q);
    const A = [[m[0], m[3], m[6]], [m[1], m[4], m[7]], [m[2], m[5], m[8]]];
    const hits = all ? new Set() : null;
    const prisms = scene.prisms;
    for (const b of obj.boxes) {
      const c = b.c, h = b.h;
      const C = [
        p[0] + m[0] * c[0] + m[1] * c[1] + m[2] * c[2],
        p[1] + m[3] * c[0] + m[4] * c[1] + m[5] * c[2],
        p[2] + m[6] * c[0] + m[7] * c[1] + m[8] * c[2],
      ];
      const e = [
        Math.abs(m[0]) * h[0] + Math.abs(m[1]) * h[1] + Math.abs(m[2]) * h[2],
        Math.abs(m[3]) * h[0] + Math.abs(m[4]) * h[1] + Math.abs(m[5]) * h[2],
        Math.abs(m[6]) * h[0] + Math.abs(m[7]) * h[1] + Math.abs(m[8]) * h[2],
      ];
      for (let i = 0; i < prisms.length; i++) {
        const P = prisms[i];
        if (C[0] + e[0] <= P.min[0] || C[0] - e[0] >= P.max[0] ||
            C[1] + e[1] <= P.min[1] || C[1] - e[1] >= P.max[1] ||
            C[2] + e[2] <= P.min[2] || C[2] - e[2] >= P.max[2]) continue;
        if (boxPrismOverlap(C, A, h, P)) {
          if (!all) return true;
          hits.add(i);
        }
      }
    }
    return all ? [...hits] : false;
  }

  // ---------- space templates ----------
  const T = 20; // thickness used for walls, floors and ceilings

  function region(x0, x1, y0, y1, z0, z1, weight) {
    return { min: [x0, y0, z0], max: [x1, y1, z1], weight: (weight || 1) * (x1 - x0) * (y1 - y0) * (z1 - z0) };
  }

  const TEMPLATES = {
    stairTurn: {
      name: 'Stairs that turn at the bottom',
      blurb: 'You enter from a hall, go up a step or two toward the back wall, the winders turn you left, and the flight carries on up behind the banister.',
      params: [
        { key: 'upperWidth', label: 'Back wall to banister', def: 85, group: 'Your measurements' },
        { key: 'lowerWidth', label: 'Banister to right wall', def: 82, group: 'Your measurements' },
        { key: 'hallDepth', label: 'Banister to wall behind you', def: 114, group: 'Your measurements' },
        { key: 'headroom', label: 'Step to top of opening', def: 190, group: 'Your measurements' },
        { key: 'rise', label: 'Step height', def: 19, group: 'Your measurements' },
        { key: 'headroomStep', label: 'Headroom measured from step no.', def: 2, step: 1, min: 0, group: 'Stairs' },
        { key: 'going', label: 'Step depth', def: 25, group: 'Stairs' },
        { key: 'nBottom', label: 'Straight steps before the turn', def: 1, step: 1, min: 0, group: 'Stairs' },
        { key: 'nWinders', label: 'Turning steps', def: 3, step: 1, min: 1, group: 'Stairs' },
        { key: 'nUpper', label: 'Steps after the turn', def: 8, step: 1, min: 1, group: 'Stairs' },
        { key: 'openingOffset', label: 'Opening starts past banister by', def: 0, min: -100, group: 'Stairs' },
        { key: 'railHeight', label: 'Handrail height above treads', def: 90, group: 'Stairs' },
        { key: 'newelHeight', label: 'Newel post height', def: 140, group: 'Stairs' },
        { key: 'hallLength', label: 'Hall length to the left', def: 300, group: 'Stairs' },
        { key: 'upCeiling', label: 'Upstairs ceiling height', def: 240, group: 'Stairs' },
      ],
      build(p) {
        const W1 = p.lowerWidth, W2 = p.upperWidth, Hd = p.hallDepth, r = p.rise, g = p.going;
        const nB = Math.round(p.nBottom), nW = Math.max(1, Math.round(p.nWinders)), nU = Math.max(1, Math.round(p.nUpper));
        const hallCeil = Math.round(p.headroomStep) * r + p.headroom;
        const nTurn = nB + nW;
        const floorUp = (nTurn + nU + 1) * r;
        const topCeil = Math.max(floorUp + p.upCeiling, hallCeil + 40);
        const xEnd = -p.hallLength;
        const xTop = -nU * g;
        const xFar = Math.min(xEnd, xTop - 200);
        const off = Math.min(Math.max(p.openingOffset, -Hd + 1), W2 - 1);
        const prisms = [];
        prisms.push(box(xFar - T, W1 + T, -Hd - T, W2 + T, -T, 0, 'floor', 'floor'));
        prisms.push(box(W1, W1 + T, -Hd - T, W2 + T, 0, topCeil, 'wall', 'right wall'));
        prisms.push(box(xFar - T, W1 + T, W2, W2 + T, 0, topCeil, 'wall', 'back wall'));
        prisms.push(box(xFar - T, W1, -Hd - T, -Hd, 0, hallCeil, 'wall', 'wall behind you'));
        prisms.push(box(xEnd - T, xEnd, -Hd, 0, 0, hallCeil, 'wall', 'end of hall'));
        prisms.push(box(xFar - T, xFar, 0, W2, 0, topCeil, 'wall', 'upstairs wall'));
        // Hall ceiling: solid above the hall; over the turn it may reach past the banister line.
        prisms.push(box(xFar - T, 0, -Hd - T, 0, hallCeil, topCeil, 'ceiling', 'hall ceiling'));
        prisms.push(box(0, W1, -Hd - T, off, hallCeil, topCeil, 'ceiling', 'edge of stair opening'));
        prisms.push(box(xFar - T, W1 + T, -Hd - T, W2 + T, topCeil, topCeil + T, 'ceiling', 'upstairs ceiling'));
        for (let i = 1; i <= nB; i++) {
          prisms.push(box(0, W1, -(nB - i + 1) * g, -(nB - i) * g, 0, i * r, 'step', 'step ' + i));
        }
        for (let k = 1; k <= nW; k++) {
          const a0 = (k - 1) * Math.PI / 2 / nW, a1 = k * Math.PI / 2 / nW;
          prisms.push(prism(wedgeInRect(a0, a1, W1, W2), 0, (nB + k) * r, 'step', 'step ' + (nB + k)));
        }
        for (let j = 1; j <= nU; j++) {
          prisms.push(box(-j * g, -(j - 1) * g, 0, W2, 0, (nTurn + j) * r, 'step', 'step ' + (nTurn + j)));
          prisms.push(box(-j * g, -(j - 1) * g, -5, 0, 0, Math.min((nTurn + j) * r + p.railHeight, hallCeil), 'rail', 'banister'));
        }
        prisms.push(box(xFar, xTop, 0, W2, 0, floorUp, 'step', 'upstairs floor'));
        prisms.push(box(-9, 0, -9, 0, 0, Math.min(p.newelHeight, hallCeil), 'rail', 'newel post'));

        const pitch = Math.atan2(r, g);
        return {
          prisms,
          regions: [
            region(xEnd, W1, -Hd, 0, 0, hallCeil, 1),
            region(0, W1, 0, W2, 0, topCeil, 4),
            region(xTop - 60, 0, 0, W2, 0, topCeil, 1.5),
          ],
          goalRegions: [region(xTop, 0, 0, W2, (nTurn + 1) * r, topCeil)],
          goalTilts: [0, pitch / 2, pitch, -pitch / 2, -pitch, Math.PI / 2 - pitch, -(Math.PI / 2 - pitch)].map((a) => Q.axisAngle([0, 1, 0], a)),
          inGoal: (cs) => cs.every((c) => c[0] < 0 && c[1] > -5),
          goalText: 'fully past the newel post and on the flight going up',
          startPose: (obj) => ({ p: [xEnd + obj.half[0] + 5, -Hd / 2, obj.half[2] + 0.5], q: Q.identity() }),
          startRegions: [region(xEnd, 0, -Hd, 0, 0, hallCeil)],
          inStart: (cs) => cs.every((c) => c[1] < 0 && c[0] < W1),
          bounds: { min: [xEnd, -Hd, 0], max: [W1, W2, topCeil] },
          views: {
            orbit: { pos: [260, -420, 420], target: [-20, 0, 90] },
            photo: { pos: [45, -Hd + 15, 168], target: [25, 60, 40] },
            top: { pos: [-40, -10, 900], target: [-40, -10, 0] },
          },
          notes: [
            'Hall ceiling height = step ' + Math.round(p.headroomStep) + ' (' + Math.round(p.headroomStep) * r + ' cm) + ' + p.headroom + ' cm = ' + Math.round(hallCeil) + ' cm.',
            'Upstairs floor at ' + Math.round(floorUp) + ' cm (' + (nTurn + nU + 1) + ' risers).',
          ],
        };
      },
    },

    corner: {
      name: 'Corridor corner',
      blurb: 'Two corridors meeting at a right angle, flat floor, one ceiling height.',
      params: [
        { key: 'widthA', label: 'First corridor width', def: 90 },
        { key: 'widthB', label: 'Second corridor width', def: 80 },
        { key: 'height', label: 'Ceiling height', def: 240 },
        { key: 'lenA', label: 'First corridor length', def: 350 },
        { key: 'lenB', label: 'Second corridor length', def: 350 },
      ],
      build(p) {
        const wA = p.widthA, wB = p.widthB, h = p.height, yEnd = wA + p.lenB;
        const prisms = [
          box(-p.lenA - T, wB + T, -T, yEnd + T, -T, 0, 'floor', 'floor'),
          box(-p.lenA - T, wB + T, -T, yEnd + T, h, h + T, 'ceiling', 'ceiling'),
          box(-p.lenA - T, wB + T, -T, 0, 0, h, 'wall', 'outer wall'),
          box(wB, wB + T, 0, yEnd + T, 0, h, 'wall', 'outer wall'),
          box(-p.lenA - T, 0, wA, yEnd + T, 0, h, 'wall', 'inside corner'),
          box(-p.lenA - T, -p.lenA, 0, wA, 0, h, 'wall', 'end wall'),
          box(0, wB, yEnd, yEnd + T, 0, h, 'wall', 'end wall'),
        ];
        return {
          prisms,
          regions: [region(-p.lenA, wB, 0, wA, 0, h, 1), region(0, wB, 0, yEnd, 0, h, 1), region(0, wB, 0, wA, 0, h, 3)],
          goalRegions: [region(0, wB, wA, yEnd, 0, h)],
          goalTilts: [Q.identity()],
          inGoal: (cs) => cs.every((c) => c[1] > wA),
          goalText: 'fully round the corner',
          startPose: (obj) => ({ p: [-p.lenA + obj.half[0] + 5, wA / 2, obj.half[2] + 0.5], q: Q.identity() }),
          startRegions: [region(-p.lenA, 0, 0, wA, 0, h)],
          inStart: (cs) => cs.every((c) => c[0] < 0),
          bounds: { min: [-p.lenA, 0, 0], max: [wB, yEnd, h] },
          views: {
            orbit: { pos: [300, -350, 450], target: [-40, wA, 60] },
            photo: { pos: [-p.lenA + 30, wA / 2, 165], target: [wB / 2, wA / 2, 80] },
            top: { pos: [-40, wA + 40, 900], target: [-40, wA + 40, 0] },
          },
          notes: [],
        };
      },
    },

    doorway: {
      name: 'Doorway off a corridor',
      blurb: 'A corridor with a door in its side wall leading into a room.',
      params: [
        { key: 'corridorWidth', label: 'Corridor width', def: 100 },
        { key: 'doorWidth', label: 'Door opening width', def: 76 },
        { key: 'doorHeight', label: 'Door opening height', def: 198 },
        { key: 'wallThickness', label: 'Wall thickness', def: 12 },
        { key: 'height', label: 'Ceiling height', def: 240 },
        { key: 'corridorLength', label: 'Corridor length', def: 500 },
        { key: 'roomDepth', label: 'Room depth', def: 350 },
      ],
      build(p) {
        const c = p.corridorWidth, dw = p.doorWidth, t = p.wallThickness, h = p.height;
        const x0 = -p.corridorLength / 2, x1 = p.corridorLength / 2, yRoom = c + t, yEnd = yRoom + p.roomDepth;
        const dh = Math.min(p.doorHeight, h);
        const prisms = [
          box(x0 - T, x1 + T, -T, yEnd + T, -T, 0, 'floor', 'floor'),
          box(x0 - T, x1 + T, -T, yEnd + T, h, h + T, 'ceiling', 'ceiling'),
          box(x0 - T, x1 + T, -T, 0, 0, h, 'wall', 'corridor wall'),
          box(x0 - T, x0, 0, yEnd, 0, h, 'wall', 'end wall'),
          box(x1, x1 + T, 0, yEnd, 0, h, 'wall', 'end wall'),
          box(x0, 0, c, yRoom, 0, h, 'wall', 'door frame'),
          box(dw, x1, c, yRoom, 0, h, 'wall', 'door frame'),
          box(0, dw, yEnd, yEnd + T, 0, h, 'wall', 'room wall'),
          box(x0, x1, yEnd, yEnd + T, 0, h, 'wall', 'room wall'),
        ];
        if (dh < h) prisms.push(box(0, dw, c, yRoom, dh, h, 'wall', 'door head'));
        return {
          prisms,
          regions: [region(x0, x1, 0, c, 0, h, 1), region(-60, dw + 60, 0, yRoom + 120, 0, h, 3), region(x0, x1, yRoom, yEnd, 0, h, 0.6)],
          goalRegions: [region(x0, x1, yRoom, yEnd, 0, h)],
          goalTilts: [Q.identity()],
          inGoal: (cs) => cs.every((q) => q[1] > yRoom),
          goalText: 'fully through the door',
          startPose: (obj) => ({ p: [x0 + obj.half[0] + 5, c / 2, obj.half[2] + 0.5], q: Q.identity() }),
          startRegions: [region(x0, x1, 0, c, 0, h)],
          inStart: (cs) => cs.every((q) => q[1] < c),
          bounds: { min: [x0, 0, 0], max: [x1, yEnd, h] },
          views: {
            orbit: { pos: [250, -380, 420], target: [dw / 2, c, 80] },
            photo: { pos: [x0 + 30, c / 2, 165], target: [dw / 2, c + 50, 90] },
            top: { pos: [0, yRoom, 900], target: [0, yRoom, 0] },
          },
          notes: [],
        };
      },
    },
  };

  function buildScene(templateKey, values) {
    const tpl = TEMPLATES[templateKey];
    const p = {};
    for (const def of tpl.params) p[def.key] = values && values[def.key] != null && isFinite(values[def.key]) ? +values[def.key] : def.def;
    const scene = tpl.build(p);
    scene.template = templateKey;
    scene.params = p;
    return scene;
  }

  // ---------- planner (bidirectional RRT-Connect over position + orientation) ----------
  const TRAPPED = 0, ADVANCED = 1, REACHED = 2;

  function createPlanner(scene, obj, opts) {
    opts = opts || {};
    const rng = mulberry32(opts.seed || 12345);
    const W = obj.radius;                       // turns an angle into the distance a far corner travels
    const stepSize = opts.stepSize || 14;
    const res = opts.resolution || 2;
    const stats = { samples: 0, nodes: 0, checks: 0 };
    const gauss = () => { let u = 0; while (u === 0) u = rng(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * rng()); };

    const free = (p, q) => { stats.checks++; return !collide(scene, obj, p, q, false); };
    const dist = (a, b) => Math.hypot(a.p[0] - b.p[0], a.p[1] - b.p[1], a.p[2] - b.p[2]) + W * Q.angle(a.q, b.q);
    const interp = (a, b, t) => ({
      p: [a.p[0] + (b.p[0] - a.p[0]) * t, a.p[1] + (b.p[1] - a.p[1]) * t, a.p[2] + (b.p[2] - a.p[2]) * t],
      q: Q.slerp(a.q, b.q, t),
    });
    // Checks the end pose first, then bisects, so blocked moves fail after few checks.
    function edgeFree(a, b) {
      const n = Math.ceil(dist(a, b) / res);
      if (!free(b.p, b.q)) return false;
      const seen = new Uint8Array(n + 1);
      seen[0] = seen[n] = 1;
      for (let div = 2; ; div *= 2) {
        for (let j = 1; j < div; j += 2) {
          const i = Math.round(j * n / div);
          if (seen[i]) continue;
          seen[i] = 1;
          const s = interp(a, b, i / n);
          if (!free(s.p, s.q)) return false;
        }
        if (div >= 2 * n) break;
      }
      return true;
    }
    function pickRegion(list) {
      let total = 0;
      for (const r of list) total += r.weight;
      let x = rng() * total;
      for (const r of list) { x -= r.weight; if (x <= 0) return r; }
      return list[list.length - 1];
    }
    const inRegion = (r) => [0, 1, 2].map((i) => r.min[i] + rng() * (r.max[i] - r.min[i]));

    // Trees keep a position grid so nearest-neighbour lookups stay fast as they grow.
    const CELL = 40;
    const cellKey = (i, j, k) => ((i + 512) * 1024 + (j + 512)) * 1024 + (k + 512);
    function makeTree() {
      const tree = [];
      tree.grid = new Map();
      tree.cells = [];
      tree.add = (node) => {
        const idx = tree.length;
        tree.push(node);
        const key = cellKey(Math.floor(node.p[0] / CELL), Math.floor(node.p[1] / CELL), Math.floor(node.p[2] / CELL));
        let list = tree.grid.get(key);
        if (!list) {
          tree.grid.set(key, (list = []));
          list.lo = [Math.floor(node.p[0] / CELL) * CELL, Math.floor(node.p[1] / CELL) * CELL, Math.floor(node.p[2] / CELL) * CELL];
          tree.cells.push(list);
        }
        list.push(idx);
        return idx;
      };
      return tree;
    }

    // Roots of the two trees.
    const startTree = makeTree(), goalTree = makeTree();
    // opts.startPose / opts.goalPose pin either end to one exact pose instead of the template's areas.
    const sp = opts.startPose || scene.startPose(obj);
    if (free(sp.p, sp.q)) startTree.add({ p: sp.p, q: sp.q, parent: -1 });
    for (let i = 0; i < 3000 && startTree.length === 0 && !opts.startPose; i++) {
      const p = inRegion(pickRegion(scene.startRegions));
      const q = rng() < 0.5 ? AXIS_ROTATIONS[Math.floor(rng() * 24)] : Q.random(rng);
      if (scene.inStart(worldCorners(obj, p, q)) && free(p, q)) startTree.add({ p, q, parent: -1 });
    }
    if (opts.goalPose && free(opts.goalPose.p, opts.goalPose.q)) goalTree.add({ p: opts.goalPose.p, q: opts.goalPose.q, parent: -1 });
    for (let i = 0; i < 40000 && goalTree.length < 30 && !opts.goalPose; i++) {
      const p = inRegion(pickRegion(scene.goalRegions));
      let q;
      if (rng() < 0.7) {
        const tilt = scene.goalTilts[Math.floor(rng() * scene.goalTilts.length)];
        q = Q.mul(tilt, AXIS_ROTATIONS[Math.floor(rng() * 24)]);
      } else q = Q.random(rng);
      if (scene.inGoal(worldCorners(obj, p, q)) && free(p, q)) goalTree.add({ p, q, parent: -1 });
    }

    const state = {
      stats,
      status: !startTree.length ? 'no-start' : !goalTree.length ? 'no-goal' : 'searching',
      path: null,
      startPose: startTree[0] || sp,
    };

    const WORLD_AXES = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    // Real moves are mostly "square-on, tipped in one plane", so favour orientations of that shape.
    function sampleOrientation() {
      if (rng() < 0.35) return Q.random(rng);
      const base = AXIS_ROTATIONS[Math.floor(rng() * 24)];
      const tilt = Q.axisAngle(WORLD_AXES[Math.floor(rng() * 3)], (rng() * 2 - 1) * Math.PI / 2);
      const wobble = Q.axisAngle(WORLD_AXES[Math.floor(rng() * 3)], (rng() * 2 - 1) * 0.12);
      return Q.mul(wobble, Q.mul(tilt, base));
    }
    function sample() {
      stats.samples++;
      return { p: inRegion(pickRegion(scene.regions)), q: sampleOrientation() };
    }
    // Exact nearest neighbour: seed with the surrounding cells, then skip any cell that can't beat the best so far.
    function nearest(tree, x) {
      const px = x.p[0], py = x.p[1], pz = x.p[2];
      const ci = Math.floor(px / CELL), cj = Math.floor(py / CELL), ck = Math.floor(pz / CELL);
      let best = -1, bd = Infinity;
      const scan = (list) => {
        for (let t = 0; t < list.length; t++) {
          const n = tree[list[t]];
          const dp = Math.hypot(n.p[0] - px, n.p[1] - py, n.p[2] - pz);
          if (dp >= bd) continue;
          const d = dp + W * Q.angle(n.q, x.q);
          if (d < bd) { bd = d; best = list[t]; }
        }
      };
      for (let i = ci - 1; i <= ci + 1; i++) for (let j = cj - 1; j <= cj + 1; j++) for (let k = ck - 1; k <= ck + 1; k++) {
        const list = tree.grid.get(cellKey(i, j, k));
        if (list) { list.seen = x; scan(list); }
      }
      for (const list of tree.cells) {
        if (list.seen === x) continue;
        const lo = list.lo;
        const dx = Math.max(lo[0] - px, 0, px - lo[0] - CELL);
        const dy = Math.max(lo[1] - py, 0, py - lo[1] - CELL);
        const dz = Math.max(lo[2] - pz, 0, pz - lo[2] - CELL);
        if (Math.hypot(dx, dy, dz) < bd) scan(list);
      }
      return best;
    }

    // Bridge test: two nearby blocked poses whose midpoint is free usually sit in a narrow gap.
    function bridgeSample() {
      for (let tries = 0; tries < 30; tries++) {
        const a = sample();
        if (free(a.p, a.q)) continue;
        const b = {
          p: [a.p[0] + gauss() * 15, a.p[1] + gauss() * 15, a.p[2] + gauss() * 15],
          q: Q.mul(Q.axisAngle(WORLD_AXES[Math.floor(rng() * 3)], gauss() * 0.3), a.q),
        };
        if (free(b.p, b.q)) continue;
        const m = interp(a, b, 0.5);
        if (free(m.p, m.q)) return m;
      }
      return sample();
    }

    function extend(tree, target) {
      const i = nearest(tree, target), from = tree[i];
      const d = dist(from, target);
      const reached = d <= stepSize;
      const to = reached ? target : interp(from, target, stepSize / d);
      if (!edgeFree(from, to)) return TRAPPED;
      tree.add({ p: to.p, q: to.q, parent: i });
      stats.nodes++;
      return reached ? REACHED : ADVANCED;
    }
    function connect(tree, target) {
      let s = ADVANCED;
      for (let k = 0; k < 400 && s === ADVANCED; k++) s = extend(tree, target);
      return s;
    }
    const chain = (tree, i) => { const out = []; for (; i >= 0; i = tree[i].parent) out.push({ p: tree[i].p, q: tree[i].q }); return out; };

    // Nudge a node from a randomly chosen grid cell, so sparse frontiers get as much attention as dense ones.
    function expandLocal(tree) {
      const list = tree.cells[Math.floor(rng() * tree.cells.length)];
      const i = list[Math.floor(rng() * list.length)], from = tree[i];
      const axis = rng() < 0.6 ? WORLD_AXES[Math.floor(rng() * 3)] : Q.toMat(Q.random(rng)).slice(0, 3);
      const to = {
        p: [from.p[0] + gauss() * 10, from.p[1] + gauss() * 10, from.p[2] + gauss() * 10],
        q: Q.mul(Q.axisAngle(axis, gauss() * 0.25), from.q),
      };
      if (!edgeFree(from, to)) return false;
      tree.add({ p: to.p, q: to.q, parent: i });
      stats.nodes++;
      return true;
    }

    let fromStart = true;
    function iterate() {
      const a = fromStart ? startTree : goalTree, b = fromStart ? goalTree : startTree;
      fromStart = !fromStart;
      const u = rng();
      if (u < 0.45) {
        if (!expandLocal(a)) return false;
      } else if (extend(a, u < 0.7 ? bridgeSample() : sample()) === TRAPPED) return false;
      const fresh = a[a.length - 1];
      if (connect(b, fresh) !== REACHED) return false;
      const ca = chain(a, a.length - 1), cb = chain(b, b.length - 1);
      const s = a === startTree ? ca : cb, g = a === startTree ? cb : ca;
      state.path = s.reverse().concat(g.slice(1));
      return true;
    }

    function smooth(path, ms) {
      const t0 = Date.now();
      path = path.slice();
      while (Date.now() - t0 < ms && path.length > 2) {
        const i = Math.floor(rng() * (path.length - 2));
        const j = i + 2 + Math.floor(rng() * (path.length - i - 2));
        if (edgeFree(path[i], path[j])) path.splice(i + 1, j - i - 1);
      }
      return path;
    }

    // Run for up to `ms` milliseconds; returns the state.
    state.run = function (ms) {
      if (state.status !== 'searching') return state;
      const t0 = Date.now();
      while (Date.now() - t0 < ms) {
        if (iterate()) {
          state.rawPath = state.path;
          state.path = smooth(state.path, opts.smoothMs || 400);
          state.status = 'found';
          break;
        }
      }
      return state;
    };
    state.trees = { start: startTree, goal: goalTree };
    state.length = (path) => { let s = 0; for (let i = 1; i < path.length; i++) s += dist(path[i - 1], path[i]); return s; };
    state.dist = dist;
    state.interp = interp;
    return state;
  }

  const api = { Q, AXIS_ROTATIONS, TEMPLATES, buildScene, makeObject, worldCorners, collide, createPlanner, mulberry32 };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else global.CouchSim = api;
})(this);
