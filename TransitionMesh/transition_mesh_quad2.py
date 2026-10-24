# -*- coding: utf-8 -*-
# ============================================================================
#  Transition Mesh
#  created by IY,China University of Petroleum (East China)
#  禁止商用 / Non-commercial use only
#  禁止商用：本软件仅供学习与研究使用，未经作者书面许可不得用于任何商业用途。
#  Non-commercial use only. Unauthorized commercial use is prohibited.
# ============================================================================

"""Pure geometry, shared by Check and native CAE partitioning."""
import math

from transition_mesh_corner_templates import (
    build_corner_2to1_wedge, _rect_quad_audit)


def count_chain(n, stages):
    if isinstance(n, bool) or int(n) != n or int(n) < 4 or int(n) % 4:
        raise ValueError('Four-side 2:1 fine count must be a positive multiple of 4.')
    if isinstance(stages, bool) or int(stages) != stages or int(stages) < 1:
        raise ValueError('Transition stages must be a positive integer.')
    counts, inputs, collars = [int(n)], [], []
    for i in range(int(stages)):
        pad = 0 if i == 0 else (counts[-1] % 4) // 2
        current = counts[-1] + 2 * pad
        inputs.append(current)
        collars.append(pad)
        counts.append(current // 2 + 2)
    return {'counts': counts, 'inputs': inputs, 'collars': collars,
            'stages': int(stages), 'ratios': [2] * int(stages),
            'required_multiple': 4, 'geometry_checked': False}


def build_plan(hx, hy, nx, ny, stages=1):
    hx, hy = float(hx), float(hy)
    if not all(math.isfinite(v) and v > 0 for v in (hx, hy)):
        raise ValueError('Fine rectangle half sizes must be finite and positive.')
    cx, cy = count_chain(nx, stages), count_chain(ny, stages)
    nx, ny = int(nx), int(ny)
    x, y = hx, hy
    sx, sy = 2 * hx / nx, 2 * hy / ny
    rows = []
    for i in range(int(stages)):
        px, py = cx['collars'][i], cy['collars'][i]
        a, b = x + px * sx, y + py * sy
        rows.append({'stage': i + 1, 'inner_hx': x, 'inner_hy': y,
                     'hx': a, 'hy': b, 'H': a + 2 * sx, 'V': b + 2 * sy,
                     'nx': cx['inputs'][i], 'ny': cy['inputs'][i],
                     'out_nx': cx['counts'][i + 1],
                     'out_ny': cy['counts'][i + 1],
                     'hx_cell': sx, 'hy_cell': sy,
                     'collar_x': px, 'collar_y': py})
        x, y = a + 2 * sx, b + 2 * sy
        sx, sy = 2 * sx, 2 * sy
    return {'hx': hx, 'hy': hy, 'nx': nx, 'ny': ny, 'stages': int(stages),
            'H': x, 'V': y, 'coarse_hx': sx, 'coarse_hy': sy,
            'chain_x': cx, 'chain_y': cy, 'levels': rows}


def build_topology(plan):
    """Each motif has two fine edges, one coarse edge and lateral ports 2/1."""
    vertices, lookup, quads, roles, stage_ids, diagonals = [], {}, [], [], [], []
    scale = max(plan['H'], plan['V'])

    def node(x, y):
        key = (round(x / scale, 12), round(y / scale, 12))
        if key not in lookup:
            lookup[key] = len(vertices)
            vertices.append((x, y))
        return lookup[key]

    def add(points, role, stage):
        ids = tuple(node(x, y) for x, y in points)
        area = sum(points[k][0] * points[(k + 1) % 4][1]
                   - points[(k + 1) % 4][0] * points[k][1] for k in range(4))
        quads.append(ids if area > 0 else tuple(reversed(ids)))
        roles.append(role)
        # @author  created by IY,China University of Petroleum (East China)  --  禁止商用 (non-commercial)
        stage_ids.append(stage)

    def patch(dx, dy, transform, role, stage):
        p = build_corner_2to1_wedge(0, 0, dx, dy)
        for q in p['quads']:
            add([transform(*p['vertices'][v]) for v in q], role, stage)
        return p

    def regular(xs, ys, stage):
        for i in range(len(xs) - 1):
            for j in range(len(ys) - 1):
                add([(xs[i], ys[j]), (xs[i + 1], ys[j]),
                     (xs[i + 1], ys[j + 1]), (xs[i], ys[j + 1])],
                    'collar', stage)

    interfaces = []

    def interface(a, b, nx, ny, kind, stage):
        sides = {'bottom': [node(-a + 2 * a * j / nx, -b) for j in range(nx + 1)],
                 'top': [node(-a + 2 * a * j / nx, b) for j in range(nx + 1)],
                 'left': [node(-a, -b + 2 * b * j / ny) for j in range(ny + 1)],
                 'right': [node(a, -b + 2 * b * j / ny) for j in range(ny + 1)]}
        interfaces.append({'hx': a, 'hy': b, 'nx': nx, 'ny': ny,
                           'kind': kind, 'stage': stage, 'sides': sides})

    interface(plan['hx'], plan['hy'], plan['nx'], plan['ny'], 'fine', 0)
    for st in plan['levels']:
        i, a, b, h, v = st['stage'], st['hx'], st['hy'], st['H'], st['V']
        sx, sy, nx, ny = st['hx_cell'], st['hy_cell'], st['nx'], st['ny']
        old_a, old_b = st['inner_hx'], st['inner_hy']
        if st['collar_x']:
            ys = [-old_b + j * sy for j in range(ny - 2 * st['collar_y'] + 1)]
            regular([-a, -old_a], ys, i)
            regular([old_a, a], ys, i)
        if st['collar_y']:
            xs = [-a + j * sx for j in range(nx + 1)]
            regular(xs, [-b, -old_b], i)
            regular(xs, [old_b, b], i)
        if st['collar_x'] or st['collar_y']:
            interface(a, b, nx, ny, 'collar', i)
        for sign in (-1, 1):
            for k in range(nx // 2):
                start = -a + 2 * sx * k
                flip = k % 2
                patch(2 * sx, 2 * sy,
                      lambda u, w, start=start, flip=flip, sign=sign:
                      (start + (2 * sx - u if flip else u), sign * (b + w)),
                      'strip', i)
            for k in range(ny // 2):
                start = -b + 2 * sy * k
                flip = k % 2
                patch(2 * sy, 2 * sx,
                      lambda u, w, start=start, flip=flip, sign=sign:
                      (sign * (a + w), start + (2 * sy - u if flip else u)),
                      'strip', i)
        for su in (-1, 1):
            for sv in (-1, 1):
                transform = lambda u, w, su=su, sv=sv: (su * (a + u), sv * (b + w))
                p = patch(2 * sx, 2 * sy, transform, 'corner', i)
                diagonals.append((node(*transform(*p['vertices']['C'])),
                                  node(*transform(*p['vertices']['O']))))
        interface(h, v, st['out_nx'], st['out_ny'], 'coarse', i)

    def boundary(it):
        return {tuple(sorted((a, b))) for path in it['sides'].values()
                for a, b in zip(path, path[1:])}

    owners, area = _rect_quad_audit(vertices, quads,
                                   boundary(interfaces[0]) | boundary(interfaces[-1]),
                                   'reference 2:1 complete rings')
    # created by IY,China University of Petroleum (East China)
    for it in interfaces[1:-1]:
        if any(len(owners.get(e, [])) != 2 for e in boundary(it)):
            raise ValueError('Layer interface is not conformal.')
    if any(len(owners.get(tuple(sorted(e)), [])) != 2 for e in diagonals):
        raise ValueError('A corner diagonal is missing from the actual quad edges.')
    expected_area = 4 * (plan['H'] * plan['V'] - plan['hx'] * plan['hy'])
    if abs(area - expected_area) > expected_area * 1e-10:
        raise ValueError('Reference rings have a gap or overlap.')
    return {'vertices': vertices, 'quads': quads, 'edges': sorted(owners),
            'roles': roles, 'stage_ids': stage_ids, 'interfaces': interfaces,
            'diagonals': diagonals, 'area': area,
            'transition_quads': sum(r != 'collar' for r in roles),
            'collar_quads': roles.count('collar')}
