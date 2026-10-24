# -*- coding: utf-8 -*-
# ============================================================================
#  Transition Mesh
#  created by IY,China University of Petroleum (East China)
#  禁止商用 / Non-commercial use only
#  禁止商用：本软件仅供学习与研究使用，未经作者书面许可不得用于任何商业用途。
#  Non-commercial use only. Unauthorized commercial use is prohibited.
# ============================================================================

"""Native CAE adapter for the case/squad reference topology (pure 2:1)."""
import math
import bisect

import transition_mesh_quad2 as geometry
from transition_mesh_plugin import _single


def rectangle_frame(part, edges):
    """Rectangle from an intact or already segmented loop; pick-order invariant."""
    import transition_mesh_plugin_planar2d as p
    links = {}
    for e in edges:
        vs = e.getVertices()
        if len(vs) != 2:
            raise ValueError('The fine/coarse loop must contain straight edges.')
        a, b = vs
        links.setdefault(a, []).append(b)
        links.setdefault(b, []).append(a)
    if not links or any(len(v) != 2 for v in links.values()):
        raise ValueError('Pick one complete closed rectangle loop, without duplicate edges.')
    start = min(links)
    order, previous, current = [], None, start
    while current not in order:
        order.append(current)
        choices = links[current]
        nxt = choices[0] if choices[0] != previous else choices[1]
        previous, current = current, nxt
    if current != start or len(order) != len(links):
        raise ValueError('The selection contains disconnected loops.')
    points = [tuple(part.vertices[v].pointOn[0]) for v in order]
    corners = []
    for i, b in enumerate(points):
        a, c = points[i - 1], points[(i + 1) % len(points)]
        u, v = p._vec_sub(b, a), p._vec_sub(c, b)
        if abs(p._vec_dot(u, v)) < (1.0 - 1.0e-5) * p._vec_norm(u) * p._vec_norm(v):
            corners.append(b)
    if len(corners) != 4:
        raise ValueError('The fine/coarse loop must resolve to four rectangle corners.')
    # Keep the loop order.  _rect_frame_from_loop validates right angles and
    # works in both 2D and an arbitrary planar face of a 3D solid.
    fr = p._rect_frame_from_loop(corners)
    # Nx follows the in-plane axis most aligned with global X (then Y/Z),
    # independent of pick order and valid on side faces too.
    choices = [fr['ux'], fr['uy']]
    ux = max(choices, key=lambda a: tuple(round(abs(x), 8) for x in a))
    if p._axis_sign(ux) < 0:
        ux = p._vec_scale(ux, -1.0)
    other = fr['uy'] if abs(p._vec_dot(ux, fr['ux'])) > 0.9 else fr['ux']
    if p._axis_sign(other) < 0:
        other = p._vec_scale(other, -1.0)
    fr['ux'], fr['uy'] = ux, other
    loc = [p._to_local(fr, xy) for xy in corners]
    fr['hx'] = max(abs(v[0]) for v in loc)
    fr['hy'] = max(abs(v[1]) for v in loc)
    return fr


def build_plan(part, region_faces, fine_edges, coarse_edges, nx, ny,
               domain_faces=None, mesh_regular=False, stages=1,
               ratioSequence='', stageWidths=''):
    import transition_mesh_plugin_planar2d as p
    import transition_mesh_plan as counts
    seq = counts.parse_sequence(ratioSequence, counts._ratio)
    if seq and seq != [2] * int(stages):
        raise ValueError('Reference four-side 2:1 requires one 2:1 ratio per stage.')
    fr = rectangle_frame(part, fine_edges)
    geo = geometry.build_plan(fr['hx'], fr['hy'], nx, ny, stages)
    if str(stageWidths or '').strip():
        widths = counts.parse_sequence(stageWidths)
        expected = [s['V'] - s['inner_hy'] for s in geo['levels']]
        if len(widths) != len(expected) or any(abs(a - b) > 1e-7 * max(geo['V'], 1)
                                               for a, b in zip(widths, expected)):
            raise ValueError('Case-style 2:1 stage widths (including regular collars) '
                             'must be %s; clear Stage widths for automatic sizing.' % expected)
    auto = not coarse_edges
    if not auto:
        out = rectangle_frame(part, coarse_edges)
        if not p._axes_aligned(fr, out):
            raise ValueError('Fine and coarse rectangle axes must be parallel.')
        p._align_frame_axes(out, fr)
        tol = 1e-7 * max(geo['H'], geo['V'], 1)
        if (p._frame_center_distance(fr, out) > tol
                or abs(out['hx'] - geo['H']) > tol or abs(out['hy'] - geo['V']) > tol):
            raise ValueError('Case-style 2:1 requires a concentric coarse frame with '
                             'half-sizes H=%.9g V=%.9g (including regular collars). '
                             'Pick that frame or leave Coarse empty to create it.'
                             % (geo['H'], geo['V']))
    topo = geometry.build_topology(geo)
    targets = {f.index: f for f in list(region_faces or []) + list(domain_faces or [])}
    if not targets:
        raise ValueError('Select the Face(s) covering the transition rings.')
    target_faces = list(targets.values())
    H, V = geo['H'], geo['V']
    # Expected full mesh in local coordinates.
    # Only the band is included unless the user explicitly selected a regular domain.
    quads = [[topo['vertices'][n] for n in q] for q in topo['quads']]
    roles = list(topo['roles'])
    partition_quads = list(quads)

    def grid(xs, ys, role):
        if len(xs) > 1 and len(ys) > 1:
            partition_quads.append([(xs[0], ys[0]), (xs[-1], ys[0]),
                                    (xs[-1], ys[-1]), (xs[0], ys[-1])])
        for i in range(len(xs) - 1):
            for j in range(len(ys) - 1):
                quads.append([(xs[i], ys[j]), (xs[i + 1], ys[j]),
                              (xs[i + 1], ys[j + 1]), (xs[i], ys[j + 1])])
                roles.append(role)

    ext = None
    if mesh_regular and domain_faces:
        boxes = [p._face_local_bbox(part, f, fr) for f in target_faces]
        boxes = [b for b in boxes if b]
        u0, u1 = min(b[0] for b in boxes), max(b[1] for b in boxes)
        v0, v1 = min(b[2] for b in boxes), max(b[3] for b in boxes)
        if not p._rect_outer_covered(part, target_faces, fr, u0, u1, v0, v1):
            raise ValueError('Full domain must have a rectangular outer boundary.')
        tol = 1e-7 * max(H, V, 1)
        if u0 > -H + tol or u1 < H - tol or v0 > -V + tol or v1 < V - tol:
            raise ValueError('Full domain is too small: need H=%.9g V=%.9g.' % (H, V))

        def existing_margin(axis_id, lo, hi, boundary):
            if hi - lo <= tol:
                return None
            spans, number = [], 0
            for e in part.edges:
                ids = e.getVertices()
                if len(ids) != 2:
                    continue
                ends = [p._to_local(fr, part.vertices[v].pointOn[0]) for v in ids]
                a, b = sorted(v[axis_id] for v in ends)
                if (all(abs(v[1-axis_id] - boundary) <= tol for v in ends)
                        and a >= lo-tol and b <= hi+tol and b-a > tol):
                    seeded = p._edge_seed_count(part, e)
                    if not seeded:
                        seeded = max(0, len(e.getNodes()) - 1)
                    if not seeded:
                        return None
                    spans.append((a, b))
                    number += seeded
            spans.sort()
            if (not spans or abs(spans[0][0]-lo) > tol or abs(spans[-1][1]-hi) > tol
                    or any(abs(a[1]-b[0]) > tol for a, b in zip(spans, spans[1:]))):
                return None
            return number

        def axis(lo, hi, half, n, step, axis_id, bounds):
            left = max(0, int(round((-half - lo) / step)))
            right = max(0, int(round((hi - half) / step)))
            if -half - lo > tol:
                left = max(1, left)
            if hi - half > tol:
                right = max(1, right)
            for side, (a, b) in enumerate(((lo, -half), (half, hi))):
                seeds = [existing_margin(axis_id, a, b, bound) for bound in bounds]
                known = {s for s in seeds if s is not None}
                if len(known) > 1:
                    raise ValueError('Opposite external margins have incompatible existing seeds.')
                if known:
                    if side == 0:
                        left = known.pop()
                    else:
                        right = known.pop()
            xs = ([lo + (-half - lo) * i / left for i in range(left)] if left else [])
            xs += [-half + 2 * half * i / n for i in range(n + 1)]
            xs += [half + (hi - half) * i / right for i in range(1, right + 1)]
            return xs, left, right

        xs, kl, kr = axis(u0, u1, H, geo['chain_x']['counts'][-1], geo['coarse_hx'], 0, (v0, v1))
        ys, kb, kt = axis(v0, v1, V, geo['chain_y']['counts'][-1], geo['coarse_hy'], 1, (u0, u1))
        # Separate the corner blocks:
        # the far-side margin may have a different spacing from the transition outlet.
        middle_x = xs[kl:len(xs) - kr] if kr else xs[kl:]
        for xspan in (xs[:kl + 1], middle_x, xs[len(xs) - kr - 1:]):
            grid(xspan, ys[:kb + 1], 'external')
            grid(xspan, ys[len(ys) - kt - 1:], 'external')
        middle_y = ys[kb:len(ys) - kt] if kt else ys[kb:]
        grid(xs[:kl + 1], middle_y, 'external')
        grid(xs[len(xs) - kr - 1:], middle_y, 'external')
        # @author  created by IY,China University of Petroleum (East China)  --  禁止商用 (non-commercial)
        grid([-geo['hx'] + 2 * geo['hx'] * j / nx for j in range(nx + 1)],
             [-geo['hy'] + 2 * geo['hy'] * j / ny for j in range(ny + 1)], 'center')
        ext = {'k_left': kl, 'k_right': kr, 'k_top': kt, 'k_bottom': kb}

    # Check every proposed cell against the explicitly selected faces. This is
    # read-only and prevents automatic partitioning outside the selected scope.
    for q in quads:
        mid = p._to_global(fr, sum(v[0] for v in q) / 4, sum(v[1] for v in q) / 4)
        try:
            found = part.faces.findAt((p._point3(mid),), printWarning=False)
            face = _single(found)
            if not hasattr(face, 'index') or callable(face.index):
                raise ValueError('No unique live face found at %s.' % (mid,))
        except Exception:
            raise ValueError('Automatic frame crosses a hole or leaves the Part at %s.' % (mid,))
        if face.index not in targets:
            raise ValueError('Automatic frame/regular mesh leaves the selected Faces at %s. '
                             'Include the covering Face in Region / Full domain.' % (mid,))
    return {'reference_quad2': True, 'label': 'case-squad-2to1', 'mode': 'PLANAR_QUAD_FOUR', 'ratio': 2,
            'frame_in': fr, 'hxi': geo['hx'], 'hyi': geo['hy'], 'H': H, 'V': V,
            'Nx': nx, 'Ny': ny, 'stages': int(stages), 'geometry': geo, 'topology': topo,
            'quads_xy': quads, 'partition_quads': partition_quads,
            'quad_roles': roles, 'target_faces': target_faces,
            'auto_coarse': auto, 'corner_participating': True,
            'stage_counts_x': geo['chain_x']['counts'],
            'stage_counts_y': geo['chain_y']['counts'],
            'nx_coarse': geo['chain_x']['counts'][-1],
            'ny_coarse': geo['chain_y']['counts'][-1],
            'external': ext, 'external_expected': roles.count('external'),
            'external_note': 'frame' if ext is not None else 'none',
            'rule': {'h_u': 2 * geo['hx'] / nx, 'h_v': 2 * geo['hy'] / ny,
                     'width_u': [s['H'] - s['inner_hx'] for s in geo['levels']],
                     'width_v': [s['V'] - s['inner_hy'] for s in geo['levels']],
                     'H_required': H, 'V_required': V},
            'model': p._owner_model(part), 'sheet_size': 4 * max(H, V, 1),
            'sketch_origin': p._point3(p._to_global(fr, 0.0, 0.0)),
            'sketch_up_edge': fine_edges[0],
            'sketch_name': '__tm2d_reference_quad2__'}


def apply(part, plan, action, element2d):
    import transition_mesh_plugin_planar2d as p
    from abaqusConstants import QUAD, STRUCTURED, FIXED
    from transition_mesh_runtime import transaction_active, ModelTransaction
    if not transaction_active():
        with ModelTransaction(p.mdb, plan['model'].name):
            return apply(part, plan, action, element2d)
    scale = max(plan['H'], plan['V'])
    axes = [sorted({round(v[axis] / scale, 10) for q in plan['quads_xy'] for v in q})
            for axis in (0, 1)]

    def key(pt):
        # created by IY,China University of Petroleum (East China)
        # CAE mesh node coordinates use single precision.
        # Snap only within a bounded relative tolerance to known planned coordinates;
        out = []
        for axis in (0, 1):
            v = pt[axis] / scale
            pos = bisect.bisect_left(axes[axis], v)
            candidates = axes[axis][max(0, pos - 1):pos + 1]
            near = min(candidates, key=lambda x: abs(x - v))
            out.append(near if abs(near - v) <= 1e-6 else round(v, 10))
        return tuple(out)

    def draw(sk, _):
        edges = {}
        vertices = {key(v): v for q in plan['partition_quads'] for v in q}
        # A regular block's long boundary overlaps several short template boundaries.
        # Split it at ALL geometric vertices before deduplication;
        for quad in plan['partition_quads']:
            for a, b in zip(quad, quad[1:] + quad[:1]):
                dx, dy = b[0] - a[0], b[1] - a[1]
                length = math.hypot(dx, dy)
                tol = scale * 1e-8
                points = [(k, v) for k, v in vertices.items()
                          if min(a[0], b[0])-tol <= v[0] <= max(a[0], b[0])+tol
                          and min(a[1], b[1])-tol <= v[1] <= max(a[1], b[1])+tol
                          and abs(dx*(v[1]-a[1])-dy*(v[0]-a[0])) <= tol*length]
                points.sort(key=lambda item: dx*item[1][0] + dy*item[1][1])
                for (ka, va), (kb, vb) in zip(points, points[1:]):
                    if ka != kb:
                        edges[tuple(sorted((ka, kb)))] = (va, vb)
        fr, origin = plan['frame_in'], plan['sketch_origin']
        native = []
        for edge in part.edges:
            vs = edge.getVertices()
            if len(vs) == 2:
                native.append([p._to_local(fr, part.vertices[i].pointOn[0]) for i in vs])

        def already_present(a, b):
            tol = scale * 1e-6
            for u, v in native:
                dx, dy = v[0] - u[0], v[1] - u[1]
                length = math.hypot(dx, dy)
                if all(min(u[0], v[0])-tol <= w[0] <= max(u[0], v[0])+tol
                       and min(u[1], v[1])-tol <= w[1] <= max(u[1], v[1])+tol
                       and abs(dx*(w[1]-u[1])-dy*(w[0]-u[0])) <= tol*length for w in (a, b)):
                    return True
            return False

        for a, b in edges.values():
            if already_present(a, b):
                continue
            ga, gb = p._to_global(fr, *a), p._to_global(fr, *b)
            sk.Line(point1=p._sketch_xy(plan, ga),
                    point2=p._sketch_xy(plan, gb))

    expected = {tuple(sorted(key(v) for v in q)): role
                for q, role in zip(plan['quads_xy'], plan['quad_roles'])}
    partition_expected = {tuple(sorted(key(v) for v in q)) for q in plan['partition_quads']}
    if len(expected) != len(plan['quads_xy']):
        raise ValueError('Duplicate planned reference cell.')

    def face_signature(f):
        coords = [p._to_local(plan['frame_in'], part.vertices[v].pointOn[0])
                  for v in f.getVertices()]
        if len(coords) > 4:
            xs, ys = [v[0] for v in coords], [v[1] for v in coords]
            a, b, c, d = min(xs), max(xs), min(ys), max(ys)
            tol = scale * 1e-6
            # created by IY,China University of Petroleum (East China)  |  禁止商用 / non-commercial use only
            if all(min(abs(x-a), abs(x-b), abs(y-c), abs(y-d)) < tol for x, y in coords):
                coords = [(a, c), (b, c), (b, d), (a, d)]
        return tuple(sorted(key(xy) for xy in coords))

    existing = {face_signature(f): f.index for f in part.faces}
    reused = all(k in existing for k in partition_expected)
    if not reused:
        p._partition_patch_plan(part, plan, draw)
    matched = {face_signature(f): f.index for f in part.faces if face_signature(f) in partition_expected}
    if set(matched) != partition_expected:
        raise ValueError('Native partition differs from reference topology: %d/%d cells. '
                         'Remove conflicting pre-partitions inside the transition region.'
                         % (len(matched), len(partition_expected)))
    ids = sorted(matched.values())
    result = {'mode': 'PLANAR_QUAD_FOUR', 'ratio': 2, 'stages': plan['stages'],
              'Nx': plan['Nx'], 'Ny': plan['Ny'], 'corner_participating': True,
              'auto_coarse': plan['auto_coarse'], 'H': plan['H'], 'V': plan['V'],
              'stage_counts_x': plan['stage_counts_x'], 'stage_counts_y': plan['stage_counts_y'],
              'stage_details': plan['geometry']['levels'],
              'mesh_strategy': 'case_squad_native_quad2', 'reused_partition': reused,
              'expected_elements': len(expected), 'expected_transition_faces': len(plan['topology']['quads']),
              'transition_elements_expected': plan['topology']['transition_quads'],
              'collar_elements_expected': plan['topology']['collar_quads'],
              'center_expected': plan['quad_roles'].count('center'),
              'external_expected': plan['quad_roles'].count('external'),
              'action': action, 'status': action, 'compatibility': 'OK'}
    if action == 'PARTITION_ONLY':
        result['faces_meshed'] = 0
        return result
    edge_ids = sorted({e for i in ids for e in part.faces[i].getEdges()})
    points = {key(v): v for q in plan['quads_xy'] for v in q}
    groups = {}
    for ei in edge_ids:
        e = part.edges[ei]
        a, b = [tuple(x * scale for x in key(p._to_local(plan['frame_in'], part.vertices[v].pointOn[0])))
                for v in e.getVertices()]
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        tol = scale * 1e-8
        on = [v for v in points.values()
              if min(a[0], b[0])-tol <= v[0] <= max(a[0], b[0])+tol
              and min(a[1], b[1])-tol <= v[1] <= max(a[1], b[1])+tol
              and abs(dx*(v[1]-a[1])-dy*(v[0]-a[0])) <= tol*math.sqrt(length2)]
        number = len(on) - 1
        if number < 1:
            raise ValueError('Native edge is missing planned mesh nodes.')
        groups.setdefault(number, []).append(e)
    for number, edges in groups.items():
        part.seedEdgeByNumber(edges=tuple(edges), number=number, constraint=FIXED)
    if action == 'PARTITION_AND_SEED':
        if p._is_solid(part):
            return p._finish_solid_mesh(
                part, result, plan.get('solid_transition_cells'), action)
        result['faces_meshed'] = 0
        return result
    if p._is_solid(part):
        return p._finish_solid_mesh(
            part, result, plan.get('solid_transition_cells'), action)
    faces = p._face_seq(part, ids)
    part.setMeshControls(regions=faces, elemShape=QUAD, technique=STRUCTURED)
    p._apply_elem_type(part, ids, element2d)
    part.generateMesh(regions=faces)
    audit = p._audit_patch_mesh(part, ids)
    result.update(audit)
    if audit['elements'] != len(expected) or any(audit.get(k) for k in
            ('tri', 'unmeshed_faces', 'duplicate_elements', 'negative_area',
             'zero_area', 'bowtie', 'hanging_nodes', 'jacobian_bad')):
        raise ValueError('Reference mesh audit failed: %s' % audit)
    actual, coord_labels = set(), {}
    for i in ids:
        for el in part.faces[i].getElements():
            pts = []
            for n in el.getNodes():
                xy = p._to_local(plan['frame_in'], n.coordinates)
                k = key(xy)
                coord_labels.setdefault(k, set()).add(n.label)
                pts.append(k)
            actual.add(tuple(sorted(pts)))
    if actual != set(expected) or any(len(v) != 1 for v in coord_labels.values()):
        raise ValueError('Actual nodes/connectivity differ from reference: missing=%d extra=%d '
                         'duplicate_positions=%d; sample=%s.'
                         % (len(set(expected) - actual), len(actual - set(expected)),
                            sum(len(v) != 1 for v in coord_labels.values()),
                            list(actual - set(expected))[:1]))
    result.update(status='PASS', faces_meshed=len(ids), mesh_control='QUAD+STRUCTURED',
                  matched_quads=len(actual), shared_nodes_verified=True,
                  corner_diagonals_verified=4 * plan['stages'])
    return result
