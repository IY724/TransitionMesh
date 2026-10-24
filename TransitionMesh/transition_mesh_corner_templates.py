# -*- coding: utf-8 -*-
# ============================================================================
#  Transition Mesh
#  created by IY,China University of Petroleum (East China)
#  禁止商用 / Non-commercial use only
#  禁止商用：本软件仅供学习与研究使用，未经作者书面许可不得用于任何商业用途。
#  Non-commercial use only. Unauthorized commercial use is prohibited.
# ============================================================================

"""Production sketch code lives in ``transition_mesh_plugin_planar2d``. This module has no Abaqus imports."""
from __future__ import print_function

import math


class CornerTemplateError(ValueError):
    """Raised when a corner cannot satisfy the declared port contract."""


def _xy(value):
    if isinstance(value, dict):
        value = value.get('point', value.get('xy'))
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        raise CornerTemplateError('corner point must be an (x, y) pair: %r'
                                  % (value,))
    return (float(value[0]), float(value[1]))


def _cross(a, b, c):
    return ((b[0] - a[0]) * (c[1] - a[1])
            - (b[1] - a[1]) * (c[0] - a[0]))
# @author  created by IY,China University of Petroleum (East China)  --  禁止商用 (non-commercial)


def _edge_key(a, b):
    return tuple(sorted((str(a), str(b))))


def _port_count(port):
    if port is None:
        return None
    if isinstance(port, int):
        return int(port)
    try:
        n = len(port)
    except Exception:
        return None
    if n == 0:
        return 0
    first = port[0]
    # A list of edge pairs is an edge count; a node path is edges = nodes-1.
    if isinstance(first, (list, tuple)) and len(first) == 2 \
            and not isinstance(first[0], (list, tuple)):
        # Could be either a node-name pair or an edge of node names.
        # Ports in this module are lists of edges, so count edges.
        return n
    return n - 1


def build_corner_patch_2to1(corner_box, port_counts=None, label='corner'):
    """``corner_box`` is ``(u0, u1, v0, v1)`` in the selected rectangle frame."""
    if not isinstance(corner_box, (list, tuple)) or len(corner_box) != 4:
        raise CornerTemplateError('%s needs (u0,u1,v0,v1)' % label)
    u0, u1, v0, v1 = [float(x) for x in corner_box]
    if not (u1 > u0 and v1 > v0):
        raise CornerTemplateError('%s has non-positive width/height' % label)
    if port_counts is None:
        raise CornerTemplateError(
            '%s has no extracted port counts; refusing to infer corner topology'
            % label)
    required = ('inner_u', 'outer_u', 'inner_v', 'outer_v')
    counts = {}
    for key in required:
        if key not in port_counts:
            raise CornerTemplateError('%s missing port count %s' % (label, key))
        count = _port_count(port_counts[key])
        if count != 1:
            raise CornerTemplateError(
                '%s requires one terminal edge on %s; got %r'
                % (label, key, port_counts[key]))
        counts[key] = 1

    vertices = {
        'SW': (u0, v0), 'SE': (u1, v0),
        'NE': (u1, v1), 'NW': (u0, v1),
    }
    quads = [('SW', 'SE', 'NE', 'NW')]
    ports = {
        'inner_u': [('SW', 'SE')],
        'outer_u': [('NW', 'NE')],
        'inner_v': [('SW', 'NW')],
        'outer_v': [('SE', 'NE')],
    }
    return {
        'template_id': 'RECT_CORNER_2TO1_PORT1',
        'label': str(label),
        'vertices': vertices,
        'quads': quads,
        'ports': ports,
        'port_counts': counts,
        'area': (u1 - u0) * (v1 - v0),
        'participating': False,
    }


def build_participating_corner_2to1(dx, dy, label='corner'):
    """Local frame: inner corner at ``(0, 0)``, outer at ``(dx, dy)``."""
    dx = float(dx)
    dy = float(dy)
    if not (dx > 0.0 and dy > 0.0):
        raise CornerTemplateError('%s needs positive dx/dy' % label)
    # Keep a geometric inset so every ring quad stays positive-area.
    mx = dx / 3.0
    my = dy / 3.0
    vertices = {
        'I': (0.0, 0.0),
        'Su': (dx, 0.0),
        'Wv': (0.0, dy),
        'O': (dx, dy),
        'i0': (mx, my),
        'i1': (dx - mx, my),
        'i2': (dx - mx, dy - my),
        'i3': (mx, dy - my),
    }
    quads = [
        ('I', 'Su', 'i1', 'i0'),
        ('Su', 'O', 'i2', 'i1'),
        ('O', 'Wv', 'i3', 'i2'),
        ('Wv', 'I', 'i0', 'i3'),
        ('i0', 'i1', 'i2', 'i3'),
    ]
    ports = {
        'inner_u': [('I', 'Su')],
        'outer_u': [('Wv', 'O')],
        'inner_v': [('I', 'Wv')],
        'outer_v': [('Su', 'O')],
    }
    return {
        'template_id': 'RECT_CORNER_2TO1_PARTICIPATING',
        'label': str(label),
        'vertices': vertices,
        'quads': quads,
        'ports': ports,
        'port_counts': {
            'inner_u': 1, 'outer_u': 1, 'inner_v': 1, 'outer_v': 1,
        },
        'area': dx * dy,
        'participating': True,
        'outer_contribution_u': 1,
        'outer_contribution_v': 1,
    }


def corner_draw_lines(patch):
    """The side strips already draw the inner_u / inner_v terminal edges."""
    if not patch or 'vertices' not in patch:
        raise CornerTemplateError('corner patch is missing vertices')
    vertices = patch['vertices']
    shared = set()
    for name in ('inner_u', 'inner_v'):
        for a, b in (patch.get('ports') or {}).get(name, []):
            shared.add(_edge_key(a, b))
    seen = set(shared)
    lines = []
    for quad in patch.get('quads', []):
        for i, name in enumerate(quad):
            other = quad[(i + 1) % 4]
            key = _edge_key(name, other)
            if key in seen:
                continue
            seen.add(key)
            lines.append((_xy(vertices[name]), _xy(vertices[other])))
    return lines


def patch_lines(patch):
    """Return unique sketch lines as ``((u0,v0),(u1,v1))`` tuples."""
    if not patch or 'vertices' not in patch:
        raise CornerTemplateError('corner patch is missing vertices')
    vertices = patch['vertices']
    seen = set()
    lines = []
    for quad in patch.get('quads', []):
        if len(quad) != 4:
            raise CornerTemplateError('corner patch quad must have four nodes')
        for i, name in enumerate(quad):
            other = quad[(i + 1) % 4]
            key = _edge_key(name, other)
            if key in seen:
                continue
            seen.add(key)
            lines.append((_xy(vertices[name]), _xy(vertices[other])))
    # Also emit port edges that are not yet drawn (shared with side strips).
    for edges in (patch.get('ports') or {}).values():
        for a, b in edges:
            key = _edge_key(a, b)
            if key in seen:
                continue
            seen.add(key)
            lines.append((_xy(vertices[a]), _xy(vertices[b])))
    return lines


def validate_corner_patch(patch, tol=1.0e-12):
    """Validate closure, positive orientation and declared ports."""
    if not patch:
        raise CornerTemplateError('empty corner patch')
    template_id = patch.get('template_id')
    if template_id not in (
            'RECT_CORNER_2TO1_PORT1',
            'RECT_CORNER_2TO1_PARTICIPATING'):
        raise CornerTemplateError('unsupported corner template %r' % template_id)
    vertices = patch.get('vertices') or {}
    quads = patch.get('quads') or []
    if template_id == 'RECT_CORNER_2TO1_PORT1':
        if len(quads) != 1 or len(quads[0]) != 4:
            raise CornerTemplateError('legacy corner patch must contain one quad')
    else:
        if len(quads) < 2:
            raise CornerTemplateError(
                'participating 2:1 corner must contain at least two quads')
    total_area2 = 0.0
    for quad in quads:
        if len(quad) != 4:
            raise CornerTemplateError('corner patch quad must have four nodes')
        points = [_xy(vertices[name]) for name in quad]
        area2 = sum(points[i][0] * points[(i + 1) % 4][1]
                    - points[(i + 1) % 4][0] * points[i][1]
                    for i in range(4))
        if area2 <= tol:
            raise CornerTemplateError(
                'corner patch quad %r has non-positive orientation' % (quad,))
        total_area2 += area2
    ports = patch.get('ports') or {}
    for name in ('inner_u', 'outer_u', 'inner_v', 'outer_v'):
        if name not in ports or not ports[name]:
            raise CornerTemplateError('corner port %s is missing' % name)
    if template_id == 'RECT_CORNER_2TO1_PARTICIPATING':
        declared = patch.get('port_counts') or {}
        if declared.get('outer_u') != 1 or _port_count(ports['outer_u']) != 1:
            raise CornerTemplateError('outer_u must stay one coarse edge')
        if declared.get('outer_v') != 1 or _port_count(ports['outer_v']) != 1:
            raise CornerTemplateError('outer_v must stay one coarse edge')
    return {
        'ok': True,
        'template_id': template_id,
        'area': 0.5 * total_area2,
        'quad_count': len(quads),
        'port_counts': dict(patch.get('port_counts') or {}),
        'participating': bool(patch.get('participating')),
    }


def build_four_corner_patches_2to1(frame, port_counts):
    """Build four legacy single-quad corners (compatibility helper)."""
    for key in ('hxi', 'hyi', 'H', 'V'):
        if key not in frame:
            raise CornerTemplateError('frame missing %s' % key)
    hxi, hyi, H, V = [float(frame[key])
                      for key in ('hxi', 'hyi', 'H', 'V')]
    if not (H > hxi and V > hyi):
        raise CornerTemplateError('outer frame must exceed inner frame')
    shared = all(key in port_counts for key in
                 ('inner_u', 'outer_u', 'inner_v', 'outer_v'))

    def counts_for(name):
        if shared:
            return port_counts
        if name not in port_counts:
            raise CornerTemplateError('missing port counts for %s' % name)
        return port_counts[name]

    boxes = {
        'SW': (-H, -hxi, -V, -hyi),
        'SE': (hxi, H, -V, -hyi),
        'NE': (hxi, H, hyi, V),
        'NW': (-H, -hxi, hyi, V),
    }
    patches = {}
    for name in ('SW', 'SE', 'NE', 'NW'):
        patch = build_corner_patch_2to1(
            boxes[name], counts_for(name), label=name)
        validate_corner_patch(patch)
        patches[name] = patch
    # created by IY,China University of Petroleum (East China)
    return patches


def build_four_participating_corners_2to1(hxi, hyi, H, V):
    """Each patch is built in its own local frame then translated, so sketch drawing can use global (u,"""
    hxi = float(hxi)
    hyi = float(hyi)
    H = float(H)
    V = float(V)
    if not (H > hxi and V > hyi):
        raise CornerTemplateError('outer frame must exceed inner frame')
    dx = H - hxi
    dy = V - hyi
    placements = {
        'SW': (-1.0, -1.0, -hxi, -hyi),
        'SE': (+1.0, -1.0, +hxi, -hyi),
        'NE': (+1.0, +1.0, +hxi, +hyi),
        'NW': (-1.0, +1.0, -hxi, +hyi),
    }
    patches = {}
    for name, (su, sv, ou, ov) in placements.items():
        patch = build_participating_corner_2to1(dx, dy, label=name)
        mapped = {}
        for node, (lu, lv) in patch['vertices'].items():
            mapped[node] = (ou + su * lu, ov + sv * lv)
        patch = dict(patch)
        patch['vertices'] = mapped
        if su * sv < 0:
            # Mirror flips orientation; reverse each quad to stay CCW.
            patch['quads'] = [tuple(reversed(q)) for q in patch['quads']]
            flipped_ports = {}
            for key, edges in patch['ports'].items():
                flipped_ports[key] = [(b, a) for (a, b) in edges]
            patch['ports'] = flipped_ports
        validate_corner_patch(patch)
        patches[name] = patch
    return patches


def _rect_quad_audit(vertices, quads, boundary, label):
    """核验逐角凸性、共边方向、边关联数及单元连通性，返回边表和面积。"""
    owners, directed, areas = {}, set(), []
    for face, quad in enumerate(quads):
        if len(quad) != 4 or len(set(quad)) != 4:
            raise CornerTemplateError('%s: a quad must have four distinct nodes' % label)
        points = [vertices[i] for i in quad]
        if any(not (_cross(points[i], points[(i + 1) % 4],
                           points[(i + 2) % 4]) > 0.0) for i in range(4)):
            raise CornerTemplateError('%s: element is not strictly convex or not CCW' % label)
        areas.append((_cross(points[0], points[1], points[2])
                      + _cross(points[0], points[2], points[3])) * 0.5)
        for a, b in zip(quad, quad[1:] + quad[:1]):
            if (a, b) in directed:
                raise CornerTemplateError('%s: overlapping elements or same-direction shared edge' % label)
            directed.add((a, b))
            owners.setdefault(tuple(sorted((a, b))), []).append(face)
    if not boundary.issubset(owners) or any(
            len(faces) != (1 if edge in boundary else 2)
            for edge, faces in owners.items()):
        raise CornerTemplateError('%s: edge incidence mismatch, interface missing edges or dangling edges' % label)
    neighbours = [set() for _ in quads]
    for faces in owners.values():
        if len(faces) == 2:
            a, b = faces
            neighbours[a].add(b)
            neighbours[b].add(a)
    visited, pending = set(), [0] if quads else []
    # created by IY,China University of Petroleum (East China)  |  禁止商用 / non-commercial use only
    while pending:
        face = pending.pop()
        if face not in visited:
            visited.add(face)
            pending.extend(neighbours[face] - visited)
    if not quads or len(visited) != len(quads):
        raise CornerTemplateError('%s: template elements are disconnected' % label)
    return owners, math.fsum(areas)


def _rect_unit_patch_2to1(patch):
    """只接受单位深度、四细两粗、端边不分段的完整六单元模板。"""
    try:
        for key, expected in (('ratio', 2), ('h', 1), ('depth', 1),
                              ('fine_segments', 4), ('coarse_segments', 2),
                              ('faces', 6)):
            if patch.get(key, expected) != expected:
                raise CornerTemplateError('Template parameter %s must be %s' % (key, expected))
        raw = patch['vertices']
        points = dict((key, tuple(float(x) for x in value))
                      for key, value in raw.items())
        if len(points) != 11 or any(
                len(p) != 2 or not (0.0 <= p[0] <= 4.0 and 0.0 <= p[1] <= 1.0)
                for p in points.values()):
            raise CornerTemplateError('Template needs 11 nodes with finite coordinates inside the unit template')
        keys = sorted(points, key=lambda key: points[key])
        indices = dict((key, i) for i, key in enumerate(keys))
        vertices = [points[key] for key in keys]
        lookup = dict((p, i) for i, p in enumerate(vertices))
        quads = [tuple(indices[key] for key in quad) for quad in patch['quads']]
        if len(lookup) != 11 or len(quads) != 6 or set(
                i for quad in quads for i in quad) != set(range(11)):
            raise CornerTemplateError('Template nodes duplicated/unused or the 6-element connectivity is invalid')
        loop = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (4, 1), (2, 1), (0, 1)]
        boundary = set(tuple(sorted((lookup[a], lookup[b])))
                       for a, b in zip(loop, loop[1:] + loop[:1]))
    except (KeyError, TypeError, AttributeError, ValueError, OverflowError) as exc:
        raise CornerTemplateError('Malformed 2:1 template: %s' % exc)
    _, area = _rect_quad_audit(vertices, quads, boundary, 'unit template')
    if abs(area - 4.0) > 4.0e-12:
        raise CornerTemplateError('Template does not fully cover the unit region')
    return vertices, quads


def build_rect_multistage_2to1(frames, patch):
    """每级提供 hxi、hyi、H、V 半宽及沿框轴的 Nx、Ny 细分段数；patch 为 build_transition_patch(2) 的单位模板，不允许自由网格兜底。"""
    template_vertices, template_quads = _rect_unit_patch_2to1(patch)
    clean = []
    try:
        for frame in frames:
            values = []
            for key in ('hxi', 'hyi', 'H', 'V', 'Nx', 'Ny'):
                value = float(frame[key])
                if isinstance(frame[key], bool) or not (0.0 < value < float('inf')):
                    raise CornerTemplateError('%s must be a finite positive value' % key)
                values.append(value)
            a, b, h, v, nx, ny = values
            if nx % 4 or ny % 4:
                raise CornerTemplateError('Nx/Ny must be positive integers divisible by 4')
            if clean:
                previous = clean[-1]
                if any(abs(x - y) > 1.0e-12 * max(x, y)
                       for x, y in zip((a, b), previous[2:4])):
                    raise CornerTemplateError('frames geometry is discontinuous')
                if (nx * 2, ny * 2) != previous[4:6]:
                    raise CornerTemplateError('Adjacent stages must each halve Nx/Ny; do not swap the frame axes')
                a, b = previous[2:4]
            if not (h > a and v > b):
                raise CornerTemplateError('Both outer-frame half-widths must be strictly greater than the inner frame')
            clean.append((a, b, h, v, int(nx), int(ny)))
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise CornerTemplateError('Invalid frames: %s' % exc)
    if not clean:
        raise CornerTemplateError('frames must not be empty')

    # 分轴相对量化，不使用绝对容差下限，避免小尺度或高宽比造成误归并。
    sx, sy = clean[-1][2:4]
    vertices, normalized, node_ids = [], [], {}

    def node(point):
        uv = (point[0] / sx, point[1] / sy)
        key = tuple(int(round(x * 1.0e12)) for x in uv)
        if key not in node_ids:
            node_ids[key] = len(vertices)
            vertices.append(point)
            normalized.append(uv)
        return node_ids[key]

    # 镜像只改变绕序；轴计数始终保持 top/bottom=Nx、left/right=Ny。
    sides = (('top', 0, 1, False), ('bottom', 0, -1, True),
             ('left', 1, -1, False), ('right', 1, 1, True))

    def side_point(axis, sign, s, hx, hy):
        return ((2.0 * s - 1.0) * hx, sign * hy) if axis == 0 else (
            sign * hx, (2.0 * s - 1.0) * hy)

    interfaces, interface_edges = [], []
    rings = [clean[0][:2] + clean[0][4:6]]
    rings.extend((h, v, nx // 2, ny // 2) for a, b, h, v, nx, ny in clean)
    # 先登记均匀接口，让映射产生的端点统一引用这些精确坐标。
    for hx, hy, nx, ny in rings:
        paths, boundary = {}, set()
        for name, axis, sign, reverse in sides:
            count = (nx, ny)[axis]
            path = [node(side_point(axis, sign, float(j) / count, hx, hy))
                    for j in range(count + 1)]
            if len(set(path)) != count + 1:
                raise CornerTemplateError('Interface nodes are below the relative quantization resolution')
            paths[name] = path
            boundary.update(tuple(sorted(edge)) for edge in zip(path, path[1:]))
        interfaces.append({'hx': hx, 'hy': hy, 'nx': nx, 'ny': ny, 'sides': paths})
        interface_edges.append(boundary)

    quads, quad_stages, expected_per_stage = [], [], []
    for stage, (a, b, h, v, nx, ny) in enumerate(clean):
        start = len(quads)
        for name, axis, sign, reverse in sides:
            count = (nx, ny)[axis]
            for k in range(count // 4):
                mapped = [node(side_point(axis, sign, (4.0 * k + u) / count,
                                          (1.0 - t) * a + t * h,
                                          (1.0 - t) * b + t * v))
                          for u, t in template_vertices]
                for quad in template_quads:
                    ids = tuple(mapped[i] for i in quad)
                    quads.append(tuple(reversed(ids)) if reverse else ids)
                    quad_stages.append(stage)
        expected_per_stage.append(3 * (nx + ny))
        if len(quads) - start != expected_per_stage[-1]:
            raise CornerTemplateError('Six-element template count mismatch per stage')
        _, area = _rect_quad_audit(
            normalized, quads[start:], interface_edges[stage] | interface_edges[stage + 1],
            'stage %d' % stage)
        # 差分形式等价于 4*(H*V-hxi*hyi)，避免薄环的大数相减。
        expected_area = 4.0 * (((h - a) / sx) * (v / sy) + (a / sx) * ((v - b) / sy))
        if not (expected_area > 0.0) or abs(area - expected_area) > expected_area * 1.0e-10:
            raise CornerTemplateError('Stage %d area does not fully cover the rectangular ring' % stage)

    owners, area = _rect_quad_audit(
        normalized, quads, interface_edges[0] | interface_edges[-1], 'full rectangular ring')
    for stage in range(1, len(interface_edges) - 1):
        for edge in interface_edges[stage]:
            if sorted(quad_stages[i] for i in owners[edge]) != [stage - 1, stage]:
                raise CornerTemplateError('Inter-layer interfaces must each be shared once by two adjacent stages')
    area = (area * sx) * sy
    if not (0.0 < area < float('inf')):
        raise CornerTemplateError('Rectangular ring area exceeds the float-representable range')
    return {'vertices': vertices, 'quads': quads, 'quad_stages': quad_stages,
            'interfaces': interfaces, 'edges': sorted(owners),
            'expected_per_stage': expected_per_stage, 'area': area}


_WEDGE_C_CACHE = {}


def _wedge_quad_angles(quads, pts):
    """按给定顶点顺序计算各四边形全部内角（度）。"""
    out = []
    for q in quads:
        v = [pts[n] for n in q]
        m = len(v)
        for i in range(m):
            ax, ay = v[i - 1]
            bx, by = v[i]
            cx, cy = v[(i + 1) % m]
            ux, uy = ax - bx, ay - by
            vx, vy = cx - bx, cy - by
            den = math.hypot(ux, uy) * math.hypot(vx, vy)
            if den <= 0.0:
                return None
            cosv = (ux * vx + uy * vy) / den
            out.append(math.degrees(math.acos(max(-1.0, min(1.0, cosv)))))
    return out


def _wedge_convex(quads, pts):
    """四个角全部左转（凸且顶点序一致）。"""
    for q in quads:
        v = [pts[n] for n in q]
        m = len(v)
        for i in range(m):
            x0, y0 = v[i]
            x1, y1 = v[(i + 1) % m]
            x2, y2 = v[(i + 2) % m]
            if (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1) <= 1.0e-12:
                return False
    return True


def _wedge_min_edge(quads, pts):
    """全部模板边的最短长度。"""
    seen, best = set(), None
    for q in quads:
        m = len(q)
        for i in range(m):
            e = tuple(sorted((q[i], q[(i + 1) % m])))
            if e in seen:
                continue
            seen.add(e)
            ax, ay = pts[e[0]]
            bx, by = pts[e[1]]
            d = math.hypot(bx - ax, by - ay)
            if best is None or d < best:
                best = d
    return best


def _balanced_wedge_c(dx, dy):
    """三个四边形凸、边长不下陷的前提下，最大化全部内角相对结构化划网可行窗口 [32°,148°] 的最小裕度（窗口由 Abaqus 实测标定：32/148 可划，30/150 拒绝）。"""
    scale = max(dx, dy)
    ndx, ndy = dx / scale, dy / scale
    key = (round(ndx, 9), round(ndy, 9))
    hit = _WEDGE_C_CACHE.get(key)
    if hit is not None:
        return hit
    if abs(ndx - ndy) <= 1.0e-9:
        _WEDGE_C_CACHE[key] = (0.5, 0.5)
        return (0.5, 0.5)
    base = {
        'I': (0.0, 0.0), 'B1': (ndx / 2.0, 0.0), 'B2': (ndx, 0.0),
        'L1': (0.0, ndy / 2.0), 'L2': (0.0, ndy), 'O': (ndx, ndy),
    }
    quads = [('I', 'B1', 'C', 'L1'), ('B1', 'B2', 'O', 'C'),
             ('L1', 'C', 'O', 'L2')]
    edge_floor = 0.05
    # 可行窗口与逐级放宽序列（全部实测标定；放宽只影响极端长宽比）
    windows = ((33.0, 147.0), (32.0, 148.0), (31.0, 149.0),
               (30.0, 150.0), (25.0, 155.0))
    best, best_score = None, None
    steps = 140
    for w_lo, w_hi in windows:
        for ia in range(1, steps):
            a = ndx * ia / steps
            for ib in range(1, steps):
                b = ndy * ib / steps
                if min(a, b) < edge_floor:
                    continue
                pts = dict(base)
                pts['C'] = (ndx - a, ndy - b)
                if not _wedge_convex(quads, pts):
                    continue
                if _wedge_min_edge(quads, pts) < edge_floor:
                    continue
                angs = _wedge_quad_angles(quads, pts)
                if angs is None:
                    continue
                if min(angs) < w_lo or max(angs) > w_hi:
                    continue
                score = min(min(angs) - w_lo, w_hi - max(angs))
                if best_score is None or score > best_score:
                    best_score, best = score, (a, b)
        if best is not None:
            break
    if best is not None:
        result = best
    else:
        # 极端长宽比无窗口解：钳制在块内，保持几何合法（后续划网审计
        # 会如实反映质量）。
        result = (min(0.5, ndx - 0.05), min(0.5, ndy - 0.05))
    _WEDGE_C_CACHE[key] = result
    return result


def build_corner_2to1_wedge(ix, iy, dx, dy):
    """局部以 u 沿 x、v 沿 y。内小方占 (0..dx/2, 0.."""
    dx = float(dx)
    dy = float(dy)
    if not (dx > 0.0 and dy > 0.0):
        raise CornerTemplateError('2:1 corner needs positive dx/dy')
    hx, hy = dx / 2.0, dy / 2.0
    scale = max(dx, dy)
    ca, cb = _balanced_wedge_c(dx, dy)
    cx, cy = dx - ca * scale, dy - cb * scale
    V = {
        'I': (ix, iy), 'B1': (ix + hx, iy), 'B2': (ix + dx, iy),
        'L1': (ix, iy + hy), 'L2': (ix, iy + dy),
        'C': (ix + cx, iy + cy), 'O': (ix + dx, iy + dy),
    }
    quads = [
        ('I', 'B1', 'C', 'L1'),          # 内小方
        ('B1', 'B2', 'O', 'C'),          # 右下梯形
        ('L1', 'C', 'O', 'L2'),          # 左上梯形
    ]
    ports = {
        'inner_u': [('I', 'L1'), ('L1', 'L2')],   # 左竖边 x=ix，2 细段
        'inner_v': [('I', 'B1'), ('B1', 'B2')],   # 下横边 y=iy，2 细段
        'outer_u': [('L2', 'O')],                  # 顶边 y=iy+dy，1 粗段
        'outer_v': [('B2', 'O')],                  # 右边 x=ix+dx，1 粗段
    }
    # 逐角凸性校验（CCW 正面积）。
    def _ccw(q):
        pts = [V[n] for n in q]
        s = 0.0
        for i in range(len(pts)):
            x0, y0 = pts[i]
            x1, y1 = pts[(i + 1) % len(pts)]
            s += x0 * y1 - x1 * y0
        return s
    for q in quads:
        if _ccw(q) <= 0.0:
            raise CornerTemplateError('2:1 corner quad %r not CCW/convex' % (q,))
    return {
        'template_id': 'RECT_CORNER_2TO1_WEDGE',
        'vertices': V, 'quads': quads, 'ports': ports,
        'inner_edge_segs': {'inner_u': 2, 'inner_v': 2},
        'outer_edge_segs': {'outer_u': 1, 'outer_v': 1},
        'area': dx * dy,  # 3 个四边形恰好铺满角方块
    }


def corner_wedge_draw_lines(patch):
    """角块内部需绘的线（不含与边条/外框共享的边界）。"""
    V = patch['vertices']
    shared = set()
    for name in ('inner_u', 'inner_v', 'outer_u', 'outer_v'):
        for a, b in patch['ports'].get(name, []):
            shared.add(_edge_key(a, b))
    seen = set(shared)
    lines = []
    for quad in patch['quads']:
        for i, name in enumerate(quad):
            other = quad[(i + 1) % 4]
            key = _edge_key(name, other)
            if key in seen:
                continue
            seen.add(key)
            lines.append((V[name], V[other]))
    return lines
