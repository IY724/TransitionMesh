# -*- coding: utf-8 -*-
# ============================================================================
#  Transition Mesh
#  created by IY,China University of Petroleum (East China)
#  禁止商用 / Non-commercial use only
#  禁止商用：本软件仅供学习与研究使用，未经作者书面许可不得用于任何商业用途。
#  Non-commercial use only. Unauthorized commercial use is prohibited.
# ============================================================================

"""Abaqus transition mesh helper plugin. This is the rewritten Transition Mesh implementation."""

from __future__ import print_function

import math
from transition_mesh_plan import plan_counts, allocate_widths

# GUI 运行时保持静默：信息性 print 默认不输出到 CAE 底部信息区，仅
# TM_VERBOSE=1 （回归脚本会设置）时回显；异常/返回值/模型不受影响。
import os as _tm_os


def _tm_quiet_print(*args, **kwargs):
    return 0


if not _tm_os.environ.get('TM_VERBOSE'):
    print = _tm_quiet_print  # 有意遮蔽模块内建 print

try:
    from abaqus import mdb, session
    import mesh as abaqus_mesh
    from abaqusConstants import (
        SIDE1, ON, OFF, FIXED, XYPLANE, XZPLANE, YZPLANE, RIGHT, COPLANAR_EDGES,
        HEX, SWEEP, STRUCTURED, ADVANCING_FRONT, MEDIAL_AXIS, C3D8R, STANDARD,
        TECHNIQUE, NUMBER, CONSTRAINT
    )
except Exception:
    mdb = None
    session = None
    abaqus_mesh = None
    SIDE1 = None
    FIXED = "FIXED"
    XYPLANE = "XYPLANE"
    XZPLANE = "XZPLANE"
    YZPLANE = "YZPLANE"
    RIGHT = "RIGHT"
    COPLANAR_EDGES = "COPLANAR_EDGES"
    ON = True
    OFF = False
    HEX = "HEX"
    SWEEP = "SWEEP"
    STRUCTURED = "STRUCTURED"
    ADVANCING_FRONT = "ADVANCING_FRONT"
    MEDIAL_AXIS = "MEDIAL_AXIS"
    C3D8R = "C3D8R"
    STANDARD = "STANDARD"
    TECHNIQUE = "TECHNIQUE"
    NUMBER = "NUMBER"
    CONSTRAINT = "CONSTRAINT"


EPS = 1.0e-6


# Vector helpers

def _as_tuple(value):
    if hasattr(value, "pointOn"):
        return tuple(value.pointOn[0])
    return tuple(value)


def _vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vmul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _vdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _vcross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _vnorm(a):
    return math.sqrt(_vdot(a, a))


def _vunit(a):
    length = _vnorm(a)
    if length < EPS:
        raise ValueError("Zero-length vector cannot define a transition frame.")
    return (a[0] / length, a[1] / length, a[2] / length)


def _single(obj):
    try:
        if len(obj) == 1:
            return obj[0]
    except Exception:
        pass
    return obj


def _flatten_geom(items):
    """Flatten Abaqus GeomSequence-like containers into plain items."""
    if items is None:
        return []
    try:
        iterator = list(items)
    except TypeError:
        iterator = [items]
    out = []
    for item in iterator:
        try:
            if len(item) > 0 and not hasattr(item, "pointOn"):
                out.extend([item[i] for i in range(len(item))])
            else:
                out.append(item)
        except Exception:
            out.append(item)
    return out


# Geometry extraction helpers

def _edge_points(edge, part=None):
    """Best-effort endpoint extraction for Abaqus Edge objects."""
    if part is not None:
        try:
            verts = edge.getVertices()
            if len(verts) >= 2:
                v0 = part.vertices[verts[0]]
                v1 = part.vertices[verts[1]]
                return tuple(v0.pointOn[0]), tuple(v1.pointOn[0])
        except Exception:
            pass
    try:
        verts = edge.getVertices()
        if len(verts) >= 2:
            v0 = edge.instance.vertices[verts[0]]
            v1 = edge.instance.vertices[verts[1]]
            return tuple(v0.pointOn[0]), tuple(v1.pointOn[0])
    except Exception:
        pass
    try:
        verts = edge.getVertices()
        part = edge.getParent()
        v0 = part.vertices[verts[0]]
        v1 = part.vertices[verts[1]]
        return tuple(v0.pointOn[0]), tuple(v1.pointOn[0])
    except Exception:
        pass
    try:
        pts = edge.pointOn
        if len(pts) >= 2:
            return tuple(pts[0]), tuple(pts[-1])
    except Exception:
        pass
    raise ValueError("Cannot determine endpoints for the selected finer edge.")


def _edge_midpoint(edge, part=None):
    p0, p1 = _edge_points(edge, part=part)
    return (
        0.5 * (p0[0] + p1[0]),
        0.5 * (p0[1] + p1[1]),
        0.5 * (p0[2] + p1[2]),
    )


def _face_point(face):
    try:
        point = tuple(face.getCentroid())
        if len(point) >= 3:
            return point
    except Exception:
        pass
    try:
        return tuple(face.pointOn[0])
    except Exception:
        pass
    raise ValueError("Cannot determine a point on the selected face.")


def _face_normal(face):
    point = _face_point(face)
    try:
        return _vunit(tuple(face.getNormal(point=point)))
    except Exception:
        pass
    try:
        return _vunit(tuple(face.getNormal()))
    except Exception:
        pass
    raise ValueError("Cannot determine face normal. Curved faces need cylinder mode.")


# Transition count / ratio helpers

def transition_counts(fine_seeds=None, ratio=2, layers=1):
    """coarse = fine / (ratio ** layers). Must be an exact integer division."""
    if fine_seeds is None:
        raise ValueError("fine_seeds is required.")
    fine = int(fine_seeds)
    if fine <= 0:
        raise ValueError("fine_seeds must be positive.")
    ratio = int(ratio)
    if ratio not in (2, 3):
        raise ValueError("ratio must be 2 or 3.")
    layers = int(layers)
    if layers < 0:
        raise ValueError("layers must be non-negative.")

    denom = int(pow(ratio, layers))
    if denom == 0:
        raise ValueError("ratio ** layers is zero; check parameters.")
    if fine % denom != 0:
        raise ValueError(
            "fine_seeds (%d) is not divisible by ratio**layers (%d^%d = %d). "
            "Adjust Seeds Number or Layers so the division is exact."
            % (fine, ratio, layers, denom)
        )
    if ratio == 2 and layers > 0:
        template_divisor = 4 * int(pow(2, layers - 1))
        if fine % template_divisor != 0:
            raise ValueError(
                "2:1 uses a 4-to-2 template. fine_seeds (%d) must be "
                "divisible by %d for %d layer(s)."
                % (fine, template_divisor, layers)
            )
    coarse = fine // denom
    if coarse <= 0:
        raise ValueError("Computed coarse count is zero; reduce layers or ratio.")
    return coarse, fine, ratio


# Explicit single-layer transition templates (pure geometry, no Abaqus objects).
# Local frame: u along the fine edge, v from fine (0) to coarse (1), spacing h.

def build_transition_patch(ratio, fine_spacing=1.0, depth=1.0):
    """2:1 (N%4==0): 6 quads per 4 fine segments. 3:1 (N%3==0): 4 quads per 3 fine segments."""
    ratio = int(ratio)
    h = float(fine_spacing)
    d = float(depth)
    if ratio == 2:
        # A0..A4 on fine; B1..B3 mid; C0,C2,C4 coarse
        V = {
            'A0': (0 * h, 0.0), 'A1': (1 * h, 0.0), 'A2': (2 * h, 0.0),
            'A3': (3 * h, 0.0), 'A4': (4 * h, 0.0),
            'B1': (1 * h, 0.5), 'B2': (2 * h, 0.5), 'B3': (3 * h, 0.5),
            'C0': (0 * h, 1.0), 'C2': (2 * h, 1.0), 'C4': (4 * h, 1.0),
        }
        Q = [
            ('A0', 'A1', 'B1', 'C0'),
            ('A1', 'A2', 'B2', 'B1'),
            ('A2', 'A3', 'B3', 'B2'),
            ('A3', 'A4', 'C4', 'B3'),
            ('C0', 'B1', 'B2', 'C2'),
            ('B2', 'B3', 'C4', 'C2'),
        ]
        fine_segs = 4
        coarse_segs = 2
    elif ratio == 3:
        V = {
            'A0': (0 * h, 0.0), 'A1': (1 * h, 0.0), 'A2': (2 * h, 0.0),
            'A3': (3 * h, 0.0),
            'B1': (1 * h, 0.5), 'B2': (2 * h, 0.5),
            'C0': (0 * h, 1.0), 'C3': (3 * h, 1.0),
        }
        Q = [
            ('A0', 'A1', 'B1', 'C0'),
            ('A1', 'A2', 'B2', 'B1'),
            ('A2', 'A3', 'C3', 'B2'),
            ('C0', 'B1', 'B2', 'C3'),
        ]
        fine_segs = 3
        coarse_segs = 1
    else:
        raise ValueError('ratio must be 2 or 3, got %r' % ratio)
    # scale v by depth so callers can use physical depth if needed
    V = dict((k, (uv[0], uv[1] * d)) for k, uv in V.items())
    return {
        'vertices': V, 'quads': Q, 'ratio': ratio,
        'fine_segments': fine_segs, 'coarse_segments': coarse_segs,
        'faces': len(Q),
        'h': h, 'depth': d,
    }


def patch_unique_edges(patch):
    """Unique undirected edges of a patch (vertex-id pairs)."""
    seen = set()
    edges = []
    for q in patch['quads']:
        n = len(q)
        for i in range(n):
            a, b = q[i], q[(i + 1) % n]
            key = tuple(sorted((a, b)))
            if key not in seen:
                seen.add(key)
                edges.append((a, b))
    return edges


def expected_transition_elements(ratio, n_fine):
    """Template face count for a full-circle or full-side fine count N."""
    ratio = int(ratio)
    n_fine = int(n_fine)
    if ratio == 2:
        if n_fine % 4 != 0:
            raise ValueError('2:1 requires N %% 4 == 0, got %d' % n_fine)
        return (n_fine // 4) * 6
    if n_fine % 3 != 0:
        raise ValueError('3:1 requires N %% 3 == 0, got %d' % n_fine)
    return (n_fine // 3) * 4


def hybrid_counts(fine_seeds, coarse_seeds):
    """给定细边 N 和粗边 M 的种子数，计算：a = N - 2M：3进1 组数（每组 3 细 → 1 粗）b = 3M - N：2进1 组数（每组 2 细 → 1 粗）约束：2M ≤ N ≤ 3M，"""
    N = int(fine_seeds)
    M = int(coarse_seeds)
    if N <= 0 or M <= 0:
        raise ValueError("fine_seeds and coarse_seeds must be positive")
    if N == 3 * M:
        return (M, 0, '3:1')
    if N == 2 * M:
        return (0, M, '2:1')
    if N < 2 * M:
        raise ValueError(
            "Hybrid constraint violated: N=%d < 2M=%d (too few fine seeds to "
            "fill with a 2:1+3:1 combination)" % (N, 2 * M))
    if N > 3 * M:
        raise ValueError(
            "Hybrid constraint violated: N=%d > 3M=%d (too many fine seeds, "
            "beyond the 2:1+3:1 combination range)" % (N, 3 * M))
    a = N - 2 * M   # 3:1 组数
    b = 3 * M - N   # 2:1 组数
    if a + b != M:
        raise ValueError("Internal error: a+b=%d != M=%d" % (a + b, M))
    return (a, b, 'HYBRID(%dx3:1 + %dx2:1)' % (a, b))


def _legacy_transition_counts(m=None, n=None):
    """Returns (coarse, fine, ratio) using the old m * 2**n formula."""
    if m is None or n is None:
        raise ValueError("Both m and n are required for legacy transition counts.")
    m = int(m)
    n = int(n)
    if m <= 0 or n < 0:
        raise ValueError("m must be positive and n must be non-negative.")
    coarse = m
    fine = m * int(pow(2, n))
    return coarse, fine, 2


# Frame / projection helpers

def build_face_frame(face, finer_edge=None, part=None):
    """Return origin, u_dir, v_dir, normal for planar face local coordinates."""
    origin = _face_point(face)
    normal = _face_normal(face)

    if finer_edge is not None:
        p0, p1 = _edge_points(finer_edge, part=part)
        u_dir = _vunit(_vsub(p1, p0))
    else:
        # Pick a stable axis not parallel to the normal.
        axis = (1.0, 0.0, 0.0)
        if abs(_vdot(axis, normal)) > 0.9:
            axis = (0.0, 1.0, 0.0)
        u_dir = _vunit(_vcross(normal, axis))

    v_dir = _vunit(_vcross(normal, u_dir))
    return origin, u_dir, v_dir, normal


def project_xyz_to_uv(point, origin, u_dir, v_dir):
    """Convert 3D point on selected face to local 2D coordinates."""
    rel = _vsub(tuple(point), tuple(origin))
    return (_vdot(rel, u_dir), _vdot(rel, v_dir))


def _orientation_vector(orientation):
    """Map GUI orientation to a global model-space direction."""
    orientation = str(orientation).upper()
    mapping = {
        "HU": (0.0, -1.0, 0.0),
        "HD": (0.0, 1.0, 0.0),
        "VR": (-1.0, 0.0, 0.0),
        "VL": (1.0, 0.0, 0.0),
    }
    if orientation not in mapping:
        raise ValueError("PLANAR orientation must be HU, HD, VR, or VL.")
    return mapping[orientation]


def _face_sample_points(face, part=None):
    """Return representative vertex/edge points for a face."""
    points = []
    if part is not None:
        try:
            for vid in face.getVertices():
                points.append(tuple(part.vertices[vid].pointOn[0]))
        except Exception:
            pass
        try:
            for eid in face.getEdges():
                p0, p1 = _edge_points(part.edges[eid], part=part)
                points.append(p0)
                points.append(p1)
        except Exception:
            pass
    try:
        points.append(tuple(face.pointOn[0]))
    except Exception:
        pass
    return points


def _face_local_w_range(face, part, frame_origin, v_dir):
    points = _face_sample_points(face, part=part)
    if not points:
        return None
    values = [_vdot(_vsub(point, frame_origin), v_dir) for point in points]
    return (min(values), max(values))


def _regular_extension_from_faces(faces, part, frame_origin, v_dir, side, depth):
    far = float(depth)
    for item in faces:
        for point in _face_sample_points(item, part=part):
            far = max(far, -float(side) * _vdot(
                _vsub(point, frame_origin), v_dir))
    ext = far - float(depth)
    if ext <= EPS:
        return 0.0
    return ext + 1.0e-3 * max(abs(far), abs(float(depth)), 1.0)


def _face_regular_extension(face, part, frame_origin, v_dir, side, depth):
    return _regular_extension_from_faces(
        [face], part, frame_origin, v_dir, side, depth)


def _extension_candidate_faces(face, regular_faces, part, frame_origin,
                               u_dir, v_dir, normal, side, depth, length,
                               tol=1.0e-4):
    """Full-domain faces coplanar with the transition face and beyond the coarse edge."""
    if not regular_faces:
        return []
    plane_point = _face_point(face)
    half = 0.5 * float(length)
    span_tol = max(float(tol), 1.0e-4 * max(1.0, float(length)))
    out = []
    seen = set()
    for candidate in _flatten_geom(regular_faces):
        item = _single(candidate)
        try:
            idx = int(item.index)
        except Exception:
            continue
        if idx in seen or idx == getattr(face, 'index', None):
            continue
        seen.add(idx)
        try:
            candidate_normal = _face_normal(item)
        except Exception:
            continue
        if abs(_vdot(candidate_normal, normal)) < 0.999:
            continue
        points = _face_sample_points(item, part=part)
        if not points:
            continue
        if max(abs(_vdot(_vsub(point, plane_point), normal))
               for point in points) > span_tol:
            continue
        local = []
        for point in points:
            rel = _vsub(point, frame_origin)
            local.append((_vdot(rel, u_dir),
                          -float(side) * _vdot(rel, v_dir)))
        if max(w for _, w in local) <= float(depth) + span_tol:
            continue
        if (max(u for u, _ in local) < -half - span_tol
                or min(u for u, _ in local) > half + span_tol):
            continue
        out.append(item)
    return out


def _resolve_planar_side(face, edge, part, orientation, depth, normal, u_dir, v_dir):
    """With MakeSketchTransform(..."""
    desired = _orientation_vector(orientation)
    if abs(_vdot(desired, normal)) > 0.25:
        raise ValueError(
            "Orientation %s is not in the selected face plane." % orientation
        )

    lateral = _vdot(desired, v_dir)
    along = abs(_vdot(desired, u_dir))
    if abs(lateral) < 0.25 or along > 0.75:
        raise ValueError(
            "Orientation %s is nearly parallel to the selected finer edge; "
            "pick the orientation perpendicular to that edge." % orientation
        )

    side = -1.0 if lateral > 0.0 else 1.0
    p0, p1 = _edge_points(edge, part=part)
    frame_origin = (
        0.5 * (p0[0] + p1[0]),
        0.5 * (p0[1] + p1[1]),
        0.5 * (p0[2] + p1[2]),
    )
    target_w = -side * float(depth)
    w_range = _face_local_w_range(face, part, frame_origin, v_dir)
    if w_range is not None:
        w_min, w_max = w_range
        if target_w < w_min - 1.0e-5 or target_w > w_max + 1.0e-5:
            raise ValueError(
                "Orientation %s points outside the selected face or Depth %.6g "
                "exceeds the available transition width on that side. "
                "Available local width range is [%.6g, %.6g]."
                % (orientation, depth, w_min, w_max)
            )
    return side


def resolve_depth_from_coarse_edge(part, face, fine_edge, coarse_edge,
                                   v_dir, u_dir, normal, tol=1.0e-5):
    """The coarse edge replaces the typed Depth:"""
    f0, f1 = _edge_points(fine_edge, part=part)
    origin = (
        0.5 * (f0[0] + f1[0]),
        0.5 * (f0[1] + f1[1]),
        0.5 * (f0[2] + f1[2]),
    )
    c0, c1 = _edge_points(coarse_edge, part=part)
    for point in (c0, c1):
        rel = _vsub(point, origin)
        if abs(_vdot(rel, normal)) > max(tol, 1.0e-4 * max(1.0, _vnorm(rel))):
            raise ValueError(
                "TM-COARSE-PLANE: the picked coarse edge does not lie in the "
                "selected partition face, so it cannot define a transition "
                "width. Pick the coarse edge on the same face as the finer "
                "edge.")

    dc = _vsub(c1, c0)
    len_c = _vnorm(dc)
    if len_c <= tol:
        raise ValueError("TM-COARSE-DEGENERATE: the picked coarse edge has zero length.")
    cross = _vcross(dc, u_dir)
    if _vnorm(cross) / len_c > 1.0e-4:
        raise ValueError(
            "TM-COARSE-PARALLEL: the picked coarse edge is not parallel to the "
            "finer edge. A straight transition band needs both boundary edges "
            "parallel.")

    w0 = _vdot(_vsub(c0, origin), v_dir)
    w1 = _vdot(_vsub(c1, origin), v_dir)
    if abs(w0 - w1) > max(1.0e-4, 1.0e-4 * max(abs(w0), abs(w1))):
        raise ValueError(
            "TM-COARSE-PARALLEL: the coarse edge is not at a constant distance "
            "from the finer edge (%.6g vs %.6g), so it cannot be the coarse "
            "boundary of a straight transition band." % (w0, w1))

    target_w = 0.5 * (w0 + w1)
    depth = abs(target_w)
    if depth <= tol:
        raise ValueError(
            "TM-COARSE-ZERO: the picked coarse edge coincides with the finer "
            "edge, so the transition width would be zero.")

    # target_w = -side * depth  ->  side = -sign(target_w)
    side = -1.0 if target_w > 0.0 else 1.0
    orientation = _orientation_from_side(side, u_dir, v_dir, normal)
    return depth, side, orientation


def _orientation_from_side(side, u_dir, v_dir, normal):
    """The GUI labels are global model directions,"""
    desired = _vmul(v_dir, -float(side))
    ax = abs(_vdot(desired, (1.0, 0.0, 0.0)))
    ay = abs(_vdot(desired, (0.0, 1.0, 0.0)))
    if ay >= ax:
        return "HU" if _vdot(desired, (0.0, -1.0, 0.0)) > 0.0 else "HD"
    return "VL" if _vdot(desired, (1.0, 0.0, 0.0)) > 0.0 else "VR"


# Core transition drawing

def _compute_layer_boundaries(edge_length, fine, ratio, layers):
    """Returns a list of lists, index 0 = finest (fine edge), index layers = coarsest."""
    boundaries = []
    half = 0.5 * float(edge_length)
    for level in range(layers + 1):
        seg_count = fine // int(pow(ratio, level))
        if seg_count < 1:
            seg_count = 1
        pts = [-half + i * float(edge_length) / float(seg_count)
               for i in range(seg_count + 1)]
        boundaries.append(pts)
    return boundaries


def _compute_depth_levels(depth, layers):
    """Returns layers+1 values from 0 (fine edge) to depth (coarse edge), evenly spaced."""
    if layers == 0:
        return [0.0, float(depth)]
    return [i * float(depth) / float(layers) for i in range(layers + 1)]


def _draw_transition_pattern(sketch, edge_length, depth, fine, ratio, layers, side,
                             ratio_sequence=None, stage_widths=None):
    """The fine edge is at depth=0 (sketch x=0), coarse edge is at depth=depth."""
    if int(layers) > 1 or ratio_sequence or stage_widths:
        plan = plan_counts(fine, layers, ratio, ratio_sequence)
        widths = allocate_widths(depth, plan['counts'], stage_widths)
        offset = 0.0
        seen = set()
        for count, rr, width in zip(plan['counts'], plan['ratios'], widths):
            h = float(edge_length) / count
            patch = build_transition_patch(rr, h, width)
            for ip in range(count // patch['fine_segments']):
                for a, b in patch_unique_edges(patch):
                    points = []
                    for key in (a, b):
                        u, v = patch['vertices'][key]
                        points.append((side * (offset + v),
                                       -0.5 * edge_length + u + ip * patch['fine_segments'] * h))
                    edge_key = tuple(sorted(tuple(round(x, 10) for x in p) for p in points))
                    if edge_key not in seen:
                        sketch.Line(point1=points[0], point2=points[1])
                        seen.add(edge_key)
            offset += width
        return
    # Spec §4: pure single-layer uses the shared template topology
    if int(layers) == 1 and int(ratio) in (2, 3):
        h = float(edge_length) / float(fine)
        patch = build_transition_patch(int(ratio), fine_spacing=h,
                                       depth=float(depth))
        n_period = int(fine) // patch['fine_segments']
        half0 = -0.5 * float(edge_length)  # first period origin (centered)

        def P(name, ip):
            u, v = patch['vertices'][name]
            uu = half0 + u + ip * patch['fine_segments'] * h
            ww = v  # v already scaled by depth in build_transition_patch
            return (side * ww, uu)

        for ip in range(n_period):
            for a, b in patch_unique_edges(patch):
                sketch.Line(point1=P(a, ip), point2=P(b, ip))
        return

    x_bnd = _compute_layer_boundaries(edge_length, fine, ratio, layers)
    y_lvl = _compute_depth_levels(depth, layers)

    def pt(u, w):
        return (side * w, u)

    def line(u_a, w_a, u_b, w_b):
        sketch.Line(point1=pt(u_a, w_a), point2=pt(u_b, w_b))

    if ratio == 2 and layers > 0:
        for level in range(layers):
            if (len(x_bnd[level]) - 1) % 4 != 0:
                raise ValueError(
                    "2:1 4-to-2 template requires each layer fine count "
                    "to be divisible by 4."
                )

    # Horizontal layer boundaries at each depth level.
    for level_idx in range(layers + 1):
        y = y_lvl[level_idx]
        u_min = x_bnd[0][0]
        u_max = x_bnd[0][-1]
        line(u_min, y, u_max, y)

    # Per-layer transition templates
    for level in range(layers):
        fine_bnd = x_bnd[level]
        coarse_bnd = x_bnd[level + 1]
        y_lo = y_lvl[level]
        y_hi = y_lvl[level + 1]
        y_mid = 0.5 * (y_lo + y_hi)

        if ratio == 2:
            # User-defined 2:1 transition: 4 fine segments become 2 coarse
            # segments.  The top split is at f2; the lower side keeps f1/f2/f3.
            for start in range(0, len(fine_bnd) - 1, 4):
                group = fine_bnd[start:start + 5]
                if len(group) != 5:
                    continue
                f0, f1, f2, f3, f4 = group
                line(f0, y_lo, f0, y_hi)
                line(f4, y_lo, f4, y_hi)
                line(f1, y_lo, f1, y_mid)
                line(f2, y_lo, f2, y_hi)
                line(f3, y_lo, f3, y_mid)
                line(f0, y_hi, f1, y_mid)
                line(f1, y_mid, f3, y_mid)
                line(f3, y_mid, f4, y_hi)

        elif ratio == 3:
            # User-defined 3:1 transition: 3 fine segments become 1 coarse
            # segment.
            for start in range(0, len(fine_bnd) - 1, 3):
                group = fine_bnd[start:start + 4]
                if len(group) != 4:
                    continue
                f0, f1, f2, f3 = group
                line(f0, y_lo, f0, y_hi)
                line(f3, y_lo, f3, y_hi)
                line(f1, y_lo, f1, y_mid)
                line(f2, y_lo, f2, y_mid)
                line(f0, y_hi, f1, y_mid)
                line(f1, y_mid, f2, y_mid)
                line(f2, y_mid, f3, y_hi)


def _draw_edge_anchored_pattern(sketch, edge_length, depth, fine, ratio, layers, side=1.0, inset=1.0e-4,
                                ratio_sequence=None, stage_widths=None):
    """Abaqus maps sketchUpEdge to the sketch Y direction for the usual RIGHT orientation."""
    # Use exact edge length - no artificial shrink
    effective_length = edge_length
    _draw_transition_pattern(
        sketch=sketch,
        edge_length=effective_length,
        depth=depth,
        fine=fine,
        ratio=ratio,
        layers=layers,
        side=side, ratio_sequence=ratio_sequence, stage_widths=stage_widths
    )


def _draw_hybrid_pattern(sketch, edge_length, depth, fine, coarse, side,
                         pattern='', regular_extension=0.0):
    """块只有两种，都直接来自 ``build_transition_patch``：``'3'`` : 3 细端口 → 1 粗端口，4 个 quad ``'22'`` : 4 细端口 → 2 粗端口，"""
    import transition_mesh_rules as tm_rules
    split = tm_rules.hybrid_split(fine, coarse, sequence=pattern)
    if not split['ok']:
        raise ValueError(split['error'])

    N = int(fine)
    half = 0.5 * float(edge_length)
    dx = float(edge_length) / float(N)  # 细边等分间距
    y_lo = 0.0
    y_hi = float(depth)

    def pt(u, w):
        return (side * w, u)

    def line(u_a, w_a, u_b, w_b):
        sketch.Line(point1=pt(u_a, w_a), point2=pt(u_b, w_b))

    # 细边和粗边的水平边界线（整条带一条，不逐块重复）
    line(-half, y_lo, half, y_lo)
    line(-half, y_hi, half, y_hi)

    patches = {
        3: build_transition_patch(3, fine_spacing=dx, depth=1.0),
        4: build_transition_patch(2, fine_spacing=dx, depth=1.0),
    }
    cursor = -half
    for index, kind in enumerate(split['blocks']):
        width = 3 if kind == '3' else 4
        patch = patches[width]
        verts = patch['vertices']
        for e_a, e_b in patch_unique_edges(patch):
            u_a, v_a = verts[e_a]
            u_b, v_b = verts[e_b]
            if v_a == 0.0 and v_b == 0.0:
                continue          # 在细边边界线上，已整条绘过
            if v_a == 1.0 and v_b == 1.0:
                continue          # 在粗边边界线上
            if abs(u_a - u_b) < 1e-9 and abs(u_a) < 1e-9 and index > 0:
                continue          # 与前一块的右边重合
            line(cursor + u_a, v_a * depth, cursor + u_b, v_b * depth)
        cursor += width * dx

    if abs(cursor - half) > 1e-6:
        raise ValueError(
            'Hybrid layout error: consumed %g fine segments != total fine segments %d'
            % ((cursor + half) / dx, N))

    coarse_offsets = []
    cursor = -half
    for kind in split['blocks']:
        steps = (3,) if kind == '3' else (2, 2)
        for step in steps:
            cursor += step * dx
            coarse_offsets.append(cursor)
    ext = max(0.0, float(regular_extension or 0.0))
    extension_lines = 0
    if ext > EPS:
        for u in coarse_offsets[:-1]:
            line(u, y_hi, u, y_hi + ext)
            extension_lines += 1
    return {'a_3to1': split['a'], 'b_2to1': split['b'],
            'pairs': split['pairs'], 'blocks': list(split['blocks']),
            'pattern': ','.join(split['blocks']),
            'auto_order': split['auto_order'],
            'coarse_ports': split['coarse_ports'],
            'coarse_offsets': coarse_offsets,
            'regular_extension': ext,
            'regular_extension_lines': extension_lines,
            'expected_faces': split['element_faces'],
            'label': 'HYBRID(%s)' % split['label']}


# Partition creation

def _partition_planar_transition_from_edge(part, face, edge, model, depth, fine, ratio,
                                           layers, side=1.0, name="transition_tmp",
                                           coarse=None, ratio_sequence=None,
                                           stage_widths=None, hybrid_pattern='',
                                           hybrid_regular_extension=0.0,
                                           regular_faces=None):
    return _partition_planar_transition_from_edge_side(
        part=part,
        face=face,
        edge=edge,
        model=model,
        depth=depth,
        fine=fine,
        ratio=ratio,
        layers=layers,
        side=side,
        name=name,
        coarse=coarse, ratio_sequence=ratio_sequence, stage_widths=stage_widths,
        hybrid_pattern=hybrid_pattern,
        hybrid_regular_extension=hybrid_regular_extension,
        regular_faces=regular_faces
    )


def _partition_planar_transition_from_edge_side(part, face, edge, model, depth, fine, ratio,
                                                layers, side, name="transition_tmp",
                                                coarse=None, ratio_sequence=None,
                                                stage_widths=None, hybrid_pattern='',
                                                hybrid_regular_extension=0.0,
                                                regular_faces=None):
    p0, p1 = _edge_points(edge, part=part)
    length = _vnorm(_vsub(p1, p0))
    if length <= EPS:
        raise ValueError("Selected finer edge has zero length.")
    origin = _edge_midpoint(edge, part=part)
    ext = max(0.0, float(hybrid_regular_extension or 0.0))
    sheet_size = max(length, depth + ext) * 4.0

    sketch_name = name
    suffix = 1
    # @author  created by IY,China University of Petroleum (East China)  --  禁止商用 (non-commercial)
    while sketch_name in model.sketches:
        sketch_name = "%s_%d" % (name, suffix)
        suffix += 1
    try:
        transform = part.MakeSketchTransform(
            sketchPlane=face,
            sketchUpEdge=edge,
            sketchPlaneSide=SIDE1,
            sketchOrientation=RIGHT,
            origin=origin
        )
        sketch = model.ConstrainedSketch(
            name=sketch_name, sheetSize=sheet_size, transform=transform
        )
        part.projectReferencesOntoSketch(sketch=sketch, filter=COPLANAR_EDGES)
        hybrid_meta = None
        if ratio == 'HYBRID' and coarse is not None:
            hybrid_meta = _draw_hybrid_pattern(
                sketch=sketch,
                edge_length=length,
                depth=depth,
                fine=fine,
                coarse=coarse,
                side=side,
                pattern=hybrid_pattern,
                regular_extension=hybrid_regular_extension
            )
        else:
            _draw_edge_anchored_pattern(
                sketch=sketch,
                edge_length=length,
                depth=depth,
                fine=fine,
                ratio=int(ratio) if ratio != 'HYBRID' else 3,
                layers=layers,
                side=side, ratio_sequence=ratio_sequence, stage_widths=stage_widths
            )
        partition_faces = [face]
        for item in (regular_faces or []):
            item = _single(item)
            if getattr(item, 'index', None) != getattr(face, 'index', None):
                partition_faces.append(item)
        feat = part.PartitionFaceBySketch(
            faces=tuple(partition_faces), sketch=sketch, sketchUpEdge=edge)
        del model.sketches[sketch_name]
        feat_name = getattr(feat, 'name', None)
        out = {"side": side, "length": length, "partition_feature": feat_name,
               "regular_extension_faces": [int(f.index)
                                           for f in partition_faces[1:]]}
        if hybrid_meta is not None:
            out['hybrid'] = hybrid_meta
        return out
    except Exception as exc:
        try:
            if sketch_name in model.sketches:
                del model.sketches[sketch_name]
        except Exception:
            pass
        raise RuntimeError("Edge-anchored transition partition failed: %s" % exc)


def _edge_in_transition_band(part, edge, frame, tol=1.0e-5):
    """Return True when an edge lies on the selected face inside the transition band."""
    try:
        p0, p1 = _edge_points(edge, part=part)
    except Exception:
        return False

    origin = frame.get("origin")
    u_dir = frame.get("u_dir")
    v_dir = frame.get("v_dir")
    normal = frame.get("normal")
    edge_length = float(frame.get("edge_length"))
    depth = float(frame.get("depth"))
    if origin is None or u_dir is None or v_dir is None or normal is None:
        return False

    points = (
        p0,
        p1,
        (0.5 * (p0[0] + p1[0]), 0.5 * (p0[1] + p1[1]), 0.5 * (p0[2] + p1[2])),
    )
    half = 0.5 * edge_length
    for point in points:
        rel = _vsub(point, origin)
        # Only seed the transition face itself; through-thickness side edges
        # should keep the user's/global seed behavior.
        if abs(_vdot(rel, normal)) > tol:
            return False
        u = _vdot(rel, u_dir)
        w = _vdot(rel, v_dir)
        if u < -half - tol or u > half + tol:
            return False
        side = frame.get("side")
        if side is None:
            if abs(w) > depth + tol:
                return False
        else:
            target_w = -float(side) * depth
            low = min(0.0, target_w) - tol
            high = max(0.0, target_w) + tol
            if w < low or w > high:
                return False
    return True


def partition_planar_transition(part, face, finer_edge=None, depth=None, layers=2,
                                seeds=8, orientation="HU", ratio=2,
                                m=None, n=None, coarse_seeds=None,
                                coarser_edge=None,
                                inset=1.0e-4, name="transition_tmp",
                                ratio_sequence=None, stage_widths=None,
                                hybrid_pattern='', hybrid_regular_extension=False,
                                regular_faces=None):
    """depth is the transition width normal to the selected finer edge in sketch coordinates."""
    # Support legacy m/n call signature for script backward compat
    if m is not None and n is not None:
        coarse, fine, ratio = _legacy_transition_counts(m=m, n=n)
        layers = int(n)
    elif coarse_seeds is not None:
        # Hybrid 模式：用户显式提供粗边种子数
        import transition_mesh_rules as tm_rules
        fine = int(seeds)
        coarse = int(coarse_seeds)
        split = tm_rules.hybrid_split(fine, coarse, sequence=hybrid_pattern)
        if not split['ok']:
            raise ValueError(split['error'])
        layers = 1  # Hybrid 当前仅支持单层
        if split['mode'] == 'pure2':
            ratio = 2  # 纯 2:1
        elif split['mode'] == 'pure3':
            ratio = 3  # 纯 3:1
        else:
            ratio = 'HYBRID'  # 混合模式
    else:
        chain = plan_counts(seeds, layers, ratio, ratio_sequence)
        fine, coarse = chain['counts'][0], chain['counts'][-1]
        ratio = int(ratio)

    face = _single(face)
    edge = _single(finer_edge) if finer_edge is not None else None
    origin, u_dir, v_dir, normal = build_face_frame(face, edge, part=part)

    if edge is not None:
        p0, p1 = _edge_points(edge, part=part)
        length = _vnorm(_vsub(p1, p0))
        frame_origin = (
            0.5 * (p0[0] + p1[0]),
            0.5 * (p0[1] + p1[1]),
            0.5 * (p0[2] + p1[2]),
        )
    else:
        length = float(fine)  # fallback: 1 unit per seed
        frame_origin = origin
    if depth is None:
        depth = max(length / max(float(ratio), 1.0), length * 0.1)

    # A picked coarse edge wins over the typed depth:
    # the transition width is the real in-plane distance to that edge and the drawing side follows from where
    coarse_edge_obj = (_single(coarser_edge)
                       if coarser_edge is not None else None)
    side_from_coarse = None
    depth_source = 'user_depth'
    if coarse_edge_obj is not None:
        if edge is None:
            raise ValueError(
                "TM-COARSE-NOFINE: a coarse edge needs a finer edge to measure "
                "from; pick the finer edge too.")
        depth, side_from_coarse, orientation = resolve_depth_from_coarse_edge(
            part=part, face=face, fine_edge=edge, coarse_edge=coarse_edge_obj,
            v_dir=v_dir, u_dir=u_dir, normal=normal)
        depth_source = 'coarse_edge'

    # Find owning model
    model = mdb.models[part.modelName] if hasattr(part, "modelName") else None
    if model is None:
        for mdl in mdb.models.values():
            for p in mdl.parts.values():
                if p is part:
                    model = mdl
                    break
            if model is not None:
                break
    if model is None:
        raise RuntimeError("Cannot find owning model for part.")

    if edge is not None:
        if side_from_coarse is not None:
            side = side_from_coarse
        else:
            side = _resolve_planar_side(
                face=face,
                edge=edge,
                part=part,
                orientation=orientation,
                depth=depth,
                normal=normal,
                u_dir=u_dir,
                v_dir=v_dir
            )
        regular_extension = 0.0
        extension_faces = []
        if ratio == 'HYBRID' and hybrid_regular_extension:
            extension_faces = _extension_candidate_faces(
                face, regular_faces, part, frame_origin, u_dir, v_dir,
                normal, side, depth, length)
            if isinstance(hybrid_regular_extension, bool):
                regular_extension = _regular_extension_from_faces(
                    [face] + extension_faces, part, frame_origin, v_dir,
                    side, depth)
            else:
                regular_extension = float(hybrid_regular_extension)
        anchored = _partition_planar_transition_from_edge(
            part=part,
            face=face,
            edge=edge,
            model=model,
            depth=depth,
            fine=fine,
            ratio=ratio,
            layers=layers,
            side=side,
            name=name,
            coarse=coarse if ratio == 'HYBRID' else None,
            ratio_sequence=ratio_sequence, stage_widths=stage_widths,
            hybrid_pattern=hybrid_pattern,
            hybrid_regular_extension=regular_extension,
            regular_faces=extension_faces
        )
    else:
        # Non-edge-anchored: draw bounding-box pattern
        x0 = inset
        x1 = max(inset * 2.0, length - inset)
        if orientation in ("HD", "HU"):
            y0 = depth - inset
            y1 = inset
        else:
            y0 = inset
            y1 = max(inset * 2.0, depth - inset)

        transform = part.MakeSketchTransform(sketchPlane=face, sketchPlaneSide=SIDE1)
        sketch = model.ConstrainedSketch(name=name, sheetSize=max(length, depth) * 4.0, transform=transform)
        part.projectReferencesOntoSketch(sketch=sketch)
        _draw_transition_pattern(
            sketch=sketch,
            edge_length=x1 - x0,
            depth=abs(y0 - y1),
            fine=fine,
            ratio=ratio,
            layers=layers,
            side=1.0
        )
        feat = part.PartitionFaceBySketch(faces=(face,), sketch=sketch)
        del model.sketches[name]
        anchored = {"side": 1.0, "length": length,
                    "partition_feature": getattr(feat, 'name', None)}

    result = {
        "coarse": coarse,
        "fine": fine,
        "ratio": ratio,
        "depth": depth,
        "depth_source": depth_source,
        "orientation": orientation,
        "fine_spacing": length / float(fine) if fine else None,
        "origin": frame_origin,
        "u_dir": u_dir,
        "v_dir": v_dir,
        "normal": normal,
        "edge_length": length,
    }
    if coarse_edge_obj is not None:
        try:
            result['coarse_edge_index'] = coarse_edge_obj.index
        except Exception:
            pass
    if ratio != 'HYBRID':
        chain = plan_counts(fine, layers, ratio, ratio_sequence)
        result['stage_counts'] = chain['counts']
        result['stage_ratios'] = chain['ratios']
        result['stage_widths'] = allocate_widths(depth, chain['counts'], stage_widths)
        result['coarse_spacing'] = length / coarse
    if anchored is not None:
        result.update({
            "anchored": True,
            "side": anchored["side"],
            "edge_length": anchored["length"],
            "partition_features": (
                [anchored["partition_feature"]]
                if anchored.get("partition_feature") else []),
        })
        if anchored.get('hybrid') is not None:
            result['hybrid'] = anchored['hybrid']
        if anchored.get('regular_extension_faces'):
            result['regular_extension_faces'] = anchored[
                'regular_extension_faces']
    else:
        result["partition_features"] = []
    return result


# Cylinder transition

def partition_cylinder_transition(part, cylindrical_face, finer_edge=None, depth=None,
                                  layers=2, seeds=16, direction="AXIAL", ratio=2):
    """This helper intentionally uses datum planes because sketch partitioning on curved faces is fragile."""
    face = _single(cylindrical_face)
    edge = _single(finer_edge) if finer_edge is not None else None

    coarse, fine, ratio = transition_counts(
        fine_seeds=seeds, ratio=ratio, layers=layers
    )

    created = []
    z_levels = []
    seed_counts = []
    radius = None

    if direction.upper().startswith("AX"):
        # Determine the z-coordinate of the picked finer edge
        if edge is not None:
            edge_point = _edge_average_point(edge, part=part)
            fine_z = edge_point[2]
            samples = _edge_sample_points(edge, part=part)
            if samples:
                radius = sum(math.sqrt(p[0] ** 2 + p[1] ** 2) for p in samples) / float(len(samples))
        else:
            fine_z = _face_point(face)[2]
            point = _face_point(face)
            radius = math.sqrt(point[0] ** 2 + point[1] ** 2)

        # Compute axial extent from part vertices
        all_z = [tuple(v.pointOn[0])[2] for v in part.vertices]
        z_min = min(all_z)
        z_max = max(all_z)

        # Determine inward sign: fine edge at z_max transitions toward lower z
        z_mid = 0.5 * (z_min + z_max)
        sign = -1.0 if fine_z >= z_mid else 1.0

        # Default depth: span to the opposite end
        if depth is None:
            depth = abs(z_max - z_min) - abs(fine_z - z_mid) * 2.0
            if depth < EPS:
                depth = abs(z_max - z_min) * 0.5

        # Axial boundary levels, evenly split per layer from fine_z to fine_z + sign * depth.
        layers = int(layers)
        if layers <= 0:
            raise ValueError("Cylinder AXIAL transition requires Layers / n > 0.")
        z_levels = [fine_z + sign * depth * i / float(layers) for i in range(layers + 1)]

        # Partition by internal boundary planes (skip the fine_z level itself)
        for i in range(1, layers + 1):
            z = z_levels[i]
            if z < z_min - EPS or z > z_max + EPS:
                print("Info: z=%.3f outside part range [%.3f, %.3f], skipping partition." % (z, z_min, z_max))
                continue
            try:
                datum = part.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE, offset=z)
                part.PartitionCellByDatumPlane(cells=part.cells, datumPlane=part.datums[datum.id])
                created.append(datum.id)
                print("Partition: datum plane at z=%.3f (level %d)" % (z, i))
            except Exception as exc:
                print("Warning: cylinder axial partition at z=%.3f failed: %s" % (z, exc))

        # Build circular ring seed counts for later local seeding.
        for level in range(layers + 1):
            z = z_levels[level]
            target_count = fine // int(pow(ratio, level))
            if target_count < 1:
                target_count = 1
            seed_counts.append(target_count)
            print("Level %d (z=%.3f): seed=%d" % (level, z, target_count))

    else:
        for i in range(1, int(layers) + 1):
            angle = math.pi * i / float(layers + 1)
            print("Info: circumferential layer %d target angle %.3f rad; create datum plane through axis if needed." % (i, angle))

    return {
        "coarse": coarse,
        "fine": fine,
        "ratio": ratio,
        "datum_ids": created,
        "z_levels": z_levels,
        "seed_counts": seed_counts,
        "radius": radius,
    }


# Adjacent seed sync (script-only helper, not exposed in GUI)

def _same_edge(e1, e2, tol=1.0e-5):
    try:
        a0, a1 = _edge_points(e1)
        b0, b1 = _edge_points(e2)
    except Exception:
        return False
    return (
        _vnorm(_vsub(a0, b0)) < tol and _vnorm(_vsub(a1, b1)) < tol
    ) or (
        _vnorm(_vsub(a0, b1)) < tol and _vnorm(_vsub(a1, b0)) < tol
    )


def _edge_average_point(edge, part=None):
    points = _edge_sample_points(edge, part=part)
    if points:
        count = float(len(points))
        return (
            sum(point[0] for point in points) / count,
            sum(point[1] for point in points) / count,
            sum(point[2] for point in points) / count,
        )
    return _edge_midpoint(edge, part=part)


def sync_adjacent_face_seeds(part, primary_face, adjacent_faces=None, shared_edges=None, seed_number=None):
    """This is a script-only helper. It is not exposed in the GUI."""
    if seed_number is None:
        seed_number = 1
    edges = []
    if shared_edges is not None:
        edges.extend(_flatten_geom(shared_edges))
    else:
        primary_edges = list(primary_face.getEdges())
        for face in adjacent_faces or []:
            for eid in face.getEdges():
                if eid in primary_edges:
                    try:
                        edges.append(part.edges[eid])
                    except Exception:
                        pass
    if not edges:
        print("Warning: no shared edges found for synchronization.")
        return 0
    for edge in edges:
        part.seedEdgeByNumber(edges=(edge,), number=int(seed_number), constraint=FIXED)
    return len(edges)


# Validation helpers

def validate_meshability(part, regions=None):
    """Print actionable diagnostics for meshability."""
    try:
        cells = part.cells if regions is None else regions
        print("Meshability check: cells/faces region count = %s" % len(cells))
    except Exception:
        print("Meshability check: region count unavailable.")
    print("Reminder: transition face partitions do not guarantee 3D hex mesh.")
    print("For solids, keep source/target faces topologically consistent and use Sweep/Structured controls.")
    return True


def validate_meshability_by_temp_mesh(part, cleanup=True):
    """This is intentionally stronger than static topology messages."""
    had_mesh = False
    try:
        had_mesh = len(part.elements) > 0
    except Exception:
        had_mesh = False

    if had_mesh:
        elem_count = len(part.elements)
        node_count = len(part.nodes)
        print(
            "Meshability validation skipped temporary generation because an "
            "existing mesh is already present: %d elements, %d nodes."
            % (elem_count, node_count)
        )
        return {
            "status": "EXISTING_MESH",
            "elements": elem_count,
            "nodes": node_count,
            "mesh_deleted": False,
        }

    part.generateMesh()
    elem_count = len(part.elements)
    node_count = len(part.nodes)
    if elem_count <= 0 or node_count <= 0:
        raise RuntimeError(
            "Temporary mesh generation produced %d elements and %d nodes."
            % (elem_count, node_count)
        )
    print(
        "Meshability validation passed with temporary mesh: %d elements, %d nodes."
        % (elem_count, node_count)
    )

    mesh_deleted = False
    if cleanup:
        part.deleteMesh()
        mesh_deleted = True
        print("Temporary validation mesh deleted; seeds and mesh controls are preserved.")

    return {
        "status": "PASSED",
        "elements": elem_count,
        "nodes": node_count,
        "mesh_deleted": mesh_deleted,
    }


# Mesh control helper

def set_mesh_controls_hex(part, region=None):
    """Tries SWEEP+ADVANCING_FRONT first, then SWEEP+MEDIAL_AXIS, then STRUCTURED,"""
    target = region if region is not None else part.cells
    # (label, technique, algorithm|None)
    attempts = [
        ('SWEEP+ADVANCING_FRONT', SWEEP, ADVANCING_FRONT),
        ('SWEEP+MEDIAL_AXIS',    SWEEP, MEDIAL_AXIS),
        ('STRUCTURED',           STRUCTURED, None),
        ('SWEEP',                SWEEP, None),
    ]
    for label, technique, algorithm in attempts:
        try:
            kw = dict(regions=target, elemShape=HEX, technique=technique)
            if algorithm is not None:
                kw['algorithm'] = algorithm
            part.setMeshControls(**kw)
            print("Mesh controls set to %s (elemShape=HEX)." % label)
            return label
        except Exception:
            continue
    print("Warning: could not set Hex/Sweep mesh controls.")
    return None


def _seed_planar_solid_width_edges(part, frame, cells, per_edge_seed=1):
    """按源面宽度边链同步对应边的固定段数，不切分实体或改扫掠路径。"""
    if not frame or not cells:
        return []
    tol = 1.e-6 * max(1.0, float(frame['depth']), float(frame['edge_length']))
    half = .5 * float(frame['edge_length'])
    target = -float(frame['side']) * float(frame['depth'])
    band_lo, band_hi = sorted((0.0, target))
    edge_ids = sorted(set(ei for cell in cells for fi in cell.getFaces()
                          for ei in part.faces[fi].getEdges()))
    source = {-1: [], 1: []}
    opposite = []
    for index in edge_ids:
        edge = part.edges[index]
        if len(edge.getVertices()) != 2:
            continue
        points = _edge_points(edge, part=part)
        local = []
        for point in points:
            rel = _vsub(point, frame['origin'])
            local.append(tuple(_vdot(rel, frame[k])
                               for k in ('u_dir', 'v_dir', 'normal')))
        (u0, v0, z0), (u1, v1, z1) = local
        lo, hi = sorted((v0, v1))
        if (abs(u0-u1) > tol or abs(z0-z1) > tol or hi-lo <= tol
                or abs(abs(u0)-half) > tol
                or lo < band_lo-tol or hi > band_hi+tol):
            continue
        # 只处理直棱柱的宽度方向边，排除曲边与厚度方向边。
        if abs(float(edge.getSize(printResults=False)) - (hi-lo)) > tol:
            continue
        side = -1 if u0 < 0 else 1
        if abs(z0) <= tol:
            source[side].append((lo, hi, int(per_edge_seed)))
        else:
            opposite.append((index, side, lo, hi))

    updates = []
    for index, side, lo, hi in opposite:
        spans = sorted((a, b, n) for a, b, n in source[side]
                       if a >= lo-tol and b <= hi+tol)
        if not spans:
            continue
        cursor = lo
        number = 0
        for a, b, n in spans:
            if abs(a-cursor) > tol:
                raise ValueError('TM-WIDTH-SEED: source width chain has a gap '
                                 'opposite edge %d.' % index)
            cursor = b
            number += n
        if abs(cursor-hi) > tol:
            raise ValueError('TM-WIDTH-SEED: opposite edge %d does not align '
                             'with the source stage boundaries.' % index)
        edge = part.edges[index]
        old_number = int(part.getEdgeSeeds(edge, NUMBER))
        old_constraint = str(part.getEdgeSeeds(edge, CONSTRAINT))
        try:
            part.seedEdgeByNumber(edges=(edge,), number=number, constraint=FIXED)
        except Exception as exc:
            raise RuntimeError('TM-WIDTH-SEED: edge %d needs %d fixed segments: %s'
                               % (index, number, exc))
        updates.append({'edge': index, 'number': number,
                        'previous_number': old_number,
                        'previous_constraint': old_constraint})
    return updates


def prepare_planar_transition_mesh(part, transition_info=None, fine_spacing=None,
                                   per_edge_seed=1, region=None,
                                   transition_cells=None,
                                   auto_structure_regular=True):
    """This helper intentionally preserves any global seed already set by the user."""
    seeded = 0
    frame = transition_info if isinstance(transition_info, dict) else None
    for edge in part.edges:
        # created by IY,China University of Petroleum (East China)
        try:
            if frame is not None:
                should_seed = _edge_in_transition_band(part, edge, frame)
            elif fine_spacing is not None:
                p0, p1 = _edge_points(edge, part=part)
                length = _vnorm(_vsub(p1, p0))
                should_seed = length <= 3.1 * float(fine_spacing) + EPS
            else:
                should_seed = True
            if not should_seed:
                continue
            part.seedEdgeByNumber(
                edges=(edge,),
                number=int(per_edge_seed),
                constraint=FIXED
            )
            seeded += 1
        except Exception as exc:
            raise RuntimeError('TM-PLANAR-SEED: cannot seed transition edge %s: %s'
                               % (getattr(edge, 'index', '?'), exc))

    width_seeds = _seed_planar_solid_width_edges(
        part, frame, transition_cells, per_edge_seed)
    elem_label = None
    if abaqus_mesh is not None:
        try:
            # Element family is common to the solid; this does not change the
            # per-Cell STRUCTURED/SWEEP technique or its sweep path.
            elem_region = region if region is not None else (part.cells,)
            elem_type = abaqus_mesh.ElemType(elemCode=C3D8R, elemLibrary=STANDARD)
            part.setElementType(regions=elem_region, elemTypes=(elem_type,))
            elem_label = "C3D8R"
            print("Element type set to C3D8R.")
        except Exception as exc:
            print("Warning: could not set C3D8R element type: %s" % exc)

    controls = {
        'transition_cells': [],
        'transition_preserved': [],
        'transition_auto_structured': [],
        'regular_structured': [],
    }
    try:
        all_cells = list(part.cells)
    except Exception:
        all_cells = []

    if all_cells:
        picked = []
        seen = set()
        for cell in (transition_cells or []):
            try:
                idx = int(cell.index)
            except Exception:
                try:
                    idx = int(cell)
                except Exception:
                    continue
            if idx not in seen and 0 <= idx < len(part.cells):
                seen.add(idx)
                picked.append(part.cells[idx])
        controls['transition_cells'] = sorted(seen)

        # Preserve a user-defined Sweep (and its path) or Structured control.
        # Only an unassigned/system Cell is automatically made Structured.
        for cell in picked:
            technique = None
            try:
                technique = part.getMeshControl(region=cell,
                                                attribute=TECHNIQUE)
            except Exception:
                pass
            name = str(technique).upper() if technique is not None else ''
            if 'SWEEP' in name or 'STRUCTURED' in name:
                controls['transition_preserved'].append(
                    {'cell': int(cell.index), 'technique': str(technique)})
                continue
            try:
                part.setMeshControls(regions=(cell,), elemShape=HEX,
                                     technique=STRUCTURED)
                controls['transition_auto_structured'].append(int(cell.index))
            except Exception as exc:
                raise RuntimeError(
                    'Transition Cell %s has no usable Structured/Sweep control. '
                    'Assign Sweep and its sweep direction/path before running '
                    'the plugin. Abaqus detail: %s' % (cell.index, exc))

        if auto_structure_regular:
            regular = tuple(cell for cell in all_cells
                            if int(cell.index) not in seen)
            if regular:
                try:
                    part.setMeshControls(regions=regular, elemShape=HEX,
                                         technique=STRUCTURED)
                    controls['regular_structured'] = [int(c.index)
                                                       for c in regular]
                except Exception as exc:
                    raise RuntimeError(
                        'Regular Cells could not be assigned Structured mesh '
                        'controls: %s' % exc)

    ctrl = 'PRESERVE_TRANSITION__STRUCTURE_REGULAR'
    return {"seeded_edges": seeded, "element_type": elem_label,
            "mesh_control": ctrl, "cell_controls": controls,
            "width_edge_seeds": width_seeds}


def _edge_sample_points(edge, part=None):
    points = []
    if part is not None:
        try:
            verts = edge.getVertices()
            for vid in verts:
                points.append(tuple(part.vertices[vid].pointOn[0]))
        except Exception:
            pass
    try:
        for point in edge.pointOn:
            points.append(tuple(point))
    except Exception:
        pass
    return points


def prepare_cylinder_transition_mesh(part, transition_info=None, per_edge_seed=None,
                                     region=None):
    """The function preserves existing/global seeds and locally seeds circular ring edges at the axial"""
    info = transition_info if isinstance(transition_info, dict) else {}
    z_levels = info.get("z_levels") or []
    seed_counts = info.get("seed_counts") or []
    radius = info.get("radius") if info.get("match_radius_only") else None
    seeded = 0

    for level_index, z_value in enumerate(z_levels):
        if level_index < len(seed_counts):
            target_count = int(seed_counts[level_index])
        elif per_edge_seed is not None:
            target_count = int(per_edge_seed)
        else:
            continue
        for edge in part.edges:
            try:
                points = _edge_sample_points(edge, part=part)
                if not points:
                    continue
                if max(abs(point[2] - z_value) for point in points) > 1.0e-5:
                    continue
                if radius is not None:
                    radial = [math.sqrt(point[0] ** 2 + point[1] ** 2) for point in points]
                    if max(abs(value - float(radius)) for value in radial) > 1.0e-4:
                        continue
                part.seedEdgeByNumber(
                    edges=(edge,),
                    number=target_count,
                    constraint=FIXED
                )
                seeded += 1
            except Exception:
                pass

    elem_label = None
    if abaqus_mesh is not None:
        try:
            elem_region = region if region is not None else (part.cells,)
            elem_type = abaqus_mesh.ElemType(elemCode=C3D8R, elemLibrary=STANDARD)
            part.setElementType(regions=elem_region, elemTypes=(elem_type,))
            elem_label = "C3D8R"
            print("Element type set to C3D8R.")
        except Exception as exc:
            print("Warning: could not set C3D8R element type: %s" % exc)

    ctrl = set_mesh_controls_hex(part, region=region)
    return {"seeded_edges": seeded, "element_type": elem_label, "mesh_control": ctrl}


def install_menu():
    """Abaqus GUI registration differs between RSG/custom toolsets."""
    print("Transition mesh plugin loaded. Use transition_mesh_plugin functions or the documented macro entry.")
