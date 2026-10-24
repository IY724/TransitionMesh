# -*- coding: utf-8 -*-
# ============================================================================
#  Transition Mesh
#  created by IY,China University of Petroleum (East China)
#  禁止商用 / Non-commercial use only
#  禁止商用：本软件仅供学习与研究使用，未经作者书面许可不得用于任何商业用途。
#  Non-commercial use only. Unauthorized commercial use is prohibited.
# ============================================================================

"""2D transition kernels for PLANAR_ANNULAR and PLANAR_QUAD_FOUR; every template edge is seeded FIXED."""
from __future__ import print_function

import math

from abaqus import mdb
from abaqusConstants import (
    FIXED, STRUCTURED, SWEEP, ADVANCING_FRONT, STANDARD,
    CPS4R, CPE4R, C3D8R, SIDE1, SIDE2, RIGHT, QUAD, HEX, NUMBER,
    TECHNIQUE,
)
from mesh import ElemType

from transition_mesh_plugin import (
    build_transition_patch, patch_unique_edges, expected_transition_elements,
)
import transition_mesh_rules as tm_rules

# GUI 运行时保持静默：信息性 print 默认不输出到 CAE 底部信息区，仅
# TM_VERBOSE=1 （回归脚本会设置）时回显；异常/返回值/模型不受影响。
import os as _tm_os


def _tm_quiet_print(*args, **kwargs):
    return 0


if not _tm_os.environ.get('TM_VERBOSE'):
    print = _tm_quiet_print  # 有意遮蔽模块内建 print

_DBG = None
_TOL = 1e-4
_ACTIONS = ('PARTITION_ONLY', 'PARTITION_AND_SEED', 'PARTITION_SEED_MESH')


def _dbg(msg):
    global _DBG
    try:
        if _DBG is None:
            import os
            import tempfile
            path = os.path.join(
                tempfile.gettempdir(), 'TransitionMesh', 'planar2d.log')
            d = os.path.dirname(path)
            if not os.path.isdir(d):
                try:
                    os.makedirs(d)
                except Exception:
                    pass
            _DBG = open(path, 'a')
        _DBG.write(str(msg) + '\n')
        _DBG.flush()
    except Exception:
        pass
    print(msg)


def _div_msg(label, n, k):
    """Divisibility error with the nearest valid values (spec: no silent fix)."""
    lo = (n // k) * k
    hi = lo + k
    return ('%s: N=%d must be divisible by %d; nearest valid: %d or %d'
            % (label, n, k, max(k, lo), hi))


def _safe_elem(name):
    name = str(name or '').strip().upper()
    if name in ('', 'KEEP', 'NONE', 'DEFAULT'):
        return None
    if name not in ('CPS4R', 'CPE4R'):
        raise ValueError('element2d must be CPS4R or CPE4R, got %r' % name)
    return CPS4R if name == 'CPS4R' else CPE4R


def _owner_model(part):
    try:
        return mdb.models[part.modelName]
    except Exception:
        pass
    for mdl in mdb.models.values():
        for p in mdl.parts.values():
            if p is part:
                return mdl
    raise RuntimeError('Cannot find owning model for part.')


def _as_index(x):
    if isinstance(x, int):
        return x
    return x.index


def _require_2d(part):
    dim = str(getattr(part, 'dimensionality', '')).upper()
    if dim and ('TWO_D' not in dim and 'PLANAR' not in dim):
        raise ValueError(
            'PLANAR_ANNULAR needs a 2D Part; use PLANAR_QUAD_FOUR on a '
            'planar source face of a solid, got %s' % dim)


def _is_solid(part):
    try:
        return len(part.cells) > 0
    except Exception:
        return False


def _cell_indices_from_faces(faces):
    """Capture adjacent solid Cells before PartitionFaceBySketch changes faces."""
    out = set()
    for face in faces or []:
        try:
            out.update(int(i) for i in face.getCells())
        except Exception:
            pass
    return sorted(out)


def _prepare_solid_cell_controls(part, transition_cell_indices):
    """A user-prepared SWEEP Cell keeps its source/path. STRUCTURED is also accepted."""
    transition = set(int(i) for i in (transition_cell_indices or []))
    if not transition:
        raise ValueError(
            'No transition Cell is attached to the selected planar face(s).')
    detail = {'transition_cells': sorted(transition),
              'transition_preserved': [],
              'transition_auto_structured': [],
              'regular_structured': []}
    for idx in sorted(transition):
        cell = part.cells[idx]
        technique = None
        try:
            technique = part.getMeshControl(region=cell, attribute=TECHNIQUE)
        except Exception:
            pass
        name = str(technique).upper() if technique is not None else ''
        if 'SWEEP' in name or 'STRUCTURED' in name:
            detail['transition_preserved'].append(
                {'cell': idx, 'technique': str(technique)})
        else:
            try:
                part.setMeshControls(regions=(cell,), elemShape=HEX,
                                     technique=STRUCTURED)
                detail['transition_auto_structured'].append(idx)
            except Exception as exc:
                raise ValueError(
                    'Transition Cell %d is not Structured-meshable and has no '
                    'user-defined Sweep path. Assign Sweep + sweep direction '
                    'before running Mesh. Abaqus detail: %s' % (idx, exc))
    regular = tuple(part.cells[i] for i in range(len(part.cells))
                    if i not in transition)
    if regular:
        try:
            part.setMeshControls(regions=regular, elemShape=HEX,
                                 technique=STRUCTURED)
            detail['regular_structured'] = [int(c.index) for c in regular]
        except Exception as exc:
            raise ValueError(
                'Regular Cells could not be assigned Structured mesh controls: '
                '%s' % exc)
    try:
        part.setElementType(
            regions=(part.cells,),
            elemTypes=(ElemType(elemCode=C3D8R, elemLibrary=STANDARD),))
        detail['element_type'] = 'C3D8R'
    except Exception as exc:
        detail['element_type_warning'] = str(exc)
    return detail


def _finish_solid_mesh(part, result, transition_cell_indices, action):
    """Finish a planar-face operation on a solid without touching Sweep paths."""
    detail = _prepare_solid_cell_controls(part, transition_cell_indices)
    result['solid_cell_controls'] = detail
    result['solid_planar_face'] = True
    if action == 'PARTITION_AND_SEED':
        result['faces_meshed'] = 0
        return result
    part.generateMesh()
    result['elements'] = len(part.elements)
    result['nodes'] = len(part.nodes)
    if result['elements'] <= 0:
        raise ValueError(
            'Solid planar-face mesh produced 0 elements. The transition Cell '
            'must be Sweep/Structured meshable and regular Cells Structured.')
    return result


def _edge_seed_count(part, edge):
    try:
        s = part.getEdgeSeeds(edge, NUMBER)
        return int(s) if s else None
    except Exception:
        return None


def _seed_fixed(part, edges, n):
    idxs = []
    for e in edges:
        if e is None:
            continue
        idxs.append(_as_index(e))
    if not idxs:
        return 0
    part.seedEdgeByNumber(
        edges=tuple(part.edges[i] for i in sorted(set(idxs))),
        number=int(n), constraint=FIXED)
    return len(set(idxs))


def _face_seq(part, faces):
    idx = sorted(set(_as_index(f) for f in faces))
    if not idx:
        raise ValueError('empty face set')
    words = {}
    for i in idx:
        words[i // 32] = words.get(i // 32, 0) | (1 << (i % 32))
    top = max(words)
    mask = '[#' + ' #'.join('%x' % words.get(k, 0) for k in range(top + 1)) + ']'
    return part.faces.getSequenceFromMask((mask,),)


def _apply_elem_type(part, faces, elem_name):
    """An empty element2d (GUI default) leaves the part element type alone:"""
    code = _safe_elem(elem_name)
    if code is None:
        return
    name = '__TM2D__'
    try:
        del part.sets[name]
    except Exception:
        pass
    seq = _face_seq(part, faces)
    part.Set(faces=seq, name=name)
    try:
        part.setElementType(
            regions=part.sets[name],
            elemTypes=(ElemType(elemCode=code, elemLibrary=STANDARD),))
    finally:
        try:
            del part.sets[name]
        except Exception:
            pass


def _set_quad_structured(part, faces, elem_name):
    """Returns the applied label ('QUAD+STRUCTURED' / 'QUAD+SWEEP+ADVANCING_FRONT') or False when even the"""
    seq = _face_seq(part, faces)
    ctrl = False
    try:
        part.setMeshControls(regions=seq, elemShape=QUAD, technique=STRUCTURED)
        ctrl = 'QUAD+STRUCTURED'
    except Exception:
        try:
            part.setMeshControls(
                regions=seq, elemShape=QUAD, technique=SWEEP,
                algorithm=ADVANCING_FRONT, allowMapped=False)
            ctrl = 'QUAD+SWEEP+ADVANCING_FRONT'
        except Exception:
            ctrl = False
    _apply_elem_type(part, faces, elem_name)
    return ctrl


def _set_quad_sweep(part, faces, elem_name):
    """QUAD + SWEEP + ADVANCING_FRONT with mapped OFF (full circles /
    annuli cannot be mapped; never enable the mapped option here)."""
    seq = _face_seq(part, faces)
    ctrl = False
    try:
        part.setMeshControls(regions=seq, elemShape=QUAD, technique=SWEEP,
                             algorithm=ADVANCING_FRONT, allowMapped=False)
        ctrl = 'QUAD+SWEEP+ADVANCING_FRONT'
    except Exception:
        try:
            part.setMeshControls(regions=seq, elemShape=QUAD)
            ctrl = 'QUAD(default)'
        except Exception:
            ctrl = False
    _apply_elem_type(part, faces, elem_name)
    return ctrl


# Mesh audit (counts + quad quality: area sign, bowtie, dup, hanging nodes)

def _orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _seg_strict_intersect(p1, p2, p3, p4):
    d1 = _orient(p3, p4, p1)
    d2 = _orient(p3, p4, p2)
    d3 = _orient(p1, p2, p3)
    d4 = _orient(p1, p2, p4)
    return (((d1 > 1e-12 and d2 < -1e-12) or (d1 < -1e-12 and d2 > 1e-12))
            and ((d3 > 1e-12 and d4 < -1e-12)
                 or (d3 < -1e-12 and d4 > 1e-12)))


def _quality_audit(part, elem_nodes):
    """elem_nodes: list of tuples; items may be MeshNode objects or labels."""
    def key(nd):
        return nd.label if hasattr(nd, 'label') else int(nd)

    def coord(nd):
        if hasattr(nd, 'coordinates'):
            q = nd.coordinates
        else:
            q = part.nodes[nd].coordinates
        return (float(q[0]), float(q[1]))

    coords = {}
    elem_keys = []
    for nodes in elem_nodes:
        ks = tuple(key(nd) for nd in nodes)
        elem_keys.append(ks)
        for nd, k in zip(nodes, ks):
            if k not in coords:
                try:
                    coords[k] = coord(nd)
                except Exception:
                    pass
    seen = set()
    dup = neg = zero = bowtie = 0
    jac_bad = 0
    ar_max = 0.0
    ar_warn = 0
    for nodes in elem_keys:
        if not nodes:
            continue
        skey = tuple(sorted(nodes))
        if skey in seen:
            dup += 1
        else:
            seen.add(skey)
        if len(nodes) != 4:
            continue
        pts = [coords.get(i) for i in nodes]
        if any(p is None for p in pts):
            continue
        area = 0.0
        for i in range(4):
            x0, y0 = pts[i]
            x1, y1 = pts[(i + 1) % 4]
            area += x0 * y1 - x1 * y0
        area *= 0.5
        # Local edge scale for Jacobian tolerance.
        e2 = []
        for i in range(4):
            dx = pts[(i + 1) % 4][0] - pts[i][0]
            dy = pts[(i + 1) % 4][1] - pts[i][1]
            e2.append(dx * dx + dy * dy)
        emean = sum(e2) / 4.0 if e2 else 0.0
        jac_tol = max(1e-12, 1e-6 * emean)
        if area < -1e-9:
            neg += 1
        elif abs(area) < 1e-12:
            zero += 1
        elif _seg_strict_intersect(pts[0], pts[1], pts[2], pts[3]) \
                or _seg_strict_intersect(pts[1], pts[2], pts[3], pts[0]):
            bowtie += 1
        # 2x2 Gauss Jacobian of the bilinear quad (must stay > 0).
        try:
            if _quad_jacobian_min(pts) <= jac_tol:
                jac_bad += 1
        except Exception:
            pass
        # Aspect ratio from opposite edge lengths (warning only).
        try:
            sides = [math.sqrt(x) for x in e2]
            if min(sides) > 1e-12:
                ar = max(sides) / min(sides)
                if ar > ar_max:
                    ar_max = ar
                if ar > 5.0:
                    ar_warn += 1
        except Exception:
            pass
    hanging = set()
    node_items = [(i, coords[i]) for i in sorted(
        set(j for n in elem_keys for j in n)) if i in coords]
    for nodes in elem_keys:
        n = len(nodes)
        for k in range(n):
            a = coords.get(nodes[k])
            b = coords.get(nodes[(k + 1) % n])
            if a is None or b is None:
                continue
            dx = b[0] - a[0]
            dy = b[1] - a[1]
            l2 = dx * dx + dy * dy
            if l2 <= 0.0:
                continue
            lo0, hi0 = (a[0], b[0]) if a[0] <= b[0] else (b[0], a[0])
            lo1, hi1 = (a[1], b[1]) if a[1] <= b[1] else (b[1], a[1])
            for j, q in node_items:
                if j in nodes:
                    continue
                if (q[0] < lo0 - 1e-7 or q[0] > hi0 + 1e-7
                        or q[1] < lo1 - 1e-7 or q[1] > hi1 + 1e-7):
                    continue
                t = ((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / l2
                if t <= 1e-6 or t >= 1.0 - 1e-6:
                    continue
                if (abs(a[0] + t * dx - q[0]) < 1e-6
                        and abs(a[1] + t * dy - q[1]) < 1e-6):
                    ek = tuple(sorted((nodes[k], nodes[(k + 1) % n])))
                    hanging.add((ek, j))
    return {'duplicate_elements': dup, 'negative_area': neg,
            'zero_area': zero, 'bowtie': bowtie,
            'hanging_nodes': len(hanging),
            'jacobian_bad': jac_bad,
            'aspect_ratio_max': round(ar_max, 3),
            'aspect_ratio_warn': ar_warn}


def _quad_jacobian_min(pts):
    """Minimum signed Jacobian over the 2x2 Gauss points of a bilinear quad."""
    # Shape functions dN/dxi, dN/deta at Gauss pts (+-1/sqrt(3)).
    g = 1.0 / math.sqrt(3.0)
    gps = [(-g, -g), (g, -g), (g, g), (-g, g)]
    jmin = None
    for xi, eta in gps:
        # Derivatives of bilinear shape functions.
        dN = [
            (-0.25 * (1 - eta), -0.25 * (1 - xi)),   # N1
            (0.25 * (1 - eta), -0.25 * (1 + xi)),    # N2
            (0.25 * (1 + eta), 0.25 * (1 + xi)),     # N3
            (-0.25 * (1 + eta), 0.25 * (1 - xi)),    # N4
        ]
        jx_xi = jy_xi = jx_et = jy_et = 0.0
        for (px, py), (dn_xi, dn_et) in zip(pts, dN):
            jx_xi += px * dn_xi
            jy_xi += py * dn_xi
            jx_et += px * dn_et
            jy_et += py * dn_et
        det = jx_xi * jy_et - jx_et * jy_xi
        if jmin is None or det < jmin:
            jmin = det
    return jmin if jmin is not None else 0.0


def _audit_patch_mesh(part, face_idx):
    tri = quad = 0
    unmeshed = 0
    elem_nodes = []
    for fi in face_idx:
        try:
            els = part.faces[fi].getElements()
        except Exception:
            els = None
        if not els:
            unmeshed += 1
            continue
        for e in els:
            try:
                raw = e.getNodes()
            except Exception:
                raw = ()
            nodes = tuple(raw)
            nn = len(nodes) if nodes else 4
            if nn <= 3:
                tri += 1
            else:
                quad += 1
            if nodes:
                elem_nodes.append(nodes)
    audit = {'elements': tri + quad, 'quad': quad, 'tri': tri,
             'unmeshed_faces': unmeshed}
    try:
        audit.update(_quality_audit(part, elem_nodes))
    except Exception as exc:
        audit['quality_error'] = repr(exc)
    return audit


# Unified partition / mesh steps

def _partition_patch_plan(part, plan, draw_fn):
    """One PartitionFaceBySketch call with the plan's sketch, then cleanup."""
    model = plan['model']
    target_faces = plan['target_faces']
    if not target_faces:
        # Empty targets surface as a bare IndexError otherwise; report the
        # actionable cause (stale picks / geometry changed since picking).
        raise ValueError(
            '%s: no target Face available for the partition sketch. The '
            'picked geometry may have changed since selection -- repick '
            'the transition region / Full domain and run Check again.'
            % plan['label'])
    origin = plan['sketch_origin']
    normal = tuple(target_faces[0].getNormal())
    side = SIDE1
    transform_kw = dict(
        sketchPlane=target_faces[0], sketchPlaneSide=side,
        sketchOrientation=RIGHT, origin=_point3(origin))
    sketch_up = plan.get('sketch_up_edge')
    if sketch_up is not None:
        transform_kw['sketchUpEdge'] = sketch_up
    transform = part.MakeSketchTransform(**transform_kw)
    plan['_sketch_transform_matrix'] = transform.matrix()
    sk = model.ConstrainedSketch(name=plan['sketch_name'],
                                 sheetSize=plan['sheet_size'],
                                 transform=transform)
    try:
        draw_fn(sk, plan)
        partition_kw = dict(faces=tuple(target_faces), sketch=sk)
        if sketch_up is not None:
            partition_kw['sketchUpEdge'] = sketch_up
        part.PartitionFaceBySketch(**partition_kw)
    except Exception as ex:
        raise ValueError('%s template partition failed: %r' % (plan['label'], ex))
    finally:
        try:
            del model.sketches[plan['sketch_name']]
        except Exception:
            pass
        try:
            del sk
        except Exception:
            pass
        plan.pop('_sketch_transform_matrix', None)


def _seed_patch_plan(part, band_idx):
    """Every template edge of the transition band = 1 FIXED (spec section 7)."""
    eidx = set()
    for fi in band_idx:
        for e in part.faces[fi].getEdges():
            eidx.add(_as_index(e))
    return _seed_fixed(part, [part.edges[i] for i in sorted(eidx)], 1)


def _mesh_patch_plan(part, face_idx, element2d, prefer_sweep=False):
    """prefer_sweep=True skips the STRUCTURED attempt (full annuli and participating-corner patches are not"""
    from abaqusConstants import FREE as _FREE
    faces = [part.faces[i] for i in face_idx]
    if prefer_sweep:
        ctrl = _set_quad_sweep(part, faces, element2d)
    else:
        ctrl = _set_quad_structured(part, faces, element2d)
    mesh_err = None
    try:
        part.generateMesh(regions=_face_seq(part, faces))
    except Exception as mex:
        mesh_err = repr(mex)
        _dbg('[p2d] generateMesh error: %s' % mesh_err)
    audit = _audit_patch_mesh(part, face_idx)
    if audit['elements'] <= 0:
        try:
            seq = _face_seq(part, faces)
            part.setMeshControls(
                regions=seq, elemShape=QUAD, technique=SWEEP,
                algorithm=ADVANCING_FRONT, allowMapped=False)
            try:
                part.deleteMesh(regions=seq)
            except Exception:
                pass
            part.generateMesh(regions=seq)
            ctrl = 'QUAD+SWEEP+ADVANCING_FRONT'
        except Exception as mex2:
            _dbg('[p2d] sweep fallback error: %r' % (mex2,))
        audit = _audit_patch_mesh(part, face_idx)
    if audit['elements'] <= 0:
        try:
            seq = _face_seq(part, faces)
            part.setMeshControls(regions=seq, elemShape=QUAD, technique=_FREE)
            try:
                part.deleteMesh(regions=seq)
            except Exception:
                pass
            part.generateMesh(regions=seq)
            ctrl = 'QUAD+FREE'
        except Exception as mex3:
            _dbg('[p2d] free fallback error: %r' % (mex3,))
        audit = _audit_patch_mesh(part, face_idx)
    if audit['elements'] <= 0:
        # Do not broaden a failed regional request to the whole Part: that
        # would delete unrelated meshes and defeat the explicit GUI pick scope.
        mesh_err = (mesh_err or
                    'The selected faces produced no elements with the available '
                    'regional quad mesh techniques. Adjust the geometry/seeds '
                    'or select the intended Full domain explicitly.')
        _dbg('[p2d] regional meshing failed: %s' % mesh_err)
    return ctrl, mesh_err, audit


# Planar geometry helpers

def _circle_from_edges(part, edges):
    """Center/radius from circular edge(s); works with one closed circle."""
    if not edges:
        raise ValueError('need circular edges for annular frame')
    try:
        cv = edges[0].getCurvature(0.5)
        ep = tuple(cv['evaluationPoint'])[:2]
        k = tuple(cv['curvature'])[:2]
        kn = math.hypot(k[0], k[1])
        if kn > 1e-12:
            c = (ep[0] + k[0] / (kn * kn), ep[1] + k[1] / (kn * kn))
            r = math.hypot(ep[0] - c[0], ep[1] - c[1])
            return c, r
    except Exception:
        pass
    pts = []
    for e in edges:
        try:
            pts.append(tuple(e.pointOn[0])[:2])
        except Exception:
            pass
    if not pts:
        raise ValueError('need circular edges for annular frame')
    if len(pts) == 1:
        raise ValueError('need circular edges for annular frame')
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    r = sum(math.hypot(p[0] - cx, p[1] - cy) for p in pts) / len(pts)
    return (cx, cy), r


def _edge_radius(part, edge, center):
    pt = edge.pointOn[0]
    return math.hypot(pt[0] - center[0], pt[1] - center[1])


def _point_angle_deg(center, pt):
    return math.degrees(math.atan2(pt[1] - center[1], pt[0] - center[0]))


def _normalize_deg(a):
    a = math.fmod(a, 360.0)
    if a <= -180.0:
        a += 360.0
    elif a > 180.0:
        a -= 360.0
    return a


def _loop_vertex_angles(part, edges, center):
    """Distinct vertex angles (deg) of the picked loop; [] for full circles."""
    out = []
    for e in edges:
        try:
            vs = e.getVertices()
        except Exception:
            continue
        if len(vs) != 2:
            continue
        for vid in vs:
            try:
                p = part.vertices[vid].pointOn[0]
            except Exception:
                continue
            a = _point_angle_deg(center, p)
            if not any(abs(_normalize_deg(a - b)) < 1e-6 for b in out):
                out.append(a)
    return out


def _align_residual_deg(angle, phase, grid):
    r = math.fmod(angle - phase, grid)
    if r < 0.0:
        r += grid
    return min(r, grid - r)


def _check_annular_partition_compat(plan_phase, N, ratio, fine_verts,
                                    coarse_verts):
    """Example: four 90-deg partitions + 2:1 + N=180 needs 22.5 coarse elements per sector."""
    if not fine_verts and not coarse_verts:
        return
    angles = sorted({_normalize_deg(a) for a in (fine_verts or coarse_verts or [])})
    if len(angles) < 2:
        return
    # Sector angles between consecutive partition vertices.
    sectors = []
    for i in range(len(angles)):
        a0 = angles[i]
        a1 = angles[(i + 1) % len(angles)]
        d = a1 - a0
        if d <= 0:
            d += 360.0
        sectors.append(d)
    if not sectors:
        return
    # Minimum sector must hold at least one coarse element.
    coarse = N // ratio
    h_deg = 360.0 / float(N)
    min_sec = min(sectors)
    n_min = min_sec / (ratio * h_deg)
    if n_min >= 1.0 - 1e-9:
        # Check integer coarse counts per sector when sectors share a gcd.
        # Equal quadrants: coarse must be divisible by n_sec.
        uniq = {round(s, 6) for s in sectors}
        if len(uniq) == 1:
            n_sec = len(sectors)
            if coarse % n_sec != 0:
                step = ratio * n_sec
                lo = (N // step) * step
                hi = lo + step
                raise ValueError(
                    'N=%d is incompatible with the %g-deg partitions.\n'
                    'Each sector would contain %g coarse elements.\n'
                    'Use N=%d or N=%d, or revise the partitions manually.'
                    % (N, min_sec, float(coarse) / n_sec,
                       max(step, lo), hi))
    else:
        raise ValueError(
            'N=%d is too small for the %g-deg partitions.\n'
            'The smallest sector would contain %g coarse elements (<1).\n'
            'Increase N or merge the partitions.'
            % (N, min_sec, n_min))


def _face_radius_range(part, face, center):
    rs = []
    try:
        for e in face.getEdges():
            try:
                rs.append(_edge_radius(part, part.edges[_as_index(e)], center))
            except Exception:
                pass
    except Exception:
        pass
    try:
        for vid in face.getVertices():
            try:
                p = part.vertices[vid].pointOn[0]
                rs.append(math.hypot(p[0] - center[0], p[1] - center[1]))
            except Exception:
                pass
    except Exception:
        pass
    if not rs:
        return None
    return (min(rs), max(rs))


def _edge_arc_info(part, edge, center):
    """Returns dict with is_circular, radius, span_deg, r_min, r_max or None."""
    pts = []
    try:
        vs = edge.getVertices()
    except Exception:
        vs = ()
    for vid in vs:
        try:
            v = part.vertices[vid] if isinstance(vid, int) \
                else part.vertices[vid.index]
            p = v.pointOn[0]
            pts.append((p[0], p[1],
                        math.hypot(p[0] - center[0], p[1] - center[1])))
        except Exception:
            continue
    is_circ = False
    try:
        cv = edge.getCurvature(0.5)
        k = tuple(cv['curvature'])[:2]
        is_circ = math.hypot(k[0], k[1]) > 1e-9
    except Exception:
        is_circ = False
    if len(pts) >= 2:
        r0, r1 = pts[0][2], pts[1][2]
        if abs(r0 - r1) < 1e-3:
            a0 = _point_angle_deg(center, pts[0])
            a1 = _point_angle_deg(center, pts[1])
            span = abs(_normalize_deg(a1 - a0))
            if span < 1e-6 or not is_circ:
                # closed edge with seam vertices, or imprinted arc whose
                # curvature query failed: equal radii => circle/arc
                span = span if span > 1e-6 else 360.0
            return {'is_circular': True, 'radius': 0.5 * (r0 + r1),
                    'span_deg': span, 'r_min': min(r0, r1),
                    'r_max': max(r0, r1)}
        return {'is_circular': False, 'radius': None, 'span_deg': None,
                'r_min': min(r0, r1), 'r_max': max(r0, r1)}
    if is_circ:
        try:
            p = edge.pointOn[0]
            r = math.hypot(p[0] - center[0], p[1] - center[1])
            return {'is_circular': True, 'radius': r, 'span_deg': 360.0,
                    'r_min': r, 'r_max': r}
        except Exception:
            return None
    if pts:
        rs = [p[2] for p in pts]
        return {'is_circular': False, 'radius': None, 'span_deg': None,
                'r_min': min(rs), 'r_max': max(rs)}
    return None


def _allocate_by_span(total, spans_deg):
    """Largest-remainder allocation: integer counts sum exactly to total."""
    tot = float(sum(spans_deg))
    if tot <= 0.0:
        raise ValueError('Arc span is 0; cannot assign circumferential seeds')
    exact = [total * s / tot for s in spans_deg]
    base = [int(math.floor(v + 1e-9)) for v in exact]
    rem = total - sum(base)
    order = sorted(range(len(exact)), key=lambda i: -(exact[i] - base[i]))
    k = 0
    while rem > 0 and k < len(order):
        base[order[k]] += 1
        rem -= 1
        k += 1
    for v in base:
        if v < 1:
            raise ValueError(
                'Got 0 segments on an arc (N=%d too small for existing '
                'partitions). Increase N or reduce radial partitions.' % total)
    if sum(base) != total:
        raise ValueError('Circumferential seed allocation mismatch: %d != %d' % (sum(base), total))
    return base


# Rect loop frame

def _loop_ordered_points(edges, part):
    """Order a closed planar edge loop into vertex points (robust)."""
    segs = []
    for e in edges:
        vs = e.getVertices()
        if len(vs) != 2:
            raise ValueError('Four-side edge loop must be straight segments')
        p0 = tuple(part.vertices[vs[0]].pointOn[0])
        p1 = tuple(part.vertices[vs[1]].pointOn[0])
        segs.append((p0, p1))
    if not segs:
        raise ValueError('Edge loop is empty: no straight segments resolved '
                         'from the picked edges')
    if len(segs) < 3:
        raise ValueError('Edge loop needs at least 3 edges, got %d' % len(segs))
    TOL = 1e-3
    pts = [segs[0][0], segs[0][1]]
    used = {0}
    while len(used) < len(segs):
        tail = pts[-1]
        found = False
        for i, (a, b) in enumerate(segs):
            if i in used:
                continue
            if _point_distance(a, tail) < TOL:
                pts.append(b)
                used.add(i)
                found = True
                break
            if _point_distance(b, tail) < TOL:
                pts.append(a)
                used.add(i)
                found = True
                break
        if not found:
            raise ValueError(
                'Edge loop is open: tail=%s unused=%d/%d' % (
                    tuple(round(x, 4) for x in tail),
                    len(segs) - len(used), len(segs)))
    if _point_distance(pts[0], pts[-1]) > TOL:
        raise ValueError('Edge loop ends do not meet')
    return pts[:-1]


def _vec_sub(a, b):
    return tuple(float(x) - float(y) for x, y in zip(a, b))


def _vec_add(a, b):
    return tuple(float(x) + float(y) for x, y in zip(a, b))


def _vec_scale(a, s):
    return tuple(float(x) * float(s) for x in a)


def _vec_dot(a, b):
    return sum(float(x) * float(y) for x, y in zip(a, b))


def _vec_norm(a):
    return math.sqrt(_vec_dot(a, a))


def _point_distance(a, b):
    return _vec_norm(_vec_sub(a, b))


def _vec_unit(a):
    n = _vec_norm(a)
    if n <= 1.0e-12:
        raise ValueError('Rectangle loop has a zero-length direction')
    return _vec_scale(a, 1.0 / n)


def _axis_sign(axis):
    """Deterministic sign in 2D/3D without assuming the plane is global XY."""
    for value in axis:
        if abs(value) > 1.0e-9:
            return -1.0 if value < 0.0 else 1.0
    return 1.0


def _rect_frame_from_loop(pts):
    """Validates right angles, opposite sides equal, and corner layout."""
    if len(pts) != 4:
        raise ValueError('Four-side mode needs a 4-edge loop, got %d points'
                         % len(pts))
    dim = len(pts[0])
    if any(len(p) != dim for p in pts) or dim not in (2, 3):
        raise ValueError('Rectangle loop points must all be 2D or 3D')
    center = tuple(sum(p[j] for p in pts) / 4.0 for j in range(dim))
    e0 = _vec_sub(pts[1], pts[0])
    e1 = _vec_sub(pts[2], pts[1])
    e2 = _vec_sub(pts[3], pts[2])
    e3 = _vec_sub(pts[0], pts[3])
    lens = [_vec_norm(v) for v in (e0, e1, e2, e3)]
    if min(lens) < 1e-9:
        raise ValueError('Edge loop has a zero-length edge')
    scale = max(lens)
    for i in range(4):
        if abs(lens[i] - lens[(i + 2) % 4]) > 1e-2 * scale:
            raise ValueError('Opposite sides are not equal (need a rectangle)')
    for a, b in ((e0, e1), (e1, e2), (e2, e3), (e3, e0)):
        dot = _vec_dot(a, b)
        if abs(dot) > 1e-2 * scale * scale:
            raise ValueError('Corner is not a right angle (need a rectangle)')
    ux = _vec_unit(e0)
    uy_raw = _vec_unit(e1)
    if _axis_sign(ux) < 0.0:
        ux = _vec_scale(ux, -1.0)
    # Pick the perpendicular direction that points toward the next rectangle
    # side.  Its sign is later aligned between inner/outer loops.
    uy = uy_raw
    loc = []
    for p in pts:
        rel = _vec_sub(p, center)
        loc.append((_vec_dot(rel, ux), _vec_dot(rel, uy)))
    xs = [p[0] for p in loc]
    ys = [p[1] for p in loc]
    hx = 0.5 * (max(xs) - min(xs))
    hy = 0.5 * (max(ys) - min(ys))
    for p in loc:
        if abs(abs(p[0]) - hx) > 1e-2 * max(hx, 1) \
                and abs(abs(p[1]) - hy) > 1e-2 * max(hy, 1):
            raise ValueError('Edge loop is not a rectangle (corner point missing)')
    frame = {'c': center, 'ux': ux, 'uy': uy, 'hx': hx, 'hy': hy}
    if dim == 3:
        # Coplanarity check and a stable normal for diagnostics/sketch setup.
        normal = _vec_unit((
            ux[1] * uy[2] - ux[2] * uy[1],
            ux[2] * uy[0] - ux[0] * uy[2],
            ux[0] * uy[1] - ux[1] * uy[0]))
        scale = max(hx, hy, 1.0)
        for p in pts:
            if abs(_vec_dot(_vec_sub(p, center), normal)) > 1.0e-6 * scale:
                raise ValueError('Four-side edge loop is not planar')
        frame['normal'] = normal
    return frame


def _build_rect_loop_frame(part, edges):
    """Ordered closed loop -> rectangle frame (spec section 7 helper)."""
    pts = _loop_ordered_points(edges, part)
    fr = _rect_frame_from_loop(pts)
    fr['pts'] = pts
    return fr


def _to_global(fr, u, v):
    return tuple(fr['c'][i] + u * fr['ux'][i] + v * fr['uy'][i]
                 for i in range(len(fr['c'])))


def _to_local(fr, p):
    rel = _vec_sub(tuple(p)[:len(fr['c'])], fr['c'])
    return (_vec_dot(rel, fr['ux']), _vec_dot(rel, fr['uy']))


def _axes_aligned(fr_a, fr_b, tol=1e-2):
    """Two frames share axes (allowing a swapped/negated orientation)."""
    ax, ay = fr_a['ux'], fr_a['uy']
    bx, by = fr_b['ux'], fr_b['uy']
    combos = (abs(_vec_dot(ax, bx)), abs(_vec_dot(ax, by)))
    return max(combos) > 1.0 - tol


def _align_frame_axes(fr_target, fr_ref, tol=1e-2):
    """If fr_target's u-axis is aligned with fr_ref's v-axis (swapped),"""
    ax, ay = fr_target['ux'], fr_target['uy']
    bx, by = fr_ref['ux'], fr_ref['uy']
    dot_uu = abs(_vec_dot(ax, bx))
    dot_uv = abs(_vec_dot(ax, by))
    if dot_uu >= 1.0 - tol:
        fr_target['ux'], fr_target['uy'] = bx, by
    elif dot_uv >= 1.0 - tol:
        fr_target['hx'], fr_target['hy'] = fr_target['hy'], fr_target['hx']
        fr_target['ux'], fr_target['uy'] = bx, by
    if 'pts' in fr_target:
        fr_target['local_pts'] = [_to_local(fr_target, p)
                                  for p in fr_target['pts']]


def _frame_center_distance(fr_a, fr_b):
    return _point_distance(fr_a['c'], fr_b['c'])


def _point3(point):
    point = tuple(point)
    if len(point) == 2:
        return (point[0], point[1], 0.0)
    return point[:3]


def _sketch_xy(plan, point):
    """Project a Part-space point through MakeSketchTransform.matrix()."""
    matrix = plan.get('_sketch_transform_matrix')
    if not matrix or len(matrix) != 12:
        raise RuntimeError('Sketch transform matrix is not available.')
    xaxis = matrix[0:3]
    yaxis = matrix[3:6]
    origin = matrix[9:12]
    rel = _vec_sub(_point3(point), origin)
    return (_vec_dot(rel, xaxis), _vec_dot(rel, yaxis))


# PLANAR_ANNULAR

def _build_annular_patch_plan(part, region_faces, fine_edges, coarse_edges,
                              ratio, fine_seeds, phase_deg=None,
                              domain_faces=None, stages=1, ratioSequence='',
                              stageWidths=''):
    """Validate annular inputs and build the polar patch plan (no mutations)."""
    ratio = int(ratio)
    N = int(fine_seeds)
    n_stages = int(stages) if stages else 1
    if n_stages < 1:
        raise ValueError('Stages must be >= 1.')
    import transition_mesh_plan as tm_plan
    chain = tm_plan.plan_counts(N, n_stages, ratio, ratioSequence)
    if ratio == 2:
        if N % 4 != 0:
            raise ValueError(_div_msg('Annular 2:1', N, 4))
        per = 4
    elif ratio == 3:
        if N % 3 != 0:
            raise ValueError(_div_msg('Annular 3:1', N, 3))
        per = 3
    else:
        raise ValueError('ratio must be 2 or 3, got %r' % ratio)
    n_period = N // per
    if n_period < 3:
        raise ValueError('Too few circumferential seeds: N=%d needs at least 3 template periods' % N)
    coarse_seeds = chain['counts'][-1]

    center, _r0 = _circle_from_edges(part, fine_edges)
    r_fine = _edge_radius(part, fine_edges[0], center)
    r_coarse = _edge_radius(part, coarse_edges[0], center)
    for e in fine_edges:
        if abs(_edge_radius(part, e, center) - r_fine) > 1e-2:
            raise ValueError('Fine loop edges are not concentric / same radius')
    for e in coarse_edges:
        if abs(_edge_radius(part, e, center) - r_coarse) > 1e-2:
            raise ValueError('Coarse loop edges are not concentric / same radius')
    if abs(r_fine - r_coarse) < 1e-6:
        raise ValueError('Fine and coarse radii are the same')

    h_deg = 360.0 / float(N)
    fine_verts = _loop_vertex_angles(part, fine_edges, center)
    coarse_verts = _loop_vertex_angles(part, coarse_edges, center)

    if phase_deg is None:
        base = None
        try:
            base = _point_angle_deg(center, tuple(fine_edges[0].pointOn[0])[:2])
        except Exception:
            base = None
        if fine_verts:
            if base is None:
                phase_deg = fine_verts[0]
            else:
                phase_deg = min(
                    fine_verts,
                    key=lambda a: abs(_normalize_deg(a - base)))
        elif base is not None:
            phase_deg = base
        else:
            phase_deg = 0.0

    # Sector-integer check first so N=180 / 2:1 / 4-quadrants yields the
    # actionable "22.5 coarse elements / use 176 or 184" message.
    _check_annular_partition_compat(plan_phase=phase_deg, N=N, ratio=ratio,
                                    fine_verts=fine_verts,
                                    coarse_verts=coarse_verts)
    # Existing radial partitions must sit on compatible template boundaries.
    tol_deg = 1e-3
    for a in fine_verts:
        res = _align_residual_deg(a, phase_deg, h_deg)
        if res > tol_deg:
            raise ValueError(
                'Fine-loop vertex at %.6f deg is not on the phase+%g deg '
                'grid (residual %.6f deg). Adjust N or the partitions.'
                % (a, h_deg, res))
    for a in coarse_verts:
        # created by IY,China University of Petroleum (East China)
        res = _align_residual_deg(a, phase_deg, ratio * h_deg)
        if res > tol_deg:
            raise ValueError(
                'Coarse-loop vertex at %.6f deg is not on the phase+%g deg '
                'grid (residual %.6f deg). Adjust N or the partitions.'
                % (a, ratio * h_deg, res))

    r_a = min(r_fine, r_coarse)
    r_b = max(r_fine, r_coarse)

    # Domain faces (spec 5.4): classify fine ring / band parents / coarse ring.
    domain = None
    if domain_faces:
        tol = 1e-2
        inner_loop = r_fine < r_coarse
        fine_pre = []
        coarse_pre = []
        band_parent = []
        bad = []
        for f in domain_faces:
            rr = _face_radius_range(part, f, center)
            if rr is None:
                bad.append(f)
                continue
            r0, r1 = rr
            if r1 - r0 <= 1e-9:
                bad.append(f)
                continue
            if inner_loop:
                if r1 <= r_fine + tol:
                    fine_pre.append((r0, r1, f))
                elif r0 >= r_coarse - tol:
                    coarse_pre.append((r0, r1, f))
                elif r0 <= r_fine + tol and r1 >= r_coarse - tol:
                    band_parent.append((r0, r1, f))
                else:
                    bad.append(f)
            else:
                if r0 >= r_fine - tol:
                    fine_pre.append((r0, r1, f))
                elif r1 <= r_coarse + tol:
                    coarse_pre.append((r0, r1, f))
                elif r0 <= r_coarse + tol and r1 >= r_fine - tol:
                    band_parent.append((r0, r1, f))
                else:
                    bad.append(f)
        if bad:
            raise ValueError(
                'Domain pick crosses the transition boundary on %d Face(s). '
                'Repick the full domain so each Face stays inside one region.'
                % len(bad))
        if not band_parent:
            raise ValueError('Domain pick must cover the transition band Face')
        if inner_loop:
            Rf_far = min(q[0] for q in fine_pre + band_parent) \
                if (fine_pre or band_parent) else r_fine
            Rc_far = max(q[1] for q in coarse_pre + band_parent) \
                if (coarse_pre or band_parent) else r_coarse
        else:
            Rf_far = max(q[1] for q in fine_pre + band_parent) \
                if (fine_pre or band_parent) else r_fine
            Rc_far = min(q[0] for q in coarse_pre + band_parent) \
                if (coarse_pre or band_parent) else r_coarse
        domain = {'fine_pre': fine_pre, 'coarse_pre': coarse_pre,
                  'band_parent': [q[2] for q in band_parent],
                  'Rf_far': Rf_far, 'Rc_far': Rc_far,
                  'inner_loop': inner_loop}

    # Target face(s) covering the annular band.
    targets = []
    for f in (region_faces or list(part.faces)):
        rr = _face_radius_range(part, f, center)
        if rr is None:
            continue
        r0, r1 = rr
        if r0 <= r_a + 1e-2 and r1 >= r_b - 1e-2 and r1 - r0 > 1e-3:
            targets.append(f)
    if not targets:
        raise ValueError('No Face covers the transition band')
    if domain and domain['band_parent']:
        targets = list(domain['band_parent'])

    patch = build_transition_patch(ratio, fine_spacing=h_deg, depth=1.0)
    stage_specs = []
    if n_stages > 1:
        # Intermediate radii from fine→coarse using stage widths (or auto).
        span = abs(r_coarse - r_fine)
        sign = 1.0 if r_coarse > r_fine else -1.0
        bounds = tm_plan.stage_boundaries(span, chain['counts'], stageWidths or None)
        for i in range(n_stages):
            ri = r_fine + sign * bounds[i]
            ro = r_fine + sign * bounds[i + 1]
            ni = chain['counts'][i]
            ri_ratio = chain['ratios'][i]
            per_i = 4 if ri_ratio == 2 else 3
            if ni % per_i != 0:
                raise ValueError(
                    'Stage %d fine count %d is not divisible by the %d:1 period %d.'
                    % (i + 1, ni, ri_ratio, per_i))
            h_i = 360.0 / float(ni)
            stage_specs.append({
                'index': i, 'ratio': int(ri_ratio), 'fine_seeds': ni,
                'coarse_seeds': chain['counts'][i + 1],
                'r_in': ri, 'r_out': ro, 'per': per_i, 'h_deg': h_i,
                'n_period': ni // per_i,
                'patch': build_transition_patch(int(ri_ratio),
                                                fine_spacing=h_i, depth=1.0),
            })
    return {
        'label': 'annular', 'mode': 'PLANAR_ANNULAR', 'ratio': ratio,
        'fine_seeds': N, 'coarse_seeds': coarse_seeds,
        'n_period': n_period, 'period_segments': per, 'h_deg': h_deg,
        'patch': patch, 'center': center, 'r_fine': r_fine,
        'r_coarse': r_coarse, 'phase_deg': float(phase_deg),
        'fine_verts': fine_verts, 'coarse_verts': coarse_verts,
        'target_faces': targets, 'domain': domain,
        'stages': n_stages, 'stage_counts': chain['counts'],
        'stage_ratios': chain['ratios'], 'stage_specs': stage_specs,
        'sketch_name': '__tm2d_ann__', 'sheet_size': 4.0 * max(r_b, 1.0),
        'sketch_origin': (center[0], center[1], 0.0),
        'model': _owner_model(part),
    }


def _draw_annular_patch(sk, plan):
    """Draw every template period explicitly (sketch origin = circle center)."""
    def SP(r, deg):
        a = math.radians(deg)
        return (r * math.sin(a), r * math.cos(a))

    def draw_one_patch(patch, r_fine, r_coarse, phase, h_deg, per, n_period):
        def ppt(name, ip):
            u, v = patch['vertices'][name]
            r = r_fine + v * (r_coarse - r_fine)
            deg = phase + u + ip * per * h_deg
            return SP(r, deg)
        for ip in range(int(n_period)):
            for a, b in patch_unique_edges(patch):
                pa, pb = ppt(a, ip), ppt(b, ip)
                va = patch['vertices'][a][1]
                vb = patch['vertices'][b][1]
                ua = patch['vertices'][a][0]
                ub = patch['vertices'][b][0]
                if abs(va - vb) < 1e-9 and abs(ua - ub) > 1e-12:
                    r = r_fine + va * (r_coarse - r_fine)
                    um = 0.5 * (ua + ub)
                    dm = phase + um + ip * per * h_deg
                    sk.Arc3Points(point1=pa, point2=pb, point3=SP(r, dm))
                else:
                    sk.Line(point1=pa, point2=pb)

    if plan.get('stages') and int(plan['stages']) > 1:
        for stage in plan['stage_specs']:
            draw_one_patch(stage['patch'], stage['r_in'], stage['r_out'],
                           plan['phase_deg'], stage['h_deg'],
                           stage['per'], stage['n_period'])
        return
    patch = plan['patch']
    draw_one_patch(patch, plan['r_fine'], plan['r_coarse'], plan['phase_deg'],
                   plan['h_deg'], plan['period_segments'], plan['n_period'])


def _annular_band_finder(part, plan):
    r_fine = plan['r_fine']
    r_coarse = plan['r_coarse']
    r_a, r_b = min(r_fine, r_coarse), max(r_fine, r_coarse)
    band = []
    for i in range(len(part.faces)):
        rr = _face_radius_range(part, part.faces[i], plan['center'])
        if rr is None:
            continue
        r0, r1 = rr
        if r0 >= r_a - 1e-2 and r1 <= r_b + 1e-2 and (r1 - r0) > 1e-6:
            band.append(i)
    return band


def _resolve_annular_regions(part, plan):
    """Post-partition fine ring / coarse ring faces (only when domain given)."""
    dom = plan['domain']
    if not dom:
        return None
    tol = 1e-2
    r_fine = plan['r_fine']
    r_coarse = plan['r_coarse']
    inner_loop = dom['inner_loop']
    fine, coarse = [], []
    for i in range(len(part.faces)):
        rr = _face_radius_range(part, part.faces[i], plan['center'])
        if rr is None:
            continue
        r0, r1 = rr
        if r1 - r0 <= 1e-9:
            continue
        if inner_loop:
            if r1 <= r_fine + tol and r0 >= dom['Rf_far'] - tol:
                fine.append(i)
            elif r0 >= r_coarse - tol and r1 <= dom['Rc_far'] + tol:
                coarse.append(i)
        else:
            if r0 >= r_fine - tol and r1 <= dom['Rf_far'] + tol:
                fine.append(i)
            elif r1 <= r_coarse + tol and r0 >= dom['Rc_far'] - tol:
                coarse.append(i)
    return {'fine': fine, 'coarse': coarse}


def _seed_annular_rings(part, plan, rings):
    """Seed regular rings: circles follow N / N//ratio, radials by spacing."""
    if not rings:
        return None
    N = plan['fine_seeds']
    Nc = plan['coarse_seeds']
    r_fine = plan['r_fine']
    r_coarse = plan['r_coarse']
    tol = 1e-2
    stats = {'fine_faces': len(rings['fine']),
             'coarse_faces': len(rings['coarse']),
             'fine_edges': 0, 'coarse_edges': 0, 'circles': {}}
    for kind, idxs in (('fine', rings['fine']), ('coarse', rings['coarse'])):
        if not idxs:
            continue
        target = N if kind == 'fine' else Nc
        circles = {}
        radial = []
        seen = set()
        for fi in idxs:
            for e in part.faces[fi].getEdges():
                ei = _as_index(e)
                if ei in seen:
                    continue
                seen.add(ei)
                info = _edge_arc_info(part, part.edges[ei], plan['center'])
                if info is None:
                    continue
                if info['is_circular']:
                    r = info['radius']
                    if abs(r - r_fine) < tol or abs(r - r_coarse) < tol:
                        continue  # template interface arcs stay 1 segment
                    circles.setdefault(round(r, 3), []).append(
                        (ei, info['span_deg']))
                else:
                    radial.append((ei, info['r_min'], info['r_max']))
        for key in sorted(circles):
            arcs = circles[key]
            counts = _allocate_by_span(target, [s for (_, s) in arcs])
            for (ei, _s), cnt in zip(arcs, counts):
                part.seedEdgeByNumber(edges=(part.edges[ei],),
                                      number=int(cnt), constraint=FIXED)
                if kind == 'fine':
                    stats['fine_edges'] += 1
                else:
                    stats['coarse_edges'] += 1
            stats['circles']['%s_r%g' % (kind, key)] = {
                'total': int(sum(counts)), 'arcs': len(arcs)}
        for (ei, r0, r1) in radial:
            if r1 - r0 < 1e-4:
                # safety net: an edge with (nearly) equal endpoint radii is a
                # circle/arc that slipped through; seed it as a full circle
                part.seedEdgeByNumber(edges=(part.edges[ei],),
                                      number=int(target), constraint=FIXED)
                if kind == 'fine':
                    stats['fine_edges'] += 1
                else:
                    stats['coarse_edges'] += 1
                stats['circles']['%s_r%g' % (kind, round(0.5 * (r0 + r1), 3))] = {
                    'total': int(target), 'arcs': 1}
                continue
            r_mid = 0.5 * (r0 + r1)
            h = 2.0 * math.pi * r_mid / float(target)
            rows = max(1, int(round((r1 - r0) / h))) if h > 0 else 1
            part.seedEdgeByNumber(edges=(part.edges[ei],), number=rows,
                                  constraint=FIXED)
            if kind == 'fine':
                stats['fine_edges'] += 1
            else:
                stats['coarse_edges'] += 1
    return stats


def _verify_annular_interfaces(part, plan):
    """Sum of seeded segments on the fine / coarse circles must match N."""
    tol = 1e-2
    got = {'fine': 0, 'coarse': 0}
    for ei in range(len(part.edges)):
        info = _edge_arc_info(part, part.edges[ei], plan['center'])
        if not info or not info['is_circular']:
            continue
        s = _edge_seed_count(part, part.edges[ei]) or 0
        if abs(info['radius'] - plan['r_fine']) < tol:
            got['fine'] += s
        elif abs(info['radius'] - plan['r_coarse']) < tol:
            got['coarse'] += s
    ok = (got['fine'] == plan['fine_seeds']
          and got['coarse'] == plan['coarse_seeds'])
    return {'fine_actual': got['fine'], 'coarse_actual': got['coarse'],
            'fine_expected': plan['fine_seeds'],
            'coarse_expected': plan['coarse_seeds'], 'ok': ok}


def apply_planar_annular_transition(
        part, region_faces, fine_edges, coarse_edges,
        ratio=2, fine_seeds=40, action='PARTITION_SEED_MESH',
        element2d='CPS4R', phase_deg=None, domain_faces=None,
        mesh_regular=None, stages=1, ratioSequence='', stageWidths=''):
    """fine_seeds = full-circle fine count N."""
    if action not in _ACTIONS:
        raise ValueError('Illegal action %r' % (action,))
    _require_2d(part)
    fine_edges = [e for e in (fine_edges or []) if e is not None]
    coarse_edges = [e for e in (coarse_edges or []) if e is not None]
    if not fine_edges or not coarse_edges:
        raise ValueError('Pick both fine and coarse edge loops')
    domain_faces = [f for f in (domain_faces or []) if f is not None]
    if mesh_regular is False:
        domain_faces = []

    plan = _build_annular_patch_plan(
        part, region_faces, fine_edges, coarse_edges, ratio, fine_seeds,
        phase_deg=phase_deg, domain_faces=domain_faces,
        stages=stages, ratioSequence=ratioSequence, stageWidths=stageWidths)
    N = plan['fine_seeds']
    n_stages = int(plan.get('stages') or 1)
    if n_stages > 1:
        expect_faces = 0
        for st in plan['stage_specs']:
            expect_faces += expected_transition_elements(st['ratio'],
                                                         st['fine_seeds'])
    else:
        expect_faces = expected_transition_elements(plan['ratio'], N)

    part.backup()
    try:
        _partition_patch_plan(part, plan, _draw_annular_patch)
        band = _annular_band_finder(part, plan)
        if not band:
            raise ValueError('No transition-band Face found after partition')
        rings = _resolve_annular_regions(part, plan)
        if plan['domain']:
            _dbg('[p2d-ann] domain Rf_far=%g Rc_far=%g rings=%r'
                 % (plan['domain']['Rf_far'], plan['domain']['Rc_far'],
                    None if rings is None else
                    (len(rings['fine']), len(rings['coarse']))))

        result = {
            'mode': 'PLANAR_ANNULAR', 'ratio': plan['ratio'],
            'fine_seeds': N, 'coarse_seeds': plan['coarse_seeds'],
            'n_periods': plan['n_period'], 'template_periods': plan['n_period'],
            'fine_interface_segments': N,
            'coarse_interface_segments': plan['coarse_seeds'],
            'expected_template_faces': expect_faces,
            'band_faces': len(band), 'r_fine': round(plan['r_fine'], 6),
            'r_coarse': round(plan['r_coarse'], 6), 'action': action,
            'element2d': str(element2d),
            'phase_deg': round(plan['phase_deg'], 4),
            'stages': n_stages,
            'stage_counts': plan.get('stage_counts') or [N, plan['coarse_seeds']],
            'stage_ratios': plan.get('stage_ratios') or [plan['ratio']],
        }
        if rings is not None:
            result['domain'] = {'fine_ring_faces': len(rings['fine']),
                                'coarse_ring_faces': len(rings['coarse'])}
        _dbg('[p2d-ann] band_faces=%d expect=%d n_period=%d phase=%g'
             % (len(band), expect_faces, plan['n_period'], plan['phase_deg']))
        if action == 'PARTITION_ONLY':
            result.update(_audit_patch_mesh(part, band))
            result['faces_meshed'] = 0
            return result

        result['template_edges_seeded'] = _seed_patch_plan(part, band)
        ring_stats = _seed_annular_rings(part, plan, rings)
        if ring_stats:
            result['ring_seeding'] = ring_stats
        iface = _verify_annular_interfaces(part, plan)
        result['interface_counts'] = iface
        result['compatibility'] = 'OK' if iface['ok'] else 'INTERFACE_MISMATCH'
        _dbg('[p2d-ann] seeded_edges=%d iface=%r'
             % (result['template_edges_seeded'], iface))
        if not iface['ok']:
            raise ValueError('Fine/coarse interface seed counts do not match the plan: %r' % (iface,))

        if action == 'PARTITION_AND_SEED':
            result.update(_audit_patch_mesh(part, band))
            result['faces_meshed'] = 0
            return result

        ctrl, mesh_err, audit = _mesh_patch_plan(part, band, element2d)
        result['mesh_control'] = ctrl
        result.update(audit)
        _dbg('[p2d-ann] mesh_ctrl=%s audit=%s' % (ctrl, audit))
        if audit['elements'] <= 0:
            raise ValueError('Annular mesh produced 0 elements (mesh_err=%s)' % mesh_err)
        if audit['unmeshed_faces'] != 0:
            raise ValueError('Annular still has %d unmeshed Face(s)'
                             % audit['unmeshed_faces'])
        result['faces_meshed'] = len(band)
        result['expected_elements'] = expect_faces

        if rings is not None and (rings['fine'] or rings['coarse']):
            records = {}
            total = audit['elements']
            for kind in ('fine', 'coarse'):
                idxs = rings[kind]
                if not idxs:
                    continue
                rctrl, rerr, raudit = _mesh_patch_plan(part, idxs, element2d,
                                                       prefer_sweep=True)
                records[kind] = {
                    'control': rctrl, 'elements': raudit['elements'],
                    'tri': raudit['tri'],
                    'unmeshed_faces': raudit['unmeshed_faces'],
                    'quality': {k: raudit.get(k) for k in
                                ('duplicate_elements', 'negative_area',
                                 'zero_area', 'bowtie', 'hanging_nodes')
                                if k in raudit},
                }
                _dbg('[p2d-ann] ring %s ctrl=%s audit=%s'
                     % (kind, rctrl, raudit))
                if raudit['elements'] <= 0:
                    raise ValueError('Regular ring (%s) mesh failed (mesh_err=%s)'
                                     % (kind, rerr))
                if raudit['unmeshed_faces'] != 0:
                    raise ValueError('Regular ring (%s) still has %d unmeshed Face(s)'
                                     % (kind, raudit['unmeshed_faces']))
                total += raudit['elements']
            result['rings_mesh'] = records
            result['elements_total'] = total
        return result
    except BaseException as exc:
        try:
            part.restore()
        except BaseException as rex:
            raise RuntimeError('Annular failed: %r; restore: %r' % (exc, rex))
        raise


# PLANAR_QUAD_FOUR

def _face_local_bbox(part, face, fr):
    """Local-frame bbox of a face boundary (None when un-samplable)."""
    us, vs = [], []
    try:
        for e in face.getEdges():
            edge = part.edges[_as_index(e)]
            try:
                p = edge.pointOn[0]
                u, v = _to_local(fr, p)
                us.append(u)
                vs.append(v)
            except Exception:
                pass
        for vid in face.getVertices():
            try:
                p = part.vertices[vid].pointOn[0]
                u, v = _to_local(fr, p)
                us.append(u)
                vs.append(v)
            except Exception:
                pass
    except Exception:
        pass
    if not us:
        return None
    return (min(us), max(us), min(vs), max(vs))


def _rect_outer_covered(part, faces, fr, umin, umax, vmin, vmax, tol=1e-2):
    """Check the domain outer boundary is the full local-frame rectangle."""
    need = {'vmax': umax - umin, 'vmin': umax - umin,
            'umax': vmax - vmin, 'umin': vmax - vmin}
    got = {'vmax': 0.0, 'vmin': 0.0, 'umax': 0.0, 'umin': 0.0}
    seen = set()
    for f in faces:
        try:
            edges = list(f.getEdges())
        except Exception:
            continue
        for e in edges:
            ei = _as_index(e)
            if ei in seen:
                continue
            seen.add(ei)
            edge = part.edges[ei]
            try:
                vs = edge.getVertices()
                if len(vs) != 2:
                    continue
                p0 = part.vertices[vs[0]].pointOn[0]
                p1 = part.vertices[vs[1]].pointOn[0]
            except Exception:
                continue
            u0, v0 = _to_local(fr, p0)
            u1, v1 = _to_local(fr, p1)
            if abs(v0 - v1) < tol and abs(v0 - vmax) < tol:
                got['vmax'] += abs(u1 - u0)
            elif abs(v0 - v1) < tol and abs(v0 - vmin) < tol:
                got['vmin'] += abs(u1 - u0)
            elif abs(u0 - u1) < tol and abs(u0 - umax) < tol:
                got['umax'] += abs(v1 - v0)
            elif abs(u0 - u1) < tol and abs(u0 - umin) < tol:
                got['umin'] += abs(v1 - v0)
    scale = max(umax - umin, vmax - vmin, 1.0)
    for k in need:
        if abs(got[k] - need[k]) > 1e-2 * scale:
            return False
    return True


def _rect_face_role(plan, bb, tol=1e-2):
    """Classify a post-partition face inside the local frame."""
    u0, u1, v0, v1 = bb
    hxi, hyi = plan['hxi'], plan['hyi']
    H, V = plan['H'], plan['V']
    ext = plan['external']
    in_model = (ext is None
                or (u0 >= ext['umin'] - tol and u1 <= ext['umax'] + tol
                    and v0 >= ext['vmin'] - tol and v1 <= ext['vmax'] + tol))
    if not in_model:
        return None
    if u0 >= -hxi - tol and u1 <= hxi + tol \
            and v0 >= -hyi - tol and v1 <= hyi + tol:
        return 'center'
    if u0 >= -H - tol and u1 <= H + tol and v0 >= -V - tol and v1 <= V + tol:
        return 'band'
    if v0 >= V - tol and u0 >= -H - tol and u1 <= H + tol:
        return 'ext_top'
    if v1 <= -V + tol and u0 >= -H - tol and u1 <= H + tol:
        return 'ext_bottom'
    if u0 >= H - tol and v0 >= -V - tol and v1 <= V + tol:
        return 'ext_right'
    if u1 <= -H + tol and v0 >= -V - tol and v1 <= V + tol:
        return 'ext_left'
    if u1 <= -H + tol and v0 >= V - tol:
        return 'ext_tl'
    if u0 >= H - tol and v0 >= V - tol:
        return 'ext_tr'
    if u1 <= -H + tol and v1 <= -V + tol:
        return 'ext_bl'
    if u0 >= H - tol and v1 <= -V + tol:
        return 'ext_br'
    return None


def _rect_ext_edge_count(plan, u0, u1, v0, v1, tol=1e-2):
    """After interface-node extension the external region is split into single-column / single-row"""
    ext = plan['external']
    H, V = plan['H'], plan['V']
    k_l = ext['k_left']
    k_r = ext['k_right']
    k_t = ext['k_top']
    k_b = ext['k_bottom']
    if abs(v1 - v0) < tol:  # horizontal
        vv = 0.5 * (v0 + v1)
        uc = 0.5 * (u0 + u1)
        if abs(vv - V) < tol or abs(vv + V) < tol:
            if abs(uc) <= H + tol:
                return None       # transition outer: already seeded
            return k_l if uc < 0 else k_r   # corner width direction
        if abs(vv - ext['vmax']) < tol or abs(vv - ext['vmin']) < tol:
            if abs(uc) <= H + tol:
                return 1          # one column of the top/bottom strip
            return k_l if uc < 0 else k_r   # corner far edge
        # Interior horizontal separators live in the left/right strips.
        if abs(u0 + H) < tol or abs(u1 + H) < tol or abs(u0 - H) < tol \
                or abs(u1 - H) < tol or abs(u0 - ext['umin']) < tol \
                or abs(u1 - ext['umin']) < tol or abs(u0 - ext['umax']) < tol \
                or abs(u1 - ext['umax']) < tol:
            return k_l if uc < 0 else k_r
        return 1
    if abs(u1 - u0) < tol:  # vertical
        uu = 0.5 * (u0 + u1)
        vc = 0.5 * (v0 + v1)
        if abs(uu - H) < tol or abs(uu + H) < tol:
            if abs(vc) <= V + tol:
                return None       # transition outer: already seeded
            return k_t if vc > 0 else k_b
        if abs(uu - ext['umax']) < tol or abs(uu - ext['umin']) < tol:
            if abs(vc) <= V + tol:
                return 1          # one row of the left/right strip
            return k_t if vc > 0 else k_b
        # Interior vertical separators live in the top/bottom strips.
        if abs(v0 - V) < tol or abs(v1 - V) < tol or abs(v0 + V) < tol \
                or abs(v1 + V) < tol or abs(v0 - ext['vmin']) < tol \
                or abs(v1 - ext['vmin']) < tol or abs(v0 - ext['vmax']) < tol \
                or abs(v1 - ext['vmax']) < tol:
            return k_t if vc > 0 else k_b
        return 1
    return None


def quad_four_rule(ratio, Nx, Ny, hxi, hyi, layers):
    """Thin re-export: the arithmetic lives in ``transition_mesh_rules`` so the dialog,"""
    return tm_rules.quad_four_rule(ratio, Nx, Ny, hxi, hyi, layers)


def _build_rect_four_patch_plan(part, region_faces, fine_edges, coarse_edges,
                                ratio, Nx, Ny, domain_faces=None,
                                mesh_regular=False, stages=1,
                                ratioSequence='', stageWidths=''):
    """``coarse_edges`` may be empty:"""
    ratio = int(ratio)
    if ratio == 2:
        from transition_mesh_quad2_cae import build_plan
        return build_plan(part, region_faces, fine_edges, coarse_edges,
                          int(Nx), int(Ny), domain_faces, mesh_regular, stages,
                          ratioSequence, stageWidths)
    Nx = int(Nx)
    Ny = int(Ny)
    n_stages = int(stages) if stages else 1
    if n_stages < 1:
        raise ValueError('Stages must be >= 1.')
    import transition_mesh_plan as tm_plan
    # Four-side multi-stage includes +2 corner contribution at each outer.
    if n_stages > 1:
        chain_x = tm_plan.plan_quad_counts(Nx, n_stages, ratio, ratioSequence)
        chain_y = tm_plan.plan_quad_counts(Ny, n_stages, ratio, ratioSequence)
    else:
        chain_x = tm_plan.plan_counts(Nx, n_stages, ratio, ratioSequence)
        chain_y = tm_plan.plan_counts(Ny, n_stages, ratio, ratioSequence)
    if ratio == 2:
        per = 4
    elif ratio == 3:
        per = 3
    else:
        raise ValueError('ratio must be 2 or 3, got %r' % ratio)
    if Nx % per != 0:
        raise ValueError(_div_msg('Nx', Nx, per))
    if Ny % per != 0:
        raise ValueError(_div_msg('Ny', Ny, per))
    if Nx // per < 1 or Ny // per < 1:
        raise ValueError('Four-side transition needs at least %d fine segments per side' % per)

    fr_in = _build_rect_loop_frame(part, fine_edges)
    auto_coarse = not [e for e in (coarse_edges or []) if e is not None]
    rule = quad_four_rule(ratio, Nx, Ny, fr_in['hx'], fr_in['hy'], n_stages)
    if auto_coarse:
        # No coarse loop picked: synthesise the outer frame from rule (4).
        # The fitting check happens later when the covering Face is found.
        # Vertex points are 3D even on planar parts; keep the full centre.
        fr_out = {'c': fr_in['c'], 'ux': fr_in['ux'], 'uy': fr_in['uy'],
                  'hx': rule['H_required'], 'hy': rule['V_required']}
        H, V = rule['H_required'], rule['V_required']
        hxi, hyi = fr_in['hx'], fr_in['hy']
    else:
        fr_out = _build_rect_loop_frame(part, coarse_edges)
        if _frame_center_distance(fr_in, fr_out) > 1e-2:
            raise ValueError('Inner and outer rectangle centers must match')
        if not _axes_aligned(fr_in, fr_out):
            raise ValueError('Inner and outer rectangle sides must be parallel')
        # Align inner frame axes to outer frame orientation.
        # The loop builder picks u from the first edge,
        _align_frame_axes(fr_in, fr_out)
        hxi, hyi = fr_in['hx'], fr_in['hy']
        H, V = fr_out['hx'], fr_out['hy']
    dx = H - hxi
    dy = V - hyi
    if dx <= 1e-6 or dy <= 1e-6:
        # Direction check: the transition is built from the inner (fine) loop outward.
        raise ValueError(
            'PLANAR_QUAD_FOUR builds the transition from the inner loop '
            'outward, so Fine must be the INNER loop and Coarse the OUTER '
            'one. The current picks are reversed (fine outside: '
            'fine half-size=(%.6g, %.6g), coarse half-size=(%.6g, %.6g)). '
            'Swap the two picks, or use PLANAR_ANNULAR for a ring.'
            % (hxi, hyi, H, V))
    # Rule (4) compliance of a hand-picked outer loop.
    rule_dev = None
    if not auto_coarse:
        rule_dev = tm_rules.layer_rule_deviation(rule, H, V)
    rule['deviation'] = rule_dev

    # Outer loop counts are derived from side-template coarse ports plus an explicit +1 corner contribution on
    # each end (strict 2:1 participating corners; 3:1 keeps the same numeric contract with legacy corners).
    hcx = 2.0 * hxi / float(Nx) * ratio   # coarse spacing, u direction
    hcy = 2.0 * hyi / float(Ny) * ratio
    corner_contrib = 1
    nx_coarse = Nx // ratio + 2 * corner_contrib
    ny_coarse = Ny // ratio + 2 * corner_contrib
    # Tangential interface node coordinates; external strips extend them to the
    # domain boundary so the far edge matches the transition outer edge.
    n_u = Nx // ratio
    n_v = Ny // ratio
    u_iface = [-hxi + j * hcx for j in range(n_u + 1)]
    v_iface = [-hyi + j * hcy for j in range(n_v + 1)]
    # Case rule (2)/(3): the corner is ONE plain coarse element and does not participate in the merge (the
    # participating 2:1 corner made the ring look wrong against the reference case).
    corner_participating = False
    # Multi-stage outermost count already includes +2 corners in plan_quad_counts.
    if n_stages > 1:
        nx_coarse = chain_x['counts'][-1]
        ny_coarse = chain_y['counts'][-1]
    stage_frames = []
    if n_stages > 1:
        # With an auto outer loop the widths come from rule (4) per direction;
        # a hand-picked loop keeps the user's explicit widths or the proportional default.
        wu = wv = None
        if auto_coarse and not str(stageWidths or '').strip():
            wu = rule['width_u']
            wv = rule['width_v']
        bu = tm_plan.stage_boundaries(dx, chain_x['counts'],
                                      wu or (stageWidths or None))
        bv = tm_plan.stage_boundaries(dy, chain_y['counts'],
                                      wv or (stageWidths or None))
        for i in range(n_stages):
            nx_i = chain_x['counts'][i]
            ny_i = chain_y['counts'][i]
            ri = chain_x['ratios'][i]
            per_i = 4 if ri == 2 else 3
            strip_x = nx_i if i == 0 else max(1, nx_i - 2)
            strip_y = ny_i if i == 0 else max(1, ny_i - 2)
            if strip_x % per_i or strip_y % per_i:
                raise ValueError(
                    'Stage %d strip counts %d/%d not divisible by %d:1 period %d.'
                    % (i + 1, strip_x, strip_y, ri, per_i))
            hxi_i = hxi + bu[i]
            H_i = hxi + bu[i + 1]
            hyi_i = hyi + bv[i]
            V_i = hyi + bv[i + 1]
            # Strips must share their terminal edges with the corner boxes.
            # For stage 0 the inner interface is the user's fine loop.
            if i == 0:
                hxi_strip, hyi_strip = hxi, hyi
                hxi_corner, hyi_corner = hxi, hyi
                n_strip_x, n_strip_y = strip_x, strip_y
            else:
                prev = stage_frames[i - 1]
                hxi_corner, hyi_corner = prev['H'], prev['V']
                hxi_strip, hyi_strip = hxi_corner, hyi_corner
                n_strip_x, n_strip_y = strip_x, strip_y
            sf = 2.0 * hxi_strip / float(n_strip_x)
            sgy = 2.0 * hyi_strip / float(n_strip_y)
            stage_frames.append({
                'index': i, 'ratio': int(ri),
                'Nx': nx_i, 'Ny': ny_i, 'per': per_i,
                'n_strip_x': n_strip_x, 'n_strip_y': n_strip_y,
                'hxi': hxi_i, 'hyi': hyi_i, 'H': H_i, 'V': V_i,
                'hxi_strip': hxi_strip, 'hyi_strip': hyi_strip,
                'hxi_corner': hxi_corner, 'hyi_corner': hyi_corner,
                # Every 2:1 stage uses the 5-quad participating corner so
                # the strip terminal and corner inner port share one edge.
                'corner_participating': (ri == 2),
                'patch_x': build_transition_patch(
                    int(ri), fine_spacing=sf, depth=(V_i - hyi_corner)),
                'patch_y': build_transition_patch(
                    int(ri), fine_spacing=sgy, depth=(H_i - hxi_corner)),
            })

    # External frame detection (only when a full domain selection is given AND mesh_regular is on -- otherwise
    # the domain must not be partitioned).
    sep_u, sep_v = u_iface, v_iface
    hcx_out, hcy_out = hcx, hcy
    if n_stages > 1 and stage_frames:
        last = stage_frames[-1]
        n_mid_u = max(1, int(last['n_strip_x']) // max(1, int(last['ratio'])))
        n_mid_v = max(1, int(last['n_strip_y']) // max(1, int(last['ratio'])))
        hcx_out = 2.0 * last['hxi_corner'] / n_mid_u
        hcy_out = 2.0 * last['hyi_corner'] / n_mid_v
        sep_u = [-last['hxi_corner'] + j * hcx_out
                 for j in range(n_mid_u + 1)]
        sep_v = [-last['hyi_corner'] + j * hcy_out
                 for j in range(n_mid_v + 1)]
    external = None
    external_note = 'none'
    if domain_faces and mesh_regular:
        us, vs = [], []
        for f in domain_faces:
            bb = _face_local_bbox(part, f, fr_in)
            if bb is None:
                continue
            us.extend([bb[0], bb[1]])
            vs.extend([bb[2], bb[3]])
        if not us:
            raise ValueError('Cannot resolve local coordinates of the domain Face pick')
        umin, umax = min(us), max(us)
        vmin, vmax = min(vs), max(vs)
        if not _rect_outer_covered(part, domain_faces, fr_in,
                                   umin, umax, vmin, vmax):
            raise ValueError(
                'Domain outer boundary is not an axis-aligned rectangle. '
                'Pre-partition holes or complex outlines.')
        extents = (-umin - H, umax - H, -vmin - V, vmax - V)
        if min(extents) > 1e-2:
            external = {
                'umin': umin, 'umax': umax, 'vmin': vmin, 'vmax': vmax,
                'k_left': max(1, int(round((-umin - H) / hcx_out))),
                'k_right': max(1, int(round((umax - H) / hcx_out))),
                'k_top': max(1, int(round((vmax - V) / hcy_out))),
                'k_bottom': max(1, int(round((-vmin - V) / hcy_out))),
                'sep_u': list(sep_u), 'sep_v': list(sep_v),
                'u_nodes': [umin] + list(u_iface) + [umax],
                'v_nodes': [vmin] + list(v_iface) + [vmax],
            }
            external_note = 'frame'
        elif max(extents) > 1e-2:
            external_note = 'partial_skipped'

    # Target faces: with a full domain selection partition every domain face in
    # one call, otherwise only the face covering the outer rectangle.
    if auto_coarse:
        # The auto coarse loop must fit inside the picked geometry: a partition can never enlarge a Face,
        # so report the numbers and the ways out instead of a bare 'no face covers' error.
        src = list(domain_faces or []) or list(region_faces or []) \
            or list(part.faces)
        us, vs = [], []
        for f in src:
            bb = _face_local_bbox(part, f, fr_in)
            if bb is None:
                continue
            us.extend([bb[0], bb[1]])
            vs.extend([bb[2], bb[3]])
        slack = 1e-4 * max(H, V, 1.0)
        if us and (min(us) > -H + slack or max(us) < H - slack
                   or min(vs) > -V + slack or max(vs) < V - slack):
            raise ValueError(
                'Auto coarse loop needs half-sizes H=%.6g V=%.6g (case rule '
                '4: side + 2 x coarse element) but the picked Face only '
                'spans U=(%.6g, %.6g) V=(%.6g, %.6g). Enlarge the part, '
                'lower Layers, increase Seeds, or pick the coarse loop '
                'yourself.'
                % (H, V, min(us), max(us), min(vs), max(vs)))
    if domain_faces and mesh_regular:
        targets = list(domain_faces)
    else:
        targets = []
        for f in (region_faces or list(part.faces)):
            bb = _face_local_bbox(part, f, fr_in)
            if bb is None:
                continue
            u0, u1, v0, v1 = bb
            if (u0 <= -H + 1e-2 and u1 >= H - 1e-2
                    and v0 <= -V + 1e-2 and v1 >= V - 1e-2):
                targets.append(f)
        if not targets:
            raise ValueError(
                'No Face covers the outer rectangle (half-sizes %.6g x '
                '%.6g). Pick the transition Face, or the Full domain.'
                % (H, V))

    plan = {
        'label': 'quad-four', 'mode': 'PLANAR_QUAD_FOUR', 'ratio': ratio,
        'Nx': Nx, 'Ny': Ny, 'period_segments': per,
        'auto_coarse': auto_coarse, 'rule': rule,
        'frame_in': fr_in, 'frame_out': fr_out,
        'hxi': hxi, 'hyi': hyi, 'H': H, 'V': V, 'dx': dx, 'dy': dy,
        'hcx': hcx, 'hcy': hcy,
        'nx_coarse': nx_coarse, 'ny_coarse': ny_coarse,
        'corner_contrib_u': corner_contrib,
        'corner_contrib_v': corner_contrib,
        'corner_participating': corner_participating,
        'stages': n_stages,
        'stage_counts_x': chain_x['counts'],
        'stage_counts_y': chain_y['counts'],
        'stage_ratios': chain_x['ratios'],
        'stage_frames': stage_frames,
        'u_iface': u_iface, 'v_iface': v_iface,
        'external': external, 'external_note': external_note,
        'k_left': external['k_left'] if external else None,
        'k_right': external['k_right'] if external else None,
        'k_top': external['k_top'] if external else None,
        'k_bottom': external['k_bottom'] if external else None,
        'patch_x': build_transition_patch(ratio, fine_spacing=2.0 * hxi / Nx,
                                          depth=dy),
        'patch_y': build_transition_patch(ratio, fine_spacing=2.0 * hyi / Ny,
                                          depth=dx),
        'target_faces': targets, 'domain': bool(domain_faces and mesh_regular),
        'mesh_regular': bool(mesh_regular),
        'sketch_name': '__tm2d_quad__',
        'sheet_size': 4.0 * max(H, V, 1.0),
        'sketch_origin': _point3(_to_global(fr_in, 0.0, 0.0)),
        'sketch_up_edge': fine_edges[0],
        'model': _owner_model(part),
    }
    plan['external_expected'] = _rect_external_expected(plan)
    return plan


def _rect_external_expected(plan):
    """Expected element count of the external frame (None when absent)."""
    ext = plan['external']
    if not ext:
        return None
    nxc = plan['nx_coarse']
    nyc = plan['ny_coarse']
    k_l, k_r = ext['k_left'], ext['k_right']
    k_t, k_b = ext['k_top'], ext['k_bottom']
    return (nxc * (k_t + k_b) + nyc * (k_l + k_r)
            + (k_l + k_r) * (k_t + k_b))


def _draw_rect_four_patch(sk, plan):
    """Center + 4 strips + participating corners (+ external frame)."""
    from transition_mesh_corner_templates import (
        build_four_participating_corners_2to1, corner_draw_lines)

    fr = plan['frame_in']
    hxi = plan['hxi']
    hyi = plan['hyi']
    H = plan['H']
    V = plan['V']
    origin = plan['sketch_origin']
    # Participating corners keep 1-edge ports; no terminal mid-split.
    split_side_mid = False
    use_participating_corners = bool(plan.get('corner_participating'))

    def S(u, v):
        g = _to_global(fr, u, v)
        return _sketch_xy(plan, g)

    def line(u0, v0, u1, v1):
        sk.Line(point1=S(u0, v0), point2=S(u1, v1))

    def draw_strip(patch, along_u, origin_u, origin_v, dir_u, dir_v):
        per = plan['period_segments']
        h = patch['h']
        n_period = plan['_strip_periods'][id(patch)]
        period_len = per * h
        for k in range(n_period):
            off = k * period_len
            for a, b in patch_unique_edges(patch):
                pu, pv = patch['vertices'][a]
                s = pu + off
                qu, qv = patch['vertices'][b]
                t = qu + off
                # along-strip coordinate is always patch.u; depth is patch.v.
                depth_edge = abs(pu - qu) < 1e-12
                # Only the two strip terminals attach to corners. Interior
                # period boundaries must stay unsplit (no T-junctions).
                is_terminal = False
                if split_side_mid and depth_edge:
                    if k == 0 and min(pu, qu) < 1e-12:
                        is_terminal = True
                    if k == n_period - 1 and max(pu, qu) > period_len - 1e-12:
                        is_terminal = True
                if along_u:
                    A = (origin_u + dir_u * s, origin_v + dir_v * pv)
                    B = (origin_u + dir_u * t, origin_v + dir_v * qv)
                else:
                    A = (origin_u + dir_u * pv, origin_v + dir_v * s)
                    B = (origin_u + dir_u * qv, origin_v + dir_v * t)
                if is_terminal:
                    mid = (0.5 * (A[0] + B[0]), 0.5 * (A[1] + B[1]))
                    line(A[0], A[1], mid[0], mid[1])
                    line(mid[0], mid[1], B[0], B[1])
                    # Close the T-junction: connect mid to the adjacent mid-depth B node of this period so the
                    # end cell stays a quad instead of a pentagon.
                    b_name = None
                    for name, uv in patch['vertices'].items():
                        if name.startswith('B') and abs(uv[1] - 0.5 * patch['depth']) < 1e-12:
                            if min(pu, qu) < 1e-12 and abs(uv[0] - h) < 1e-12:
                                b_name = name
                            elif (max(pu, qu) > period_len - 1e-12
                                  and abs(uv[0] - (period_len - h)) < 1e-12):
                                b_name = name
                    if b_name:
                        bu, bv = patch['vertices'][b_name]
                        bs = bu + off
                        if along_u:
                            Bp = (origin_u + dir_u * bs, origin_v + dir_v * bv)
                        else:
                            Bp = (origin_u + dir_u * bv, origin_v + dir_v * bs)
                        line(mid[0], mid[1], Bp[0], Bp[1])
                else:
                    line(A[0], A[1], B[0], B[1])

    def draw_frame(hxi_strip, hyi_strip, hxi_corner, hyi_corner,
                   H_f, V_f, patch_x, patch_y, per_f,
                   n_strip_x, n_strip_y, use_participating, skip_inner=False):
        plan['_strip_periods'] = {
            id(patch_x): max(1, int(n_strip_x) // per_f),
            id(patch_y): max(1, int(n_strip_y) // per_f),
        }
        def draw_strip(patch, along_u, origin_u, origin_v, dir_u, dir_v):
            h = patch['h']
            n_period = plan['_strip_periods'][id(patch)]
            period_len = per_f * h
            for k in range(n_period):
                off = k * period_len
                for a, b in patch_unique_edges(patch):
                    pu, pv = patch['vertices'][a]
                    qu, qv = patch['vertices'][b]
                    if skip_inner and abs(pv) < 1e-12 and abs(qv) < 1e-12:
                        continue
                    s = pu + off
                    t = qu + off
                    if along_u:
                        A = (origin_u + dir_u * s, origin_v + dir_v * pv)
                        B = (origin_u + dir_u * t, origin_v + dir_v * qv)
                    else:
                        A = (origin_u + dir_u * pv, origin_v + dir_v * s)
                        B = (origin_u + dir_u * qv, origin_v + dir_v * t)
                    line(A[0], A[1], B[0], B[1])
        # Strips sit on the straight portion of the inner interface.
        draw_strip(patch_x, True, -hxi_strip, hyi_corner, +1.0, +1.0)
        draw_strip(patch_x, True, -hxi_strip, -hyi_corner, +1.0, -1.0)
        draw_strip(patch_y, False, hxi_corner, -hyi_strip, +1.0, +1.0)
        draw_strip(patch_y, False, -hxi_corner, -hyi_strip, -1.0, +1.0)
        if use_participating:
            corners = build_four_participating_corners_2to1(
                hxi_corner, hyi_corner, H_f, V_f)
            for patch in corners.values():
                for (u0, v0), (u1, v1) in corner_draw_lines(patch):
                    line(u0, v0, u1, v1)
        else:
            for su in (-1, 1):
                for sv in (-1, 1):
                    u0, u1 = sorted((su * hxi_corner, su * H_f))
                    v0, v1 = sorted((sv * hyi_corner, sv * V_f))
                    line(u0, v0, u1, v0)
                    line(u1, v0, u1, v1)
                    line(u1, v1, u0, v1)
                    line(u0, v1, u0, v0)

    if int(plan.get('stages') or 1) > 1 and plan.get('stage_frames'):
        for i, st in enumerate(plan['stage_frames']):
            draw_frame(
                st['hxi_strip'], st['hyi_strip'],
                st['hxi_corner'], st['hyi_corner'],
                st['H'], st['V'], st['patch_x'], st['patch_y'], st['per'],
                st.get('n_strip_x', st['Nx']), st.get('n_strip_y', st['Ny']),
                bool(st.get('corner_participating')),
                skip_inner=(i > 0))
    else:
        draw_frame(hxi, hyi, hxi, hyi, H, V, plan['patch_x'], plan['patch_y'],
                   plan['period_segments'], plan['Nx'], plan['Ny'],
                   use_participating_corners)

    if plan.get('auto_coarse'):
        # No coarse loop was picked: draw it so the band faces, the
        # interface edges and the corner boxes all exist after partitioning.
        line(-H, -V, H, -V)
        line(H, -V, H, V)
        line(H, V, -H, V)
        line(-H, V, -H, -V)

    ext = plan.get('external')
    if ext:
        # Keep a short overhang so partition lines reach past the outer
        # frame and Abaqus can imprint them on the domain boundary.
        margin = 0.05 * max(ext['umax'] - ext['umin'],
                            ext['vmax'] - ext['vmin'], 1.0)
        # Extend every interface node straight into the external region (spec 4.3).
        # Far-edge nodes then share the same tangential coordinates as the transition outer edge.
        for u in plan.get('u_iface') or []:
            line(u, V, u, ext['vmax'] + margin)
            line(u, -V, u, ext['vmin'] - margin)
        for v in plan.get('v_iface') or []:
            line(H, v, ext['umax'] + margin, v)
            line(-H, v, ext['umin'] - margin, v)
        # Outer frame side separators (existing behaviour).
        for su in (-1, 1):
            line(su * H, V, su * H, ext['vmax'] + margin)
            # created by IY,China University of Petroleum (East China)  |  禁止商用 / non-commercial use only
            line(su * H, ext['vmin'] - margin, su * H, -V)
        for sv in (-1, 1):
            line(H, sv * V, ext['umax'] + margin, sv * V)
            line(ext['umin'] - margin, sv * V, -H, sv * V)


def _rect_band_finder(part, plan):
    fr = plan['frame_in']
    hxi = plan['hxi']
    hyi = plan['hyi']
    H = plan['H']
    V = plan['V']
    band = []
    for i in range(len(part.faces)):
        bb = _face_local_bbox(part, part.faces[i], fr)
        if bb is None:
            continue
        u0, u1, v0, v1 = bb
        if u0 >= -H - 1e-2 and u1 <= H + 1e-2 \
                and v0 >= -V - 1e-2 and v1 <= V + 1e-2:
            if u0 >= -hxi - 1e-2 and u1 <= hxi + 1e-2 \
                    and v0 >= -hyi - 1e-2 and v1 <= hyi + 1e-2:
                continue
            band.append(i)
    return band


def _resolve_rect_regions(part, plan):
    """Post-partition center / external regular faces (mesh_regular only)."""
    if not (plan['mesh_regular'] and plan['domain']):
        return None
    fr = plan['frame_in']
    out = {'center': [], 'ext': {}}
    for i in range(len(part.faces)):
        bb = _face_local_bbox(part, part.faces[i], fr)
        if bb is None:
            continue
        role = _rect_face_role(plan, bb)
        if role == 'center':
            out['center'].append(i)
        elif role and role.startswith('ext_'):
            out['ext'].setdefault(role, []).append(i)
    return out


def _seed_rect_external(part, plan, regions):
    """Explicit counts for every external-frame edge (spec section 6.4)."""
    idxs = []
    for lst in regions['ext'].values():
        idxs.extend(lst)
    if not idxs:
        return {'edges': 0}
    fr = plan['frame_in']
    seen = set()
    seeded = 0
    detail = {}
    for fi in idxs:
        for e in part.faces[fi].getEdges():
            ei = _as_index(e)
            if ei in seen:
                continue
            seen.add(ei)
            edge = part.edges[ei]
            try:
                vs = edge.getVertices()
                if len(vs) != 2:
                    continue
                p0 = part.vertices[vs[0]].pointOn[0]
                p1 = part.vertices[vs[1]].pointOn[0]
            except Exception:
                continue
            u0, v0 = _to_local(fr, p0)
            u1, v1 = _to_local(fr, p1)
            cnt = _rect_ext_edge_count(plan, u0, u1, v0, v1)
            if cnt is None:
                continue
            part.seedEdgeByNumber(edges=(edge,), number=int(cnt),
                                  constraint=FIXED)
            detail[cnt] = detail.get(cnt, 0) + 1
            seeded += 1
    return {'edges': seeded, 'by_count': detail}


def _verify_rect_external_straight(part, plan, ext_faces=None, tol=1e-6):
    """The external ring is a set of axis-aligned rectangular blocks meshed with rows/columns of quads."""
    ext = plan.get('external')
    if not ext:
        return {'ok': True, 'skipped': True}
    if not ext_faces:
        return {'ok': False, 'unverified': True,
                'reason': 'external faces were not supplied'}
    fr = plan['frame_in']
    scale = max(abs(ext['umax'] - ext['umin']),
                abs(ext['vmax'] - ext['vmin']), 1.0)
    lin_tol = max(tol * scale, 1e-4)
    checked = 0
    bad = 0
    max_dev = 0.0
    samples = []
    seen = set()
    for fi in sorted(set(_as_index(f) for f in ext_faces)):
        try:
            elements = part.faces[fi].getElements()
        except Exception:
            elements = None
        if not elements:
            return {'ok': False, 'unverified': True,
                    'reason': 'external Face %d is not meshed' % fi}
        for e in elements:
            try:
                nodes = e.getNodes()
            except Exception:
                continue
            if nodes is None or len(nodes) < 4:
                continue
            coords = [_to_local(fr, n.coordinates) for n in nodes]
            last = len(coords)
            for i in range(last):
                j = i + 1 if i + 1 < last else 0
                key = tuple(sorted((nodes[i].label, nodes[j].label)))
                if key in seen:
                    continue
                seen.add(key)
                du = abs(coords[i][0] - coords[j][0])
                dv = abs(coords[i][1] - coords[j][1])
                dev = min(du, dv)
                checked += 1
                if dev > max_dev:
                    max_dev = dev
                if dev > max(lin_tol, 1e-3):
                    bad += 1
                    if len(samples) < 5:
                        samples.append({
                            'face': fi, 'nodes': list(key),
                            'du': round(du, 6), 'dv': round(dv, 6)})
    return {'ok': bad == 0, 'bad_edges': bad, 'checked_edges': checked,
            'max_dev': max_dev, 'lin_tol': lin_tol, 'samples': samples}


def _verify_rect_interfaces(part, plan):
    """Sum of seeded segments on the four outer / inner boundary lines."""
    fr = plan['frame_in']
    hxi = plan['hxi']
    hyi = plan['hyi']
    H = plan['H']
    V = plan['V']
    tol = 1e-2
    got = {'outer_x': 0, 'outer_y': 0, 'inner_x': 0, 'inner_y': 0}
    seen = set()
    for ei in range(len(part.edges)):
        if ei in seen:
            continue
        seen.add(ei)
        edge = part.edges[ei]
        try:
            vs = edge.getVertices()
            if len(vs) != 2:
                continue
            p0 = part.vertices[vs[0]].pointOn[0]
            p1 = part.vertices[vs[1]].pointOn[0]
        except Exception:
            continue
        u0, v0 = _to_local(fr, p0)
        u1, v1 = _to_local(fr, p1)
        s = _edge_seed_count(part, edge) or 0
        if s <= 0:
            continue
        if abs(v0 - V) < tol and abs(v1 - V) < tol:
            if abs(0.5 * (u0 + u1)) <= H + tol:
                got['outer_x'] += s
        elif abs(v0 + V) < tol and abs(v1 + V) < tol:
            if abs(0.5 * (u0 + u1)) <= H + tol:
                got['outer_x'] += s
        elif abs(u0 - H) < tol and abs(u1 - H) < tol:
            if abs(0.5 * (v0 + v1)) <= V + tol:
                got['outer_y'] += s
        elif abs(u0 + H) < tol and abs(u1 + H) < tol:
            if abs(0.5 * (v0 + v1)) <= V + tol:
                got['outer_y'] += s
        elif abs(v0 - hyi) < tol and abs(v1 - hyi) < tol:
            if abs(0.5 * (u0 + u1)) <= hxi + tol:
                got['inner_x'] += s
        elif abs(v0 + hyi) < tol and abs(v1 + hyi) < tol:
            if abs(0.5 * (u0 + u1)) <= hxi + tol:
                got['inner_x'] += s
        elif abs(u0 - hxi) < tol and abs(u1 - hxi) < tol:
            if abs(0.5 * (v0 + v1)) <= hyi + tol:
                got['inner_y'] += s
        elif abs(u0 + hxi) < tol and abs(u1 + hxi) < tol:
            if abs(0.5 * (v0 + v1)) <= hyi + tol:
                got['inner_y'] += s
    exp = {'outer_x': 2 * plan['nx_coarse'],
           'outer_y': 2 * plan['ny_coarse'],
           'inner_x': 2 * plan['Nx'],
           'inner_y': 2 * plan['Ny']}
    ok = all(got[k] == exp[k] for k in exp)
    return {'outer_x': got['outer_x'], 'outer_y': got['outer_y'],
            'inner_x': got['inner_x'], 'inner_y': got['inner_y'],
            'expected': exp, 'ok': ok}


def _imprint_rect_frames(part, plan):
    """Partition intermediate rectangular loops for multi-stage four-side."""
    frames = plan.get('stage_frames') or []
    if len(frames) <= 1:
        return
    model = plan['model']
    fr = plan['frame_in']
    origin = plan['sketch_origin']
    targets = list(plan['target_faces'])
    if not targets:
        raise ValueError(
            'Multi-stage four-side: no target Face available for the frame '
            'imprint. The picked geometry may have changed since selection '
            '-- repick the transition region / Full domain and run Check '
            'again.')
    name = '__tm2d_ms_frames__'
    side = SIDE1
    transform_kw = dict(
        sketchPlane=targets[0], sketchPlaneSide=side,
        sketchOrientation=RIGHT, origin=_point3(origin))
    sketch_up = plan.get('sketch_up_edge')
    if sketch_up is not None:
        transform_kw['sketchUpEdge'] = sketch_up
    transform = part.MakeSketchTransform(**transform_kw)
    plan['_sketch_transform_matrix'] = transform.matrix()
    sk = model.ConstrainedSketch(name=name, sheetSize=plan['sheet_size'],
                                 transform=transform)
    try:
        def S(u, v):
            g = _to_global(fr, u, v)
            return _sketch_xy(plan, g)

        def line(u0, v0, u1, v1):
            sk.Line(point1=S(u0, v0), point2=S(u1, v1))

        # Intermediate frames only (not the picked inner/outer loops).
        # With an auto coarse loop the outermost frame must be drawn as well,
        frames_to_draw = frames if plan.get('auto_coarse') else frames[:-1]
        for st in frames_to_draw:
            hxi, hyi, H, V = st['hxi'], st['hyi'], st['H'], st['V']
            corners = [S(-H, -V), S(H, -V), S(H, V), S(-H, V)]
            for i in range(4):
                sk.Line(point1=corners[i], point2=corners[(i + 1) % 4])
        # External ring separators (same construction as the single-stage full template):
        # without them the outer ring stays one Face and the planned center/external element counts cannot be
        ext = plan.get('external')
        if ext:
            H, V = plan['H'], plan['V']
            margin = 0.05 * max(ext['umax'] - ext['umin'],
                                ext['vmax'] - ext['vmin'], 1.0)
            for u in (ext.get('sep_u') or plan.get('u_iface') or []):
                line(u, V, u, ext['vmax'] + margin)
                line(u, -V, u, ext['vmin'] - margin)
            for v in (ext.get('sep_v') or plan.get('v_iface') or []):
                line(H, v, ext['umax'] + margin, v)
                line(-H, v, ext['umin'] - margin, v)
            for su in (-1, 1):
                line(su * H, V, su * H, ext['vmax'] + margin)
                line(su * H, ext['vmin'] - margin, su * H, -V)
            for sv in (-1, 1):
                line(H, sv * V, ext['umax'] + margin, sv * V)
                line(ext['umin'] - margin, sv * V, -H, sv * V)
        partition_kw = dict(faces=tuple(targets), sketch=sk)
        if sketch_up is not None:
            partition_kw['sketchUpEdge'] = sketch_up
        part.PartitionFaceBySketch(**partition_kw)
    finally:
        try:
            del model.sketches[name]
        except Exception:
            pass
        plan.pop('_sketch_transform_matrix', None)


def _rect_loop_at_half(part, half, fr, tol=1e-2):
    """Edges forming the axis-aligned rectangle of half-size `half`."""
    out = []
    for i in range(len(part.edges)):
        edge = part.edges[i]
        try:
            vs = edge.getVertices()
            if len(vs) != 2:
                continue
            p0 = part.vertices[vs[0]].pointOn[0]
            p1 = part.vertices[vs[1]].pointOn[0]
            u0, v0 = _to_local(fr, p0)
            u1, v1 = _to_local(fr, p1)
        except Exception:
            continue
        on_u = (abs(abs(u0) - half) < tol and abs(abs(u1) - half) < tol
                and abs(v0) <= half + tol and abs(v1) <= half + tol
                and abs(v0 - v1) > tol)
        on_v = (abs(abs(v0) - half) < tol and abs(abs(v1) - half) < tol
                and abs(u0) <= half + tol and abs(u1) <= half + tol
                and abs(u0 - u1) > tol)
        if on_u or on_v:
            out.append(edge)
    return out


def _apply_quad_four_multistage(part, plan, region_faces, fine_edges,
                                coarse_edges, action, element2d,
                                mesh_regular, domain_faces):
    """Template-controlled multi-stage four-side needs a trapezoid corner (inner/outer port lengths differ)"""
    from abaqusConstants import FIXED, QUAD, FREE
    n_stages = int(plan['stages'])
    frames = plan['stage_frames']
    part.backup()
    try:
        # 1) Imprint intermediate rectangles.
        _imprint_rect_frames(part, plan)
        band = _rect_band_finder(part, plan)
        if not band:
            raise ValueError('No transition-band Face after multi-stage imprint')
        result = {
            'mode': 'PLANAR_QUAD_FOUR', 'ratio': plan['ratio'],
            'stages': n_stages,
            'stage_counts_x': plan.get('stage_counts_x'),
            'stage_counts_y': plan.get('stage_counts_y'),
            'stage_ratios': plan.get('stage_ratios'),
            'Nx': plan['Nx'], 'Ny': plan['Ny'],
            'nx_coarse': plan['nx_coarse'], 'ny_coarse': plan['ny_coarse'],
            'band_faces': len(band), 'action': action,
            'element2d': str(element2d),
            'corner_participating': bool(frames[-1].get('corner_participating')),
            'external_mode': plan.get('external_note') or 'none',
            'mesh_strategy': 'imprint_free',
        }
        if action == 'PARTITION_ONLY':
            result.update(_audit_patch_mesh(part, band))
            result['faces_meshed'] = 0
            return result

        # 2) Seed every band edge to 1, then override interface edges
        #    with the planned per-segment counts, then the external ring.
        result['template_edges_seeded'] = _seed_patch_plan(part, band)
        regions = _resolve_rect_regions(part, plan)
        if regions is not None and plan.get('external'):
            result['external_seeding'] = _seed_rect_external(
                part, plan, regions)
        _seed_multistage_interfaces(part, plan, frames)
        iface = _verify_rect_interfaces(part, plan)
        result['interface_counts'] = iface
        if not iface['ok']:
            result['compatibility'] = 'IFACE_FAIL'
            raise ValueError(
                'Multi-stage four-side interface mismatch: outer %d/%d '
                'vs planned %d/%d.'
                % (iface['outer_x'], iface['outer_y'],
                   iface['expected']['outer_x'], iface['expected']['outer_y']))
        result['compatibility'] = 'OK'
        if action == 'PARTITION_AND_SEED':
            if _is_solid(part):
                return _finish_solid_mesh(
                    part, result, plan.get('solid_transition_cells'), action)
            result.update(_audit_patch_mesh(part, band))
            result['faces_meshed'] = 0
            return result

        if _is_solid(part):
            return _finish_solid_mesh(
                part, result, plan.get('solid_transition_cells'), action)

        # 3) Free-mesh the band only (case-model style).  The scope is the
        #    processed band faces; the whole Part must never be meshed here.
        faces = [part.faces[i] for i in band]
        try:
            seq = _face_seq(part, faces)
            part.setMeshControls(regions=seq, elemShape=QUAD, technique=FREE)
            _apply_elem_type(part, faces, element2d)
            part.generateMesh(regions=seq)
            ctrl = 'QUAD+FREE'
        except Exception as mex:
            result['mesh_error'] = repr(mex)
            ctrl = False
        audit = _audit_patch_mesh(part, band)
        result['mesh_control'] = ctrl
        result.update(audit)
        if audit['elements'] <= 0:
            raise ValueError(
                'Multi-stage free mesh produced 0 elements.')
        if audit['unmeshed_faces'] != 0:
            raise ValueError(
                'Multi-stage still has %d unmeshed Face(s)'
                % audit['unmeshed_faces'])
        result['faces_meshed'] = len(band)
        # Pure QUAD is a product requirement: triangles are NOT accepted.
        if audit.get('tri'):
            raise ValueError(
                'Multi-stage band produced %d triangle(s); pure QUAD is '
                'required.' % audit['tri'])
        for key in ('duplicate_elements', 'negative_area', 'zero_area',
                    'bowtie', 'hanging_nodes', 'jacobian_bad'):
            if audit.get(key):
                raise ValueError(
                    'Multi-stage band quality failed: %s=%s'
                    % (key, audit.get(key)))
        # Estimate expected elements from interface seed product.
        nxc = plan['nx_coarse']
        nyc = plan['ny_coarse']
        nxi = plan['Nx']
        nyi = plan['Ny']
        result['expected_elements'] = (
            nxi * nyi  # center (not in band, for reference)
            + 2 * nxi * ((plan['V'] - plan['hyi']) / (2 * plan['hyi'] / nxi))
            + 2 * nyc * ((plan['H'] - plan['hxi']) / (2 * plan['hxi'] / nxi))
        )
        # Center / external regular regions: the returned audits must be
        # validated, not just executed.
        if regions is None:
            regions = _resolve_rect_regions(part, plan)
        if regions is not None and mesh_regular and domain_faces:
            if regions.get('center'):
                cctrl, cerr, caudit = _mesh_patch_plan(
                    part, regions['center'], element2d)
                result['center_mesh_control'] = cctrl
                result['center_elements'] = caudit['elements']
                result['center_tri'] = caudit['tri']
                result['center_unmeshed_faces'] = caudit['unmeshed_faces']
                _dbg('[p2d-quad-ms] center_ctrl=%s audit=%s' % (cctrl, caudit))
                if caudit['elements'] != nxi * nyi:
                    raise ValueError(
                        'Multi-stage center elements %d != Nx*Ny=%d'
                        % (caudit['elements'], nxi * nyi))
                if caudit.get('tri') or caudit.get('unmeshed_faces') \
                        or caudit.get('jacobian_bad'):
                    raise ValueError(
                        'Multi-stage center region quality failed: '
                        'tri=%s unmeshed=%s jacobian_bad=%s'
                        % (caudit.get('tri'), caudit.get('unmeshed_faces'),
                           caudit.get('jacobian_bad')))
            ext_idx = [i for lst in regions.get('ext', {}).values() for i in lst]
            if ext_idx and plan.get('external'):
                ectrl, eerr, eaudit = _mesh_patch_plan(part, ext_idx, element2d)
                result['external_mesh_control'] = ectrl
                result['external_elements'] = eaudit['elements']
                result['external_tri'] = eaudit['tri']
                result['external_unmeshed_faces'] = eaudit['unmeshed_faces']
                straight = _verify_rect_external_straight(part, plan, ext_idx)
                result['external_straight'] = straight
                _dbg('[p2d-quad-ms] external_ctrl=%s audit=%s straight=%s'
                     % (ectrl, eaudit, straight))
                if eaudit['elements'] != plan['external_expected']:
                    raise ValueError(
                        'Multi-stage external elements %d != plan %d'
                        % (eaudit['elements'], plan['external_expected']))
                if eaudit.get('tri') or eaudit.get('unmeshed_faces') \
                        or eaudit.get('jacobian_bad'):
                    raise ValueError(
                        'Multi-stage external region quality failed: '
                        'tri=%s unmeshed=%s jacobian_bad=%s'
                        % (eaudit.get('tri'), eaudit.get('unmeshed_faces'),
                           eaudit.get('jacobian_bad')))
                if straight.get('unverified'):
                    result['status'] = 'WARN'
                    result.setdefault('warnings', []).append(
                        'External regularity not verified: %s'
                        % straight.get('reason'))
                elif not straight.get('ok', True):
                    strict = 'STRUCTURED' in str(ectrl)
                    result['external_straight_warn'] = straight
                    result['status'] = 'WARN'
                    result.setdefault('warnings', []).append(
                        'External layout deviates from straight rows/columns '
                        '(bad=%s of %s edges, max_dev=%s, technique=%s).'
                        % (straight.get('bad_edges'),
                           straight.get('checked_edges'),
                           straight.get('max_dev'), ectrl))
                    if strict:
                        raise ValueError(
                            'Multi-stage external region is not row/column '
                            'aligned although STRUCTURED meshing was used: '
                            '%s bad edge(s), max_dev=%s, samples=%s'
                            % (straight.get('bad_edges'),
                               straight.get('max_dev'),
                               straight.get('samples')))
        return result
    except BaseException as exc:
        try:
            part.restore()
        except BaseException as rex:
            raise RuntimeError(
                'Multi-stage four-side failed: %r; restore: %r' % (exc, rex))
        raise


def _seed_multistage_interfaces(part, plan, frames):
    """Seed each inter-stage rectangle boundary to its planned segment count."""
    fr = plan['frame_in']
    tol = 1e-2
    # (half_u, half_v, count_per_horizontal_edge, count_per_vertical_edge)
    rects = []
    for i, st in enumerate(frames):
        hu = st['hxi'] if i == 0 else st['hxi_corner']
        hv = st['hyi'] if i == 0 else st['hyi_corner']
        # Per-edge counts: the full loop has 2*count on opposite edges.
        cu = st['Nx'] if i == 0 else st['n_strip_x'] + 2
        cv = st['Ny'] if i == 0 else st['n_strip_y'] + 2
        rects.append((hu, hv, cu, cv))
    # The outermost rectangle is re-seeded only when it is NOT subdivided by external separators:
    # it is then a single piece per side, so it needs the full loop count.
    if not plan.get('external'):
        rects.append((plan['H'], plan['V'], plan['nx_coarse'],
                      plan['ny_coarse']))
    seen = set()
    for hu, hv, cu, cv in rects:
        for ei in range(len(part.edges)):
            if ei in seen:
                continue
            edge = part.edges[ei]
            try:
                vs = edge.getVertices()
                if len(vs) != 2:
                    continue
                p0 = part.vertices[vs[0]].pointOn[0]
                p1 = part.vertices[vs[1]].pointOn[0]
            except Exception:
                continue
            u0, v0 = _to_local(fr, p0)
            u1, v1 = _to_local(fr, p1)
            # Top/bottom edge of this rectangle
            if (abs(abs(v0) - hv) < tol and abs(abs(v1) - hv) < tol
                    and abs(0.5 * (u0 + u1)) <= hu + tol):
                part.seedEdgeByNumber(edges=(edge,), number=int(cu),
                                      constraint=FIXED)
                seen.add(ei)
            # Left/right edge
            elif (abs(abs(u0) - hu) < tol and abs(abs(u1) - hu) < tol
                    and abs(0.5 * (v0 + v1)) <= hv + tol):
                part.seedEdgeByNumber(edges=(edge,), number=int(cv),
                                      constraint=FIXED)
                seen.add(ei)
    return len(seen)


def apply_planar_quad_four_transition(
        part, region_faces, fine_edges, coarse_edges,
        ratio=2, fine_seeds=10, fine_seeds_y=None,
        action='PARTITION_SEED_MESH', element2d='CPS4R',
        mesh_regular=False, domain_faces=None,
        stages=1, ratioSequence='', stageWidths=''):
    """Nx = fine_seeds (u sides), Ny = fine_seeds_y (v sides; default Nx)."""
    if action not in _ACTIONS:
        raise ValueError('Illegal action %r' % (action,))
    solid = _is_solid(part)
    if not solid:
        _require_2d(part)
    Nx = int(fine_seeds)
    Ny = int(fine_seeds_y) if fine_seeds_y else Nx
    fine_edges = [e for e in (fine_edges or []) if e is not None]
    coarse_edges = [e for e in (coarse_edges or []) if e is not None]
    if not fine_edges:
        raise ValueError('Pick the inner fine edge loop')
    # An empty coarse_edges list is the supported "auto coarse loop" mode:
    # the outer frame is computed from case rule (4) and partitioned here.
    domain_faces = [f for f in (domain_faces or []) if f is not None]
    solid_transition_cells = (_cell_indices_from_faces(region_faces)
                              if solid else [])
    if solid and not solid_transition_cells:
        raise ValueError(
            'PLANAR_QUAD_FOUR on a solid needs planar source Face(s) attached '
            'to the user-prepared transition Cell.')

    plan = _build_rect_four_patch_plan(
        part, region_faces, fine_edges, coarse_edges, ratio, Nx, Ny,
        domain_faces=domain_faces, mesh_regular=mesh_regular,
        stages=stages, ratioSequence=ratioSequence, stageWidths=stageWidths)
    plan['solid_transition_cells'] = solid_transition_cells
    n_stages = int(plan.get('stages') or 1)
    if plan.get('reference_quad2'):
        from transition_mesh_quad2_cae import apply
        return apply(part, plan, action, element2d)
    if n_stages > 1:
        return _apply_quad_four_multistage(
            part, plan, region_faces, fine_edges, coarse_edges,
            action, element2d, mesh_regular, domain_faces)
    corner_quads = 5 if plan.get('corner_participating') else 1
    expect_faces = (2 * expected_transition_elements(plan['ratio'], Nx)
                    + 2 * expected_transition_elements(plan['ratio'], Ny)
                    + 4 * corner_quads)
    Nx = plan['Nx']
    Ny = plan['Ny']

    part.backup()
    try:
        _partition_patch_plan(part, plan, _draw_rect_four_patch)
        band = _rect_band_finder(part, plan)
        if not band:
            raise ValueError('No transition-band Face found after four-side partition')
        regions = _resolve_rect_regions(part, plan)

        result = {
            'mode': 'PLANAR_QUAD_FOUR', 'ratio': plan['ratio'],
            'Nx': Nx, 'Ny': Ny, 'fine_seeds_per_side': Nx,
            'L_fine_u': round(2 * plan['hxi'], 6),
            'L_fine_v': round(2 * plan['hyi'], 6),
            'dx': round(plan['dx'], 6), 'dy': round(plan['dy'], 6),
            'fine_interface_segments_x': Nx,
            'fine_interface_segments_y': Ny,
            'coarse_interface_segments_x': Nx // plan['ratio'],
            'coarse_interface_segments_y': Ny // plan['ratio'],
            'outer_side_segments_x': plan['nx_coarse'],
            'outer_side_segments_y': plan['ny_coarse'],
            'corner_contribution_u': plan.get('corner_contrib_u', 1),
            'corner_contribution_v': plan.get('corner_contrib_v', 1),
            'corner_participating': bool(plan.get('corner_participating')),
            'stages': n_stages,
            'stage_counts_x': plan.get('stage_counts_x'),
            'stage_counts_y': plan.get('stage_counts_y'),
            'expected_per_strip_x':
                expected_transition_elements(plan['ratio'], Nx),
            'expected_per_strip_y':
                expected_transition_elements(plan['ratio'], Ny),
            'expected_transition_faces': expect_faces,
            'band_faces': len(band), 'action': action,
            'element2d': str(element2d),
            'external_mode': plan['external_note'],
        }
        if plan['external']:
            ext = plan['external']
            result['external_plan'] = {
                'k_left': ext['k_left'], 'k_right': ext['k_right'],
                'k_top': ext['k_top'], 'k_bottom': ext['k_bottom'],
                'expected_elements': plan['external_expected'],
            }
        _dbg('[p2d-quad] band_faces=%d expect=%d Nx=%d Ny=%d ext=%s'
             % (len(band), expect_faces, Nx, Ny, plan['external_note']))
        if action == 'PARTITION_ONLY':
            result.update(_audit_patch_mesh(part, band))
            result['faces_meshed'] = 0
            return result

        result['template_edges_seeded'] = _seed_patch_plan(part, band)
        if regions is not None and plan['external']:
            result['external_seeding'] = _seed_rect_external(
                part, plan, regions)
        iface = _verify_rect_interfaces(part, plan)
        result['interface_counts'] = iface
        result['compatibility'] = 'OK' if iface['ok'] else 'INTERFACE_MISMATCH'
        _dbg('[p2d-quad] seeded_edges=%d iface=%r'
             % (result['template_edges_seeded'], iface))
        if not iface['ok']:
            raise ValueError('Four-side interface seed counts do not match the plan: %r' % (iface,))

        if action == 'PARTITION_AND_SEED':
            if solid:
                return _finish_solid_mesh(
                    part, result, solid_transition_cells, action)
            result.update(_audit_patch_mesh(part, band))
            result['faces_meshed'] = 0
            return result

        if solid:
            return _finish_solid_mesh(
                part, result, solid_transition_cells, action)

        ctrl, mesh_err, audit = _mesh_patch_plan(
            part, band, element2d,
            prefer_sweep=bool(plan.get('corner_participating')))
        result['mesh_control'] = ctrl
        result.update(audit)
        _dbg('[p2d-quad] mesh_ctrl=%s audit=%s' % (ctrl, audit))
        if audit['elements'] <= 0:
            raise ValueError('Four-side mesh produced 0 elements (mesh_err=%s)' % mesh_err)
        if audit['unmeshed_faces'] != 0:
            raise ValueError('Four-side still has %d unmeshed Face(s)'
                             % audit['unmeshed_faces'])
        result['faces_meshed'] = len(band)
        result['expected_elements'] = expect_faces

        if regions is not None:
            if regions['center']:
                cctrl, cerr, caudit = _mesh_patch_plan(
                    part, regions['center'], element2d)
                result['center_mesh_control'] = cctrl
                result['center_elements'] = caudit['elements']
                result['center_expected'] = Nx * Ny
                result['center_tri'] = caudit['tri']
                result['center_unmeshed_faces'] = caudit['unmeshed_faces']
                _dbg('[p2d-quad] center_ctrl=%s audit=%s' % (cctrl, caudit))
                if caudit['elements'] != Nx * Ny:
                    raise ValueError(
                        'Center regular region elements %d != Nx*Ny=%d'
                        % (caudit['elements'], Nx * Ny))
            ext_idx = [i for lst in regions['ext'].values() for i in lst]
            if ext_idx and plan['external']:
                ectrl, eerr, eaudit = _mesh_patch_plan(part, ext_idx, element2d)
                result['external_mesh_control'] = ectrl
                result['external_elements'] = eaudit['elements']
                result['external_tri'] = eaudit['tri']
                result['external_unmeshed_faces'] = eaudit['unmeshed_faces']
                result['external_quality'] = {
                    k: eaudit.get(k) for k in
                    ('duplicate_elements', 'negative_area', 'zero_area',
                     'bowtie', 'hanging_nodes', 'jacobian_bad',
                     'aspect_ratio_max', 'aspect_ratio_warn')
                    if k in eaudit}
                straight = _verify_rect_external_straight(part, plan, ext_idx)
                result['external_straight'] = straight
                _dbg('[p2d-quad] external_ctrl=%s audit=%s straight=%s'
                     % (ectrl, eaudit, straight))
                if eaudit['elements'] != plan['external_expected']:
                    raise ValueError(
                        'External regular region elements %d != plan %d'
                        % (eaudit['elements'], plan['external_expected']))
                if eaudit['unmeshed_faces'] != 0:
                    raise ValueError('External regular region still has %d unmeshed Face(s)'
                                     % eaudit['unmeshed_faces'])
                if eaudit.get('jacobian_bad'):
                    raise ValueError(
                        'External mesh has %d element(s) with non-positive '
                        'Jacobian' % eaudit['jacobian_bad'])
                if straight.get('unverified'):
                    result['external_straight_unverified'] = straight
                    result['status'] = 'WARN'
                    result.setdefault('warnings', []).append(
                        'External regularity not verified: %s'
                        % straight.get('reason'))
                elif not straight.get('ok', True):
                    # Row/column layout deviates.
                    strict = 'STRUCTURED' in str(ectrl)
                    result['external_straight_warn'] = straight
                    result['status'] = 'WARN'
                    result.setdefault('warnings', []).append(
                        'External layout deviates from straight rows/columns '
                        '(bad=%s of %s edges, max_dev=%s, technique=%s).'
                        % (straight.get('bad_edges'),
                           straight.get('checked_edges'),
                           straight.get('max_dev'), ectrl))
                    if strict:
                        raise ValueError(
                            'External regular region is not row/column '
                            'aligned although STRUCTURED meshing was used: '
                            '%s bad edge(s), max_dev=%s, samples=%s'
                            % (straight.get('bad_edges'),
                               straight.get('max_dev'),
                               straight.get('samples')))
                    _dbg('[p2d-quad] external straight warn: %s' % straight)
        # Band audit: fail on quality issues that indicate a broken mesh.
        for key in ('duplicate_elements', 'negative_area', 'zero_area',
                    'bowtie', 'hanging_nodes', 'jacobian_bad'):
            if audit.get(key):
                raise ValueError(
                    'Four-side band quality failed: %s=%s'
                    % (key, audit.get(key)))
        return result
    except BaseException as exc:
        try:
            part.restore()
        except BaseException as rex:
            raise RuntimeError('Four-side failed: %r; restore: %r' % (exc, rex))
        raise
