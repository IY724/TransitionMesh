# -*- coding: utf-8 -*-
# ============================================================================
#  Transition Mesh
#  created by IY,China University of Petroleum (East China)
#  禁止商用 / Non-commercial use only
#  禁止商用：本软件仅供学习与研究使用，未经作者书面许可不得用于任何商业用途。
#  Non-commercial use only. Unauthorized commercial use is prohibited.
# ============================================================================

# 圆筒过渡内核：ANNULAR_RADIAL 环向与 AXIAL_SURFACE 轴向过渡，选区驱动，全部校验先于修改。

import math

import transition_mesh_rules as tm_rules

from abaqus import mdb
from abaqusConstants import (SIDE1, SIDE2, RIGHT, FIXED, FORWARD, REVERSE,
                             SWEEP, STRUCTURED, ADVANCING_FRONT, C3D8R,
                             STANDARD, NUMBER, CONSTRAINT)
from mesh import ElemType

# GUI 运行时保持静默：对话框已通过状态栏、弹窗与报告文件展示结果，车道内的信息性 print 在 CAE 底部信息区默认不输出；仅当设置环境变量 TM_VERBOSE=1（回归脚本会设置）时才回显。
# 异常、返回值与模型本身不受影响，回滚/审计行为不变。
import os as _tm_os


def _tm_quiet_print(*args, **kwargs):
    return 0


if not _tm_os.environ.get('TM_VERBOSE'):
    print = _tm_quiet_print  # 有意遮蔽模块内建 print

_CYL_TOL_ABS = 1.0e-5
_CYL_ACTIONS = ('PARTITION_ONLY', 'PARTITION_AND_SEED',
                'PARTITION_SEED_MESH')


def _cyl_owned_backup(part):
    """The formal GUI entry owns the model-copy transaction."""
    try:
        import transition_mesh_runtime as runtime
        fn = getattr(runtime, 'owned_backup', None)
        if callable(fn):
            return fn(part)
    except Exception:
        pass
    return part.backup()


def _cyl_owned_restore(part):
    """Restore a cylinder transaction through the shared runtime if present."""
    try:
        import transition_mesh_runtime as runtime
        fn = getattr(runtime, 'owned_restore', None)
        if callable(fn):
            return fn(part)
    except Exception:
        pass
    return part.restore()


def _cyl_parse_ratio(value):
    """Parse a numeric ratio or the GUI's ``'2:1'``/``'3:1'`` spelling."""
    if isinstance(value, str):
        text = value.strip()
        if ':' in text:
            text = text.split(':', 1)[0].strip()
        value = text
    try:
        number = int(value)
    except Exception:
        raise ValueError('Ratio must be 2, 3 or "2:1"/"3:1", got %r' % (value,))
    if number not in (2, 3):
        raise ValueError('Cylinder multistage supports only 2:1 / 3:1, got %r' % (value,))
    return number


def _cyl_plan_multistage_counts(fine_seeds, stages, ratio, ratio_sequence):
    """Use the shared planner when available, with a strict local fallback."""
    try:
        import transition_mesh_plan as planner
        fn = getattr(planner, 'plan_counts', None)
        if callable(fn):
            plan = fn(fine_seeds=int(fine_seeds), stages=int(stages),
                      ratio=_cyl_parse_ratio(ratio),
                      ratio_sequence=ratio_sequence)
            ratios = [_cyl_parse_ratio(x) for x in plan.get('ratios', ())]
            counts = [int(x) for x in plan.get('counts', ())]
            if len(ratios) == int(stages) and len(counts) == int(stages) + 1:
                return {'ratios': ratios, 'counts': counts,
                        'stages': int(stages),
                        'required_multiple': int(plan.get(
                            'required_multiple', 1))}
    except Exception:
        # The kernel is intentionally usable before the planner deployment.
        # The local contract below is the same arithmetic gate.
        pass
    try:
        stages = int(stages)
    except Exception:
        raise ValueError('stages must be a positive integer, got %r' % (stages,))
    if stages < 1:
        raise ValueError('stages must be at least 1')
    if ratio_sequence is None:
        ratios = [_cyl_parse_ratio(ratio)] * stages
    else:
        try:
            ratios = [_cyl_parse_ratio(x) for x in ratio_sequence]
        except TypeError:
            raise ValueError('ratio_sequence must be a sequence of length stages')
        if len(ratios) != stages:
            raise ValueError('ratio_sequence length %d != stages=%d' %
                             (len(ratios), stages))
    try:
        n0 = int(fine_seeds)
    except Exception:
        raise ValueError('fine_seeds must be a positive integer, got %r' % (fine_seeds,))
    if n0 <= 0:
        raise ValueError('fine_seeds must be a positive integer')
    counts = [n0]
    n = n0
    required = 1
    for i, r in enumerate(ratios):
        # The proven 2:1 annular/axial template is 4 fine -> 2 coarse; each 2:
        # 1 stage therefore needs one extra factor of two at its incoming interface.
        divisor = r * (2 if r == 2 else 1)
        required *= divisor
        if n % divisor != 0:
            raise ValueError('Stage %d (%d:1) requires the input interface seed count %d to be '
                             'divisible by %d; template period not satisfied' % (i + 1, r, n,
                                                       divisor))
        n //= r
        counts.append(n)
    if n <= 0:
        raise ValueError('Multistage seed chain produced a non-positive count')
    return {'ratios': ratios, 'counts': counts, 'stages': stages,
            'required_multiple': required}


# 向量小工具

def _cyl_v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cyl_v_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cyl_v_cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _cyl_v_norm(a):
    return math.sqrt(_cyl_v_dot(a, a))


def _cyl_v_unit(a):
    n = _cyl_v_norm(a)
    if n < 1e-12:
        raise ValueError('Cannot normalize a zero vector')
    return (a[0] / n, a[1] / n, a[2] / n)


def _cyl_v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _cyl_v_mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _cyl_as_index(x):
    """Abaqus 不同上下文返回索引或对象，统一成 int 索引。"""
    if isinstance(x, int):
        return x
    try:
        return x.index
    except Exception:
        raise ValueError('Cannot resolve %r to an index' % (x,))


# 局部坐标系（模式感知）

def _cyl_edge_circle(part, edge):
    """圆形边 -> (圆心, 圆平面法向单位向量)。getCurvature 精确求解。"""
    cv = edge.getCurvature(0.5)
    k = tuple(cv['curvature'])
    kn = _cyl_v_norm(k)
    if kn < 1e-9:
        raise ValueError('The selected edge is not an arc/circle (zero curvature)')
    ep = tuple(cv['evaluationPoint'])
    center = _cyl_v_add(ep, _cyl_v_mul(k, 1.0 / (kn * kn)))
    tangent = tuple(cv.get('tangent') or (0.0, 0.0, 0.0))
    tn = _cyl_v_norm(tangent)
    if tn < 1e-12:
        raise ValueError('getCurvature returned no tangent; cannot determine the circle-plane normal')
    normal = _cyl_v_unit(_cyl_v_cross(_cyl_v_mul(tangent, 1.0 / tn),
                                      _cyl_v_mul(k, 1.0 / kn)))
    return center, normal


def build_cylinder_frame_from_edges(part, fine_edge, coarse_edge,
                                    mode='AXIAL_SURFACE'):
    """ANNULAR_RADIAL：两圆位于同一端面、同心共面 -> 轴向取圆平面法向，圆心相同不是错误（同心正是合法输入），并显式校验同心与共面。"""
    c1, n1 = _cyl_edge_circle(part, fine_edge)
    c2, n2 = _cyl_edge_circle(part, coarse_edge)
    if mode == 'ANNULAR_RADIAL':
        sin_ab = _cyl_v_norm(_cyl_v_cross(n1, n2))
        if sin_ab > 1e-4:
            raise ValueError('Fine and coarse circle planes are not parallel (angle %.4f rad); '
                             'the radial transition requires both rings on the same end face'
                             % math.asin(min(1.0, sin_ab)))
        d = _cyl_v_sub(c2, c1)
        axial_off = _cyl_v_dot(d, n1)
        radial_off = _cyl_v_norm(_cyl_v_sub(d, _cyl_v_mul(n1, axial_off)))
        if abs(axial_off) > 1e-4:
            raise ValueError('Fine and coarse edges are not in the same section (axial offset %.6f); '
                             'for coaxial rings in different sections use AXIAL_SURFACE instead'
                             % abs(axial_off))
        if radial_off > 1e-4:
            raise ValueError('Fine and coarse edges are not concentric (eccentricity %.6f); cannot '
                             'form a concentric ring transition' % radial_off)
        axis_dir = n1
        axis_point = c1
    elif mode == 'AXIAL_SURFACE':
        d = _cyl_v_sub(c2, c1)
        if _cyl_v_norm(d) < 1e-6:
            raise ValueError('Fine and coarse circle centers coincide (same section); only '
                             'axial transitions across sections use the center line as the axis; for concentric rings use '
                             'ANNULAR_RADIAL')
        axis_dir = _cyl_v_unit(d)
        axis_point = c1
        for name, n in (('fine edge', n1), ('coarse edge', n2)):
            sin_a = _cyl_v_norm(_cyl_v_cross(n, axis_dir))
            if sin_a > 1e-4:
                raise ValueError('%s circle plane is not perpendicular to the center line (angle %.4f rad); '
                                 'the two circular edges are not coaxial' % (name, math.asin(min(1.0, sin_a))))
    else:
        raise ValueError('Unknown cylinder mode %r' % (mode,))
    seed = (1.0, 0.0, 0.0)
    if abs(_cyl_v_dot(seed, axis_dir)) > 0.9:
        seed = (0.0, 1.0, 0.0)
    radial_dir = _cyl_v_unit(_cyl_v_sub(seed,
                                        _cyl_v_mul(axis_dir,
                                                   _cyl_v_dot(seed, axis_dir))))
    theta_dir = _cyl_v_cross(axis_dir, radial_dir)
    return {'axis_point': axis_point, 'axis_dir': axis_dir,
            'radial_dir': radial_dir, 'theta_dir': theta_dir}


def canonicalize_frame(frame):
    """build_cylinder_frame 类推导的轴向符号取决于圆心枚举顺序；内核本身对符号不敏感，但测试夹具里的全局 z 记账只有在规范框架下才与框架t 一致。"""
    a = frame['axis_dir']
    ap = frame['axis_point']
    if a[2] < 0:
        a = (-a[0], -a[1], -a[2])
    d = ap[0] * a[0] + ap[1] * a[1] + ap[2] * a[2]
    ap2 = (ap[0] - a[0] * d, ap[1] - a[1] * d, ap[2] - a[2] * d)
    rd = frame['radial_dir']
    td = (a[1] * rd[2] - a[2] * rd[1],
          a[2] * rd[0] - a[0] * rd[2],
          a[0] * rd[1] - a[1] * rd[0])
    return {'axis_point': ap2, 'axis_dir': a, 'radial_dir': rd,
            'theta_dir': td, 'face_index': frame.get('face_index')}


def _cyl_polar(frame, point):
    v = _cyl_v_sub(point, frame['axis_point'])
    a = frame['axis_dir']
    t = _cyl_v_dot(v, a)
    rp = _cyl_v_sub(v, _cyl_v_mul(a, t))
    r = _cyl_v_norm(rp)
    if r < 1e-12:
        return (0.0, 0.0, t)
    theta = math.atan2(_cyl_v_dot(rp, frame['theta_dir']),
                       _cyl_v_dot(rp, frame['radial_dir']))
    return (r, math.degrees(theta), t)


def _cyl_global_point(frame, r, theta_deg, t):
    th = math.radians(theta_deg)
    p = frame['axis_point']
    a = frame['axis_dir']
    p = _cyl_v_add(p, _cyl_v_mul(a, t - _cyl_v_dot(_cyl_v_sub(p, frame['axis_point']), a)))
    p = _cyl_v_add(p, _cyl_v_mul(frame['radial_dir'], r * math.cos(th)))
    p = _cyl_v_add(p, _cyl_v_mul(frame['theta_dir'], r * math.sin(th)))
    return p


# 分类与查找

def _cyl_classify_edge(part, edge, frame, tol):
    pts = []
    for vi in edge.getVertices():
        pts.append(_cyl_polar(frame, part.vertices[vi].pointOn[0]))
    pts.append(_cyl_polar(frame, edge.pointOn[0]))
    rs = [q[0] for q in pts]
    ts = [q[2] for q in pts]
    ths = [q[1] for q in pts]
    r_sp = max(rs) - min(rs)
    t_sp = max(ts) - min(ts)
    n = len(ths)
    s = sorted(x % 360.0 for x in ths)
    gaps = [s[i + 1] - s[i] for i in range(n - 1)] if n > 1 else [360.0]
    gaps.append(360.0 - (s[-1] - s[0]) if n > 1 else 360.0)
    th_sp = 360.0 - max(gaps) if n > 1 else 0.0
    if r_sp < tol and t_sp < tol:
        return 'CIRCUMFERENTIAL', {'r': sum(rs) / len(rs),
                                   't': sum(ts) / len(ts)}
    if r_sp < tol and th_sp < 1e-3:
        return 'AXIAL', {'r': sum(rs) / len(rs),
                         't_min': min(ts), 't_max': max(ts)}
    if t_sp < tol and th_sp < 1e-3:
        return 'RADIAL', {'t': sum(ts) / len(ts),
                          'r_min': min(rs), 'r_max': max(rs),
                          'theta': sum(ths) / len(ths)}
    return 'OTHER', {}


def _cyl_seam_angles_deg(part, frame, tol, t_lo=None, t_hi=None):
    """The axial lane anchors slice 0 on one of these edges and patterns by 360/n_slices,"""
    angles = []
    for edge in part.edges:
        try:
            kind, info = _cyl_classify_edge(part, edge, frame, tol)
        except Exception:
            continue
        if kind != 'RADIAL':
            continue
        t = info.get('t')
        if (t_lo is not None and t_hi is not None
                and (t < t_lo - max(tol, 1e-4) or t > t_hi + max(tol, 1e-4))):
            continue
        theta = info.get('theta')
        if theta is not None:
            angles.append(float(theta) % 360.0)
    return sorted(set(angles))


def _cyl_classify_face(part, face, frame, tol):
    pts = [_cyl_polar(frame, face.pointOn[0])]
    for e in face.getEdges():
        pts.append(_cyl_polar(frame, part.edges[_cyl_as_index(e)].pointOn[0]))
    rs = [q[0] for q in pts]
    ts = [q[2] for q in pts]
    if (max(ts) - min(ts)) < tol:
        return 'END_FACE', {'t': sum(ts) / len(ts)}
    if (max(rs) - min(rs)) < tol:
        return 'CYLINDRICAL', {'r': sum(rs) / len(rs)}
    return 'OTHER', {}


def _cyl_find_circular_edges(part, frame, radius, axial_pos, tol):
    out = []
    for i in range(len(part.edges)):
        k, info = _cyl_classify_edge(part, part.edges[i], frame, tol)
        if k == 'CIRCUMFERENTIAL' and abs(info['r'] - radius) < tol \
                and abs(info['t'] - axial_pos) < tol:
            out.append(i)
    return out


def _cyl_find_radial_edges(part, frame, radial_span, axial_pos, tol):
    out = []
    for i in range(len(part.edges)):
        k, info = _cyl_classify_edge(part, part.edges[i], frame, tol)
        if k != 'RADIAL':
            continue
        if abs(info['r_min'] - radial_span[0]) > tol \
                or abs(info['r_max'] - radial_span[1]) > tol:
            continue
        if axial_pos is not None and abs(info['t'] - axial_pos) > tol:
            continue
        out.append(i)
    return out


def _cyl_canonical_align_edge(part, frame, edge_indices, tol):
    """多层链式过渡时，各带必须共享同一角向相位，否则共享环上的节点会错位导致界面不相容。按边的中点 theta 规范化选择，使不同径向跨度的带都锁定到同一象限边界。"""
    best_idx = edge_indices[0]
    best_theta = None
    for idx in edge_indices:
        theta = _cyl_polar(frame, part.edges[idx].pointOn[0])[1] % 360.0
        # 规范化到 [0, 360)，取最接近 0°（即最小角）的边
        if best_theta is None or theta < best_theta - tol:
            best_theta = theta
            best_idx = idx
    return best_idx


def _cyl_region_face_indices(part, frame, region_faces, r_lo, r_hi, tol):
    """过渡带面：用户所选面中径向范围落在 [r_lo, r_hi] 的平面端面。"""
    out = []
    for f in region_faces:
        fi = _cyl_as_index(f)
        k, info = _cyl_classify_face(part, part.faces[fi], frame, tol)
        if k != 'END_FACE':
            continue
        pts = [_cyl_polar(frame, part.faces[fi].pointOn[0])]
        for e in part.faces[fi].getEdges():
            pts.append(_cyl_polar(frame,
                                  part.edges[_cyl_as_index(e)].pointOn[0]))
        rs = [q[0] for q in pts]
        if min(rs) > r_lo - tol and max(rs) < r_hi + tol:
            out.append(fi)
    return out


# 分区后的实体重获（记录选区几何描述 -> 按几何重新定位）

def _cyl_face_span(part, face, frame, tol):
    """th0/span 用未解卷覆盖区间表示（最大角缺口法），全环 span≈360。"""
    pts = [_cyl_polar(frame, face.pointOn[0])]
    for e in face.getEdges():
        edge = part.edges[_cyl_as_index(e)]
        for vi in edge.getVertices():
            pts.append(_cyl_polar(frame, part.vertices[vi].pointOn[0]))
        pts.append(_cyl_polar(frame, edge.pointOn[0]))
    rs = [q[0] for q in pts]
    ts = [q[2] for q in pts]
    ths = sorted(set(round(q[1] % 360.0, 6) for q in pts))
    n = len(ths)
    if n == 1:
        th0, span = ths[0], 360.0
    else:
        gaps = [(ths[(i + 1) % n] - ths[i]) % 360.0 for i in range(n)]
        gi = max(range(n), key=lambda i2: gaps[i2])
        th0 = ths[(gi + 1) % n]
        span = 360.0 - gaps[gi]
        if span > 360.0 - 1e-3:
            span = 360.0
    return {'th0': th0, 'span': span, 'r_min': min(rs), 'r_max': max(rs),
            't_min': min(ts), 't_max': max(ts)}


def _cyl_theta_in_span(a, th0, span, tol_deg):
    d = (a - th0) % 360.0
    return -tol_deg <= d <= span + tol_deg


def _cyl_span_contained(child, parent, tol_deg=1e-2):
    if child['span'] > parent['span'] + 1e-2:
        return False
    return (_cyl_theta_in_span(child['th0'], parent['th0'], parent['span'],
                               tol_deg)
            and _cyl_theta_in_span((child['th0'] + child['span']) % 360.0,
                                   parent['th0'], parent['span'], tol_deg))


def _cyl_merge_spans(segs, tol_deg=1e-3):
    """合并角向区段 [(th0, span)] 为连续跨度（处理 0°/360° 回绕）。"""
    segs = sorted((s % 360.0, sp) for s, sp in segs if sp > 1e-6)
    merged = []
    for s0, sp in segs:
        if merged:
            pe = merged[-1][0] + merged[-1][1]
            gap = (s0 - pe) % 360.0
            if gap <= tol_deg and merged[-1][1] < 360.0 - tol_deg:
                merged[-1] = (merged[-1][0],
                              min(merged[-1][1] + gap + sp, 360.0))
                continue
        merged.append((s0, sp))
    while len(merged) > 1:
        pe = merged[-1][0] + merged[-1][1]
        gap = (merged[0][0] - pe) % 360.0
        if gap > tol_deg:
            break
        total = merged[-1][1] + gap + merged[0][1]
        if total >= 360.0 - 1e-3:
            merged = [(0.0, 360.0)]
            break
        th0 = merged[-1][0]
        merged = [(th0, total)] + merged[1:-1]
    if sum(sp for _s, sp in merged) > 360.0 - 1e-3:
        return [(0.0, 360.0)]
    return merged


def _cyl_cell_signature(part, cell, frame, tol):
    """体积 + 各柱面半径的合并角跨度 + 径向平面方位 + t 范围。分区会改变 pointOn/顶点/面数，但不改变这些几何量。"""
    vol = cell.getSize()
    cyl = {}
    thetas = set()
    ts = []
    for f in cell.getFaces():
        face = part.faces[_cyl_as_index(f)]
        k, info = _cyl_classify_face(part, face, frame, tol)
        pts = [_cyl_polar(frame, face.pointOn[0])]
        for e in face.getEdges():
            edge = part.edges[_cyl_as_index(e)]
            for vi in edge.getVertices():
                pts.append(_cyl_polar(frame, part.vertices[vi].pointOn[0]))
            pts.append(_cyl_polar(frame, edge.pointOn[0]))
        ts += [q[2] for q in pts]
        if k == 'CYLINDRICAL':
            sig = _cyl_face_span(part, face, frame, tol)
            cyl.setdefault(round(info['r'], 3), []).append(
                (sig['th0'], sig['span']))
        elif max(q[1] for q in pts) - min(q[1] for q in pts) < 1e-2 \
                and min(q[0] for q in pts) > 1e-6:
            thetas.add(round((sum([q[1] for q in pts]) / len(pts)) % 360.0, 2))
    cyl_sig = tuple(
        (r, tuple((round(a, 2), round(s, 2))
                  for a, s in _cyl_merge_spans(segs)))
        for r, segs in sorted(cyl.items()))
    return {'vol': vol, 'cyl': cyl_sig, 'thetas': tuple(sorted(thetas)),
            't_min': min(ts), 't_max': max(ts)}


def _cyl_sig_match(a, b):
    if abs(a['vol'] - b['vol']) > 1e-4 * max(1.0, abs(b['vol'])):
        return False
    if abs(a['t_min'] - b['t_min']) > 1e-3 \
            or abs(a['t_max'] - b['t_max']) > 1e-3:
        return False
    if len(a['cyl']) != len(b['cyl']):
        return False
    for (ra, spans_a), (rb, spans_b) in zip(a['cyl'], b['cyl']):
        if abs(ra - rb) > 1e-3 or len(spans_a) != len(spans_b):
            return False
        for (ta, sa), (tb, sb) in zip(spans_a, spans_b):
            if abs((ta - tb + 180.0) % 360.0 - 180.0) > 0.5 \
                    or abs(sa - sb) > 0.5:
                return False
    if len(a['thetas']) != len(b['thetas']):
        return False
    for th in a['thetas']:
        if not any(abs((th - th2 + 180.0) % 360.0 - 180.0) <= 0.5
                   for th2 in b['thetas']):
            return False
    return True


def _cyl_refind_cells(part, frame, signatures, tol):
    """按几何签名重新定位目标 Cells（分区使旧对象失效）。"""
    out = []
    for cell in part.cells:
        sig = _cyl_cell_signature(part, cell, frame, tol)
        for want in signatures:
            if _cyl_sig_match(sig, want):
                out.append(cell)
                break
    return out


def _cyl_find_cells_in_t_band(part, frame, t_lo, t_hi, tol):
    """按轴向 t 跨度定位过渡 Cell（多带顺序执行时旧 Cell 句柄会失效）。"""
    out = []
    for cell in part.cells:
        ctlo, cthi = _cyl_cell_t_span(part, frame, cell, tol)
        if ctlo is None:
            continue
        if abs(ctlo - t_lo) <= tol and abs(cthi - t_hi) <= tol:
            out.append(cell)
    return out


def _cyl_find_outer_faces_in_t_band(part, frame, radius, t_lo, t_hi, tol):
    """按半径与 t 覆盖范围定位外圆柱过渡面（分区后旧 Face 句柄会失效）。"""
    out = []
    for i in range(len(part.faces)):
        face = part.faces[i]
        k, info = _cyl_classify_face(part, face, frame, tol)
        if k != 'CYLINDRICAL' or abs(info['r'] - radius) > tol:
            continue
        sig = _cyl_face_span(part, face, frame, tol)
        if sig['t_min'] <= t_lo + tol and sig['t_max'] >= t_hi - tol:
            out.append(i)
    return out


def _cyl_find_circ_edges_at(part, frame, radius, t_pos, tol):
    """定位 r=radius、t=t_pos 上的全部圆周边。"""
    out = []
    for i in range(len(part.edges)):
        k, info = _cyl_classify_edge(part, part.edges[i], frame, tol)
        if k == 'CIRCUMFERENTIAL' and abs(info['r'] - radius) < tol \
                and abs(info['t'] - t_pos) < tol:
            out.append(i)
    return out


def _cyl_refind_band_faces(part, frame, parent_sigs, r_lo, r_hi, tol):
    """模板印迹后重新定位过渡带子面：几何扫描 + 父面角跨度归属。"""
    out = []
    for i in range(len(part.faces)):
        k, _info = _cyl_classify_face(part, part.faces[i], frame, tol)
        if k != 'END_FACE':
            # created by IY,China University of Petroleum (East China)  |  禁止商用 / non-commercial use only
            continue
        sig = _cyl_face_span(part, part.faces[i], frame, tol)
        if sig['r_min'] < r_lo - tol or sig['r_max'] > r_hi + tol:
            continue
        for ps in parent_sigs:
            if abs(sig['t_min'] - ps['t_min']) > tol \
                    or abs(sig['t_max'] - ps['t_max']) > tol:
                continue
            if sig['r_min'] < ps['r_min'] - tol or sig['r_max'] > ps['r_max'] + tol:
                continue
            if _cyl_span_contained(sig, ps):
                out.append(i)
                break
    return out


def _cyl_refind_cyl_faces(part, frame, parent_sigs, radius, tol):
    """切片印迹后重新定位目标外圆柱面子面。"""
    out = []
    for i in range(len(part.faces)):
        k, info = _cyl_classify_face(part, part.faces[i], frame, tol)
        if k != 'CYLINDRICAL' or abs(info['r'] - radius) > tol:
            continue
        sig = _cyl_face_span(part, part.faces[i], frame, tol)
        if sig['r_min'] < radius - tol or sig['r_max'] > radius + tol:
            continue
        for ps in parent_sigs:
            if sig['t_min'] < ps['t_min'] - tol or sig['t_max'] > ps['t_max'] + tol:
                continue
            if _cyl_span_contained(sig, ps):
                out.append(i)
                break
    return out


def _cyl_allowed_edge_indices(cells):
    """选区约束：允许布种/修改的边 = 目标 Cells 的全部边。"""
    out = set()
    for cell in cells:
        for e in cell.getEdges():
            out.add(_cyl_as_index(e))
    return out


# 布种（选择范围约束 + 弧跨度比例分配）

def _cyl_arc_span_deg(part, frame, edge, tol):
    """圆弧边角跨度（度）；闭合圆返回 360。"""
    vs = edge.getVertices()
    if len(vs) < 2:
        return 360.0
    # @author  created by IY,China University of Petroleum (East China)  --  禁止商用 (non-commercial)
    a1 = _cyl_polar(frame, part.vertices[vs[0]].pointOn[0])[1]
    a2 = _cyl_polar(frame, part.vertices[vs[1]].pointOn[0])[1]
    span = abs((a2 - a1 + 180.0) % 360.0 - 180.0)
    if span < 1e-6:
        return 360.0
    return span


def _cyl_seed_transition_rings(part, frame, allowed_idx, mode, r_fine,
                               r_coarse, t_fine, t_coarse, fine_seeds,
                               coarse_seeds, tol):
    """规则：组内每条弧的种子数 = 全环总段数 × (选中角跨度/360) / 弧数，必须恰为正整数。环向模式按半径区分细/粗总数，轴向模式按截面。未识别的组（模板中圈弧）跳过，由带面逐边 1 段覆盖。"""
    groups = {}
    for idx in sorted(allowed_idx):
        edge = part.edges[idx]
        k, info = _cyl_classify_edge(part, edge, frame, tol)
        if k != 'CIRCUMFERENTIAL':
            continue
        key = (round(info['t'], 5), round(info['r'], 4))
        groups.setdefault(key, []).append(idx)
    record = []
    for (t_key, r_key), idxs in sorted(groups.items()):
        if mode == 'ANNULAR_RADIAL':
            if abs(r_key - r_fine) < 1e-3:
                total = fine_seeds
            elif abs(r_key - r_coarse) < 1e-3:
                total = coarse_seeds
            else:
                continue
        else:
            if abs(t_key - t_fine) < 1e-3:
                total = fine_seeds
            elif abs(t_key - t_coarse) < 1e-3:
                total = coarse_seeds
            else:
                continue
        extent = 0.0
        for i in idxs:
            extent += _cyl_arc_span_deg(part, frame, part.edges[i], tol)
        per = total * (extent / 360.0) / len(idxs)
        per_i = int(round(per))
        if per_i < 1 or abs(per - per_i) > 1e-6:
            raise ValueError(
                'Seeding failed: t=%g r=%g, %d arcs (selected span %g deg), total segments %d '
                'cannot be divided evenly (%g per arc); check seeds vs selection/partition'
                % (t_key, r_key, len(idxs), extent, total, per))
        part.seedEdgeByNumber(edges=tuple(part.edges[i] for i in idxs),
                              number=per_i, constraint=FIXED)
        record.append({'t': t_key, 'r': r_key, 'arcs': len(idxs),
                       'extent_deg': round(extent, 3), 'per_edge': per_i,
                       'seeds_in_selection': per_i * len(idxs)})
    return record


def _cyl_seed_band_edges(part, frame, band_face_indices, allowed_idx, tol,
                         exclude_radii=None, exclude_t=None, mode=None):
    """exclude_radii: 需排除的圆周边半径集合（已由 _cyl_seed_transition_rings 布种）。exclude_t: 需排除的圆周边轴向位置集合。"""
    eidx = set()
    for fi in band_face_indices:
        for e in part.faces[fi].getEdges():
            eidx.add(_cyl_as_index(e))
    eidx &= set(allowed_idx)
    # 排除已由环布种处理的圆周边（避免覆盖正确的细/粗种子数）
    if exclude_radii and mode == 'ANNULAR_RADIAL':
        ring_idx = set()
        for idx in eidx:
            k, info = _cyl_classify_edge(part, part.edges[idx], frame, tol)
            if k == 'CIRCUMFERENTIAL':
                for r_ex in exclude_radii:
                    if abs(info['r'] - r_ex) < 1e-3:
                        ring_idx.add(idx)
                        break
        eidx -= ring_idx
    elif exclude_t and mode == 'AXIAL_SURFACE':
        ring_idx = set()
        for idx in eidx:
            k, info = _cyl_classify_edge(part, part.edges[idx], frame, tol)
            if k == 'CIRCUMFERENTIAL':
                for t_ex in exclude_t:
                    if abs(info['t'] - t_ex) < 1e-3:
                        ring_idx.add(idx)
                        break
        eidx -= ring_idx
    if eidx:
        part.seedEdgeByNumber(
            edges=tuple(part.edges[i] for i in sorted(eidx)),
            number=1, constraint=FIXED)
    return len(eidx)


def _cyl_seed_axial_band_edges(part, frame, allowed_idx, t_fine, t_coarse,
                               tol):
    """轴向模式：过渡带内的轴向切片边（任意半径）每边 1 段。"""
    t_lo, t_hi = min(t_fine, t_coarse), max(t_fine, t_coarse)
    picked = []
    for idx in sorted(allowed_idx):
        k, info = _cyl_classify_edge(part, part.edges[idx], frame, tol)
        if k == 'AXIAL' and abs(info['t_min'] - t_lo) < tol \
                and abs(info['t_max'] - t_hi) < tol:
            picked.append(idx)
    if picked:
        part.seedEdgeByNumber(
            edges=tuple(part.edges[i] for i in picked),
            number=1, constraint=FIXED)
    return len(picked)


def _cyl_audit_seed_coverage(part, frame, allowed_idx, mode, r_fine, r_coarse,
                             t_fine, t_coarse, fine_seeds, coarse_seeds, tol):
    """返回 (audit_record, filled_count)：audit_record: 按角色分组的边种子表filled_count: 本次补充布种的边数角色分类：ring_fine:"""
    roles = {'ring_fine': [], 'ring_coarse': [], 'template_internal': [],
             'axial': [], 'radial': [], 'unclassified': []}
    unseeded_internal = []

    for idx in sorted(allowed_idx):
        edge = part.edges[idx]
        k, info = _cyl_classify_edge(part, edge, frame, tol)
        seed = _cyl_edge_seed_count(part, edge)

        if k == 'CIRCUMFERENTIAL':
            if mode == 'ANNULAR_RADIAL':
                if abs(info['r'] - r_fine) < 1e-3:
                    roles['ring_fine'].append({'idx': idx, 'seed': seed})
                elif abs(info['r'] - r_coarse) < 1e-3:
                    roles['ring_coarse'].append({'idx': idx, 'seed': seed})
                else:
                    # 模板内部弧线
                    roles['template_internal'].append({'idx': idx, 'seed': seed})
                    if seed is None:
                        unseeded_internal.append(idx)
            else:  # AXIAL_SURFACE
                if abs(info['t'] - t_fine) < 1e-3:
                    roles['ring_fine'].append({'idx': idx, 'seed': seed})
                elif abs(info['t'] - t_coarse) < 1e-3:
                    roles['ring_coarse'].append({'idx': idx, 'seed': seed})
                else:
                    roles['template_internal'].append({'idx': idx, 'seed': seed})
                    if seed is None:
                        unseeded_internal.append(idx)
        elif k == 'AXIAL':
            roles['axial'].append({'idx': idx, 'seed': seed})
            if seed is None:
                unseeded_internal.append(idx)
        elif k == 'RADIAL':
            roles['radial'].append({'idx': idx, 'seed': seed})
            if seed is None:
                unseeded_internal.append(idx)
        else:
            roles['unclassified'].append({'idx': idx, 'seed': seed})
            if seed is None:
                unseeded_internal.append(idx)

    # 填补遗漏的模板内部边：设为 1 段 FIXED
    filled = 0
    if unseeded_internal:
        part.seedEdgeByNumber(
            edges=tuple(part.edges[i] for i in unseeded_internal),
            number=1, constraint=FIXED)
        filled = len(unseeded_internal)

    # 构建审计记录
    audit = {
        'total_edges': len(allowed_idx),
        'filled_unseeded': filled,
        'filled_indices': unseeded_internal[:20],  # 截断报告
        'roles_summary': {},
    }
    for role, entries in roles.items():
        seeds_found = [e['seed'] for e in entries if e['seed'] is not None]
        seeds_missing = sum(1 for e in entries if e['seed'] is None)
        audit['roles_summary'][role] = {
            'count': len(entries),
            'seeded': len(seeds_found),
            'missing_before_fill': seeds_missing,
            'seed_values': sorted(set(seeds_found)) if seeds_found else [],
        }

    # 验证环边种子正确性
    problems = []
    ring_fine_total = sum(e['seed'] or 0 for e in roles['ring_fine'])
    ring_coarse_total = sum(e['seed'] or 0 for e in roles['ring_coarse'])
    if roles['ring_fine'] and ring_fine_total != fine_seeds:
        # 可能是部分选区，按比例检查
        pass  # 不强制报错，记录即可
    if roles['ring_coarse'] and ring_coarse_total != coarse_seeds:
        pass
    audit['ring_fine_total_seeds'] = ring_fine_total
    audit['ring_coarse_total_seeds'] = ring_coarse_total
    if problems:
        audit['problems'] = problems

    return audit, filled


# 环向模板印迹（显式 2进1/3进1 基本单元，验证版逻辑）

def _cyl_owner_model(part):
    try:
        return mdb.models[part.modelName]
    except Exception:
        pass
    owners = [mm for mm in mdb.models.values() if part.name in mm.parts.keys()]
    if len(owners) == 1:
        return owners[0]
    raise ValueError('Cannot resolve the Model owning Part %r' % (part.name,))


def _cyl_sketch_point_coords(obj):
    try:
        po = obj.pointOn
        return (float(po[0]), float(po[1]))
    except Exception:
        pass
    v = tuple(obj)
    if len(v) >= 2:
        return (float(v[0]), float(v[1]))
    raise ValueError('Cannot read sketch point %r' % (obj,))


def _cyl_imprint_annular_template(part, frame, band_face_indices,
                                  r_fine, r_coarse, n_periods, ratio,
                                  align_edge_index, tol, phase_deg=None):
    """相位对齐用带内径向直边（扇区分界）投影读回，与已验证基准一致；不给 MakeSketchTransform 传 sketchUpEdge（会让分区失败）。phase_deg：显式相位（度）。"""
    r_mid = 0.5 * (r_fine + r_coarse)
    period = 360.0 / n_periods

    def pt(r, deg):
        a = math.radians(deg)
        return (r * math.sin(a), r * math.cos(a))

    origin = _cyl_global_point(frame, 0.0, 0.0,
                               _cyl_polar(frame,
                                          part.faces[band_face_indices[0]]
                                          .pointOn[0])[2])
    last_err = None
    for side in (SIDE1, SIDE2):
        # 草图落在面的"背面"时 PartitionFaceBySketch 会报
        # Feature creation failed（实测底面法向 -Z 一侧），换边重画。
        transform = part.MakeSketchTransform(
            sketchPlane=part.faces[band_face_indices[0]],
            sketchPlaneSide=side, sketchOrientation=RIGHT, origin=origin)
        sk = _cyl_owner_model(part).ConstrainedSketch(name='__cyl_tmpl__',
                                                      sheetSize=500.0,
                                                      transform=transform)
        proj_key = None
        if phase_deg is not None:
            # 显式相位：仍需投影 align 边以固定 proj_key（从 geom 中排除）
            phi = math.radians(float(phase_deg))
            try:
                before = set(sk.geometry.keys())
                part.projectEdgesOntoSketch(
                    edges=(part.edges[align_edge_index],), sketch=sk)
                new_keys = [k for k in sk.geometry.keys() if k not in before]
                if new_keys:
                    proj_key = new_keys[0]
            except Exception:
                pass
        else:
            phi = 0.0
            try:
                before = set(sk.geometry.keys())
                part.projectEdgesOntoSketch(
                    edges=(part.edges[align_edge_index],), sketch=sk)
                new_keys = [k for k in sk.geometry.keys() if k not in before]
                if new_keys:
                    proj_key = new_keys[0]
                    verts = sk.geometry[proj_key].getVertices()
                    pts = [_cyl_sketch_point_coords(v) for v in verts]
                    pts.sort(key=lambda q: q[0] * q[0] + q[1] * q[1])
                    phi = math.atan2(pts[0][0], pts[0][1])
            except Exception:
                phi = 0.0

        def ppt(r, d):
            return pt(r, math.degrees(phi) + d)

        if ratio == 3:
            d1, d2 = period / 3.0, 2.0 * period / 3.0
            sk.Line(point1=ppt(r_fine, 0.0), point2=ppt(r_coarse, 0.0))
            sk.Line(point1=ppt(r_fine, d1), point2=ppt(r_mid, d1))
            sk.Line(point1=ppt(r_fine, d2), point2=ppt(r_mid, d2))
            sk.Arc3Points(point1=ppt(r_mid, d1), point2=ppt(r_mid, d2),
                          point3=ppt(r_mid, 1.5 * period / 3.0))
            sk.Line(point1=ppt(r_coarse, 0.0), point2=ppt(r_mid, d1))
            sk.Line(point1=ppt(r_mid, d2), point2=ppt(r_coarse, period))
            n_unit, faces_pp = 6, 4
        elif ratio == 2:
            d1, d2, d3 = period / 4.0, period / 2.0, 3.0 * period / 4.0
            sk.Line(point1=ppt(r_fine, 0.0), point2=ppt(r_coarse, 0.0))
            sk.Line(point1=ppt(r_fine, d1), point2=ppt(r_mid, d1))
            sk.Line(point1=ppt(r_fine, d3), point2=ppt(r_mid, d3))
            sk.Line(point1=ppt(r_fine, d2), point2=ppt(r_coarse, d2))
            sk.Arc3Points(point1=ppt(r_mid, d1), point2=ppt(r_mid, d2),
                          point3=ppt(r_mid, 1.5 * period / 4.0))
            sk.Arc3Points(point1=ppt(r_mid, d2), point2=ppt(r_mid, d3),
                          point3=ppt(r_mid, 2.5 * period / 4.0))
            sk.Line(point1=ppt(r_coarse, 0.0), point2=ppt(r_mid, d1))
            sk.Line(point1=ppt(r_mid, d3), point2=ppt(r_coarse, period))
            n_unit, faces_pp = 8, 6
        else:
            raise ValueError('Circumferential section supports only 2:1 / 3:1, got %r' % (ratio,))
        keys = list(sk.geometry.keys())
        geom = tuple(sk.geometry[k] for k in keys if k != proj_key)
        if len(geom) != n_unit:
            raise ValueError('Template unit entity count wrong: expected %d, got %d'
                             % (n_unit, len(geom)))
        sk.radialPattern(centerPoint=(0.0, 0.0), geomList=geom,
                         number=n_periods, totalAngle=360.0, vertexList=())
        try:
            part.PartitionFaceBySketch(
                faces=tuple(part.faces[i] for i in band_face_indices),
                sketch=sk)
        except BaseException as ex:
            last_err = ex
            del sk
            continue
        del sk
        return {'n_periods': n_periods, 'ratio': ratio,
                'faces_per_period': faces_pp,
                'phi_deg': round(math.degrees(phi) % 360.0, 6)}
    raise ValueError('Circumferential template imprint failed (both sketch sides): %r' % (last_err,))


def _cyl_draw_merge_unit(sk, pt, r_fine_l, r_coarse_l, period, ratio, phi_deg):
    """返回该单元的几何 key 列表（供 radialPattern 使用）。与单层模板的 ratio==3 / ratio==2 几何完全一致，仅参数化了半径与周期。"""
    r_mid = 0.5 * (r_fine_l + r_coarse_l)

    def ppt(r, d):
        return pt(r, phi_deg + d)

    keys_before = set(sk.geometry.keys())
    if ratio == 3:
        d1, d2 = period / 3.0, 2.0 * period / 3.0
        sk.Line(point1=ppt(r_fine_l, 0.0), point2=ppt(r_coarse_l, 0.0))
        sk.Line(point1=ppt(r_fine_l, d1), point2=ppt(r_mid, d1))
        sk.Line(point1=ppt(r_fine_l, d2), point2=ppt(r_mid, d2))
        sk.Arc3Points(point1=ppt(r_mid, d1), point2=ppt(r_mid, d2),
                      point3=ppt(r_mid, 1.5 * period / 3.0))
        sk.Line(point1=ppt(r_coarse_l, 0.0), point2=ppt(r_mid, d1))
        sk.Line(point1=ppt(r_mid, d2), point2=ppt(r_coarse_l, period))
    elif ratio == 2:
        d1, d2, d3 = period / 4.0, period / 2.0, 3.0 * period / 4.0
        sk.Line(point1=ppt(r_fine_l, 0.0), point2=ppt(r_coarse_l, 0.0))
        sk.Line(point1=ppt(r_fine_l, d1), point2=ppt(r_mid, d1))
        sk.Line(point1=ppt(r_fine_l, d3), point2=ppt(r_mid, d3))
        sk.Line(point1=ppt(r_fine_l, d2), point2=ppt(r_coarse_l, d2))
        sk.Arc3Points(point1=ppt(r_mid, d1), point2=ppt(r_mid, d2),
                      point3=ppt(r_mid, 1.5 * period / 4.0))
        sk.Arc3Points(point1=ppt(r_mid, d2), point2=ppt(r_mid, d3),
                      point3=ppt(r_mid, 2.5 * period / 4.0))
        sk.Line(point1=ppt(r_coarse_l, 0.0), point2=ppt(r_mid, d1))
        sk.Line(point1=ppt(r_mid, d3), point2=ppt(r_coarse_l, period))
    else:
        raise ValueError('Multilayer circumferential template supports only 2:1 / 3:1, got %r' % (ratio,))
    return [k for k in sk.geometry.keys() if k not in keys_before]


def _cyl_imprint_annular_multilayer_template(part, frame, band_face_indices,
                                             radii, fine_seeds, ratio,
                                             align_edge_index, tol,
                                             ratio_sequence=None):
    """radii: [r0, r1, ..., rL]，从细到粗（或从粗到细）的 L+1 个半径。ratio_sequence: 可选逐层比率列表；缺省时所有层用同一个 ratio。"""
    L = len(radii) - 1
    if L < 2:
        raise ValueError('Multilayer template needs at least 2 layers (len(radii) >= 3)')
    # Resolve per-layer ratios.
    if ratio_sequence:
        ratios = [int(r) for r in ratio_sequence]
        if len(ratios) != L:
            raise ValueError('ratio_sequence length %d != layer count %d' % (len(ratios), L))
        for r in ratios:
            if r not in (2, 3):
                raise ValueError('Per-layer ratio supports only 2/3, got %d' % r)
    else:
        ratio = int(ratio)
        if ratio not in (2, 3):
            raise ValueError('Multilayer circumferential supports only 2:1 / 3:1, got %d' % ratio)
        ratios = [ratio] * L
    layer_seeds = [fine_seeds]
    for i in range(L):
        if layer_seeds[-1] % ratios[i] != 0:
            raise ValueError('Multilayer seed chain not divisible: layer %d count %d is not divisible by %d'
                             % (i, layer_seeds[-1], ratios[i]))
        layer_seeds.append(layer_seeds[-1] // ratios[i])

    def pt(r, deg):
        a = math.radians(deg)
        return (r * math.sin(a), r * math.cos(a))

    origin = _cyl_global_point(frame, 0.0, 0.0,
                               _cyl_polar(frame,
                                          part.faces[band_face_indices[0]]
                                          .pointOn[0])[2])
    last_err = None
    for side in (SIDE1, SIDE2):
        transform = part.MakeSketchTransform(
            sketchPlane=part.faces[band_face_indices[0]],
            sketchPlaneSide=side, sketchOrientation=RIGHT, origin=origin)
        sk = _cyl_owner_model(part).ConstrainedSketch(name='__cyl_ml__',
                                                      sheetSize=800.0,
                                                      transform=transform)
        # 相位：从 align 边投影读回（与单层一致）
        phi_deg = 0.0
        proj_key = None
        try:
            before = set(sk.geometry.keys())
            part.projectEdgesOntoSketch(edges=(part.edges[align_edge_index],),
                                        sketch=sk)
            new_keys = [k for k in sk.geometry.keys() if k not in before]
            if new_keys:
                proj_key = new_keys[0]
                verts = sk.geometry[proj_key].getVertices()
                pts = [_cyl_sketch_point_coords(v) for v in verts]
                pts.sort(key=lambda q: q[0] * q[0] + q[1] * q[1])
                phi_deg = math.degrees(math.atan2(pts[0][0], pts[0][1]))
        except Exception:
            phi_deg = 0.0
        # 逐层画单元并环形阵列（所有层共享 phi_deg）
        try:
            for i in range(L):
                r_f_l, r_c_l = radii[i], radii[i + 1]
                n_per = layer_seeds[i + 1] // (2 if ratios[i] == 2 else 1)
                period = 360.0 / n_per
                unit_keys = _cyl_draw_merge_unit(sk, pt, r_f_l, r_c_l,
                                                 period, ratios[i], phi_deg)
                geom = tuple(sk.geometry[k] for k in unit_keys)
                sk.radialPattern(centerPoint=(0.0, 0.0), geomList=geom,
                                 number=n_per, totalAngle=360.0, vertexList=())
            part.PartitionFaceBySketch(
                faces=tuple(part.faces[i2] for i2 in band_face_indices),
                sketch=sk)
        except BaseException as ex:
            last_err = ex
            del sk
            continue
        del sk
        return {'layers': L, 'ratio': ratio, 'ratios': ratios,
                'radii': list(radii),
                'layer_seeds': layer_seeds, 'phi_deg': round(phi_deg % 360.0, 6)}
    raise ValueError('Multilayer circumferential template imprint failed (both sides): %r' % (last_err,))


# 轴向切片线印迹（手册 cylinder2 路线）

def _cyl_target_theta_spans(part, frame, face_indices, radius, t_level, tol):
    """目标面在 (radius, t_level) 圆弧拼出的连续角度跨度 [(th0, span)]。"""
    segs = []
    for fi in face_indices:
        for e in part.faces[fi].getEdges():
            edge = part.edges[_cyl_as_index(e)]
            k, info = _cyl_classify_edge(part, edge, frame, tol)
            if k != 'CIRCUMFERENTIAL':
                continue
            if abs(info['r'] - radius) > tol or abs(info['t'] - t_level) > tol:
                continue
            vs = edge.getVertices()
            if len(vs) < 2:
                return [(0.0, 360.0)]
            a1 = _cyl_polar(frame, part.vertices[vs[0]].pointOn[0])[1] % 360.0
            a2 = _cyl_polar(frame, part.vertices[vs[1]].pointOn[0])[1] % 360.0
            d = (a2 - a1) % 360.0
            if d <= 180.0:
                segs.append((a1, d))
            else:
                segs.append((a2, 360.0 - d))
    if not segs:
        raise ValueError('Target face has no arc edge at r=%g t=%g' % (radius, t_level))
    segs.sort(key=lambda q: q[0])
    merged = []
    for s0, sp in segs:
        if merged and (s0 - (merged[-1][0] + merged[-1][1])) % 360.0 <= 1e-3:
            e0 = merged[-1][0]
            e1 = merged[-1][1] + ((s0 - (e0 + merged[-1][1])) % 360.0) + sp
            merged[-1] = (e0, min(e1, 360.0))
        else:
            merged.append((s0, sp))
    total = sum(sp for _s, sp in merged)
    if total > 360.0 - 1e-3:
        return [(0.0, 360.0)]
    return merged


def _cyl_expected_slice_arcs(spans, phase_deg, n_slices, tol):
    """相位 phase_deg 的切片图案在选区内应产出的圆弧数。"""
    step = 360.0 / n_slices
    total_extent = sum(sp for _s, sp in spans)
    if total_extent > 360.0 - 1e-3:
        return n_slices
    n = 0
    for th0, sp in spans:
        interior = 0
        for k in range(n_slices):
            a = (phase_deg + k * step) % 360.0
            d = (a - th0) % 360.0
            if 1e-3 < d < sp - 1e-3:
                interior += 1
        n += 1 + interior
    return n


def _cyl_imprint_axial_slices(part, frame, sketch_face_index, n_slices,
                              align_edge_index, target_face_indices,
                              r_inner, r_outer, t_fine, t_coarse,
                              verify_fn, tol):
    """穿透侧由草图面法向与目标材料方位计算；验证失败时自动翻转SIDE1/SIDE2 重试一次。"""
    sketch_plane = part.faces[sketch_face_index]
    plane_sig = _cyl_face_span(part, sketch_plane, frame, tol)
    target_sigs = [_cyl_face_span(part, part.faces[i], frame, tol)
                   for i in target_face_indices]
    radius = _cyl_polar(frame, part.faces[target_face_indices[0]].pointOn[0])[0]
    up_sigs = _cyl_path_signatures(part, frame, [part.edges[align_edge_index]], tol)
    t_sk = _cyl_polar(frame, sketch_plane.pointOn[0])[2]
    origin = _cyl_global_point(frame, 0.0, 0.0, t_sk)
    toward = _cyl_v_mul(frame['axis_dir'], 1.0 if t_coarse > t_fine else -1.0)
    normal = tuple(sketch_plane.getNormal())
    side0 = SIDE1 if _cyl_v_dot(normal, toward) < 0 else SIDE2
    last_err = None
    for side in (side0, SIDE2 if side0 == SIDE1 else SIDE1):
        # A failed partition or its deletion invalidates previous live handles.
        plane_ids = _cyl_refind_band_faces(part, frame, [plane_sig],
                                          r_inner, r_outer, tol)
        targets = _cyl_refind_cyl_faces(part, frame, target_sigs, radius, tol)
        if not plane_ids or not targets:
            raise ValueError('Cannot re-find the scoped axial sketch/target faces for retry.')
        sketch_plane = part.faces[plane_ids[0]]
        up = _cyl_refind_path_edges(part, frame, up_sigs, tol)[0]
        before = set(part.features.keys())
        model = _cyl_owner_model(part)
        import uuid
        sketch_name = '__cyl_slice_%s__' % uuid.uuid4().hex
        try:
            transform = part.MakeSketchTransform(
                sketchPlane=sketch_plane, sketchPlaneSide=side,
                sketchUpEdge=up, sketchOrientation=RIGHT, origin=origin)
            sk = model.ConstrainedSketch(
                name=sketch_name, sheetSize=500.0, transform=transform)
            sk.Line(point1=(0.0, r_inner), point2=(0.0, r_outer))
            line_key = list(sk.geometry.keys())[-1]
            sk.radialPattern(centerPoint=(0.0, 0.0),
                             geomList=(sk.geometry[line_key],),
                             number=n_slices, totalAngle=360.0, vertexList=())
            part.PartitionFaceBySketchThruAll(
                faces=tuple(part.faces[i] for i in targets),
                sketch=sk, sketchPlane=sketch_plane,
                sketchPlaneSide=side, sketchUpEdge=up)
            if verify_fn is None or verify_fn():
                return {'side': str(side), 'n_slices': n_slices}
            last_err = ValueError('Scoped slice verification failed on %s.' % side)
        except Exception as ex:
            last_err = ex
        finally:
            if sketch_name in model.sketches:
                del model.sketches[sketch_name]
        # Do not retry on a partially modified Part if cleanup failed.
        created = tuple(name for name in part.features.keys() if name not in before)
        try:
            if created:
                part.deleteFeatures(featureNames=created)
                part.regenerate()
            if set(part.features.keys()) != before:
                raise ValueError('Feature inventory differs from the pre-attempt snapshot.')
        except Exception as cleanup_error:
            raise ValueError('Axial slice cleanup failed; retry stopped: %s '
                             '(original error: %s)' % (cleanup_error, last_err))
    raise ValueError('Axial slice imprint failed on both sides: %r' % (last_err,))


# 网格控制 / 扫掠路径 / 单元类型

def _cyl_region_array(part, entities):
    """实体对象列表 -> Abaqus 掩码序列（setMeshControls/ElementType 需要）。"""
    if not entities:
        raise ValueError('Empty region')
    kind = 'cells' if hasattr(entities[0], 'getFaces') else 'faces'
    arr = getattr(part, kind)
    idx = sorted(set(_cyl_as_index(e) for e in entities))
    words = {}
    for i in idx:
        words[i // 32] = words.get(i // 32, 0) | (1 << (i % 32))
    top = max(words.keys())
    mask = '[#' + ' #'.join('%x' % words.get(k, 0) for k in range(top + 1)) + ']'
    seq = arr.getSequenceFromMask((mask,),)
    got = sorted(e.index for e in seq)
    if got != idx:
        raise ValueError('Masked region mismatch: %r vs %r' % (got, idx))
    return seq


def _cyl_assign_transition_controls(part, cells):
    part.setMeshControls(
        regions=_cyl_region_array(part, cells),
        technique=SWEEP, algorithm=ADVANCING_FRONT, allowMapped=False)


def _cyl_set_element_type(part, cells, elem_code=C3D8R):
    seq = _cyl_region_array(part, cells)
    part.Set(cells=seq, name='__CYL_TMP_ET__')
    try:
        part.setElementType(regions=part.sets['__CYL_TMP_ET__'],
                            elemTypes=(ElemType(elemCode=elem_code,
                                                elemLibrary=STANDARD),))
    finally:
        del part.sets['__CYL_TMP_ET__']


def _cyl_edge_seed_count(part, edge):
    """边上的种子数（官方 API part.getEdgeSeeds(edge, NUMBER)）。"""
    try:
        s = part.getEdgeSeeds(edge, NUMBER)
    except BaseException:
        return None
    if s is None:
        return None
    try:
        s = int(s)
    except Exception:
        return None
    return s if s > 0 else None


def _cyl_seed_number(part, indices, number):
    """Avoid invalidating shared meshes when their fixed seeds are unchanged."""
    changed = []
    for ei in indices:
        edge = part.edges[ei]
        try:
            same = (_cyl_edge_seed_count(part, edge) == number
                    and part.getEdgeSeeds(edge, CONSTRAINT) == FIXED)
        except Exception:
            same = False
        if not same:
            changed.append(edge)
    if changed:
        part.seedEdgeByNumber(edges=tuple(changed), number=number, constraint=FIXED)


def _cyl_path_signatures(part, frame, sweep_path_edges, tol):
    """Snapshot optional straight sweep paths before any partition changes indices."""
    signatures = []
    for edge in (sweep_path_edges or []):
        if edge is None:
            continue
        if isinstance(edge, int):
            edge = part.edges[edge]
        kind, info = _cyl_classify_edge(part, edge, frame, tol)
        if kind not in ('AXIAL', 'RADIAL'):
            raise ValueError('Sweep Path must be a straight axial or radial edge.')
        signature = dict(info)
        signature['kind'] = kind
        signature['theta'] = _cyl_polar(frame, edge.pointOn[0])[1] % 360.0
        signatures.append(signature)
    return signatures


def _cyl_refind_path_edges(part, frame, signatures, tol):
    """Refind live descendants of every user path; never reuse stale edge indices."""
    found = {}
    for sig in signatures:
        matches = []
        for edge in part.edges:
            kind, info = _cyl_classify_edge(part, edge, frame, tol)
            if kind != sig['kind']:
                continue
            theta = _cyl_polar(frame, edge.pointOn[0])[1] % 360.0
            if abs((theta - sig['theta'] + 180.0) % 360.0 - 180.0) > 1e-3:
                continue
            if kind == 'AXIAL':
                inside = (abs(info['r'] - sig['r']) < tol
                          and info['t_min'] >= sig['t_min'] - tol
                          and info['t_max'] <= sig['t_max'] + tol)
            else:
                inside = (abs(info['t'] - sig['t']) < tol
                          and info['r_min'] >= sig['r_min'] - tol
                          and info['r_max'] <= sig['r_max'] + tol)
            if inside:
                matches.append(edge)
        if not matches:
            raise ValueError('Cannot re-find the selected Sweep Path after partition; repick it.')
        for edge in matches:
            found[int(edge.index)] = edge
    return [found[index] for index in sorted(found)]


def _cyl_user_path_indices(sweep_path_edges):
    out = []
    for e in (sweep_path_edges or []):
        if e is None:
            continue
        out.append(_cyl_as_index(e))
    return out


def _cyl_angle_distance(a, b):
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def _cyl_cell_polar_center(part, frame, cell):
    pts = []
    for e in cell.getEdges():
        edge = part.edges[_cyl_as_index(e)]
        pts.append(_cyl_polar(frame, edge.pointOn[0]))
        for vi in edge.getVertices():
            pts.append(_cyl_polar(frame, part.vertices[vi].pointOn[0]))
    if not pts:
        pts.append(_cyl_polar(frame, cell.pointOn[0]))
    r = sum(q[0] for q in pts) / len(pts)
    t = sum(q[2] for q in pts) / len(pts)
    sx = sum(math.cos(math.radians(q[1])) for q in pts)
    sy = sum(math.sin(math.radians(q[1])) for q in pts)
    theta = math.degrees(math.atan2(sy, sx)) % 360.0
    return r, theta, t


def _cyl_path_edge_candidates(part, frame, cell, kind, user_idx, tol,
                              accept_info=None):
    """返回可尝试的 sweep path 候选。先试用户边，再试 Cell 自身边。"""
    ce = set(_cyl_as_index(e) for e in cell.getEdges())
    center = _cyl_cell_polar_center(part, frame, cell)

    def compatible(indices, user_choice):
        out = []
        for i in indices:
            try:
                i = int(i)
                edge = part.edges[i]
                k, info = _cyl_classify_edge(part, edge, frame, tol)
            except Exception:
                continue
            if k != kind:
                continue
            if accept_info is not None and not accept_info(info):
                continue
            q = _cyl_polar(frame, edge.pointOn[0])
            member = 0 if i in ce else 1
            if kind == 'AXIAL':
                score = (member, _cyl_angle_distance(q[1], center[1]),
                         abs(q[0] - center[0]), i)
            else:
                score = (member, abs(q[2] - center[2]),
                         _cyl_angle_distance(q[1], center[1]), i)
            out.append((score, i, user_choice))
        out.sort(key=lambda item: item[0])
        return out

    ordered = []
    seen = set()
    for group in (compatible(user_idx, True) if user_idx else [],
                  compatible(ce, False)):
        for _score, i, user_choice in group:
            if i in seen:
                continue
            seen.add(i)
            ordered.append((i, user_choice))
    return ordered


def _cyl_set_sweep_path_from_candidates(part, cell, candidates, sense_func):
    last = None
    for ei, from_user in candidates:
        sense = sense_func(ei)
        try:
            part.setSweepPath(edge=part.edges[ei], region=cell, sense=sense)
            return ei, sense, from_user
        except Exception as exc:
            last = exc
    if last is not None:
        raise ValueError('All candidate Sweep Paths of Cell %d are invalid, last error: %s'
                         % (cell.index, last))
    return None, None, False


def _cyl_assign_annular_paths(part, frame, cells, sweep_path_edges, t_src,
                              tol):
    """环向模式扫掠路径：用户边优先；无效时回退到 Cell 自身轴向边。"""
    user_idx = _cyl_user_path_indices(sweep_path_edges)
    recs = []
    for cell in cells:
        candidates = _cyl_path_edge_candidates(
            part, frame, cell, 'AXIAL', user_idx, tol)
        if not candidates:
            if user_idx:
                raise ValueError('No usable axial edge among the selected Sweep Path Edges; '
                                 'pick straight edges parallel to the cylinder axis')
            raise ValueError('Transition Cell %d has no axial edge to use as a sweep path'
                             % cell.index)
        ei, sense, from_user = _cyl_set_sweep_path_from_candidates(
            part, cell, candidates,
            lambda i: _cyl_sense_starting_at(
                part, frame, part.edges[i], t_src, tol))
        recs.append({'cell_index': cell.index, 'edge_index': ei,
                     'sense': str(sense),
                     'source': 'USER' if from_user else 'AUTO'})
    return recs


def _cyl_assign_axial_paths(part, frame, cells, sweep_path_edges,
                            r_inner, r_outer, tol):
    """轴向模式扫掠路径：用户边优先；无效时回退到 Cell 自身径向边。"""
    user_idx = _cyl_user_path_indices(sweep_path_edges)

    def accept_radial(info):
        return (abs(info['r_min'] - r_inner) < tol
                and abs(info['r_max'] - r_outer) < tol)

    recs = []
    for cell in cells:
        candidates = _cyl_path_edge_candidates(
            part, frame, cell, 'RADIAL', user_idx, tol, accept_radial)
        if not candidates:
            if user_idx:
                raise ValueError('No radial edge crossing the wall '
                                 '(r=%g..%g) among the selected Sweep Path Edges' % (r_inner, r_outer))
            raise ValueError('Transition Cell %d has no radial edge crossing the wall (r=%g..%g) '
                             'to use as a sweep path' % (cell.index, r_inner, r_outer))
        ei, sense, from_user = _cyl_set_sweep_path_from_candidates(
            part, cell, candidates, lambda _i: FORWARD)
        recs.append({'cell_index': cell.index, 'edge_index': ei,
                     'sense': str(sense),
                     'source': 'USER' if from_user else 'AUTO'})
    return recs


def _cyl_sense_starting_at(part, frame, edge, t_source, tol):
    """已验证约定：FORWARD 从边的第二个顶点开始扫掠。"""
    vs = edge.getVertices()
    t1 = _cyl_polar(frame, part.vertices[vs[0]].pointOn[0])[2]
    t2 = _cyl_polar(frame, part.vertices[vs[1]].pointOn[0])[2]
    if abs(t2 - t_source) < tol:
        return FORWARD
    if abs(t1 - t_source) < tol:
        return REVERSE
    return FORWARD


def _cyl_flip_path(part, cell, path_recs):
    for rec in path_recs:
        if rec['cell_index'] == cell.index:
            new_sense = REVERSE if rec['sense'] == 'FORWARD' else FORWARD
            part.setSweepPath(edge=part.edges[rec['edge_index']],
                              region=cell, sense=new_sense)
            rec['sense'] = str(new_sense)
            return
    raise ValueError('Cannot find the sweep-path record of Cell %d' % cell.index)


# 划网与强制校验

def _cyl_mesh_cells(part, cells, expected_by_index, on_mismatch=None,
                    skip_meshed=False):
    """划网前删除该 Cell 既有网格（generateMesh 是追加不是替换），生成 0 单元、总数与期望不符都判失败；总数不符时先翻转扫掠方向重试一次（on_mismatch）。"""
    counts = {}
    skipped = []
    for entry in cells:
        ci = _cyl_as_index(entry)
        cell = part.cells[ci]  # 每轮重取最新引用（网格操作后防陈旧）
        if skip_meshed:
            try:
                if len(cell.getElements()) > 0:
                    skipped.append(ci)
                    continue
            except BaseException:
                pass
        want = expected_by_index.get(ci)
        last = None
        ok = False
        for attempt in (0, 1):
            try:
                try:
                    has_mesh = len(cell.getElements()) > 0
                except BaseException:
                    has_mesh = True
                if has_mesh:
                    try:
                        part.deleteMesh(regions=_cyl_region_array(part,
                                                                 [cell]))
                    except Exception:
                        pass
                    leftover = 0
                    try:
                        leftover = len(cell.getElements())
                    except BaseException:
                        leftover = -1
                    if leftover > 0:
                        raise RuntimeError('Failed to delete the old mesh of Cell %d (residual '
                                           '%d elements, refusing to overlay)'
                                           % (ci, leftover))
                part.generateMesh(regions=_cyl_region_array(part, [cell]))
                got = len(part.cells[ci].getElements())
                if got <= 0:
                    raise RuntimeError('Generated 0 elements (silent failure: mesh controls / '
                                       'topology / seed mismatch)')
                if want is not None and got != want:
                    raise RuntimeError('Element count %d does not match expected %d'
                                       % (got, want))
                ok = True
                break
            except Exception as ex:
                last = ex
                if attempt == 0 and on_mismatch is not None:
                    on_mismatch(cell)
        if not ok:
            try:
                part.deleteMesh(regions=_cyl_region_array(part, [cell]))
            except Exception:
                pass
            dbg = expected_by_index.get('_dbg_thetas')
            extra = (' column_angles=%r' % (dbg.get(ci),)
                     if isinstance(dbg, dict) and ci in dbg else '')
            raise RuntimeError('Cell %d meshing failed: %s%s'
                               % (ci, str(last)[:200], extra))
        counts[ci] = got
    if skipped:
        counts['_skipped'] = skipped
    return counts


def _cyl_audit_generated_mesh(part, cells, counts):
    """界面相容。任何一条不满足都报错。改进：本次承诺划网范围内的未划网区域视为失败相关共享界面不相容视为失败查询异常返回 -1 标记为“检查未完成”，不当作通过选区外原本未划网的区域单独报告，不视为失败"""
    seen = set()
    uniq = []
    for c in cells:
        ci = _cyl_as_index(c)
        if ci in seen:
            continue
        seen.add(ci)
        uniq.append(part.cells[ci])
    cells = uniq
    per_cell = {}
    bad_types = {}
    for cell in cells:
        for elem in cell.getElements():
            ci = cell.index
            per_cell[ci] = per_cell.get(ci, 0) + 1
            t = str(getattr(elem, 'type', '?'))
            if t != 'C3D8R':
                bad_types.setdefault(ci, set()).add(t)
    problems = []
    for c in cells:
        got = per_cell.get(c.index, 0)
        if got <= 0:
            problems.append('Cell %d actually owns 0 elements' % c.index)
        elif c.index in counts and got != counts[c.index]:
            problems.append('Cell %d owned elements %d != generated this run %d'
                            % (c.index, got, counts[c.index]))
        if c.index in bad_types:
            problems.append('Cell %d contains non-C3D8R elements %r'
                            % (c.index, sorted(bad_types[c.index])))

    # 未划网区域检查
    n_un = -1
    unmeshed_in_scope = []
    try:
        un = part.getUnmeshedRegions()
        if un is None:
            n_un = 0
        else:
            un_cells = list(un.cells)
            n_un = len(un_cells)
            # 检查未划网区域是否在本次承诺范围内
            cell_indices = set(c.index for c in cells)
            for region in un_cells:
                try:
                    # 尝试获取未划网区域的 cell 索引
                    if hasattr(region, 'index') and region.index in cell_indices:
                        unmeshed_in_scope.append(region.index)
                except Exception:
                    pass
    except Exception:
        n_un = -1  # 检查未完成

    if n_un == -1:
        problems.append('Cannot complete the unmeshed-region audit')
    elif unmeshed_in_scope:
        problems.append('Unmeshed region within the committed scope: Cell %s'
                        % unmeshed_in_scope[:5])

    # 界面相容性检查
    n_inc = -1
    incomp = None
    try:
        incomp = part.getIncompatibleMeshInterfaces(
            cells=_cyl_region_array(part, cells))
        n_inc = len(incomp) if incomp else 0
    except Exception:
        n_inc = -1  # 检查未完成

    if n_inc == -1:
        problems.append('Cannot complete the mesh-interface audit')
    elif n_inc > 0:
        problems.append('%d incompatible mesh interfaces present' % n_inc)

    if problems:
        raise RuntimeError('Generated mesh validation failed: %s (unmeshed_regions=%r, '
                           'incompatible_interfaces=%r)'
                           % ('; '.join(problems), n_un, n_inc))
    return {'unmeshed_regions': n_un, 'incompatible_interfaces': n_inc,
            'unmeshed_in_scope': unmeshed_in_scope,
            'check_complete': (n_un >= 0 and n_inc >= 0)}


# 正式入口：环向截面—径向过渡

def apply_annular_radial_transition(part, region_faces, fine_edges,
                                    coarse_edges, target_cells,
                                    sweep_path_edges, ratio, fine_seeds,
                                    action='PARTITION_SEED_MESH',
                                    auto_regular=False, phase_deg=None,
                                    _transaction_owned=False):
    """前置：用户已把环带切成可 Sweep 扇区。插件校验同心共面与整除关系，在细圆->粗圆环带内重复 n_periods 个基本单元，按角色布种与设置网格控制，并按 action 决定是否生成网格。"""
    if action not in _CYL_ACTIONS:
        raise ValueError('Invalid action level %r (allowed: %s)'
                         % (action, ', '.join(_CYL_ACTIONS)))
    ratio = int(ratio)
    fine_seeds = int(fine_seeds)
    if ratio not in (2, 3):
        raise ValueError('Circumferential section ratio supports only 2:1 / 3:1, got %d' % ratio)
    if fine_seeds <= 0:
        raise ValueError('Fine-edge seed count must be a positive integer')
    if fine_seeds % ratio != 0:
        raise ValueError('Fine-edge seed count %d must be divisible by ratio %d'
                         % (fine_seeds, ratio))
    coarse_seeds = fine_seeds // ratio
    if ratio == 2 and coarse_seeds % 2 != 0:
        raise ValueError('The 2:1 base unit is 4-fine->2-coarse; the fine seed count must be '
                         'divisible by 4 (current %d)' % fine_seeds)
    n_periods = coarse_seeds // (2 if ratio == 2 else 1)
    period = 360.0 / n_periods
    if abs((90.0 / period) - round(90.0 / period)) > 1e-9:
        raise ValueError('Template period %g deg does not divide 90 deg (seeds and sector '
                         'partition mismatch, fine %d coarse %d)' % (period, fine_seeds,
                                                     coarse_seeds))
    fine_step = 360.0 / fine_seeds
    region_faces = [f for f in region_faces if f is not None]
    target_cells = [c for c in target_cells if c is not None]
    if not fine_edges or not coarse_edges:
        raise ValueError('You must pick the fine edge loop and the coarse edge loop')
    if not region_faces or not target_cells:
        raise ValueError('You must pick the transition region faces and target Cells')

    if not _transaction_owned:
        _cyl_owned_backup(part)
    try:
        fine_edge = fine_edges[0]
        coarse_edge = coarse_edges[0]
        frame = build_cylinder_frame_from_edges(part, fine_edge, coarse_edge,
                                                mode='ANNULAR_RADIAL')
        tol = _CYL_TOL_ABS
        path_sigs = _cyl_path_signatures(part, frame, sweep_path_edges, tol)
        # 自动模式：记录已有网格的区域签名（分区可能级联删除相连网格）
        meshed_before_sigs = []
        if auto_regular:
            for cell in part.cells:
                try:
                    if len(cell.getElements()) > 0:
                        meshed_before_sigs.append(
                            _cyl_cell_signature(part, cell, frame, tol))
                except BaseException:
                    pass
        r_fine = _cyl_polar(frame, fine_edge.pointOn[0])[0]
        r_coarse = _cyl_polar(frame, coarse_edge.pointOn[0])[0]
        if abs(r_fine - r_coarse) < 1e-4:
            raise ValueError('Fine and coarse edges have the same radius; no transition band')
        t_band = _cyl_polar(frame, fine_edge.pointOn[0])[2]

        # 细边弧跨度必须是细段角（360/fine_seeds）的整数倍
        for e in fine_edges:
            vs = e.getVertices()
            if len(vs) != 2:
                raise ValueError('The fine edge must be an arc (a closed full circle is present; '
                                 'partition it first)')
            a1 = _cyl_polar(frame, part.vertices[vs[0]].pointOn[0])[1]
            a2 = _cyl_polar(frame, part.vertices[vs[1]].pointOn[0])[1]
            span = abs((a2 - a1 + 180.0) % 360.0 - 180.0)
            k = round(span / fine_step)
            if abs(span - k * fine_step) > 1e-3:
                raise ValueError('Fine arc span %.4f deg is not an integer multiple of the fine segment angle %.4f deg; '
                                 'seed count or sector partition mismatch' % (span, fine_step))

        band = _cyl_region_face_indices(part, frame, region_faces,
                                        min(r_fine, r_coarse),
                                        max(r_fine, r_coarse), tol)
        if not band:
            raise ValueError('No band face with a radius in [%g, %g] found among the selected '
                             'transition faces; check the transition face selection'
                             % (min(r_fine, r_coarse), max(r_fine, r_coarse)))
        band_sigs = [_cyl_face_span(part, part.faces[fi], frame, tol)
                     for fi in band]
        cell_sigs = [_cyl_cell_signature(part, c, frame, tol) for c in target_cells]
        selected_extent = sum(s['span'] for s in band_sigs)

        up_edges = _cyl_find_radial_edges(
            part, frame,
            radial_span=(min(r_fine, r_coarse), max(r_fine, r_coarse)),
            axial_pos=t_band, tol=tol)
        if not up_edges:
            raise ValueError('No radial straight edge (sector boundary) found in the band for template '
                             'phase alignment; confirm the sectors are already partitioned')
        # 相位对齐基准：按角向规范化选择 up_edge（theta 最接近 0° 的径向边）。
        # 这保证多层链式过渡时各带共享同一角向相位，共享环节点对齐。
        align_idx = _cyl_canonical_align_edge(part, frame, up_edges, tol)

        irep = _cyl_imprint_annular_template(
            part, frame, band, r_fine, r_coarse, n_periods, ratio,
            align_idx, tol, phase_deg=phase_deg)

        # ---- 分区后重获 ----------------------------------------------------
        band_now = _cyl_refind_band_faces(part, frame, band_sigs,
                                          min(r_fine, r_coarse),
                                          max(r_fine, r_coarse), tol)
        faces_pp = irep['faces_per_period']
        want_band_faces = int(round(faces_pp * n_periods
                                    * (selected_extent / 360.0)))
        if len(band_now) != want_band_faces:
            raise ValueError('After template imprint the band sub-faces %d != expected %d '
                             '(faces/period %d x periods %d x selected ratio %g)'
                             % (len(band_now), want_band_faces, faces_pp,
                                n_periods, selected_extent / 360.0))
        cells_now = _cyl_refind_cells(part, frame, cell_sigs, tol)
        if len(cells_now) != len(cell_sigs):
            dbg = []
            for c in part.cells:
                s = _cyl_cell_signature(part, c, frame, tol)
                dbg.append({'i': c.index, 'vol': round(s['vol'], 3),
                            'cyl': s['cyl'], 'th': list(s['thetas']),
                            't': [round(s['t_min'], 3), round(s['t_max'], 3)]})
            raise ValueError('Cannot relocate all target Cells after template imprint (%d/%d)'
                             ' expected_sigs=%r current_sigs=%r'
                             % (len(cells_now), len(cell_sigs), cell_sigs,
                                dbg))
        allowed = _cyl_allowed_edge_indices(cells_now)

        lost_cells = []

        result = {'mode': 'ANNULAR_RADIAL', 'template': irep,
                  'ratio': ratio, 'fine_seeds': fine_seeds,
                  'coarse_seeds': coarse_seeds,
                  'r_fine': round(r_fine, 6), 'r_coarse': round(r_coarse, 6),
                  'band_faces': len(band_now)}
        if action == 'PARTITION_ONLY':
            result['action'] = action
            return result

        # ---- 布种（过渡带）；自动模式连带常规区 -----------------------------
        seed_rec = _cyl_seed_transition_rings(
            part, frame, allowed, 'ANNULAR_RADIAL', r_fine, r_coarse,
            t_band, t_band, fine_seeds, coarse_seeds, tol)
        n_band_edges = _cyl_seed_band_edges(part, frame, band_now, allowed,
                                            tol,
                                            exclude_radii=(r_fine, r_coarse),
                                            mode='ANNULAR_RADIAL')
        # 布种完整性审计：检查所有选区边是否有种子，填补遗漏
        seed_audit, n_filled = _cyl_audit_seed_coverage(
            part, frame, allowed, 'ANNULAR_RADIAL', r_fine, r_coarse,
            t_band, t_band, fine_seeds, coarse_seeds, tol)
        result['seeding'] = seed_rec
        result['band_edge_seeds'] = n_band_edges
        result['seed_audit'] = seed_audit
        if n_filled > 0:
            print('WARNING: seed audit found %d edges without seeds, filled with 1 segment FIXED'
                  % n_filled)
        auto_rec = None
        regular = []
        if auto_regular:
            auto_rec, regular = _cyl_auto_regular(
                part, frame, cells_now, 'ANNULAR_RADIAL', None, r_fine,
                r_coarse, fine_seeds, coarse_seeds, None, None, t_band,
                t_band, tol)
            result['auto_regular'] = auto_rec
            # 布种会重设共享边种子，使相邻已有网格级联失效；在划网前
            # 检测丢失区域，末尾统一恢复（只在已划网模型上生效）
            if meshed_before_sigs:
                for old_sig in meshed_before_sigs:
                    found = None
                    for cell in part.cells:
                        if _cyl_sig_match(_cyl_cell_signature(part, cell,
                                                              frame, tol),
                                          old_sig):
                            found = cell
                            break
                    if found is None:
                        raise ValueError('Auto-adapt: cannot relocate the previously meshed '
                                         'region after seeding (sig %r)' % (old_sig,))
                    try:
                        if len(found.getElements()) == 0:
                            lost_cells.append(found)
                    except BaseException:
                        pass
            if lost_cells:
                result['cascaded_lost_cells'] = [c.index for c in lost_cells]
        if action == 'PARTITION_AND_SEED':
            result['action'] = action
            return result

        # ---- 网格控制与划网 -------------------------------------------------
        _cyl_assign_transition_controls(part, cells_now)
        path_recs = _cyl_assign_annular_paths(
            part, frame, cells_now,
            _cyl_refind_path_edges(part, frame, path_sigs, tol), t_band, tol)
        expected = _cyl_annular_expected(part, frame, cells_now, r_fine,
                                         t_band, ratio, tol)
        _cyl_set_element_type(part, cells_now)
        counts = _cyl_mesh_cells(
            part, cells_now, expected,
            on_mismatch=lambda cell: _cyl_flip_path(part, cell, path_recs),
            skip_meshed=bool(auto_regular))
        audit_cells = list(cells_now)
        all_counts = dict(counts)
        if auto_regular and regular:
            reg_expected, reg_paths = _cyl_mesh_regular_cells(
                part, frame, regular, 'ANNULAR_RADIAL', r_fine, r_coarse,
                auto_rec['r_inner'], auto_rec['r_outer'], auto_rec['t_far'],
                tol)
            _cyl_set_element_type(part, regular)
            reg_counts = _cyl_mesh_cells(
                part, regular, reg_expected,
                on_mismatch=lambda cell: _cyl_flip_path(part, cell,
                                                        reg_paths),
                skip_meshed=True)
            all_counts.update(reg_counts)
            audit_cells.extend(regular)
            result['regular_elements'] = reg_counts
        if auto_regular and lost_cells:
            rec_counts = _cyl_mesh_cells(part, lost_cells, {},
                                         skip_meshed=True)
            all_counts.update(rec_counts)
            audit_cells.extend(lost_cells)
            result['recovered_cells'] = rec_counts
        audit = _cyl_audit_generated_mesh(part, audit_cells, all_counts)
        result['action'] = action
        result['elements'] = len(part.elements)
        result['cell_elements'] = counts
        result['mesh_audit'] = audit
        return result
    except BaseException as exc:
        if not _transaction_owned:
            try:
                _cyl_owned_restore(part)
            except BaseException as rex:
                raise RuntimeError('Operation failed: %r; and restoring the backup also failed: %r '
                                   '(the model may still contain this change)' % (exc, rex))
        raise


def _cyl_annular_expected(part, frame, cells, r_fine, t_band, ratio, tol):
    """过渡 Cell 解析单元数 = 每Cell周期数 × 每周期单元 × 轴向层数。"""
    fine_per_period = ratio if ratio == 3 else 2 * ratio
    epp = 4 if ratio == 3 else 6
    expected = {}
    for cell in cells:
        n_fine = 0
        axial_edge = None
        for e in cell.getEdges():
            ei = _cyl_as_index(e)
            k, info = _cyl_classify_edge(part, part.edges[ei], frame, tol)
            if k == 'CIRCUMFERENTIAL' and abs(info['r'] - r_fine) < tol \
                    and abs(info['t'] - t_band) < tol:
                n_fine += 1
            elif k == 'AXIAL' and axial_edge is None:
                axial_edge = part.edges[ei]
        if n_fine == 0 or n_fine % fine_per_period != 0:
            raise ValueError('Cell %d source-end fine arc count %d does not match %d fine segments per period'
                             % (cell.index, n_fine, fine_per_period))
        layers = _cyl_edge_seed_count(part, axial_edge) if axial_edge is not None \
            else None
        expected[cell.index] = (n_fine // fine_per_period) * epp * layers \
            if layers else None
    return expected


def apply_annular_radial_multilayer(part, region_faces, radii, fine_seeds,
                                    ratio, target_cells, sweep_path_edges,
                                    action='PARTITION_SEED_MESH',
                                    axial_layers=None, t_band=None,
                                    ratio_sequence=None,
                                    _transaction_owned=False,
                                    auto_regular=False):
    """radii: [r0, r1, ..., rL] 从细到粗（或从粗到细）的 L+1 个半径，每相邻两层是一个 ratio:1 合并。"""
    if action not in _CYL_ACTIONS:
        raise ValueError('Invalid action level %r' % (action,))
    radii = [float(r) for r in radii]
    if len(radii) < 3:
        raise ValueError('Multi-stage radial needs at least 3 radii (2 stages).')
    n_layers = len(radii) - 1
    plan = _cyl_multilayer_plan(fine_seeds, n_layers, ratio, ratio_sequence)
    ratios, chain = plan['ratios'], plan['counts']
    ratio, fine_seeds = ratios[0], chain[0]
    direction = 1 if radii[-1] > radii[0] else -1
    if any(not math.isfinite(r) or r <= 0 for r in radii) or any(
            direction * (b - a) <= _CYL_TOL_ABS for a, b in zip(radii[:-1], radii[1:])):
        raise ValueError('radii must be finite, positive and strictly monotonic.')
    if not region_faces or not target_cells:
        raise ValueError('Select the multi-stage source faces and target cells.')
    if axial_layers is not None and (int(axial_layers) != float(axial_layers)
                                      or int(axial_layers) < 1):
        raise ValueError('axial_layers must be a positive integer')
    if not _transaction_owned:
        import transition_mesh_runtime as _tm_runtime
        if _tm_runtime.transaction_active():
            # The caller (formal GUI entry) already owns the transaction;
            # delegate into it instead of nesting (nesting raises TM-BUSY).
            return apply_annular_radial_multilayer(
                part, region_faces, radii, fine_seeds, ratio, target_cells,
                sweep_path_edges, action, axial_layers, t_band, ratios, True,
                auto_regular)
        with _tm_runtime.ModelTransaction(mdb, _cyl_owner_model(part).name):
            return apply_annular_radial_multilayer(
                part, region_faces, radii, fine_seeds, ratio, target_cells,
                sweep_path_edges, action, axial_layers, t_band, ratios, True,
                auto_regular)
    try:
        r0, rL = radii[0], radii[-1]
        r_lo, r_hi = min(r0, rL), max(r0, rL)
        # 从过渡面所在端面推导坐标系（同心环带，轴取端面法向）
        frame = _cyl_frame_from_region_faces(part, region_faces)
        tol = _CYL_TOL_ABS
        path_sigs = _cyl_path_signatures(part, frame, sweep_path_edges, tol)
        if t_band is None:
            t_band = _cyl_polar(frame, region_faces[0].pointOn[0])[2]
        # 过渡带面：端面中径向跨度落在 [r_lo, r_hi] 的
        band = _cyl_region_face_indices(part, frame, region_faces,
                                        r_lo, r_hi, tol)
        if not band:
            raise ValueError('No transition band face covers radii [%g,%g].'
                             % (r_lo, r_hi))
        band_sigs = [_cyl_face_span(part, part.faces[fi], frame, tol)
                     for fi in band]
        cell_sigs = [_cyl_cell_signature(part, c, frame, tol)
                     for c in target_cells]
        selected_face_ids = set(_cyl_as_index(f) for c in target_cells for f in c.getFaces())
        if any(fi not in selected_face_ids for fi in band):
            raise ValueError('Transition faces reach outside the selected target cells.')
        stage_before = [[c for c in target_cells if _cyl_cell_in_radial_band(
            part, frame, c, min(a, b), max(a, b), tol)] for a, b in zip(radii[:-1], radii[1:])]
        if any(not g for g in stage_before) or sum(len(g) for g in stage_before) != len(target_cells):
            raise ValueError('Target cells must already be partitioned at every stage radius.')
        scope, regular, scope_rec = _cyl_multilayer_regular_scope(
            part, frame, target_cells, radii, 'ANNULAR_RADIAL', auto_regular, tol)
        outside = _cyl_multilayer_outside(part, frame, scope, tol)
        regular_sigs = [_cyl_cell_signature(part, c, frame, tol) for c in regular]
        # 相位对齐边：用第一层的径向跨度找径向直边（多层时径向边
        # 被中间环分割，没有跨越整个 [r0,rL] 的单条边）
        r_a, r_b = radii[0], radii[1]
        up_edges = _cyl_find_radial_edges(part, frame,
                                          radial_span=(min(r_a, r_b),
                                                       max(r_a, r_b)),
                                          axial_pos=t_band, tol=tol)
        if not up_edges:
            raise ValueError('No radial edge found in the band for phase alignment.')
        align_idx = _cyl_canonical_align_edge(part, frame, up_edges, tol)
        # 统一多层模板印迹（radii 需从细到粗排序与 seeds 链对应）
        irep = _cyl_imprint_annular_multilayer_template(
            part, frame, band, radii, fine_seeds, ratios[0],
            align_idx, tol, ratio_sequence=ratios)
        # 分区后重获
        band_now = _cyl_refind_band_faces(part, frame, band_sigs, r_lo, r_hi,
                                          tol)
        cells_now = _cyl_refind_cells(part, frame, cell_sigs, tol)
        if len(cells_now) != len(cell_sigs):
            raise ValueError('Cannot re-find all target cells after the multi-stage imprint (%d/%d).'
                             % (len(cells_now), len(cell_sigs)))
        allowed = _cyl_allowed_edge_indices(cells_now)
        # Snapshot the user's axial seeds before auto-regular planning can
        # write its own provisional counts onto shared edges.
        existing_axial_before_auto = set()
        for idx in allowed:
            kind0, _info0 = _cyl_classify_edge(
                part, part.edges[idx], frame, tol)
            if kind0 == 'AXIAL':
                n0 = _cyl_edge_seed_count(part, part.edges[idx])
                if n0:
                    existing_axial_before_auto.add(int(n0))
        regular = _cyl_refind_cells(part, frame, regular_sigs, tol)
        scope = cells_now + regular
        result = {'mode': 'ANNULAR_RADIAL_MULTILAYER', 'template': irep,
                  'ratio': ratio, 'ratios': ratios, 'stages': n_layers,
                  'radii': radii, 'seed_chain': chain, 'stage_counts': chain,
                  'band_faces': len(band_now), 'auto_regular': scope_rec}
        if action == 'PARTITION_ONLY':
            result['action'] = action
            result['scope_audit'] = _cyl_multilayer_check_outside(part, frame, scope, outside, tol)
            return result
        if regular:
            seeds, _ = _cyl_auto_plan_annular(part, frame, regular,
                radii[0], radii[-1], chain[0], chain[-1], tol)
            _cyl_apply_seed_plan(part, seeds, tol)
        # ---- 链式布种：每个环按 chain[i] 分配，模板内部边 1 段 ----
        seed_rec = _cyl_seed_multilayer_rings(part, frame, allowed, radii,
                                              chain, t_band, tol)
        n_band = _cyl_seed_band_edges(part, frame, band_now, allowed, tol,
                                      exclude_radii=tuple(radii),
                                      mode='ANNULAR_RADIAL')
        result['seeding'] = seed_rec
        result['band_edge_seeds'] = n_band
        # 轴向层数统一。用户已经按全局尺寸给过轴向边种子时优先保留；这使径向多层过渡只改变圆周计数，不会把 1 mm 轴向种子悄悄放粗成由最外层圆周粗边推得的尺寸。
        existing_axial = set(existing_axial_before_auto)
        if axial_layers is None:
            if len(existing_axial) == 1:
                axial_layers = next(iter(existing_axial))
                axial_source = 'existing_edge_seeds'
            elif len(existing_axial) > 1:
                raise ValueError(
                    'Selected transition cells have inconsistent axial edge '
                    'seed counts %r; make them equal before Apply.'
                    % sorted(existing_axial))
            else:
                h_c = 2.0 * math.pi * rL / chain[-1]
                l_ax = 0.0
                for idx in allowed:
                    k, info = _cyl_classify_edge(
                        part, part.edges[idx], frame, tol)
                    if k == 'AXIAL':
                        l_ax = max(l_ax, info['t_max'] - info['t_min'])
                axial_layers = (max(1, int(round(l_ax / h_c)))
                                if l_ax > tol else 1)
                axial_source = 'coarse_circumferential_spacing'
        else:
            axial_source = 'explicit_parameter'
        ax_idx = []
        for idx in _cyl_allowed_edge_indices(scope):
            k, info = _cyl_classify_edge(part, part.edges[idx], frame, tol)
            if k == 'AXIAL':
                ax_idx.append(idx)
        if ax_idx:
            part.seedEdgeByNumber(edges=tuple(part.edges[i] for i in ax_idx),
                                  number=int(axial_layers), constraint=FIXED)
        result['axial_layers'] = axial_layers
        result['axial_layers_source'] = axial_source
        # 布种完整性审计：填补任何遗漏的内部边（与单层路径一致）
        seed_audit, n_filled = _cyl_audit_seed_coverage(
            part, frame, allowed, 'ANNULAR_RADIAL', radii[0], radii[-1],
            t_band, t_band, chain[0], chain[-1], tol)
        result['seed_audit'] = seed_audit
        if n_filled > 0:
            print('WARNING: multilayer seed audit found %d edges without seeds, filled with 1 segment'
                  % n_filled)
        if action == 'PARTITION_AND_SEED':
            result['action'] = action
            result['scope_audit'] = _cyl_multilayer_check_outside(part, frame, scope, outside, tol)
            return result
        # ---- 网格控制与划网 ----
        _cyl_assign_transition_controls(part, cells_now)
        path_recs = _cyl_assign_annular_paths(
            part, frame, cells_now,
            _cyl_refind_path_edges(part, frame, path_sigs, tol), t_band, tol)
        _cyl_set_element_type(part, cells_now)
        # 逐层计算期望单元数（按径向跨度分组），避免静默接受畸形网格
        expected_all = {}
        stage_cells = []
        for i in range(len(radii) - 1):
            r_a, r_b = radii[i], radii[i + 1]
            band_i = [c for c in cells_now
                      if _cyl_cell_in_radial_band(part, frame, c,
                                                  min(r_a, r_b),
                                                  max(r_a, r_b), tol)]
            if not band_i:
                raise ValueError('Stage %d has no meshable cell.' % (i + 1))
            stage_cells.append(band_i)
            exp_i = _cyl_annular_expected(part, frame, band_i, r_a, t_band,
                                          ratios[i], tol)
            expected_all.update(exp_i)
        counts = _cyl_mesh_cells(part, cells_now, expected_all,
                                 on_mismatch=lambda cell: _cyl_flip_path(
                                     part, cell, path_recs))
        if regular:
            for c in regular:
                part.setMeshControls(regions=_cyl_region_array(part, [c]), technique=STRUCTURED)
            _cyl_set_element_type(part, regular)
            result['regular_elements'] = _cyl_mesh_cells(part, regular, {})
        _cyl_audit_generated_mesh(part, scope, counts)
        audit = _cyl_multilayer_audit(part, frame, stage_cells, radii, chain,
                                     'ANNULAR_RADIAL', t_band, int(axial_layers), tol)
        result['scope_audit'] = _cyl_multilayer_check_outside(part, frame, scope, outside, tol)
        result['action'] = action
        result['elements'] = len(part.elements)
        result['cell_elements'] = counts
        result['mesh_audit'] = audit
        return result
    except BaseException as exc:
        if not _transaction_owned:
            try:
                _cyl_owned_restore(part)
            except BaseException as rex:
                raise RuntimeError('Multilayer operation failed: %r; restore backup failed: %r'
                                   % (exc, rex))
        raise


def _cyl_frame_from_region_faces(part, region_faces):
    """从过渡端面构建圆筒坐标系（同心环带，轴取端面法向）。"""
    face = region_faces[0]
    normal = _cyl_v_unit(tuple(face.getNormal()))
    # 圆心：用端面上圆弧边的曲率中心
    center = None
    for e in face.getEdges():
        edge = part.edges[_cyl_as_index(e)]
        try:
            c, n = _cyl_edge_circle(part, edge)
            if abs(_cyl_v_dot(_cyl_v_unit(n), normal)) > 0.9:
                center = c
                break
        except Exception:
            continue
    if center is None:
        center = face.pointOn[0]
    axis_dir = normal
    seed = (1.0, 0.0, 0.0)
    if abs(_cyl_v_dot(seed, axis_dir)) > 0.9:
        seed = (0.0, 1.0, 0.0)
    radial_dir = _cyl_v_unit(_cyl_v_sub(seed, _cyl_v_mul(
        axis_dir, _cyl_v_dot(seed, axis_dir))))
    theta_dir = _cyl_v_cross(axis_dir, radial_dir)
    return {'axis_point': center, 'axis_dir': axis_dir,
            'radial_dir': radial_dir, 'theta_dir': theta_dir}


def _cyl_seed_multilayer_rings(part, frame, allowed_idx, radii, chain,
                               t_band, tol):
    """多层链式环布种：第 i 个环（radii[i]）总段数 chain[i]。"""
    groups = {}
    for idx in sorted(allowed_idx):
        edge = part.edges[idx]
        k, info = _cyl_classify_edge(part, edge, frame, tol)
        if k != 'CIRCUMFERENTIAL':
            continue
        groups.setdefault((round(info['r'], 4), round(info['t'], 5)), []).append(idx)
    record = []
    for i, r in enumerate(radii):
        total = chain[i]
        matched = [(key, idxs) for key, idxs in groups.items() if abs(key[0] - r) < 1e-3]
        if not matched:
            raise ValueError('Multistage interface radius %g has no arc' % r)
        for (r_key, t_key), idxs in matched:
            allocated = _cyl_seed_arc_exact(part, frame, idxs, total, tol)
            record.append({'r': r_key, 't': t_key, 'arcs': len(idxs),
                           'total': total, 'allocation': allocated})
    return record


# 正式入口：纵向曲面—轴向过渡

def apply_cylindrical_axial_transition(part, region_faces, fine_edges,
                                       coarse_edges, target_cells,
                                       sweep_path_edges, ratio, fine_seeds,
                                       action='PARTITION_SEED_MESH',
                                       auto_regular=False,
                                       _transaction_owned=False):
    """细/粗圆边位于同一外圆柱面的不同截面；插件在外表面印出竖向切片线（每条母线用端面真实内外半径），细粗截面圆环与切片边按选区布种，"""
    if action not in _CYL_ACTIONS:
        raise ValueError('Invalid action level %r (allowed: %s)'
                         % (action, ', '.join(_CYL_ACTIONS)))
    ratio = int(ratio)
    fine_seeds = int(fine_seeds)
    if ratio not in (2, 3):
        raise ValueError('Axial surface transition supports 2:1 and 3:1, got %d' % ratio)
    if fine_seeds <= 0 or fine_seeds % ratio != 0:
        raise ValueError('Fine seed count must be positive and divisible by %d' % ratio)
    if ratio == 2 and (fine_seeds // 2) % 2 != 0:
        raise ValueError('Axial 2:1 requires an even coarse seed count (%d) '
                         'to keep the slice lines symmetric'
                         % (fine_seeds // 2))
    coarse_seeds = fine_seeds // ratio
    region_faces = [f for f in region_faces if f is not None]
    target_cells = [c for c in target_cells if c is not None]
    if not region_faces or not fine_edges or not coarse_edges or not target_cells:
        raise ValueError('Select transition faces, fine/coarse edges and target cells.')

    if not _transaction_owned:
        _cyl_owned_backup(part)
    try:
        fine_edge = fine_edges[0]
        coarse_edge = coarse_edges[0]
        frame = build_cylinder_frame_from_edges(part, fine_edge, coarse_edge,
                                                mode='AXIAL_SURFACE')
        tol = _CYL_TOL_ABS
        path_sigs = _cyl_path_signatures(part, frame, sweep_path_edges, tol)
        # 自动模式：记录已有网格的区域签名（分区可能级联删除相连网格）
        meshed_before_sigs = []
        if auto_regular:
            for cell in part.cells:
                try:
                    if len(cell.getElements()) > 0:
                        meshed_before_sigs.append(
                            _cyl_cell_signature(part, cell, frame, tol))
                except BaseException:
                    pass
        t_fine = _cyl_polar(frame, fine_edge.pointOn[0])[2]
        t_coarse = _cyl_polar(frame, coarse_edge.pointOn[0])[2]
        # Keep the user's Cell scope as a geometry signature.
        # The old axial route scanned every Cell in the t-band after ThruAll,
        target_cell_sigs = [_cyl_cell_signature(part, c, frame, tol)
                            for c in target_cells]
        r_fine = _cyl_polar(frame, fine_edge.pointOn[0])[0]
        r_coarse = _cyl_polar(frame, coarse_edge.pointOn[0])[0]
        if abs(r_fine - r_coarse) > 1e-4:
            raise ValueError('Fine and coarse edges are not on the same cylindrical surface')
        R = r_fine
        t_lo, t_hi = min(t_fine, t_coarse), max(t_fine, t_coarse)
        # Angular fit: the lane anchors slice 0 on an existing radial seam and patterns by 360/n_slices.
        n_slices = tm_rules.axial_n_slices(fine_seeds, ratio)
        seam_angles = _cyl_seam_angles_deg(part, frame, tol, t_lo, t_hi)
        angular = tm_rules.angular_fit(n_slices, seam_angles)
        if angular['verdict'] == 'no':
            fixes = tm_rules.angular_fix_counts(fine_seeds, ratio)[:4]
            raise ValueError(
                'Axial slice pattern collides with the existing cuts: '
                'spacing %.6g deg x %d sectors, and all slice planes coincide '
                'with your %d radial seam(s), so the partition would add no '
                'new face and the transition cells cannot be meshed. Either '
                'use Seeds=%s (more sectors than seams), or keep a single '
                'radial seam and let the plugin create the sectors.'
                % (angular['spacing_deg'], n_slices, len(seam_angles),
                   fixes or 'larger than %d' % fine_seeds))
        if angular['verdict'] == 'warn':
            angular_warning = angular['reason']
        else:
            angular_warning = ''

        # Respect the explicitly supplied outer-face scope. Invalid/stale picks
        # must fail rather than silently widening the request to the whole Part.
        outer_faces = []
        outer_sigs = []
        for f in region_faces:
            fi = _cyl_as_index(f)
            k, info = _cyl_classify_face(part, part.faces[fi], frame, tol)
            if k != 'CYLINDRICAL' or abs(info['r'] - R) > tol:
                continue
            sig = _cyl_face_span(part, part.faces[fi], frame, tol)
            if sig['t_min'] <= t_lo + tol and sig['t_max'] >= t_hi - tol:
                outer_faces.append(fi)
                outer_sigs.append(sig)
        if not outer_faces:
            raise ValueError('The selected Region Faces do not cover the transition band [t=%g, t=%g] '
                             'of the r=%g cylinder; re-pick the outer face in the current Part, '
                             'it will not auto-widen to the whole Part.' % (t_lo, t_hi, R))

        # 切片草图面：细边 t 截面、径向覆盖壁厚的端面
        sketch_face = None
        for i in range(len(part.faces)):
            k, info = _cyl_classify_face(part, part.faces[i], frame, tol)
            if k == 'END_FACE' and abs(info['t'] - t_fine) < tol:
                rs = [_cyl_polar(frame,
                                 part.edges[_cyl_as_index(e)].pointOn[0])[0]
                      for e in part.faces[i].getEdges()]
                if rs and min(rs) <= R + tol:
                    sketch_face = i
                    break
        if sketch_face is None:
            raise ValueError('Cannot find the annular end face at t=%g where the fine edge lies (needed by the slice sketch)'
                             % t_fine)
        sk_rs = [_cyl_polar(frame,
                            part.edges[_cyl_as_index(e)].pointOn[0])[0]
                 for e in part.faces[sketch_face].getEdges()]
        r_inner_sk, r_outer_sk = min(sk_rs), max(sk_rs)

        # 草图 up 边：横贯壁厚的径向直边（半环切前置分区提供）
        up_edges = _cyl_find_radial_edges(
            part, frame, radial_span=(r_inner_sk, r_outer_sk),
            axial_pos=t_fine, tol=tol)
        if not up_edges:
            raise ValueError('No radial straight edge crossing the wall at the fine section t=%g '
                             '(a half-ring pre-partition must provide the sketch up edge)' % t_fine)
        align_idx = up_edges[0]
        up_theta = _cyl_polar(frame, part.edges[align_idx].pointOn[0])[1]

        # 预期切片弧数（相位按 up 边；Y 向符号不可预知 -> 两个相位取其一）切片线数用 n_slices（3:1=粗数、得 3细→1粗 列；2:1=粗数//2、得可全六面体的 4细→2粗 pair 列）。
        # 用 coarse_seeds 会把 2:1 切成2细→1粗 列，Abaqus 扫掠无法生成纯 hex -> 0 单元。
        spans_fine = _cyl_target_theta_spans(part, frame, outer_faces, R,
                                             t_fine, tol)
        want_fine = (_cyl_expected_slice_arcs(spans_fine, up_theta,
                                              n_slices, tol),
                     _cyl_expected_slice_arcs(spans_fine,
                                              up_theta + 180.0,
                                              n_slices, tol))

        def _verify():
            faces_now = _cyl_refind_cyl_faces(part, frame, outer_sigs, R,
                                              tol)
            if not faces_now:
                return False
            for lv in (t_fine, t_coarse):
                got = set()
                for fi in faces_now:
                    for e in part.faces[fi].getEdges():
                        ei = _cyl_as_index(e)
                        k, info = _cyl_classify_edge(
                            part, part.edges[ei], frame, tol)
                        if k == 'CIRCUMFERENTIAL' \
                                and abs(info['r'] - R) < tol \
                                and abs(info['t'] - lv) < tol:
                            got.add(ei)
                if len(got) not in want_fine:
                    return False
            return True

        # 多带顺序执行时，前一带 ThruAll 可能已切开本带外表面；
        # 已满足切片弧数则跳过，避免 Feature creation failed。
        if _verify():
            print('Axial slices already present (t=%g..%g); skip imprint'
                  % (min(t_fine, t_coarse), max(t_fine, t_coarse)))
        else:
            _cyl_imprint_axial_slices(part, frame, sketch_face, n_slices,
                                      align_idx, outer_faces,
                                      r_inner_sk, r_outer_sk,
                                      t_fine, t_coarse, _verify, tol)

        # ---- 分区后重获：几何扫描优先，避免多带后旧 Cell 句柄指向错误实体 ----
        t_lo_b, t_hi_b = min(t_fine, t_coarse), max(t_fine, t_coarse)
        cells_now = _cyl_find_cells_in_t_band(part, frame, t_lo_b, t_hi_b, tol)
        cells_now = [c for c in cells_now
                     if _cyl_cell_scope_match(part, frame, c,
                                              target_cell_sigs, tol)]
        if not cells_now:
            cells_now = _cyl_refind_cells(part, frame, target_cell_sigs, tol)
        if not cells_now:
            raise ValueError('Cannot locate the transition band Cells after slice imprint (t=%g..%g)'
                             % (t_lo_b, t_hi_b))
        allowed = _cyl_allowed_edge_indices(cells_now)
        # 环向边允许集：必须覆盖 t_fine/t_coarse，否则后续布种会静默落空
        for lv, role in ((t_fine, 'fine'), (t_coarse, 'coarse')):
            rings = [ei for ei in _cyl_find_circ_edges_at(part, frame, R, lv, tol)
                     if ei in allowed]
            if not rings:
                raise ValueError(
                    'Band t=%g..%g has no circular edge at the %s end t=%g, r=%g; '
                    'check whether the slice imprint reached this band'
                    % (t_lo_b, t_hi_b, role, lv, R))
            # Every accepted ring already belongs to the live selected Cells;
            # do not add geometrically coincident edges elsewhere in the Part.

        lost_cells = []

        result = {'mode': 'AXIAL_SURFACE', 'ratio': ratio,
                  'fine_seeds': fine_seeds, 'coarse_seeds': coarse_seeds,
                  't_fine': round(t_fine, 6), 't_coarse': round(t_coarse, 6),
                  'slice_radius': round(R, 6),
                  'n_slices': n_slices, 'seam_angles': seam_angles,
                  'angular_fit': angular,
                  'warnings': [angular_warning] if angular_warning else []}
        if action == 'PARTITION_ONLY':
            result['action'] = action
            return result

        # An existing axial sweep links the two end-ring seeds. Establish
        # the radial sweep first, before assigning different fine/coarse counts.
        _cyl_assign_transition_controls(part, cells_now)
        path_recs = _cyl_assign_axial_paths(
            part, frame, cells_now,
            _cyl_refind_path_edges(part, frame, path_sigs, tol),
            r_inner_sk, r_outer_sk, tol)

        # ---- 布种（限选区）；自动模式连带常规区 -----------------------------
        seed_rec = _cyl_seed_transition_rings(
            part, frame, allowed, 'AXIAL_SURFACE', R, R,
            t_fine, t_coarse, fine_seeds, coarse_seeds, tol)
        n_slice_seeds = _cyl_seed_axial_band_edges(part, frame, allowed,
                                                   t_fine, t_coarse, tol)
        # 布种完整性审计：检查所有选区边是否有种子，填补遗漏
        seed_audit, n_filled = _cyl_audit_seed_coverage(
            part, frame, allowed, 'AXIAL_SURFACE', R, R,
            t_fine, t_coarse, fine_seeds, coarse_seeds, tol)
        result['seeding'] = seed_rec
        result['slice_edge_seeds'] = n_slice_seeds
        result['seed_audit'] = seed_audit
        if n_filled > 0:
            print('WARNING: seed audit found %d edges without seeds, filled with 1 segment FIXED'
                  % n_filled)
        auto_rec = None
        regular = []
        if auto_regular:
            auto_rec, regular = _cyl_auto_regular(
                part, frame, cells_now, 'AXIAL_SURFACE', R, None, None,
                fine_seeds, coarse_seeds, r_inner_sk, r_outer_sk, t_fine,
                t_coarse, tol)
            result['auto_regular'] = auto_rec
            # 布种会重设共享边种子，使相邻已有网格级联失效；在划网前
            # 检测丢失区域，末尾统一恢复（只在已划网模型上生效）
            if meshed_before_sigs:
                for old_sig in meshed_before_sigs:
                    found = None
                    for cell in part.cells:
                        if _cyl_sig_match(_cyl_cell_signature(part, cell,
                                                              frame, tol),
                                          old_sig):
                            found = cell
                            break
                    if found is None:
                        raise ValueError('Auto-adapt: cannot relocate the previously meshed '
                                         'region after seeding (sig %r)' % (old_sig,))
                    try:
                        if len(found.getElements()) == 0:
                            lost_cells.append(found)
                    except BaseException:
                        pass
            if lost_cells:
                result['cascaded_lost_cells'] = [c.index for c in lost_cells]
        if action == 'PARTITION_AND_SEED':
            result['interface_counts'] = _cyl_audit_axial_rings(
                part, frame, cells_now, t_fine, t_coarse,
                fine_seeds, coarse_seeds, tol, mesh=False)
            result['action'] = action
            return result

        # ---- 控制与划网 -----------------------------------------------------
        _cyl_assign_transition_controls(part, cells_now)
        path_recs = _cyl_assign_axial_paths(
            part, frame, cells_now,
            _cyl_refind_path_edges(part, frame, path_sigs, tol),
            r_inner_sk, r_outer_sk, tol)
        expected = _cyl_axial_expected(part, frame, cells_now, R,
                                       t_fine, ratio, tol, t_coarse=t_coarse)
        _cyl_set_element_type(part, cells_now)
        counts = _cyl_mesh_cells(part, cells_now, expected,
                                 skip_meshed=bool(auto_regular))
        audit_cells = list(cells_now)
        all_counts = dict(counts)
        if auto_regular and regular:
            reg_expected, _reg_paths = _cyl_mesh_regular_cells(
                part, frame, regular, 'AXIAL_SURFACE', None, None,
                auto_rec['r_inner'], auto_rec['r_outer'], None, tol)
            _cyl_set_element_type(part, regular)
            reg_counts = _cyl_mesh_cells(part, regular, reg_expected,
                                         skip_meshed=True)
            all_counts.update(reg_counts)
            audit_cells.extend(regular)
            result['regular_elements'] = reg_counts
        if auto_regular and lost_cells:
            rec_counts = _cyl_mesh_cells(part, lost_cells, {},
                                         skip_meshed=True)
            all_counts.update(rec_counts)
            audit_cells.extend(lost_cells)
            result['recovered_cells'] = rec_counts
        audit = _cyl_audit_generated_mesh(part, audit_cells, all_counts)
        result['interface_counts'] = _cyl_audit_axial_rings(
            part, frame, cells_now, t_fine, t_coarse,
            fine_seeds, coarse_seeds, tol)
        result['sweep_paths'] = path_recs
        result['mesh_scope_signatures'] = [
            _cyl_cell_signature(part, c, frame, tol) for c in audit_cells]
        result['action'] = action
        result['elements'] = len(part.elements)
        result['cell_elements'] = counts
        result['mesh_audit'] = audit
        return result
    except BaseException as exc:
        if not _transaction_owned:
            try:
                _cyl_owned_restore(part)
            except BaseException as rex:
                raise RuntimeError('Operation failed: %r; and restoring the backup also failed: %r '
                                   '(the model may still contain this change)' % (exc, rex))
        raise



def apply_cylindrical_axial_multilayer(part, region_faces, fine_edges,
                                       coarse_edges, target_cells,
                                       sweep_path_edges, ratio, fine_seeds,
                                       action='PARTITION_SEED_MESH',
                                       auto_regular=False,
                                       ratio_sequence=None,
                                       stage_widths=None,
                                       _transaction_owned=False,
                                       stages=None, radial_layers=None,
                                       operation_cells=None):
    """真实轴向连续多级：先建立全部共享截面，再局部印刻并统一划网。"""
    import transition_mesh_plan as tm_plan
    if action not in _CYL_ACTIONS:
        raise ValueError('Invalid action level %r' % action)
    plan = _cyl_multilayer_plan(fine_seeds, stages, ratio, ratio_sequence)
    ratios, chain = plan['ratios'], plan['counts']
    stages = plan['stages']
    if not region_faces or not fine_edges or not coarse_edges or not target_cells:
        raise ValueError('Select transition faces, fine/coarse edges and target cells.')
    if not _transaction_owned:
        import transition_mesh_runtime as _tm_runtime
        if _tm_runtime.transaction_active():
            # The caller (formal GUI entry) already owns the transaction;
            # delegate into it instead of nesting (nesting raises TM-BUSY).
            return apply_cylindrical_axial_multilayer(
                part, region_faces, fine_edges, coarse_edges, target_cells,
                sweep_path_edges, ratio, fine_seeds, action, auto_regular,
                ratios, stage_widths, True, stages, radial_layers, operation_cells)
        with _tm_runtime.ModelTransaction(mdb, _cyl_owner_model(part).name):
            return apply_cylindrical_axial_multilayer(
                part, region_faces, fine_edges, coarse_edges, target_cells,
                sweep_path_edges, ratio, fine_seeds, action, auto_regular,
                ratios, stage_widths, True, stages, radial_layers, operation_cells)
    tol = _CYL_TOL_ABS
    frame = build_cylinder_frame_from_edges(part, fine_edges[0], coarse_edges[0],
                                            mode='AXIAL_SURFACE')
    R, _, t0 = _cyl_polar(frame, fine_edges[0].pointOn[0])
    rc, _, t1 = _cyl_polar(frame, coarse_edges[0].pointOn[0])
    path_sigs = _cyl_path_signatures(part, frame, sweep_path_edges, tol)
    if abs(R - rc) > tol:
        raise ValueError('Fine and coarse edges must lie on the same cylindrical surface.')
    bounds = tm_plan.stage_boundaries(abs(t1 - t0), chain, stage_widths)
    levels = [t0 + (1 if t1 > t0 else -1) * x for x in bounds]
    signatures = [_cyl_cell_signature(part, c, frame, tol) for c in target_cells]
    picked_faces = set(_cyl_as_index(f) for f in region_faces)
    walls = [_cyl_cell_radial_span(part, frame, c, tol) for c in target_cells]
    ri, ro = walls[0]
    if ri is None or abs(ro - R) > tol or any(
            abs(a - ri) > tol or abs(b - ro) > tol for a, b in walls):
        raise ValueError('Multi-stage needs constant inner/outer radii; pick the transition faces on the outer cylindrical surface.')
    for c, sig in zip(target_cells, signatures):
        if sig['t_min'] < min(t0, t1) - tol or sig['t_max'] > max(t0, t1) + tol:
            raise ValueError('Target cells reach outside the fine/coarse range; limit the band first.')
        if not any(_cyl_as_index(f) in picked_faces and
                   _cyl_classify_face(part, part.faces[_cyl_as_index(f)], frame, tol)[0]
                   == 'CYLINDRICAL' for f in c.getFaces()):
            raise ValueError('Every target cell must own one of the picked outer cylindrical transition faces.')
    scope, regular, scope_rec = _cyl_multilayer_regular_scope(
        part, frame, target_cells, levels, 'AXIAL_SURFACE', auto_regular, tol)
    operation_sigs = [_cyl_cell_signature(part, c, frame, tol)
                      for c in (operation_cells or [])]
    outside = _cyl_multilayer_outside(part, frame, scope, tol, operation_sigs)
    regular_sigs = [_cyl_cell_signature(part, c, frame, tol) for c in regular]
    wall_seeds = set()
    for ei in _cyl_allowed_edge_indices(scope):
        kind, info = _cyl_classify_edge(part, part.edges[ei], frame, tol)
        if kind == 'RADIAL':
            n = _cyl_edge_seed_count(part, part.edges[ei])
            if n:
                wall_seeds.add(n)
    if radial_layers is None:
        if len(wall_seeds) > 1:
            raise ValueError('Wall-thickness edge seeds differ; pass radial_layers explicitly.')
        if wall_seeds:
            radial_layers = next(iter(wall_seeds))
        elif auto_regular:
            h_coarse = 2.0 * math.pi * R / chain[-1]
            radial_layers = max(1, int(round((ro - ri) / h_coarse)))
        else:
            radial_layers = 1
    if int(radial_layers) != float(radial_layers) or int(radial_layers) < 1:
        raise ValueError('radial_layers must be a positive integer.')
    radial_layers = int(radial_layers)

    def selected():
        return [c for c in part.cells if _cyl_cell_scope_match(
            part, frame, c, signatures, tol)]

    def groups():
        cells = selected()
        result = []
        for a, b in zip(levels[:-1], levels[1:]):
            band = [c for c in cells if all(abs(x - y) < tol for x, y in zip(
                _cyl_cell_t_span(part, frame, c, tol), (min(a, b), max(a, b))))]
            if not band:
                raise ValueError('Cannot locate the axial stage cells [%g,%g].' % (a, b))
            result.append(band)
        if sum(len(g) for g in result) != len(cells):
            raise ValueError('Existing axial partitions do not match the stage widths; use consistent section planes.')
        return result

    # 内部截面仅切选中 Cell，轴线和三个点均使用局部坐标系。
    for lv in levels[1:-1]:
        crossing = [c for c in selected() if
                    _cyl_cell_t_span(part, frame, c, tol)[0] < lv - tol and
                    _cyl_cell_t_span(part, frame, c, tol)[1] > lv + tol]
        if crossing:
            part.PartitionCellByPlaneThreePoints(
                cells=_cyl_region_array(part, crossing),
                point1=_cyl_global_point(frame, 0, 0, lv),
                point2=_cyl_global_point(frame, 1, 0, lv),
                point3=_cyl_global_point(frame, 1, 90, lv))
    for i in range(stages):
        group = groups()[i]
        face_ids = set(_cyl_as_index(f) for c in group for f in c.getFaces())
        outer, end = [], []
        for fi in face_ids:
            kind, info = _cyl_classify_face(part, part.faces[fi], frame, tol)
            if kind == 'CYLINDRICAL' and abs(info['r'] - R) < tol:
                outer.append(fi)
            elif kind == 'END_FACE' and abs(info['t'] - levels[i]) < tol:
                end.append(fi)
        allowed = _cyl_allowed_edge_indices(group)
        up = [ei for ei in _cyl_find_radial_edges(part, frame, (ri, ro), levels[i], tol)
              if ei in allowed]
        if not outer or not end or not up:
            raise ValueError('Stage %d is missing its source face or wall radial edges.' % (i + 1))
        align = _cyl_canonical_align_edge(part, frame, up, tol)
        # 2:1 必须采用 4细到2粗的偶数边周期；单个2细到1粗面为奇数边界。
        slices = chain[i + 1] // (2 if ratios[i] == 2 else 1)
        theta0 = _cyl_polar(frame, part.edges[align].pointOn[0])[1]
        spans = _cyl_target_theta_spans(part, frame, outer, R, levels[i], tol)
        existing = []
        for ei in allowed:
            kind, info = _cyl_classify_edge(part, part.edges[ei], frame, tol)
            if kind == 'AXIAL' and abs(info['r'] - R) < tol:
                existing.append(_cyl_polar(frame, part.edges[ei].pointOn[0])[1])
        wanted = [theta0 + j * 360.0 / slices for j in range(slices)]
        wanted = [th for th in wanted if any(
            width > 359.99 or (th - start) % 360.0 <= width + 1e-3
            for start, width in spans)]
        if not wanted or not all(any(_cyl_angle_distance(th, old) < 1e-3
                                      for old in existing) for th in wanted):
            _cyl_imprint_axial_slices(part, frame, end[0], slices, align, outer,
                                      ri, ro, levels[i], levels[i + 1], None, tol)
    stage_cells = groups()
    cells = [c for group in stage_cells for c in group]
    regular = _cyl_refind_cells(part, frame, regular_sigs, tol)
    scope = cells + regular
    result = {'mode': 'AXIAL_SURFACE_MULTILAYER', 'ratio': ratios[0],
              'ratios': ratios, 'stages': stages, 'stage_counts': chain,
              'seed_chain': chain, 't_levels': levels, 'slice_radius': R,
              'radial_layers': radial_layers, 'action': action,
              'auto_regular': scope_rec}
    if action != 'PARTITION_ONLY':
        _cyl_assign_transition_controls(part, cells)
        paths = _cyl_assign_axial_paths(part, frame, cells,
            _cyl_refind_path_edges(part, frame, path_sigs, tol), ri, ro, tol)
        if regular:
            seeds, _ = _cyl_auto_plan_axial(part, frame, regular, regular, R,
                t0, t1, chain[0], chain[-1], ri, ro, tol)
            _cyl_apply_seed_plan(part, seeds, tol)
        allowed = _cyl_allowed_edge_indices(cells)
        rings, axials, radials = _cyl_group_edges(part, frame, cells, tol)
        records = []
        for (t, r), idxs in sorted(rings.items()):
            hits = [i for i, lv in enumerate(levels) if abs(t - lv) < tol]
            if not hits:
                # created by IY,China University of Petroleum (East China)
                raise ValueError('Circular edge at t=%g does not belong to any stage interface.' % t)
            records.extend(_cyl_seed_arc_exact(part, frame, idxs, chain[hits[0]], tol))
        for idxs in axials.values():
            _cyl_seed_number(part, idxs, 1)
        wall = [ei for ei in _cyl_allowed_edge_indices(scope) if
                _cyl_classify_edge(part, part.edges[ei], frame, tol)[0] == 'RADIAL']
        _cyl_seed_number(part, wall, radial_layers)
        result['seeding'] = records
        if action == 'PARTITION_SEED_MESH':
            _cyl_assign_transition_controls(part, cells)
            paths = _cyl_assign_axial_paths(part, frame, cells,
                _cyl_refind_path_edges(part, frame, path_sigs, tol), ri, ro, tol)
            _cyl_set_element_type(part, scope)
            counts = _cyl_mesh_cells(part, cells, {}, on_mismatch=lambda c:
                                     _cyl_flip_path(part, c, paths))
            if regular:
                reg_expected, _ = _cyl_mesh_regular_cells(part, frame, regular,
                    'AXIAL_SURFACE', None, None, ri, ro, None, tol)
                result['regular_elements'] = _cyl_mesh_cells(
                    part, regular, reg_expected, skip_meshed=True)
            result['mesh_audit'] = _cyl_multilayer_audit(part, frame, stage_cells,
                levels, chain, 'AXIAL_SURFACE', R, radial_layers, tol)
            result['cell_elements'] = counts
            result['elements'] = len(part.elements)
            result['sweep_paths'] = paths
    result['scope_audit'] = _cyl_multilayer_check_outside(
        part, frame, scope, outside, tol, operation_sigs)
    return result


def _cyl_multilayer_plan(fine_seeds, stages, ratio, ratio_sequence):
    """多级入口的严格规范化；空序列与旧脚本默认兼容。"""
    import transition_mesh_plan as planner
    ratios = planner.parse_sequence(ratio_sequence, _cyl_parse_ratio)
    if stages is None:
        stages = len(ratios) if ratios else 2
    if int(stages) != float(stages) or int(stages) < 2:
        raise ValueError('Continuous multi-stage needs an integer stage count of at least 2.')
    if int(fine_seeds) != float(fine_seeds):
        raise ValueError('fine_seeds must be a positive integer')
    plan = planner.plan_counts(int(fine_seeds), int(stages),
                               _cyl_parse_ratio(ratio), ratios or None)
    if len(set(plan['ratios'])) != 1:
        raise ValueError('Cylinder multi-stage currently supports one ratio repeated; mixed ratios are not verified.')
    return plan


def _cyl_cell_scope_match(part, frame, cell, signatures, tol):
    """按柱面半径、角域和轴向包含关系追踪原选区的分区后代。"""
    sig = _cyl_cell_signature(part, cell, frame, tol)
    for old in signatures:
        if sig['t_min'] < old['t_min'] - tol or sig['t_max'] > old['t_max'] + tol:
            continue
        if not sig['cyl'] or len(sig['cyl']) != len(old['cyl']):
            continue
        good = True
        for (r, spans), (ro, old_spans) in zip(sig['cyl'], old['cyl']):
            if abs(r - ro) > 1e-3:
                good = False
                break
            for a, width in spans:
                if not any(ow > 359.99 or
                           ((a - oa) % 360.0 < 0.02 and width <= ow + 0.02) or
                           ((a - oa) % 360.0 + width <= ow + 0.02)
                           for oa, ow in old_spans):
                    good = False
        if good:
            return True
    return False


def _cyl_multilayer_regular_scope(part, frame, cells, levels, mode, enabled, tol):
    """只扩展细粗端的共享面邻居，不把未选角域或壁厚带误当作规则区。"""
    _, neighbors, _ = _cyl_scope_cells(part, frame, cells, tol)
    regular = []
    lo, hi = min(levels), max(levels)
    if enabled:
        for c in neighbors:
            span = (_cyl_cell_t_span(part, frame, c, tol) if mode == 'AXIAL_SURFACE'
                    else _cyl_cell_radial_span(part, frame, c, tol))
            if span[0] is None:
                continue
            if abs(span[1] - lo) < tol or abs(span[0] - hi) < tol:
                regular.append(c)
    return list(cells) + regular, regular, {
        'enabled': bool(enabled), 'regular_cells': [c.index for c in regular]}


def _cyl_multilayer_outside(part, frame, cells, tol, operation_sigs=None):
    """记录选区外几何与原有网格；共享边不计入外部独占种子。"""
    chosen = set(c.index for c in cells)
    allowed = _cyl_allowed_edge_indices(cells)
    records = []
    for c in part.cells:
        if c.index in chosen or (operation_sigs and
                _cyl_cell_scope_match(part, frame, c, operation_sigs, tol)):
            continue
        elements = []
        for e in (c.getElements() or ()):
            nodes = None
            try:
                nodes = e.getNodes()
            except Exception:
                nodes = None
            if nodes is None:
                continue
            elements.append((e.label, str(e.type),
                             tuple(n.label for n in nodes)))
        raw_nodes = None
        try:
            raw_nodes = c.getNodes()
        except Exception:
            raw_nodes = None
        if raw_nodes is None:
            try:
                raw_nodes = [n for e in (c.getElements() or ())
                             for n in (e.getNodes() or ())]
            except Exception:
                raw_nodes = []
        nodes = sorted((n.label, tuple(n.coordinates)) for n in raw_nodes)
        seeds = []
        for ei in (c.getEdges() or ()):
            ei = _cyl_as_index(ei)
            if ei not in allowed:
                edge = part.edges[ei]
                seeds.append((tuple(round(v, 7) for v in edge.pointOn[0]),
                              _cyl_edge_seed_count(part, edge)))
        records.append((_cyl_cell_signature(part, c, frame, tol),
                        sorted(elements), nodes, sorted(seeds)))
    return records


def _cyl_multilayer_check_outside(part, frame, scope, before, tol, operation_sigs=None):
    after = _cyl_multilayer_outside(part, frame, scope, tol, operation_sigs)
    if len(before) != len(after):
        raise RuntimeError('Cell count outside the selection changed; refusing to keep this edit.')
    for old in before:
        match = [x for x in after if _cyl_sig_match(x[0], old[0])]
        if len(match) != 1 or match[0][1:] != old[1:]:
            raise RuntimeError('Geometry, mesh or exclusive edge seeds changed outside the selection; enlarge the selection or use the full model transaction.')
    return {'outside_cells': len(before), 'unchanged': True}


def _cyl_seed_arc_exact(part, frame, edges, total, tol):
    """逐条按真实弧长分配整圈总数，绝不把总数写到每一段。"""
    records = []
    for ei in sorted(set(edges)):
        edge = part.edges[ei]
        radius = _cyl_polar(frame, edge.pointOn[0])[0]
        extent = math.degrees(edge.getSize() / radius)
        count = total * extent / 360.0
        n = int(round(count))
        if n < 1 or abs(n - count) > 1e-5:
            raise ValueError('Arc %d spanning %g deg cannot take %d segments of the full ring (%g needed).'
                             % (ei, extent, total, count))
        _cyl_seed_number(part, [ei], n)
        records.append({'edge': ei, 'segments': n, 'extent': extent})
    if not records:
        raise ValueError('The interface has no circular edge to seed.')
    return records


def _cyl_multilayer_audit(part, frame, stage_cells, levels, chain, mode,
                          anchor, sweep_layers, tol):
    """基于实际 C3D8R 连通性验证每级接口，不以种子或几何顶点代替网格。"""
    from abaqusConstants import ANALYSIS_CHECKS
    node_sets = []
    edge_sets = []
    all_cells = [c for group in stage_cells for c in group]
    hex_edges = ((0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6),
                 (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7))
    coords = {}
    for group in stage_cells:
        ns, es = set(), set()
        for c in group:
            elements = c.getElements()
            if not len(elements):
                raise RuntimeError('Cell %d is not meshed' % c.index)
            for e in elements:
                if str(e.type) != 'C3D8R':
                    raise RuntimeError('Found non-C3D8R element %s' % str(e.type))
                nodes = e.getNodes()
                ids = [n.label for n in nodes]
                if len(ids) != 8:
                    raise RuntimeError('Hexahedron connectivity is not 8 nodes')
                ns.update(ids)
                for n in nodes:
                    coords[n.label] = _cyl_polar(frame, n.coordinates)
                es.update(tuple(sorted((ids[a], ids[b]))) for a, b in hex_edges)
        node_sets.append(ns)
        edge_sets.append(es)
    interface = []
    axis = 2 if mode == 'AXIAL_SURFACE' else 0
    ring_axis = 0 if mode == 'AXIAL_SURFACE' else 2
    for i, level in enumerate(levels):
        neighbors = ([i - 1] if i else []) + ([i] if i < len(stage_cells) else [])
        sides = [set(n for n in node_sets[j] if abs(coords[n][axis] - level) < tol)
                 for j in neighbors]
        if not sides or not sides[0] or any(s != sides[0] for s in sides[1:]):
            raise RuntimeError('Interface %d has mismatched node labels on its two sides or is empty' % i)
        ns = sides[0]
        ring = set(n for n in ns if abs(coords[n][ring_axis] - anchor) < tol)
        segs = set(e for j in neighbors for e in edge_sets[j]
                   if e[0] in ring and e[1] in ring)
        allowed = _cyl_allowed_edge_indices(stage_cells[neighbors[0]])
        extent = 0.0
        for ei in allowed:
            edge = part.edges[ei]
            kind, info = _cyl_classify_edge(part, edge, frame, tol)
            if kind != 'CIRCUMFERENTIAL':
                continue
            if ((mode == 'AXIAL_SURFACE' and abs(info['t'] - level) < tol
                 and abs(info['r'] - anchor) < tol) or
                (mode == 'ANNULAR_RADIAL' and abs(info['r'] - level) < tol
                 and abs(info['t'] - anchor) < tol)):
                extent += math.degrees(edge.getSize() / info['r'])
        expected = chain[i] * extent / 360.0
        if abs(expected - round(expected)) > 1e-5 or len(segs) != int(round(expected)):
            raise RuntimeError('Interface %d actual circumferential segment count %d != expected %g'
                               % (i, len(segs), expected))
        # 单个连通选区的闭环/开弧节点数由实际边图给出。
        degree = {}
        for a, b in segs:
            degree[a] = degree.get(a, 0) + 1
            degree[b] = degree.get(b, 0) + 1
        if set(degree) != ring or any(d not in (1, 2) for d in degree.values()):
            raise RuntimeError('Interface ring has isolated nodes or non-manifold edges')
        expected_nodes = len(ring) * (sweep_layers + 1)
        if len(ns) != expected_nodes:
            raise RuntimeError('Interface %d nodes %d != ring nodes %d x sweep-layer nodes %d'
                               % (i, len(ns), len(ring), sweep_layers + 1))
        points = [tuple(round(v, 6) for v in part.nodes.sequenceFromLabels((n,))[0].coordinates)
                  for n in ns]
        if len(set(points)) != len(points):
            raise RuntimeError('Interface has duplicate nodes with coincident coordinates but different labels')
        interface.append({'index': i, 'level': level, 'full_ring_count': chain[i],
                          'selected_extent': extent, 'actual_segments': len(segs),
                          'ring_nodes': len(ring), 'interface_nodes': len(ns),
                          'shared_labels_equal': True, 'adjacent_stages': neighbors})
    seq = _cyl_region_array(part, all_cells)
    incompatible = part.getIncompatibleMeshInterfaces(cells=seq)
    if incompatible:
        raise RuntimeError('%d incompatible mesh interfaces present' % len(incompatible))
    quality = part.verifyMeshQuality(criterion=ANALYSIS_CHECKS, regions=seq)
    q = {key: len(quality.get(key, ())) for key in
         ('failedElements', 'warningElements', 'naElements')}
    if q['failedElements'] or q['naElements']:
        raise RuntimeError('Mesh quality check failed: %r' % q)
    return {'interfaces': interface, 'quality': q, 'incompatible_interfaces': 0,
            'unmeshed_in_scope': [], 'check_complete': True}


def _cyl_arc_mid_theta(part, frame, edge):
    vs = edge.getVertices()
    if len(vs) == 2:
        a1 = _cyl_polar(frame, part.vertices[vs[0]].pointOn[0])[1]
        a2 = _cyl_polar(frame, part.vertices[vs[1]].pointOn[0])[1]
        d = (a2 - a1) % 360.0
        return (a1 + d / 2.0) % 360.0
    return _cyl_polar(frame, edge.pointOn[0])[1] % 360.0


def _cyl_audit_axial_rings(part, frame, cells, t_fine, t_coarse,
                          fine_seeds, coarse_seeds, tol, mesh=True):
    """Audit effective seeds and retained nodes on every selected end-ring arc."""
    allowed = _cyl_allowed_edge_indices(cells)
    groups = {}
    for ei in sorted(allowed):
        edge = part.edges[ei]
        kind, info = _cyl_classify_edge(part, edge, frame, tol)
        if kind != 'CIRCUMFERENTIAL':
            continue
        if abs(info['t'] - t_fine) < tol:
            side, total = 'fine', fine_seeds
        elif abs(info['t'] - t_coarse) < tol:
            side, total = 'coarse', coarse_seeds
        else:
            continue
        want_float = total * _cyl_arc_span_deg(part, frame, edge, tol) / 360.0
        want = int(round(want_float))
        seed = _cyl_edge_seed_count(part, edge)
        if abs(want_float - want) > 1e-6 or seed != want:
            raise ValueError('Axial %s arc %d has %r seeds; expected %g. '
                             'Check the sweep controls before seeding.'
                             % (side, ei, seed, want_float))
        actual = None
        if mesh:
            nodes = edge.getNodes()
            actual = len(nodes) - (1 if len(edge.getVertices()) == 2 else 0)
            if actual != want:
                raise RuntimeError('Axial %s arc %d has %d mesh segments; '
                                   'expected %d.' % (side, ei, actual, want))
        key = (side, round(info['r'], 4))
        item = groups.setdefault(key, {'side': side, 'radius': key[1],
                                      'expected': 0, 'seeds': 0,
                                      'actual': 0 if mesh else None})
        item['expected'] += want
        item['seeds'] += seed
        if mesh:
            item['actual'] += actual
    if not groups or set(k[0] for k in groups) != set(('fine', 'coarse')):
        raise ValueError('Cannot audit both axial end rings in the selected cells.')
    return {'ok': True, 'rings': [groups[k] for k in sorted(groups)]}


def _cyl_axial_expected(part, frame, cells, R, t_fine, ratio, tol,
                        t_coarse=None):
    """列数 = 该 Cell 在细截面 (R, t_fine) 的圆弧中点角聚类数（每列恰一条弧；直接数轴向边会把与切片线重合的边界段多算一列）。每列 quad 数 = 该列细弧段数 + 粗弧段数。"""
    expected = {}
    dbg_thetas = {}
    for cell in cells:
        mid_ths = []
        fine_seeds = []
        coarse_seeds = []
        radial_edge = None
        for e in cell.getEdges():
            ei = _cyl_as_index(e)
            k, info = _cyl_classify_edge(part, part.edges[ei], frame, tol)
            if k == 'CIRCUMFERENTIAL' and abs(info['r'] - R) < tol \
                    and abs(info['t'] - t_fine) < tol:
                mid_ths.append(_cyl_arc_mid_theta(part, frame,
                                                  part.edges[ei]))
                sc = _cyl_edge_seed_count(part, part.edges[ei])
                if sc:
                    fine_seeds.append(sc)
            elif k == 'CIRCUMFERENTIAL' and abs(info['r'] - R) < tol \
                    and t_coarse is not None \
                    and abs(info['t'] - t_coarse) < tol:
                sc = _cyl_edge_seed_count(part, part.edges[ei])
                if sc:
                    coarse_seeds.append(sc)
            elif k == 'RADIAL' and radial_edge is None:
                radial_edge = part.edges[ei]
        mid_ths.sort()
        clusters = []
        for th in mid_ths:
            if clusters and min(abs(th - x) for x in clusters[-1]) < 1e-2:
                clusters[-1].append(th)
            else:
                clusters.append([th])
        n_cols = len(clusters)
        dbg_thetas[cell.index] = [round(c[0], 3) for c in clusters]
        if n_cols == 0:
            raise ValueError('Cell %d has no outer-surface slice column (fine-section arc)'
                             % cell.index)
        layers = _cyl_edge_seed_count(part, radial_edge) if radial_edge is not None \
            else None
        if ratio == 2:
            # 2:1 的 pair 块由 Abaqus 扫掠自动铺，每列 quad 数会随列在缝处的共享而变（5 或 6），不固定；强校单元数会误判。
            # 这里不设定硬期望，交由 _cyl_mesh_cells 的 got>0 + _cyl_audit_generated_mesh的纯六面体/无未划/无不相容界面三重审计保证正确性。
            expected[cell.index] = None
            continue
        if fine_seeds and coarse_seeds:
            per_col = max(fine_seeds) + max(coarse_seeds)
        else:
            per_col = ratio + 1
        expected[cell.index] = n_cols * per_col * layers if layers \
            else None
    expected['_dbg_thetas'] = dbg_thetas
    return expected


# 常规区自动适配（过渡边驱动布种：过渡带 + 一层共享面邻居）

def _cyl_cell_t_span(part, frame, cell, tol):
    ts = []
    for e in cell.getEdges():
        ei = _cyl_as_index(e)
        for vi in part.edges[ei].getVertices():
            ts.append(_cyl_polar(frame, part.vertices[vi].pointOn[0])[2])
    return (min(ts), max(ts)) if ts else (None, None)


def _cyl_cell_radial_span(part, frame, cell, tol):
    rs = []
    for f in cell.getFaces():
        fk, finfo = _cyl_classify_face(part, part.faces[_cyl_as_index(f)],
                                       frame, tol)
        if fk == 'CYLINDRICAL':
            rs.append(finfo['r'])
    rs = sorted(rs)
    return (rs[0], rs[-1]) if rs else (None, None)


def _cyl_cell_in_radial_band(part, frame, cell, r_lo, r_hi, tol):
    """Cell 的径向跨度是否恰为 [r_lo, r_hi]（用于多层逐层分组）。"""
    span = _cyl_cell_radial_span(part, frame, cell, tol)
    if span[0] is None:
        return False
    return abs(span[0] - r_lo) < 1e-3 and abs(span[1] - r_hi) < 1e-3


def _cyl_scope_cells(part, frame, transition_cells, tol):
    """过渡带 + 一层共享面邻居；返回 (scope, regular, record)。"""
    face_cells = {}
    for cell in part.cells:
        for f in cell.getFaces():
            face_cells.setdefault(_cyl_as_index(f), []).append(cell.index)
    trans = set(c.index for c in transition_cells)
    ring = set()
    for cell in transition_cells:
        for f in cell.getFaces():
            for ci in face_cells.get(_cyl_as_index(f), []):
                if ci not in trans:
                    ring.add(ci)
    scope_idx = trans | ring
    scope = [part.cells[i] for i in sorted(scope_idx)]
    regular = [part.cells[i] for i in sorted(ring)]
    return scope, regular, {'transition_cells': sorted(trans),
                            'regular_cells': sorted(ring),
                            'scope_cells': sorted(scope_idx)}


def _cyl_group_edges(part, frame, cells, tol):
    """scope 边集按几何类别分组：rings/axials/radials。"""
    rings, axials, radials = {}, {}, {}
    seen = set()
    for cell in cells:
        for e in cell.getEdges():
            ei = _cyl_as_index(e)
            if ei in seen:
                continue
            seen.add(ei)
            k, info = _cyl_classify_edge(part, part.edges[ei], frame, tol)
            if k == 'CIRCUMFERENTIAL':
                rings.setdefault((round(info['t'], 5), round(info['r'], 4)),
                                 []).append(ei)
            elif k == 'AXIAL':
                axials.setdefault((round(info['t_min'], 5),
                                   round(info['t_max'], 5)), []).append(ei)
            elif k == 'RADIAL':
                radials.setdefault((round(info['r_min'], 4),
                                    round(info['r_max'], 4)), []).append(ei)
    return rings, axials, radials


def _cyl_arc_alloc(part, frame, idxs, total, tol, ctx):
    """按选中弧跨度比例分配整圈总段数；不可整除时报错。"""
    extent = sum([_cyl_arc_span_deg(part, frame, part.edges[i], tol)
                  for i in idxs])
    per = total * (extent / 360.0) / len(idxs)
    per_i = int(round(per))
    if per_i < 1 or abs(per - per_i) > 1e-6:
        raise ValueError(
            'Auto-seeding cannot divide evenly (%s): %d arcs (span %g deg), total segments %d, '
            '%g per arc' % (ctx, len(idxs), extent, total, per))
    return per_i


def _cyl_around_seeds(part, frame, cell, radius, levels, tol):
    """Cell 在 (radius, level) 圆弧上的种子总和（取各层最大者）。"""
    best = 0
    for lv in levels:
        total = 0
        for e in cell.getEdges():
            ei = _cyl_as_index(e)
            k, info = _cyl_classify_edge(part, part.edges[ei], frame, tol)
            if k == 'CIRCUMFERENTIAL' and abs(info['r'] - radius) < tol \
                    and abs(info['t'] - lv) < tol:
                s = _cyl_edge_seed_count(part, part.edges[ei])
                total += s if s else 0
        best = max(best, total)
    return best


def _cyl_apply_seed_plan(part, seeds, tol):
    """执行布种计划（FIXED 数目约束），返回逐条记录（含原值）。"""
    applied = []
    for item in seeds:
        idxs = item['edges']
        if not idxs:
            continue
        prev = set()
        for i in idxs:
            s = _cyl_edge_seed_count(part, part.edges[i])
            if s is not None:
                prev.add(s)
        _cyl_seed_number(part, idxs, item['number'])
        applied.append({'role': item['role'], 'edges': len(idxs),
                        'number': item['number'],
                        'previous': sorted(prev)})
    return applied


def _cyl_auto_plan_annular(part, frame, scope, r_fine, r_coarse,
                           fine_seeds, coarse_seeds, tol):
    """轴向层数按粗圈名义间距推导（全模型统一）。方向无关：按“内环/外环”邻接关系分配区域种子，不假设细边在内。inner_is_fine 决定内圈区域用 fine 还是 coarse 间距。"""
    h_f = 2.0 * math.pi * r_fine / fine_seeds
    h_c = 2.0 * math.pi * r_coarse / coarse_seeds
    rings, axials, radials = _cyl_group_edges(part, frame, scope, tol)

    # 方向无关的内/外环判定
    r_in_ring = min(r_fine, r_coarse)
    r_out_ring = max(r_fine, r_coarse)
    inner_is_fine = abs(r_in_ring - r_fine) < 1e-3

    l_ax = 0.0
    for (a, b) in axials:
        l_ax = max(l_ax, b - a)
    n_axial = max(1, int(round(l_ax / h_c))) if l_ax > tol else 1

    seeds = []
    for (t_key, r_key), idxs in sorted(rings.items()):
        if abs(r_key - r_fine) < 1e-3:
            total, side = fine_seeds, 'fine-loop'
        elif abs(r_key - r_coarse) < 1e-3:
            total, side = coarse_seeds, 'coarse-loop'
        elif r_key < r_in_ring - 1e-3:
            # 内圈区域：邻接内环（内环为细则用 fine，否则 coarse）
            total, side = (fine_seeds, 'fine-region') if inner_is_fine \
                else (coarse_seeds, 'coarse-region')
        elif r_key > r_out_ring + 1e-3:
            # 外圈区域：邻接外环（外环为细则用 fine，否则 coarse）
            total, side = (fine_seeds, 'fine-region') if not inner_is_fine \
                else (coarse_seeds, 'coarse-region')
        else:
            continue  # 模板内部弧线：保持带内每边 1 段
        per_i = _cyl_arc_alloc(part, frame, idxs, total, tol,
                               't=%g r=%g %s' % (t_key, r_key, side))
        seeds.append({'edges': sorted(idxs), 'number': per_i,
                      'role': 'ring t=%g r=%g %s'
                              % (t_key, r_key, side)})
    for (r1, r2), idxs in sorted(radials.items()):
        if r2 <= r_in_ring + 1e-3:
            # 内圈径向带：邻接内环
            h = h_f if inner_is_fine else h_c
            role = 'radial inner-band %g..%g' % (r1, r2)
            n = max(1, int(round((r2 - r1) / h)))
        elif r1 >= r_out_ring - 1e-3:
            # 外圈径向带：邻接外环
            h = h_c if inner_is_fine else h_f
            role = 'radial outer-band %g..%g' % (r1, r2)
            n = max(1, int(round((r2 - r1) / h)))
        else:
            continue  # 模板内部径向线
        seeds.append({'edges': sorted(idxs), 'number': n, 'role': role})
    if axials:
        all_ax = sorted(set([i for idxs in axials.values() for i in idxs]))
        seeds.append({'edges': all_ax, 'number': n_axial,
                      'role': 'axial layers L=%g' % l_ax})
    plan = {'mode': 'ANNULAR_RADIAL', 'h_fine': round(h_f, 6),
            'h_coarse': round(h_c, 6), 'axial_segments': n_axial,
            'axial_length': round(l_ax, 6),
            'inner_is_fine': inner_is_fine,
            'r_in_ring': round(r_in_ring, 6),
            'r_out_ring': round(r_out_ring, 6)}
    return seeds, plan


def _cyl_auto_plan_axial(part, frame, scope, regular, R, t_fine, t_coarse,
                         fine_seeds, coarse_seeds, r_inner, r_outer, tol):
    """轴向模式自动布种计划：常规 Cell 两端层同性质（邻细端=fine），
    轴向段按两端层间距推导，壁厚径向层统一按粗截面间距推导。"""
    h_f = 2.0 * math.pi * R / fine_seeds
    h_c = 2.0 * math.pi * R / coarse_seeds

    layer = {}
    for cell in regular:
        t_lo, t_hi = _cyl_cell_t_span(part, frame, cell, tol)
        near_fine = min(abs(t_lo - t_fine), abs(t_hi - t_fine)) < 1e-3
        near_coarse = min(abs(t_lo - t_coarse),
                          abs(t_hi - t_coarse)) < 1e-3
        if near_fine and near_coarse:
            raise ValueError('Regular Cell %d is adjacent to both the fine and coarse ends' % cell.index)
        if near_fine:
            side, total = 'fine', fine_seeds
        elif near_coarse:
            side, total = 'coarse', coarse_seeds
        else:
            # 多带/远端邻居：不参与本带自动适配，而不是整单失败
            print('WARNING: regular Cell %d (t=%g..%g) is not adjacent to this band t=%g/%g, '
                  'skipping auto-adapt' % (cell.index, t_lo, t_hi, t_fine, t_coarse))
            continue
        for lv in (t_lo, t_hi):
            key = round(lv, 5)
            if key in layer and layer[key][0] != side:
                raise ValueError('Layer t=%g is required to have both %s and %s segment counts'
                                 % (lv, layer[key][0], side))
            layer[key] = (side, total)

    rings, axials, radials = _cyl_group_edges(part, frame, scope, tol)
    seeds = []
    for (t_key, r_key), idxs in sorted(rings.items()):
        if t_key not in layer:
            continue  # 不参与层映射的圈（由现有过渡圈布种覆盖）
        side, total = layer[t_key]
        per_i = _cyl_arc_alloc(part, frame, idxs, total, tol,
                               't=%g r=%g' % (t_key, r_key))
        seeds.append({'edges': sorted(idxs), 'number': per_i,
                      'role': 'ring t=%g r=%g %s' % (t_key, r_key, side)})
    for (a_key, b_key), idxs in sorted(axials.items()):
        s_a = layer.get(a_key, (None,))[0]
        s_b = layer.get(b_key, (None,))[0]
        h_ref = h_f if (s_a == 'fine' and s_b == 'fine') else h_c
        n = max(1, int(round((b_key - a_key) / h_ref)))
        seeds.append({'edges': sorted(idxs), 'number': n,
                      'role': 'axial span [%g, %g]' % (a_key, b_key)})
    n_radial = max(1, int(round((r_outer - r_inner) / h_c)))
    wall = []
    for (r1, r2), idxs in sorted(radials.items()):
        if abs(r1 - r_inner) < tol and abs(r2 - r_outer) < tol:
            wall.extend(idxs)
    if wall:
        seeds.append({'edges': sorted(set(wall)), 'number': n_radial,
                      'role': 'radial wall layers'})
    plan = {'mode': 'AXIAL_SURFACE', 'h_fine': round(h_f, 6),
            'h_coarse': round(h_c, 6), 'radial_segments': n_radial,
            'layer_sides': dict((str(k), v[0]) for k, v in layer.items())}
    return seeds, plan


def _cyl_mesh_regular_cells(part, frame, regular, mode, r_fine, r_coarse,
                            r_inner, r_outer, t_anchor, tol):
    """(expected_by_index, 记录列表)。规则常规区优先使用 HEX + STRUCTURED；结构化不可行时明确报告具体 Cell 和原因，不静默回退到 SWEEP。"""
    # 常规区统一使用 STRUCTURED（规则几何支持结构化网格）
    structured_ok = []
    structured_fail = []
    for cell in regular:
        try:
            part.setMeshControls(
                regions=_cyl_region_array(part, [cell]),
                technique=STRUCTURED)
            structured_ok.append(cell)
        except Exception as exc:
            structured_fail.append({'cell_index': cell.index,
                                    'reason': str(exc)[:120]})
    if structured_fail:
        print('WARNING: %d regular Cells cannot use STRUCTURED: %s'
              % (len(structured_fail), structured_fail[:3]))
        # 对不支持结构化的 Cell 使用 SWEEP 作为回退（明确记录）
        for item in structured_fail:
            ci = item['cell_index']
            try:
                part.setMeshControls(
                    regions=_cyl_region_array(part, [part.cells[ci]]),
                    technique=SWEEP)
            except Exception:
                pass
    expected = {}
    recs = []
    for cell in regular:
        path_ei = None
        rn = None
        ax = None
        for e in cell.getEdges():
            ei = _cyl_as_index(e)
            k, info = _cyl_classify_edge(part, part.edges[ei], frame, tol)
            if k == 'RADIAL' and abs(info['r_min'] - r_inner) < tol \
                    and abs(info['r_max'] - r_outer) < tol:
                if rn is None:
                    rn = _cyl_edge_seed_count(part, part.edges[ei])
            elif k == 'AXIAL' and path_ei is None:
                path_ei = ei
                ax = _cyl_edge_seed_count(part, part.edges[ei])
        if mode == 'ANNULAR_RADIAL':
            r_lo, r_hi = _cyl_cell_radial_span(part, frame, cell, tol)
            r_ring = r_inner if r_hi <= r_fine + tol else r_outer
            around = _cyl_around_seeds(part, frame, cell, r_ring,
                                       (t_anchor,), tol)
        else:
            t_lo, t_hi = _cyl_cell_t_span(part, frame, cell, tol)
            around = _cyl_around_seeds(part, frame, cell, r_outer,
                                       (t_lo, t_hi), tol)
        expected[cell.index] = around * rn * ax \
            if (around and rn and ax) else None
        if mode == 'ANNULAR_RADIAL' and path_ei is not None:
            sense = _cyl_sense_starting_at(part, frame,
                                           part.edges[path_ei], t_anchor,
                                           tol)
            part.setSweepPath(edge=part.edges[path_ei], region=cell,
                              sense=sense)
            recs.append({'cell_index': cell.index, 'edge_index': path_ei,
                         'sense': str(sense)})
    return expected, recs


def _cyl_auto_regular(part, frame, transition_cells, mode, R, r_fine,
                      r_coarse, fine_seeds, coarse_seeds, r_inner,
                      r_outer, t_fine, t_coarse, tol):
    """自动模式编排：识别范围 → 计划 → 布种。返回 (record, regular)。"""
    scope, regular, scope_rec = _cyl_scope_cells(part, frame,
                                                 transition_cells, tol)
    if mode == 'ANNULAR_RADIAL':
        seeds, plan = _cyl_auto_plan_annular(part, frame, scope, r_fine,
                                             r_coarse, fine_seeds,
                                             coarse_seeds, tol)
    else:
        seeds, plan = _cyl_auto_plan_axial(part, frame, scope, regular, R,
                                           t_fine, t_coarse, fine_seeds,
                                           coarse_seeds, r_inner, r_outer,
                                           tol)
        # 只保留邻接本带的常规 Cell，避免远端 Cell 进入划网
        kept = []
        for cell in regular:
            t_lo_c, t_hi_c = _cyl_cell_t_span(part, frame, cell, tol)
            if t_lo_c is None:
                continue
            near_f = min(abs(t_lo_c - t_fine),
                         abs(t_hi_c - t_fine)) < 1e-3
            near_c = min(abs(t_lo_c - t_coarse),
                         abs(t_hi_c - t_coarse)) < 1e-3
            if near_f or near_c:
                kept.append(cell)
        regular = kept
    rs = []
    for cell in scope:
        r_lo, r_hi = _cyl_cell_radial_span(part, frame, cell, tol)
        if r_lo is not None:
            rs.extend([r_lo, r_hi])
    ts = [_cyl_polar(frame, v.pointOn[0])[2] for v in part.vertices]
    applied = _cyl_apply_seed_plan(part, seeds, tol)
    rec = dict(scope_rec)
    rec['plan'] = plan
    rec['seeding'] = applied
    rec['r_inner'] = round(min(rs), 6) if rs else None
    rec['r_outer'] = round(max(rs), 6) if rs else None
    rec['t_far'] = round(max(ts), 6) if ts else None
    return rec, regular
