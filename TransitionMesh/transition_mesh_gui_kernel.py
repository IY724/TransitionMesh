# -*- coding: utf-8 -*-
# ============================================================================
#  Transition Mesh
#  created by IY,China University of Petroleum (East China)
#  禁止商用 / Non-commercial use only
#  禁止商用：本软件仅供学习与研究使用，未经作者书面许可不得用于任何商业用途。
#  Non-commercial use only. Unauthorized commercial use is prohibited.
# ============================================================================

"""Kernel entry for the Abaqus Transition Mesh GUI dialog."""

from __future__ import print_function

import os
import re
import sys
import inspect
import uuid
import json

this_dir = os.path.dirname(os.path.abspath(__file__))
if this_dir not in sys.path:
    sys.path.insert(0, this_dir)

import transition_mesh_plan as tm_plan
import transition_mesh_rules as tm_rules
import transition_mesh_runtime as tm_runtime

from abaqus import mdb

import transition_mesh_plugin as tmp
import transition_mesh_plugin_cylinder_kernel as tmp_cyl

# GUI 运行时保持静默：对话框已通过状态栏/弹窗/报告文件展示 Check 与Pre-mesh 结果，本模块的信息性 print 默认不输出到 CAE 底部信息区；仅 TM_VERBOSE=1（回归脚本会设置）时回显。
# 异常/返回值/模型不受影响。
import os as _tm_os


def _tm_quiet_print(*args, **kwargs):
    return 0


if not _tm_os.environ.get('TM_VERBOSE'):
    print = _tm_quiet_print  # 有意遮蔽模块内建 print


def _parse_point(text):
    values = [item.strip() for item in str(text).replace(';', ',').split(',') if item.strip()]
    if len(values) != 3:
        raise ValueError("Point must be formatted as X,Y,Z; got %r" % text)
    return tuple(float(v) for v in values)


def _is_blank_pick(value):
    if value is None:
        return True
    try:
        text = str(value).strip()
        return text == "" or text.startswith("<")
    except Exception:
        return False


def _single_object(value):
    try:
        if len(value) == 1 and not hasattr(value, "pointOn"):
            return value[0]
    except Exception:
        pass
    return value


def _resolve_picked_object(value):
    if _is_blank_pick(value):
        return None
    if isinstance(value, str):
        namespace = {"mdb": mdb}
        try:
            value = eval(value, namespace)
        except Exception as exc:
            raise ValueError("Cannot resolve picked object %r: %s" % (value, exc))
    return _single_object(value)


def _planar_coarse_edge(pickedCoarseEdges):
    """An empty pick means "no coarse edge", which keeps the historic behaviour where the numeric Depth wins."""
    items = _flatten_objects(pickedCoarseEdges)
    if len(items) > 1:
        raise ValueError(
            'Classic PLANAR Coarse Edge requires exactly one edge; got %d. '
            'Repick one edge and press Done.' % len(items))
    return items[0] if items else None


def _find_face(part, point):
    if not str(point).strip():
        return None
    return _single_object(part.faces.findAt((point,)))


def _find_edge(part, point):
    if not str(point).strip():
        return None
    return _single_object(part.edges.findAt((_parse_point(point),)))


_MODEL_PART_RE = re.compile(
    r"mdb\.models\[\s*['\"]([^'\"]+)['\"]\s*\]"
    r"\.parts\[\s*['\"]([^'\"]+)['\"]\s*\]")


def _infer_model_part_from_objects(*picked_values):
    """当 GUI 传递已解析对象而非命令字符串时，遍历 mdb 查找包含相同几何的 Part。仅在唯一匹配时返回结果。"""
    candidates = set()
    for value in picked_values:
        if value is None or isinstance(value, str):
            continue
        objects = []
        try:
            if hasattr(value, '__iter__') and not hasattr(value, 'pointOn'):
                objects = list(value)
            else:
                objects = [value]
        except Exception:
            continue
        for obj in objects:
            if not hasattr(obj, 'pointOn') or not hasattr(obj, 'index'):
                continue
            try:
                pt = obj.pointOn[0] if isinstance(obj.pointOn[0], (list, tuple)) \
                    else obj.pointOn
                idx = obj.index
            except Exception:
                continue
            for mname in mdb.models.keys():
                if mname.startswith('__TM_BACKUP_'):
                    continue
                model = mdb.models[mname]
                for pname in model.parts.keys():
                    part = model.parts[pname]
                    for coll_name in ('faces', 'edges', 'cells'):
                        coll = getattr(part, coll_name, None)
                        if coll is None or idx >= len(coll):
                            continue
                        try:
                            entity = coll[idx]
                            ep = entity.pointOn[0] \
                                if isinstance(entity.pointOn[0], (list, tuple)) \
                                else entity.pointOn
                            if (abs(pt[0] - ep[0]) < 1e-6
                                    and abs(pt[1] - ep[1]) < 1e-6
                                    and abs(pt[2] - ep[2]) < 1e-6):
                                candidates.add((mname, pname))
                        except Exception:
                            continue
    if len(candidates) == 1:
        return candidates.pop()
    if len(candidates) > 1:
        # 多个候选（测试环境常见）：不报错，返回 None 由调用方回退显式参数
        return None
    return None


def _infer_model_part(*picked_values):
    """GUI 的 AFXObjectKeyword 值可为：1. 形如 ``mdb.models['Model-1'].parts['Part-1'].faces.findAt((..."""
    found = []
    for value in picked_values:
        if not isinstance(value, str):
            continue
        match = _MODEL_PART_RE.search(value)
        if match:
            found.append((match.group(1), match.group(2)))
    if found:
        first = found[0]
        for other in found[1:]:
            if other != first:
                raise ValueError(
                    "Picked entities come from different Model/Part: %r vs %r. "
                    "Pick entities on one Part only." % (first, other))
        return first
    # 字符串推导失败，尝试对象推导
    return _infer_model_part_from_objects(*picked_values)


def _parse_ratio(text):
    """Parse ratio string like '2:1' or '3:1' and return integer ratio value."""
    text = str(text).strip()
    if ':' in text:
        parts = text.split(':')
        if len(parts) == 2:
            return int(parts[0].strip())
    return int(text)


# Canonical transition-type codes. Display strings that start with a canonical code (e.g.
_CANONICAL_TTYPES = (
    'PLANAR_ANNULAR', 'PLANAR_QUAD_FOUR',
    'PLANAR', 'ANNULAR_RADIAL', 'AXIAL_SURFACE',
)
_PLANAR_2D = ('PLANAR_ANNULAR', 'PLANAR_QUAD_FOUR')


def _canonical_transition_type(value):
    v = str(value).strip().upper()
    if v in _CANONICAL_TTYPES:
        return v
    for code in _CANONICAL_TTYPES:
        if v.startswith(code):
            return code
    return None


def _classic_planar_coarse_pick(values):
    """``None`` means a legacy script did not provide the new keyword,"""
    isolated = values.get('pickedPlanarCoarseEdge', None)
    if isolated is None:
        return values.get('pickedCoarseEdges', '')
    return isolated


def _route_pick_values(ttype, values):
    """This prevents a valid selection retained by another Type from changing target inference or failing"""
    if ttype == 'PLANAR':
        return (
            values.get('pickedFace', ''),
            values.get('pickedEdge', ''),
            _classic_planar_coarse_pick(values),
            values.get('pickedDomainFaces', ''),
        )
    if ttype in _PLANAR_2D:
        return (
            values.get('pickedRegionFaces', ''),
            values.get('pickedFineEdges', ''),
            values.get('pickedCoarseEdges', ''),
            values.get('pickedDomainFaces', ''),
        )
    if ttype in ('ANNULAR_RADIAL', 'AXIAL_SURFACE'):
        return (
            values.get('pickedRegionFaces', ''),
            values.get('pickedFineEdges', ''),
            values.get('pickedCoarseEdges', ''),
            values.get('pickedCells', ''),
            values.get('pickedPathEdges', ''),
        )
    return tuple(values.get(name, '') for name in (
        'pickedFace', 'pickedEdge', 'pickedPlanarCoarseEdge',
        'pickedRegionFaces', 'pickedFineEdges', 'pickedCoarseEdges',
        'pickedCells', 'pickedPathEdges', 'pickedDomainFaces'))


_CYL_ACTIONS = ('PARTITION_ONLY', 'PARTITION_AND_SEED',
                'PARTITION_SEED_MESH')
# CHECK_ONLY is read-only: build/validate the plan, never mutate the model.
# PRE_MESH runs the real partition/seed/mesh, audits it and then rolls the trial back,
_ALL_ACTIONS = _CYL_ACTIONS + ('CHECK_ONLY', 'PRE_MESH')


class _PreMeshRollback(Exception):
    """Sentinel: Pre-mesh must undo the trial after auditing it."""


def _same_point(a, b, tol=1e-6):
    return (abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol
            and abs(a[2] - b[2]) <= tol)


def _verify_entities_in_part(part, faces, edges, cells):
    """确认所有选区实体确实来自当前 Part：同索引的本地实体必须与所选
    实体几何一致（防跨 Part 下标碰巧相同改错对象）。发现外来实体立即
    报错。"""
    bad = []
    for f in faces:
        ok = False
        try:
            f2 = part.faces[f.index]
            ok = _same_point(f2.pointOn[0], f.pointOn[0])
        except Exception:
            ok = False
        if not ok:
            bad.append('face#%s@%s' % (f.index,
                                       tuple(round(x, 3) for x in f.pointOn[0])))
    for e in edges:
        ok = False
        try:
            e2 = part.edges[e.index]
            ok = _same_point(e2.pointOn[0], e.pointOn[0]) \
                and abs(e2.getSize(printResults=False)
                        - e.getSize(printResults=False)) < 1e-6
        except Exception:
            ok = False
        if not ok:
            bad.append('edge#%s@%s' % (e.index,
                                       tuple(round(x, 3) for x in e.pointOn[0])))
    for c in cells:
        ok = False
        try:
            ok = _same_point(part.cells[c.index].pointOn[0], c.pointOn[0])
        except Exception:
            ok = False
        if not ok:
            ok = any(_same_point(pc.pointOn[0], c.pointOn[0])
                     for pc in part.cells)
        if not ok:
            bad.append('cell#%s@%s' % (c.index,
                                       tuple(round(x, 3) for x in c.pointOn[0])))
    if bad:
        raise ValueError(
            "These picks do not belong to Part %r. Repick on the current Part: %s"
            % (part.name, ', '.join(bad[:6])))


def _set_mesh_controls_hex(part, region=None):
    """Delegate to the core set_mesh_controls_hex helper."""
    return tmp.set_mesh_controls_hex(part, region=region)


def _flatten_objects(value):
    """Resolve a multi-pick keyword string into a flat list of entities."""
    if _is_blank_pick(value):
        return []
    seq = value
    if isinstance(seq, str):
        namespace = {"mdb": mdb}
        try:
            seq = eval(seq, namespace)
        except Exception as exc:
            raise ValueError("Cannot resolve picked selection %r: %s"
                             % (value, exc))
    items = []
    try:
        iterator = iter(seq)
    except TypeError:
        return [seq]
    for item in iterator:
        try:
            inner = list(item)
            if len(inner) == 1 and not hasattr(item, 'pointOn'):
                item = inner[0]
        except TypeError:
            pass
        items.append(item)
    return items


def _group_axial_bands(part, fine_edges, coarse_edges, region_faces,
                       target_cells, path_edges):
    """每组包含：t_fine, t_coarse, fine_edges, coarse_edges, faces, cells, paths。"""
    if not fine_edges or not coarse_edges:
        return []

    frame = tmp_cyl.build_cylinder_frame_from_edges(
        part, fine_edges[0], coarse_edges[0], mode='AXIAL_SURFACE')
    def _axial_t(edge):
        return tmp_cyl._cyl_polar(frame, edge.pointOn[0])[2]

    # 按轴向位置聚类细边
    tol_t = 0.01
    fine_groups = {}  # t -> [edges]
    for e in fine_edges:
        t = round(_axial_t(e), 4)
        placed = False
        for key in fine_groups:
            if abs(key - t) < tol_t:
                fine_groups[key].append(e)
                placed = True
                break
        if not placed:
            fine_groups[t] = [e]

    coarse_groups = {}
    for e in coarse_edges:
        t = round(_axial_t(e), 4)
        placed = False
        for key in coarse_groups:
            if abs(key - t) < tol_t:
                coarse_groups[key].append(e)
                placed = True
                break
        if not placed:
            coarse_groups[t] = [e]

    # 如果只有一组细边和一组粗边，返回单组
    if len(fine_groups) <= 1 and len(coarse_groups) <= 1:
        return [{'t_fine': _axial_t(fine_edges[0]),
                 't_coarse': _axial_t(coarse_edges[0]),
                 'fine_edges': fine_edges,
                 'coarse_edges': coarse_edges,
                 'faces': region_faces,
                 'cells': target_cells,
                 'paths': path_edges}]

    # 多组：配对每组细边与最近的粗边
    if len(fine_groups) != len(coarse_groups):
        raise ValueError('Axial grouping requires one coarse section for each '
                         'fine section; got %d fine and %d coarse sections.'
                         % (len(fine_groups), len(coarse_groups)))
    bands = []
    used_coarse = set()
    for t_f, f_edges in sorted(fine_groups.items()):
        # 找最近的未使用粗边组
        best_t_c = None
        best_dist = float('inf')
        for t_c in coarse_groups:
            if t_c in used_coarse:
                continue
            dist = abs(t_c - t_f)
            if dist < best_dist:
                best_dist = dist
                best_t_c = t_c
        if best_t_c is None:
            raise ValueError(
                'Axial grouping failed: fine group t=%g has no free coarse '
                'group (all %d coarse groups are taken).'
                % (t_f, len(coarse_groups)))
        used_coarse.add(best_t_c)
        c_edges = coarse_groups[best_t_c]

        # 分配面、Cells、路径边到该组（按轴向范围过滤，禁止整模型重复塞入）
        t_lo = min(t_f, best_t_c)
        t_hi = max(t_f, best_t_c)

        def _t_overlaps(a_lo, a_hi, b_lo, b_hi):
            return min(a_hi, b_hi) - max(a_lo, b_lo) > 1e-5

        band_faces = []
        for f in region_faces:
            try:
                fi = tmp_cyl._cyl_as_index(f)
                sig = tmp_cyl._cyl_face_span(part, part.faces[fi], frame, 1e-3)
                if _t_overlaps(sig['t_min'], sig['t_max'], t_lo, t_hi):
                    band_faces.append(part.faces[fi])
            except Exception:
                continue

        band_cells = []
        for c in target_cells:
            try:
                ci = tmp_cyl._cyl_as_index(c)
                t_lo_c, t_hi_c = tmp_cyl._cyl_cell_t_span(
                    part, frame, part.cells[ci], 1e-3)
                if t_lo_c is not None and _t_overlaps(t_lo_c, t_hi_c,
                                                      t_lo, t_hi):
                    band_cells.append(part.cells[ci])
            except Exception:
                continue

        cell_edge_ids = set()
        for c in band_cells:
            for e in c.getEdges():
                cell_edge_ids.add(tmp_cyl._cyl_as_index(e))
        band_paths = []
        for p in path_edges or []:
            try:
                if tmp_cyl._cyl_as_index(p) in cell_edge_ids:
                    band_paths.append(p)
            except Exception:
                continue

        if not band_faces or not band_cells:
            raise ValueError(
                'Axial grouping failed: band t=%g..%g has faces=%d cells=%d '
                'after filter. Confirm Region Faces / Target Cells cover '
                'this band.'
                % (t_lo, t_hi, len(band_faces), len(band_cells)))

        bands.append({'t_fine': t_f, 't_coarse': best_t_c,
                      'fine_edges': f_edges, 'coarse_edges': c_edges,
                      'faces': band_faces,
                      'cells': band_cells,
                      'paths': band_paths})

    # 检查未配对的粗边组
    for t_c in coarse_groups:
        if t_c not in used_coarse:
            raise ValueError('Axial coarse section t=%g was not paired.' % t_c)

    return bands if bands else [{'t_fine': _axial_t(fine_edges[0]),
                                 't_coarse': _axial_t(coarse_edges[0]),
                                 'fine_edges': fine_edges,
                                 'coarse_edges': coarse_edges,
                                 'faces': region_faces,
                                 'cells': target_cells,
                                 'paths': path_edges}]


def _resolve_axial_band(part, frame, band, tol):
    cells = [c for c in part.cells if tmp_cyl._cyl_cell_scope_match(
        part, frame, c, band['cell_signatures'], tol)]
    faces = tmp_cyl._cyl_refind_cyl_faces(
        part, frame, band['face_signatures'], band['radius'], tol)
    allowed = tmp_cyl._cyl_allowed_edge_indices(cells)
    loops = [[part.edges[j] for j in tmp_cyl._cyl_find_circ_edges_at(
        part, frame, band['radius'], band[key], tol) if j in allowed]
        for key in ('t_fine', 't_coarse')]
    paths = tmp_cyl._cyl_refind_path_edges(part, frame, band['path_signatures'], tol)
    if not cells or not faces or not all(loops):
        raise ValueError('Cannot re-locate the selected axial band after partition.')
    return dict(part=part, region_faces=[part.faces[j] for j in faces],
                fine_edges=loops[0], coarse_edges=loops[1],
                target_cells=cells, sweep_path_edges=paths)


def _verify_multistage_result(result, stages, planned_counts):
    """The kernel return must report the levels it really built."""
    result['planned_stages'] = int(stages)
    result['planned_stage_counts'] = [int(x) for x in planned_counts]
    actual_stages = result.get('stages')
    if actual_stages is not None and int(actual_stages) != int(stages):
        raise ValueError(
            'Multi-stage mismatch: requested %d stage(s), kernel built %d.'
            % (int(stages), int(actual_stages)))
    actual_counts = result.get('stage_counts')
    if actual_counts is not None:
        actual = [int(x) for x in actual_counts]
        planned = [int(x) for x in planned_counts]
        if len(actual) != len(planned) or any(a != b for a, b in
                                              zip(actual, planned)):
            raise ValueError(
                'Multi-stage count mismatch: planned %r, kernel built %r.'
                % (planned, actual))
    return result


def run_cylinder_transition_gui(modelName='Model-1', partName='Part-1',
                                cylMode='ANNULAR_RADIAL',
                                pickedRegionFaces='', pickedFineEdges='',
                                pickedCoarseEdges='', pickedCells='',
                                pickedPathEdges='', ratio='3:1',
                                seeds=180, action='PARTITION_SEED_MESH',
                                autoRegular=False, stages=1,
                                ratioSequence='', stageWidths=''):
    """圆筒过渡入口：用户选区驱动，调用已验证的圆筒内核。"""
    all_picks = (pickedRegionFaces, pickedFineEdges, pickedCoarseEdges,
                 pickedCells, pickedPathEdges)
    inferred = _infer_model_part(*all_picks)
    if inferred is not None:
        modelName, partName = inferred
    else:
        # 推导失败：如果显式参数仍为默认值且有非空拾取，禁止静默回退
        has_real_picks = any(not _is_blank_pick(v) for v in all_picks)
        at_defaults = (modelName == 'Model-1' and partName == 'Part-1')
        if has_real_picks and at_defaults:
            raise ValueError(
                "Cannot infer Model/Part from the picks. "
                "Open the target Part in the viewport and repick, "
                "or pass modelName/partName explicitly. "
                "(Defaults %s/%s do not match the picks.)"
                % (modelName, partName))
    if modelName not in mdb.models:
        raise ValueError("Model not found: %s" % modelName)
    model = mdb.models[modelName]
    if partName not in model.parts:
        raise ValueError("Part not found in %s: %s" % (modelName, partName))
    part = model.parts[partName]

    region_faces = _flatten_objects(pickedRegionFaces)
    fine_edges = _flatten_objects(pickedFineEdges)
    coarse_edges = _flatten_objects(pickedCoarseEdges)
    target_cells = _flatten_objects(pickedCells)
    path_edges = _flatten_objects(pickedPathEdges)
    ratio_int = _parse_ratio(ratio)

    if action not in _CYL_ACTIONS:
        raise ValueError("Illegal action %r (allowed: %s)"
                         % (action, ', '.join(_CYL_ACTIONS)))
    _verify_entities_in_part(part, region_faces, fine_edges + coarse_edges
                             + path_edges, target_cells)

    print("Cylinder Transition GUI: mode=%s model=%s part=%s ratio=%d:1 "
          "seeds=%d action=%s auto_regular=%s faces=%d fine=%d coarse=%d "
          "cells=%d paths=%d"
          % (cylMode, modelName, partName, ratio_int, int(seeds), action,
             bool(autoRegular), len(region_faces), len(fine_edges),
             len(coarse_edges), len(target_cells), len(path_edges)))

    n_stages = int(stages) if stages else 1
    ratios = (tm_plan.parse_sequence(ratioSequence, _parse_ratio)
              if ratioSequence else None)
    if cylMode == 'ANNULAR_RADIAL':
        if n_stages > 1:
            # Continuous multi-stage radial: reuse the multilayer imprint.
            chain = tm_plan.plan_counts(int(seeds), n_stages, ratio_int,
                                        ratioSequence)
            # Radii from the picked fine/coarse loops.
            fine_e = fine_edges[0]
            coarse_e = coarse_edges[0]
            frame = tmp_cyl.build_cylinder_frame_from_edges(
                part, fine_e, coarse_e, mode='ANNULAR_RADIAL')
            r_fine = tmp_cyl._cyl_polar(frame, fine_e.pointOn[0])[0]
            r_coarse = tmp_cyl._cyl_polar(frame, coarse_e.pointOn[0])[0]
            span = abs(r_coarse - r_fine)
            sign = 1.0 if r_coarse > r_fine else -1.0
            bounds = tm_plan.stage_boundaries(span, chain['counts'],
                                              stageWidths or None)
            radii = [r_fine + sign * b for b in bounds]
            result = tmp_cyl.apply_annular_radial_multilayer(
                part=part, region_faces=region_faces, radii=radii,
                fine_seeds=int(seeds), ratio=ratio_int,
                target_cells=target_cells, sweep_path_edges=path_edges,
                action=action, ratio_sequence=ratios,
                auto_regular=bool(autoRegular))
            return _verify_multistage_result(result, n_stages,
                                             chain['counts'])
        result = tmp_cyl.apply_annular_radial_transition(
            part=part, region_faces=region_faces, fine_edges=fine_edges,
            coarse_edges=coarse_edges, target_cells=target_cells,
            sweep_path_edges=path_edges, ratio=ratio_int,
            fine_seeds=int(seeds), action=action,
            auto_regular=bool(autoRegular))
        result['stages'] = 1
        result['stage_counts'] = [int(seeds), int(seeds) // ratio_int]
        return result
    elif cylMode == 'AXIAL_SURFACE':
        chain = tm_plan.plan_counts(int(seeds), n_stages, ratio_int, ratioSequence)
        # 轴向模式：按轴向位置分组，支持左右双过渡带独立处理
        bands = _group_axial_bands(part, fine_edges, coarse_edges,
                                   region_faces, target_cells, path_edges)
        if len(bands) <= 1:
            if n_stages > 1:
                result = tmp_cyl.apply_cylindrical_axial_multilayer(
                    part=part, region_faces=region_faces, fine_edges=fine_edges,
                    coarse_edges=coarse_edges, target_cells=target_cells,
                    sweep_path_edges=path_edges, ratio=ratio_int,
                    fine_seeds=int(seeds), action=action,
                    auto_regular=bool(autoRegular), ratio_sequence=ratios,
                    stage_widths=stageWidths or None, stages=n_stages)
                return _verify_multistage_result(result, n_stages, chain['counts'])
            result = tmp_cyl.apply_cylindrical_axial_transition(
                part=part, region_faces=region_faces, fine_edges=fine_edges,
                coarse_edges=coarse_edges, target_cells=target_cells,
                sweep_path_edges=path_edges, ratio=ratio_int,
                fine_seeds=int(seeds), action=action,
                auto_regular=bool(autoRegular))
        else:
            # 多组过渡带：预检全部通过后才改变模型
            print('Axial grouping: detected %d transition bands' % len(bands))
            for i, band in enumerate(bands):
                print('  Band %d: t_fine=%g t_coarse=%g fine=%d coarse=%d '
                      'cells=%d faces=%d'
                      % (i, band['t_fine'], band['t_coarse'],
                         len(band['fine_edges']), len(band['coarse_edges']),
                         len(band['cells']), len(band['faces'])))
            results = []
            # Snapshot every entity before the first partition changes indices.
            frame0 = tmp_cyl.build_cylinder_frame_from_edges(
                part, fine_edges[0], coarse_edges[0], mode='AXIAL_SURFACE')
            tol = tmp_cyl._CYL_TOL_ABS
            for band in bands:
                band['radius'] = tmp_cyl._cyl_polar(
                    frame0, band['fine_edges'][0].pointOn[0])[0]
                band['face_signatures'] = [tmp_cyl._cyl_face_span(
                    part, f, frame0, tol) for f in band['faces']]
                band['cell_signatures'] = [tmp_cyl._cyl_cell_signature(
                    part, c, frame0, tol) for c in band['cells']]
                band['path_signatures'] = tmp_cyl._cyl_path_signatures(
                    part, frame0, band['paths'], tol)
            operation_scope = (tmp_cyl._cyl_scope_cells(
                part, frame0, target_cells, tol)[0] if autoRegular
                else list(target_cells))
            operation_sigs = [tmp_cyl._cyl_cell_signature(part, c, frame0, tol)
                              for c in operation_scope]
            outside_before = tmp_cyl._cyl_multilayer_outside(
                part, frame0, operation_scope, tol)
            if action != 'PARTITION_ONLY':
                tmp_cyl._cyl_assign_transition_controls(part, target_cells)
                ri, ro = tmp_cyl._cyl_cell_radial_span(
                    part, frame0, target_cells[0], tol)
                tmp_cyl._cyl_assign_axial_paths(
                    part, frame0, target_cells, path_edges, ri, ro, tol)
            audit_signatures = []
            if n_stages > 1 and action != 'PARTITION_ONLY':
                # Partition all bands before meshing: a later section cut can
                # invalidate an earlier band's mesh through shared topology.
                for band in bands:
                    operation_cells = [c for c in part.cells if
                        tmp_cyl._cyl_cell_scope_match(part, frame0, c, operation_sigs, tol)]
                    tmp_cyl.apply_cylindrical_axial_multilayer(**dict(
                        _resolve_axial_band(part, frame0, band, tol),
                        ratio=ratio_int, fine_seeds=int(seeds), action='PARTITION_ONLY',
                        auto_regular=bool(autoRegular), ratio_sequence=ratios,
                        stage_widths=stageWidths or None, stages=n_stages,
                        operation_cells=operation_cells))
            for i, band in enumerate(bands):
                print('  Executing band %d...' % i)
                band_kwargs = dict(
                    _resolve_axial_band(part, frame0, band, tol),
                    ratio=ratio_int, fine_seeds=int(seeds),
                    action=action, auto_regular=bool(autoRegular))
                if n_stages > 1:
                    operation_cells = [c for c in part.cells
                        if tmp_cyl._cyl_cell_scope_match(
                            part, frame0, c, operation_sigs, tol)]
                    r = tmp_cyl.apply_cylindrical_axial_multilayer(
                        **dict(band_kwargs, ratio_sequence=ratios,
                               stage_widths=stageWidths or None, stages=n_stages,
                               operation_cells=operation_cells))
                    r = _verify_multistage_result(r, n_stages, chain['counts'])
                else:
                    r = tmp_cyl.apply_cylindrical_axial_transition(**band_kwargs)
                r['band_index'] = i
                r['t_fine'] = band['t_fine']
                r['t_coarse'] = band['t_coarse']
                results.append(r)
                if action == 'PARTITION_SEED_MESH':
                    ids = set()
                    for key in ('cell_elements', 'regular_elements', 'recovered_cells'):
                        counts = r.get(key) or {}
                        ids.update(k for k in counts if isinstance(k, int))
                        ids.update(counts.get('_skipped') or [])
                    audit_signatures.extend(tmp_cyl._cyl_cell_signature(
                        part, part.cells[j], frame0, tol) for j in sorted(ids))
            result = {'mode': 'AXIAL_SURFACE', 'multi_band': True,
                      'n_bands': len(bands), 'bands': results,
                      'action': action, 'elements': len(part.elements),
                      'stages': n_stages, 'stage_counts': chain['counts']}
            if action == 'PARTITION_SEED_MESH':
                audit_cells = [c for c in part.cells
                               if tmp_cyl._cyl_cell_scope_match(
                                   part, frame0, c, audit_signatures, tol)]
                lost = [c for c in audit_cells if len(c.getElements()) == 0]
                if lost:
                    result['recovered_cells'] = tmp_cyl._cyl_mesh_cells(
                        part, lost, {}, skip_meshed=True)
                    result['elements'] = len(part.elements)
                counts = dict((c.index, len(c.getElements())) for c in audit_cells)
                result['cell_elements'] = counts
                result['mesh_audit'] = tmp_cyl._cyl_audit_generated_mesh(
                    part, audit_cells, counts)
                for band, r in zip(bands, results):
                    live_cells = [c for c in part.cells
                                  if tmp_cyl._cyl_cell_scope_match(
                                      part, frame0, c, band['cell_signatures'], tol)]
                    r['interface_counts'] = tmp_cyl._cyl_audit_axial_rings(
                        part, frame0, live_cells, band['t_fine'],
                        band['t_coarse'], int(seeds), chain['counts'][-1], tol)
                    if n_stages > 1:
                        bounds = tm_plan.stage_boundaries(
                            abs(band['t_coarse'] - band['t_fine']), chain['counts'],
                            stageWidths or None)
                        sign = 1 if band['t_coarse'] > band['t_fine'] else -1
                        levels = [band['t_fine'] + sign * b for b in bounds]
                        stage_cells = [[c for c in live_cells if all(
                            abs(x-y) < tol for x,y in zip(
                                tmp_cyl._cyl_cell_t_span(part, frame0, c, tol),
                                (min(a,b), max(a,b))))]
                            for a,b in zip(levels[:-1],levels[1:])]
                        r['mesh_audit'] = tmp_cyl._cyl_multilayer_audit(
                            part, frame0, stage_cells, levels, chain['counts'],
                            'AXIAL_SURFACE', band['radius'], r['radial_layers'], tol)
                result['interface_counts'] = {'ok': True,
                    'bands': [r['interface_counts'] for r in results]}
            final_scope = [c for c in part.cells
                if tmp_cyl._cyl_cell_scope_match(part, frame0, c, operation_sigs, tol)]
            result['scope_audit'] = tmp_cyl._cyl_multilayer_check_outside(
                part, frame0, final_scope, outside_before, tol)
    else:
        raise ValueError("Unsupported cylinder mode: %s" % cylMode)
    print("Cylinder Transition GUI completed: %s" % result)
    return result


def _check_planar2d(ttype, part, faces, fine, coarse, ratio, seeds,
                    fineSeedsY, meshRegular2d, domain, stages=1,
                    ratioSequence='', stageWidths=''):
    """Builds the plan (no partition/seed/mesh)."""
    import transition_mesh_plugin_planar2d as p2d
    ratio_i = _parse_ratio(ratio)
    N = int(seeds)
    mr = bool(meshRegular2d) if meshRegular2d is not None else True
    use_domain = list(domain) if (mr and domain) else []
    n_stages = int(stages) if stages else 1
    if n_stages < 1:
        raise ValueError('Stages must be >= 1.')

    if ttype == 'PLANAR_ANNULAR':
        chain = tm_plan.plan_counts(N, n_stages, ratio_i, ratioSequence)
        plan = p2d._build_annular_patch_plan(
            part, faces, fine, coarse, ratio_i, N,
            domain_faces=use_domain or None,
            stages=n_stages, ratioSequence=ratioSequence,
            stageWidths=stageWidths)
        coarse_N = plan['coarse_seeds']
        lines = [
            'Input checks passed  PLANAR_ANNULAR  stages=%d' % n_stages,
            'Fine loop N=%d  chain=%s  periods=%d'
            % (N, chain['counts'], plan['n_period']),
        ]
        if use_domain:
            lines.append('Regular rings: included (Full domain picked).')
        else:
            lines.append('Transition only (no Full domain / regular off).')
        if plan.get('fine_verts'):
            lines.append('Existing radial partitions: %d on fine loop.'
                         % len(plan['fine_verts']))
        lines.append('Counts + plan only. Use Pre-mesh to trial-mesh and roll back.')
        if n_stages > 1:
            exp = sum(p2d.expected_transition_elements(st['ratio'],
                                                       st['fine_seeds'])
                      for st in plan['stage_specs'])
        else:
            exp = p2d.expected_transition_elements(ratio_i, N)
        return {
            'status': 'PASS', 'mode': ttype, 'ratio': ratio_i,
            'checks_pending': ['generated mesh quality (use Pre-mesh)'],
            'fine_seeds': N, 'coarse_seeds': coarse_N,
            'n_periods': plan['n_period'],
            'expected_template_faces': exp,
            'stage_counts': chain['counts'],
            'scope': 'domain' if use_domain else 'transition_only',
            'summary_lines': lines,
            'summary': '\n'.join(lines),
        }

    ny = int(fineSeedsY) if fineSeedsY else N
    if ratio_i == 2:
        plan = p2d._build_rect_four_patch_plan(
            part, faces, fine, coarse, ratio_i, N, ny,
            domain_faces=use_domain, mesh_regular=mr, stages=n_stages,
            ratioSequence=ratioSequence, stageWidths=stageWidths)
        geo = plan['geometry']
        lines = ['Case squad 2:1: %d transition levels; four diagonal corners per level.' % n_stages,
                 'Original fine seeds: Nx=%d Ny=%d (preserved).' % (N, ny)]
        for st in geo['levels']:
            lines.append('Level %d: input %d/%d -> output %d/%d; regular collar x/y=%d/%d.'
                         % (st['stage'], st['nx'], st['ny'], st['out_nx'], st['out_ny'],
                            st['collar_x'], st['collar_y']))
        lines.append('Coarse frame %s: half-sizes H=%.9g V=%.9g.'
                     % ('AUTO' if plan['auto_coarse'] else 'picked', plan['H'], plan['V']))
        lines.append('Geometry checked. Use Pre-mesh to verify the generated mesh.')
        return {'status': 'PASS', 'mode': ttype, 'ratio': 2, 'Nx': N, 'Ny': ny,
                'stages': n_stages, 'corner_participating': True,
                'nx_coarse': plan['nx_coarse'], 'ny_coarse': plan['ny_coarse'],
                'stage_counts': plan['stage_counts_x'], 'stage_details': geo['levels'],
                'auto_coarse': plan['auto_coarse'], 'summary_lines': lines,
                'summary': '\n'.join(lines), 'checks_pending': ['generated mesh quality']}
    if n_stages > 1:
        chain = tm_plan.plan_quad_counts(N, n_stages, ratio_i, ratioSequence)
        chain_y = tm_plan.plan_quad_counts(ny, n_stages, ratio_i, ratioSequence)
    else:
        chain = tm_plan.plan_counts(N, n_stages, ratio_i, ratioSequence)
        chain_y = tm_plan.plan_counts(ny, n_stages, ratio_i, ratioSequence)
    plan = p2d._build_rect_four_patch_plan(
        part, faces, fine, coarse, ratio_i, N, ny,
        domain_faces=use_domain, mesh_regular=mr,
        stages=n_stages, ratioSequence=ratioSequence,
        stageWidths=stageWidths)
    ext = plan.get('external')
    corner_on = bool(plan.get('corner_participating'))
    rule = plan.get('rule') or {}
    wu = (rule.get('width_u') or [0.0])[-1]
    wv = (rule.get('width_v') or [0.0])[-1]
    lines = [
        'Input checks passed  PLANAR_QUAD_FOUR  layers=%d' % n_stages,
        'Nx=%d Ny=%d  chain_x=%s  outer %d / %d'
        % (N, ny, chain['counts'], plan['nx_coarse'], plan['ny_coarse']),
        'Fine size h_u=%.6g h_v=%.6g  coarse=%.6g/%.6g  corners=plain'
        % (rule.get('h_u', 0.0), rule.get('h_v', 0.0), wu, wv),
    ]
    if plan.get('auto_coarse'):
        lines.append('Coarse loop: AUTO at half-sizes H=%.6g V=%.6g '
                     '(case rule 4: side + 2 x coarse element).'
                     % (plan['H'], plan['V']))
    else:
        lines.append('Coarse loop: picked, half-sizes H=%.6g V=%.6g.'
                     % (plan['H'], plan['V']))
    warn = False
    cap = tm_rules.capability(ttype, ratio_i, n_stages)
    lines.append('Capability: %s%s' % (
        cap['headline'],
        ('  ' + cap['precondition']) if cap['precondition'] else ''))
    if cap['status'] == 'no':
        warn = False
        lines.append('NOT supported: %s %s' % (cap['reason'], cap['fix']))
    elif cap['status'] == 'warn':
        warn = True
        lines.append('Caveat: %s' % (cap['note'] or cap['reason']))
    dev = rule.get('deviation')
    if dev:
        warn = True
        lines.append(
            'Rule 4 not met: need half-sizes %.6g/%.6g, picked %.6g/%.6g. '
            'Constructible, but the corner element will differ from the '
            'coarse size (untidy ring).' % (dev['H_required'],
                                            dev['V_required'],
                                            dev['H_actual'], dev['V_actual']))
    if ratio_i == 3:
        warn = True
        lines.append('3:1 four-side is constructible but the mid-depth row is '
                     'inherent (element aspect ~3); 2:1 stays square.')
    if cap['status'] == 'no':
        fail = True
    else:
        fail = False
    if ext:
        lines.append(
            'Regular frame: kL=%s kR=%s kT=%s kB=%s  ext~%s elems'
            % (ext['k_left'], ext['k_right'], ext['k_top'], ext['k_bottom'],
               plan.get('external_expected')))
    else:
        lines.append('Transition only (no Full domain / regular off).')
    if abs(plan['dx'] - plan['hcx']) > 1e-3 or abs(plan['dy'] - plan['hcy']) > 1e-3:
        try:
            nx_eq = int(round(2.0 * plan['hxi'] * ratio_i
                              / max(plan['dx'], 1e-12)))
            ny_eq = int(round(2.0 * plan['hyi'] * ratio_i
                              / max(plan['dy'], 1e-12)))
            per = 4 if ratio_i == 2 else 3
            if nx_eq > 0 and nx_eq % per == 0:
                lines.append('Equal-width hint: try Nx=%d (now corner=%.3g).'
                             % (nx_eq, plan['dx']))
            if ny_eq > 0 and ny_eq % per == 0:
                lines.append('Equal-width hint: try Ny=%d (now corner=%.3g).'
                             % (ny_eq, plan['dy']))
        except Exception:
            pass
    lines.append('Counts + plan only. Use Pre-mesh to trial-mesh and roll back.')
    return {
        'status': ('FAIL' if fail else ('WARN' if warn else 'PASS')),
        'mode': ttype, 'ratio': ratio_i,
        'capability': cap['status'],
        'checks_pending': ['generated mesh quality (use Pre-mesh)'],
        'Nx': N, 'Ny': ny,
        'nx_coarse': plan['nx_coarse'], 'ny_coarse': plan['ny_coarse'],
        'corner_participating': corner_on,
        'auto_coarse': bool(plan.get('auto_coarse')),
        'stages': n_stages,
        'external_expected': plan.get('external_expected'),
        'stage_counts': chain['counts'],
        'scope': 'domain' if ext else 'transition_only',
        'summary_lines': lines,
        'summary': '\n'.join(lines),
    }


def _resolve_hybrid_blocks(seeds, blocks):
    """The user supplies b (the number of 2:1 groups); the 3:"""
    N = int(seeds)
    b = int(blocks)
    if b <= 0:
        raise ValueError('Hybrid needs a positive 2:1 block count b.')
    if b % 2:
        raise ValueError('Hybrid 2:1 block count b=%d must be even: the 2:1 '
                         'template tiles in pairs (4 fine -> 2 coarse).' % b)
    rem = N - 2 * b
    if rem < 0:
        raise ValueError('Hybrid b=%d is too large for N=%d: needs 2b <= N.'
                         % (b, N))
    if rem % 3:
        raise ValueError('Hybrid b=%d is invalid for N=%d: (N-2b)=%d is not '
                         'a multiple of 3, so the 3:1 block count would not '
                         'be an integer.' % (b, N, rem))
    a = rem // 3
    return a + b, a, b


def _run_transition_mesh_gui(modelName='Model-1', partName='Part-1', mode='PLANAR',
                            orientation='HU', facePoint='',
                            edgePoint='', depth=5.0, layers=1,
                            seeds=4, ratio='2:1', curvedSurface=False,
                            generateMesh=False, validate=True,
                            pickedFace='', pickedEdge='',
                            transitionType='PLANAR',
                            pickedRegionFaces='', pickedFineEdges='',
                            pickedCoarseEdges='', pickedCells='',
                            pickedPathEdges='', action='PARTITION_SEED_MESH',
                            autoRegular=False, coarseSeeds=0,
                            hybridPattern='', hybridBlocks=0,
                            element2d='CPS4R', fineSeedsY=0,
                            meshRegular2d=True, pickedDomainFaces='',
                            checkOnly=False, stages=1, ratioSequence='',
                            stageWidths='', pickedPlanarCoarseEdge=None):
    """checkOnly=True runs a read-only geometry/count check and never mutates the model (shared by every"""
    ttype = _canonical_transition_type(transitionType)
    if ttype is None:
        raise ValueError(
            "Unsupported transition type: %r (use PLANAR, PLANAR_ANNULAR, "
            "PLANAR_QUAD_FOUR, ANNULAR_RADIAL or AXIAL_SURFACE)"
            % (transitionType,))
    selection_values = {
        'pickedFace': pickedFace,
        'pickedEdge': pickedEdge,
        'pickedPlanarCoarseEdge': pickedPlanarCoarseEdge,
        'pickedRegionFaces': pickedRegionFaces,
        'pickedFineEdges': pickedFineEdges,
        'pickedCoarseEdges': pickedCoarseEdges,
        'pickedCells': pickedCells,
        'pickedPathEdges': pickedPathEdges,
        'pickedDomainFaces': pickedDomainFaces,
    }
    classic_planar_coarse = _classic_planar_coarse_pick(selection_values)
    all_picks = _route_pick_values(ttype, selection_values)
    inferred = _infer_model_part(*all_picks)
    if inferred is not None:
        modelName, partName = inferred
    else:
        # 推导失败：如果显式参数仍为默认值且有非空拾取，禁止静默回退
        has_real_picks = any(not _is_blank_pick(v) for v in all_picks)
        at_defaults = (modelName == 'Model-1' and partName == 'Part-1')
        if has_real_picks and at_defaults:
            raise ValueError(
                "Cannot infer Model/Part from the picks. "
                "Open the target Part in the viewport and repick, "
                "or pass modelName/partName explicitly. "
                "(Defaults %s/%s do not match the picks.)"
                % (modelName, partName))
    if modelName not in mdb.models:
        raise ValueError("Model not found: %s" % modelName)
    model = mdb.models[modelName]
    if partName not in model.parts:
        raise ValueError("Part not found in %s: %s" % (modelName, partName))

    if ttype in ('ANNULAR_RADIAL', 'AXIAL_SURFACE'):
        # Shared Check: cylinder templates are not rebuilt this round;
        # return a count-only summary without mutating.
        if checkOnly or str(action).upper() == 'CHECK_ONLY':
            ratio_i = _parse_ratio(ratio)
            N = int(seeds)
            n_stages = int(stages) if stages else 1
            chain = tm_plan.plan_counts(N, n_stages, ratio_i, ratioSequence)
            # Same-ratio contract as the multilayer kernels this round: a
            # mixed sequence must be refused at Check time, not at Apply.
            if n_stages > 1 and len(set(chain['ratios'])) != 1:
                raise ValueError(
                    'Cylinder multi-stage currently supports one ratio '
                    'repeated (2:1 or 3:1); mixed sequence %r is not '
                    'verified. Use a single ratio, or Stages=1.'
                    % (chain['ratios'],))
            part = model.parts[partName]
            faces = _flatten_objects(pickedRegionFaces)
            fine = _flatten_objects(pickedFineEdges)
            coarse = _flatten_objects(pickedCoarseEdges)
            cells = _flatten_objects(pickedCells)
            paths = _flatten_objects(pickedPathEdges)
            # sweep path 由内核从细/粗圆边自行推导（cylinder frame），
            # 用户拾取可不传 —— 3e7043e 曾误加 not paths 导致圆筒 Check全挂（验收矩阵夹具从不传 path）。
            if not faces or not fine or not coarse or not cells:
                raise ValueError('Select transition faces, fine/coarse edges and Cells.')
            _verify_entities_in_part(part, faces, fine + coarse + paths, cells)
            tmp_cyl.build_cylinder_frame_from_edges(part, fine[0], coarse[0], mode=ttype)
            if ttype == 'ANNULAR_RADIAL':
                for count, rr in zip(chain['counts'], chain['ratios']):
                    if count % (16 if rr == 2 else 12):
                        raise ValueError('Annular cylinder quadrant templates require each stage '
                                         'fine count to be divisible by %d.' % (16 if rr == 2 else 12))
            per = 4 if ratio_i == 2 else 3
            lines = [
                'Input checks passed  %s  %d:1' % (ttype, ratio_i),
                'Fine loop N=%d  stages=%d' % (N, n_stages),
                'Stage chain: %s' % (chain['counts'],),
            ]
            if N % per == 0:
                lines.append('Divisible by %d: OK. coarse N/r=%d.'
                             % (per, N // ratio_i))
            else:
                lo = (N // per) * per
                lines.append(
                    'Need divisible by %d. near %d/%d'
                    % (per, max(per, lo), lo + per))
            # One shared capability table decides what this combination can
            # do, so the dialog line and this report can never disagree.
            cap = tm_rules.capability(ttype, ratio_i, n_stages)
            lines.append('Capability: %s%s' % (
                cap['headline'],
                ('  ' + cap['precondition']) if cap['precondition'] else ''))
            warn = cap['status'] == 'warn'
            fail = cap['status'] == 'no'
            if n_stages > 1:
                if ttype == 'ANNULAR_RADIAL':
                    warn = True
                    lines.append('Stages > 1: the picked target Cells must '
                                 'already be partitioned at the intermediate radii.')
                else:
                    lines.append('Intermediate axial sections are created automatically '
                                 'inside each selected transition band.')
            if ttype == 'AXIAL_SURFACE':
                # The axial lane patterns the wall by 360/n_slices anchored on an existing radial seam;
                # if every plane lands on your own cuts the partition adds nothing and the cells stay unmeshed.
                try:
                    bands = _group_axial_bands(part, fine, coarse, faces,
                                               cells, paths)
                    lines.append('Axial transition bands: %d; stages per band: %d.'
                                 % (len(bands), n_stages))
                    for band in bands:
                        tm_plan.stage_boundaries(abs(band['t_coarse'] - band['t_fine']),
                                                chain['counts'], stageWidths or None)
                    frame = tmp_cyl.build_cylinder_frame_from_edges(
                        part, fine[0], coarse[0], mode=ttype)
                    tf = tmp_cyl._cyl_polar(frame, fine[0].pointOn[0])[2]
                    tc = tmp_cyl._cyl_polar(frame, coarse[0].pointOn[0])[2]
                    seams = tmp_cyl._cyl_seam_angles_deg(
                        part, frame, tmp_cyl._CYL_TOL_ABS,
                        min(tf, tc), max(tf, tc))
                    fit = tm_rules.angular_fit(tm_rules.axial_n_slices(
                        N, ratio_i), seams)
                    lines.append(
                        'Angular fit: %d sectors x %.6g deg, %d existing '
                        'seam(s), %d new slice plane(s) -> %s'
                        % (fit['n_slices'], fit['spacing_deg'],
                           len(fit['seams']), fit['new_planes'],
                           fit['verdict']))
                    if fit['verdict'] == 'no':
                        fail = True
                        lines.append(
                            'FIX: axial slice pattern collides with the '
                            'existing cuts -- %s Use Seeds=%s, or keep ONE '
                            'radial seam and let the plugin make the sectors.'
                            % (fit['reason'],
                               tm_rules.angular_fix_counts(N, ratio_i)[:4]))
                    elif fit['verdict'] == 'warn':
                        warn = True
                        lines.append('NOTE: %s' % fit['reason'])
                except Exception as exc:
                    fail = True
                    lines.append('Angular fit not evaluated: %s'
                                 % str(exc)[:140])
            lines.append('Counts + frame only. Use Pre-mesh to trial-mesh and roll back.')
            raw = 'FAIL' if fail else ('WARN' if warn else 'PASS')
            return {
                'status': raw, 'mode': ttype, 'ratio': ratio_i,
                'stages': n_stages,
                'checks_pending': ['generated mesh quality and sweepability '
                                   '(use Pre-mesh)'],
                'stage_counts': chain['counts'],
                'capability': cap['status'],
                'fine_seeds': N, 'summary_lines': lines,
                'summary': '\n'.join(lines),
            }
        return run_cylinder_transition_gui(
            modelName=modelName, partName=partName, cylMode=ttype,
            pickedRegionFaces=pickedRegionFaces,
            pickedFineEdges=pickedFineEdges,
            pickedCoarseEdges=pickedCoarseEdges,
            pickedCells=pickedCells,
            pickedPathEdges=pickedPathEdges,
            ratio=ratio, seeds=seeds, action=action,
            autoRegular=autoRegular,
            stages=stages, ratioSequence=ratioSequence,
            stageWidths=stageWidths)

    part = model.parts[partName]

    if ttype in _PLANAR_2D:
        import transition_mesh_plugin_planar2d as p2d
        faces = _flatten_objects(pickedRegionFaces)
        if not faces:
            faces = _flatten_objects(pickedFace)
        fine = _flatten_objects(pickedFineEdges)
        if not fine:
            e0 = _resolve_picked_object(pickedEdge)
            # created by IY,China University of Petroleum (East China)
            fine = [e0] if e0 is not None else []
        coarse = _flatten_objects(pickedCoarseEdges)
        domain = _flatten_objects(pickedDomainFaces)
        if not fine or (ttype == 'PLANAR_ANNULAR' and not coarse):
            raise ValueError(
                'PLANAR_ANNULAR needs fine and coarse edge loops and '
                'PLANAR_QUAD_FOUR needs the fine loop; use 2) Fine loop / '
                '3) Coarse loop (the coarse loop may be left out for '
                'PLANAR_QUAD_FOUR: it is then computed from the ratio and '
                'Layers).')
        # Shared Action contract (spec 6.1 / 7): CHECK_ONLY never mutates.
        if checkOnly or str(action).upper() == 'CHECK_ONLY':
            return _check_planar2d(
                ttype, part, faces, fine, coarse, ratio, seeds, fineSeedsY,
                meshRegular2d, domain, stages=stages,
                ratioSequence=ratioSequence, stageWidths=stageWidths)
        if ttype == 'PLANAR_ANNULAR':
            # mesh_regular False: ignore any stale domain pick (spec 6.1).
            # mesh_regular None keeps legacy script behaviour (domain => mesh).
            mr = None if meshRegular2d is None else bool(meshRegular2d)
            use_domain = domain if (mr is None or mr) else []
            if mr is False and domain:
                print('Planar2D: meshRegular2d=False; domain pick ignored.')
            if mr is True and not domain:
                print('Planar2D: No domain selected: transition only.')
            result = p2d.apply_planar_annular_transition(
                part=part, region_faces=faces, fine_edges=fine,
                coarse_edges=coarse, ratio=_parse_ratio(ratio),
                fine_seeds=int(seeds), action=action,
                element2d=element2d, domain_faces=use_domain,
                stages=int(stages) if stages else 1,
                ratioSequence=ratioSequence or '',
                stageWidths=stageWidths or '')
        else:
            ny = int(fineSeedsY) if fineSeedsY else int(seeds)
            mr = bool(meshRegular2d)
            use_domain = domain if mr else []
            if mr and not domain:
                print('Planar2D: No domain selected: transition only.')
            result = p2d.apply_planar_quad_four_transition(
                part=part, region_faces=faces, fine_edges=fine,
                coarse_edges=coarse, ratio=_parse_ratio(ratio),
                fine_seeds=int(seeds), fine_seeds_y=ny,
                action=action, element2d=element2d,
                mesh_regular=mr, domain_faces=use_domain,
                stages=int(stages) if stages else 1,
                ratioSequence=ratioSequence or '',
                stageWidths=stageWidths or '')
        if (ttype == 'PLANAR_QUAD_FOUR' and len(part.cells) > 0
                and str(action).upper() == 'PARTITION_SEED_MESH'):
            result['solid_audit'] = _planar_solid_audit(part, result)
            result['final_elements'] = len(part.elements)
            result['final_nodes'] = len(part.nodes)
        print("Planar2D GUI completed: %s" % result)
        return result

    mode = str(mode).upper()
    orientation = str(orientation).upper()
    if curvedSurface and mode == 'PLANAR':
        mode = 'CYLINDER'

    face_items = _flatten_objects(pickedFace)
    edge_items = _flatten_objects(pickedEdge)
    if len(face_items) > 1:
        raise ValueError(
            'Partition Face requires exactly one face; got %d. '
            'Repick one face and press Done.' % len(face_items))
    if len(edge_items) > 1:
        raise ValueError(
            'Finer Edge requires exactly one edge; got %d. '
            'Repick one edge and press Done.' % len(edge_items))
    face = face_items[0] if face_items else None
    edge = edge_items[0] if edge_items else None
    face_point = None
    if face is None:
        if not str(facePoint).strip():
            raise ValueError("Pick a partition face from the viewport.")
        face_point = _parse_point(facePoint)
        face = _find_face(part, face_point)
    if edge is None:
        if str(edgePoint).strip():
            edge = _find_edge(part, edgePoint)

    if face is None:
        raise ValueError("Pick a partition face or provide Face point X,Y,Z.")
    if mode == 'PLANAR' and edge is None:
        raise ValueError("Pick a finer edge for PLANAR transition.")

    n = int(stages) if int(stages) != 1 else int(layers)
    seeds = int(seeds)
    depth = float(depth)
    ratio_int = _parse_ratio(ratio)

    # Shared Action: CHECK_ONLY is read-only for classic PLANAR too.
    if checkOnly or str(action).upper() == 'CHECK_ONLY':
        if hybridBlocks:
            cs, _hyb_a, _hyb_b = _resolve_hybrid_blocks(seeds, hybridBlocks)
        else:
            cs = int(coarseSeeds) if coarseSeeds else None
        hybrid_info = None
        coarse_edge = _planar_coarse_edge(classic_planar_coarse)
        _verify_entities_in_part(
            part, [face],
            ([e for e in (edge, coarse_edge) if e is not None]), [])
        if depth <= 0 and coarse_edge is None:
            raise ValueError('Transition depth must be positive.')
        if cs:
            if n != 1:
                raise ValueError('Hybrid within one stage cannot be combined with multiple stages.')
            # Same splitter Apply uses: unique a/b from (N, M) plus the
            # even-pair requirement and the deterministic block order.
            hybrid_info = tm_rules.hybrid_split(seeds, cs,
                                                sequence=hybridPattern)
            if not hybrid_info['ok']:
                raise ValueError(hybrid_info['error'])
            stage_counts = [int(seeds), cs]
        else:
            chain = tm_plan.plan_counts(seeds, n, ratio_int, ratioSequence)
            stage_counts = chain['counts']
        frame = tmp.build_face_frame(face, edge, part=part)
        depth_source = 'user_depth'
        if coarse_edge is not None:
            # The coarse edge owns both the width and the side here; the typed
            # Depth and Orientation become informational only.
            depth, side, orientation = tmp.resolve_depth_from_coarse_edge(
                part=part, face=face, fine_edge=edge, coarse_edge=coarse_edge,
                v_dir=frame[2], u_dir=frame[1], normal=frame[3])
            depth_source = 'coarse_edge'
        else:
            side = tmp._resolve_planar_side(face, edge, part, orientation, depth,
                                            frame[3], frame[1], frame[2])
        # Use the same width planner as partition_planar_transition. Invalid
        # per-stage widths must fail Check before the first model mutation.
        stage_widths = tm_plan.allocate_widths(depth, stage_counts, stageWidths)
        fine_endpoints = tmp._edge_points(edge, part=part)
        coarse_boundary = [tuple(p[i] - side * depth * frame[2][i]
                                 for i in range(3)) for p in fine_endpoints]
        domain_for_planar = []
        if bool(meshRegular2d):
            domain_for_planar = _flatten_objects(pickedDomainFaces)
            _verify_entities_in_part(part, domain_for_planar, [], [])
        lines = [
            'Input checks passed  PLANAR  %d:1' % ratio_int,
            'Fine edge N=%d  stages=%d  depth=%g  orient=%s'
            % (seeds, n, depth, orientation),
            'Stage chain: %s' % (stage_counts,),
            'Stage widths: %s (total Depth=%g)' % (stage_widths, depth),
        ]
        if depth_source == 'coarse_edge':
            lines.append('Depth=%g measured from the picked coarse edge; the '
                          'typed Depth value was ignored.' % depth)
        else:
            lines.append('Coarse boundary computed from the finer edge + '
                         'Depth=%g; no coarse-edge pick is needed.' % depth)
        if cs:
            lines.append('Hybrid coarse seeds=%d  a=%d (3:1 blocks)  '
                         'b=%d (2:1, %d pair blocks)  blocks=%d'
                         % (cs, hybrid_info['a'], hybrid_info['b'],
                            hybrid_info['pairs'], len(hybrid_info['blocks'])))
            order = hybrid_info['pattern']
            if len(order) > 120:
                order = order[:117] + '...'
            _order_label = {'user': 'user pattern',
                            'interleave': 'auto even interleave',
                            'grouped': 'auto grouped, 2:1 on the left half, '
                                       '3:1 on the right half'}
            lines.append('Block order (%s): %s'
                         % (_order_label.get(hybrid_info.get('order'),
                                             'auto'), order))
            lines.append('Coarse side alternates 2h/3h element widths; this '
                         'is inherent to a mixed ratio.')
        if domain_for_planar:
            lines.append('Regular domain faces: %d picked.' % len(domain_for_planar))
            if cs and len(part.cells) == 0:
                p0, p1 = tmp._edge_points(edge, part=part)
                frame_origin = tuple(
                    0.5 * (p0[i] + p1[i]) for i in range(3))
                length = tmp._vnorm(tmp._vsub(p1, p0))
                ext_faces = tmp._extension_candidate_faces(
                    face, domain_for_planar, part, frame_origin,
                    frame[1], frame[2], frame[3], side, depth, length)
                ext_value = tmp._regular_extension_from_faces(
                    [face] + ext_faces, part, frame_origin, frame[2],
                    side, depth)
                if ext_value > 0.0:
                    lines.append(
                        'Hybrid extension: %d Full-domain face(s), %d '
                        'internal coarse line(s).'
                        % (len(ext_faces), max(0, int(cs) - 1)))
                else:
                    lines.append(
                        'Hybrid extension: no face beyond the coarse edge; '
                        'the regular domain will not be cut.')
        else:
            lines.append('Transition only (no Full domain / regular off).')
        lines.append('Counts + plan only. Use Pre-mesh to trial-mesh and roll back.')
        return {
            'status': 'PASS', 'mode': 'PLANAR', 'ratio': ratio_int,
            'checks_pending': ['generated mesh quality (use Pre-mesh)'],
            'fine_seeds': seeds, 'layers': n, 'depth': depth,
            'depth_source': depth_source,
            'orientation': orientation,
            'coarse_seeds': cs,
            'stage_counts': stage_counts,
            'stage_widths': stage_widths,
            'coarse_boundary': coarse_boundary,
            'scope': 'domain' if domain_for_planar else 'transition_only',
            'summary_lines': lines,
            'summary': '\n'.join(lines),
        }

    print("Transition Mesh GUI: mode=%s model=%s part=%s ratio=%d:1 pickedFace=%s pickedEdge=%s facePoint=%s edgePoint=%s" %
          (mode, modelName, partName, ratio_int, face is not None, edge is not None, face_point, edgePoint))

    if mode == 'PLANAR':
        # Hybrid: coarseSeeds > 0 passes an explicit coarse-side count; the GUI instead sends a 2:
        # 1 block count b (hybridBlocks) from which the coarse count M is derived.
        if hybridBlocks:
            cs, _hyb_a, _hyb_b = _resolve_hybrid_blocks(seeds, hybridBlocks)
        else:
            cs = int(coarseSeeds) if coarseSeeds else None
        domain_for_planar = []
        if bool(meshRegular2d):
            domain_for_planar = _flatten_objects(pickedDomainFaces)
        features_before = set(part.features.keys())
        # A planar face on a solid is the source face of the user-prepared transition Cell(s).
        # Capture those Cells before face partitioning;
        transition_cell_indices = []
        if len(part.cells) > 0:
            try:
                transition_cell_indices = sorted(set(int(i)
                    for i in face.getCells()))
            except Exception:
                transition_cell_indices = []
            if not transition_cell_indices:
                raise ValueError(
                    'The selected planar face is not attached to a solid Cell. '
                    'Pick the source face of the user-prepared transition Cell.')
        source_index = face.index
        include_source_regular = any(f.index == source_index for f in domain_for_planar)
        regular_points = [tuple(f.pointOn[0]) for f in domain_for_planar
                          if f.index != source_index]
        # Snapshot source-face UV domain BEFORE partition so children can be re-located geometrically after
        # PartitionFaceBySketch (which does not expose getFeatureFaces on 2D planar parts).
        origin0, u_dir0, v_dir0, normal0 = tmp.build_face_frame(
            face, edge, part=part)
        src_us, src_vs = [], []
        try:
            for vid in face.getVertices():
                p = part.vertices[vid].pointOn[0]
                rel = tmp._vsub(p, origin0)
                if abs(tmp._vdot(rel, normal0)) > 1e-3:
                    continue
                src_us.append(tmp._vdot(rel, u_dir0))
                src_vs.append(tmp._vdot(rel, v_dir0))
        except Exception:
            pass
        try:
            p_on = tuple(face.pointOn[0])
            rel = tmp._vsub(p_on, origin0)
            if abs(tmp._vdot(rel, normal0)) <= 1e-3:
                src_us.append(tmp._vdot(rel, u_dir0))
                src_vs.append(tmp._vdot(rel, v_dir0))
        except Exception:
            pass
        src_bbox = None
        if src_us and src_vs:
            src_bbox = (min(src_us), max(src_us), min(src_vs), max(src_vs))
        domain_bboxes = []
        for df in domain_for_planar:
            if df.index == source_index:
                continue
            us, vs = [], []
            try:
                for p in tmp._face_sample_points(df, part=part):
                    rel = tmp._vsub(p, origin0)
                    if abs(tmp._vdot(rel, normal0)) > 1e-2:
                        us = []
                        break
                    us.append(tmp._vdot(rel, u_dir0))
                    vs.append(tmp._vdot(rel, v_dir0))
            except Exception:
                us = []
            if us and vs:
                domain_bboxes.append(
                    (min(us), max(us), min(vs), max(vs)))
        result = tmp.partition_planar_transition(
            part=part,
            face=face,
            finer_edge=edge,
            depth=depth,
            layers=n,
            seeds=seeds,
            orientation=orientation,
            ratio=ratio_int,
            coarse_seeds=cs,
            coarser_edge=_planar_coarse_edge(classic_planar_coarse),
            ratio_sequence=ratioSequence or None,
            stage_widths=stageWidths or None,
            hybrid_pattern=hybridPattern or '',
            hybrid_regular_extension=(bool(cs) and bool(meshRegular2d)
                                      and bool(domain_for_planar)),
            regular_faces=([f for f in domain_for_planar
                            if f.index != source_index]
                           if len(part.cells) == 0 else None)
        )
        result['domain_faces'] = domain_for_planar
        result['mesh_regular'] = bool(meshRegular2d) and bool(domain_for_planar)
        result['transition_cell_indices'] = transition_cell_indices
        if len(part.cells) == 0:
            children = _refind_planar_children(
                part, result, src_bbox, origin0, u_dir0, v_dir0, normal0,
                features_before)
            if not children:
                raise ValueError(
                    'TM-SCOPE-REFIND: cannot identify newly partitioned faces '
                    'inside the source-face domain.')
            extra_regular = _refind_planar_domain_children(
                part, domain_bboxes, origin0, u_dir0, v_dir0, normal0)
            extra_regular -= set(children)
            for point in regular_points:
                try:
                    # Normalize Face/FaceArray before reading the scalar index.
                    regular_face = _single_object(part.faces.findAt((point,)))
                    extra_regular.add(int(regular_face.index))
                except Exception:
                    pass
            return _complete_classic_2d(part, result, children, extra_regular,
                                        include_source_regular, action, element2d)
    elif mode == 'CYLINDER':
        direction = orientation if orientation in ('AXIAL', 'CIRCUMFERENTIAL') else 'AXIAL'
        result = tmp.partition_cylinder_transition(
            part=part,
            cylindrical_face=face,
            finer_edge=edge,
            depth=depth,
            layers=n,
            seeds=seeds,
            direction=direction,
            ratio=ratio_int
        )
    else:
        raise ValueError("Unsupported mode: %s" % mode)

    mesh_prep = None
    validation = None

    # 统一动作层级：action 为主要来源，兼容旧 generateMesh/validate 参数
    planar_action = action if action in _CYL_ACTIONS else 'PARTITION_SEED_MESH'
    if planar_action == 'PARTITION_ONLY':
        # 仅分区：不布种不划网
        result['action'] = planar_action
        print("Transition Mesh GUI completed (PARTITION_ONLY): %s" % result)
        return result

    # PARTITION_AND_SEED 或 PARTITION_SEED_MESH：执行布种准备
    is_2d = (len(part.cells) == 0)
    if mode == 'PLANAR':
        if is_2d:
            # Pure 2D: QUAD control + explicit CPS4R/CPE4R on the target
            # scope (transition face + optional regular domain faces).
            mesh_prep = _prepare_planar_2d_mesh(
                part, result, element2d,
                target_faces=([face] if face is not None else None),
                domain_faces=result.get('domain_faces') or None)
        else:
            mesh_prep = tmp.prepare_planar_transition_mesh(
                part,
                transition_info=result,
                fine_spacing=result.get("fine_spacing"),
                transition_cells=[part.cells[i] for i in
                                  result.get('transition_cell_indices', [])]
            )
        print("Transition Mesh GUI: local seed preparation: %s" % mesh_prep)
    else:
        mesh_prep = tmp.prepare_cylinder_transition_mesh(
            part,
            transition_info=result
        )
        print("Transition Mesh GUI: cylinder local seed preparation: %s" % mesh_prep)

    if planar_action == 'PARTITION_AND_SEED':
        # 分区并布种：不生成最终网格
        result['mesh_prep'] = mesh_prep
        result['action'] = planar_action
        print("Transition Mesh GUI completed (PARTITION_AND_SEED): %s" % result)
        return result

    # PARTITION_SEED_MESH：生成并保留最终网格For classic 2D PLANAR,
    # skip only when the TARGET face already has mesh -- not when any other face on the part does (spec 6.2).
    had_mesh = False
    try:
        if is_2d and mode == 'PLANAR' and face is not None:
            try:
                had_mesh = bool(face.getElements())
            except Exception:
                had_mesh = len(part.elements) > 0
        elif not is_2d and mode == 'PLANAR':
            # Mesh means mesh the complete solid.
            had_mesh = False
        else:
            had_mesh = len(part.elements) > 0
    except Exception:
        pass

    if not had_mesh:
        # 无已有网格：直接生成最终网格
        try:
            part.generateMesh()
        except Exception as exc:
            raise RuntimeError(
                "Partition and seed succeeded, but final mesh failed: %s" % exc)
        elem_count = len(part.elements)
        node_count = len(part.nodes)
        if elem_count == 0 and is_2d and mode == 'PLANAR':
            # A structured control may be accepted but leave the transition
            # templates empty. Retry the same quadrilateral faces with Sweep.
            from abaqusConstants import QUAD, SWEEP, ADVANCING_FRONT
            part.setMeshControls(regions=part.faces, elemShape=QUAD,
                                 technique=SWEEP, algorithm=ADVANCING_FRONT,
                                 allowMapped=False)
            part.generateMesh(regions=part.faces)
            elem_count = len(part.elements)
            node_count = len(part.nodes)
            mesh_prep['mesh_control'] = 'QUAD+SWEEP+ADVANCING_FRONT'
        if elem_count <= 0:
            raise RuntimeError(
                "Mesh produced 0 elements (controls/seeds/topology mismatch).")
        validation = {'status': 'MESH_GENERATED', 'elements': elem_count,
                      'nodes': node_count, 'mesh_deleted': False}
        if not is_2d and mode == 'PLANAR':
            # Solid part: the 2D quad audit fields do not apply.
            # Report a 3D schema instead so _audit_execution judges hex-ness,
            solid_audit = _planar_solid_audit(part, result)
            validation['solid_audit'] = solid_audit
            result['solid_audit'] = solid_audit
        print("Transition Mesh GUI: final mesh generated: %d elements, %d nodes"
              % (elem_count, node_count))
        # Classic pure-2D PLANAR: auto-mesh regular domain faces (spec 6.2).
        if (is_2d and mode == 'PLANAR'
                and result.get('mesh_regular')
                and result.get('domain_faces')):
            try:
                reg = _mesh_planar_regular_regions(
                    part, result, result.get('domain_faces'),
                    element2d=element2d)
                result['regular_regions'] = reg
                if reg and reg.get('regular_elements'):
                    print('Planar regular regions: %s' % reg)
            except Exception as rexc:
                result['regular_regions'] = {'error': repr(rexc)}
                print('Planar regular regions failed: %s' % rexc)
    else:
        # 已有网格：报告现有状态，不删除不覆盖
        elem_count = len(part.elements)
        node_count = len(part.nodes)
        validation = {'status': 'EXISTING_MESH_PRESERVED',
                      'elements': elem_count, 'nodes': node_count,
                      'mesh_deleted': False}
        print("Transition Mesh GUI: existing mesh preserved: %d elements"
              % elem_count)

    result['mesh_prep'] = mesh_prep
    result['mesh_validation'] = validation
    result['action'] = planar_action
    result['final_elements'] = len(part.elements)
    result['final_nodes'] = len(part.nodes)
    print("Transition Mesh GUI completed: %s" % result)
    return result


def _refind_planar_children(part, result, src_bbox, origin, u_dir, v_dir,
                            normal, features_before, tol=1e-3):
    """2D planar partitions do not expose getFeatureFaces."""
    children = set()
    if src_bbox is None:
        # Fallback: try partition feature names, then any new face.
        for feature in (result.get('partition_features')
                        or (set(part.features.keys()) - features_before)):
            try:
                faces = part.getFeatureFaces(name=feature)
            except Exception:
                faces = None
            for f in (faces or ()):
                children.add(f.index)
        if children:
            return children
        # Last resort: all faces coplanar with the source frame.
        for i, f in enumerate(part.faces):
            try:
                p = tuple(f.pointOn[0])
                rel = tmp._vsub(p, origin)
                if abs(tmp._vdot(rel, normal)) <= tol:
                    children.add(i)
            except Exception:
                continue
        return children

    umin, umax, vmin, vmax = src_bbox
    # Expand slightly so edge-adjacent children are not clipped.
    pad = 1e-2 * max(1.0, umax - umin, vmax - vmin)
    umin -= pad
    umax += pad
    vmin -= pad
    vmax += pad
    print('TM refind: src_bbox uv=(%.4f,%.4f,%.4f,%.4f) faces=%d'
          % (umin, umax, vmin, vmax, len(part.faces)))
    for i in range(len(part.faces)):
        f = part.faces[i]
        samples = []
        try:
            samples.append(tuple(f.pointOn[0]))
        except Exception:
            pass
        try:
            for vid in f.getVertices():
                samples.append(tuple(part.vertices[vid].pointOn[0]))
        except Exception:
            pass
        if not samples:
            continue
        inside = True
        for p in samples:
            rel = tmp._vsub(p, origin)
            if abs(tmp._vdot(rel, normal)) > 1e-2:
                inside = False
                break
            u = tmp._vdot(rel, u_dir)
            v = tmp._vdot(rel, v_dir)
            if u < umin - tol or u > umax + tol or v < vmin - tol or v > vmax + tol:
                inside = False
                break
        if inside:
            children.add(i)
    print('TM refind: children=%s' % (sorted(children),))
    return children


def _refind_planar_domain_children(part, bboxes, origin, u_dir, v_dir,
                                   normal, tol=1e-3):
    """All children of the pre-partition Full-domain faces, found geometrically."""
    children = set()
    if not bboxes:
        return children
    for i in range(len(part.faces)):
        f = part.faces[i]
        samples = []
        try:
            samples.append(tuple(f.pointOn[0]))
        except Exception:
            pass
        try:
            for vid in f.getVertices():
                samples.append(tuple(part.vertices[vid].pointOn[0]))
        except Exception:
            pass
        if not samples:
            continue
        local = []
        coplanar = True
        for p in samples:
            rel = tmp._vsub(p, origin)
            if abs(tmp._vdot(rel, normal)) > 1e-2:
                coplanar = False
                break
            local.append((tmp._vdot(rel, u_dir),
                          tmp._vdot(rel, v_dir)))
        if not coplanar:
            continue
        for umin, umax, vmin, vmax in bboxes:
            if all(umin - tol <= u <= umax + tol
                   and vmin - tol <= v <= vmax + tol for u, v in local):
                children.add(i)
                break
    print('TM refind domain: children=%s' % (sorted(children),))
    return children


def _complete_classic_2d(part, result, children, extra_regular,
                         include_source_regular, action, element2d):
    """Mesh only live descendants of the selected face and explicit domain."""
    from abaqusConstants import FIXED
    import transition_mesh_plugin_planar2d as p2d
    band = []
    for fi in sorted(children):
        if all(tmp._edge_in_transition_band(part, part.edges[ei], result)
               for ei in part.faces[fi].getEdges()):
            band.append(fi)
    if not band:
        raise ValueError('No transition faces found inside the requested band.')
    regular = set(extra_regular)
    if include_source_regular:
        regular.update(children - set(band))
    result.pop('domain_faces', None)
    result.update(scope_faces=band, regular_faces=sorted(regular), action=action)
    if action == 'PARTITION_ONLY':
        return result
    tm_runtime.stage('SEED', faces=len(band), regular_faces=len(regular))
    band_edges = set(ei for fi in band for ei in part.faces[fi].getEdges())
    for ei in sorted(band_edges):
        part.seedEdgeByNumber(edges=part.edges[ei:ei+1], number=1, constraint=FIXED)
    spacing = result.get('coarse_spacing') or result['edge_length'] / result['coarse']
    for ei in sorted(set(e for fi in regular for e in part.faces[fi].getEdges()) - band_edges):
        edge = part.edges[ei]
        count = max(1, int(round(edge.getSize(printResults=False) / spacing)))
        part.seedEdgeByNumber(edges=part.edges[ei:ei+1], number=count, constraint=FIXED)
    for indices in (band, sorted(regular)):
        if indices:
            ctrl = p2d._set_quad_structured(part, [part.faces[i] for i in indices], element2d)
            if not ctrl:
                raise ValueError('Cannot assign quadrilateral mesh controls to the selected region.')
    result['mesh_prep'] = {'seeded_edges': len(band_edges), 'element_type': element2d,
                           'scope_faces': len(band), 'is_2d': True}
    if action == 'PARTITION_AND_SEED':
        return result
    tm_runtime.stage('MESH', mode='PLANAR')
    ctrl, err, audit = p2d._mesh_patch_plan(part, band, element2d)
    _require_quad_audit(part, band, audit, element2d)
    result['transition_audit'] = audit
    if regular:
        rctrl, rerr, raudit = p2d._mesh_patch_plan(part, sorted(regular), element2d)
        _require_quad_audit(part, regular, raudit, element2d)
        result['regular_regions'] = dict(raudit, regular_faces=len(regular),
                                         regular_elements=raudit['elements'],
                                         element_type=element2d, mesh_control=rctrl)
    result.update(final_elements=len(part.elements), final_nodes=len(part.nodes))
    return result


def _require_quad_audit(part, indices, audit, element2d):
    for key in ('tri', 'unmeshed_faces', 'duplicate_elements', 'negative_area',
                'zero_area', 'bowtie', 'hanging_nodes', 'jacobian_bad'):
        if key not in audit or audit[key]:
            raise ValueError('Mesh audit failed or incomplete: %s=%s' % (key, audit.get(key)))
    if audit.get('elements', 0) <= 0:
        raise ValueError('The selected region contains no elements.')
    want = str(element2d or '').strip().upper()
    if want in ('', 'KEEP', 'NONE', 'DEFAULT'):
        # GUI default: the part keeps whatever element type it already has.
        return
    for fi in indices:
        for element in part.faces[fi].getElements():
            if str(element.type) != want:
                raise ValueError('Requested %s, generated %s.' % (want, element.type))


def _planar_solid_audit(part, transition_info=None):
    """插件只做面划分，块切分与扫掠由用户提前做好，所以这里只核三件事：单元非空、类型集合里没有四面体/楔形、未划网区域与不相容界面为零。"""
    types = sorted(set(str(e.type) for e in part.elements))
    audit = {'element_types': types,
             'elements': len(part.elements),
             'nodes': len(part.nodes),
             'cells': len(part.cells),
             'forbidden_types': [t for t in types
                                 if any(b in t.upper() for b in
                                        ('TET', 'WEDGE', 'C3D4', 'C3D6',
                                          'C3D10', 'C3D8I'))]}
    try:
        un = part.getUnmeshedRegions()
        audit['unmeshed_regions'] = 0 if un is None else len(un)
    except BaseException as exc:
        # 查询失败不等于有问题：记 None 让上层降级为 WARN（与圆筒车道一致）
        audit['unmeshed_regions'] = None
        audit['unmeshed_query_error'] = str(exc)[:80]
    try:
        inc = part.getIncompatibleMeshInterfaces(cells=part.cells)
        audit['incompatible_interfaces'] = len(inc) if inc else 0
    except BaseException as exc:
        audit['incompatible_interfaces'] = None
        audit['incompatible_query_error'] = str(exc)[:80]
    frame = transition_info if isinstance(transition_info, dict) else None
    normal = frame.get('normal') if frame else None
    if normal:
        vals = set()
        try:
            for node in part.nodes:
                rel = tmp._vsub(tuple(node.coordinates), tuple(frame['origin']))
                vals.add(round(tmp._vdot(rel, normal), 6))
        except BaseException:
            vals = set()
        if vals:
            audit['layers_along_normal'] = len(vals) - 1
    return audit


def _audit_execution(result, action):
    """Hard failures raise so ModelTransaction rolls back."""
    hard = []
    warn = []
    raw = str(result.get('status') or '').upper()
    if raw in ('FAIL',):
        hard.append(result.get('status_reason') or 'execution reported FAIL')
    # Interface compatibility (four-side / annular)
    compat = str(result.get('compatibility') or '').upper()
    if compat in ('IFACE_FAIL', 'IFACE_WARN', 'INTERFACE_MISMATCH'):
        hard.append('interface mismatch: %s' % compat)
    if result.get('gap_regions'):
        hard.append('uncontrolled gap regions: %s' % result['gap_regions'])
    # Mesh quality keys (Generate Mesh only)
    is_mesh = action in ('PARTITION_SEED_MESH', 'GENERATE_MESH')
    if is_mesh:
        mode = str(result.get('mode') or '').upper()
        solid = result.get('solid_audit') or {}
        if solid or 'CYLINDER' in mode or 'RADIAL' in mode \
                or 'AXIAL' in mode or 'cell_elements' in result:
            # Cylinder lanes run their own hex audit inside the kernel (_cyl_audit_generated_mesh raises on
            # 0-element cells, wrong type or incompatible interfaces) and report a 3D schema:
            elems = (result.get('elements')
                     or result.get('final_elements') or 0)
            cell_counts = result.get('cell_elements') or {}
            if not elems and not cell_counts:
                hard.append('Generate Mesh produced 0 elements')
            caudit = result.get('mesh_audit') or {}
            if isinstance(caudit, dict):
                if caudit.get('unmeshed_in_scope'):
                    hard.append('unmeshed cells in scope: %s'
                                % caudit.get('unmeshed_in_scope'))
                if (caudit.get('incompatible_interfaces') or 0) > 0:
                    hard.append('incompatible mesh interfaces: %s'
                                % caudit.get('incompatible_interfaces'))
            for index, band in enumerate(result.get('bands') or [result]):
                quality = (band.get('mesh_audit') or {}).get('quality') or {}
                if quality.get('failedElements') or quality.get('naElements'):
                    hard.append('Band %d mesh analysis checks failed: %s'
                                % (index + 1, quality))
                if quality.get('warningElements'):
                    warn.append('Band %d mesh analysis check: %d elements have warnings.'
                                % (index + 1, quality['warningElements']))
            if solid:
                # 经典 PLANAR 作用在三维实体上：Apply 走的是整件 generateMesh，
                # 所以核六面性、覆盖与界面相容，不再索要 2D 的 tri/bowtie 字段。
                if not solid.get('elements'):
                    hard.append('3D planar mesh produced 0 elements')
                if solid.get('forbidden_types'):
                    hard.append('Non-hex element types: %s' % solid['forbidden_types'])
                for key, label in (('unmeshed_regions', 'Unmeshed regions present'),
                                   ('incompatible_interfaces',
                                    'Incompatible mesh interfaces present')):
                    val = solid.get(key)
                    if val is None:
                        warn.append('%s check incomplete (query error)' % label)
                    elif val:
                        hard.append('%s: %s' % (label, val))
        else:
            audit = (result.get('transition_audit')
                     or result.get('mesh_audit') or {})
            for key in ('tri', 'unmeshed_faces', 'duplicate_elements',
                        'negative_area', 'zero_area', 'bowtie',
                        'hanging_nodes', 'jacobian_bad'):
                val = audit.get(key, result.get(key))
                if val is None:
                    hard.append('missing audit field: %s' % key)
                elif val:
                    hard.append('mesh quality %s=%s' % (key, val))
            # Top-level elements check
            elems = result.get('elements') or result.get('final_elements') or 0
            if not elems and not result.get('band_faces'):
                pass  # PARTITION_AND_SEED may have 0 elements
            elif is_mesh and not elems:
                hard.append('Generate Mesh produced 0 elements')
    # Non-structural warnings
    for w in (result.get('warnings') or []):
        warn.append(str(w))
    if result.get('external_straight_warn') and not (
            result.get('external_straight') or {}).get('ok', True):
        warn.append('external straightness warning')
    status = 'PASS' if not hard else 'FAIL'
    if not hard and warn:
        status = 'WARN'
    return status, hard, warn



    for key in ('tri', 'unmeshed_faces', 'duplicate_elements', 'negative_area',
                'zero_area', 'bowtie', 'hanging_nodes', 'jacobian_bad'):
        if key not in audit or audit[key]:
            raise ValueError('Mesh audit failed or incomplete: %s=%s' % (key, audit.get(key)))
    if audit.get('elements', 0) <= 0:
        raise ValueError('The selected region contains no elements.')
    for fi in indices:
        for element in part.faces[fi].getElements():
            if str(element.type) != element2d:
                raise ValueError('Requested %s, generated %s.' % (element2d, element.type))


def run_transition_mesh_batch(model_name, regions, action='PARTITION_SEED_MESH',
                              request_id=None, result_file=''):
    """All regions are pre-validated before any mutation."""
    import transition_mesh_regions as tm_reg
    request_id = request_id or uuid.uuid4().hex
    if model_name not in mdb.models:
        raise ValueError('Model not found: %s' % model_name)
    model = mdb.models[model_name]

    # Pre-flight: validate batch.
    errors = tm_reg.validate_region_batch(regions)
    if errors:
        raise ValueError('Batch validation failed: %s' % '; '.join(errors))

    # Resolve all region signatures against the current Part.
    part = None
    for p in model.parts.values():
        part = p
        break
    if part is None:
        raise ValueError('Model %s has no Part.' % model_name)

    resolved, unresolved = tm_reg.build_region_plans(part, regions)
    if unresolved:
        ids = [r.get('region_id', '?') for r in unresolved]
        raise ValueError(
            'Cannot re-locate region(s) %s after topology change. '
            'Repick the affected regions.' % ids)

    results = []
    with tm_runtime.ModelTransaction(mdb, model_name, request_id):
        tm_runtime.stage('BATCH_START', regions=len(resolved), action=action)
        for i, (region, entities) in enumerate(resolved):
            rid = region.get('region_id', 'region_%d' % i)
            params = region.get('params') or {}
            tm_runtime.stage('REGION_START', region_id=rid, index=i)
            try:
                # Build kwargs for the single-region entry point.
                kw = dict(params)
                kw['modelName'] = model_name
                kw['partName'] = part.name
                kw['action'] = action
                kw['checkOnly'] = False
                # Re-resolve: previous region's partition may invalidate handles.
                entities = {}
                for kind in ('face', 'fine_edge', 'coarse_edge'):
                    obj = tm_reg.resolve_selection(part, region, kind)
                    if obj is not None:
                        entities[kind] = obj
                if 'face' in entities:
                    kw['pickedFace'] = entities['face']
                if 'fine_edge' in entities:
                    kw['pickedEdge'] = entities['fine_edge']
                    kw['pickedFineEdges'] = entities['fine_edge']
                if 'coarse_edge' in entities:
                    kw['pickedCoarseEdges'] = entities['coarse_edge']
                output = _run_transition_mesh_gui(**kw)
                status, hard, warn = _audit_execution(output, action)
                if hard:
                    raise ValueError(
                        'Region %s audit failed: %s' % (rid, '; '.join(hard)))
                output['region_id'] = rid
                output['status'] = status
                results.append(output)
                tm_runtime.stage('REGION_DONE', region_id=rid, status=status)
            except Exception as exc:
                tm_runtime.stage('REGION_FAIL', region_id=rid, error=str(exc))
                raise ValueError(
                    'Region %s failed: %s. Entire batch rolled back.'
                    % (rid, exc))
        tm_runtime.stage('BATCH_DONE', regions=len(results))

    batch_result = {
        'mode': 'BATCH',
        'regions_executed': len(results),
        'results': results,
        'operation_id': request_id,
        'status': 'PASS',
    }
    if result_file:
        tm_runtime.publish(result_file, request_id, batch_result)
    return batch_result


def create_region_from_picks(part, face=None, fine_edge=None, coarse_edge=None,
                             **params):
    """Helper: build a region dict from current GUI picks."""
    import transition_mesh_regions as tm_reg
    return tm_reg.create_region(
        part, face=face, fine_edge=fine_edge, coarse_edge=coarse_edge,
        **params)

def _log_kernel_exception(request_id, exc):
    try:
        import traceback
        import tempfile
        import time
        log_dir = os.path.join(tempfile.gettempdir(), 'TransitionMesh', 'logs')
        if not os.path.isdir(log_dir):
            os.makedirs(log_dir)
        with open(os.path.join(log_dir, 'kernel_errors.log'), 'a',
                  encoding='utf-8') as stream:
            stream.write('[%s] request=%s %s\n%s\n' % (
                time.strftime('%Y-%m-%d %H:%M:%S'), request_id,
                type(exc).__name__, traceback.format_exc()))
    except Exception:
        pass


def dispatch_transition_mesh_command(command, request_id='', result_file='',
                                     feedback=False, namespace=None):
    """Guard serialized picks/loader verification as well as the kernel call."""
    request_id = request_id or uuid.uuid4().hex
    try:
        exec(command, namespace if namespace is not None else globals())
    except Exception as exc:
        # Execution failures inside the public boundary already published a
        # richer result (including rollback/suggestions); do not replace it.
        result = None
        try:
            if result_file and os.path.isfile(result_file):
                with open(result_file, 'r') as stream:
                    result = json.load(stream)
                if (not isinstance(result, dict)
                        or str(result.get('request_id', '')) != str(request_id)):
                    result = None
        except Exception:
            result = None
        if result is None:
            _log_kernel_exception(request_id, exc)
            summary = 'Command dispatch failed: %s: %s' % (type(exc).__name__, exc)
            repick = isinstance(exc, (IndexError, AttributeError))
            if repick:
                summary += ' Repick the active geometry from the current Part and retry.'
            result = tm_runtime.publish(result_file, request_id, {
                'status': 'FAIL', 'summary': summary, 'suggestions': {},
                'checks_pending': [], 'repick_required': repick,
            })
        if feedback:
            return result
        raise


def run_transition_mesh_gui(*args, **kwargs):
    """Public request boundary: real Check feedback and complete-model rollback."""
    request_id = kwargs.pop('requestId', '') or uuid.uuid4().hex
    result_file = kwargs.pop('resultFile', '')
    params = dict(kwargs)
    checking = bool(params.get('checkOnly')) or params.get('action') == 'CHECK_ONLY'
    pre_mesh = str(params.get('action', '')).upper() == 'PRE_MESH'
    try:
        binding = inspect.signature(_run_transition_mesh_gui).bind(*args, **kwargs)
        binding.apply_defaults()
        params = dict(binding.arguments)
        checking = bool(params['checkOnly']) or params['action'] == 'CHECK_ONLY'
        pre_mesh = str(params['action']).upper() == 'PRE_MESH'
        if params['action'] not in _ALL_ACTIONS:
            raise ValueError('Choose Partition + Seed or Generate Mesh.')
        ttype = _canonical_transition_type(params.get('transitionType', ''))
        picks = _route_pick_values(ttype, params)
        inferred = _infer_model_part(*picks)
        if inferred is not None:
            params['modelName'], params['partName'] = inferred
        check_params = dict(params, checkOnly=True)
        check_result = _run_transition_mesh_gui(**check_params)
        if checking:
            output = dict(check_result)
            raw = str(output.get('status', 'PASS')).upper()
            # Normalize legacy aliases. A successful input/geometry check is PASS even though the
            # generated-mesh quality can only be judged by Apply/Pre-mesh:
            if raw in ('CHECK_OK', 'OK', 'SUCCESS'):
                raw = 'PASS'
            if raw not in ('PASS', 'WARN', 'FAIL'):
                raw = 'FAIL'
            output['status'] = raw
            summary = output.get('summary', 'Input checks completed.')
            if 'CHECK OK' in summary:
                summary = summary.replace('CHECK OK', 'Input checks passed')
            output['summary'] = summary
            output.setdefault('checks_pending',
                              ['generated mesh quality (use Pre-mesh)'])
            output.setdefault('suggestions', {})
            print('CHECK %s: %s' % (raw, output.get('summary', '')))
            return tm_runtime.publish(result_file, request_id, output)
        params['checkOnly'] = False
        pre_mesh = str(params['action']).upper() == 'PRE_MESH'
        if pre_mesh:
            params['action'] = 'PARTITION_SEED_MESH'
        try:
            with tm_runtime.ModelTransaction(mdb, params['modelName'], request_id):
                tm_runtime.stage('EXECUTE_START', mode=params['transitionType'],
                                 action=params['action'])
                output = _run_transition_mesh_gui(**params)
                # In-transaction audit: hard failures roll back.
                status, hard, warn = _audit_execution(output, params['action'])
                if hard:
                    tm_runtime.stage('AUDIT_FAIL', failures=hard)
                    raise ValueError(
                        'Execution audit failed: %s' % '; '.join(hard))
                tm_runtime.stage('AUDIT_COMPLETE', status=status)
                output['status'] = status
                output['audit_hard_failures'] = []
                output['warnings'] = list(output.get('warnings') or []) + warn
                if pre_mesh:
                    # Trial proved meshability: undo it on purpose.
                    raise _PreMeshRollback()
        except _PreMeshRollback:
            raw = str(output.get('status') or 'PASS').upper()
            warnings = list(output.get('warnings') or [])
            summary = ('Pre-mesh passed: the region meshed cleanly and the '
                       'trial was rolled back; the model is unchanged.')
            if raw == 'WARN' and warnings:
                summary = ('Pre-mesh finished with warnings; the trial was '
                           'rolled back. ' + ' '.join(warnings)[:200])
            payload = {
                'status': raw, 'summary': summary,
                'repick_required': False, 'warnings': warnings,
                'pre_mesh': True, 'model_unchanged': True,
                'metrics': dict((k, output.get(k)) for k in (
                    'elements', 'tri', 'unmeshed_faces', 'jacobian_bad',
                    'aspect_ratio_max', 'interface_counts',
                    'external_straight', 'stage_counts_x', 'stage_counts')
                    if k in output),
            }
            print('PRE-MESH %s: %s' % (raw, summary))
            if result_file:
                tm_runtime.publish(result_file, request_id, payload)
            return payload
        output['operation_id'] = request_id
        raw = str(output.get('status') or 'PASS').upper()
        if raw not in ('PASS', 'WARN', 'FAIL'):
            raw = 'WARN'
        warnings = list(output.get('warnings') or [])
        summary = 'Operation completed. Repick before another operation.'
        if raw == 'WARN':
            summary = 'Operation completed with warnings. Repick before another operation.'
            if warnings:
                summary = summary + ' ' + ' '.join(warnings)[:200]
        output['status'] = raw
        if result_file:
            tm_runtime.publish(result_file, request_id,
                               {'status': raw, 'summary': summary,
                                'repick_required': True,
                                'warnings': warnings,
                                'result': output})
        return output
    except Exception as exc:
        _log_kernel_exception(request_id, exc)
        suggestions = {}
        # Count candidates from the SAME planner Check/Apply use.
        try:
            ttype = _canonical_transition_type(params.get('transitionType', ''))
            quad = (ttype == 'PLANAR_QUAD_FOUR')
            ratio_i = _parse_ratio(params.get('ratio', '2:1'))
            suggestion_stages = params.get('stages', 1)
            if ttype == 'PLANAR' and int(suggestion_stages) == 1:
                suggestion_stages = params.get('layers', 1)
            cand = tm_plan.suggest_counts(
                params.get('seeds', 0), suggestion_stages, ratio_i,
                params.get('ratioSequence', ''), quad=quad)
            if cand:
                suggestions = {'seeds': cand[0], 'candidates': cand,
                               'geometry_checked': False}
            if quad:
                ny = int(params.get('fineSeedsY', 0) or 0)
                if ny > 0:
                    cand_y = tm_plan.suggest_counts(
                        ny, params.get('stages', 1), ratio_i,
                        params.get('ratioSequence', ''), quad=True)
                    if cand_y:
                        suggestions['fineSeedsY'] = cand_y[0]
                        suggestions['candidatesY'] = cand_y
        except Exception:
            pass
        output = {
            'status': 'FAIL',
            'summary': '%s: %s' % (type(exc).__name__, exc),
            'suggestions': suggestions,
            'checks_pending': [], 'repick_required': not checking,
        }
        if str(params.get('action', '')).upper() == 'PARTITION_SEED_MESH' \
                and pre_mesh:
            output['summary'] = ('Pre-mesh failed (model rolled back, '
                                 'unchanged): %s' % exc)
            output['model_unchanged'] = True
            output['pre_mesh'] = True
        tm_runtime.publish(result_file, request_id, output)
        if checking:
            # Check path: return structured result, don't raise.
            return output
        raise


def _mesh_planar_regular_regions(part, transition_info, domain_faces,
                                 element2d='CPS4R'):
    """Shares the same contract as PLANAR_QUAD_FOUR / PLANAR_ANNULAR regular regions: QUAD+STRUCTURED,"""
    from abaqusConstants import (QUAD, STRUCTURED, SWEEP, ADVANCING_FRONT,
                                 CPS4R, CPE4R, STANDARD, FIXED, TWO_D_PLANAR)
    from mesh import ElemType

    if not domain_faces:
        return None
    fine_spacing = (transition_info or {}).get('fine_spacing')
    ratio = int((transition_info or {}).get('ratio') or 2)
    if not fine_spacing:
        return {'skipped': 'no_fine_spacing'}
    coarse_h = float(fine_spacing) * float(ratio)

    # Classify domain faces: skip those already inside the transition band.
    frame = transition_info if isinstance(transition_info, dict) else None
    has_frame = bool(frame and frame.get('origin') is not None
                     and frame.get('u_dir') is not None
                     and frame.get('v_dir') is not None)
    regular_idx = []
    seen = set()
    # Fallback: if frame-based classification yields nothing, treat every
    # face that still has a long edge as a regular-region candidate.
    for f in domain_faces:
        try:
            fi = f.index if hasattr(f, 'index') else int(f)
        except Exception:
            continue
        if fi in seen or fi >= len(part.faces):
            continue
        seen.add(fi)
        has_long = False
        try:
            for e in part.faces[fi].getEdges():
                ei = e if isinstance(e, int) else getattr(e, 'index', None)
                if ei is None:
                    continue
                edge = part.edges[ei]
                try:
                    vs = edge.getVertices()
                    if len(vs) != 2:
                        continue
                    p0 = part.vertices[vs[0]].pointOn[0]
                    p1 = part.vertices[vs[1]].pointOn[0]
                    length = ((p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2) ** 0.5
                except Exception:
                    continue
                if length > 1.5 * coarse_h:
                    has_long = True
                    break
        except Exception:
            continue
        if has_long:
            regular_idx.append(fi)
    if not regular_idx:
        return {'regular_faces': 0}

    # Seed each regular edge by length / coarse_h (min 1 FIXED).
    seeded = 0
    seed_err = None
    seen_e = set()
    for fi in regular_idx:
        for e in part.faces[fi].getEdges():
            ei = e if isinstance(e, int) else getattr(e, 'index', None)
            if ei is None or ei in seen_e:
                continue
            seen_e.add(ei)
            edge = part.edges[ei]
            try:
                vs = edge.getVertices()
                if len(vs) != 2:
                    continue
                p0 = part.vertices[vs[0]].pointOn[0]
                p1 = part.vertices[vs[1]].pointOn[0]
                length = ((p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2) ** 0.5
                nseg = max(1, int(round(length / max(coarse_h, 1e-12))))
                part.seedEdgeByNumber(edges=(edge,), number=nseg,
                                      constraint=FIXED)
                seeded += 1
            except Exception as sexc:
                seed_err = str(sexc)

    faces = [part.faces[i] for i in regular_idx]
    ctrl = None
    try:
        part.setMeshControls(regions=tuple(faces), elemShape=QUAD,
                             technique=STRUCTURED)
        ctrl = 'QUAD+STRUCTURED'
    except Exception:
        try:
            part.setMeshControls(regions=tuple(faces), elemShape=QUAD,
                                 technique=SWEEP, algorithm=ADVANCING_FRONT)
            ctrl = 'QUAD+SWEEP+ADVANCING_FRONT'
        except Exception as exc2:
            ctrl = 'FAILED: %s' % str(exc2)[:60]

    elem_name = str(element2d or '').strip().upper()
    elem_label = None
    if elem_name in ('', 'KEEP', 'NONE', 'DEFAULT'):
        # GUI default: never overwrite the part's element type.
        elem_label = 'KEEP(part default)'
    else:
        try:
            elem_code = CPS4R if elem_name == 'CPS4R' else CPE4R
            set_name = '__TM_REG2D__'
            for _try in range(3):
                try:
                    del part.sets[set_name]
                except Exception:
                    pass
                try:
                    part.Set(faces=tuple(faces), name=set_name)
                    break
                except Exception:
                    set_name = '__TM_REG2D_%d__' % _try
            try:
                part.setElementType(
                    regions=part.sets[set_name],
                    elemTypes=(ElemType(elemCode=elem_code,
                                        elemLibrary=STANDARD),))
                elem_label = str(elem_code)
            finally:
                try:
                    del part.sets[set_name]
                except Exception:
                    pass
        except Exception as exc:
            elem_label = 'WARN:%s' % str(exc)[:40]

    meshed = 0
    tri = 0
    try:
        seq = faces
        part.generateMesh(regions=tuple(seq))
        for fi in regular_idx:
            try:
                els = part.faces[fi].getElements()
            except Exception:
                els = ()
            for el in (els or ()):
                meshed += 1
                try:
                    if len(el.getNodes()) <= 3:
                        tri += 1
                except Exception:
                    pass
    except Exception as mex:
        return {
            'regular_faces': len(regular_idx), 'seeded_edges': seeded,
            'seed_error': seed_err, 'mesh_control': ctrl,
            'element_type': elem_label, 'mesh_error': repr(mex),
        }

    return {
        'regular_faces': len(regular_idx),
        'seeded_edges': seeded,
        'seed_error': seed_err,
        'mesh_control': ctrl,
        'element_type': elem_label,
        'regular_elements': meshed,
        'regular_tri': tri,
        'coarse_h': round(coarse_h, 6),
    }


def _prepare_planar_2d_mesh(part, transition_info, element2d='',
                            target_faces=None, domain_faces=None):
    """When target_faces/domain_faces are given,"""
    from abaqusConstants import (QUAD, STRUCTURED, SWEEP, ADVANCING_FRONT,
                                 CPS4R, CPE4R, S4R, STANDARD, FIXED,
                                 TWO_D_PLANAR)
    from mesh import ElemType

    elem_name = str(element2d or '').strip().upper()
    if elem_name not in ('CPS4R', 'CPE4R', '', 'KEEP', 'NONE', 'DEFAULT'):
        raise ValueError('element2d must be CPS4R or CPE4R (or empty to '
                         'keep the part default), got %r' % (element2d,))

    seeded = 0
    fine_spacing = transition_info.get('fine_spacing') if transition_info else None
    frame = transition_info if isinstance(transition_info, dict) else None
    has_frame = bool(frame and frame.get('origin') is not None
                     and frame.get('u_dir') is not None
                     and frame.get('v_dir') is not None
                     and frame.get('normal') is not None)

    # Scope: transition face(s) + optional regular domain faces.
    scope_faces = []
    seen_fi = set()
    for src in (target_faces, domain_faces):
        if not src:
            continue
        for f in src:
            try:
                fi = f.index if hasattr(f, 'index') else int(f)
            except Exception:
                continue
            if fi not in seen_fi:
                seen_fi.add(fi)
                scope_faces.append(part.faces[fi])
    if not scope_faces:
        scope_faces = list(part.faces)

    seed_fail = 0
    seed_error = None
    for i in range(len(part.edges)):
        edge = part.edges[i]
        try:
            if has_frame:
                should_seed = tmp._edge_in_transition_band(part, edge, frame)
            elif fine_spacing is not None:
                vs = edge.getVertices()
                should_seed = True
                if len(vs) == 2:
                    p1 = part.vertices[vs[0]].pointOn[0]
                    p2 = part.vertices[vs[1]].pointOn[0]
                    length = ((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2
                              + (p2[2]-p1[2])**2) ** 0.5
                    if length > 3.1 * float(fine_spacing) + 1e-6:
                        should_seed = False
            else:
                should_seed = True
            if not should_seed:
                continue
            part.seedEdgeByNumber(edges=(edge,), number=1, constraint=FIXED)
            seeded += 1
        except Exception as sexc:
            seed_fail += 1
            if seed_error is None:
                seed_error = str(sexc)

    # Mesh control: QUAD + STRUCTURED on the scoped faces only.
    ctrl_label = None
    try:
        part.setMeshControls(regions=tuple(scope_faces), elemShape=QUAD,
                             technique=STRUCTURED)
        ctrl_label = 'QUAD+STRUCTURED'
    except Exception:
        try:
            part.setMeshControls(regions=tuple(scope_faces), elemShape=QUAD,
                                 technique=SWEEP, algorithm=ADVANCING_FRONT)
            ctrl_label = 'QUAD+SWEEP+ADVANCING_FRONT'
        except Exception:
            try:
                part.setMeshControls(regions=tuple(scope_faces), elemShape=QUAD)
                ctrl_label = 'QUAD(default)'
            except Exception as exc2:
                ctrl_label = 'FAILED: %s' % str(exc2)[:80]

    elem_label = None
    elem_code = None
    elem_error = None
    if elem_name in ('', 'KEEP', 'NONE', 'DEFAULT'):
        # GUI default: the part keeps its own element type (a 3D solid face
        # must never receive a 2D element type from this plugin).
        elem_label = 'KEEP(part default)'
    else:
        try:
            part_dim = part.space
            if part_dim == TWO_D_PLANAR:
                elem_code = CPS4R if elem_name == 'CPS4R' else CPE4R
            else:
                elem_code = S4R
            set_name = '__TM2D__'
            try:
                del part.sets[set_name]
            except Exception:
                pass
            part.Set(faces=tuple(scope_faces), name=set_name)
            try:
                elem_type = ElemType(elemCode=elem_code, elemLibrary=STANDARD)
                part.setElementType(regions=part.sets[set_name],
                                    elemTypes=(elem_type,))
                elem_label = str(elem_code)
            finally:
                try:
                    del part.sets[set_name]
                except Exception:
                    pass
        except Exception as exc:
            elem_error = str(exc)
            print("Warning: could not set 2D element type: %s" % exc)

    return {'seeded_edges': seeded, 'seed_failed': seed_fail,
            'seed_error': seed_error, 'element_type': elem_label,
            'elem_error': elem_error, 'mesh_control': ctrl_label,
            'is_2d': True, 'scope_faces': len(scope_faces)}
