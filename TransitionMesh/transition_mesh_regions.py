# -*- coding: utf-8 -*-
# ============================================================================
#  Transition Mesh
#  created by IY,China University of Petroleum (East China)
#  禁止商用 / Non-commercial use only
#  禁止商用：本软件仅供学习与研究使用，未经作者书面许可不得用于任何商业用途。
#  Non-commercial use only. Unauthorized commercial use is prohibited.
# ============================================================================

"""Multi-region batch management: a region is a self-contained transition task stored as a plain dict."""
from __future__ import print_function

import hashlib
import json
import uuid


class RegionError(ValueError):
    """Raised when a region spec is invalid or cannot be resolved."""


def make_region_id():
    return uuid.uuid4().hex[:12]


def build_region_signature(part, face=None, fine_edge=None, coarse_edge=None,
                           region_faces=None, tol=1e-4):
    """Stores local geometric fingerprints (centroid, area, edge lengths) rather than Abaqus indices,"""
    sig = {
        'part_name': getattr(part, 'name', ''),
        'tol': float(tol),
    }
    try:
        if face is not None:
            p = face.pointOn[0]
            sig['face_point'] = [round(float(x), 6) for x in p[:3]]
            try:
                sig['face_area'] = round(float(face.getSize(printResults=False)), 6)
            except Exception:
                pass
        if fine_edge is not None:
            p = fine_edge.pointOn[0]
            sig['fine_point'] = [round(float(x), 6) for x in p[:3]]
            try:
                sig['fine_len'] = round(float(fine_edge.getSize(printResults=False)), 6)
            except Exception:
                pass
        if coarse_edge is not None:
            # created by IY,China University of Petroleum (East China)
            p = coarse_edge.pointOn[0]
            sig['coarse_point'] = [round(float(x), 6) for x in p[:3]]
            try:
                sig['coarse_len'] = round(float(coarse_edge.getSize(printResults=False)), 6)
            except Exception:
                pass
        if region_faces:
            pts = []
            for f in region_faces:
                try:
                    pts.append([round(float(x), 6) for x in f.pointOn[0][:3]])
                except Exception:
                    pass
            sig['region_face_points'] = pts[:20]
    except Exception:
        pass
    # Stable hash of the signature for quick comparison.
    blob = json.dumps(sig, sort_keys=True, default=str)
    sig['hash'] = hashlib.sha256(blob.encode('utf-8')).hexdigest()[:16]
    return sig


def create_region(part, face=None, fine_edge=None, coarse_edge=None,
                  region_faces=None, fine_edges=None, coarse_edges=None,
                  target_cells=None, path_edges=None, domain_faces=None,
                  **params):
    """Create a region dict from current picks + parameters."""
    rid = make_region_id()
    sig = build_region_signature(
        part, face=face, fine_edge=fine_edge, coarse_edge=coarse_edge,
        region_faces=region_faces)
    region = {
        'region_id': rid,
        'signature': sig,
        'params': dict(params),
    }
    return region


def validate_region_batch(regions):
    """Returns a list of error strings (empty = OK)."""
    errors = []
    if not regions:
        errors.append('Empty region batch.')
        # created by IY,China University of Petroleum (East China)  |  禁止商用 / non-commercial use only
        return errors
    part_names = set()
    hashes = []
    for i, r in enumerate(regions):
        rid = r.get('region_id', '?')
        sig = r.get('signature') or {}
        pname = sig.get('part_name', '')
        if pname:
            part_names.add(pname)
        h = sig.get('hash', '')
        if h:
            hashes.append(h)
        params = r.get('params') or {}
        if not params.get('seeds') and not params.get('fine_seeds'):
            errors.append('Region %s: missing seed count.' % rid)
        if not params.get('ratio') and not params.get('ratioSequence'):
            errors.append('Region %s: missing ratio.' % rid)
    if len(part_names) > 1:
        errors.append('Batch spans multiple Parts: %s' % sorted(part_names))
    # Duplicate signatures suggest the same geometry picked twice.
    seen = set()
    for h in hashes:
        if h in seen:
            errors.append('Duplicate region signature %s (same geometry picked twice).' % h)
        seen.add(h)
    return errors


def resolve_selection(part, region, entity_kind, tol=1e-4):
    """Returns the matched Abaqus object, or None if not found / ambiguous."""
    sig = region.get('signature') or {}
    key_map = {
        'face': ('face_point', 'face_area'),
        'fine_edge': ('fine_point', 'fine_len'),
        'coarse_edge': ('coarse_point', 'coarse_len'),
    }
    if entity_kind not in key_map:
        return None
    pt_key, len_key = key_map[entity_kind]
    target_pt = sig.get(pt_key)
    if not target_pt:
        return None
    collection = {
        'face': part.faces,
        'fine_edge': part.edges,
        'coarse_edge': part.edges,
    }[entity_kind]
    matches = []
    for i in range(len(collection)):
        obj = collection[i]
        try:
            p = obj.pointOn[0]
            dist = sum((p[j] - target_pt[j]) ** 2 for j in range(3)) ** 0.5
            if dist <= tol:
                expected_len = sig.get(len_key)
                if expected_len is not None:
                    try:
                        actual_len = obj.getSize(printResults=False)
                        if abs(actual_len - expected_len) > tol * max(1.0, expected_len):
                            continue
                    except Exception:
                        pass
                matches.append(obj)
        except Exception:
            continue
    if len(matches) == 1:
        return matches[0]
    return None  # 0 or ambiguous


def build_region_plans(part, regions):
    """Returns (resolved, unresolved) where each item is (region_dict, resolved_entities_dict)."""
    resolved = []
    # @author  created by IY,China University of Petroleum (East China)  --  禁止商用 (non-commercial)
    unresolved = []
    for r in regions:
        entities = {}
        ok = True
        for kind in ('face', 'fine_edge', 'coarse_edge'):
            obj = resolve_selection(part, r, kind)
            if obj is not None:
                entities[kind] = obj
            elif (r.get('signature') or {}).get(
                    {'face': 'face_point', 'fine_edge': 'fine_point',
                     'coarse_edge': 'coarse_point'}[kind]):
                ok = False
        if ok:
            resolved.append((r, entities))
        else:
            unresolved.append(r)
    return resolved, unresolved
