# -*- coding: utf-8 -*-
# ============================================================================
#  Transition Mesh
#  created by IY,China University of Petroleum (East China)
#  禁止商用 / Non-commercial use only
#  禁止商用：本软件仅供学习与研究使用，未经作者书面许可不得用于任何商业用途。
#  Non-commercial use only. Unauthorized commercial use is prohibited.
# ============================================================================

"""Pure Python on purpose (no ``abaqus`` imports) so that the CAE dialog and the kernel ask the SAME table"""
from __future__ import division
import math

import transition_mesh_plan as tm_plan

RATIOS = (2, 3)

# Segments of the side template per merge group (2:1 = 4 fine -> 2 coarse,
# 3:1 = 3 fine -> 1 coarse).
PERIOD = {2: 4, 3: 3}


def period(ratio):
    r = int(ratio)
    if r not in PERIOD:
        raise ValueError('Ratio must be 2 or 3, got %r' % (ratio,))
    return PERIOD[r]


# Capability matrix status: 'ok' fully supported 'warn' supported with a known quality/precondition caveat 'no'
# not supported by the current templates Every entry carries the reason and, where one exists,
CAPABILITY = {
    ('PLANAR', 2): {
        'status': 'ok', 'layers': True, 'bidirectional': True,
        'precondition': '',
        'note': 'Classic plane strip; 2:1 stays square.'},
    ('PLANAR', 3): {
        'status': 'ok', 'layers': True, 'bidirectional': True,
        'precondition': '',
        'note': 'Classic plane strip; 3:1 mid-depth row is inherent.'},
    ('PLANAR', 'hybrid'): {
        'status': 'warn', 'layers': False, 'bidirectional': False,
        'precondition': 'Layers must be 1; needs 2M <= N <= 3M and an even '
                        'number of 2:1 groups (they tile in pairs)',
        'note': 'Mixed 3:1+2:1: the 2:1 groups must come in pairs (a lone '
                '2 fine -> 1 coarse column needs a hanging node), and the '
                'coarse side then alternates 2h/3h element widths. Default '
                'block order groups them (one side all 2:1, the other all '
                '3:1); hybridPattern/interleave keeps the even spread.'},
    ('PLANAR_ANNULAR', 2): {
        'status': 'ok', 'layers': True, 'bidirectional': True,
        'precondition': '',
        'note': 'Flat ring: sector-period template, full ring or sectors.'},
    ('PLANAR_ANNULAR', 3): {
        'status': 'ok', 'layers': True, 'bidirectional': True,
        'precondition': '',
        'note': 'Flat ring sector template; arc lengths differ per radius, '
                'so the template is angular.'},
    ('PLANAR_QUAD_FOUR', 2): {
        'status': 'ok', 'layers': True, 'bidirectional': False,
        'precondition': 'Coarse loop optional: left out it is computed from '
                        'the ratio and Layers (case rule 4).',
        'note': 'Case squad diagonal corners; regular collars are inserted '
                'between levels when needed, without changing fine seeds.'},
    ('PLANAR_QUAD_FOUR', 3): {
        'status': 'warn', 'layers': True, 'bidirectional': False,
        'precondition': 'Coarse loop optional (auto outer loop).',
        'note': '3:1 four-side is constructible but the mid-depth row is '
                'inherent (element aspect ~3); use 2:1 for a square ring.'},
    ('ANNULAR_RADIAL', 2): {
        'status': 'ok', 'layers': True, 'bidirectional': True,
        'precondition': 'Layers > 1 needs the wall already partitioned at '
                        'the intermediate radii.',
        'note': 'Cylindrical band: angular sector template on the shell.'},
    ('ANNULAR_RADIAL', 3): {
        'status': 'ok', 'layers': True, 'bidirectional': True,
        'precondition': 'Layers > 1 needs the wall already partitioned at '
                        'the intermediate radii.',
        'note': 'Cylindrical band, angular sector template.'},
    ('AXIAL_SURFACE', 3): {
        'status': 'ok', 'layers': True, 'bidirectional': False,
        'precondition': 'Needs target Cells plus a continuous axial path.',
        'note': 'Wall is sliced by radial planes and swept along the axis.'},
    ('AXIAL_SURFACE', 2): {
        'status': 'ok', 'layers': True, 'bidirectional': False,
        'precondition': 'Needs Seeds divisible by 4 (coarse count even); '
                        'the angular-fit guard refuses a slice pattern that '
                        'collides with the existing radial seams.',
        'note': 'Each sector carries a 4-fine -> 2-coarse pair block '
                '(6 hex, no hanging node) and is swept through the wall; '
                'meshes cleanly when the sector count exceeds the seam count.'},
}


def capability(transition_type, ratio, layers=1, hybrid=False,
               seeds=None, coarse_seeds=None):
    """Returns a dict with at least ``status``, ``headline``, ``reason``,"""
    ttype = str(transition_type or '').upper()
    entry = None
    key = None
    if hybrid:
        key = (ttype, 'hybrid')
        entry = CAPABILITY.get(key)
    if entry is None:
        r = None
        try:
            r = tm_plan._ratio(ratio)
        except Exception:
            r = None
        key = (ttype, r)
        entry = CAPABILITY.get(key)
    if entry is None:
        return {'status': 'unknown', 'transition_type': ttype,
                'ratio': None, 'headline': 'Unknown transition combination',
                'reason': 'No capability entry for %s / %s' % (ttype, ratio),
                'fix': 'Pick PLANAR, PLANAR_ANNULAR, PLANAR_QUAD_FOUR, '
                       'ANNULAR_RADIAL or AXIAL_SURFACE with ratio 2:1/3:1.',
                'note': '', 'layers_supported': False,
                'bidirectional': False, 'precondition': ''}
    layers = int(layers) if layers else 1
    status = entry['status']
    reason = entry['precondition']
    fix = ''
    if entry.get('layers') is False and layers > 1:
        status = 'no'
        reason = ('%s cannot be combined with Layers=%d'
                  % ('Hybrid' if hybrid else ttype, layers))
        fix = 'Set Layers=1 for this combination.'
    if hybrid and seeds and coarse_seeds:
        # The same splitter Apply uses decides the verdict, so the dialog can
        # show an untileable mix (odd 2:1 count, out of range) before any run.
        try:
            split = hybrid_split(seeds, coarse_seeds)
        except Exception:
            split = None
        if split is not None and not split['ok']:
            status = 'no'
            reason = split['error']
            fix = ('Valid coarse counts nearby: %s'
                   % split['candidate_ms'] if split['candidate_ms']
                   else 'Change Seeds or Coarse Seeds.')
        elif split is not None:
            status = 'ok' if split['mode'] != 'hybrid' else 'warn'
            reason = ('blocks=%d, a=%d (3:1), b=%d (2:1), order=%s; coarse '
                      'side alternates 2h/3h element widths'
                      % (len(split['blocks']), split['a'], split['b'],
                         split.get('order') or 'auto'))
    ratio_label = ('%d:1' % key[1]) if isinstance(key[1], int) else 'hybrid'
    headline = '%s %s %s: %s' % (
        ttype.replace('PLANAR_', '').replace('_', ' ').title(),
        ratio_label,
        'L%d' % layers, {'ok': 'supported', 'warn': 'supported with caveat',
                         'no': 'NOT supported',
                         'unknown': 'unknown'}[status])
    return {
        'status': status,
        'transition_type': ttype,
        'ratio': key[1],
        'hybrid': bool(hybrid),
        'layers': layers,
        'layers_supported': bool(entry.get('layers')),
        'bidirectional': bool(entry.get('bidirectional')),
        'precondition': entry.get('precondition', ''),
        'note': entry.get('note', ''),
        'headline': headline,
        'reason': reason,
        'fix': fix,
    }


# Count legality (thin wrappers so callers never re-derive periods)

def required_multiple(ratio, layers=1, quad=False):
    """Non-quad mirrors ``transition_mesh_plan.plan_counts`` (LCM of the per-stage terms)."""
    r = int(ratio)
    k = max(1, int(layers))
    p = period(r)
    if quad:
        if r == 2:
            return 4
        return p * r ** (k - 1)
    divisor, product = 1, 1
    for _ in range(k):
        term = product * p
        divisor = divisor * term // math.gcd(divisor, term)
        product *= r
    return divisor


def counts_ok(seeds, ratio, layers=1, quad=False, ratio_sequence=''):
    """(ok, error_or_None) using the SAME planner Apply uses."""
    try:
        planner = tm_plan.plan_quad_counts if quad else tm_plan.plan_counts
        planner(int(seeds), int(layers), ratio, ratio_sequence)
        return True, None
    except Exception as exc:
        return False, str(exc)


def nearby_valid_counts(seeds, ratio, layers=1, quad=False,
                        ratio_sequence='', how_many=2):
    """Nearest legal counts from the shared planner (never a local floor)."""
    cands = tm_plan.suggest_counts(int(seeds), int(layers), ratio,
                                   ratio_sequence, quad=quad)
    return list(cands[:max(1, int(how_many))])


# Four-side outer size (case rule 4)

def _interleave(n_a, n_b):
    """Bresenham style: item B goes where its ideal cumulative count advances."""
    n_a = int(n_a)
    n_b = int(n_b)
    total = n_a + n_b
    if total <= 0:
        return []
    out = []
    for i in range(total):
        before = (i * n_b) // total
        after = ((i + 1) * n_b) // total
        out.append('B' if after > before else 'A')
    return out


def hybrid_split(N, M, sequence='', order=''):
    """The linear system is unique: with ``a`` 3:1 groups (3 fine ports -> 1 coarse port) and ``b`` 2:"""
    N = int(N)
    M = int(M)
    out = {'N': N, 'M': M, 'ok': False, 'mode': 'invalid', 'a': 0, 'b': 0,
           'pairs': 0, 'blocks': [], 'ports': [], 'coarse_ports': 0,
           'pattern': '', 'element_faces': 0, 'label': '',
           'error': '', 'candidate_ms': [], 'auto_order': False,
           'order': ''}
    if N <= 0 or M <= 0:
        out['error'] = ('Hybrid needs a positive fine count N and coarse '
                        'count M (got N=%d, M=%d).' % (N, M))
        return out
    lo_m = int(math.ceil(N / 3.0))
    hi_m = int(N // 2)
    if M < lo_m or M > hi_m:
        # created by IY,China University of Petroleum (East China)
        out['error'] = ('Hybrid range violated: N=%d with M=%d coarse ports '
                        'needs %d <= M <= %d (each coarse port can carry '
                        'only 2 or 3 fine ports).' % (N, M, lo_m, hi_m))
        out['candidate_ms'] = [m for m in range(max(1, lo_m), hi_m + 1)
                               if (3 * m - N) % 2 == 0][:6]
        return out
    a = N - 2 * M
    b = 3 * M - N
    out['a'] = a
    out['b'] = b
    if b % 2:
        out['error'] = ('Hybrid needs an even number of 2:1 groups: N=%d, M=%d '
                        'gives b=%d (odd). The 2:1 template tiles in pairs '
                        '(4 fine -> 2 coarse). Valid M nearby: %s.'
                        % (N, M, b, [m for m in range(max(1, lo_m), hi_m + 1)
                                     if (3 * m - N) % 2 == 0][:6]))
        out['candidate_ms'] = [m for m in range(max(1, lo_m), hi_m + 1)
                               if (3 * m - N) % 2 == 0][:6]
        return out
    pairs = b // 2
    out['pairs'] = pairs
    if a == 0:
        out['mode'] = 'pure2'
    elif b == 0:
        out['mode'] = 'pure3'
    else:
        out['mode'] = 'hybrid'
    text = str(sequence or '').strip()
    if text:
        blocks = [x.strip() for x in text.replace(';', ',').split(',')
                  if x.strip()]
        norm = []
        for item in blocks:
            key = item.replace(' ', '').lower()
            if key in ('3', '3:1'):
                norm.append('3')
            elif key in ('22', '2:1,2:1', '22pair', 'pair2', '22:1'):
                norm.append('22')
            elif key in ('2', '2:1'):
                norm.append('2')
            else:
                norm.append('?')
        bad = [x for x in norm if x not in ('3', '22')]
        ports = 0
        coarse = 0
        for kind in norm:
            if kind == '3':
                ports += 3
                coarse += 1
            else:
                ports += 4
                coarse += 2
        if bad or ports != N or coarse != M:
            out['error'] = (
                'Hybrid pattern %r does not match N=%d/M=%d: it gives %d '
                'fine ports and %d coarse ports. Use comma-separated blocks '
                "'3' (3:1) and '22' (a 2:1 pair); bad entries: %s."
                % (text, N, M, ports, coarse, bad or 'none'))
            out['blocks'] = norm
            out['ports'] = []
            return out
        out['blocks'] = norm
        out['auto_order'] = False
        out['order'] = 'user'
    else:
        if str(order or '').strip().lower() in ('interleave', 'spread', 'mix'):
            spread = _interleave(a, pairs)
            out['blocks'] = ['3' if kind == 'A' else '22'
                             for kind in spread]
            out['order'] = 'interleave'
        else:
            out['blocks'] = ['3'] * a + ['22'] * pairs
            out['order'] = 'grouped'
        out['auto_order'] = True
    for kind in out['blocks']:
        out['ports'].append(3 if kind == '3' else 4)
    out['coarse_ports'] = sum(1 if kind == '3' else 2 for kind in
                              out['blocks'])
    out['element_faces'] = sum(4 if kind == '3' else 6 for kind in
                               out['blocks'])
    out['pattern'] = ','.join(out['blocks'])
    out['ok'] = True
    out['label'] = '%d x 3:1 + %d x 2:1 (%d pair blocks)' % (a, b, pairs)
    return out


def quad_four_rule(ratio, Nx, Ny, hxi, hyi, layers):
    """With a fine count N on a side of length 2a the fine element is h = 2a/N and layer i merges to r^i*h,"""
    r = int(ratio)
    if r == 2:
        from transition_mesh_quad2 import build_plan
        plan = build_plan(hxi, hyi, Nx, Ny, layers)
        return {'ratio': 2, 'layers': int(layers),
                'h_u': 2.0 * hxi / Nx, 'h_v': 2.0 * hyi / Ny,
                'width_u': [s['H'] - s['inner_hx'] for s in plan['levels']],
                'width_v': [s['V'] - s['inner_hy'] for s in plan['levels']],
                'H_required': plan['H'], 'V_required': plan['V'],
                'levels': plan['levels']}
    k = max(1, int(layers))
    Nx = int(Nx)
    Ny = int(Ny)
    if Nx <= 0 or Ny <= 0:
        raise ValueError('Nx/Ny must be positive.')
    h_u = 2.0 * float(hxi) / Nx
    h_v = 2.0 * float(hyi) / Ny
    width_u = [h_u * r ** i for i in range(1, k + 1)]
    width_v = [h_v * r ** i for i in range(1, k + 1)]
    return {
        'ratio': r, 'layers': k, 'h_u': h_u, 'h_v': h_v,
        'width_u': width_u, 'width_v': width_v,
        'H_required': float(hxi) + sum(width_u),
        'V_required': float(hyi) + sum(width_v),
    }


def layer_rule_deviation(rule, H, V, rel_tol=1e-3):
    """None when the picked outer loop satisfies rule 4, else the numbers."""
    tol = float(rel_tol) * max(abs(H), abs(V), 1.0)
    du = abs(float(H) - rule['H_required'])
    dv = abs(float(V) - rule['V_required'])
    if du <= tol and dv <= tol:
        return None
    return {'du': du, 'dv': dv,
            'H_required': rule['H_required'], 'V_required': rule['V_required'],
            'H_actual': float(H), 'V_actual': float(V)}


# Cylindrical angular fit (axial lane)

def axial_n_slices(seeds, ratio):
    """3:1 puts one 3-fine -> 1-coarse unit per sector, so the sector count equals the coarse count."""
    r = int(ratio)
    n = int(seeds)
    coarse = n // r
    return (coarse // 2) if r == 2 else coarse


def angular_fit(n_slices, seam_angles_deg, tol_deg=0.05):
    """``seam_angles_deg`` are the radial seam angles already present in the model (the plugin anchors slice"""
    n = int(n_slices)
    if n < 1:
        raise ValueError('n_slices must be >= 1.')
    spacing = 360.0 / n
    seams = sorted({float(a) % 360.0 for a in (seam_angles_deg or [])})
    coincident = 0
    crossing = []
    for a in seams:
        k = int(round(a / spacing))
        dist = abs(a - k * spacing)
        dist = min(dist, 360.0 - dist)
        if dist <= max(tol_deg, 1e-6):
            coincident += 1
        else:
            crossing.append(round(a, 4))
    total_planes = n
    new_planes = max(0, total_planes - coincident)
    verdict = 'ok'
    reason = ''
    if seams and new_planes == 0:
        verdict = 'no'
        reason = ('All %d slice planes coincide with existing radial seams, '
                  'so the partition would add no new face.' % n)
    elif crossing:
        verdict = 'warn'
        reason = ('%d existing seam(s) fall inside a sector (%s deg), '
                  'splitting it unevenly.' % (len(crossing),
                                              ', '.join(str(x) for x in
                                                        crossing[:4])))
    return {'n_slices': n, 'spacing_deg': spacing,
            'seams': [round(a, 4) for a in seams],
            'coincident': coincident, 'new_planes': new_planes,
            'crossing': crossing, 'verdict': verdict, 'reason': reason}


def angular_fix_counts(seeds, ratio, layers=1, quad=False, candidates=8):
    """Used for the actionable fix line:"""
    out = []
    n = int(seeds)
    for delta in range(-candidates, candidates + 1):
        c = n + delta
        if c <= 0 or c == n:
            continue
        ok, _err = counts_ok(c, ratio, layers, quad=quad)
        if ok:
            out.append(c)
    return out
