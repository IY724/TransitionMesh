# -*- coding: utf-8 -*-
# ============================================================================
#  Transition Mesh
#  created by IY,China University of Petroleum (East China)
#  禁止商用 / Non-commercial use only
#  禁止商用：本软件仅供学习与研究使用，未经作者书面许可不得用于任何商业用途。
#  Non-commercial use only. Unauthorized commercial use is prohibited.
# ============================================================================

"""Pure, shared count planning. Geometry adapters remain responsible for geometry."""
import math
import re


def parse_sequence(value, converter=float):
    if value is None or value == '':
        return []
    if isinstance(value, str):
        value = [x for x in re.split(r'[,;\s]+', value.strip()) if x]
    return [converter(x) for x in value]


def _ratio(value):
    text = str(value).strip()
    if ':' in text:
        a, b = text.split(':')
        if b.strip() != '1':
            raise ValueError('Only 2:1 and 3:1 transitions are supported.')
        text = a
    n = int(text)
    if n not in (2, 3):
        raise ValueError('Ratio must be 2:1 or 3:1.')
    return n


def plan_counts(fine_seeds, stages=1, ratio=2, ratio_sequence=None):
    n = int(fine_seeds)
    k = int(stages)
    if n <= 0 or k <= 0:
        raise ValueError('Fine count and transition stages must be positive.')
    ratios = parse_sequence(ratio_sequence, _ratio) or [_ratio(ratio)] * k
    if len(ratios) != k:
        raise ValueError('Ratio sequence needs exactly %d entries.' % k)
    product = 1
    divisor = 1
    for r in ratios:
        term = product * (4 if r == 2 else 3)
        divisor = divisor * term // math.gcd(divisor, term)
        product *= r
    if n % divisor:
        lo = max(divisor, (n // divisor) * divisor)
        hi = max(divisor, ((n + divisor - 1) // divisor) * divisor)
        raise ValueError('Fine count %d cannot form the requested stage chain. '
                         'Template requires multiples of %d; nearby counts: %d / %d.'
                         % (n, divisor, lo, hi))
    counts = [n]
    for r in ratios:
        counts.append(counts[-1] // r)
    return {'stages': k, 'ratios': ratios, 'counts': counts,
            'required_multiple': divisor, 'geometry_checked': False}


def allocate_widths(total, counts, widths=None):
    total = float(total)
    if not math.isfinite(total) or total <= 0:
        raise ValueError('Transition width must be finite and positive.')
    values = parse_sequence(widths)
    k = len(counts) - 1
    if values:
        if len(values) != k or any(not math.isfinite(x) or x <= 0 for x in values):
            raise ValueError('Provide one positive width per transition stage.')
        if abs(sum(values) - total) > 1e-7 * max(total, 1.0):
            raise ValueError('Stage widths must sum to the selected total width (%g).' % total)
        return values
    weights = [1.0 / n for n in counts[:-1]]
    return [total * w / sum(weights) for w in weights]


def plan_quad_counts(fine_seeds, stages=1, ratio=2, ratio_sequence=None):
    """轴对齐直条跨内边长，两侧各由一个角方块补齐到外框，故每边外圈段数为 fine//r + 2（两个角各贡献 1）。多层逐层重复此规则。"""
    seq = parse_sequence(ratio_sequence, _ratio)
    if _ratio(ratio) == 2 and (not seq or seq == [2] * int(stages)):
        from transition_mesh_quad2 import count_chain
        return count_chain(fine_seeds, stages)
    # @author  created by IY,China University of Petroleum (East China)  --  禁止商用 (non-commercial)
    n = int(fine_seeds)
    k = int(stages)
    if n <= 0 or k <= 0:
        raise ValueError('Fine count and transition stages must be positive.')
    ratios = parse_sequence(ratio_sequence, _ratio) or [_ratio(ratio)] * k
    if len(ratios) != k:
        raise ValueError('Ratio sequence needs exactly %d entries.' % k)
    counts = [n]
    for i, r in enumerate(ratios):
        per = 4 if r == 2 else 3
        strip = counts[-1] if i == 0 else counts[-1] - 2
        if strip <= 0 or strip % per:
            lo = (strip // per) * per if strip > 0 else per
            raise ValueError(
                'Four-side stage %d strip count %d is not divisible by %d '
                '(period for %d:1). Nearby: %d / %d.'
                % (i + 1, strip, per, r, max(per, lo), lo + per))
        nxt = strip // r + 2
        counts.append(nxt)
    return {'stages': k, 'ratios': ratios, 'counts': counts,
            'corner_contribution': 2, 'geometry_checked': False}


def nearby_quad_counts(fine_seeds, stages=1, ratio=2, ratio_sequence=None):
    ratios = parse_sequence(ratio_sequence, _ratio) or [_ratio(ratio)] * int(stages)
    n = int(fine_seeds)
    per = 4 if (ratios[0] if ratios else _ratio(ratio)) == 2 else 3
    lo = max(per, (n // per) * per)
    return sorted({lo, lo + per}, key=lambda x: (abs(x - n), x))


def suggest_counts(fine_seeds, stages=1, ratio=2, ratio_sequence=None,
                   quad=False):
    """Codex 09 P1 section 7: suggestions must come from one kernel planner that jointly considers the"""
    n = int(fine_seeds)
    k = int(stages) if stages else 1
    planner = plan_quad_counts if quad else plan_counts

    def legal(c):
        if c <= 0:
            return False
        try:
            planner(c, k, ratio, ratio_sequence)
            return True
        except Exception:
            return False

    below = None
    above = None
    max_steps = max(32, 16 * k)
    for step in range(1, max_steps + 1):
        if above is None and legal(n + step):
            above = n + step
        if below is None and legal(n - step):
            below = n - step
        if below is not None and above is not None:
            break
    out = []
    if below is not None:
        out.append(below)
    if above is not None:
        out.append(above)
    out.sort(key=lambda c: (abs(c - n), c))
    return out


def stage_boundaries(total_span, counts, widths=None):
    """Cumulative stage offsets from 0 to total_span (len = stages+1)."""
    w = allocate_widths(total_span, counts, widths)
    out = [0.0]
    for x in w:
        out.append(out[-1] + float(x))
    return out


def validate_stage_chain(fine_seeds, stages=1, ratio=2, ratio_sequence=None):
    """Alias of plan_counts for adapters that only need the chain."""
    return plan_counts(fine_seeds, stages, ratio, ratio_sequence)


def nearby_counts(fine_seeds, stages=1, ratio=2, ratio_sequence=None):
    ratios = parse_sequence(ratio_sequence, _ratio) or [_ratio(ratio)] * int(stages)
    # Obtain the necessary multiple without rounding any requested count.
    product, divisor = 1, 1
    for r in ratios:
        term = product * (4 if r == 2 else 3)
        # created by IY,China University of Petroleum (East China)
        divisor = divisor * term // math.gcd(divisor, term)
        product *= r
    n = int(fine_seeds)
    return sorted(set((max(divisor, n // divisor * divisor),
                       max(divisor, ((n + divisor - 1) // divisor) * divisor))),
                  key=lambda x: (abs(x - n), x))
