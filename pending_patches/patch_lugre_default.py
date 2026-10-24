"""Make LuGre the default drive friction law; keep the switch-specific checks on the switch model.

- DRIVE_SLIDING_LAW = "lugre".
- toe02: LuGre wheels creep slightly under load, so the gripping-kinematics tolerance is 1e-3 under
  LuGre (1e-4 for the exact grip of the switch model).
- speed02 (4.14) tests the switch model's grip/slide blend, so it runs the switch law whatever the
  configured law is.
Usage: python patch_lugre_default.py NOTEBOOK...   (after patch_lugre_v2)
"""
import json
import sys


def rep(s, old, new, count=1):
    assert s.count(old) == count, (s.count(old), old[:80])
    return s.replace(old, new)


def lines(text):
    parts = text.rstrip('\n').split('\n')
    return [p + '\n' for p in parts[:-1]] + [parts[-1]]


def config(s):
    s = rep(s, '''#   "switch" (default): the switch model of Leine et al. (1998).''',
               '''#   "switch": the switch model of Leine et al. (1998).''')
    s = rep(s, '''#   "lugre": LuGre dynamic friction (Canudas de Wit et al. 1995), one bristle state per wheel
#     (LUGRE_* below). No grip/slide decision and no blend; steady sliding gives the same
#     mu_d(|v|) N. Section 4.15 compares it with "switch".''',
               '''#   "lugre" (default): LuGre dynamic friction (Canudas de Wit et al. 1995), one bristle state per
#     wheel (LUGRE_* below). No grip/slide decision and no blend; the force is continuous, holds the
#     bristle load through zero slip, and in sliding follows the same mu_d(|v|) N. Sections 4.15-4.16
#     compare it with "switch" (about 2x the run time).''')
    s = rep(s, '''DRIVE_SLIDING_LAW = "switch"''', '''DRIVE_SLIDING_LAW = "lugre"''')
    return s


def toe(s):
    return rep(s, '''                assert gripping.any() and kinematic_gap < 1e-4, f"{layout}: chassis twist and wheel kinematics disagree for {label}"''',
               '''                # LuGre wheels creep slightly under load (LUGRE_CREEP_SLIP), so their grip is not exact.
                kinematic_tolerance = 1e-3 if DRIVE_SLIDING_LAW == "lugre" else 1e-4
                assert gripping.any() and kinematic_gap < kinematic_tolerance, f"{layout}: chassis twist and wheel kinematics disagree for {label}"''')


def at_speed(s):
    return rep(s, '''    at_speed = {"instant switch": run_at_speed(0.0), f"blend {DRIVE_MODE_BLEND_TIME * 1e3:.0f} ms": run_at_speed(DRIVE_MODE_BLEND_TIME)}''',
               '''    # This section tests the switch model's grip/slide blend, so it runs the switch law whatever
    # DRIVE_SLIDING_LAW is; sections 4.15-4.16 cover LuGre at speed.
    with friction_law("switch"):
        at_speed = {"instant switch": run_at_speed(0.0), f"blend {DRIVE_MODE_BLEND_TIME * 1e3:.0f} ms": run_at_speed(DRIVE_MODE_BLEND_TIME)}''')


def at_speed_md(s):
    return rep(s, '''run with the instant switch (`DRIVE_MODE_BLEND_TIME = 0`) and with the configured blend time.''',
               '''run with the instant switch (`DRIVE_MODE_BLEND_TIME = 0`) and with the configured blend time. It always uses the switch law, whatever `DRIVE_SLIDING_LAW` is, because it tests that law's blend.''')


def lugre_md(s):
    return rep(s, '''- **switch** (the default):''', '''- **switch**:''')


EDITS = [('549569af', config), ('toe02', toe), ('speed02', at_speed), ('speed01', at_speed_md), ('lugre01', lugre_md)]

if __name__ == "__main__":
    for path in sys.argv[1:]:
        nb = json.load(open(path, encoding='utf-8'))
        cells = {c.get('id'): c for c in nb['cells']}
        for cid, fn in EDITS:
            c = cells[cid]
            c['source'] = lines(fn(''.join(c['source'])))
            if c['cell_type'] == 'code':
                c['outputs'] = []
                c['execution_count'] = None
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            json.dump(nb, f, ensure_ascii=False, indent=1)
            f.write('\n')
        print('patched', path)
