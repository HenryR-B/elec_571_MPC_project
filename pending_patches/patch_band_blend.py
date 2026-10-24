"""Apply the Karnopp/Leine transition band in blend mode too; make the needed-force overlay visible.

Before: with DRIVE_MODE_BLEND_TIME > 0 the band was skipped, so a sliding wheel whose slip crossed
zero got the raw slip law there (zero at zero slip). Now any wheel released within the band gets
sign(needed) mu_d(|v|) N as its sliding-law part, in both modes.
Usage: python patch_band_blend.py NOTEBOOK...   (after patch_toroll)
"""
import json
import sys


def rep(s, old, new, count=1):
    assert s.count(old) == count, (s.count(old), old[:80])
    return s.replace(old, new)


def lines(text):
    parts = text.rstrip('\n').split('\n')
    return [p + '\n' for p in parts[:-1]] + [parts[-1]]


def slip_plant(s):
    s = rep(s, '''        force_slide = force_d_sliding.copy()  # force of each sliding wheel (Karnopp band below)''',
               '''        force_slide = force_d_sliding.copy()  # force of each sliding wheel (Karnopp band below)
        force_law = force_d_sliding.copy()  # sliding-law force, with the band applied to released wheels''')
    s = rep(s, '''                    sliding * (blend * force_slide + (1.0 - blend) * force_d_sliding)
                    + rolling * (blend * (grip_restoring_force - wheel_torque / rc.WHEEL_RADIUS)
                                 + (1.0 - blend) * force_d_sliding)''',
               '''                    sliding * (blend * force_slide + (1.0 - blend) * force_law)
                    + rolling * (blend * (grip_restoring_force - wheel_torque / rc.WHEEL_RADIUS)
                                 + (1.0 - blend) * force_law)''')
    s = rep(s, '''            rolling[worst] = False
            if DRIVE_MODE_BLEND_TIME > 0.0:
                # Blending: the capped rolling part holds the grip limit in the direction of the
                # force the wheel needed; the blend weight then moves the force to the sliding law.
                force_slide[worst] = np.sign(force_required[worst]) * traction_limits[worst]
            elif DRIVE_SLIDING_LAW == "switch" and abs(slip_d[worst]) < rc.DRIVE_BREAKAWAY_BAND:
                # Transition band (Leine et al. 1998, after Karnopp 1985): a wheel breaking loose
                # near zero slip keeps mu_d(|v|) N (the static
                # limit at v = 0) in the direction of the force it needed, instead of the slip law,
                # which is zero at zero slip. At the band edge the slip law is equal to it (same
                # magnitude, and the slip grows opposite to this force), so nothing jumps.
                force_slide[worst] = (np.sign(force_required[worst]) * self.friction_scale[worst]
                                      * normal_loads[worst] * mu_d(abs(slip_d[worst])))''',
               '''            rolling[worst] = False
            if DRIVE_SLIDING_LAW == "switch" and abs(slip_d[worst]) < rc.DRIVE_BREAKAWAY_BAND:
                # Transition band (Leine et al. 1998, after Karnopp 1985): a wheel sliding
                # near zero slip (breaking loose, or its slip reversing) keeps mu_d(|v|) N (the static
                # limit at v = 0) in the direction of the force it needed, instead of the slip law,
                # which is zero at zero slip. At the band edge the slip law is equal to it (same
                # magnitude, and the slip grows opposite to this force), so nothing jumps.
                force_law[worst] = (np.sign(force_required[worst]) * self.friction_scale[worst]
                                    * normal_loads[worst] * mu_d(abs(slip_d[worst])))
                force_slide[worst] = force_law[worst]
            if DRIVE_MODE_BLEND_TIME > 0.0:
                # Blending: the capped rolling part holds the grip limit in the direction of the
                # force the wheel needed; the blend weight then moves the force to the sliding law.
                force_slide[worst] = np.sign(force_required[worst]) * traction_limits[worst]''')
    s = rep(s, '''        force_used = blend * rolling_part + (1.0 - blend) * force_d_sliding''',
               '''        force_used = blend * rolling_part + (1.0 - blend) * force_law''')
    return s


def harness(s):
    return rep(s, '''    ax.plot(time, needed, color, alpha=0.45, linestyle=":", label=f"needed to roll, {label}")''',
               '''    ax.plot(time, needed, color, linewidth=2.2, alpha=0.9, linestyle=":", label=f"needed to roll, {label}")''')


EDITS = [('9676641c', slip_plant), ('plantv02', harness)]

if __name__ == "__main__":
    for path in sys.argv[1:]:
        nb = json.load(open(path, encoding='utf-8'))
        cells = {c.get('id'): c for c in nb['cells']}
        for cid, fn in EDITS:
            c = cells[cid]
            c['source'] = lines(fn(''.join(c['source'])))
            c['outputs'] = []
            c['execution_count'] = None
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            json.dump(nb, f, ensure_ascii=False, indent=1)
            f.write('\n')
        print('patched', path)
