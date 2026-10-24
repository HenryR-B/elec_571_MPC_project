"""LuGre drive friction as DRIVE_SLIDING_LAW = "lugre" (friction_model_plan.md, section 4).

Adds one bristle-deflection state z_i per wheel after the blend weights (current plant: 18 states,
voltage plant: 22). With "lugre" each wheel's drive force follows from (z_i, slip, load), so the
contact solve takes no grip/slide decision and solves the robot-level equations once. The other
laws are unchanged (their z stays zero).
Usage: python patch_lugre.py NOTEBOOK...   (after patch_speed_a and patch_speed_b)
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
    s = rep(s, '''#   "atan": the previous law, -N mu_d(|v|) (2/pi) atan(K_SMOOTH_D v). Zero at zero slip, and
#     about 5 cm/s of slip to recover, so a wheel lost its drive force when it broke away.
''', '''#   "atan": the previous law, -N mu_d(|v|) (2/pi) atan(K_SMOOTH_D v). Zero at zero slip, and
#     about 5 cm/s of slip to recover, so a wheel lost its drive force when it broke away.
#   "lugre": LuGre dynamic friction (Canudas de Wit et al. 1995), one bristle state per wheel
#     (LUGRE_* below). No grip/slide decision and no blend; steady sliding gives the same
#     mu_d(|v|) N. Section 4.15 compares it with "switch".
''')
    s = rep(s, '''DRIVE_MODE_BLEND_TIME = 0.033  # about 5 cm of travel at 1.5 m/s
''', '''DRIVE_MODE_BLEND_TIME = 0.033  # about 5 cm of travel at 1.5 m/s

# LuGre drive friction (DRIVE_SLIDING_LAW = "lugre"). None of these is measured; the reasoning is in
# friction_model_plan.md, section 4.2. For each wheel, with slip v and g(v) = mu_d(|v|) N:
#   dz/dt = v - sigma0 |v| z / g(v),   F = -(sigma0 z + sigma1 dz/dt + sigma2 v)
# LUGRE_PRESLIDING_DEFLECTION: bristle deflection z at the static limit mu_s N [m]; sets the
#   stiffness sigma0 = mu_s N / deflection (about 7200 N/m for a rear wheel at 0.5 mm).
# LUGRE_DAMPING_RATIO: sigma1 as a fraction of critical damping of sigma0 against the rim inertia
#   J_w / R_w^2 (18.5 g).
# LUGRE_VISCOUS_COEFF: sigma2 [N s/m]. 0 keeps steady sliding equal to mu_d(|v|) N.
LUGRE_PRESLIDING_DEFLECTION = 0.5e-3
LUGRE_DAMPING_RATIO = 1.0
LUGRE_VISCOUS_COEFF = 0.0
''')
    return s


def friction_law(s):
    s = rep(s, '''    "atan": -N mu_d(|v|) (2/pi) atan(K_SMOOTH_D v), zero at zero slip (previous law).
    Both are smooth, odd functions of the slip.''', '''    "atan": -N mu_d(|v|) (2/pi) atan(K_SMOOTH_D v), zero at zero slip (previous law).
    "lugre": the "continuous" curve, for diagnostics only; the LuGre force comes from the bristle
    state in the contact solve.
    Both are smooth, odd functions of the slip.''')
    s = rep(s, '''    if DRIVE_SLIDING_LAW in ("continuous", "switch"):''',
               '''    if DRIVE_SLIDING_LAW in ("continuous", "switch", "lugre"):''')
    s = rep(s, '''        raise ValueError("DRIVE_SLIDING_LAW must be 'switch', 'continuous', or 'atan'")''',
               '''        raise ValueError("DRIVE_SLIDING_LAW must be 'switch', 'continuous', 'atan', or 'lugre'")''')
    return s


LUGRE_BRANCH = '''        bristle_rate = np.zeros(N_WHEELS)
        if DRIVE_SLIDING_LAW == "lugre":
            # LuGre (Canudas de Wit et al. 1995): each wheel's drive force follows from its bristle
            # deflection z_i and its slip, so there is no grip/slide decision and the robot-level
            # equations are solved once:
            #   dz/dt = v - sigma0 |v| z / g(v),   F = -(sigma0 z + sigma1 dz/dt + sigma2 v),
            #   g(v) = mu_d(|v|) N, the Stribeck curve of the other laws.
            # sigma0 / g(v) = mu_s / (mu_d(|v|) LUGRE_PRESLIDING_DEFLECTION) does not depend on the
            # load, so a lifted wheel (N = 0) needs no special case.
            bristle = self.bristle_deflections(state_world)
            sigma0, sigma1 = self.lugre_coefficients(normal_loads)
            bristle_rate = slip_d - (rc.MU_D_STAT / (mu_d(slip_d) * LUGRE_PRESLIDING_DEFLECTION)) * np.abs(slip_d) * bristle
            force_used = -(sigma0 * bristle + sigma1 * bristle_rate + LUGRE_VISCOUS_COEFF * slip_d)
            if not self.enable_friction:
                force_used = np.zeros(N_WHEELS)
            generalized_acceleration = np.linalg.solve(
                self.body_mass, self.H @ force_used + a_hat_wrench + wrench_body - coriolis
            )
            # For plots and counts only: a wheel counts as sliding once its bristles carry nearly the
            # sliding force and it slips faster than the transition band.
            sliding_force = self.friction_scale * np.asarray(normal_loads, dtype=float) * mu_d(slip_d)
            rolling = ~((np.abs(sigma0 * bristle) >= 0.95 * sliding_force)
                        & (np.abs(slip_d) >= rc.DRIVE_BREAKAWAY_BAND)) & self.enable_friction
            force_required = force_to_roll.copy()
            grip_restoring_force = np.zeros(N_WHEELS)
            blend = np.ones(N_WHEELS)
        else:
'''


def slip_plant(s):
    s = rep(s, '''    # First of the four grip/slide blend weights lambda_i (1 = rolling, 0 = sliding).
    blend_start = 10
''', '''    # First of the four grip/slide blend weights lambda_i (1 = rolling, 0 = sliding).
    blend_start = 10
    # First of the four LuGre bristle deflections z_i [m] (zero with the other friction laws).
    bristle_start = 14
''')
    s = rep(s, '''        supplied_blend = None
        if initial_state is not None and np.shape(initial_state) == (14,):
            supplied_blend = np.asarray(initial_state, dtype=float)[10:14].copy()
            initial_state = np.asarray(initial_state, dtype=float)[:10]''', '''        supplied_extra = None
        if initial_state is not None and np.shape(initial_state) in ((14,), (18,)):
            supplied_extra = np.asarray(initial_state, dtype=float)[10:].copy()
            initial_state = np.asarray(initial_state, dtype=float)[:10]''')
    s = rep(s, '''        # Grip/slide blend weights, rolling at the start unless given.
        self.state = np.r_[self.state, np.ones(N_WHEELS) if supplied_blend is None else supplied_blend]''',
               '''        # Grip/slide blend weights (rolling) and LuGre bristle deflections (relaxed) at the start,
        # unless given.
        extra = np.r_[np.ones(N_WHEELS), np.zeros(N_WHEELS)]
        if supplied_extra is not None:
            extra[:supplied_extra.size] = supplied_extra
        self.state = np.r_[self.state, extra]''')
    s = rep(s, '''    def blend_rates(self, state_world):
        """d(lambda_i)/dt = (target_i - lambda_i) / tau; target from the last contact solve."""
        if DRIVE_MODE_BLEND_TIME <= 0.0:
            return np.zeros(N_WHEELS)''', '''    def blend_rates(self, state_world):
        """d(lambda_i)/dt = (target_i - lambda_i) / tau; target from the last contact solve."""
        if DRIVE_MODE_BLEND_TIME <= 0.0 or DRIVE_SLIDING_LAW == "lugre":
            return np.zeros(N_WHEELS)''')
    s = rep(s, '''    def update_contact_disturbances(self):
        """Sample the friction disturbance; this step's wheel loads come from the load model."""''',
               '''    def bristle_deflections(self, state_world):
        """LuGre bristle deflections z_i [m] (zero with the other friction laws)."""
        b = self.bristle_start
        return np.asarray(state_world[b:b + N_WHEELS], dtype=float)

    def bristle_rates(self):
        """dz_i/dt from the last contact solve (zero with the other friction laws)."""
        return self._force_selection_diagnostics["bristle_rate"]

    def lugre_coefficients(self, normal_loads):
        """LuGre bristle stiffness sigma0 [N/m] and damping sigma1 [N s/m] for each wheel.

        sigma0 puts the static limit mu_s N at LUGRE_PRESLIDING_DEFLECTION of deflection, so it is
        proportional to the load; sigma1 is LUGRE_DAMPING_RATIO of critical damping of sigma0
        against the rim inertia J_w / R_w^2. Neither is measured (friction_model_plan.md, 4.2).
        """
        sigma0 = rc.MU_D_STAT * self.friction_scale * np.asarray(normal_loads, dtype=float) / LUGRE_PRESLIDING_DEFLECTION
        sigma1 = 2.0 * LUGRE_DAMPING_RATIO * np.sqrt(sigma0 * rc.WHEEL_INERTIA / rc.WHEEL_RADIUS**2)
        return sigma0, sigma1

    def update_contact_disturbances(self):
        """Sample the friction disturbance; this step's wheel loads come from the load model."""''')
    # The grip/slide search becomes the "else" of the LuGre branch.
    start = s.index('        for _ in range(N_WHEELS + 1):\n')
    end = s.index('        drive_wrench = self.H @ force_used\n')
    block = s[start:end]
    indented = ''.join(('    ' + line) if line.strip() else line for line in block.splitlines(keepends=True))
    s = s[:start] + LUGRE_BRANCH + indented + s[end:]
    s = rep(s, '''            "blend": blend,
        }''', '''            "blend": blend,
            "bristle_rate": bristle_rate,
        }''')
    s = rep(s, '''            generalized_acceleration[2], theta_ddot, self.blend_rates(state_world),
        ]''', '''            generalized_acceleration[2], theta_ddot, self.blend_rates(state_world), self.bristle_rates(),
        ]''')
    s = rep(s, '''        snapshot["blend"] = snapshot["state_after"][b:b + N_WHEELS].copy()''',
               '''        snapshot["blend"] = snapshot["state_after"][b:b + N_WHEELS].copy()
        z = self.bristle_start
        snapshot["bristle_deflection"] = snapshot["state_after"][z:z + N_WHEELS].copy()''')
    return s


def voltage_plant(s):
    s = rep(s, '''    blend_start = 14  # the grip/slide blend weights follow the winding currents
''', '''    blend_start = 14  # the grip/slide blend weights follow the winding currents
    bristle_start = 18  # then the LuGre bristle deflections
''')
    s = rep(s, '''        if initial_state is not None and np.shape(initial_state) in ((14,), (18,)):''',
               '''        if initial_state is not None and np.shape(initial_state) in ((14,), (18,), (22,)):''')
    s = rep(s, '''        # State order: 10 slip-plant states, 4 winding currents, 4 blend weights.''',
               '''        # State order: 10 slip-plant states, 4 winding currents, 4 blend weights, 4 bristle deflections.''')
    return s


def state_test(s):
    s = rep(s, '''assert _test_plant.state.shape == (14,), "Slip-plant state: 6 chassis states, 4 wheel speeds, 4 blend weights"''',
               '''assert _test_plant.state.shape == (18,), "Slip-plant state: 6 chassis states, 4 wheel speeds, 4 blend weights, 4 bristle deflections"''')
    s = rep(s, '''assert _test_derivative.shape == (14,), "Plant derivative must match the 14-state vector"''',
               '''assert _test_derivative.shape == (18,), "Plant derivative must match the 18-state vector"''')
    s = rep(s, '''assert np.allclose(
    _test_diagnostics["force_used"][_test_diagnostics["rolling_mask"]],
    _test_diagnostics["force_required"][_test_diagnostics["rolling_mask"]],
), "Rolling wheels must use the required static traction reaction"''', '''if DRIVE_SLIDING_LAW != "lugre":  # LuGre takes no grip decision: its force comes from the bristles
    assert np.allclose(
        _test_diagnostics["force_used"][_test_diagnostics["rolling_mask"]],
        _test_diagnostics["force_required"][_test_diagnostics["rolling_mask"]],
    ), "Rolling wheels must use the required static traction reaction"''')
    return s


EDITS = [('549569af', config), ('f26c71d2', friction_law), ('9676641c', slip_plant),
         ('vplant02', voltage_plant), ('c4e5f6a7', state_test)]

if __name__ == "__main__":
    for path in sys.argv[1:]:
        nb = json.load(open(path, encoding='utf-8'))
        cells = {c.get('id'): c for c in nb['cells']}
        for cid, fn in EDITS:
            if cid not in cells:
                continue
            c = cells[cid]
            c['source'] = lines(fn(''.join(c['source'])))
            if c['cell_type'] == 'code':
                c['outputs'] = []
                c['execution_count'] = None
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            json.dump(nb, f, ensure_ascii=False, indent=1)
            f.write('\n')
        print('patched', path)
