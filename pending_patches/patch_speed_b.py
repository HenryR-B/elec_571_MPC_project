"""Plant solver: reuse the Jacobian across steps (PLANT_REUSE_JACOBIAN) and expose the tolerances.

Each plant step is a fresh solve_ivp call, so Radau re-estimated its Jacobian (one evaluation per
state) every step. With PLANT_REUSE_JACOBIAN the step starts from the previous step's Jacobian and
the previous step size; a fresh Jacobian is computed when there is none, when a wheel's grip/slide
mode changed since it was computed, or when Radau asks again (slow Newton convergence). The
Jacobian only steers Newton's iterations; every step still meets PLANT_RTOL / PLANT_ATOL.
Usage: python patch_speed_b.py NOTEBOOK...   (after patch_speed_a)
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
    return rep(s, '''PLANT_INTEGRATOR = "Radau"
''', '''PLANT_INTEGRATOR = "Radau"
# Plant solver tolerances. The slip error they allow must stay far below the grip/slide
# transition band (DRIVE_BREAKAWAY_BAND, 5 mm/s). Looser is faster.
PLANT_RTOL = 1e-5
PLANT_ATOL = 1e-7
# True: each plant step starts from the previous step's Jacobian and step size; a fresh Jacobian
# is computed when a wheel changes grip/slide mode or the solver converges slowly. This changes
# how quickly each step is solved, not the tolerances it meets. False: a fresh one every step.
PLANT_REUSE_JACOBIAN = True
''')


def slip_plant(s):
    s = rep(s, '''    integrator_rtol = 1e-5
    integrator_atol = 1e-7
''', '''    integrator_rtol = PLANT_RTOL
    integrator_atol = PLANT_ATOL
''')
    s = rep(s, '''        self._step_cache = {}
        self.load_model = WHEEL_LOAD_MODEL if load_model is None else load_model''',
               '''        self._step_cache = {}
        # Jacobian of the last plant step and the grip/slide modes it was computed for, and the
        # solver's last step size (PLANT_REUSE_JACOBIAN).
        self._stored_jacobian = None
        self._last_step_size = None
        self.load_model = WHEEL_LOAD_MODEL if load_model is None else load_model''')
    s = rep(s, '''        result = solve_ivp(
            lambda time_s, state_world: self.physical_dynamics(
                state_world, current, wrench_body
            )[0],
            (0.0, self.dt),
            np.asarray(state, dtype=float),
            method=PLANT_INTEGRATOR,
            rtol=self.integrator_rtol,
            atol=self.integrator_atol,
            max_step=self.dt,
        )
        self.last_nfev = result.nfev  # solver effort for this step; chattering shows up here''',
               '''        last = {}  # the latest evaluation, so the Jacobian does not repeat it

        def rates(time_s, state_world):
            derivative = self.physical_dynamics(state_world, current, wrench_body)[0]
            last["state"], last["rates"] = state_world.copy(), derivative
            last["mode"] = self._force_selection_diagnostics["rolling_mask"]
            return derivative

        options = {}
        if PLANT_REUSE_JACOBIAN and PLANT_INTEGRATOR in ("Radau", "BDF", "LSODA"):
            options["jac"] = self._step_jacobian(rates, last)
            if self._last_step_size is not None:
                options["first_step"] = min(self._last_step_size, self.dt)
        result = solve_ivp(
            rates,
            (0.0, self.dt),
            np.asarray(state, dtype=float),
            method=PLANT_INTEGRATOR,
            rtol=self.integrator_rtol,
            atol=self.integrator_atol,
            max_step=self.dt,
            **options,
        )
        self.last_nfev = result.nfev  # solver effort for this step; chattering shows up here
        if result.t.size > 1:
            self._last_step_size = float(result.t[-1] - result.t[-2])''')
    s = rep(s, '''    def inverse_dynamics_current(self, desired_qdot, state_world, wrench_body, nominal=False):''',
               '''    def _step_jacobian(self, rates, last):
        """Jacobian function for one plant step (PLANT_REUSE_JACOBIAN).

        Radau asks once when the step starts, and again only when its Newton iteration converged
        slowly. The first request gets the stored Jacobian if the wheels' grip/slide modes match the
        ones it was computed for; otherwise, and on every later request, a fresh forward-difference
        Jacobian is computed and stored.
        """
        requests = [0]

        def jacobian(time_s, state_world):
            requests[0] += 1
            state_world = np.asarray(state_world, dtype=float)
            if "state" in last and np.array_equal(last["state"], state_world):
                base, mode = last["rates"], last["mode"]
            else:
                base = rates(time_s, state_world)
                mode = last["mode"]
            stored = self._stored_jacobian
            if (requests[0] == 1 and stored is not None and stored[0].shape == (state_world.size,) * 2
                    and np.array_equal(stored[1], mode)):
                return stored[0]
            steps = np.sqrt(np.finfo(float).eps) * np.maximum(np.abs(state_world), 1.0)
            matrix = np.empty((state_world.size, state_world.size))
            for j, step in enumerate(steps):
                shifted = state_world.copy()
                shifted[j] += step
                matrix[:, j] = (rates(time_s, shifted) - base) / step
            self._stored_jacobian = (matrix, np.array(mode))
            return matrix

        return jacobian

    def inverse_dynamics_current(self, desired_qdot, state_world, wrench_body, nominal=False):''')
    return s


EDITS = [('549569af', config), ('9676641c', slip_plant)]

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
