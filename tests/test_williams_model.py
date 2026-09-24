import unittest

import numpy as np

import williams_model as wm


class WilliamsModelTests(unittest.TestCase):
    def setUp(self):
        self.wheels = wm.ROBOT_WHEELS
        self.params = wm.ModelParams(
            mass=1.0,
            inertia=1.0,
            mu_W_roller=0.25,
            mu_T_roller=0.15,
            mu_W_gap=0.56,
            mu_T_gap=0.56,
        )
        self.state = wm.make_rest_state(self.wheels)

    def test_zero_velocity_and_zero_wheel_speed_gives_zero_force(self):
        data = wm.wheel_forces(self.state, np.zeros(4), self.params, self.wheels)
        np.testing.assert_allclose(data["force"], 0.0, atol=1e-12)
        np.testing.assert_allclose(data["v_W"], 0.0, atol=1e-12)
        np.testing.assert_allclose(data["v_T"], 0.0, atol=1e-12)

    def test_no_longitudinal_slip_gives_zero_longitudinal_force(self):
        self.state[2] = 0.37
        self.state[3:5] = np.array([0.62, -0.21])
        self.state[5] = 0.8

        p_body, s_body, _ = wm.wheel_frames(self.wheels)
        R = wm.rotation_matrix(self.state[2])
        p_world = p_body @ R.T
        s_world = s_body @ R.T
        contact = self.state[3:5] + np.stack(
            (-self.state[5] * p_world[:, 1], self.state[5] * p_world[:, 0]), axis=1
        )
        v_contact_s = np.einsum("ij,ij->i", contact, s_world)
        rho = np.array([w.rho for w in self.wheels])
        u = -v_contact_s / rho

        data = wm.wheel_forces(self.state, u, self.params, self.wheels)
        np.testing.assert_allclose(data["v_W"], 0.0, atol=1e-12)
        F_long = np.einsum("ij,ij->i", data["force"], data["s_world"])
        np.testing.assert_allclose(F_long, 0.0, atol=1e-10)

    def test_force_reverses_when_slip_reverses(self):
        state_a = self.state.copy()
        state_a[3:5] = np.array([0.8, -0.35])
        state_a[5] = 0.4
        u = np.array([2.0, -1.0, 0.5, -1.5])

        state_b = state_a.copy()
        state_b[3:6] *= -1.0
        u_b = -u

        Fa = wm.wheel_forces(state_a, u, self.params, self.wheels)["force"]
        Fb = wm.wheel_forces(state_b, u_b, self.params, self.wheels)["force"]
        np.testing.assert_allclose(Fb, -Fa, rtol=1e-10, atol=1e-10)

    def test_theta_periodicity(self):
        state_a = self.state.copy()
        state_a[3:6] = np.array([0.9, 0.2, -0.3])
        state_b = state_a.copy()
        state_b[6:] += np.array([2 * np.pi / w.n_rollers for w in self.wheels])
        u = np.array([1.1, -0.7, 0.4, 0.9])

        da = wm.wheel_forces(state_a, u, self.params, self.wheels)
        db = wm.wheel_forces(state_b, u, self.params, self.wheels)
        np.testing.assert_array_equal(da["roller_contact"], db["roller_contact"])
        np.testing.assert_allclose(da["force"], db["force"], rtol=1e-12, atol=1e-12)

    def test_gap_has_higher_friction_than_roller_at_same_large_slip(self):
        wheel = self.wheels[0]
        pitch = 2 * np.pi / wheel.n_rollers
        state_r = self.state.copy()
        state_g = self.state.copy()
        state_r[6] = wheel.phase_offset + 0.25 * wheel.roller_fraction * pitch
        state_g[6] = wheel.phase_offset + (wheel.roller_fraction + 0.25 * (1 - wheel.roller_fraction)) * pitch
        state_r[3] = 2.0
        state_g[3] = 2.0
        u = np.zeros(4)

        dr = wm.wheel_forces(state_r, u, self.params, self.wheels)
        dg = wm.wheel_forces(state_g, u, self.params, self.wheels)
        fr = abs(np.dot(dr["force"][0], dr["s_world"][0]))
        fg = abs(np.dot(dg["force"][0], dg["s_world"][0]))
        self.assertGreater(fg, fr)

    def test_friction_opposes_each_slip_component(self):
        state = self.state.copy()
        state[2] = 0.4
        state[3:6] = np.array([0.71, -0.33, 0.52])
        u = np.array([3.0, -2.0, 0.5, -1.5])
        data = wm.wheel_forces(state, u, self.params, self.wheels)
        power_like = (
            np.einsum("ij,ij->i", data["force"], data["s_world"]) * data["v_W"]
            + np.einsum("ij,ij->i", data["force"], data["r_world"]) * data["v_T"]
        )
        self.assertTrue(np.all(power_like <= 1e-10))

    def test_body_command_generates_zero_slip_at_the_commanded_twist(self):
        state = self.state.copy()
        state[2] = 0.73
        command = np.array([0.62, -0.24, 0.81])

        wheel_speeds = wm.wheel_speeds_from_body_command(command, self.wheels)

        R = wm.rotation_matrix(state[2])
        state[3:5] = R @ command[:2]
        state[5] = command[2]

        data = wm.wheel_forces(state, wheel_speeds, self.params, self.wheels)
        np.testing.assert_allclose(data["v_W"], 0.0, atol=1e-12)

    def test_xy_only_command_defaults_to_zero_yaw_rate(self):
        command = np.array([0.5, -0.3])
        wheel_speeds = wm.wheel_speeds_from_body_command(command, self.wheels)
        wheel_speeds_3 = wm.wheel_speeds_from_body_command(
            np.array([0.5, -0.3, 0.0]), self.wheels
        )
        np.testing.assert_allclose(wheel_speeds, wheel_speeds_3, atol=1e-12)

    def test_zero_command_keeps_robot_at_rest(self):
        t, x, wheel_speed_history = wm.simulate_command(
            self.state,
            command_fn=lambda _t: np.array([0.0, 0.0]),
            duration=0.05,
            dt=0.001,
            params=self.params,
            wheels=self.wheels,
        )
        np.testing.assert_allclose(wheel_speed_history, 0.0, atol=1e-12)
        np.testing.assert_allclose(x, np.tile(self.state, (len(t), 1)), atol=1e-12)
        self.assertEqual(len(t), len(x))

    def test_command_changes_update_wheel_speeds_without_velocity_feedback(self):
        first = np.array([0.5, 0.0])
        second = np.array([0.0, 0.5])
        first_speeds = wm.wheel_speeds_from_body_command(first, self.wheels)
        second_speeds = wm.wheel_speeds_from_body_command(second, self.wheels)

        t, x, wheel_speed_history = wm.simulate_command(
            self.state,
            command_fn=lambda current_t: first if current_t < 0.02 else second,
            duration=0.04,
            dt=0.001,
            params=self.params,
            wheels=self.wheels,
        )

        np.testing.assert_allclose(
            wheel_speed_history[:20], np.tile(first_speeds, (20, 1)), atol=1e-12
        )
        np.testing.assert_allclose(
            wheel_speed_history[20:], np.tile(second_speeds, (len(wheel_speed_history) - 20, 1)), atol=1e-12
        )
        self.assertEqual(len(t) - 1, len(wheel_speed_history))
        self.assertEqual(len(x), len(t))

    def test_continuous_dynamics_shape_and_wheel_angle_rate(self):
        state = self.state.copy()
        u = np.array([1.0, -2.0, 3.0, -4.0])
        xdot = wm.continuous_dynamics(state, u, self.params, self.wheels)
        self.assertEqual(xdot.shape, state.shape)
        np.testing.assert_allclose(xdot[6:], u)


if __name__ == "__main__":
    unittest.main()
