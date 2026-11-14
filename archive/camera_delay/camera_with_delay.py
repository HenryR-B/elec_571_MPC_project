# Henry's OmniRobotCamera with the delay (commit 7d17ac4): each frame handed over camera_delay late. Not used (Henry: no camera delay).
# Its parameters were: 'camera_delay_on': False, 'camera_delay': 0.05 (s), 'sigma_camera_delay': 0.0 (s).


# ==============================================================================================================
# PlantModels: OmniRobotCamera with the delay
# ==============================================================================================================
"""
The camera
==========

OmniRobotCamera is the simulation's camera: it takes the robot's true pose every camera_period and hands each frame
over late (camera_delay, when camera_delay_on). Each frame's delay is drawn from a normal distribution,
camera_delay +- sigma_camera_delay (sigma 0: every frame exactly camera_delay late), never below zero, rounded to
whole plant steps. With camera_delay_on False, each frame arrives the step it is taken.

Each frame's pose has white noise: sigma_camera_position on X and Y (m), sigma_camera_heading on theta (rad). The
defaults are the vision noise the team's robot Kalman filter is tuned with (Rustware master,
crates/robot_protocol/config/config.toml, at rest: 5 mm and 0.005 rad). There the heading noise also grows with
speed (0.01 rad at 0.1 m/s, 0.02 rad at 1 rad/s); that is not modelled here.

Call step once per plant step, with the pose at the end of that step. It returns the newest frame that has arrived
(None if no new frame arrived); last_frame_delay is how old that frame is (s), for a later delay compensation.
"""

import numpy as np


class OmniRobotCamera:
    """
    The camera: a pose frame every camera_period, each handed over camera_delay later (when switched on).
    """
    def __init__(self, params: dict, random_seed: int = 42):
        self.dt = params['dt']
        self.steps_per_frame = int(round(params['camera_period'] / self.dt))
        self.delay_on = bool(params['camera_delay_on'])
        self.delay = params['camera_delay']
        self.sigma_delay = params['sigma_camera_delay']
        self.sigma_pose = np.array([params['sigma_camera_position'], params['sigma_camera_position'],
                                    params['sigma_camera_heading']])

        # The camera's own random generator, so the robot's noise sequence is not changed
        self.rng = np.random.default_rng(seed=random_seed + 2)
        self.reset()

    def reset(self):
        self.step_count = 0
        self.in_flight = []              # frames taken, not yet handed over: (arrival step, step taken, pose)
        self.newest_taken = -1           # the step the newest handed-over frame was taken
        self.last_frame_delay = 0.0      # how old the last handed-over frame was when it arrived (s)

    def step(self, pose: np.ndarray):
        """
        One plant step. pose: the true pose [X, Y, theta] at the end of the step.
        Returns the newest frame that arrived this step, or None.
        """
        self.step_count += 1

        # 1. Take a frame every camera_period, with its noise and its delay
        if self.step_count % self.steps_per_frame == 0:
            frame = pose + self.rng.normal(0.0, 1.0, 3) * self.sigma_pose
            delay = 0.0
            if self.delay_on:
                delay = self.delay
                if self.sigma_delay > 0:
                    delay = self.rng.normal(self.delay, self.sigma_delay)
                delay = max(0.0, delay)
            delay_steps = int(round(delay / self.dt))
            self.in_flight.append((self.step_count + delay_steps, self.step_count, frame))

        # 2. Hand over the newest frame that has arrived; drop older ones (with a varying delay, frames can overtake)
        arrived = [frame for frame in self.in_flight if frame[0] <= self.step_count]
        self.in_flight = [frame for frame in self.in_flight if frame[0] > self.step_count]
        if not arrived:
            return None
        _, step_taken, frame_pose = max(arrived, key=lambda frame: frame[1])
        if step_taken <= self.newest_taken:
            return None
        self.newest_taken = step_taken
        self.last_frame_delay = (self.step_count - step_taken) * self.dt
        return frame_pose
