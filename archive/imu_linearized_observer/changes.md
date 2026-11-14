# Edits to existing cells for the IMU and the linearized observers (11-13)

(11-14: the IMU plant, its two lines in the plant and its parameters are in merged/PlantModels, with the `imu_on` switch.)

**ControllerModels, `MPCControllerLinearized` (cell 96fdc523)**: `imu` option (the `ekf` option is archived: archive/ekf_linearized/); the observer is chosen from them, and `estimator_update` takes the camera pose (instead of a `camera_frame` flag) and the IMU readings.
```python
    def __init__(self, simulation_params: dict, substeps: int = 20, state_estimate: str = 'pose', dt: float = 0.02,
                 imu: bool = False, ekf: bool = False, terminal_cost: str = 'riccati',
                 **mpc_options):
        ...
        self.imu = imu or ekf                      # the EKF takes the IMU readings too
        if ekf:
            self.state_observer = RobotObserverEKF(simulation_params)
        else:
            observer_class = RobotObserver1msIMU if imu else RobotObserver1ms
            self.state_observer = observer_class(simulation_params)

    def estimator_update(self, measured_State: OmniRobotObservableState, camera_pose: np.ndarray = None):
        if self.imu:
            self.state_observer.update(measured_State.V_actual, measured_State.omega, measured_State.I, camera_pose,
                                       measured_State.imu_accel, measured_State.imu_yaw_rate)
        else:
            self.state_observer.update(measured_State.V_actual, measured_State.omega, measured_State.I, camera_pose)
```
Before: `estimator_update(self, measured_State, camera_frame: bool)`, which took the pose from `measured_State.pos_global` when `camera_frame` was True. The class docstring's `'observer'` line and the ControllerModels intro got one line each about the options.

**ControllerAnalysis, controller list (settings cell)**, added:
```python
    # MERGED (Henry's 7d17ac4): the observer options with the IMU, and the Kalman filter
    'Linearized MPC observer IMU':           (lambda: MPCControllerLinearized(p, state_estimate='observer', imu=True, **mpc_settings), 'teal'),
```

**ControllerAnalysis, globals cell (37a9c88d)** and **runner**:
```python
global OmniRobotSimulationODESolveIMU, OmniRobotCamera   # MERGED (Henry's 7d17ac4): the IMU plant and the camera
    plant = OmniRobotSimulationODESolveIMU(params, random_seed=42)     # MERGED (7d17ac4): with the IMU (same motion)   # was OmniRobotSimulationODESolve(SIMULATION_PARAMATERS, ...)
```
