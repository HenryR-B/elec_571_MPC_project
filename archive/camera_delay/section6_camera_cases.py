# ControllerAnalysis section 6's camera cases (Henry's): the IMU observer with the camera on time, 50 ms late used as if
# on time, and 50 ms late replayed. Not used (no camera delay).


# ==============================================================================================================
# Camera cases
# ==============================================================================================================
if POSE_SOURCE == 'linearized':     # section 6: Henry's linearized observers (settings)
    # The IMU observer with the camera on time, with 50 ms late frames used as if on time, and with them replayed to now.
    # Each case is its own entry (the same controller with its own camera), so the plots can tell them apart.
    CONTROLLERS['camera on time'] = (CONTROLLERS['Linearized MPC observer IMU'][0], 'dimgray')
    CONTROLLERS['late, no replay'] = (CONTROLLERS['Linearized MPC observer IMU no replay'][0], 'crimson')
    CONTROLLERS['late, replayed'] = (CONTROLLERS['Linearized MPC observer IMU'][0], 'teal')
    camera_cases = {
        'camera on time':  dict(SIMULATION_PARAMATERS, camera_delay_on=False),
        'late, no replay': dict(SIMULATION_PARAMATERS, camera_delay_on=True),
        'late, replayed':  dict(SIMULATION_PARAMATERS, camera_delay_on=True),
    }
    camera_results = {}
    for label, params in camera_cases.items():
        camera_results.update(run_and_report(f"Camera: {label}", observer_paths, [label], params=params))
    camera_labels = list(camera_cases)
    plot_paths(camera_results, observer_paths, camera_labels)
    plot_errors(camera_results, observer_paths, camera_labels)
