# From robot_model_voltage.ipynb, section 4.7 (cell toe02)

if RUN_PLANT_CHECKS:
    # 4.7 Non-tangential wheels: the same geometry functions, matrices, and kinematics for any layout.
    TOE_LAYOUT = {"position": [30.0, 150.0, 225.0, 315.0], "alpha": [45.0, 135.0, 225.0, 315.0]}
    TOE_COMMANDS = [(f"forward {TEST_FORWARD_SPEED} m/s", body_command(forward=TEST_FORWARD_SPEED)),
                    (f"left {TEST_LATERAL_SPEED} m/s", body_command(left=TEST_LATERAL_SPEED)),
                    (f"yaw {TEST_YAW_RATE} rad/s", body_command(yaw_rate=TEST_YAW_RATE))]
    saved_layout = (list(rc.WHEEL_POSITION_DEG), list(rc.WHEEL_ALPHA_DEG))
    toe_runs = {}
    try:
        for layout in ("tangential", "non-tangential"):
            if layout == "non-tangential":
                rc.WHEEL_POSITION_DEG, rc.WHEEL_ALPHA_DEG = list(TOE_LAYOUT["position"]), list(TOE_LAYOUT["alpha"])
            drive, axle = wheel_directions_body()
            positions = wheel_positions_body()
            drive_levers = positions[0] * drive[1] - positions[1] * drive[0]
            axle_levers = positions[0] * axle[1] - positions[1] * axle[0]
            print(f"{layout}: position {rc.WHEEL_POSITION_DEG} deg, alpha {rc.WHEEL_ALPHA_DEG} deg")
            for i, name in enumerate(rc.WHEEL_NAMES):
                print(f"  {name:12s} r = [{positions[0, i]:+.4f}, {positions[1, i]:+.4f}] m, d = [{drive[0, i]:+.3f}, {drive[1, i]:+.3f}], "
                      f"drive lever r x d = {drive_levers[i]:+.4f} m, roller lever r x a = {axle_levers[i]:+.4f} m")
            # 1. Kinematics round trip, and the plant's H uses the same levers.
            for _, twist in TOE_COMMANDS:
                assert np.allclose(forward_kinematics(inverse_kinematics(twist)), twist, atol=1e-12)
            with centred_cog():
                probe_layout = OpenLoopPlant(load_model="equal").plant
            assert np.allclose(probe_layout.H[2], drive_levers) and np.allclose(probe_layout.positions, positions)
            # 2. Open-loop commands with only the layout acting.
            for label, command in TOE_COMMANDS:
                with centred_cog():
                    log = run_open_loop_schedule(lambda t, c=command: c, 2.0, load_model="equal")
                final = log["body_velocity"][-20:].mean(axis=0)
                toe_runs[(layout, label)] = log
                gripping = log["rolling"][1:].all(axis=1)
                kinematic = np.array([forward_kinematics(w) for w in log["wheel_speeds"][1:][gripping]])
                kinematic_gap = np.max(np.abs(kinematic - log["body_velocity"][1:][gripping])) if gripping.any() else np.nan
                error = np.linalg.norm(final - command) / np.linalg.norm(command)
                print(f"  {label:16s}: final twist [u, v, omega] = {np.round(final, 4)} vs command {command} "
                      f"(servo error {100 * error:.1f}%); |twist - FK(wheel speeds)| while gripping {kinematic_gap:.1e}; "
                      f"sliding samples per wheel {(~log['rolling'][1:]).sum(axis=0)}")
                # LuGre wheels creep slightly under load (LUGRE_CREEP_SLIP), so their grip is not exact.
                kinematic_tolerance = 1e-3 if DRIVE_SLIDING_LAW == "lugre" else 1e-4
                assert gripping.any() and kinematic_gap < kinematic_tolerance, f"{layout}: chassis twist and wheel kinematics disagree for {label}"
    finally:
        rc.WHEEL_POSITION_DEG, rc.WHEEL_ALPHA_DEG = saved_layout
    print("PASS: kinematics, H, and the plant's rolling constraint agree for tangential and non-tangential layouts.")

    fig, axes = plt.subplots(1, 3, figsize=(17, 4.2))
    for ax, (label, command) in zip(axes, TOE_COMMANDS):
        for layout, style in (("tangential", "--"), ("non-tangential", "-")):
            log = toe_runs[(layout, label)]
            for k, (name, color) in enumerate((("u (right)", "C0"), ("v (forward)", "C1"), ("omega", "C2"))):
                ax.plot(log["time"], log["body_velocity"][:, k], color + style, label=f"{name}, {layout}")
        ax.set(xlabel="Time (s)", ylabel="Body velocity (m/s, rad/s)", title=f"Command: {label}")
        ax.grid(True, alpha=0.3)
    axes[0].legend(fontsize=7)
    plt.tight_layout(); plt.show()
else:
    print("Plant check skipped (RUN_PLANT_CHECKS = False).")