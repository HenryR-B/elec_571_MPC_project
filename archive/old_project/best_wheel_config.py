import omni_roller_drag
from omni_roller_drag import PARAMS, Wheel
import argparse
import matplotlib.pyplot as plt
import numpy as np
# VERIFY THESE AGAINST CAD before trusting any number that comes out.
# From CAD: front wheels 30 deg from side axis = 60 deg from forward.
#           rear wheels  45 deg from side axis = 45 deg from forward
#           -> sit at 180 - 45 = 135 deg from forward in CCW convention.
# 152mm is the diameter at which the wheels touch the ground, not the outer diameter
# wheel angles I want to test are [45, 84] [-45, -84] [122, 148] [-122, -148]. i have to check that each wheel angle I have picked satisfies the condition:
# wheel angle -/+ 31.8deg must not coincide with other wheel angle -/+ 31.8deg
# ie angle1 = 84 angle2 = 122
# angle1 + 31.8 = 115.8
# angle2 - 31.8 = 90.2
# angle1 is smaller thus; angle1 + 31.8 <= angle2 +/- 31.8
FRONT_ANGLES = list(range(45, 85))
BACK_ANGLES = list(range(122,149))

all_results = []

def check_wheel_angles(angle1, angle2):
    if (angle1 < angle2):
        good = angle1 + 31.8 <= angle2 - 31.8
    else:
        good = angle2 + 31.8 <= angle1 - 31.8
    return good

def array_specs(goodness):
    avg = np.mean(goodness)
    minimum = np.min(goodness)
    maximum = np.max(goodness)

    return avg, maximum, minimum



def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", type=float, default=1.0, help="sweep speed [m/s]")
    ap.add_argument("--omega", type=float, default=0.0,
                    help="yaw rate for the main sweep [rad/s]")
    ap.add_argument("--mu-r", type=float, default=None,
                    help="override roller friction coefficient")
    ap.add_argument("--outdir", default="figures", help="output directory")
    ap.add_argument("--show", action="store_true", help="open interactive windows")
    args = ap.parse_args()

    params = dict(PARAMS)
    if args.mu_r is not None:
        params["mu_r"] = args.mu_r

    omni_roller_drag.prepare_outdir(args.outdir)

    for index_front, front_angle in enumerate(FRONT_ANGLES):
        print(f"\n{'='*80}")
        print(f"Running experiment with front angle = {front_angle}")
        for index_back, back_angle in enumerate(BACK_ANGLES):
            print(f"\n{'='*80}")
            print(f"Running experiment with back angle = {back_angle}")
            wheels = [
                    Wheel("front-left",  alpha_deg=front_angle,   L=0.076),
                    Wheel("front-right", alpha_deg=-front_angle,  L=0.076),
                    Wheel("back-left",   alpha_deg=back_angle,  L=0.076),
                    Wheel("back-right",  alpha_deg=-back_angle, L=0.076),
                ]
            
            if check_wheel_angles(front_angle, back_angle):
                # wheel angles not conflicting
                # run experiment
                
                goodness, clean, drive_norm = omni_roller_drag.run_one_wheel_angle_config(params, args, wheels)
                avg, maximum, minimum = array_specs(goodness)
                all_results.append((wheels, avg, maximum, minimum))
            else:
                #not good, wheel angles conflict
                # skip this configuration
                print(f"Skipping Configuration [Front,Back] = [{front_angle},{back_angle}]")
                
            # interpret this stage of results and keep track
    plot_performance_metrics(all_results, args.outdir, show=False)

def plot_performance_metrics(all_results, outdir, show=False):
    """
    Plots 2D heatmaps of the min, avg, and max metrics for all tested configurations.
    Highlights the absolute maximum points for each metric, plus the baseline CAD configuration,
    displaying their specific scores for the CURRENT metric being viewed.
    """
    if not all_results:
        print("\nNo valid configurations were run. Cannot generate plot.")
        return

    print(f"\nGenerating 2D Heatmap across {len(all_results)} configurations...")
    
    # 1. Extract the unique front and back angles to form our grid axes
    front_angles = sorted(list(set(res[0][0].alpha_deg for res in all_results)))
    back_angles = sorted(list(set(res[0][2].alpha_deg for res in all_results)))
    
    # 2. Initialize empty 2D arrays (filled with np.nan for the skipped configs)
    avg_grid = np.full((len(back_angles), len(front_angles)), np.nan)
    max_grid = np.full((len(back_angles), len(front_angles)), np.nan)
    min_grid = np.full((len(back_angles), len(front_angles)), np.nan)

    # 3. Populate the grids with our results
    for res in all_results:
        wheels, avg, mx, mn = res
        
        f_idx = front_angles.index(wheels[0].alpha_deg)
        b_idx = back_angles.index(wheels[2].alpha_deg)
        
        avg_grid[b_idx, f_idx] = avg
        max_grid[b_idx, f_idx] = mx
        min_grid[b_idx, f_idx] = mn

    # --- Find the coordinates of the maximum value in each grid ---
    min_peak_idx = np.unravel_index(np.nanargmax(min_grid), min_grid.shape)
    avg_peak_idx = np.unravel_index(np.nanargmax(avg_grid), avg_grid.shape)
    max_peak_idx = np.unravel_index(np.nanargmax(max_grid), max_grid.shape)

    opt_min_pt = (front_angles[min_peak_idx[1]], back_angles[min_peak_idx[0]])
    opt_avg_pt = (front_angles[avg_peak_idx[1]], back_angles[avg_peak_idx[0]])
    opt_max_pt = (front_angles[max_peak_idx[1]], back_angles[max_peak_idx[0]])

    # --- Find the coordinates for the current CAD configuration ---
    try:
        curr_f_idx = front_angles.index(60)
        curr_b_idx = back_angles.index(135)
        curr_idx = (curr_b_idx, curr_f_idx)
        curr_pt_valid = True
    except ValueError:
        curr_pt_valid = False
        print("Warning: Current configuration (60, 135) was not in the tested parameter sweep.")

    # Print the absolute winners and the baseline to the terminal
    print("\nOptimal vs Current Configurations:")
    print(f"  -> Peak Min (Worst-Case Guarantee): Front {int(opt_min_pt[0])}°, Back {int(opt_min_pt[1])}° (Score: {min_grid[min_peak_idx]:.3f})")
    print(f"  -> Peak Avg (True Generalist)     : Front {int(opt_avg_pt[0])}°, Back {int(opt_avg_pt[1])}° (Score: {avg_grid[avg_peak_idx]:.3f})")
    print(f"  -> Peak Max (Direction Specialist): Front {int(opt_max_pt[0])}°, Back {int(opt_max_pt[1])}° (Score: {max_grid[max_peak_idx]:.3f})")
    
    if curr_pt_valid:
        print(f"  -> Current Baseline Config        : Front 60°, Back 135° (Min: {min_grid[curr_idx]:.3f}, Avg: {avg_grid[curr_idx]:.3f}, Max: {max_grid[curr_idx]:.3f})\n")
    else:
        print()

    # Store the grid indices, colors, markers, and sizes 
    points_info = [
        ("Peak Min Config", opt_min_pt, min_peak_idx, 'cyan', 'o', 120),
        ("Peak Avg Config", opt_avg_pt, avg_peak_idx, 'lime', 's', 120),
        ("Peak Max Config", opt_max_pt, max_peak_idx, 'magenta', '^', 120)
    ]
    
    if curr_pt_valid:
        # Added as a slightly larger white star so it stands out from the calculated peaks
        points_info.append(("Baseline Config", (60, 135), curr_idx, 'white', '*', 220))

    # 4. Create the plot (1 row, 3 columns)
    fig, axes = plt.subplots(1, 3, figsize=(18, 5), sharey=True)
    X, Y = np.meshgrid(front_angles, back_angles)
    
    metrics = [
        ("Min Goodness Score", min_grid, axes[0]),
        ("Average Goodness Score", avg_grid, axes[1]),
        ("Max Goodness Score", max_grid, axes[2])
    ]
    
    for title, grid, ax in metrics:
        im = ax.pcolormesh(X, Y, grid, cmap='magma', shading='nearest')
        
        # Plot the optimal points + baseline on this specific subplot
        for name, pt, idx, color, marker, size in points_info:
            
            # DYNAMIC LOOKUP: Get the score for this specific configuration on the CURRENT grid!
            val = grid[idx]
            label_str = f"{name}: F={int(pt[0])}°, B={int(pt[1])}° ({val:.3f})"
            
            ax.scatter(pt[0], pt[1], color=color, marker=marker, 
                       edgecolor='black', s=size, linewidths=1.5, label=label_str, zorder=5)

        ax.set_title(title)
        ax.set_xlabel('Front Angle [deg]')
        if ax == axes[0]:
            ax.set_ylabel('Back Angle [deg]')
            
        ax.legend(loc='lower right', fontsize=9, framealpha=0.9)
        fig.colorbar(im, ax=ax, label='Score')

    plt.tight_layout()

    # Save and optionally display
    filepath = f"{outdir}/metrics_heatmap.png"
    plt.savefig(filepath, dpi=300)
    print(f"Plot saved to: {filepath}")
    
    if show:
        plt.show()
    plt.close()

if __name__ == "__main__":
    main()