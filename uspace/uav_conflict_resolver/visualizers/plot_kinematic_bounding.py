import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Ellipse
import os


def plot_kinematic_bounding(p0=(0.0, 0.0), v0=(5.0, 1.0), A_max=2.0, V_max=8.0,
                            T=10.0, N=20, intruder=None, save_dir=None):
    """Plot a 2D top-down kinematic bounding visualization.

    Parameters
    - p0: tuple, initial position (x,y)
    - v0: tuple, initial velocity vector (vx, vy)
    - A_max: float, maximum acceleration magnitude
    - V_max: float, speed bound (used to size OBB length)
    - T: float, time horizon
    - N: int, number of discretization steps
    - intruder: dict or None, e.g. {'p0':(x,y),'v0':(vx,vy),'color':'orange'}
    - save_dir: optional directory to save produced images. If None, file is saved
                next to this script.
    """
    p0 = np.asarray(p0, dtype=float)
    v0 = np.asarray(v0, dtype=float)
    dt = T / float(N)
    t = np.linspace(0.0, T, N + 1)

    # nominal trajectory points
    pts = p0.reshape(2, 1) + np.outer(v0, t)

    r = 0.5 * A_max * t**2  # radial growth per sample

    fig, ax = plt.subplots(figsize=(8, 6))

    # Plot nominal trajectory
    ax.plot(pts[0, :], pts[1, :], color='tab:blue', lw=2, label='Nominal trajectory')
    ax.scatter(pts[0, :], pts[1, :], color='tab:blue', s=20)

    # Orientation angle for OBBs/ellipses
    angle = np.degrees(np.arctan2(v0[1], v0[0])) if np.linalg.norm(v0) > 1e-8 else 0.0

    # Draw per-sample bounds (circles and elongated ellipses)
    for k in range(len(t)):
        x, y = pts[0, k], pts[1, k]
        rk = r[k]
        if rk <= 0:
            continue
        circ = Circle((x, y), rk, facecolor='tab:red', edgecolor='none', alpha=0.18)
        ax.add_patch(circ)

        # elongated ellipse to convey heading uncertainty (length along velocity)
        length = max(0.1, V_max * dt * 1.5)
        width = max(0.01, rk * 2.0)
        ell = Ellipse((x, y), width=length, height=width, angle=angle,
                      facecolor='tab:red', edgecolor='none', alpha=0.22)
        ax.add_patch(ell)

    # Plot velocity vector at p0
    ax.quiver(p0[0], p0[1], v0[0], v0[1], angles='xy', scale_units='xy', scale=1,
              color='tab:cyan', width=0.008, label='Velocity')

    # Optionally plot intruder and its bound (simple)
    if intruder is not None:
        ip0 = np.asarray(intruder.get('p0', (10.0, 5.0)), dtype=float)
        iv0 = np.asarray(intruder.get('v0', (-3.0, 0.0)), dtype=float)
        icolor = intruder.get('color', 'tab:orange')
        it = np.linspace(0.0, T, N + 1)
        ipts = ip0.reshape(2, 1) + np.outer(iv0, it)
        ax.plot(ipts[0, :], ipts[1, :], color=icolor, lw=2, ls='--', label='Intruder traj')
        for k in range(len(it)):
            ix, iy = ipts[0, k], ipts[1, k]
            ir = 0.5 * intruder.get('A_max', A_max) * it[k] ** 2
            if ir <= 0:
                continue
            ic = Circle((ix, iy), ir, facecolor=icolor, edgecolor='none', alpha=0.16)
            ax.add_patch(ic)

    ax.set_aspect('equal', 'box')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Kinematic Bounding — Top-down View')
    ax.grid(True, linestyle='--', alpha=0.3)
    ax.legend(loc='upper left')

    # Autoscale with margin
    all_x = pts[0, :]
    all_y = pts[1, :]
    margin = max(1.0, np.max(r) * 1.5)
    ax.set_xlim(np.min(all_x) - margin, np.max(all_x) + margin)
    ax.set_ylim(np.min(all_y) - margin, np.max(all_y) + margin)

    # Save
    if save_dir is None:
        save_dir = os.path.dirname(__file__)
    os.makedirs(save_dir, exist_ok=True)
    png_path = os.path.join(save_dir, 'kinematic_bounding_example.png')
    svg_path = os.path.join(save_dir, 'kinematic_bounding_example.svg')
    fig.savefig(png_path, dpi=200, bbox_inches='tight')
    fig.savefig(svg_path, dpi=200, bbox_inches='tight')

    print(f'Saved example figures to: {png_path} and {svg_path}')
    plt.close(fig)
    return png_path, svg_path


if __name__ == '__main__':
    # default demonstration parameters
    intr = {'p0': (30.0, 6.0), 'v0': (-4.0, -0.5), 'A_max': 1.5, 'color': 'tab:orange'}
    plot_kinematic_bounding(p0=(0.0, 0.0), v0=(6.0, 1.2), A_max=2.0, V_max=10.0,
                            T=8.0, N=32, intruder=intr, save_dir=None)
    # also print quick usage note
    print('Example complete. To customize, import plot_kinematic_bounding and call with parameters.')
