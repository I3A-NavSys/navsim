import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Circle
import os


def sample_swept_circle(center, radius, n=32, z=0.0):
    theta = np.linspace(0, 2 * np.pi, n)
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)
    z = np.full_like(x, z)
    return np.vstack([x, y, z]).T


def plot_kinematic_bounding_3d(p0=(0,0,0), v0=(6,1,0.2), A_max=2.0, V_max=10.0,
                               T=8.0, N=24, intruder=None, save_dir=None):
    p0 = np.asarray(p0, dtype=float)
    v0 = np.asarray(v0, dtype=float)
    dt = T / float(N)
    t = np.linspace(0.0, T, N + 1)

    pts = p0.reshape(3,1) + np.outer(v0, t)
    r = 0.5 * A_max * t**2

    fig = plt.figure(figsize=(10,8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot nominal 3D trajectory
    ax.plot(pts[0,:], pts[1,:], pts[2,:], color='tab:blue', lw=2, label='Nominal trajectory')
    ax.scatter(pts[0,:], pts[1,:], pts[2,:], color='tab:blue', s=20)

    # Create stacked rings (disks) along trajectory
    faces = []
    for k in range(len(t)):
        center = (pts[0,k], pts[1,k], pts[2,k])
        rk = r[k]
        if rk <= 0: continue
        ring = sample_swept_circle(center, rk, n=48, z=center[2])
        faces.append(ring)

    # Render translucent disks
    for ring in faces:
        poly = Poly3DCollection([ring], alpha=0.18, facecolor='tab:red', edgecolor='none')
        ax.add_collection3d(poly)

    # Connect convex hull-like surface between rings (simple patching)
    for i in range(len(faces)-1):
        A = faces[i]
        B = faces[i+1]
        M = A.shape[0]
        polys = []
        for j in range(M-1):
            quad = [A[j], A[j+1], B[j+1], B[j]]
            polys.append(quad)
        polyc = Poly3DCollection(polys, alpha=0.15, facecolor='tab:red', edgecolor='none')
        ax.add_collection3d(polyc)

    # Plot velocity vector at p0
    ax.quiver(p0[0], p0[1], p0[2], v0[0], v0[1], v0[2], length=1.0, color='tab:cyan', linewidth=1.5)

    # Intruder 3D
    if intruder is not None:
        ip0 = np.asarray(intruder.get('p0',(30,6,0)), dtype=float)
        iv0 = np.asarray(intruder.get('v0',(-4,0.1,0)), dtype=float)
        it = np.linspace(0.0, T, N+1)
        ipts = ip0.reshape(3,1) + np.outer(iv0, it)
        ax.plot(ipts[0,:], ipts[1,:], ipts[2,:], color='tab:orange', lw=2, ls='--', label='Intruder')
        for k in range(len(it)):
            ir = 0.5 * intruder.get('A_max', A_max) * it[k]**2
            if ir<=0: continue
            ring = sample_swept_circle((ipts[0,k], ipts[1,k], ipts[2,k]), ir, n=36)
            poly = Poly3DCollection([ring], alpha=0.12, facecolor='tab:orange', edgecolor='none')
            ax.add_collection3d(poly)

    # Axes limits and labels
    all_x = pts[0,:]
    all_y = pts[1,:]
    all_z = pts[2,:]
    margin = max(1.0, np.max(r)*1.5)
    ax.set_xlim(np.min(all_x)-margin, np.max(all_x)+margin)
    ax.set_ylim(np.min(all_y)-margin, np.max(all_y)+margin)
    ax.set_zlim(np.min(all_z)-margin, np.max(all_z)+margin)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Kinematic Bounding — 3D Perspective')
    ax.legend()

    if save_dir is None:
        save_dir = os.path.dirname(__file__)
    os.makedirs(save_dir, exist_ok=True)
    png_path = os.path.join(save_dir, 'kinematic_bounding_3d_example.png')
    svg_path = os.path.join(save_dir, 'kinematic_bounding_3d_example.svg')
    fig.savefig(png_path, dpi=200, bbox_inches='tight')
    fig.savefig(svg_path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved 3D figures to: {png_path} and {svg_path}')
    return png_path, svg_path


if __name__ == '__main__':
    intr = {'p0': (30, 6, 1.0), 'v0': (-4.0, -0.5, 0.0), 'A_max': 1.5}
    plot_kinematic_bounding_3d(p0=(0,0,0), v0=(6.0, 1.2, 0.5), A_max=2.0, V_max=10.0, T=8.0, N=28, intruder=intr)
    print('3D example complete.')
