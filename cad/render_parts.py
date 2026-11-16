"""Quick 3D preview of the assembly -> figures/parts_iso.png (matplotlib, no CAD GUI).

    uv run --python 3.12 --with cadquery --with shapely --with matplotlib python cad/render_parts.py [theta_deg]
"""
import math, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "build_parts.py")).read().split("# " + "-" * 64 + " outputs")[0]
g = {"__file__": os.path.join(HERE, "build_parts.py")}
exec(src, g)                                    # parts + placement only, no export
th = math.radians(float(sys.argv[1]) if len(sys.argv) > 1 else 40)
P = g["placed"](th)
colors = dict(base_plate=(0.6, 0.7, 0.8, 1), axle_stub=(0.3, 0.3, 0.3, 1), pivot_post=(0.3, 0.3, 0.3, 1), disc=(0.92, 0.92, 0.96, 1), slider=(0.85, 0.7, 0.45, 1),
              hopper=(0.4, 0.4, 0.45, 1), front_cover=(0.6, 0.85, 1.0, 0.25), rocker=(0.8, 0.2, 0.8, 1),
              pinion=(0.2, 0.6, 0.2, 1), motor_bracket=(0.3, 0.3, 0.3, 1), motor_TT=(0.95, 0.8, 0.1, 1))

# All triangles in one collection so matplotlib depth-sorts them together.
# Plot axes: (x, -z, y) so the wall is vertical and the viewer stands at +z.
T, FC = [], []
for name, s in P.items():
    v, f = s.val().tessellate(0.3, 0.3)
    tri = np.array([[(v[i].x, v[i].y, v[i].z) for i in t] for t in f])
    nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    lit = 0.45 + 0.55 * np.abs(nrm @ np.array([-0.4, 0.5, 0.77]))   # simple two-sided diffuse
    c = np.array(colors[name])
    FC.append(np.column_stack([np.outer(lit, c[:3]).clip(0, 1), np.full(len(lit), c[3])]))
    T.append(tri[:, :, [0, 2, 1]] * np.array([1, -1, 1]))
T, FC = np.concatenate(T), np.concatenate(FC)

fig = plt.figure(figsize=(15, 10))
for n, (elev, azim, title) in enumerate([(18, -62, "iso"), (0, -90, "front (wall view)")]):
    ax = fig.add_subplot(1, 2, n + 1, projection="3d", proj_type="ortho")
    ax.add_collection3d(Poly3DCollection(T, facecolors=FC, edgecolors="none"))
    ax.set_xlim(-82, 128); ax.set_ylim(-60, 60); ax.set_zlim(-92, 205)
    ax.set_box_aspect((210, 120, 297))
    ax.view_init(elev=elev, azim=azim); ax.set_title(f"{title}, theta = {math.degrees(th):.0f} deg")
    ax.set_axis_off()
fig.tight_layout()
out = os.path.join(os.path.dirname(HERE), "figures", "parts_iso.png")
fig.savefig(out, dpi=110)
print("Wrote", out)
