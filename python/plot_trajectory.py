import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

# ====== CONFIG ======
filename = "logs/1obs_ree_7.csv"

target = np.array([-0.450, -0.030, 0.150])

# Obstacle
obs = np.array([-0.305, 0.108, 0.200])
axes = np.array([0.04, 0.04, 0.04])  # a, b, c

# ====== LOAD DATA ======
data = pd.read_csv(filename)

x = data["actual_TCP_pose_0"]
y = data["actual_TCP_pose_1"]
z = data["actual_TCP_pose_2"]

start = np.array([x.iloc[0], y.iloc[0], z.iloc[0]])

# ====== PLOT XY ======
plt.figure()
plt.plot(x, y, linewidth=2)

plt.scatter(start[0], start[1], marker='s', label="Start")
plt.scatter(target[0], target[1], marker='o', label="Target")

#  Obstacle (ellipse XY)
ellipse_xy = Ellipse(
    (obs[0], obs[1]),
    width=2*axes[0],
    height=2*axes[1],
    edgecolor='r',
    facecolor='none',
    linewidth=2,
    label="Obstacle"
)
plt.gca().add_patch(ellipse_xy)

plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Top View (XY Plane)")
plt.axis("equal")
plt.grid()
plt.legend()

# ====== PLOT XZ ======
plt.figure()
plt.plot(x, z, linewidth=2)

plt.scatter(start[0], start[2], marker='s', label="Start")
plt.scatter(target[0], target[2], marker='o', label="Target")

#  Obstacle (ellipse XZ)
ellipse_xz = Ellipse(
    (obs[0], obs[2]),
    width=2*axes[0],
    height=2*axes[2],
    edgecolor='r',
    facecolor='none',
    linewidth=2,
    label="Obstacle"
)
plt.gca().add_patch(ellipse_xz)

plt.xlabel("X (m)")
plt.ylabel("Z (m)")
plt.title("Side View (XZ Plane)")
plt.axis("equal")
plt.grid()
plt.legend()

plt.show()