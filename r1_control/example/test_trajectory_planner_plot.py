import csv

import matplotlib.pyplot as plt
import numpy as np

t = []
x = []
y = []
theta = []
distance = []
v_trans = []
a_trans = []
j_trans = []
omega = []
curvature = []

x_wp = []
y_wp = []
theta_wp = []
v_trans_wp = []

with open("test_trajectory_planner_output.csv", "r") as f:
    reader = csv.reader(f)
    for row in reader:
        t.append(float(row[0]))
        x.append(float(row[1]))
        y.append(float(row[2]))
        theta.append(float(row[3]))
        distance.append(float(row[4]))
        v_trans.append(float(row[5]))
        a_trans.append(float(row[6]))
        j_trans.append(float(row[7]))
        omega.append(float(row[8]))
        curvature.append(float(row[9]))

with open("test_trajectory_planner_waypoint_output.csv", "r") as f:
    reader = csv.reader(f)
    for row in reader:
        x_wp.append(float(row[0]))
        y_wp.append(float(row[1]))
        theta_wp.append(float(row[2]) if row[2] != "" else np.inf)
        v_trans_wp.append(float(row[3]) if row[3] != "" else np.inf)

plt.figure(figsize=(8, 6))
scatter = plt.scatter(x, y, c=v_trans, cmap="viridis", s=15, label="Trajectory")
plt.scatter(x_wp, y_wp, color="red", marker="x", s=40, label="Waypoints", zorder=5)
plt.colorbar(scatter, label="Translational Velocity (v_trans)")
plt.xlabel("Position (x)")
plt.ylabel("Position (y)")
plt.legend()
plt.grid(True, linestyle="--", alpha=0.3)

fig, axes = plt.subplots(9, 1, sharex=True, figsize=(10, 16))

axes[0].plot(t, x, label="Position (x)")
axes[0].set_ylabel("x [m]")

axes[1].plot(t, y, label="Position (y)", color="orange")
axes[1].set_ylabel("y [m]")

axes[2].plot(t, theta, label="Orientation (theta)", color="green")
axes[2].set_ylabel("theta [rad]")

axes[3].plot(t, distance, label="Distance", color="purple")
axes[3].set_ylabel("dist [m]")

axes[4].plot(t, v_trans, label="Translational Velocity (v_trans)", color="brown")
axes[4].set_ylabel("v [m/s]")

axes[5].plot(t, a_trans, label="Translational Acceleration (a_trans)", color="red")
axes[5].set_ylabel("a [m/s^2]")

axes[6].plot(t, j_trans, label="Translational Jerk (j_trans)", color="gray")
axes[6].set_ylabel("j [m/s^3]")

axes[7].plot(t, omega, label="Angular Velocity (omega)", color="pink")
axes[7].set_ylabel("omega [rad/s]")

axes[8].plot(t, curvature, label="Curvature", color="black")
axes[8].set_ylabel("curv [1/m]")
axes[8].set_xlabel("Time (s)")

for ax in axes[:-1]:
    ax.label_outer()  # 上8つは x ラベルを隠して間を詰める

fig.tight_layout(h_pad=0.2)
plt.show()
