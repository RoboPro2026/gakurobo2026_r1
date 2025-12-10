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
        theta_wp.append(float(row[2]))
        v_trans_wp.append(float(row[3]))

plt.figure(figsize=(8, 6))
scatter = plt.scatter(x, y, c=v_trans, cmap="viridis", s=15, label="Trajectory")
plt.scatter(x_wp, y_wp, color="red", marker="x", s=40, label="Waypoints", zorder=5)
plt.colorbar(scatter, label="Translational Velocity (v_trans)")
plt.xlabel("Position (x)")
plt.ylabel("Position (y)")
plt.legend()
plt.grid(True, linestyle="--", alpha=0.3)

plt.figure(figsize=(10, 12))
plt.subplot(6, 1, 1)
plt.plot(t, x, label="Position (x)")
plt.ylabel("Position (x)")
# plt.grid()
plt.subplot(6, 1, 2)
plt.plot(t, y, label="Position (y)", color="orange")
plt.ylabel("Position (y)")
# plt.grid()
plt.subplot(6, 1, 3)
plt.plot(t, theta, label="Orientation (theta)", color="green")
plt.ylabel("Orientation (theta)")
# plt.grid()
plt.subplot(6, 1, 4)
plt.plot(t, distance, label="Distance", color="purple")
plt.ylabel("Distance")
# plt.grid()
plt.subplot(6, 1, 5)
plt.plot(t, v_trans, label="Translational Velocity (v_trans)", color="brown")
plt.ylabel("Translational Velocity (v_trans)")
# plt.grid()
plt.subplot(6, 1, 6)
plt.plot(t, omega, label="Angular Velocity (omega)", color="pink")
plt.ylabel("Angular Velocity (omega)")
plt.xlabel("Time (s)")
# plt.grid()
plt.tight_layout()
plt.show()
