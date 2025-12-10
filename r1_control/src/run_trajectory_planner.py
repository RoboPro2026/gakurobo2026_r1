import matplotlib.pyplot as plt
import numpy as np
import trajectory_planner

x_wp = [0.0, 3.0, 6.0, 9.0, 12.0]
y_wp = [0.0, 5.0, 0.5, 2.5, 7.0]
theta_wp = [0.0, np.pi / 6, np.pi / 4, np.pi / 3, np.pi / 2]
v_trans_wp = [0.0, 5.0, 5.0, 5.0, 0.0]
dt = 0.01
v_max = 5.0
a_max = 5.0
j_max = 10.0

t, x, y, theta, distance, v_trans, a_trans, j_trans, omega, curvature = (
    trajectory_planner.calculate_trajectory(
        x_wp, y_wp, theta_wp, v_trans_wp, dt, v_max, a_max, j_max
    )
)

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
