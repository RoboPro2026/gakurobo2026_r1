import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import trajectory_planner

fig, ax = plt.subplots(figsize=(12, 12))


def plot_field(zone):
    sign = 1.0
    if zone == "red":
        sign = -1.0
    # フィールドの外枠
    field = patches.Rectangle(
        # xyは左下の座標
        xy=(0, 0),
        width=6.0,
        height=12.0,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    # R1スタートゾーン
    r1_start_zone = patches.Rectangle(
        # xyは左下の座標
        xy=(5.0, 11.0),
        width=1.0,
        height=1.0,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    r2_start_zone = patches.Rectangle(
        # xyは左下の座標
        xy=(0.875, 11.2),
        width=0.8,
        height=0.8,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    retry_zone = patches.Rectangle(
        # xyは左下の座標
        xy=(5.0, 0.0),
        width=1.0,
        height=1.0,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    poll_rack = patches.Rectangle(
        # xyは左下の座標
        xy=(3.0, 11.7),
        width=0.8,
        height=0.3,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    head_rack = patches.Rectangle(
        # xyは左下の座標
        xy=(0.0, 10.45),
        width=0.15,
        height=1.2,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    hidensyo_rack = patches.Rectangle(
        # xyは左下の座標
        xy=(0.0, 0.437),
        width=0.16,
        height=1.626,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    yariokiba = patches.Rectangle(
        # xyは左下の座標
        xy=(0.525, 2.2),
        width=1.5,
        height=0.3,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    yariokiba = patches.Rectangle(
        # xyは左下の座標
        xy=(0.525, 2.2),
        width=1.5,
        height=0.3,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    # アリーナの柵
    arena_entrance = patches.Rectangle(
        # xyは左下の座標
        xy=(0.0, 2.2),
        width=4.025,
        height=0.05,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest1 = patches.Rectangle(
        # xyは左下の座標
        xy=(3.6, 7.6),
        width=1.2,
        height=1.2,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest1_square = patches.Rectangle(
        # xyは左下の座標
        xy=(3.6 + 0.428, 7.6 + 0.428),
        width=0.35,
        height=0.35,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest2 = patches.Rectangle(
        # xyは左下の座標
        xy=(2.4, 7.6),
        width=1.2,
        height=1.2,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest2_square = patches.Rectangle(
        # xyは左下の座標
        xy=(2.4 + 0.428, 7.6 + 0.428),
        width=0.35,
        height=0.35,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest3 = patches.Rectangle(
        # xyは左下の座標
        xy=(1.2, 7.6),
        width=1.2,
        height=1.2,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest3_square = patches.Rectangle(
        # xyは左下の座標
        xy=(1.2 + 0.428, 7.6 + 0.428),
        width=0.35,
        height=0.35,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest4 = patches.Rectangle(
        # xyは左下の座標
        xy=(3.6, 6.4),
        width=1.2,
        height=1.2,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest4_square = patches.Rectangle(
        # xyは左下の座標
        xy=(3.6 + 0.428, 6.4 + 0.428),
        width=0.35,
        height=0.35,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest5 = patches.Rectangle(
        # xyは左下の座標
        xy=(2.4, 6.4),
        width=1.2,
        height=1.2,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest5_square = patches.Rectangle(
        # xyは左下の座標
        xy=(2.4 + 0.428, 6.4 + 0.428),
        width=0.35,
        height=0.35,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest6 = patches.Rectangle(
        # xyは左下の座標
        xy=(1.2, 6.4),
        width=1.2,
        height=1.2,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest6_square = patches.Rectangle(
        # xyは左下の座標
        xy=(1.2 + 0.428, 6.4 + 0.428),
        width=0.35,
        height=0.35,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest7 = patches.Rectangle(
        # xyは左下の座標
        xy=(3.6, 5.2),
        width=1.2,
        height=1.2,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest7_square = patches.Rectangle(
        # xyは左下の座標
        xy=(3.6 + 0.428, 5.2 + 0.428),
        width=0.35,
        height=0.35,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest8 = patches.Rectangle(
        # xyは左下の座標
        xy=(2.4, 5.2),
        width=1.2,
        height=1.2,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest8_square = patches.Rectangle(
        # xyは左下の座標
        xy=(2.4 + 0.428, 5.2 + 0.428),
        width=0.35,
        height=0.35,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest9 = patches.Rectangle(
        # xyは左下の座標
        xy=(1.2, 5.2),
        width=1.2,
        height=1.2,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest9_square = patches.Rectangle(
        # xyは左下の座標
        xy=(1.2 + 0.428, 5.2 + 0.428),
        width=0.35,
        height=0.35,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest10 = patches.Rectangle(
        # xyは左下の座標
        xy=(3.6, 4.0),
        width=1.2,
        height=1.2,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest10_square = patches.Rectangle(
        # xyは左下の座標
        xy=(3.6 + 0.428, 4.0 + 0.428),
        width=0.35,
        height=0.35,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest11 = patches.Rectangle(
        # xyは左下の座標
        xy=(2.4, 4.0),
        width=1.2,
        height=1.2,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest11_square = patches.Rectangle(
        # xyは左下の座標
        xy=(2.4 + 0.428, 4.0 + 0.428),
        width=0.35,
        height=0.35,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest12 = patches.Rectangle(
        # xyは左下の座標
        xy=(1.2, 4.0),
        width=1.2,
        height=1.2,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    forest12_square = patches.Rectangle(
        # xyは左下の座標
        xy=(1.2 + 0.428, 4.0 + 0.428),
        width=0.35,
        height=0.35,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )

    ax.add_patch(field)
    ax.add_patch(r1_start_zone)
    ax.add_patch(r2_start_zone)
    ax.add_patch(retry_zone)
    ax.add_patch(poll_rack)
    ax.add_patch(head_rack)
    ax.add_patch(hidensyo_rack)
    ax.add_patch(yariokiba)
    ax.add_patch(arena_entrance)
    ax.add_patch(forest1)
    ax.add_patch(forest2)
    ax.add_patch(forest3)
    ax.add_patch(forest4)
    ax.add_patch(forest5)
    ax.add_patch(forest6)
    ax.add_patch(forest7)
    ax.add_patch(forest8)
    ax.add_patch(forest9)
    ax.add_patch(forest10)
    ax.add_patch(forest11)
    ax.add_patch(forest12)
    ax.add_patch(forest1_square)
    ax.add_patch(forest2_square)
    ax.add_patch(forest3_square)
    ax.add_patch(forest4_square)
    ax.add_patch(forest5_square)
    ax.add_patch(forest6_square)
    ax.add_patch(forest7_square)
    ax.add_patch(forest8_square)
    ax.add_patch(forest9_square)
    ax.add_patch(forest10_square)
    ax.add_patch(forest11_square)
    ax.add_patch(forest12_square)


plot_field("red")
plt.xlim(-1, 13)
plt.ylim(-1, 13)

x_wp = [5.5, 5.5, 5.5, 5.5, 5.5, 4.0, 2.5, 0.25]
y_wp = [11.5, 10.0, 9.0, 8.0, 3.5, 1.5, 1.0, 1.0]
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

# plt.figure(figsize=(8, 6))
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
plt.show()
