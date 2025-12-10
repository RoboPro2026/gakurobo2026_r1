import csv

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import trajectory_planner
from matplotlib.transforms import Affine2D
from matplotlib.widgets import Button

fig, ax = plt.subplots(figsize=(12, 12))

zone = "blue"  # "red" または "blue"
assert zone in ["red", "blue"], "zone must be 'red' or 'blue'"


# memo
# 5.5,11.5,1.5707963267948966,0.0
# 5.5,8.0,1.5707963267948966,5.0
# 4.0,1.5,1.5707963267948966,5.0
# 2.5,1.0,2.08,2.0
# 1.25,1.0,3.14,0.0


def plot_object_with_zone(field_object):
    """オブジェクトをゾーンに応じてプロット"""
    # 赤ゾーンの場合はY軸を中心に線対称に移動
    # 数式で表すと、x' = -(x + w) となる
    if zone == "red":
        if isinstance(field_object, patches.Rectangle):
            # Rectangleはset_x/get_xで位置を更新する
            mirrored_x = -(field_object.get_x() + field_object.get_width())
            field_object.set_x(mirrored_x)
    # プロット
    ax.add_patch(field_object)


def plot_field():
    """フィールドの描画"""
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
    # 秘伝書の仕切り描画用
    hidensyo_rack1 = patches.Rectangle(
        # xyは左下の座標
        xy=(0.0, 0.437),
        width=0.16,
        height=0.542,
        linewidth=1,
        edgecolor="black",
        facecolor="none",
    )
    # 秘伝書の仕切り描画用
    hidensyo_rack2 = patches.Rectangle(
        # xyは左下の座標
        xy=(0.0, 0.437),
        width=0.16,
        height=1.084,
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
        xy=(0.0, 2.5),
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

    # オブジェクトをプロット
    plot_object_with_zone(field)
    plot_object_with_zone(r1_start_zone)
    plot_object_with_zone(r2_start_zone)
    plot_object_with_zone(retry_zone)
    plot_object_with_zone(poll_rack)
    plot_object_with_zone(head_rack)
    plot_object_with_zone(hidensyo_rack)
    plot_object_with_zone(hidensyo_rack1)
    plot_object_with_zone(hidensyo_rack2)
    plot_object_with_zone(yariokiba)
    plot_object_with_zone(arena_entrance)
    plot_object_with_zone(forest1)
    plot_object_with_zone(forest2)
    plot_object_with_zone(forest3)
    plot_object_with_zone(forest4)
    plot_object_with_zone(forest5)
    plot_object_with_zone(forest6)
    plot_object_with_zone(forest7)
    plot_object_with_zone(forest8)
    plot_object_with_zone(forest9)
    plot_object_with_zone(forest10)
    plot_object_with_zone(forest11)
    plot_object_with_zone(forest12)
    plot_object_with_zone(forest1_square)
    plot_object_with_zone(forest2_square)
    plot_object_with_zone(forest3_square)
    plot_object_with_zone(forest4_square)
    plot_object_with_zone(forest5_square)
    plot_object_with_zone(forest6_square)
    plot_object_with_zone(forest7_square)
    plot_object_with_zone(forest8_square)
    plot_object_with_zone(forest9_square)
    plot_object_with_zone(forest10_square)
    plot_object_with_zone(forest11_square)
    plot_object_with_zone(forest12_square)


x_wp = []
y_wp = []
theta_wp = []
v_trans_wp = []


def load_waypoint():
    with open("waypoints.csv", "r") as f:
        reader = csv.reader(f)
        for row in reader:
            x_wp.append(float(row[0]))
            y_wp.append(float(row[1]))
            theta_wp.append(float(row[2]))
            v_trans_wp.append(float(row[3]))


def save_waypoint():
    with open("waypoints.csv", "w", newline="") as f:
        writer = csv.writer(f)
        for i in range(len(x_wp)):
            writer.writerow([x_wp[i], y_wp[i], theta_wp[i], v_trans_wp[i]])


def plot_robot(ax, x, y, theta, width, height):

    # 中心を原点にした矩形
    rect = patches.Rectangle(
        (-width / 2, -height / 2),
        width,
        height,
        facecolor="none",
        edgecolor="blue",
        linewidth=2,
    )

    # 回転 → 平行移動
    trans = Affine2D().rotate(theta).translate(x, y) + ax.transData  # <-- ラジアン！

    rect.set_transform(trans)
    ax.add_patch(rect)

    # 正面がわかるように三角マーカーを追加
    nose = patches.Polygon(
        [
            (width / 2, 0.0),
            (width / 2 - width * 0.3, height / 4),
            (width / 2 - width * 0.3, -height / 4),
        ],
        closed=True,
        facecolor="red",
        edgecolor="red",
    )
    nose.set_transform(trans)
    ax.add_patch(nose)

    print(f"Robot position: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")


# フィールドを描画、ゾーンに応じて範囲を変える
plot_field()
if zone == "blue":
    # Xが正の部分をプロットする
    plt.xlim(-1, 13)
    plt.ylim(-1, 13)
else:
    # Xが負の部分をプロットする
    plt.xlim(-13, 1)
    plt.ylim(-1, 13)

# ax_save = plt.axes([0.25, 0.03, 0.2, 0.08])
# ax_generate = plt.axes([0.55, 0.03, 0.2, 0.08])

# btn_save = Button(ax_save, "保存 (CSV)")
# btn_generate = Button(ax_generate, "生成 (Trajectory)")

# btn_save.on_clicked(save_csv)
# btn_generate.on_clicked(generate_trajectory)

load_waypoint()

dt = 0.01
v_max = 5.0
a_max = 5.0
j_max = 10.0
print_robot_dt = 0.5
robot_x = []
robot_y = []
robot_theta = []
robot_width = 0.8
robot_height = 0.8

save_waypoint()

# 赤ゾーンの場合はY軸を中心に線対称に移動
if zone == "red":
    for i in range(len(x_wp)):
        x_wp[i] = -x_wp[i]
        theta_wp[i] = np.pi - theta_wp[i]

# 軌道の計算
t, x, y, theta, distance, v_trans, a_trans, j_trans, omega, curvature = (
    trajectory_planner.calculate_trajectory(
        x_wp, y_wp, theta_wp, v_trans_wp, dt, v_max, a_max, j_max
    )
)

# 描画用のロボットの位置を取得
for i in range(0, len(t), int(print_robot_dt / dt)):
    robot_x.append(x[i])
    robot_y.append(y[i])
    robot_theta.append(theta[i])

# 最後の位置も追加
robot_x.append(x[-1])
robot_y.append(y[-1])
robot_theta.append(theta[-1])

# ロボットをプロット
for i in range(len(robot_x)):
    plot_robot(ax, robot_x[i], robot_y[i], robot_theta[i], robot_width, robot_height)

# 軌道をプロット
# plt.figure(figsize=(8, 6))
scatter = plt.scatter(x, y, c=v_trans, cmap="viridis", s=15, label="Trajectory")
plt.scatter(x_wp, y_wp, color="red", marker="x", s=40, label="Waypoints", zorder=5)
plt.colorbar(scatter, label="Translational Velocity (v_trans)")
plt.xlabel("Position (x)")
plt.ylabel("Position (y)")
plt.legend()
plt.grid(True, linestyle="--", alpha=0.3)

# 台形制御の詳細をプロット
plt.figure(figsize=(10, 12))
plt.subplot(6, 1, 1)
plt.plot(t, x, label="Position (x)")
plt.ylabel("Position (x)")
plt.subplot(6, 1, 2)
plt.plot(t, y, label="Position (y)", color="orange")
plt.ylabel("Position (y)")
plt.subplot(6, 1, 3)
plt.plot(t, theta, label="Orientation (theta)", color="green")
plt.ylabel("Orientation (theta)")
plt.subplot(6, 1, 4)
plt.plot(t, distance, label="Distance", color="purple")
plt.ylabel("Distance")
plt.subplot(6, 1, 5)
plt.plot(t, v_trans, label="Translational Velocity (v_trans)", color="brown")
plt.ylabel("Translational Velocity (v_trans)")
plt.subplot(6, 1, 6)
plt.plot(t, omega, label="Angular Velocity (omega)", color="pink")
plt.ylabel("Angular Velocity (omega)")
plt.xlabel("Time (s)")
plt.tight_layout()
plt.show()
plt.show()
