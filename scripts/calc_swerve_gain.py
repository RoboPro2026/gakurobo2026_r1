# サーボの旋回用モータのPIDゲインを計算するスクリプト

J = [0.001838, 0.002829, 0.001680, 0.001742]
D = [0.015188, 0.010747, 0.007198, 0.016644]
OMEGA_N = [314.159271, 314.159271, 314.159271, 314.159271]

# モータの出力軸からアブソまでの減速比
GEAR_RATIO = 3.0

# 減速機構がある→トルクが増幅される→モータから見た負荷が低下する byさばね
# JとDをギア比で割る
for i in range(4):
    # ギア比で割るのか、かけるのかはよくわからないので、コメントアウトして切り替えられるようにしてある。
    # J[i] /= GEAR_RATIO
    # D[i] /= GEAR_RATIO
    J[i] *= GEAR_RATIO
    D[i] *= GEAR_RATIO

# PIDゲインの計算
K_vp = [0.0] * 4
K_vi = [0.0] * 4
K_vd = [0.0] * 4
K_pp = [0.0] * 4
K_pi = [0.0] * 4
K_pd = [0.0] * 4

for i in range(4):
    K_vp[i] = OMEGA_N[i] * J[i]
    K_vi[i] = OMEGA_N[i] * D[i]
    K_pp[i] = OMEGA_N[i] / 4.0

# 結果の出力
res = ""
# 速度Pゲイン
res += "speed_gain_p: ["
for i in range(4):
    res += f"{K_vp[i]}"
    if i < 3:
        res += ", "
res += "]\n"
# 速度Iゲイン
res += "speed_gain_i: ["
for i in range(4):
    res += f"{K_vi[i]}"
    if i < 3:
        res += ", "
res += "]\n"
# 速度Dゲイン
res += "speed_gain_d: ["
for i in range(4):
    res += f"{K_vd[i]}"
    if i < 3:
        res += ", "
res += "]\n"
# 位置Pゲイン
res += "position_gain_p: ["
for i in range(4):
    res += f"{K_pp[i]}"
    if i < 3:
        res += ", "
res += "]\n"
# 位置Iゲイン
res += "position_gain_i: ["
for i in range(4):
    res += f"{K_pi[i]}"
    if i < 3:
        res += ", "
res += "]\n"
# 位置Dゲイン
res += "position_gain_d: ["
for i in range(4):
    res += f"{K_pd[i]}"
    if i < 3:
        res += ", "
res += "]\n"

print(res)
