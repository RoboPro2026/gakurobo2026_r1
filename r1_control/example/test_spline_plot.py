import csv

import matplotlib.pyplot as plt
import numpy as np

x_point = []
y_point = []
with open("test_spline_point.csv", "r") as f:
    reader = csv.reader(f)
    for row in reader:
        x_point.append(float(row[0]))
        y_point.append(float(row[1]))

x = []
y = []
curvature = []

with open("test_spline.csv", "r") as f:
    reader = csv.reader(f)
    for row in reader:
        x.append(float(row[0]))
        y.append(float(row[1]))
        curvature.append(float(row[2]))

t = np.linspace(0, 1, len(x))

plt.figure(figsize=(10, 8))
plt.subplot(2, 1, 1)
plt.plot(x, y, label="Spline Curve")
plt.scatter(x_point, y_point, color="red", label="Control Points")
plt.ylabel("Y")
plt.xlabel("X")
plt.title("Cubic Spline Interpolation")
plt.legend()
# plt.grid()
plt.subplot(2, 1, 2)
plt.plot(t, np.array(curvature), label="Curvature", color="green")
plt.ylabel("Curvature")
plt.title("Curvature along the Spline")
plt.legend()
# plt.grid()
plt.tight_layout()
plt.show()
