import csv

import matplotlib.pyplot as plt
import numpy as np

t = []
x = []
v = []
a = []
j = []
with open("test_trapezoid_output.csv", "r") as f:
    reader = csv.reader(f)
    for row in reader:
        t.append(float(row[0]))
        x.append(float(row[1]))
        v.append(float(row[2]))
        a.append(float(row[3]))
        j.append(float(row[4]))

plt.figure(figsize=(10, 8))
plt.subplot(4, 1, 1)
plt.plot(t, x, label="Position (x)")
plt.ylabel("Position (x)")
plt.grid()
plt.subplot(4, 1, 2)
plt.plot(t, v, label="Velocity (v)", color="orange")
plt.ylabel("Velocity (v)")
plt.grid()
plt.subplot(4, 1, 3)
plt.plot(t, a, label="Acceleration (a)", color="green")
plt.ylabel("Acceleration (a)")
plt.grid()
plt.subplot(4, 1, 4)
plt.plot(t, j, label="Jerk (j)", color="red")
plt.ylabel("Jerk (j)")
plt.xlabel("Time (s)")
plt.grid()
plt.tight_layout()
plt.show()
