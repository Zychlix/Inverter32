import numpy as np
import matplotlib.pyplot as plt

fname = 'runs/integrated_vs_speed.csv'
i, t, x, y, omega, v = np.loadtxt(fname, delimiter=",", dtype=float, skiprows=1, unpack=True)


a, b = np.polyfit(v, omega, deg=1)
print(a, b)

base = np.linspace(0, 50, 10)

plt.plot(v, omega, 'o')
plt.plot(base, a * base + b, 'k-')

plt.show()

