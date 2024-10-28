import pandas as pd
import numpy as np

import matplotlib.pyplot as plt

# read conventional and toroidal csv files

biblade = pd.read_csv('loading/data/bi-blade.csv')
#biblade = pd.read_csv('loading/data/bi-blade_hysteresis_ODV.csv')
triblade = pd.read_csv('loading/data/tri-blade.csv')
toroidal = pd.read_csv('loading/data/toroidal.csv')
noprop = pd.read_csv('loading/data/no-blade.csv')

# plot power vs thrust

fig, ax = plt.subplots()

ax.plot(biblade['Power (W)'], biblade['Thrust (N)'], '-o', label="Bi-Blade", linewidth=1.5, markersize=4)
ax.plot(triblade['Power (W)'], triblade['Thrust (N)'], '-o', label="Tri-Blade", linewidth=1.5, markersize=4)
ax.plot(toroidal['Power (W)'], toroidal['Thrust (N)'], '-o', label="Toroidal", linewidth=1.5, markersize=4)
ax.plot(np.abs(noprop['Power (W)']), noprop['Thrust (N)'], '-o', label="No Prop", linewidth=1.5, markersize=4)

ax.set_xlabel('Power (W)')
ax.set_ylabel('Thrust (N)')

plt.legend()
plt.grid()

plt.savefig('loading/figures/power_vs_thrust.eps')
plt.show()


# speed vs thrust

fig, ax = plt.subplots()

target_speeds = np.linspace(10, 250, 8) * 60
target_speeds = np.concatenate([target_speeds, target_speeds[::-1]])

ax.loglog(target_speeds, biblade['Thrust (N)'], '-o', label="Bi-Blade")
ax.loglog(target_speeds, triblade['Thrust (N)'], '-o', label="Tri-Blade")
ax.loglog(target_speeds, toroidal['Thrust (N)'], '-o', label="Toroidal")

x = np.linspace(10, 250, 100) * 60
ax.plot(
    x, 1e-8 * x ** 2, label="Quadratic"
)

ax.set_xlabel('Approximate Speed (RPM)')
ax.set_ylabel('Thrust (N)')

plt.legend()
plt.grid()

plt.savefig('loading/figures/speed_vs_thrust.eps')
plt.show()