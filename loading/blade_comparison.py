import pandas as pd
import numpy as np

import matplotlib.pyplot as plt

# read conventional and toroidal csv files

biblade = pd.read_csv('loading/bi-blade.csv')
triblade = pd.read_csv('loading/tri-blade.csv')
toroidal = pd.read_csv('loading/toroidal.csv')
noprop = pd.read_csv('loading/no-blade.csv')

# plot power vs thrust

fig, ax = plt.subplots()

ax.plot(biblade['Power (W)'], biblade['Thrust (N)'], '-o', label="Bi-Blade", linewidth=1.5, markersize=4)
ax.plot(triblade['Power (W)'], triblade['Thrust'], '-o', label="Tri-Blade", linewidth=1.5, markersize=4)
ax.plot(toroidal['Power (W)'], toroidal['Thrust'], '-o', label="Toroidal", linewidth=1.5, markersize=4)
ax.plot(np.abs(noprop['Power (W)']), noprop['Thrust (N)'], '-o', label="No Prop", linewidth=1.5, markersize=4)

ax.set_xlabel('Power (W)')
ax.set_ylabel('Thrust (N)')

plt.legend()
plt.grid()

plt.savefig('loading/power_vs_thrust.eps')
plt.show()


# speed vs thrust

fig, ax = plt.subplots()

target_speeds = np.linspace(10, 250, 10) * 60

ax.plot(target_speeds, biblade['Thrust'], '-o', label="Bi-Blade")
ax.plot(target_speeds, triblade['Thrust'], '-o', label="Tri-Blade")
ax.plot(target_speeds, toroidal['Thrust'], '-o', label="Toroidal")

ax.set_xlabel('Approximate Speed (RPM)')
ax.set_ylabel('Thrust (N)')

plt.legend()
plt.grid()

plt.savefig('loading/speed_vs_thrust.eps')
plt.show()