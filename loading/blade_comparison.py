import pandas as pd
import numpy as np

import matplotlib.pyplot as plt

# read conventional and toroidal csv files

biblade = pd.read_csv('loading/bi-blade.csv')
triblade = pd.read_csv('loading/tri-blade.csv')
toroidal = pd.read_csv('loading/toroidal.csv')

# plot power vs thrust

fig, ax = plt.subplots()

ax.plot(biblade['Power'], biblade['Thrust'], '-o', label="Bi-Blade")
ax.plot(triblade['Power'], triblade['Thrust'], '-o', label="Tri-Blade")
ax.plot(toroidal['Power'], toroidal['Thrust'], '-o', label="Toroidal")

ax.set_xlabel('Power (W)')
ax.set_ylabel('Thrust (N)')

plt.legend()

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

plt.show()