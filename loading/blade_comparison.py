import pandas as pd
import numpy as np

import matplotlib.pyplot as plt

# read conventional and toroidal csv files

esc = False

if esc:
    biblade = pd.read_csv('loading/data/bi-blade_esc.csv')
    triblade = pd.read_csv('loading/data/tri-blade_esc.csv')
    toroidal = pd.read_csv('loading/data/toroidal_esc.csv')
else:
    biblade = pd.read_csv('loading/data/bi-blade.csv')
    triblade = pd.read_csv('loading/data/tri-blade.csv')
    toroidal = pd.read_csv('loading/data/toroidal.csv')
    noprop = pd.read_csv('loading/data/no-blade.csv')


if not esc:
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

if esc:
    ax.loglog(biblade['Throttle (%)'], biblade['Thrust (N)'], '-o', label="Bi-Blade")
    ax.loglog(triblade['Throttle (%)'], triblade['Thrust (N)'], '-o', label="Tri-Blade")
    ax.loglog(toroidal['Throttle (%)'], toroidal['Thrust (N)'], '-o', label="Toroidal")

    x = np.linspace(3e-2, 3e0, 100)
    ax.plot(
        x, 3 * x ** 2, label="Quadratic"
    )

    ax.set_xlabel('Throttle (%)')

    

else:
    ax.loglog(biblade['Speed (RPM)'], biblade['Thrust (N)'], '-o', label="Bi-Blade")
    ax.loglog(triblade['Speed (RPM)'], triblade['Thrust (N)'], '-o', label="Tri-Blade")
    ax.loglog(toroidal['Speed (RPM)'], toroidal['Thrust (N)'], '-o', label="Toroidal")

    x = np.linspace( np.min(toroidal['Speed (RPM)']), np.max(toroidal['Speed (RPM)']), 100)
    ax.plot(
        x, 1e-8 * x ** 2, label="Quadratic"
    )

    ax.set_xlabel('Approximate Speed (RPM)')

ax.set_ylabel('Thrust (N)')

plt.legend()
plt.grid(which='both')

if esc:
    plt.savefig('loading/figures/throttle_vs_thrust.eps')

else:
    plt.savefig('loading/figures/speed_vs_thrust.eps')

plt.show()