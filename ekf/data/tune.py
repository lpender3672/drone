import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import allantools

import ASD_to_GaussMarkovFirstOrder as ASD2GM1

def plot_allan_deviation_and_convert_gm(ax, tau, Fs, adev):
    """
    Plot Allan deviation and estimate noise parameters.
    
    Parameters:
    -----------
    ax : matplotlib axis
        Axis to plot on
    Fs : float
        Sampling frequency in Hz
    tau : array
        Time delays
    adev : array
        Allan deviation values
    
    Returns:
    --------
    ASD_to_GaussMarkovFirstOrder : class containing 1st order gauss markov parameters
    """
    # Plot the data
    ax.loglog(tau, adev, 'o', label='Allan Deviation')
    ax.set_xlabel('Delay, seconds')
    ax.set_ylabel('ASD, m/s^2')
    ax.minorticks_on()
    ax.grid(which='both', axis='both')
    
    # Estimate parameters from the Allan deviation curve
    # N (white noise): ADEV at tau = 1/sqrt(tau) extrapolated to tau=1
    # Find the slope at short tau (white noise region)
    short_tau_mask = tau < 1
    if np.sum(short_tau_mask) > 2:
        log_tau_short = np.log10(tau[short_tau_mask])
        log_adev_short = np.log10(adev[short_tau_mask])
        # Fit line: should have slope ~ -0.5 for white noise
        coeffs = np.polyfit(log_tau_short, log_adev_short, 1)
        # N = ADEV(tau=1) * sqrt(tau=1) for white noise
        adev_at_1 = 10**(coeffs[1] + coeffs[0] * 0)  # log10(1) = 0
        N = adev_at_1 * np.sqrt(1)
    else:
        # Fallback: use minimum value
        N = np.min(adev) * np.sqrt(tau[np.argmin(adev)])
    
    # B (bias instability): minimum of the Allan deviation
    min_idx = np.argmin(adev)
    B = adev[min_idx]
    Tp = tau[min_idx]  # The tau at which bias instability occurs

    gm1 = ASD2GM1.ASD_to_GaussMarkovFirstOrder(N, B, Tp, Fs)
    
    return gm1


path = 'sensor_data.csv'
print(f"Loading data from {path}")
data = pd.read_csv(path)
sample_t = data['timestamp'].values.flatten()


# Calculate Allan deviation for the IMU data
vals = np.arange(1, 10, dtype=float)  # Create an array of values from 1 to 9
# Define the time intervals for Allan deviation
#taus = np.concatenate([0.01*vals, 0.1*vals, vals, 10*vals, 100*vals])  # Concatenate to create a range of taus
taus = np.logspace(-2, 2.5, 50)

T_avg = np.mean(np.diff(sample_t))
Fs = 1/ T_avg  # Sampling frequency in Hz

data_headers = ['gyro_x', 'gyro_y', 'gyro_z',
                'accel_x', 'accel_y', 'accel_z']

fig, axes = plt.subplots(
    2, 3,
    figsize=(10, 6))

for idx, name in enumerate(data_headers):
    print(f"\n{'-'*40}")
    print(f"\nProcessing axis: {name}")

    raw_axis_data = data[name].values.flatten()
    print(f"raw_axis_data shape: {raw_axis_data.shape}")
    tau, adev, adeverr, n = allantools.oadev(raw_axis_data, rate=Fs, data_type='freq', taus=taus)
    gm1 = plot_allan_deviation_and_convert_gm(axes[idx//3, idx%3], tau, Fs, adev)

    axes[idx//3, idx%3].set_title(f'Allan Deviation - {name}')

    print(f"\nEstimated parameters for {name}:")
    print(f"Fs = {Fs:.3f}    # Sampling frequency, Hz")
    print(f"N  = {gm1.N:.5e} # ASD, m/s/s/rtHz = m/s/rtsec")
    print(f"B  = {gm1.B:.5e}   # bias instability, m/s/s")
    print(f"Tp = {gm1.Tp:.3f}    # desired delay for the GM ASD peak, seconds")



plt.tight_layout()
plt.show()