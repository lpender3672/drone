import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import allantools
from pathlib import Path
import struct
from typing import Dict, Iterable, Iterator, Optional, Tuple

import ASD_to_GaussMarkovFirstOrder as ASD2GM1

G_TO_MS2 = 9.80665


RecordHeader = Tuple[int, int, int, int]  # (t_ms, len, truncated, reserved)


def iter_sensorbase_bin_records(path: Path) -> Iterator[Tuple[int, bytes, bool]]:
    """Iterate SensorBase records from a .bin file.

    Format (little-endian):
      uint32 t_ms
      uint16 len
      uint8  truncated
      uint8  reserved
      then `len` bytes of payload.
    """
    header_struct = struct.Struct('<I H B B')
    with path.open('rb') as f:
        while True:
            hdr_bytes = f.read(header_struct.size)
            if not hdr_bytes:
                return
            if len(hdr_bytes) != header_struct.size:
                return
            t_ms, payload_len, truncated, _reserved = header_struct.unpack(hdr_bytes)
            payload = f.read(payload_len)
            if len(payload) != payload_len:
                return
            yield t_ms, payload, bool(truncated)


def load_teensy_imu_bin(path: Path) -> pd.DataFrame:
    # struct ImuLogSample { float acc_g[3]; float gyro_rads[3]; float dt; }
    payload_struct = struct.Struct('<7f')
    rows = []
    for t_ms, payload, _truncated in iter_sensorbase_bin_records(path):
        if len(payload) < payload_struct.size:
            continue
        acc_x_g, acc_y_g, acc_z_g, gyro_x, gyro_y, gyro_z, dt = payload_struct.unpack_from(payload, 0)
        rows.append((t_ms * 1e-3, gyro_x, gyro_y, gyro_z, acc_x_g, acc_y_g, acc_z_g, dt))
    return pd.DataFrame(
        rows,
        columns=['timestamp', 'gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z', 'dt'],
    )


def load_teensy_baro_bin(path: Path) -> pd.DataFrame:
    # Current format (2025-12):
    #   struct BaroLogSample { float alt_relative_m; float pressure_pa; float temp_c; float var; }
    # Old format (kept for back-compat):
    #   struct BaroLogSample { float alt_relative_m; float var; }
    payload_struct_v2 = struct.Struct('<4f')
    payload_struct_v1 = struct.Struct('<2f')

    rows = []
    for t_ms, payload, _truncated in iter_sensorbase_bin_records(path):
        ts_s = t_ms * 1e-3
        if len(payload) >= payload_struct_v2.size:
            alt_m, pressure_pa, temp_c, var = payload_struct_v2.unpack_from(payload, 0)
        elif len(payload) >= payload_struct_v1.size:
            alt_m, var = payload_struct_v1.unpack_from(payload, 0)
            pressure_pa = float('nan')
            temp_c = float('nan')
        else:
            continue
        rows.append((ts_s, alt_m, pressure_pa, temp_c, var))

    return pd.DataFrame(
        rows,
        columns=['timestamp', 'baro_altitude', 'pressure_pa', 'temp_c', 'baro_var'],
    )


def load_teensy_mag_bin(path: Path) -> pd.DataFrame:
    # struct MagLogSample { float mag_uT[3]; }
    payload_struct = struct.Struct('<3f')
    rows = []
    for t_ms, payload, _truncated in iter_sensorbase_bin_records(path):
        if len(payload) < payload_struct.size:
            continue
        mx, my, mz = payload_struct.unpack_from(payload, 0)
        rows.append((t_ms * 1e-3, mx, my, mz))
    return pd.DataFrame(rows, columns=['timestamp', 'mag_x', 'mag_y', 'mag_z'])


def _estimate_sample_rate_hz(sample_t_s: np.ndarray) -> float:
    if sample_t_s.size < 2:
        raise ValueError('Not enough samples to estimate sample rate')
    dt = np.diff(sample_t_s)
    dt = dt[np.isfinite(dt) & (dt > 0)]
    if dt.size == 0:
        raise ValueError('Invalid timestamps (non-increasing or non-finite)')
    T_avg = float(np.mean(dt))
    return 1.0 / T_avg


def tune_dataframe(data: pd.DataFrame, *, include_mag: bool = False, fs_key: str = 'Fs'):
    """Tune from a dataframe with timestamp + sensor columns.

    Expected columns (any subset is ok):
      timestamp (seconds)
      gyro_x/y/z (rad/s)
      accel_x/y/z (g)
      baro_altitude (m)
      mag_x/y/z (uT)  [optional; not written to C++ params]
    """
    error_params: Dict[str, Dict[str, float]] = {}

    sample_t = data['timestamp'].values.astype(float).flatten()
    Fs = _estimate_sample_rate_hz(sample_t)

    taus = np.logspace(-2, 2.5, 50)

    # IMU axes
    imu_headers = [
        'gyro_x', 'gyro_y', 'gyro_z',
        'accel_x', 'accel_y', 'accel_z',
    ]
    available_imu = [h for h in imu_headers if h in data.columns]

    if available_imu:
        fig, axes = plt.subplots(2, 3, figsize=(10, 6))
        for idx, name in enumerate(imu_headers):
            if name not in data.columns:
                continue

            print(f"\n{'-'*40}")
            print(f"\nProcessing axis: {name}")

            raw_axis_data = data[name].values.astype(float).flatten()

            # Convert accelerometer from g to m/s²
            if name.startswith('accel'):
                raw_axis_data = raw_axis_data * G_TO_MS2

            print(f"raw_axis_data shape: {raw_axis_data.shape}")
            tau, adev, _adeverr, _n = allantools.oadev(raw_axis_data, rate=Fs, data_type='freq', taus=taus)
            gm1 = plot_allan_deviation_and_convert_gm(axes[idx//3, idx%3], tau, Fs, adev)

            error_params[name] = {
                'N': float(gm1.N),
                'B': float(gm1.B),
                'Tp': float(gm1.Tp),
            }

            if name.startswith('gyro'):
                error_params[name]['N_unit'] = 'rad/s/rtHz'
                error_params[name]['B_unit'] = 'rad/s'
            elif name.startswith('accel'):
                error_params[name]['N_unit'] = 'm/s^2/rtHz'
                error_params[name]['B_unit'] = 'm/s^2'

            axes[idx//3, idx%3].set_title(f'Allan Deviation - {name}')
            axes[idx//3, idx%3].set_ylabel(f'ADEV, {error_params[name]["B_unit"]}')

    # Barometer
    if 'baro_altitude' in data.columns:
        baro_name = 'baro_altitude'
        altitude = data['baro_altitude'].values.astype(float).flatten()

        tau, adev, _adeverr, _n = allantools.oadev(altitude, rate=Fs, data_type='freq', taus=taus)
        _fig_baro, ax_baro = plt.subplots(figsize=(5, 4))
        gm1_baro = plot_allan_deviation_and_convert_gm(ax_baro, tau, Fs, adev)

        error_params[baro_name] = {
            'N': float(gm1_baro.N),
            'B': float(gm1_baro.B),
            'Tp': float(gm1_baro.Tp),
            'N_unit': 'm/rtHz',
            'B_unit': 'm'
        }

        ax_baro.set_title(f'Allan Deviation - {baro_name}')
        ax_baro.set_ylabel(f'ADEV, {error_params[baro_name]["B_unit"]}')

    # Optional magnetometer (computed but not used in C++ struct)
    if include_mag:
        mag_headers = ['mag_x', 'mag_y', 'mag_z']
        available_mag = [h for h in mag_headers if h in data.columns]
        if available_mag:
            fig_mag, axes_mag = plt.subplots(1, 3, figsize=(10, 3))
            for idx, name in enumerate(mag_headers):
                if name not in data.columns:
                    continue
                raw_axis_data = data[name].values.astype(float).flatten()
                tau, adev, _adeverr, _n = allantools.oadev(raw_axis_data, rate=Fs, data_type='freq', taus=taus)
                gm1 = plot_allan_deviation_and_convert_gm(axes_mag[idx], tau, Fs, adev)
                error_params[name] = {
                    'N': float(gm1.N),
                    'B': float(gm1.B),
                    'Tp': float(gm1.Tp),
                    'N_unit': 'uT/rtHz',
                    'B_unit': 'uT'
                }
                axes_mag[idx].set_title(f'Allan Deviation - {name}')
                axes_mag[idx].set_ylabel('ADEV, uT')
            fig_mag.tight_layout()

    error_params[fs_key] = float(Fs)
    return error_params

def plot_allan_deviation_and_convert_gm(ax, tau, Fs, adev):
    """
    Plot Allan deviation and estimate noise parameters.
    
    Returns:
    --------
    ASD_to_GaussMarkovFirstOrder : class containing 1st order gauss markov parameters
    """
    ax.loglog(tau, adev, 'o', label='Allan Deviation')
    ax.set_xlabel('Delay, seconds')
    ax.minorticks_on()
    ax.grid(which='both', axis='both')
    
    # Estimate N (white noise) from short tau region
    short_tau_mask = tau < 1
    if np.sum(short_tau_mask) > 2:
        log_tau_short = np.log10(tau[short_tau_mask])
        log_adev_short = np.log10(adev[short_tau_mask])
        coeffs = np.polyfit(log_tau_short, log_adev_short, 1)
        adev_at_1 = 10**(coeffs[1] + coeffs[0] * 0)
        N = adev_at_1 * np.sqrt(1)
    else:
        N = np.min(adev) * np.sqrt(tau[np.argmin(adev)])
    
    # B (bias instability): minimum of the Allan deviation
    min_idx = np.argmin(adev)
    B = adev[min_idx]
    Tp = tau[min_idx]

    gm1 = ASD2GM1.ASD_to_GaussMarkovFirstOrder(N, B, Tp, Fs)
    
    return gm1


def tune_csv(path):
    print(f"Loading data from {path}")
    try:
        data = pd.read_csv(path)
    except FileNotFoundError:
        return {}

    # Back-compat: if CSV contains pressure, convert to baro_altitude
    if 'baro_altitude' not in data.columns and 'pressure' in data.columns:
        raw_pressure = data['pressure'].values.astype(float).flatten()
        data = data.copy()
        data['baro_altitude'] = 44330.0 * (1.0 - np.power(raw_pressure / 1013.25, 0.1903))

    return tune_dataframe(data)


def tune_teensy_bin_dir(data_dir: Path):
    """Tune IMU + Baro from Teensy SensorBase .bin logs in a directory."""
    imu_path = data_dir / 'IMU.bin'
    baro_path = data_dir / 'Baro.bin'
    mag_path = data_dir / 'Mag.bin'

    if not imu_path.exists() and not baro_path.exists() and not mag_path.exists():
        print(f"No expected Teensy bin logs found in {data_dir}")
        return {}

    imu_df: Optional[pd.DataFrame] = None
    baro_df: Optional[pd.DataFrame] = None

    if imu_path.exists():
        print(f"Loading IMU bin from {imu_path}")
        imu_df = load_teensy_imu_bin(imu_path)
        plt.plot(imu_df['timestamp'], imu_df['gyro_x'], label='gyro_x')
        plt.show()

    if baro_path.exists():
        print(f"Loading Baro bin from {baro_path}")
        baro_df = load_teensy_baro_bin(baro_path)
        plt.plot(baro_df['timestamp'], baro_df['baro_altitude'], label='baro_altitude')
        plt.show()

    if mag_path.exists():
        print(f"Loading Mag bin from {mag_path}")
        _mag_df = load_teensy_mag_bin(mag_path)
        plt.plot(_mag_df['timestamp'], _mag_df['mag_x'], label='mag_x')
        plt.show()

    else:
        _mag_df = None

    if imu_df is None and baro_df is None:
        return {}

    # Tune IMU and baro using their *own* timestamp streams.
    # `EkfErrorParameters` only has one `sampling_freq`, so we keep that as IMU Fs.
    params: Dict[str, Dict[str, float]] = {}

    if imu_df is not None and not imu_df.empty:
        params.update(tune_dataframe(imu_df, include_mag=False, fs_key='imu_Fs'))
    else:
        params['imu_Fs'] = 0.0

    if baro_df is not None and not baro_df.empty:
        baro_params = tune_dataframe(baro_df, include_mag=False, fs_key='baro_Fs')
        if 'baro_altitude' in baro_params:
            params['baro_altitude'] = baro_params['baro_altitude']
        params['baro_Fs'] = baro_params.get('baro_Fs', 0.0)

    return params



def main():

    # Resolve paths relative to this script so it works from any CWD.
    ekf_dir = Path(__file__).resolve().parent.parent
    repo_root = ekf_dir.parent

    sense_hat_csv = ekf_dir / 'data' / 'sense_hat_data.csv'
    teensy_teensy_bin_dir = repo_root / 'ekf-teensy' / 'data'
    teensy_csv_fallback = ekf_dir / 'data' / 'teensy_ptype_data.csv'

    all_params = {}

    if sense_hat_csv.exists():
        all_params['sense_hat_data'] = tune_csv(sense_hat_csv)
    else:
        print(f"WARNING: missing {sense_hat_csv}")
        all_params['sense_hat_data'] = {}

    teensy_params = tune_teensy_bin_dir(teensy_teensy_bin_dir)
    if teensy_params:
        all_params['teensy_ptype_data'] = teensy_params
    elif teensy_csv_fallback.exists():
        print("Falling back to teensy_ptype_data.csv")
        all_params['teensy_ptype_data'] = tune_csv(teensy_csv_fallback)
    else:
        print(f"WARNING: missing Teensy bin dir and CSV fallback: {teensy_teensy_bin_dir} / {teensy_csv_fallback}")
        all_params['teensy_ptype_data'] = {}

    # Save to C++ header
    output_file = ekf_dir / 'shared' / 'tuned_ekf_params.h'
    with output_file.open('w') as f:
        f.write("// Auto-generated EKF error parameters from Allan deviation analysis\n")
        f.write("// Generated by tune.py\n\n")
        f.write("#ifndef TUNED_EKF_PARAMS_H\n")
        f.write("#define TUNED_EKF_PARAMS_H\n\n")
        f.write('#include "ekf.h"\n\n')

        for setup, error_params in all_params.items():
            imu_fs = error_params.get('imu_Fs', error_params.get('Fs', 0.0))
            if not error_params or imu_fs == 0.0:
                print(f"Skipping {setup}: no data")
                continue

            f.write(f"inline constexpr EkfErrorParameters {setup.upper()}_PARAMS = \n")
            f.write("{\n")
            f.write(f"    .sampling_freq = {imu_fs:.6f},\n")
            
            for axis_name, params in error_params.items():
                if not isinstance(params, dict): continue
                clean_name = axis_name.lower()
                f.write(f"    .{clean_name}_n = {params['N']:.10e},\n")
                f.write(f"    .{clean_name}_b = {params['B']:.10e},\n")
                f.write(f"    .{clean_name}_tp = {params['Tp']:.6f},\n")
            
            # Remove trailing comma if we wrote at least one field
            f.seek(f.tell() - 2)
            f.write("\n};\n\n")

        f.write("#endif // TUNED_EKF_PARAMS_H\n")

    print(f"\n{'-'*40}")
    print(f"IMU error model parameters saved to {output_file}")


    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
