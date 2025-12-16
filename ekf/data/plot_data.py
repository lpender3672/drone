import pathlib
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def plot_gps_data(csv_path='gps_data.csv', save_path='gps_plots.png'):
    """
    Plot GPS data from CSV file including position, velocity, and accuracy metrics.
    
    Args:
        csv_path: Path to the GPS data CSV file
        save_path: Path to save the figure (default: 'gps_plots.png')
    """
    # Read the CSV file
    df = pd.read_csv(csv_path)
    
    # Create figure with subplots
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('GPS Data Analysis', fontsize=16)
    
    # Plot 1: Latitude vs Longitude (trajectory)
    axes[0, 0].plot(df['longitude'], df['latitude'], 'b-', linewidth=0.5)
    axes[0, 0].scatter(df['longitude'].iloc[0], df['latitude'].iloc[0], 
                       c='green', s=100, marker='o', label='Start')
    axes[0, 0].scatter(df['longitude'].iloc[-1], df['latitude'].iloc[-1], 
                       c='red', s=100, marker='x', label='End')
    axes[0, 0].set_xlabel('Longitude (degrees)')
    axes[0, 0].set_ylabel('Latitude (degrees)')
    axes[0, 0].set_title('GPS Trajectory')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    
    # Plot 2: Altitude over time
    axes[0, 1].plot(df['timestamp'], df['altitude_msl'], 'r-', label='MSL', linewidth=0.8)
    axes[0, 1].plot(df['timestamp'], df['altitude_ellipsoid'], 'b-', label='Ellipsoid', linewidth=0.8)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Altitude (m)')
    axes[0, 1].set_title('Altitude vs Time')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    
    # Plot 3: Velocity components
    axes[1, 0].plot(df['timestamp'], df['vel_north'], label='North', linewidth=0.8)
    axes[1, 0].plot(df['timestamp'], df['vel_east'], label='East', linewidth=0.8)
    axes[1, 0].plot(df['timestamp'], df['vel_down'], label='Down', linewidth=0.8)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Velocity (m/s)')
    axes[1, 0].set_title('Velocity Components')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    
    # Plot 4: Ground speed and heading
    ax1 = axes[1, 1]
    ax2 = ax1.twinx()
    line1 = ax1.plot(df['timestamp'], df['ground_speed'], 'b-', label='Ground Speed', linewidth=0.8)
    line2 = ax2.plot(df['timestamp'], df['heading'], 'r-', label='Heading', linewidth=0.8)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Ground Speed (m/s)', color='b')
    ax2.set_ylabel('Heading (degrees)', color='r')
    ax1.set_title('Ground Speed and Heading')
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels)
    ax1.grid(True)
    
    # Plot 5: Accuracy metrics
    axes[2, 0].plot(df['timestamp'], df['h_accuracy'], label='Horizontal', linewidth=0.8)
    axes[2, 0].plot(df['timestamp'], df['v_accuracy'], label='Vertical', linewidth=0.8)
    axes[2, 0].plot(df['timestamp'], df['speed_accuracy'], label='Speed', linewidth=0.8)
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].set_ylabel('Accuracy (m or m/s)')
    axes[2, 0].set_title('Accuracy Metrics')
    axes[2, 0].legend()
    axes[2, 0].grid(True)
    
    # Plot 6: Satellite info
    ax1 = axes[2, 1]
    ax2 = ax1.twinx()
    line1 = ax1.plot(df['timestamp'], df['num_sats'], 'g-', label='Num Satellites', linewidth=0.8)
    line2 = ax2.plot(df['timestamp'], df['pdop'], 'orange', label='PDOP', linewidth=0.8)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Number of Satellites', color='g')
    ax2.set_ylabel('PDOP', color='orange')
    ax1.set_title('Satellite Status')
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels)
    ax1.grid(True)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"GPS plots saved to {save_path}")
    plt.close(fig)
    return fig


def plot_sensor_data(csv_path='sensor_data.csv', save_path='sensor_plots.png'):
    """
    Plot IMU sensor data from CSV file including accelerometer, gyroscope, magnetometer, and pressure.
    
    Args:
        csv_path: Path to the sensor data CSV file
        save_path: Path to save the figure (default: 'sensor_plots.png')
    """
    # Read the CSV file
    df = pd.read_csv(csv_path)
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('IMU Sensor Data Analysis', fontsize=16)
    
    # Plot 1: Accelerometer data
    axes[0, 0].plot(df['timestamp'], df['accel_x'], label='X', linewidth=0.8)
    axes[0, 0].plot(df['timestamp'], df['accel_y'], label='Y', linewidth=0.8)
    axes[0, 0].plot(df['timestamp'], df['accel_z'], label='Z', linewidth=0.8)
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Acceleration (g)')
    axes[0, 0].set_title('Accelerometer Data')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    
    # Plot 2: Gyroscope data
    axes[0, 1].plot(df['timestamp'], df['gyro_x'], label='X', linewidth=0.8)
    axes[0, 1].plot(df['timestamp'], df['gyro_y'], label='Y', linewidth=0.8)
    axes[0, 1].plot(df['timestamp'], df['gyro_z'], label='Z', linewidth=0.8)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Angular Velocity (rad/s)')
    axes[0, 1].set_title('Gyroscope Data')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    
    # Plot 3: Magnetometer data
    axes[1, 0].plot(df['timestamp'], df['mag_x'], label='X', linewidth=0.8)
    axes[1, 0].plot(df['timestamp'], df['mag_y'], label='Y', linewidth=0.8)
    axes[1, 0].plot(df['timestamp'], df['mag_z'], label='Z', linewidth=0.8)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Magnetic Field (μT)')
    axes[1, 0].set_title('Magnetometer Data')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    
    # Plot 4: Pressure data
    axes[1, 1].plot(df['timestamp'], df['pressure'], 'purple', linewidth=0.8)
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Pressure (hPa)')
    axes[1, 1].set_title('Barometric Pressure')
    axes[1, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"Sensor plots saved to {save_path}")
    plt.close(fig)
    return fig

def plot_ekf_data(csv_path='ekf_data.csv', save_path='ekf_plots.png'):
    """
    Plot EKF data from CSV file including attitude, velocity, and position.
    
    Args:
        csv_path: Path to the EKF data CSV file
        save_path: Path to save the figure (default: 'ekf_plots.png')
    """
    # Read the CSV file
    df = pd.read_csv(csv_path)
    
    # Create figure with subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 15))
    fig.suptitle('EKF Data Analysis', fontsize=16)

    # Plot 1: Velocity (North, East, Down)
    axes[1].plot(df['time'], df['vn'], label='North', linewidth=0.8)
    axes[1].plot(df['time'], df['ve'], label='East', linewidth=0.8)
    axes[1].plot(df['time'], df['vd'], label='Down', linewidth=0.8)
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Velocity (m/s)')
    axes[1].set_title('INS Velocity')
    axes[1].legend()
    axes[1].grid(True)
    
    # Plot 2: Position (Latitude, Longitude)
    scatter = axes[2].scatter(df['lon'], df['lat'], c=df['time'], cmap='viridis', 
                              s=10, linewidth=0)
    axes[2].scatter(df['lon'].iloc[0], df['lat'].iloc[0], 
                    c='green', s=100, marker='o', label='Start', edgecolors='black')
    axes[2].scatter(df['lon'].iloc[-1], df['lat'].iloc[-1], 
                    c='red', s=100, marker='x', label='End', linewidths=3)
    cbar = plt.colorbar(scatter, ax=axes[2])
    cbar.set_label('Time (s)')
    axes[2].set_xlabel('Longitude (degrees)')
    axes[2].set_ylabel('Latitude (degrees)')
    axes[2].set_title('INS Position Trajectory')
    axes[2].legend()
    axes[2].grid(True)

    # Plot 3: Altitude
    axes[0].plot(df['time'], df['alt'], label='Altitude', linewidth=0.8)
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Altitude (m)')
    axes[0].set_title('INS Altitude')
    axes[0].legend()
    axes[0].grid(True)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"INS plots saved to {save_path}")
    plt.close(fig)
    return fig

def plot_all_data(gps_csv='gps_data.csv', sensor_csv='sensor_data.csv',
                  ekf_csv='ekf_data.csv',
                  gps_save='gps_plots.png', sensor_save='sensor_plots.png',
                  ekf_save='ekf_plots.png'):
    """
    Plot both GPS and sensor data and save to files.
    
    Args:
        gps_csv: Path to the GPS data CSV file
        sensor_csv: Path to the sensor data CSV file
        gps_save: Path to save GPS plots
        sensor_save: Path to save sensor plots
    """
    
    #plot_gps_data(gps_csv, gps_save)
    #plot_sensor_data(sensor_csv, sensor_save)
    plot_ekf_data(ekf_csv, ekf_save)
    print("All plots saved successfully!")


if __name__ == '__main__':
    plot_all_data()
