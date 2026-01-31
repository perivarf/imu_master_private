# Author: Per-Ivar Faust
# Playback Kalman filter on logged IMU data

import pandas as pd
import numpy as np
from pathlib import Path
from kalman_imu import IMUKalmanFilter
import sys

def process_datalog(csv_file):
    """
    Process a datalog CSV file through Kalman filter and add state columns.
    Output will be saved to same filename with _kalman suffix.
    
    Parameters:
    -----------
    csv_file : str
        Path to input CSV file (e.g. dataLog_20260131_133859.csv)
    """
    
    # Load data
    print(f"Loading data from {csv_file}...")
    df = pd.read_csv(csv_file)
    print(f"Loaded {len(df)} samples")
    
    # Prepare output filename
    input_path = Path(csv_file)
    output_file = str(input_path.parent / f"{input_path.stem}_kalman.csv")
    
    # Initialize Kalman filter
    print("\nInitializing Kalman filter...")
    kalman = IMUKalmanFilter()
    kalman.reset_height(z0=0.0)
    
    # Storage for states
    n_samples = len(df)
    z_estimates = np.zeros(n_samples)
    vz_estimates = np.zeros(n_samples)
    roll_estimates = np.zeros(n_samples)
    pitch_estimates = np.zeros(n_samples)
    yaw_estimates = np.zeros(n_samples)
    az_bias = np.zeros(n_samples)
    
    # Process each sample
    print(f"\nProcessing {n_samples} samples through Kalman filter...")
    last_time = None
    
    for idx, row in df.iterrows():
        # Extract measurements
        accel = np.array([row['aX'], row['aY'], row['aZ']], dtype=float)
        gyro = np.array([row['gX'], row['gY'], row['gZ']], dtype=float)
        
        # Parse timestamp (format: HH:MM:SS.mmm)
        time_str = row['rtcTime']
        try:
            h, m, s = map(float, time_str.split(':'))
            current_time = h * 3600 + m * 60 + s
            
            if last_time is None:
                last_time = current_time
                timestamp = 0.0
            else:
                timestamp = current_time - last_time
                last_time = current_time
        except:
            timestamp = None
        
        # Update filter
        kalman.update(accel, gyro, timestamp=timestamp)
        
        # Extract state
        z_estimates[idx] = kalman.get_height()
        vz_estimates[idx] = kalman.get_velocity_z()
        roll, pitch, yaw = kalman.get_orientation()
        roll_estimates[idx] = np.degrees(roll)
        pitch_estimates[idx] = np.degrees(pitch)
        yaw_estimates[idx] = np.degrees(yaw)
        az_bias[idx] = kalman.accel_bias[2]
        
        # Progress
        if (idx + 1) % 500 == 0:
            print(f"  {idx + 1}/{n_samples} | Z:{z_estimates[idx]:.2f}m | Vz:{vz_estimates[idx]:.3f}m/s | " +
                  f"Roll:{roll_estimates[idx]:.1f}° | Pitch:{pitch_estimates[idx]:.1f}°")
    
    # Add columns to dataframe
    print("\nAdding Kalman state columns to dataframe...")
    df['z_kalman'] = z_estimates
    df['vz_kalman'] = vz_estimates
    df['roll_kalman'] = roll_estimates
    df['pitch_kalman'] = pitch_estimates
    df['yaw_kalman'] = yaw_estimates
    df['az_bias_kalman'] = az_bias
    
    # Save results
    print(f"\nSaving results to {output_file}...")
    df.to_csv(output_file, index=False)
    print(f"Saved {len(df)} rows to {output_file}")
    
    # Display summary
    print("\n" + "="*70)
    print("KALMAN FILTER RESULTS SUMMARY")
    print("="*70)
    print(f"Total samples: {len(df)}")
    print(f"\nLogging period:")
    print(f"  Start time: {df['rtcTime'].iloc[0]}")
    print(f"  End time:   {df['rtcTime'].iloc[-1]}")
    
    print(f"\nHeight (Z):")
    print(f"  Start: {df['z_kalman'].iloc[0]:.2f} m")
    print(f"  End:   {df['z_kalman'].iloc[-1]:.2f} m")
    print(f"  Min:   {df['z_kalman'].min():.2f} m")
    print(f"  Max:   {df['z_kalman'].max():.2f} m")
    print(f"  Mean:  {df['z_kalman'].mean():.2f} m")
    print(f"  Std:   {df['z_kalman'].std():.2f} m")
    
    print(f"\nVertical velocity (Vz):")
    print(f"  Start: {df['vz_kalman'].iloc[0]:.4f} m/s")
    print(f"  End:   {df['vz_kalman'].iloc[-1]:.4f} m/s")
    print(f"  Min:   {df['vz_kalman'].min():.4f} m/s")
    print(f"  Max:   {df['vz_kalman'].max():.4f} m/s")
    print(f"  Mean:  {df['vz_kalman'].mean():.4f} m/s")
    
    print(f"\nOrientation:")
    print(f"  Roll  - Start: {df['roll_kalman'].iloc[0]:6.1f}°, End: {df['roll_kalman'].iloc[-1]:6.1f}° | Mean: {df['roll_kalman'].mean():6.1f}° (std: {df['roll_kalman'].std():.1f}°)")
    print(f"  Pitch - Start: {df['pitch_kalman'].iloc[0]:6.1f}°, End: {df['pitch_kalman'].iloc[-1]:6.1f}° | Mean: {df['pitch_kalman'].mean():6.1f}° (std: {df['pitch_kalman'].std():.1f}°)")
    print(f"  Yaw   - Start: {df['yaw_kalman'].iloc[0]:6.1f}°, End: {df['yaw_kalman'].iloc[-1]:6.1f}° | Mean: {df['yaw_kalman'].mean():6.1f}° (std: {df['yaw_kalman'].std():.1f}°)")
    
    print(f"\nAccelerometer Z bias:")
    print(f"  Start: {df['az_bias_kalman'].iloc[0]:.4f} m/s²")
    print(f"  End:   {df['az_bias_kalman'].iloc[-1]:.4f} m/s²")
    print(f"  Mean:  {df['az_bias_kalman'].mean():.4f} m/s²")
    print(f"  Std:   {df['az_bias_kalman'].std():.4f} m/s²")
    print("="*70)
    
    return df


def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Process datalog with Kalman filter')
    parser.add_argument('input', help='Input CSV file (dataLog_*.csv)')
    
    args = parser.parse_args()
    
    # Check if input file exists
    if not Path(args.input).exists():
        print(f"ERROR: Input file not found: {args.input}")
        sys.exit(1)
    
    try:
        df = process_datalog(args.input)
        print(f"\n✓ Processing complete!")
        
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
