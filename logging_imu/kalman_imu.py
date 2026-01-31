# Author: Per-Ivar Faust
# Kalman Filter for IMU - estimates position and orientation

import numpy as np


class IMUKalmanFilter:
    """
    Simplified Kalman Filter for IMU - estimates position and orientation.
    Focus: Accurate Z-position (height) estimation in global frame.
    
    State vector: [z, vz, roll, pitch, yaw, az_bias, gx_bias, gy_bias, gz_bias]
    """
    
    def __init__(self, 
                 process_noise_accel=0.01,
                 process_noise_gyro=0.001,
                 measurement_noise_accel=0.05,
                 measurement_noise_gyro=0.02,
                 accel_correction_gain=0.15,
                 accel_bias_gain=0.005,
                 accel_bias_limit=0.5,
                 gravity=9.81):
        """
        Initialize Kalman Filter for IMU.
        
        Parameters:
        -----------
        process_noise_accel : float
            How much acceleration can deviate from model
        process_noise_gyro : float
            How much angular velocity can deviate from model
        measurement_noise_accel : float
            How much we trust accelerometer measurements
        measurement_noise_gyro : float
            How much we trust gyroscope measurements
        accel_correction_gain : float
            How strongly accelerometer corrects roll/pitch (0-1)
        accel_bias_gain : float
            How quickly accelerometer bias is adapted (low-pass gain)
        accel_bias_limit : float
            Absolute limit for accelerometer bias (m/s²)
        gravity : float
            Gravitational acceleration (m/s²)
        """
        self.g = gravity
        self.accel_correction_gain = accel_correction_gain
        self.accel_bias_gain = accel_bias_gain
        self.accel_bias_limit = accel_bias_limit
        self.dt = 0.01  # Will be updated with actual time delta
        
        # State: [z, vz, roll, pitch, yaw, az_bias, gx_bias, gy_bias, gz_bias]
        self.state = np.zeros(9)
        # z and vz at index 0,1; euler at 2,3,4; biases at 5,6,7,8
        self.accel_bias = np.zeros(3)
        
        # Track orientation confidence for adaptive gain
        self.orientation_confidence = 0.0  # 0 = no confidence, 1 = high confidence
        self.samples_since_init = 0
        
        # Covariance matrix (uncertainty in state estimate)
        self.P = np.eye(9) * 0.1
        self.P[0, 0] = 0.01      # Z position uncertainty (10cm)
        self.P[1, 1] = 0.01      # Z velocity uncertainty
        self.P[2:5, 2:5] *= 1.0  # Start with HIGH orientation uncertainty - we don't know how IMU is mounted!
        self.P[5:9, 5:9] *= 0.001  # Bias uncertainty
        
        # Process noise covariance (Q) - how much we expect state to change
        self.Q = np.eye(9) * 0.0001
        self.Q[0, 0] = process_noise_accel**2 * 0.1  # Z position process noise
        self.Q[1, 1] = process_noise_accel**2        # Z velocity process noise
        self.Q[2:5, 2:5] = process_noise_gyro**2     # Orientation process noise
        self.Q[5:9, 5:9] = 0.00001  # Bias changes very slowly
        
        # Measurement noise covariance (R) - how much we trust sensors
        self.R_accel = measurement_noise_accel**2
        self.R_gyro = measurement_noise_gyro**2
        
        self.last_time = None
        self.first_update = True
    
    def update(self, accel, gyro, timestamp=None):
        """
        Update Kalman filter with new IMU measurements.
        
        Parameters:
        -----------
        accel : array-like
            Accelerometer readings [ax, ay, az] in m/s² (in IMU frame)
        gyro : array-like
            Gyroscope readings [gx, gy, gz] in rad/s (in IMU frame)
        timestamp : float
            Current timestamp. If None, uses 0.01s default.
        """
        accel = np.array(accel)
        gyro = np.array(gyro)
        
        # Calculate time step
        if timestamp is not None and self.last_time is not None:
            self.dt = timestamp - self.last_time
            self.dt = np.clip(self.dt, 0.001, 0.1)  # Sanity check
        
        if self.first_update:
            # Initialize with zero orientation - let filter converge from scratch
            # This allows the filter to find the correct orientation regardless of initial mounting
            self.state[2] = 0.0  # roll
            self.state[3] = 0.0  # pitch
            self.state[4] = 0.0  # yaw
            self.first_update = False
            self.last_time = timestamp if timestamp is not None else 0
            return
        
        self.last_time = timestamp
        
        # Prediction step (uses accel as control input)
        self._predict_step(gyro, accel)
    
    def _predict_step(self, gyro, accel):
        
        """Predict next state using IMU kinematic model with gravity-based orientation correction"""
        z = self.state[0]
        vz = self.state[1]
        roll = self.state[2]
        pitch = self.state[3]
        yaw = self.state[4]
        az_bias = self.state[5]
        gx_bias = self.state[6]
        gy_bias = self.state[7]
        gz_bias = self.state[8]
        
        accel_imu = np.array(accel, dtype=float)
        
        # Predict orientation from gyro
        roll_gyro = roll + (gyro[0] - gx_bias) * self.dt
        pitch_gyro = pitch + (gyro[1] - gy_bias) * self.dt
        yaw_new = yaw + (gyro[2] - gz_bias) * self.dt

        # Get rotation matrix from current orientation estimate
        R = self._euler_to_rotation_matrix(roll_gyro, pitch_gyro, yaw_new)
        
        # Expected gravity in body frame (what we should measure if just gravity)
        g_global = np.array([0.0, 0.0, self.g])
        g_body_expected = R.T @ g_global
        
        # Compare measured acceleration to expected gravity
        # The difference tells us how far off our orientation is
        accel_residual = accel_imu - g_body_expected
        accel_residual_magnitude = np.linalg.norm(accel_residual)
        
        # Adaptive correction gain - higher when residual is large (orientation very wrong)
        # and when we have low confidence
        residual_correction = min(1.0, accel_residual_magnitude / self.g) * (1.0 - self.orientation_confidence)
        adaptive_gain = self.accel_correction_gain * (1.0 + 5.0 * residual_correction)
        adaptive_gain = np.clip(adaptive_gain, 0.1, 1.0)  # Between 10% and 100%
        
        # Derive roll/pitch from accelerometer (treats accel as gravity measurement)
        accel_magnitude = np.linalg.norm(accel_imu)
        if accel_magnitude > 0.1:
            # Use optimized orientation search for better robustness
            roll_acc, pitch_acc = self._find_best_orientation(accel_imu, yaw_new)
            
            # Blend gyro prediction with accel measurement
            roll_new = (1.0 - adaptive_gain) * roll_gyro + adaptive_gain * roll_acc
            pitch_new = (1.0 - adaptive_gain) * pitch_gyro + adaptive_gain * pitch_acc
        else:
            roll_new = roll_gyro
            pitch_new = pitch_gyro
        
        # Wrap angles
        roll_new = self._wrap_angle(roll_new)
        pitch_new = self._wrap_angle(pitch_new)
        yaw_new = self._wrap_angle(yaw_new)
        
        # Update orientation confidence based on how close accel residual is to zero
        # When residual is small, orientation is good
        confidence_this_step = max(0, 1.0 - accel_residual_magnitude / self.g)
        self.orientation_confidence = 0.98 * self.orientation_confidence + 0.02 * confidence_this_step
        self.samples_since_init += 1
        
        # Estimate accelerometer bias (continuous low-pass)
        R = self._euler_to_rotation_matrix(roll_new, pitch_new, yaw_new)
        g_body = R.T @ g_global
        
        # Adaptive bias gain - learn faster when orientation uncertain
        adaptive_bias_gain = self.accel_bias_gain * (1.0 + 3.0 * (1.0 - self.orientation_confidence))
        adaptive_bias_gain = np.clip(adaptive_bias_gain, 0.001, 0.03)
        
        bias_error = accel_imu - g_body
        self.accel_bias = (1.0 - adaptive_bias_gain) * self.accel_bias + adaptive_bias_gain * bias_error
        self.accel_bias = np.clip(self.accel_bias, -self.accel_bias_limit, self.accel_bias_limit)

        # Transform accel to global frame and remove gravity
        accel_imu_corrected = accel_imu - self.accel_bias
        accel_global = R @ accel_imu_corrected
        az_global = accel_global[2] - self.g

        # Predict velocity and position using linear acceleration
        vz_new = vz + az_global * self.dt
        z_new = z + vz * self.dt + 0.5 * az_global * (self.dt ** 2)

        # Biases remain constant (random walk assumption)
        az_bias_new = self.accel_bias[2]
        gx_bias_new = gx_bias
        gy_bias_new = gy_bias
        gz_bias_new = gz_bias
        
        # Update state
        self.state[0] = z_new
        self.state[1] = vz_new
        self.state[2] = roll_new
        self.state[3] = pitch_new
        self.state[4] = yaw_new
        self.state[5] = az_bias_new
        self.state[6] = gx_bias_new
        self.state[7] = gy_bias_new
        self.state[8] = gz_bias_new
        
        # Update covariance: P = F * P * F^T + Q
        # F is Jacobian of state transition (simplified to identity for now)
        self.P = self.P + self.Q
        
        # Reduce covariance for orientation as we become more confident
        orientation_reduction = self.orientation_confidence
        self.P[2:5, 2:5] *= (1.0 - 0.1 * orientation_reduction)
        
        # Add some damping to prevent covariance explosion
        self.P = np.clip(self.P, 0, 10)
    
    def _measurement_update(self, accel, gyro):
        """Reserved for future external measurements (GPS/baro)."""
        return

    @staticmethod
    def _accel_to_roll_pitch(accel):
        """Compute roll/pitch from accelerometer (gravity direction)."""
        ax, ay, az = accel
        roll = np.arctan2(ay, az)
        pitch = np.arctan2(-ax, np.sqrt(ay * ay + az * az))
        return roll, pitch
    
    def _find_best_orientation(self, accel_imu, yaw_current):
        """
        Find best roll/pitch that minimizes deviation from expecting gravity only.
        This is more robust than _accel_to_roll_pitch because it accounts for 
        possible linear acceleration components.
        """
        from scipy.optimize import minimize
        
        accel_mag = np.linalg.norm(accel_imu)
        if accel_mag < 0.1:
            return 0, 0
        
        def objective(angles):
            roll, pitch = angles
            # Assume yaw doesn't change (only gyro drives yaw)
            R = self._euler_to_rotation_matrix(roll, pitch, yaw_current)
            g_global = np.array([0.0, 0.0, self.g])
            g_body_expected = R.T @ g_global
            # Minimize difference between measured and expected gravity
            residual = accel_imu - g_body_expected
            return np.dot(residual, residual)
        
        # Optimize near current estimate
        roll_current = self.state[2]
        pitch_current = self.state[3]
        
        result = minimize(objective, [roll_current, pitch_current], 
                         method='Nelder-Mead',
                         options={'xatol': 1e-4, 'fatol': 1e-6, 'maxiter': 100})
        
        roll_opt, pitch_opt = result.x
        return roll_opt, pitch_opt
    
    def _euler_to_rotation_matrix(self, roll, pitch, yaw):
        """
        Convert Euler angles to rotation matrix (ZYX order).
        Transforms from body frame to global frame.
        """
        # Rotation about X (roll)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        # Rotation about Y (pitch)
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        # Rotation about Z (yaw)
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # Combined rotation (apply yaw, then pitch, then roll)
        return Rx @ Ry @ Rz
    
    @staticmethod
    def _wrap_angle(angle):
        """Wrap angle to [-π, π]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    # Getter methods
    def get_height(self):
        """Return current height (Z) above reference point"""
        return self.state[0]
    
    def get_velocity_z(self):
        """Return current vertical velocity"""
        return self.state[1]
    
    def get_orientation(self):
        """Return current orientation [roll, pitch, yaw] in radians"""
        return self.state[2:5].copy()
    
    def get_state(self):
        """Return full state vector"""
        return self.state.copy()
    
    def reset_height(self, z0=0.0):
        """Reset height to known value (e.g., start position)"""
        self.state[0] = z0
        self.state[1] = 0.0  # Reset velocity
        self.P[0:2, 0:2] = np.eye(2) * 0.01  # Reset uncertainty
    
    def set_initial_orientation(self, roll=0, pitch=0, yaw=0):
        """Set initial orientation"""
        self.state[2] = roll
        self.state[3] = pitch
        self.state[4] = yaw
        self.P[2:5, 2:5] = np.eye(3) * 0.001
