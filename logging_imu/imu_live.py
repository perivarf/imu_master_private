# Author: Per-Ivar Faust

import os
# Suppress Qt portal warnings
os.environ['QT_LOGGING_RULES'] = '*.debug=false;qt.qpa.*=false'

import serial
import time
from collections import deque
import threading
import plotly.graph_objects as go
from dash import Dash, dcc, html
from dash.dependencies import Input, Output, State
import webbrowser
import numpy as np
from ahrs.filters import Madgwick

class IMULiveReader:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, buffer_size=100):
        """
        Initialize IMU live reader
        
        Parameters:
        port (str): Serial port for IMU device
        baudrate (int): Baud rate for serial connection
        buffer_size (int): Number of data points to keep in buffer
        """
        self.port = port
        self.baudrate = baudrate
        self.buffer_size = buffer_size
        
        # Data buffers (using deque for efficient append/pop)
        self.time_buffer = deque(maxlen=buffer_size)
        self.ax_buffer = deque(maxlen=buffer_size)
        self.ay_buffer = deque(maxlen=buffer_size)
        self.az_buffer = deque(maxlen=buffer_size)
        self.gx_buffer = deque(maxlen=buffer_size)
        self.gy_buffer = deque(maxlen=buffer_size)
        self.gz_buffer = deque(maxlen=buffer_size)
        self.mx_buffer = deque(maxlen=buffer_size)
        self.my_buffer = deque(maxlen=buffer_size)
        self.mz_buffer = deque(maxlen=buffer_size)
        
        # Global frame buffers
        self.ax_global_buffer = deque(maxlen=buffer_size)
        self.ay_global_buffer = deque(maxlen=buffer_size)
        self.az_global_buffer = deque(maxlen=buffer_size)
        
        # Orientation buffers
        self.roll_buffer = deque(maxlen=buffer_size)
        self.pitch_buffer = deque(maxlen=buffer_size)
        self.yaw_buffer = deque(maxlen=buffer_size)
        
        # Madgwick filter for sensor fusion
        self.madgwick = Madgwick()
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        self.sample_period = 0.1  # Will be updated with actual sample rate
        
        # Magnetometer calibration
        self.mag_calibration_mode = False
        self.mag_calibrated = False
        self.mag_cal_samples = []
        self.mag_offset = np.array([0.0, 0.0, 0.0])  # Hard-iron offset
        self.mag_scale = np.array([1.0, 1.0, 1.0])  # Soft-iron scale
        
        self.serial_conn = None
        self.running = False
        self.data_count = 0
        self.error_count = 0
        self.last_error_print = 0
        self.reset_orientation_flag = False
        
    def connect(self):
        """Establish serial connection to IMU"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            print(f"Connected to {self.port} at {self.baudrate} baud")
            print(f"Serial settings: 8N1, no flow control")
            
            # Wait for connection to stabilize
            time.sleep(1)
            
            # Clear any existing data in buffer
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            # Wait a bit and check if data is coming
            print("Waiting for data...")
            time.sleep(2)
            
            waiting = self.serial_conn.in_waiting
            print(f"Bytes in buffer: {waiting}")
            
            if waiting == 0:
                print("WARNING: No data received yet. Is the IMU sending data?")
            
            return True
        except Exception as e:
            print(f"Error connecting to {self.port}: {e}")
            return False
    
    def parse_line(self, line):
        """
        Parse a line of CSV data from IMU
        Expected format: rtcDate,rtcTime,aX,aY,aZ,gX,gY,gZ,mX,mY,mZ,output_Hz,
        """
        try:
            parts = line.strip().split(',')
            
            # Skip empty or too short lines
            if len(parts) < 11:
                return None
            
            # Skip header line
            if parts[0] == 'rtcDate' or 'rtcDate' in line:
                return None
            
            # Try to parse the numeric fields
            data = {
                'rtcTime': parts[1],
                'aX': float(parts[2]),
                'aY': float(parts[3]),
                'aZ': float(parts[4]),
                'gX': float(parts[5]),
                'gY': float(parts[6]),
                'gZ': float(parts[7]),
                'mX': float(parts[8]),
                'mY': float(parts[9]),
                'mZ': float(parts[10]),
            }
            
            # Validate data is reasonable (optional sanity check)
            # Skip data with extreme values that indicate corruption
            for key, val in data.items():
                if key != 'rtcTime' and (abs(val) > 10000):
                    return None
                    
            return data
        except (ValueError, IndexError, TypeError):
            # Increment error counter
            self.error_count += 1
            # Only print error message every 10 errors to avoid spam
            if self.error_count - self.last_error_print >= 10:
                print(f"\n[Warning] Skipped {self.error_count} corrupted lines (serial data corruption is normal)")
                self.last_error_print = self.error_count
            return None
    
    def read_data_thread(self):
        """Background thread to continuously read data from serial port"""
        line_buffer = ""
        
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting:
                    # Read available bytes
                    chunk = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                    line_buffer += chunk
                    
                    # Process complete lines
                    while '\n' in line_buffer:
                        line, line_buffer = line_buffer.split('\n', 1)
                        line = line.strip()
                        
                        if not line:
                            continue
                        
                        # Debug: print every 50th line to see what we're getting
                        if (self.data_count + self.error_count) % 50 == 0:
                            print(f'\n[DEBUG] Raw line: {line[:100]}...' if len(line) > 100 else f'\n[DEBUG] Raw line: {line}')
                        
                        data = self.parse_line(line)
                        
                        if data:
                            # Add raw data to buffers
                            self.time_buffer.append(self.data_count)
                            self.ax_buffer.append(data['aX'])
                            self.ay_buffer.append(data['aY'])
                            self.az_buffer.append(data['aZ'])
                            self.gx_buffer.append(data['gX'])
                            self.gy_buffer.append(data['gY'])
                            self.gz_buffer.append(data['gZ'])
                            self.mx_buffer.append(data['mX'])
                            self.my_buffer.append(data['mY'])
                            self.mz_buffer.append(data['mZ'])
                            
                            # Perform sensor fusion
                            self.update_orientation(data)
                            
                            self.data_count += 1
                else:
                    time.sleep(0.01)  # Small delay if no data available
            except Exception as e:
                print(f"\nError reading data: {e}")
                time.sleep(0.1)
    
    def start_reading(self):
        """Start background thread for reading data"""
        self.running = True
        self.read_thread = threading.Thread(target=self.read_data_thread, daemon=True)
        self.read_thread.start()
    
    def stop_reading(self):
        """Stop reading data"""
        self.running = False
        if hasattr(self, 'read_thread'):
            self.read_thread.join(timeout=2)
        if self.serial_conn:
            self.serial_conn.close()
            print("Serial connection closed")
    
    def reset_orientation(self):
        """Reset orientation to identity quaternion"""
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Reset to identity
        print("\n[RESET] Orientation reset to identity quaternion")
    
    def start_mag_calibration(self):
        """Start magnetometer calibration data collection"""
        self.mag_calibration_mode = True
        self.mag_cal_samples = []
        print("\n[MAG CAL] Started magnetometer calibration")
        print("[MAG CAL] Rotate sensor slowly in ALL directions (like drawing a figure-8)")
        print("[MAG CAL] Continue for 20-30 seconds...")
    
    def finish_mag_calibration(self):
        """Calculate magnetometer calibration from collected samples"""
        self.mag_calibration_mode = False
        
        if len(self.mag_cal_samples) < 50:
            print(f"\n[MAG CAL] ERROR: Not enough samples ({len(self.mag_cal_samples)}). Need at least 50.")
            self.mag_calibrated = False
            return False
        
        # Convert to numpy array
        samples = np.array(self.mag_cal_samples)
        
        # Calculate hard-iron offset (center of sphere)
        mag_max = np.max(samples, axis=0)
        mag_min = np.min(samples, axis=0)
        self.mag_offset = (mag_max + mag_min) / 2.0
        
        # Calculate soft-iron scale (normalize to sphere)
        mag_range = mag_max - mag_min
        avg_range = np.mean(mag_range)
        self.mag_scale = avg_range / mag_range
        
        print(f"\n[MAG CAL] Calibration complete with {len(self.mag_cal_samples)} samples")
        print(f"[MAG CAL] Offset (hard-iron): X={self.mag_offset[0]:.2f}, Y={self.mag_offset[1]:.2f}, Z={self.mag_offset[2]:.2f}")
        print(f"[MAG CAL] Scale (soft-iron): X={self.mag_scale[0]:.3f}, Y={self.mag_scale[1]:.3f}, Z={self.mag_scale[2]:.3f}")
        print(f"[MAG CAL] Raw range: X={mag_range[0]:.2f}, Y={mag_range[1]:.2f}, Z={mag_range[2]:.2f}")
        
        # Mark as calibrated
        self.mag_calibrated = True
        
        # Reset orientation after calibration
        self.reset_orientation()
        return True
    
    def update_orientation(self, data):
        """Update orientation using Madgwick filter and compute global acceleration"""
        try:
            # Check if reset was requested
            if self.reset_orientation_flag:
                self.reset_orientation()
                self.reset_orientation_flag = False
            
            # Convert to SI units
            # Accelerometer: mg to m/s² (1mg = 0.001g = 0.00981 m/s²)
            acc = np.array([data['aX'], data['aY'], data['aZ']]) * 0.00981
            
            # Gyroscope: deg/s to rad/s
            gyr = np.array([data['gX'], data['gY'], data['gZ']]) * np.pi / 180.0
            
            # Magnetometer: µT (already in correct units)
            mag = np.array([data['mX'], data['mY'], data['mZ']])
            
            # Store raw magnetometer samples during calibration
            if self.mag_calibration_mode:
                self.mag_cal_samples.append(mag.copy())
            
            # Apply magnetometer calibration
            mag = (mag - self.mag_offset) * self.mag_scale
            
            # Normalize accelerometer and magnetometer
            acc_norm = acc / np.linalg.norm(acc) if np.linalg.norm(acc) > 0 else acc
            mag_norm = mag / np.linalg.norm(mag) if np.linalg.norm(mag) > 0 else mag
            
            # Update quaternion using Madgwick filter
            self.quaternion = self.madgwick.updateMARG(
                q=self.quaternion,
                gyr=gyr,
                acc=acc_norm,
                mag=mag_norm
            )
            
            # Convert quaternion to rotation matrix
            q = self.quaternion
            w, x, y, z = q[0], q[1], q[2], q[3]
            
            # Rotation matrix from body to world frame
            R = np.array([
                [1-2*(y**2+z**2), 2*(x*y-w*z), 2*(x*z+w*y)],
                [2*(x*y+w*z), 1-2*(x**2+z**2), 2*(y*z-w*x)],
                [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x**2+y**2)]
            ])
            
            # Transform acceleration to global frame
            acc_global = R @ acc
            
            # Remove gravity to get true acceleration
            # Accelerometer measures proper acceleration (a_proper = a_true - g)
            # When stationary: a_measured ≈ -g in local frame (pointing down)
            # After rotation to global: still ≈ -g in Z direction
            # To get true motion: a_true = a_proper - g_vector
            # where g_vector = [0, 0, -9.81] (gravity points down)
            # So: a_true = a_proper - (-9.81) = a_proper + 9.81
            gravity_global = np.array([0.0, 0.0, -9.81])
            acc_global = acc_global - gravity_global
            
            # Store global acceleration (convert back to mg for consistency)
            self.ax_global_buffer.append(acc_global[0] / 0.00981)
            self.ay_global_buffer.append(acc_global[1] / 0.00981)
            self.az_global_buffer.append(acc_global[2] / 0.00981)
            
            # Calculate Euler angles for display
            # Roll (x-axis rotation)
            roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
            # Pitch (y-axis rotation)
            pitch = np.arcsin(2*(w*y - z*x))
            # Yaw (z-axis rotation)
            yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
            
            # Convert to degrees and store
            self.roll_buffer.append(np.degrees(roll))
            self.pitch_buffer.append(np.degrees(pitch))
            self.yaw_buffer.append(np.degrees(yaw))
            
        except Exception as e:
            # If orientation update fails, use zero for global acceleration
            self.ax_global_buffer.append(0.0)
            self.ay_global_buffer.append(0.0)
            self.az_global_buffer.append(0.0)
            self.roll_buffer.append(0.0)
            self.pitch_buffer.append(0.0)
            self.yaw_buffer.append(0.0)
    
    def visualize_live(self, update_interval=100):
        """
        Create live 3D vector visualization of IMU data using Dash
        
        Parameters:
        update_interval (int): Update interval in milliseconds
        """
        # Scaling factors for visualization
        a_scale = 1.0 / 1000  # Acceleration scaling
        g_scale = 1.0  # Gyroscope scaling
        m_scale = 1.0 / 50  # Magnetometer scaling
        
        # Create Dash app
        app = Dash(__name__)
        
        app.layout = html.Div([
            html.H1("IMU 3D Vektorer - Lokal & Global (Live)", style={'textAlign': 'center'}),
            html.Div([
                html.Button('⏸ Pause', id='pause-button', n_clicks=0, style={
                    'fontSize': '18px',
                    'padding': '10px 30px',
                    'margin': '10px',
                    'backgroundColor': '#4CAF50',
                    'color': 'white',
                    'border': 'none',
                    'borderRadius': '5px',
                    'cursor': 'pointer'
                }),
                html.Button('🔄 Reset Orientering', id='reset-button', n_clicks=0, style={
                    'fontSize': '18px',
                    'padding': '10px 30px',
                    'margin': '10px',
                    'backgroundColor': '#FF9800',
                    'color': 'white',
                    'border': 'none',
                    'borderRadius': '5px',
                    'cursor': 'pointer'
                }),
                html.Button('🧭 Start Mag Kalibrering', id='mag-cal-start-button', n_clicks=0, style={
                    'fontSize': '18px',
                    'padding': '10px 30px',
                    'margin': '10px',
                    'backgroundColor': '#2196F3',
                    'color': 'white',
                    'border': 'none',
                    'borderRadius': '5px',
                    'cursor': 'pointer'
                }),
                html.Button('✅ Fullfør Mag Kalibrering', id='mag-cal-finish-button', n_clicks=0, style={
                    'fontSize': '18px',
                    'padding': '10px 30px',
                    'margin': '10px',
                    'backgroundColor': '#9C27B0',
                    'color': 'white',
                    'border': 'none',
                    'borderRadius': '5px',
                    'cursor': 'pointer'
                }),
                html.Div(id='mag-cal-status', style={
                    'fontSize': '16px',
                    'padding': '10px',
                    'margin': '10px',
                    'backgroundColor': '#f0f0f0',
                    'borderRadius': '5px',
                    'minWidth': '300px',
                    'textAlign': 'center'
                }, children='Magnetometer: Ikke kalibrert'),
                html.Div([
                    html.Label('Vis sensorer:', style={'fontWeight': 'bold', 'marginRight': '10px'}),
                    dcc.Checklist(
                        id='sensor-checklist',
                        options=[
                            {'label': ' Akselerometer', 'value': 'accel'},
                            {'label': ' Gyroskop', 'value': 'gyro'},
                            {'label': ' Magnetometer', 'value': 'mag'}
                        ],
                        value=['accel', 'gyro', 'mag'],
                        inline=True,
                        style={'display': 'inline-block'}
                    )
                ], style={'display': 'inline-block', 'marginLeft': '20px'})
            ], style={'textAlign': 'center'}),
            html.Div([
                html.Div([
                    html.H3("Lokal (IMU-referanse)", style={'textAlign': 'center'}),
                    dcc.Graph(id='local-graph', style={'height': '70vh'})
                ], style={'width': '50%', 'display': 'inline-block'}),
                html.Div([
                    html.H3("Global (Jord-referanse)", style={'textAlign': 'center'}),
                    dcc.Graph(id='global-graph', style={'height': '70vh'})
                ], style={'width': '50%', 'display': 'inline-block'})
            ]),
            html.Div(id='info-text', style={
                'marginTop': '20px',
                'fontFamily': 'monospace',
                'fontSize': '14px',
                'display': 'flex',
                'flexWrap': 'wrap',
                'justifyContent': 'space-around'
            }),
            dcc.Interval(
                id='interval-component',
                interval=500,  # 500ms = 2 updates per second
                n_intervals=0
            ),
            dcc.Store(id='paused-state', data=False),
            html.Div(id='reset-output', style={'display': 'none'})
        ])
        
        @app.callback(
            Output('reset-output', 'children'),
            Input('reset-button', 'n_clicks'),
            prevent_initial_call=True
        )
        def reset_orientation_callback(n_clicks):
            if n_clicks > 0:
                self.reset_orientation_flag = True
            return ''
        
        @app.callback(
            Output('mag-cal-status', 'children'),
            [Input('mag-cal-start-button', 'n_clicks'),
             Input('mag-cal-finish-button', 'n_clicks'),
             Input('interval-component', 'n_intervals')],
            prevent_initial_call=True
        )
        def mag_cal_callback(start_clicks, finish_clicks, n):
            from dash import callback_context
            
            if not callback_context.triggered:
                return 'Magnetometer: Ikke kalibrert'
            
            # Find which input triggered the callback
            button_id = callback_context.triggered[0]['prop_id'].split('.')[0]
            
            # If interval triggered and we're in calibration mode, show sample count
            if button_id == 'interval-component' and self.mag_calibration_mode:
                return f'🔄 KALIBRERING PÅGÅR - {len(self.mag_cal_samples)} samples samlet (trenger 50+)'
            
            # Handle button clicks
            if button_id == 'mag-cal-start-button' and start_clicks > 0:
                self.start_mag_calibration()
                return '🔄 KALIBRERING PÅGÅR - 0 samples samlet (trenger 50+)'
            
            elif button_id == 'mag-cal-finish-button' and finish_clicks > 0:
                result = self.finish_mag_calibration()
                if result:
                    return f'✅ KALIBRERT - Offset: [{self.mag_offset[0]:.1f}, {self.mag_offset[1]:.1f}, {self.mag_offset[2]:.1f}] µT, Scale: [{self.mag_scale[0]:.2f}, {self.mag_scale[1]:.2f}, {self.mag_scale[2]:.2f}]'
                else:
                    return f'❌ FEIL - For få samples ({len(self.mag_cal_samples)}). Trenger minst 50. Start på nytt!'
            
            # Default status
            if self.mag_calibration_mode:
                return f'🔄 KALIBRERING PÅGÅR - {len(self.mag_cal_samples)} samples samlet (trenger 50+)'
            elif self.mag_calibrated:
                return f'✅ KALIBRERT - Offset: [{self.mag_offset[0]:.1f}, {self.mag_offset[1]:.1f}, {self.mag_offset[2]:.1f}] µT, Scale: [{self.mag_scale[0]:.2f}, {self.mag_scale[1]:.2f}, {self.mag_scale[2]:.2f}]'
            else:
                return 'Magnetometer: Ikke kalibrert'
        
        @app.callback(
            [Output('local-graph', 'figure'),
             Output('global-graph', 'figure'),
             Output('info-text', 'children')],
            Input('interval-component', 'n_intervals'),
            [State('paused-state', 'data'),
             State('sensor-checklist', 'value')]
        )
        def update_graph(n, paused, selected_sensors):
            # If paused, prevent update by raising PreventUpdate
            if paused:
                from dash.exceptions import PreventUpdate
                raise PreventUpdate
            
            if len(self.ax_buffer) == 0:
                # Return empty figures if no data yet
                empty_fig = go.Figure()
                empty_fig.update_layout(
                    scene=dict(
                        xaxis=dict(title='X', range=[-2, 2]),
                        yaxis=dict(title='Y', range=[-2, 2]),
                        zaxis=dict(title='Z', range=[-2, 2]),
                        aspectmode='cube'
                    ),
                    title="Venter på data..."
                )
                return empty_fig, empty_fig, "Ingen data ennå..."
            
            # Get latest LOCAL data
            ax_val = self.ax_buffer[-1] * a_scale
            ay_val = self.ay_buffer[-1] * a_scale
            az_val = self.az_buffer[-1] * a_scale
            
            gx_val = self.gx_buffer[-1] * g_scale
            gy_val = self.gy_buffer[-1] * g_scale
            gz_val = self.gz_buffer[-1] * g_scale
            
            mx_val = self.mx_buffer[-1] * m_scale
            my_val = self.my_buffer[-1] * m_scale
            mz_val = self.mz_buffer[-1] * m_scale
            
            # Get latest GLOBAL data
            ax_global = self.ax_global_buffer[-1] * a_scale if len(self.ax_global_buffer) > 0 else 0
            ay_global = self.ay_global_buffer[-1] * a_scale if len(self.ay_global_buffer) > 0 else 0
            az_global = self.az_global_buffer[-1] * a_scale if len(self.az_global_buffer) > 0 else 0
            
            # Get orientation
            roll = self.roll_buffer[-1] if len(self.roll_buffer) > 0 else 0
            pitch = self.pitch_buffer[-1] if len(self.pitch_buffer) > 0 else 0
            yaw = self.yaw_buffer[-1] if len(self.yaw_buffer) > 0 else 0
            
            # Create LOCAL figure
            fig_local = go.Figure()
            
            # Add accelerometer vector (blue) - LOCAL
            if 'accel' in selected_sensors:
                fig_local.add_trace(go.Scatter3d(
                    x=[0, ax_val], y=[0, ay_val], z=[0, az_val],
                    mode='lines+markers',
                    line=dict(color='blue', width=8),
                    marker=dict(size=[0, 10], color='blue'),
                    name='Akselerometer',
                    hovertemplate=f'aX: {self.ax_buffer[-1]:.1f} mg<br>aY: {self.ay_buffer[-1]:.1f} mg<br>aZ: {self.az_buffer[-1]:.1f} mg'
                ))
                
                # Add cone for accelerometer
                fig_local.add_trace(go.Cone(
                    x=[ax_val], y=[ay_val], z=[az_val],
                    u=[ax_val*0.3], v=[ay_val*0.3], w=[az_val*0.3],
                    colorscale='Blues',
                    showscale=False,
                    sizemode='absolute',
                    sizeref=0.3,
                    showlegend=False
                ))
            
            # Add gyroscope vector (red) - LOCAL
            if 'gyro' in selected_sensors:
                fig_local.add_trace(go.Scatter3d(
                    x=[0, gx_val], y=[0, gy_val], z=[0, gz_val],
                    mode='lines+markers',
                    line=dict(color='red', width=8),
                    marker=dict(size=[0, 10], color='red'),
                    name='Gyroskop',
                    hovertemplate=f'gX: {self.gx_buffer[-1]:.2f} °/s<br>gY: {self.gy_buffer[-1]:.2f} °/s<br>gZ: {self.gz_buffer[-1]:.2f} °/s'
                ))
                
                # Add cone for gyroscope
                fig_local.add_trace(go.Cone(
                    x=[gx_val], y=[gy_val], z=[gz_val],
                    u=[gx_val*0.3], v=[gy_val*0.3], w=[gz_val*0.3],
                    colorscale='Reds',
                    showscale=False,
                    sizemode='absolute',
                    sizeref=0.3,
                    showlegend=False
                ))
            
            # Add magnetometer vector (green) - LOCAL
            if 'mag' in selected_sensors:
                fig_local.add_trace(go.Scatter3d(
                    x=[0, mx_val], y=[0, my_val], z=[0, mz_val],
                    mode='lines+markers',
                    line=dict(color='green', width=8),
                    marker=dict(size=[0, 10], color='green'),
                    name='Magnetometer',
                    hovertemplate=f'mX: {self.mx_buffer[-1]:.1f} µT<br>mY: {self.my_buffer[-1]:.1f} µT<br>mZ: {self.mz_buffer[-1]:.1f} µT'
                ))
                
                # Add cone for magnetometer
                fig_local.add_trace(go.Cone(
                    x=[mx_val], y=[my_val], z=[mz_val],
                    u=[mx_val*0.3], v=[my_val*0.3], w=[mz_val*0.3],
                    colorscale='Greens',
                    showscale=False,
                    sizemode='absolute',
                    sizeref=0.3,
                    showlegend=False
                ))
            
            # Update LOCAL layout
            error_rate = (self.error_count / (self.data_count + self.error_count) * 100) if (self.data_count + self.error_count) > 0 else 0
            
            fig_local.update_layout(
                scene=dict(
                    xaxis=dict(title='X', range=[-2, 2]),
                    yaxis=dict(title='Y', range=[-2, 2]),
                    zaxis=dict(title='Z', range=[-2, 2]),
                    aspectmode='cube'
                ),
                showlegend=True,
                title=f"Lokal (IMU-koordinater)",
                margin=dict(l=0, r=0, t=40, b=0)
            )
            
            # Create GLOBAL figure
            fig_global = go.Figure()
            
            # Add GLOBAL acceleration vector (purple/magenta)
            fig_global.add_trace(go.Scatter3d(
                x=[0, ax_global], y=[0, ay_global], z=[0, az_global],
                mode='lines+markers',
                line=dict(color='purple', width=10),
                marker=dict(size=[0, 12], color='purple'),
                name='Global Akselerasjon',
                hovertemplate=f'aX: {self.ax_global_buffer[-1]:.1f} mg<br>aY: {self.ay_global_buffer[-1]:.1f} mg<br>aZ: {self.az_global_buffer[-1]:.1f} mg'
            ))
            
            # Add cone for global acceleration
            if abs(ax_global) + abs(ay_global) + abs(az_global) > 0.01:  # Only show if significant
                fig_global.add_trace(go.Cone(
                    x=[ax_global], y=[ay_global], z=[az_global],
                    u=[ax_global*0.3], v=[ay_global*0.3], w=[az_global*0.3],
                    colorscale=[[0, 'purple'], [1, 'magenta']],
                    showscale=False,
                    sizemode='absolute',
                    sizeref=0.4,
                    showlegend=False
                ))
            
            # Add reference frame axes (thinner, dashed)
            # X-axis (red)
            fig_global.add_trace(go.Scatter3d(
                x=[0, 1], y=[0, 0], z=[0, 0],
                mode='lines',
                line=dict(color='red', width=2, dash='dash'),
                name='X (øst)',
                showlegend=True
            ))
            # Y-axis (green)
            fig_global.add_trace(go.Scatter3d(
                x=[0, 0], y=[0, 1], z=[0, 0],
                mode='lines',
                line=dict(color='green', width=2, dash='dash'),
                name='Y (nord)',
                showlegend=True
            ))
            # Z-axis (blue)
            fig_global.add_trace(go.Scatter3d(
                x=[0, 0], y=[0, 0], z=[0, 1],
                mode='lines',
                line=dict(color='blue', width=2, dash='dash'),
                name='Z (opp)',
                showlegend=True
            ))
            
            # Update GLOBAL layout
            fig_global.update_layout(
                scene=dict(
                    xaxis=dict(title='X (øst)', range=[-2, 2]),
                    yaxis=dict(title='Y (nord)', range=[-2, 2]),
                    zaxis=dict(title='Z (opp)', range=[-2, 2]),
                    aspectmode='cube'
                ),
                showlegend=True,
                title=f"Global (Jord-koordinater) - R:{roll:.1f}° P:{pitch:.1f}° Y:{yaw:.1f}°",
                margin=dict(l=0, r=0, t=40, b=0)
            )
            
            # Info text - horizontal layout
            info_text = html.Div([
                # Akselerometer
                html.Div([
                    html.Strong("Akselerometer (mg):"),
                    html.Br(),
                    f"X: {self.ax_buffer[-1]:7.1f}",
                    html.Br(),
                    f"Y: {self.ay_buffer[-1]:7.1f}",
                    html.Br(),
                    f"Z: {self.az_buffer[-1]:7.1f}",
                ], style={'padding': '10px', 'minWidth': '180px'}),
                
                # Gyroskop
                html.Div([
                    html.Strong("Gyroskop (°/s):"),
                    html.Br(),
                    f"X: {self.gx_buffer[-1]:7.1f}",
                    html.Br(),
                    f"Y: {self.gy_buffer[-1]:7.1f}",
                    html.Br(),
                    f"Z: {self.gz_buffer[-1]:7.1f}",
                ], style={'padding': '10px', 'minWidth': '180px'}),
                
                # Magnetometer
                html.Div([
                    html.Strong("Magnetometer (µT):"),
                    html.Br(),
                    f"X: {self.mx_buffer[-1]:7.1f}",
                    html.Br(),
                    f"Y: {self.my_buffer[-1]:7.1f}",
                    html.Br(),
                    f"Z: {self.mz_buffer[-1]:7.1f}",
                ], style={'padding': '10px', 'minWidth': '180px'}),
                
                # Orientering
                html.Div([
                    html.Strong("Orientering:"),
                    html.Br(),
                    f"Roll:  {roll:7.1f}°",
                    html.Br(),
                    f"Pitch: {pitch:7.1f}°",
                    html.Br(),
                    f"Yaw:   {yaw:7.1f}°",
                ], style={'padding': '10px', 'minWidth': '180px'}),
                
                # Global akselerasjon
                html.Div([
                    html.Strong("Global akselerasjon (mg):"),
                    html.Br(),
                    f"X: {self.ax_global_buffer[-1]:7.1f}",
                    html.Br(),
                    f"Y: {self.ay_global_buffer[-1]:7.1f}",
                    html.Br(),
                    f"Z: {self.az_global_buffer[-1]:7.1f}",
                ], style={'padding': '10px', 'minWidth': '200px'}),
                
                # Samples
                html.Div([
                    html.Strong(f"Samples: {self.data_count}")
                ], style={'padding': '10px', 'minWidth': '150px'})
            ], style={'display': 'flex', 'flexWrap': 'wrap', 'justifyContent': 'space-around'})
            
            # Print to console
            print(f"\rSamples: {self.data_count} | "
                  f"a:[{self.ax_buffer[-1]:.1f}, {self.ay_buffer[-1]:.1f}, {self.az_buffer[-1]:.1f}] | "
                  f"g:[{self.gx_buffer[-1]:.1f}, {self.gy_buffer[-1]:.1f}, {self.gz_buffer[-1]:.1f}] | "
                  f"m:[{self.mx_buffer[-1]:.1f}, {self.my_buffer[-1]:.1f}, {self.mz_buffer[-1]:.1f}] | "
                  f"global:[{self.ax_global_buffer[-1]:.1f}, {self.ay_global_buffer[-1]:.1f}, {self.az_global_buffer[-1]:.1f}] | "
                  f"R:{roll:.1f}° P:{pitch:.1f}° Y:{yaw:.1f}°",
                  end='', flush=True)
            
            return fig_local, fig_global, info_text
        
        @app.callback(
            [Output('paused-state', 'data'),
             Output('pause-button', 'children'),
             Output('pause-button', 'style')],
            Input('pause-button', 'n_clicks'),
            State('paused-state', 'data')
        )
        def toggle_pause(n_clicks, paused):
            if n_clicks == 0:
                return False, '⏸ Pause', {
                    'fontSize': '18px',
                    'padding': '10px 30px',
                    'margin': '10px',
                    'backgroundColor': '#4CAF50',
                    'color': 'white',
                    'border': 'none',
                    'borderRadius': '5px',
                    'cursor': 'pointer'
                }
            
            new_paused = not paused
            
            if new_paused:
                return True, '▶ Resume', {
                    'fontSize': '18px',
                    'padding': '10px 30px',
                    'margin': '10px',
                    'backgroundColor': '#f44336',
                    'color': 'white',
                    'border': 'none',
                    'borderRadius': '5px',
                    'cursor': 'pointer'
                }
            else:
                return False, '⏸ Pause', {
                    'fontSize': '18px',
                    'padding': '10px 30px',
                    'margin': '10px',
                    'backgroundColor': '#4CAF50',
                    'color': 'white',
                    'border': 'none',
                    'borderRadius': '5px',
                    'cursor': 'pointer'
                }
        
        print("\nLive 3D plotting started.")
        print(f"Reading from {self.port}...")
        print("Opening browser at http://127.0.0.1:8050")
        print("-" * 50)
        print("Press Ctrl+C in terminal to stop.")
        print("-" * 50)
        
        # Open browser after a short delay
        def open_browser():
            time.sleep(2)
            webbrowser.open('http://127.0.0.1:8050')
        
        threading.Thread(target=open_browser, daemon=True).start()
        
        # Run the server in blocking mode
        print("Starting Flask server... (this will block)")
        app.run(debug=False, host='127.0.0.1', port=8050)


def main():
    print("=" * 60)
    print("IMU Live Plotter")
    print("=" * 60)
    
    # Create IMU reader
    imu_reader = IMULiveReader(
        port='/dev/ttyUSB1',
        baudrate=115200,
        buffer_size=100  # Keep last 100 samples
    )
    
    # Connect to device
    if not imu_reader.connect():
        print("Failed to connect to IMU. Exiting...")
        return
    
    print("Connection successful!")
    
    # Start reading data in background
    imu_reader.start_reading()
    
    print("Background data reading started...")
    
    # Wait a moment to collect some initial data
    time.sleep(1)
    
    print(f"Initial data collected: {imu_reader.data_count} samples")
    
    # Start live visualization (this will block)
    try:
        imu_reader.visualize_live(update_interval=500)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        print("Stopping data acquisition...")
        imu_reader.stop_reading()
        print("Done.")


if __name__ == "__main__":
    main()
