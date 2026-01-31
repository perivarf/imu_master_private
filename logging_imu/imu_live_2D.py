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
        self.temp_buffer = deque(maxlen=buffer_size)
        self.gps_lat_buffer = deque(maxlen=buffer_size)
        self.gps_lng_buffer = deque(maxlen=buffer_size)
        self.gps_alt_buffer = deque(maxlen=buffer_size)
        self.gps_speed_buffer = deque(maxlen=buffer_size)
        self.gps_course_buffer = deque(maxlen=buffer_size)
        
        self.serial_conn = None
        self.running = False
        self.data_count = 0
        self.error_count = 0
        self.last_error_print = 0
        self.reset_orientation_flag = False
        self.first_error_printed = False  # Track if we've printed first error line
        
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
        """Parse CSV data from STM32: HH:MM:SS.mmm,ax,ay,az,gx,gy,gz,t,lat,lng,alt,speed_mps,course_deg"""
        try:
            parts = line.strip().split(',')
            if len(parts) < 13:
                return None
            if parts[0] == 'gps_time_ms' or parts[0] == 'rtcDate' or 'time' in parts[0].lower():
                return None
            data = {
                'rtcTime': parts[0],
                'aX': float(parts[1]),
                'aY': float(parts[2]),
                'aZ': float(parts[3]),
                'gX': float(parts[4]),
                'gY': float(parts[5]),
                'gZ': float(parts[6]),
                'temp': float(parts[7]),
                'gps_lat': float(parts[8]),
                'gps_lng': float(parts[9]),
                'gps_alt': float(parts[10]),
                'gps_speed': float(parts[11]),
                'gps_course': float(parts[12]),
            }
            for key, val in data.items():
                if key != 'rtcTime' and (abs(val) > 10000):
                    return None
            return data
        except (ValueError, IndexError, TypeError):
            self.error_count += 1
            if self.data_count > 0 and not self.first_error_printed:
                print(f"\n[ERROR] First corrupted line: {line}")
                self.first_error_printed = True
            if self.error_count - self.last_error_print >= 10:
                print(f"\n[Warning] Skipped {self.error_count} corrupted lines")
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
                            self.temp_buffer.append(data['temp'])
                            self.gps_lat_buffer.append(data['gps_lat'])
                            self.gps_lng_buffer.append(data['gps_lng'])
                            self.gps_alt_buffer.append(data['gps_alt'])
                            self.gps_speed_buffer.append(data['gps_speed'])
                            self.gps_course_buffer.append(data['gps_course'])
                            
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
    
    
    def update_orientation(self, data):
        """Store orientation data received from STM32"""
        # Data is already processed on the STM32 side
        # No additional processing needed, just pass through the data
        pass
    
    
    def visualize_live(self, update_interval=100):
        """
        Create live 2D time-series visualization of IMU data using Dash
        
        Parameters:
        update_interval (int): Update interval in milliseconds
        """
        # Scaling factors for visualization
        a_scale = 1000.0  # Acceleration scaling (G -> mg)
        a_axis_range = 12000  # Plot range in mg
        g_scale = 1.0  # Gyroscope scaling
        
        # Create Dash app
        app = Dash(__name__)
        
        app.layout = html.Div([
            html.H1("IMU 2D Time Series - Live", style={'textAlign': 'center'}),
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
                html.Div([
                    html.Label('Vis sensorer:', style={'fontWeight': 'bold', 'marginRight': '10px'}),
                    dcc.Checklist(
                        id='sensor-checklist',
                        options=[
                            {'label': ' Akselerometer', 'value': 'accel'},
                            {'label': ' Gyroskop', 'value': 'gyro'},
                            {'label': ' Temperatur', 'value': 'temp'}
                        ],
                        value=['accel', 'gyro', 'temp'],
                        inline=True,
                        style={'display': 'inline-block'}
                    )
                ], style={'display': 'inline-block', 'marginLeft': '20px'})
            ], style={'textAlign': 'center'}),
            html.Div(id='info-text', style={
                'marginTop': '10px',
                'marginBottom': '10px',
                'fontFamily': 'monospace',
                'fontSize': '14px',
                'display': 'flex',
                'flexWrap': 'wrap',
                'justifyContent': 'space-around'
            }),
            html.Div([
                html.Div([
                    html.H3("Akselerometer (mg)", style={'textAlign': 'center'}),
                    dcc.Graph(id='accel-graph', style={'height': '24vh'})
                ], id='accel-container'),
                html.Div([
                    html.H3("Gyroskop (°/s)", style={'textAlign': 'center'}),
                    dcc.Graph(id='gyro-graph', style={'height': '24vh'})
                ], id='gyro-container'),
                html.Div([
                    html.H3("Temperatur (°C)", style={'textAlign': 'center'}),
                    dcc.Graph(id='temp-graph', style={'height': '20vh'})
                ], id='temp-container')
            ]),
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
            [Output('accel-graph', 'figure'),
             Output('gyro-graph', 'figure'),
             Output('temp-graph', 'figure'),
             Output('accel-container', 'style'),
             Output('gyro-container', 'style'),
             Output('temp-container', 'style'),
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
                empty_fig.update_layout(title="Venter på data...")
                accel_style = {'display': 'none'} if 'accel' not in selected_sensors else {}
                gyro_style = {'display': 'none'} if 'gyro' not in selected_sensors else {}
                temp_style = {'display': 'none'} if 'temp' not in selected_sensors else {}
                return empty_fig, empty_fig, empty_fig, accel_style, gyro_style, temp_style, "Ingen data ennå..."
            
            time_vals = list(self.time_buffer)
            accel_fig = go.Figure()
            gyro_fig = go.Figure()
            temp_fig = go.Figure()
            accel_style = {'display': 'none'} if 'accel' not in selected_sensors else {}
            gyro_style = {'display': 'none'} if 'gyro' not in selected_sensors else {}
            temp_style = {'display': 'none'} if 'temp' not in selected_sensors else {}

            if 'accel' in selected_sensors:
                accel_fig.add_trace(go.Scatter(
                    x=time_vals, y=[v * a_scale for v in self.ax_buffer],
                    mode='lines', name='aX (mg)'
                ))
                accel_fig.add_trace(go.Scatter(
                    x=time_vals, y=[v * a_scale for v in self.ay_buffer],
                    mode='lines', name='aY (mg)'
                ))
                accel_fig.add_trace(go.Scatter(
                    x=time_vals, y=[v * a_scale for v in self.az_buffer],
                    mode='lines', name='aZ (mg)'
                ))
            accel_fig.update_layout(
                yaxis=dict(title='mg', range=[-a_axis_range, a_axis_range]),
                margin=dict(l=40, r=10, t=20, b=30),
                legend=dict(orientation='h')
            )

            if 'gyro' in selected_sensors:
                gyro_fig.add_trace(go.Scatter(
                    x=time_vals, y=list(self.gx_buffer),
                    mode='lines', name='gX (°/s)'
                ))
                gyro_fig.add_trace(go.Scatter(
                    x=time_vals, y=list(self.gy_buffer),
                    mode='lines', name='gY (°/s)'
                ))
                gyro_fig.add_trace(go.Scatter(
                    x=time_vals, y=list(self.gz_buffer),
                    mode='lines', name='gZ (°/s)'
                ))
            gyro_fig.update_layout(
                yaxis=dict(title='°/s'),
                margin=dict(l=40, r=10, t=20, b=30),
                legend=dict(orientation='h')
            )

            if 'temp' in selected_sensors:
                temp_fig.add_trace(go.Scatter(
                    x=time_vals, y=list(self.temp_buffer),
                    mode='lines', name='Temp (°C)'
                ))
                temp_fig.update_layout(
                    yaxis=dict(title='°C'),
                    margin=dict(l=40, r=10, t=20, b=30),
                    showlegend=False
                )
            
            # Info text - horizontal layout
            info_text = html.Div([
                # Akselerometer
                html.Div([
                    html.Strong("Akselerometer (mg):"),
                    html.Br(),
                    f"X: {self.ax_buffer[-1] * 1000:7.1f}",
                    html.Br(),
                    f"Y: {self.ay_buffer[-1] * 1000:7.1f}",
                    html.Br(),
                    f"Z: {self.az_buffer[-1] * 1000:7.1f}",
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
                
                # Temperature
                html.Div([
                    html.Strong("Temperatur (°C):"),
                    html.Br(),
                    f"T: {self.temp_buffer[-1]:7.1f}",
                ], style={'padding': '10px', 'minWidth': '180px'}),
                
                # GPS info
                html.Div([
                    html.Strong("GPS:"),
                    html.Br(),
                    f"Lat: {self.gps_lat_buffer[-1]:7.4f}°",
                    html.Br(),
                    f"Lng: {self.gps_lng_buffer[-1]:7.4f}°",
                ], style={'padding': '10px', 'minWidth': '180px'}),
                
                # Samples
                html.Div([
                    html.Strong(f"Samples: {self.data_count}")
                ], style={'padding': '10px', 'minWidth': '150px'})
            ], style={'display': 'flex', 'flexWrap': 'wrap', 'justifyContent': 'space-around'})
            
            # Print to console
            print(f"\rSamples: {self.data_count} | "
                f"a:[{self.ax_buffer[-1] * 1000:.1f}, {self.ay_buffer[-1] * 1000:.1f}, {self.az_buffer[-1] * 1000:.1f}] | "
                  f"g:[{self.gx_buffer[-1]:.1f}, {self.gy_buffer[-1]:.1f}, {self.gz_buffer[-1]:.1f}] | "
                  f"T:{self.temp_buffer[-1]:.1f}°C | "
                  f"GPS:[{self.gps_lat_buffer[-1]:.4f}, {self.gps_lng_buffer[-1]:.4f}]",
                  end='', flush=True)
            
            return accel_fig, gyro_fig, temp_fig, accel_style, gyro_style, temp_style, info_text
        
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
                # Paused
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
                # Running
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
        port='/dev/ttyUSB0',
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
