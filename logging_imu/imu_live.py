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
from dash.dependencies import Input, Output
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
        self.mx_buffer = deque(maxlen=buffer_size)
        self.my_buffer = deque(maxlen=buffer_size)
        self.mz_buffer = deque(maxlen=buffer_size)
        
        self.serial_conn = None
        self.running = False
        self.data_count = 0
        self.error_count = 0
        self.last_error_print = 0
        
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
                            # Add to buffers
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
            html.H1("IMU 3D Vektorer (Live)", style={'textAlign': 'center'}),
            dcc.Graph(id='live-graph', style={'height': '80vh'}),
            dcc.Interval(
                id='interval-component',
                interval=500,  # 500ms = 2 updates per second
                n_intervals=0
            ),
            html.Div(id='info-text', style={
                'textAlign': 'center',
                'fontFamily': 'monospace',
                'fontSize': '14px',
                'marginTop': '10px'
            })
        ])
        
        @app.callback(
            [Output('live-graph', 'figure'),
             Output('info-text', 'children')],
            Input('interval-component', 'n_intervals')
        )
        def update_graph(n):
            if len(self.ax_buffer) == 0:
                # Return empty figure if no data yet
                fig = go.Figure()
                fig.update_layout(
                    scene=dict(
                        xaxis=dict(title='X', range=[-2, 2]),
                        yaxis=dict(title='Y', range=[-2, 2]),
                        zaxis=dict(title='Z', range=[-2, 2]),
                        aspectmode='cube'
                    ),
                    title="Venter på data..."
                )
                return fig, "Ingen data ennå..."
            
            # Get latest data
            ax_val = self.ax_buffer[-1] * a_scale
            ay_val = self.ay_buffer[-1] * a_scale
            az_val = self.az_buffer[-1] * a_scale
            
            gx_val = self.gx_buffer[-1] * g_scale
            gy_val = self.gy_buffer[-1] * g_scale
            gz_val = self.gz_buffer[-1] * g_scale
            
            mx_val = self.mx_buffer[-1] * m_scale
            my_val = self.my_buffer[-1] * m_scale
            mz_val = self.mz_buffer[-1] * m_scale
            
            # Create figure
            fig = go.Figure()
            
            # Add accelerometer vector (blue)
            fig.add_trace(go.Scatter3d(
                x=[0, ax_val], y=[0, ay_val], z=[0, az_val],
                mode='lines+markers',
                line=dict(color='blue', width=8),
                marker=dict(size=[0, 10], color='blue'),
                name='Akselerometer',
                hovertemplate=f'aX: {self.ax_buffer[-1]:.1f} mg<br>aY: {self.ay_buffer[-1]:.1f} mg<br>aZ: {self.az_buffer[-1]:.1f} mg'
            ))
            
            # Add cone for accelerometer
            fig.add_trace(go.Cone(
                x=[ax_val], y=[ay_val], z=[az_val],
                u=[ax_val*0.3], v=[ay_val*0.3], w=[az_val*0.3],
                colorscale='Blues',
                showscale=False,
                sizemode='absolute',
                sizeref=0.3,
                showlegend=False
            ))
            
            # Add gyroscope vector (red)
            fig.add_trace(go.Scatter3d(
                x=[0, gx_val], y=[0, gy_val], z=[0, gz_val],
                mode='lines+markers',
                line=dict(color='red', width=8),
                marker=dict(size=[0, 10], color='red'),
                name='Gyroskop',
                hovertemplate=f'gX: {self.gx_buffer[-1]:.2f} °/s<br>gY: {self.gy_buffer[-1]:.2f} °/s<br>gZ: {self.gz_buffer[-1]:.2f} °/s'
            ))
            
            # Add cone for gyroscope
            fig.add_trace(go.Cone(
                x=[gx_val], y=[gy_val], z=[gz_val],
                u=[gx_val*0.3], v=[gy_val*0.3], w=[gz_val*0.3],
                colorscale='Reds',
                showscale=False,
                sizemode='absolute',
                sizeref=0.3,
                showlegend=False
            ))
            
            # Add magnetometer vector (green)
            fig.add_trace(go.Scatter3d(
                x=[0, mx_val], y=[0, my_val], z=[0, mz_val],
                mode='lines+markers',
                line=dict(color='green', width=8),
                marker=dict(size=[0, 10], color='green'),
                name='Magnetometer',
                hovertemplate=f'mX: {self.mx_buffer[-1]:.1f} µT<br>mY: {self.my_buffer[-1]:.1f} µT<br>mZ: {self.mz_buffer[-1]:.1f} µT'
            ))
            
            # Add cone for magnetometer
            fig.add_trace(go.Cone(
                x=[mx_val], y=[my_val], z=[mz_val],
                u=[mx_val*0.3], v=[my_val*0.3], w=[mz_val*0.3],
                colorscale='Greens',
                showscale=False,
                sizemode='absolute',
                sizeref=0.3,
                showlegend=False
            ))
            
            # Update layout
            error_rate = (self.error_count / (self.data_count + self.error_count) * 100) if (self.data_count + self.error_count) > 0 else 0
            
            fig.update_layout(
                scene=dict(
                    xaxis=dict(title='X', range=[-2, 2]),
                    yaxis=dict(title='Y', range=[-2, 2]),
                    zaxis=dict(title='Z', range=[-2, 2]),
                    aspectmode='cube'
                ),
                showlegend=True,
                title=f"IMU 3D Vektorer - Samples: {self.data_count} | Errors: {self.error_count} ({error_rate:.1f}%)",
                margin=dict(l=0, r=0, t=40, b=0)
            )
            
            # Info text
            info_text = f"Aksel: [{self.ax_buffer[-1]:.1f}, {self.ay_buffer[-1]:.1f}, {self.az_buffer[-1]:.1f}] mg | " \
                       f"Gyro: [{self.gx_buffer[-1]:.2f}, {self.gy_buffer[-1]:.2f}, {self.gz_buffer[-1]:.2f}] °/s | " \
                       f"Mag: [{self.mx_buffer[-1]:.1f}, {self.my_buffer[-1]:.1f}, {self.mz_buffer[-1]:.1f}] µT"
            
            # Print to console
            print(f"\rSamples: {self.data_count} | "
                  f"a:[{self.ax_buffer[-1]:.1f}, {self.ay_buffer[-1]:.1f}, {self.az_buffer[-1]:.1f}] | "
                  f"g:[{self.gx_buffer[-1]:.2f}, {self.gy_buffer[-1]:.2f}, {self.gz_buffer[-1]:.2f}] | "
                  f"m:[{self.mx_buffer[-1]:.1f}, {self.my_buffer[-1]:.1f}, {self.mz_buffer[-1]:.1f}]",
                  end='', flush=True)
            
            return fig, info_text
        
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
