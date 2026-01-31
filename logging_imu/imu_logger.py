# Author: Per-Ivar Faust
# Simple IMU data logger: Logs X seconds of serial data to CSV file

import serial
import time
from datetime import datetime
from pathlib import Path
import sys

class IMUSimpleLogger:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, duration=60):
        """
        Initialize IMU simple logger
        
        Parameters:
        port (str): Serial port for IMU device
        baudrate (int): Baud rate for serial connection
        duration (int): Duration to log in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.duration = duration
        self.serial_conn = None
        self.data_list = []
        self.error_count = 0
        self.data_count = 0
        
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
            )
            
            # Clear any pending data
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            # Wait for first data to confirm connection
            print(f"Connecting to {self.port} at {self.baudrate} baud...")
            waiting = 0
            while waiting < 50 and not self.serial_conn.in_waiting:
                time.sleep(0.1)
                waiting += 1
            
            if waiting == 0:
                print("WARNING: No data received yet. Is the IMU sending data?")
            else:
                print(f"Connection established. Received first data after {waiting*0.1:.1f}s")
            
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
            
            # Validate numeric fields
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
            
            # Sanity check: reject obviously bad values
            for key, val in data.items():
                if key != 'rtcTime' and (abs(val) > 100000):
                    return None
            
            return data
        except (ValueError, IndexError, TypeError):
            self.error_count += 1
            return None
    
    def log_data(self):
        """Log data from serial port for specified duration"""
        if not self.serial_conn:
            print("ERROR: Not connected to serial port")
            return False
        
        print(f"\nLogging data for {self.duration} seconds...")
        print("(Press Ctrl-C to stop and save data)\n")
        start_time = time.time()
        line_buffer = ""
        last_print_time = start_time
        
        try:
            while time.time() - start_time < self.duration:
                try:
                    if self.serial_conn.in_waiting:
                        # Read available bytes
                        chunk = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                        line_buffer += chunk
                        
                        # Process complete lines
                        while '\n' in line_buffer:
                            line, line_buffer = line_buffer.split('\n', 1)
                            line = line.strip()
                            
                            if not line:
                                continue
                            
                            data = self.parse_line(line)
                            
                            if data:
                                self.data_list.append(data)
                                self.data_count += 1
                            
                            # Print progress every second
                            current_time = time.time()
                            if current_time - last_print_time >= 1.0:
                                elapsed = current_time - start_time
                                print(f"  {elapsed:.1f}s: {self.data_count} samples collected ({self.error_count} errors)")
                                last_print_time = current_time
                    
                    time.sleep(0.01)
                    
                except Exception as e:
                    print(f"Error during logging: {e}")
                    return False
            
            print(f"\nLogging complete!")
            print(f"Total samples: {self.data_count}")
            print(f"Errors: {self.error_count}")
            
            return True
        
        except KeyboardInterrupt:
            # Re-raise to be caught by run()
            raise
    
    def save_to_csv(self):
        """Save logged data to CSV file with timestamp"""
        if not self.data_list:
            print("ERROR: No data to save")
            return False
        
        # Create logfiles directory if it doesn't exist
        logfiles_dir = Path('logfiles')
        logfiles_dir.mkdir(exist_ok=True)
        
        # Generate filename with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = logfiles_dir / f'dataLog_{timestamp}.csv'
        
        try:
            # Write CSV with headers
            with open(filename, 'w') as f:
                # Write header
                headers = ['rtcTime', 'aX', 'aY', 'aZ', 'gX', 'gY', 'gZ', 'temp', 
                          'gps_lat', 'gps_lng', 'gps_alt', 'gps_speed', 'gps_course']
                f.write(','.join(headers) + '\n')
                
                # Write data rows
                for data in self.data_list:
                    row = [
                        str(data['rtcTime']),
                        f"{data['aX']:.6f}",
                        f"{data['aY']:.6f}",
                        f"{data['aZ']:.6f}",
                        f"{data['gX']:.6f}",
                        f"{data['gY']:.6f}",
                        f"{data['gZ']:.6f}",
                        f"{data['temp']:.6f}",
                        f"{data['gps_lat']:.6f}",
                        f"{data['gps_lng']:.6f}",
                        f"{data['gps_alt']:.6f}",
                        f"{data['gps_speed']:.6f}",
                        f"{data['gps_course']:.6f}",
                    ]
                    f.write(','.join(row) + '\n')
            
            print(f"Data saved to: {filename}")
            return True
            
        except Exception as e:
            print(f"Error saving to CSV: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection"""
        if self.serial_conn:
            self.serial_conn.close()
            print("Serial connection closed")
    
    def run(self):
        """Main logging routine"""
        try:
            if not self.connect():
                return False
            
            if not self.log_data():
                return False
            
            if not self.save_to_csv():
                return False
            
            return True
            
        except KeyboardInterrupt:
            print("\n\nInterrupted by user (Ctrl-C)")
            if self.data_list:
                print(f"Saving {self.data_count} samples before exiting...")
                self.save_to_csv()
            return True
            
        finally:
            self.disconnect()


def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Simple IMU data logger')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--duration', type=int, default=60, help='Logging duration in seconds (default: 60)')
    
    args = parser.parse_args()
    
    logger = IMUSimpleLogger(
        port=args.port,
        baudrate=args.baud,
        duration=args.duration
    )
    
    success = logger.run()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
