import serial
import csv
import time
from datetime import datetime

# Configuration
PORT = 'COM4'  # Linux/Mac: '/dev/ttyUSB0' or '/dev/ttyACM0'
BAUD = 115200
TIMEOUT = 2
CSV_FILE = f"pendulum_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

def serial_ports():
    """List available serial ports"""
    import serial.tools.list_ports
    return [port.device for port in serial.tools.list_ports.comports()]

def main():
    print("Available ports:")
    print("\n".join(serial_ports()))
    
    try:
        with serial.Serial(PORT, BAUD, timeout=TIMEOUT) as ser, \
             open(CSV_FILE, 'w', newline='') as f:

            writer = csv.writer(f)
            headers = ['timestamp_ms','x_pos(m)','phi(rad)','u(PWM)','x_dot','phi_dot']
            writer.writerow(headers)
            
            print(f"\nRecording to {CSV_FILE} (Press Ctrl+C to stop)")
            print("Waiting for Arduino...")
            
            time.sleep(2)  # Wait for Arduino reset
            ser.flushInput()
            
            # Debug: Verify connection
            ser.write(b'?')  # Send test character
            if ser.in_waiting:
                print(f"Connection test OK: {ser.readline().decode().strip()}")
            else:
                print("No response from Arduino - check wiring!")

            print("\nStarting capture...")
            print(",".join(headers))  # Print headers to console
            
            while True:
                if ser.in_waiting:
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                    if line:
                        print(line)  # Debug: Show live data
                        try:
                            row = [float(x) for x in line.split(',')]
                            if len(row) == len(headers):
                                writer.writerow(row)
                                f.flush()  # Immediate save
                            else:
                                print(f"! Invalid row: {line}")
                        except ValueError:
                            print(f"! Parse error: {line}")

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nRecording stopped by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        print(f"Data saved to {CSV_FILE}")

if __name__ == "__main__":
    main()
