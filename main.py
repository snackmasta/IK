import time
import math
import sys
import socket
import json
from mpu6050 import MPU6050
from hmc5883l import HMC5883L

# Local Declination Angle (optional, adjust for your location)
# Find yours at http://www.magnetic-declination.com/
DECLINATION_ANGLE_DEG = 0.0  # Set this for accurate true north calculation

# UDP Configuration
UDP_IP = "127.0.0.1"  # Target IP address (change to receiver's IP or "255.255.255.255" for broadcast)
UDP_PORT = 5005       # Target UDP Port

def calculate_heading(mag_x, mag_y, declination_deg=0.0):
    """
    Calculates the heading in degrees (0 - 360) from magnetometer X and Y readings.
    """
    heading_rad = math.atan2(mag_y, mag_x)
    
    # Add declination angle (convert to radians)
    declination_rad = math.radians(declination_deg)
    heading_rad += declination_rad
    
    # Correct for when signs are discarded
    if heading_rad < 0:
        heading_rad += 2 * math.pi
    if heading_rad > 2 * math.pi:
        heading_rad -= 2 * math.pi
        
    # Convert to degrees
    return math.degrees(heading_rad)

def main():
    print("=" * 60)
    print("       Raspberry Pi 4 Sensor Reader: IMU & Magnetometer")
    print("=" * 60)
    print(f"Connecting to I2C bus 1...")
    
    # Initialize sensors
    imu = MPU6050(bus_id=1)
    mag = HMC5883L(bus_id=1)
    
    try:
        print("Initializing MPU6050 IMU (0x68)...", end=" ")
        imu.initialize()
        print("[SUCCESS]")
        
        print("Initializing HMC5883L Magnetometer (0x1E)...", end=" ")
        mag.initialize()
        print("[SUCCESS]")
    except Exception as e:
        print(f"\n[ERROR] Initialization failed: {e}")
        print("Make sure your connections are secure and I2C is enabled on the Pi.")
        sys.exit(1)
        
    # Set up UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if UDP_IP == "255.255.255.255":
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    print(f"UDP Socket initialized targeting {UDP_IP}:{UDP_PORT}")
    
    print("\nStarting data stream. Press Ctrl+C to stop.\n")
    print("-" * 75)
    print(f"{'Accel (g)':^18} | {'Gyro (°/s)':^18} | {'Mag (µT)':^18} | {'Heading':^8} | {'Temp':^6}")
    print(f"{'X / Y / Z':^18} | {'X / Y / Z':^18} | {'X / Y / Z':^18} | {'(Deg)':^8} | {'(°C)':^6}")
    print("-" * 75)
    
    try:
        while True:
            # Read IMU
            accel = imu.read_accel_data()
            gyro = imu.read_gyro_data()
            temp = imu.read_temp()
            
            # Read Magnetometer
            magnetic = mag.read_magnetometer_data()
            
            # Compute Heading
            heading = calculate_heading(magnetic['x'], magnetic['y'], DECLINATION_ANGLE_DEG)
            
            # Construct payload
            payload = {
                "timestamp": time.time(),
                "accel": accel,
                "gyro": gyro,
                "mag": magnetic,
                "heading": heading,
                "temp": temp
            }
            
            # Send via UDP
            try:
                message = json.dumps(payload).encode('utf-8')
                sock.sendto(message, (UDP_IP, UDP_PORT))
            except Exception as udp_err:
                # Do not crash the loop if UDP send temporarily fails
                pass
            
            # Print formatted data line
            accel_str = f"{accel['x']:.2f},{accel['y']:.2f},{accel['z']:.2f}"
            gyro_str = f"{gyro['x']:.1f},{gyro['y']:.1f},{gyro['z']:.1f}"
            mag_str = f"{magnetic['x']:.1f},{magnetic['y']:.1f},{magnetic['z']:.1f}"
            
            sys.stdout.write(
                f"\r{accel_str:^18} | {gyro_str:^18} | {mag_str:^18} | {heading:^8.1f} | {temp:^6.1f}"
            )
            sys.stdout.flush()
            
            time.sleep(0.1) # 10Hz sampling rate
            
    except KeyboardInterrupt:
        print("\n\nStopping sensor data stream.")
    finally:
        imu.close()
        mag.close()
        sock.close()
        print("Connections closed cleanly.")

if __name__ == '__main__':
    main()

