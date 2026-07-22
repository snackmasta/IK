import time
import math
import sys
import socket
import json
from mpu6050 import MPU6050
from hmc5883l import HMC5883L
from pose_estimator import PoseEstimator

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
    print("                  6DoF VR Controller Estimation")
    print("=" * 60)
    print(f"Connecting to I2C bus 1...")
    
    # Initialize sensors
    imu = MPU6050(bus_id=1)
    mag = HMC5883L(bus_id=1)
    
    # Initialize 6DoF Pose Estimator
    pose_estimator = PoseEstimator(beta=0.1, vel_decay=0.95, zero_velocity_thresh=0.15)
    
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
    
    print("\nStarting 6DoF data stream. Press Ctrl+C to stop.\n")
    print("-" * 85)
    print(f"{'Position (m)':^18} | {'Euler Angles (deg)':^24} | {'Quaternion (w,x,y,z)':^24} | {'Heading':^8}")
    print(f"{'X / Y / Z':^18} | {'Roll / Pitch / Yaw':^24} | {'w / x / y / z':^24} | {'(Deg)':^8}")
    print("-" * 85)
    
    last_time = time.time()
    
    try:
        while True:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # Read IMU & Magnetometer
            accel = imu.read_accel_data()
            gyro = imu.read_gyro_data()
            temp = imu.read_temp()
            magnetic = mag.read_magnetometer_data()
            
            # Compute Heading
            heading = calculate_heading(magnetic['x'], magnetic['y'], DECLINATION_ANGLE_DEG)
            
            # Compute 6DoF Pose (Rotation & Translation)
            pose = pose_estimator.update(accel_g=accel, gyro_deg=gyro, mag_ut=magnetic, dt=dt)
            
            # Construct comprehensive payload
            payload = {
                "timestamp": current_time,
                "dt": dt,
                "rotation": pose["rotation"],
                "translation": pose["translation"],
                "heading": heading,
                "raw_imu": {
                    "accel": accel,
                    "gyro": gyro,
                    "mag": magnetic,
                    "temp": temp
                }
            }
            
            # Send via UDP
            try:
                message = json.dumps(payload).encode('utf-8')
                sock.sendto(message, (UDP_IP, UDP_PORT))
            except Exception:
                pass
            
            # Formatted console telemetry output
            pos = pose["translation"]["position"]
            euler = pose["rotation"]["euler"]
            quat = pose["rotation"]["quaternion"]
            
            pos_str = f"{pos['x']:.2f},{pos['y']:.2f},{pos['z']:.2f}"
            euler_str = f"{euler['roll']:.1f},{euler['pitch']:.1f},{euler['yaw']:.1f}"
            quat_str = f"{quat['w']:.2f},{quat['x']:.2f},{quat['y']:.2f},{quat['z']:.2f}"
            
            sys.stdout.write(
                f"\r{pos_str:^18} | {euler_str:^24} | {quat_str:^24} | {heading:^8.1f}"
            )
            sys.stdout.flush()
            
            time.sleep(0.02) # 50Hz update rate for smoother VR controller tracking
            
    except KeyboardInterrupt:
        print("\n\nStopping sensor data stream.")
    finally:
        imu.close()
        mag.close()
        sock.close()
        print("Connections closed cleanly.")

if __name__ == '__main__':
    main()


