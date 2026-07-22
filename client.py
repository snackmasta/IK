import socket
import json
import sys

# Listen on all interfaces on the configured port
LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5005

def main():
    print("=" * 60)
    print("      UDP Sensor Data Receiver - 6DoF VR Pose Client")
    print("=" * 60)
    print(f"Binding to {LISTEN_IP}:{LISTEN_PORT}...")

    # Set up UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        sock.bind((LISTEN_IP, LISTEN_PORT))
        print("[SUCCESS] Listening for incoming 6DoF UDP telemetry packets.\n")
    except Exception as e:
        print(f"[ERROR] Failed to bind to socket: {e}")
        sys.exit(1)

    print("-" * 85)
    print(f"{'Position (m)':^18} | {'Euler Angles (deg)':^24} | {'Quaternion (w,x,y,z)':^24} | {'Heading':^8}")
    print(f"{'X / Y / Z':^18} | {'Roll / Pitch / Yaw':^24} | {'w / x / y / z':^24} | {'(Deg)':^8}")
    print("-" * 85)

    try:
        while True:
            data, addr = sock.recvfrom(4096) # Buffer size 4096 bytes
            try:
                # Decode JSON payload
                payload = json.loads(data.decode('utf-8'))
                
                # Extract rotation and translation states
                rotation = payload.get("rotation", {})
                translation = payload.get("translation", {})
                heading = payload.get("heading", 0.0)
                
                pos = translation.get("position", {"x": 0.0, "y": 0.0, "z": 0.0})
                euler = rotation.get("euler", {"roll": 0.0, "pitch": 0.0, "yaw": 0.0})
                quat = rotation.get("quaternion", {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})
                
                # Format output strings
                pos_str = f"{pos['x']:.2f},{pos['y']:.2f},{pos['z']:.2f}"
                euler_str = f"{euler['roll']:.1f},{euler['pitch']:.1f},{euler['yaw']:.1f}"
                quat_str = f"{quat['w']:.2f},{quat['x']:.2f},{quat['y']:.2f},{quat['z']:.2f}"
                
                # Print output in-place
                sys.stdout.write(
                    f"\r{pos_str:^18} | {euler_str:^24} | {quat_str:^24} | {heading:^8.1f}"
                )
                sys.stdout.flush()

            except (json.JSONDecodeError, ValueError) as parse_err:
                print(f"\n[WARNING] Received invalid packet from {addr}: {parse_err}")
                
    except KeyboardInterrupt:
        print("\n\nReceiver stopped by user.")
    finally:
        sock.close()
        print("Socket closed cleanly.")

if __name__ == "__main__":
    main()

