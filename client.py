import socket
import json
import sys

# Listen on all interfaces on the configured port
LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5005

def main():
    print("=" * 60)
    print("         UDP Sensor Data Receiver / Client")
    print("=" * 60)
    print(f"Binding to {LISTEN_IP}:{LISTEN_PORT}...")

    # Set up UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        sock.bind((LISTEN_IP, LISTEN_PORT))
        print("[SUCCESS] Listening for incoming UDP telemetry packets.\n")
    except Exception as e:
        print(f"[ERROR] Failed to bind to socket: {e}")
        sys.exit(1)

    print("-" * 75)
    print(f"{'Accel (g)':^18} | {'Gyro (°/s)':^18} | {'Mag (µT)':^18} | {'Heading':^8} | {'Temp':^6}")
    print(f"{'X / Y / Z':^18} | {'X / Y / Z':^18} | {'X / Y / Z':^18} | {'(Deg)':^8} | {'(°C)':^6}")
    print("-" * 75)

    try:
        while True:
            data, addr = sock.recvfrom(4096) # Buffer size 4096 bytes
            try:
                # Decode JSON payload
                payload = json.loads(data.decode('utf-8'))
                
                accel = payload.get("accel", {"x": 0.0, "y": 0.0, "z": 0.0})
                gyro = payload.get("gyro", {"x": 0.0, "y": 0.0, "z": 0.0})
                mag = payload.get("mag", {"x": 0.0, "y": 0.0, "z": 0.0})
                heading = payload.get("heading", 0.0)
                temp = payload.get("temp", 0.0)
                
                # Format output strings
                accel_str = f"{accel['x']:.2f},{accel['y']:.2f},{accel['z']:.2f}"
                gyro_str = f"{gyro['x']:.1f},{gyro['y']:.1f},{gyro['z']:.1f}"
                mag_str = f"{mag['x']:.1f},{mag['y']:.1f},{mag['z']:.1f}"
                
                # Print output in-place
                sys.stdout.write(
                    f"\r{accel_str:^18} | {gyro_str:^18} | {mag_str:^18} | {heading:^8.1f} | {temp:^6.1f}"
                )
                sys.stdout.flush()

            except (json.JSONDecodeError, ValueError) as parse_err:
                # Print parsing errors on a new line to not mess up formatting
                print(f"\n[WARNING] Received invalid packet from {addr}: {parse_err}")
                
    except KeyboardInterrupt:
        print("\n\nReceiver stopped by user.")
    finally:
        sock.close()
        print("Socket closed cleanly.")

if __name__ == "__main__":
    main()
