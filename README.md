# Raspberry Pi 4 IMU & Magnetometer Reader

A lightweight, robust, and dependency-minimal Python codebase for Raspberry Pi 4 to read real-time data from an IMU (MPU6050/MPU9250 at address `0x68`) and a Magnetometer (HMC5883L/QMC5883L at address `0x1E`).

## Hardware Connection Guide

Connect both sensors to the Raspberry Pi 4 I2C bus as follows:

| Sensor Pin | Pi 4 GPIO Pin | Description |
|---|---|---|
| **VCC** | Pin 1 (3.3V) | Power Supply |
| **GND** | Pin 9 (Ground) | Common Ground |
| **SDA** | Pin 3 (SDA / GPIO 2) | Serial Data Line |
| **SCL** | Pin 5 (SCL / GPIO 3) | Serial Clock Line |

*Note: Since I2C is a shared bus, both sensors can be wired in parallel to the same SDA and SCL pins on the Raspberry Pi.*

---

## Software Setup

### 1. Enable I2C on Raspberry Pi
Open the Raspberry Pi terminal and open the configuration tool:
```bash
sudo raspi-config
```
Navigate to **Interface Options** -> **I2C** and select **Yes** to enable the I2C interface. Reboot your Pi:
```bash
sudo reboot
```

### 2. Verify I2C Connections
Install `i2c-tools` to check if the Pi detects the sensors:
```bash
sudo apt-get install -y i2c-tools
```
Run `i2cdetect` to scan the bus:
```bash
i2cdetect -y 1
```
You should see output indicating devices are present at address `1e` and `68`:
```text
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10:  -- -- -- -- -- -- -- -- -- -- -- -- -- -- 1e -- 
20:  -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30:  -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40:  -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50:  -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60:  -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- -- 
70:  -- -- -- -- -- -- -- --                        
```

### 3. Install Dependencies
Install Python dependencies using the provided `requirements.txt`:
```bash
pip install -r requirements.txt
```

---

## Running the Code

### 1. Run the Receiver / Client (on your monitoring PC or the Pi)
To start listening for incoming sensor telemetry, run:
```bash
python client.py
```

### 2. Run the Sensor Publisher (on the Raspberry Pi)
Execute the main script to start streaming sensor readings:
```bash
python main.py
```

### Key Customization
In [main.py](file:///c:/Users/Legion/Desktop/IK/main.py), you can customize:
- `DECLINATION_ANGLE_DEG`: Set this to get accurate true-north headings based on your location. Find your declination angle at [magnetic-declination.com](http://www.magnetic-declination.com/).
- `UDP_IP`: Target IP address to send sensor data (e.g., target laptop's IP, or `"255.255.255.255"` for local network broadcast).
- `UDP_PORT`: Port number for UDP transmission (default is `5005`).

### UDP Payload format
The payload is sent as a JSON string with the following structure:
```json
{
  "timestamp": 1721389811.234,
  "accel": {"x": 0.01, "y": -0.02, "z": 0.98},
  "gyro": {"x": 0.1, "y": -0.2, "z": 0.05},
  "mag": {"x": -2.3, "y": 25.1, "z": -40.2},
  "heading": 85.3,
  "temp": 28.5
}
```


