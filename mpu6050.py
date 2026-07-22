import time
import smbus2

class MPU6050:
    # MPU6050 Default I2C Addresses (0x68 when AD0 is GND, 0x69 when AD0 is VCC)
    DEFAULT_I2C_ADDR = 0x68

    # MPU6050 Registers
    REG_PWR_MGMT_1 = 0x6B
    REG_SMPLRT_DIV = 0x19
    REG_CONFIG = 0x1A
    REG_GYRO_CONFIG = 0x1B
    REG_ACCEL_CONFIG = 0x1C
    
    # Data Registers
    REG_ACCEL_XOUT_H = 0x3B
    REG_TEMP_OUT_H = 0x41
    REG_GYRO_XOUT_H = 0x43

    # Scale Factors for Default Ranges
    # Accel: ±2g -> 16384 LSB/g
    # Gyro: ±250 °/s -> 131 LSB/(°/s)
    ACCEL_SCALE_2G = 16384.0
    GYRO_SCALE_250 = 131.0

    def __init__(self, bus_id=1, i2c_addr=0x68):
        """
        Initializes the MPU6050 connection.
        :param bus_id: I2C bus ID (normally 1 on Raspberry Pi 4).
        :param i2c_addr: Preferred I2C Address (default 0x68).
        """
        self.bus_id = bus_id
        self.I2C_ADDR = i2c_addr
        self.bus = None
        self._last_accel = {'x': 0.0, 'y': 0.0, 'z': 1.0}
        self._last_gyro = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def initialize(self):
        """
        Initializes the sensor with auto-address fallback (0x68 / 0x69) and retry handling.
        """
        self.bus = smbus2.SMBus(self.bus_id)
        
        # Auto-detect address between 0x68 and 0x69 if preferred fails
        addresses_to_try = [self.I2C_ADDR, 0x68, 0x69] if self.I2C_ADDR not in (0x68, 0x69) else [self.I2C_ADDR, 0x69 if self.I2C_ADDR == 0x68 else 0x68]
        
        sensor_found = False
        for addr in addresses_to_try:
            try:
                # Wake up MPU6050 (write 0 to PWR_MGMT_1)
                self.bus.write_byte_data(addr, self.REG_PWR_MGMT_1, 0x00)
                time.sleep(0.05)
                self.I2C_ADDR = addr
                sensor_found = True
                print(f"[MPU6050] Successfully initialized on bus {self.bus_id} at address {hex(addr)}")
                break
            except OSError:
                continue

        if not sensor_found:
            raise RuntimeError(f"MPU6050 not found on I2C bus {self.bus_id} at address 0x68 or 0x69. Check wiring & AD0 pin.")

        try:
            # Set sample rate divider to 7 (1kHz / (1 + 7) = 125Hz)
            self.bus.write_byte_data(self.I2C_ADDR, self.REG_SMPLRT_DIV, 0x07)
            # Set DLPF (Digital Low Pass Filter) configuration to ~94Hz bandpass
            self.bus.write_byte_data(self.I2C_ADDR, self.REG_CONFIG, 0x02)
            # Set Gyro range to ±250 deg/s (FS_SEL = 0)
            self.bus.write_byte_data(self.I2C_ADDR, self.REG_GYRO_CONFIG, 0x00)
            # Set Accel range to ±2g (AFS_SEL = 0)
            self.bus.write_byte_data(self.I2C_ADDR, self.REG_ACCEL_CONFIG, 0x00)
        except OSError as e:
            raise RuntimeError(f"Failed to configure MPU6050 registers: {e}")

    def _read_block_6bytes(self, register, retries=3):
        """
        Reads 6 consecutive bytes from I2C register with retries.
        """
        for attempt in range(retries):
            try:
                data = self.bus.read_i2c_block_data(self.I2C_ADDR, register, 6)
                
                # Unpack 3 signed 16-bit integers
                x = (data[0] << 8) | data[1]
                y = (data[2] << 8) | data[3]
                z = (data[4] << 8) | data[5]

                if x >= 0x8000: x -= 0x10000
                if y >= 0x8000: y -= 0x10000
                if z >= 0x8000: z -= 0x10000

                return x, y, z
            except OSError:
                if attempt == retries - 1:
                    return None
                time.sleep(0.002)
        return None

    def read_accel_data(self):
        """
        Reads accelerometer values in 'g' units with robust fallback.
        """
        vals = self._read_block_6bytes(self.REG_ACCEL_XOUT_H)
        if vals is not None:
            raw_x, raw_y, raw_z = vals
            self._last_accel = {
                'x': raw_x / self.ACCEL_SCALE_2G,
                'y': raw_y / self.ACCEL_SCALE_2G,
                'z': raw_z / self.ACCEL_SCALE_2G
            }
        return self._last_accel

    def read_gyro_data(self):
        """
        Reads gyroscope values in °/s with robust fallback.
        """
        vals = self._read_block_6bytes(self.REG_GYRO_XOUT_H)
        if vals is not None:
            raw_x, raw_y, raw_z = vals
            self._last_gyro = {
                'x': raw_x / self.GYRO_SCALE_250,
                'y': raw_y / self.GYRO_SCALE_250,
                'z': raw_z / self.GYRO_SCALE_250
            }
        return self._last_gyro

    def read_temp(self):
        """Reads temperature data in Celsius (°C)."""
        try:
            data = self.bus.read_i2c_block_data(self.I2C_ADDR, self.REG_TEMP_OUT_H, 2)
            raw_temp = (data[0] << 8) | data[1]
            if raw_temp >= 0x8000: raw_temp -= 0x10000
            return (raw_temp / 340.0) + 36.53
        except OSError:
            return 25.0

    def close(self):
        """Closes the I2C bus connection."""
        if self.bus:
            try:
                self.bus.close()
            except Exception:
                pass
