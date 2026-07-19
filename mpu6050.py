import time
import smbus2

class MPU6050:
    # MPU6050 I2C Address
    I2C_ADDR = 0x68

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

    def __init__(self, bus_id=1):
        """
        Initializes the MPU6050 connection.
        :param bus_id: I2C bus ID (normally 1 on Raspberry Pi 4).
        """
        self.bus_id = bus_id
        self.bus = None

    def initialize(self):
        """
        Initializes the sensor by waking it up and setting default configurations.
        """
        try:
            self.bus = smbus2.SMBus(self.bus_id)
            # Wake up MPU6050 (default sleep mode is active on power-on)
            # Write 0 to PWR_MGMT_1 to wake up the sensor
            self.bus.write_byte_data(self.I2C_ADDR, self.REG_PWR_MGMT_1, 0x00)
            time.sleep(0.1) # Wait for sensor to stabilize

            # Set sample rate divider to 7 (1kHz / (1 + 7) = 125Hz)
            self.bus.write_byte_data(self.I2C_ADDR, self.REG_SMPLRT_DIV, 0x07)

            # Set DLPF (Digital Low Pass Filter) configuration to ~94Hz bandpass
            self.bus.write_byte_data(self.I2C_ADDR, self.REG_CONFIG, 0x02)

            # Set Gyro range to ±250 deg/s (FS_SEL = 0)
            self.bus.write_byte_data(self.I2C_ADDR, self.REG_GYRO_CONFIG, 0x00)

            # Set Accel range to ±2g (AFS_SEL = 0)
            self.bus.write_byte_data(self.I2C_ADDR, self.REG_ACCEL_CONFIG, 0x00)
        except Exception as e:
            raise RuntimeError(f"Failed to initialize MPU6050 on bus {self.bus_id}: {e}")

    def _read_raw_word(self, register):
        """
        Reads a 16-bit signed word from the specified high byte register.
        """
        high = self.bus.read_byte_data(self.I2C_ADDR, register)
        low = self.bus.read_byte_data(self.I2C_ADDR, register + 1)
        val = (high << 8) + low
        
        # Convert to 16-bit signed integer
        if val >= 0x8000:
            val -= 0x10000
        return val

    def read_accel_data(self):
        """
        Reads accelerometer values.
        :return: Dict containing X, Y, Z acceleration in 'g' units.
        """
        raw_x = self._read_raw_word(self.REG_ACCEL_XOUT_H)
        raw_y = self._read_raw_word(self.REG_ACCEL_XOUT_H + 2)
        raw_z = self._read_raw_word(self.REG_ACCEL_XOUT_H + 4)

        return {
            'x': raw_x / self.ACCEL_SCALE_2G,
            'y': raw_y / self.ACCEL_SCALE_2G,
            'z': raw_z / self.ACCEL_SCALE_2G
        }

    def read_gyro_data(self):
        """
        Reads gyroscope values.
        :return: Dict containing X, Y, Z angular velocity in degrees per second (°/s).
        """
        raw_x = self._read_raw_word(self.REG_GYRO_XOUT_H)
        raw_y = self._read_raw_word(self.REG_GYRO_XOUT_H + 2)
        raw_z = self._read_raw_word(self.REG_GYRO_XOUT_H + 4)

        return {
            'x': raw_x / self.GYRO_SCALE_250,
            'y': raw_y / self.GYRO_SCALE_250,
            'z': raw_z / self.GYRO_SCALE_250
        }

    def read_temp(self):
        """
        Reads temperature data.
        :return: Temperature in degrees Celsius (°C).
        """
        raw_temp = self._read_raw_word(self.REG_TEMP_OUT_H)
        # Temperature calculation from datasheet: Temp = (RAW / 340) + 36.53
        return (raw_temp / 340.0) + 36.53

    def close(self):
        """Closes the I2C bus connection."""
        if self.bus:
            self.bus.close()
