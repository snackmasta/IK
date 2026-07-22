import time
import smbus2

class HMC5883L:
    # HMC5883L I2C Address
    I2C_ADDR = 0x1E

    # Registers
    REG_CONFIG_A = 0x00
    REG_CONFIG_B = 0x01
    REG_MODE = 0x02
    
    # Data registers - Note the non-standard order (X, Z, Y)
    REG_DATA_X_H = 0x03

    # Gain scaling configurations (LSB per Gauss)
    GAINS = {
        0.88: (0x00, 1370.0),
        1.3:  (0x20, 1090.0), # Default
        1.9:  (0x40, 820.0),
        2.5:  (0x60, 660.0),
        4.0:  (0x80, 440.0),
        4.7:  (0xA0, 390.0),
        5.6:  (0xC0, 330.0),
        8.1:  (0xE0, 230.0)
    }

    def __init__(self, bus_id=1, gauss_range=1.3):
        """
        Initializes the HMC5883L connection.
        """
        self.bus_id = bus_id
        self.bus = None
        if gauss_range not in self.GAINS:
            raise ValueError(f"Invalid Gauss range. Choose from: {list(self.GAINS.keys())}")
        self.gauss_range = gauss_range
        self.gain_val, self.scale_factor = self.GAINS[gauss_range]
        self._last_mag = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def initialize(self):
        """
        Initializes the sensor config registers for continuous measurement mode with retries.
        """
        self.bus = smbus2.SMBus(self.bus_id)
        
        for attempt in range(3):
            try:
                # Config A: 8 samples averaged, 15 Hz measurement rate
                self.bus.write_byte_data(self.I2C_ADDR, self.REG_CONFIG_A, 0x70)
                # Config B: Set gain/range scale
                self.bus.write_byte_data(self.I2C_ADDR, self.REG_CONFIG_B, self.gain_val)
                # Mode Register: Continuous measurement mode
                self.bus.write_byte_data(self.I2C_ADDR, self.REG_MODE, 0x00)
                time.sleep(0.05)
                print(f"[HMC5883L] Successfully initialized on bus {self.bus_id} at address 0x1E")
                return
            except OSError as e:
                if attempt == 2:
                    raise RuntimeError(f"Failed to initialize HMC5883L on bus {self.bus_id}: {e}")
                time.sleep(0.05)

    def read_magnetometer_data(self, retries=3):
        """
        Reads magnetometer axis values in micro-Tesla (µT) using single 6-byte I2C block read.
        """
        for attempt in range(retries):
            try:
                # HMC5883L outputs X_H, X_L, Z_H, Z_L, Y_H, Y_L in sequence
                data = self.bus.read_i2c_block_data(self.I2C_ADDR, self.REG_DATA_X_H, 6)
                
                raw_x = (data[0] << 8) | data[1]
                raw_z = (data[2] << 8) | data[3]
                raw_y = (data[4] << 8) | data[5]

                if raw_x >= 0x8000: raw_x -= 0x10000
                if raw_y >= 0x8000: raw_y -= 0x10000
                if raw_z >= 0x8000: raw_z -= 0x10000

                gauss_x = raw_x / self.scale_factor
                gauss_y = raw_y / self.scale_factor
                gauss_z = raw_z / self.scale_factor

                self._last_mag = {
                    'x': gauss_x * 100.0,
                    'y': gauss_y * 100.0,
                    'z': gauss_z * 100.0
                }
                return self._last_mag
            except OSError:
                if attempt == retries - 1:
                    return self._last_mag
                time.sleep(0.002)
        return self._last_mag

    def close(self):
        """Closes the I2C bus connection."""
        if self.bus:
            try:
                self.bus.close()
            except Exception:
                pass
