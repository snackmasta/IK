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
    REG_DATA_Z_H = 0x05
    REG_DATA_Y_H = 0x07

    # Gain scaling configurations (LSB per Gauss)
    # Default is ±1.3 Gauss (gain setting 1) -> 1090 LSB/Gauss
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
        :param bus_id: I2C bus ID (normally 1 on Raspberry Pi 4).
        :param gauss_range: Sensor measurement range in Gauss (default ±1.3).
        """
        self.bus_id = bus_id
        self.bus = None
        if gauss_range not in self.GAINS:
            raise ValueError(f"Invalid Gauss range. Choose from: {list(self.GAINS.keys())}")
        self.gauss_range = gauss_range
        self.gain_val, self.scale_factor = self.GAINS[gauss_range]

    def initialize(self):
        """
        Initializes the sensor config registers for continuous measurement mode.
        """
        try:
            self.bus = smbus2.SMBus(self.bus_id)
            
            # Config A: 8 samples averaged, 15 Hz measurement rate, normal measurement configuration
            # 0b01110000 = 0x70
            self.bus.write_byte_data(self.I2C_ADDR, self.REG_CONFIG_A, 0x70)

            # Config B: Set gain/range scale
            self.bus.write_byte_data(self.I2C_ADDR, self.REG_CONFIG_B, self.gain_val)

            # Mode Register: Continuous measurement mode
            # 0b00000000 = 0x00
            self.bus.write_byte_data(self.I2C_ADDR, self.REG_MODE, 0x00)
            time.sleep(0.1) # Wait for setup stabilization
        except Exception as e:
            raise RuntimeError(f"Failed to initialize HMC5883L on bus {self.bus_id}: {e}")

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

    def read_magnetometer_data(self):
        """
        Reads the magnetometer axis values.
        :return: Dict containing X, Y, Z magnetic fields in micro-Tesla (µT).
                 (1 Gauss = 100 µT)
        """
        # HMC5883L registers sequence is X, Z, Y!
        raw_x = self._read_raw_word(self.REG_DATA_X_H)
        raw_z = self._read_raw_word(self.REG_DATA_Z_H)
        raw_y = self._read_raw_word(self.REG_DATA_Y_H)

        # Convert raw LSB to Gauss, then multiply by 100 to get micro-Tesla (µT)
        gauss_x = raw_x / self.scale_factor
        gauss_y = raw_y / self.scale_factor
        gauss_z = raw_z / self.scale_factor

        return {
            'x': gauss_x * 100.0,
            'y': gauss_y * 100.0,
            'z': gauss_z * 100.0
        }

    def close(self):
        """Closes the I2C bus connection."""
        if self.bus:
            self.bus.close()
