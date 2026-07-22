import math
import numpy as np

class MahonyAHRS:
    """
    SlimeVR-style Mahony AHRS Complementary Filter with Integral Gyro Bias Correction
    and Magnetic Anomaly Rejection.
    """
    def __init__(self, kp=0.5, ki=0.005, mag_baseline_ut=45.0):
        self.kp = kp  # Proportional feedback gain
        self.ki = ki  # Integral feedback gain
        self.mag_baseline_ut = mag_baseline_ut

        # Initial resting quaternion: Roll = 90 deg, Pitch = 0 deg, Yaw = 0 deg
        # q = [cos(pi/4), sin(pi/4), 0, 0] = [0.70710678, 0.70710678, 0, 0]
        self.q = np.array([0.7071067811865476, 0.7071067811865476, 0.0, 0.0], dtype=float)  # [qw, qx, qy, qz]
        self.e_int = np.zeros(3, dtype=float)                 # Gyro bias integral accumulator
        self.is_mag_anomaly = False

    def update(self, gyro, accel, mag, dt):
        """
        Updates quaternion given gyro (rad/s), accel (g), mag (uT), and dt (seconds).
        """
        if dt <= 0 or dt > 0.5:
            dt = 0.01

        gx, gy, gz = gyro[0], gyro[1], gyro[2]
        ax, ay, az = accel[0], accel[1], accel[2]
        mx, my, mz = mag[0], mag[1], mag[2]

        q1, q2, q3, q4 = self.q[0], self.q[1], self.q[2], self.q[3]

        # 1. Normalize Accelerometer Measurement
        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if accel_norm == 0:
            return self.q
        ax /= accel_norm
        ay /= accel_norm
        az /= accel_norm

        # 2. Check for Magnetic Anomaly (SlimeVR Rejection Gating)
        mag_norm = math.sqrt(mx * mx + my * my + mz * mz)
        use_mag = False
        if mag_norm > 0:
            mx /= mag_norm
            my /= mag_norm
            mz /= mag_norm
            # If magnetic field intensity deviates > 30% from baseline, flag anomaly and ignore mag
            if 0.7 * self.mag_baseline_ut <= (mag_norm) <= 1.3 * self.mag_baseline_ut:
                use_mag = True
                self.is_mag_anomaly = False
            else:
                self.is_mag_anomaly = True

        # 3. Estimated Direction of Gravity in Body Frame
        vx = 2.0 * (q2 * q4 - q1 * q3)
        vy = 2.0 * (q1 * q2 + q3 * q4)
        vz = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4

        # Error is Cross Product between Measured Acceleration and Estimated Gravity
        ex_a = ay * vz - az * vy
        ey_a = az * vx - ax * vz
        ez_a = ax * vy - ay * vx

        ex_m, ey_m, ez_m = 0.0, 0.0, 0.0
        if use_mag:
            # Estimated Direction of Magnetometer Reference Field
            hx = 2.0 * mx * (0.5 - q3*q3 - q4*q4) + 2.0 * my * (q2*q3 - q1*q4) + 2.0 * mz * (q2*q4 + q1*q3)
            hy = 2.0 * mx * (q2*q3 + q1*q4) + 2.0 * my * (0.5 - q2*q2 - q4*q4) + 2.0 * mz * (q3*q4 - q1*q2)
            bx = math.sqrt(hx * hx + hy * hy)
            bz = 2.0 * mx * (q2*q4 - q1*q3) + 2.0 * my * (q3*q4 + q1*q2) + 2.0 * mz * (0.5 - q2*q2 - q3*q3)

            wx = 2.0 * bx * (0.5 - q3*q3 - q4*q4) + 2.0 * bz * (q2*q4 - q1*q3)
            wy = 2.0 * bx * (q2*q3 - q1*q4) + 2.0 * bz * (q1*q2 + q3*q4)
            wz = 2.0 * bx * (q1*q3 + q2*q4) + 2.0 * bz * (0.5 - q2*q2 - q3*q3)

            ex_m = my * wz - mz * wy
            ey_m = mz * wx - mx * wz
            ez_m = mx * wy - my * wx

        # Total Error Signal
        ex = ex_a + ex_m
        ey = ey_a + ey_m
        ez = ez_a + ez_m

        # Integral Feedback (Gyro Bias Accumulation)
        if self.ki > 0:
            self.e_int[0] += ex * self.ki * dt
            self.e_int[1] += ey * self.ki * dt
            self.e_int[2] += ez * self.ki * dt

        # Apply Proportional and Integral Feedback to Gyroscope Signals
        gx += self.kp * ex + self.e_int[0]
        gy += self.kp * ey + self.e_int[1]
        gz += self.kp * ez + self.e_int[2]

        # Integrate Quaternion Rate
        q_dot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz)
        q_dot2 = 0.5 * ( q1 * gx + q3 * gz - q4 * gy)
        q_dot3 = 0.5 * ( q1 * gy - q2 * gz + q4 * gx)
        q_dot4 = 0.5 * ( q1 * gz + q2 * gy - q3 * gx)

        self.q[0] += q_dot1 * dt
        self.q[1] += q_dot2 * dt
        self.q[2] += q_dot3 * dt
        self.q[3] += q_dot4 * dt

        # Normalize Quaternion
        q_norm = np.linalg.norm(self.q)
        if q_norm > 0:
            self.q /= q_norm
        return self.q


class PoseEstimator:
    """
    SlimeVR 6DoF Pose Estimator with Mahony AHRS Sensor Fusion,
    Kinematic Arm Forward Kinematics, and IMU Drift Compensation.
    """
    GRAVITY = 9.80665  # m/s^2

    def __init__(self, kp=0.6, ki=0.005, vel_decay=0.92, zero_velocity_thresh=0.20):
        self.ahrs = MahonyAHRS(kp=kp, ki=ki)
        self.vel_decay = vel_decay
        self.zero_velocity_thresh = zero_velocity_thresh

        self.position = np.zeros(3, dtype=float)      # [x, y, z] in meters
        self.velocity = np.zeros(3, dtype=float)      # [vx, vy, vz] in m/s
        self.linear_accel = np.zeros(3, dtype=float)  # [ax, ay, az] in m/s^2 (world frame)

        # Kinematic Arm Chain Offsets (SlimeVR Skeleton Model)
        self.shoulder_pos = np.array([0.2, -0.2, 0.0], dtype=float)  # Right shoulder offset from body
        self.upper_arm_length = 0.32  # meters
        self.forearm_length = 0.28    # meters

        # Drift Offset Biases
        self.gyro_bias = np.zeros(3, dtype=float)
        self.accel_bias = np.zeros(3, dtype=float)

        self.is_calibrating = False
        self.calib_accel_samples = []
        self.calib_gyro_samples = []
        self.calib_target_samples = 100

    def start_calibration(self, num_samples=100):
        self.is_calibrating = True
        self.calib_target_samples = num_samples
        self.calib_accel_samples = []
        self.calib_gyro_samples = []

    def reset_drift(self):
        self.position = np.zeros(3, dtype=float)
        self.velocity = np.zeros(3, dtype=float)
        self.linear_accel = np.zeros(3, dtype=float)

    def quaternion_to_rotation_matrix(self, q):
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]
        return np.array([
            [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
            [    2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qw*qx)],
            [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ], dtype=float)

    def quaternion_to_euler(self, q):
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]

        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        sinp = max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))
        pitch = math.asin(sinp)
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def update(self, accel_g, gyro_deg, mag_ut, dt):
        if dt <= 0 or dt > 0.5:
            dt = 0.01

        raw_ax = accel_g['x'] if isinstance(accel_g, dict) else accel_g[0]
        raw_ay = accel_g['y'] if isinstance(accel_g, dict) else accel_g[1]
        raw_az = accel_g['z'] if isinstance(accel_g, dict) else accel_g[2]

        raw_gx = gyro_deg['x'] if isinstance(gyro_deg, dict) else gyro_deg[0]
        raw_gy = gyro_deg['y'] if isinstance(gyro_deg, dict) else gyro_deg[1]
        raw_gz = gyro_deg['z'] if isinstance(gyro_deg, dict) else gyro_deg[2]

        if self.is_calibrating:
            self.calib_accel_samples.append([raw_ax, raw_ay, raw_az])
            self.calib_gyro_samples.append([raw_gx, raw_gy, raw_gz])

            if len(self.calib_accel_samples) >= self.calib_target_samples:
                accel_arr = np.array(self.calib_accel_samples)
                gyro_arr = np.array(self.calib_gyro_samples)

                self.gyro_bias = np.mean(gyro_arr, axis=0)
                mean_accel = np.mean(accel_arr, axis=0)
                self.accel_bias = mean_accel - np.array([0.0, 0.0, 1.0])

                self.is_calibrating = False
                self.reset_drift()

        # Subtract calibrated biases
        ax = raw_ax - self.accel_bias[0]
        ay = raw_ay - self.accel_bias[1]
        az = raw_az - self.accel_bias[2]

        gx = raw_gx - self.gyro_bias[0]
        gy = raw_gy - self.gyro_bias[1]
        gz = raw_gz - self.gyro_bias[2]

        gx_rad = math.radians(gx)
        gy_rad = math.radians(gy)
        gz_rad = math.radians(gz)

        mx = mag_ut['x'] if isinstance(mag_ut, dict) else mag_ut[0]
        my = mag_ut['y'] if isinstance(mag_ut, dict) else mag_ut[1]
        mz = mag_ut['z'] if isinstance(mag_ut, dict) else mag_ut[2]

        # 1. Mahony AHRS Sensor Fusion Update
        q = self.ahrs.update(
            gyro=[gx_rad, gy_rad, gz_rad],
            accel=[ax, ay, az],
            mag=[mx, my, mz],
            dt=dt
        )

        R = self.quaternion_to_rotation_matrix(q)
        roll, pitch, yaw = self.quaternion_to_euler(q)

        # 3. Angle-only Mode: zero out position/translation movement integration
        self.position = np.zeros(3, dtype=float)
        self.velocity = np.zeros(3, dtype=float)
        self.linear_accel = np.zeros(3, dtype=float)

        return {
            "is_calibrating": self.is_calibrating,
            "magnetic_anomaly": self.ahrs.is_mag_anomaly,
            "rotation": {
                "quaternion": {"w": float(q[0]), "x": float(q[1]), "y": float(q[2]), "z": float(q[3])},
                "euler": {"roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)}
            },
            "translation": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                "linear_accel": {"x": 0.0, "y": 0.0, "z": 0.0}
            }
        }
