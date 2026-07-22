import math
import numpy as np

class MadgwickAHRS:
    """
    Madgwick's 9DoF Gradient Descent Orientation Filter (AHRS).
    Fuses Accelerometer, Gyroscope, and Magnetometer into a unit quaternion Q = [qw, qx, qy, qz].
    """
    def __init__(self, beta=0.1):
        self.beta = beta  # Filter gain
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # [qw, qx, qy, qz]

    def update(self, gyro, accel, mag, dt):
        """
        Update quaternion state given gyro (rad/s), accel (g), mag (uT), and dt (seconds).
        """
        gx, gy, gz = gyro[0], gyro[1], gyro[2]
        ax, ay, az = accel[0], accel[1], accel[2]
        mx, my, mz = mag[0], mag[1], mag[2]

        q1, q2, q3, q4 = self.q[0], self.q[1], self.q[2], self.q[3]

        # Normalize accelerometer measurement
        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if accel_norm == 0:
            return self.q
        ax /= accel_norm
        ay /= accel_norm
        az /= accel_norm

        # Normalize magnetometer measurement
        mag_norm = math.sqrt(mx * mx + my * my + mz * mz)
        if mag_norm == 0:
            return self.q
        mx /= mag_norm
        my /= mag_norm
        mz /= mag_norm

        # Reference direction of Earth's magnetic field
        _2q1mx = 2.0 * q1 * mx
        _2q1my = 2.0 * q1 * my
        _2q1mz = 2.0 * q1 * mz
        _2q2mx = 2.0 * q2 * mx

        hx = mx * (q1*q1 + q2*q2 - q3*q3 - q4*q4) + 2.0 * my * (q2*q3 - q1*q4) + 2.0 * mz * (q2*q4 + q1*q3)
        hy = 2.0 * mx * (q2*q3 + q1*q4) + my * (q1*q1 - q2*q2 + q3*q3 - q4*q4) + 2.0 * mz * (q3*q4 - q1*q2)
        bx = math.sqrt(hx * hx + hy * hy)
        bz = 2.0 * mx * (q2*q4 - q1*q3) + 2.0 * my * (q3*q4 + q1*q2) + mz * (q1*q1 - q2*q2 - q3*q3 + q4*q4)

        # Auxiliary variables
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _2bx = 2.0 * bx
        _2bz = 2.0 * bz
        _4bx = 4.0 * bx
        _4bz = 4.0 * bz
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        # Gradient descent objective function and Jacobian
        f = np.array([
            2.0 * (q2q4 - q1q3) - ax,
            2.0 * (q1q2 + q3q4) - ay,
            2.0 * (0.5 - q2q2 - q3q3) - az,
            _2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx,
            _2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my,
            _2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz
        ], dtype=float)

        J = np.array([
            [-2.0 * q3,                 2.0 * q4,                -2.0 * q1,                 2.0 * q2],
            [ 2.0 * q2,                 2.0 * q1,                 2.0 * q4,                 2.0 * q3],
            [ 0.0,                     -4.0 * q2,                -4.0 * q3,                 0.0],
            [-_2bz * q3,                _2bz * q4,               -_4bx * q3 - _2bz * q1,    -_4bx * q4 + _2bz * q2],
            [-_2bx * q4 + _2bz * q2,    _2bx * q3 + _2bz * q1,    _2bx * q2 + _2bz * q4,    -_2bx * q1 + _2bz * q3],
            [ _2bx * q3,                _2bx * q4 - _4bz * q2,    _2bx * q1 - _4bz * q3,     _2bx * q2]
        ], dtype=float)


        step = J.T @ f
        step_norm = np.linalg.norm(step)
        if step_norm > 0:
            step /= step_norm

        # Quaternion rate of change from gyroscope
        q_dot_gyro = 0.5 * np.array([
            -q2 * gx - q3 * gy - q4 * gz,
             q1 * gx + q3 * gz - q4 * gy,
             q1 * gy - q2 * gz + q4 * gx,
             q1 * gz + q2 * gy - q3 * gx
        ], dtype=float)

        # Apply feedback step
        q_dot = q_dot_gyro - self.beta * step

        # Integrate rate of change to compute quaternion
        self.q += q_dot * dt
        self.q /= np.linalg.norm(self.q)  # Normalize quaternion
        return self.q


class PoseEstimator:
    """
    6DoF Pose Estimator combining 3DoF Rotation (Quaternion + Euler)
    and 3DoF Translation (Position + Velocity from double integration).
    """
    GRAVITY = 9.80665  # m/s^2

    def __init__(self, beta=0.1, vel_decay=0.95, zero_velocity_thresh=0.15):
        self.ahrs = MadgwickAHRS(beta=beta)
        self.vel_decay = vel_decay
        self.zero_velocity_thresh = zero_velocity_thresh

        self.position = np.zeros(3, dtype=float)      # [x, y, z] in meters
        self.velocity = np.zeros(3, dtype=float)      # [vx, vy, vz] in m/s
        self.linear_accel = np.zeros(3, dtype=float)  # [ax, ay, az] in m/s^2 (world frame)

    def quaternion_to_rotation_matrix(self, q):
        """
        Converts quaternion [qw, qx, qy, qz] to a 3x3 Rotation Matrix.
        """
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]
        return np.array([
            [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
            [    2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qw*qx)],
            [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ], dtype=float)

    def quaternion_to_euler(self, q):
        """
        Converts quaternion [qw, qx, qy, qz] to Euler angles (Roll, Pitch, Yaw) in degrees.
        """
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def update(self, accel_g, gyro_deg, mag_ut, dt):
        """
        Updates 6DoF state.
        :param accel_g: Dict or list/tuple [ax, ay, az] in g units.
        :param gyro_deg: Dict or list/tuple [gx, gy, gz] in °/s units.
        :param mag_ut: Dict or list/tuple [mx, my, mz] in µT units.
        :param dt: Delta time in seconds since last update.
        """
        if dt <= 0 or dt > 0.5:
            dt = 0.01  # Safeguard against timing anomalies

        ax = accel_g['x'] if isinstance(accel_g, dict) else accel_g[0]
        ay = accel_g['y'] if isinstance(accel_g, dict) else accel_g[1]
        az = accel_g['z'] if isinstance(accel_g, dict) else accel_g[2]

        gx_rad = math.radians(gyro_deg['x'] if isinstance(gyro_deg, dict) else gyro_deg[0])
        gy_rad = math.radians(gyro_deg['y'] if isinstance(gyro_deg, dict) else gyro_deg[1])
        gz_rad = math.radians(gyro_deg['z'] if isinstance(gyro_deg, dict) else gyro_deg[2])

        mx = mag_ut['x'] if isinstance(mag_ut, dict) else mag_ut[0]
        my = mag_ut['y'] if isinstance(mag_ut, dict) else mag_ut[1]
        mz = mag_ut['z'] if isinstance(mag_ut, dict) else mag_ut[2]

        # 1. Update 3DoF Rotation Quaternion
        q = self.ahrs.update(
            gyro=[gx_rad, gy_rad, gz_rad],
            accel=[ax, ay, az],
            mag=[mx, my, mz],
            dt=dt
        )

        # 2. Compute Rotation Matrix & Euler Angles
        R = self.quaternion_to_rotation_matrix(q)
        roll, pitch, yaw = self.quaternion_to_euler(q)

        # 3. Convert Body Acceleration from g to m/s^2 and Rotate into World Frame
        accel_body_mps2 = np.array([ax, ay, az], dtype=float) * self.GRAVITY
        accel_world = R @ accel_body_mps2

        # 4. Remove Earth Gravity Vector [0, 0, +1g] from World Frame Acceleration
        self.linear_accel = accel_world - np.array([0.0, 0.0, self.GRAVITY], dtype=float)

        # 5. Zero-Velocity Update (ZUPT) check for drift reduction
        lin_accel_norm = np.linalg.norm(self.linear_accel)
        if lin_accel_norm < self.zero_velocity_thresh:
            self.linear_accel = np.zeros(3)
            self.velocity *= 0.5  # Rapid velocity decay when stationary
        else:
            self.velocity += self.linear_accel * dt
            self.velocity *= self.vel_decay  # High-pass dampening

        # 6. Integrate Velocity to Position (Translation)
        self.position += self.velocity * dt

        return {
            "rotation": {
                "quaternion": {"w": float(q[0]), "x": float(q[1]), "y": float(q[2]), "z": float(q[3])},
                "euler": {"roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)}
            },
            "translation": {
                "position": {"x": float(self.position[0]), "y": float(self.position[1]), "z": float(self.position[2])},
                "velocity": {"x": float(self.velocity[0]), "y": float(self.velocity[1]), "z": float(self.velocity[2])},
                "linear_accel": {"x": float(self.linear_accel[0]), "y": float(self.linear_accel[1]), "z": float(self.linear_accel[2])}
            }
        }
