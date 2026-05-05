import math

import numpy as np

from constants_loader import load_structural_constants


class SatellitePhysics:
    def __init__(self, max_voltage=3.3, dipole_strength=2.88, structural_constants=None):
        constants = structural_constants or load_structural_constants()

        # 1. Mass Properties from W26 source constants (full inertia tensor in body frame)
        self.I = np.array(constants.inertia_tensor_body, dtype=float)
        self.I_inv = np.linalg.inv(self.I)
        self.center_of_mass_mm = np.array(constants.center_of_mass_mm, dtype=float)
        self.principal_axes_body = np.array(constants.principal_axes_body, dtype=float)
        self.principal_moments_kg_m2 = np.array(constants.principal_moments_kg_m2, dtype=float)

        # 2. Magnetorquer Characteristics (from mtq-power-char.png)
        self.R = 28.0  # Ohms
        self.L = np.array([0.025, 0.025, 0.012])  # XY: 25mH, Z: 12mH (averages)

        # Calibration Parameters
        self.max_voltage = max_voltage
        self.dipole_strength = dipole_strength

        # 3. Orbit/Magnetic model parameters from structural constants
        self.semimajor_axis_m = float(constants.semimajor_axis_m)
        self.orbit_period_s = float(constants.period_s)
        self.inclination_rad = math.radians(float(constants.inclination_deg))
        self.raan_rad = math.radians(float(constants.raan_deg))
        self.earth_rotation_period_s = 86400.0
        self.earth_dipole_tilt_rad = math.radians(11.7)
        self.B0_tesla = 5.0e-5

        # 4. State Vector [q0, q1, q2, q3, wx, wy, wz, ix, iy, iz]
        # q = [qw, qx, qy, qz], w = [wx, wy, wz], i = current in Amps
        self.state = np.zeros(10)
        self.state[0] = 1.0  # qw = 1.0

    def get_magnetic_field_inertial(self, t):
        # Tilted dipole model (inertial frame)
        n = 2.0 * np.pi / self.orbit_period_s
        theta_orbit = n * t
        r_orbit = np.array([np.cos(theta_orbit), np.sin(theta_orbit), 0.0])

        r_inc = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, np.cos(self.inclination_rad), -np.sin(self.inclination_rad)],
                [0.0, np.sin(self.inclination_rad), np.cos(self.inclination_rad)],
            ]
        )
        r_raan = np.array(
            [
                [np.cos(self.raan_rad), -np.sin(self.raan_rad), 0.0],
                [np.sin(self.raan_rad), np.cos(self.raan_rad), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        pos_eci = r_raan @ (r_inc @ r_orbit)

        omega_earth = 2.0 * np.pi / self.earth_rotation_period_s
        theta_earth = omega_earth * t
        epsilon = self.earth_dipole_tilt_rad
        m_hat = np.array(
            [
                np.sin(epsilon) * np.cos(theta_earth),
                np.sin(epsilon) * np.sin(theta_earth),
                np.cos(epsilon),
            ]
        )

        r_hat = pos_eci / max(np.linalg.norm(pos_eci), 1e-12)
        m_dot_r = float(np.dot(m_hat, r_hat))
        b_inertial = self.B0_tesla * (3.0 * m_dot_r * r_hat - m_hat)
        return b_inertial

    def get_b_field(self, q, b_inertial):
        # Rotate into body frame: B_body = R_body_eci.T * B_inertial
        r_body_eci = self._quat_to_dcm(q)
        return r_body_eci.T @ b_inertial

    def dynamics(self, state, t, v_command, b_inertial):
        # Extract state [q0-3, w0-2, i0-2]
        q = state[0:4]
        w = state[4:7]
        i = state[7:10]

        # 1. Attitude Kinematics
        omega = np.array(
            [
                [0.0, -w[0], -w[1], -w[2]],
                [w[0], 0.0, -w[2], w[1]],
                [w[1], w[2], 0.0, -w[0]],
                [w[2], -w[1], w[0], 0.0],
            ]
        )
        q_dot = 0.5 * omega @ q

        # 2. MTQ Current Dynamics (dI/dt = (V - IR) / L)
        i_dot = (v_command - i * self.R) / self.L

        # 3. Magnetic Torque
        b_body = self.get_b_field(q, b_inertial)
        m_actual = i * self.dipole_strength
        torque_mag = np.cross(m_actual, b_body)

        # 4. Rigid Body Dynamics
        i_w = self.I @ w
        gyroscopic = np.cross(w, i_w)
        w_dot = self.I_inv @ (torque_mag - gyroscopic)

        return np.concatenate((q_dot, w_dot, i_dot))

    def rk4_step(self, t, dt, v_command, b_inertial):
        # 1. Physical Saturation: Clamp voltage to +/- max_voltage
        v_command = np.clip(v_command, -self.max_voltage, self.max_voltage)

        # Sub-stepping: RL circuit needs ~0.5ms steps for stability.
        target_sub_dt = 0.0005
        n_sub = max(1, int(math.ceil(dt / target_sub_dt)))
        sub_dt = dt / n_sub

        current_state = self.state.copy()

        for _ in range(n_sub):
            k1 = self.dynamics(current_state, t, v_command, b_inertial)
            k2 = self.dynamics(current_state + 0.5 * sub_dt * k1, t + 0.5 * sub_dt, v_command, b_inertial)
            k3 = self.dynamics(current_state + 0.5 * sub_dt * k2, t + 0.5 * sub_dt, v_command, b_inertial)
            k4 = self.dynamics(current_state + sub_dt * k3, t + sub_dt, v_command, b_inertial)

            current_state = current_state + (sub_dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

            # Normalize Quaternion every sub-step to prevent drift
            q_norm = np.linalg.norm(current_state[0:4])
            if q_norm > 1e-6:
                current_state[0:4] /= q_norm
            else:
                current_state[0:4] = np.array([1.0, 0.0, 0.0, 0.0])

        # Final safety: check for exploded values (NaN/Inf)
        if not np.isfinite(current_state).all():
            print("WARNING: Physics diverged! Stabilizing current...")
            current_state[7:10] = np.zeros(3)
            current_state = np.nan_to_num(current_state, nan=0.0, posinf=1e6, neginf=-1e6)

        # Clip values for float32 / protocol safety
        current_state[4:7] = np.clip(current_state[4:7], -100.0, 100.0)
        current_state[7:10] = np.clip(current_state[7:10], -1.0, 1.0)

        self.state = current_state

    def _quat_to_dcm(self, q):
        w, x, y, z = q
        return np.array(
            [
                [1 - 2 * y**2 - 2 * z**2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
                [2 * x * y + 2 * z * w, 1 - 2 * x**2 - 2 * z**2, 2 * y * z - 2 * x * w],
                [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x**2 - 2 * y**2],
            ]
        )
