import numpy as np
import math

class SatellitePhysics:
    def __init__(self, max_voltage=3.3, dipole_strength=2.88):
        # 1. Mass Properties (2U CubeSat approximation)
        # 10cm x 10cm x 20cm, 2kg
        # I = m/12 * (h^2 + d^2)
        # Ixx = 1/12 * 2 * (0.1^2 + 0.2^2) = 1/6 * 0.05 = 0.00833
        # Iyy = 1/12 * 2 * (0.1^2 + 0.2^2) = 0.00833
        # Izz = 1/12 * 2 * (0.1^2 + 0.1^2) = 1/6 * 0.02 = 0.00333
        self.I = np.diag([0.00833, 0.00833, 0.00333])
        self.I_inv = np.linalg.inv(self.I)
        
        # 2. Magnetorquer Characteristics (from mtq-power-char.png)
        self.R = 28.0  # Ohms
        self.L = np.array([0.025, 0.025, 0.012]) # XY: 25mH, Z: 12mH (averages)
        
        # Calibration Parameters
        self.max_voltage = max_voltage
        self.dipole_strength = dipole_strength

        # 3. State Vector [q0, q1, q2, q3, wx, wy, wz, ix, iy, iz]
        # q = [qw, qx, qy, qz], w = [wx, wy, wz], i = current in Amps
        self.state = np.zeros(10)
        self.state[0] = 1.0 # qw = 1.0

    def get_magnetic_field_inertial(self, t):
        # TILTED DIPOLE MODEL (Inertial Frame)
        n = 2 * np.pi / 5400.0
        theta_orbit = n * t
        r_orbit = np.array([np.cos(theta_orbit), np.sin(theta_orbit), 0])
        inc = np.radians(51.6)
        R_inc = np.array([
            [1, 0, 0],
            [0, np.cos(inc), -np.sin(inc)],
            [0, np.sin(inc), np.cos(inc)]
        ])
        pos_eci = R_inc @ r_orbit
        omega_earth = 2 * np.pi / 86400.0
        theta_earth = omega_earth * t
        epsilon = np.radians(11.7)
        m_hat = np.array([
            np.sin(epsilon) * np.cos(theta_earth),
            np.sin(epsilon) * np.sin(theta_earth),
            np.cos(epsilon)
        ])
        B0 = 5.0e-5
        r_hat = pos_eci
        m_dot_r = np.dot(m_hat, r_hat)
        B_inertial = B0 * (3 * m_dot_r * r_hat - m_hat)
        return B_inertial

    def get_b_field(self, q, B_inertial):
        # Rotate into Body Frame: B_body = R_body_eci.T * B_inertial
        R_body_eci = self._quat_to_dcm(q)
        return R_body_eci.T @ B_inertial

    def dynamics(self, state, t, v_command, B_inertial):
        # Extract state [q0-3, w0-2, i0-2]
        q = state[0:4]
        w = state[4:7]
        i = state[7:10]
        
        # 1. Attitude Kinematics
        Omega = np.array([
            [0, -w[0], -w[1], -w[2]],
            [w[0], 0, -w[2], w[1]],
            [w[1], w[2], 0, -w[0]],
            [w[2], -w[1], w[0], 0]
        ])
        q_dot = 0.5 * Omega @ q
        
        # 2. MTQ Current Dynamics (dI/dt = (V - IR) / L)
        i_dot = (v_command - i * self.R) / self.L
        
        # 3. Magnetic Torque
        B_body = self.get_b_field(q, B_inertial)
        m_actual = i * self.dipole_strength  # Datasheet: 0.34 Am² @ 3.3V, R=28Ω → 2.88 Am²/A (default)
        torque_mag = np.cross(m_actual, B_body)
        
        # 4. Rigid Body Dynamics
        Iw = self.I @ w
        gyroscopic = np.cross(w, Iw)
        w_dot = self.I_inv @ (torque_mag - gyroscopic)
        
        return np.concatenate((q_dot, w_dot, i_dot))

    def rk4_step(self, t, dt, v_command, B_inertial):
        # 1. Physical Saturation: Clamp voltage to +/- max_voltage (datasheet typical supply)
        v_command = np.clip(v_command, -self.max_voltage, self.max_voltage)
        
        # SUB-STEPPING: RL circuit needs ~0.5ms steps for stability at 28 ohms / 12mH
        # Calculate steps needed to keep sub_dt <= 0.5ms
        target_sub_dt = 0.0005
        n_sub = max(1, int(math.ceil(dt / target_sub_dt)))
        sub_dt = dt / n_sub
        
        current_state = self.state.copy()
        
        for _ in range(n_sub):
            k1 = self.dynamics(current_state, t, v_command, B_inertial)
            k2 = self.dynamics(current_state + 0.5*sub_dt*k1, t + 0.5*sub_dt, v_command, B_inertial)
            k3 = self.dynamics(current_state + 0.5*sub_dt*k2, t + 0.5*sub_dt, v_command, B_inertial)
            k4 = self.dynamics(current_state + sub_dt*k3, t + sub_dt, v_command, B_inertial)
            
            current_state = current_state + (sub_dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)
            
            # Normalize Quaternion every sub-step to prevent drift
            q_norm = np.linalg.norm(current_state[0:4])
            if q_norm > 1e-6:
                current_state[0:4] /= q_norm
            else:
                current_state[0:4] = np.array([1.0, 0.0, 0.0, 0.0])
        
        # FINAL SAFETY: Check for exploded values (NaN/Inf)
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
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,     1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x**2 - 2*y**2]
        ])
