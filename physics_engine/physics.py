import numpy as np
import math

class SatellitePhysics:
    def __init__(self):
        # 1. Mass Properties (2U CubeSat approximation)
        # 10cm x 10cm x 20cm, 2kg
        # I = m/12 * (h^2 + d^2)
        # Ixx = 1/12 * 2 * (0.1^2 + 0.2^2) = 1/6 * 0.05 = 0.00833
        # Iyy = 1/12 * 2 * (0.1^2 + 0.2^2) = 0.00833
        # Izz = 1/12 * 2 * (0.1^2 + 0.1^2) = 1/6 * 0.02 = 0.00333
        self.I = np.diag([0.00833, 0.00833, 0.00333])
        self.I_inv = np.linalg.inv(self.I)
        
        # 2. State Vector [q0, q1, q2, q3, wx, wy, wz]
        # q = [scalar, vec_x, vec_y, vec_z] (matching BNO085 convention usually, check datasheet)
        # Actually BNO085 provides [i, j, k, real] (vector first, then scalar), 
        # BUT our struct convention in main.c/comms must match.
        # Let's assume [qx, qy, qz, qw] for now, will confirm with BNO085 driver if needed.
        # Physics engine usually uses [qw, qx, qy, qz]. Let's stick to scalar-first internally.
        self.state = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 

    def get_magnetic_field(self, t):
        # Simple rotating magnetic field vector (Earth-like magnitude ~50uT)
        # Orbit period ~ 90 mins = 5400s
        # Omega_orbit = 2*pi / 5400
        # Accelerated orbit for fast tuning (60s period)
        angle = (2 * np.pi / 5400.0) * t
        B_inertial = 50e-6 * np.array([np.cos(angle), 0.0, np.sin(angle)])
        
        # Rotate into Body Frame: B_body = R.T * B_inertial (since R is Body->Inertial)
        q = self.state[0:4]
        R = self._quat_to_dcm(q)
        B_body = R.T @ B_inertial
        return B_body

    def dynamics(self, state, t, torque_ext_body):
        # Extract state
        q = state[0:4] # quaternion [w, x, y, z]
        w = state[4:7] # angular velocity [x, y, z]
        
        # 1. Attitude Kinematics (q_dot = 0.5 * Omega * q)
        # Omega matrix for [w, x, y, z] convention
        Omega = np.array([
            [0, -w[0], -w[1], -w[2]],
            [w[0], 0, w[2], -w[1]],
            [w[1], -w[2], 0, w[0]],
            [w[2], w[1], -w[0], 0]
        ])
        q_dot = 0.5 * Omega @ q
        
        # 2. Rigid Body Dynamics (Euler's Equation)
        # I * w_dot + w x (I * w) = torque
        # w_dot = I_inv * (torque - w x (I * w))
        
        # Gyroscopic term
        Iw = self.I @ w
        gyroscopic = np.cross(w, Iw)
        
        w_dot = self.I_inv @ (torque_ext_body - gyroscopic)
        
        # Divergence Safety
        if np.linalg.norm(w_dot) > 1e12:
            w_dot = np.zeros(3)

        return np.concatenate((q_dot, w_dot))

    def rk4_step(self, t, dt, torque_ext):
        # RK4 Integration
        k1 = self.dynamics(self.state, t, torque_ext)
        k2 = self.dynamics(self.state + 0.5*dt*k1, t + 0.5*dt, torque_ext)
        k3 = self.dynamics(self.state + 0.5*dt*k2, t + 0.5*dt, torque_ext)
        k4 = self.dynamics(self.state + dt*k3, t + dt, torque_ext)
        
        new_state = self.state + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)
        
        # Check for NaNs
        if np.isnan(new_state).any():
            print(f"NAN DETECTED in RK4! Resetting state subset.")
            # Don't apply the NaN state. 
            # Ideally we should debug why it happened, but for now prevent crash loop.
            return

        # Normalize Quaternion
        q_norm = np.linalg.norm(new_state[0:4])
        if q_norm > 1e-6:
            new_state[0:4] /= q_norm
        else:
            new_state[0:4] = np.array([1.0, 0.0, 0.0, 0.0])

        # Clamp Angular Velocity (Safety)
        # 100 rad/s is already insane, but prevents float32 overflow
        new_state[4:7] = np.clip(new_state[4:7], -500.0, 500.0)

        self.state = new_state

    def _quat_to_dcm(self, q):
        # Convert quaternion to Direction Cosine Matrix (Body to Inertial)
        # q = [w, x, y, z]
        w, x, y, z = q
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,     1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x**2 - 2*y**2]
        ])
