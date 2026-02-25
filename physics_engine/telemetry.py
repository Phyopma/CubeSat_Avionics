import math
import socket
import struct
import time

import numpy as np


# --- Telemetry Manager (Teleplot) ---
class TelemetryManager:
    def __init__(self, addr):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = addr
        self.mode_map = {0: "Idle", 1: "Detumble", 2: "Spin", 3: "Pointing"}

    def send_telemetry(
        self,
        w,
        q,
        tau_applied,
        sim_time,
        adcs_mode_id,
        target_i,
        actual_i,
        pwm_cmd,
        b_body,
        b_eci,
        m_cmd_fw,
        m_applied,
        tau_raw_fw,
        tau_proj_fw,
        proj_loss_fw,
        int_clamp_fw,
        telemetry_flags,
    ):
        now_ms = int(time.time() * 1000)

        # Calculate angle error from scalar part only (same convention as existing view)
        angle_err = 2 * math.acos(min(1.0, abs(float(q[0])))) * 180.0 / math.pi

        b_body_mag = float(np.linalg.norm(b_body))
        b_eci_mag = float(np.linalg.norm(b_eci))

        telemetry = (
            f"OmegaX:{now_ms}:{w[0]:.6f}\n"
            f"OmegaY:{now_ms}:{w[1]:.6f}\n"
            f"OmegaZ:{now_ms}:{w[2]:.6f}\n"
            f"AngleError:{now_ms}:{angle_err:.3f}\n"
            f"SimTime:{now_ms}:{sim_time:.3f}\n"
            f"ADCS_Mode:{now_ms}:{adcs_mode_id}\n"
            f"IntClampFW:{now_ms}:{int_clamp_fw}\n"
            f"ProjLossFW:{now_ms}:{proj_loss_fw:.6f}\n"
            f"TelemSatFlags:{now_ms}:{telemetry_flags}\n"
            f"QuatW:{now_ms}:{q[0]:.6f}\n"
            f"QuatX:{now_ms}:{q[1]:.6f}\n"
            f"QuatY:{now_ms}:{q[2]:.6f}\n"
            f"QuatZ:{now_ms}:{q[3]:.6f}\n"
            f"B_body_X:{now_ms}:{b_body[0]:.8e}\n"
            f"B_body_Y:{now_ms}:{b_body[1]:.8e}\n"
            f"B_body_Z:{now_ms}:{b_body[2]:.8e}\n"
            f"B_body_mag:{now_ms}:{b_body_mag:.8e}\n"
            f"B_eci_X:{now_ms}:{b_eci[0]:.8e}\n"
            f"B_eci_Y:{now_ms}:{b_eci[1]:.8e}\n"
            f"B_eci_Z:{now_ms}:{b_eci[2]:.8e}\n"
            f"B_eci_mag:{now_ms}:{b_eci_mag:.8e}\n"
            f"mCmdFW_X:{now_ms}:{m_cmd_fw[0]:.6e}\n"
            f"mCmdFW_Y:{now_ms}:{m_cmd_fw[1]:.6e}\n"
            f"mCmdFW_Z:{now_ms}:{m_cmd_fw[2]:.6e}\n"
            f"mApplied_X:{now_ms}:{m_applied[0]:.6e}\n"
            f"mApplied_Y:{now_ms}:{m_applied[1]:.6e}\n"
            f"mApplied_Z:{now_ms}:{m_applied[2]:.6e}\n"
            f"tauRawFW_X:{now_ms}:{tau_raw_fw[0]:.6e}\n"
            f"tauRawFW_Y:{now_ms}:{tau_raw_fw[1]:.6e}\n"
            f"tauRawFW_Z:{now_ms}:{tau_raw_fw[2]:.6e}\n"
            f"tauProjFW_X:{now_ms}:{tau_proj_fw[0]:.6e}\n"
            f"tauProjFW_Y:{now_ms}:{tau_proj_fw[1]:.6e}\n"
            f"tauProjFW_Z:{now_ms}:{tau_proj_fw[2]:.6e}\n"
            f"tauApplied_X:{now_ms}:{tau_applied[0]:.6e}\n"
            f"tauApplied_Y:{now_ms}:{tau_applied[1]:.6e}\n"
            f"tauApplied_Z:{now_ms}:{tau_applied[2]:.6e}\n"
            f"Tgt_I_X:{now_ms}:{target_i[0]:.6f}\n"
            f"Tgt_I_Y:{now_ms}:{target_i[1]:.6f}\n"
            f"Tgt_I_Z:{now_ms}:{target_i[2]:.6f}\n"
            f"Act_I_X:{now_ms}:{actual_i[0]:.6f}\n"
            f"Act_I_Y:{now_ms}:{actual_i[1]:.6f}\n"
            f"Act_I_Z:{now_ms}:{actual_i[2]:.6f}\n"
            f"PWM_X:{now_ms}:{pwm_cmd[0]:.6f}\n"
            f"PWM_Y:{now_ms}:{pwm_cmd[1]:.6f}\n"
            f"PWM_Z:{now_ms}:{pwm_cmd[2]:.6f}\n"
        )

        try:
            self.sock.sendto(telemetry.encode(), self.addr)
        except Exception:
            pass  # Socket errors shouldn't crash sim

    def send_3d_cube(self, name, q, color, pos=(0, 0, 0), dims=(1, 1, 2)):
        # Format: 3D|name:S:cube:P:x:y:z:Q:x:y:z:w:W:width:H:height:D:depth:C:color
        packet = (
            f"3D|{name}:S:cube"
            f":P:{pos[0]}:{pos[1]}:{pos[2]}"
            f":Q:{q[1]:.4f}:{q[2]:.4f}:{q[3]:.4f}:{q[0]:.4f}"
            f":W:{dims[0]}:H:{dims[1]}:D:{dims[2]}"
            f":C:{color}"
        )
        try:
            self.sock.sendto(packet.encode(), self.addr)
        except Exception:
            pass

    def send_3d_vector(self, name, pos, vec, color, scale=1.0, thickness=0.05):
        """
        Visualize a 3D vector as a thin oriented cube (teleplot only supports cube/sphere).
        """
        vec = np.array(vec) * scale
        length = np.linalg.norm(vec)

        if length < 1e-9:
            return  # Skip zero-length vectors

        center = np.array(pos) + vec * 0.5
        quat = self._direction_to_quaternion(vec)

        packet = (
            f"3D|{name}:S:cube"
            f":P:{center[0]:.4f}:{center[1]:.4f}:{center[2]:.4f}"
            f":Q:{quat[1]:.4f}:{quat[2]:.4f}:{quat[3]:.4f}:{quat[0]:.4f}"
            f":W:{thickness}:H:{thickness}:D:{length:.4f}"
            f":C:{color}"
        )
        try:
            self.sock.sendto(packet.encode(), self.addr)
        except Exception:
            pass

    def send_3d_plane(self, name, normal_vector, color, size=10.0, opacity=0.3):
        """
        Visualize a plane perpendicular to the given normal vector.
        The plane is represented as a very thin, large cube.
        """
        normal = np.array(normal_vector)
        length = np.linalg.norm(normal)
        if length < 1e-9:
            return

        quat = self._direction_to_quaternion(normal)
        thickness = 0.01
        pos = (0, 0, 0)

        packet = (
            f"3D|{name}:S:cube"
            f":P:{pos[0]}:{pos[1]}:{pos[2]}"
            f":Q:{quat[1]:.4f}:{quat[2]:.4f}:{quat[3]:.4f}:{quat[0]:.4f}"
            f":W:{size}:H:{size}:D:{thickness}"
            f":C:{color}:O:{opacity}"
        )
        try:
            self.sock.sendto(packet.encode(), self.addr)
        except Exception:
            pass

    def _direction_to_quaternion(self, direction):
        """
        Compute quaternion to rotate from +Z axis [0,0,1] to the given direction.
        Returns quaternion as [w, x, y, z].
        """
        direction = np.array(direction, dtype=float)
        length = np.linalg.norm(direction)
        if length < 1e-9:
            return np.array([1.0, 0.0, 0.0, 0.0])

        dir_normalized = direction / length
        z_axis = np.array([0.0, 0.0, 1.0])

        dot = np.dot(z_axis, dir_normalized)
        if dot < -0.999999:
            return np.array([0.0, 1.0, 0.0, 0.0])
        if dot > 0.999999:
            return np.array([1.0, 0.0, 0.0, 0.0])

        axis = np.cross(z_axis, dir_normalized)
        axis = axis / np.linalg.norm(axis)

        half_angle = np.arccos(np.clip(dot, -1.0, 1.0)) / 2.0
        sin_half = np.sin(half_angle)
        cos_half = np.cos(half_angle)

        return np.array([cos_half, axis[0] * sin_half, axis[1] * sin_half, axis[2] * sin_half])


# --- Visualizer Sender (UDP Broadcast) ---
class VisualizerSender:
    def __init__(self, ip="127.0.0.1", port=5555):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (ip, port)

    def send_state(self, t, q, w, torque, b_field, mode_id):
        # Packet Format (Little Endian):
        # 1 double (time) + 4 floats (q) + 3 floats (w) + 3 floats (torque) + 3 floats (B) + 1 int (mode)
        packet = struct.pack(
            "<dfffffffffffffi",
            t,
            q[0],
            q[1],
            q[2],
            q[3],
            w[0],
            w[1],
            w[2],
            torque[0],
            torque[1],
            torque[2],
            b_field[0],
            b_field[1],
            b_field[2],
            int(mode_id),
        )

        try:
            self.sock.sendto(packet, self.addr)
        except Exception:
            pass
