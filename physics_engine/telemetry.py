import socket
import time
import math
import struct
import numpy as np

# --- Telemetry Manager (Teleplot) ---
class TelemetryManager:
    def __init__(self, addr):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = addr
        self.mode_map = {0: "Idle", 1: "Detumble", 2: "Spin", 3: "Pointing"}

    def send_telemetry(self, w, q, torque, sim_time, adcs_mode_id, target_I, actual_I, pwm_cmd):
        now_ms = int(time.time() * 1000)
        
        # Calculate angle error (approx to scalar=1)
        # q = [w, x, y, z]
        angle_err = 2 * math.acos(min(1.0, abs(q[0]))) * 180 / math.pi
        
        # Basic Telemetry
        telemetry = (
            f"OmegaX:{now_ms}:{w[0]:.6f}\n"
            f"OmegaY:{now_ms}:{w[1]:.6f}\n"
            f"OmegaZ:{now_ms}:{w[2]:.6f}\n"
            f"AngleError:{now_ms}:{angle_err:.2f}\n"
            f"TrqX:{now_ms}:{torque[0]:.6f}\n"
            f"TrqY:{now_ms}:{torque[1]:.6f}\n"
            f"TrqZ:{now_ms}:{torque[2]:.6f}\n"
            f"QuatW:{now_ms}:{q[0]:.4f}\n"
            f"QuatX:{now_ms}:{q[1]:.4f}\n"
            f"QuatY:{now_ms}:{q[2]:.4f}\n"
            f"QuatZ:{now_ms}:{q[3]:.4f}\n"
            f"ADCS_Mode:{now_ms}:{adcs_mode_id}\n"
            f"Tgt_I_X:{now_ms}:{target_I[0]:.4f}\n"
            f"Tgt_I_Y:{now_ms}:{target_I[1]:.4f}\n"
            f"Tgt_I_Z:{now_ms}:{target_I[2]:.4f}\n"
            f"Act_I_X:{now_ms}:{actual_I[0]:.4f}\n"
            f"Act_I_Y:{now_ms}:{actual_I[1]:.4f}\n"
            f"Act_I_Z:{now_ms}:{actual_I[2]:.4f}\n"
            f"PWM_X:{now_ms}:{pwm_cmd[0]:.2f}\n"
            f"PWM_Y:{now_ms}:{pwm_cmd[1]:.2f}\n"
            f"PWM_Z:{now_ms}:{pwm_cmd[2]:.2f}\n"
        )
        try:
            self.sock.sendto(telemetry.encode(), self.addr)
        except:
            pass # Socket errors shouldn't crash sim

    def send_3d_cube(self, name, q, color, pos=(0, 0, 0), dims=(1, 1, 2)):
        # Format: 3D|name:S:cube:P:x:y:z:Q:x:y:z:w:W:width:H:height:D:depth:C:color
        # Dimensions for 2U: W=1, H=1, D=2 (normalized scale)
        # q = [qw, qx, qy, qz] matching physics.py
        packet = (
            f"3D|{name}:S:cube"
            f":P:{pos[0]}:{pos[1]}:{pos[2]}"
            f":Q:{q[1]:.4f}:{q[2]:.4f}:{q[3]:.4f}:{q[0]:.4f}"
            f":W:{dims[0]}:H:{dims[1]}:D:{dims[2]}"
            f":C:{color}"
        )
        try:
            self.sock.sendto(packet.encode(), self.addr)
        except:
            pass

    def send_3d_vector(self, name, pos, vec, color, scale=1.0, thickness=0.05):
        """
        Visualize a 3D vector as a thin oriented cube (teleplot only supports cube/sphere).
        
        Args:
            name: Unique name for this vector (use "name,widget" to group in same widget)
            pos: Starting position [x, y, z]
            vec: Direction vector [vx, vy, vz] - magnitude determines length
            color: Color string (e.g. "red", "#ff0000")
            scale: Scale factor for vector length
            thickness: Thickness of the vector representation
        """
        vec = np.array(vec) * scale
        length = np.linalg.norm(vec)
        
        if length < 1e-9:
            return  # Skip zero-length vectors
        
        # Compute center position (midpoint of vector)
        center = np.array(pos) + vec * 0.5
        
        # Compute quaternion to rotate from +Z axis to vec direction
        quat = self._direction_to_quaternion(vec)
        
        # Cube dimensions: thin in X/Y, elongated in Z (depth)
        packet = (
            f"3D|{name}:S:cube"
            f":P:{center[0]:.4f}:{center[1]:.4f}:{center[2]:.4f}"
            f":Q:{quat[1]:.4f}:{quat[2]:.4f}:{quat[3]:.4f}:{quat[0]:.4f}"
            f":W:{thickness}:H:{thickness}:D:{length:.4f}"
            f":C:{color}"
        )
        try:
            self.sock.sendto(packet.encode(), self.addr)
        except:
            pass

    def send_3d_plane(self, name, normal_vector, color, size=10.0, opacity=0.3):
        """
        Visualize a plane perpendicular to the given normal vector.
        The plane is represented as a very thin, large cube.
        
        Args:
            name: Unique name for this plane (use "name,widget" to group in same widget)
            normal_vector: Vector [nx, ny, nz] normal to the plane
            color: Color string (e.g. "red", "#ff0000")
            size: Width and Height of the plane (square)
            opacity: Transparency level (0=invisible, 1=opaque)
        """
        normal = np.array(normal_vector)
        length = np.linalg.norm(normal)
        if length < 1e-9:
            return

        # Calculate rotation to align Z-axis with normal vector
        quat = self._direction_to_quaternion(normal)
        
        # Dimensions: W=size, H=size, D=thin (along normal)
        thickness = 0.01 
        
        # Position at origin (unless we want to offset it, but usually B-field passes through origin)
        pos = (0, 0, 0)

        # Format: 3D|name:S:cube:P:x:y:z:Q:x:y:z:w:W:width:H:height:D:depth:C:color:O:opacity
        packet = (
            f"3D|{name}:S:cube"
            f":P:{pos[0]}:{pos[1]}:{pos[2]}"
            f":Q:{quat[1]:.4f}:{quat[2]:.4f}:{quat[3]:.4f}:{quat[0]:.4f}"
            f":W:{size}:H:{size}:D:{thickness}"
            f":C:{color}:O:{opacity}"
        )
        try:
            self.sock.sendto(packet.encode(), self.addr)
        except:
            pass
    
    def _direction_to_quaternion(self, direction):
        """
        Compute quaternion to rotate from +Z axis [0,0,1] to the given direction.
        Returns quaternion as [w, x, y, z].
        """
        direction = np.array(direction, dtype=float)
        length = np.linalg.norm(direction)
        if length < 1e-9:
            return np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
        
        dir_normalized = direction / length
        z_axis = np.array([0.0, 0.0, 1.0])
        
        # Handle anti-parallel case (pointing in -Z direction)
        dot = np.dot(z_axis, dir_normalized)
        if dot < -0.999999:
            # Rotate 180 degrees around X axis
            return np.array([0.0, 1.0, 0.0, 0.0])
        elif dot > 0.999999:
            # Already aligned
            return np.array([1.0, 0.0, 0.0, 0.0])
        
        # General case: rotation axis = cross(z, dir), angle = acos(dot)
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
        # Total: 8 + 16 + 12 + 12 + 12 + 4 = 64 bytes
        
        # q order: [w, x, y, z] matching physics.py
        packet = struct.pack(
            "<dfffffffffffffi",
            t, 
            q[0], q[1], q[2], q[3],  # Quat [w, x, y, z]
            w[0], w[1], w[2],        # Omega
            torque[0], torque[1], torque[2], # Torque
            b_field[0], b_field[1], b_field[2], # B-Field (Body)
            int(mode_id)
        )
        
        try:
            self.sock.sendto(packet, self.addr)
        except Exception as e:
            # Don't print every frame to avoid spam, but maybe once
            pass
