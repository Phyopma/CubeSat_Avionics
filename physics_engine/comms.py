import serial
import struct


def decode_output_payload(payload, m_cmd_full_scale, tau_full_scale, expected_version):
    fmt = "<3fB9hB"
    expected_size = struct.calcsize(fmt)
    if len(payload) != expected_size:
        raise ValueError(f"Invalid payload length: {len(payload)} (expected {expected_size})")

    unpacked = struct.unpack(fmt, payload)
    command_voltage = unpacked[0:3]
    packed_mode = unpacked[3] & 0xFF
    q15_vals = unpacked[4:13]
    telemetry_flags = unpacked[13] & 0xFF

    version = telemetry_flags & 0x0F
    if version != expected_version:
        raise RuntimeError(
            f"Telemetry version mismatch: got {version}, expected {expected_version}. "
            "Update host/firmware together for HITL packet v2."
        )

    def dequant(q, full_scale):
        return (float(q) / 32767.0) * full_scale

    m_cmd = [dequant(q15_vals[0], m_cmd_full_scale), dequant(q15_vals[1], m_cmd_full_scale), dequant(q15_vals[2], m_cmd_full_scale)]
    tau_raw = [dequant(q15_vals[3], tau_full_scale), dequant(q15_vals[4], tau_full_scale), dequant(q15_vals[5], tau_full_scale)]
    tau_proj = [dequant(q15_vals[6], tau_full_scale), dequant(q15_vals[7], tau_full_scale), dequant(q15_vals[8], tau_full_scale)]

    return {
        "command_voltage": command_voltage,
        "packed_mode": packed_mode,
        "m_cmd": m_cmd,
        "tau_raw": tau_raw,
        "tau_proj": tau_proj,
        "telemetry_flags": telemetry_flags,
    }


class SerialInterface:
    def __init__(self, port, baud_rate, m_cmd_full_scale, tau_full_scale, expected_telemetry_version):
        self.ser = serial.Serial(port, baud_rate, timeout=0.1)
        # Input to Firmware:
        # Header(uint16), Current(3), Gyro(3), Mag(3), Quat(4),
        # Kbdot+Kp+Ki+Kd+Dt(5), Flags(1), MaxVoltage_mV(uint16), Dipole_milli(uint16)
        self.struct_fmt_in = "<H3f3f3f4f5fBHH"
        # Output from Firmware v2 payload (after sync header):
        # Voltage(3), PackedMode(1), m_cmd_q15(3), tau_raw_q15(3), tau_proj_q15(3), flags(1)
        self.struct_fmt_out = "<3fB9hB"
        self.payload_size_out = struct.calcsize(self.struct_fmt_out)
        self.sync_byte_1 = b"\xb5"
        self.sync_byte_2 = b"\x62"
        self.m_cmd_full_scale = float(m_cmd_full_scale)
        self.tau_full_scale = float(tau_full_scale)
        self.expected_telemetry_version = int(expected_telemetry_version)

    def send_packet(
        self,
        current_xyz,
        gyro,
        mag,
        quat,
        k_bdot,
        kp,
        ki,
        kd,
        dt,
        debug_flags,
        max_voltage_mV,
        dipole_strength_milli,
    ):
        # Sync Header 0x62B5 (B5 then 62 on Little Endian)
        data = (
            [0x62B5]
            + list(current_xyz)
            + list(gyro)
            + list(mag)
            + list(quat)
            + [k_bdot, kp, ki, kd, dt, debug_flags, max_voltage_mV, dipole_strength_milli]
        )
        packet = struct.pack(self.struct_fmt_in, *data)

        # Flush input to ensure we get fresh data response to this packet
        self.ser.reset_input_buffer()
        self.ser.write(packet)

    def read_packet(self):
        # Try to find Sync Header. Max scan steps to avoid infinite loop.
        for _ in range(50):
            b1 = self.ser.read(1)
            if not b1:
                return None  # Timeout

            if b1 == self.sync_byte_1:
                b2 = self.ser.read(1)
                if b2 == self.sync_byte_2:
                    payload = self.ser.read(self.payload_size_out)
                    if len(payload) == self.payload_size_out:
                        return decode_output_payload(
                            payload,
                            m_cmd_full_scale=self.m_cmd_full_scale,
                            tau_full_scale=self.tau_full_scale,
                            expected_version=self.expected_telemetry_version,
                        )
        return None

    def close(self):
        self.ser.close()
