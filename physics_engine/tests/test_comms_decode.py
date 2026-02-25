import pathlib
import struct
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from comms import decode_output_payload


class DecodeOutputPayloadTests(unittest.TestCase):
    def test_decode_payload_v2(self):
        payload = struct.pack(
            "<3fB9hB",
            1.0,
            -2.0,
            3.5,
            0x1D,
            16384,
            -16384,
            8192,
            4096,
            -4096,
            2048,
            -1024,
            1024,
            0,
            0x11,
        )

        decoded = decode_output_payload(
            payload,
            m_cmd_full_scale=8.0,
            tau_full_scale=0.05,
            expected_version=1,
        )

        self.assertAlmostEqual(decoded["command_voltage"][0], 1.0, places=6)
        self.assertEqual(decoded["packed_mode"], 0x1D)
        self.assertEqual(decoded["telemetry_flags"], 0x11)
        self.assertEqual(len(decoded["m_cmd"]), 3)
        self.assertEqual(len(decoded["tau_raw"]), 3)
        self.assertEqual(len(decoded["tau_proj"]), 3)

    def test_version_mismatch_raises(self):
        payload = struct.pack("<3fB9hB", 0.0, 0.0, 0.0, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x02)

        with self.assertRaises(RuntimeError):
            decode_output_payload(payload, m_cmd_full_scale=8.0, tau_full_scale=0.05, expected_version=1)


if __name__ == "__main__":
    unittest.main()
