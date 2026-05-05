import pathlib
import sys
import unittest

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from constants_loader import DEFAULT_STRUCTURAL_CONFIG_PATH, load_structural_constants


class StructuralConstantsTests(unittest.TestCase):
    def test_load_default_w26_constants(self):
        constants = load_structural_constants(DEFAULT_STRUCTURAL_CONFIG_PATH)

        self.assertAlmostEqual(constants.period_s, 5739.0, places=6)
        self.assertAlmostEqual(constants.inclination_deg, 97.5977, places=6)
        self.assertEqual(constants.inertia_tensor_body.shape, (3, 3))

    def test_inertia_tensor_is_symmetric_positive_definite(self):
        constants = load_structural_constants(DEFAULT_STRUCTURAL_CONFIG_PATH)
        inertia = constants.inertia_tensor_body

        self.assertTrue(np.allclose(inertia, inertia.T, atol=1e-9))

        eigvals = np.linalg.eigvalsh(inertia)
        self.assertTrue(np.all(eigvals > 0.0), msg=f"Invalid eigenvalues: {eigvals}")


if __name__ == "__main__":
    unittest.main()
