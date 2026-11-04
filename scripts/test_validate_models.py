"""Focused regression checks: python -m unittest discover -s scripts."""
import contextlib
import io
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from validate_models import V5, validate_hashes, validate_v5_contract


class AssetContractTests(unittest.TestCase):
    def test_published_assets(self):
        with contextlib.redirect_stdout(io.StringIO()):
            validate_hashes()
            validate_v5_contract(V5 / "sprite0825_float.urdf", V5 / "mujoco/sprite0825_v5_external_pd.xml")

    def assert_rejects_joint_edit(self, xpath, attribute, value, message):
        tree = ET.parse(V5 / "sprite0825_float.urdf")
        tree.find(xpath).set(attribute, value)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "modified.urdf"
            tree.write(path)
            with self.assertRaisesRegex(ValueError, message):
                validate_v5_contract(path, V5 / "mujoco/sprite0825_v5_external_pd.xml")

    def test_axis_reversal_rejected(self):
        self.assert_rejects_joint_edit("joint[@name='left_ankle_pitch_joint']/axis", "xyz", "0 1 0", "axis mismatch")

    def test_limit_change_rejected(self):
        self.assert_rejects_joint_edit("joint[@name='right_knee_joint']/limit", "upper", "0.5", "position-limit mismatch")

    def test_old_shoulder_velocity_rejected(self):
        self.assert_rejects_joint_edit("joint[@name='left_shoulder_roll_joint']/limit", "velocity", "3.8", "shoulder contract changed")

    def test_invalid_mass_rejected(self):
        self.assert_rejects_joint_edit("link[@name='pelvis_link']/inertial/mass", "value", "0", "Invalid mass/inertia")

    def test_checksum_tampering_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "models").mkdir()
            (root / "models/changed.xml").write_text("changed", encoding="ascii")
            (root / "ASSET_SHA256SUMS").write_text("0" * 64 + "  models/changed.xml\n", encoding="ascii")
            with self.assertRaisesRegex(ValueError, "Asset checksum mismatch"):
                validate_hashes(root)


if __name__ == "__main__":
    unittest.main()
