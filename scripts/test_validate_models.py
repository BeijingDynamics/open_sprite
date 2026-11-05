"""Focused regression checks: python -m unittest discover -s scripts."""
import contextlib
import copy
import io
from pathlib import Path
import tempfile
import unittest
import shutil
import xml.etree.ElementTree as ET

from validate_models import V5, LATEST_V5, validate_hashes, validate_v5_contract, validate_waist_revision


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

    def test_latest_inertia_matches_compiled_models(self):
        with contextlib.redirect_stdout(io.StringIO()):
            validate_v5_contract(LATEST_V5 / "sprite0825_float.urdf", LATEST_V5 / "mujoco/sprite0825_v5_external_pd.xml")
            validate_waist_revision()

    def test_only_waist_inertial_and_root_labels_changed(self):
        def signature(element):
            return element.tag, tuple(sorted(element.attrib.items())), tuple(signature(c) for c in element)
        for name in ("sprite0825_float.urdf", "mujoco/sprite0825_v5_external_pd.xml", "mujoco/sprite0825_v5_visual_locked.xml"):
            roots = [copy.deepcopy(ET.parse(folder / name).getroot()) for folder in (V5, LATEST_V5)]
            for root in roots:
                root.attrib.pop("name" if root.tag == "robot" else "model")
                parent = root.find("link[@name='waist_yaw_link']" if root.tag == "robot" else ".//body[@name='waist_yaw_link']")
                parent.remove(parent.find("inertial"))
            self.assertEqual(signature(roots[0]), signature(roots[1]), name)

    def test_old_waist_mass_rejected_in_latest_revision(self):
        with tempfile.TemporaryDirectory() as directory:
            asset = Path(directory) / "asset"
            shutil.copytree(LATEST_V5, asset)
            path = asset / "sprite0825_float.urdf"
            tree = ET.parse(path)
            tree.find("link[@name='waist_yaw_link']/inertial/mass").set("value", "2.07092842025253")
            tree.write(path)
            with self.assertRaisesRegex(ValueError, "waist mass/COM/frame mismatch"):
                validate_waist_revision(asset)

    def test_legacy_meshes_unchanged(self):
        for path in (V5 / "meshes").glob("*.STL"):
            self.assertEqual(path.read_bytes(), (LATEST_V5 / "meshes" / path.name).read_bytes())

    def test_mjcf_off_diagonal_inertia_error_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            asset = Path(directory) / "asset"
            shutil.copytree(LATEST_V5, asset)
            path = asset / "mujoco/sprite0825_v5_external_pd.xml"
            tree = ET.parse(path)
            inertia = tree.find(".//body[@name='waist_yaw_link']/inertial")
            values = inertia.get("fullinertia").split()
            values[4] = "-5.083e-05"
            inertia.set("fullinertia", " ".join(values))
            tree.write(path)
            with self.assertRaisesRegex(ValueError, "source waist inertia mismatch"):
                validate_waist_revision(asset)


if __name__ == "__main__":
    unittest.main()
