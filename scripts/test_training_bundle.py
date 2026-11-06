"""Regression checks for the shared, versioned run002 model/control pair."""
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from build_run002_bundle import REVISION, MODEL_FILE, SEMANTIC_SHA256, semantic_sha
from control_profile import ROOT
from validate_training_bundle import validate_training_bundle


class TrainingBundleTests(unittest.TestCase):
    def test_compiled_bundle(self):
        model, profile = validate_training_bundle()
        self.assertEqual(model.nv, 37)
        self.assertEqual(profile.data['timing']['simulation_pd_hz'], 500)

    def test_training_xml_matches_archived_source_semantics(self):
        tree = ET.parse(ROOT / 'models' / REVISION / MODEL_FILE).getroot()
        self.assertEqual(semantic_sha(tree), SEMANTIC_SHA256)

    def test_shoulder_armature_regression_is_detected(self):
        tree = ET.parse(ROOT / 'models' / REVISION / MODEL_FILE).getroot()
        tree.find(".//joint[@name='left_shoulder_roll_joint']").set('armature', '0.0018')
        self.assertNotEqual(semantic_sha(tree), SEMANTIC_SHA256)

    def test_disabled_contact_regression_is_detected(self):
        tree = ET.parse(ROOT / 'models' / REVISION / MODEL_FILE).getroot()
        tree.find(".//geom[@name='left_foot_collision']").set('contype', '0')
        self.assertNotEqual(semantic_sha(tree), SEMANTIC_SHA256)

    def test_pointer_mismatch_rejected(self):
        bundle = json.loads((ROOT / 'TRAINING_BUNDLE.json').read_text())
        bundle['model'] = 'models/historical.xml'
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'TRAINING_BUNDLE.json').write_text(json.dumps(bundle))
            with self.assertRaisesRegex(ValueError, 'pointer mismatch'):
                validate_training_bundle(root)

    def test_new_meshes_and_urdf_equal_public_source(self):
        base = ROOT / 'models/sprite0825_v5_4340_shoulders_waist20261105'
        asset = ROOT / 'models' / REVISION
        self.assertEqual((base / 'sprite0825_float.urdf').read_bytes(), (asset / 'sprite0825_float.urdf').read_bytes())
        for path in (base / 'meshes').glob('*.STL'):
            self.assertEqual(path.read_bytes(), (asset / 'meshes' / path.name).read_bytes())


if __name__ == '__main__':
    unittest.main()
