"""Control snapshot loading and integrity checks, without torch or hardware."""
import copy
import json
from pathlib import Path
import tempfile
import subprocess
import sys
import unittest

from control_profile import DEFAULT_PROFILE, HISTORICAL_PROFILE, ROOT, _pairs, load_control_profile, validate_profile_data


class ControlProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile = load_control_profile()

    def changed(self):
        return copy.deepcopy(self.profile.data)

    def test_complete_snapshot(self):
        self.assertEqual(len(self.profile.joints), 31)
        self.assertFalse(self.profile.data['hardware_qualified'])
        self.assertEqual(self.profile.joints['left_hip_roll_joint'].kp, 150)
        self.assertEqual(self.profile.joints['left_knee_joint'].kd, 5)
        self.assertEqual(self.profile.joints['waist_yaw_joint'].command_effort_nm, 5)
        self.assertEqual(self.profile.joints['left_wrist_pitch_joint'].command_effort_nm, .03)
        self.assertEqual(self.profile.joints['left_ankle_pitch_joint'].kp, 14.212230682373047)

    def test_order_is_caller_selected(self):
        names = tuple(reversed(self.profile.joints))
        controls = self.profile.for_joint_names(names)
        self.assertEqual(tuple(controls), names)
        self.assertEqual(controls['left_knee_joint'], self.profile.joints['left_knee_joint'])

    def test_duplicate_requested_joint_rejected(self):
        names = list(self.profile.joints)
        with self.assertRaisesRegex(ValueError, 'Duplicate requested'):
            self.profile.for_joint_names(names + names[:1])

    def test_missing_requested_joint_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Joint set mismatch'):
            self.profile.for_joint_names(list(self.profile.joints)[1:])

    def test_unknown_requested_joint_rejected(self):
        names = list(self.profile.joints)
        names[0] = 'left_ankle_motor_a'
        with self.assertRaisesRegex(ValueError, 'Joint set mismatch'):
            self.profile.for_joint_names(names)

    def test_missing_profile_joint_rejected(self):
        data = self.changed()
        data['joints'].pop('head_yaw_joint')
        with self.assertRaisesRegex(ValueError, 'exactly the31'):
            validate_profile_data(data)

    def test_duplicate_json_keys_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Duplicate JSON key'):
            json.loads('{"joints":{"head_yaw_joint":1,"head_yaw_joint":2}}', object_pairs_hook=_pairs)

    def test_nonfinite_values_rejected(self):
        for value in (float('nan'), float('inf'), float('-inf'), True, '130'):
            with self.subTest(value=value):
                data = self.changed()
                data['joints']['left_knee_joint']['kp'] = value
                with self.assertRaisesRegex(ValueError, 'finite number'):
                    validate_profile_data(data)

    def test_negative_gain_rejected(self):
        data = self.changed()
        data['joints']['left_knee_joint']['kd'] = -1
        with self.assertRaisesRegex(ValueError, 'must be >='):
            validate_profile_data(data)

    def test_missing_gain_rejected_without_default(self):
        data = self.changed()
        data['joints']['left_knee_joint'].pop('kp')
        with self.assertRaisesRegex(ValueError, 'Missing control fields'):
            validate_profile_data(data)

    def test_wrong_units_rejected(self):
        data = self.changed()
        data['units']['kp'] = 'N*m/degree'
        with self.assertRaisesRegex(ValueError, 'control units'):
            validate_profile_data(data)

    def test_changed_finite_gain_rejected_by_source_hash(self):
        data = self.changed()
        data['joints']['left_knee_joint']['kp'] = 60
        with self.assertRaisesRegex(ValueError, 'differ from frozen source'):
            validate_profile_data(data)

    def test_hardware_qualification_rejected(self):
        data = self.changed()
        data['hardware_qualified'] = True
        with self.assertRaisesRegex(ValueError, 'hardware qualification'):
            validate_profile_data(data)

    def test_model_hash_mismatch_rejected(self):
        data = self.changed()
        binding = data['model_binding']['files_sha256']
        binding[next(iter(binding))] = '0' * 64
        with self.assertRaisesRegex(ValueError, 'Model hash mismatch'):
            validate_profile_data(data)

    def test_wrong_asset_revision_rejected(self):
        data = self.changed()
        data['model_binding']['asset_revision'] = 'historical_v5'
        with self.assertRaisesRegex(ValueError, 'Asset revision mismatch'):
            validate_profile_data(data)

    def test_missing_model_binding_rejected(self):
        data = self.changed()
        data['model_binding']['files_sha256'].popitem()
        with self.assertRaisesRegex(ValueError, 'Incomplete model hash binding'):
            validate_profile_data(data)

    def test_binding_path_traversal_rejected(self):
        data = self.changed()
        data['model_binding']['asset_directory'] = '../outside'
        with self.assertRaisesRegex(ValueError, 'relative path'):
            validate_profile_data(data)

    def test_default_outside_mechanical_limits_rejected(self):
        data = self.changed()
        data['joints']['head_yaw_joint']['default_pos_rad'] = 10
        with self.assertRaisesRegex(ValueError, 'outside mechanical limits'):
            validate_profile_data(data)

    def test_wrong_mechanical_limits_rejected(self):
        data = self.changed()
        data['joints']['head_yaw_joint']['mechanical_position_limit_rad'] = [-10, 10]
        with self.assertRaisesRegex(ValueError, 'Mechanical limit mismatch'):
            validate_profile_data(data)

    def test_timing_and_delay_rejected(self):
        for key, value in (('decimation', 4), ('physics_dt_s', .005)):
            data = self.changed()
            data['timing'][key] = value
            with self.assertRaisesRegex(ValueError, 'timing'):
                validate_profile_data(data)
        data = self.changed()
        data['timing']['command_delay']['max_physics_ticks'] = 2
        with self.assertRaisesRegex(ValueError, 'Command delay'):
            validate_profile_data(data)

    def test_projection_and_parallel_mapping_rejected(self):
        data = self.changed()
        data['action']['joint_target_projection_enabled'] = True
        with self.assertRaisesRegex(ValueError, 'projection'):
            validate_profile_data(data)
        data = self.changed()
        data['controller']['parallel_linkage_conversion'] = True
        with self.assertRaisesRegex(ValueError, 'runtime emulation'):
            validate_profile_data(data)

    def test_profile_checksum_mismatch_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'control_profiles').mkdir()
            (root / DEFAULT_PROFILE).write_text('{}', encoding='utf-8')
            inventory = ROOT / 'control_profiles/SHA256SUMS'
            (root / 'control_profiles/SHA256SUMS').write_bytes(inventory.read_bytes())
            with self.assertRaisesRegex(ValueError, 'checksum mismatch'):
                load_control_profile(repository_root=root)

    def test_historical_profile_is_still_exactly_loadable(self):
        old = load_control_profile(HISTORICAL_PROFILE)
        self.assertEqual(old.sha256, '184ec989fd24b4bc6ea5034e6fdae796b5b9ce83ef2c884dfe4fc61d8d6987dc')
        self.assertEqual(old.data['timing']['simulation_pd_hz'], 200)
        self.assertEqual(dict(old.joints), dict(self.profile.joints))

    def test_default_is_run002_five_hundred_hz(self):
        self.assertEqual(self.profile.data['timing']['simulation_pd_hz'], 500)
        self.assertEqual(self.profile.data['timing']['decimation'], 10)
        self.assertEqual(self.profile.data['timing']['command_delay']['refresh_physics_ticks'], 10)
        self.assertTrue(self.profile.data['model_binding']['same_complete_model_as_source_training'])

    def test_import_by_file_without_scripts_on_pythonpath(self):
        code = '''
import importlib.util, sys
spec = importlib.util.spec_from_file_location('external_controls', sys.argv[1])
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
profile = module.load_control_profile(repository_root=module.ROOT)
assert profile.data['timing']['simulation_pd_hz'] == 500
'''
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run([sys.executable, '-I', '-c', code, str(ROOT/'scripts/control_profile.py')],
                                    cwd=directory, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_wrong_source_model_rejected(self):
        data = self.changed()
        data['model_binding']['source_training_model_sha256'] = '0' * 64
        with self.assertRaisesRegex(ValueError, 'training source'):
            validate_profile_data(data)

    def test_tracker_velocity_compatibility_name_is_not_urdf_value(self):
        name = 'left_ankle_pitch_joint'
        control = self.profile.joints[name]
        self.assertEqual(control.urdf_velocity_rad_s, control.velocity_limit_rad_s)
        self.assertNotEqual(control.velocity_limit_rad_s, self.profile.data['joints'][name]['urdf_velocity_metadata_rad_s'])


if __name__ == '__main__':
    unittest.main()
