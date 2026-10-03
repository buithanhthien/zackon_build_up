from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot_ui'))

from waypoint_store import (WaypointError, normalize_waypoints, load_waypoint_file,
                            save_waypoint_file, deletion_reason)


def waypoint(**fields):
    return dict(x=1., y=2., z=0., qx=0., qy=0., qz=0., qw=1.,
                map_name='test', **fields)


class SchemaTests(unittest.TestCase):
    def test_legacy_migration_preserves_all_data_and_stable_identity(self):
        original = {'X5.1': waypoint(aliases=['room']), 'home': waypoint(aliases=['nhà'])}
        clean = normalize_waypoints(original)
        self.assertEqual(set(clean), set(original))
        self.assertEqual(clean['home']['x'], original['home']['x'])
        self.assertFalse(clean['X5.1']['deletable'])
        self.assertFalse(clean['home']['deletable'])
        self.assertTrue(clean['home']['deletion_permission_pending'])
        identifier = clean['home']['id']
        clean['home'].update(display_name='Tên mới', aliases=['alias mới'])
        self.assertEqual(normalize_waypoints(clean)['home']['id'], identifier)
        self.assertNotIn('id', original['home'])

    def test_x_rooms_cannot_be_unlocked_or_renamed_to_bypass(self):
        for permission in (None, True, False):
            wp = waypoint()
            if permission is not None:
                wp['deletable'] = permission
            self.assertTrue(deletion_reason('x5.2.4', wp))
        clean = normalize_waypoints({'X5.1': waypoint()})['X5.1']
        clean.update(display_name='Cafe', aliases=['coffee'], deletable=True)
        self.assertTrue(deletion_reason('renamed-key', clean))

    def test_invalid_data_is_rejected_not_silently_discarded(self):
        for fields in ({'deletable': 'false'}, {'deletable': 1}, {'x': float('nan')},
                       {'x': True}, {'aliases': 'home'}, {'aliases': [1]},
                       {'map_name': ''}, {'id': ''}, {'qw': 0.},
                       {'deletion_permission_pending': 'true'}):
            wp = waypoint()
            wp.update(fields)
            with self.subTest(fields=fields), self.assertRaises(WaypointError):
                normalize_waypoints({'bad': wp})
        with self.assertRaises(WaypointError):
            normalize_waypoints({'a': waypoint(id='same'), 'b': waypoint(id='same')})

    def test_atomic_roundtrip_and_failed_replace_preserves_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'waypoints.json'
            clean, raw = save_waypoint_file(path, {'a': waypoint(deletable=True),
                                                 'b': waypoint(deletable=False)}, None)
            self.assertEqual(load_waypoint_file(path)[0], clean)
            with patch('waypoint_store.os.replace', side_effect=OSError('disk full')):
                with self.assertRaises(OSError):
                    save_waypoint_file(path, {}, raw)
            self.assertEqual(path.read_bytes(), raw)
            self.assertEqual(list(Path(directory).iterdir()), [path])
            path.write_text('{}')
            with self.assertRaises(WaypointError):
                save_waypoint_file(path, clean, raw)
            path.write_text('{"a": {}, "a": {}}')
            with self.assertRaises(WaypointError):
                load_waypoint_file(path)


if __name__ == '__main__':
    unittest.main()
