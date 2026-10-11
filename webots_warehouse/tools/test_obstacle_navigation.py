"""Regression checks for blocked goals, detours and stationary waiting."""
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "warehouse_controller"))
sys.modules.setdefault("controller", types.SimpleNamespace(Supervisor=Mock()))
from warehouse_controller import WarehouseController, State
from navigation import Navigator

class NavigationRegression(unittest.TestCase):
    def setUp(self):
        self.c = WarehouseController.__new__(WarehouseController)
        c = self.c
        c.navigator = Navigator(str(ROOT / "data" / "warehouse_layout.json"))
        c.layout = {"cols": 16, "rows": 12}
        c.static_blocked = set(c.navigator.blocked)
        c.temp_blocked = set()
        c.blocked_until = {}
        c.dynamic_blocked = set()
        c.robot = Mock()
        c.robot.getTime.return_value = 20.0
        self.children = c.robot.getRoot().getField.return_value
        self.children.getCount.return_value = 0
        c.stop_wheels = Mock()
        c.set_lift = Mock()
        c.wait_seconds = Mock()
        c.state = State.NAVIGATE
        c.current_goal_cell = (3, 7)
        c.current_cell = Mock(return_value=(4, 7))
        c.resume_navigation_state = State.NAVIGATE
        c.avoid_attempts = 0

    def obstacle(self, position):
        self.children.getCount.return_value = 1
        node = self.children.getMFNode.return_value
        node.getDef.return_value = "OBSTACLE_2"
        node.getPosition.return_value = position
        node.getOrientation.return_value = [1,0,0,0,1,0,0,0,1]
        box = node.getField().getSFNode()
        box.getTypeName.return_value = "Box"
        box.getField().getSFVec3f.return_value = [0.4,0.4,0.4]

    def test_occupied_goal_is_not_exempt(self):
        self.obstacle([-4.5685,0.2,1.4989])
        self.assertEqual(self.c.compute_path((4,7),(3,7)), [])
        self.assertIn((3,7), self.c.dynamic_blocked)

    def test_intermediate_obstacle_has_detour(self):
        self.obstacle([-0.5,0.2,-1.5])
        path = self.c.compute_path((7,7),(7,1))
        self.assertTrue(path)
        self.assertNotIn((7,4),path)
        self.assertFalse(set(path) & self.c.dynamic_blocked)

    def test_waits_without_backing_and_resumes_when_clear(self):
        self.obstacle([-4.5685,0.2,1.4989])
        self.c.backup = Mock()
        self.c._wait_for_path(State.NAVIGATE)
        for _ in range(5):
            self.c._state_wait_for_path()
        self.assertEqual(self.c.state,State.WAIT_FOR_PATH)
        self.c.backup.assert_not_called()
        self.children.getCount.return_value = 0
        self.c._state_wait_for_path()
        self.assertEqual(self.c.state,State.NAVIGATE)

    def test_sensor_goal_block_is_respected_until_expiry(self):
        self.c.temp_blocked.add((3,7))
        self.c.blocked_until[(3,7)] = 30.0
        self.assertEqual(self.c.compute_path((4,7),(3,7)),[])
        self.c.robot.getTime.return_value = 31.0
        self.assertTrue(self.c.compute_path((4,7),(3,7)))

    def test_last_leg_keeps_sensors_enabled(self):
        self.c.get_pose = Mock(return_value=(-3.5,1.5,0))
        self.c.rotate_to = Mock(return_value=True)
        self.c.drive_to = Mock(return_value=False)
        self.assertFalse(self.c.follow_path([(4,7),(3,7)],goal_is_access=True))
        self.c.drive_to.assert_called_once_with(-4.5,1.5)

    def test_failed_turn_stops_path_before_driving(self):
        self.c.get_pose = Mock(return_value=(-3.5,1.5,0))
        self.c.rotate_to = Mock(return_value=False)
        self.c.drive_to = Mock()
        self.assertFalse(self.c.follow_path([(4,7),(3,7)]))
        self.c.set_lift.assert_called_once_with(0.0)
        self.c.drive_to.assert_not_called()

    def test_scan_lines_up_with_slot_before_lifting(self):
        c = self.c
        c.current_target = types.SimpleNamespace(position=(-4.75,0.8,0.5),
            access_cell=(3,7), shelf_cell=(3,6),product_id="P015")
        c.get_pose = Mock(return_value=(-4.5,1.5,0))
        c.rotate_to = Mock(return_value=True)
        c.drive_to = Mock(return_value=True)
        c.devices = {"camera":Mock()}
        c.scanner = Mock()
        c.scanner.scan.return_value = "P015;3.0"
        sequence = Mock()
        sequence.attach_mock(c.set_lift,"lift")
        sequence.attach_mock(c.rotate_to,"turn")
        sequence.attach_mock(c.drive_to,"drive")
        c._state_align_and_scan()
        c.drive_to.assert_called_once_with(-4.75,1.5)
        self.assertEqual(sequence.mock_calls[0].args,(0.0,))
        self.assertEqual([x[0] for x in sequence.mock_calls],
                         ["lift","turn","drive","turn","lift"])
        self.assertEqual(c.state,State.VERIFY_ITEM)

    def test_carried_box_physics_restored_on_unload(self):
        c = self.c
        item = types.SimpleNamespace(product_id="P015", expected_weight=3.0)
        c.current_target = item
        node = Mock()
        fields = {name:Mock() for name in
                  ["physics","boundingObject","translation","rotation"]}
        fields["physics"].getSFNode().exportString.return_value = "Physics { mass 3 }"
        fields["boundingObject"].getSFNode().exportString.return_value = "Box { size .25 .25 .25 }"
        node.getField.side_effect = fields.__getitem__
        c.verify_node = node
        c.carried = []
        c.load_weight = 0
        c.set_gripper = Mock()
        c._update_carried = Mock()
        c._continue_after_item = Mock()
        c.order_manager = Mock()
        c._state_pick_item()
        fields["physics"].removeSF.assert_called_once()
        fields["boundingObject"].removeSF.assert_called_once()
        c.order_manager.mark_ok.assert_called_once_with(item)
        c.current_order = types.SimpleNamespace(items=[])
        c.trips_done = 0
        c._state_unload()
        fields["physics"].importSFNodeFromString.assert_called_once_with("Physics { mass 3 }")
        fields["boundingObject"].importSFNodeFromString.assert_called_once_with("Box { size .25 .25 .25 }")
        self.assertEqual(c.carried,[])

    def test_blocked_terminal_does_not_unload(self):
        self.c.current_cell.return_value = (7,10)
        self.c.compute_path = Mock(return_value=[])
        self.c._state_return_to_terminal()
        self.assertEqual(self.c.state,State.WAIT_FOR_PATH)
        self.assertEqual(self.c.resume_navigation_state,State.RETURN_TO_TERMINAL)

if __name__ == "__main__":
    unittest.main()
