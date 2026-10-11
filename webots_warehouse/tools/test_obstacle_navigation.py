"""Regression checks for blocked goals, detours and stationary waiting."""
import json
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
from fleet_traffic import refuge_bid, winner

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
        c.robot_id = "WarehouseRobot"
        c.current_order = None
        c.drop_index = 0
        c.terminal_alternate_tried = False
        c.unload_failed = False
        c.wait_time = 0.0
        c.fleet_hold = False
        c._update_hud = Mock()
        c.localizer = None

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

    def test_unloaded_box_does_not_block_terminal_bays(self):
        self.obstacle([1.2, 0.15, 6.4])
        node = self.children.getMFNode.return_value
        node.getDef.return_value = "PRODUCT_P015"
        node.getField().getSFNode().getField().getSFVec3f.return_value = [0.3, 0.3, 0.3]
        self.c.compute_path((7, 10), (8, 11))
        self.assertNotIn((7, 11), self.c.dynamic_blocked)
        self.assertNotIn((8, 11), self.c.dynamic_blocked)

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

    def test_path_skips_recentring_after_shelf_pickup(self):
        c = self.c
        c.get_pose = Mock(return_value=(-2.25, -1.5, 0.0))
        c.rotate_to = Mock(return_value=True)
        c.drive_to = Mock(return_value=True)
        self.assertTrue(c.follow_path([(5, 4), (6, 4)]))
        c.drive_to.assert_called_once_with(-1.5, -1.5)

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
        self.assertEqual(fields["translation"].setSFVec3f.call_args.args[0][2], 6.4)

    def test_blocked_terminal_keeps_cargo_and_reports_failure(self):
        c = self.c
        item = types.SimpleNamespace(product_id="P018", status="ok")
        node = Mock()
        c.carried = [{"item": item, "node": node}]
        c.current_order = types.SimpleNamespace(items=[item])
        c.order_manager = Mock()
        c._emergency_unload()
        self.assertEqual(c.carried[0]["node"], node)
        self.assertTrue(c.unload_failed)
        c.order_manager.mark_unreachable.assert_called_once()
        node.getField.assert_not_called()
        self.assertEqual(c.state, State.REPORT)

    def test_blocked_terminal_does_not_unload(self):
        self.c.current_cell.return_value = (7,10)
        self.c.compute_path = Mock(return_value=[])
        self.c._state_return_to_terminal()
        self.assertEqual(self.c.state,State.WAIT_FOR_PATH)
        self.assertEqual(self.c.resume_navigation_state,State.RETURN_TO_TERMINAL)

    def test_each_robot_has_its_own_terminal_bay(self):
        self.c.current_cell.return_value = (9, 11)
        self.c.compute_path = Mock(return_value=[(9, 11), (8, 11), (7, 11)])
        self.c.follow_path = Mock(return_value=True)
        self.c._state_return_to_terminal()
        self.assertEqual(self.c.current_goal_cell, (7, 11))
        self.c.robot_id = "WarehouseRobot2"
        self.c.state = State.RETURN_TO_TERMINAL
        self.c._state_return_to_terminal()
        self.assertEqual(self.c.current_goal_cell, (8, 11))

    def test_auction_uses_detour_cost_and_safe_refuge(self):
        blocked = self.c.static_blocked
        a = refuge_bid((9, 11), (7, 11), (8, 11), (8, 10), blocked, 16, 12)
        b = refuge_bid((8, 11), (8, 10), (9, 11), (7, 11), blocked, 16, 12)
        self.assertEqual(a[0], 2)
        self.assertEqual(b[0], 0)
        self.assertEqual(a[1], (9, 10))
        self.assertEqual(winner("WarehouseRobot", a[0], "WarehouseRobot2", b[0]),
                         "WarehouseRobot")
        self.assertEqual(winner("WarehouseRobot", 2, "WarehouseRobot2", 2),
                         "WarehouseRobot")

    def test_first_meeting_yields_to_side_not_down_winners_route(self):
        blocked = self.c.static_blocked
        left = refuge_bid((7, 4), (2, 4), (5, 4), (6, 1), blocked, 16, 12)
        right = refuge_bid((5, 4), (6, 1), (7, 4), (2, 4), blocked, 16, 12)
        self.assertEqual(left, (2, (7, 5), [(7, 4), (7, 5)]))
        self.assertEqual(right[1], (1, 4))
        self.assertEqual(winner("WarehouseRobot", left[0],
                                "WarehouseRobot2", right[0]),
                         "WarehouseRobot2")

    def test_announces_next_goal_while_picking(self):
        c = self.c
        c.state = State.PICK_ITEM
        c.trip_queue = [types.SimpleNamespace(access_cell=(6, 1))]
        c._fleet_last_announcement = None
        c._announce_fleet_intent()
        payload = c.robot.getSelf().getField.return_value.setSFString.call_args.args[0]
        self.assertEqual(json.loads(payload)["goal"], [6, 1])

    def test_done_state_is_announced_before_controller_stops(self):
        c = self.c
        c._announce_fleet_intent = Mock()
        c.set_state(State.DONE)
        c._announce_fleet_intent.assert_called_once_with()

if __name__ == "__main__":
    unittest.main()
