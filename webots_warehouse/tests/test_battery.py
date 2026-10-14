import ast
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
CONTROLLERS = ROOT / 'controllers/warehouse_controller'
sys.path.insert(0, str(CONTROLLERS))
from battery_model import Battery

# Load the actual FSM methods without requiring Webots native libraries.
tree = ast.parse((CONTROLLERS / 'warehouse_controller.py').read_text(encoding='utf-8'))
state = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'State')
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'WarehouseController')
selected = {'_maybe_charge', '_state_charging', '_dock_contact'}
cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in selected]
namespace = {'math': math, 'normalize_angle': lambda x: (x + math.pi) % (2 * math.pi) - math.pi}
exec(compile(ast.Module(body=[state, cls], type_ignores=[]), '<battery-fsm>', 'exec'), namespace)
Controller, State = namespace['WarehouseController'], namespace['State']


class BatteryTests(unittest.TestCase):
    def test_energy_tracks_time_not_frame_count(self):
        a, b = Battery(), Battery()
        a.update(10, wheel_speed=5, load_kg=20, manipulating=True)
        for _ in range(1000):
            b.update(0.01, wheel_speed=5, load_kg=20, manipulating=True)
        self.assertAlmostEqual(a.level, b.level, places=8)

    def test_load_and_manipulator_use_more_energy(self):
        idle, moving, loaded, lifting = [Battery() for _ in range(4)]
        idle.update(10)
        moving.update(10, wheel_speed=5)
        loaded.update(10, wheel_speed=5, load_kg=50)
        lifting.update(10, wheel_speed=5, load_kg=50, manipulating=True)
        self.assertGreater(idle.level, moving.level)
        self.assertGreater(moving.level, loaded.level)
        self.assertGreater(loaded.level, lifting.level)

    def test_limits_and_charging(self):
        b = Battery(level=1)
        self.assertEqual(b.update(1000, wheel_speed=5), 0)
        self.assertEqual(b.update(1000, docked=True), 100)

    def test_reserve_includes_return_route(self):
        b = Battery(level=50)
        self.assertFalse(b.needs_charge(10))
        self.assertTrue(b.needs_charge(50))
        b.level = 95
        self.assertFalse(b.needs_charge(1000))

    def test_charge_request_preserves_target_and_cargo(self):
        c = Controller()
        c.battery = Battery(level=25)
        c.state = State.NAVIGATE
        c.current_target = SimpleNamespace(access_cell=(2, 1))
        c.trip_queue = ['next']
        c.carried = ['parcel']
        c.charge_cell = (15, 11)
        c.current_cell = lambda: (3, 1)
        c.compute_path = lambda start, end: [start, end]
        c._request_charge = Mock()
        c._maybe_charge()
        c._request_charge.assert_called_once_with(State.NAVIGATE)
        self.assertEqual(c.trip_queue, ['next'])
        self.assertEqual(c.carried, ['parcel'])
        self.assertEqual(c.current_target.access_cell, (2, 1))

    def test_charging_resumes_only_at_target(self):
        c = Controller()
        c.stop_wheels = Mock()
        c._dock_contact = lambda: True
        c.battery = Battery(level=94)
        c.wait_seconds, c.set_state = Mock(), Mock()
        c.charge_cycles = 0
        c.robot_id = 'test'
        c.charge_resume_state = State.NAVIGATE
        c._state_charging()
        c.wait_seconds.assert_called_once()
        c.set_state.assert_not_called()
        c.battery.level = 95
        c._state_charging()
        c.set_state.assert_called_once_with(State.NAVIGATE)
        self.assertEqual(c.charge_cycles, 1)

    def test_lost_dock_returns_to_navigation(self):
        c = Controller()
        c.stop_wheels, c.set_state = Mock(), Mock()
        c._dock_contact = lambda: False
        c._state_charging()
        c.set_state.assert_called_once_with(State.GO_CHARGE)

    def test_contact_requires_position_and_heading(self):
        c = Controller()
        c.charge_cell = (15, 11)
        c.navigator = SimpleNamespace(cell_to_world=lambda cell: (7.5, 5.5))
        node = Mock()
        c.robot = SimpleNamespace(getSelf=lambda: node)
        node.getPosition.return_value = [7.5, 0, 5.5]
        node.getOrientation.return_value = [-1, 0, 0, 0, 1, 0, 0, 0, -1]
        self.assertTrue(c._dock_contact())
        node.getPosition.return_value = [7.5, 0, 4.5]
        self.assertFalse(c._dock_contact())
        node.getPosition.return_value = [7.5, 0, 5.5]
        node.getOrientation.return_value = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        self.assertFalse(c._dock_contact())


if __name__ == '__main__':
    unittest.main()
