# -*- coding: utf-8 -*-
"""
================================================================================
PLIK: warehouse_controller.py — główny kontroler robota magazynowego (Webots)
================================================================================

PRZEZNACZENIE
-------------
Kontroler steruje robotem mobilnym WarehouseRobot w świecie `worlds/warehouse.wbt`
(Webots R2023b) i realizuje pełny cykl kompletacji zamówień magazynowych:
pobranie zamówienia, optymalizacja trasy, nawigacja po siatce magazynu,
reaktywne omijanie przeszkód, skanowanie kodów QR, weryfikacja produktów
(id + waga), pobieranie produktów na platformę odkładczą (limit 50 kg),
rozładunek w Terminalu Zamówień oraz generowanie raportów JSON.

SPOSÓB UŻYCIA
-------------
Plik jest uruchamiany automatycznie przez Webots jako kontroler robota
(pole `controller "warehouse_controller"` w instancji WarehouseRobot.proto;
robot musi mieć `supervisor TRUE`). Nie przyjmuje argumentów wiersza poleceń.
Wymaga modułów współpracujących w tym samym katalogu:
    - navigation.py    -> klasa Navigator (A*, planowanie kursów, siatka),
    - qr_scanner.py    -> klasa QRScanner + funkcja parse_payload (OpenCV),
    - order_manager.py -> klasy OrderManager / Order / OrderItem (raporty JSON).
Korzeń projektu wyznaczany jest jako Path(__file__).resolve().parents[2].

STRUKTURY DANYCH
----------------
- Stan FSM: łańcuch znaków (klasa State), przejścia w metodach `_state_*`.
- self.current_order : Order      — bieżące zamówienie (order_id, items).
- self.trip_queue    : list[OrderItem] — pozycje bieżącego kursu w kolejności
  minimalizującej dystans (z Navigator.plan_trips).
- self.carried       : list[dict] — noszone produkty:
  {"item": OrderItem, "node": Node, "slot": int}; pozycja węzła jest
  aktualizowana kinematycznie co krok symulacji względem pozy robota.
- self.temp_blocked  : set[tuple[int,int]] — komórki chwilowo zablokowane
  przez wykryte przeszkody dynamiczne (uwzględniane przy przeplanowaniu).
- Pozycja slotu platformy dla indeksu k (w lokalnym układzie robota):
  (±0.15, 0.45 + 0.3*k, ±0.15).

MASZYNA STANÓW (FSM)
--------------------
INIT                — inicjalizacja urządzeń, modułów i Supervisor API.
LOAD_ORDER          — pobranie kolejnego zamówienia (OrderManager.next_order);
                      brak zamówień -> DONE.
PLAN_TRIP           — podział pozycji zamówienia na kursy (limit 50 kg,
                      greedy nearest-neighbour) przez Navigator.plan_trips.
NAVIGATE            — jazda do komórki dostępu (access_cell) kolejnego produktu
                      po waypointach z Navigator.find_path; w locie pilnuje
                      limitu wagowego 50 kg (nadwyżka -> RETURN_TO_TERMINAL).
AVOID_OBSTACLE      — reakcja na przeszkodę (czujniki ds_*): stop, cofka 0.3 m,
                      chwilowa blokada komórki przed robotem, przeplanowanie.
ALIGN_AND_SCAN      — podnosnik na wysokość poziomu (level_height - 0.2),
                      orientacja na regał (r∈{2,5,8} -> patrz +Z;
                      r∈{3,6,9} -> patrz -Z), do 3 prób odczytu QR.
VERIFY_ITEM         — weryfikacja id i wagi z zamówieniem:
                      zgodne -> mark_ok + PICK_ITEM; złe id/waga -> mark_wrong
                      (produkt zostaje na półce); brak boxa (brak DEF
                      PRODUCT_<id> w świecie) -> mark_missing.
PICK_ITEM           — zamknięcie chwytaka, kinematyczne przypięcie produktu
                      do slotu platformy (Supervisor), zwolnienie chwytaka.
RETURN_TO_TERMINAL  — powrót do najbliższej komórki terminala.
UNLOAD              — zrzut noszonych produktów w strefie terminala, zerowanie
                      ładunku; jeśli są pozycje oczekujące -> PLAN_TRIP,
                      w przeciwnym razie -> REPORT.
REPORT              — zbudowanie i zapis raportu JSON (OrderManager), potem
                      kolejne zamówienie -> LOAD_ORDER.
DONE                — koniec pracy, zatrzymanie silników.

NAWIGACJA I POZA
----------------
Poza robota odczytywana z Supervisor API (getSelf().getField("translation" /
"rotation")). Rotacja axis-angle wokół osi Y: jeśli axis.y < 0, kąt przyjmuje
wartość ujemną. Przód robota (lokalne -Z) w świecie: (-sin(theta), 0, -cos(theta)).
Ruch: obrót w miejscu do azymutu waypointa + jazda prosto (napęd różnicowy,
dodatnia prędkość obu kół = jazda do przodu).

LOGI
----
Wszystkie komunikaty po polsku, z prefiksami modułów:
[FSM] — zmiany stanów, [NAV] — nawigacja, [QR] — skanowanie,
[WAGA] — limit udźwigu, [RAPORT] — raporty, [CHWYTAK]/[PODNOSNIK] — efektory.
================================================================================
"""

import heapq
import json
import math
from pathlib import Path

from controller import Supervisor

from navigation import Navigator
from qr_scanner import QRScanner, parse_payload, recognition_fallback
from order_manager import OrderManager
from localization import Localizer
from battery_model import Battery, BatteryEmpty
from fleet_traffic import refuge_bid, winner


# ---------------------------------------------------------------------------
# Stałe konfiguracyjne (zgodne z kontraktem geometrycznym projektu)
# ---------------------------------------------------------------------------

ROOT_DIR = Path(__file__).resolve().parents[2]  # korzeń projektu webots_warehouse/
LAYOUT_PATH = ROOT_DIR / "data" / "warehouse_layout.json"
PRODUCTS_PATH = ROOT_DIR / "data" / "products.json"
ORDERS_DIR = ROOT_DIR / "orders"
REPORTS_DIR = ROOT_DIR / "reports"

WHEEL_RADIUS = 0.08          # promień koła [m] (zgodnie z WarehouseRobot.proto)
TRACK_WIDTH = 0.54           # rozstaw kol [m] (kola na x=+-0.27 w proto)

USE_LOCALIZATION = True      # F1: odometria (enkodery+IMU) zamiast Supervisor ground truth
MAX_WHEEL_SPEED = 6.0        # maks. prędkość kątowa kół [rad/s]
DRIVE_SPEED = 5.0            # prędkość jazdy na wprost [rad/s]
TURN_SPEED = 2.5             # prędkość obrotu w miejscu [rad/s]
BACKUP_SPEED = 2.0           # prędkość cofania [rad/s]

ANGLE_TOLERANCE = 0.04       # tolerancja osiągnięcia azymutu [rad]
DIST_TOLERANCE = 0.015        # tolerancja dojazdu do waypointa [m]
HEADING_GAIN = 2.0           # wzmocnienie korekcji kursu podczas jazdy

DS_THRESHOLD = 900.0         # próg czujników odległości (większe = bliżej);
                             # 900 ≈ 0.18 m — poniżej luzu bocznego w alei
                             # (~0.25 m → odczyt ~853) i dystansu do produktu
                             # w komórce dostępu (~0.48 m → odczyt ~700)
BACKUP_DISTANCE = 0.3        # długość cofki po wykryciu przeszkody [m]
FRONT_PROBE_DISTANCE = 0.6   # odległość sondowania komórki przed robotem [m]
MAX_AVOID_ATTEMPTS = 8       # maks. liczba kolejnych prób omijania przeszkody
WAIT_TIMEOUT_S = 20.0        # po tylu sekundach czekania na droge pozycja jest odkladana
MAX_ITEM_DEFERS = 2          # po tylu odlozeniach pozycja -> status niedostepna
FLEET_MEETING_DISTANCE = 2.1   # mijanka zanim czujniki wymusza objazd
FLEET_CLEAR_DISTANCE = 3.0    # zwyciezca musi minac skrzyzowanie [m]

PLATFORM_LOAD_LIMIT = 50.0   # limit ładunku platformy [kg]
WEIGHT_TOLERANCE = 0.05      # tolerancja porównania wagi z QR [kg]

CAMERA_MOUNT_HEIGHT = 0.25   # wysokość kamery bez podnosnika [m]
LIFT_MIN = 0.0               # zakres LinearMotor podnosnika [m]
LIFT_MAX = 1.6
LIFT_SETTLE_TIME = 2.0       # czas na osiągnięcie pozycji podnosnika [s]

GRIPPER_OPEN_POS = 0.0       # pozycja palców chwytaka: otwarty
GRIPPER_CLOSED_POS = 0.05    # pozycja palców chwytaka: zamknięty (maxPosition wg proto)
GRIPPER_SETTLE_TIME = 0.6    # czas na ruch palców [s]

SCAN_TRIES = 3               # liczba prób odczytu kodu QR
SCAN_SETTLE_TIME = 0.3       # pauza między próbami skanowania [s]

PLATFORM_SLOT_X = 0.15       # offset slotu platformy wzdłuż osi X robota [m]
PLATFORM_SLOT_Z = 0.15       # offset slotu platformy wzdłuż osi Z robota [m]
PLATFORM_SLOT_Y0 = 0.50      # bazowa wysokość slotu platformy [m]
PLATFORM_SLOT_DY = 0.3       # przyrost wysokości kolejnego slotu [m]

# Siatka zrzutu ZA miejscem parkowania w strefie terminala (paczki nigdy
# nie lądują w footprintcie robota ani na torze dojazdu do komórek (7,11)/
# (8,11)); blok kolumn przesunięty dla WarehouseRobot2 (brak kolizji zrzutow
# miedzy robotami); licznik zrzutu jest TRWALY (brak nakladania miedzy
# kursami i zamowieniami).
DROP_GRID_X0 = 1.2           # pierwsza kolumna siatki zrzutu, x [m]
DROP_GRID_DX = 0.35          # odstep miedzy kolumnami [m]
DROP_GRID_COLS = 5           # liczba kolumn w bloku jednego robota
DROP_GRID_Z0 = 6.4           # poza rzedem dojazdu terminala (z=5.5)
DROP_GRID_DZ = 0.3           # drugi rzad miesci sie przed sciana (z<7)
DROP_ZONE_Y = 0.13            # wysokość zrzutu (środek paczki) [m]

SHELF_ROWS_SOUTH_VIEW = (2, 5, 8)   # regały oglądane z północy -> robot patrzy +Z
SHELF_ROWS_NORTH_VIEW = (3, 6, 9)   # regały oglądane z południa -> robot patrzy -Z

DEVICE_NAMES = {
    "wheel_left": "wheel_left",
    "wheel_right": "wheel_right",
    "lift_motor": "lift_motor",
    "gripper_left": "gripper_left",
    "gripper_right": "gripper_right",
    "camera": "camera",
    "ds_front_left": "ds_front_left",
    "ds_front_right": "ds_front_right",
    "ds_left": "ds_left",
    "ds_right": "ds_right",
    "ds_back": "ds_back",
}


class State:
    """Identyfikatory stanów maszyny FSM (łańcuchy znaków do logów)."""

    INIT = "INIT"
    LOAD_ORDER = "LOAD_ORDER"
    PLAN_TRIP = "PLAN_TRIP"
    NAVIGATE = "NAVIGATE"
    AVOID_OBSTACLE = "AVOID_OBSTACLE"
    WAIT_FOR_PATH = "WAIT_FOR_PATH"
    ALIGN_AND_SCAN = "ALIGN_AND_SCAN"
    VERIFY_ITEM = "VERIFY_ITEM"
    PICK_ITEM = "PICK_ITEM"
    RETURN_TO_TERMINAL = "RETURN_TO_TERMINAL"
    UNLOAD = "UNLOAD"
    REPORT = "REPORT"
    GO_CHARGE = "GO_CHARGE"
    CHARGING = "CHARGING"
    BATTERY_EMPTY = "BATTERY_EMPTY"
    DONE = "DONE"


def normalize_angle(angle: float) -> float:
    """Normalizuje kąt do przedziału (-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


class WarehouseController:
    """Główny kontroler robota magazynowego — maszyna stanów FSM."""

    def __init__(self) -> None:
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.state = State.INIT

        # Urządzenia robota (nazwy sztywne, zgodne z WarehouseRobot.proto)
        self.devices = {}

        # Moduły współpracujące (API sztywne wg kontraktu integracyjnego)
        self.navigator: Navigator | None = None
        self.scanner: QRScanner | None = None
        self.order_manager: OrderManager | None = None

        # Dane statyczne magazynu (do lokalnego przeplanowania z blokadami)
        self.layout: dict = {}
        self.static_blocked: set[tuple[int, int]] = set()
        self.temp_blocked: set[tuple[int, int]] = set()
        self.blocked_until = {}
        self.dynamic_blocked = set()
        self.resume_navigation_state = State.NAVIGATE

        # Stan realizacji zamówienia
        self.current_order = None            # Order | None
        self.trip_queue: list = []           # list[OrderItem] bieżącego kursu
        self.current_target = None           # OrderItem | None
        self.current_goal_cell: tuple[int, int] | None = None
        self.trips_done = 0                  # liczba wykonanych kursów (UNLOAD)

        # Ładunek na platformie
        self.carried: list[dict] = []        # {"item", "node", "slot"}
        self.load_weight = 0.0

        # Wynik ostatniego skanowania QR
        self.scan_payload: str | None = None
        self.avoid_attempts = 0
        self.verify_node = None           # węzeł produktu zweryfikowanego do pobrania
        self.drop_index = 0             # trwaly licznik zrzutow (siatka)
        self.wait_time = 0.0            # czas spedzony w WAIT_FOR_PATH [s]
        self.defer_counts: dict = {}    # product_id -> liczba odlozen
        self.terminal_alternate_tried = False  # proba drugiej komorki terminala
        self.unload_failed = False
        self.fleet_refuge = None
        self.fleet_refuge_path = []
        self.fleet_yielding = False
        self.fleet_hold_time = 0.0
        self.fleet_hold = False
        self.fleet_peer = None
        self._fleet_last_announcement = None
        self._fleet_last_bid = None
        self.finished = False             # flaga zakończenia pracy (stan DONE)
        self._lift_pos = 0.0            # biezaca pozycja podnosnika (do animacji)

        # F1: estymator pozy z czujnikow pokladowych (enkodery kol + IMU)
        self.localizer: Localizer | None = None
        # F8: identyfikator robota (z pola name instancji w swiecie)
        self.robot_id = "WarehouseRobot"
        # F3: katalog wag produktow (do fallbacku Recognition)
        self.products_db: dict = {}
        self.battery = None
        self.charge_resume_state = State.LOAD_ORDER
        self.charge_cell = None
        self.charge_cycles = 0
        self._battery_hud_at = 0.0
        self._battery_fields = None
        self._charger_light = None

    # ------------------------------------------------------------------
    # Narzędzia kroku symulacji i pozy robota
    # ------------------------------------------------------------------

    def step(self) -> bool:
        """Wykonuje krok symulacji i aktualizuje kinematykę noszonych produktów."""
        if self.robot.step(self.timestep) == -1:
            return False
        if self.localizer is not None:
            self.localizer.update()
        self._update_carried()
        self._update_energy()
        self._announce_fleet_intent()
        return True

    def wait_seconds(self, seconds: float) -> None:
        """Odczekuje zadany czas symulacji (z aktualizacją ładunku)."""
        steps = max(1, int(seconds * 1000.0 / self.timestep))
        for _ in range(steps):
            if not self.step():
                return

    def get_pose(self) -> tuple[float, float, float]:
        """Zwraca pozę robota (x, z, theta).

        F1: domyslnie estymata z Localizer (enkodery kol + IMU) -- bez
        Supervisor ground truth w trakcie ruchu. Fallback do Supervisor API,
        gdy enkodery niedostepne albo USE_LOCALIZATION=False.
        """
        if USE_LOCALIZATION and self.localizer is not None \
                and self.localizer.available:
            return self.localizer.pose
        node = self.robot.getSelf()
        pos = node.getField("translation").getSFVec3f()
        # Kurs z macierzy orientacji (odporny na pochylenie kadluba):
        # przod robota = (-sin(theta), 0, -cos(theta)) = -kolumna 3 R.
        ori = node.getOrientation()
        theta = math.atan2(ori[2], ori[8])
        return pos[0], pos[2], normalize_angle(theta)

    def world_to_cell(self, x: float, z: float) -> tuple[int, int]:
        """Przelicza współrzędne świata na najbliższą komórkę siatki."""
        origin = self.layout.get("origin", {"x": -7.5, "z": -5.5})
        cols = int(self.layout.get("cols", 16))
        rows = int(self.layout.get("rows", 12))
        c = int(round(x - origin["x"]))
        r = int(round(z - origin["z"]))
        return (min(max(c, 0), cols - 1), min(max(r, 0), rows - 1))

    def current_cell(self) -> tuple[int, int]:
        """Zwraca komórkę siatki, w której stoi robot."""
        x, z, _ = self.get_pose()
        return self.world_to_cell(x, z)

    # ------------------------------------------------------------------
    # Sterowanie efektorami
    # ------------------------------------------------------------------

    def stop_wheels(self) -> None:
        """Zatrzymuje oba koła napędowe."""
        self.devices["wheel_left"].setVelocity(0.0)
        self.devices["wheel_right"].setVelocity(0.0)

    def set_lift(self, position: float) -> None:
        """Ustawia podnosnik na zadanej wysokości [m] (zakres 0-1.6)."""
        position = min(max(position, LIFT_MIN), LIFT_MAX)
        self._lift_pos = position
        self.devices["lift_motor"].setPosition(position)
        self.wait_seconds(LIFT_SETTLE_TIME)

    def set_gripper(self, closed: bool) -> None:
        """Otwiera lub zamyka palce chwytaka."""
        position = GRIPPER_CLOSED_POS if closed else GRIPPER_OPEN_POS
        self.devices["gripper_left"].setPosition(position)
        self.devices["gripper_right"].setPosition(position)
        self.wait_seconds(GRIPPER_SETTLE_TIME)

    # ------------------------------------------------------------------
    # Czujniki odległości
    # ------------------------------------------------------------------

    def obstacle_detected(self) -> bool:
        """Sprawdza progi czujnikow przednich ds_front_left/right.

        Czujniki boczne (ds_left/ds_right) sa wylaczone z wykrywania: w alei
        miedzy regalami (luz boczny ~0.25-0.30 m) najmniejszy dryf z osi
        alei falszywie wyzwalal alarm. Wykrywanie przeszkod dynamicznych
        (OBSTACLE_*) dziala czujnikami przednimi podczas jazdy.
        """
        for name in ("ds_front_left", "ds_front_right"):
            value = self.devices[name].getValue()
            if value > DS_THRESHOLD:
                print(f"[NAV] PRZESZKODA: {name}={value:.0f} > {DS_THRESHOLD}")
                return True
        return False

    def _announce_fleet_intent(self):
        """Publish a small snapshot for the other controller in Robot.customData."""
        if not hasattr(self, "robot_id"):
            return
        cell = self.current_cell()
        goal = self.current_goal_cell
        if self.state in (State.ALIGN_AND_SCAN, State.VERIFY_ITEM,
                          State.PICK_ITEM):
            if self.trip_queue:
                goal = tuple(self.trip_queue[0].access_cell)
            else:
                terminal_cells = self.layout.get("terminal_cells", [[7, 11], [8, 11]])
                goal = tuple(terminal_cells[0 if self.robot_id == "WarehouseRobot" else 1])
        snapshot = (cell, goal, self.state)
        if snapshot == self._fleet_last_announcement:
            return
        payload = {"cell": cell, "goal": goal,
                   "state": self.state}
        self.robot.getSelf().getField("customData").setSFString(json.dumps(payload))
        self._fleet_last_announcement = snapshot

    def _other_robot(self):
        children = self.robot.getRoot().getField("children")
        for i in range(children.getCount()):
            node = children.getMFNode(i)
            if node.getTypeName() != "WarehouseRobot":
                continue
            name = node.getField("name").getSFString()
            if name == self.robot_id:
                continue
            try:
                intent = json.loads(node.getField("customData").getSFString() or "{}")
            except (ValueError, TypeError):
                intent = {}
            return name, node.getPosition(), intent
        return None

    def _fleet_meeting(self):
        peer = self._other_robot()
        if peer is None:
            return None
        name, pos, intent = peer
        if intent.get("state") == State.DONE:
            return None
        x, z, theta = self.get_pose()
        dx, dz = pos[0]-x, pos[2]-z
        distance = math.hypot(dx, dz)
        if distance < 0.01 or distance > FLEET_MEETING_DISTANCE:
            return None
        forward = -math.sin(theta)*dx-math.cos(theta)*dz
        lateral = abs(math.cos(theta)*dx-math.sin(theta)*dz)
        if forward < 0.1 or lateral > 0.8:
            return None
        return name, pos, intent

    def _other_robot_orientation(self, name):
        children = self.robot.getRoot().getField("children")
        for i in range(children.getCount()):
            node = children.getMFNode(i)
            if (node.getTypeName() == "WarehouseRobot"
                    and node.getField("name").getSFString() == name):
                return node.getOrientation()
        return None

    def _fleet_auction(self, peer):
        """Compare the extra distance each robot would travel to give way."""
        name, pos, intent = peer
        other_ori = self._other_robot_orientation(name)
        if other_ori is not None:
            _, _, theta = self.get_pose()
            other_theta = math.atan2(other_ori[2], other_ori[8])
            if math.cos(theta-other_theta) > 0.4:
                self.stop_wheels()
                self.wait_seconds(0.25)
                return "wait"  # jazda za robotem: poczekaj, bez objazdu
        their_cell = self.world_to_cell(pos[0], pos[2])
        their_goal = tuple(intent.get("goal") or their_cell)
        our_cell = self.current_cell()
        our_goal = self.current_goal_cell or our_cell
        blocked = self.static_blocked | self.temp_blocked | self.dynamic_blocked
        blocked -= {our_cell, their_cell}
        cols, rows = self.navigator.cols, self.navigator.rows
        our_bid, refuge, route = refuge_bid(
            our_cell, our_goal, their_cell, their_goal, blocked, cols, rows)
        their_bid, _, _ = refuge_bid(
            their_cell, their_goal, our_cell, our_goal, blocked, cols, rows)
        chosen = winner(self.robot_id, our_bid, name, their_bid)
        bid = (our_cell, their_cell, our_goal, their_goal, our_bid,
               their_bid, chosen)
        if bid != self._fleet_last_bid:
            print(f"[FLEET] Licytacja z {name}: komorki {our_cell}/{their_cell}, "
                  f"cele {our_goal}/{their_goal}, koszt ustapienia "
                  f"{self.robot_id}={our_bid}, {name}={their_bid}; "
                  f"jedzie {chosen}.")
            self._fleet_last_bid = bid
        if chosen == self.robot_id:
            self.stop_wheels()
            self.wait_seconds(0.5)
            return "wait"
        if refuge is None:
            self.stop_wheels()
            self.wait_seconds(0.5)
            return "wait"
        self.fleet_refuge = refuge
        self.fleet_refuge_path = route
        self.fleet_peer = name
        return "yield"

    def _yield_to_refuge(self):
        """Retreat on a checked route; the peer keeps the contested corridor."""
        route = self.fleet_refuge_path
        if not route:
            return False
        self.fleet_yielding = True
        try:
            print(f"[FLEET] Ustepuje do {self.fleet_refuge}: {route}")
            arrived = self.follow_path(route)
        finally:
            self.fleet_yielding = False
        if arrived:
            self.fleet_hold = True
            self.fleet_hold_time = 0.0
            self.stop_wheels()
        return arrived

    # ------------------------------------------------------------------
    # Prymitywy ruchu (zamknięta pętla na pozie z Supervisor API)
    # ------------------------------------------------------------------

    def rotate_to(self, target_theta: float) -> bool:
        """Obrót w miejscu do zadanego kursu theta [rad].

        Watchdog: brak zmniejszania bledu kata przez 5 s symulacji ->
        przerwanie (robot fizycznie unieruchomiony / poslizg)."""
        last_abs_error = None
        stall_time = 0.0
        while True:
            _, _, theta = self.get_pose()
            error = normalize_angle(target_theta - theta)
            if abs(error) < ANGLE_TOLERANCE:
                break
            if last_abs_error is not None and abs(error) > last_abs_error - 0.002:
                stall_time += self.timestep / 1000.0
            else:
                stall_time = 0.0
            last_abs_error = abs(error)
            if stall_time > 5.0:
                print(f"[NAV] WATCHDOG rotate_to: theta={theta:.2f} "
                      f"cel={target_theta:.2f} -- przerwanie obrotu")
                self.stop_wheels()
                return False
            # predkosc proporcjonalna do bledu; przy celu bardzo wolno,
            # zeby bezwladnosc nie przenosila robota przez cel
            speed = min(TURN_SPEED, max(0.3, abs(error) * 2.5))
            if abs(error) < 0.35:
                speed = min(speed, 0.6)
            # Dodatni błąd -> obrót dodatni wokół Y: lewe koło w tył, prawe w przód
            if error > 0:
                self.devices["wheel_left"].setVelocity(-speed)
                self.devices["wheel_right"].setVelocity(speed)
            else:
                self.devices["wheel_left"].setVelocity(speed)
                self.devices["wheel_right"].setVelocity(-speed)
            if not self.step():
                self.stop_wheels()
                return False
        self.stop_wheels()
        self.step()
        return True

    def drive_to(self, target_x: float, target_z: float,
                 check_obstacles: bool = True) -> bool:
        """Jazda na wprost do punktu (x, z) z korekcją kursu.

        check_obstacles=False wylacza wykrywanie przeszkod -- uzywane na
        koncowym odcinku do komorki dostepu, bo regal (cel pobrania) zawsze
        jest "przeszkoda" przed robotem. Zwraca False, gdy czujniki wykryja
        przeszkode (przejscie do AVOID_OBSTACLE w stanie NAVIGATE) albo
        watchdog wykryje brak postepu przez 5 s symulacji.
        """
        last_dist = None
        stall_time = 0.0
        telemetry_counter = 0
        sensor_l0 = self.devices.get("wheel_left_sensor")
        sensor_r0 = self.devices.get("wheel_right_sensor")
        sensor_l0 = sensor_l0.getValue() if sensor_l0 else 0.0
        sensor_r0 = sensor_r0.getValue() if sensor_r0 else 0.0
        while True:
            x, z, theta = self.get_pose()
            dx = target_x - x
            dz = target_z - z
            distance = math.hypot(dx, dz)
            telemetry_counter += 1
            if telemetry_counter % 32 == 0:
                sl = self.devices.get("wheel_left_sensor")
                sr = self.devices.get("wheel_right_sensor")
                dsl = (sl.getValue() - sensor_l0) if sl else -1.0
                dsr = (sr.getValue() - sensor_r0) if sr else -1.0
                print(f"[TEL] cel=({target_x:.2f},{target_z:.2f}) "
                      f"poza=({x:.2f},{z:.2f}) theta={theta:.2f} "
                      f"dist={distance:.2f} kolkoL/R+={dsl:.2f}/{dsr:.2f} rad")
            if distance < DIST_TOLERANCE:
                break
            if last_dist is not None and distance > last_dist - 0.002:
                stall_time += self.timestep / 1000.0
            else:
                stall_time = 0.0
            last_dist = distance
            if stall_time > 5.0:
                self.stop_wheels()
                print(f"[NAV] WATCHDOG drive_to: dystans {distance:.2f} m "
                      f"bez postepu, poza=({x:.2f},{z:.2f},{theta:.2f}) -- "
                      "traktuje jak przeszkode")
                return False
            if check_obstacles and not self.fleet_yielding:
                peer = self._fleet_meeting()
                if peer is not None:
                    outcome = self._fleet_auction(peer)
                    if outcome == "yield":
                        self.stop_wheels()
                        return False
                    stall_time = 0.0  # celowo czekamy na przejazd zwyciezcy
                    continue
            if check_obstacles and self.obstacle_detected():
                self.stop_wheels()
                print("[NAV] Wykryto przeszkodę na trasie!")
                return False
            # Azymut celu: przód robota = (-sin(theta), 0, -cos(theta))
            target_angle = math.atan2(-dx, -dz)
            error = normalize_angle(target_angle - theta)
            if abs(error) > 0.6:
                # za duzy blad kursu -- zatrzymaj sie i wyrownaj, zamiast
                # okrazac waypoint po luku
                self.stop_wheels()
                print(f"[NAV] Re-align: theta={theta:.2f} cel={target_angle:.2f}")
                if not self.rotate_to(target_angle):
                    return False
                continue
            left = DRIVE_SPEED - HEADING_GAIN * error
            right = DRIVE_SPEED + HEADING_GAIN * error
            left = min(max(left, -MAX_WHEEL_SPEED), MAX_WHEEL_SPEED)
            right = min(max(right, -MAX_WHEEL_SPEED), MAX_WHEEL_SPEED)
            self.devices["wheel_left"].setVelocity(left)
            self.devices["wheel_right"].setVelocity(right)
            if not self.step():
                break
        self.stop_wheels()
        self.step()
        return True

    def backup(self, distance: float) -> None:
        """Cofa robot o zadaną odległość [m] (po wykryciu przeszkody)."""
        x0, z0, _ = self.get_pose()
        stall_time = 0.0
        while True:
            x, z, _ = self.get_pose()
            stall_time += self.timestep / 1000.0
            if stall_time > 5.0:
                print("[NAV] WATCHDOG backup: robot nie moze sie cofnac")
                break
            if math.hypot(x - x0, z - z0) >= distance:
                break
            if self.devices["ds_back"].getValue() > DS_THRESHOLD:
                print("[NAV] Cofanie zatrzymane: przeszkoda za robotem.")
                break
            self.devices["wheel_left"].setVelocity(-BACKUP_SPEED)
            self.devices["wheel_right"].setVelocity(-BACKUP_SPEED)
            if not self.step():
                break
        self.stop_wheels()
        self.step()

    # ------------------------------------------------------------------
    # Planowanie ścieżki (Navigator + lokalne przeplanowanie z blokadami)
    # ------------------------------------------------------------------

    def compute_path(self, start, goal):
        """Plan around observed boxes and remembered sensor detections."""
        now = self.robot.getTime()
        self.temp_blocked = {cell for cell in self.temp_blocked
                             if self.blocked_until.get(cell, math.inf) > now}
        self.dynamic_blocked = set()
        children = self.robot.getRoot().getField("children")
        for i in range(children.getCount()):
            node = children.getMFNode(i)
            # drugi robot (flota F8): blokuj jego biezaca komorke, zeby
            # nie planowac trasy przez niego (kolizje czołowe w alejach)
            if node.getTypeName() == "WarehouseRobot":
                name_field = node.getField("name")
                other = name_field.getSFString() if name_field else ""
                if other and other != self.robot_id:
                    pos = node.getPosition()
                    if 0 <= pos[1] < 2.1:
                        self.dynamic_blocked.add(
                            self.world_to_cell(pos[0], pos[2]))
                continue
            definition = node.getDef()
            is_obstacle = definition.startswith("OBSTACLE_")
            is_dropped_product = definition.startswith("PRODUCT_") and (
                node.getPosition()[0] >= 1.0 and node.getPosition()[2] >= 5.8)
            if not (is_obstacle or is_dropped_product):
                continue
            box = node.getField("boundingObject").getSFNode()
            if box is None or box.getTypeName() != "Box":
                continue
            size = box.getField("size").getSFVec3f()
            pos, ori = node.getPosition(), node.getOrientation()
            if pos[1] + size[1] / 2 < 0 or pos[1] - size[1] / 2 > 2.1:
                continue
            # Footprint includes the extended fingers and room to turn.
            half_x = sum(abs(ori[j]) * size[j] / 2 for j in range(3)) + 0.60
            half_z = sum(abs(ori[6+j]) * size[j] / 2 for j in range(3)) + 0.60
            for c in range(self.navigator.cols):
                for r in range(self.navigator.rows):
                    x, z = self.navigator.cell_to_world((c, r))
                    if abs(x-pos[0]) < half_x and abs(z-pos[2]) < half_z:
                        self.dynamic_blocked.add((c, r))
        return self._local_astar(start, goal)

    def _local_astar(self, start: tuple[int, int],
                     goal: tuple[int, int]) -> list[tuple[int, int]]:
        """Lokalny algorytm A* (4 kierunki) z blokadami statycznymi i chwilowymi."""
        cols = int(self.layout.get("cols", 16))
        rows = int(self.layout.get("rows", 12))
        blocked = self.static_blocked | self.temp_blocked | self.dynamic_blocked
        if goal in blocked or start in self.static_blocked:
            return []
        blocked.discard(start)

        def heuristic(cell: tuple[int, int]) -> int:
            return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

        open_heap: list[tuple[int, tuple[int, int]]] = [(heuristic(start), start)]
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        g_score: dict[tuple[int, int], float] = {start: 0.0}
        closed: set[tuple[int, int]] = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            if current in closed:
                continue
            closed.add(current)
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = (current[0] + dc, current[1] + dr)
                if not (0 <= neighbor[0] < cols and 0 <= neighbor[1] < rows):
                    continue
                if neighbor in blocked or neighbor in closed:
                    continue
                tentative = g_score[current] + 1.0
                if tentative < g_score.get(neighbor, math.inf):
                    g_score[neighbor] = tentative
                    came_from[neighbor] = current
                    heapq.heappush(open_heap,
                                   (tentative + heuristic(neighbor), neighbor))
        return []

    def follow_path(self, path, goal_is_access=False):
        """Follow cell centers, keeping obstacle detection active on every leg."""
        self.set_lift(LIFT_MIN)
        preview = " -> ".join(f"({c[0]},{c[1]})" for c in path[:6])
        self._update_hud(f"trasa: {preview}")
        cells = path
        if cells:
            center_x, center_z = self.navigator.cell_to_world(cells[0])
            x, z, _ = self.get_pose()
            # Po skanowaniu robot jest w tej samej komorce, lecz przesuniety
            # w stronę produktu. Powrot do jej srodka tworzy zbedne nawroty.
            if (self.world_to_cell(x, z) == cells[0]
                    and math.hypot(center_x-x, center_z-z) < 0.4):
                cells = cells[1:]
        for cell in cells:
            waypoint_x, waypoint_z = self.navigator.cell_to_world(cell)
            x, z, _ = self.get_pose()
            if math.hypot(waypoint_x-x, waypoint_z-z) < DIST_TOLERANCE:
                continue
            if not self.rotate_to(math.atan2(-(waypoint_x-x), -(waypoint_z-z))):
                return False
            if not self.drive_to(waypoint_x, waypoint_z):
                return False
            # F1: korekta dryfu -- robot stoi w znanym punkcie siatki
            if self.localizer is not None and self.localizer.available:
                self.localizer.snap(waypoint_x, waypoint_z)
        return True

    def platform_slot_offset(self, slot: int) -> tuple[float, float, float]:
        """Offset slotu platformy w lokalnym układzie robota: (±0.15, 0.45+0.3k, ±0.15)."""
        offset_x = PLATFORM_SLOT_X if slot % 2 == 0 else -PLATFORM_SLOT_X
        offset_z = PLATFORM_SLOT_Z if (slot // 2) % 2 == 0 else -PLATFORM_SLOT_Z
        offset_y = PLATFORM_SLOT_Y0 + PLATFORM_SLOT_DY * slot
        return offset_x, offset_y, offset_z

    def _update_carried(self) -> None:
        """Aktualizuje pozycje noszonych produktów względem pozy robota."""
        if not self.carried:
            return
        x, z, theta = self.get_pose()
        rotation = self.robot.getSelf().getField("rotation").getSFRotation()
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        for entry in self.carried:
            lx, ly, lz = self.platform_slot_offset(entry["slot"])
            # Rotacja offsetu wokół osi Y o kąt theta (układ robota -> świat)
            world_x = x + lx * cos_t + lz * sin_t
            world_z = z - lx * sin_t + lz * cos_t
            entry["node"].getField("translation").setSFVec3f([world_x, ly, world_z])
            entry["node"].getField("rotation").setSFRotation(rotation)

    # ------------------------------------------------------------------
    # Zmiana stanu FSM z logowaniem
    # ------------------------------------------------------------------

    def _update_hud(self, extra: str = "") -> None:
        """F6: nakladka statusowa w viewporcie (Supervisor.setLabel).

        Label 0: stan/robot/zamowienie/cel/ladunek/kursy (kolor wg stanu).
        Label 1: informacja dodatkowa (np. podglad sciezki), gdy podana.
        """
        order_id = self.current_order.order_id if self.current_order else "-"
        target = self.current_target.product_id if self.current_target else "-"
        color = 0xFFFFFF
        if self.state == State.CHARGING:
            color = 0x55DDFF
        elif self.battery and self.battery.level < self.battery.low:
            color = 0xFFAA44
        elif self.state == State.AVOID_OBSTACLE:
            color = 0xFF8800
        elif self.state in (State.REPORT, State.DONE):
            color = 0x66FF66
        text = (f"[{self.robot_id}] {self.state} | zam: {order_id} | "
                f"cel: {target} | ladunek {self.load_weight:.1f}/50 kg | "
                f"kursy: {self.trips_done}"
                + (f" | bateria {self.battery.level:.0f}%" if self.battery else ""))
        # osobne wiersze labeli per robot (F8): robot1 u gory, robot2 nizej --
        # pionowo, bo dlugie teksty nachodzily na siebie w poziomie
        base_id = 0 if self.robot_id == "WarehouseRobot" else 2
        base_y = 0.02 if self.robot_id == "WarehouseRobot" else 0.19
        try:
            self.robot.setLabel(base_id, text, 0.01, base_y, 0.065,
                                color, 0.0)
            if extra:
                self.robot.setLabel(base_id + 1, extra, 0.01, base_y + 0.085,
                                    0.05, 0xFFFF99, 0.0)
        except Exception:
            pass

    def set_state(self, new_state: str) -> None:
        """Przełącza stan FSM i wypisuje log."""
        if new_state != self.state:
            print(f"[FSM] Stan: {self.state} -> {new_state}")
            self.state = new_state
            self._update_hud()
            if new_state == State.DONE and self.navigator is not None:
                # Kontroler moze zakonczyc petle bez kolejnego step().
                self._announce_fleet_intent()

    # ------------------------------------------------------------------
    # Stany FSM
    # ------------------------------------------------------------------

    def _state_init(self) -> None:
        """INIT: inicjalizacja urządzeń, modułów i danych magazynu."""
        print("[FSM] Inicjalizacja kontrolera magazynu...")

        for key, name in DEVICE_NAMES.items():
            device = self.robot.getDevice(name)
            if device is None:
                raise RuntimeError(f"[FSM] Brak urządzenia '{name}' w robocie!")
            self.devices[key] = device

        self.devices["wheel_left"].setPosition(math.inf)
        self.devices["wheel_right"].setPosition(math.inf)
        self.stop_wheels()

        for name in ("ds_front_left", "ds_front_right", "ds_left",
                     "ds_right", "ds_back"):
            self.devices[name].enable(self.timestep)

        self.devices["camera"].enable(self.timestep)

        # Sensory kol (diagnostyka napedu; dodane w WarehouseRobot.proto)
        for sensor_name in ("wheel_left_sensor", "wheel_right_sensor"):
            sensor = self.robot.getDevice(sensor_name)
            if sensor is not None:
                sensor.enable(self.timestep)
                self.devices[sensor_name] = sensor

        # F1: IMU (+ GPS dostepny, nieuzywany) i estymator odometryczny
        imu = self.robot.getDevice("imu")
        if imu is not None:
            imu.enable(self.timestep)
        gps = self.robot.getDevice("gps")
        if gps is not None:
            gps.enable(self.timestep)
        self.localizer = Localizer(self.devices.get("wheel_left_sensor"),
                                   self.devices.get("wheel_right_sensor"),
                                   imu=imu, wheel_radius=WHEEL_RADIUS,
                                   track_width=TRACK_WIDTH)
        # Kalibracja startowa: jednorazowy odczyt pozy z Supervisor API
        # (odpowiednik znanej pozycji startowej w realnym magazynie).
        if self.localizer.available:
            node = self.robot.getSelf()
            pos0 = node.getField("translation").getSFVec3f()
            ori0 = node.getOrientation()
            self.localizer.start(pos0[0], pos0[2],
                                 normalize_angle(math.atan2(ori0[2], ori0[8])))
            print(f"[F1] Lokalizacja odometryczna aktywna "
                  f"(start: {pos0[0]:.2f}, {pos0[2]:.2f}).")
        else:
            print("[F1] Brak enkoderow kola -- poza z Supervisor API.")

        # F8: identyfikator robota z pola name instancji
        self.robot_id = self.robot.getSelf().getField("name").getSFString()
        # Przy starcie (lub reload swiata) oba roboty startuja razem --
        # blokady zamowien z poprzedniej sesji sa osierocone, czyscimy je.
        # Best effort: na Windows plik moze byc chwilowo zablokowany
        # (OneDrive/inny proces) -- wtedy zostaje; wygasa po CLAIM_STALE_S.
        claims_dir = ORDERS_DIR / ".claims"
        if claims_dir.exists():
            for stale_lock in claims_dir.glob("*.lock"):
                try:
                    stale_lock.unlink(missing_ok=True)
                except OSError:
                    pass

        # F3: Recognition w kamerze (fallback identyfikacji produktu)
        try:
            self.devices["camera"].recognitionEnable(self.timestep)
        except Exception as exc:
            print(f"[QR] Recognition niedostepne w kamerze: {exc}")
        with open(PRODUCTS_PATH, encoding="utf-8") as pf:
            self.products_db = {p["id"]: p for p in json.load(pf)["products"]}

        # Przegub wysuwu ramienia (animacja pobrania)
        self.devices["reach_motor"] = self.robot.getDevice("reach_motor")
        self.devices["reach_motor"].setPosition(0.0)

        self.devices["lift_motor"].setPosition(LIFT_MIN)
        self.set_gripper(False)

        with open(LAYOUT_PATH, encoding="utf-8") as layout_file:
            self.layout = json.load(layout_file)
        self.static_blocked = {tuple(cell)
                               for cell in self.layout.get("blocked_cells", [])}

        self.navigator = Navigator(str(LAYOUT_PATH))
        self.scanner = QRScanner(self.devices["camera"])
        self.order_manager = OrderManager(str(ORDERS_DIR), str(PRODUCTS_PATH),
                                          str(REPORTS_DIR),
                                          robot_id=self.robot_id)

        me = self.robot.getSelf()
        self._battery_fields = (me.getField("batteryLevel"), me.getField("batteryColor"))
        self.battery = Battery(level=self._battery_fields[0].getSFFloat())
        index = 0 if self.robot_id == "WarehouseRobot" else 1
        self.charge_cell = tuple(self.layout.get("charging_cells", [[15, 11], [0, 11]])[index])
        light = self.robot.getFromDef(f"CHARGER_LED_{index + 1}")
        self._charger_light = light
        print(f"[BATERIA] {self.robot_id}: {self.battery.level:.0f}%, stacja {self.charge_cell}.")
        print("[FSM] Inicjalizacja zakończona — robot gotowy do pracy.")
        self.set_state(State.LOAD_ORDER)

    def _state_load_order(self) -> None:
        """LOAD_ORDER: pobranie kolejnego zamówienia z katalogu orders/."""
        order = self.order_manager.next_order()
        if order is None:
            print("[FSM] Brak kolejnych zamówień.")
            self.set_state(State.DONE)
            return
        self.current_order = order
        self.trips_done = 0
        self.trip_queue = []
        self.current_target = None
        print(f"[FSM] Pobrano zamówienie {order.order_id} "
              f"({len(order.items)} pozycji).")
        self.set_state(State.PLAN_TRIP)

    def _state_plan_trip(self) -> None:
        """PLAN_TRIP: podział oczekujących pozycji na kursy (limit 50 kg)."""
        pending = [item for item in self.current_order.items
                   if item.status == "pending"]
        if not pending:
            self.set_state(State.REPORT)
            return

        items_payload = [{
            "id": item.product_id,
            "weight": item.expected_weight,
            "access_cell": tuple(item.access_cell),
        } for item in pending]

        trips = self.navigator.plan_trips(self.current_cell(), items_payload)
        if not trips:
            print("[FSM] Nie udało się zaplanować kursu dla pozostałych pozycji.")
            self.set_state(State.REPORT)
            return

        trip = trips[0]
        by_id = {item.product_id: item for item in pending}
        self.trip_queue = [by_id[entry["id"]] for entry in trip["items"]]
        self.current_target = None
        print(f"[FSM] Zaplanowano kurs: {len(self.trip_queue)} pozycji, "
              f"ładunek {trip['total_weight']:.1f} kg.")
        self.set_state(State.NAVIGATE)

    def _state_navigate(self) -> None:
        """NAVIGATE: dojazd do komórki dostępu kolejnego produktu."""
        if self.current_target is None:
            if not self.trip_queue:
                self.set_state(State.RETURN_TO_TERMINAL)
                return
            next_item = self.trip_queue[0]
            # Weryfikacja limitu wagowego w locie (runtime, oprócz plan_trips)
            if self.load_weight + next_item.expected_weight > PLATFORM_LOAD_LIMIT:
                print(f"[WAGA] Pobranie {next_item.product_id} "
                      f"({next_item.expected_weight} kg) przekroczyłoby limit "
                      f"{PLATFORM_LOAD_LIMIT:.0f} kg (obecnie "
                      f"{self.load_weight:.1f} kg) — powrót do terminala.")
                self.set_state(State.RETURN_TO_TERMINAL)
                return
            self.current_target = self.trip_queue.pop(0)

        item = self.current_target
        goal = tuple(item.access_cell)
        self.current_goal_cell = goal
        self.resume_navigation_state = State.NAVIGATE
        start = self.current_cell()
        print(f"[NAV] Nawigacja do produktu {item.product_id} — "
              f"cel: komórka {goal}.")
        path = self.compute_path(start, goal)
        if not path:
            self._wait_for_path(State.NAVIGATE)
            return
        if self.follow_path(path):
            print(f"[NAV] Dotarto do komórki dostępu {goal}.")
            self.avoid_attempts = 0
            self.set_state(State.ALIGN_AND_SCAN)
        else:
            self.set_state(State.AVOID_OBSTACLE)

    def _state_avoid_obstacle(self):
        """Remember the obstruction before backing up; never erase a blocked goal."""
        self.stop_wheels()
        if self.fleet_refuge is not None:
            reached = self._yield_to_refuge()
            self.fleet_refuge = None
            self.fleet_refuge_path = []
            self._wait_for_path(self.resume_navigation_state)
            if not reached:
                print("[FLEET] Dojazd do mijanki nieudany; czekam na droge.")
            return
        self.avoid_attempts += 1
        if self.avoid_attempts > MAX_AVOID_ATTEMPTS:
            self._wait_for_path(self.resume_navigation_state)
            return
        x, z, theta = self.get_pose()
        cell = self.world_to_cell(x-math.sin(theta)*FRONT_PROBE_DISTANCE,
                                  z-math.cos(theta)*FRONT_PROBE_DISTANCE)
        if cell not in self.static_blocked:
            self.temp_blocked.add(cell)
            self.blocked_until[cell] = self.robot.getTime() + 15.0
        print(f"[NAV] Przeszkoda w {cell}; cofanie i szukanie innej trasy.")
        self.backup(BACKUP_DISTANCE)
        path = self.compute_path(self.current_cell(), self.current_goal_cell)
        if not path:
            self._wait_for_path(self.resume_navigation_state)
            return
        print(f"[NAV] Trasa alternatywna: {path}")
        if self.follow_path(path):
            self.avoid_attempts = 0
            next_state = (State.GO_CHARGE if self.resume_navigation_state == State.GO_CHARGE
                          else State.UNLOAD if self.resume_navigation_state == State.RETURN_TO_TERMINAL
                          else State.ALIGN_AND_SCAN)
            self.set_state(next_state)

    def _wait_for_path(self, resume_state):
        self.stop_wheels()
        self.resume_navigation_state = resume_state
        print(f"[NAV] Cel {self.current_goal_cell} lub dojazd zajety. "
              "Czekam na zwolnienie drogi; silniki zatrzymane.")
        self.set_state(State.WAIT_FOR_PATH)

    def _state_wait_for_path(self):
        """WAIT_FOR_PATH z eskalacja: po WAIT_TIMEOUT_S oczekiwania na cel
        produktu pozycja jest odkladana (kolejne proby w nastepnych kursach);
        po MAX_ITEM_DEFERS odlozeniach pozycja dostaje status niedostepna
        (flaga red). Powroty do terminala nie eskaluja (cel musi sie
        zwolnic)."""
        self.stop_wheels()
        self.wait_seconds(2.0)
        if self.fleet_hold:
            self.fleet_hold_time += 2.0
            peer = self._other_robot()
            if peer is not None:
                pos = peer[1]
                x, z, _ = self.get_pose()
                if (math.hypot(pos[0]-x, pos[2]-z) < FLEET_CLEAR_DISTANCE
                        and self.fleet_hold_time < WAIT_TIMEOUT_S):
                    return
            self.fleet_hold = False
        self.wait_time += 2.0
        if self.compute_path(self.current_cell(), self.current_goal_cell):
            print("[NAV] Droga ponownie dostepna; wznawiam jazde.")
            self.wait_time = 0.0
            self.set_state(self.resume_navigation_state)
            return
        if self.wait_time >= WAIT_TIMEOUT_S and \
                self.resume_navigation_state == State.RETURN_TO_TERMINAL:
            # Terminal zajety: najpierw druga komorka, potem zatrzymanie z ladunkiem
            if not self.terminal_alternate_tried:
                alt = self._alternate_terminal_cell()
                if alt is not None:
                    print(f"[NAV] Komórka {self.current_goal_cell} zajęta — "
                          f"próbuję komórkę {alt}.")
                    self.current_goal_cell = alt
                    self.terminal_alternate_tried = True
                    self.wait_time = 0.0
                    self.set_state(State.RETURN_TO_TERMINAL)
                    return
            print("[NAV] Terminal niedostępny — zatrzymuję robota z ładunkiem.")
            self._emergency_unload()
            return
        if self.wait_time >= WAIT_TIMEOUT_S and                 self.resume_navigation_state == State.NAVIGATE and                 self.current_target is not None:
            item = self.current_target
            count = self.defer_counts.get(item.product_id, 0) + 1
            self.defer_counts[item.product_id] = count
            self.wait_time = 0.0
            if count >= MAX_ITEM_DEFERS:
                print(f"[NAV] Cel {self.current_goal_cell} trwale zajety — "
                      f"pozycja {item.product_id} oznaczona jako niedostepna.")
                self.order_manager.mark_unreachable(item)
            else:
                print(f"[NAV] Cel {self.current_goal_cell} zajety — pozycja "
                      f"{item.product_id} odlozona (proba "
                      f"{count}/{MAX_ITEM_DEFERS}).")
            self.current_target = None
            self.set_state(State.NAVIGATE)

    def _state_align_and_scan(self) -> None:
        """ALIGN_AND_SCAN: podnosnik na poziom, orientacja na regał, skan QR."""
        item = self.current_target
        # Lower before rotating; line up with the actual shelf slot, not cell center.
        self.set_lift(LIFT_MIN)
        target_x = item.position[0]
        _, access_z = self.navigator.cell_to_world(tuple(item.access_cell))
        x, z, _ = self.get_pose()
        if math.hypot(target_x-x, access_z-z) > DIST_TOLERANCE:
            if (not self.rotate_to(math.atan2(-(target_x-x), -(access_z-z)))
                    or not self.drive_to(target_x, access_z)):
                self.set_state(State.AVOID_OBSTACLE)
                return
        target_theta = math.pi if int(item.shelf_cell[1]) in SHELF_ROWS_SOUTH_VIEW else 0.0
        if not self.rotate_to(target_theta):
            self.set_state(State.AVOID_OBSTACLE)
            return
        lift_position = item.position[1] - CAMERA_MOUNT_HEIGHT
        print(f"[PODNOSNIK] Skan {item.product_id}: x={target_x:.2f}, lift={lift_position:.2f}")
        self.set_lift(lift_position)

        self.scan_payload = None
        for attempt in range(1, SCAN_TRIES + 1):
            self.wait_seconds(SCAN_SETTLE_TIME)
            self.devices["camera"].saveImage(str(ROOT_DIR / "logs" / f"scan_{item.product_id}_{attempt}.png"), 100)
            self.scan_payload = self.scanner.scan()
            if self.scan_payload:
                print(f"[QR] Odczytano kod (próba {attempt}): "
                      f"'{self.scan_payload}'.")
                break
            print(f"[QR] Nie odczytano kodu (próba {attempt}/{SCAN_TRIES}).")

        self.set_state(State.VERIFY_ITEM)

    def _state_verify_item(self) -> None:
        """VERIFY_ITEM: weryfikacja id i wagi produktu z zamówieniem."""
        item = self.current_target
        node = self.robot.getFromDef(f"PRODUCT_{item.product_id}")

        if node is None:
            print(f"[QR] Brak produktu na półce (brak DEF "
                  f"PRODUCT_{item.product_id}).")
            self.order_manager.mark_missing(item)
            self.current_target = None
            self._continue_after_item()
            return

        found_id = None
        found_weight = None
        if self.scan_payload:
            try:
                found_id, found_weight = parse_payload(self.scan_payload)
            except ValueError:
                print(f"[QR] Nieprawidłowy format payloadu: "
                      f"'{self.scan_payload}'.")

        if found_id is None:
            # F3: fallback -- identyfikacja przez Recognition kamery
            fallback = recognition_fallback(self.devices["camera"],
                                            item.position, self.products_db)
            if fallback is not None:
                found_id, found_weight = fallback
                print(f"[QR] Fallback Recognition: zidentyfikowano "
                      f"{found_id} (waga katalogowa {found_weight} kg).")

        if found_id is None:
            print("[QR] Kod QR nieodczytany — produkt oznaczony jako "
                  "niezgodny.")
            self.order_manager.mark_wrong(item, "NIEODCZYTANY", None)
            self.current_target = None
            self._continue_after_item()
            return

        if found_id != item.product_id:
            print(f"[QR] Zły produkt na półce: znaleziono {found_id}, "
                  f"oczekiwano {item.product_id}.")
            self.order_manager.mark_wrong(item, found_id, found_weight)
            self.current_target = None
            self._continue_after_item()
            return

        if found_weight is None or \
                abs(found_weight - item.expected_weight) > WEIGHT_TOLERANCE:
            print(f"[WAGA] Niezgodna waga {item.product_id}: "
                  f"oczekiwano {item.expected_weight} kg, "
                  f"jest {found_weight} kg.")
            self.order_manager.mark_wrong(item, found_id, found_weight)
            self.current_target = None
            self._continue_after_item()
            return

        print(f"[QR] Produkt {item.product_id} zweryfikowany poprawnie "
              f"({found_weight} kg).")
        self.verify_node = node
        self.set_state(State.PICK_ITEM)

    def _state_pick_item(self) -> None:
        """PICK_ITEM: chwytak, przypięcie produktu do platformy, zwolnienie."""
        item = self.current_target
        node = self.verify_node

        print(f"[CHWYTAK] Zamykam chwytak na produkcie {item.product_id}.")

        # Extend only the telescopic fingers, retract the parcel, lower, stow.
        self.stop_wheels()
        start_pos = list(node.getPosition())
        translation = node.getField("translation")
        reach = self.devices.get("reach_motor")
        lift_start = self._lift_pos
        lift_clear = min(LIFT_MAX, lift_start + 0.06)
        clearance = lift_clear - lift_start
        # Heavier parcels move more deliberately (weight is metadata).
        pace = 1.0 + 0.3 * min(max(item.expected_weight, 0.0) / 20.0, 1.0)

        def animate(duration, update, loaded=True):
            steps = max(1, math.ceil(duration * (pace if loaded else 1.0) * 1000 / self.timestep))
            for k in range(1, steps + 1):
                t = k / steps
                # Quintic easing: zero velocity and acceleration at both ends.
                s = t * t * t * (10.0 + t * (-15.0 + 6.0 * t))
                update(s)
                if not self.step():
                    return False
            return True

        robot_node = self.robot.getSelf()
        rail_translation = robot_node.getField("telescopeTranslation")
        rail_scale = robot_node.getField("telescopeScale")
        if rail_translation is None or rail_scale is None:
            raise RuntimeError("Reload world: telescopic gripper PROTO required")
        # Measure from the actual chassis, not wheel odometry after a contact.
        body = robot_node.getPosition()
        orientation = robot_node.getOrientation()
        forward_x, forward_z = -orientation[2], -orientation[8]
        distance = ((start_pos[0] - body[0]) * forward_x
                    + (start_pos[2] - body[2]) * forward_z)
        extension = max(0.0, distance - 0.48)
        if extension > 0.85:
            print("[CHWYTAK] Paczka poza zasiegiem lap; ponowne ustawienie.")
            self.set_state(State.ALIGN_AND_SCAN)
            return
        # Match the parcel centre even on the lowest shelf (lift is clamped).
        vertical_offset = start_pos[1] - (body[1] + 0.25 + lift_start)

        def set_extension(value):
            rail_translation.setSFVec3f([0.0, vertical_offset, -value / 2.0])
            rail_scale.setSFVec3f([1.0, 1.0, (0.30 + value) / 0.30])

        # The old reach joint moved the entire lift into the shelf.
        if reach is not None:
            reach.setPosition(0.0)
        self.set_gripper(False)
        if not animate(1.3, lambda s: set_extension(extension * s), loaded=False):
            return

        # Take ownership before closing so contact cannot push the chassis.
        saved_fields = {}
        for field_name in ("physics", "boundingObject"):
            field = node.getField(field_name)
            value = field.getSFNode()
            if value is not None:
                saved_fields[field_name] = value.exportString()
                field.removeSF()
        self.set_gripper(True)

        def lift_box(s):
            self.devices["lift_motor"].setPosition(lift_start + clearance * s)
            translation.setSFVec3f(
                [start_pos[0], start_pos[1] + clearance * s, start_pos[2]])

        if not animate(0.55, lift_box):
            return
        self._lift_pos = lift_clear
        lifted = [start_pos[0], start_pos[1] + clearance, start_pos[2]]
        retracted = [lifted[0] - extension * forward_x, lifted[1],
                     lifted[2] - extension * forward_z]

        def retract(s):
            set_extension(extension * (1.0 - s))
            translation.setSFVec3f(
                [a + (b - a) * s for a, b in zip(lifted, retracted)])

        if not animate(1.5, retract):
            return
        # Descend only after the parcel and both fingers clear the shelf.
        # Stop at a low carrying height so the parcel stays above the floor.
        lower_lift = min(lift_clear, max(LIFT_MIN, 0.20 - vertical_offset))
        drop = lift_clear - lower_lift

        def lower(s):
            self.devices["lift_motor"].setPosition(lift_clear - drop * s)
            translation.setSFVec3f(
                [retracted[0], retracted[1] - drop * s, retracted[2]])

        if not animate(max(0.65, drop / 0.3), lower):
            return
        self._lift_pos = lower_lift
        # Let the lowered parcel settle visibly before the rear handoff.
        if not animate(0.7, lambda s: None, loaded=False):
            return
        self.set_gripper(False)
        # Intentional handoff to the rear platform after lowering.
        rail_translation.setSFVec3f([0.0, 0.0, 0.0])

        slot = len(self.carried)
        self.carried.append({"item": item, "node": node, "slot": slot,
                             "saved_fields": saved_fields})
        self.load_weight += item.expected_weight
        self._update_carried()
        self.order_manager.mark_ok(item)
        print(f"[FSM] Produkt {item.product_id} umieszczony na platformie "
              f"(slot {slot}, ładunek {self.load_weight:.1f}/"
              f"{PLATFORM_LOAD_LIMIT:.0f} kg).")

        self.set_gripper(False)
        self.set_lift(LIFT_MIN)

        self.current_target = None
        self._continue_after_item()

    def _continue_after_item(self) -> None:
        """Wybiera kolejny krok po obsłużeniu pozycji zamówienia."""
        self.set_lift(LIFT_MIN)
        if self.trip_queue:
            self.set_state(State.NAVIGATE)
        else:
            self.set_state(State.RETURN_TO_TERMINAL)

    def _state_return_to_terminal(self) -> None:
        """RETURN_TO_TERMINAL: kazdy robot ma wlasne miejsce rozladunku."""
        terminal_cells = [tuple(c) for c in
                          self.layout.get("terminal_cells", [[7, 11], [8, 11]])]
        start = self.current_cell()
        goal = terminal_cells[0] if self.robot_id == "WarehouseRobot" else terminal_cells[1]
        if self.terminal_alternate_tried:
            others = [c for c in terminal_cells if c != goal]
            if others:
                goal = others[0]
        self.current_goal_cell = goal
        print(f"[NAV] Powrót do terminala — cel: komórka {goal}.")
        path = self.compute_path(start, goal)
        self.resume_navigation_state = State.RETURN_TO_TERMINAL
        if not path:
            self._wait_for_path(State.RETURN_TO_TERMINAL)
            return
        if self.follow_path(path):
            print("[NAV] Robot w strefie terminala.")
            self.set_state(State.UNLOAD)
        else:
            self.set_state(State.AVOID_OBSTACLE)

    def _alternate_terminal_cell(self):
        """Use the other bay only when its owner has vacated the terminal."""
        terminal_cells = [tuple(c) for c in
                          self.layout.get("terminal_cells", [[7, 11], [8, 11]])]
        peer = self._other_robot()
        for cell in terminal_cells:
            if cell == self.current_goal_cell:
                continue
            if peer is not None:
                pos = peer[1]
                x, z = self.navigator.cell_to_world(cell)
                if math.hypot(pos[0]-x, pos[2]-z) < 1.2:
                    continue
                if tuple(peer[2].get("goal") or ()) == cell:
                    continue
            if self.compute_path(self.current_cell(), cell):
                return cell
        return None

    def _emergency_unload(self) -> None:
        """Stop with cargo on board when every terminal route remains blocked."""
        print("[FSM] AWARIA: terminal niedostepny; ladunek pozostaje na robocie.")
        self.unload_failed = True
        for entry in self.carried:
            self.order_manager.mark_unreachable(
                entry["item"], "terminal zablokowany — ladunek na robocie")
        if self.current_order is not None:
            for item in self.current_order.items:
                if item.status == "pending":
                    self.order_manager.mark_unreachable(
                        item, "terminal zablokowany — ladunek na robocie")
        self.set_state(State.REPORT)

    def _state_unload(self) -> None:
        """UNLOAD: zrzut noszonych produktów w strefie terminala."""
        self.terminal_alternate_tried = False
        if self.carried:
            print(f"[FSM] Rozładunek {len(self.carried)} produktów "
                  "w strefie terminala.")
        for entry in self.carried:
            n = self.drop_index
            self.drop_index += 1
            lane_x = DROP_GRID_X0 + (0.0 if self.robot_id == "WarehouseRobot"
                                     else DROP_GRID_DX * DROP_GRID_COLS + 0.6)
            drop_x = lane_x + DROP_GRID_DX * (n % DROP_GRID_COLS)
            drop_z = DROP_GRID_Z0 + DROP_GRID_DZ * (n // DROP_GRID_COLS)
            node = entry["node"]
            node.getField("translation").setSFVec3f(
                [drop_x, DROP_ZONE_Y, drop_z])
            node.getField("rotation").setSFRotation([0, 1, 0, 0])
            for field_name in ("boundingObject", "physics"):
                definition = entry.get("saved_fields", {}).get(field_name)
                if definition:
                    node.getField(field_name).importSFNodeFromString(definition)
            node.resetPhysics()
            print(f"[FSM] Produkt {entry['item'].product_id} odłożony "
                  f"w punkcie ({drop_x:.2f}, {drop_z:.2f}).")
        self.carried.clear()
        self.load_weight = 0.0
        self.trips_done += 1

        pending = [item for item in self.current_order.items
                   if item.status == "pending"]
        if pending:
            print(f"[FSM] Pozostało {len(pending)} pozycji — "
                  "planowanie kolejnego kursu.")
            self.set_state(State.PLAN_TRIP)
        else:
            self.set_state(State.REPORT)

    def _state_report(self) -> None:
        """REPORT: zbudowanie i zapis raportu JSON dla zamówienia."""
        trips = max(1, self.trips_done)
        report = self.order_manager.build_report(self.current_order, trips)
        path = self.order_manager.save_report(report)
        print(f"[RAPORT] Zamówienie {report['order_id']} zakończone — "
              f"flaga: {report['flag']}, kursy: {report['trips']}.")
        print(f"[RAPORT] Raport zapisany: {path}")
        self.current_order = None
        self.set_state(State.DONE if self.unload_failed else State.LOAD_ORDER)

    def _dock_contact(self):
        if self.charge_cell is None:
            return False
        x, z = self.navigator.cell_to_world(self.charge_cell)
        pos = self.robot.getSelf().getPosition()
        ori = self.robot.getSelf().getOrientation()
        heading = math.atan2(ori[2], ori[8])
        return (math.hypot(pos[0] - x, pos[2] - z) < 0.20
                and abs(normalize_angle(heading - math.pi)) < 0.12)

    def _update_energy(self):
        if self.battery is None:
            return
        level_field, color_field = self._battery_fields
        # Public PROTO field also permits deliberate low-battery demo scenarios.
        self.battery.level = max(0.0, min(100.0, level_field.getSFFloat()))
        speed = sum(abs(self.devices[k].getVelocity())
                    for k in ("wheel_left", "wheel_right")) / 2.0
        docked = self.state == State.CHARGING and speed < 0.01 and self._dock_contact()
        handling = self.state in (State.ALIGN_AND_SCAN, State.PICK_ITEM, State.UNLOAD)
        self.battery.update(self.timestep / 1000.0, speed, self.load_weight,
                            handling, docked)
        level_field.setSFFloat(self.battery.level)
        now = self.robot.getTime()
        if now >= self._battery_hud_at:
            color = ([0.05, 0.7, 1.0] if docked else
                     [1.0, 0.15, 0.05] if self.battery.level < self.battery.low else
                     [0.2, 0.9, 0.3])
            color_field.setSFColor(color)
            if self._charger_light:
                light_color = [0.05, 0.7, 1.0] if docked else [0.1, 0.9, 0.4]
                self._charger_light.getField("baseColor").setSFColor(light_color)
                self._charger_light.getField("emissiveColor").setSFColor(light_color)
            self._update_hud()
            self._battery_hud_at = now + 0.5
        if self.battery.level <= 0.0 and self.state != State.BATTERY_EMPTY and not docked:
            self.stop_wheels()
            for name in ("lift_motor", "reach_motor", "gripper_left", "gripper_right"):
                self.devices[name].setVelocity(0.0)
            raise BatteryEmpty("Brak energii — robot zatrzymany")

    def _request_charge(self, resume):
        self.charge_resume_state = resume
        self.stop_wheels()
        self.set_lift(LIFT_MIN)
        self.current_goal_cell = self.charge_cell
        self.resume_navigation_state = State.GO_CHARGE
        self.avoid_attempts = 0
        print(f"[BATERIA] {self.robot_id}: {self.battery.level:.1f}% — jade do ladowarki.")
        self.set_state(State.GO_CHARGE)

    def _maybe_charge(self):
        if self.battery is None:
            return
        resume = self.state
        if resume == State.WAIT_FOR_PATH:
            resume = self.resume_navigation_state
        if resume not in (State.NAVIGATE, State.RETURN_TO_TERMINAL,
                          State.LOAD_ORDER, State.PLAN_TRIP):
            return
        goal = self.current_cell()
        if resume == State.NAVIGATE:
            item = self.current_target or (self.trip_queue[0] if self.trip_queue else None)
            if item is not None:
                goal = tuple(item.access_cell)
        elif resume == State.RETURN_TO_TERMINAL:
            index = 0 if self.robot_id == "WarehouseRobot" else 1
            goal = tuple(self.layout["terminal_cells"][index])
        legs = [(self.current_cell(), goal), (goal, self.charge_cell)]
        distance = 0
        for start, end in legs:
            route = self.compute_path(start, end)
            distance += (len(route) - 1 if route else
                         2 * (abs(start[0] - end[0]) + abs(start[1] - end[1])))
        if self.battery.needs_charge(distance):
            self._request_charge(resume)

    def _state_go_charge(self):
        self.current_goal_cell = self.charge_cell
        self.resume_navigation_state = State.GO_CHARGE
        path = self.compute_path(self.current_cell(), self.charge_cell)
        if not path:
            self._wait_for_path(State.GO_CHARGE)
            return
        if not self.follow_path(path):
            self.set_state(State.AVOID_OBSTACLE)
            return
        dock_x, dock_z = self.navigator.cell_to_world(self.charge_cell)
        x, z, _ = self.get_pose()
        if math.hypot(dock_x - x, dock_z - z) > DIST_TOLERANCE:
            if (not self.rotate_to(math.atan2(-(dock_x - x), -(dock_z - z)))
                    or not self.drive_to(dock_x, dock_z)):
                self._wait_for_path(State.GO_CHARGE)
                return
        if not self.rotate_to(math.pi):
            self._wait_for_path(State.GO_CHARGE)
            return
        self.stop_wheels()
        if not self._dock_contact():
            # Correct odometry at the marked dock, then approach it again.
            pos = self.robot.getSelf().getPosition()
            if self.localizer and self.localizer.available:
                self.localizer.snap(pos[0], pos[2])
            self._wait_for_path(State.GO_CHARGE)
            return
        print(f"[BATERIA] {self.robot_id}: ladowanie od {self.battery.level:.1f}%.")
        self.set_state(State.CHARGING)

    def _state_charging(self):
        self.stop_wheels()
        if not self._dock_contact():
            self.set_state(State.GO_CHARGE)
            return
        if self.battery.level < self.battery.target:
            self.wait_seconds(0.25)
            return
        self.charge_cycles += 1
        print(f"[BATERIA] {self.robot_id}: {self.battery.level:.1f}%, "
              f"wznawiam {self.charge_resume_state} (cykl {self.charge_cycles}).")
        self.current_goal_cell = None
        self.wait_time = 0.0
        self.avoid_attempts = 0
        self.set_state(self.charge_resume_state)

    def _state_battery_empty(self):
        self.stop_wheels()
        self.wait_seconds(1.0)

    def _state_done(self):
        # Park and recharge in the robot's own bay, outside shared aisles.
        if not self._dock_contact() or self.battery.level < self.battery.target - 0.5:
            self._request_charge(State.DONE)
            return
        self.stop_wheels()
        print(f"[FSM] {self.robot_id}: koniec pracy, zaparkowany przy ladowarce.")
        self.finished = True

    # ------------------------------------------------------------------
    # Pętla główna
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Główna pętla maszyny stanów — wywoływana do końca symulacji."""
        self.finished = False
        handlers = {
            State.INIT: self._state_init,
            State.LOAD_ORDER: self._state_load_order,
            State.PLAN_TRIP: self._state_plan_trip,
            State.NAVIGATE: self._state_navigate,
            State.AVOID_OBSTACLE: self._state_avoid_obstacle,
            State.WAIT_FOR_PATH: self._state_wait_for_path,
            State.ALIGN_AND_SCAN: self._state_align_and_scan,
            State.VERIFY_ITEM: self._state_verify_item,
            State.PICK_ITEM: self._state_pick_item,
            State.RETURN_TO_TERMINAL: self._state_return_to_terminal,
            State.UNLOAD: self._state_unload,
            State.REPORT: self._state_report,
            State.GO_CHARGE: self._state_go_charge,
            State.CHARGING: self._state_charging,
            State.BATTERY_EMPTY: self._state_battery_empty,
            State.DONE: self._state_done,
        }
        import traceback
        while True:
            try:
                if not self.step():
                    break
                self._maybe_charge()
                handlers[self.state]()
            except BatteryEmpty:
                print(f"[BATERIA] {self.robot_id}: 0% — awaryjne zatrzymanie.")
                self.set_state(State.BATTERY_EMPTY)
            except Exception:
                print("[FSM] WYJATEK w handlerze stanu "
                      f"{self.state}:\n" + traceback.format_exc())
                self.stop_wheels()
                raise
            if self.finished:
                break
        self.stop_wheels()


if __name__ == "__main__":
    import sys as _sys
    _LOGS = Path(__file__).resolve().parents[2] / "logs"
    _LOGS.mkdir(parents=True, exist_ok=True)
    _fh = open(_LOGS / "fsm_debug.log", "a", buffering=1, encoding="utf-8")
    _sys.stdout = _fh
    _sys.stderr = _fh
    print("\n===== START KONTROLERA =====")
    WarehouseController().run()
