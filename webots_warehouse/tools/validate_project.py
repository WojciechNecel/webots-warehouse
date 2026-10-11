# -*- coding: utf-8 -*-
"""
================================================================================
validate_project.py — statyczny walidator spójności projektu magazynu Webots
================================================================================

PRZEZNACZENIE
-------------
Narzędzie weryfikuje kompletność i wzajemną zgodność wszystkich artefaktów
projektu symulacyjnego systemu obsługi zamówień magazynowych (Webots R2023b)
z KONTRAKTEM INTEGRACYJNYM. Walidator jest czystym skryptem Pythona —
NIE wymaga środowiska Webots ani uruchomionej symulacji. Uruchamia się go
PO zakończeniu prac nad projektem (przed oddaniem), jako ostatni etap
kontroli jakości (Etap V specyfikacji).

SPOSÓB UŻYCIA
-------------
    python tools/validate_project.py            # z katalogu webots_warehouse/
    python webots_warehouse/tools/validate_project.py   # z dowolnego miejsca

Korzeń projektu jest wyznaczany automatycznie jako katalog nadrzędny
katalogu tools/ (czyli parents[1] pliku), więc skrypt działa niezależnie
od bieżącego katalogu roboczego.

Kod wyjścia:
    0 — PASS (wszystkie kontrole zaliczone),
    1 — FAIL (co najmniej jedna kontrola niezaliczona lub brak plików).

Raport ma postać czytelnej listy kontroli ze statusem PASS/FAIL oraz
szczegółami błędów (po polsku). Skrypt NIGDY nie kończy się wyjątkiem —
brakujące lub uszkodzone pliki są raportowane jako FAIL.

STRUKTURA KONTROLI
------------------
Kontrole są zorganizowane jako lista funkcji o sygnaturze
    check() -> CheckResult
gdzie CheckResult to dataclass:
    name    : str        — nazwa kontroli,
    ok      : bool       — wynik zbiorczy,
    details : list[str]  — komunikaty (błędy/opisy) po polsku.

Realizowane grupy kontroli:
    1. Istnienie wszystkich plików kontraktowych
       (świat, proto, kontrolery, dane, zamówienia, tekstury QR, dokumentacja).
    2. Zgodność data/products.json z tabelą kontraktu: 20 produktów,
       pozycje wg wzoru (x = -7.5 + c + slot, z = -5.5 + r, y = levels[level]),
       payload QR "<id>;<waga>", access_cell wg reguły dostępu,
       P005 present=false, P020 world_qr="P999;6.0".
    3. data/warehouse_layout.json: 60 blocked_cells (rzędy {2,3,5,6,8,9}
       x kolumny {2,3,4,5,6,9,10,11,12,13}), parametry siatki 16x12,
       poziomy/sloty/komórki terminala/limity wagowe.
    4. Zamówienia orders/*.json: ORD-001 (4 pozycje, 13.0 kg),
       ORD-002 (7 pozycji, 85.0 kg), ORD-003 (zawiera P005 i P020).
    5. worlds/warehouse.wbt: balans nawiasów, DEF PRODUCT_<id> dla wszystkich
       produktów present=true, brak PRODUCT_P005, tekstura P999.png wewnątrz
       bloku PRODUCT_P020, DEF TERMINAL, OBSTACLE_1..3, kontroler
       "warehouse_controller", referencja EXTERNPROTO do proto.
    6. protos/WarehouseRobot.proto: balans nawiasów, nazwy urządzeń
       (wheel_left, wheel_right, lift_motor, gripper_left, gripper_right,
       camera, ds_front_left, ds_front_right, ds_left, ds_right, ds_back),
       pole supervisor TRUE.
    7. Kompilacja składniowa (py_compile) wszystkich plików .py projektu.
    8. Roundtrip QR: dekodowanie każdego textures/qr/*.png przez
       cv2.QRCodeDetector i zgodność payloadu z tabelą kontraktu
       (P001..P020 -> "<id>;<waga>", P999.png -> "P999;6.0").
       Wymaga opencv-python i numpy (zgodnie ze stosem projektu).
    9. Import i kontrola API modułów navigation / qr_scanner / order_manager
       (bez webots): obecność klas Navigator, QRScanner, Order, OrderItem,
       OrderManager, funkcji parse_payload oraz metod sygnaturowych
       z kontraktu (kontrolowana jest obecność nazw parametrów).

STRUKTURY DANYCH
----------------
PRODUCT_TABLE : dict[str, dict]
    Sztywna tabela kontraktowa 20 produktów. Klucz: id ("P001".."P020").
    Wartości: cell [c,r], level (0..2), slot (-0.25/+0.25), weight [kg],
    present (bool), world_qr (str|None). Źródło prawdy dla kontroli 2, 4, 5, 8.
EXPECTED_LAYOUT : dict
    Oczekiwane parametry siatki magazynu (kontrola 3).
CONTRACT_FILES : list[str]
    Względne ścieżki wszystkich plików kontraktowych (kontrola 1).
================================================================================
"""

from __future__ import annotations

import importlib
import inspect
import json
import py_compile
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

# Korzeń projektu: webots_warehouse/ (katalog nadrzędny tools/)
PROJECT_ROOT = Path(__file__).resolve().parents[1]

# ---------------------------------------------------------------------------
# Sztywna tabela kontraktowa produktów (źródło prawdy dla walidacji)
# ---------------------------------------------------------------------------

LEVELS = [0.2, 0.8, 1.4]           # wysokości środków produktów na regale [m]
SHELF_ROWS_NORTH = {2, 5, 8}       # dostęp od północy: robot staje w (c, r-1)
SHELF_ROWS_SOUTH = {3, 6, 9}       # dostęp od południa: robot staje w (c, r+1)

PRODUCT_TABLE: dict[str, dict] = {
    "P001": {"cell": [2, 2],  "level": 0, "slot": -0.25, "weight": 3.5,  "present": True,  "world_qr": None},
    "P002": {"cell": [4, 2],  "level": 1, "slot": 0.25,  "weight": 7.0,  "present": True,  "world_qr": None},
    "P003": {"cell": [6, 2],  "level": 2, "slot": -0.25, "weight": 12.0, "present": True,  "world_qr": None},
    "P004": {"cell": [9, 2],  "level": 1, "slot": 0.25,  "weight": 5.0,  "present": True,  "world_qr": None},
    "P005": {"cell": [11, 2], "level": 0, "slot": -0.25, "weight": 18.0, "present": False, "world_qr": None},
    "P006": {"cell": [13, 2], "level": 2, "slot": 0.25,  "weight": 9.5,  "present": True,  "world_qr": None},
    "P007": {"cell": [2, 3],  "level": 1, "slot": -0.25, "weight": 4.0,  "present": True,  "world_qr": None},
    "P008": {"cell": [5, 3],  "level": 0, "slot": 0.25,  "weight": 15.0, "present": True,  "world_qr": None},
    "P009": {"cell": [10, 3], "level": 2, "slot": -0.25, "weight": 6.5,  "present": True,  "world_qr": None},
    "P010": {"cell": [13, 3], "level": 1, "slot": 0.25,  "weight": 11.0, "present": True,  "world_qr": None},
    "P011": {"cell": [2, 5],  "level": 2, "slot": -0.25, "weight": 2.5,  "present": True,  "world_qr": None},
    "P012": {"cell": [6, 5],  "level": 0, "slot": 0.25,  "weight": 8.0,  "present": True,  "world_qr": None},
    "P013": {"cell": [9, 5],  "level": 1, "slot": -0.25, "weight": 14.0, "present": True,  "world_qr": None},
    "P014": {"cell": [12, 5], "level": 0, "slot": 0.25,  "weight": 19.0, "present": True,  "world_qr": None},
    "P015": {"cell": [3, 6],  "level": 1, "slot": -0.25, "weight": 3.0,  "present": True,  "world_qr": None},
    "P016": {"cell": [11, 6], "level": 2, "slot": 0.25,  "weight": 10.5, "present": True,  "world_qr": None},
    "P017": {"cell": [5, 8],  "level": 0, "slot": -0.25, "weight": 5.5,  "present": True,  "world_qr": None},
    "P018": {"cell": [9, 8],  "level": 2, "slot": 0.25,  "weight": 13.0, "present": True,  "world_qr": None},
    "P019": {"cell": [2, 9],  "level": 1, "slot": -0.25, "weight": 7.5,  "present": True,  "world_qr": None},
    "P020": {"cell": [13, 9], "level": 0, "slot": -0.25, "weight": 16.0, "present": True,  "world_qr": "P999;6.0"},
}

# Oczekiwane zamówienia: order_id -> lista id produktów (wg kontraktu)
EXPECTED_ORDERS: dict[str, list[str]] = {
    "ORD-001": ["P001", "P007", "P011", "P015"],
    "ORD-002": ["P003", "P006", "P008", "P010", "P013", "P016", "P018"],
    "ORD-003": ["P002", "P005", "P020", "P019"],
}

# Nazwy urządzeń robota (kontrakt proto)
DEVICE_NAMES = [
    "wheel_left", "wheel_right", "lift_motor",
    "gripper_left", "gripper_right", "camera",
    "ds_front_left", "ds_front_right", "ds_left", "ds_right", "ds_back",
]

# Pełna lista plików kontraktowych (względem korzenia projektu)
CONTRACT_FILES: list[str] = (
    [
        "README.md",
        "docs/architecture.md",
        "docs/test_scenarios.md",
        "worlds/warehouse.wbt",
        "protos/WarehouseRobot.proto",
        "data/warehouse_layout.json",
        "data/products.json",
        "orders/order_01.json",
        "orders/order_02.json",
        "orders/order_03.json",
        "tools/generate_assets.py",
        "tools/validate_project.py",
        "controllers/warehouse_controller/warehouse_controller.py",
        "controllers/warehouse_controller/navigation.py",
        "controllers/warehouse_controller/qr_scanner.py",
        "controllers/warehouse_controller/order_manager.py",
    ]
    + [f"textures/qr/P{i:03d}.png" for i in range(1, 21)]
    + ["textures/qr/P999.png"]
)

EPS = 1e-6  # tolerancja porównań zmiennoprzecinkowych


@dataclass
class CheckResult:
    """Wynik pojedynczej kontroli: nazwa, status zbiorczy, lista komunikatów."""
    name: str
    ok: bool = True
    details: list[str] = field(default_factory=list)

    def fail(self, message: str) -> None:
        """Rejestruje błąd i ustawia status kontroli na FAIL."""
        self.ok = False
        self.details.append(f"  [BŁĄD] {message}")

    def note(self, message: str) -> None:
        """Rejestruje komunikat informacyjny (nie wpływa na status)."""
        self.details.append(f"  [INFO] {message}")


# ---------------------------------------------------------------------------
# Funkcje pomocnicze
# ---------------------------------------------------------------------------

def access_cell_for(cell: list[int]) -> list[int]:
    """Zwraca komórkę dostępu (access cell) wg reguły kontraktu.

    r in {2,5,8} -> (c, r-1);  r in {3,6,9} -> (c, r+1).
    """
    c, r = cell
    if r in SHELF_ROWS_NORTH:
        return [c, r - 1]
    if r in SHELF_ROWS_SOUTH:
        return [c, r + 1]
    return []  # nieznany rząd — sygnalizowane pustą listą


def expected_position(entry: dict) -> list[float]:
    """Pozycja świata produktu: x = -7.5 + c + slot, z = -5.5 + r, y = levels[level]."""
    c, r = entry["cell"]
    return [-7.5 + c + entry["slot"], LEVELS[entry["level"]], -5.5 + r]


def qr_payload_for(product_id: str) -> str:
    """Oczekiwany payload QR dla danego id (z tabeli kontraktu)."""
    if product_id == "P999":
        return "P999;6.0"
    entry = PRODUCT_TABLE[product_id]
    return f"{product_id};{entry['weight']}"


def load_json(rel_path: str, result: CheckResult):
    """Wczytuje plik JSON; przy braku/błędzie rejestruje FAIL i zwraca None."""
    path = PROJECT_ROOT / rel_path
    if not path.is_file():
        result.fail(f"Brak pliku: {rel_path}")
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError, UnicodeDecodeError) as exc:
        result.fail(f"Nie można sparsować {rel_path}: {exc}")
        return None


def check_brace_balance(text: str) -> tuple[bool, str]:
    """Sprawdza balans nawiasów {} i [] w pliku VRML/PROTO.

    Znaki wewnątrz napisów w cudzysłowach są ignorowane.
    Zwraca (czy_zbalansowane, opis_problemu).
    """
    pairs = {"{": "}", "[": "]"}
    stack: list[tuple[str, int]] = []
    in_string = False
    for lineno, line in enumerate(text.splitlines(), start=1):
        i = 0
        while i < len(line):
            ch = line[i]
            if in_string:
                if ch == "\\":
                    i += 2
                    continue
                if ch == '"':
                    in_string = False
            else:
                if ch == '"':
                    in_string = True
                elif ch in pairs:
                    stack.append((ch, lineno))
                elif ch in pairs.values():
                    if not stack or pairs[stack[-1][0]] != ch:
                        return False, f"niedopasowany '{ch}' w linii {lineno}"
                    stack.pop()
            i += 1
    if stack:
        ch, lineno = stack[-1]
        return False, f"niezamknięty '{ch}' otwarty w linii {lineno}"
    return True, ""


def extract_node_block(text: str, def_name: str) -> str | None:
    """Wycina blok DEF <def_name> ... wraz z zbalansowanymi nawiasami.

    Używane do sprawdzenia zawartości konkretnego węzła świata
    (np. czy DEF PRODUCT_P020 używa tekstury P999.png).
    Zwraca tekst bloku lub None, gdy definicji nie znaleziono.
    """
    match = re.search(rf"\bDEF\s+{re.escape(def_name)}\b", text)
    if not match:
        return None
    start = match.start()
    open_idx = text.find("{", match.end())
    if open_idx == -1:
        return text[start:]
    depth = 0
    for i in range(open_idx, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
    return text[start:]


# ---------------------------------------------------------------------------
# Kontrola 1: istnienie plików kontraktowych
# ---------------------------------------------------------------------------

def check_contract_files() -> CheckResult:
    res = CheckResult("1. Istnienie plików kontraktowych")
    missing = [rel for rel in CONTRACT_FILES if not (PROJECT_ROOT / rel).is_file()]
    for rel in missing:
        res.fail(f"Brak pliku: {rel}")
    res.note(f"Sprawdzono {len(CONTRACT_FILES)} plików; brakuje: {len(missing)}.")
    return res


# ---------------------------------------------------------------------------
# Kontrola 2: products.json vs tabela kontraktu
# ---------------------------------------------------------------------------

def check_products_json() -> CheckResult:
    res = CheckResult("2. Zgodność data/products.json z tabelą kontraktu")
    data = load_json("data/products.json", res)
    if data is None:
        return res
    products = data.get("products")
    if not isinstance(products, list):
        res.fail("products.json: brak listy 'products'.")
        return res
    by_id = {}
    for p in products:
        pid = p.get("id")
        if pid in by_id:
            res.fail(f"Zduplikowany produkt: {pid}")
        by_id[pid] = p

    expected_ids = set(PRODUCT_TABLE)
    actual_ids = set(by_id)
    for pid in sorted(expected_ids - actual_ids):
        res.fail(f"Brak produktu {pid} w products.json.")
    for pid in sorted(actual_ids - expected_ids):
        res.fail(f"Nadmiarowy produkt {pid} w products.json (poza kontraktem).")

    for pid in sorted(expected_ids & actual_ids):
        entry = PRODUCT_TABLE[pid]
        p = by_id[pid]

        if p.get("weight") != entry["weight"]:
            res.fail(f"{pid}: waga {p.get('weight')} != {entry['weight']}.")
        if p.get("shelf_cell") != entry["cell"]:
            res.fail(f"{pid}: shelf_cell {p.get('shelf_cell')} != {entry['cell']}.")
        if p.get("level") != entry["level"]:
            res.fail(f"{pid}: level {p.get('level')} != {entry['level']}.")
        if p.get("slot") != entry["slot"]:
            res.fail(f"{pid}: slot {p.get('slot')} != {entry['slot']}.")

        expected_qr = qr_payload_for(pid)
        if p.get("qr_payload") != expected_qr:
            res.fail(f"{pid}: qr_payload '{p.get('qr_payload')}' != '{expected_qr}'.")

        exp_pos = expected_position(entry)
        pos = p.get("position")
        if not (isinstance(pos, list) and len(pos) == 3):
            res.fail(f"{pid}: nieprawidłowa pozycja {pos!r}.")
        elif any(abs(a - b) > EPS for a, b in zip(pos, exp_pos)):
            res.fail(f"{pid}: position {pos} != oczekiwane {exp_pos}.")

        exp_access = access_cell_for(entry["cell"])
        if p.get("access_cell") != exp_access:
            res.fail(f"{pid}: access_cell {p.get('access_cell')} != {exp_access}.")

        exp_tex = f"textures/qr/{pid}.png"
        if p.get("texture") != exp_tex:
            res.fail(f"{pid}: texture '{p.get('texture')}' != '{exp_tex}'.")

        if p.get("present") != entry["present"]:
            res.fail(f"{pid}: present={p.get('present')} != {entry['present']}.")
        if p.get("world_qr") != entry["world_qr"]:
            res.fail(f"{pid}: world_qr {p.get('world_qr')!r} != {entry['world_qr']!r}.")

    if res.ok:
        res.note(f"Wszystkie {len(PRODUCT_TABLE)} produktów zgodne z tabelą kontraktu.")
    return res


# ---------------------------------------------------------------------------
# Kontrola 3: warehouse_layout.json
# ---------------------------------------------------------------------------

def check_layout_json() -> CheckResult:
    res = CheckResult("3. Parametry data/warehouse_layout.json")
    data = load_json("data/warehouse_layout.json", res)
    if data is None:
        return res

    def expect(key, expected):
        if data.get(key) != expected:
            res.fail(f"layout: '{key}' = {data.get(key)!r}, oczekiwano {expected!r}.")

    expect("cell_size", 1.0)
    expect("cols", 16)
    expect("rows", 12)
    expect("origin", {"x": -7.5, "z": -5.5})
    expect("terminal_cells", [[7, 11], [8, 11]])
    expect("terminal_world", [0.0, 5.5])
    expect("robot_start", {"cell": [7, 11], "world": [-0.5, 5.5], "heading_deg": 0})
    expect("levels", LEVELS)
    expect("slots", [-0.25, 0.25])
    expect("shelf_rows", {"north_access": [2, 5, 8], "south_access": [3, 6, 9]})
    expect("max_platform_load_kg", 50)
    expect("max_package_weight_kg", 20)

    blocked = data.get("blocked_cells")
    if not isinstance(blocked, list):
        res.fail("layout: brak listy 'blocked_cells'.")
    else:
        expected_blocked = sorted(
            [c, r] for r in (2, 3, 5, 6, 8, 9) for c in (2, 3, 4, 5, 6, 9, 10, 11, 12, 13)
        )
        actual = sorted(map(list, blocked))
        if len(blocked) != 60:
            res.fail(f"layout: liczba blocked_cells = {len(blocked)}, oczekiwano 60.")
        if len({tuple(cell) for cell in actual}) != len(actual):
            res.fail("layout: blocked_cells zawiera duplikaty.")
        if actual != expected_blocked:
            res.fail("layout: zbiór blocked_cells niezgodny z kontraktem "
                     "(rzędy {2,3,5,6,8,9} x kolumny {2,3,4,5,6,9,10,11,12,13}).")

    if res.ok:
        res.note("Siatka 16x12, 60 zablokowanych komórek regałowych — zgodnie.")
    return res


# ---------------------------------------------------------------------------
# Kontrola 4: zamówienia orders/*.json
# ---------------------------------------------------------------------------

def check_orders() -> CheckResult:
    res = CheckResult("4. Zamówienia orders/*.json")
    order_files = sorted((PROJECT_ROOT / "orders").glob("order_*.json")) \
        if (PROJECT_ROOT / "orders").is_dir() else []
    if not order_files:
        res.fail("Brak plików orders/order_*.json.")
        return res

    found: dict[str, list[str]] = {}
    for path in order_files:
        data = load_json(f"orders/{path.name}", res)
        if data is None:
            continue
        oid = data.get("order_id")
        items = data.get("items")
        if not oid:
            res.fail(f"{path.name}: brak pola 'order_id'.")
            continue
        if not isinstance(items, list):
            res.fail(f"{oid} ({path.name}): brak listy 'items'.")
            continue
        found[oid] = []
        for item in items:
            pid = item.get("product_id")
            if pid not in PRODUCT_TABLE:
                res.fail(f"{oid}: nieznany produkt '{pid}'.")
                continue
            found[oid].append(pid)
            exp_w = PRODUCT_TABLE[pid]["weight"]
            if item.get("expected_weight") != exp_w:
                res.fail(f"{oid}/{pid}: expected_weight {item.get('expected_weight')} != {exp_w}.")

    for oid, expected_ids in EXPECTED_ORDERS.items():
        if oid not in found:
            res.fail(f"Brak zamówienia {oid}.")
            continue
        actual_ids = found[oid]
        if actual_ids != expected_ids:
            res.fail(f"{oid}: pozycje {actual_ids} != oczekiwane {expected_ids}.")
            continue
        total = sum(PRODUCT_TABLE[pid]["weight"] for pid in actual_ids)
        res.note(f"{oid}: {len(actual_ids)} pozycji, łączna waga {total:.1f} kg.")

    # Dodatkowe twarde asercje kontraktu
    if found.get("ORD-001") == EXPECTED_ORDERS["ORD-001"]:
        total = sum(PRODUCT_TABLE[pid]["weight"] for pid in found["ORD-001"])
        if abs(total - 13.0) > EPS:
            res.fail(f"ORD-001: waga {total} != 13.0 kg.")
    if found.get("ORD-002") == EXPECTED_ORDERS["ORD-002"]:
        total = sum(PRODUCT_TABLE[pid]["weight"] for pid in found["ORD-002"])
        if abs(total - 85.0) > EPS:
            res.fail(f"ORD-002: waga {total} != 85.0 kg (powinna wymuszać split).")
    if "ORD-003" in found:
        for pid in ("P005", "P020"):
            if pid not in found["ORD-003"]:
                res.fail(f"ORD-003: brak produktu testowego {pid}.")
    return res


# ---------------------------------------------------------------------------
# Kontrola 5: worlds/warehouse.wbt
# ---------------------------------------------------------------------------

def check_world_file() -> CheckResult:
    res = CheckResult("5. Spójność worlds/warehouse.wbt")
    path = PROJECT_ROOT / "worlds/warehouse.wbt"
    if not path.is_file():
        res.fail("Brak pliku worlds/warehouse.wbt.")
        return res
    text = path.read_text(encoding="utf-8", errors="replace")

    ok, problem = check_brace_balance(text)
    if not ok:
        res.fail(f"Balans nawiasów: {problem}.")

    # DEF PRODUCT_<id> dla wszystkich present=true; brak PRODUCT_P005
    for pid, entry in PRODUCT_TABLE.items():
        present_in_world = re.search(rf"\bDEF\s+PRODUCT_{pid}\b", text) is not None
        if entry["present"] and not present_in_world:
            res.fail(f"Brak DEF PRODUCT_{pid} w świecie (present=true).")
        if not entry["present"] and present_in_world:
            res.fail(f"DEF PRODUCT_{pid} istnieje, choć present=false (pusta półka).")

    # Tekstura P999.png wewnątrz bloku PRODUCT_P020
    block = extract_node_block(text, "PRODUCT_P020")
    if block is None:
        res.fail("Nie znaleziono bloku DEF PRODUCT_P020.")
    elif "P999.png" not in block:
        res.fail("Blok PRODUCT_P020 nie używa tekstury P999.png (zły produkt na półce).")

    # Referencje do tekstur QR produktów obecnych w świecie
    for pid, entry in PRODUCT_TABLE.items():
        if entry["present"] and pid != "P020":
            if f"qr/{pid}.png" not in text:
                res.fail(f"Świat nie referencjonuje tekstury qr/{pid}.png.")

    for def_name in ["TERMINAL", "OBSTACLE_1", "OBSTACLE_2", "OBSTACLE_3"]:
        if not re.search(rf"\bDEF\s+{def_name}\b", text):
            res.fail(f"Brak DEF {def_name} w świecie.")

    if '"warehouse_controller"' not in text:
        res.fail('Brak przypisania controller "warehouse_controller" dla robota.')
    if "EXTERNPROTO" not in text or "WarehouseRobot.proto" not in text:
        res.fail("Brak referencji EXTERNPROTO do protos/WarehouseRobot.proto.")

    if res.ok:
        res.note("Świat zgodny: produkty, terminal, przeszkody, kontroler, EXTERNPROTO.")
    return res


# ---------------------------------------------------------------------------
# Kontrola 6: protos/WarehouseRobot.proto
# ---------------------------------------------------------------------------

def check_proto_file() -> CheckResult:
    res = CheckResult("6. Spójność protos/WarehouseRobot.proto")
    path = PROJECT_ROOT / "protos/WarehouseRobot.proto"
    if not path.is_file():
        res.fail("Brak pliku protos/WarehouseRobot.proto.")
        return res
    text = path.read_text(encoding="utf-8", errors="replace")

    ok, problem = check_brace_balance(text)
    if not ok:
        res.fail(f"Balans nawiasów: {problem}.")

    for name in DEVICE_NAMES:
        if not re.search(rf'"{re.escape(name)}"', text):
            res.fail(f"Brak nazwy urządzenia \"{name}\" w proto.")

    if not re.search(r"\bsupervisor\s+TRUE\b", text):
        res.fail("Pole 'supervisor' nie jest ustawione na TRUE.")

    if res.ok:
        res.note(f"Proto zgodne: {len(DEVICE_NAMES)} urządzeń, supervisor TRUE.")
    return res


# ---------------------------------------------------------------------------
# Kontrola 7: kompilacja składniowa wszystkich .py
# ---------------------------------------------------------------------------

def check_py_compile() -> CheckResult:
    res = CheckResult("7. Kompilacja składniowa plików .py")
    py_files = sorted(
        p for p in PROJECT_ROOT.rglob("*.py") if ".git" not in p.parts
    )
    if not py_files:
        res.fail("Nie znaleziono żadnych plików .py w projekcie.")
        return res
    for path in py_files:
        rel = path.relative_to(PROJECT_ROOT)
        try:
            py_compile.compile(str(path), doraise=True)
        except py_compile.PyCompileError as exc:
            res.fail(f"Błąd kompilacji {rel}: {exc.msg.strip()}")
        except OSError as exc:
            res.fail(f"Nie można skompilować {rel}: {exc}")
    res.note(f"Skompilowano {len(py_files)} plików .py.")
    return res


# ---------------------------------------------------------------------------
# Kontrola 8: roundtrip QR (dekodowanie tekstur)
# ---------------------------------------------------------------------------

def check_qr_roundtrip() -> CheckResult:
    res = CheckResult("8. Roundtrip QR: dekodowanie textures/qr/*.png")
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        res.fail(f"Brak wymaganego pakietu ({exc.name}); roundtrip QR niemożliwy.")
        return res

    texture_ids = [f"P{i:03d}" for i in range(1, 21)] + ["P999"]
    detector = cv2.QRCodeDetector()
    decoded_ok = 0
    for pid in texture_ids:
        rel = f"textures/qr/{pid}.png"
        path = PROJECT_ROOT / rel
        if not path.is_file():
            res.fail(f"Brak pliku tekstury: {rel}")
            continue
        img = cv2.imread(str(path))
        if img is None:
            res.fail(f"Nie można wczytać obrazu: {rel}")
            continue
        payload, _, _ = detector.detectAndDecode(img)
        if not payload:
            res.fail(f"{rel}: nie udało się zdekodować kodu QR.")
            continue
        expected = qr_payload_for(pid)
        if payload != expected:
            res.fail(f"{rel}: payload '{payload}' != oczekiwany '{expected}'.")
            continue
        decoded_ok += 1
    res.note(f"Poprawnie zdekodowano {decoded_ok}/{len(texture_ids)} kodów QR.")
    return res


# ---------------------------------------------------------------------------
# Kontrola 9: import i API modułów kontrolera (bez webots)
# ---------------------------------------------------------------------------

def _check_signature(res: CheckResult, owner: str, func, expected_params: list[str]) -> None:
    """Sprawdza, że funkcja/metoda przyjmuje parametry o nazwach z kontraktu."""
    try:
        params = list(inspect.signature(func).parameters)
    except (TypeError, ValueError) as exc:
        res.fail(f"{owner}: nie można odczytać sygnatury ({exc}).")
        return
    params = [p for p in params if p != "self"]
    missing = [p for p in expected_params if p not in params]
    if missing:
        res.fail(f"{owner}: brak parametrów {missing} w sygnaturze {params}.")


def check_module_api() -> CheckResult:
    res = CheckResult("9. API modułów navigation / qr_scanner / order_manager")
    controller_dir = PROJECT_ROOT / "controllers" / "warehouse_controller"
    if str(controller_dir) not in sys.path:
        sys.path.insert(0, str(controller_dir))

    modules = {}
    for mod_name in ("navigation", "qr_scanner", "order_manager"):
        path = controller_dir / f"{mod_name}.py"
        if not path.is_file():
            res.fail(f"Brak modułu controllers/warehouse_controller/{mod_name}.py.")
            continue
        try:
            modules[mod_name] = importlib.import_module(mod_name)
        except Exception as exc:  # noqa: BLE001 — import musi raportować FAIL, nie crash
            res.fail(f"Import {mod_name}.py nieudany: {type(exc).__name__}: {exc}")

    nav = modules.get("navigation")
    if nav is not None:
        navigator = getattr(nav, "Navigator", None)
        if navigator is None:
            res.fail("navigation: brak klasy Navigator.")
        else:
            _check_signature(res, "Navigator.__init__", navigator.__init__, ["layout_path"])
            for method, params in (
                ("cell_to_world", ["cell"]),
                ("find_path", ["start", "goal"]),
                ("plan_trips", ["start_cell", "items"]),
            ):
                func = getattr(navigator, method, None)
                if func is None:
                    res.fail(f"Navigator: brak metody {method}().")
                else:
                    _check_signature(res, f"Navigator.{method}", func, params)

    qr = modules.get("qr_scanner")
    if qr is not None:
        scanner = getattr(qr, "QRScanner", None)
        if scanner is None:
            res.fail("qr_scanner: brak klasy QRScanner.")
        else:
            _check_signature(res, "QRScanner.__init__", scanner.__init__, ["camera"])
            scan = getattr(scanner, "scan", None)
            if scan is None:
                res.fail("QRScanner: brak metody scan().")
        parse = getattr(qr, "parse_payload", None)
        if parse is None:
            res.fail("qr_scanner: brak funkcji parse_payload().")
        else:
            _check_signature(res, "parse_payload", parse, ["payload"])

    om = modules.get("order_manager")
    if om is not None:
        for cls_name in ("Order", "OrderItem", "OrderManager"):
            if getattr(om, cls_name, None) is None:
                res.fail(f"order_manager: brak klasy {cls_name}.")
        manager = getattr(om, "OrderManager", None)
        if manager is not None:
            _check_signature(res, "OrderManager.__init__", manager.__init__,
                             ["orders_dir", "products_path", "reports_dir"])
            for method, params in (
                ("next_order", []),
                ("mark_ok", ["item"]),
                ("mark_wrong", ["item", "found_id", "found_weight"]),
                ("mark_missing", ["item"]),
                ("build_report", ["order", "trips"]),
                ("save_report", ["report"]),
            ):
                func = getattr(manager, method, None)
                if func is None:
                    res.fail(f"OrderManager: brak metody {method}().")
                else:
                    _check_signature(res, f"OrderManager.{method}", func, params)

    if res.ok:
        res.note("Wszystkie klasy i metody sygnaturowe z kontraktu obecne.")
    return res


# ---------------------------------------------------------------------------
# Uruchomienie wszystkich kontroli i raport
# ---------------------------------------------------------------------------

ALL_CHECKS = [
    check_contract_files,
    check_products_json,
    check_layout_json,
    check_orders,
    check_world_file,
    check_proto_file,
    check_py_compile,
    check_qr_roundtrip,
    check_module_api,
]


def main() -> int:
    """Uruchamia wszystkie kontrole, wypisuje raport PASS/FAIL i zwraca kod wyjścia."""
    # Polskie znaki w raporcie również na konsoli Windows (cp1250/cp852)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    print("=" * 72)
    print("WALIDATOR PROJEKTU — symulacyjny system obsługi zamówień (Webots)")
    print(f"Korzeń projektu: {PROJECT_ROOT}")
    print("=" * 72)

    results: list[CheckResult] = []
    for check in ALL_CHECKS:
        try:
            results.append(check())
        except Exception as exc:  # noqa: BLE001 — walidator nigdy nie może crashować
            res = CheckResult(getattr(check, "__name__", "nieznana kontrola"))
            res.fail(f"Wyjątek wewnętrzny kontroli: {type(exc).__name__}: {exc}")
            results.append(res)

    passed = 0
    for res in results:
        status = "PASS" if res.ok else "FAIL"
        if res.ok:
            passed += 1
        print(f"\n[{status}] {res.name}")
        for line in res.details:
            print(line)

    print("\n" + "=" * 72)
    total = len(results)
    if passed == total:
        print(f"WYNIK: PASS — {passed}/{total} kontroli zaliczonych.")
        print("=" * 72)
        return 0
    print(f"WYNIK: FAIL — {passed}/{total} kontroli zaliczonych, "
          f"{total - passed} niezaliczonych.")
    print("=" * 72)
    return 1


if __name__ == "__main__":
    sys.exit(main())
