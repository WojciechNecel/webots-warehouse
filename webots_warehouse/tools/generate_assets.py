# -*- coding: utf-8 -*-
"""
================================================================================
Plik:        tools/generate_assets.py
Projekt:     Symulacyjny system obsługi zamówień magazynowych (Webots)
Przeznaczenie:
    Generator wszystkich statycznych artefaktów projektu:
      1) tekstury kodów QR produktów  -> textures/qr/P001.png .. P020.png, P999.png
         (payload każdego kodu: "<id_produktu>;<waga_kg>", np. "P001;3.5";
          P999.png z payloadem "P999;6.0" to fizyczny "zły produkt" leżący
          na półce produktu P020 — scenariusz testowy wrong_product),
      2) opis geometrii magazynu      -> data/warehouse_layout.json,
      3) katalog produktów            -> data/products.json,
      4) definicje zamówień testowych -> orders/order_01.json .. order_03.json.

Sposób użycia:
    python tools/generate_assets.py
    Skrypt nadpisuje generowane artefakty i na końcu uruchamia weryfikację:
      - roundtrip QR: każdy wygenerowany PNG jest dekodowany przez
        cv2.QRCodeDetector i payload porównywany z oczekiwanym,
      - walidacja JSON: pozycje produktów liczone wg wzoru
        x = -7.5 + c + slot, z = -5.5 + r, y = levels[level],
      - kontrola kompletności listy 60 zablokowanych komórek regałów.
    Wyjście zerowe (exit code 0) oznacza pełną zgodność z kontraktem.

Struktury danych (zgodne z kontraktem integracyjnym projektu):
    PRODUCTS: list[dict] — sztywna tabela produktów (źródło prawdy):
        {"id", "cell": (c, r), "level": 0..2, "slot": -0.25|+0.25,
         "weight": kg, "present": bool, "world_qr": str|None}
    warehouse_layout.json:
        {"cell_size": 1.0, "cols": 16, "rows": 12,
         "origin": {"x": -7.5, "z": -5.5},
         "cell_to_world": "x = origin.x + c; z = origin.z + r",
         "terminal_cells": [[7,11],[8,11]], "terminal_world": [0.0, 5.5],
         "robot_start": {"cell": [7,11], "world": [-0.5, 5.5], "heading_deg": 0},
         "blocked_cells": [[c, r], ...],           # 60 komórek regałów
         "shelf_rows": {"north_access": [2,5,8], "south_access": [3,6,9]},
         "levels": [0.2, 0.8, 1.4], "slots": [-0.25, 0.25],
         "access_rule": "r in {2,5,8} -> (c,r-1); r in {3,6,9} -> (c,r+1)",
         "max_platform_load_kg": 50, "max_package_weight_kg": 20}
    products.json:
        {"products": [{"id", "qr_payload", "weight", "shelf_cell": [c,r],
                       "level", "slot", "position": [x, y, z],
                       "access_cell": [c,r], "texture", "present",
                       "world_qr"}, ...]}
    orders/order_XX.json:
        {"order_id": "ORD-00X",
         "items": [{"product_id": "Pxxx", "expected_weight": float}, ...]}

Funkcje:
    cell_to_world(cell)      — środek komórki siatki w świecie (x, z).
    access_cell_for(cell)    — komórka dostępu do regału wg reguły kontraktu.
    product_position(p)      — pozycja [x, y, z] środka produktu w świecie.
    generate_qr_codes()      — renderuje tekstury QR (qrcode + PIL).
    generate_layout()        — buduje data/warehouse_layout.json.
    generate_products()      — buduje data/products.json.
    generate_orders()        — buduje orders/order_01..03.json.
    verify()                 — test roundtrip QR + walidacja JSON-ów.
    main()                   — generuje wszystko i uruchamia verify().

Wymagane pakiety: qrcode, pillow (zapis PNG), opencv-python + numpy (weryfikacja).
================================================================================
"""

import json
import sys
from pathlib import Path

import cv2
import numpy as np
import qrcode
from PIL import Image

# --- Ścieżki projektu ---------------------------------------------------------
ROOT = Path(__file__).resolve().parents[1]          # webots_warehouse/
TEXTURES_DIR = ROOT / "textures" / "qr"
DATA_DIR = ROOT / "data"
ORDERS_DIR = ROOT / "orders"

# --- Stałe geometrii magazynu (kontrakt integracyjny) -------------------------
CELL_SIZE = 1.0
COLS = 16
ROWS = 12
ORIGIN_X = -7.5   # x = ORIGIN_X + c
ORIGIN_Z = -5.5   # z = ORIGIN_Z + r
TERMINAL_CELLS = [[7, 11], [8, 11]]
TERMINAL_WORLD = [0.0, 5.5]
ROBOT_START = {"cell": [7, 11], "world": [-0.5, 5.5], "heading_deg": 0}
LEVELS = [0.2, 0.8, 1.4]        # wysokości środków produktów na poziomach
SLOTS = [-0.25, 0.25]           # offsety slotów wzdłuż osi X
SHELF_ROWS_NORTH_ACCESS = [2, 5, 8]    # robot staje w (c, r-1), patrzy na +Z
SHELF_ROWS_SOUTH_ACCESS = [3, 6, 9]    # robot staje w (c, r+1), patrzy na -Z
SHELF_COLS = [2, 3, 4, 5, 6, 9, 10, 11, 12, 13]
MAX_PLATFORM_LOAD_KG = 50
MAX_PACKAGE_WEIGHT_KG = 20

# --- Sztywna tabela produktów (źródło prawdy dla świata, danych i zamówień) ---
PRODUCTS = [
    {"id": "P001", "cell": (2, 2),  "level": 0, "slot": -0.25, "weight": 3.5,  "present": True,  "world_qr": None},
    {"id": "P002", "cell": (4, 2),  "level": 1, "slot": 0.25,  "weight": 7.0,  "present": True,  "world_qr": None},
    {"id": "P003", "cell": (6, 2),  "level": 2, "slot": -0.25, "weight": 12.0, "present": True,  "world_qr": None},
    {"id": "P004", "cell": (9, 2),  "level": 1, "slot": 0.25,  "weight": 5.0,  "present": True,  "world_qr": None},
    # P005: pusta półka (brak boxa w świecie) — scenariusz testowy "missing".
    {"id": "P005", "cell": (11, 2), "level": 0, "slot": -0.25, "weight": 18.0, "present": False, "world_qr": None},
    {"id": "P006", "cell": (13, 2), "level": 2, "slot": 0.25,  "weight": 9.5,  "present": True,  "world_qr": None},
    {"id": "P007", "cell": (2, 3),  "level": 1, "slot": -0.25, "weight": 4.0,  "present": True,  "world_qr": None},
    {"id": "P008", "cell": (5, 3),  "level": 0, "slot": 0.25,  "weight": 15.0, "present": True,  "world_qr": None},
    {"id": "P009", "cell": (10, 3), "level": 2, "slot": -0.25, "weight": 6.5,  "present": True,  "world_qr": None},
    {"id": "P010", "cell": (13, 3), "level": 1, "slot": 0.25,  "weight": 11.0, "present": True,  "world_qr": None},
    {"id": "P011", "cell": (2, 5),  "level": 2, "slot": -0.25, "weight": 2.5,  "present": True,  "world_qr": None},
    {"id": "P012", "cell": (6, 5),  "level": 0, "slot": 0.25,  "weight": 8.0,  "present": True,  "world_qr": None},
    {"id": "P013", "cell": (9, 5),  "level": 1, "slot": -0.25, "weight": 14.0, "present": True,  "world_qr": None},
    {"id": "P014", "cell": (12, 5), "level": 0, "slot": 0.25,  "weight": 19.0, "present": True,  "world_qr": None},
    {"id": "P015", "cell": (3, 6),  "level": 1, "slot": -0.25, "weight": 3.0,  "present": True,  "world_qr": None},
    {"id": "P016", "cell": (11, 6), "level": 2, "slot": 0.25,  "weight": 10.5, "present": True,  "world_qr": None},
    {"id": "P017", "cell": (5, 8),  "level": 0, "slot": -0.25, "weight": 5.5,  "present": True,  "world_qr": None},
    {"id": "P018", "cell": (9, 8),  "level": 2, "slot": 0.25,  "weight": 13.0, "present": True,  "world_qr": None},
    {"id": "P019", "cell": (2, 9),  "level": 1, "slot": -0.25, "weight": 7.5,  "present": True,  "world_qr": None},
    # P020: na półce fizycznie leży box z kodem P999 (zły produkt) — scenariusz
    # testowy "wrong_product". Payload P999 zgodny z formatem "<id>;<waga>".
    {"id": "P020", "cell": (13, 9), "level": 0, "slot": -0.25, "weight": 16.0, "present": True,  "world_qr": "P999;6.0"},
]

# Dodatkowy kod QR niebędący produktem katalogowym (zły produkt na półce P020).
EXTRA_QR = {"P999": "P999;6.0"}

# --- Zamówienia testowe (kontrakt integracyjny) -------------------------------
# ORD-001: zielone, jeden kurs (13 kg).
# ORD-002: zielone, wymusza podział >50 kg (85 kg -> min. 2 kursy).
# ORD-003: czerwone — P005 brak na półce, P020 zwróci P999 zamiast P020.
ORDERS = [
    ("order_01.json", "ORD-001", ["P001", "P007", "P011", "P015"]),
    ("order_02.json", "ORD-002", ["P003", "P006", "P008", "P010", "P013", "P016", "P018"]),
    ("order_03.json", "ORD-003", ["P002", "P005", "P020", "P019"]),
]


def qr_payload(product_id: str, weight: float) -> str:
    """Buduje payload kodu QR w formacie kontraktowym ``"<id>;<waga>"``."""
    return f"{product_id};{weight}"


def cell_to_world(cell: tuple[int, int]) -> tuple[float, float]:
    """Zwraca współrzędne środka komórki siatki w świecie Webots (x, z)."""
    c, r = cell
    return (ORIGIN_X + c, ORIGIN_Z + r)


def access_cell_for(cell: tuple[int, int]) -> tuple[int, int]:
    """Komórka dostępu do regału: r∈{2,5,8} -> (c,r-1); r∈{3,6,9} -> (c,r+1)."""
    c, r = cell
    if r in SHELF_ROWS_NORTH_ACCESS:
        return (c, r - 1)
    if r in SHELF_ROWS_SOUTH_ACCESS:
        return (c, r + 1)
    raise ValueError(f"Komórka {cell} nie jest komórką regału.")


def product_position(product: dict) -> list[float]:
    """Pozycja [x, y, z] środka produktu: x=-7.5+c+slot, y=levels[level], z=-5.5+r."""
    c, r = product["cell"]
    x = ORIGIN_X + c + product["slot"]
    y = LEVELS[product["level"]]
    z = ORIGIN_Z + r
    return [x, y, z]


def generate_qr_codes() -> list[Path]:
    """Generuje tekstury QR produktów (P001..P020) oraz dodatkowy kod P999.

    Każdy kod zawiera payload ``"<id>;<waga>"``. Rozmiar modułu i margines
    dobrano tak, by kody były stabilnie dekodowalne przez cv2.QRCodeDetector
    (także po nałożeniu jako tekstura na bryłę w Webots).
    Obraz jest skalowany metodą NEAREST do 512x512 px — Webots wymaga
    tekstur o wymiarach będących potęgą dwójki (eliminuje to ostrzeżenie
    o rescalingu i deformację przy ładowaniu świata).
    Zwraca listę ścieżek utworzonych plików PNG.
    """
    TEXTURES_DIR.mkdir(parents=True, exist_ok=True)
    written = []
    payloads = {p["id"]: qr_payload(p["id"], p["weight"]) for p in PRODUCTS}
    payloads.update(EXTRA_QR)
    for product_id, payload in sorted(payloads.items()):
        img = qrcode.make(payload, box_size=10, border=4)
        img = img.resize((512, 512), Image.NEAREST)
        path = TEXTURES_DIR / f"{product_id}.png"
        img.save(path)
        written.append(path)
        print(f"[QR] {path.relative_to(ROOT)} <- '{payload}'")
    return written


def generate_layout() -> Path:
    """Generuje data/warehouse_layout.json zgodnie ze schematem kontraktu."""
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    shelf_rows = SHELF_ROWS_NORTH_ACCESS + SHELF_ROWS_SOUTH_ACCESS
    blocked = [[c, r] for r in sorted(shelf_rows) for c in SHELF_COLS]
    layout = {
        "cell_size": CELL_SIZE,
        "cols": COLS,
        "rows": ROWS,
        "origin": {"x": ORIGIN_X, "z": ORIGIN_Z},
        "cell_to_world": "x = origin.x + c; z = origin.z + r",
        "terminal_cells": TERMINAL_CELLS,
        "charging_cells": [[15, 11], [0, 11]],
        "terminal_world": TERMINAL_WORLD,
        "robot_start": ROBOT_START,
        "blocked_cells": blocked,
        "shelf_rows": {
            "north_access": SHELF_ROWS_NORTH_ACCESS,
            "south_access": SHELF_ROWS_SOUTH_ACCESS,
        },
        "levels": LEVELS,
        "slots": SLOTS,
        "access_rule": "r in {2,5,8} -> (c,r-1); r in {3,6,9} -> (c,r+1)",
        "max_platform_load_kg": MAX_PLATFORM_LOAD_KG,
        "max_package_weight_kg": MAX_PACKAGE_WEIGHT_KG,
    }
    path = DATA_DIR / "warehouse_layout.json"
    path.write_text(json.dumps(layout, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"[JSON] {path.relative_to(ROOT)} ({len(blocked)} blocked_cells)")
    return path


def generate_products() -> Path:
    """Generuje data/products.json (katalog produktów z pozycjami i payloadami)."""
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    entries = []
    for p in PRODUCTS:
        entries.append({
            "id": p["id"],
            "qr_payload": qr_payload(p["id"], p["weight"]),
            "weight": p["weight"],
            "shelf_cell": list(p["cell"]),
            "level": p["level"],
            "slot": p["slot"],
            "position": product_position(p),
            "access_cell": list(access_cell_for(p["cell"])),
            "texture": f"textures/qr/{p['id']}.png",
            "present": p["present"],
            "world_qr": p["world_qr"],
        })
    path = DATA_DIR / "products.json"
    path.write_text(json.dumps({"products": entries}, indent=2, ensure_ascii=False) + "\n",
                    encoding="utf-8")
    print(f"[JSON] {path.relative_to(ROOT)} ({len(entries)} produktów)")
    return path


def generate_orders() -> list[Path]:
    """Generuje orders/order_01..03.json wg definicji ORD-001/002/003."""
    ORDERS_DIR.mkdir(parents=True, exist_ok=True)
    weights = {p["id"]: p["weight"] for p in PRODUCTS}
    written = []
    for filename, order_id, product_ids in ORDERS:
        order = {
            "order_id": order_id,
            "items": [
                {"product_id": pid, "expected_weight": weights[pid]}
                for pid in product_ids
            ],
        }
        path = ORDERS_DIR / filename
        path.write_text(json.dumps(order, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        total = sum(weights[pid] for pid in product_ids)
        print(f"[JSON] {path.relative_to(ROOT)} ({order_id}, {len(product_ids)} pozycji, {total} kg)")
        written.append(path)
    return written


def verify() -> bool:
    """Weryfikuje wygenerowane artefakty względem kontraktu integracyjnego.

    Kroki:
      1) roundtrip QR — każdy PNG dekodowany przez cv2.QRCodeDetector,
         payload musi być identyczny z oczekiwanym,
      2) walidacja JSON — wszystkie pliki parsują się; pozycje produktów
         zgodne ze wzorem x=-7.5+c+slot, z=-5.5+r, y=levels[level]; komórki
         dostępu zgodne z regułą; zamówienia zgodne z tabelą produktów,
      3) kompletność 60 zablokowanych komórek regałów.
    Zwraca True, gdy wszystkie kontrole przechodzą.
    """
    ok = True

    def fail(msg: str) -> None:
        nonlocal ok
        ok = False
        print(f"[WERYFIKACJA][BŁĄD] {msg}")

    # --- 1) Roundtrip QR ------------------------------------------------------
    detector = cv2.QRCodeDetector()
    expected_payloads = {p["id"]: qr_payload(p["id"], p["weight"]) for p in PRODUCTS}
    expected_payloads.update(EXTRA_QR)
    for product_id, expected in sorted(expected_payloads.items()):
        path = TEXTURES_DIR / f"{product_id}.png"
        if not path.is_file():
            fail(f"brak pliku {path.relative_to(ROOT)}")
            continue
        img = cv2.imread(str(path))
        if img is None:
            fail(f"nie można wczytać {path.relative_to(ROOT)}")
            continue
        decoded, _, _ = detector.detectAndDecode(img)
        if decoded != expected:
            fail(f"{product_id}.png: zdekodowano '{decoded}', oczekiwano '{expected}'")
        else:
            print(f"[WERYFIKACJA] QR {product_id}: OK ('{decoded}')")

    # --- 2) Walidacja JSON ----------------------------------------------------
    try:
        layout = json.loads((DATA_DIR / "warehouse_layout.json").read_text(encoding="utf-8"))
        products = json.loads((DATA_DIR / "products.json").read_text(encoding="utf-8"))["products"]
        orders = {
            fname: json.loads((ORDERS_DIR / fname).read_text(encoding="utf-8"))
            for fname, _, _ in ORDERS
        }
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"błąd parsowania JSON: {exc}")
        return False

    by_id = {p["id"]: p for p in PRODUCTS}
    if len(products) != len(PRODUCTS):
        fail(f"products.json: {len(products)} wpisów, oczekiwano {len(PRODUCTS)}")
    for entry in products:
        ref = by_id.get(entry["id"])
        if ref is None:
            fail(f"products.json: nieznany produkt {entry['id']}")
            continue
        if entry["qr_payload"] != qr_payload(ref["id"], ref["weight"]):
            fail(f"{entry['id']}: zły qr_payload '{entry['qr_payload']}'")
        if entry["weight"] != ref["weight"]:
            fail(f"{entry['id']}: zła waga {entry['weight']}")
        if entry["shelf_cell"] != list(ref["cell"]):
            fail(f"{entry['id']}: zła komórka {entry['shelf_cell']}")
        if entry["position"] != product_position(ref):
            fail(f"{entry['id']}: zła pozycja {entry['position']}, "
                 f"oczekiwano {product_position(ref)}")
        if entry["access_cell"] != list(access_cell_for(ref["cell"])):
            fail(f"{entry['id']}: zła komórka dostępu {entry['access_cell']}")
        if entry["present"] != ref["present"] or entry["world_qr"] != ref["world_qr"]:
            fail(f"{entry['id']}: niezgodne present/world_qr")

    for fname, order_id, product_ids in ORDERS:
        order = orders[fname]
        if order["order_id"] != order_id:
            fail(f"{fname}: order_id '{order['order_id']}', oczekiwano '{order_id}'")
        got_ids = [it["product_id"] for it in order["items"]]
        if got_ids != product_ids:
            fail(f"{fname}: pozycje {got_ids}, oczekiwano {product_ids}")
        for it in order["items"]:
            if it["expected_weight"] != by_id[it["product_id"]]["weight"]:
                fail(f"{fname}/{it['product_id']}: zła expected_weight")

    # --- 3) Kompletność blocked_cells -----------------------------------------
    expected_blocked = sorted(
        (c, r) for r in SHELF_ROWS_NORTH_ACCESS + SHELF_ROWS_SOUTH_ACCESS
        for c in SHELF_COLS
    )
    got_blocked = sorted(tuple(b) for b in layout["blocked_cells"])
    if len(layout["blocked_cells"]) != 60:
        fail(f"blocked_cells: {len(layout['blocked_cells'])} wpisów, oczekiwano 60")
    if got_blocked != expected_blocked:
        fail("blocked_cells: lista niezgodna z siatką regałów (rzędy {2,3,5,6,8,9} "
             "× kolumny {2..6,9..13})")
    if len(set(got_blocked)) != len(got_blocked):
        fail("blocked_cells: wykryto duplikaty")

    print("[WERYFIKACJA] " + ("WSZYSTKO OK" if ok else "WYKRYTO BŁĘDY"))
    return ok


def main() -> int:
    """Generuje wszystkie artefakty projektu i uruchamia weryfikację."""
    print(f"[GEN] Korzeń projektu: {ROOT}")
    generate_qr_codes()
    generate_layout()
    generate_products()
    generate_orders()
    return 0 if verify() else 1


if __name__ == "__main__":
    sys.exit(main())
