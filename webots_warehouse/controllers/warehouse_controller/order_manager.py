# -*- coding: utf-8 -*-
"""
================================================================================
order_manager.py — Zarządzanie zamówieniami i raportowanie (system magazynowy Webots)
================================================================================

PRZEZNACZENIE PLIKU
-------------------
Moduł odpowiada za wczytywanie zamówień z katalogu ``orders/``, wzbogacanie
ich pozycji o dane katalogowe z ``data/products.json`` (waga, komórka regału,
poziom, slot, komórka dostępu, pozycja światowa), śledzenie statusu każdej
pozycji podczas kompletacji oraz generowanie i zapisywanie raportów JSON
do katalogu ``reports/`` po zakończeniu zamówienia.

Moduł jest całkowicie niezależny od Webots (bez importu ``controller``),
dzięki czemu może być testowany poza symulacją.

SPOSÓB UŻYCIA
-------------
Kontroler główny (``warehouse_controller.py``) używa modułu w cyklu::

    manager = OrderManager(orders_dir, products_path, reports_dir)
    order = manager.next_order()            # None, gdy brak kolejnych zamówień
    for item in order.items:
        ...
        manager.mark_ok(item)                          # pobrano zgodny produkt
        manager.mark_wrong(item, found_id, found_w)    # zły produkt / zła waga
        manager.mark_missing(item)                     # pusta półka
    report = manager.build_report(order, trips=2)
    path = manager.save_report(report)

STRUKTURY DANYCH
----------------
``OrderItem`` (dataclass) — pojedyncza pozycja zamówienia:
    product_id       str            identyfikator produktu, np. "P001"
    expected_weight  float          waga oczekiwana z pliku zamówienia [kg]
    weight           float | None   waga katalogowa z products.json [kg]
    shelf_cell       tuple[int,int] | None  komórka regału (c, r)
    level            int | None     poziom regału (0, 1, 2)
    slot             float | None   przesunięcie slotu wzdłuż osi X (±0.25)
    access_cell      tuple[int,int] | None  komórka, w której staje robot
    position         tuple[float,float,float] | None  pozycja (x, y, z) w świecie
    status           str   "pending" | "ok" | "wrong_product" | "missing"
    message          str   komunikat do raportu (po polsku)

``Order`` (dataclass) — zamówienie:
    order_id         str            identyfikator, np. "ORD-001"
    items            list[OrderItem]  pozycje w kolejności z pliku zamówienia

Format pliku zamówienia (orders/order_XX.json)::
    {"order_id": "ORD-001",
     "items": [{"product_id": "P001", "expected_weight": 3.5}, ...]}

Format raportu (reports/report_<order_id>.json)::
    {"order_id": "ORD-001",
     "flag": "green" | "red",
     "trips": 2,
     "timestamp": "2025-01-01T12:00:00",
     "items": [{"product_id": "P001", "status": "ok",
                "weight": 3.5, "message": "pobrano pomyślnie"}, ...],
     "logs": ["...", ...]}

FUNKCJE / METODY
----------------
OrderManager.__init__(orders_dir, products_path, reports_dir)
    Wczytuje katalog produktów i indeksuje pliki zamówień (kolejność wg nazwy).
OrderManager.next_order() -> Order | None
    Zwraca kolejne zamówienie z pozycjami wzbogaconymi o dane z products.json;
    resetuje dziennik logów zamówienia; None gdy zamówienia się skończyły.
OrderManager.mark_ok(item)
    Ustawia status "ok" z komunikatem "pobrano pomyślnie".
OrderManager.mark_wrong(item, found_id, found_weight)
    Status "wrong_product". Jeśli znaleziono inny produkt:
    "Błąd lokalizacji [regał [c,r] poziom L] - znaleziono [Y] zamiast [Z]".
    Jeśli id jest zgodne, lecz waga nie: wariant wagowy
    "Błąd wagi [regał [c,r] poziom L] - oczekiwano [A] kg, jest [B] kg".
OrderManager.mark_missing(item)
    Ustawia status "missing" z komunikatem "brak produktu".
OrderManager.build_report(order, trips) -> dict
    Buduje słownik raportu; flaga "red", gdy cokolwiek nie jest "ok".
OrderManager.save_report(report) -> str
    Zapisuje raport do reports/report_<order_id>.json (tworzy katalogi),
    zwraca ścieżkę zapisanego pliku.

WERYFIKACJA
-----------
``python order_manager.py`` uruchamia testy na danych tymczasowych
(tempfile): zamówienie mieszane (ok / wrong_product / missing) → flaga red
z dokładnymi komunikatami; zamówienie samych sukcesów → flaga green;
zapis i odczyt pliku raportu.
================================================================================
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path


@dataclass
class OrderItem:
    """Pojedyncza pozycja zamówienia wzbogacona o dane katalogowe produktu.

    Pola ``weight``, ``shelf_cell``, ``level``, ``slot``, ``access_cell``
    i ``position`` pochodzą z ``data/products.json`` (wypełnia je
    ``OrderManager.next_order``). ``status`` startuje jako ``"pending"``
    i zmieniają go metody ``mark_ok`` / ``mark_wrong`` / ``mark_missing``.
    """

    product_id: str
    expected_weight: float
    weight: float | None = None
    shelf_cell: tuple[int, int] | None = None
    level: int | None = None
    slot: float | None = None
    access_cell: tuple[int, int] | None = None
    position: tuple[float, float, float] | None = None
    status: str = "pending"
    message: str = ""


@dataclass
class Order:
    """Zamówienie: identyfikator oraz lista pozycji w kolejności z pliku."""

    order_id: str
    items: list[OrderItem] = field(default_factory=list)


class OrderManager:
    """Wczytuje zamówienia, śledzi statusy pozycji i zapisuje raporty JSON.

    Parametry
    ---------
    orders_dir : str
        Katalog z plikami zamówień ``order_*.json`` (przetwarzane wg nazwy).
    products_path : str
        Ścieżka do ``data/products.json`` (katalog produktów).
    reports_dir : str
        Katalog wyjściowy na raporty ``report_<order_id>.json``.
    """

    def __init__(self, orders_dir: str, products_path: str, reports_dir: str) -> None:
        self._orders_dir = Path(orders_dir)
        self._reports_dir = Path(reports_dir)
        with open(products_path, encoding="utf-8") as f:
            products = json.load(f)["products"]
        # Indeks katalogu produktów po identyfikatorze (np. "P001").
        self._products: dict[str, dict] = {p["id"]: p for p in products}
        # Pliki zamówień posortowane wg nazwy — kolejność przetwarzania.
        self._order_files: list[Path] = sorted(self._orders_dir.glob("*.json"))
        self._next_index = 0
        # Logi bieżącego zamówienia (trafiają do raportu).
        self._logs: list[str] = []

    def next_order(self) -> Order | None:
        """Zwraca kolejne zamówienie (wg nazwy pliku) lub None, gdy koniec.

        Każda pozycja jest wzbogacana o dane z ``products.json``: wagę
        katalogową, komórkę regału, poziom, slot, komórkę dostępu i pozycję
        w świecie. Produkt nieobecny w katalogu pozostawia te pola jako None.
        """
        if self._next_index >= len(self._order_files):
            return None
        path = self._order_files[self._next_index]
        self._next_index += 1
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        items: list[OrderItem] = []
        for raw in data["items"]:
            item = OrderItem(
                product_id=raw["product_id"],
                expected_weight=float(raw["expected_weight"]),
            )
            product = self._products.get(item.product_id)
            if product is not None:
                item.weight = float(product["weight"])
                item.shelf_cell = tuple(product["shelf_cell"])
                item.level = int(product["level"])
                item.slot = float(product["slot"])
                item.access_cell = tuple(product["access_cell"])
                item.position = tuple(product["position"])
            items.append(item)
        self._logs = []
        order = Order(order_id=data["order_id"], items=items)
        self._logs.append(
            f"Wczytano zamówienie {order.order_id} ({len(items)} pozycji)."
        )
        return order

    def mark_ok(self, item: OrderItem) -> None:
        """Oznacza pozycję jako pobraną pomyślnie (status ``ok``)."""
        item.status = "ok"
        item.message = "pobrano pomyślnie"
        self._logs.append(f"{item.product_id}: pobrano pomyślnie.")

    def mark_wrong(
        self,
        item: OrderItem,
        found_id: str,
        found_weight: float | None,
    ) -> None:
        """Oznacza pozycję jako niezgodną (status ``wrong_product``).

        Jeśli ``found_id`` różni się od oczekiwanego ``product_id``, komunikat
        ma wariant lokalizacyjny:
        ``Błąd lokalizacji [regał [c,r] poziom L] - znaleziono [Y] zamiast [Z]``.
        Jeśli identyfikator jest zgodny, lecz waga nie, stosowany jest wariant
        wagowy:
        ``Błąd wagi [regał [c,r] poziom L] - oczekiwano [A] kg, jest [B] kg``.
        """
        item.status = "wrong_product"
        location = self._location_label(item)
        if found_id != item.product_id:
            item.message = (
                f"Błąd lokalizacji {location} - znaleziono {found_id} "
                f"zamiast {item.product_id}"
            )
        else:
            item.message = (
                f"Błąd wagi {location} - oczekiwano {item.expected_weight} kg, "
                f"jest {found_weight} kg"
            )
        self._logs.append(f"{item.product_id}: {item.message}.")

    def mark_missing(self, item: OrderItem) -> None:
        """Oznacza pozycję jako nieobecną na półce (status ``missing``)."""
        item.status = "missing"
        item.message = "brak produktu"
        self._logs.append(f"{item.product_id}: brak produktu na półce.")

    def build_report(self, order: Order, trips: int) -> dict:
        """Buduje słownik raportu dla zakończonego zamówienia.

        Flaga jest ``green`` wyłącznie gdy wszystkie pozycje mają status
        ``ok``; w przeciwnym razie ``red``. ``trips`` to liczba kursów
        robota do regałów wykonanych w ramach zamówienia.
        """
        flag = (
            "green"
            if all(item.status == "ok" for item in order.items)
            else "red"
        )
        return {
            "order_id": order.order_id,
            "flag": flag,
            "trips": trips,
            "timestamp": datetime.now().isoformat(),
            "items": [
                {
                    "product_id": item.product_id,
                    "status": item.status,
                    "weight": item.weight,
                    "message": item.message,
                }
                for item in order.items
            ],
            "logs": list(self._logs),
        }

    def save_report(self, report: dict) -> str:
        """Zapisuje raport do ``reports/report_<order_id>.json``.

        Tworzy katalogi nadrzędne, jeśli nie istnieją. Zwraca ścieżkę
        zapisanego pliku.
        """
        self._reports_dir.mkdir(parents=True, exist_ok=True)
        path = self._reports_dir / f"report_{report['order_id']}.json"
        with open(path, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        self._logs.append(f"Zapisano raport: {path}")
        return str(path)

    @staticmethod
    def _location_label(item: OrderItem) -> str:
        """Etykieta lokalizacji w formacie ``[regał [c,r] poziom L]``."""
        if item.shelf_cell is not None and item.level is not None:
            c, r = item.shelf_cell
            return f"[regał [{c},{r}] poziom {item.level}]"
        return f"[{item.product_id}]"


if __name__ == "__main__":
    # Testy modułu na tymczasowych danych (bez Webots i bez plików projektu).
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        orders_dir = tmp_path / "orders"
        orders_dir.mkdir()
        reports_dir = tmp_path / "reports"

        # Tymczasowy katalog produktów — 3 produkty z pełnymi danymi.
        products = {
            "products": [
                {
                    "id": "P001",
                    "qr_payload": "P001;3.5",
                    "weight": 3.5,
                    "shelf_cell": [2, 2],
                    "level": 0,
                    "slot": -0.25,
                    "position": [-5.75, 0.2, -3.5],
                    "access_cell": [2, 1],
                    "texture": "textures/qr/P001.png",
                    "present": True,
                    "world_qr": None,
                },
                {
                    "id": "P020",
                    "qr_payload": "P020;16.0",
                    "weight": 16.0,
                    "shelf_cell": [13, 9],
                    "level": 0,
                    "slot": -0.25,
                    "position": [5.25, 0.2, 3.5],
                    "access_cell": [13, 10],
                    "texture": "textures/qr/P020.png",
                    "present": True,
                    "world_qr": "P999;6.0",
                },
                {
                    "id": "P005",
                    "qr_payload": "P005;18.0",
                    "weight": 18.0,
                    "shelf_cell": [11, 2],
                    "level": 0,
                    "slot": -0.25,
                    "position": [3.25, 0.2, -3.5],
                    "access_cell": [11, 1],
                    "texture": "textures/qr/P005.png",
                    "present": False,
                    "world_qr": None,
                },
            ]
        }
        products_path = tmp_path / "products.json"
        products_path.write_text(
            json.dumps(products), encoding="utf-8"
        )

        # Zamówienie 1: pozycja ok, wrong_product (zły id na półce), missing.
        (orders_dir / "order_01.json").write_text(
            json.dumps(
                {
                    "order_id": "ORD-T01",
                    "items": [
                        {"product_id": "P001", "expected_weight": 3.5},
                        {"product_id": "P020", "expected_weight": 16.0},
                        {"product_id": "P005", "expected_weight": 18.0},
                    ],
                }
            ),
            encoding="utf-8",
        )
        # Zamówienie 2: wariant wagowy wrong_product oraz same ok.
        (orders_dir / "order_02.json").write_text(
            json.dumps(
                {
                    "order_id": "ORD-T02",
                    "items": [{"product_id": "P001", "expected_weight": 3.5}],
                }
            ),
            encoding="utf-8",
        )
        (orders_dir / "order_03.json").write_text(
            json.dumps(
                {
                    "order_id": "ORD-T03",
                    "items": [{"product_id": "P001", "expected_weight": 3.5}],
                }
            ),
            encoding="utf-8",
        )

        manager = OrderManager(
            str(orders_dir), str(products_path), str(reports_dir)
        )

        # --- Zamówienie mieszane → flaga red, dokładne komunikaty ---
        order = manager.next_order()
        assert order is not None and order.order_id == "ORD-T01"
        assert len(order.items) == 3
        # Wzbogacenie pozycji danymi z products.json.
        assert order.items[0].weight == 3.5
        assert order.items[0].shelf_cell == (2, 2)
        assert order.items[0].level == 0
        assert order.items[0].slot == -0.25
        assert order.items[0].access_cell == (2, 1)
        assert order.items[0].position == (-5.75, 0.2, -3.5)

        manager.mark_ok(order.items[0])
        manager.mark_wrong(order.items[1], "P999", 6.0)   # zły produkt na półce
        manager.mark_missing(order.items[2])               # pusta półka

        assert order.items[0].status == "ok"
        assert order.items[0].message == "pobrano pomyślnie"
        assert order.items[1].status == "wrong_product"
        assert (
            order.items[1].message
            == "Błąd lokalizacji [regał [13,9] poziom 0] - znaleziono P999 zamiast P020"
        ), order.items[1].message
        assert order.items[2].status == "missing"
        assert order.items[2].message == "brak produktu"

        report = manager.build_report(order, trips=1)
        assert report["order_id"] == "ORD-T01"
        assert report["flag"] == "red"
        assert report["trips"] == 1
        assert len(report["items"]) == 3
        assert report["items"][0]["weight"] == 3.5
        assert report["logs"], "raport powinien zawierać logi"
        datetime.fromisoformat(report["timestamp"])  # poprawny format ISO

        # Zapis raportu i odczyt pliku z dysku.
        saved_path = manager.save_report(report)
        assert saved_path.endswith("report_ORD-T01.json")
        with open(saved_path, encoding="utf-8") as f:
            loaded = json.load(f)
        assert loaded == report
        assert loaded["flag"] == "red"
        assert (
            loaded["items"][1]["message"]
            == "Błąd lokalizacji [regał [13,9] poziom 0] - znaleziono P999 zamiast P020"
        )

        # --- Zamówienie z niezgodną wagą → wariant wagowy komunikatu ---
        order = manager.next_order()
        assert order is not None and order.order_id == "ORD-T02"
        manager.mark_wrong(order.items[0], "P001", 4.0)  # id zgodne, waga nie
        assert (
            order.items[0].message
            == "Błąd wagi [regał [2,2] poziom 0] - oczekiwano 3.5 kg, jest 4.0 kg"
        ), order.items[0].message
        report = manager.build_report(order, trips=1)
        assert report["flag"] == "red"

        # --- Zamówienie samych sukcesów → flaga green ---
        order = manager.next_order()
        assert order is not None and order.order_id == "ORD-T03"
        manager.mark_ok(order.items[0])
        report = manager.build_report(order, trips=1)
        assert report["flag"] == "green"
        saved_path = manager.save_report(report)
        with open(saved_path, encoding="utf-8") as f:
            assert json.load(f)["flag"] == "green"

        # --- Koniec zamówień ---
        assert manager.next_order() is None

    print("Wszystkie testy order_manager przeszły pomyślnie.")
