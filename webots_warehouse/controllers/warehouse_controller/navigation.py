# -*- coding: utf-8 -*-
"""
navigation.py — Moduł nawigacji robota magazynowego (Webots).

PRZEZNACZENIE:
    Czysto pythonowa (bez zależności od Webots) warstwa planowania ruchu robota
    po siatce magazynu. Używana przez główny kontroler `warehouse_controller.py`
    do: (1) przeliczania współrzędnych komórki siatki na współrzędne świata
    Webots, (2) wyznaczania najkrótszej bezkolizyjnej ścieżki algorytmem A*
    oraz (3) planowania kursów kompletacji zamówienia z uwzględnieniem limitu
    udźwigu platformy (50 kg) i minimalizacji dystansu (greedy nearest-neighbour).

SPOSÓB UŻYCIA:
    from navigation import Navigator

    nav = Navigator(str(PROJECT_ROOT / "data" / "warehouse_layout.json"))
    x, z = nav.cell_to_world((2, 1))                      # komórka -> świat
    path = nav.find_path((7, 11), (2, 1))                 # A* po siatce
    trips = nav.plan_trips((7, 11), items)                # podział na kursy

    `items` to lista słowników: [{"id": "P001", "weight": 3.5,
    "access_cell": (2, 1)}, ...] — komórka dostępu (access cell), w której
    robot staje, aby pobrać produkt z danego regału.

STRUKTURY DANYCH:
    - Layout (wczytywany z `data/warehouse_layout.json`): rozmiar siatki
      (cols x rows), punkt odniesienia `origin` (x, z) środka komórki (0,0),
      lista komórek zablokowanych `blocked_cells` (regały), komórki terminala,
      limity wagowe (`max_platform_load_kg`).
    - Ścieżka: lista krotek (c, r) zawierająca komórkę startową i docelową;
      pusta lista oznacza brak drogi.
    - Kurs (trip): {"items": [pozycje w kolejności pobrania],
                    "targets": [(c, r), ...]  # komórki dostępu w tej kolejności,
                    "total_weight": float}    # suma wag kursu, <= 50 kg.

KLASY:
    Navigator — główna klasa modułu (patrz docstringi metod).

ALGORYTMY:
    - find_path: A* na siatce 4-kierunkowej (N/S/E/W), heurystyka Manhattan,
      pomija komórki zablokowane; deterministyczny (tie-breaking po współrzędnych).
    - plan_trips: zachłanny najbliższy sąsiad od bieżącej pozycji; gdy dodanie
      kolejnego produktu przekroczyłoby limit 50 kg, kurs jest zamykany,
      a nowy rozpoczyna się od terminala (komórka [7, 11]).

Moduł celowo nie importuje Webots — jest w pełni testowalny offline
(patrz blok `if __name__ == "__main__":` z testami na syntetycznym layoutcie).
"""

from __future__ import annotations

import heapq
import json
from pathlib import Path

# Komórka terminala, od której rozpoczyna się każdy kolejny kurs
# po rozładunku (zgodnie z kontraktem: terminal_cells = [[7, 11], [8, 11]]).
TERMINAL_CELL: tuple[int, int] = (7, 11)


class Navigator:
    """Planista nawigacji robota po siatce magazynu.

    Atrybuty:
        cols (int): liczba kolumn siatki.
        rows (int): liczba rzędów siatki.
        origin_x (float): współrzędna X środka komórki (0, 0) w świecie.
        origin_z (float): współrzędna Z środka komórki (0, 0) w świecie.
        blocked (set[tuple[int, int]]): zbiór zablokowanych komórek (regały).
        max_load (float): maksymalny udźwig platformy robota w kg.
    """

    def __init__(self, layout_path: str) -> None:
        """Wczytuje układ magazynu z pliku JSON.

        Args:
            layout_path: ścieżka do `data/warehouse_layout.json`.

        Raises:
            FileNotFoundError: gdy plik layoutu nie istnieje.
            KeyError: gdy plik nie zawiera wymaganych pól.
        """
        with open(layout_path, "r", encoding="utf-8") as f:
            layout = json.load(f)

        self.cols: int = int(layout["cols"])
        self.rows: int = int(layout["rows"])
        self.origin_x: float = float(layout["origin"]["x"])
        self.origin_z: float = float(layout["origin"]["z"])
        self.blocked: set[tuple[int, int]] = {
            (int(c), int(r)) for c, r in layout.get("blocked_cells", [])
        }
        self.max_load: float = float(layout.get("max_platform_load_kg", 50))

    # ------------------------------------------------------------------
    # Konwersja współrzędnych
    # ------------------------------------------------------------------
    def cell_to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        """Przelicza komórkę siatki na współrzędne świata Webots (x, z).

        Zgodnie z kontraktem: x = origin.x + c, z = origin.z + r
        (środek komórki; podłoga to płaszczyzna XZ).

        Args:
            cell: krotka (kolumna, rząd).

        Returns:
            Krotka (x, z) — środek komórki w metrach.
        """
        c, r = cell
        return (self.origin_x + c, self.origin_z + r)

    # ------------------------------------------------------------------
    # Pomocnicze
    # ------------------------------------------------------------------
    def _in_bounds(self, cell: tuple[int, int]) -> bool:
        """Sprawdza, czy komórka mieści się w siatce magazynu."""
        c, r = cell
        return 0 <= c < self.cols and 0 <= r < self.rows

    def _is_free(self, cell: tuple[int, int]) -> bool:
        """Sprawdza, czy komórka jest w siatce i nie jest zablokowana."""
        return self._in_bounds(cell) and cell not in self.blocked

    @staticmethod
    def _manhattan(a: tuple[int, int], b: tuple[int, int]) -> int:
        """Odległość Manhattan między komórkami (heurystyka A*)."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _neighbors(self, cell: tuple[int, int]) -> list[tuple[int, int]]:
        """Zwraca wolnych sąsiadów komórki w 4 kierunkach (N, S, E, W)."""
        c, r = cell
        result = []
        for n in ((c, r - 1), (c, r + 1), (c + 1, r), (c - 1, r)):
            if self._is_free(n):
                result.append(n)
        return result

    # ------------------------------------------------------------------
    # A*
    # ------------------------------------------------------------------
    def find_path(
        self, start: tuple[int, int], goal: tuple[int, int]
    ) -> list[tuple[int, int]]:
        """Wyznacza najkrótszą ścieżkę A* między komórkami (4 kierunki).

        Args:
            start: komórka startowa (c, r).
            goal: komórka docelowa (c, r).

        Returns:
            Lista komórek od startu do celu (włącznie z oboma).
            Pusta lista, gdy start/cel jest zablokowany, poza siatką
            albo droga nie istnieje.
        """
        if not self._is_free(start) or not self._is_free(goal):
            return []
        if start == goal:
            return [start]

        # Kolejka priorytetowa: (f = g + h, licznik dla determinizmu, komórka).
        counter = 0
        open_heap: list[tuple[int, int, tuple[int, int]]] = []
        heapq.heappush(open_heap, (self._manhattan(start, goal), counter, start))
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        g_score: dict[tuple[int, int], int] = {start: 0}
        closed: set[tuple[int, int]] = set()

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if current == goal:
                # Rekonstrukcja ścieżki od celu do startu.
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            closed.add(current)
            for neighbor in self._neighbors(current):
                if neighbor in closed:
                    continue
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, 1 << 30):
                    g_score[neighbor] = tentative_g
                    came_from[neighbor] = current
                    counter += 1
                    f = tentative_g + self._manhattan(neighbor, goal)
                    heapq.heappush(open_heap, (f, counter, neighbor))

        return []  # brak drogi

    # ------------------------------------------------------------------
    # Planowanie kursów kompletacji
    # ------------------------------------------------------------------
    def plan_trips(
        self, start_cell: tuple[int, int], items: list[dict]
    ) -> list[dict]:
        """Dzieli pozycje zamówienia na kursy z limitem udźwigu 50 kg.

        Algorytm zachłanny (greedy nearest-neighbour): od bieżącej pozycji
        wybierany jest najbliższy produkt (odległość Manhattan do komórki
        dostępu), którego waga nie przekroczy limitu platformy. Gdy żaden
        z pozostałych produktów się nie mieści, kurs jest zamykany,
        a kolejny rozpoczyna się od terminala [7, 11] (robot wraca
        rozładować towar i kontynuuje to samo zamówienie).

        Args:
            start_cell: bieżąca komórka robota (start pierwszego kursu).
            items: lista słowników {"id": str, "weight": float,
                "access_cell": (c, r)}.

        Returns:
            Lista kursów w kolejności realizacji:
            [{"items": [...], "targets": [(c, r), ...],
              "total_weight": float}, ...].
            Kolejność pozycji i celów w kursie minimalizuje dystans (NN).
            Dla pustej listy `items` zwraca [].

        Raises:
            ValueError: gdy pojedynczy produkt przekracza limit udźwigu.
        """
        remaining = list(items)
        for item in remaining:
            if float(item["weight"]) > self.max_load:
                raise ValueError(
                    f"Produkt {item.get('id')} waży {item['weight']} kg "
                    f"i przekracza limit platformy {self.max_load} kg"
                )

        trips: list[dict] = []
        position = start_cell

        while remaining:
            trip_items: list[dict] = []
            trip_targets: list[tuple[int, int]] = []
            trip_weight = 0.0

            while True:
                # Najbliższy produkt, który jeszcze mieści się na platformie.
                best_idx = -1
                best_dist = 1 << 30
                for idx, item in enumerate(remaining):
                    weight = float(item["weight"])
                    if trip_weight + weight > self.max_load:
                        continue
                    target = tuple(item["access_cell"])
                    dist = self._manhattan(position, target)
                    # Tie-breaking: mniejszy dystans, potem id (determinizm).
                    if dist < best_dist or (
                        dist == best_dist
                        and best_idx >= 0
                        and str(item.get("id")) < str(remaining[best_idx].get("id"))
                    ):
                        best_dist = dist
                        best_idx = idx

                if best_idx < 0:
                    break  # nic więcej nie zmieści się w tym kursie

                chosen = remaining.pop(best_idx)
                position = tuple(chosen["access_cell"])
                trip_items.append(chosen)
                trip_targets.append(position)
                trip_weight += float(chosen["weight"])

            trips.append(
                {
                    "items": trip_items,
                    "targets": trip_targets,
                    "total_weight": trip_weight,
                }
            )
            # Kolejny kurs zaczyna się od terminala (powrót na rozładunek).
            position = TERMINAL_CELL

        return trips


# ======================================================================
# Testy offline (syntetyczny layout zgodny z kontraktem integracyjnym)
# ======================================================================
if __name__ == "__main__":
    import tempfile

    def _build_synthetic_layout() -> dict:
        """Buduje syntetyczny layout identyczny z kontraktem projektu.

        Siatka 16x12; regały: rzędy {2,3,5,6,8,9} x kolumny {2..6, 9..13}
        (60 zablokowanych komórek); origin (-7.5, -5.5); terminal [7,11]/[8,11].
        """
        blocked = [
            [c, r]
            for r in (2, 3, 5, 6, 8, 9)
            for c in (2, 3, 4, 5, 6, 9, 10, 11, 12, 13)
        ]
        return {
            "cell_size": 1.0,
            "cols": 16,
            "rows": 12,
            "origin": {"x": -7.5, "z": -5.5},
            "cell_to_world": "x = origin.x + c; z = origin.z + r",
            "terminal_cells": [[7, 11], [8, 11]],
            "terminal_world": [0.0, 5.5],
            "robot_start": {"cell": [7, 11], "world": [-0.5, 5.5], "heading_deg": 0},
            "blocked_cells": blocked,
            "shelf_rows": {"north_access": [2, 5, 8], "south_access": [3, 6, 9]},
            "levels": [0.2, 0.8, 1.4],
            "slots": [-0.25, 0.25],
            "access_rule": "r in {2,5,8} -> (c,r-1); r in {3,6,9} -> (c,r+1)",
            "max_platform_load_kg": 50,
            "max_package_weight_kg": 20,
        }

    # Preferuj prawdziwy plik layoutu, jeśli już istnieje; inaczej syntetyk.
    real_layout = Path(__file__).resolve().parents[2] / "data" / "warehouse_layout.json"
    if real_layout.exists():
        layout_path = str(real_layout)
        print(f"[TEST] Używam prawdziwego layoutu: {layout_path}")
        nav = Navigator(layout_path)
    else:
        tmp = tempfile.NamedTemporaryFile(
            "w", suffix=".json", delete=False, encoding="utf-8"
        )
        json.dump(_build_synthetic_layout(), tmp)
        tmp.close()
        layout_path = tmp.name
        print(f"[TEST] Używam syntetycznego layoutu: {layout_path}")
        nav = Navigator(layout_path)

    failures = 0

    def check(name: str, condition: bool, detail: str = "") -> None:
        """Prosta asercja testowa z raportowaniem wyniku."""
        global failures
        status = "OK " if condition else "FAIL"
        if not condition:
            failures += 1
        print(f"[{status}] {name}" + (f" — {detail}" if detail else ""))

    # --- Test 0: konwersja komórka -> świat (sanity check kontraktu) ---
    x, z = nav.cell_to_world((7, 11))
    check("cell_to_world (7,11) -> (-0.5, 5.5)", (x, z) == (-0.5, 5.5), f"({x}, {z})")

    # --- Test 1: find_path omija zablokowane komórki ---
    # Cel (6, 1) — komórka dostępu do regału rzędu 2; trasa z terminala
    # musi ominąć wszystkie 60 regałów.
    path = nav.find_path((7, 11), (6, 1))
    check("find_path znajduje drogę (7,11)->(6,1)", len(path) > 0)
    check(
        "ścieżka zawiera start i cel",
        bool(path) and path[0] == (7, 11) and path[-1] == (6, 1),
    )
    check(
        "ścieżka nie wchodzi na regały",
        all(p not in nav.blocked for p in path),
    )
    check(
        "ścieżka jest ciągła (kroki o 1 komórkę)",
        all(
            abs(path[i][0] - path[i + 1][0]) + abs(path[i][1] - path[i + 1][1]) == 1
            for i in range(len(path) - 1)
        ),
    )
    expected_len = abs(7 - 6) + abs(11 - 1) + 1  # Manhattan + 1 (trasę da się poprowadzić bez omiatania)
    check(
        "ścieżka ma długość optymalną (Manhattan)",
        len(path) == expected_len,
        f"len={len(path)}, oczekiwano {expected_len}",
    )

    # --- Test 2: find_path do komórki zablokowanej -> [] ---
    check(
        "find_path do regału (2,2) -> []",
        nav.find_path((7, 11), (2, 2)) == [],
    )
    check(
        "find_path z regału (2,2) -> []",
        nav.find_path((2, 2), (7, 11)) == [],
    )

    # --- Test 3: plan_trips dla 85 kg -> >= 2 kursy, żaden > 50 kg ---
    # ORD-002 z kontraktu: P003, P006, P008, P010, P013, P016, P018 (85 kg).
    def access_cell(cell: tuple[int, int]) -> tuple[int, int]:
        """Reguła dostępu z kontraktu: r w {2,5,8} -> (c,r-1); r w {3,6,9} -> (c,r+1)."""
        c, r = cell
        return (c, r - 1) if r in (2, 5, 8) else (c, r + 1)

    heavy_items = [
        {"id": "P003", "weight": 12.0, "access_cell": access_cell((6, 2))},
        {"id": "P006", "weight": 9.5, "access_cell": access_cell((13, 2))},
        {"id": "P008", "weight": 15.0, "access_cell": access_cell((5, 3))},
        {"id": "P010", "weight": 11.0, "access_cell": access_cell((13, 3))},
        {"id": "P013", "weight": 14.0, "access_cell": access_cell((9, 5))},
        {"id": "P016", "weight": 10.5, "access_cell": access_cell((11, 6))},
        {"id": "P018", "weight": 13.0, "access_cell": access_cell((9, 8))},
    ]
    trips = nav.plan_trips((7, 11), heavy_items)
    total_planned = sum(t["total_weight"] for t in trips)
    check("plan_trips zwraca >= 2 kursy dla 85 kg", len(trips) >= 2, f"kursy={len(trips)}")
    check(
        "żaden kurs nie przekracza 50 kg",
        all(t["total_weight"] <= 50.0 for t in trips),
        str([round(t["total_weight"], 1) for t in trips]),
    )
    check(
        "suma wag kursów = 85 kg",
        abs(total_planned - 85.0) < 1e-9,
        f"suma={total_planned}",
    )
    check(
        "wszystkie 7 produktów zaplanowane",
        sum(len(t["items"]) for t in trips) == 7,
    )
    check(
        "targets zgodne z items (kolejność 1:1)",
        all(
            t["targets"] == [tuple(i["access_cell"]) for i in t["items"]]
            for t in trips
        ),
    )

    # --- Test 4: kolejność celów w kursie jest sensowna dystansowo (NN) ---
    # W każdym kursie każdy kolejny cel musi być najbliższym (Manhattan)
    # spośród celów pozostałych w tym kursie, licząc od poprzedniej pozycji.
    nn_ok = True
    for trip in trips:
        pos = (7, 11)  # każdy kurs startuje od terminala (kurs 1: od start_cell=(7,11))
        remaining_targets = list(trip["targets"])
        for target in trip["targets"]:
            dists = [Navigator._manhattan(pos, t) for t in remaining_targets]
            if Navigator._manhattan(pos, target) != min(dists):
                nn_ok = False
            remaining_targets.remove(target)
            pos = target
    check("kolejność celów w kursach jest NN-optymalna", nn_ok)

    # --- Test 5 (dodatkowy): pusta lista pozycji ---
    check("plan_trips dla pustej listy -> []", nav.plan_trips((7, 11), []) == [])

    # --- Test 6 (dodatkowy): produkt cięższy niż limit -> ValueError ---
    try:
        nav.plan_trips((7, 11), [{"id": "PX", "weight": 60.0, "access_cell": (1, 1)}])
        check("produkt > 50 kg zgłasza ValueError", False)
    except ValueError:
        check("produkt > 50 kg zgłasza ValueError", True)

    print()
    if failures:
        print(f"[TEST] WYNIK: {failures} test(ów) NIE POWIODŁO SIĘ")
        raise SystemExit(1)
    print("[TEST] WYNIK: wszystkie testy przeszły pomyślnie")
