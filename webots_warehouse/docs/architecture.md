<!--
================================================================================
PLIK: webots_warehouse/docs/architecture.md

PRZEZNACZENIE:
    Dokument architektury projektu "Symulacyjny system obsługi zamówień
    magazynowych w Webots". Opisuje rolę każdego pliku projektu, maszynę
    stanów FSM kontrolera, konwencje współrzędnych i siatki nawigacyjnej,
    schematy wszystkich plików JSON (z przykładami), API publiczne modułów
    Pythona, model robota i jego urządzenia, mechanizm podziału zamówienia
    na kursy (limit 50 kg) oraz format komunikatów błędów w raportach.

SPOSÓB UŻYCIA:
    Dokument referencyjny dla deweloperów i recenzentów. Wszystkie nazwy
    własne (DEF-e, nazwy urządzeń, sygnatury funkcji, komunikaty) są
    SZTYWNYM kontraktem integracyjnym — zmiana któregokolwiek elementu
    wymaga spójnej zmiany we wszystkich plikach projektu.

STRUKTURA DOKUMENTU:
    1. Przegląd plików projektu
    2. Konwencje współrzędnych i siatka nawigacyjna
    3. Model robota i urządzenia
    4. Maszyna stanów FSM (diagram ASCII)
    5. API modułów Pythona
    6. Schematy JSON z przykładami
    7. Mechanizm split-order (limit 50 kg)
    8. Format komunikatów błędów
================================================================================
-->

# Architektura systemu

## 1. Przegląd plików projektu

| Plik | Rola |
|------|------|
| `worlds/warehouse.wbt` | Świat symulacji Webots R2025a: hala 18×14 m (`RectangleArena`), 60 regałów (`SHELF_c_r`), 19 boxów produktów (`PRODUCT_<id>`), terminal (`TERMINAL`), 3 przeszkody dynamiczne (`OBSTACLE_1..3`), oświetlenie bez cieni (12× `PointLight` z kwadratowym tłumieniem), instancja robota z `WarehouseRobot.proto`, robot-pomost `MCP_BRIDGE`. Węzły `TexturedBackground` / `TexturedBackgroundLight` / `RectangleArena` deklarowane przez `EXTERNPROTO` (wymóg od R2023b+). **Plik generowany — nie edytować ręcznie.** `WorldInfo.basicTimeStep = 32`. |
| `protos/WarehouseRobot.proto` | Definicja PROTO robota: napęd różnicowy (2 koła r = 0.08 m na `HingeJoint` + `RotationalMotor`), podnosnik z `LinearMotor`, chwytak dwupalcy, kamera, 5 czujników odległości, platforma odkładcza 0.5×0.5 m. Pole `supervisor = TRUE`. |
| `textures/qr/P001.png … P020.png, P999.png` | Tekstury kodów QR 512×512 px (potęga dwójki — wymóg Webots) naklejone na boxy produktów. `P999.png` to kod "złego produktu" umieszczony na półce P020. |
| `data/warehouse_layout.json` | Geometria magazynu: siatka, origin, komórki zablokowane, terminal, start robota, wysokości poziomów, sloty, reguła dostępu, limity wagowe. |
| `data/products.json` | Katalog 20 produktów: pozycja, waga, payload QR, komórka dostępu, flagi anomalii (`present`, `world_qr`). |
| `orders/order_01..03.json` | Zamówienia testowe ORD-001, ORD-002, ORD-003. |
| `reports/` | Katalog tworzony w runtime; kontroler zapisuje tu `report_<order_id>.json`. |
| `tools/generate_assets.py` | Generator tekstur QR (`qrcode` + `pillow`, 512×512 px) i plików danych spójnych z kontraktem. |
| `tools/generate_world.py` | Generator świata `worlds/warehouse.wbt` z `data/*.json` — źródło prawdy o strukturze świata; odtwarza plik po przypadkowym nadpisaniu przez Webots. |
| `tools/validate_project.py` | Walidator spójności: świat ↔ `products.json` ↔ zamówienia ↔ tekstury. |
| `controllers/mcp_bridge/` | Kontroler-pomost MCP (z repo [webotMCP](https://github.com/MHasaan/webotMCP)): supervisor wystawiający symulację po TCP (porty 10022/10023) dla serwera MCP. |
| `controllers/mcp_robot/` | Generyczny kontroler-agent MCP, doczepiany do dowolnego robota (opcjonalny, z repo webotMCP). |
| `controllers/warehouse_controller/warehouse_controller.py` | Główny kontroler robota — maszyna stanów FSM, zamknięta pętla nawigacji po Supervisor API, pobieranie/rozładunek kinematyczny. |
| `controllers/warehouse_controller/navigation.py` | Siatka nawigacyjna, konwersja współrzędnych, A* (4 kierunki), planowanie kursów z limitem 50 kg. |
| `controllers/warehouse_controller/qr_scanner.py` | Odczyt kodu QR z obrazu kamery Webots przez `cv2.QRCodeDetector`; parser payloadu. |
| `controllers/warehouse_controller/order_manager.py` | Wczytywanie zamówień, statusowanie pozycji, budowa i zapis raportów JSON. |

Korzeń projektu z poziomu kontrolerów:
`Path(__file__).resolve().parents[2]`.

## 2. Konwencje współrzędnych i siatka nawigacyjna

### 2.1 Układ współrzędnych Webots

- Podłoga = płaszczyzna **XZ**, oś **Y w górę**.
- Hala: `RectangleArena` o `floorSize` 18×14 m, ściany wysokości 2.5 m.
- Rotacje axis-angle wokół osi Y; dodatni kąt = obrót przeciwnie do wskazówek
  zegara patrząc z góry (zgodnie z regułą prawej dłoni dla +Y).

### 2.2 Siatka nawigacyjna

- **16 kolumn × 12 rzędów**, komórka kwadratowa **1.0 m**.
- Środek komórki `(c, r)` w świecie:

  ```
  x = −7.5 + c
  z = −5.5 + r
  ```

- Komórka `[0,0]` to południowo-zachodni (−X, −Z) narożnik siatki.

### 2.3 Regały i komórki dostępu

- **Komórki regałów (zablokowane dla nawigacji):** rzędy
  `r ∈ {2, 3, 5, 6, 8, 9}` × kolumny `c ∈ {2, 3, 4, 5, 6, 9, 10, 11, 12, 13}`
  — łącznie 60 regałów, DEF `SHELF_c_r`.
- Regał: bryła 0.9 × 1.8 × 0.9 m (X × Y × Z), 3 poziomy półek.
- **Wysokości środków produktów:** `levels = [0.2, 0.8, 1.4]` m.
- **Sloty na poziomie:** 2 sztuki, offset `±0.25` m wzdłuż osi X.
- **Reguła dostępu (access cell)** — komórka, na której staje robot:

  | Rzęd regału | Komórka dostępu | Orientacja robota |
  |-------------|-----------------|-------------------|
  | `r ∈ {2, 5, 8}` (rząd "północny" pary) | `(c, r − 1)` | przód na południe (+Z) |
  | `r ∈ {3, 6, 9}` (rząd "południowy" pary) | `(c, r + 1)` | przód na północ (−Z) |

  Tekstura QR znajduje się na ścianie produktu zwróconej do alei dostępu.

### 2.4 Terminal i start robota

- Terminal: komórki `[[7,11], [8,11]]`, środek strefy `(0.0, 5.5)`,
  zielony patch 2×2 m na podłodze + kiosk DEF `TERMINAL`.
- Start robota: komórka `[7,11]` → pozycja `(−0.5, 0, 5.5)`,
  rotacja `0 1 0 0` — przód robota na północ (−Z).

### 2.5 Przeszkody dynamiczne

Trzy kostki 0.4 m z fizyką (DEF `OBSTACLE_1`, `OBSTACLE_2`, `OBSTACLE_3`)
na alejach, na pozycjach `(0.5, −1.5)`, `(−4.5, 1.5)`, `(2.5, 4.5)`.
Kontroler **nie zna** ich pozycji — wykrywa je wyłącznie czujnikami
odległości (stan `AVOID_OBSTACLE`).

### 2.6 Orientacja robota i heading

- **Lokalny przód robota = −Z.** Dodatnia prędkość obu kół = jazda do przodu.
- Heading z pola `rotation` (axis-angle wokół Y): jeśli `axis.y < 0`,
  kąt przyjmuje się ze znakiem przeciwnym (`kąt = −kąt`).
- Wektor przodu w świecie: `(−sin θ, 0, −cos θ)`; `θ = 0` → przód na −Z
  (północ).

## 3. Model robota i urządzenia

Zdefiniowany w `protos/WarehouseRobot.proto`; instancja w świecie:

```
WarehouseRobot {
  translation -0.5 0 5.5
  rotation 0 1 0 0
  controller "warehouse_controller"
}
```

- Początek układu PROTO na poziomie podłogi (`y = 0` = kontakt kół
  z podłożem); masa robota ~30 kg.
- Napęd różnicowy: 2 koła o promieniu 0.08 m na `HingeJoint` +
  `RotationalMotor`; stabilność zapewniają wpusty/ślizgi w bryle korpusu.
- Podnosnik z kamerą na głowicy: `LinearMotor`, zakres 0–1.6 m. Kamera jest
  zamontowana na wysokości `0.2 + lift`, więc poziom `L` wymaga
  `lift = levels[L] − 0.2`.
- Platforma odkładcza na korpusie 0.5×0.5 m — **limit logiczny 50 kg**
  (wymuszany przez kontroler, nie przez fizykę). Sloty platformy
  dla kolejnych produktów: offsety `(±0.15, 0.45 + 0.3·k, ±0.15)` w układzie
  robota (k = indeks produktu na platformie).
- Maksymalna waga pojedynczej paczki: 20 kg (`max_package_weight_kg`).

### Nazwy urządzeń (kontrakt SZTYWNY)

| Nazwa | Typ | Przeznaczenie |
|-------|-----|---------------|
| `wheel_left` | RotationalMotor | lewe koło napędowe |
| `wheel_right` | RotationalMotor | prawe koło napędowe |
| `lift_motor` | LinearMotor | podnosnik głowicy z kamerą (0–1.6 m) |
| `gripper_left` | RotationalMotor | lewy palec chwytaka |
| `gripper_right` | RotationalMotor | prawy palec chwytaka |
| `camera` | Camera | 640×480 px, fov 1.0 rad, na głowicy, patrzy na lokalne −Z |
| `ds_front_left` | DistanceSensor | przód-lewo (wykrywanie przeszkód) |
| `ds_front_right` | DistanceSensor | przód-prawo (wykrywanie przeszkód) |
| `ds_left` | DistanceSensor | bok lewy |
| `ds_right` | DistanceSensor | bok prawy |
| `ds_back` | DistanceSensor | tył (bezpieczeństwo cofania) |

Robot ma pole `supervisor TRUE`, więc kontroler importuje
`from controller import Robot, Supervisor` i pracuje na instancji
`Supervisor` — umożliwia to odczyt pozy
(`getSelf().getField("translation" / "rotation")`) oraz kinematyczne
przenoszenie produktów (ustawianie ich pól `translation`).

### Oświetlenie świata

`TexturedBackground` + `TexturedBackgroundLight`, `DirectionalLight`
z `castShadows FALSE` oraz kilka `PointLight`. Jasne, równomierne
oświetlenie bez cieni jest warunkiem czytelności kodów QR
(eliminuje błędy `cv2.QRCodeDetector` wynikające z kontrastu i cieni).

## 4. Maszyna stanów FSM

Kontroler `warehouse_controller.py` jest maszyną stanów o stanach:

`INIT, LOAD_ORDER, PLAN_TRIP, NAVIGATE, AVOID_OBSTACLE, ALIGN_AND_SCAN,
VERIFY_ITEM, PICK_ITEM, RETURN_TO_TERMINAL, UNLOAD, REPORT, DONE`

Przejścia są logowane po polsku, np. `[FSM] Stan: NAVIGATE`.

```
                    ┌──────────┐
                    │   INIT   │  inicjalizacja urządzeń, wczytanie layoutu
                    └────┬─────┘  i katalogu produktów
                         ▼
              ┌────────────────────┐
        ┌─────│    LOAD_ORDER      │◄───────────────────────────┐
        │     └─────────┬──────────┘                            │
        │               ▼                                       │
        │     ┌────────────────────┐                            │
        │     │     PLAN_TRIP      │  plan_trips: greedy NN,    │
        │     └─────────┬──────────┘  limit 50 kg na kurs        │
        │               ▼                                       │
        │     ┌────────────────────┐    przeszkoda wykryta      │
        │  ┌─►│      NAVIGATE      │───────────┐                │
        │  │  └─────────┬──────────┘           ▼                │
        │  │            ▼              ┌────────────────────┐   │
        │  │  ┌────────────────────┐  │  AVOID_OBSTACLE    │   │
        │  │  │   ALIGN_AND_SCAN   │  │ stop, cofnij 0.3m, │   │
        │  │  │ lift → poziom,     │  │ zablokuj komórkę,  │   │
        │  │  │ skan QR (3 próby)  │  │ przeplanuj ścieżkę │   │
        │  │  └─────────┬──────────┘  └─────────┬──────────┘   │
        │  │            ▼                       │              │
        │  │  ┌────────────────────┐            │              │
        │  │  │    VERIFY_ITEM     │            │              │
        │  │  └─┬───────┬───────┬──┘            │              │
        │  │    │ok     │wrong  │missing        │              │
        │  │    ▼       │       │               │              │
        │  │ ┌────────┐ │       │               │              │
        │  └─┤PICK_ITEM◄┤       │               │              │
        │    └───┬────┘ │       │               │              │
        │        │      ▼       ▼               │              │
        │        │   (pozycja oznaczona wrong_product/missing  │
        │        │    — pomijamy, jedziemy do następnego celu) │
        │        │                                            │
        │        ▼  wszystkie cele kursu obsłużone            │
        │  ┌────────────────────┐                             │
        │  │ RETURN_TO_TERMINAL │                             │
        │  └─────────┬──────────┘                             │
        │            ▼                                        │
        │  ┌────────────────────┐                             │
        │  │       UNLOAD       │  teleport produktów do      │
        │  └─────────┬──────────┘  strefy terminala           │
        │            ▼                                        │
        │     zostały pozycje w zamówieniu? ──tak──► PLAN_TRIP┘
        │            │nie
        │            ▼
        │  ┌────────────────────┐
        │  │       REPORT       │  zapis reports/report_<id>.json
        │  └─────────┬──────────┘
        └────────────┘  następne zamówienie → LOAD_ORDER
                     brak zamówień
                         ▼
                  ┌──────────┐
                  │   DONE   │
                  └──────────┘
```

Uwagi do stanów:

- **NAVIGATE** — ruch w zamkniętej pętli po pozie z Supervisor API; obrót
  w miejscu do kierunku kolejnego waypointa, potem jazda prosto. Ścieżka
  pochodzi z `Navigator.find_path` (A*, 4 kierunki).
- **AVOID_OBSTACLE** — wyzwalany, gdy `ds_front_left` / `ds_front_right` /
  `ds_left` / `ds_right` przekroczą próg; po cofnięciu 0.3 m komórka przed
  robotem jest oznaczana jako chwilowo zablokowana i ścieżka jest
  przeplanowywana; następnie powrót do `NAVIGATE`.
- **ALIGN_AND_SCAN** — podnosnik ustawiany na `lift = levels[L] − 0.2`;
  skan QR z maks. 3 próbami.
- **VERIFY_ITEM** — porównanie payloadu QR z zamówieniem: identyfikator
  i waga. Statusy pozycji: `ok`, `wrong_product`, `missing`.
- **PICK_ITEM** — zamknięcie chwytaka, kinematyczne przypięcie produktu
  do slotu platformy (Supervisor co krok ustawia `translation` produktu
  względem pozy robota), zwolnienie chwytaka. Jeżeli kolejny produkt
  z planu przekroczyłby 50 kg ładunku, następuje wcześniejsze
  `RETURN_TO_TERMINAL` → `UNLOAD` i kontynuacja tego samego zamówienia.
- **UNLOAD** — teleport noszonych produktów do rządka przy strefie terminala
  (okolice `(0.5, 5.0 ± …)`), wyzerowanie ładunku.

## 5. API modułów Pythona

Sygnatury są SZTYWNE — kontroler główny jest kodowany przeciwko nim.

### 5.1 `navigation.py`

```python
class Navigator:
    def __init__(self, layout_path: str) -> None
    def cell_to_world(self, cell: tuple[int, int]) -> tuple[float, float]
        # (x, z) środka komórki; x = origin.x + c, z = origin.z + r
    def find_path(self, start: tuple[int, int], goal: tuple[int, int]
                  ) -> list[tuple[int, int]]
        # A*, 4 kierunki; lista zawiera start i goal; [] gdy brak ścieżki
    def plan_trips(self, start_cell: tuple[int, int], items: list[dict]
                   ) -> list[dict]
        # items: [{"id", "weight", "access_cell": (c, r)}]
        # greedy nearest-neighbour + limit 50 kg na kurs;
        # zwraca [{"items": [...], "targets": [(c, r), ...],
        #          "total_weight": float}]
        # — kolejność celów minimalizuje dystans
```

### 5.2 `qr_scanner.py`

```python
class QRScanner:
    def __init__(self, camera) -> None   # webots Camera (enable(timestep))
    def scan(self) -> str | None
        # payload QR lub None; cv2.QRCodeDetector;
        # obraz: np.frombuffer(camera.getImage(), np.uint8)
        #        .reshape((h, w, 4)) → cv2.cvtColor(..., cv2.COLOR_BGRA2BGR)

def parse_payload(payload: str) -> tuple[str, float]
    # "P001;3.5" -> ("P001", 3.5); ValueError przy złym formacie
```

### 5.3 `order_manager.py`

```python
class OrderItem:
    # pola: product_id, expected_weight, weight, shelf_cell, level, slot,
    #       access_cell, position,
    #       status ("pending" | "ok" | "wrong_product" | "missing"), message

class Order:
    # pola: order_id, items: list[OrderItem]

class OrderManager:
    def __init__(self, orders_dir: str, products_path: str,
                 reports_dir: str) -> None
    def next_order(self) -> Order | None   # kolejno wg nazwy pliku; None gdy koniec
    def mark_ok(self, item: OrderItem) -> None
    def mark_wrong(self, item: OrderItem, found_id: str,
                   found_weight: float | None) -> None
    def mark_missing(self, item: OrderItem) -> None
    def build_report(self, order: Order, trips: int) -> dict
    def save_report(self, report: dict) -> str   # zwraca ścieżkę pliku
```

## 6. Schematy JSON z przykładami

### 6.1 `data/warehouse_layout.json`

```json
{
  "cell_size": 1.0,
  "cols": 16,
  "rows": 12,
  "origin": {"x": -7.5, "z": -5.5},
  "cell_to_world": "x = origin.x + c; z = origin.z + r",
  "terminal_cells": [[7, 11], [8, 11]],
  "terminal_world": [0.0, 5.5],
  "robot_start": {"cell": [7, 11], "world": [-0.5, 5.5], "heading_deg": 0},
  "blocked_cells": [[2, 2], [3, 2], "… 60 komórek …"],
  "shelf_rows": {"north_access": [2, 5, 8], "south_access": [3, 6, 9]},
  "levels": [0.2, 0.8, 1.4],
  "slots": [-0.25, 0.25],
  "access_rule": "r in {2,5,8} -> (c,r-1); r in {3,6,9} -> (c,r+1)",
  "max_platform_load_kg": 50,
  "max_package_weight_kg": 20
}
```

### 6.2 `data/products.json`

Format pozycji światowej: `x = −7.5 + c + slot`, `z = −5.5 + r`,
`y = levels[level]`. Payload QR: `<id>;<waga>`. DEF w świecie:
`PRODUCT_<id>`, tekstura `textures/qr/<id>.png` (w `.wbt`:
`../textures/qr/P001.png`).

```json
{
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
      "present": true,
      "world_qr": null
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
      "present": false,
      "world_qr": null
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
      "present": true,
      "world_qr": "P999;6.0"
    }
  ]
}
```

- `present: false` (P005) — w świecie nie ma boxa; półka jest pusta
  (test statusu `missing`).
- `world_qr` (P020) — fizyczny box `PRODUCT_P020` nosi teksturę `P999.png`
  z payloadem `P999;6.0` (test statusu `wrong_product`).

### 6.3 Tabela produktów (pełna)

| id | cell | level | slot | waga [kg] | Uwagi |
|----|------|-------|------|-----------|-------|
| P001 | [2,2]  | 0 | −0.25 | 3.5  | |
| P002 | [4,2]  | 1 | +0.25 | 7.0  | |
| P003 | [6,2]  | 2 | −0.25 | 12.0 | |
| P004 | [9,2]  | 1 | +0.25 | 5.0  | |
| P005 | [11,2] | 0 | −0.25 | 18.0 | **present=false — pusta półka** |
| P006 | [13,2] | 2 | +0.25 | 9.5  | |
| P007 | [2,3]  | 1 | −0.25 | 4.0  | |
| P008 | [5,3]  | 0 | +0.25 | 15.0 | |
| P009 | [10,3] | 2 | −0.25 | 6.5  | |
| P010 | [13,3] | 1 | +0.25 | 11.0 | |
| P011 | [2,5]  | 2 | −0.25 | 2.5  | |
| P012 | [6,5]  | 0 | +0.25 | 8.0  | |
| P013 | [9,5]  | 1 | −0.25 | 14.0 | |
| P014 | [12,5] | 0 | +0.25 | 19.0 | |
| P015 | [3,6]  | 1 | −0.25 | 3.0  | |
| P016 | [11,6] | 2 | +0.25 | 10.5 | |
| P017 | [5,8]  | 0 | −0.25 | 5.5  | |
| P018 | [9,8]  | 2 | +0.25 | 13.0 | |
| P019 | [2,9]  | 1 | −0.25 | 7.5  | |
| P020 | [13,9] | 0 | −0.25 | 16.0 | **world_qr=`P999;6.0` — zły produkt na półce** |

### 6.4 `orders/order_XX.json`

```json
{
  "order_id": "ORD-001",
  "items": [
    {"product_id": "P001", "expected_weight": 3.5},
    {"product_id": "P007", "expected_weight": 4.0},
    {"product_id": "P011", "expected_weight": 2.5},
    {"product_id": "P015", "expected_weight": 3.0}
  ]
}
```

Zamówienia testowe:

- **ORD-001** — zielone, 1 kurs: P001, P007, P011, P015 (razem 13 kg).
- **ORD-002** — zielone, split >50 kg: P003, P006, P008, P010, P013, P016,
  P018 (razem 85 kg → minimum 2 kursy).
- **ORD-003** — czerwone: P002, P005 (brak na półce), P020 (na półce box
  z kodem P999), P019.

### 6.5 `reports/report_<order_id>.json`

```json
{
  "order_id": "ORD-001",
  "flag": "green",
  "trips": 1,
  "timestamp": "2024-05-17T14:32:05.123456",
  "items": [
    {
      "product_id": "P001",
      "status": "ok",
      "weight": 3.5,
      "message": "pobrano pomyślnie"
    }
  ],
  "logs": ["[FSM] Stan: INIT", "[FSM] Stan: LOAD_ORDER", "…"]
}
```

- `flag`: `"green"` gdy wszystkie pozycje mają status `ok`;
  `"red"` gdy cokolwiek ≠ `ok`.
- `trips`: liczba kursów (powrotów do terminala z towarem).
- `timestamp`: znacznik czasu w formacie ISO.
- `logs`: logi kontrolera z przebiegu zamówienia.

## 7. Mechanizm split-order (limit 50 kg)

Platforma odkładcza ma limit logiczny **50 kg**. Podział zamówienia na kursy
działa na dwóch poziomach:

1. **Planowanie (`Navigator.plan_trips`).** Pozycje zamówienia są
   porządkowane algorytmem greedy nearest-neighbour (od komórki startowej /
   terminala, zawsze do najbliższego nieobsłużonego celu) i pakowane do
   kursu, dopóki suma wag nie przekroczy 50 kg. Przekroczenie zamyka bieżący
   kurs i otwiera kolejny. Wynik: lista kursów
   `[{"items": [...], "targets": [(c, r), ...], "total_weight": float}]`
   o minimalizowanym dystansie.
2. **Weryfikacja w runtime (kontroler).** Przed każdym `PICK_ITEM`
   kontroler sprawdza, czy `ładunek + waga_produktu > 50 kg`. Jeśli tak
   (np. gdy zeskanowana waga różni się od planowanej), przerywa objazd:
   `RETURN_TO_TERMINAL` → `UNLOAD` → kontynuacja **tego samego zamówienia**
   od przerwanej pozycji.

Przykład: ORD-002 (85 kg: 12.0 + 9.5 + 15.0 + 11.0 + 14.0 + 10.5 + 13.0)
nie mieści się w jednym kursie — planista dzieli je na co najmniej 2 kursy,
każdy ≤ 50 kg, a raport odnotowuje `"trips": 2`.

## 8. Format komunikatów błędów

Komunikaty w raporcie są SZTYWNE (testy porównują je tekstowo):

| Status pozycji | Dokładny komunikat |
|----------------|--------------------|
| `ok` | `pobrano pomyślnie` |
| `wrong_product` (złe id na półce) | `Błąd lokalizacji [X] - znaleziono [Y] zamiast [Z]` |
| `wrong_product` (zgodne id, zła waga) | `Błąd wagi [X] - oczekiwano [A] kg, jest [B] kg` |
| `missing` | `brak produktu` |

Gdzie:

- `[X]` — lokalizacja półki, np. `regał [13,9] poziom 0`;
- `[Y]` — identyfikator znaleziony w kodzie QR (np. `P999`);
- `[Z]` — identyfikator oczekiwany (np. `P020`);
- `[A]` / `[B]` — waga oczekiwana / zeskanowana w kilogramach.

Przykłady:

```
Błąd lokalizacji regał [13,9] poziom 0 - znaleziono P999 zamiast P020
Błąd wagi regał [2,2] poziom 0 - oczekiwano 3.5 kg, jest 6.0 kg
```

Flaga zamówienia: `green` tylko gdy wszystkie pozycje `ok`; w każdym innym
przypadku `red` (braki, złe produkty, złe wagi).
