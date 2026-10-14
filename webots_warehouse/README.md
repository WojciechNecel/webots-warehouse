<!--
================================================================================
PLIK: webots_warehouse/README.md

PRZEZNACZENIE:
    Główna dokumentacja projektu "Symulacyjny system obsługi zamówień
    magazynowych w Webots". Zawiera: cel projektu, wymagania środowiskowe,
    instrukcję instalacji i uruchomienia, opis działania systemu end-to-end,
    strukturę katalogów oraz ograniczenia symulacji.

SPOSÓB UŻYCIA:
    Dokument czytany przez użytkownika/recenzenta projektu jako punkt wejścia.
    Szczegóły architektury: docs/architecture.md.
    Scenariusze testowe i oczekiwane raporty: docs/test_scenarios.md.

STRUKTURA DOKUMENTU:
    1. Cel projektu
    2. Wymagania
    3. Instalacja
    4. Uruchomienie symulacji
    5. Jak działa system (end-to-end)
    6. Struktura katalogów
    7. Pliki danych i raporty
    8. Ograniczenia symulacji
================================================================================
-->

# Symulacyjny system obsługi zamówień magazynowych (Webots)

Symulacja autonomicznego robota mobilnego kompletującego zamówienia
w magazynie wysokiego składowania. Robot odczytuje zamówienia z plików JSON,
planuje trasę po siatce magazynu (A*), weryfikuje produkty kodami QR i wagą,
przewozi je na platformie odkładczej (limit 50 kg wymuszający podział
zamówienia na kursy), omija niezaplanowane przeszkody i generuje raporty JSON
z flagą statusu zamówienia (zielona / czerwona).

## 1. Cel projektu

System odwzorowuje pełny, zautomatyzowany proces kompletacji zamówień:

1. **Pobranie zamówienia** — odczyt listy pozycji z pliku zamówienia
   (`orders/order_XX.json`), stanowiącego rolę Terminala Zamówień.
2. **Lokalizacja i optymalizacja** — wyznaczenie pozycji produktów na regałach
   i optymalizacja kolejności pobierania (najbliższy sąsiad + limit wagowy).
3. **Weryfikacja i załadunek** — rozpoznanie produktu po kodzie QR,
   weryfikacja wagi z payloadu kodu, pobranie na platformę odkładczą.
4. **Rozładunek i raportowanie** — dostarczenie produktów do Terminala,
   wygenerowanie raportu JSON z flagą statusu, pobranie kolejnego zlecenia.

## 2. Wymagania

| Komponent | Wersja |
|-----------|--------|
| Webots | R2023b lub nowszy |
| Python | 3.10+ (rozwijane i testowane na 3.12) |
| opencv-python | 5.0.0 (moduł `cv2.QRCodeDetector` do odczytu QR) |
| numpy | dowolna aktualna |
| qrcode | dowolna aktualna (generowanie tekstur QR) |
| pillow | dowolna aktualna (obrazy tekstur) |

Kontroler robota wymaga pakietów Pythona dostępnych **w interpreterze
używanym przez Webots** (patrz punkt 3).

## 3. Instalacja

```bash
# 1. Zainstaluj Webots R2023b+: https://cyberbotics.com/doc/guide/installation-procedure

# 2. Sklonuj / rozpakuj projekt i przejdź do katalogu głównego
cd webots_warehouse

# 3. Zainstaluj zależności Pythona
pip install opencv-python numpy qrcode pillow
```

Jeżeli Webots używa innego interpretera niż domyślny `python`, należy:

- ustawić w Webots: **Tools → Preferences → General → Python command**
  na ścieżkę do interpretera z zainstalowanymi pakietami, albo
- zainstalować pakiety w interpreterze wbudowanym w Webots.

Tekstury kodów QR (`textures/qr/P001.png … P020.png, P999.png`) są częścią
repozytorium. W razie potrzeby można je odtworzyć narzędziem
`tools/generate_assets.py`.

## 4. Uruchomienie symulacji

1. W Webots: **File → Open World…** i wskaż `worlds/warehouse.wbt`.
2. Upewnij się, że symulacja nie jest w pauzie — naciśnij **Run**
   (robot startuje automatycznie; kontroler `warehouse_controller` jest
   przypisany do instancji robota w pliku świata).
3. Robot samodzielnie realizuje kolejno wszystkie zamówienia z katalogu
   `orders/` (w kolejności alfabetycznej nazw plików: `order_01.json`,
   `order_02.json`, `order_03.json`).
4. Postęp obserwuj w konsoli Webots — kontroler loguje przejścia maszyny
   stanów po polsku, np. `[FSM] Stan: NAVIGATE`.
5. Po zakończeniu każdego zamówienia w katalogu `reports/` pojawia się plik
   `report_<order_id>.json` (katalog jest tworzony automatycznie w runtime).

Do weryfikacji spójności plików projektu (świat, dane, zamówienia, tekstury)
służy skrypt `tools/validate_project.py`.

## 5. Jak działa system (end-to-end)

1. **INIT** — kontroler inicjalizuje urządzenia robota (silniki kół
   `wheel_left`/`wheel_right`, podnosnik `lift_motor`, chwytak
   `gripper_left`/`gripper_right`, kamerę `camera`, pięć czujników odległości)
   oraz wczytuje układ magazynu (`data/warehouse_layout.json`) i katalog
   produktów (`data/products.json`).
2. **LOAD_ORDER** — menedżer zamówień wczytuje kolejny plik z `orders/`
   i buduje listę pozycji z pełnymi metadanymi (komórka regału, poziom, slot,
   komórka dostępu, waga oczekiwana).
3. **PLAN_TRIP** — nawigator dzieli pozycje na kursy (greedy nearest-neighbour,
   limit 50 kg na platformę) i wyznacza kolejność celów minimalizującą dystans.
4. **NAVIGATE** — robot jedzie po siatce 16×12 ścieżką wyznaczoną algorytmem
   A* (4 kierunki, komórki regałów zablokowane), sterując się w zamkniętej
   pętli po pozie odczytywanej z Supervisor API.
5. **AVOID_OBSTACLE** — gdy czujniki odległości wykryją nieznaną przeszkodę
   (DEF `OBSTACLE_1..3`, pozycje nieznane kontrolerowi), robot zatrzymuje się,
   cofa 0.3 m, oznacza komórkę przed sobą jako chwilowo zablokowaną
   i przeplanowuje ścieżkę.
6. **ALIGN_AND_SCAN** — na komórce dostępu robot ustawia się twarzą do regału,
   podnosi kamerę na wysokość poziomu (`lift = wysokość_poziomu − 0.2 m`)
   i skanuje kod QR (maks. 3 próby).
7. **VERIFY_ITEM** — payload QR (`<id>;<waga>`, np. `P001;3.5`) jest
   porównywany z zamówieniem: zgodność identyfikatora i wagi. Rozpoznawane
   przypadki: produkt OK, zły produkt na półce, zła waga, brak produktu
   (pusta półka / brak odczytu).
8. **PICK_ITEM** — przy poprawnej weryfikacji robot zamyka chwytak
   i kinematycznie przypina produkt do platformy odkładcej (Supervisor API).
   Jeżeli kolejny produkt przekroczyłby 50 kg ładunku, robot najpierw wraca
   do Terminala (RETURN_TO_TERMINAL → UNLOAD) i kontynuuje to samo zamówienie.
9. **RETURN_TO_TERMINAL / UNLOAD** — powrót do strefy terminala
   (komórki `[7,11]`, `[8,11]`) i odłożenie przewożonych produktów.
10. **REPORT** — po ostatniej pozycji zamówienia generowany jest raport
    `reports/report_<order_id>.json` z flagą `green` (wszystkie pozycje OK)
    lub `red` (braki / złe produkty / złe wagi).
11. **DONE** — po ostatnim zamówieniu robot kończy pracę.

Szczegółowy diagram maszyny stanów: `docs/architecture.md`.
Gotowe scenariusze weryfikacyjne z oczekiwanymi raportami:
`docs/test_scenarios.md`.

## 6. Struktura katalogów

```
webots_warehouse/
├── README.md                       # ten plik
├── docs/
│   ├── architecture.md             # architektura, FSM, API modułów, schematy JSON
│   └── test_scenarios.md           # scenariusze testowe z oczekiwanymi raportami
├── worlds/
│   └── warehouse.wbt               # świat symulacji (hala, regały, produkty, robot)
├── protos/
│   └── WarehouseRobot.proto        # definicja PROTO robota magazynowego
├── textures/qr/
│   ├── P001.png … P020.png         # tekstury kodów QR produktów
│   └── P999.png                    # kod "złego produktu" (na półce P020)
├── data/
│   ├── warehouse_layout.json       # geometria siatki, regały, terminal, start robota
│   └── products.json               # katalog 20 produktów (pozycje, wagi, payloady QR)
├── orders/
│   ├── order_01.json               # ORD-001 — zielone, jednokursowe (13 kg)
│   ├── order_02.json               # ORD-002 — zielone, split >50 kg (85 kg)
│   └── order_03.json               # ORD-003 — czerwone (brak P005, P999 zamiast P020)
├── reports/                        # raporty JSON generowane w runtime
├── tools/
│   ├── generate_assets.py          # generator tekstur QR (512x512) i danych
│   ├── generate_world.py           # generator świata worlds/warehouse.wbt
│   └── validate_project.py         # walidator spójności projektu
├── controllers/warehouse_controller/
│   ├── warehouse_controller.py     # główny kontroler — maszyna stanów FSM
│   ├── navigation.py               # siatka, A*, planowanie kursów (split 50 kg)
│   ├── qr_scanner.py               # odczyt QR z kamery (OpenCV)
│   └── order_manager.py            # zamówienia, statusy pozycji, raporty JSON
├── controllers/mcp_bridge/         # kontroler-pomost MCP (supervisor, TCP 10022/10023)
└── controllers/mcp_robot/          # generyczny kontroler-agent MCP (opcjonalny)
```

> **Uwaga o wersji Webots:** świat jest zgodny z **R2025a**. Węzły
> `TexturedBackground`, `TexturedBackgroundLight` i `RectangleArena` są od
> R2023b+ zewnętrznymi PROTO — świat deklaruje je przez `EXTERNPROTO`
> (na samej górze pliku, przed wszystkimi węzłami — to wymóg parsera).
> Pierwsze otwarcie świata wymaga internetu (Webots pobiera te PROTO z
> repozytorium cyberbotics i zapisuje w cache).
>
> **Nie zapisuj świata z Webots, gdy symulacja jest w błędnym stanie**
> (np. po pominięciu węzłów bez deklaracji) — Webots nadpisuje plik stanem
> fizyki. W razie potrzeby świat odtworzysz: `python tools/generate_world.py`.

## 7. Pliki danych i raporty

- **`data/warehouse_layout.json`** — jedyno źródło prawdy o geometrii:
  siatka 16 kolumn × 12 rzędów, komórka 1.0 m, przeliczenie
  `x = −7.5 + c`, `z = −5.5 + r`, 60 komórek regałów, wysokości poziomów
  `[0.2, 0.8, 1.4]` m, sloty `±0.25` m, limity 50 kg (platforma) i 20 kg
  (paczka).
- **`data/products.json`** — 20 produktów z identyfikatorem, wagą, komórką
  regału, poziomem, slotem, pozycją światową, komórką dostępu i payloadem QR.
  Dwa produkty celowo anomalne: **P005** (`present: false` — pusta półka)
  i **P020** (fizyczny box ma kod `P999;6.0` — zły produkt na półce).
- **`orders/order_XX.json`** — zamówienia: `order_id` + lista pozycji
  (`product_id`, `expected_weight`).
- **`reports/report_<order_id>.json`** — raport: flaga zamówienia, liczba
  kursów, znacznik czasu, status i komunikat każdej pozycji oraz logi.
  Dokładny format komunikatów błędów opisuje `docs/architecture.md`.

## 8. Ograniczenia symulacji

- **Waga jako metadane QR.** Webots symuluje masę fizyczną obiektów, ale
  "waga" produktu w logice biznesowej pochodzi wyłącznie z payloadu kodu QR
  (`<id>;<waga>`) i z `products.json` — robot nie waży paczek czujnikiem.
  Weryfikacja wagi to porównanie metadanych.
- **Pobieranie kinematyczne przez Supervisor API.** Chwytak nie przenosi
  produktów siłami fizycznymi. Po zamknięciu palców kontroler (robot ma
  `supervisor TRUE`) co krok symulacji ustawia pole `translation` produktu
  względem pozy robota, "przyklejając" go do slotu platformy (offsety
  `(±0.15, 0.45 + 0.3·k, ±0.15)` w układzie robota). Rozładunek to teleport
  produktów do strefy terminala.
- **Przeszkody dynamiczne.** Kostki `OBSTACLE_1..3` mają fizykę, ale ich
  pozycje nie są znane kontrolerowi ani planistowi — wykrywane są wyłącznie
  reaktywnie czujnikami odległości.
- **Oświetlenie bez cieni.** Świat używa `DirectionalLight` z
  `castShadows FALSE` oraz kilku `PointLight`, aby obraz kamery był jasny
  i równomierny — to warunek niezawodnego odczytu QR przez
  `cv2.QRCodeDetector`.
- **Nawigacja po siatce.** Ruch odbywa się między środkami komórek siatki
  16×12 (obrót w miejscu + jazda prosto), a nie po trajektoriach ciągłych.
- **Jeden robot, zamówienia sekwencyjne.** System realizuje zamówienia
  jedno po drugim, w kolejności nazw plików w `orders/`.

## 9. Integracja MCP (opcjonalna)

Projekt jest przygotowany do współpracy z serwerem MCP
[webotMCP](https://github.com/MHasaan/webotMCP) — umożliwia on asystentowi AI
podgląd i modyfikację działającej symulacji (drzewo sceny, zrzuty widoku,
sterowanie symulacją, dowolne roboty).

1. Serwer jest zainstalowany w `~/webotMCP` i zarejestrowany w
   `~/.kimi-code/mcp.json` (serwer `webots`, stdio:
   `python ~/webotMCP/server/main.py`).
2. W świecie istnieje robot-pomost `DEF MCP_BRIDGE Robot`
   (`controller "mcp_bridge"`, `supervisor TRUE`), a jego kontroler
   (wraz z `mcp_robot`) leży w `controllers/` — Webots znajdzie go
   automatycznie po załadowaniu świata.
3. Serwer MCP łączy się z mostkiem przez TCP (localhost:10022) dopiero
   **w nowej sesji** Kimi Code (serwery dodane w trakcie sesji nie są
   dokładane do otwartych sesji). Po restarcie sesji status sprawdzisz
   komendą `/mcp`.

Mostkiem steruje się narzędziami MCP, np. `get_scene_tree`,
`watch_simulation`, `get_object_catalog`, `validate_world`.

## Baterie i ładowarki

Dwie stacje ładowania, poziom baterii w HUD, automatyczny powrót do stacji i wznowienie zadania po osiągnięciu 95%. Zużycie energii jest przyspieszone demonstracyjnie. Parametry i opis: [docs/battery_charging.md](docs/battery_charging.md).
