<!--
================================================================================
PLIK: webots_warehouse/docs/architecture.md

PRZEZNACZENIE:
    Dokument architektury projektu "Symulacyjny system obsługi zamówień
    magazynowych w Webots". Opisuje rolę każdego pliku projektu, generator
    świata, maszynę stanów FSM kontrolera, konwencje współrzędnych i siatki
    nawigacyjnej, schematy wszystkich plików JSON (z przykładami), API
    publiczne modułów Pythona, model robota i jego urządzenia, mechanizm
    podziału zamówienia na kursy (limit 50 kg), format komunikatów błędów
    w raportach oraz diagnostykę (logs/, telemetria [TEL]).

SPOSÓB UŻYCIA:
    Dokument referencyjny dla deweloperów i recenzentów. Wszystkie nazwy
    własne (DEF-e, nazwy urządzeń, sygnatury funkcji, komunikaty) są
    SZTYWNYM kontraktem integracyjnym — zmiana któregokolwiek elementu
    wymaga spójnej zmiany we wszystkich plikach projektu.
    UWAGA: worlds/warehouse.wbt jest GENEROWANY przez tools/generate_world.py
    — wnioski o strukturze świata weryfikuj w generatorze, nie w pliku .wbt.

STRUKTURA DOKUMENTU:
    1. Przegląd plików projektu
    2. Generowanie świata (tools/generate_world.py)
    3. Konwencje współrzędnych i siatka nawigacyjna
    4. Model robota i urządzenia
    5. Maszyna stanów FSM (diagram ASCII)
    6. Logowanie i diagnostyka (logs/, telemetria [TEL])
    7. API modułów Pythona
    8. Schematy JSON z przykładami
    9. Mechanizm split-order (limit 50 kg)
    10. Format komunikatów błędów
================================================================================
-->

# Architektura systemu

## 1. Przegląd plików projektu

| Plik | Rola |
|------|------|
| `worlds/warehouse.wbt` | Świat symulacji Webots R2025a: hala 18×14 m jako **jawne statyczne Solidy** (podłoga `FLOOR` + 4 ściany `WALL_N/S/E/W` — nie `RectangleArena`, patrz §2), 60 regałów (`SHELF_c_r`), 19 boxów produktów (`PRODUCT_<id>`), strefa terminala (`TERMINAL_ZONE`) + kiosk (`TERMINAL`), 3 przeszkody dynamiczne (`OBSTACLE_1..3`), oświetlenie bez cieni (`DirectionalLight` + 12× `PointLight` z kwadratowym tłumieniem), instancja robota z `WarehouseRobot.proto`, robot-pomost `MCP_BRIDGE`. `WorldInfo`: `basicTimeStep 32`, `coordinateSystem "NUE"`, `contactProperties` tire/floor i body/floor z `softCFM 0.001`. **Plik generowany przez `tools/generate_world.py` — nie edytować ręcznie.** |
| `protos/WarehouseRobot.proto` | Definicja PROTO robota: napęd różnicowy (2 koła r = 0.08 m na `HingeJoint` + `RotationalMotor`, z markerami na obręczach i `PositionSensor`ami), podnosnik z `LinearMotor`, chwytak dwupalcy na `LinearMotor`ach, kamera na głowicy, 5 czujników odległości, platforma odkładcza 0.5×0.5 m. Pole `supervisor = TRUE`. Szczegóły: §4. |
| `textures/qr/P001.png … P020.png, P999.png` | Tekstury kodów QR 512×512 px (potęga dwójki — wymóg Webots) naklejone na boxy produktów. `P999.png` to kod "złego produktu" umieszczony na półce P020. |
| `data/warehouse_layout.json` | Geometria magazynu: siatka, origin, komórki zablokowane, terminal, start robota, wysokości poziomów, sloty, reguła dostępu, limity wagowe. |
| `data/products.json` | Katalog 20 produktów: pozycja, waga, payload QR, komórka dostępu, flagi anomalii (`present`, `world_qr`). |
| `orders/order_01..03.json` | Zamówienia testowe ORD-001, ORD-002, ORD-003. |
| `reports/` | Katalog tworzony w runtime; kontroler zapisuje tu `report_<order_id>.json`. |
| `logs/` | Katalog diagnostyczny tworzony w runtime: `fsm_debug.log` (cały stdout/stderr kontrolera, patrz §6), zdjęcia `scan_<id>_<próba>.png` zapisywane przy każdej próbie skanu QR, artefakty pomocnicze (`axis_verification.json`, zrzuty *.jpg). |
| `tools/generate_assets.py` | Generator tekstur QR (`qrcode` + `pillow`, 512×512 px) i plików danych spójnych z kontraktem. |
| `tools/generate_world.py` | Generator świata `worlds/warehouse.wbt` z `data/*.json` — źródło prawdy o strukturze świata; odtwarza plik po przypadkowym nadpisaniu przez Webots. Patrz §2. |
| `tools/validate_project.py` | Walidator spójności: świat ↔ `products.json` ↔ zamówienia ↔ tekstury. |
| `tools/test_obstacle_navigation.py` | Testy regresyjne nawigacji (unittest, mock `Supervisor`): zablokowane cele, objazdy, WAIT_FOR_PATH. |
| `tools/_patch_camera.py` | Latka narzędziowa (jednorazowa): pozycja kamery poza zasięgiem palców, sztywniejsze kontakty, logi watchdog. Zachowana dla historii zmian modelu. |
| `controllers/mcp_bridge/` | Kontroler-pomost MCP (z repo [webotMCP](https://github.com/MHasaan/webotMCP)): supervisor wystawiający symulację po TCP (porty 10022/10023) dla serwera MCP. W świecie instancja `DEF MCP_BRIDGE Robot { supervisor TRUE }`. |
| `controllers/mcp_robot/` | Generyczny kontroler-agent MCP, doczepiany do dowolnego robota (opcjonalny, z repo webotMCP). |
| `controllers/warehouse_controller/warehouse_controller.py` | Główny kontroler robota — maszyna stanów FSM, zamknięta pętla nawigacji po Supervisor API, pobieranie/rozładunek kinematyczny. Przy starcie przekierowuje stdout/stderr do `logs/fsm_debug.log`. |
| `controllers/warehouse_controller/navigation.py` | Siatka nawigacyjna, konwersja współrzędnych, A* (4 kierunki), planowanie kursów z limitem 50 kg. |
| `controllers/warehouse_controller/qr_scanner.py` | Odczyt kodu QR z obrazu kamery Webots przez `cv2.QRCodeDetector`; parser payloadu. |
| `controllers/warehouse_controller/order_manager.py` | Wczytywanie zamówień, statusowanie pozycji, budowa i zapis raportów JSON. |

Korzeń projektu z poziomu kontrolerów:
`Path(__file__).resolve().parents[2]`.

## 2. Generowanie świata (`tools/generate_world.py`)

`worlds/warehouse.wbt` **nie jest edytowany ręcznie** — jest w całości
generowany skryptem `tools/generate_world.py` z danych `data/warehouse_layout.json`
i `data/products.json`. Generator przed zapisem tworzy kopię
`worlds/warehouse.wbt.bak`. Uruchomienie:

```
python tools/generate_world.py
```

Kluczowe decyzje konstrukcyjne generatora (źródło: kod skryptu):

- **Układ współrzędnych NUE.** `WorldInfo` deklaruje jawnie
  `coordinateSystem "NUE"` (podłoga = płaszczyzna XZ, oś Y w górę). Bez tej
  deklaracji R2025a przyjąłby domyślne ENU (Z w górę) i geometria magazynu
  "ległaby na boku". `WorldInfo.gravity` jest od R2023a skalarem (SFFloat).
- **Podłoga i ściany jako jawne Solidy.** `RectangleArena` z EXTERNPROTO nie
  dawała czynnego `boundingObject` — obiekty dynamiczne przelatywały przez
  podłogę. Dlatego hala to: `DEF FLOOR Solid` (box 18×0.02×14 m na
  y = −0.01, `contactMaterial "floor"`) oraz cztery ściany `DEF WALL_N/S/E/W`
  (wysokość 2.5 m, `locked TRUE`). **`RectangleArena` nie występuje w świecie.**
- **Właściwości kontaktów.** `WorldInfo.contactProperties` definiuje dwie pary
  materiałów: `tire`↔`floor` (`coulombFriction 2.5`) i `body`↔`floor`
  (`coulombFriction 0.05`), obie z `softCFM 0.001` (stabilizacja kontaktów —
  koła trzymają podłoże, korpus i ślizgi mogą się ślizgać). Robot deklaruje
  `contactMaterial "tire"` (koła) i `"body"` (korpus).
- **EXTERNPROTO** tylko dla `TexturedBackground`, `TexturedBackgroundLight`
  (adresy R2025a) i lokalnego `../protos/WarehouseRobot.proto`; deklaracje
  muszą być na początku pliku, przed węzłami.
- **Regały** (`DEF SHELF_<c>_<r>`): 4 słupki narożne + 3 półki, statyczne
  (`locked TRUE`); `boundingObject` obejmuje tylko słupki i półki, więc
  produkty stojące na półkach nie kolidują z regałem.
- **Produkty** (`DEF PRODUCT_<id>`): box 0.25 m z fizyką, `mass = waga`,
  tekstura QR; P005 jest celowo pomijany (scenariusz `missing`), a box na
  miejscu P020 nosi teksturę `P999.png` (scenariusz `wrong_product`).
- **Oświetlenie bezcieniowe** (warunek czytelności QR): `DirectionalLight`
  z `castShadows FALSE` + 12× `PointLight` (`attenuation 0 0 1` — kwadratowe
  tłumienie, `radius 7`, wysokość 2.7 m, nad alejami).
- Wszystkie węzły `Solid` mają unikalne pola `name` (Webots ostrzega o
  duplikatach domyślnej nazwy "solid").

## 3. Konwencje współrzędnych i siatka nawigacyjna

### 3.1 Układ współrzędnych Webots

- Podłoga = płaszczyzna **XZ**, oś **Y w górę** (`coordinateSystem "NUE"`).
- Hala: podłoga 18×14 m + ściany wysokości 2.5 m jako jawne statyczne Solidy
  (`FLOOR`, `WALL_N/S/E/W`) — patrz §2.
- Rotacje axis-angle wokół osi Y; dodatni kąt = obrót przeciwnie do wskazówek
  zegara patrząc z góry (zgodnie z regułą prawej dłoni dla +Y).

### 3.2 Siatka nawigacyjna

- **16 kolumn × 12 rzędów**, komórka kwadratowa **1.0 m**.
- Środek komórki `(c, r)` w świecie:

  ```
  x = −7.5 + c
  z = −5.5 + r
  ```

- Komórka `[0,0]` to południowo-zachodni (−X, −Z) narożnik siatki.

### 3.3 Regały i komórki dostępu

- **Komórki regałów (zablokowane dla nawigacji):** rzędy
  `r ∈ {2, 3, 5, 6, 8, 9}` × kolumny `c ∈ {2, 3, 4, 5, 6, 9, 10, 11, 12, 13}`
  — łącznie 60 regałów, DEF `SHELF_c_r`.
- Regał: obrys 0.9 × 0.9 m (X × Z), wysokość 1.8 m, 3 poziomy półek
  (konstrukcyjnie: 4 słupki + 3 półki, patrz §2).
- **Wysokości środków produktów:** `levels = [0.2, 0.8, 1.4]` m.
- **Sloty na poziomie:** 2 sztuki, offset `±0.25` m wzdłuż osi X.
- **Reguła dostępu (access cell)** — komórka, na której staje robot:

  | Rząd regału | Komórka dostępu | Orientacja robota |
  |-------------|-----------------|-------------------|
  | `r ∈ {2, 5, 8}` (rząd "północny" pary) | `(c, r − 1)` | przód na południe (+Z) |
  | `r ∈ {3, 6, 9}` (rząd "południowy" pary) | `(c, r + 1)` | przód na północ (−Z) |

  Tekstura QR znajduje się na ścianie produktu zwróconej do alei dostępu.

### 3.4 Terminal i start robota

- Terminal: komórki `[[7,11], [8,11]]`, środek strefy `(0.0, 5.5)`,
  zielony patch 2×2 m na podłodze (`TERMINAL_ZONE`) + kiosk DEF `TERMINAL`
  przy południowej ścianie (`(0, 0, 6.6)`).
- Start robota: komórka `[7,11]` → pozycja `(−0.5, 0.01, 5.5)` (lekki uniesiony
  spoczynek na podłodze), rotacja `0 1 0 0` — przód robota na północ (−Z).

### 3.5 Przeszkody dynamiczne

Trzy kostki 0.4 m z fizyką (DEF `OBSTACLE_1`, `OBSTACLE_2`, `OBSTACLE_3`)
na alejach, na pozycjach `(0.5, −1.5)`, `(−4.5, 1.5)`, `(2.5, 4.5)`.

Kontroler obsługuje je **dwukanałowo**:

1. **Prewencja (planowanie):** przy każdym planowaniu ścieżki
   (`compute_path`) kontroler odczytuje przez Supervisor API bieżące pozycje
   wszystkich węzłów `OBSTACLE_*` i oznacza jako zablokowane komórki siatki
   pod ich rzutem, **z marginesem 0.60 m** na palce chwytaka i promień obrotu.
   Przeszkody są więc znane planistowi *w chwili planowania*, ale nie są
   wpisane w statyczną mapę — przesunięcie kostki w trakcie symulacji jest
   uwzględniane przy kolejnym przeplanowaniu.
2. **Reakcja (czujniki):** jeśli mimo to robot najedzie na przeszkodę
   (np. kostka przesunięta podczas jazdy danego legu), przednie czujniki
   odległości wyzwalają stan `AVOID_OBSTACLE` (patrz §5).

### 3.6 Orientacja robota i heading

- **Lokalny przód robota = −Z.** Dodatnia prędkość obu kół = jazda do przodu.
- Heading jest odczytywany z **macierzy orientacji** (`node.getOrientation()`),
  nie z pola `rotation`: `theta = atan2(ori[2], ori[8])` — jest to odporne
  na pochylenie kadłuba (dynamika kontaktów).
- Wektor przodu w świecie: `(−sin θ, 0, −cos θ)`; `θ = 0` → przód na −Z
  (północ).

## 4. Model robota i urządzenia

Zdefiniowany w `protos/WarehouseRobot.proto`; instancja w świecie:

```
WarehouseRobot {
  translation -0.5 0.01 5.5
  rotation 0 1 0 0
  controller "warehouse_controller"
}
```

- Początek układu PROTO na poziomie podłogi (`y = 0` = kontakt kół
  z podłożem); masa robota ~30 kg (korpus 24 + koła 2×2 + głowica 1.4
  + palce 2×0.25).
- Korpus zwężony do 0.5×0.3×0.7 m (X×Y×Z) — obrót w alei 1.0 m bez kontaktu
  z regałami; `contactMaterial "body"`.
- Napęd różnicowy: 2 koła (Cylinder r = 0.08 m, szer. 0.05 m) na `HingeJoint`
  (oś `−1 0 0`, kotwice x = ±0.27 m) + `RotationalMotor`
  (maxVelocity 12 rad/s); **na każdym kole biały marker na obręczy**
  (rotacja widoczna na nagraniach) oraz **`PositionSensor`**
  (`wheel_left_sensor` / `wheel_right_sensor`) używany przez kontroler do
  diagnostyki napędu w telemetrii `[TEL]`. Koła mają `contactMaterial "tire"`.
- Stabilność: dwa ślizgi (kulki r = 0.025 m, przód/tył) wpisane w
  `boundingObject` korpusu.
- Podnosnik (winda): `SliderJoint` oś +Y + `LinearMotor`, zakres 0–1.6 m.
  Głowica (`lift_head`) jest **cofnięta względem środka korpusu
  (z = −0.15 m)** — wysunięta nad stronę regałową (przód = −Z).
- Kamera na głowicy: **optyka na wysokości `0.25 + lift`**, więc poziom `L`
  regału wymaga `lift = levels[L] − 0.25`. Kamera ma rotację
  `−1 1 1 2.094…` rad, która obraca oś optyczną kamery (+X) na lokalne **−Z**
  (przód robota), a górę obrazu na +Y. Jest przesunięta poza zasięg palców
  (z = −0.24 w układzie głowicy).
- Chwytak: dwa **smukłe palce** (box 0.02×0.10×0.10 m) na `SliderJoint`ach
  (osi ±X) z `LinearMotor`ami, pozycja 0–0.05 m; startowo na x = ±0.20 m
  (światło 0.38 m), domknięcie zwęża światło do ~0.28 m (paczka 0.25 m).
- Platforma odkładcza na korpusie 0.5×0.5 m (y = 0.36, przesunięta o
  z = +0.12 m) — **limit logiczny 50 kg** (wymuszany przez kontroler, nie
  przez fizykę). Sloty dla kolejnych produktów w lokalnym układzie robota:
  `(±0.15, 0.50 + 0.3·k, ±0.15)` — znak x zmienia się co slot, znak z co dwa
  sloty (k = indeks produktu na platformie).
- Maksymalna waga pojedynczej paczki: 20 kg (`max_package_weight_kg`).

### Nazwy urządzeń (kontrakt SZTYWNY)

| Nazwa | Typ | Przeznaczenie |
|-------|-----|---------------|
| `wheel_left` | RotationalMotor | lewe koło napędowe |
| `wheel_right` | RotationalMotor | prawe koło napędowe |
| `wheel_left_sensor` | PositionSensor | położenie lewego koła (telemetria `[TEL]`) |
| `wheel_right_sensor` | PositionSensor | położenie prawego koła (telemetria `[TEL]`) |
| `lift_motor` | LinearMotor | podnosnik głowicy z kamerą (0–1.6 m) |
| `gripper_left` | LinearMotor | lewy palec chwytaka (oś +X, 0–0.05 m) |
| `gripper_right` | LinearMotor | prawy palec chwytaka (oś −X, 0–0.05 m) |
| `camera` | Camera | 640×480 px, fov 1.0 rad, na głowicy; patrzy na lokalne −Z po rotacji `−1 1 1 2.094` (oś optyczna +X → −Z) |
| `ds_front_left` | DistanceSensor | przód-lewo (wykrywanie przeszkód) |
| `ds_front_right` | DistanceSensor | przód-prawo (wykrywanie przeszkód) |
| `ds_left` | DistanceSensor | bok lewy (nie bierze udziału w wykrywaniu — patrz §5) |
| `ds_right` | DistanceSensor | bok prawy (nie bierze udziału w wykrywaniu — patrz §5) |
| `ds_back` | DistanceSensor | tył (bezpieczeństwo cofania) |

Czujniki odległości: typ `generic`, `lookupTable [0→1023 … 1.5→0]`
(odczyt rośnie, gdy przeszkoda jest bliżej).

Robot ma pole `supervisor TRUE`, więc kontroler importuje
`from controller import Supervisor` i pracuje na instancji
`Supervisor` — umożliwia to odczyt pozy
(`getSelf().getField("translation")` + `getSelf().getOrientation()`),
skanowanie pozycji przeszkód `OBSTACLE_*` oraz kinematyczne
przenoszenie produktów (ustawianie ich pól `translation`).

### Oświetlenie świata

`TexturedBackground` + `TexturedBackgroundLight`, `DirectionalLight`
z `castShadows FALSE` oraz 12 `PointLight` nad alejami (`attenuation 0 0 1`,
`radius 7`, y = 2.7 m). Jasne, równomierne oświetlenie bez cieni jest
warunkiem czytelności kodów QR (eliminuje błędy `cv2.QRCodeDetector`
wynikające z kontrastu i cieni).

## 5. Maszyna stanów FSM

Kontroler `warehouse_controller.py` jest maszyną stanów o stanach:

`INIT, LOAD_ORDER, PLAN_TRIP, NAVIGATE, AVOID_OBSTACLE, WAIT_FOR_PATH,
ALIGN_AND_SCAN, VERIFY_ITEM, PICK_ITEM, RETURN_TO_TERMINAL, UNLOAD,
REPORT, DONE`

Przejścia są logowane po polsku w formacie ze strzałką, np.
`[FSM] Stan: NAVIGATE -> ALIGN_AND_SCAN` (cały log trafia do
`logs/fsm_debug.log` — patrz §6).

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
        │  │  │ lift → poziom,     │  │ zablokuj komórkę   │   │
        │  │  │ skan QR (3 próby)  │  │ (15 s), przeplanuj │   │
        │  │  └─────────┬──────────┘  └─────────┬──────────┘   │
        │  │            ▼                       │              │
        │  │  ┌────────────────────┐   brak trasy / >8 prób    │
        │  │  │    VERIFY_ITEM     │            ▼              │
        │  │  └─┬───────┬───────┬──┘  ┌────────────────────┐   │
        │  │    │ok     │wrong  │miss │   WAIT_FOR_PATH    │   │
        │  │    ▼       │       │     │ czekaj 2 s, sprawdź│   │
        │  │ ┌────────┐ │       │     │ trasę ponownie     │   │
        │  └─┤PICK_ITEM◄┤       │     └─────────┬──────────┘   │
        │    └───┬────┘ │       │               │              │
        │        │      ▼       ▼               ▼              │
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

- **NAVIGATE** — ruch w zamkniętej pętli po pozie z Supervisor API. Ścieżka
  pochodzi z lokalnego A* (`compute_path`, 4 kierunki; blokuje komórki
  regałów, komórki chwilowo zablokowane i komórki pod przeszkodami
  `OBSTACLE_*` — patrz §3.5). Dla każdego legu (odcinka do kolejnego
  waypointa) robot najpierw wykonuje **obrót w miejscu** (`rotate_to`) do
  azymutu waypointa, dopiero potem jedzie prosto (`drive_to`). W trakcie
  jazdy działa korekcja kursu (`HEADING_GAIN`), a gdy błąd kursu przekroczy
  0.6 rad, robot zatrzymuje się i wykonuje **re-align** (ponowny obrót
  w miejscu, log `[NAV] Re-align: ...`). `drive_to` ma parametr
  `check_obstacles` (domyślnie `True`) pozwalający wyłączyć wykrywanie
  przeszkód na odcinku; obecnie wszystkie wywołania używają wartości
  domyślnej, a fałszywe alarmy od regału docelowego eliminuje strojenie
  progu czujników (patrz AVOID_OBSTACLE).
- **Watchdogi 5 s** — `rotate_to`, `drive_to` i `backup` przerywają ruch,
  gdy przez 5 s czasu symulacji nie ma postępu (błąd kąta / dystans nie
  maleje); zdarzenie jest logowane (`[NAV] WATCHDOG ...`) i traktowane jak
  przeszkoda (powrót `False` → `AVOID_OBSTACLE`). To mechanizm odzyskiwania
  po fizycznym unieruchomieniu lub poślizgu.
- **AVOID_OBSTACLE** — wyzwalany, gdy **`ds_front_left` lub
  `ds_front_right`** przekroczą próg `DS_THRESHOLD = 900` (≈ 0.18 m; próg jest
  poniżej luzu bocznego w alei ~0.25 m → odczyt ~853 i dystansu do produktu
  w komórce dostępu ~0.48 m → odczyt ~700). Czujniki boczne `ds_left` /
  `ds_right` są **wyłączone z wykrywania** — najmniejszy dryf z osi alei
  fałszywie je wyzwalał. Po cofnięciu 0.3 m (`backup`, pilnowany przez
  `ds_back`) komórka przed robotem (sonda 0.6 m) jest oznaczana jako chwilowo
  zablokowana **na 15 s** i ścieżka jest przeplanowywana; po **8**
  nieudanych próbach (`MAX_AVOID_ATTEMPTS`) następuje przejście do
  `WAIT_FOR_PATH`.
- **WAIT_FOR_PATH** — robot stoi z zatrzymanymi silnikami i co 2 s sprawdza,
  czy cel lub dojazd się zwolniły (blokady chwilowe wygasają); po zwolnieniu
  wraca do stanu wznowienia (`NAVIGATE` lub `RETURN_TO_TERMINAL`).
- **ALIGN_AND_SCAN** — najpierw podnosnik na dół i (jeśli trzeba) dojazd do
  punktu `(x slotu produktu, z komórki dostępu)`, potem obrót na regał
  (`r ∈ {2,5,8}` → θ = π, przód +Z; `r ∈ {3,6,9}` → θ = 0, przód −Z);
  podnosnik ustawiany na `lift = levels[L] − 0.25` (wysokość optyki kamery to
  `0.25 + lift`); skan QR z maks. 3 próbami, przy każdej próbie zdjęcie
  z kamery jest zapisywane do `logs/scan_<id>_<próba>.png`.
- **VERIFY_ITEM** — porównanie payloadu QR z zamówieniem: identyfikator
  i waga (tolerancja 0.05 kg). Brak węzła `PRODUCT_<id>` w świecie →
  `missing`; kod nieodczytany po 3 próbach → `wrong_product` ze znalezionym
  id `NIEODCZYTANY`; niezgodne id lub waga → `wrong_product` (produkt zostaje
  na półce). Statusy pozycji: `ok`, `wrong_product`, `missing`.
- **PICK_ITEM** — zamknięcie chwytaka, kinematyczne przypięcie produktu
  do slotu platformy: pola `physics` i `boundingObject` produktu są
  **usuwane** (zapamiętane jako tekst), a Supervisor co krok ustawia jego
  `translation`/`rotation` względem pozy robota — noszony box nie koliduje
  z robotem. Jeżeli kolejny produkt z planu przekroczyłby 50 kg ładunku,
  następuje wcześniejsze `RETURN_TO_TERMINAL` → `UNLOAD` i kontynuacja tego
  samego zamówienia (log `[WAGA] ...`).
- **RETURN_TO_TERMINAL** — roboty mają odrębne komórki rozładunku `(7,11)`
  i `(8,11)`. Cel drugiego robota pozostaje przeszkodą dynamiczną dla planisty.
- **UNLOAD** — produkty trafiają do siatki za pasem dojazdu: pierwszy rząd
  ma `z=6.40`, drugi `z=6.70`, a roboty używają osobnych zakresów `x`.
  Przy odkładaniu pola `boundingObject` i `physics` są przywracane i wywoływane
  jest `resetPhysics()`. Jeśli żadna komórka terminala nie jest dostępna,
  robot zachowuje ładunek, kończy raportem błędu i zatrzymuje się.

Roboty publikują bieżącą komórkę, stan i następny cel w `customData` PROTO.
Cel kolejnej paczki jest dostępny już podczas skanowania i pobrania. Przy
spotkaniu `fleet_traffic.py` oblicza koszt ustąpienia dla obu robotów;
miejsce mijanki leży poza całą trasą robota, który otrzymał pierwszeństwo.
Nawigacja pomija powrót do środka tej samej komórki po podjechaniu do półki.

## 6. Logowanie i diagnostyka (`logs/`)

### 6.1 `logs/fsm_debug.log`

Blok `__main__` kontrolera tworzy katalog `logs/` i **przekierowuje stdout
i stderr do `logs/fsm_debug.log`** (tryb dopisywania, buforowanie liniowe).
Konsekwencja: logi kontrolera **nie pojawiają się w konsoli Webots** —
analizę przebiegu prowadzi się na tym pliku.

Konwencje pliku:

- Nowy przebieg zaczyna się bannerem `===== START KONTROLERA =====`.
- Przejścia FSM: `[FSM] Stan: <STARY> -> <NOWY>`.
- Prefiksy modułów: `[FSM]` stany, `[NAV]` nawigacja i watchdogi,
  `[QR]` skanowanie, `[WAGA]` limit udźwigu, `[RAPORT]` raporty,
  `[CHWYTAK]` / `[PODNOSNIK]` efektory.
- **Telemetria `[TEL]`** — wypisywana przez `drive_to` co 32 kroki
  symulacji (~1 s przy `basicTimeStep 32`):
  `[TEL] cel=(x,z) poza=(x,z) theta=… dist=… kolkoL/R+=…/… rad`,
  gdzie `kolkoL/R+` to przyrost położenia kół od początku legu
  (z `wheel_left_sensor` / `wheel_right_sensor`) — pozwala odróżnić
  poślizg kół od unieruchomienia robota.

### 6.2 Pozostałe artefakty

- `logs/scan_<id>_<próba>.png` — zdjęcie z kamery przy każdej próbie skanu
  QR (debugowanie ALIGN_AND_SCAN).
- `logs/axis_verification.json`, `logs/obstacles_before_pick_test.json`,
  `logs/*.jpg` — artefakty z sesji kalibracyjnych modelu robota.

## 7. API modułów Pythona

Sygnatury są SZTYWNE — kontroler główny jest kodowany przeciwko nim.

### 7.1 `navigation.py`

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

### 7.2 `qr_scanner.py`

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

### 7.3 `order_manager.py`

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

## 8. Schematy JSON z przykładami

### 8.1 `data/warehouse_layout.json`

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

### 8.2 `data/products.json`

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

### 8.3 Tabela produktów (pełna)

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

### 8.4 `orders/order_XX.json`

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

### 8.5 `reports/report_<order_id>.json`

```json
{
  "order_id": "ORD-001",
  "flag": "green",
  "trips": 1,
  "timestamp": "2026-10-11T22:27:13.487220",
  "items": [
    {
      "product_id": "P001",
      "status": "ok",
      "weight": 3.5,
      "message": "pobrano pomyślnie"
    }
  ],
  "logs": [
    "Wczytano zamówienie ORD-001 (4 pozycji).",
    "P015: pobrano pomyślnie.",
    "P007: pobrano pomyślnie.",
    "P011: pobrano pomyślnie.",
    "P001: pobrano pomyślnie."
  ]
}
```

- `flag`: `"green"` gdy wszystkie pozycje mają status `ok`;
  `"red"` gdy cokolwiek ≠ `ok`.
- `trips`: liczba kursów (powrotów do terminala z towarem).
- `timestamp`: znacznik czasu w formacie ISO.
- `logs`: **wewnętrzne logi `OrderManager`** z przebiegu zamówienia
  (komunikaty wczytania i statusowania pozycji, w kolejności zdarzeń —
  kolejność pobierania może się różnić od kolejności w `items`).
  Pełny log FSM (przejścia stanów, `[NAV]`, `[QR]`, telemetria `[TEL]`)
  znajduje się w `logs/fsm_debug.log`, **nie** w raporcie.

## 9. Mechanizm split-order (limit 50 kg)

Platforma odkładcza ma limit logiczny **50 kg**. Podział zamówienia na kursy
działa na dwóch poziomach:

1. **Planowanie (`Navigator.plan_trips`).** Pozycje zamówienia są
   porządkowane algorytmem greedy nearest-neighbour (od komórki startowej /
   terminala, zawsze do najbliższego nieobsłużonego celu) i pakowane do
   kursu, dopóki suma wag nie przekroczy 50 kg. Przekroczenie zamyka bieżący
   kurs i otwiera kolejny. Wynik: lista kursów
   `[{"items": [...], "targets": [(c, r), ...], "total_weight": float}]`
   o minimalizowanym dystansie.
2. **Weryfikacja w runtime (kontroler).** Przed każdym pobraniem kolejnej
   pozycji (w `NAVIGATE`) kontroler sprawdza, czy
   `ładunek + waga_produktu > 50 kg`. Jeśli tak (np. gdy zeskanowana waga
   różni się od planowanej), przerywa objazd: `RETURN_TO_TERMINAL` →
   `UNLOAD` → kontynuacja **tego samego zamówienia** od przerwanej pozycji
   (log `[WAGA] Pobranie <id> przekroczyłoby limit ...`).

Przykład: ORD-002 (85 kg: 12.0 + 9.5 + 15.0 + 11.0 + 14.0 + 10.5 + 13.0)
nie mieści się w jednym kursie — planista dzieli je na co najmniej 2 kursy,
każdy ≤ 50 kg, a raport odnotowuje `"trips": 2` (potwierdzone:
`reports/report_ORD-002.json`).

## 10. Format komunikatów błędów

Komunikaty w raporcie są SZTYWNE (testy porównują je tekstowo):

| Status pozycji | Dokładny komunikat |
|----------------|--------------------|
| `ok` | `pobrano pomyślnie` |
| `wrong_product` (złe id na półce) | `Błąd lokalizacji [X] - znaleziono [Y] zamiast [Z]` |
| `wrong_product` (zgodne id, zła waga) | `Błąd wagi [X] - oczekiwano [A] kg, jest [B] kg` |
| `wrong_product` (kod nieodczytany po 3 próbach) | `Błąd lokalizacji [X] - znaleziono NIEODCZYTANY zamiast [Z]` |
| `missing` | `brak produktu` |

Gdzie:

- `[X]` — lokalizacja półki, np. `regał [13,9] poziom 0`;
- `[Y]` — identyfikator znaleziony w kodzie QR (np. `P999`); gdy kodu nie
  udało się odczytać, kontroler przekazuje `NIEODCZYTANY`;
- `[Z]` — identyfikator oczekiwany (np. `P020`);
- `[A]` / `[B]` — waga oczekiwana / zeskanowana w kilogramach.

Przykłady:

```
Błąd lokalizacji regał [13,9] poziom 0 - znaleziono P999 zamiast P020
Błąd wagi regał [2,2] poziom 0 - oczekiwano 3.5 kg, jest 6.0 kg
```

Flaga zamówienia: `green` tylko gdy wszystkie pozycje `ok`; w każdym innym
przypadku `red` (braki, złe produkty, złe wagi).

## Powiązane pliki

- [test_scenarios.md](test_scenarios.md) — scenariusze testowe z oczekiwanymi
  raportami i uwagami z zarejestrowanych przebiegów
- [../tools/generate_world.py](../tools/generate_world.py) — generator świata
  (źródło prawdy o `worlds/warehouse.wbt`)
- [../protos/WarehouseRobot.proto](../protos/WarehouseRobot.proto) — model robota
- [../controllers/warehouse_controller/warehouse_controller.py](../controllers/warehouse_controller/warehouse_controller.py) — kontroler FSM
- [../logs/fsm_debug.log](../logs/fsm_debug.log) — zarejestrowane przebiegi
  kontrolera (dowody zachowania)
- [../reports/](../reports/) — wygenerowane raporty zamówień
