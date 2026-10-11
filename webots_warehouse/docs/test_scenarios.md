<!--
================================================================================
PLIK: webots_warehouse/docs/test_scenarios.md

PRZEZNACZENIE:
    Scenariusze testowe projektu "Symulacyjny system obsługi zamówień
    magazynowych w Webots" (Etap V specyfikacji). Każdy scenariusz opisuje:
    cel, warunki początkowe, kroki wykonania, obserwowane zachowanie robota
    oraz oczekiwany raport JSON z DOKŁADNYMI komunikatami (porównanie
    tekstowe). Scenariusze pokrywają: zamówienie zielone jednokursowe,
    zamówienie zielone z podziałem na kursy (limit 50 kg), zamówienie
    czerwone (pusta półka + zły produkt) oraz reaktywne omijanie przeszkód.
    Opisy zachowań uwzględniają bieżącą implementację kontrolera: obrót
    przed każdym legiem, re-align w trakcie jazdy, watchdogi 5 s jako
    mechanizm odzyskiwania, stan WAIT_FOR_PATH oraz telemetrię [TEL].

SPOSÓB UŻYCIA:
    1. Wygeneruj/odtwórz świat (python tools/generate_world.py) i otwórz
       worlds/warehouse.wbt w Webots R2025a.
    2. Uruchom symulację. UWAGA: kontroler przekierowuje stdout/stderr do
       logs/fsm_debug.log — logi "[FSM] Stan: ..." NIE pojawiają się
       w konsoli Webots; analizuj logs/fsm_debug.log (nowy przebieg zaczyna
       banner "===== START KONTROLERA =====") i obserwuj robota w 3D.
    3. Porównaj wygenerowane pliki reports/report_<order_id>.json
       z raportami oczekiwanymi w tym dokumencie (pola flag, trips,
       status, message muszą się zgadzać; timestamp i logs są zmienne).

STRUKTURA DOKUMENTU:
    TS-01  ORD-001 — zamówienie zielone, jednokursowe
    TS-02  ORD-002 — zamówienie zielone, split >50 kg (min. 2 kursy)
    TS-03  ORD-003 — zamówienie czerwone (brak P005, P999 zamiast P020)
    TS-04  Omijanie przeszkód dynamicznych OBSTACLE_1..3
    Dodatek: kryteria zaliczenia i uwagi o porównywaniu raportów
================================================================================
-->

# Scenariusze testowe

## Wspólne warunki początkowe

- Świat `worlds/warehouse.wbt` (generowany przez `tools/generate_world.py`)
  załadowany w Webots R2025a.
- Robot (`WarehouseRobot`) na pozycji startowej `(−0.5, 0.01, 5.5)` (komórka
  `[7,11]`), rotacja `0 1 0 0` (przód na północ, −Z).
- W katalogu `orders/` znajdują się pliki `order_01.json`, `order_02.json`,
  `order_03.json`; kontroler realizuje je sekwencyjnie w tej kolejności.
- Katalog `reports/` jest pusty (lub nie istnieje — zostanie utworzony).
- Przeszkody `OBSTACLE_1..3` na pozycjach `(0.5, −1.5)`, `(−4.5, 1.5)`,
  `(2.5, 4.5)`; nie są wpisane w statyczną mapę nawigacyjną — kontroler
  poznaje ich bieżące pozycje przez Supervisor API przy każdym planowaniu
  ścieżki oraz wykrywa je reaktywnie czujnikami przednimi (patrz TS-04).
- Logi kontrolera trafiają do `logs/fsm_debug.log` (przekierowanie stdout),
  a zdjęcia z prób skanowania QR do `logs/scan_<id>_<próba>.png`.

Aby uruchomić pojedynczy scenariusz niezależnie, można tymczasowo zostawić
w `orders/` tylko wybrany plik zamówienia.

## Wspólne cechy zachowania robota (wszystkie scenariusze)

- **Obrót przed legiem:** dla każdego odcinka ścieżki (waypoint → waypoint)
  robot najpierw obraca się w miejscu do azymutu celu (`rotate_to`), dopiero
  potem jedzie prosto (`drive_to`). W `fsm_debug.log` widać to jako naprzemienne
  odcinki po osiach siatki (theta ≈ 0 / ±π/2 / π).
- **Re-align:** jeśli w trakcie jazdy błąd kursu przekroczy 0.6 rad, robot
  zatrzymuje się i ponownie wyrównuje kurs w miejscu — w logu pojawia się
  wpis `[NAV] Re-align: theta=… cel=…`.
- **Watchdogi 5 s (mechanizm odzyskiwania):** gdy obrót, jazda lub cofanie
  nie przynoszą postępu przez 5 s czasu symulacji (np. fizyczne
  unieruchomienie, poślizg), ruch jest przerywany (wpis `[NAV] WATCHDOG
  rotate_to/drive_to/backup: …`) i traktowany jak przeszkoda — FSM przechodzi
  do `AVOID_OBSTACLE`, a po 8 nieudanych próbach do `WAIT_FOR_PATH`
  (czekanie 2 s i ponowne sprawdzenie trasy, aż blokady chwilowe wygasną).
- **Telemetria:** co ~1 s czasu symulacji `drive_to` wypisuje linię
  `[TEL] cel=(x,z) poza=(x,z) theta=… dist=… kolkoL/R+=…/… rad`
  (przyrosty z `wheel_left_sensor` / `wheel_right_sensor` pozwalają odróżnić
  poślizg kół od unieruchomienia).
- **Czasy orientacyjne** (z przebiegu zarejestrowanego w `logs/fsm_debug.log`,
  czas symulacji): ORD-001 ≈ 1.5–2 min, ORD-002 ≈ 7–8 min (dwa kursy,
  objazdy), dojazd do pojedynczej komórki dostępu ≈ kilka–kilkanaście sekund,
  cykl ALIGN_AND_SCAN → PICK_ITEM ≈ 5–8 s (podnosnik 2 s, chwytak 0.6 s,
  próby skanu po 0.3 s).

---

## TS-01: ORD-001 — zamówienie zielone, jednokursowe

**Plik zamówienia:** `orders/order_01.json`

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

Łączna waga: **13.0 kg** — poniżej limitu 50 kg, więc dokładnie **1 kurs**.

### Kroki

1. Uruchom symulację (tylko `order_01.json` w `orders/`) lub poczekaj na
   pierwsze zamówienie w sekwencji.
2. Obserwuj logi `[FSM] Stan: ...` w `logs/fsm_debug.log` oraz ruch robota.

### Obserwowane zachowanie

1. `INIT` → `LOAD_ORDER` — wczytanie ORD-001.
2. `PLAN_TRIP` — planista zwraca jeden kurs (13.0 kg ≤ 50 kg); kolejność
   celów wyznaczona metodą najbliższego sąsiada od komórki `[7,11]`
   (w zarejestrowanym przebiegu: **P015 → P007 → P011 → P001** — kolejność
   pobierania różni się od kolejności w pliku zamówienia i to jest poprawne).
3. Dla każdej pozycji cykl: `NAVIGATE` → `ALIGN_AND_SCAN` → `VERIFY_ITEM`
   → `PICK_ITEM`:
   - robot jedzie po waypointach siatki (obrót przed każdym legiem,
     ewentualne re-aligny) i staje na komórce dostępu regału zgodnie
     z regułą dostępu:
     P001 → `[2,1]` (regał `[2,2]`, patrzy na południe),
     P007 → `[2,4]` (regał `[2,3]`, patrzy na północ),
     P011 → `[2,4]` (regał `[2,5]`, patrzy na południe — ta sama komórka
     dostępu co P007, ale regał po przeciwnej stronie alei),
     P015 → `[3,7]` (regał `[3,6]`, patrzy na północ),
   - w `ALIGN_AND_SCAN` robot dociąga najpierw do punktu
     `(x slotu produktu, z komórki dostępu)`, potem obraca się na regał,
   - podnosnik ustawia kamerę na wysokość poziomu
     (`lift = level − 0.25`, bo optyka kamery jest na `0.25 + lift`:
     P001 → 0.0 [wartość −0.05 jest obcinana do 0], P007 → 0.55,
     P011 → 1.15, P015 → 0.55 — zgodnie z wpisami `[PODNOSNIK] Skan …`
     w logu),
   - skan QR (do 3 prób; każda próba zapisuje `logs/scan_<id>_<próba>.png`)
     zwraca payload zgodny z oczekiwanym (`P001;3.5` itd.),
   - weryfikacja id i wagi przechodzi, chwytak zamyka się, produkt jest
     kinematycznie przypinany do kolejnego slotu platformy (jego `physics`
     i `boundingObject` są na czas transportu usuwane).
4. Po ostatnim produkcie: `RETURN_TO_TERMINAL` → `UNLOAD` (4 produkty
   lądują w siatce za pasem dojazdu, od `z=6.40`) →
   `REPORT` → `LOAD_ORDER` dla kolejnego zamówienia (lub `DONE`, gdy brak).
5. W żadnym momencie nie występuje `RETURN_TO_TERMINAL` w środku kursu
   z powodu wagi.

### Oczekiwany raport `reports/report_ORD-001.json`

```json
{
  "order_id": "ORD-001",
  "flag": "green",
  "trips": 1,
  "timestamp": "<ISO 8601 — zmienny>",
  "items": [
    {"product_id": "P001", "status": "ok", "weight": 3.5, "message": "pobrano pomyślnie"},
    {"product_id": "P007", "status": "ok", "weight": 4.0, "message": "pobrano pomyślnie"},
    {"product_id": "P011", "status": "ok", "weight": 2.5, "message": "pobrano pomyślnie"},
    {"product_id": "P015", "status": "ok", "weight": 3.0, "message": "pobrano pomyślnie"}
  ],
  "logs": ["Wczytano zamówienie ORD-001 (4 pozycji).", "..."]
}
```

**Kryterium zaliczenia:** `flag == "green"`, `trips == 1`, wszystkie pozycje
`ok` z komunikatem `pobrano pomyślnie`.
**Status:** potwierdzone w zarejestrowanym przebiebie —
`reports/report_ORD-001.json` (flaga green, 1 kurs).

---

## TS-02: ORD-002 — zamówienie zielone, split >50 kg

**Plik zamówienia:** `orders/order_02.json`

```json
{
  "order_id": "ORD-002",
  "items": [
    {"product_id": "P003", "expected_weight": 12.0},
    {"product_id": "P006", "expected_weight": 9.5},
    {"product_id": "P008", "expected_weight": 15.0},
    {"product_id": "P010", "expected_weight": 11.0},
    {"product_id": "P013", "expected_weight": 14.0},
    {"product_id": "P016", "expected_weight": 10.5},
    {"product_id": "P018", "expected_weight": 13.0}
  ]
}
```

Łączna waga: **85.0 kg** — przekracza limit 50 kg platformy, wymagany podział
na **minimum 2 kursy** (każdy ≤ 50 kg). Dokładny podział i kolejność zależą
od planisty greedy nearest-neighbour; w zarejestrowanym przebiegu `trips == 2`
(kurs 1: 4 pozycje / 44.0 kg, kurs 2: 3 pozycje / 41.0 kg — podział zależny
od kolejności NN).

### Kroki

1. Zostaw w `orders/` tylko `order_02.json` (lub poczekaj na drugie
   zamówienie sekwencji).
2. Uruchom symulację i obserwuj logi FSM oraz liczbę powrotów do terminala.

### Obserwowane zachowanie

1. `PLAN_TRIP` — planista zwraca **co najmniej 2 kursy**; suma wag każdego
   kursu ≤ 50 kg (wpis `[FSM] Zaplanowano kurs: N pozycji, ładunek X kg.`).
2. Robot realizuje kurs 1: pobiera produkty (każdy z weryfikacją QR i wagi),
   po czym `RETURN_TO_TERMINAL` → `UNLOAD`.
3. **Kontynuacja tego samego zamówienia:** po rozładunku robot NIE przechodzi
   do ORD-003 — wraca do `PLAN_TRIP`/`NAVIGATE` dla kursu 2.
4. Dodatkowo kontroler weryfikuje wagę w locie: gdyby kolejny produkt
   przekroczył 50 kg ładunku, nastąpiłby przedwczesny powrót do terminala
   (wpis `[WAGA] …`).
5. Po ostatnim kursie: `UNLOAD` → `REPORT`. Wszystkie 7 produktów
   jest fizycznie odłożonych w strefie terminala (rządek
   `(0.5, 0.13, 4.7 + 0.35·i)`; indeksacja restartuje się w każdym kursie,
   więc produkty kursu 2 lądują w tych samych punktach co kursu 1 —
   po rozładunku kursu 1 te pozycje są już puste).

### Oczekiwany raport `reports/report_ORD-002.json`

```json
{
  "order_id": "ORD-002",
  "flag": "green",
  "trips": 2,
  "timestamp": "<ISO 8601 — zmienny>",
  "items": [
    {"product_id": "P003", "status": "ok", "weight": 12.0, "message": "pobrano pomyślnie"},
    {"product_id": "P006", "status": "ok", "weight": 9.5,  "message": "pobrano pomyślnie"},
    {"product_id": "P008", "status": "ok", "weight": 15.0, "message": "pobrano pomyślnie"},
    {"product_id": "P010", "status": "ok", "weight": 11.0, "message": "pobrano pomyślnie"},
    {"product_id": "P013", "status": "ok", "weight": 14.0, "message": "pobrano pomyślnie"},
    {"product_id": "P016", "status": "ok", "weight": 10.5, "message": "pobrano pomyślnie"},
    {"product_id": "P018", "status": "ok", "weight": 13.0, "message": "pobrano pomyślnie"}
  ],
  "logs": ["..."]
}
```

**Kryterium zaliczenia:** `flag == "green"`, `trips >= 2` (typowo 2),
wszystkie pozycje `ok`. W `logs/fsm_debug.log` widoczne są co najmniej dwa
przejścia przez `UNLOAD` w ramach jednego zamówienia.
**Status:** potwierdzone w zarejestrowanym przebiegu —
`reports/report_ORD-002.json` (flaga green, `trips: 2`).

---

## TS-03: ORD-003 — zamówienie czerwone (brak produktu + zły produkt)

**Plik zamówienia:** `orders/order_03.json`

```json
{
  "order_id": "ORD-003",
  "items": [
    {"product_id": "P002", "expected_weight": 7.0},
    {"product_id": "P005", "expected_weight": 18.0},
    {"product_id": "P020", "expected_weight": 16.0},
    {"product_id": "P019", "expected_weight": 7.5}
  ]
}
```

Anomalie wbudowane w świat:

- **P005** — `present: false`: na półce (regał `[11,2]`, poziom 0) **nie ma
  boxa**. Skan QR kończy się niepowodzeniem (3 puste próby), a `VERIFY_ITEM`
  dodatkowo stwierdza brak węzła `PRODUCT_P005` w świecie → status
  `missing`.
- **P020** — `world_qr: "P999;6.0"`: box `PRODUCT_P020` na półce
  (regał `[13,9]`, poziom 0) nosi teksturę `P999.png`. Skan zwraca payload
  `P999;6.0` — identyfikator niezgodny z oczekiwanym `P020` → status
  `wrong_product` (błąd lokalizacji). Box **nie jest pobierany** na platformę.

### Kroki

1. Zostaw w `orders/` tylko `order_03.json` (lub poczekaj na trzecie
   zamówienie sekwencji).
2. Uruchom symulację i obserwuj stany `ALIGN_AND_SCAN` / `VERIFY_ITEM`.

### Obserwowane zachowanie

1. `PLAN_TRIP` — pozycje planowane wg wag katalogowych (razem 48.5 kg
   ≤ 50 kg, więc 1 kurs planowany; wpis `[FSM] Zaplanowano kurs: 4 pozycji,
   ładunek 48.5 kg.`).
2. P002: skan `P002;7.0`, weryfikacja OK → pobranie na platformę.
3. P005: robot staje na komórce dostępu `[11,1]`, podnosi kamerę na poziom 0
   (`lift = 0.0`; wyliczone −0.05 jest obcinane do zera), wykonuje 3 próby
   skanu — brak kodu (pusta półka). Pozycja oznaczana `missing`; robot
   **nie pobiera niczego** i jedzie dalej.
4. P020: robot staje na komórce dostępu `[13,10]`, skan zwraca `P999;6.0`.
   Weryfikacja: id `P999` ≠ `P020` → pozycja oznaczana `wrong_product`
   z komunikatem błędu lokalizacji; box **pozostaje na półce**.
5. P019: skan `P019;7.5`, weryfikacja OK → pobranie.
6. `RETURN_TO_TERMINAL` → `UNLOAD` — do terminala trafiają tylko P002 i P019.
7. `REPORT` — flaga `red`, ponieważ nie wszystkie pozycje mają status `ok`.

### Oczekiwany raport `reports/report_ORD-003.json`

```json
{
  "order_id": "ORD-003",
  "flag": "red",
  "trips": 1,
  "timestamp": "<ISO 8601 — zmienny>",
  "items": [
    {"product_id": "P002", "status": "ok",            "weight": 7.0,  "message": "pobrano pomyślnie"},
    {"product_id": "P005", "status": "missing",       "weight": null, "message": "brak produktu"},
    {"product_id": "P020", "status": "wrong_product", "weight": 6.0,  "message": "Błąd lokalizacji regał [13,9] poziom 0 - znaleziono P999 zamiast P020"},
    {"product_id": "P019", "status": "ok",            "weight": 7.5,  "message": "pobrano pomyślnie"}
  ],
  "logs": ["..."]
}
```

Uwagi do porównania:

- Komunikat dla P020 musi być **tekstowo identyczny**:
  `Błąd lokalizacji regał [13,9] poziom 0 - znaleziono P999 zamiast P020`.
- Dla `wrong_product` pole `weight` zawiera wagę **zeskanowaną** z kodu
  (6.0 z payloadu `P999;6.0`); dla `missing` — `null`.
- Gdyby produkt miał zgodne id, ale błędną wagę, status to również
  `wrong_product`, z komunikatem w formacie
  `Błąd wagi regał [c,r] poziom L - oczekiwano A kg, jest B kg`
  (nie występuje w domyślnym zestawie danych — format zarezerwowany
  dla testów rozszerzonych).
- Gdyby box był na półce, ale kodu nie udało się odczytać w 3 próbach,
  status to `wrong_product` z komunikatem
  `Błąd lokalizacji regał [c,r] poziom L - znaleziono NIEODCZYTANY zamiast <id>`.

**Kryterium zaliczenia:** `flag == "red"`, pozycje P005/P020 mają statusy
i komunikaty jak wyżej, P002/P019 pobrane poprawnie, `trips == 1`.

**Ostatni pełny przebieg headless:** `ORD-001` green (1 kurs), `ORD-002`
green (2 kursy), `ORD-003` red (1 kurs), wszystkie raporty zapisane.
Paczki zostały odłożone w terminalu; nie wystąpił awaryjny zrzut.

---

## TS-04: Omijanie przeszkód dynamicznych (OBSTACLE_1..3)

**Cel:** weryfikacja obsługi przeszkód nieobecnych w statycznej mapie
nawigacyjnej. Przeszkody (kostki 0.4 m z fizyką) leżą na alejach w punktach
`(0.5, −1.5)`, `(−4.5, 1.5)`, `(2.5, 4.5)` — na potencjalnych trasach między
terminalem a regałami.

Kontroler działa dwukanałowo (szczegóły: `docs/architecture.md` §3.5 i §5):

1. **Prewencja:** przy każdym planowaniu ścieżki (`compute_path`) pozycje
   wszystkich węzłów `OBSTACLE_*` są odczytywane przez Supervisor API,
   a komórki siatki pod ich rzutem (z marginesem 0.60 m) blokowane —
   typowo trasa A* omija kostki już na starcie.
2. **Reakcja:** jeśli kostka pojawi się na trasie w trakcie jazdy (np.
   przeciągnięta myszą), przednie czujniki `ds_front_left` / `ds_front_right`
   (próg 900 ≈ 0.18 m) wyzwalają `AVOID_OBSTACLE`. Czujniki boczne nie
   biorą udziału w wykrywaniu (fałszywe alarmy przy dryfie z osi alei).

### Kroki

1. Uruchom symulację z dowolnym zamówieniem (najlepiej ORD-001 — trasy
   prowadzą przez aleje z przeszkodami).
2. Obserwuj, czy trasa robota przecina komórkę zajętą przez którąś kostkę.
3. Wariant rozszerzony: podczas jazdy przeciągnij w Webots jedną z kostek
   na aktualną trasę robota (obiekty mają fizykę — można je przesuwać
   myszą z wciśniętym Shift).

### Obserwowane zachowanie

1. Robot jedzie po ścieżce A* wyznaczonej wokół **regałów** oraz wokół
   komórek zajętych przez kostki **w chwili planowania**.
2. Gdy mimo to `ds_front_left` / `ds_front_right` przekroczą próg, FSM
   przechodzi do `AVOID_OBSTACLE`:
   - natychmiastowe zatrzymanie,
   - cofnięcie o 0.3 m (pilnowane przez `ds_back`),
   - oznaczenie komórki przed robotem (sonda 0.6 m) jako chwilowo
     zablokowanej **na 15 s**,
   - przeplanowanie ścieżki `compute_path` do bieżącego celu.
3. Powrót do `NAVIGATE` — robot omija przeszkodę objazdem po sąsiednich
   komórkach; w `logs/fsm_debug.log` pojawiają się wpisy o wykryciu
   i przeplanowaniu (format rzeczywisty):

   ```
   [NAV] PRZESZKODA: ds_front_right=924 > 900.0
   [NAV] Wykryto przeszkodę na trasie!
   [FSM] Stan: NAVIGATE -> AVOID_OBSTACLE
   [NAV] Przeszkoda w (c, r); cofanie i szukanie innej trasy.
   [NAV] Trasa alternatywna: [(c1, r1), (c2, r2), ...]
   ```

4. Jeśli objazd się nie udaje: po 8 kolejnych próbach robot przechodzi do
   `WAIT_FOR_PATH` (stoi, co 2 s sprawdza trasę; blokady chwilowe wygasają
   po 15 s, a kostka odczytywana przez Supervisor może w międzyczasie
   zostać usunięta z trasy). Watchdogi 5 s dodatkowo przerywają ruch bez
   postępu (jazda, obrót, cofanie) i też kierują do `AVOID_OBSTACLE`.
5. Nie dochodzi do kolizji (kostka nie zostaje przesunięta przez robota);
   zamówienie kończy się normalnie, a raport jest identyczny jak w
   scenariuszu bez interakcji z przeszkodą (np. TS-01: `flag == "green"`).

### Oczekiwany raport

Raport zamówienia pozostaje niezmieniony względem scenariusza bazowego —
omijanie przeszkód nie wpływa na statusy pozycji ani flagę. Ślad zdarzenia
jest w `logs/fsm_debug.log` (sekcja danego zamówienia, między bannerami
`[FSM] Pobrano zamówienie …` i `[RAPORT] …`), np.:

```
[FSM] Stan: NAVIGATE -> AVOID_OBSTACLE
[NAV] Przeszkoda w (8, 4); cofanie i szukanie innej trasy.
[NAV] Trasa alternatywna: [...]
[FSM] Stan: AVOID_OBSTACLE -> NAVIGATE
```

(Pole `logs` w raporcie zawiera wyłącznie logi `OrderManager` — statusowanie
pozycji — i **nie** odnotowuje zdarzeń nawigacyjnych).

**Kryterium zaliczenia:** robot nie zderza się z przeszkodą, wpisy
`AVOID_OBSTACLE` / `PRZESZKODA` pojawiają się w `logs/fsm_debug.log`,
trasa zostaje przeplanowana, zamówienie kończy się raportem zgodnym
ze scenariuszem bazowym.

---

## Dodatek: zasady porównywania raportów

- Porównywane tekstowo: `order_id`, `flag`, `trips`, `items[].product_id`,
  `items[].status`, `items[].weight`, `items[].message`.
- Zmienne (nieporównywane): `timestamp`, treść `logs` (to logi
  `OrderManager`, np. `"Wczytano zamówienie ORD-001 (4 pozycji)."` /
  `"P015: pobrano pomyślnie."`; kluczowe przejścia FSM dla TS-04 sprawdza
  się w `logs/fsm_debug.log`, nie w raporcie).
- Kolejność pozycji w `items` raportu odpowiada kolejności pozycji w pliku
  zamówienia (nie kolejności pobierania — ta wynika z greedy nearest-
  neighbour i jest widoczna w kolejności wpisów w `logs`).
- Przed każdym przebiegiem testowym wyczyść katalog `reports/`, aby raporty
  z różnych przebiegów się nie mieszały. `logs/fsm_debug.log` jest
  dopisywany — przebiegi rozdzielają bannery `===== START KONTROLERA =====`.

## Powiązane pliki

- [architecture.md](architecture.md) — architektura systemu, FSM, diagnostyka
- [../orders/](../orders/) — pliki zamówień testowych ORD-001..003
- [../logs/fsm_debug.log](../logs/fsm_debug.log) — zarejestrowane przebiegi
  (dowody zachowania opisanego w tym dokumencie)
- [../reports/](../reports/) — wygenerowane raporty do porównania
- [../tools/test_obstacle_navigation.py](../tools/test_obstacle_navigation.py) —
  testy regresyjne nawigacji omawiane w TS-04
