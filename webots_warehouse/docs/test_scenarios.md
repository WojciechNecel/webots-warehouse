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

SPOSÓB UŻYCIA:
    1. Otwórz worlds/warehouse.wbt w Webots R2023b+.
    2. Uruchom symulację i obserwuj konsolę (logi "[FSM] Stan: ...")
       oraz zachowanie robota.
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

- Świat `worlds/warehouse.wbt` załadowany w Webots R2023b lub nowszym.
- Robot (`WarehouseRobot`) na pozycji startowej `(−0.5, 0, 5.5)` (komórka
  `[7,11]`), rotacja `0 1 0 0` (przód na północ, −Z).
- W katalogu `orders/` znajdują się pliki `order_01.json`, `order_02.json`,
  `order_03.json`; kontroler realizuje je sekwencyjnie w tej kolejności.
- Katalog `reports/` jest pusty (lub nie istnieje — zostanie utworzony).
- Przeszkody `OBSTACLE_1..3` na pozycjach `(0.5, −1.5)`, `(−4.5, 1.5)`,
  `(2.5, 4.5)`; kontroler nie zna ich pozycji.

Aby uruchomić pojedynczy scenariusz niezależnie, można tymczasowo zostawić
w `orders/` tylko wybrany plik zamówienia.

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
2. Obserwuj logi `[FSM] Stan: ...` w konsoli Webots oraz ruch robota.

### Obserwowane zachowanie

1. `INIT` → `LOAD_ORDER` — wczytanie ORD-001.
2. `PLAN_TRIP` — planista zwraca jeden kurs (13.0 kg ≤ 50 kg); kolejność
   celów wyznaczona metodą najbliższego sąsiada od komórki `[7,11]`.
3. Dla każdej pozycji cykl: `NAVIGATE` → `ALIGN_AND_SCAN` → `VERIFY_ITEM`
   → `PICK_ITEM`:
   - robot staje na komórce dostępu regału zgodnie z regułą dostępu:
     P001 → `[2,1]` (regał `[2,2]`, patrzy na południe),
     P007 → `[2,4]` (regał `[2,3]`, patrzy na północ),
     P011 → `[2,4]` (regał `[2,5]`, patrzy na południe — ta sama komórka
     dostępu co P007, ale regał po przeciwnej stronie alei),
     P015 → `[3,7]` (regał `[3,6]`, patrzy na północ),
   - podnosnik ustawia kamerę na wysokość poziomu (`lift = level − 0.2`:
     P001 → 0.0, P007 → 0.6, P011 → 1.2, P015 → 0.6),
   - skan QR zwraca payload zgodny z oczekiwanym (`P001;3.5` itd.),
   - weryfikacja id i wagi przechodzi, chwytak zamyka się, produkt jest
     kinematycznie przypinany do kolejnego slotu platformy.
4. Po ostatnim produkcie: `RETURN_TO_TERMINAL` → `UNLOAD` (4 produkty
   lądują w rządku przy strefie terminala) → `REPORT` → `DONE`
   (lub `LOAD_ORDER` dla kolejnego zamówienia).
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
  "logs": ["..."]
}
```

**Kryterium zaliczenia:** `flag == "green"`, `trips == 1`, wszystkie pozycje
`ok` z komunikatem `pobrano pomyślnie`.

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
od planisty greedy nearest-neighbour; typowo `trips == 2`.

### Kroki

1. Zostaw w `orders/` tylko `order_02.json` (lub poczekaj na drugie
   zamówienie sekwencji).
2. Uruchom symulację i obserwuj logi FSM oraz liczbę powrotów do terminala.

### Obserwowane zachowanie

1. `PLAN_TRIP` — planista zwraca **co najmniej 2 kursy**; suma wag każdego
   kursu ≤ 50 kg (np. kurs 1: 12.0 + 9.5 + 15.0 + 11.0 = 47.5 kg, kurs 2:
   14.0 + 10.5 + 13.0 = 37.5 kg — podział poglądowy).
2. Robot realizuje kurs 1: pobiera produkty (każdy z weryfikacją QR i wagi),
   po czym `RETURN_TO_TERMINAL` → `UNLOAD`.
3. **Kontynuacja tego samego zamówienia:** po rozładunku robot NIE przechodzi
   do ORD-003 — wraca do `PLAN_TRIP`/`NAVIGATE` dla kursu 2.
4. Dodatkowo kontroler weryfikuje wagę w locie: gdyby kolejny produkt
   przekroczył 50 kg ładunku, nastąpiłby przedwczesny powrót do terminala.
5. Po ostatnim kursie: `UNLOAD` → `REPORT`. Wszystkie 7 produktów
   jest fizycznie odłożonych w strefie terminala.

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
wszystkie pozycje `ok`. W logach FSM widoczne są co najmniej dwa przejścia
przez `UNLOAD` w ramach jednego zamówienia.

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
  boxa**. Skan QR kończy się niepowodzeniem (3 puste próby) → status
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
   ≤ 50 kg, więc 1 kurs planowany).
2. P002: skan `P002;7.0`, weryfikacja OK → pobranie na platformę.
3. P005: robot staje na komórce dostępu `[11,1]`, podnosi kamerę na poziom 0
   (`lift = 0.0`), wykonuje 3 próby skanu — brak kodu (pusta półka).
   Pozycja oznaczana `missing`; robot **nie pobiera niczego** i jedzie dalej.
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

**Kryterium zaliczenia:** `flag == "red"`, pozycje P005/P020 mają statusy
i komunikaty jak wyżej, P002/P019 pobrane poprawnie, `trips == 1`.

---

## TS-04: Omijanie przeszkód dynamicznych (OBSTACLE_1..3)

**Cel:** weryfikacja reaktywnego unikania przeszkód nieznanych planistowi.
Przeszkody (kostki 0.4 m z fizyką) leżą na alejach w punktach `(0.5, −1.5)`,
`(−4.5, 1.5)`, `(2.5, 4.5)` — na potencjalnych trasach między terminalem
a regałami.

### Kroki

1. Uruchom symulację z dowolnym zamówieniem (najlepiej ORD-001 — trasy
   prowadzą przez aleje z przeszkodami).
2. Obserwuj, czy trasa robota przecina komórkę zajętą przez którąś kostkę.
3. Wariant rozszerzony: podczas jazdy przeciągnij w Webots jedną z kostek
   na aktualną trasę robota (obiekty mają fizykę — można je przesuwać
   myszą z wciśniętym Shift).

### Obserwowane zachowanie

1. Robot jedzie po ścieżce A* wyznaczonej wokół **regałów** (planista nie
   zna przeszkód dynamicznych).
2. Gdy `ds_front_left` / `ds_front_right` (lub czujniki boczne) przekroczą
   próg, FSM przechodzi do `AVOID_OBSTACLE`:
   - natychmiastowe zatrzymanie,
   - cofnięcie o 0.3 m,
   - oznaczenie komórki przed robotem jako chwilowo zablokowanej,
   - przeplanowanie ścieżki `find_path` do bieżącego celu.
3. Powrót do `NAVIGATE` — robot omija przeszkodę objazdem po sąsiednich
   komórkach; w logach pojawia się wpis o wykryciu przeszkody
   i przeplanowaniu.
4. Nie dochodzi do kolizji (kostka nie zostaje przesunięta przez robota);
   zamówienie kończy się normalnie, a raport jest identyczny jak w
   scenariuszu bez interakcji z przeszkodą (np. TS-01: `flag == "green"`).

### Oczekiwany raport

Raport zamówienia pozostaje niezmieniony względem scenariusza bazowego —
omijanie przeszkód nie wpływa na statusy pozycji ani flagę. Jedyny ślad
zdarzenia to wpisy w polu `logs`, np.:

```json
{
  "logs": [
    "[FSM] Stan: NAVIGATE",
    "[FSM] Stan: AVOID_OBSTACLE",
    "[NAV] Wykryto przeszkodę — przeplanowuję ścieżkę",
    "[FSM] Stan: NAVIGATE"
  ]
}
```

**Kryterium zaliczenia:** robot nie zderza się z przeszkodą, stan
`AVOID_OBSTACLE` pojawia się w logach, trasa zostaje przeplanowana,
zamówienie kończy się raportem zgodnym ze scenariuszem bazowym.

---

## Dodatek: zasady porównywania raportów

- Porównywane tekstowo: `order_id`, `flag`, `trips`, `items[].product_id`,
  `items[].status`, `items[].weight`, `items[].message`.
- Zmienne (nieporównywane): `timestamp`, treść `logs` (z wyjątkiem obecności
  kluczowych przejść FSM w TS-04).
- Kolejność pozycji w `items` raportu odpowiada kolejności pozycji w pliku
  zamówienia (nie kolejności pobierania).
- Przed każdym przebiegiem testowym wyczyść katalog `reports/`, aby raporty
  z różnych przebiegów się nie mieszały.
