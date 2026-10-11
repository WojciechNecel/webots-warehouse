# Webots Warehouse

Symulacja robota magazynowego: planowanie tras A*, odczyt QR, kompletacja
zamówień i rozładunek. Projekt rozwijany na Windows, Webots R2025a i Python 3.12.

## Uruchomienie

```powershell
python -m pip install -r requirements.txt
```

W Webots otwórz `webots_warehouse/worlds/warehouse.wbt` i uruchom symulację.
W preferencjach Webots ustaw interpreter Pythona z powyższymi zależnościami.
Log kontrolera: `webots_warehouse/logs/fsm_debug.log`.
Raporty powstają w `webots_warehouse/reports/`.

## Sprawdzenie projektu

```powershell
python webots_warehouse/tools/test_obstacle_navigation.py
python webots_warehouse/tools/validate_project.py
python webots_warehouse/tools/run_headless_tests.py --timeout 420
```

## Aktualne zachowanie

- Poprawione osie kamery, kół i czujników; chwytak mieści się w alejkach.
- Planer respektuje przeszkody i zajęty cel. `WAIT_FOR_PATH` oznacza oczekiwanie
  na zwolnienie drogi, z zatrzymanymi silnikami.
- Kamera ustawia się na pozycję konkretnej skrzynki. Podnośnik jest opuszczany
  przed jazdą i obrotem; nieudany obrót przerywa manewr.
- Transport paczek jest **kinematyczny**: skrypt ustawia ich pozycję na platformie.
  Na czas transportu wyłącza fizykę i kolizje paczki, przy rozładunku je przywraca.
  Nie jest to fizyczna symulacja chwytu ani obciążenia napędu przewożoną masą.
- Roboty publikują pozycję i kolejny cel w `customData`. Gdy ich trasy się
  przecinają, porównują koszt ustąpienia i wybierają bezpieczną mijankę poza
  trasą drugiego robota. Cel kolejnej paczki jest ogłaszany już podczas pobrania.
- Każdy robot ma własną komórkę rozładunku. Paczki trafiają za pasem dojazdu;
  przy niedostępnym terminalu robot zatrzymuje się z ładunkiem i zgłasza błąd.

Przeszkoda `OBSTACLE_2` jest poza komórką dostępu do P015, więc zamówienie
`ORD-001` można zrealizować bez ręcznej zmiany świata.

Pełny test headless: `ORD-001` green (1 kurs), `ORD-002` green (2 kursy),
`ORD-003` red zgodnie ze scenariuszem (1 kurs); bez awaryjnego zrzutu.

## Webots MCP

Kontrolery `mcp_bridge` i `mcp_robot` pochodzą z projektu
[MHasaan/webotMCP](https://github.com/MHasaan/webotMCP).
Serwer MCP instaluje się osobno według instrukcji tego projektu;
konfiguracja lokalnego klienta i ścieżki użytkownika nie są częścią repozytorium.
Sam kontroler magazynu działa bez klienta MCP.

Szczegółowy opis: [dokumentacja projektu](webots_warehouse/README.md).
Założenia: [specyfikacja](za_o_enia_projektu_swarm.md).
