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

Świat startowy zawiera przeszkodę `OBSTACLE_2` w komórce dostępu `(3, 7)`.
Aby pobrać stamtąd produkt P015, przesuń przeszkodę poza trasę w Webots.
Pozostawienie jej na miejscu powoduje prawidłowe oczekiwanie robota.

Walidacja 2026-10-11: 9 testów regresji i 9/9 kontroli projektu zaliczonych.
W działającej symulacji, po usunięciu przeszkody z celu, `ORD-001` zakończyło
się wynikiem `green`: P001, P007, P011 i P015 pobrane oraz rozładowane.
Nie oznacza to pełnej walidacji pozostałych zamówień.

## Webots MCP

Kontrolery `mcp_bridge` i `mcp_robot` pochodzą z projektu
[MHasaan/webotMCP](https://github.com/MHasaan/webotMCP).
Serwer MCP instaluje się osobno według instrukcji tego projektu;
konfiguracja lokalnego klienta i ścieżki użytkownika nie są częścią repozytorium.
Sam kontroler magazynu działa bez klienta MCP.

Szczegółowy opis: [dokumentacja projektu](webots_warehouse/README.md).
Założenia: [specyfikacja](za_o_enia_projektu_swarm.md).
