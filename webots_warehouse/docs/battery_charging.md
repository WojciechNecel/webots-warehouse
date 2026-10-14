# Baterie i ładowarki

Każdy robot ma własną stację: WarehouseRobot w komórce (15,11), WarehouseRobot2 w (0,11). Stacje znajdują się przy dolnej ścianie, poza terminalem. Pola `charging_cells` w `data/warehouse_layout.json` są używane przez kontroler i generator świata.

Robot rozpoczyna z 65% baterii (pole `batteryLevel` w WarehouseRobot.proto). HUD pokazuje aktualny procent. Lampka robota jest zielona, pomarańczowo-czerwona poniżej progu, a niebieska podczas ładowania. Wyświetlacz stacji zmienia kolor na niebieski podczas ładowania.

## Model demonstracyjny

Zużycie jest przyspieszone względem rzeczywistych akumulatorów. `controllers/warehouse_controller/battery_model.py` określa procenty na sekundę czasu symulacji:

- elektronika: 0,012%/s;
- jazda przy 5 rad/s: dodatkowo 0,14%/s, do 35% więcej przy pełnym ładunku;
- skanowanie, chwytak i rozładunek: dodatkowo 0,07%/s;
- ładowanie po zadokowaniu: 1,8%/s, cel 95%.

Przed przejazdem robot sprawdza minimum 35% oraz zapas na trasę do celu i następnie do ładowarki. Bezpiecznie kończy bieżące pobranie przed zmianą zadania. Zachowuje paczki i kolejkę zamówienia podczas ładowania. Po naładowaniu wraca do zapamiętanego stanu. Brak drogi uruchamia standardowe oczekiwanie i omijanie przeszkód. Ładowanie wymaga zatrzymania, odległości od punktu dokowania poniżej 20 cm i prawidłowego kierunku robota. Po zakończeniu zamówień robot parkuje i ładuje się we własnej stacji.

Przy 0% napędy zatrzymują się w stanie BATTERY_EMPTY; nie następuje zdalne ładowanie ani teleport do stacji. Ponowne uruchomienie świata przywraca poziom startowy. Nieprzewidziane długie blokady mogą wyczerpać rezerwę.

Do demonstracji niskiego poziomu można zmienić pole `batteryLevel` robota w drzewie sceny, np. na 32. Kontroler uwzględnia tę wartość w następnym kroku, a powrót zaplanuje w bezpiecznym punkcie między zadaniami. Tryb Fast skraca czas obserwacji, ale nie zmienia zużycia na sekundę symulacji.

Testy: `python tests/test_battery.py`.
