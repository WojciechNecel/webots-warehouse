# Założenia Projektu: Symulacyjny system obsługi zamówień magazynowych (Swarm)

## 1. Cel Pracy
Celem pracy jest zaprojektowanie i implementacja symulacyjnego systemu obsługi zamówień magazynowych z wykorzystaniem robota mobilnego w środowisku **Webots**. 

System ma w sposób zautomatyzowany odwzorowywać pełny proces kompletacji zamówień:
1. **Pobranie zamówienia:** Odczytanie listy pozycji z Terminala Zamówień.
2. **Lokalizacja i Optymalizacja:** Wyznaczenie położenia produktów na regałach i optymalizacja kolejności ich pobierania (najkrótsza trasa).
3. **Weryfikacja i Załadunek:** Rozpoznanie produktu po kodzie QR, weryfikacja wagi i pobranie go na platformę odkładczą.
4. **Rozładunek i Raportowanie:** Dostarczenie produktów do Terminala, wygenerowanie raportu z flagami statusu oraz pobranie kolejnego zlecenia.

---

## 2. Główne Zadania i Etapy Realizacji

### Etap I: Analiza i Badania
- Analiza istniejących rozwiązań (robotyka mobilna, WMS, automatyzacja magazynu).
- Zbadanie możliwości środowiska symulacyjnego Webots pod kątem czujników odległości, wizji maszynowej (odczyt QR) oraz symulacji fizyki obiektów (waga paczek, kolizje).

### Etap II: Projektowanie Środowiska (Świata) i Robota
- **Model Magazynu:** Zaprojektowanie układu przestrzennego obejmującego aleje, regały piętrowe, produkty (z unikalnymi kodami QR i przypisaną wagą do 20 kg) oraz Terminal Zamówień (pełniący jednocześnie rolę strefy wydania).
- **Oświetlenie Środowiska:** Zaprojektowanie jasnego i równomiernego oświetlenia całego magazynu (przy użyciu węzłów typu `DirectionalLight` lub `PointLight`). Jest to warunek konieczny do zminimalizowania błędów algorytmów wizji maszynowej wynikających z cieni i słabego naświetlenia kodów QR.
- **Model Robota:** Budowa modelu robota dostosowanego do specyficznych wymagań:
  - **Układ podnoszenia i chwytak:** Wyposażenie w mechanizm przesuwny w osi pionowej (Z) pozwalający sięgać na różne piętra regałów oraz chwytak przystosowany do operowania paczkami o wadze do 20 kg.
  - **Platforma odkładcza:** Przestrzeń ładunkowa na robocie o maksymalnym udźwigu do 50 kg.
  - **Sensoryka:** Kamera do skanowania kodów QR oraz czujniki do omijania przeszkód.
  - *Dokumentacja referencyjna kamery Webots:* [https://cyberbotics.com/doc/reference/camera](https://cyberbotics.com/doc/reference/camera)

### Etap III: Logika Kompletacji, Architektura i Komunikacja
- **Rozpoznawanie kodów QR (Przykładowy Stos Technologiczny):** Przykładowym i rekomendowanym rozwiązaniem do odczytu kodów jest biblioteka **OpenCV z wbudowanym modułem `cv2.QRCodeDetector`**. Wybór ten zapewnia wysoką wydajność działania i brak konieczności instalacji dodatkowych, problematycznych zależności (wymaga jedynie pakietu `opencv-python`), a jej wrażliwość na słabe światło zostanie zniwelowana przez optymalne oświetlenie na poziomie projektu świata. 
  - *Oficjalna dokumentacja klasy cv2.QRCodeDetector:* [https://docs.opencv.org/4.x/de/dc3/classcv_1_1QRCodeDetector.html](https://docs.opencv.org/4.x/de/dc3/classcv_1_1QRCodeDetector.html)
- **Zarządzanie udźwigiem (Split Order):** Robot w locie weryfikuje wagę paczek na liście. Jeśli pobranie kolejnego produktu z listy spowoduje przekroczenie limitu 50 kg na platformie, robot przerywa objazd, wraca do Terminala rozładować towar, a następnie kontynuuje to samo zamówienie.
- **Maszyna Stanów (FSM):** Sterowanie oparte na stanach obejmujących pełen cykl życia zlecenia (m.in. bezczynność, nawigacja do półki, weryfikacja QR, pobranie, powrót do rozładunku z powodu wagi, ostateczny zrzut i raport).
- **Raportowanie i Flagi (JSON):** Po zrealizowaniu zlecenia robot wysyła raport do bazy. Statusy per produkt: *pobrano pomyślnie*, *zły produkt na półce* (log: `"Błąd lokalizacji [X] - znaleziono [Y] zamiast [Z]"`), *brak produktu*.
- **Globalny status zamówienia:** 
  - **Zielona flaga:** Zamówienie kompletne, wszystkie pozycje zgodne.
  - **Czerwona flaga:** Zamówienie problematyczne (braki lub złe produkty).

### Etap IV: Nawigacja i Algorytmy Autonomiczne
- **Optymalizacja trasy:** Wdrożenie algorytmu planowania zlecenia minimalizującego pokonywany dystans przy jednoczesnym uwzględnieniu limitu wagowego (robot planuje, które paczki wziąć w pierwszym kursie, a które w drugim, jeśli waga przekracza 50 kg).
- **Algorytmy Pathfinding:** Obliczanie najkrótszej, bezkolizyjnej ścieżki w siatce magazynu między bieżącą pozycją robota a celem (np. Terminalem lub konkretnym regałem).
- **Omijanie przeszkód:** Wdrożenie układu reaktywnego wymijania przeszkód nieprzewidzianych na trasie.
  - *Tutorial omijania przeszkód:* [https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers](https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers)

### Etap V: Testy, Wdrożenie i Dokumentacja
- **Kompleksowa Dokumentacja Kodu:** Stworzenie szczegółowej dokumentacji opisującej architekturę projektu. **Każdy plik** (świat `.wbt`, kontrolery Pythona `.py`, pliki konfiguracyjne zamówień `.json`) musi zawierać precyzyjny opis zastosowania, opisy funkcji i klas oraz udokumentowane struktury danych.
- Przeprowadzenie scenariuszy testowych: poprawne zamówienia jedno- i wielokrotnego kursu (przekroczenie udźwigu), testy nawigacji przy przeszkodach oraz testy walidacji błędów (nieprawidłowy kod QR, pusta półka).
- Przygotowanie pełnej dokumentacji projektowej i wyników symulacyjnych.

---

## 3. Przydatne zasoby i referencje Webots

* **Webots User Guide:** [https://cyberbotics.com/doc/guide/index](https://cyberbotics.com/doc/guide/index)
* **Webots Reference Manual:** [https://cyberbotics.com/doc/reference/index](https://cyberbotics.com/doc/reference/index)
* **Repozytorium Webots:** [https://github.com/cyberbotics/webots](https://github.com/cyberbotics/webots)