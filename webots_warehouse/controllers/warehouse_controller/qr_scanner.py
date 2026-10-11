# -*- coding: utf-8 -*-
"""Moduł skanowania kodów QR dla kontrolera magazynowego Webots.

Przeznaczenie pliku:
    Dostarcza klasę ``QRScanner`` (odczyt kodu QR z obrazu kamery Webots)
    oraz funkcję pomocniczą ``parse_payload`` (walidacja i rozkład treści
    kodu QR na identyfikator produktu i wagę). Używany przez główny kontroler
    (``warehouse_controller.py``) w stanie FSM ``ALIGN_AND_SCAN``.

Sposób użycia:
    scanner = QRScanner(robot.getDevice("camera"))   # camera: webots Camera, już enable()
    payload = scanner.scan()                          # np. "P007;4.0" albo None
    product_id, weight = parse_payload(payload)       # -> ("P007", 4.0); ValueError przy błędzie

Struktury danych i funkcje:
    class QRScanner:
        __init__(camera)  - przyjmuje gotowe urządzenie Camera Webots (włączone zewnętrznie).
        scan() -> str|None - pobiera klatkę camera.getImage(), konwertuje przez numpy
            (uint8, reshape (h, w, 4)) i cv2 (BGRA->BGR) oraz dekoduje kod QR
            za pomocą cv2.QRCodeDetector.detectAndDecode. Zwraca tekst payloadu
            lub None, gdy kodu nie wykryto / nie rozpoznano.
    parse_payload(payload: str) -> tuple[str, float]
        Rozkłada payload w formacie "<ID>;<waga>" (np. "P001;3.5") na parę
        (identyfikator_produktu, waga_kg). Rzuca ValueError przy złym formacie
        (brak separatora, pusty identyfikator, nie-liczbowa lub ujemna waga).

Format payloadu QR (kontrakt projektowy): "<product_id>;<waga>", np. "P999;6.0".

Uwaga: moduł NIE importuje środowiska Webots - urządzenie kamery jest
przekazywane z zewnątrz, więc plik da się zaimportować i przetestować offline
(poza Webots). Test offline: uruchom ``python qr_scanner.py``.
"""

from __future__ import annotations

import cv2
import numpy as np


class QRScanner:
    """Skaner kodów QR oparty o kamerę Webots i OpenCV.

    Atrybuty:
        camera: urządzenie ``Camera`` Webots (musi być wcześniej włączone
            przez ``enable(timestep)`` w kontrolerze).
        detector: instancja ``cv2.QRCodeDetector`` wykonująca dekodowanie.
    """

    def __init__(self, camera) -> None:
        """Inicjalizuje skaner.

        Args:
            camera: włączone urządzenie kamery Webots (obiekt ``Camera``).
        """
        self.camera = camera
        self.detector = cv2.QRCodeDetector()

    def scan(self) -> str | None:
        """Pobiera klatkę z kamery i próbuje zdekodować kod QR.

        Returns:
            Tekst payloadu (np. ``"P007;4.0"``) albo ``None``, gdy na obrazie
            nie ma czytelnego kodu QR.
        """
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        raw = self.camera.getImage()
        if raw is None:
            return None

        frame = np.frombuffer(raw, np.uint8).reshape((height, width, 4))
        bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        payload, _points, _straight = self.detector.detectAndDecode(bgr)
        if payload:
            return payload
        return None


def parse_payload(payload: str) -> tuple[str, float]:
    """Rozkłada payload kodu QR na identyfikator produktu i wagę.

    Args:
        payload: tekst w formacie ``"<ID>;<waga>"``, np. ``"P001;3.5"``.

    Returns:
        Krotka ``(product_id, weight)`` - identyfikator jako ``str``,
        waga w kilogramach jako ``float``.

    Raises:
        ValueError: gdy format jest niepoprawny (brak dokładnie jednego
            separatora ``;``, pusty identyfikator, waga nie jest liczbą
            dodatnią).
    """
    if not isinstance(payload, str):
        raise ValueError(f"Payload musi być tekstem, otrzymano: {type(payload).__name__}")

    parts = payload.split(";")
    if len(parts) != 2:
        raise ValueError(
            f"Zły format payloadu: {payload!r} - oczekiwano '<ID>;<waga>'"
        )

    product_id, weight_text = parts[0].strip(), parts[1].strip()
    if not product_id:
        raise ValueError(f"Zły format payloadu: {payload!r} - pusty identyfikator produktu")

    try:
        weight = float(weight_text)
    except ValueError:
        raise ValueError(
            f"Zły format payloadu: {payload!r} - waga {weight_text!r} nie jest liczbą"
        ) from None
    if weight <= 0:
        raise ValueError(
            f"Zły format payloadu: {payload!r} - waga musi być dodatnia"
        )

    return product_id, weight


if __name__ == "__main__":
    # ------------------------------------------------------------------
    # Weryfikacja offline (bez Webots):
    # 1) generujemy testowy kod QR (payload "P007;4.0") biblioteką qrcode,
    #    dekodujemy cv2.QRCodeDetector i porównujemy z oryginałem;
    #    jeśli istnieją ../../textures/qr/*.png - dekodujemy dodatkowo 2 z nich;
    # 2) testujemy parse_payload na poprawnym i 2 błędnych payloadach.
    # ------------------------------------------------------------------
    from pathlib import Path

    import qrcode

    failures = 0

    # --- (1) dekodowanie QR ---
    expected = "P007;4.0"
    qr_img = qrcode.make(expected)
    qr_bgr = cv2.cvtColor(np.array(qr_img.convert("RGB")), cv2.COLOR_RGB2BGR)
    decoded, _pts, _straight = cv2.QRCodeDetector().detectAndDecode(qr_bgr)
    if decoded == expected:
        print(f"[TEST] OK - wygenerowany QR zdekodowany: {decoded!r}")
    else:
        failures += 1
        print(f"[TEST] BŁĄD - oczekiwano {expected!r}, zdekodowano {decoded!r}")

    textures_dir = Path(__file__).resolve().parents[2] / "textures" / "qr"
    if textures_dir.is_dir():
        pngs = sorted(textures_dir.glob("*.png"))[:2]
        for png in pngs:
            img = cv2.imread(str(png))
            text, _pts, _straight = cv2.QRCodeDetector().detectAndDecode(img)
            status = "OK" if text else "BRAK KODU"
            print(f"[TEST] tekstura {png.name}: {status} -> {text!r}")
    else:
        print(f"[TEST] pominięto tekstury - brak katalogu {textures_dir}")

    # --- (2) parse_payload ---
    pid, weight = parse_payload("P007;4.0")
    if pid == "P007" and weight == 4.0:
        print("[TEST] OK - parse_payload('P007;4.0') -> ('P007', 4.0)")
    else:
        failures += 1
        print(f"[TEST] BŁĄD - parse_payload zwrócił ({pid!r}, {weight!r})")

    for bad in ("P0074.0", "P007;abc"):
        try:
            parse_payload(bad)
            failures += 1
            print(f"[TEST] BŁĄD - {bad!r}: oczekiwano ValueError, a nie rzucił")
        except ValueError as exc:
            print(f"[TEST] OK - {bad!r}: ValueError ({exc})")

    if failures:
        print(f"[TEST] WYNIK: {failures} niepowodzeń")
        raise SystemExit(1)
    print("[TEST] WYNIK: wszystkie testy przeszły")
