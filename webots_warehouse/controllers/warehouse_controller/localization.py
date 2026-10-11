# ============================================================================
# localization.py -- lokalizacja robota bez Supervisor API (funkcja F1)
# ----------------------------------------------------------------------------
# PRZEZNACZENIE:
#   Estymator pozy (x, z, theta) robota magazynowego oparty WYLACZNIE o
#   czujniki pokladowe: enkodery kol (PositionSensor wheel_left/right_sensor)
#   oraz IMU (InertialUnit "imu"). Zastepuje odczyt "ground truth" przez
#   Supervisor.getSelf() w trakcie ruchu -- dzieki temu logika nawigacji jest
#   przenoszna na rzeczywiste roboty (poza symulatorem nie ma getSelf()).
#
# MODEL:
#   - theta: z IMU (getRollPitchYaw) z kalibracja offsetu na starcie
#     (imu_yow_start zostaje zrownany ze znanym kursem poczatkowym);
#     gdy IMU brak -- integracja roznicowa z enkoderow (dtheta = (dr-dl)/track).
#   - x, z: calkowanie przemieszczenia z enkoderow wzdluz kursu theta
#     (przod robota = (-sin theta, 0, -cos theta)).
#   - Kalibracja startowa: JEDNORAZOWY odczyt pozy z Supervisor API w INIT
#     (udokumentowane odchylenie: odpowiednik "znanej pozycji startowej"
#     robota w rzeczywistym magazynie).
#   - Korekta dryfu: snap() wywolywany przy dojezdzie do waypointa siatki
#     (robot stoi wtedy w znanym punkcie; koryguje x,z -- theta zostaje z IMU).
#
# API:
#   Localizer(wheel_left_sensor, wheel_right_sensor, imu=None,
#             wheel_radius=0.08, track_width=0.54)
#   start(x, z, theta)        -- ustawienie pozy poczatkowej (kalibracja)
#   update()                  -- wywolywane co krok symulacji
#   pose -> (x, z, theta)     -- biezaca estymata
#   snap(x, z)                -- korekta x,z do znanego punktu (anti-dryf)
# ============================================================================

import math


def _normalize_angle(angle: float) -> float:
    """Normalizuje kat do zakresu (-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


class Localizer:
    """Estymator pozy robota z enkoderow kol i IMU (patrz naglowek modulu)."""

    def __init__(self, wheel_left_sensor, wheel_right_sensor, imu=None,
                 wheel_radius: float = 0.08, track_width: float = 0.54):
        """wheel_*_sensor: PositionSensor kol; imu: InertialUnit lub None."""
        self._sensor_l = wheel_left_sensor
        self._sensor_r = wheel_right_sensor
        self._imu = imu
        self._wheel_radius = wheel_radius
        self._track_width = track_width
        self.x = 0.0
        self.z = 0.0
        self.theta = 0.0
        self._prev_l: float | None = None
        self._prev_r: float | None = None
        self._imu_offset = 0.0

    @property
    def available(self) -> bool:
        """True, gdy oba enkodery kol sa dostepne."""
        return self._sensor_l is not None and self._sensor_r is not None

    def start(self, x: float, z: float, theta: float) -> None:
        """Kalibracja startowa: znana poza poczatkowa + offset IMU.

        Czujniki przed pierwszym krokiem symulacji zwracaja NaN -- zapamietane
        wartości poprzednie ustawiane sa dopiero przy pierwszym pomiarze
        skończonym (patrz update()).
        """
        self.x, self.z, self.theta = x, z, theta
        self._prev_l = None
        self._prev_r = None
        if self._imu is not None:
            _roll, _pitch, yaw = self._imu.getRollPitchYaw()
            if math.isnan(yaw):
                yaw = theta
            self._imu_offset = theta - yaw

    def update(self) -> None:
        """Aktualizacja estymaty -- wywolywac co krok symulacji.

        Odporne na NaN z czujnikow (przed pierwszym krokiem symulacji
        PositionSensor/InertialUnit zwracaja NaN): taki odczyt jest pomijany,
        a referencja enkodera reinicjowana.
        """
        if not self.available:
            return
        value_l = self._sensor_l.getValue()
        value_r = self._sensor_r.getValue()
        if math.isnan(value_l) or math.isnan(value_r):
            return
        if self._prev_l is None or self._prev_r is None:
            self._prev_l, self._prev_r = value_l, value_r
            return

        dl = (value_l - self._prev_l) * self._wheel_radius
        dr = (value_r - self._prev_r) * self._wheel_radius
        self._prev_l, self._prev_r = value_l, value_r

        ds = (dl + dr) / 2.0
        if self._imu is not None:
            _roll, _pitch, yaw = self._imu.getRollPitchYaw()
            if not math.isnan(yaw):
                self.theta = _normalize_angle(yaw + self._imu_offset)
        else:
            self.theta = _normalize_angle(
                self.theta + (dr - dl) / self._track_width)

        self.x += ds * -math.sin(self.theta)
        self.z += ds * -math.cos(self.theta)

    def snap(self, x: float, z: float) -> None:
        """Korekta polozenia do znanego punktu siatki (anti-dryf).

        Wywolywana przy dojezdzie do waypointa; theta zostaje z IMU/enkoderow.
        """
        self.x, self.z = x, z

    @property
    def pose(self) -> tuple[float, float, float]:
        """Biezaca estymata pozy (x, z, theta)."""
        return self.x, self.z, self.theta
