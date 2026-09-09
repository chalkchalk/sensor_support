"""WGS-84 helpers for the driver's local east-north coordinate frame."""

from dataclasses import dataclass, field
import math
from typing import Tuple


_WGS84_SEMI_MAJOR_AXIS_M = 6378137.0
_WGS84_FLATTENING = 1.0 / 298.257223563
_WGS84_ECCENTRICITY_SQUARED = (
    _WGS84_FLATTENING * (2.0 - _WGS84_FLATTENING)
)
_SECONDS_PER_DAY = 86400.0


def _geodetic_to_ecef(
    latitude_deg: float,
    longitude_deg: float,
) -> Tuple[float, float, float]:
    """Convert a WGS-84 latitude/longitude at zero altitude to ECEF."""
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    prime_vertical_radius = _WGS84_SEMI_MAJOR_AXIS_M / math.sqrt(
        1.0 - _WGS84_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude
    )
    x = prime_vertical_radius * cos_latitude * math.cos(longitude)
    y = prime_vertical_radius * cos_latitude * math.sin(longitude)
    z = (
        prime_vertical_radius * (1.0 - _WGS84_ECCENTRICITY_SQUARED)
        * sin_latitude
    )
    return x, y, z


@dataclass(frozen=True)
class LocalEnuProjector:
    """Project WGS-84 coordinates into the tangent plane at a fixed origin."""

    origin_latitude_deg: float
    origin_longitude_deg: float
    _origin_ecef: Tuple[float, float, float] = field(init=False, repr=False)
    _sin_latitude: float = field(init=False, repr=False)
    _cos_latitude: float = field(init=False, repr=False)
    _sin_longitude: float = field(init=False, repr=False)
    _cos_longitude: float = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if not math.isfinite(self.origin_latitude_deg):
            raise ValueError('origin latitude must be finite')
        if not math.isfinite(self.origin_longitude_deg):
            raise ValueError('origin longitude must be finite')
        if not -90.0 <= self.origin_latitude_deg <= 90.0:
            raise ValueError('origin latitude must be in [-90, 90] degrees')
        if not -180.0 <= self.origin_longitude_deg <= 180.0:
            raise ValueError('origin longitude must be in [-180, 180] degrees')

        latitude = math.radians(self.origin_latitude_deg)
        longitude = math.radians(self.origin_longitude_deg)
        object.__setattr__(
            self,
            '_origin_ecef',
            _geodetic_to_ecef(
                self.origin_latitude_deg,
                self.origin_longitude_deg,
            ),
        )
        object.__setattr__(self, '_sin_latitude', math.sin(latitude))
        object.__setattr__(self, '_cos_latitude', math.cos(latitude))
        object.__setattr__(self, '_sin_longitude', math.sin(longitude))
        object.__setattr__(self, '_cos_longitude', math.cos(longitude))

    def project(self, latitude_deg: float, longitude_deg: float) -> Tuple[float, float]:
        """Return east (x) and north (y) offsets from the origin in metres."""
        if not math.isfinite(latitude_deg) or not -90.0 <= latitude_deg <= 90.0:
            raise ValueError('latitude must be finite and in [-90, 90] degrees')
        if not math.isfinite(longitude_deg) or not -180.0 <= longitude_deg <= 180.0:
            raise ValueError('longitude must be finite and in [-180, 180] degrees')

        ecef = _geodetic_to_ecef(latitude_deg, longitude_deg)
        delta_x = ecef[0] - self._origin_ecef[0]
        delta_y = ecef[1] - self._origin_ecef[1]
        delta_z = ecef[2] - self._origin_ecef[2]

        east = -self._sin_longitude * delta_x + self._cos_longitude * delta_y
        north = (
            -self._sin_latitude * self._cos_longitude * delta_x
            - self._sin_latitude * self._sin_longitude * delta_y
            + self._cos_latitude * delta_z
        )
        return east, north


def nmea_utc_seconds(utc_time: str) -> float:
    """Parse an NMEA hhmmss.ss UTC field into seconds since midnight."""
    value = utc_time.strip()
    if len(value) < 6:
        raise ValueError('NMEA UTC time must contain hhmmss')
    try:
        hours = int(value[0:2])
        minutes = int(value[2:4])
        seconds = float(value[4:])
    except ValueError as error:
        raise ValueError('invalid NMEA UTC time') from error
    if not 0 <= hours < 24 or not 0 <= minutes < 60 or not 0.0 <= seconds < 60.0:
        raise ValueError('NMEA UTC time is outside its valid range')
    return hours * 3600.0 + minutes * 60.0 + seconds


def nmea_utc_difference_seconds(first: str, second: str) -> float:
    """Return the shortest UTC difference, including midnight rollover."""
    difference = abs(nmea_utc_seconds(first) - nmea_utc_seconds(second))
    return min(difference, _SECONDS_PER_DAY - difference)
