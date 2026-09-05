"""Small, dependency-free helpers for framing and parsing NMEA GGA data."""

from dataclasses import dataclass
import math
from typing import List


@dataclass(frozen=True)
class GgaData:
    """Fields needed by the NTRIP bridge and NavSatFix publisher."""

    sentence: str
    utc_time: str
    latitude: float
    longitude: float
    fix_quality: int
    satellites: int
    hdop: float
    altitude: float


class NmeaLineBuffer:
    r"""Extract complete ``$...\n`` sentences from arbitrary serial chunks."""

    def __init__(self, max_buffer_size: int = 16384) -> None:
        if max_buffer_size < 128:
            raise ValueError('max_buffer_size must be at least 128 bytes')
        self._buffer = bytearray()
        self._max_buffer_size = max_buffer_size

    def feed(self, data: bytes) -> List[str]:
        if data:
            self._buffer.extend(data)

        sentences: List[str] = []
        while self._buffer:
            start = self._buffer.find(b'$')
            if start < 0:
                if len(self._buffer) > self._max_buffer_size:
                    self._buffer.clear()
                break
            if start:
                del self._buffer[:start]

            end = self._buffer.find(b'\n', 1)
            if end < 0:
                if len(self._buffer) > self._max_buffer_size:
                    # Retain the newest possible sentence and discard stale noise.
                    newest_start = self._buffer.rfind(b'$')
                    if newest_start > 0:
                        del self._buffer[:newest_start]
                    elif len(self._buffer) > self._max_buffer_size:
                        self._buffer.clear()
                break

            raw_line = bytes(self._buffer[:end + 1])
            del self._buffer[:end + 1]
            line = raw_line.decode('ascii', errors='ignore').strip()
            if line.startswith('$'):
                sentences.append(line)

        return sentences


def checksum_is_valid(sentence: str) -> bool:
    """Return whether a sentence contains a valid two-digit NMEA checksum."""
    sentence = sentence.strip()
    if not sentence.startswith('$') or '*' not in sentence:
        return False
    body, checksum_text = sentence[1:].rsplit('*', 1)
    if len(checksum_text) < 2:
        return False
    try:
        expected = int(checksum_text[:2], 16)
    except ValueError:
        return False
    actual = 0
    for character in body:
        actual ^= ord(character)
    return actual == expected


def _coordinate(value: str, hemisphere: str, degree_digits: int) -> float:
    if not value:
        return math.nan
    if len(value) < degree_digits + 3:
        raise ValueError('coordinate is too short')
    degrees = float(value[:degree_digits])
    minutes = float(value[degree_digits:])
    if not 0.0 <= minutes < 60.0:
        raise ValueError('coordinate minutes are outside [0, 60)')
    hemisphere = hemisphere.upper()
    if hemisphere not in ('N', 'S', 'E', 'W'):
        raise ValueError('invalid coordinate hemisphere')
    result = degrees + minutes / 60.0
    if hemisphere in ('S', 'W'):
        result = -result
    return result


def _optional_float(value: str) -> float:
    return float(value) if value else math.nan


def parse_gga(sentence: str, validate_checksum: bool = True) -> GgaData:
    """
    Parse a GP/GN/...GGA sentence.

    A GGA sentence with fix quality zero is still returned because an NTRIP caster
    may need to receive it. Missing position fields are represented by NaN.
    """
    sentence = sentence.strip()
    if validate_checksum and not checksum_is_valid(sentence):
        raise ValueError('invalid or missing NMEA checksum')

    body = sentence[1:].split('*', 1)[0] if sentence.startswith('$') else sentence
    fields = body.split(',')
    if len(fields) < 10 or not fields[0].upper().endswith('GGA'):
        raise ValueError('not a complete GGA sentence')

    try:
        fix_quality = int(fields[6]) if fields[6] else 0
        satellites = int(fields[7]) if fields[7] else 0
        latitude = _coordinate(fields[2], fields[3], 2) if fields[2] else math.nan
        longitude = _coordinate(fields[4], fields[5], 3) if fields[4] else math.nan
        hdop = _optional_float(fields[8])
        altitude = _optional_float(fields[9])
    except (ValueError, IndexError) as error:
        raise ValueError(f'invalid GGA field: {error}') from error

    return GgaData(
        sentence=sentence,
        utc_time=fields[1],
        latitude=latitude,
        longitude=longitude,
        fix_quality=fix_quality,
        satellites=satellites,
        hdop=hdop,
        altitude=altitude,
    )
