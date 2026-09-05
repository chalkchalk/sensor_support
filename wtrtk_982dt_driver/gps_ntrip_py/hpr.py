"""Parse WTRTK-982DT GPHPR output and convert its heading to ROS ENU yaw."""

from dataclasses import dataclass
import math

from gps_ntrip_py.nmea import checksum_is_valid


VALID_SOLUTION_QUALITIES = frozenset((1, 2, 4, 5, 6, 7, 8, 9))


@dataclass(frozen=True)
class HprData:
    """Fields in a Unicore extended NMEA HPR sentence."""

    sentence: str
    source_sentence: str
    utc_time: str
    heading_deg: float
    pitch_deg: float
    roll_deg: float
    solution_quality: int
    satellites: int
    correction_age_sec: float
    station_id: str


def _optional_float(value: str) -> float:
    return float(value) if value else math.nan


def _optional_int(value: str) -> int:
    return int(value) if value else 0


def parse_hpr(sentence: str, validate_checksum: bool = True) -> HprData:
    """Parse a GP/GN/...HPR sentence produced after issuing ``GPHPR``."""
    sentence = sentence.strip()
    if validate_checksum and not checksum_is_valid(sentence):
        raise ValueError('invalid or missing NMEA checksum')

    body = sentence[1:].split('*', 1)[0] if sentence.startswith('$') else sentence
    fields = body.split(',')
    if len(fields) < 9 or not fields[0].upper().endswith('HPR'):
        raise ValueError('not a complete HPR sentence')

    try:
        heading = _optional_float(fields[2])
        pitch = _optional_float(fields[3])
        roll = _optional_float(fields[4])
        solution_quality = _optional_int(fields[5])
        satellites = _optional_int(fields[6])
        correction_age = _optional_float(fields[7])
    except ValueError as error:
        raise ValueError(f'invalid HPR field: {error}') from error

    if math.isfinite(heading) and not 0.0 <= heading <= 360.0:
        raise ValueError('HPR heading is outside [0, 360]')
    if math.isfinite(pitch) and not -90.0 <= pitch <= 90.0:
        raise ValueError('HPR pitch is outside [-90, 90]')
    if math.isfinite(roll) and not -90.0 <= roll <= 90.0:
        raise ValueError('HPR roll is outside [-90, 90]')
    if not 0 <= solution_quality <= 255:
        raise ValueError('HPR solution quality is outside uint8 range')
    if not 0 <= satellites <= 65535:
        raise ValueError('HPR satellite count is outside uint16 range')

    return HprData(
        sentence=sentence,
        source_sentence=fields[0].upper(),
        utc_time=fields[1],
        heading_deg=heading,
        pitch_deg=pitch,
        roll_deg=roll,
        solution_quality=solution_quality,
        satellites=satellites,
        correction_age_sec=correction_age,
        station_id=fields[8],
    )


def corrected_heading(raw_heading_deg: float, offset_deg: float) -> float:
    """Apply antenna-to-vehicle mounting offset and wrap to [0, 360)."""
    if not math.isfinite(raw_heading_deg) or not math.isfinite(offset_deg):
        return math.nan
    return (raw_heading_deg + offset_deg) % 360.0


def heading_to_enu_yaw(heading_deg: float) -> float:
    """Convert true-north clockwise heading to ROS ENU counter-clockwise yaw."""
    if not math.isfinite(heading_deg):
        return math.nan
    yaw = math.pi / 2.0 - math.radians(heading_deg)
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


def data_is_valid(data: HprData) -> bool:
    """Return whether the HPR payload contains a currently usable attitude."""
    return (
        data.solution_quality in VALID_SOLUTION_QUALITIES
        and math.isfinite(data.heading_deg)
        and math.isfinite(data.pitch_deg)
        and math.isfinite(data.roll_deg)
    )
