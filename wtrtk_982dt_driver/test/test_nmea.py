import math

from gps_ntrip_py.nmea import (
    checksum_is_valid,
    NmeaLineBuffer,
    parse_gga,
)
import pytest


SAMPLE = '$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47'


def test_frames_fragmented_and_multiple_sentences():
    framer = NmeaLineBuffer()
    assert framer.feed(b'noise$GPGGA,123') == []
    lines = framer.feed(b'\r\n$GNRMC,456*00\r\n')
    assert lines == ['$GPGGA,123', '$GNRMC,456*00']


def test_checksum_and_gga_coordinates():
    assert checksum_is_valid(SAMPLE)
    gga = parse_gga(SAMPLE)
    assert gga.latitude == pytest.approx(48.1173)
    assert gga.longitude == pytest.approx(11.5166666667)
    assert gga.altitude == pytest.approx(545.4)
    assert gga.fix_quality == 1
    assert gga.satellites == 8


def test_no_fix_gga_preserves_nan_position():
    gga = parse_gga('$GNGGA,123519,,,,,0,00,99.9,,,,,,*00', validate_checksum=False)
    assert gga.fix_quality == 0
    assert math.isnan(gga.latitude)
    assert math.isnan(gga.longitude)


def test_invalid_checksum_is_rejected():
    with pytest.raises(ValueError, match='checksum'):
        parse_gga(SAMPLE[:-2] + '00')
