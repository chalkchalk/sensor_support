import math

from gps_ntrip_py.hpr import (
    corrected_heading,
    data_is_valid,
    heading_to_enu_yaw,
    parse_hpr,
)
import pytest


SAMPLE = '$GNHPR,081212.00,341.48,-00.64,000.00,4,46,0.00,0999*4F'


def test_parses_protocol_manual_sample():
    hpr = parse_hpr(SAMPLE)
    assert hpr.source_sentence == 'GNHPR'
    assert hpr.utc_time == '081212.00'
    assert hpr.heading_deg == pytest.approx(341.48)
    assert hpr.pitch_deg == pytest.approx(-0.64)
    assert hpr.roll_deg == pytest.approx(0.0)
    assert hpr.solution_quality == 4
    assert hpr.satellites == 46
    assert hpr.correction_age_sec == pytest.approx(0.0)
    assert hpr.station_id == '0999'
    assert data_is_valid(hpr)


def test_accepts_other_talker_prefixes():
    hpr = parse_hpr(
        '$GPHPR,010203.00,90.00,1.00,-2.00,1,20,,*00',
        validate_checksum=False,
    )
    assert hpr.source_sentence == 'GPHPR'
    assert math.isnan(hpr.correction_age_sec)


def test_invalid_solution_with_missing_angles_is_published_as_invalid_data():
    hpr = parse_hpr('$GNHPR,010203.00,,,,0,0,,*00', validate_checksum=False)
    assert not data_is_valid(hpr)
    assert math.isnan(hpr.heading_deg)


def test_unknown_solution_quality_is_not_treated_as_valid():
    hpr = parse_hpr(
        '$GNHPR,010203.00,90,0,0,3,20,0,0*00',
        validate_checksum=False,
    )
    assert not data_is_valid(hpr)


def test_invalid_checksum_and_ranges_are_rejected():
    with pytest.raises(ValueError, match='checksum'):
        parse_hpr(SAMPLE[:-2] + '00')
    with pytest.raises(ValueError, match='heading'):
        parse_hpr('$GNHPR,010203.00,361,0,0,1,20,0,0*00', validate_checksum=False)


@pytest.mark.parametrize(
    ('heading', 'yaw'),
    [(0.0, math.pi / 2.0), (90.0, 0.0), (180.0, -math.pi / 2.0), (270.0, -math.pi)],
)
def test_heading_to_ros_enu_yaw(heading, yaw):
    assert heading_to_enu_yaw(heading) == pytest.approx(yaw)


def test_mounting_offset_wraps_heading():
    assert corrected_heading(350.0, 20.0) == pytest.approx(10.0)
