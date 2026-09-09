import math

from gps_ntrip_py.local_coordinates import (
    LocalEnuProjector,
    nmea_utc_difference_seconds,
    nmea_utc_seconds,
)
import pytest


def test_origin_projects_to_zero():
    projector = LocalEnuProjector(31.12345678, 121.12345678)
    x, y = projector.project(31.12345678, 121.12345678)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)


def test_east_and_north_axes_have_expected_sign_and_scale():
    projector = LocalEnuProjector(31.0, 121.0)
    east_x, east_y = projector.project(31.0, 121.001)
    north_x, north_y = projector.project(31.001, 121.0)

    assert east_x == pytest.approx(95.5, abs=0.2)
    assert abs(east_y) < 0.01
    assert north_y == pytest.approx(110.86, abs=0.2)
    assert abs(north_x) < 0.01


def test_projection_rejects_invalid_coordinates():
    with pytest.raises(ValueError, match='origin latitude'):
        LocalEnuProjector(math.nan, 121.0)
    projector = LocalEnuProjector(31.0, 121.0)
    with pytest.raises(ValueError, match='longitude'):
        projector.project(31.0, 181.0)


def test_nmea_utc_parsing_and_midnight_difference():
    assert nmea_utc_seconds('081212.50') == pytest.approx(29532.5)
    assert nmea_utc_difference_seconds('235959.95', '000000.05') == pytest.approx(0.1)


@pytest.mark.parametrize('value', ['', '1200', '246000', '126000', '120060'])
def test_invalid_nmea_utc_is_rejected(value):
    with pytest.raises(ValueError):
        nmea_utc_seconds(value)
