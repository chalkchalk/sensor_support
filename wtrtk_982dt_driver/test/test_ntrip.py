import base64

from gps_ntrip_py.ntrip import (
    build_request,
    NtripResponseError,
    NtripResponseParser,
)
import pytest


def test_builds_ntrip_v1_request_without_plaintext_password():
    request = build_request('caster.example', 2101, 'MOUNT', 'alice', 'secret')
    assert request.startswith(b'GET /MOUNT HTTP/1.0\r\n')
    assert b'secret' not in request
    token = base64.b64encode(b'alice:secret')
    assert b'Authorization: Basic ' + token in request
    assert request.endswith(b'\r\n\r\n')


def test_parses_fragmented_http_response_and_preserves_rtcm():
    parser = NtripResponseParser()
    assert parser.feed(b'HTTP/1.1 200 OK\r\nServer: test\r\n') is None
    assert parser.feed(b'\r\n\xd3\x00\x01') == b'\xd3\x00\x01'


def test_parses_classic_icy_response():
    parser = NtripResponseParser()
    assert parser.feed(b'ICY 200 OK\r\n\xd3\x00') == b'\xd3\x00'


def test_rejected_response_raises_useful_error():
    parser = NtripResponseParser()
    with pytest.raises(NtripResponseError, match='401'):
        parser.feed(b'HTTP/1.1 401 Unauthorized\r\n\r\n')
