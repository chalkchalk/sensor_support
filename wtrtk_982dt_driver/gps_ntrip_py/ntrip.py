"""NTRIP request and response helpers."""

import base64
from typing import Optional
from urllib.parse import quote


class NtripResponseError(RuntimeError):
    """Raised when an NTRIP caster rejects or malforms a response."""


def _reject_header_injection(value: str, field_name: str) -> None:
    if '\r' in value or '\n' in value:
        raise ValueError(f'{field_name} contains a newline')


def build_request(
    host: str,
    port: int,
    mountpoint: str,
    username: str,
    password: str,
    user_agent: str = 'NTRIP wtrtk_982dt_driver/1.1',
    ntrip_version: str = '1.0',
) -> bytes:
    """Build an authenticated NTRIP v1 or v2 GET request."""
    for value, name in (
        (host, 'host'),
        (username, 'username'),
        (password, 'password'),
        (user_agent, 'user_agent'),
    ):
        _reject_header_injection(value, name)
    if ntrip_version not in ('1.0', '2.0'):
        raise ValueError("ntrip_version must be '1.0' or '2.0'")

    path = quote(mountpoint.strip().lstrip('/'), safe='/')
    if not path:
        raise ValueError('mountpoint must not be empty')
    token = base64.b64encode(f'{username}:{password}'.encode('utf-8')).decode('ascii')
    http_version = '1.0' if ntrip_version == '1.0' else '1.1'
    lines = [
        f'GET /{path} HTTP/{http_version}',
        f'Host: {host}:{port}',
        f'User-Agent: {user_agent}',
        'Accept: */*',
        'Connection: close',
        f'Authorization: Basic {token}',
    ]
    if ntrip_version == '2.0':
        lines.append('Ntrip-Version: Ntrip/2.0')
    return ('\r\n'.join(lines) + '\r\n\r\n').encode('ascii')


class NtripResponseParser:
    """Incrementally separate a caster response header from RTCM payload."""

    def __init__(self, max_header_size: int = 16384) -> None:
        self._buffer = bytearray()
        self._max_header_size = max_header_size
        self._complete = False

    def feed(self, data: bytes) -> Optional[bytes]:
        """Return initial RTCM bytes once accepted, otherwise return ``None``."""
        if self._complete:
            return data
        self._buffer.extend(data)
        if len(self._buffer) > self._max_header_size:
            raise NtripResponseError('NTRIP response header is too large')

        first_line_end = self._buffer.find(b'\r\n')
        if first_line_end < 0:
            return None
        first_line = bytes(self._buffer[:first_line_end]).decode('latin-1', errors='replace')
        parts = first_line.split()
        status_ok = len(parts) >= 2 and parts[1] == '200'
        if not status_ok:
            safe_status = first_line[:200]
            raise NtripResponseError(f'NTRIP caster rejected request: {safe_status}')

        if first_line.startswith('HTTP/'):
            header_end = self._buffer.find(b'\r\n\r\n')
            if header_end < 0:
                return None
            payload = bytes(self._buffer[header_end + 4:])
        elif first_line.upper().startswith('ICY '):
            after_status = bytes(self._buffer[first_line_end + 2:])
            # Classic NTRIP v1 commonly ends the response at the ICY status line.
            # Some casters add HTTP-like headers, which are consumed when present.
            if after_status.startswith(b'\r\n'):
                payload = after_status[2:]
            elif after_status and _looks_like_ascii_header(after_status):
                header_end = self._buffer.find(b'\r\n\r\n')
                if header_end < 0:
                    return None
                payload = bytes(self._buffer[header_end + 4:])
            else:
                payload = after_status
        else:
            raise NtripResponseError(f'unsupported NTRIP response: {first_line[:200]}')

        self._complete = True
        self._buffer.clear()
        return payload


def _looks_like_ascii_header(data: bytes) -> bool:
    first_line = data.split(b'\r\n', 1)[0]
    if not first_line or b':' not in first_line:
        return False
    return all(byte in b'\t' or 32 <= byte <= 126 for byte in first_line)
