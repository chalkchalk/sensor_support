"""ROS 2 node that bridges GNSS NMEA/RTCM traffic to an NTRIP caster."""

from dataclasses import dataclass
import math
import os
import queue
import socket
import threading
import time
from typing import Any, Optional, Tuple

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from wtrtk_982dt_driver.msg import GnssAttitude
from gps_ntrip_py.hpr import (
    corrected_heading,
    data_is_valid,
    heading_to_enu_yaw,
    HprData,
    parse_hpr,
)
from gps_ntrip_py.nmea import GgaData, NmeaLineBuffer, parse_gga
from gps_ntrip_py.ntrip import (
    build_request,
    NtripResponseError,
    NtripResponseParser,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial
from std_msgs.msg import Bool, String


@dataclass(frozen=True)
class _RosEvent:
    kind: str
    payload: Any


class GpsNtripNode(Node):
    """Bidirectional serial/NTRIP bridge with ROS-facing GNSS topics."""

    def __init__(self) -> None:
        super().__init__('gps_ntrip_node')
        self._declare_parameters()
        self._load_parameters()
        self._validate_parameters()

        self._nmea_publisher = self.create_publisher(String, 'gnss/nmea', 20)
        self._fix_publisher = self.create_publisher(NavSatFix, 'gnss/fix', 10)
        self._attitude_publisher = self.create_publisher(
            GnssAttitude,
            'gnss/attitude',
            10,
        )
        connection_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._connection_publisher = self.create_publisher(
            Bool,
            'ntrip/connected',
            connection_qos,
        )
        self._device_publisher = self.create_publisher(
            Bool, 'gnss/device_connected', connection_qos
        )
        self._fix_valid_publisher = self.create_publisher(
            Bool, 'gnss/fix_valid', connection_qos
        )
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray, 'diagnostics', 10
        )

        self._stop_event = threading.Event()
        self._serial_ready = threading.Event()
        self._gga_ready = threading.Event()
        self._latest_gga_lock = threading.Lock()
        self._latest_gga: Optional[Tuple[GgaData, float]] = None
        self._latest_hpr_lock = threading.Lock()
        self._latest_hpr: Optional[Tuple[HprData, float]] = None
        self._attitude_stale_published = False
        self._rtcm_queue: queue.Queue[bytes] = queue.Queue(maxsize=256)
        self._ros_events: queue.Queue[_RosEvent] = queue.Queue(maxsize=1024)
        self._socket_lock = threading.Lock()
        self._network_socket: Optional[socket.socket] = None
        self._connection_state_lock = threading.Lock()
        self._connection_state = False
        self._last_nmea_at = 0.0
        self._parse_error_count = 0

        self._event_timer = self.create_timer(0.02, self._drain_ros_events)
        self._diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)
        self._publish_connection_state(False)
        self._publish_bool(self._device_publisher, False)
        self._publish_bool(self._fix_valid_publisher, False)

        if self._ntrip_enabled and (
            not self._ntrip_username or not self._ntrip_password
        ):
            self.get_logger().warning(
                '未配置 NTRIP 凭据；节点仍会发布定位和双天线定向结果，'
                '但不会连接 caster。请设置 NTRIP_USERNAME 和 NTRIP_PASSWORD。'
            )

        self._serial_thread = threading.Thread(
            target=self._serial_worker,
            name='gps-serial-worker',
            daemon=True,
        )
        self._network_thread = threading.Thread(
            target=self._network_worker,
            name='ntrip-network-worker',
            daemon=True,
        )
        self._serial_thread.start()
        self._network_thread.start()
        self.get_logger().info(
            f'GPS/NTRIP 节点已启动：serial={self._serial_port}@{self._baud_rate}, '
            f'GGA={1.0 / self._gga_output_interval:.3g} Hz, '
            f'HPR={1.0 / self._hpr_output_interval:.3g} Hz, '
            f'NTRIP={self._ntrip_enabled}({self._ntrip_profile}), '
            f'caster={self._ntrip_host}:{self._ntrip_port}/{self._mountpoint}'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_exclusive', True)
        self.declare_parameter('serial_reconnect_delay_sec', 3.0)
        self.declare_parameter('gga_output_interval_sec', 1.0)
        self.declare_parameter('hpr_output_interval_sec', 1.0)
        self.declare_parameter('receiver_request_retry_sec', 3.0)
        self.declare_parameter('attitude_timeout_sec', 2.5)
        self.declare_parameter('validate_nmea_checksum', True)
        self.declare_parameter('frame_id', 'gps')
        self.declare_parameter('attitude_frame_id', 'gps')
        self.declare_parameter('heading_offset_deg', 0.0)
        self.declare_parameter('ntrip_enabled', True)
        self.declare_parameter('ntrip_profile', 'ml')
        self.declare_parameter('ntrip_host', '120.253.239.161')
        self.declare_parameter('ntrip_port', 8002)
        self.declare_parameter('ntrip_mountpoint', 'RTCM33_GRCEJ')
        self.declare_parameter('ntrip_username', '')
        self.declare_parameter('ntrip_password', '')
        self.declare_parameter('ntrip_version', '1.0')
        self.declare_parameter(
            'ntrip_user_agent',
            'NTRIP wtrtk_982dt_driver/1.1',
        )
        self.declare_parameter('ntrip_connect_timeout_sec', 5.0)
        self.declare_parameter('ntrip_reconnect_delay_sec', 5.0)
        self.declare_parameter('gga_send_interval_sec', 1.0)

    def _load_parameters(self) -> None:
        def value(name: str) -> Any:
            return self.get_parameter(name).value

        self._serial_port = str(value('serial_port'))
        self._baud_rate = int(value('baud_rate'))
        self._serial_exclusive = bool(value('serial_exclusive'))
        self._serial_reconnect_delay = float(value('serial_reconnect_delay_sec'))
        self._gga_output_interval = float(value('gga_output_interval_sec'))
        self._hpr_output_interval = float(value('hpr_output_interval_sec'))
        self._receiver_request_retry = float(value('receiver_request_retry_sec'))
        self._attitude_timeout = float(value('attitude_timeout_sec'))
        self._gga_request = _receiver_output_command(
            'GPGGA',
            self._gga_output_interval,
        )
        self._hpr_request = _receiver_output_command(
            'GPHPR',
            self._hpr_output_interval,
        )
        self._validate_checksum = bool(value('validate_nmea_checksum'))
        self._frame_id = str(value('frame_id'))
        self._attitude_frame_id = str(value('attitude_frame_id'))
        self._heading_offset_deg = float(value('heading_offset_deg'))
        self._ntrip_enabled = bool(value('ntrip_enabled'))
        self._ntrip_profile = str(value('ntrip_profile')).strip().lower()
        self._ntrip_host = str(value('ntrip_host'))
        self._ntrip_port = int(value('ntrip_port'))
        self._mountpoint = str(value('ntrip_mountpoint')).lstrip('/')
        parameter_username = str(value('ntrip_username'))
        parameter_password = str(value('ntrip_password'))
        self._ntrip_username = parameter_username or os.environ.get('NTRIP_USERNAME', '')
        self._ntrip_password = parameter_password or os.environ.get('NTRIP_PASSWORD', '')
        self._ntrip_version = str(value('ntrip_version'))
        self._ntrip_user_agent = str(value('ntrip_user_agent'))
        self._connect_timeout = float(value('ntrip_connect_timeout_sec'))
        self._ntrip_reconnect_delay = float(value('ntrip_reconnect_delay_sec'))
        self._gga_send_interval = float(value('gga_send_interval_sec'))

    def _validate_parameters(self) -> None:
        if not self._serial_port:
            raise ValueError('serial_port must not be empty')
        if self._baud_rate <= 0:
            raise ValueError('baud_rate must be positive')
        if self._ntrip_profile not in ('ml', 'hk'):
            raise ValueError('ntrip_profile must be ml or hk')
        if self._ntrip_enabled:
            if not 1 <= self._ntrip_port <= 65535:
                raise ValueError('ntrip_port must be in [1, 65535]')
            if not self._ntrip_host or not self._mountpoint:
                raise ValueError('ntrip_host and ntrip_mountpoint must not be empty')
        if not self._frame_id or not self._attitude_frame_id:
            raise ValueError('frame_id and attitude_frame_id must not be empty')
        if not math.isfinite(self._heading_offset_deg):
            raise ValueError('heading_offset_deg must be finite')
        for interval, name in (
            (self._gga_output_interval, 'gga_output_interval_sec'),
            (self._hpr_output_interval, 'hpr_output_interval_sec'),
        ):
            if not 0.05 <= interval <= 60.0:
                raise ValueError(f'{name} must be in [0.05, 60.0]')
        for interval, name in (
            (self._serial_reconnect_delay, 'serial_reconnect_delay_sec'),
            (self._receiver_request_retry, 'receiver_request_retry_sec'),
            (self._attitude_timeout, 'attitude_timeout_sec'),
            (self._connect_timeout, 'ntrip_connect_timeout_sec'),
            (self._ntrip_reconnect_delay, 'ntrip_reconnect_delay_sec'),
            (self._gga_send_interval, 'gga_send_interval_sec'),
        ):
            if interval <= 0.0:
                raise ValueError(f'{name} must be positive')

    def _serial_worker(self) -> None:
        while not self._stop_event.is_set():
            port: Optional[serial.SerialBase] = None
            try:
                port = serial.serial_for_url(
                    self._serial_port,
                    baudrate=self._baud_rate,
                    timeout=0.1,
                    write_timeout=1.0,
                    exclusive=self._serial_exclusive,
                )
                self._serial_ready.set()
                self._emit('device_connection', True)
                self._emit('info', f'串口已连接：{self._serial_port}')
                self._run_serial_session(port)
            except (serial.SerialException, OSError) as error:
                if not self._stop_event.is_set():
                    self._emit('error', f'串口 {self._serial_port} 异常：{error}')
            finally:
                self._serial_ready.clear()
                self._emit('device_connection', False)
                self._emit('fix_valid', False)
                self._clear_latest_gga()
                self._clear_rtcm_queue()
                if port is not None and port.is_open:
                    try:
                        port.close()
                    except (serial.SerialException, OSError):
                        pass
            self._stop_event.wait(self._serial_reconnect_delay)

    def _run_serial_session(self, port: serial.SerialBase) -> None:
        framer = NmeaLineBuffer()
        last_gga_received_at = 0.0
        last_hpr_received_at = 0.0
        last_gga_request_at = time.monotonic()
        last_hpr_request_at = last_gga_request_at
        port.write(self._gga_request)
        port.write(self._hpr_request)

        gga_stale_after = max(
            self._receiver_request_retry,
            2.5 * self._gga_output_interval,
        )
        hpr_stale_after = max(
            self._receiver_request_retry,
            2.5 * self._hpr_output_interval,
        )
        while not self._stop_event.is_set():
            now = time.monotonic()
            if (
                now - last_gga_received_at >= gga_stale_after
                and now - last_gga_request_at >= self._receiver_request_retry
            ):
                port.write(self._gga_request)
                last_gga_request_at = now
            if (
                now - last_hpr_received_at >= hpr_stale_after
                and now - last_hpr_request_at >= self._receiver_request_retry
            ):
                port.write(self._hpr_request)
                last_hpr_request_at = now

            self._write_queued_rtcm(port)
            waiting = port.in_waiting
            data = port.read(waiting if waiting > 0 else 1)
            for sentence in framer.feed(data):
                self._last_nmea_at = time.monotonic()
                self._emit('nmea', sentence)
                if _is_gga_sentence(sentence):
                    try:
                        gga = parse_gga(
                            sentence,
                            validate_checksum=self._validate_checksum,
                        )
                    except ValueError:
                        self._parse_error_count += 1
                        continue
                    last_gga_received_at = time.monotonic()
                    with self._latest_gga_lock:
                        self._latest_gga = (gga, last_gga_received_at)
                    self._gga_ready.set()
                    self._emit('gga', gga)
                elif _is_hpr_sentence(sentence):
                    try:
                        hpr = parse_hpr(
                            sentence,
                            validate_checksum=self._validate_checksum,
                        )
                    except ValueError:
                        self._parse_error_count += 1
                        continue
                    last_hpr_received_at = time.monotonic()
                    with self._latest_hpr_lock:
                        self._latest_hpr = (hpr, last_hpr_received_at)
                    self._emit('hpr', hpr)

    def _write_queued_rtcm(self, port: serial.SerialBase) -> None:
        while not self._stop_event.is_set():
            try:
                chunk = self._rtcm_queue.get_nowait()
            except queue.Empty:
                return
            port.write(chunk)

    def _network_worker(self) -> None:
        if (
            not self._ntrip_enabled
            or not self._ntrip_username
            or not self._ntrip_password
        ):
            return
        while not self._stop_event.is_set():
            if not self._serial_ready.wait(timeout=0.2):
                continue
            if not self._gga_ready.wait(timeout=0.2):
                continue
            try:
                self._run_ntrip_session()
            except (NtripResponseError, OSError, RuntimeError, ValueError) as error:
                if not self._stop_event.is_set():
                    self._emit('error', f'NTRIP 连接异常：{error}')
            finally:
                self._set_ntrip_connected(False)
                self._close_network_socket()
            self._stop_event.wait(self._ntrip_reconnect_delay)

    def _run_ntrip_session(self) -> None:
        connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        connection.settimeout(self._connect_timeout)
        with self._socket_lock:
            self._network_socket = connection

        connection.connect((self._ntrip_host, self._ntrip_port))
        request = build_request(
            host=self._ntrip_host,
            port=self._ntrip_port,
            mountpoint=self._mountpoint,
            username=self._ntrip_username,
            password=self._ntrip_password,
            user_agent=self._ntrip_user_agent,
            ntrip_version=self._ntrip_version,
        )
        connection.sendall(request)

        parser = NtripResponseParser()
        initial_rtcm: Optional[bytes] = None
        while initial_rtcm is None and not self._stop_event.is_set():
            response = connection.recv(4096)
            if not response:
                raise NtripResponseError('caster closed while sending response')
            initial_rtcm = parser.feed(response)
        if self._stop_event.is_set():
            return

        connection.settimeout(0.2)
        if initial_rtcm:
            self._queue_rtcm(initial_rtcm)
        self._set_ntrip_connected(True)
        self._emit(
            'info',
            f'NTRIP 已连接：{self._ntrip_host}:{self._ntrip_port}/{self._mountpoint}',
        )

        last_gga_send = 0.0
        while not self._stop_event.is_set() and self._serial_ready.is_set():
            now = time.monotonic()
            latest = self._get_latest_gga()
            if latest is not None and now - last_gga_send >= self._gga_send_interval:
                connection.sendall(latest[0].sentence.encode('ascii') + b'\r\n')
                last_gga_send = now
            try:
                rtcm = connection.recv(4096)
            except socket.timeout:
                continue
            if not rtcm:
                raise OSError('caster closed the correction stream')
            self._queue_rtcm(rtcm)

    def _queue_rtcm(self, data: bytes) -> None:
        try:
            self._rtcm_queue.put(data, timeout=0.5)
        except queue.Full as error:
            raise RuntimeError('RTCM queue is full; serial output cannot keep up') from error

    def _get_latest_gga(self) -> Optional[Tuple[GgaData, float]]:
        with self._latest_gga_lock:
            return self._latest_gga

    def _get_latest_hpr(self) -> Optional[Tuple[HprData, float]]:
        with self._latest_hpr_lock:
            return self._latest_hpr

    def _clear_latest_gga(self) -> None:
        with self._latest_gga_lock:
            self._latest_gga = None
        self._gga_ready.clear()

    def _clear_rtcm_queue(self) -> None:
        while True:
            try:
                self._rtcm_queue.get_nowait()
            except queue.Empty:
                return

    def _set_ntrip_connected(self, connected: bool) -> None:
        with self._connection_state_lock:
            if connected == self._connection_state:
                return
            self._connection_state = connected
        self._emit('connection', connected)

    def _publish_connection_state(self, connected: bool) -> None:
        self._publish_bool(self._connection_publisher, connected)

    @staticmethod
    def _publish_bool(publisher: Any, value: bool) -> None:
        message = Bool()
        message.data = value
        publisher.publish(message)

    def _emit(self, kind: str, payload: Any) -> None:
        event = _RosEvent(kind, payload)
        try:
            self._ros_events.put_nowait(event)
        except queue.Full:
            try:
                self._ros_events.get_nowait()
            except queue.Empty:
                pass
            try:
                self._ros_events.put_nowait(event)
            except queue.Full:
                pass

    def _drain_ros_events(self) -> None:
        for _ in range(200):
            try:
                event = self._ros_events.get_nowait()
            except queue.Empty:
                break
            if event.kind == 'nmea':
                message = String()
                message.data = event.payload
                self._nmea_publisher.publish(message)
            elif event.kind == 'gga':
                self._publish_gga(event.payload)
            elif event.kind == 'hpr':
                self._publish_hpr(event.payload, fresh=True)
            elif event.kind == 'connection':
                self._publish_connection_state(bool(event.payload))
            elif event.kind == 'device_connection':
                self._publish_bool(self._device_publisher, bool(event.payload))
            elif event.kind == 'fix_valid':
                self._publish_bool(self._fix_valid_publisher, bool(event.payload))
            elif event.kind == 'info':
                self.get_logger().info(str(event.payload))
            elif event.kind == 'warning':
                self.get_logger().warning(str(event.payload))
            elif event.kind == 'error':
                self.get_logger().error(str(event.payload))
        self._check_attitude_timeout()

    def _publish_gga(self, gga: GgaData) -> None:
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self._frame_id
        fix.status.service = NavSatStatus.SERVICE_GPS
        has_fix = (
            gga.fix_quality > 0
            and math.isfinite(gga.latitude)
            and math.isfinite(gga.longitude)
        )
        fix.status.status = NavSatStatus.STATUS_FIX if has_fix else NavSatStatus.STATUS_NO_FIX
        fix.latitude = gga.latitude
        fix.longitude = gga.longitude
        fix.altitude = gga.altitude
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self._fix_publisher.publish(fix)
        self._publish_bool(self._fix_valid_publisher, has_fix)

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        latest_gga = self._get_latest_gga()
        latest_hpr = self._get_latest_hpr()
        gga_age = now - latest_gga[1] if latest_gga is not None else math.inf
        hpr_age = now - latest_hpr[1] if latest_hpr is not None else math.inf
        fix_valid = (
            latest_gga is not None
            and gga_age <= max(self._receiver_request_retry, 2.5 * self._gga_output_interval)
            and latest_gga[0].fix_quality > 0
            and math.isfinite(latest_gga[0].latitude)
            and math.isfinite(latest_gga[0].longitude)
        )
        attitude_valid = (
            latest_hpr is not None
            and hpr_age <= self._attitude_timeout
            and data_is_valid(latest_hpr[0])
        )
        device_connected = self._serial_ready.is_set()
        self._publish_bool(self._fix_valid_publisher, fix_valid)

        status = DiagnosticStatus()
        status.name = 'wtrtk_982dt_driver: GNSS receiver'
        status.hardware_id = self._serial_port
        if not device_connected:
            status.level = DiagnosticStatus.ERROR
            status.message = 'serial disconnected'
        elif not fix_valid or not attitude_valid:
            status.level = DiagnosticStatus.WARN
            status.message = 'device online, positioning not valid'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'positioning valid'
        status.values = [
            KeyValue(key='device_connected', value=str(device_connected).lower()),
            KeyValue(key='fix_valid', value=str(fix_valid).lower()),
            KeyValue(key='attitude_valid', value=str(attitude_valid).lower()),
            KeyValue(key='ntrip_enabled', value=str(self._ntrip_enabled).lower()),
            KeyValue(key='ntrip_profile', value=self._ntrip_profile),
            KeyValue(key='ntrip_connected', value=str(self._connection_state).lower()),
            KeyValue(key='gga_age_sec', value=_format_age(gga_age)),
            KeyValue(key='hpr_age_sec', value=_format_age(hpr_age)),
            KeyValue(key='last_nmea_age_sec', value=_format_age(now - self._last_nmea_at) if self._last_nmea_at else 'never'),
            KeyValue(key='parse_errors', value=str(self._parse_error_count)),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostics_publisher.publish(message)

    def _publish_hpr(self, hpr: HprData, fresh: bool) -> None:
        heading = corrected_heading(hpr.heading_deg, self._heading_offset_deg)
        attitude = GnssAttitude()
        attitude.header.stamp = self.get_clock().now().to_msg()
        attitude.header.frame_id = self._attitude_frame_id
        attitude.source_sentence = hpr.source_sentence
        attitude.utc_time = hpr.utc_time
        attitude.raw_heading_deg = hpr.heading_deg
        attitude.heading_deg = heading
        attitude.pitch_deg = hpr.pitch_deg
        attitude.roll_deg = hpr.roll_deg
        attitude.yaw_enu_rad = heading_to_enu_yaw(heading)
        attitude.solution_quality = hpr.solution_quality
        attitude.satellites = hpr.satellites
        attitude.correction_age_sec = hpr.correction_age_sec
        attitude.station_id = hpr.station_id
        attitude.data_valid = fresh and data_is_valid(hpr)
        self._attitude_publisher.publish(attitude)
        if fresh:
            self._attitude_stale_published = False

    def _check_attitude_timeout(self) -> None:
        latest = self._get_latest_hpr()
        if latest is None or self._attitude_stale_published:
            return
        hpr, received_at = latest
        if time.monotonic() - received_at < self._attitude_timeout:
            return
        self._publish_hpr(hpr, fresh=False)
        self._attitude_stale_published = True
        self.get_logger().warning(
            'HPR 数据已超时，/gnss/attitude 的 data_valid 已置为 false'
        )

    def _close_network_socket(self) -> None:
        with self._socket_lock:
            connection = self._network_socket
            self._network_socket = None
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                connection.close()
            except OSError:
                pass

    def stop(self) -> None:
        if self._stop_event.is_set():
            return
        self._stop_event.set()
        self._close_network_socket()
        self._serial_thread.join(timeout=2.0)
        self._network_thread.join(timeout=2.0)

    def destroy_node(self) -> bool:
        self.stop()
        return super().destroy_node()


def _is_gga_sentence(sentence: str) -> bool:
    identifier = sentence.split(',', 1)[0].lstrip('$').upper()
    return identifier.endswith('GGA')


def _is_hpr_sentence(sentence: str) -> bool:
    identifier = sentence.split(',', 1)[0].lstrip('$').upper()
    return identifier.endswith('HPR')


def _receiver_output_command(message_name: str, interval_sec: float) -> bytes:
    period = format(interval_sec, '.12g')
    return f'{message_name} {period}\r\n'.encode('ascii')


def _format_age(age: float) -> str:
    return f'{age:.3f}' if math.isfinite(age) else 'never'


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[GpsNtripNode] = None
    try:
        node = GpsNtripNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
