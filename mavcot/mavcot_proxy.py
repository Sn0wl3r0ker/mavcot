#!/usr/bin/env python3

import ctypes
import traceback
import ipaddress
import ssl
from pymavlink import mavutil
from datetime import datetime, timedelta
from mavcot.helpers import get_geoid_height
import xml.etree.ElementTree as ET
import os, sys, time, socket, math, configparser, importlib.resources

SIO_UDP_CONNRESET = 0x9800000C


def disable_windows_udp_connreset(sock_obj, label):
    if os.name != 'nt' or sock_obj is None or not hasattr(sock_obj, 'fileno'):
        return

    disable_flag = ctypes.c_uint32(0)
    bytes_returned = ctypes.c_uint32(0)

    try:
        result = ctypes.windll.ws2_32.WSAIoctl(
            ctypes.c_size_t(sock_obj.fileno()),
            SIO_UDP_CONNRESET,
            ctypes.byref(disable_flag),
            ctypes.sizeof(disable_flag),
            None,
            0,
            ctypes.byref(bytes_returned),
            None,
            None,
        )
        if result != 0:
            error_code = ctypes.windll.ws2_32.WSAGetLastError()
            print(f'Warning: could not disable UDP connreset on {label} (WSA error {error_code})')
    except Exception as exc:
        print(f'Warning: could not disable UDP connreset on {label}: {exc}')


def iter_socket_candidates(mav_connection):
    seen = set()
    for candidate in (
        mav_connection,
        getattr(mav_connection, 'port', None),
        getattr(getattr(mav_connection, 'port', None), 'sock', None),
    ):
        if candidate is None:
            continue
        candidate_id = id(candidate)
        if candidate_id in seen:
            continue
        seen.add(candidate_id)
        yield candidate


def build_mavlink_connection_string(config):
    configured_connection = config.get('mavlink', 'connection_string', fallback='').strip()
    if configured_connection:
        return configured_connection

    mav_address = config.get('mavlink', 'address')
    mav_port = config.getint('mavlink', 'port')
    default_transport = 'udpin' if os.name == 'nt' else 'udp'
    mav_transport = config.get('mavlink', 'transport', fallback=default_transport).strip() or default_transport
    return f'{mav_transport}:{mav_address}:{mav_port}'


def is_multicast_address(address):
    try:
        return ipaddress.ip_address(address).is_multicast
    except ValueError:
        return False


def parse_bool(value, default=False):
    if value is None:
        return default
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def resolve_config_path(config_path, maybe_relative_path):
    path = (maybe_relative_path or '').strip()
    if not path:
        return ''
    if os.path.isabs(path):
        return path
    return os.path.abspath(os.path.join(os.path.dirname(config_path), path))


class TLSSocketSender:
    def __init__(
        self,
        host,
        port,
        client_cert,
        client_key,
        ca_cert,
        verify_server=True,
        server_hostname=None,
        framing='nul',
        reconnect_sec=3.0,
    ):
        self.host = host
        self.port = port
        self.client_cert = client_cert
        self.client_key = client_key
        self.ca_cert = ca_cert
        self.verify_server = verify_server
        self.server_hostname = server_hostname or host
        self.reconnect_sec = max(0.5, float(reconnect_sec))
        self.delimiter = b'\x00' if framing == 'nul' else b'\n'
        self.sock = None
        self.ctx = self._build_ssl_context()

    def _build_ssl_context(self):
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ctx.load_cert_chain(certfile=self.client_cert, keyfile=self.client_key)

        if self.verify_server:
            ctx.load_verify_locations(cafile=self.ca_cert)
            ctx.verify_mode = ssl.CERT_REQUIRED
            ctx.check_hostname = bool(self.server_hostname)
        else:
            ctx.verify_mode = ssl.CERT_NONE
            ctx.check_hostname = False

        return ctx

    def _connect(self):
        raw = socket.create_connection((self.host, self.port), timeout=10)
        raw.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock = self.ctx.wrap_socket(raw, server_hostname=self.server_hostname)
        print(f'FTS mTLS connected to {self.host}:{self.port}')

    def _close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def send(self, cot_xml):
        payload = cot_xml.encode('utf-8') + self.delimiter

        while True:
            try:
                if self.sock is None:
                    self._connect()
                self.sock.sendall(payload)
                return
            except Exception as exc:
                print(f'FTS TLS Socket Error: {exc}')
                self._close()
                time.sleep(self.reconnect_sec)


def wait_for_heartbeat(mav, status_interval=5):
    print("UDP Listening, waiting for heartbeat")
    last_status_time = time.time()

    while True:
        heartbeat = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if heartbeat is not None:
            print(
                "Heartbeat received: "
                f"sysid={mav.target_system} "
                f"compid={mav.target_component} "
                f"type={getattr(heartbeat, 'type', 'unknown')} "
                f"autopilot={getattr(heartbeat, 'autopilot', 'unknown')}"
            )
            return heartbeat

        now = time.time()
        if now - last_status_time >= status_interval:
            print("Still waiting for heartbeat...")
            last_status_time = now


def wait_for_gps_data(mav, status_interval=5):
    print("Heartbeat locked, waiting for GPS data")
    last_status_time = time.time()
    last_gps_report = None

    while True:
        msg = mav.recv_match(type=['GPS_RAW_INT', 'GLOBAL_POSITION_INT'], blocking=True, timeout=1)
        if msg is None:
            now = time.time()
            if now - last_status_time >= status_interval:
                if last_gps_report is None:
                    print("Still waiting for GPS data...")
                else:
                    print(f"Still waiting for usable position data. Last GPS status: {last_gps_report}")
                last_status_time = now
            continue

        if msg.get_type() == 'GPS_RAW_INT':
            fix_type = getattr(msg, 'fix_type', None)
            satellites_visible = getattr(msg, 'satellites_visible', None)
            last_gps_report = f"fix_type={fix_type}, satellites_visible={satellites_visible}"
            print(f"GPS_RAW_INT received: {last_gps_report}")

            if fix_type is not None and fix_type >= 2:
                print("GPS fix acquired from GPS_RAW_INT")
                return

        elif msg.get_type() == 'GLOBAL_POSITION_INT':
            lat = getattr(msg, 'lat', 0) / 10000000.0
            lon = getattr(msg, 'lon', 0) / 10000000.0
            alt_m = getattr(msg, 'alt', 0) / 1000.0
            print(
                "GLOBAL_POSITION_INT received: "
                f"lat={lat:.7f} lon={lon:.7f} alt_msl_m={alt_m:.2f}"
            )
            return

def main():
    try:
        # For Python 3.9+ we use importlib.resources.files
        config_path = str(importlib.resources.files('mavcot').joinpath('mavcot.conf'))
    except AttributeError:
        # Fallback for older python / setuptools / execution from source
        from pathlib import Path
        config_path = str(Path(__file__).parent / 'mavcot.conf')

    # allow user to specify a custom config path
    if len(sys.argv) > 1:
        config_path = sys.argv[1]

    # Parse user configuration
    config = configparser.RawConfigParser()
    config.read(config_path)

    mavlink_address_string = build_mavlink_connection_string(config)

    cot_address = config.get('cot', 'address')
    cot_port = config.getint('cot', 'port')

    # Backward/forward compatibility:
    # - legacy key: output_rate_hz
    # - ROS-style key: send_rate_hz
    if config.has_option('cot', 'output_rate_hz'):
        cot_rate_hz = config.getfloat('cot', 'output_rate_hz')
    elif config.has_option('cot', 'send_rate_hz'):
        cot_rate_hz = config.getfloat('cot', 'send_rate_hz')
    else:
        cot_rate_hz = 1.0

    if cot_rate_hz <= 0:
        raise ValueError('[cot] output_rate_hz/send_rate_hz must be > 0')

    cot_uid = config.get('cot', 'uid', fallback='UAV_NYCU')
    cot_type = config.get('cot', 'type', fallback='a-f-A-M-F-Q')
    cot_transport = config.get('cot', 'transport', fallback='udp').strip().lower()

    if cot_transport not in ('udp', 'tls'):
        raise ValueError("[cot] transport must be 'udp' or 'tls'")

    s = None
    tls_sender = None

    if cot_transport == 'udp':
        # Configure UDP Socket Connection
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # WinTAK multicast subscriptions are more reliable when loopback and TTL are explicit.
        if is_multicast_address(cot_address):
            s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
            s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)

        disable_windows_udp_connreset(s, 'CoT UDP socket')

        address = (cot_address, cot_port)
        print(f'CoT output transport: UDP -> {cot_address}:{cot_port}')
    else:
        client_cert = resolve_config_path(config_path, config.get('cot', 'client_cert', fallback=''))
        client_key = resolve_config_path(config_path, config.get('cot', 'client_key', fallback=''))
        ca_cert = resolve_config_path(config_path, config.get('cot', 'ca_cert', fallback=''))
        server_hostname = config.get('cot', 'server_hostname', fallback='').strip() or None
        verify_server = parse_bool(config.get('cot', 'verify_server', fallback='true'), default=True)
        framing = config.get('cot', 'framing', fallback='nul').strip().lower()
        reconnect_sec = config.getfloat('cot', 'reconnect_sec', fallback=3.0)

        if framing not in ('nul', 'newline'):
            raise ValueError("[cot] framing must be 'nul' or 'newline'")

        if not client_cert or not client_key or not ca_cert:
            raise ValueError(
                "TLS mode requires [cot] client_cert, client_key, and ca_cert in config."
            )

        tls_sender = TLSSocketSender(
            host=cot_address,
            port=cot_port,
            client_cert=client_cert,
            client_key=client_key,
            ca_cert=ca_cert,
            verify_server=verify_server,
            server_hostname=server_hostname,
            framing=framing,
            reconnect_sec=reconnect_sec,
        )
        print(f'CoT output transport: TLS -> {cot_address}:{cot_port} (framing={framing})')

    # Assert Mavlink 2
    os.environ['mavlink20'] = "1"

    print("Waiting for MAVLink Socket")
    print(f"Using MAVLink connection: {mavlink_address_string}")
    mav = mavutil.mavlink_connection(mavlink_address_string, retries=20)

    for sock_candidate in iter_socket_candidates(mav):
        disable_windows_udp_connreset(sock_candidate, f'MAVLink socket {type(sock_candidate).__name__}')

    wait_for_heartbeat(mav)
    wait_for_gps_data(mav)
    print("GPS ready, running")

    last_sent_time = time.time()

    while True:
        try:
            msg = mav.recv_match(type='GLOBAL_POSITION_INT')
        except Exception as e:
            if getattr(e, 'winerror', None) == 10054:
                print(
                    "MAVLink Socket Error: [WinError 10054] Windows received a UDP reset. "
                    "If needed, set [mavlink] transport: udpin or provide connection_string explicitly."
                )
            else:
                print('MAVLink Socket Error', e)
            time.sleep(1)
            msg = None
        if msg is not None and ((time.time() - last_sent_time) > (1/cot_rate_hz)):

            ''' Extract Position and Velocity Data from Mavlink Message'''
            lat = msg.lat / 10000000.0
            lon = msg.lon / 10000000.0
            alt_msl_m = msg.alt / 1000.0
            v_north_ms = msg.vx / 100.0
            v_east_ms = msg.vy / 100.0
            v_down_ms = msg.vz / 100.0

            ''' CoT Represents motion as 3d Speed, Course, and Slope '''
            groundspeed_ms = math.sqrt(v_north_ms**2 + v_east_ms**2)
            speed_3d_ms = math.sqrt(v_north_ms**2 + v_east_ms**2 + v_down_ms**2)

            course_degrees = math.atan2(v_east_ms, v_north_ms) * 180 / math.pi
            if course_degrees < 0:
                course_degrees = 360 + course_degrees

            slope_degrees = math.atan2(v_down_ms, groundspeed_ms) * 180 / math.pi 
            slope_degrees = max(min(slope_degrees, 90), -90) # constrain to +/= 90 deg

            ''' CoT height uses HAE, not MSL. Must calculate conversion from Geoid to Ellipsod height '''
            hae = alt_msl_m + get_geoid_height(lat,lon)

            now_utc = datetime.utcnow()
            stale_utc = now_utc + timedelta(seconds=60)
            timestamp_string = now_utc.strftime('%Y-%m-%dT%H:%M:%S.%fZ')
            stale_string = stale_utc.strftime('%Y-%m-%dT%H:%M:%S.%fZ')

            ''' Assemble XML Cot Message '''
            cot_event_element = ET.Element('event')
            cot_event_element.set('xmlns:xsi', 'http://www.w3.org/2001/XMLSchema-instance')
            cot_event_element.set('xmlns:xsd', 'http://www.w3.org/2001/XMLSchema')
            cot_event_element.set('version', '2.0')
            cot_event_element.set('uid', cot_uid)
            cot_event_element.set('type', cot_type)
            cot_event_element.set('how', 'm-g')
            cot_event_element.set('time', timestamp_string)
            cot_event_element.set('start', timestamp_string)
            cot_event_element.set('stale', stale_string)

            cot_point_element = ET.SubElement(cot_event_element, 'point')
            cot_point_element.set('lat', str(lat))
            cot_point_element.set('lon', str(lon))
            cot_point_element.set('hae', str(hae))
            cot_point_element.set('ce', '10.0')
            cot_point_element.set('le', '10.0')

            cot_detail_element = ET.SubElement(cot_event_element, 'detail')

            # WinTAK commonly expects a contact callsign for stable map labeling.
            cot_contact_element = ET.SubElement(cot_detail_element, 'contact')
            cot_contact_element.set('callsign', cot_uid)

            cot_track_element = ET.SubElement(cot_detail_element, 'track')
            cot_track_element.set('course', str(course_degrees))
            cot_track_element.set('speed', str(speed_3d_ms))
            cot_track_element.set('slope', str(slope_degrees))

            cot_precision_element = ET.SubElement(cot_detail_element, 'precisionlocation')
            cot_precision_element.set('altsrc', 'GPS')
            cot_precision_element.set('geopointsrc', 'GPS')

            event_xml_string = ET.tostring(cot_event_element, encoding='unicode')
            header_string = '<?xml version="1.0" standalone="yes"?>'
            out_cot_xml = header_string + event_xml_string
            print(out_cot_xml)
            try:
                if cot_transport == 'udp':
                    s.sendto(out_cot_xml.encode('utf-8'), address)
                else:
                    tls_sender.send(out_cot_xml)
                last_sent_time = time.time()
            except Exception as e:
                print('COT Socket Error:', e)
                time.sleep(1) # Never run faster than 1 Hz when the UDP connection is erroring

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        traceback.print_exc()
        input("\n[Error occurred] Press Enter to exit...")
