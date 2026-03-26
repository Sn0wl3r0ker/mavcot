#!/usr/bin/env python3

import ctypes
import traceback
from pymavlink import mavutil
from datetime import datetime
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
    cot_rate_hz = config.getfloat('cot','output_rate_hz')
    cot_uid = config.get('cot', 'uid')
    cot_type = config.get('cot', 'type')

    # Configure Socket Connection
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    disable_windows_udp_connreset(s, 'CoT UDP socket')

    address = (cot_address, cot_port)

    # Assert Mavlink 2
    os.environ['mavlink20'] = "1"

    print("Waiting for MAVLink Socket")
    print(f"Using MAVLink connection: {mavlink_address_string}")
    mav = mavutil.mavlink_connection(mavlink_address_string, retries=20)

    for sock_candidate in iter_socket_candidates(mav):
        disable_windows_udp_connreset(sock_candidate, f'MAVLink socket {type(sock_candidate).__name__}')

    print("UDP Listening, waiting for heartbeat")
    mav.wait_heartbeat()
    print("Connected, Waiting for fix")
    mav.wait_gps_fix()
    print("Fix acquired, running")

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

            timestamp_string = datetime.now().strftime('%Y-%m-%dT%H:%M:%S.%fZ')

            ''' Assemble XML Cot Message '''
            cot_event_element = ET.Element('event')
            cot_event_element.set('xmlns:xsi', 'http://www.w3.org/2001/XMLSchema-instance')
            cot_event_element.set('xmlns:xsd', 'http://www.w3.org/2001/XMLSchema')
            cot_event_element.set('uid', cot_uid)
            cot_event_element.set('type', cot_type)
            cot_event_element.set('time', timestamp_string)
            cot_event_element.set('start', timestamp_string)
            cot_event_element.set('stale', timestamp_string)

            cot_point_element = ET.SubElement(cot_event_element, 'point')
            cot_point_element.set('lat', str(lat))
            cot_point_element.set('lon', str(lon))
            cot_point_element.set('hae', str(hae))

            cot_detail_element = ET.SubElement(cot_event_element, 'detail')
            cot_track_element = ET.SubElement(cot_detail_element, 'track')
            cot_track_element.set('course', str(course_degrees))
            cot_track_element.set('speed', str(speed_3d_ms))
            cot_track_element.set('slope', str(slope_degrees))

            event_xml_string = ET.tostring(cot_event_element, encoding='unicode')
            header_string = '<?xml version="1.0" standalone="yes"?>'
            out_cot_xml = header_string + event_xml_string
            print(out_cot_xml)
            try:
                s.sendto(out_cot_xml.encode('utf-8'), address)
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
