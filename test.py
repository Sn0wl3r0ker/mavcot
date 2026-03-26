import argparse
import ipaddress
import socket
import time
from datetime import datetime, timedelta, timezone


def is_multicast(ip):
  return ipaddress.ip_address(ip).is_multicast


def parse_args():
  parser = argparse.ArgumentParser(description="Send CoT UDP packets for WinTAK testing")
  parser.add_argument("--ip", default="100.125.235.62", help="Destination IP (unicast or multicast)")
  parser.add_argument("--port", type=int, default=6969, help="Destination UDP port")
  parser.add_argument("--rate", type=float, default=1.0, help="Packets per second")
  parser.add_argument("--uid", default="UAV_NYCUtest", help="CoT uid")
  parser.add_argument("--callsign", default="UAV_NYCUtest", help="CoT callsign")
  return parser.parse_args()


args = parse_args()
dest_ip = args.ip
dest_port = args.port
interval = 1.0 / max(args.rate, 0.1)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
if is_multicast(dest_ip):
  sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
  sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)

print(f"Sending CoT to {dest_ip}:{dest_port} (multicast={is_multicast(dest_ip)})")

while True:
    now = datetime.now(timezone.utc)
    stale = now + timedelta(seconds=60)

    cot = f'''<?xml version="1.0" standalone="yes"?>
<event version="2.0"
      uid="{args.uid}"
       type="a-f-A-M-F-Q"
       how="m-g"
       time="{now.strftime('%Y-%m-%dT%H:%M:%SZ')}"
       start="{now.strftime('%Y-%m-%dT%H:%M:%SZ')}"
       stale="{stale.strftime('%Y-%m-%dT%H:%M:%SZ')}">
  <point lat="24.7859542"
         lon="120.9973364"
         hae="210.9"
         ce="10.0"
         le="10.0"/>
  <detail>
    <contact callsign="{args.callsign}"/>
    <track course="0.0" speed="0.0"/>
  </detail>
</event>'''

    sock.sendto(cot.encode("utf-8"), (dest_ip, dest_port))
    print(f"sent -> {dest_ip}:{dest_port} @ {now.strftime('%H:%M:%S')}")
    time.sleep(interval)
