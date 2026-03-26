import socket
import time
from datetime import datetime, timedelta, timezone

MCAST_GRP = "127.0.0.1"
PORT = 6969

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)

while True:
    now = datetime.now(timezone.utc)
    stale = now + timedelta(seconds=60)

    cot = f'''<?xml version="1.0" standalone="yes"?>
<event version="2.0"
       uid="UAV_NYCU"
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
    <contact callsign="UAV_NYCU"/>
    <track course="0.0" speed="0.0"/>
  </detail>
</event>'''

    sock.sendto(cot.encode("utf-8"), (MCAST_GRP, PORT))
    print("sent")
    time.sleep(1)