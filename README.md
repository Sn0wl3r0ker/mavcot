# MAVCOT
Publishes Cursor on Target events from a MAVLINK feed via UDP

Install:
```
git clone https://github.com/Sn0wl3r0ker/mavcot.git
cd mavcot
pip install .
```

Usage (with optional path to config file):
```
mavcot_proxy.py /path/to/mavcot.conf
```

Configuration Example:
```
# mavcot_cobalt.conf
[mavlink]
transport: udpin
address: 127.0.0.1
port: 14550

[cot]
# FreeTAKServer host IP or DNS name (UDP mode)
transport: udp
address: 127.0.0.1
# FreeTAKServer CoT ingest port
port: 8087
output_rate_hz: 1
uid: UAV_005
type: a-f-A-M-F-Q
```

FreeTAKServer mTLS example (aligned with ROS bridge style):
```
[cot]
transport: tls
address: 192.168.88.131
port: 8089
client_cert: certs/client.crt
client_key: certs/client.key
ca_cert: certs/ca.crt
verify_server: true
server_hostname:
framing: nul
reconnect_sec: 3
output_rate_hz: 1
uid: UAV_005
type: a-f-A-M-F-Q
```

Notes:
- In TLS mode, `client_cert`, `client_key`, and `ca_cert` are required.
- Relative cert paths are resolved from the config file directory.
- `framing` supports `nul` or `newline`.

On Windows, `transport: udpin` is usually the safest choice for incoming MAVLink UDP streams and helps avoid `WinError 10054`.
