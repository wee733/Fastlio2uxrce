"""Read only LiDAR packet metadata, not point cloud content or unrelated traffic."""
import json
import socket
import struct
import time

sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0800))
sock.bind(('enP8p1s0', 0))
sock.settimeout(3)
end = time.monotonic() + 3
while time.monotonic() < end:
    packet = sock.recv(65535)
    if (len(packet) < 80 or packet[23] != 17 or
            socket.inet_ntoa(packet[26:30]) != '192.168.1.150'):
        continue
    offset = 14 + (packet[14] & 15) * 4
    _, destination = struct.unpack_from('!HH', packet, offset)
    if destination != 56301:
        continue
    payload = packet[offset + 8:]
    stamp = struct.unpack_from('<Q', payload, 28)[0]
    now = time.time_ns()
    print(json.dumps({'source': '192.168.1.150', 'time_type': payload[11],
                      'packet_timestamp_ns': stamp, 'host_epoch_ns': now,
                      'host_minus_packet_ms': (now - stamp) * 1e-6}))
    break
else:
    print('No matching point packet in 3 seconds')
sock.close()
