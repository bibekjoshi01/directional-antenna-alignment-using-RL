import socket
import sys

UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 4210  # Must match transmitter UDP port
BUFFER_SIZE = 1024
IDLE_TIMEOUT_SEC = 5

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

try:
    sock.bind((UDP_IP, UDP_PORT))
except OSError as err:
    print(f"Bind failed on {UDP_IP}:{UDP_PORT}: {err}")
    print("Another process may already be using this UDP port.")
    sys.exit(1)

sock.settimeout(IDLE_TIMEOUT_SEC)
print(f"Listening for UDP packets on {UDP_IP}:{UDP_PORT}...")

try:
    while True:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            print(
                f"Received {len(data)} bytes from {addr[0]}:{addr[1]}: "
                f"{data.decode(errors='replace')}"
            )
        except socket.timeout:
            print("No packet received yet.")
except KeyboardInterrupt:
    print("\nReceiver stopped.")
finally:
    sock.close()
