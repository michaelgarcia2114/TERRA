import socket
import csv
from datetime import datetime

PORT = 12345

# Create the UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Attach it to port 12345
sock.bind(("", PORT))

with open("reading.csv", "a", newline="") as file:
    writer = csv.writer(file)

    print(f"Listening on port {PORT}")

    while True:
        data, addr = sock.recvfrom(1024)

        soil = data.decode()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        writer.writerow([timestamp, soil])
        file.flush()

        print(f"{timestamp} | {soil}")