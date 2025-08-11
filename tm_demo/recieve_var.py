#!/usr/bin/env python
import socket
import struct


HOST = '192.168.98.231'  # IP address of your computer
PORT = 5000          # Port number used in TMFlow TCP settings

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f"Listening for TMFlow data on {HOST}:{PORT}")

    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)
        while True:
            data = conn.recv(1024)
            if not data:
                break
            try:
                # Unpack data; '>6f' means six floats in big-endian format
                x, y, z, roll, pitch, yaw = struct.unpack('>6f', data)
                print(f"X: {x}, Y: {y}, Z: {z}, Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
            except struct.error as e:
                print(f"Error decoding data: {e}")
            print("Received data:", data)
            # Parse and use the received data
