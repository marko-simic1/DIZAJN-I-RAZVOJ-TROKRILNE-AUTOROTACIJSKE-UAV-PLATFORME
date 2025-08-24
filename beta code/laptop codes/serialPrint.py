#!/usr/bin/env python3
import evdev
import serial
import sys
import threading

# Konfiguracija
dev_port = '/dev/ttyUSB0'
baud_rate = 115200


def serial_receive_loop(ser):
    while True:
        if ser.in_waiting:
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            if data:
                print(f"Received: {data}")

def main():
    try:
        ser = serial.Serial(dev_port, baud_rate, timeout=0.1)
    except serial.SerialException as e:
        print(f"Error opening serial port {dev_port}: {e}")
        sys.exit(1)

    serial_receive_loop(ser)

if __name__ == '__main__':
    main()
