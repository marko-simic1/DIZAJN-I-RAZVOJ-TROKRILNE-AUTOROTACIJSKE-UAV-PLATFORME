#!/usr/bin/env python3
import serial
import sys
import time
import csv
import os

# Konfiguracija
dev_port = '/dev/ttyUSB0'
baud_rate = 115200
csv_file = 'imu9_log2.csv'

# Inicijalizacija CSV datoteke
def init_csv():
    file_exists = os.path.isfile(csv_file)
    with open(csv_file, 'a', newline='') as f:
        writer = csv.writer(f)
        if not file_exists:
            # Header redoslijed mora odgovarati poslanim vrijednostima
            writer.writerow([
                'timestamp',
                'ax', 'ay', 'az',
                'gx', 'gy', 'gz',
                'mx', 'my', 'mz'
            ])

# Spremanje jednog retka u CSV 
def log_imu9_data(data_list):
    with open(csv_file, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([time.time()] + data_list)

def receive_loop(ser):
    while True:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("IMU9:"):
                try:
                    payload = line.replace("IMU9:", "")
                    values = [float(v) for v in payload.split(",")]
                    if len(values) == 9:
                        log_imu9_data(values)
                        print(f"Logged: {values}")
                except Exception as e:
                    print(f"Error parsing line: {line} -- {e}")

def main():
    init_csv()

    try:
        ser = serial.Serial(dev_port, baud_rate, timeout=0.1)
    except serial.SerialException as e:
        print(f"Error opening serial port {dev_port}: {e}")
        sys.exit(1)

    print(f"Listening on {dev_port} at {baud_rate} baud...")
    receive_loop(ser)

if __name__ == '__main__':
    main()
