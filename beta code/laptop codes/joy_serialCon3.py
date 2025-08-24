#!/usr/bin/env python3
import evdev
import serial
import sys
import threading
import time

# Konfiguracija
dev_port = '/dev/ttyUSB0'
baud_rate = 115200
watchdog_interval = 0.1  # 100ms

# Početno stanje
state = {
    'x': 127,
    'y': 127,
    'z': 255,
    'a': 0,
    'b': 0,
    'c': 0,
}

state_lock = threading.Lock()

# Pronađi joystick
def get_joystick_device():
    devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
    for device in devices:
        if "Logitech Logitech Attack 3" in device.name:
            return device
    raise Exception("Joystick not found")

# Funkcija za slanje poruke
def send_state(ser):
    with state_lock:
        msg = f"{state['x']} {state['y']} {state['z']} {state['a']} {state['b']} {state['c']}\n"
    ser.write(msg.encode('utf-8'))

# Watchdog
def watchdog_loop(ser):
    while True:
        send_state(ser)
        time.sleep(watchdog_interval)

# Čitanje joystick događaja
def joystick_loop(ser):
    device = get_joystick_device()
    print(f"Found joystick: {device.name}")

    axes_map = {
        evdev.ecodes.ABS_X: 'x',
        evdev.ecodes.ABS_Y: 'y',
        evdev.ecodes.ABS_Z: 'z',
    }

    buttons_map = {
        evdev.ecodes.BTN_TOP: 'a',
        evdev.ecodes.BTN_TRIGGER: 'b',
        evdev.ecodes.BTN_THUMB: 'c',
    }

    for event in device.read_loop():
        updated = False

        if event.type == evdev.ecodes.EV_ABS and event.code in axes_map:
            val = max(0, min(255, event.value))
            key = axes_map[event.code]
            with state_lock:
                if state[key] != val:
                    state[key] = val
                    updated = True

        elif event.type == evdev.ecodes.EV_KEY and event.code in buttons_map:
            key = buttons_map[event.code]
            val = 1 if event.value else 0
            with state_lock:
                if state[key] != val:
                    state[key] = val
                    updated = True

        if updated:
            send_state(ser)

# Ispis i spremanje IMU podataka
def serial_receive_loop(ser):
    with open("imu_rec3.csv", "a") as f:
        while True:
            if ser.in_waiting:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    print(f"Received: {data}")
                    if data.startswith("d"):
                        try:
                            payload = data[1:]  # makni prefiks 'd'
                            parts = payload.split(",")
                            if len(parts) == 3:
                                roll = float(parts[0])
                                pitch = float(parts[1])
                                heading = float(parts[2])
                                timestamp = time.time()
                                f.write(f"{timestamp:.3f},{roll:.2f},{pitch:.2f},{heading:.2f}\n")
                                f.flush()
                        except ValueError:
                            print("Greška u parsiranju podatka:", data)

# Glavna funkcija
def main():
    try:
        ser = serial.Serial(dev_port, baud_rate, timeout=0.1)
    except serial.SerialException as e:
        print(f"Error opening serial port {dev_port}: {e}")
        sys.exit(1)

    threading.Thread(target=serial_receive_loop, args=(ser,), daemon=True).start()
    threading.Thread(target=watchdog_loop, args=(ser,), daemon=True).start()

    joystick_loop(ser)

if __name__ == '__main__':
    main()
