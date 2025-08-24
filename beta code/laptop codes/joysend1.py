#!/usr/bin/env python3
import threading
import time
import evdev
import serial
import sys

# Konfiguracija
dev_port = '/dev/ttyUSB0'
baud_rate = 115200
watchdog_interval = 0.5  # sekunde

# Dijeljeni podaci za watchdog
data_lock = threading.Lock()
last_send_time = 0
last_message = None

# Početno stanje
state = {
    'x': None,
    'y': None,
    'z': None,
    'a': 0,
    'b': 0,
    'c': 0,
}

# Pronalaženje jystick-a
def get_joystick_device():
    devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
    for device in devices:
        if "Logitech Logitech Attack 3" in device.name:
            return device
    raise Exception("Joystick not found")

# Watchdog koji ponavlja zadnju poruku ako se ništa ne šalje
def watchdog_task(ser):
    global last_send_time, last_message
    while True:
        time.sleep(watchdog_interval)
        with data_lock:
            if last_message and (time.time() - last_send_time) >= watchdog_interval:
                ser.write(last_message.encode('utf-8'))
                print(f"[WATCHDOG] Resent: {last_message.strip()}")
                last_send_time = time.time()

# Funkcija koja gradi i šalje trenutnu poruku
def send_state(ser):
    global last_send_time, last_message
    msg = f"x:{state['x']} y:{state['y']} z:{state['z']} " \
          f"a:{state['a']} b:{state['b']} c:{state['c']}\r\n"
    ser.write(msg.encode('utf-8'))
    print(f"Sent: {msg.strip()}")
    with data_lock:
        last_message = msg
        last_send_time = time.time()

# Čitanje događaja i update stanja
def read_joystick_send(ser):
    device = get_joystick_device()
    print(f"Found joystick: {device.name}")

    # mapiranja kodova na naše ključeve
    axes_map = {
        evdev.ecodes.ABS_X: 'x',
        evdev.ecodes.ABS_Y: 'y',
        evdev.ecodes.ABS_Z: 'z',
    }
    buttons_map = {
        evdev.ecodes.BTN_BASE2: 'a',
        evdev.ecodes.BTN_BASE3: 'b',
        evdev.ecodes.BTN_BASE4: 'c',
    }

    for event in device.read_loop():
        updated = False

        # apsolutne osi
        if event.type == evdev.ecodes.EV_ABS and event.code in axes_map:
            raw = event.value
            val = max(0, min(255, raw))
            key = axes_map[event.code]
            if state[key] != val:
                state[key] = val
                updated = True

        # digitalni gumbi
        elif event.type == evdev.ecodes.EV_KEY and event.code in buttons_map:
            key = buttons_map[event.code]
            val = 1 if event.value else 0
            if state[key] != val:
                state[key] = val
                updated = True
        
        if updated:
            if None not in (state['x'], state['y'], state['z']):
                send_state(ser)

        
def main():
    try:
        ser = serial.Serial(dev_port, baud_rate, timeout=0)
    except serial.SerialException as e:
        print(f"Error opening serial port {dev_port}: {e}")
        sys.exit(1)

    
    t = threading.Thread(target=watchdog_task, args=(ser,), daemon=True)
    t.start()
    read_joystick_send(ser)

if __name__ == '__main__':
    main()
