#!/usr/bin/env python3
import evdev
import serial
import sys
import threading

from evdev import InputDevice, list_devices

dev = InputDevice(list_devices()[0])
print(f"Device: {dev.name}")

for event in dev.read_loop():
    print(event)
