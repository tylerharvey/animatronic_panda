#!/usr/bin/env python3
import sys
import struct
from panda import Panda
import esptool

class PandaSerialWrapper:
    """Ported from animatronic_panda FakePort"""
    def __init__(self, serial=None):
        self._port = FakePort(port)
        self.panda = Panda(serial)
        self.timeout = 1.0
        self._baudrate = 230400 # Initial bootloader baud

    @property
    def baudrate(self):
        return self._baudrate

    @baudrate.setter
    def baudrate(self, baud):
        print(f"Setting Panda UART1 baud to {baud}")
        self.panda.set_uart_baud(1, baud)
        self._baudrate = baud

    def write(self, data):
        # Ported SEND_STEP logic
        SEND_STEP = 0x20
        for i in range(0, len(data), SEND_STEP):
            self.panda.serial_write(1, data[i:i+SEND_STEP])

    def read(self, size=1):
        # Panda serial_read returns a list/buffer from the specified port
        return self.panda.serial_read(1)

    def flushInput(self):
        self.panda.serial_clear(1)

    def close(self):
        pass

def main():
    # 1. Initialize the Panda hardware
    print("Connecting to Panda...")
    panda_port = PandaSerialWrapper()

    # 2. Mimic the modern esptool CLI but inject our Panda port
    # This allows you to use all modern flags (--chip esp8266, etc.)
    print(sys.version)
    # esp = detect_chip(panda_port)
    esp = esptool.cmds.detect_chip(panda_port)
    
    # Manually trigger the sync sequence
    esp.connect()
    
    # Run the standard esptool main entry point with the injected port
    esptool.main(sys.argv[1:], esp=esp)

if __name__ == "__main__":
    main()
