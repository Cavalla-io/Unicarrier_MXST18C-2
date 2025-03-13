#!/usr/bin/env python3
import can
import time

class LiftController:
    def __init__(self, input_listener):
        self.listener = input_listener
        try:
            self.bus = can.interface.Bus(channel='can0', interface='socketcan')
        except Exception as e:
            print(f"LiftController: Error initializing CAN bus: {e}")
            raise
        self.counter = 0
        self.lowering_toggle = True

    def send_can_message(self, arbitration_id, data):
        msg = can.Message(arbitration_id=arbitration_id, data=bytearray(data), is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"LiftController: CAN send error: {e}")

    def update(self):
        # Get current lift command from the listener.
        cmd = self.listener.get_lift_command()
        # Set default values for each data byte.
        current_state = 0
        current_action = 0
        tilt_byte_2 = 0
        tilt_byte_3 = 0
        sideshift_byte_4 = 0
        sideshift_byte_5 = 0
        drive_byte = 0x02  # neutral drive
        # Process lift commands:
        if cmd["lift"]:
            current_state = 0xFF
            current_action = 0x07
        elif cmd["lower"]:
            current_state = 0x00
            current_action = 0xF8 if self.lowering_toggle else 0xFF
            self.lowering_toggle = not self.lowering_toggle
        # Process sideshift commands:
        if cmd["sideshift"] == 'L':
            sideshift_byte_4 = 0x01
            sideshift_byte_5 = 0xF8
        elif cmd["sideshift"] == 'R':
            sideshift_byte_4 = 0xFF
            sideshift_byte_5 = 0x07
        # Process drive command:
        if cmd["drive"] == "FORWARD":
            drive_byte = 0x01
        elif cmd["drive"] == "BACKWARD":
            drive_byte = 0x04
        else:
            drive_byte = 0x02
        # Process tilt command:
        if cmd["tilt"] == "TILT_DOWN":
            tilt_byte_2 = 0x01
            tilt_byte_3 = 0xF8
        elif cmd["tilt"] == "TILT_UP":
            tilt_byte_2 = 0xFF
            tilt_byte_3 = 0x07
        else:
            tilt_byte_2 = 0
            tilt_byte_3 = 0
        # Heartbeat (alternates every update).
        heartbeat = 0x00 if self.counter % 2 == 0 else 0x80
        self.counter += 1

        message_data = [
            current_state,      # Byte 0: Lift state
            current_action,     # Byte 1: Lift action
            tilt_byte_2,        # Byte 2: Tilt control
            tilt_byte_3,        # Byte 3: Tilt control
            sideshift_byte_4,   # Byte 4: Sideshift control
            sideshift_byte_5,   # Byte 5: Sideshift control
            drive_byte,         # Byte 6: Drive command
            heartbeat           # Byte 7: Heartbeat
        ]
        self.send_can_message(0x1A0, message_data)

    def close(self):
        self.bus.shutdown()
