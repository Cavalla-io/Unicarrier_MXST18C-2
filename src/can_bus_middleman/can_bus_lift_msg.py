import can
import time
import threading


class LiftController:
    def __init__(self, bus, arbitration_id_1A0):
        self.bus = bus
        self.arbitration_id_1A0 = arbitration_id_1A0
        self.lifting = False
        self.counter_1A0 = 0

    def send_can_message(self, arbitration_id, data):
        """Send a CAN message on the specified bus."""
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            print(f"Message sent: {msg}")
        except can.CanError as e:
            print(f"Failed to send message: {e}")

    def start_frame_1A0(self, direction):
        """Thread for sending frame 1A0 at 65 Hz."""
        while self.lifting:
            lift_state_byte0 = 0xFF if self.lifting else 0x00
            lift_action = 0x07 if direction == "UP" else 0xF8
            heartbeat = 0x00 if self.counter_1A0 % 2 == 0 else 0x80

            message_data = [
                lift_state_byte0,  # Byte 0: Lift state
                lift_action,       # Byte 1: Lift action
                0x00, 0x00, 0x00, 0x00,  # Bytes 2-5: Always 0
                0x02,              # Byte 6: Solid 0x02
                heartbeat          # Byte 7: Heartbeat alternating 0x00 and 0x80
            ]
            self.send_can_message(self.arbitration_id_1A0, message_data)

            self.counter_1A0 += 1
            time.sleep(1 / 65)  # ~65 Hz

    def start_lift(self, direction):
        """
        Start the lift operation.
        :param direction: "UP" for lifting up, "DOWN" for lowering.
        """
        if direction not in ["UP", "DOWN"]:
            print("Invalid direction. Use 'UP' or 'DOWN'.")
            return

        self.lifting = True
        # Start thread for frame 1A0
        threading.Thread(target=self.start_frame_1A0, args=(direction,), daemon=True).start()

    def stop_lift(self):
        """Stop the lift operation."""
        self.lifting = False
        print("Lift operation stopped.")


# Example usage
if __name__ == "__main__":
    bus = can.interface.Bus(channel='can0', bustype='socketcan')
    lift_controller = LiftController(bus, arbitration_id_1A0=0x1A0)

    try:
        lift_controller.start_lift("UP")
        time.sleep(10)  # Run the lift for 10 seconds
        lift_controller.stop_lift()
    except KeyboardInterrupt:
        lift_controller.stop_lift()
