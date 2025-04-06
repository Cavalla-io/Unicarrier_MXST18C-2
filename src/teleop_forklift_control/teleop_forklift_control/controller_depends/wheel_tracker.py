#!/usr/bin/env python3
import can
import time

# --- Configuration ---
CAN_CHANNEL = 'can0'
CAN_BITRATE = 250000
TARGET_FRAME_ID = 0x199

class WheelTracker:
    def __init__(self):
        self.prev_raw_value = None
        self.total_revolutions = 0
        self.reference_value = 0xFF  # The "forward" position

    def translate_encoder_to_angle(self, raw_value):
        """
        Translate an 8-bit encoder reading (0-255) into an angle (in degrees)
        relative to the straight forward position (0xFF), allowing for full 360-degree tracking.
        
        This function keeps track of previous values to detect wrap-around and count
        complete revolutions. This allows for tracking angles beyond ±180° from the reference point.
        """
        if self.prev_raw_value is None:
            self.prev_raw_value = raw_value
            return 0.0
        
        # Detect wrap-around
        if self.prev_raw_value > 200 and raw_value < 50:
            # Clockwise wrap around (255 -> 0)
            self.total_revolutions += 1
        elif self.prev_raw_value < 50 and raw_value > 200:
            # Counter-clockwise wrap around (0 -> 255)
            self.total_revolutions -= 1
            
        # Store current value for next comparison
        self.prev_raw_value = raw_value
        
        # Calculate continuous angle
        # First get the base position within the current revolution
        current_position = raw_value * (360.0 / 256)
        # Then add the contribution from complete revolutions
        total_angle = current_position + (self.total_revolutions * 360.0)
        
        # Adjust relative to the reference position
        reference_position = self.reference_value * (360.0 / 256)
        relative_angle = total_angle - reference_position
        
        return relative_angle

# Create a standalone function that wheel_tracker_node.py is trying to import
# This maintains a global instance of WheelTracker to persist state between calls
_global_wheel_tracker = WheelTracker()
def translate_encoder_to_angle(raw_value):
    """
    Standalone function that delegates to the WheelTracker class method.
    This allows importing this function directly from the module.
    """
    return _global_wheel_tracker.translate_encoder_to_angle(raw_value)

def main():
    try:
        bus = can.interface.Bus(bustype='socketcan', channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
    except Exception as e:
        print("Error initializing CAN bus:", e)
        return

    # print(f"Listening for CAN frame 0x{TARGET_FRAME_ID:X} on channel {CAN_CHANNEL}...")
    
    wheel_tracker = WheelTracker()

    while True:
        try:
            message = bus.recv(timeout=1.0)
            if message is None:
                continue

            if message.arbitration_id == TARGET_FRAME_ID:
                # Assuming the encoder value is in the first byte of the data payload.
                raw_value = message.data[3]
                angle = wheel_tracker.translate_encoder_to_angle(raw_value)
                # print(f"Raw: 0x{raw_value:02X} -> Angle: {angle:.2f} degrees")
        except KeyboardInterrupt:
            print("Exiting...")
            break
        except Exception as e:
            print("Error reading CAN message:", e)
            time.sleep(0.1)

if __name__ == "__main__":
    main()
