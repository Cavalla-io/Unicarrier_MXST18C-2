#!/usr/bin/env python3
import can
import time
import logging
import struct

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
logger = logging.getLogger(__name__)

# --- Configuration ---
CAN_CHANNEL = 'can0'
CAN_BITRATE = 250000
TARGET_FRAME_ID = 0x199

class WheelTracker:
    def __init__(self):
        # Minimal tracking for debugging
        self.prev_raw_value = None
        self.prev_angle = None

    def translate_encoder_to_angle(self, raw_byte):
        """
        Translate a raw byte value (0-255) into an angle (in degrees)
        within the 0-360 range.
        
        Mapping:
        - 0 to 70 (0x00-0x46) maps to 0-180 degrees
        - 186 to 255 (0xBA-0xFF) maps to 180-360 degrees
        - Values between 71-185 are clamped to either 70 or 186
        """
        if raw_byte <= 70:
            # Map 0-70 to 0-180 degrees
            angle = (raw_byte * 180.0) / 70.0
        elif raw_byte >= 186:  # 186 is equivalent to -70 in two's complement
            # Map 186-255 to 180-360 degrees
            # 186 → 180°, 255 → 359.5°
            angle = 180.0 + ((raw_byte - 186) * 180.0 / 69.0)
        else:
            # Values in the gap (71-185)
            if raw_byte <= 127:
                # Closer to 70, clamp to 70
                angle = 180.0
            else:
                # Closer to 186, clamp to 186
                angle = 180.0
        
        # Store values for tracking
        self.prev_raw_value = raw_byte
        self.prev_angle = angle
        
        return angle

# Create a standalone function that wheel_tracker_node.py is trying to import
# This maintains a global instance of WheelTracker to persist state between calls
_global_wheel_tracker = WheelTracker()
def translate_encoder_to_angle(raw_byte):
    """
    Standalone function that delegates to the WheelTracker class method.
    This allows importing this function directly from the module.
    
    Input should be a raw byte value (0-255).
    """
    return _global_wheel_tracker.translate_encoder_to_angle(raw_byte)

def main():
    try:
        bus = can.interface.Bus(bustype='socketcan', channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
    except Exception as e:
        print("Error initializing CAN bus:", e)
        return

    print(f"Listening for CAN frame 0x{TARGET_FRAME_ID:X} on channel {CAN_CHANNEL}...")
    
    wheel_tracker = WheelTracker()
    prev_raw = None

    while True:
        try:
            message = bus.recv(timeout=1.0)
            if message is None:
                continue

            if message.arbitration_id == TARGET_FRAME_ID:
                # Get the raw byte value directly - no conversion to signed
                raw_byte = message.data[3]
                
                # Calculate angle
                angle = wheel_tracker.translate_encoder_to_angle(raw_byte)
                
                if prev_raw != raw_byte:
                    print(f"CAN data: {[hex(x)[2:].zfill(2) for x in message.data]}, Raw: 0x{raw_byte:02X} ({raw_byte}) -> Angle: {angle:.2f}°")
                    prev_raw = raw_byte
        except KeyboardInterrupt:
            print("Exiting...")
            break
        except Exception as e:
            print("Error reading CAN message:", e)
            time.sleep(0.1)

if __name__ == "__main__":
    main()
