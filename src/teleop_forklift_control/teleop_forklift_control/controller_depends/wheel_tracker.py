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

    def translate_encoder_to_angle(self, signed_value):
        """
        Translate a signed int8 encoder reading (-128 to 127) into an angle (in degrees)
        within the 0-360 range.
        
        Mapping:
        - 0 to 70 (hex 0x00-0x46) maps to 0-180 degrees
        - -70 to -1 (hex 0xBA-0xFF) maps to 180-360 degrees
        - Values outside these ranges are clamped
        """
        if signed_value >= 0:
            if signed_value > 70:
                # Clamp positive values to max 70
                signed_value = 70
            # Map 0-70 to 0-180 degrees
            angle = (signed_value * 180.0) / 70.0
        else:  # negative values
            if signed_value < -70:
                # Clamp negative values to min -70
                signed_value = -70
            # Map -70 to -1 to 180-360 degrees
            # -70 → 180°, -1 → 359°
            angle = 180.0 + ((70.0 + signed_value) * 180.0 / 70.0)
        
        # Debug logging for unexpected jumps
        if self.prev_raw_value is not None and self.prev_angle is not None:
            angle_diff = abs(angle - self.prev_angle)
            # Account for wrap-around
            if angle_diff > 180:
                angle_diff = 360 - angle_diff
                
            if angle_diff > 20 and (
                (signed_value >= 0 and self.prev_raw_value >= 0 and abs(signed_value - self.prev_raw_value) < 10) or
                (signed_value < 0 and self.prev_raw_value < 0 and abs(signed_value - self.prev_raw_value) < 10)
            ):
                logger.info(f"UNEXPECTED JUMP: Signed {self.prev_raw_value}->{signed_value}, Angle {self.prev_angle:.2f}°->{angle:.2f}°")
        
        # Store values for next comparison
        self.prev_raw_value = signed_value
        self.prev_angle = angle
        
        return angle

# Create a standalone function that wheel_tracker_node.py is trying to import
# This maintains a global instance of WheelTracker to persist state between calls
_global_wheel_tracker = WheelTracker()
def translate_encoder_to_angle(signed_value):
    """
    Standalone function that delegates to the WheelTracker class method.
    This allows importing this function directly from the module.
    
    Input should be a signed int8 value (-128 to 127), not a raw byte (0-255).
    """
    return _global_wheel_tracker.translate_encoder_to_angle(signed_value)

def main():
    try:
        bus = can.interface.Bus(bustype='socketcan', channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
    except Exception as e:
        print("Error initializing CAN bus:", e)
        return

    print(f"Listening for CAN frame 0x{TARGET_FRAME_ID:X} on channel {CAN_CHANNEL}...")
    
    wheel_tracker = WheelTracker()
    prev_signed = None

    while True:
        try:
            message = bus.recv(timeout=1.0)
            if message is None:
                continue

            if message.arbitration_id == TARGET_FRAME_ID:
                # Get the raw value as an unsigned byte
                raw_byte = message.data[3]
                
                # Convert to signed int8 properly
                # First method: using struct to unpack as signed char
                signed_value = struct.unpack('b', bytes([raw_byte]))[0]
                
                # Alternative manual conversion (as backup)
                # if raw_byte > 127:
                #     signed_value = raw_byte - 256
                # else:
                #     signed_value = raw_byte
                
                angle = wheel_tracker.translate_encoder_to_angle(signed_value)
                
                if prev_signed != signed_value:
                    print(f"CAN data: {[hex(x)[2:].zfill(2) for x in message.data]}, Raw: 0x{raw_byte:02X}, Signed: {signed_value} -> Angle: {angle:.2f}°")
                    prev_signed = signed_value
        except KeyboardInterrupt:
            print("Exiting...")
            break
        except Exception as e:
            print("Error reading CAN message:", e)
            time.sleep(0.1)

if __name__ == "__main__":
    main()
