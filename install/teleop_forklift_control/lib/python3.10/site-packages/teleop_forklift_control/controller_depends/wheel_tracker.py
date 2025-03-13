#!/usr/bin/env python3
import can
import time

# --- Configuration ---
CAN_CHANNEL = 'can0'
CAN_BITRATE = 250000
TARGET_FRAME_ID = 0x199

def translate_encoder_to_angle(raw_value):
    """
    Translate an 8-bit encoder reading (0-255) into an angle (in degrees)
    relative to the straight forward position (0xFF).
    
    This function calculates the signed difference between the raw value and 0xFF
    using modular arithmetic to handle wrap-around. The resulting difference is
    then scaled by (360/256) to convert encoder ticks to degrees.
    
    Note: This approach assumes that the physical movement does not exceed ±180°
    from the reference point.
    """
    # Compute the signed difference between raw_value and 0xFF.
    # This expression maps the difference into the range -128 to +127.
    diff = ((raw_value - 0xFF + 128) % 256) - 128
    # Convert ticks to degrees (each tick is 360/256 degrees)
    angle = diff * (360.0 / 256)
    return angle

def main():
    try:
        bus = can.interface.Bus(bustype='socketcan', channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
    except Exception as e:
        print("Error initializing CAN bus:", e)
        return

    print(f"Listening for CAN frame 0x{TARGET_FRAME_ID:X} on channel {CAN_CHANNEL}...")

    while True:
        try:
            message = bus.recv(timeout=1.0)
            if message is None:
                continue

            if message.arbitration_id == TARGET_FRAME_ID:
                # Assuming the encoder value is in the first byte of the data payload.
                raw_value = message.data[3]
                angle = translate_encoder_to_angle(raw_value)
                print(f"Raw: 0x{raw_value:02X} -> Angle: {angle:.2f} degrees")
        except KeyboardInterrupt:
            print("Exiting...")
            break
        except Exception as e:
            print("Error reading CAN message:", e)
            time.sleep(0.1)

if __name__ == "__main__":
    main()
