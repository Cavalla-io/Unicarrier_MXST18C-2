#!/usr/bin/env python3
import serial
import time

class DriveController:
    def __init__(self, listener,
                 steering_port='/dev/ttyUSB0', throttle_port='/dev/ttyUSB1',
                 baudrate=115200):
        # Save the shared input listener.
        self.listener = listener

        # Open Serial Connections for steering and throttle controllers.
        try:
            self.steering_ser = serial.Serial(steering_port, baudrate, timeout=1)
            self.throttle_ser = serial.Serial(throttle_port, baudrate, timeout=1)
        except Exception as e:
            print(f"Error opening serial ports: {e}")
            raise

        # Allow the boards (e.g., ESP32) time to initialize.
        time.sleep(2)

        # Initialize the state variable for steering.
        self.last_steering = None  # Can be 'L', 'R', or None
        
        # Track the last time we sent a throttle command
        self.last_throttle_time = 0

    def update(self):
        # Get the drive command from the shared listener.
        # The listener returns a dictionary with keys "throttle" (boolean) and "steering" (either 'L', 'R', or None)
        drive_cmd = self.listener.get_drive_command()

        # Handle throttle
        current_time = time.time()
        if drive_cmd.get("throttle", False):
            # Always send command immediately if it's been more than 100ms OR if this is a new button press
            if current_time - self.last_throttle_time >= 0.1 or self.last_throttle_time == 0:
                self.throttle_ser.write(b'p')  # Engage throttle (move DACs to 2.5V)
                self.last_throttle_time = current_time
        else:
            self.throttle_ser.write(b's')  # Disengage throttle (return to normal voltage)
            # Reset the timer when button is released
            self.last_throttle_time = 0

        # Determine current steering command from the listener.
        current_steering = drive_cmd.get("steering", None)

        # Process steering command:
        if current_steering is not None:
            if self.last_steering is None:
                # First key press—just engage the steering voltage update.
                self.steering_ser.write(b'p')
                self.last_steering = current_steering
            elif self.last_steering != current_steering:
                # Steering direction changed: send a reversal command then engage voltage update.
                self.steering_ser.write(b'r')
                self.steering_ser.write(b'p')
                self.last_steering = current_steering
            else:
                # Same key is being held: simply engage voltage update.
                self.steering_ser.write(b'p')
        else:
            # No steering key is pressed: hold the current voltage and reset last steering.
            self.steering_ser.write(b's')
    def close(self):
        self.steering_ser.close()
        self.throttle_ser.close()
