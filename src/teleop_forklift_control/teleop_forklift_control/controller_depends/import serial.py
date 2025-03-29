import serial
import time

def main():
    # Configure the serial connection
    baudrate = 230400
    port = '/dev/ttyUSB1'  # Adjust the port as needed
    ser = serial.Serial(port, baudrate, timeout=1)

    try:
        # Write "0.20" to the serial port with no line ending
        my_float = 0.30
        # Convert the float to a string and encode it into bytes.
        data = str(my_float).encode('utf-8')  # results in b'3.14159'

        # Write the data without appending any newline characters.
       
        while True:
            ser.write(data)
        print("Sent '0.20' to the serial port")
    except Exception as e:
        print(f"Error writing to serial port: {e}")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
