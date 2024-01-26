import serial
import time

# Set your serial port name here (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux)
SERIAL_PORT = 'COM10'
BAUD_RATE = 115200  # Set the baud rate
cmd_delay=0.25


ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

try:
    while True:
        for temp in range (55*4,65*4,1):
            temp_string=str(temp/4)
            print(temp_string)
            ser.write(temp_string.encode("ascii"))
            time.sleep(cmd_delay)
except KeyboardInterrupt:
    print("Program terminated by user.")
finally:
    # Close the serial connection
    ser.close()
