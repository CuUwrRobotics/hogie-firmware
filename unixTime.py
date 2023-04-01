import serial
import time

# Open the serial port
ser = serial.Serial('COM1', 9600)

# Get the current Unix time
current_time = int(time.time())

# Send the current Unix time over the serial port
ser.write(str(current_time).encode())

# Close the serial port
ser.close()

