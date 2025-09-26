#!/usr/bin/env python3
import serial
import signal
import sys
import time

# --------------------------------------------------
# Serial Communication Setup
# --------------------------------------------------
# Example settings for Windows (commented out)
# arduino_port = 'COM4'  # Replace with your Arduino's port
# baud_rate = 9600
# arduino = serial.Serial(arduino_port, baud_rate)

# --------------------------------------------------
# Input Parameters
# --------------------------------------------------
f_name = input("FILE name = ")

w_y = input("w_y = ")
w_s = input("w_s = ")
w_k = input("w_k = ")
rand_par = input("rand_par = ")

# Open output file for writing incoming serial data
f = open(str(f_name) + ".txt", "w")

# --------------------------------------------------
# Prepare Message to Send
# --------------------------------------------------
message = "a" + str(w_y) + "b" + str(w_s) + "c" + str(w_k) + "d" + str(rand_par) + "z\n"
b_message = bytes(message, 'utf-8') 

# Stop message (sent when Ctrl+C is pressed)
mess = "S\n"
s_message = bytes(mess, 'utf-8')

# Connection state flag
connectionFlag = 0

# --------------------------------------------------
# Signal Handler for Graceful Exit
# --------------------------------------------------
def signal_handler(sig, frame):
    """Handle Ctrl+C by notifying Arduino, closing file, and exiting."""
    ser.write(s_message)
    print("Ctrl+C detected. Closing serial connection...")
    f.close()
    # arduino.close()  # Uncomment if used with a different port setup
    sys.exit(0)

# Register handler for SIGINT (Ctrl+C)
signal.signal(signal.SIGINT, signal_handler)

# --------------------------------------------------
# Main Loop: Connect and Read Data
# --------------------------------------------------
try:
    while True:
        if __name__ == '__main__':
            ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            ser.reset_input_buffer()

            # -------------------------
            # State 0: Establish Connection
            # -------------------------
            while(connectionFlag == 0):
                ser.write(b"connect\n")
                line = ser.readline().decode('utf-8').rstrip()
                print(line)
                if(line == "CONNECTED!"):
                    connectionFlag = 1
                time.sleep(0.01)

            # -------------------------
            # State 1: Send Parameters, Wait for ACK
            # -------------------------
            while(connectionFlag == 1):
                ser.write(b_message)
                line = ser.readline().decode('utf-8').rstrip()
                print(line)
                f.write(line + "\n")
                if(line == "GET!"):
                    connectionFlag = 2
                time.sleep(0.01)

            # -------------------------
            # State 2: Continuous Data Logging
            # -------------------------
            while(connectionFlag == 2):
                line = ser.readline().decode('utf-8').rstrip()
                print(line)
                f.write(line + "\n")

except KeyboardInterrupt:
    print("Ctrl+C detected. Exiting...")


# --------------------------------------------------
# Arduino Side (Example Skeleton)
# --------------------------------------------------
'''
void setup() {
  Serial.begin(9600);
  // Your setup code here
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'S') {
      // Received a stop command from Python
      // Perform any necessary cleanup
      // Exit loop or reset as needed
    }
  }
  // Your loop code here
}
'''

