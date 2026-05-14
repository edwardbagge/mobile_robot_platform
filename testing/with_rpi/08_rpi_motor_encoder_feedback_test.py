# 08_rpi_motor_encoder_feedback_test.py
# Raspberry Pi 5 serial interface for ESP32 motor encoder feedback test
#
# Purpose:
# Send motor commands to ESP32 over USB serial and print encoder feedback.
#
# Commands:
# f = forward
# b = backward
# l = turn left
# r = turn right
# x = stop
# s = status
# z = reset encoder counts
# q = quit Python program

import serial
import time
import threading

SERIAL_PORT = "/dev/ttyUSB0"   # Change to /dev/ttyACM0 if needed
BAUD_RATE = 115200


def read_from_esp32(serial_connection, running_flag):
    while running_flag["running"]:
        try:
            if serial_connection.in_waiting > 0:
                line = serial_connection.readline().decode(errors="ignore").strip()
                if line:
                    print(line)
        except serial.SerialException:
            print("Serial read error.")
            running_flag["running"] = False
            break


def main():
    print("=== RPI MOTOR ENCODER FEEDBACK TEST ===")
    print("Opening serial port:", SERIAL_PORT)

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    except serial.SerialException:
        print("Could not open serial port.")
        print("Check the port using:")
        print("ls /dev/ttyUSB*")
        print("ls /dev/ttyACM*")
        return

    time.sleep(2)

    running_flag = {"running": True}

    reader_thread = threading.Thread(
        target=read_from_esp32,
        args=(ser, running_flag),
        daemon=True
    )
    reader_thread.start()

    print()
    print("Commands:")
    print("f = forward")
    print("b = backward")
    print("l = turn left")
    print("r = turn right")
    print("x = stop")
    print("s = status")
    print("z = reset encoder counts")
    print("q = quit")
    print()

    try:
        while True:
            command = input("Send command: ").strip()

            if command == "":
                continue

            if command == "q":
                print("Stopping motors before quitting.")
                ser.write(("x\n").encode())
                time.sleep(0.2)
                break

            if command not in ["f", "b", "l", "r", "x", "s", "z"]:
                print("Unknown local command.")
                continue

            ser.write((command + "\n").encode())

    except KeyboardInterrupt:
        print()
        print("Keyboard interrupt. Stopping motors.")

        try:
            ser.write(("x\n").encode())
            time.sleep(0.2)
        except serial.SerialException:
            pass

    running_flag["running"] = False
    time.sleep(0.2)
    ser.close()

    print("Serial port closed.")


if __name__ == "__main__":
    main()