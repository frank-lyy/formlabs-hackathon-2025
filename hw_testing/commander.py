"""
Test program to teleop the joints of the robot using the keyboard.
"""

import serial 
import time

# SERIAL_PORT0 = "/dev/ttyACM1"

SERIAL_PORT0 = "COM11"
# SERIAL_PORT0 = "COM7"


class Commander:
    DATA_LEN = 9  # Length of the command exclusing the first 0xFF and action index

    def __init__(self, ser0, ser1):
        self.ser = [ser0, ser1]

    def send_command(self, ser_idx, act_idx, data, debug=True):
        while len(data) < self.DATA_LEN:
            data.append(0x00)
        to_send = bytes([0xFF, act_idx] + data)
        if debug:
            print("Sending:", list(to_send))
        self.ser[ser_idx].write(to_send)

        resp = self.ser[ser_idx].read(2)

        if debug:
            if len(resp) == 2 and resp[0] == 0xFF:
                # 0x00: Successful Arm Movement
                # 0x01: Successful Endo Wrist Movement
                # 0x02: Successful Wrist Movement
                print(f"Arduino responded with code {hex(resp[1])}")
            else:
                print(f"No response received (timeout)")

    def move_arm(self, ser_idx, angleA, angleB, debug=True):
        signA = list(int(angleA < 0).to_bytes(1, "big"))
        signB = list(int(angleB < 0).to_bytes(1, "big"))
        angleA = list(abs(angleA).to_bytes(2, "big"))  # 0.01 degrees
        angleB = list(abs(angleB).to_bytes(2, "big"))  # 0.01 degrees
        self.send_command(ser_idx, 0x00, signA + angleA + signB + angleB, debug)

    def move_endo_wrist(self, ser_idx, l_r, angleA, angleB, theta, phi, debug=True):
        if "l" in l_r:
            wrist_idx = 0
        else:
            wrist_idx = 1
            
        angleA = max(-180, min(180, angleA)) + 180
        angleB = max(-180, min(180, angleB)) + 180
        theta = max(-180, min(180, theta)) + 180
        phi = max(-180, min(180, phi)) + 180

        wrist_idx = list(wrist_idx.to_bytes(1, "big"))
        angleA = list(angleA.to_bytes(2, "big"))  # 0.01 degrees
        angleB = list(angleB.to_bytes(2, "big"))  # 0.01 degrees
        theta = list(theta.to_bytes(2, "big"))  # 0.01 degrees
        phi = list(phi.to_bytes(2, "big"))  # 0.01 degrees
        self.send_command(ser_idx, 0x01, wrist_idx + angleA + angleB + theta + phi, debug)

    def move_wrist(self, ser_idx, l_r, angle, debug=True):
        if "l" in l_r:
            servo_idx = 4
        else:
            servo_idx = 9
        servo_idx = list(servo_idx.to_bytes(1, "big"))
        sign = list(int(angle < 0).to_bytes(1, "big"))
        angle = list(abs(angle).to_bytes(2, "big"))  # 0.01 degrees
        self.send_command(ser_idx, 0x02, servo_idx + sign + angle, debug)

def main():
    ser0 = serial.Serial(port=SERIAL_PORT0, baudrate=115200, timeout=5) 
    # ser1 = serial.Serial(port=SERIAL_PORT1, baudrate=115200, timeout=5)
    ser1 = None
    commander = Commander(ser0, ser1)
    time.sleep(1)

    try:
        while True:
            print("Enter command:")
            print("Options:")
            print("[0]: move_arm")
            print("[1]: move_endo_wrist")
            print("[2]: move_wrist")
            command = int(input())
            command_valid = command < 3

            target = 0
            if command_valid:
                # Ask which Arduino to send the command to
                # 0 or 1
                target = int(input("Target Arduino: "))

            if command == 0:
                # Move arm
                angleA = int(input("Angle A: "))*100  # convert to units of 0.01 degrees
                angleB = int(input("Angle B: "))*100  # convert to units of 0.01 degrees
                commander.move_arm(target, angleA, angleB)

            elif command == 1:
                # Move endo_wrist
                
                # Left or right endowrist
                # 0 or 1
                l_r = str(input("Left or Right Wrist ('l' or 'r'): "))
                angleA = int(input("Angle A: "))  # in degrees
                angleB = int(input("Angle B: "))  # in degrees
                angleC = int(input("Angle C: "))  # in degrees
                angleD = int(input("Angle D: "))  # in degrees
                commander.move_endo_wrist(target, l_r, angleA, angleB, angleC, angleD)

            elif command == 2:
                # Move wrist
                
                # Left or right wrist
                # 0 or 1
                l_r = str(input("Left or Right Wrist ('l' or 'r'): "))
                angle = int(input("Angle: "))*100  # convert to units of 0.01 degrees
                commander.move_wrist(target, l_r, angle)

            else:
                print("Invalid command!")

    except KeyboardInterrupt:
        ser0.close()

if __name__ == "__main__":
    main()
