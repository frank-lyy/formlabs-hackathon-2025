import serial 
import time

class Commander:
    DATA_LEN = 9 # Length of the command exclusing the first 0xFF and action index

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
                print(f"Arduino responded with code {resp[1]}")
            else:
                print(f"No response received (timeout)")

    def move_arm(self, ser_idx, angleA, angleB, debug=True):
        signA = list(int(angleA < 0).to_bytes(1, "big"))
        signB = list(int(angleB < 0).to_bytes(1, "big"))
        angleA = list(abs(angleA).to_bytes(1, "big"))
        angleB = list(abs(angleB).to_bytes(1, "big"))
        self.send_command(ser_idx, 0x00, signA + angleA + signB + angleB, debug)

    def move_endo_wrist(self, ser_idx, wrist_idx, angleA, angleB, theta, phi, debug=True):
        angleA = max(-180, min(180, angleA)) + 180
        angleB = max(-180, min(180, angleB)) + 180
        theta = max(-180, min(180, theta)) + 180
        phi = max(-180, min(180, phi)) + 180

        wrist_idx = list(wrist_idx.to_bytes(1, "big"))
        angleA = list(angleA.to_bytes(2, "big"))
        angleB = list(angleB.to_bytes(2, "big"))
        theta = list(theta.to_bytes(2, "big"))
        phi = list(phi.to_bytes(2, "big"))
        self.send_command(ser_idx, 0x01, wrist_idx + angleA + angleB + theta + phi, debug)

    def move_wrist(self, ser_idx, servo_idx, angle, debug=True):
        servo_idx = list(servo_idx.to_bytes(1, "big"))
        angle = list(angle.to_bytes(1, "big"))
        self.send_command(ser_idx, 0x02, servo_idx + angle, debug)

def main():
    ser0 = serial.Serial(port="/dev/ttyACM1", baudrate=9600, timeout=5) 
    commander = Commander(ser0, None)
    time.sleep(1)

    try:
        while True:
            print("Enter command:")
            print("Options:")
            print("[0]: move_arm")
            print("[1]: move_endo_wrist")
            print("[2]: move_wrist")
            command = int(input())

            target = 0
            if command < 3:
                target = int(input("Target Arduino: "))

            if command == 0:
                angleA = int(input("Angle A: "))
                angleB = int(input("Angle B: "))
                commander.move_arm(target, angleA, angleB)

            elif command == 1:
                wrist_idx = int(input("Wrist index: "))
                angleA = int(input("Angle A: "))
                angleB = int(input("Angle B: "))
                angleC = int(input("Angle C: "))
                angleD = int(input("Angle D: "))
                commander.move_endo_wrist(target, wrist_idx, angleA, angleB, angleC, angleD)

            elif command == 2:
                servo_idx = int(input("Servo index: "))
                angle = int(input("Angle: "))
                commander.move_wrist(target, servo_idx, angle)

            else:
                print("Invalid command!")

    except KeyboardInterrupt:
        ser0.close()

if __name__ == "__main__":
    main()
