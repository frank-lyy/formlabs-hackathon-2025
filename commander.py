import serial 
import time

DATA_LEN = 9 # Length of the command exclusing the first 0xFF

class Commander:
    def __init__(self, ser0, ser1):
        self.ser = [ser0, ser1]

    def send_command(self, ser_idx, act_idx, data):
        while len(data) < DATA_LEN:
            data.append(0x00)
        to_send = bytes([0xFF, act_idx] + data)
        print("Sending:", list(to_send))
        self.ser[ser_idx].write(to_send)

        resp = self.ser[ser_idx].read(2)

        if len(resp) == 2 and resp[0] == 0xFF:
            print(f"Arduino responded with code {resp[1]}")
        else:
            print(f"No response received (timeout)")

    def move_arm(self, ser_idx, angleA, angleB):
        angleA = list(angleA.to_bytes(1, "big"))
        angleB = list(angleB.to_bytes(1, "big"))
        self.send_command(ser_idx, 0x00, angleA + angleB)

    def move_endo_wrist(self, ser_idx, servo_idx, potA, potB, potC, potD):
        servo_idx = list(servo_idx.to_bytes(1, "big"))
        potA = list(potA.to_bytes(2, "big"))
        potB = list(potB.to_bytes(2, "big"))
        potC = list(potC.to_bytes(2, "big"))
        potD = list(potD.to_bytes(2, "big"))
        self.send_command(ser_idx, 0x01, servo_idx + potA + potB + potC + potD)

    def move_wrist(self, ser_idx, servo_idx, angle):
        servo_idx = list(servo_idx.to_bytes(1, "big"))
        angle = list(angle.to_bytes(1, "big"))
        self.send_command(ser_idx, 0x02, servo_idx + angle)

def main():
    ser0 = serial.Serial(port="/dev/ttyACM1", baudrate=9600, timeout=5) 
    commander = Commander(ser0, None)
    time.sleep(1)

#    commander.move_endo_wrist(0, 0, 200, 300, 300, 100)
#    commander.move_endo_wrist(0, 1, 200, 500, 300, 1000)
#    commander.move_wrist(0, 0, 0)
#    commander.move_wrist(0, 1, 90)
    commander.move_arm(0, 180, 180)
    commander.move_arm(0, 0, 90)

if __name__ == "__main__":
    main()
