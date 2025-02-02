import serial 
import time

DATA_LEN = 8 # Length of the command exclusing the first 0xFF

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

    def move_wrist(self, ser_idx, potA, potB, potC, potD):
        potA = list(potA.to_bytes(2, "big"))
        potB = list(potB.to_bytes(2, "big"))
        potC = list(potC.to_bytes(2, "big"))
        potD = list(potD.to_bytes(2, "big"))
        self.send_command(ser_idx, 0x01, potA + potB + potC + potD)

def main():
    ser0 = serial.Serial(port="/dev/ttyACM0", baudrate=9600, timeout=5) 
    commander = Commander(ser0, None)
    time.sleep(1)

    commander.move_wrist(0, 500, 500, 500, 100)
    commander.move_arm(0, 180, 180)

if __name__ == "__main__":
    main()
