import serial 
import time 

def main():
    ser0 = serial.Serial(port="/dev/ttyACM1", baudrate=9600, timeout=1) 
    time.sleep(2)

    try:
        while True: 
            potA = list(int(input("potA: ")).to_bytes(2, "big"))
            potB = list(int(input("potB: ")).to_bytes(2, "big"))
            potC = list(int(input("potC: ")).to_bytes(2, "big"))
            potD = list(int(input("potD: ")).to_bytes(2, "big"))
            print()
            data = [0xFF] + potA + potB + potC + potD # length 9
            ser0.write(bytes(data)) 
    except KeyboardInterrupt:
        ser0.close()

if __name__ == "__main__":
    main()
