import curses
from commander import *

def main(stdscr):
    ser0 = serial.Serial(port="/dev/ttyACM0", baudrate=9600, timeout=5) 
    commander = Commander(ser0, None)

    curses.curs_set(0)
    stdscr.nodelay(True)
    time.sleep(1)

    angle0A = 0
    angle0B = 0
    angle1A = 0
    angle1B = 0
    try:
        while True:
            key = chr(stdscr.getch())
            if key == "q":
                angle0A += 1
            elif key == "w":
                angle0A -= 1
            elif key == "e":
                angle0B += 1
            elif key == "r":
                angle0B -= 1
            elif key == "a":
                angle1A += 1
            elif key == "s":
                angle1A -= 1
            elif key == "d":
                angle1B += 1
            elif key == "f":
                angle1B -= 1
            angle0A = max(-180, min(angle0A, 180))
            angle0B = max(-180, min(angle0A, 180))
            angle1A = max(-180, min(angle0A, 180))
            angle1B = max(-180, min(angle0A, 180))
            commander.move_arm(0, angle0A, angle0B)
            commander.move_arm(1, angle1A, angle1B)
    except KeyboardInterrupt:
        ser0.close()

if __name__ == "__main__":
    curses.wrapper(main)
