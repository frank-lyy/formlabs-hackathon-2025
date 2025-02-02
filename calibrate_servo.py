import curses
import threading
from commander import *

def move_wrist_in_thread(commander, ser_idx, servo_idx, angle):
    commander.move_wrist(ser_idx, servo_idx, angle, debug=False)

def main(stdscr):
    ser0 = serial.Serial(port="/dev/ttyACM0", baudrate=9600, timeout=5) 
    commander = Commander(ser0, None)

    curses.curs_set(0)
    stdscr.nodelay(True)

    move_thread = None
    angles = [0] * 10
    prev_angles = [0] * 10
    keys = "qwertyuiop"

    try:
        while True:
            keycode = stdscr.getch()
            if keycode != -1:
                key = chr(keycode)
                for i, char in enumerate(keys):
                    if key == char:
                        angles[i] += 1

            for i, angle in enumerate(angles):
                angles[i] = max(0, min(angle, 180))


            for i, angle in enumerate(angles):
                ignore_cmd = move_thread and move_thread.is_alive()
                if (prev_angles[i] != angle) and not ignore_cmd:
                    prev_angles[i] = angle
                    move_thread = threading.Thread(target=move_wrist_in_thread, args=(commander, 0, i, angle))
                    move_thread.start()

            text = ""
            for angle in angles:
                text += f"{angle} "
            stdscr.addstr(0, 0, text + "               ")
            stdscr.addstr(1, 0, f"ignore_cmd: {ignore_cmd}     ")
            stdscr.refresh()

    except KeyboardInterrupt:
        ser0.close()

if __name__ == "__main__":
    curses.wrapper(main)
