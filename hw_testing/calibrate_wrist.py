import curses
import threading
from commander import *

WRIST_IDX = 0

def move_endo_wrist_in_thread(commander, ser_idx, angleA, angleB, theta, phi):
    commander.move_endo_wrist(ser_idx, WRIST_IDX, angleA, angleB, theta, phi, debug=False)

def main(stdscr):
    ser0 = serial.Serial(port="/dev/ttyACM0", baudrate=9600, timeout=5) 
    commander = Commander(ser0, None)

    curses.curs_set(0)
    stdscr.nodelay(True)

    move_thread = None
    angles = [0] * 4
    prev_angles = [-1] * 4
    up_keys = "qwer"
    down_keys = "asdf"

    try:
        while True:
            keycode = stdscr.getch()
            if keycode != -1:
                key = chr(keycode)
                for i, char in enumerate(up_keys):
                    if key == char:
                        angles[i] += 1
                for i, char in enumerate(down_keys):
                    if key == char:
                        angles[i] -= 1

            for i, angle in enumerate(angles):
                angles[i] = max(-180, min(angle, 180))

            ignore_cmd = move_thread and move_thread.is_alive()
            if (prev_angles != angles) and not ignore_cmd:
                prev_angles = angles.copy()
                move_thread = threading.Thread(target=move_endo_wrist_in_thread, args=(commander, 0, *angles))
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
