import curses
import threading
from commander import *

def move_arm_in_thread(commander, ser_idx, angleA, angleB):
    commander.move_arm(ser_idx, angleA, angleB, debug=False)

def main(stdscr):
    ser0 = serial.Serial(port="/dev/ttyACM0", baudrate=9600, timeout=5) 
    commander = Commander(ser0, None)

    curses.curs_set(0)
    stdscr.nodelay(True)

    angle0A = 0
    angle0B = 0
    angle1A = 0
    angle1B = 0
    prev_angle0A = 0
    prev_angle0B = 0
    prev_angle1A = 0
    prev_angle1B = 0
    move_thread_0 = None
    move_thread_1 = None

    try:
        while True:
            keycode = stdscr.getch()
            if keycode != -1:
                key = chr(keycode)
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
            angle0B = max(-180, min(angle0B, 180))
            angle1A = max(-180, min(angle1A, 180))
            angle1B = max(-180, min(angle1B, 180))

            ignore_cmd_0 = move_thread_0 and move_thread_0.is_alive()
            ignore_cmd_1 = move_thread_1 and move_thread_1.is_alive()

            if (prev_angle0A != angle0A or prev_angle0B != angle0B) and not ignore_cmd_0:
                prev_angle0A, prev_angle0B = angle0A, angle0B
                move_thread_0 = threading.Thread(target=move_arm_in_thread, args=(commander, 0, angle0A, angle0B))
                move_thread_0.start()

#            if (prev_angle1A != angle1A or prev_angle1B != angle1B) not ignore_cmd_1:
#                prev_angle1A, prev_angle1B = angle1A, angle1B
#                move_thread_1 = threading.Thread(target=move_arm_in_thread, args=(commander, 1, angle1A, angle1B))
#                move_thread_1.start()

            stdscr.addstr(0, 0, f"angle0A: {angle0A}, angle0B: {angle0B}, angle1A: {angle1A}, angle1B: {angle1B}     ")
            stdscr.addstr(1, 0, f"ignore_cmd_0: {ignore_cmd_0}     ")
            stdscr.addstr(2, 0, f"ignore_cmd_1: {ignore_cmd_1}     ")
            stdscr.refresh()

    except KeyboardInterrupt:
        ser0.close()

if __name__ == "__main__":
    curses.wrapper(main)
