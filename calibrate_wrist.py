import curses
import threading
from commander import *

WRIST_IDX = 0

def move_endo_wrist_in_thread(commander, ser_idx, potA, potB, potC, potD):
    commander.move_endo_wrist(ser_idx, WRIST_IDX, potA, potB, potC, potD, debug=False)

def main(stdscr):
    ser0 = serial.Serial(port="/dev/ttyACM1", baudrate=9600, timeout=5) 
    commander = Commander(ser0, None)

    curses.curs_set(0)
    stdscr.nodelay(True)

    move_thread = None
    pots = [511] * 4
    prev_pots = [0] * 4
    up_keys = "qwer"
    down_keys = "asdf"

    try:
        while True:
            keycode = stdscr.getch()
            if keycode != -1:
                key = chr(keycode)
                for i, char in enumerate(up_keys):
                    if key == char:
                        pots[i] += 5
                for i, char in enumerate(down_keys):
                    if key == char:
                        pots[i] -= 5

            for i, pot in enumerate(pots):
                pots[i] = max(0, min(pot, 1023))

            ignore_cmd = move_thread and move_thread.is_alive()
            if (prev_pots != pots) and not ignore_cmd:
                prev_pots = pots.copy()
                move_thread = threading.Thread(target=move_endo_wrist_in_thread, args=(commander, 0, *pots))
                move_thread.start()

            text = ""
            for pot in pots:
                text += f"{pot} "
            stdscr.addstr(0, 0, text + "               ")
            stdscr.addstr(1, 0, f"ignore_cmd: {ignore_cmd}     ")
            stdscr.refresh()

    except KeyboardInterrupt:
        ser0.close()

if __name__ == "__main__":
    curses.wrapper(main)
