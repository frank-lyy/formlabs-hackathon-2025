import os
import sys
sys.path.append('motion_planning/')

import threading
import queue
import time
from string_segmentor import main as segmentor_main
from motion_planning.main_v2 import main as robot_main

def string_segmentation_thread(stop_event):
    """Thread for running string segmentation"""
    try:
        # Run string segmentation and update shared state
        segmentor_main(stop_event)
    except Exception as e:
        print(f"Error in segmentation thread: {e}")
        stop_event.set()

def robot_control_thread(stop_event):
    """Thread for running robot state machine"""
    try:
        # Initialize robot control
        robot_main(stop_event)
    except Exception as e:
        print(f"Error in robot control thread: {e}")
        stop_event.set()

def main():
    # Create shared state and stop event
    shared_state = StringState()
    stop_event = threading.Event()

    # Create threads
    segmentation_thread = threading.Thread(
        target=string_segmentation_thread,
        args=(stop_event)
    )
    robot_thread = threading.Thread(
        target=robot_control_thread,
        args=(stop_event)
    )

    try:
        # Start threads
        segmentation_thread.start()
        robot_thread.start()

        # Wait for threads to complete
        while True:
            if not (segmentation_thread.is_alive() and robot_thread.is_alive()):
                break
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nShutting down...")
        stop_event.set()

    finally:
        # Clean up
        stop_event.set()
        segmentation_thread.join(timeout=1)
        robot_thread.join(timeout=1)

if __name__ == "__main__":
    main()
