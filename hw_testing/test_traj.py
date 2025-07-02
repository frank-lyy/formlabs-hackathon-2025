"""
Test program to teleop the joints of the robot using the keyboard.
"""

import serial 
import time
import numpy as np
import os

# SERIAL_PORT0 = "/dev/ttyACM1"

SERIAL_PORT0 = "COM11"
# SERIAL_PORT0 = "COM7"


class Commander:
    DATA_LEN = 9  # Length of the command exclusing the first 0xFF and action index

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
                # 0x00: Successful Arm Movement
                # 0x01: Successful Endo Wrist Movement
                # 0x02: Successful Wrist Movement
                print(f"Arduino responded with code {hex(resp[1])}")
            else:
                print(f"No response received (timeout)")

    def move_arm(self, ser_idx, angleA, angleB, debug=True):
        signA = list(int(angleA < 0).to_bytes(1, "big"))
        signB = list(int(angleB < 0).to_bytes(1, "big"))
        angleA = list(abs(angleA).to_bytes(2, "big"))  # 0.01 degrees
        angleB = list(abs(angleB).to_bytes(2, "big"))  # 0.01 degrees
        self.send_command(ser_idx, 0x00, signA + angleA + signB + angleB, debug)

    def move_endo_wrist(self, ser_idx, l_r, angleA, angleB, theta, phi, debug=True):
        if "l" in l_r:
            wrist_idx = 0
        else:
            wrist_idx = 1
            
        angleA = max(-180, min(180, angleA)) + 180  # normalize btwn 0 and 360
        angleB = max(-180, min(180, angleB)) + 180
        theta = max(-180, min(180, theta)) + 180
        phi = max(-180, min(180, phi)) + 180

        wrist_idx = list(wrist_idx.to_bytes(1, "big"))
        angleA = list(angleA.to_bytes(2, "big"))  # 0.01 degrees
        angleB = list(angleB.to_bytes(2, "big"))  # 0.01 degrees
        theta = list(theta.to_bytes(2, "big"))  # 0.01 degrees
        phi = list(phi.to_bytes(2, "big"))  # 0.01 degrees
        self.send_command(ser_idx, 0x01, wrist_idx + angleA + angleB + theta + phi, debug)

    def move_wrist(self, ser_idx, l_r, angle, debug=True):
        if "l" in l_r:
            servo_idx = 4
        else:
            servo_idx = 9
        servo_idx = list(servo_idx.to_bytes(1, "big"))
        sign = list(int(angle < 0).to_bytes(1, "big"))
        angle = list(abs(angle).to_bytes(2, "big"))  # 0.01 degrees
        self.send_command(ser_idx, 0x02, servo_idx + sign + angle, debug)

        
def rad_to_01_deg(rad):
    """Convert radians (float) to 0.01 degrees (integer)"""
    return int(rad * 18000 / np.pi)

def main():
    ser0 = serial.Serial(port=SERIAL_PORT0, baudrate=115200, timeout=5) 
    # ser1 = serial.Serial(port=SERIAL_PORT1, baudrate=115200, timeout=5)
    ser1 = None
    commander = Commander(ser0, ser1)
    time.sleep(1)

    try:
        target = 0  # 0 or 1 -- which arduino to send the command to
        wrist_idx = "r"  # 'l' or 'r' -- left or right wrist
        
        # Grab first file with .npy extension
        data = np.load([f for f in os.listdir() if f.endswith('.npz')][0], allow_pickle=True)
        traj = data["trajectory_data"]
        dt = data["dt"]
        
        t_start = time.time()
        while True:
            t_now = time.time()
            step_num = int((t_now - t_start) / dt)
            if step_num >= len(traj):
                break
            t = traj[step_num][0]
            q1, q2, q3, q4, q5, q6, q7 = traj[step_num][1:8]  # just get left for now
            q1, q2, q3, q4, q5, q6, q7 = rad_to_01_deg(q1.item()), rad_to_01_deg(q2.item()), rad_to_01_deg(q3.item()), rad_to_01_deg(q4.item()), rad_to_01_deg(q5.item()), rad_to_01_deg(q6.item()), rad_to_01_deg(q7.item())  # Convert to scalars
            print(f"Outputting step {step_num} at time {t} (deg):")
            print(f"\tJoint A: {q1/100}")
            print(f"\tJoint B: {q2/100}")
            print(f"\tJoint Wrist: {q3/100}")
            print(f"\tJoint Endo Wrist Phi: {q4/100}")
            print(f"\tJoint Endo Wrist Theta: {q5/100}")
            print(f"\tJoint Endo Wrist A: {q6/100}")
            print(f"\tJoint Endo Wrist B: {q7/100}")
            commander.move_arm(target, q1, q2)
            commander.move_endo_wrist(target, wrist_idx, q4, q5, q6, q7)
            commander.move_wrist(target, wrist_idx, q3)

    except KeyboardInterrupt:
        ser0.close()

if __name__ == "__main__":
    main()
