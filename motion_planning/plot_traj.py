import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

traj_data = {
    'Joint A': {'position': [], 'velocity': [], 'acceleration': []},
    'Joint B': {'position': [], 'velocity': [], 'acceleration': []},
    # 'Joint Wrist': {'position': [], 'velocity': [], 'acceleration': []},
    # 'Joint Endo Wrist Phi': {'position': [], 'velocity': [], 'acceleration': []},
    # 'Joint Endo Wrist Theta': {'position': [], 'velocity': [], 'acceleration': []},
    # 'Joint Endo Wrist A': {'position': [], 'velocity': [], 'acceleration': []},
    # 'Joint Endo Wrist B': {'position': [], 'velocity': [], 'acceleration': []}
}


def plot_motion_profile(times, traj_data):
    """
    Generate a comprehensive motion profile plot for the trajectory.
    
    Args:
        times: List of time points
        traj_data: Dictionary containing trajectory data for each joint
    """
    joint_names = [key for key in traj_data.keys()]
    
    # Create a figure with 3 rows (position, velocity, acceleration) and 1 column
    fig = plt.figure(figsize=(15, 10))
    gs = GridSpec(3, 1, height_ratios=[1, 1, 1], figure=fig)
    
    # Position subplot
    ax1 = fig.add_subplot(gs[0])
    for i, joint in enumerate(joint_names):
        if i < 2:  # First two joints (Joint A and Joint B)
            ax1.plot(times, traj_data[joint]['position'], label=joint)  # Already in units of steps
        else:  # All other joints (indices 2 and above)
            ax1.plot(times, [p/100 for p in traj_data[joint]['position']], label=joint)  # Divide by 100 to convert from 0.01 deg to degrees
    ax1.set_title('Joint Positions Over Time')
    ax1.set_ylabel('Position (steps or degrees)')
    ax1.grid(True)
    ax1.legend(loc='upper right')
    
    # Velocity subplot
    ax2 = fig.add_subplot(gs[1])
    for i, joint in enumerate(joint_names):
        if i < 2:  # First two joints (Joint A and Joint B)
            ax2.plot(times, traj_data[joint]['velocity'], label=joint)  # Already in units of steps/s
        else:  # All other joints (indices 2 and above)
            ax2.plot(times, [v/100 for v in traj_data[joint]['velocity']], label=joint)  # Divide by 100 to convert from 0.01 deg/s to degrees/s
    ax2.set_title('Joint Velocities Over Time')
    ax2.set_ylabel('Velocity (steps or degrees/s)')
    ax2.grid(True)
    ax2.legend(loc='upper right')
    
    # Acceleration subplot
    ax3 = fig.add_subplot(gs[2])
    for i, joint in enumerate(joint_names):
        if i < 2:  # First two joints (Joint A and Joint B)
            ax3.plot(times, traj_data[joint]['acceleration'], label=joint)  # Already in units of steps/s²
        else:  # All other joints (indices 2 and above)
            ax3.plot(times, [a/100 for a in traj_data[joint]['acceleration']], label=joint)  # Divide by 100 to convert from 0.01 deg/s² to degrees/s²
    ax3.set_title('Joint Accelerations Over Time')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Acceleration (steps or degrees/s²)')
    ax3.grid(True)
    ax3.legend(loc='upper right')
    
    plt.tight_layout()
    # plt.savefig('motion_profile.png', dpi=300)
    plt.show()


def main():
    # Lists to store time and joint data
    times = []
    
    # MANUALLY SPECIFY FILENAME
    filename = "2025-04-07_01-27-41-521629_L.npz"
    data = np.load(filename, allow_pickle=True)
    
    # Grab first file with .npz extension
    # data = np.load([f for f in os.listdir() if f.endswith('.npz')][0], allow_pickle=True)
    
    traj = data["trajectory_data"]
    print(len(traj))
    print(len(traj[0]))
    
    # Extract all trajectory data
    for step_num in range(len(traj)):
        t = traj[step_num][0]
        q = traj[step_num][1:8]
        v = traj[step_num][8:15]
        a = traj[step_num][15:22]
        j = traj[step_num][22:29]  # currently not used/plotted

        times.append(t)
        
        # Store position data
        traj_data['Joint A']['position'].append(q[0].item())
        traj_data['Joint B']['position'].append(q[1].item())
        # traj_data['Joint Wrist']['position'].append(q[2].item())
        # traj_data['Joint Endo Wrist Phi']['position'].append(q[3].item())
        # traj_data['Joint Endo Wrist Theta']['position'].append(q[4].item())
        # traj_data['Joint Endo Wrist A']['position'].append(q[5].item())
        # traj_data['Joint Endo Wrist B']['position'].append(q[6].item())
        
        # Store velocity data
        traj_data['Joint A']['velocity'].append(v[0].item())
        traj_data['Joint B']['velocity'].append(v[1].item())
        # traj_data['Joint Wrist']['velocity'].append(v[2].item())
        # traj_data['Joint Endo Wrist Phi']['velocity'].append(v[3].item())
        # traj_data['Joint Endo Wrist Theta']['velocity'].append(v[4].item())
        # traj_data['Joint Endo Wrist A']['velocity'].append(v[5].item())
        # traj_data['Joint Endo Wrist B']['velocity'].append(v[6].item())
        
        # Store acceleration data
        traj_data['Joint A']['acceleration'].append(a[0].item())
        traj_data['Joint B']['acceleration'].append(a[1].item())
        # traj_data['Joint Wrist']['acceleration'].append(a[2].item())
        # traj_data['Joint Endo Wrist Phi']['acceleration'].append(a[3].item())
        # traj_data['Joint Endo Wrist Theta']['acceleration'].append(a[4].item())
        # traj_data['Joint Endo Wrist A']['acceleration'].append(a[5].item())
        # traj_data['Joint Endo Wrist B']['acceleration'].append(a[6].item())
    
    # Generate motion profile plot
    plot_motion_profile(times, traj_data)

if __name__ == "__main__":
    main()