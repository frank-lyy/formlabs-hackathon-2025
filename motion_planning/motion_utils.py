from pydrake.all import (
    Diagram,
    RigidTransform,
    RotationMatrix,
    MultibodyPlant,
    Context,
    VPolytope,
    Point,
    InverseKinematics,
    Solve,
    logical_or,
    logical_and,
    Quaternion,
)

from typing import BinaryIO, Union
import numpy as np
import pydot
import yaml
import os
import sys
import time

q_nominal = np.zeros(14)


def diagram_visualize_connections(diagram: Diagram, file: Union[BinaryIO, str]) -> None:
    """
    Create SVG file of system diagram.
    """
    if type(file) is str:
        file = open(file, "bw")
    svg_data = pydot.graph_from_dot_data(
        diagram.GetGraphvizString())[0].create_svg()
    file.write(svg_data)
    

def save_traj(traj, filename, dt=0.01):
    """
    Create npz file with dense sampling of the trajectory, including dt.

    traj: Trajectory
    filename: str
    dt: float, time step
    """
    time_start = traj.start_time()
    time_end = traj.end_time()

    time_samples = np.arange(time_start, time_end, dt)
    trajectory_data = []

    for t in time_samples:
        state = traj.value(t)
        trajectory_data.append([t] + list(state))  # Append time and state as a list

    trajectory_data = np.array(trajectory_data)
    np.savez(filename, trajectory_data=trajectory_data, dt=dt)


def ik(plant, frame, pose, rotation_offset=RotationMatrix(), translation_error=0, rotation_error=0.05, regions=None, pose_as_constraint=True) -> tuple[np.ndarray, bool]:
    """
    Use Inverse Kinematics to solve for a configuration that satisfies a
    task-space pose for a given frame. 
    
    If regions is not None, this function also ensures the configuration is
    reachable within one of the regions (or return None if this isn't possible).

    pose_as_constraint can be set to False if there is a meaningful chance the
    ik program will not be able to find a viable solution for the given pose,
    i.e. if the regions passed in don't have perfect coverage. Then, the IK
    program will strictly ensure the returned solution falls within one of the
    regions but do its best on the desired pose.

    Returns the result of the IK program and a boolean for whether the program
    and all constraint were successfully solved.
    """
    satisfy_regions_constraint = regions is not None
    if regions is None:  # Make regions not None so that the for loop below runs at least once
        regions = {"_": Point(np.zeros(6))}

    # Separate IK program for each region with the constraint that the IK result must be in that region
    ik_start = time.time()
    solve_success = False
    for region in list(regions.values()):
        ik = InverseKinematics(plant, plant.CreateDefaultContext())
        q_variables = ik.q()  # Get variables for MathematicalProgram
        ik_prog = ik.get_mutable_prog()

        # q_variables must be within half-plane for every half-plane in region
        if satisfy_regions_constraint:
            ik_prog.AddConstraint(logical_and(*[expr <= const for expr, const in zip(region.A() @ q_variables, region.b())]))

        if pose_as_constraint:
            ik.AddPositionConstraint(
                frameA=plant.world_frame(),
                frameB=frame,
                p_BQ=[0, 0, 0],
                p_AQ_lower=pose.translation() - translation_error,
                p_AQ_upper=pose.translation() + translation_error,
            )
            ik.AddOrientationConstraint(
                frameAbar=plant.world_frame(),
                R_AbarA=pose.rotation(),
                frameBbar=frame,
                R_BbarB=rotation_offset,
                theta_bound=rotation_error,
            )
            ik_prog.AddQuadraticErrorCost(np.identity(len(q_variables)), q_nominal, q_variables)
        else:
            # Add costs instead of constraints for pose
            ik.AddPositionCost(plant.world_frame(),
                               pose.translation(),
                               frame,
                               [0, 0, 0],
                               np.identity(3))
            ik.AddOrientationCost(plant.world_frame(),
                                  pose.rotation(),
                                  frame,
                                  RotationMatrix(),
                                  1)

        ik_prog.SetInitialGuess(q_variables, q_nominal)
        ik_result = Solve(ik_prog)
        if ik_result.is_success():
            q = ik_result.GetSolution(q_variables)  # (6,) np array
            # print(f"IK solve succeeded. q: {q}")
            solve_success = True
            break
        # else:
            # print(f"ERROR: IK fail: {ik_result.get_solver_id().name()}: {ik_result.GetInfeasibleConstraintNames(ik_prog)}")

    # print(f"IK Runtime: {time.time() - ik_start}")

    if solve_success == False:
        print(f"ERROR: IK to pose {pose} fail: {ik_result.get_solver_id().name()}. Returning Best Guess.")
        return ik_result.GetSolution(q_variables), False
    
    return q, True


def get_left_right_joint_indices(plant, endowrist_left_model_instance_idx, 
                                 endowrist_right_model_instance_idx, arms_model_instance_idx) -> tuple[list[int], list[int]]:
    # Collect joint indices for left and right arms
    left_arm_joint_names = [
        "joint_arms_mount_arm_left",
        "joint_arm_left_wrist_left",
        "joint_wrist_left_endowrist_left",
        # Endowrist joints for left arm
        "joint_endowrist_box_endowrist_tube",  # Phi
        "joint_endowrist_tube_endowrist_body",  # Theta
        "joint_endowrist_body_endowrist_forcep1",  # Alpha
        "joint_endowrist_body_endowrist_forcep2"  # Beta
    ]

    right_arm_joint_names = [
        "joint_arms_mount_arm_right",
        "joint_arm_right_wrist_right",
        "joint_wrist_right_endowrist_right",
        # Endowrist joints for right arm
        "joint_endowrist_box_endowrist_tube",  # Phi
        "joint_endowrist_tube_endowrist_body",  # Theta
        "joint_endowrist_body_endowrist_forcep1",  # Alpha
        "joint_endowrist_body_endowrist_forcep2"  # Beta
    ]

    # Get indices for left arm joints
    left_arm_joint_indices = []
    for joint_name in left_arm_joint_names:
        try:
            if joint_name.startswith("joint_endowrist"):
                joint = plant.GetJointByName(joint_name, endowrist_left_model_instance_idx)
            else:
                joint = plant.GetJointByName(joint_name, arms_model_instance_idx)
            left_arm_joint_indices.append(joint.position_start())
        except RuntimeError:
            print(f"Warning: Could not find joint {joint_name}")

    # Get indices for right arm joints
    right_arm_joint_indices = []
    for joint_name in right_arm_joint_names:
        try:
            if joint_name.startswith("joint_endowrist"):
                joint = plant.GetJointByName(joint_name, endowrist_right_model_instance_idx)
            else:
                joint = plant.GetJointByName(joint_name, arms_model_instance_idx)
            right_arm_joint_indices.append(joint.position_start())
        except RuntimeError:
            print(f"Warning: Could not find joint {joint_name}")
            
    return left_arm_joint_indices, right_arm_joint_indices