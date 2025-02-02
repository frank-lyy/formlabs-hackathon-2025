from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    LeafSystem,
    AbstractValue,
    DiagramBuilder,
    BsplineTrajectory,
    CompositeTrajectory,
    PiecewisePolynomial,
    PathParameterizedTrajectory,
    KinematicTrajectoryOptimization,
    Parser,
    PositionConstraint,
    OrientationConstraint,
    SpatialVelocityConstraint,
    RigidTransform,
    Solve,
    RotationMatrix,
    JacobianWrtVariable,
    MinimumDistanceLowerBoundConstraint,
    LinearConstraint,
)
from manipulation.meshcat_utils import AddMeshcatTriad
from manipulation.utils import ConfigureParser

from motion_utils import ik

import time
import numpy as np


def VisualizePath(meshcat, plant, frame, traj, name):
    """
    Helper function that takes in trajopt basis and control points of Bspline
    and draws spline in meshcat.
    """
    traj_start_time = traj.start_time()
    traj_end_time = traj.end_time()

    # Build matrix of 3d positions by doing forward kinematics at time steps in the bspline
    NUM_STEPS = 50
    pos_3d_matrix = np.zeros((3,NUM_STEPS))
    ctr = 0
    plant_context = plant.CreateDefaultContext()
    for vis_t in np.linspace(traj_start_time, traj_end_time, NUM_STEPS):
        q = traj.value(vis_t)
        plant.SetPositions(plant_context, q)
        pos_3d = plant.CalcRelativeTransform(plant_context, plant.world_frame(), frame).translation()
        pos_3d_matrix[:,ctr] = pos_3d
        ctr += 1

    # Draw line
    meshcat.SetLine(name, pos_3d_matrix)
        
        
def KinematicTrajOpt(plant, plant_context, endowrist_model_instance_idx, frame_name, 
                     frame_model_instance_idx, wrist_joint_idx, X_Start, X_Goal, 
                     acceptable_pos_err=0.001, acceptable_angle_error=0.05, acceptable_vel_err=0.01):
    gripper_open_angle = 1.0
    
    frame = plant.GetFrameByName(frame_name, frame_model_instance_idx)
    
    trajopt = KinematicTrajectoryOptimization(plant.num_positions(), 8)  # 8 control points in Bspline
    prog = trajopt.get_mutable_prog()
    
    trajopt.AddPathLengthCost(1.0)
    trajopt.AddDurationCost(1.0)
    
    # Add custom cost to reward tilting the wrist higher (to raise string while moving)
    weight = 10
    control_points = trajopt.control_points()  # M-by-N matrix (M: positions, N: control points)
    for i in range(control_points.shape[1]):  # N control points
        prog.AddQuadraticCost(weight * (control_points[wrist_joint_idx, i] - (-0.4)) ** 2)  # Target wrist-to-endowrist angle: -0.4 radians
    
    trajopt.AddPositionBounds(
        plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits()
    )
    trajopt.AddVelocityBounds(
        plant.GetVelocityLowerLimits()/5, plant.GetVelocityUpperLimits()/5
    )
    trajopt.AddAccelerationBounds(
        plant.GetAccelerationLowerLimits()/25, plant.GetAccelerationUpperLimits()/25
    )
    
    start_pos_constraint = PositionConstraint(
        plant,
        plant.world_frame(),
        X_Start.translation() - acceptable_pos_err,  # lower limit
        X_Start.translation() + acceptable_pos_err,  # upper limit
        frame,
        [0, 0, 0],
        plant_context,
    )
    start_orientation_constraint = OrientationConstraint(
        plant,
        plant.world_frame(),
        X_Start.rotation(),  # orientation of X_Start in world frame ...
        frame,
        RotationMatrix(),
        acceptable_angle_error,
        plant_context
    )
    trajopt.AddPathPositionConstraint(start_pos_constraint, 0)
    trajopt.AddPathPositionConstraint(start_orientation_constraint, 0)
    
    goal_pos_constraint = PositionConstraint(
        plant,
        plant.world_frame(),
        X_Goal.translation() - acceptable_pos_err,  # lower limit
        X_Goal.translation() + acceptable_pos_err,  # upper limit
        frame,
        [0, 0, 0],
        plant_context
    )
    goal_orientation_constraint = OrientationConstraint(
        plant,
        plant.world_frame(),
        X_Goal.rotation(),  # orientation of X_Goal in world frame ...
        frame,
        RotationMatrix.MakeYRotation(-gripper_open_angle/2),  # So that the middle between the two grippers faces down
        acceptable_angle_error,
        plant_context
    )
    trajopt.AddPathPositionConstraint(goal_pos_constraint, 1)
    trajopt.AddPathPositionConstraint(goal_orientation_constraint, 1)
    
    # Zero final velocity constraint
    plant_autodiff = plant.ToAutoDiffXd()
    frame_autodiff = plant_autodiff.GetFrameByName(frame_name, frame_model_instance_idx)
    final_vel_constraint = SpatialVelocityConstraint(
        plant_autodiff,
        plant_autodiff.world_frame(),
        np.zeros(3) - acceptable_vel_err,  # lower limit
        np.zeros(3) + acceptable_vel_err,  # upper limit
        frame_autodiff,
        np.zeros(3),
        plant_autodiff.CreateDefaultContext(),
    )
    trajopt.AddVelocityConstraintAtNormalizedTime(final_vel_constraint, 1)
    
    # Non-collision constraints at finite samples
    collision_constraint = MinimumDistanceLowerBoundConstraint(plant, 0.0001, plant_context, None, 0.01)
    evaluate_at_s = np.linspace(0, 1, 25)
    # for s in evaluate_at_s:
    #     trajopt.AddPathPositionConstraint(collision_constraint, s)
        
    # Add open gripper constraint
    a = np.zeros((1, plant.num_positions()))
    a[0][plant.GetJointByName("joint_endowrist_body_endowrist_forcep1", endowrist_model_instance_idx).position_start()] = 1
    a[0][plant.GetJointByName("joint_endowrist_body_endowrist_forcep2", endowrist_model_instance_idx).position_start()] = -1
    lb = np.array([[gripper_open_angle]])
    ub = np.array([[np.inf]])
    for s in evaluate_at_s:
        trajopt.AddPathPositionConstraint(LinearConstraint(a, lb, ub), s)

    # Solve a linearly-interpolated IK problem to use as initial guess
    q_start, _ = ik(plant, plant_context, frame, X_Start, translation_error=0, rotation_error=0.05, regions=None, pose_as_constraint=True)
    q_goal, _ = ik(plant, plant_context, frame, X_Goal, translation_error=0, rotation_error=0.05, regions=None, pose_as_constraint=True)

    q_guess = np.linspace(q_start, q_goal, 8).T  # (num_positions, 8) np array
    initial_guess = BsplineTrajectory(trajopt.basis(), q_guess)
    trajopt.SetInitialGuess(initial_guess)

    result = Solve(prog)
    if not result.is_success():
        print("ERROR: Kinematic Traj Opt failed: " + str(result.get_solver_id().name()))
        print("Constraints Violated: " + str(result.GetInfeasibleConstraintNames(prog)))
    else:
        print("Kinematic Traj Opt succeeded.")

    final_traj = trajopt.ReconstructTrajectory(result)  # BSplineTrajectory
    return final_traj


def close_gripper(plant, plant_context, endowrist_model_instance_idx):
    pass


def open_gripper(plant, plant_context, endowrist_model_instance_idx):
    pass