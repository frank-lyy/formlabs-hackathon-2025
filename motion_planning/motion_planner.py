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
    MinimumDistanceLowerBoundConstraint ,
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
        
        
def KinematicTrajOpt(plant, plant_context, frame, X_Start, X_Goal, acceptable_pos_err=0.01, acceptable_angle_error=0.05):
    trajopt = KinematicTrajectoryOptimization(plant.num_positions(), 8)  # 8 control points in Bspline
    prog = trajopt.get_mutable_prog()
    
    trajopt.AddPathLengthCost(1.0)
    
    trajopt.AddPositionBounds(
        plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits()
    )
    trajopt.AddVelocityBounds(
        plant.GetVelocityLowerLimits(), plant.GetVelocityUpperLimits()
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
        RotationMatrix(),
        acceptable_angle_error,
        plant_context
    )
    trajopt.AddPathPositionConstraint(goal_pos_constraint, 1)
    trajopt.AddPathPositionConstraint(goal_orientation_constraint, 1)
    
    # Non-collision constraints at finite samples
    collision_constraint = MinimumDistanceLowerBoundConstraint(plant, 0.0001, plant_context, None, 0.01)
    evaluate_at_s = np.linspace(0, 1, 25)
    for s in evaluate_at_s:
        trajopt.AddPathPositionConstraint(collision_constraint, s)
        
    # Add open gripper constraint
    

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