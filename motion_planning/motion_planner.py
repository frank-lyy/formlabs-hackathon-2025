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

from motion_utils import ik, save_traj

import time
from datetime import datetime
import numpy as np

gripper_open_angle = 1.5  # rad
gripper_closed_angle = 0.175


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
        
        
def KinematicTrajOpt(plant, real_plant_context, endowrist_model_instance_idx, frame_name, 
                     wrist_joint_idx, X_Start, X_Goal, prev_open_close, additional_objectives=[], 
                     acceptable_pos_err=0.001, acceptable_angle_error=0.05, 
                     acceptable_vel_err=0.01, save_to_csv=True, L_R="L") -> BsplineTrajectory: 
    
    # This is based on testing at 500 steps/sec^2 on robot's joint A (which works reliably)
    JOINT_MAX_ACCELS = 3  # rad/s^2
    JOINT_MAX_JERKS = 300  # rad/s^3
    
    frame = plant.GetFrameByName(frame_name, endowrist_model_instance_idx)
    
    # If the start and goal are close, return a linear interpolation between the start and goal configurations
    if np.linalg.norm(X_Start.translation() - X_Goal.translation()) < 0.02 and np.linalg.norm((X_Start.rotation() @ X_Goal.rotation().transpose()).ToAngleAxis().angle()) < 0.1:
        q_start = plant.GetPositions(real_plant_context)
        print(f"q_start: {q_start}")
        q_goal, _ = ik(plant, frame, X_Goal, translation_error=0, rotation_error=0.05, regions=None, pose_as_constraint=True)
        
        # Very cheap workaround -- q_goal's ik solution seems to have bugged forcep angles, so we're just not gonna change the force angles at all. 
        forcep1_idx = plant.GetJointByName("joint_endowrist_body_endowrist_forcep1", endowrist_model_instance_idx).position_start()
        forcep2_idx = plant.GetJointByName("joint_endowrist_body_endowrist_forcep2", endowrist_model_instance_idx).position_start()
        q_goal[forcep1_idx] = q_start[forcep1_idx]
        q_goal[forcep2_idx] = q_start[forcep2_idx]
        
        duration = 1
        times = np.array([0., duration])
        positions = np.vstack((q_start, q_goal)).T  # Shape (num_positions, 2)
        return PiecewisePolynomial.FirstOrderHold(times, positions)
    
    plant_context = plant.CreateDefaultContext()
    
    trajopt = KinematicTrajectoryOptimization(plant.num_positions(), 8)  # 8 control points in Bspline
    prog = trajopt.get_mutable_prog()
    
    trajopt.AddPathEnergyCost(-0.5)
    trajopt.AddPathLengthCost(1.0)
    trajopt.AddDurationCost(1.0)
    
    # Add custom cost to reward tilting the wrist higher (to raise string while moving)
    weight = 10
    control_points = trajopt.control_points()  # M-by-N matrix (M: positions, N: control points)
    for i in range(control_points.shape[1]):  # N control points
        prog.AddQuadraticCost(weight * (control_points[wrist_joint_idx, i] - (-0.4)) ** 2)  # Target wrist-to-endowrist angle: -0.4 radians
    
    # Add any additional objectives defined in high_level_plan.yaml
    for objective in additional_objectives:
        prog.AddCost(objective)
    
    trajopt.AddPositionBounds(
        plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits()
    )
    trajopt.AddVelocityBounds(
        plant.GetVelocityLowerLimits()/10, plant.GetVelocityUpperLimits()/10
    )
    # Note: plant.GetAccelerationUpperLimits() just seems to return infinity
    trajopt.AddAccelerationBounds(
        -JOINT_MAX_ACCELS * np.ones(plant.num_positions()), JOINT_MAX_ACCELS * np.ones(plant.num_positions())
    )
    trajopt.AddJerkBounds(
        -JOINT_MAX_JERKS * np.ones(plant.num_positions()), JOINT_MAX_JERKS * np.ones(plant.num_positions())
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
    # Offset the gripping angle from if the gripper is open (i.e. the forcep is at an angle)
    goal_orientation_offset = RotationMatrix.MakeYRotation(-gripper_open_angle/2) if not prev_open_close else RotationMatrix()
    goal_orientation_constraint = OrientationConstraint(
        plant,
        plant.world_frame(),
        X_Goal.rotation(),  # orientation of X_Goal in world frame ...
        frame,
        goal_orientation_offset,  # So that the middle between the two grippers faces down
        acceptable_angle_error,
        plant_context
    )
    trajopt.AddPathPositionConstraint(goal_pos_constraint, 1)
    trajopt.AddPathPositionConstraint(goal_orientation_constraint, 1)
    
    # Zero Initial joint velocity constraint
    trajopt.AddPathVelocityConstraint(np.zeros(plant.num_positions()), np.zeros(plant.num_positions()), 0)
    
    # Zero final velocity constraint
    plant_autodiff = plant.ToAutoDiffXd()
    frame_autodiff = plant_autodiff.GetFrameByName(frame_name, endowrist_model_instance_idx)
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
    evaluate_at_s = np.linspace(0, 1, 25)
    # collision_constraint = MinimumDistanceLowerBoundConstraint(plant, 0.0001, plant_context, None, 0.01)
    # for s in evaluate_at_s:
    #     trajopt.AddPathPositionConstraint(collision_constraint, s)
        
    # Add open/close gripper constraint
    a = np.zeros((1, plant.num_positions()))
    a[0][plant.GetJointByName("joint_endowrist_body_endowrist_forcep1", endowrist_model_instance_idx).position_start()] = 1
    a[0][plant.GetJointByName("joint_endowrist_body_endowrist_forcep2", endowrist_model_instance_idx).position_start()] = -1
    if prev_open_close:  # Closed
        lb = np.array([[-0.01]])
        ub = np.array([[+0.01]])
    else:
        lb = np.array([[gripper_open_angle - 0.01]])
        ub = np.array([[gripper_open_angle + 0.01]])
    for s in evaluate_at_s:
        trajopt.AddPathPositionConstraint(LinearConstraint(a, lb, ub), s)

    # Set initial guess to be a linear interpolation between the start and goal
    q_start, _ = ik(plant, frame, X_Start, translation_error=0, rotation_error=0.05, regions=None, pose_as_constraint=True)
    q_goal, _ = ik(plant, frame, X_Goal, translation_error=0, rotation_error=0.05, regions=None, pose_as_constraint=True)
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
    if save_to_csv:
        filename = datetime.now().strftime('%Y-%m-%d_%H-%M-%S-%f')
        save_traj(final_traj, L_R, f"trajs/{filename}_{L_R}")
    return final_traj


def CloseGripper(plant, endowrist_model_instance_idx, joint_positions_at_close, duration=0.5) -> PiecewisePolynomial:
    forcep1_idx = plant.GetJointByName("joint_endowrist_body_endowrist_forcep1", endowrist_model_instance_idx).position_start()
    forcep2_idx = plant.GetJointByName("joint_endowrist_body_endowrist_forcep2", endowrist_model_instance_idx).position_start()
    
    current_forcep1_angle = joint_positions_at_close[forcep1_idx]
    current_forcep2_angle = joint_positions_at_close[forcep2_idx]
    
    target_forcep_angle = (current_forcep1_angle + current_forcep2_angle) / 2  # grippers close when they have the same angle
    target_plant_positions = joint_positions_at_close.copy()
    target_plant_positions[forcep1_idx] = target_forcep_angle
    target_plant_positions[forcep2_idx] = target_forcep_angle
    
    # Generate Trajectory object
    num_samples = 2
    times = np.linspace(0, duration, num_samples)

    # Initialize trajectory array: all joints remain constant except the forceps
    traj_values = np.tile(joint_positions_at_close[:, np.newaxis], num_samples)  # Shape (num_positions, num_samples)

    # Modify the forceps positions over time
    traj_values[forcep1_idx, :] = np.linspace(current_forcep1_angle, target_forcep_angle, num_samples)
    traj_values[forcep2_idx, :] = np.linspace(current_forcep2_angle, target_forcep_angle, num_samples)

    # Create a piecewise polynomial trajectory for the entire plant
    trajectory = PiecewisePolynomial.FirstOrderHold(times, traj_values)
    return trajectory


def OpenGripper(plant, endowrist_model_instance_idx, joint_positions_at_open, duration=0.5) -> PiecewisePolynomial:   
    forcep1_idx = plant.GetJointByName("joint_endowrist_body_endowrist_forcep1", endowrist_model_instance_idx).position_start()
    forcep2_idx = plant.GetJointByName("joint_endowrist_body_endowrist_forcep2", endowrist_model_instance_idx).position_start()
    
    current_forcep1_angle = joint_positions_at_open[forcep1_idx]
    current_forcep2_angle = joint_positions_at_open[forcep2_idx]
    
    target_forcep1_angle = current_forcep1_angle + gripper_open_angle/2
    target_forcep2_angle = current_forcep2_angle - gripper_open_angle/2
    
    # Generate Trajectory object
    num_samples = 50
    times = np.linspace(0, duration, num_samples)

    # Initialize trajectory array: all joints remain constant except the forceps
    traj_values = np.tile(joint_positions_at_open[:, np.newaxis], num_samples)  # Shape (num_positions, num_samples)

    # Modify the forceps positions over time
    traj_values[forcep1_idx, :] = np.linspace(current_forcep1_angle, target_forcep1_angle, num_samples)
    traj_values[forcep2_idx, :] = np.linspace(current_forcep2_angle, target_forcep2_angle, num_samples)

    # Create a piecewise polynomial trajectory for the entire plant
    trajectory = PiecewisePolynomial.FirstOrderHold(times, traj_values)
    return trajectory


def CombineTrajectories(trajectories) -> CompositeTrajectory:
    composite_segments = []
    current_time = 0.0
    
    for traj in trajectories:
        # Create time scaling to shift trajectory to start at current_time
        time_scaling = PiecewisePolynomial.FirstOrderHold(
            [current_time, current_time + traj.end_time()],
            np.array([[0, traj.end_time()-traj.start_time()]])
        )
        
        # Create time-shifted trajectory segment
        shifted_traj = PathParameterizedTrajectory(traj, time_scaling)
        composite_segments.append(shifted_traj)
        
        # Update time for next trajectory (add trajectory duration plus gap)
        current_time += traj.end_time()
    
    return CompositeTrajectory(composite_segments)