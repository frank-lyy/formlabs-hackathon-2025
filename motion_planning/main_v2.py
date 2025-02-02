from pydrake.all import (
    StartMeshcat,
    AddDefaultVisualization,
    Simulator,
    RobotDiagramBuilder,
    SceneGraphCollisionChecker,
    RandomGenerator,
    PointCloud,
    Rgba,
    Quaternion,
    RigidTransform,
    RotationMatrix,
)
from manipulation.scenarios import AddMultibodyTriad
from manipulation.meshcat_utils import AddMeshcatTriad

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from motion_utils import get_left_right_joint_indices
from motion_planner import KinematicTrajOpt, VisualizePath, CloseGripper, OpenGripper, CombineTrajectories
from high_level_planner import HighLevelPlanner

import numpy as np
np.set_printoptions(linewidth=200)  # Set the line width to 200 characters

import importlib
import time

rng = RandomGenerator(1234)
np.random.seed(1234)

src_directory = os.path.dirname(os.path.abspath(__file__))
parent_directory = os.path.dirname(src_directory)
data_directory = os.path.join(parent_directory)
scene_yaml_file = os.path.join(data_directory, "assets", "robot.dmd.yaml")

meshcat = StartMeshcat()

robot_diagram_builder = RobotDiagramBuilder()
parser = robot_diagram_builder.parser()
scene_graph = robot_diagram_builder.scene_graph()
parser.package_map().Add("Robot.SLDASM", os.path.join(data_directory, "assets/Robot.SLDASM"))
parser.package_map().Add("Endowrist Mockup.SLDASM", os.path.join(data_directory, "assets/Endowrist Mockup.SLDASM"))
parser.package_map().Add("assets", os.path.join(data_directory, "assets"))
robot_model_instances = parser.AddModels(scene_yaml_file)
plant = robot_diagram_builder.plant()

# Find model instances with actuators for convenience
model_instances_indices_with_actuators = {}
for actuator_idx in plant.GetJointActuatorIndices():
    robot_model_instance_idx = plant.get_joint_actuator(actuator_idx).model_instance()
    if robot_model_instance_idx not in model_instances_indices_with_actuators.keys():
        model_instances_indices_with_actuators[robot_model_instance_idx] = 1
    else:
        model_instances_indices_with_actuators[robot_model_instance_idx] += 1

arms_model_instance_idx = list(model_instances_indices_with_actuators.keys())[0]
endowrist_left_model_instance_idx = list(model_instances_indices_with_actuators.keys())[1]
endowrist_right_model_instance_idx = list(model_instances_indices_with_actuators.keys())[2]

# Collect EEF frames and Draw Triads at EEF
# We treat forcep 1's frame as the EEF and simply offset each forcep angle a bit if we want to open the gripper
left_eef_frame = plant.GetFrameByName("endowrist_forcep1", endowrist_left_model_instance_idx)
right_eef_frame = plant.GetFrameByName("endowrist_forcep1", endowrist_right_model_instance_idx)
AddMultibodyTriad(left_eef_frame, scene_graph, length=0.05, radius=0.001, opacity=0.5)
AddMultibodyTriad(right_eef_frame, scene_graph, length=0.05, radius=0.001, opacity=0.5)

# Visualize zed2i camera transform
AddMultibodyTriad(plant.GetFrameByName("zed2i_left_camera_optical_frame"), scene_graph, length=0.7, radius=0.001, opacity=0.5)

plant.Finalize()

# Collect wrist joint indices for convenience (must be called post-Finalize)
left_wrist_joint_idx = plant.GetJointByName("joint_wrist_left_endowrist_left").position_start()
right_wrist_joint_idx = plant.GetJointByName("joint_wrist_right_endowrist_right").position_start()

# Collect left and right arm joint indices for convenience (must be called post-Finalize)
left_arm_joint_indices, right_arm_joint_indices = get_left_right_joint_indices(plant, endowrist_left_model_instance_idx, 
                                                                              endowrist_right_model_instance_idx, arms_model_instance_idx)

AddDefaultVisualization(robot_diagram_builder.builder(), meshcat=meshcat)
diagram = robot_diagram_builder.Build()

num_robot_positions = plant.num_positions()

simulator = Simulator(diagram)
context = simulator.get_mutable_context()
plant_context = plant.GetMyMutableContextFromRoot(context)

high_level_planner = HighLevelPlanner(plant, plant_context, left_eef_frame, right_eef_frame)

meshcat.StartRecording()
simulator.AdvanceTo(0.0001)

traj_zero_time = 0

# TEMPORARY
# (Left, Right)
X_Goal_seq = [(
                  RigidTransform(RotationMatrix.MakeYRotation(np.pi/2), np.array([0, -0.5, 0.015])),
                  RigidTransform(RotationMatrix.MakeYRotation(np.pi/2), np.array([-0.2, -0.5, 0.015]))
              ),
              (
                  RigidTransform(RotationMatrix.MakeYRotation(np.pi/2), np.array([0, -0.6, 0.015])),
                  RigidTransform(RotationMatrix.MakeZRotation(np.pi/2) @ RotationMatrix.MakeYRotation(np.pi/2), np.array([-0.1, -0.4, 0.015]))
              ),
              (
                  RigidTransform(RotationMatrix.MakeZRotation(np.pi/2) @ RotationMatrix.MakeYRotation(np.pi/2), np.array([0.2, -0.4, 0.015])),
                  RigidTransform(RotationMatrix.MakeYRotation(np.pi/2), np.array([-0.3, -0.5, 0.015]))
              ),
              (
                  RigidTransform(RotationMatrix.MakeZRotation(np.pi/2) @ RotationMatrix.MakeYRotation(np.pi/2), np.array([0.2, -0.4, 0.015])),
                  RigidTransform(RotationMatrix.MakeZRotation(np.pi/2) @ RotationMatrix.MakeYRotation(np.pi/2), np.array([0, -0.4, 0.015]))
              )]
# (Left, Right)
open_close_seq = [(1, 1), (0, 0), (1, 1), (0, 0)]  # 1 = close, 0 = open

def get_next_action():
    if action_idx >= len(X_Goal_seq):
        return None, None, None, None
    return X_Goal_seq[action_idx][0], X_Goal_seq[action_idx][1], open_close_seq[action_idx][0], open_close_seq[action_idx][1]

action_idx = 0
prev_open_close = (0, 0)  # open
while True:
    # X_Goal_L, X_Goal_R, open_close_L, open_close_R = get_next_action()
    X_Goal_L, X_Goal_R, open_close_L, open_close_R = high_level_planner.get_next_action()
    if X_Goal_L is None:
        break
    
    action_idx = high_level_planner.step_num
    
    AddMeshcatTriad(meshcat, f"X_Goal{action_idx}L", X_PT=X_Goal_L, length=0.01, radius=0.001, opacity=0.5)
    AddMeshcatTriad(meshcat, f"X_Goal{action_idx}R", X_PT=X_Goal_R, length=0.01, radius=0.001, opacity=0.5)
    
    X_Start_L = plant.CalcRelativeTransform(plant_context, plant.world_frame(), left_eef_frame)
    X_Start_R = plant.CalcRelativeTransform(plant_context, plant.world_frame(), right_eef_frame)
    
    print("GENERATING TRAJS")
    print(f"current pos: {plant.GetPositions(plant_context)}")
    trajL = KinematicTrajOpt(plant, plant_context, endowrist_left_model_instance_idx, "endowrist_forcep1", 
                             left_wrist_joint_idx, X_Start_L, X_Goal_L, prev_open_close[0])
    trajR = KinematicTrajOpt(plant, plant_context, endowrist_right_model_instance_idx, "endowrist_forcep1", 
                             right_wrist_joint_idx, X_Start_R, X_Goal_R, prev_open_close[1])
    VisualizePath(meshcat, plant, left_eef_frame, trajL, f"traj{action_idx}L")
    VisualizePath(meshcat, plant, right_eef_frame, trajR, f"traj{action_idx}R")
    
    if open_close_L:
        traj2L = CloseGripper(plant, endowrist_left_model_instance_idx, trajL.value(trajL.end_time()).flatten())
        trajL = CombineTrajectories([trajL, traj2L])  # If closing gripper, close as soon as arm reaches goal
    else:  # If opening gripper, open after both arms reach goal
        traj2L = OpenGripper(plant, endowrist_left_model_instance_idx, trajL.value(trajL.end_time()).flatten())
    
    if open_close_R:
        traj2R = CloseGripper(plant, endowrist_right_model_instance_idx, trajR.value(trajR.end_time()).flatten())
        trajR = CombineTrajectories([trajR, traj2R])  # If closing gripper, close as soon as arm reaches goal
    else:  # If opening gripper, open after both arms reach goal
        traj2R = OpenGripper(plant, endowrist_right_model_instance_idx, trajR.value(trajR.end_time()).flatten())

    while context.get_time() - traj_zero_time < max(trajL.end_time(), trajR.end_time()):           
        q_L = trajL.value(context.get_time() - traj_zero_time).flatten()
        q_R = trajR.value(context.get_time() - traj_zero_time).flatten()
        
        q_combined = np.zeros(14)
        q_combined[left_arm_joint_indices] = q_L[left_arm_joint_indices]
        q_combined[right_arm_joint_indices] = q_R[right_arm_joint_indices]
        
        plant.SetPositions(plant_context, q_combined)
        simulator.AdvanceTo(context.get_time() + 0.01)
        
    print("===================================================================")
    print("FINISHED TRAJ")
    print("===================================================================")
    traj_zero_time = context.get_time()
    
    # Run secondary trajectory for opening the gripper after both arms have reached their goals
    if not open_close_L or not open_close_R:
        while context.get_time() - traj_zero_time < max(traj2L.end_time(), traj2R.end_time()):
            q_combined = np.zeros(14)
            if not open_close_L:
                q_L = traj2L.value(context.get_time() - traj_zero_time).flatten()
                q_combined[left_arm_joint_indices] = q_L[left_arm_joint_indices]
            if not open_close_R:
                q_R = traj2R.value(context.get_time() - traj_zero_time).flatten()
                q_combined[right_arm_joint_indices] = q_R[right_arm_joint_indices]
            
            plant.SetPositions(plant_context, q_combined)
            simulator.AdvanceTo(context.get_time() + 0.01)
        
    print("===================================================================")
    print("FINISHED OPEN/CLOSE")
    print("===================================================================")
    traj_zero_time = context.get_time()
        
    action_idx += 1
    prev_open_close = (open_close_L, open_close_R)
    
time.sleep(3)
meshcat.PublishRecording()

collision_checker_params = {}
collision_checker_params["robot_model_instances"] = robot_model_instances
collision_checker_params["model"] = diagram
collision_checker_params["edge_step_size"] = 0.125
collision_checker = SceneGraphCollisionChecker(**collision_checker_params)
