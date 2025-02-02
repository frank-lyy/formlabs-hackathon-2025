"""
Template file that shows how to build a generic MultibodyPlant containing one of
the 9 test scenes.
"""

from pydrake.all import (
    StartMeshcat,
    AddDefaultVisualization,
    Simulator,
    VisibilityGraph,
    RobotDiagramBuilder,
    VPolytope,
    HPolyhedron,
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
from motion_utils import ik
from motion_planner import KinematicTrajOpt, VisualizePath, CloseGripper, OpenGripper

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
AddMultibodyTriad(plant.GetFrameByName("zed2i_left_camera_optical_frame"), scene_graph, length=0.05, radius=0.001, opacity=0.5)

plant.Finalize()

# Collect wrist joint indices for convenience (must be called post-Finalize)
left_wrist_joint_idx = plant.GetJointByName("joint_wrist_left_endowrist_left").position_start()
right_wrist_joint_idx = plant.GetJointByName("joint_wrist_right_endowrist_right").position_start()

AddDefaultVisualization(robot_diagram_builder.builder(), meshcat=meshcat)
diagram = robot_diagram_builder.Build()

num_robot_positions = plant.num_positions()

simulator = Simulator(diagram)
context = simulator.get_mutable_context()
plant_context = plant.GetMyMutableContextFromRoot(context)

meshcat.StartRecording()
simulator.AdvanceTo(0.0001)

traj_zero_time = 0

# Open Gripper
traj = OpenGripper(plant, plant_context, endowrist_left_model_instance_idx)
while context.get_time() - traj_zero_time < traj.end_time():
    # left_forcep1_idx = plant.GetJointByName("joint_endowrist_body_endowrist_forcep1", endowrist_left_model_instance_idx).position_start()
    # left_forcep2_idx = plant.GetJointByName("joint_endowrist_body_endowrist_forcep2", endowrist_left_model_instance_idx).position_start()
    # right_forcep1_idx = plant.GetJointByName("joint_endowrist_body_endowrist_forcep1", endowrist_right_model_instance_idx).position_start()
    # right_forcep2_idx = plant.GetJointByName("joint_endowrist_body_endowrist_forcep2", endowrist_right_model_instance_idx).position_start()
    
    q = traj.value(context.get_time() - traj_zero_time)
    plant.SetPositions(plant_context, q)
    simulator.AdvanceTo(context.get_time() + 0.001)
traj_zero_time = context.get_time()

# TEMPORARY
X_Goal_seq = [RigidTransform(RotationMatrix.MakeYRotation(np.pi/2), np.array([0, -0.5, 0.015])),
           RigidTransform(RotationMatrix.MakeYRotation(np.pi/2), np.array([0, -0.6, 0.015])),
           RigidTransform(RotationMatrix.MakeZRotation(np.pi/2) @ RotationMatrix.MakeYRotation(np.pi/2), np.array([0.2, -0.4, 0.015]))]
open_close_seq = [1, 0, 1]  # 1 = close, 0 = open

def get_next_action():
    if action_idx >= len(X_Goal_seq):
        return None, None
    return X_Goal_seq[action_idx], open_close_seq[action_idx]

action_idx = 0
prev_open_close = 0  # open
while True:
    X_Goal, open_close = get_next_action()
    if X_Goal is None:
        break
    
    AddMeshcatTriad(meshcat, f"X_Goal{action_idx}", X_PT=X_Goal, length=0.01, radius=0.001, opacity=0.5)
    
    if X_Goal is None:
        break
    
    X_Start = plant.CalcRelativeTransform(plant_context, plant.world_frame(), left_eef_frame)
    
    traj1 = KinematicTrajOpt(plant, plant_context, endowrist_left_model_instance_idx, "endowrist_forcep1", 
                            endowrist_left_model_instance_idx, left_wrist_joint_idx, X_Start, X_Goal, prev_open_close)
    if open_close:
        traj2 = CloseGripper(plant, plant_context, endowrist_left_model_instance_idx)
    else:
        traj2 = OpenGripper(plant, plant_context, endowrist_left_model_instance_idx)
    
    VisualizePath(meshcat, plant, left_eef_frame, traj1, f"traj{action_idx}")

    for traj in [traj1, traj2]:
        while context.get_time() - traj_zero_time < traj.end_time():
            # left_forcep1_idx = plant.GetJointByName("joint_endowrist_body_endowrist_forcep1", endowrist_left_model_instance_idx).position_start()
            # left_forcep2_idx = plant.GetJointByName("joint_endowrist_body_endowrist_forcep2", endowrist_left_model_instance_idx).position_start()
            # right_forcep1_idx = plant.GetJointByName("joint_endowrist_body_endowrist_forcep1", endowrist_right_model_instance_idx).position_start()
            # right_forcep2_idx = plant.GetJointByName("joint_endowrist_body_endowrist_forcep2", endowrist_right_model_instance_idx).position_start()
            
            q = traj.value(context.get_time() - traj_zero_time)
            plant.SetPositions(plant_context, q)
            simulator.AdvanceTo(context.get_time() + 0.001)
        
        traj_zero_time = context.get_time()
        
    action_idx += 1
    prev_open_close = open_close
    
time.sleep(3)
meshcat.PublishRecording()

collision_checker_params = {}
collision_checker_params["robot_model_instances"] = robot_model_instances
collision_checker_params["model"] = diagram
collision_checker_params["edge_step_size"] = 0.125
collision_checker = SceneGraphCollisionChecker(**collision_checker_params)
