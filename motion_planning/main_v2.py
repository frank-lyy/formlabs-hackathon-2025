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
)
from manipulation.scenarios import AddMultibodyTriad
from manipulation.meshcat_utils import AddMeshcatTriad

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from motion_utils import ik

import numpy as np
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
plant.Finalize()
AddDefaultVisualization(robot_diagram_builder.builder(), meshcat=meshcat)
diagram = robot_diagram_builder.Build()

num_robot_positions = plant.num_positions()

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

simulator = Simulator(diagram)
context = simulator.get_mutable_context()
plant_context = plant.GetMyMutableContextFromRoot(context)

AddMultibodyTriad(plant.GetFrameByName("endowrist_forcep1", endowrist_left_model_instance_idx), scene_graph)
AddMultibodyTriad(plant.GetFrameByName("endowrist_forcep2", endowrist_left_model_instance_idx), scene_graph)
AddMultibodyTriad(plant.GetFrameByName("endowrist_forcep1", endowrist_right_model_instance_idx), scene_graph)
AddMultibodyTriad(plant.GetFrameByName("endowrist_forcep2", endowrist_right_model_instance_idx), scene_graph)

meshcat.StartRecording()

# ik(plant, plant_context, pose, translation_error=0, rotation_error=0.05, regions=None, pose_as_constraint=True)

interpolated_array = np.linspace(0, 0.5, 100)[:, None] * np.ones((1, num_robot_positions))
for i in range(interpolated_array.shape[0]):
    plant.SetPositions(plant_context, interpolated_array[i])
    simulator.AdvanceTo(context.get_time() + 0.001)
    
time.sleep(10)
meshcat.PublishRecording()

collision_checker_params = {}
collision_checker_params["robot_model_instances"] = robot_model_instances
collision_checker_params["model"] = diagram
collision_checker_params["edge_step_size"] = 0.125
collision_checker = SceneGraphCollisionChecker(**collision_checker_params)
