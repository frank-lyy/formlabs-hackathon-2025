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

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from station import MakeHardwareStation, load_scenario

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
parser.package_map().Add("Endowrist Mockup.SLDASM", os.path.join(data_directory, "assets/Endowrist Mockup.SLDASM"))
parser.package_map().Add("assets", os.path.join(data_directory, "assets"))
robot_model_instances = parser.AddModels(scene_yaml_file)
plant = robot_diagram_builder.plant()
plant.Finalize()
AddDefaultVisualization(robot_diagram_builder.builder(), meshcat=meshcat)
diagram = robot_diagram_builder.Build()

# Roll forward sim a bit to show the visualization
simulator = Simulator(diagram)
meshcat.StartRecording()
simulator.AdvanceTo(10)
meshcat.PublishRecording()

time.sleep(10)

plant_context = plant.CreateDefaultContext()

num_robot_positions = plant.num_positions()

collision_checker_params = {}
collision_checker_params["robot_model_instances"] = robot_model_instances
collision_checker_params["model"] = diagram
collision_checker_params["edge_step_size"] = 0.125
collision_checker = SceneGraphCollisionChecker(**collision_checker_params)
