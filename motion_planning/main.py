"""
IGNORE

For some reason, inverse dynamics controller seems to be unstable. To be debugged.
"""

from pydrake.all import (
    DiagramBuilder,
    StartMeshcat,
    MeshcatVisualizer,
    LeafSystem,
    Simulator,
    RigidTransform,
    MultibodyPlant,
    RobotDiagramBuilder,
    Parser,
    InverseDynamicsController,
    ConstantVectorSource,
    BasicVector,
    Rgba,
    RotationMatrix,
    RollPitchYaw,
    InverseKinematics,
    Solve,
)

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from station import MakeHardwareStation, load_scenario
from motion_utils import ik, average_transform, diagram_visualize_connections

import numpy as np
import time
import importlib
import argparse

src_directory = os.path.dirname(os.path.abspath(__file__))
parent_directory = os.path.dirname(src_directory)
data_directory = os.path.join(parent_directory)
scene_yaml_file = os.path.join(data_directory, "assets", "robot.dmd.yaml")


class VectorSplitter(LeafSystem):
    """
    Simple LeafSystem that takes a vector input of size out1 + out2 and splits 
    it into 2 output vectors of size out1 and out2. This is used for multi-robot
    plants so the output of one controller can control both robots.
    """
    def __init__(self, out1, out2=0, out3=0):
        super().__init__()
        self.out1 = out1
        self.out2 = out2
        self.out3 = out3
        self.DeclareVectorInputPort("input", BasicVector(out1 + out2 + out3))
        self.DeclareVectorOutputPort("output_1", BasicVector(out1), self.Output1)
        if out2 > 0:
            self.DeclareVectorOutputPort("output_2", BasicVector(out2), self.Output2)
        if out3 > 0:
            self.DeclareVectorOutputPort("output_3", BasicVector(out3), self.Output3)

    def Output1(self, context, output):
        input_vector = self.get_input_port(0).Eval(context)
        output.SetFromVector(input_vector[:self.out1])  # return first `out1` elements

    def Output2(self, context, output):
        input_vector = self.get_input_port(0).Eval(context)
        output.SetFromVector(input_vector[self.out1:self.out1 + self.out2])  # return middle `out2` elements

    def Output3(self, context, output):
        input_vector = self.get_input_port(0).Eval(context)
        output.SetFromVector(input_vector[self.out1 + self.out2:])  # return latter `out3` elements

meshcat = StartMeshcat()
meshcat.AddButton("Close")

builder = DiagramBuilder()
scenario = load_scenario(filename=scene_yaml_file)

def parser_preload_callback(parser):
    parser.package_map().Add("Robot.SLDASM", os.path.join(data_directory, "assets/Robot.SLDASM"))
    parser.package_map().Add("Endowrist Mockup.SLDASM", os.path.join(data_directory, "assets/Endowrist Mockup.SLDASM"))
    parser.package_map().Add("assets", os.path.join(data_directory, "assets"))

# Hardware station setup
station = builder.AddSystem(MakeHardwareStation(
    scenario=scenario,
    meshcat=meshcat,

    # This is to be able to load our own models from a local path
    # we can refer to this using the "package://" URI directive
    parser_preload_callback=parser_preload_callback
))
scene_graph = station.GetSubsystemByName("scene_graph")
plant = station.GetSubsystemByName("plant")

num_robot_positions = plant.num_positions()
default_joint_positions = plant.GetPositions(plant.CreateDefaultContext())

# Figure out how many robots there are and how many joints each has
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

# ee_left_frame = average_transform(plant.GetFrameByName("endowrist_forcep1", endowrist_left_model_instance_idx), plant.GetFrameByName("endowrist_forcep2", endowrist_left_model_instance_idx))
# ee_right_frame = average_transform(plant.GetFrameByName("endowrist_forcep1", endowrist_right_model_instance_idx), plant.GetFrameByName("endowrist_forcep2", endowrist_right_model_instance_idx  ))

# Add controller and splitter (for when there are multiple robots)
controller = builder.AddSystem(InverseDynamicsController(plant, [0.001]*num_robot_positions, [0]*num_robot_positions, [0]*num_robot_positions, True))  # True = exposes "desired_acceleration" port
control_splitter = builder.AddSystem(VectorSplitter(*model_instances_indices_with_actuators.values()))

# Set controller desired state
builder.Connect(station.GetOutputPort("state"), controller.GetInputPort("estimated_state"))
builder.Connect(controller.GetOutputPort("generalized_force"), control_splitter.GetInputPort("input"))

zero_state_source = builder.AddSystem(ConstantVectorSource([0, 0, 0, -1.57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] + [0]*num_robot_positions))
builder.Connect(zero_state_source.get_output_port(0), controller.GetInputPort("desired_state"))  # TEMPORARY

# Set controller desired accel
zero_accel_source = builder.AddSystem(ConstantVectorSource([0]*num_robot_positions))  # TEMPORARY
builder.Connect(zero_accel_source.get_output_port(0), controller.GetInputPort("desired_acceleration"))

# Connect each output of the splitter to the actuation input for each robot
for i, (robot_model_instance_idx, num_joints) in enumerate(model_instances_indices_with_actuators.items()):
    builder.Connect(control_splitter.GetOutputPort(f"output_{i+1}"), station.GetInputPort(f"{plant.GetModelInstanceName(robot_model_instance_idx)}_actuation"))

diagram = builder.Build()
diagram_visualize_connections(diagram, "diagram.svg")
context = diagram.CreateDefaultContext()

simulator = Simulator(diagram)
simulator_context = simulator.get_mutable_context()
station_context = station.GetMyMutableContextFromRoot(simulator_context)
plant_context = plant.GetMyMutableContextFromRoot(simulator_context)

# Main simulation loop
meshcat.StartRecording()
ctr = 0
while not meshcat.GetButtonClicks("Close"):
    simulator.AdvanceTo(simulator_context.get_time() + 0.001)
    
    if ctr == 100:
        print(plant.GetPositions(plant_context))
        ctr = 0
    ctr+= 1
    
meshcat.PublishRecording()