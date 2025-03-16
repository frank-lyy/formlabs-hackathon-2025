"""
Buggy Teleop helper program. See comment at top of motion_planning/main.py
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
    AddDefaultVisualization,
)
from manipulation.scenarios import AddMultibodyTriad

from motion_utils import get_left_right_joint_indices, ik

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from station import MakeHardwareStation, load_scenario

import numpy as np
import time
import importlib
import argparse

src_directory = os.path.dirname(os.path.abspath(__file__))
parent_directory = os.path.dirname(src_directory)
data_directory = os.path.join(parent_directory)
scene_yaml_file = os.path.join(data_directory, "assets", "robot.dmd.yaml")


class MeshcatSliderSource(LeafSystem):
    def __init__(self, meshcat):
        LeafSystem.__init__(self)
        self.meshcat = meshcat

        if joint_control:
            self.DeclareVectorOutputPort("slider_values", BasicVector(2*num_robot_positions), self.DoCalcOutput)
        else:
            self.DeclareVectorOutputPort("slider_values", BasicVector(6), self.DoCalcOutput)

    def DoCalcOutput(self, context, output):
        if joint_control:
            out = []
            for i in range(num_robot_positions):
                out.append(self.meshcat.GetSliderValue(f'q{i}'))
            out += [0]*num_robot_positions
            output.SetFromVector(out)
        else:
            x = self.meshcat.GetSliderValue('x')
            y = self.meshcat.GetSliderValue('y')
            z = self.meshcat.GetSliderValue('z')
            roll = self.meshcat.GetSliderValue('roll')
            pitch = self.meshcat.GetSliderValue('pitch')
            yaw = self.meshcat.GetSliderValue('yaw')
            output.SetFromVector([x, y, z, roll, pitch, yaw])


class InverseKinematicsSystem(LeafSystem):
    def __init__(self, plant, meshcat):
        LeafSystem.__init__(self)
        self.plant = plant
        self.meshcat = meshcat
        self.DeclareVectorInputPort("slider_values", BasicVector(6))
        self.DeclareVectorOutputPort("desired_state", BasicVector(num_robot_positions*2), self.CalculateDesiredState)
        
    def CalculateDesiredState(self, context, output):
        slider_values = self.get_input_port(0).Eval(context)
        x, y, z, roll, pitch, yaw = slider_values
        desired_pose = RigidTransform(RotationMatrix(RollPitchYaw(roll, pitch, yaw)), [x, y, z])
        
        ik = InverseKinematics(self.plant)
        ik.AddPositionConstraint(
            ee_frame,
            [0, 0, 0],
            self.plant.world_frame(),
            desired_pose.translation(),
            desired_pose.translation()
        )
        ik.AddOrientationConstraint(
            ee_frame,
            RotationMatrix(),
            self.plant.world_frame(),
            desired_pose.rotation(),
            0.05
        )
        
        prog = ik.prog()
        prog.SetInitialGuess(ik.q(), default_joint_positions)
        result = Solve(prog)
        
        if result.is_success():
            q_solution = result.GetSolution(ik.q())
            v_solution = np.zeros_like(q_solution)  # Assuming zero velocity for simplicity
            desired_state = np.concatenate([q_solution, v_solution])
        else:
            print("ik fail; defaulting to zero state.")
            desired_state = np.zeros(num_robot_positions*2)
            
        output.SetFromVector(desired_state)


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


parser = argparse.ArgumentParser()
parser.add_argument('--joint_control', default='T', help="T/F; whether to control joint positions (instead of xyz rpy)")
args = parser.parse_args()
joint_control = (args.joint_control == 'T')

assert joint_control, "Joint control is not supported yet"


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
default_joint_positions = plant.GetPositions(plant.CreateDefaultContext())

ee_frame = plant.GetFrameByName("link_endowrist_box_left_ref")

if joint_control:
    lower_limits = plant.GetPositionLowerLimits()
    upper_limits = plant.GetPositionUpperLimits()
    # Add slider for each joint
    for i in range(num_robot_positions):
        meshcat.AddSlider(f'q{i}', lower_limits[i], upper_limits[i], 0.01, default_joint_positions[i])
else:
    default_pose = plant.CalcRelativeTransform(plant.CreateDefaultContext(), plant.world_frame(), ee_frame)
    meshcat.AddSlider('x', -1.0, 2.0, 0.01, default_pose.translation()[0])
    meshcat.AddSlider('y', -1.0, 1.0, 0.01, default_pose.translation()[1])
    meshcat.AddSlider('z', 0.0, 3.0, 0.01, default_pose.translation()[2])
    meshcat.AddSlider('roll', -np.pi, np.pi, 0.01, default_pose.rotation().ToRollPitchYaw().vector()[0])
    meshcat.AddSlider('pitch', -np.pi, np.pi, 0.01, default_pose.rotation().ToRollPitchYaw().vector()[1])
    meshcat.AddSlider('yaw', -np.pi, np.pi, 0.01, default_pose.rotation().ToRollPitchYaw().vector()[2])
meshcat.AddButton("Close")


while not meshcat.GetButtonClicks("Close"):
    if joint_control:
        # Get joint positions from sliders
        q = []
        for i in range(num_robot_positions):
            q.append(meshcat.GetSliderValue(f'q{i}'))
        
    plant.SetPositions(plant_context, q)
    simulator.AdvanceTo(context.get_time() + 0.01)
















# # Figure out how many robots there are and how many joints each has
# model_instances_indices_with_actuators = {}
# for actuator_idx in plant.GetJointActuatorIndices():
#     robot_model_instance_idx = plant.get_joint_actuator(actuator_idx).model_instance()
#     if robot_model_instance_idx not in model_instances_indices_with_actuators.keys():
#         model_instances_indices_with_actuators[robot_model_instance_idx] = 1
#     else:
#         model_instances_indices_with_actuators[robot_model_instance_idx] += 1
        
# print(model_instances_indices_with_actuators)

# # for model_instance_idx in model_instances_indices_with_actuators.keys():
# #     plant.set_gravity_enabled(model_instance_idx, False)

# # Add Meshcat Slider Source System
# slider_source = builder.AddSystem(MeshcatSliderSource(meshcat))

# # Add controller and splitter (for when there are multiple robots)
# controller = builder.AddSystem(InverseDynamicsController(plant, [100]*num_robot_positions, [0]*num_robot_positions, [50]*num_robot_positions, True))  # True = exposes "desired_acceleration" port
# control_splitter = builder.AddSystem(VectorSplitter(*model_instances_indices_with_actuators.values()))

# # Set controller desired state
# builder.Connect(station.GetOutputPort("state"), controller.GetInputPort("estimated_state"))
# builder.Connect(controller.GetOutputPort("generalized_force"), control_splitter.GetInputPort("input"))

# if joint_control:
#     builder.Connect(slider_source.get_output_port(0), controller.GetInputPort("desired_state"))
# else:
#     # Add IK System
#     ik_system = builder.AddSystem(InverseKinematicsSystem(plant, meshcat))
#     # Connect sliders to IK system
#     builder.Connect(slider_source.get_output_port(0), ik_system.get_input_port(0))
#     builder.Connect(ik_system.get_output_port(0), controller.GetInputPort("desired_state"))

# slider_source_context = slider_source.GetMyMutableContextFromRoot(simulator_context)
# if not joint_control:
#     ik_system_context = ik_system.GetMyMutableContextFromRoot(simulator_context)

# # Main simulation loop
# meshcat.StartRecording()
# ctr = 0
# plant.SetPositions(plant_context, slider_source.get_output_port(0).Eval(slider_source_context)[:num_robot_positions])
# while not meshcat.GetButtonClicks("Close"):
#     # if joint_control:
#     #     plant.SetPositions(plant_context, slider_source.get_output_port(0).Eval(slider_source_context)[:num_robot_positions])
#     # else:
#     #     plant.SetPositions(plant_context, ik_system.get_output_port(0).Eval(ik_system_context)[:num_robot_positions])
#     simulator.AdvanceTo(simulator_context.get_time() + 0.001)
#     ctr += 1
#     if (ctr == 1000):
#         ctr = 0
#         print(plant.GetPositions(plant_context))
        
# meshcat.PublishRecording()