import yaml
import numpy as np

# Load YAML file
with open("high_level_plan.yaml", "r") as file:
    data = yaml.safe_load(file)

# Extract reference_point_clouds as numpy arrays
steps_data = []
for step in data["steps"]:
    step_id = step["step_id"]
    reference_point_cloud = np.array(step["reference_point_cloud"])  # Convert list to numpy array
    steps_data.append((step_id, reference_point_cloud))

# Print extracted data
for step_id, ref_cloud in steps_data:
    print(f"Step {step_id}:")
    print(ref_cloud)
    print()
