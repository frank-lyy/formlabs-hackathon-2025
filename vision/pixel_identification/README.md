# Grid Vision System

A modular infrastructure for identifying specific points in images using OpenAI's Vision API with grid overlays.

## Overview

This system leverages OpenAI's vision capabilities to identify specific points in images. It uses a grid overlay technique to improve accuracy in point identification by providing clear reference coordinates. The system supports both standard (10×10) and high-precision (100×100) grids with adjustable properties.

## Features

- **Grid Overlay**: Adds customizable grid lines to images with adjustable size, color, thickness, and opacity
- **Coordinate Systems**: Supports both grid coordinates (row, column) and pixel coordinates (x, y)
- **High Precision Mode**: Option for 100×100 grid (10,000 cells) for more accurate point identification
- **Image Processing**: Automatic resizing and formatting for API compatibility
- **Response Parsing**: Extracts point coordinates from various API response formats
- **Annotation**: Visualizes identified points on the original image

## Architecture

The system consists of the following modules:

1. **`grid_vision.py`**: Main interface module that integrates all components
2. **`grid_overlay.py`**: Handles adding grid overlays to images
3. **`image_processor.py`**: Processes images for API submission
4. **`openai_client.py`**: Communicates with OpenAI's API
5. **`response_parser.py`**: Parses and standardizes API responses
6. **`grid_vision_config.py`**: Configuration settings for the system
7. **`demo.py`**: Demonstration script for using the system

## Installation

1. Install required dependencies:
```bash
pip install opencv-python numpy pillow requests
```

2. Set your OpenAI API key:
   - Edit `grid_vision_config.py` and add your API key
   - Or set it as an environment variable:
   ```bash
   export OPENAI_API_KEY="your_api_key_here"
   ```

## Usage

### Basic Usage

```python
from vision.grid_vision import GridVision

# Initialize the system
grid_vision = GridVision()

# Identify points in an image
results = grid_vision.identify_points(
    image="path/to/image.jpg",
    prompt="Identify the following points in the image: 1. The tip of the object, 2. The center of the red circle"
)

# Access the identified points
for name, point_data in results["identified_points"].items():
    print(f"{name}: {point_data['pixel_coordinates']}")
```

### High Precision Mode

```python
# Initialize with high precision grid (100×100)
grid_vision = GridVision(high_precision=True)
```

### Customizing Grid Appearance

```python
# Customize grid appearance
grid_vision = GridVision(
    grid_size=20,  # 20×20 grid
    line_color=(0, 0, 255),  # Blue lines
    line_thickness=2,
    line_opacity=0.5
)
```

### Running the Demo

```bash
python -m vision.demo --image path/to/image.jpg --prompt "Identify the center of the object" --grid-size 10 --output result.png
```

## How It Works

1. **Grid Overlay**: The system adds a grid overlay to the input image, creating a clear coordinate system.
2. **API Submission**: The image with grid is sent to OpenAI's Vision API along with a prompt and grid reference explanation.
3. **Response Processing**: The API's response is parsed to extract point coordinates.
4. **Coordinate Conversion**: Grid coordinates are converted to pixel coordinates for practical use.
5. **Visualization**: Identified points can be visualized on the original image.

## Prompt Engineering

For best results:

1. Be specific about what points you want to identify
2. Use clear descriptions with visual cues (e.g., "the red dot at the top-left")
3. Request coordinates in the grid system (the system automatically explains the grid reference)

Example effective prompt:
```
Identify the following points in this image:
1. The top-left corner of the blue rectangle
2. The center of the circular button
3. The tip of the arrow pointing right
```

## Tips for Optimal Results

- Use a grid size appropriate for the level of precision needed
- Adjust grid opacity for best visibility against the image content
- For fine details, use high precision mode (100×100 grid)
- Request JSON responses for more structured data (default)
- Use clear and specific prompts describing what points to identify
