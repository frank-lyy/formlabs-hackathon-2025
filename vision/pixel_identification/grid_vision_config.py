"""
Configuration settings for the Grid Vision API system.
Contains settings for OpenAI API, grid parameters, etc.
"""
import os
import dotenv
from pathlib import Path

dotenv.load_dotenv(Path(__file__).parent / '.env')

# OpenAI API configuration
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
OPENAI_MODEL = "gpt-4.1"  # Options: "gpt-4o", "gpt-4", "gpt-4-vision-preview"

# Grid overlay configuration
DEFAULT_GRID_SIZE = 10  # 10x10 grid (100 cells)
HIGH_RES_GRID_SIZE = 100  # 100x100 grid (10000 cells)
GRID_LINE_COLOR = (255, 0, 0)  # Red lines by default
GRID_LINE_THICKNESS = 1
GRID_LINE_OPACITY = 0.7  # 0.0 to 1.0

# Image processing settings
MAX_IMAGE_SIZE = 1024  # Maximum dimension in pixels for API submission
IMAGE_FORMAT = "png"  # Format for sending images to API
