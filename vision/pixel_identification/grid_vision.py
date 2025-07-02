"""
Grid Vision Main Module

This is the main interface for the Grid Vision system, integrating all components
to identify specific points in images using OpenAI's API with grid overlays.
"""

import os
import cv2
import numpy as np
from typing import Dict, List, Tuple, Any, Union, Optional
import logging
import time

from grid_vision_config import DEFAULT_GRID_SIZE, HIGH_RES_GRID_SIZE
from grid_overlay import GridOverlay
from image_processor import ImageProcessor
from openai_client import OpenAIClient
from response_parser import ResponseParser

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class GridVision:
    """
    Main class for identifying points in images using grid overlays and OpenAI API.
    """
    
    def __init__(
        self,
        api_key: Optional[str] = None,
        model: Optional[str] = None,
        grid_size: int = DEFAULT_GRID_SIZE,
        high_precision: bool = False,
        line_color: Tuple[int, int, int] = (255, 0, 0),
        line_thickness: int = 1,
        line_opacity: float = 0.7
    ):
        """
        Initialize the Grid Vision system.
        
        Args:
            api_key: OpenAI API key (optional if set in config or env var)
            model: OpenAI model to use (optional if set in config)
            grid_size: Size of the grid overlay (NxN)
            high_precision: If True, uses HIGH_RES_GRID_SIZE instead of grid_size
            line_color: RGB color for grid lines
            line_thickness: Thickness of grid lines
            line_opacity: Opacity of grid lines (0.0 to 1.0)
        """
        # Use high res grid if requested
        if high_precision:
            grid_size = HIGH_RES_GRID_SIZE
            
        # Initialize components
        self.grid_overlay = GridOverlay(
            grid_size=grid_size,
            line_color=line_color,
            line_thickness=line_thickness,
            line_opacity=line_opacity
        )
        self.image_processor = ImageProcessor()
        self.openai_client = OpenAIClient(api_key=api_key, model=model)
        self.response_parser = ResponseParser()
        
    def identify_points(
        self,
        image: Union[str, np.ndarray],
        prompt: str,
        include_grid_reference: bool = True,
        return_annotated_image: bool = False,
        request_json_response: bool = True,
        apply_resize: bool = True
    ) -> Dict[str, Any]:
        """
        Identify specific points in an image using OpenAI's vision API.
        
        Args:
            image: Path to image or numpy array containing image data
            prompt: Prompt describing what points to identify in the image
            include_grid_reference: Whether to include grid reference text in the prompt
            return_annotated_image: Whether to return the image with annotations
            request_json_response: Whether to request JSON formatted response from API
            apply_resize: Whether to resize the image before API submission
            
        Returns:
            Dictionary with identified points and optional annotated image
        """
        # Start timing
        start_time = time.time()
        logger.info("Starting point identification process")
        
        # Load image if path provided
        if isinstance(image, str):
            image_data = self.image_processor.load_image(image)
        else:
            image_data = image.copy()
        
        original_shape = image_data.shape[:2]
            
        # Apply grid overlay
        grid_image = self.grid_overlay.apply_grid(image_data)
        logger.info(f"Applied {self.grid_overlay.grid_size}x{self.grid_overlay.grid_size} grid overlay")
        
        # Process image for API
        processed = self.image_processor.process_image_for_api(grid_image, apply_resize=apply_resize)
        
        # Get grid reference text if needed
        grid_reference_text = self.grid_overlay.get_grid_reference_text() if include_grid_reference else None
        
        # Send to OpenAI API
        logger.info("Sending image to OpenAI API")
        api_response = self.openai_client.analyze_image_for_points(
            image_base64=processed["base64_image"],
            prompt=prompt,
            grid_reference_text=grid_reference_text,
            request_json=request_json_response
        )
        
        # Parse the response
        if request_json_response:
            parsed_response = self.response_parser.parse_json_response(api_response)
            standardized_points = self.response_parser.standardize_points_format(parsed_response)
        else:
            # Extract points from text response
            text_response = api_response.get("text_response", "")
            extracted_points = self.response_parser.extract_grid_coordinates(text_response)
            standardized_points = {
                point["name"]: {"grid_coordinates": (point["row"], point["col"])}
                for point in extracted_points
            }
            
        # Convert grid coordinates to pixel coordinates
        result = {
            "identified_points": {}
        }
        
        for name, data in standardized_points.items():
            if "grid_coordinates" in data:
                row, col = data["grid_coordinates"]
                pixel_x, pixel_y = self.grid_overlay.convert_grid_to_pixel_coordinates(row, col, original_shape)
                
                result["identified_points"][name] = {
                    "grid_coordinates": (row, col),
                    "pixel_coordinates": (pixel_x, pixel_y)
                }
            elif "pixel_coordinates" in data:
                pixel_x, pixel_y = data["pixel_coordinates"]
                row, col = self.grid_overlay.convert_pixel_to_grid_coordinates(pixel_x, pixel_y, original_shape)
                
                result["identified_points"][name] = {
                    "grid_coordinates": (row, col),
                    "pixel_coordinates": (pixel_x, pixel_y)
                }
        
        # Add annotated image if requested
        if return_annotated_image:
            # Use the image with grid lines as the base for annotations
            annotated = grid_image.copy()
            for name, point_data in result["identified_points"].items():
                x, y = point_data["pixel_coordinates"]
                annotated = self.image_processor.annotate_image(
                    annotated,
                    {name: (x, y)}
                )
            result["annotated_image"] = annotated
            
        # Add original response
        result["raw_response"] = api_response
        
        # Add timing information
        result["processing_time"] = time.time() - start_time
        
        return result
    
    def save_annotated_image(self, annotated_image: np.ndarray, output_path: str) -> str:
        """
        Save an annotated image to a file.
        
        Args:
            annotated_image: Image with annotations
            output_path: Path to save the image
            
        Returns:
            Path to the saved file
        """
        # Create directory if it doesn't exist
        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
        
        # Save the image
        cv2.imwrite(output_path, annotated_image)
        logger.info(f"Saved annotated image to {output_path}")
        
        return output_path
