"""
Grid Overlay Module

This module handles adding grid overlays to images for better point identification
by AI vision models. It supports customizable grid properties including size,
color, thickness, and opacity.
"""

import cv2
import numpy as np
from typing import Tuple, Optional, Dict, List
from grid_vision_config import (
    DEFAULT_GRID_SIZE, 
    HIGH_RES_GRID_SIZE,
    GRID_LINE_COLOR, 
    GRID_LINE_THICKNESS, 
    GRID_LINE_OPACITY
)

class GridOverlay:
    """Class to handle adding grid overlays to images."""
    
    def __init__(
        self, 
        grid_size: int = DEFAULT_GRID_SIZE,
        line_color: Tuple[int, int, int] = GRID_LINE_COLOR,
        line_thickness: int = GRID_LINE_THICKNESS,
        line_opacity: float = GRID_LINE_OPACITY
    ):
        """
        Initialize the grid overlay with specified parameters.
        
        Args:
            grid_size: Number of cells in each dimension (grid_size x grid_size)
            line_color: RGB tuple for grid line color
            line_thickness: Thickness of grid lines in pixels
            line_opacity: Opacity of grid lines (0.0 to 1.0)
        """
        self.grid_size = grid_size
        self.line_color = line_color
        self.line_thickness = line_thickness
        self.line_opacity = line_opacity
        
    def apply_grid(self, image: np.ndarray) -> np.ndarray:
        """
        Apply grid overlay to the input image.
        
        Args:
            image: Input image as numpy array (H, W, C)
            
        Returns:
            Image with grid overlay applied
        """
        # Make a copy of the image to avoid modifying the original
        result_image = image.copy()
        h, w = image.shape[:2]
        
        # Calculate cell size
        cell_height = h / self.grid_size
        cell_width = w / self.grid_size
        
        # Create a transparent overlay
        overlay = result_image.copy()
        
        # Draw horizontal lines
        for i in range(1, self.grid_size):
            y = int(i * cell_height)
            cv2.line(overlay, (0, y), (w, y), self.line_color, self.line_thickness)
            
        # Draw vertical lines
        for i in range(1, self.grid_size):
            x = int(i * cell_width)
            cv2.line(overlay, (x, 0), (x, h), self.line_color, self.line_thickness)
            
        # Apply the overlay with specified opacity
        cv2.addWeighted(overlay, self.line_opacity, result_image, 1 - self.line_opacity, 0, result_image)
        
        return result_image
        
    def get_cell_coordinates(self, row: int, col: int, image_shape: Tuple[int, int]) -> Dict:
        """
        Get the pixel coordinates of a specific grid cell.
        
        Args:
            row: Row index (0-indexed)
            col: Column index (0-indexed)
            image_shape: Tuple of (height, width) of the image
            
        Returns:
            Dictionary with the cell's coordinates (top_left, top_right, bottom_left, bottom_right, center)
        """
        h, w = image_shape[:2]
        cell_height = h / self.grid_size
        cell_width = w / self.grid_size
        
        # Calculate coordinates
        top_left = (int(col * cell_width), int(row * cell_height))
        top_right = (int((col + 1) * cell_width), int(row * cell_height))
        bottom_left = (int(col * cell_width), int((row + 1) * cell_height))
        bottom_right = (int((col + 1) * cell_width), int((row + 1) * cell_height))
        center = (int((col + 0.5) * cell_width), int((row + 0.5) * cell_height))
        
        return {
            "top_left": top_left,
            "top_right": top_right,
            "bottom_left": bottom_left,
            "bottom_right": bottom_right,
            "center": center
        }
    
    def get_grid_reference_text(self) -> str:
        """
        Generate a textual description of the grid for prompting the AI.
        
        Returns:
            String describing the grid system
        """
        return (
            f"The image contains a {self.grid_size}x{self.grid_size} grid overlay. "
            f"Please reference specific points using grid coordinates in the format (ROW, COLUMN) "
            f"where ROW goes from 0 (top) to {self.grid_size-1} (bottom) and "
            f"COLUMN goes from 0 (left) to {self.grid_size-1} (right). "
            f"For higher precision, you can use decimal values to specify locations within a cell, "
            f"e.g., (3.2, 4.7) means slightly below row 3 and right of column 4."
        )
    
    def convert_pixel_to_grid_coordinates(self, pixel_x: int, pixel_y: int, image_shape: Tuple[int, int]) -> Tuple[float, float]:
        """
        Convert pixel coordinates to grid coordinates.
        
        Args:
            pixel_x: X coordinate in pixels
            pixel_y: Y coordinate in pixels
            image_shape: Tuple of (height, width) of the image
            
        Returns:
            Tuple of (row, col) in grid coordinates
        """
        h, w = image_shape[:2]
        
        # Handle edge cases
        pixel_x = max(0, min(pixel_x, w - 1))
        pixel_y = max(0, min(pixel_y, h - 1))
        
        # Convert to grid coordinates
        col = (pixel_x / w) * self.grid_size
        row = (pixel_y / h) * self.grid_size
        
        return (row, col)
        
    def convert_grid_to_pixel_coordinates(self, row: float, col: float, image_shape: Tuple[int, int]) -> Tuple[int, int]:
        """
        Convert grid coordinates to pixel coordinates.
        
        Args:
            row: Row in grid coordinates
            col: Column in grid coordinates
            image_shape: Tuple of (height, width) of the image
            
        Returns:
            Tuple of (pixel_x, pixel_y)
        """
        h, w = image_shape[:2]
        
        # Convert to pixel coordinates
        pixel_x = int((col / self.grid_size) * w)
        pixel_y = int((row / self.grid_size) * h)
        
        # Handle edge cases
        pixel_x = max(0, min(pixel_x, w - 1))
        pixel_y = max(0, min(pixel_y, h - 1))
        
        return (pixel_x, pixel_y)
