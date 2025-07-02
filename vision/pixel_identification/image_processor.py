"""
Image Processor Module

This module handles image processing operations required before sending images 
to the OpenAI API, including loading, resizing, and format conversion.
"""

import cv2
import numpy as np
import base64
import io
from PIL import Image
from typing import Tuple, Union, Optional, Dict, Any
from grid_vision_config import MAX_IMAGE_SIZE, IMAGE_FORMAT
import logging

logger = logging.getLogger(__name__)

class ImageProcessor:
    """Class to handle image preprocessing operations."""
    
    @staticmethod
    def load_image(image_path: str) -> np.ndarray:
        """
        Load an image from a file path.
        
        Args:
            image_path: Path to the image file
            
        Returns:
            Image as numpy array (BGR format)
            
        Raises:
            FileNotFoundError: If the image file does not exist
            ValueError: If the image cannot be read
        """
        img = cv2.imread(image_path)
        if img is None:
            raise ValueError(f"Could not read image: {image_path}")
        return img
    
    @staticmethod
    def resize_image(image: np.ndarray, max_size: int = MAX_IMAGE_SIZE) -> np.ndarray:
        """
        Resize an image to fit within max_size while preserving aspect ratio.
        
        Args:
            image: Input image as numpy array (BGR format)
            max_size: Maximum dimension (width or height) in pixels
            
        Returns:
            Resized image as numpy array
        """
        h, w = image.shape[:2]
        
        # No need to resize if already smaller than max_size
        if max(h, w) <= max_size:
            logger.info("Image is smaller than max_size, no need to resize")
            return image
        
        # Calculate new dimensions preserving aspect ratio
        if h > w:
            new_h = max_size
            new_w = int(w * (max_size / h))
        else:
            new_w = max_size
            new_h = int(h * (max_size / w))
        
        # Resize the image
        return cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
    
    @staticmethod
    def convert_to_rgb(image: np.ndarray) -> np.ndarray:
        """
        Convert BGR image (OpenCV default) to RGB.
        
        Args:
            image: Input image in BGR format
            
        Returns:
            Image in RGB format
        """
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    @staticmethod
    def convert_to_base64(image: np.ndarray, format: str = IMAGE_FORMAT) -> str:
        """
        Convert an image to base64 string for API submission.
        
        Args:
            image: Input image (RGB format recommended)
            format: Output image format (e.g., 'png', 'jpeg')
            
        Returns:
            Base64 encoded image string
        """
        # Convert to PIL Image
        pil_img = Image.fromarray(image)
        
        # Save to bytes buffer
        buffer = io.BytesIO()
        pil_img.save(buffer, format=format)
        buffer.seek(0)
        
        # Convert to base64
        img_base64 = base64.b64encode(buffer.getvalue()).decode('utf-8')
        
        return img_base64
    
    @staticmethod
    def annotate_image(
        image: np.ndarray,
        points: Dict[str, Tuple[int, int]],
        color: Tuple[int, int, int] = (0, 255, 0),
        radius: int = 5,
        thickness: int = 2,
        text_size: float = 0.5
    ) -> np.ndarray:
        """
        Annotate an image with identified points.
        
        Args:
            image: Input image
            points: Dictionary mapping point names to (x, y) coordinates
            color: RGB color for the annotations
            radius: Radius of the circle markers
            thickness: Thickness of the circle outline
            text_size: Size of the text labels
            
        Returns:
            Annotated image
        """
        result = image.copy()
        
        for name, (x, y) in points.items():
            # Draw circle
            cv2.circle(result, (x, y), radius, color, thickness)
            
            # Add label
            cv2.putText(
                result, name, (x + radius, y - radius),
                cv2.FONT_HERSHEY_SIMPLEX, text_size, color, thickness
            )
            
        return result
    
    def process_image_for_api(
        self, 
        image: Union[str, np.ndarray], 
        apply_resize: bool = True
    ) -> Dict[str, Any]:
        """
        Process an image for submission to the OpenAI API.
        
        Args:
            image: Input image path or numpy array
            apply_resize: Whether to resize the image if needed
            
        Returns:
            Dictionary with processed image data for API
        """
        # Load image if path provided
        if isinstance(image, str):
            img = self.load_image(image)
        else:
            img = image.copy()
        
        # Store original dimensions
        original_shape = img.shape[:2]
        
        # Resize if needed
        if apply_resize:
            img = self.resize_image(img)
        
        # Convert to RGB
        img_rgb = self.convert_to_rgb(img)
        
        # Convert to base64
        img_base64 = self.convert_to_base64(img_rgb)
        
        return {
            "base64_image": img_base64,
            "format": IMAGE_FORMAT,
            "original_shape": original_shape,
            "processed_shape": img_rgb.shape[:2]
        }
