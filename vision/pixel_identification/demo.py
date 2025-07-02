"""
Grid Vision Demo Script

This script demonstrates how to use the Grid Vision system to identify
specific points in images using OpenAI's API with grid overlays.
"""

import argparse
import os
import cv2
import numpy as np
import time
from typing import Dict, Any

# Import the Grid Vision system
from grid_vision import GridVision
from grid_vision_config import OPENAI_API_KEY, OPENAI_MODEL

def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Grid Vision Demo')
    parser.add_argument('--image', type=str, required=True, help='Path to input image')
    parser.add_argument('--prompt', type=str, required=True, help='Prompt describing points to identify')
    parser.add_argument('--grid-size', type=int, default=10, help='Grid size (NxN)')
    parser.add_argument('--high-precision', action='store_true', help='Use high precision grid (100x100)')
    parser.add_argument('--output', type=str, default='annotated_output.png', help='Path to save annotated output')
    parser.add_argument('--opacity', type=float, default=0.7, help='Grid line opacity (0.0-1.0)')
    parser.add_argument('--model', type=str, default=OPENAI_MODEL, help='OpenAI model to use')
    return parser.parse_args()

def display_results(results: Dict[str, Any], output_path: str):
    """Display the identification results."""
    print("\n===== GRID VISION RESULTS =====")
    print(f"Processing time: {results['processing_time']:.2f} seconds")
    print(f"\nIdentified {len(results['identified_points'])} points:")
    
    for name, point_data in results['identified_points'].items():
        grid_row, grid_col = point_data['grid_coordinates']
        pixel_x, pixel_y = point_data['pixel_coordinates']
        print(f"- {name}: Grid ({grid_row:.2f}, {grid_col:.2f}), Pixel ({pixel_x}, {pixel_y})")
    
    print(f"\nAnnotated image saved to: {output_path}")

def main():
    """Main function to run the demo."""
    args = parse_arguments()
    
    # Check if image exists
    if not os.path.exists(args.image):
        print(f"Error: Image not found at {args.image}")
        return
    
    # Initialize GridVision
    grid_vision = GridVision(
        grid_size=args.grid_size,
        high_precision=args.high_precision,
        line_opacity=args.opacity,
        model=args.model
    )
    
    print(f"Analyzing image: {args.image}")
    print(f"Using grid size: {args.grid_size}x{args.grid_size}")
    print(f"Using {'high precision mode' if args.high_precision else 'standard precision mode'}")
    print(f"Prompt: {args.prompt}")
    print("Processing...")
    
    # Identify points in the image
    results = grid_vision.identify_points(
        image=args.image,
        prompt=args.prompt,
        return_annotated_image=True
    )
    
    # Save the annotated image
    if 'annotated_image' in results:
        grid_vision.save_annotated_image(results['annotated_image'], args.output)
    
    # Display results
    display_results(results, args.output)

if __name__ == "__main__":
    main()
