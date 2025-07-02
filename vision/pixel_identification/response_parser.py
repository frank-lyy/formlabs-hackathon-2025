"""
Response Parser Module

This module handles parsing and interpreting responses from the OpenAI API,
extracting coordinates, and converting between different coordinate systems.
"""

import re
import json
from typing import Dict, List, Tuple, Any, Union, Optional
import logging
import ast

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ResponseParser:
    """Class to parse and extract information from API responses."""
    
    @staticmethod
    def extract_grid_coordinates(text: str) -> List[Dict[str, Any]]:
        """
        Extract grid coordinates from text response using regex patterns.
        
        Args:
            text: Text response from API
            
        Returns:
            List of dictionaries with point name, row, and column
        """
        # Pattern for coordinates like (3, 4) or (3.2, 4.7)
        coord_pattern = r"(\w+|\w+\s\w+).*?\((\d+\.?\d*),\s*(\d+\.?\d*)\)"
        
        points = []
        for match in re.finditer(coord_pattern, text):
            try:
                name = match.group(1).strip()
                row = float(match.group(2))
                col = float(match.group(3))
                
                points.append({
                    "name": name,
                    "row": row,
                    "col": col
                })
            except (IndexError, ValueError) as e:
                logger.warning(f"Failed to parse coordinate: {match.group(0)}, Error: {e}")
        
        return points
    
    @staticmethod
    def parse_json_response(response: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        """
        Parse JSON response from API.
        
        Args:
            response: JSON string or dictionary
            
        Returns:
            Parsed dictionary with points data
        """
        try:
            # If response is already a dictionary, use it directly
            if isinstance(response, dict):
                result = response
            else:
                # Try to parse JSON string
                result = json.loads(response)
            
            # Check if the response has a points key
            if "points" in result:
                return result
            
            # If there's no points key, try to extract points from the structure
            points = {}
            # Look for coordinates in the response
            for key, value in result.items():
                if isinstance(value, dict) and ("coordinates" in value or "position" in value):
                    # Extract coordinates from the nested dict
                    coords = value.get("coordinates", value.get("position", None))
                    if coords:
                        # Handle string-formatted coordinates like "(5.5, 6.8)"
                        if isinstance(coords, str) and coords.startswith("(") and coords.endswith(")"):
                            try:
                                # Try to parse as a tuple
                                parsed_coords = ast.literal_eval(coords)
                                logger.info(f"Parsed string coordinates: {coords} -> {parsed_coords}")
                                points[key] = parsed_coords
                            except (SyntaxError, ValueError) as e:
                                logger.warning(f"Failed to parse string coordinates: {coords}, Error: {e}")
                                # Fall back to regex parsing
                                coord_match = re.match(r"\s*\(\s*(\d+\.?\d*)\s*,\s*(\d+\.?\d*)\s*\)\s*", coords)
                                if coord_match:
                                    row = float(coord_match.group(1))
                                    col = float(coord_match.group(2))
                                    logger.info(f"Parsed coordinates with regex: {coords} -> ({row}, {col})")
                                    points[key] = (row, col)
                                else:
                                    points[key] = coords
                        else:
                            points[key] = coords
                elif isinstance(value, (list, tuple)) and len(value) >= 2:
                    # Assume list/tuple is coordinates
                    points[key] = value
            
            if points:
                return {"points": points}
            
            return result
        except (json.JSONDecodeError, AttributeError) as e:
            logger.error(f"Failed to parse JSON response: {e}")
            return {"error": str(e), "raw_response": response}
    
    @staticmethod
    def standardize_points_format(parsed_data: Dict[str, Any]) -> Dict[str, Dict[str, Union[float, Tuple[float, float]]]]:
        """
        Standardize different point formats into a consistent structure.
        
        Args:
            parsed_data: Parsed data from the API response
            
        Returns:
            Dictionary with standardized point data
        """
        standardized = {}
        
        # Handle different possible structures
        if "points" in parsed_data:
            points_data = parsed_data["points"]
            
            # Process each point
            for name, data in points_data.items():
                if isinstance(data, (list, tuple)) and len(data) >= 2:
                    # Direct coordinates as list/tuple
                    try:
                        standardized[name] = {
                            "grid_coordinates": (float(data[0]), float(data[1]))
                        }
                    except (IndexError, TypeError, ValueError) as e:
                        logger.warning(f"Failed to parse coordinates for {name}: {data}, Error: {e}")
                elif isinstance(data, dict):
                    # Coordinates in a nested structure
                    if "grid" in data:
                        grid_coords = data["grid"]
                        if isinstance(grid_coords, (list, tuple)) and len(grid_coords) >= 2:
                            standardized[name] = {
                                "grid_coordinates": (float(grid_coords[0]), float(grid_coords[1]))
                            }
                        elif isinstance(grid_coords, dict) and "row" in grid_coords and "col" in grid_coords:
                            standardized[name] = {
                                "grid_coordinates": (float(grid_coords["row"]), float(grid_coords["col"]))
                            }
                    elif "row" in data and "col" in data:
                        standardized[name] = {
                            "grid_coordinates": (float(data["row"]), float(data["col"]))
                        }
                    elif "coordinates" in data:
                        coords = data["coordinates"]
                        if isinstance(coords, (list, tuple)) and len(coords) >= 2:
                            standardized[name] = {
                                "grid_coordinates": (float(coords[0]), float(coords[1]))
                            }
                        elif isinstance(coords, dict) and "row" in coords and "col" in coords:
                            standardized[name] = {
                                "grid_coordinates": (float(coords["row"]), float(coords["col"]))
                            }
                        elif isinstance(coords, str):
                            # Try to parse string format like "(5.5, 6.8)"
                            coord_match = re.match(r"\s*\(\s*(\d+\.?\d*)\s*,\s*(\d+\.?\d*)\s*\)\s*", coords)
                            if coord_match:
                                row = float(coord_match.group(1))
                                col = float(coord_match.group(2))
                                logger.info(f"Parsed string coordinates for {name}: {coords} -> ({row}, {col})")
                                standardized[name] = {
                                    "grid_coordinates": (row, col)
                                }
                    elif "x" in data and "y" in data:
                        # Direct pixel coordinates
                        standardized[name] = {
                            "pixel_coordinates": (int(data["x"]), int(data["y"]))
                        }
        elif all(isinstance(item, dict) and "name" in item and "row" in item and "col" in item 
                for item in parsed_data.get("identified_points", [])):
            # Format with list of identified points
            for point in parsed_data["identified_points"]:
                standardized[point["name"]] = {
                    "grid_coordinates": (float(point["row"]), float(point["col"]))
                }
        
        return standardized
