"""
OpenAI API Client Module

This module handles communication with the OpenAI API for vision tasks,
including sending requests and parsing responses.
"""

import os
import json
import requests
from typing import Dict, Any, List, Optional, Union
import logging

from grid_vision_config import OPENAI_API_KEY, OPENAI_MODEL

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class OpenAIClient:
    """Client for interacting with OpenAI's API for vision tasks."""
    
    def __init__(self, api_key: Optional[str] = None, model: Optional[str] = None):
        """
        Initialize the OpenAI API client.
        
        Args:
            api_key: OpenAI API key (if None, will use from config or env var)
            model: OpenAI model to use (if None, will use from config)
        """
        self.api_key = api_key or OPENAI_API_KEY or os.environ.get("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OpenAI API key is required. Set it in config, pass to constructor, or set OPENAI_API_KEY env var.")
            
        self.model = model or OPENAI_MODEL
        
        # API endpoints
        self.api_base = "https://api.openai.com/v1"
        self.chat_endpoint = f"{self.api_base}/chat/completions"
    
    def analyze_image(
        self,
        image_base64: str,
        prompt: str,
        max_tokens: int = 500,
        temperature: float = 0.3,
        response_format: Optional[Dict[str, str]] = None
    ) -> Dict[str, Any]:
        """
        Send an image to OpenAI API with a prompt for analysis.
        
        Args:
            image_base64: Base64-encoded image string
            prompt: Text prompt for the vision model
            max_tokens: Maximum tokens in response
            temperature: Sampling temperature (0.0 to 1.0)
            response_format: Optional format specification for response
            
        Returns:
            API response as dictionary
        """
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }
        
        # Create message with image
        content = [
            {"type": "text", "text": prompt},
            {
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/png;base64,{image_base64}"
                }
            }
        ]
        
        # Prepare the payload
        payload = {
            "model": self.model,
            "messages": [{"role": "user", "content": content}],
            "max_tokens": max_tokens,
            "temperature": temperature,
        }
        
        # Add response format if specified
        if response_format:
            payload["response_format"] = response_format
        
        try:
            logger.info(f"Sending request to OpenAI API using model: {self.model}")
            response = requests.post(self.chat_endpoint, headers=headers, json=payload)
            response.raise_for_status()  # Raise exception for HTTP errors
            
            return response.json()
        except requests.exceptions.RequestException as e:
            logger.error(f"Error communicating with OpenAI API: {e}")
            if hasattr(e, 'response') and e.response:
                logger.error(f"Response status: {e.response.status_code}")
                logger.error(f"Response body: {e.response.text}")
            raise
    
    def extract_json_from_response(self, response: Dict[str, Any]) -> Union[Dict[str, Any], List, str]:
        """
        Extract JSON content from an API response.
        
        Args:
            response: Raw API response
            
        Returns:
            Parsed JSON object or original text if parsing fails
        """
        try:
            # Extract the message content
            content = response["choices"][0]["message"]["content"].strip()
            
            # Try to parse as JSON
            # First, try to find JSON between code blocks if present
            json_pattern = "```json\n"
            if json_pattern in content:
                start = content.find(json_pattern) + len(json_pattern)
                end = content.rfind("```")
                if end > start:
                    json_str = content[start:end].strip()
                    logger.info(f"Found JSON in response: {json_str}")
                    return json.loads(json_str)
            
            # If not in code blocks, try the whole content
            logger.info(f"No JSON found in response, trying whole content: {content}")
            return json.loads(content)
        except (json.JSONDecodeError, KeyError) as e:
            logger.warning(f"Failed to parse JSON from response: {e}")
            
            # Return the raw text content if JSON parsing fails
            if "choices" in response and response["choices"]:
                return response["choices"][0]["message"]["content"]
            logger.info(f"No choices found in response: {response}")
            return response
            
    def analyze_image_for_points(
        self,
        image_base64: str,
        prompt: str,
        grid_reference_text: Optional[str] = None,
        request_json: bool = True
    ) -> Dict[str, Any]:
        """
        Analyze an image to identify specific points based on the prompt.
        
        Args:
            image_base64: Base64-encoded image string
            prompt: Text prompt describing what points to identify
            grid_reference_text: Optional text explaining the grid coordinate system
            request_json: Whether to request JSON formatted response
            
        Returns:
            Dictionary containing identified points with coordinates
        """
        # Combine the prompt with grid reference if provided
        full_prompt = prompt
        if grid_reference_text:
            full_prompt = f"{grid_reference_text}\n\n{prompt}"
            
        if request_json:
            full_prompt += "\n\nPlease respond in JSON format with the identified points and their coordinates."
            response_format = {"type": "json_object"}
        else:
            response_format = None
            
        # Send request to OpenAI API
        response = self.analyze_image(
            image_base64=image_base64,
            prompt=full_prompt,
            response_format=response_format
        )
        
        # Extract and parse the response
        if request_json:
            return self.extract_json_from_response(response)
        else:
            if "choices" in response and response["choices"]:
                return {"text_response": response["choices"][0]["message"]["content"]}
            return {"error": "Failed to get response from API"}
