"""
Helper functions used across the application.
"""
import re
import json
from typing import Union, Dict, Any, Optional
from datetime import datetime


def validate_command_string(command: str, max_length: int = 10000) -> bool:
    """
    Validate a command string.
    
    Args:
        command: Command string to validate
        max_length: Maximum allowed length
        
    Returns:
        bool: True if valid, False otherwise
    """
    if not isinstance(command, str):
        return False
    
    if len(command.strip()) == 0:
        return False
    
    if len(command) > max_length:
        return False
    
    return True


def sanitize_command(command: str) -> str:
    """
    Sanitize a command string by removing dangerous characters.
    
    Args:
        command: Command string to sanitize
        
    Returns:
        str: Sanitized command
    """
    # Remove null bytes and control characters (except newline, tab, carriage return)
    sanitized = re.sub(r'[\x00-\x08\x0B-\x0C\x0E-\x1F]', '', command)
    
    # Limit length
    max_length = 10000
    if len(sanitized) > max_length:
        sanitized = sanitized[:max_length]
    
    return sanitized.strip()


def is_valid_json(data: str) -> bool:
    """
    Check if a string is valid JSON.
    
    Args:
        data: String to check
        
    Returns:
        bool: True if valid JSON, False otherwise
    """
    try:
        json.loads(data)
        return True
    except (json.JSONDecodeError, TypeError):
        return False


def parse_json_safely(data: str) -> Optional[Dict[str, Any]]:
    """
    Safely parse JSON string.
    
    Args:
        data: JSON string to parse
        
    Returns:
        dict: Parsed JSON or None if invalid
    """
    try:
        return json.loads(data)
    except (json.JSONDecodeError, TypeError):
        return None


def format_timestamp(dt: Optional[datetime] = None) -> str:
    """
    Format datetime to ISO 8601 string.
    
    Args:
        dt: Datetime object (defaults to now)
        
    Returns:
        str: ISO formatted timestamp
    """
    if dt is None:
        dt = datetime.now()
    return dt.isoformat()


def validate_topic_name(topic_name: str) -> bool:
    """
    Validate ROS2 topic name format.
    
    Args:
        topic_name: Topic name to validate
        
    Returns:
        bool: True if valid ROS2 topic name
    """
    if not isinstance(topic_name, str):
        return False
    
    # ROS2 topic names must start with /
    if not topic_name.startswith('/'):
        return False
    
    # Check for valid characters (alphanumeric, _, /)
    pattern = r'^/[a-zA-Z0-9_/]+$'
    return bool(re.match(pattern, topic_name))


def validate_node_name(node_name: str) -> bool:
    """
    Validate ROS2 node name format.
    
    Args:
        node_name: Node name to validate
        
    Returns:
        bool: True if valid ROS2 node name
    """
    if not isinstance(node_name, str):
        return False
    
    # Node names should not be empty
    if len(node_name.strip()) == 0:
        return False
    
    # Check for valid characters (alphanumeric, _)
    pattern = r'^[a-zA-Z0-9_]+$'
    return bool(re.match(pattern, node_name))


def truncate_string(s: str, max_length: int = 100, suffix: str = "...") -> str:
    """
    Truncate a string to maximum length.
    
    Args:
        s: String to truncate
        max_length: Maximum length
        suffix: Suffix to add if truncated
        
    Returns:
        str: Truncated string
    """
    if len(s) <= max_length:
        return s
    return s[:max_length - len(suffix)] + suffix


def safe_get_nested(data: Dict[str, Any], *keys, default: Any = None) -> Any:
    """
    Safely get nested dictionary value.
    
    Args:
        data: Dictionary to search
        *keys: Keys to traverse
        default: Default value if key not found
        
    Returns:
        Value at nested key or default
    """
    result = data
    for key in keys:
        if isinstance(result, dict) and key in result:
            result = result[key]
        else:
            return default
    return result

