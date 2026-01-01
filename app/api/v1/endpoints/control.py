"""
Control endpoints - Structures HTTP data into ROS2 message format.
"""
from fastapi import APIRouter, Request, HTTPException, status
from typing import Optional
import logging

from app.models.schemas import CommandRequest, CommandResponse, ErrorResponse
from app.utils.validators import validate_command_string, sanitize_command

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post(
    "/command",
    response_model=CommandResponse,
    status_code=status.HTTP_200_OK,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid command"},
        500: {"model": ErrorResponse, "description": "ROS2 node error"},
        503: {"model": ErrorResponse, "description": "ROS2 node not available"}
    },
    summary="Send control command to ROS2",
    description="Structures HTTP data into ROS2 message format and publishes to ROS2 topic"
)
async def send_command(
    request: CommandRequest,
    http_request: Request
) -> CommandResponse:
    """
    Send a control command to ROS2.
    
    This endpoint:
    1. Receives HTTP request with command data
    2. Validates and sanitizes the command
    3. Structures it into ROS2 message format
    4. Publishes to ROS2 topic
    
    Args:
        request: Command request with command data
        http_request: FastAPI request object to access app state
        
    Returns:
        CommandResponse: Response with command status
    """
    # Get ROS2 node from app state
    ros2_node = getattr(http_request.app.state, 'ros2_node', None)
    
    if ros2_node is None:
        logger.error("ROS2 node not initialized")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="ROS2 node is not initialized. Please check the service status."
        )
    
    # Validate command
    if not validate_command_string(request.command):
        logger.warning(f"Invalid command received: {request.command[:50]}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid command. Command must be a non-empty string."
        )
    
    # Sanitize command
    sanitized_command = sanitize_command(request.command)
    
    # Prepare command data (could include metadata if needed)
    command_data = sanitized_command
    
    # If command type is JSON, validate it
    if request.command_type == "json":
        from app.utils.validators import is_valid_json
        if not is_valid_json(sanitized_command):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid JSON format in command"
            )
    
    # Publish to ROS2
    try:
        success = ros2_node.send_command(command_data)
        
        if success:
            logger.info(f"Command published successfully: {sanitized_command[:50]}")
            return CommandResponse(
                status="sent",
                command=sanitized_command,
                success=True,
                message="Command published successfully to ROS2 topic"
            )
        else:
            logger.error(f"Failed to publish command: {sanitized_command[:50]}")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to publish command to ROS2 topic"
            )
            
    except Exception as e:
        logger.error(f"Error publishing command: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error publishing command: {str(e)}"
        )


@router.post(
    "/command/simple",
    response_model=CommandResponse,
    status_code=status.HTTP_200_OK,
    summary="Send simple command (string only)",
    description="Simplified endpoint that accepts just a command string"
)
async def send_simple_command(
    command: str,
    http_request: Request
) -> CommandResponse:
    """
    Send a simple command string to ROS2.
    
    This is a simplified endpoint that accepts just a string command.
    
    Args:
        command: Command string to send
        http_request: FastAPI request object
        
    Returns:
        CommandResponse: Response with command status
    """
    # Create a CommandRequest from the simple string
    request = CommandRequest(command=command)
    
    # Use the main send_command function
    return await send_command(request, http_request)

