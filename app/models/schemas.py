from pydantic import BaseModel, Field, validator
from typing import Optional, Dict, Any, List
from datetime import datetime
from enum import Enum


class CommandType(str, Enum):
    """Types of commands that can be sent."""
    STRING = "string"
    JSON = "json"
    RAW = "raw"


class CommandRequest(BaseModel):
    """
    Request model for sending control commands.
    
    This structures HTTP data into ROS2 message format.
    """
    command: str = Field(
        ...,
        description="Command string to send to ROS2",
        min_length=1,
        max_length=10000
    )
    command_type: Optional[CommandType] = Field(
        default=CommandType.STRING,
        description="Type of command being sent"
    )
    metadata: Optional[Dict[str, Any]] = Field(
        default=None,
        description="Optional metadata associated with the command"
    )
    
    @validator('command')
    def validate_command(cls, v):
        """Validate that command is not empty after stripping."""
        if not v.strip():
            raise ValueError('Command cannot be empty or whitespace only')
        return v.strip()
    
    class Config:
        json_schema_extra = {
            "example": {
                "command": "move_forward",
                "command_type": "string",
                "metadata": {"priority": "high", "timeout": 5}
            }
        }


class CommandResponse(BaseModel):
    """
    Response model for command submission.
    """
    status: str = Field(..., description="Status of the command submission")
    command: str = Field(..., description="The command that was sent")
    timestamp: datetime = Field(default_factory=datetime.now, description="When the command was sent")
    success: bool = Field(..., description="Whether the command was published successfully")
    message: Optional[str] = Field(default=None, description="Additional message or error details")
    
    class Config:
        json_schema_extra = {
            "example": {
                "status": "sent",
                "command": "move_forward",
                "timestamp": "2024-01-01T12:00:00",
                "success": True,
                "message": "Command published successfully"
            }
        }


class HealthStatus(str, Enum):
    """Health status values."""
    HEALTHY = "healthy"
    UNHEALTHY = "unhealthy"
    DEGRADED = "degraded"


class ROS2Status(BaseModel):
    """
    ROS2 node status information.
    """
    connected: bool = Field(..., description="Whether ROS2 node is connected")
    node_name: Optional[str] = Field(default=None, description="Name of the ROS2 node")
    topic_name: Optional[str] = Field(default=None, description="ROS2 topic name")
    publisher_ready: bool = Field(default=False, description="Whether publisher is ready")
    subscribers_count: int = Field(default=0, description="Number of subscribers")


class HealthResponse(BaseModel):
    """
    Response model for health check endpoint.
    
    Reports current status of FastAPI and ROS2 Node states.
    """
    status: HealthStatus = Field(..., description="Overall health status")
    fastapi: Dict[str, Any] = Field(..., description="FastAPI application status")
    ros2: ROS2Status = Field(..., description="ROS2 node status")
    timestamp: datetime = Field(default_factory=datetime.now, description="When the health check was performed")
    version: str = Field(..., description="Application version")
    
    class Config:
        json_schema_extra = {
            "example": {
                "status": "healthy",
                "fastapi": {
                    "running": True,
                    "uptime_seconds": 3600
                },
                "ros2": {
                    "connected": True,
                    "node_name": "fastapi_control_node",
                    "topic_name": "/control/commands",
                    "publisher_ready": True,
                    "subscribers_count": 1
                },
                "timestamp": "2024-01-01T12:00:00",
                "version": "1.0.0"
            }
        }


class ErrorResponse(BaseModel):
    """
    Standard error response model.
    """
    error: str = Field(..., description="Error message")
    detail: Optional[str] = Field(default=None, description="Detailed error information")
    timestamp: datetime = Field(default_factory=datetime.now, description="When the error occurred")
    
    class Config:
        json_schema_extra = {
            "example": {
                "error": "ROS2 node not initialized",
                "detail": "The ROS2 node has not been initialized yet",
                "timestamp": "2024-01-01T12:00:00"
            }
        }


class MoveIt2PoseRequest(BaseModel):
    """
    Request model for MoveIt2 pose commands.
    
    Sends target end-effector pose to MoveIt2.
    """
    x: float = Field(..., description="X position in meters", ge=-10.0, le=10.0)
    y: float = Field(..., description="Y position in meters", ge=-10.0, le=10.0)
    z: float = Field(..., description="Z position in meters", ge=-10.0, le=10.0)
    frame_id: str = Field(
        default="base_link",
        description="Reference frame for the pose"
    )
    orientation: Optional[Dict[str, float]] = Field(
        default=None,
        description="Orientation quaternion {x, y, z, w}. Defaults to identity if not provided."
    )
    
    @validator('orientation')
    def validate_orientation(cls, v):
        """Validate quaternion if provided."""
        if v is not None:
            required_keys = {'x', 'y', 'z', 'w'}
            if not all(key in v for key in required_keys):
                raise ValueError('Orientation must contain x, y, z, w keys')
            # Normalize quaternion (basic check)
            x, y, z, w = v['x'], v['y'], v['z'], v['w']
            norm = (x**2 + y**2 + z**2 + w**2) ** 0.5
            if norm < 0.1:  # Very small quaternion
                raise ValueError('Quaternion norm too small, likely invalid')
        return v
    
    class Config:
        json_schema_extra = {
            "example": {
                "x": 0.5,
                "y": 0.0,
                "z": 0.5,
                "frame_id": "base_link",
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
            }
        }


class MoveIt2JointTrajectoryRequest(BaseModel):
    """
    Request model for MoveIt2 joint trajectory commands.
    
    Sends joint positions directly to MoveIt2.
    """
    joint_names: List[str] = Field(
        ...,
        description="List of joint names",
        min_items=1
    )
    positions: List[float] = Field(
        ...,
        description="Joint positions in radians",
        min_items=1
    )
    velocities: Optional[List[float]] = Field(
        default=None,
        description="Joint velocities (optional)"
    )
    accelerations: Optional[List[float]] = Field(
        default=None,
        description="Joint accelerations (optional)"
    )
    time_from_start: float = Field(
        default=2.0,
        description="Time from start in seconds",
        ge=0.1,
        le=60.0
    )
    
    @validator('positions')
    def validate_positions_length(cls, v, values):
        """Validate positions match joint_names length."""
        if 'joint_names' in values and len(v) != len(values['joint_names']):
            raise ValueError('Number of positions must match number of joint names')
        return v
    
    @validator('velocities')
    def validate_velocities_length(cls, v, values):
        """Validate velocities match joint_names length if provided."""
        if v is not None and 'joint_names' in values:
            if len(v) != len(values['joint_names']):
                raise ValueError('Number of velocities must match number of joint names')
        return v
    
    class Config:
        json_schema_extra = {
            "example": {
                "joint_names": ["joint1", "joint2", "joint3"],
                "positions": [0.5, 0.3, 0.1],
                "velocities": [0.0, 0.0, 0.0],
                "time_from_start": 2.0
            }
        }


class MoveIt2Response(BaseModel):
    """
    Response model for MoveIt2 commands.
    """
    status: str = Field(..., description="Status of the MoveIt2 command")
    command_type: str = Field(..., description="Type of command sent (pose or joint_trajectory)")
    success: bool = Field(..., description="Whether the command was published successfully")
    timestamp: datetime = Field(default_factory=datetime.now, description="When the command was sent")
    message: Optional[str] = Field(default=None, description="Additional message or error details")
    topic: Optional[str] = Field(default=None, description="Topic where command was published")
    
    class Config:
        json_schema_extra = {
            "example": {
                "status": "sent",
                "command_type": "pose",
                "success": True,
                "timestamp": "2024-01-01T12:00:00",
                "message": "Pose published successfully to MoveIt2",
                "topic": "/move_group/goal_pose"
            }
        }

