from pydantic_settings import BaseSettings
from pydantic import Field
from typing import Optional


class Settings(BaseSettings):
    """
    Application configuration settings.
    Can be overridden via environment variables or .env file.
    """
    
    # FastAPI Settings
    app_title: str = Field(default="FastAPI ROS2 Bridge", description="FastAPI app title")
    app_description: str = Field(
        default="Bridge for receiving control commands via HTTP and publishing to ROS2",
        description="FastAPI app description"
    )
    app_version: str = Field(default="1.0.0", description="Application version")
    
    # Server Settings
    host: str = Field(default="0.0.0.0", description="Server host address")
    port: int = Field(default=8000, description="Server port")
    reload: bool = Field(default=False, description="Enable auto-reload for development")
    
    # ROS2 Settings
    ros2_node_name: str = Field(
        default="fastapi_control_node",
        description="Name of the ROS2 node"
    )
    ros2_topic_name: str = Field(
        default="/control/commands",
        description="ROS2 topic name for publishing commands"
    )
    ros2_queue_size: int = Field(
        default=10,
        description="ROS2 publisher queue size"
    )
    
    # Optional: ROS2 Domain ID (for network isolation)
    ros2_domain_id: Optional[int] = Field(
        default=None,
        description="ROS2 domain ID (None = use default)"
    )
    
    # MoveIt2 Settings
    moveit_pose_topic: str = Field(
        default="/move_group/goal_pose",
        description="MoveIt2 pose topic name"
    )
    moveit_planning_group: str = Field(
        default="arm_group",
        description="MoveIt2 planning group name"
    )
    moveit_joint_trajectory_topic: str = Field(
        default="/joint_trajectory_controller/joint_trajectory",
        description="MoveIt2 joint trajectory topic name"
    )
    moveit_default_frame_id: str = Field(
        default="base_link",
        description="Default reference frame for MoveIt2 poses"
    )
    
    # Environment
    environment: str = Field(
        default="development",
        description="Environment: development, production, testing"
    )
    debug: bool = Field(
        default=False,
        description="Enable debug mode"
    )
    
    class Config:
        env_file = ".env"  # Load from .env file
        env_file_encoding = "utf-8"
        case_sensitive = False  # Allow lowercase env vars


# Create global settings instance
settings = Settings()