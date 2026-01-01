"""
Tests for Pydantic schema models.
"""
import pytest
from pydantic import ValidationError
from datetime import datetime
from app.models.schemas import (
    CommandType,
    CommandRequest,
    CommandResponse,
    HealthStatus,
    ROS2Status,
    HealthResponse,
    ErrorResponse,
    MoveIt2PoseRequest,
    MoveIt2JointTrajectoryRequest,
    MoveIt2Response
)


@pytest.mark.utils
class TestCommandSchemas:
    """Test command-related schemas."""
    
    def test_command_request_valid(self):
        """Test creating a valid CommandRequest."""
        request = CommandRequest(command="move_forward")
        assert request.command == "move_forward"
        assert request.command_type == CommandType.STRING
        assert request.metadata is None
    
    def test_command_request_with_type(self):
        """Test creating CommandRequest with command type."""
        request = CommandRequest(
            command="move_forward",
            command_type=CommandType.JSON
        )
        assert request.command_type == CommandType.JSON
    
    def test_command_request_with_metadata(self):
        """Test creating CommandRequest with metadata."""
        metadata = {"priority": "high", "timeout": 5}
        request = CommandRequest(
            command="move_forward",
            metadata=metadata
        )
        assert request.metadata == metadata
    
    def test_command_request_empty_command(self):
        """Test CommandRequest with empty command."""
        with pytest.raises(ValidationError):
            CommandRequest(command="")
    
    def test_command_request_whitespace_only(self):
        """Test CommandRequest with whitespace-only command."""
        with pytest.raises(ValidationError):
            CommandRequest(command="   ")
    
    def test_command_request_strips_whitespace(self):
        """Test that CommandRequest strips whitespace."""
        request = CommandRequest(command="  move_forward  ")
        assert request.command == "move_forward"
    
    def test_command_request_too_long(self):
        """Test CommandRequest with command that's too long."""
        long_command = "a" * 10001
        with pytest.raises(ValidationError):
            CommandRequest(command=long_command)
    
    def test_command_response_valid(self):
        """Test creating a valid CommandResponse."""
        response = CommandResponse(
            status="sent",
            command="move_forward",
            success=True,
            message="Command sent"
        )
        assert response.status == "sent"
        assert response.command == "move_forward"
        assert response.success is True
        assert response.message == "Command sent"
        assert isinstance(response.timestamp, datetime)
    
    def test_command_response_default_timestamp(self):
        """Test CommandResponse with default timestamp."""
        response = CommandResponse(
            status="sent",
            command="move_forward",
            success=True
        )
        assert isinstance(response.timestamp, datetime)


@pytest.mark.utils
class TestHealthSchemas:
    """Test health-related schemas."""
    
    def test_health_status_enum(self):
        """Test HealthStatus enum values."""
        assert HealthStatus.HEALTHY == "healthy"
        assert HealthStatus.UNHEALTHY == "unhealthy"
        assert HealthStatus.DEGRADED == "degraded"
    
    def test_ros2_status_valid(self):
        """Test creating a valid ROS2Status."""
        status = ROS2Status(
            connected=True,
            node_name="test_node",
            topic_name="/test/topic",
            publisher_ready=True,
            subscribers_count=1
        )
        assert status.connected is True
        assert status.node_name == "test_node"
        assert status.topic_name == "/test/topic"
        assert status.publisher_ready is True
        assert status.subscribers_count == 1
    
    def test_ros2_status_defaults(self):
        """Test ROS2Status with default values."""
        status = ROS2Status(connected=False)
        assert status.connected is False
        assert status.node_name is None
        assert status.publisher_ready is False
        assert status.subscribers_count == 0
    
    def test_health_response_valid(self):
        """Test creating a valid HealthResponse."""
        fastapi_status = {
            "running": True,
            "uptime_seconds": 3600,
            "environment": "production",
            "debug": False
        }
        ros2_status = ROS2Status(
            connected=True,
            node_name="test_node",
            topic_name="/test/topic",
            publisher_ready=True,
            subscribers_count=1
        )
        
        response = HealthResponse(
            status=HealthStatus.HEALTHY,
            fastapi=fastapi_status,
            ros2=ros2_status,
            version="1.0.0"
        )
        
        assert response.status == HealthStatus.HEALTHY
        assert response.fastapi == fastapi_status
        assert response.ros2 == ros2_status
        assert response.version == "1.0.0"
        assert isinstance(response.timestamp, datetime)
    
    def test_error_response_valid(self):
        """Test creating a valid ErrorResponse."""
        response = ErrorResponse(
            error="Test error",
            detail="Detailed error message"
        )
        assert response.error == "Test error"
        assert response.detail == "Detailed error message"
        assert isinstance(response.timestamp, datetime)
    
    def test_error_response_minimal(self):
        """Test creating ErrorResponse with minimal fields."""
        response = ErrorResponse(error="Test error")
        assert response.error == "Test error"
        assert response.detail is None


@pytest.mark.utils
class TestMoveIt2Schemas:
    """Test MoveIt2-related schemas."""
    
    def test_moveit2_pose_request_valid(self):
        """Test creating a valid MoveIt2PoseRequest."""
        request = MoveIt2PoseRequest(x=0.5, y=0.0, z=0.5)
        assert request.x == 0.5
        assert request.y == 0.0
        assert request.z == 0.5
        assert request.frame_id == "base_link"  # Default
    
    def test_moveit2_pose_request_with_orientation(self):
        """Test MoveIt2PoseRequest with orientation."""
        orientation = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        request = MoveIt2PoseRequest(
            x=0.5,
            y=0.0,
            z=0.5,
            orientation=orientation
        )
        assert request.orientation == orientation
    
    def test_moveit2_pose_request_invalid_position(self):
        """Test MoveIt2PoseRequest with invalid position."""
        with pytest.raises(ValidationError):
            MoveIt2PoseRequest(x=15.0, y=0.0, z=0.5)  # x exceeds max
    
    def test_moveit2_pose_request_invalid_orientation_missing_keys(self):
        """Test MoveIt2PoseRequest with invalid orientation (missing keys)."""
        with pytest.raises(ValidationError):
            MoveIt2PoseRequest(
                x=0.5,
                y=0.0,
                z=0.5,
                orientation={"x": 0.0, "y": 0.0}  # Missing z and w
            )
    
    def test_moveit2_pose_request_invalid_orientation_norm(self):
        """Test MoveIt2PoseRequest with invalid quaternion norm."""
        with pytest.raises(ValidationError):
            MoveIt2PoseRequest(
                x=0.5,
                y=0.0,
                z=0.5,
                orientation={"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}  # Zero norm
            )
    
    def test_moveit2_joint_trajectory_request_valid(self):
        """Test creating a valid MoveIt2JointTrajectoryRequest."""
        request = MoveIt2JointTrajectoryRequest(
            joint_names=["joint1", "joint2"],
            positions=[0.5, 0.3]
        )
        assert request.joint_names == ["joint1", "joint2"]
        assert request.positions == [0.5, 0.3]
        assert request.time_from_start == 2.0  # Default
    
    def test_moveit2_joint_trajectory_request_with_velocities(self):
        """Test MoveIt2JointTrajectoryRequest with velocities."""
        request = MoveIt2JointTrajectoryRequest(
            joint_names=["joint1", "joint2"],
            positions=[0.5, 0.3],
            velocities=[0.1, 0.1]
        )
        assert request.velocities == [0.1, 0.1]
    
    def test_moveit2_joint_trajectory_request_with_accelerations(self):
        """Test MoveIt2JointTrajectoryRequest with accelerations."""
        request = MoveIt2JointTrajectoryRequest(
            joint_names=["joint1", "joint2"],
            positions=[0.5, 0.3],
            accelerations=[0.05, 0.05]
        )
        assert request.accelerations == [0.05, 0.05]
    
    def test_moveit2_joint_trajectory_request_empty_joints(self):
        """Test MoveIt2JointTrajectoryRequest with empty joint names."""
        with pytest.raises(ValidationError):
            MoveIt2JointTrajectoryRequest(
                joint_names=[],
                positions=[]
            )
    
    def test_moveit2_joint_trajectory_request_mismatched_lengths(self):
        """Test MoveIt2JointTrajectoryRequest with mismatched array lengths."""
        with pytest.raises(ValidationError):
            MoveIt2JointTrajectoryRequest(
                joint_names=["joint1", "joint2"],
                positions=[0.5]  # Mismatched length
            )
    
    def test_moveit2_joint_trajectory_request_mismatched_velocities(self):
        """Test MoveIt2JointTrajectoryRequest with mismatched velocities length."""
        with pytest.raises(ValidationError):
            MoveIt2JointTrajectoryRequest(
                joint_names=["joint1", "joint2"],
                positions=[0.5, 0.3],
                velocities=[0.1]  # Mismatched length
            )
    
    def test_moveit2_joint_trajectory_request_invalid_time(self):
        """Test MoveIt2JointTrajectoryRequest with invalid time_from_start."""
        with pytest.raises(ValidationError):
            MoveIt2JointTrajectoryRequest(
                joint_names=["joint1"],
                positions=[0.5],
                time_from_start=100.0  # Exceeds max
            )
    
    def test_moveit2_response_valid(self):
        """Test creating a valid MoveIt2Response."""
        response = MoveIt2Response(
            status="sent",
            command_type="pose",
            success=True,
            message="Pose sent",
            topic="/move_group/goal_pose"
        )
        assert response.status == "sent"
        assert response.command_type == "pose"
        assert response.success is True
        assert response.message == "Pose sent"
        assert response.topic == "/move_group/goal_pose"
        assert isinstance(response.timestamp, datetime)
    
    def test_moveit2_response_minimal(self):
        """Test creating MoveIt2Response with minimal fields."""
        response = MoveIt2Response(
            status="sent",
            command_type="pose",
            success=True
        )
        assert response.message is None
        assert response.topic is None

