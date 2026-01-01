"""
Shared pytest fixtures for all tests.
"""
import pytest
import sys
from unittest.mock import Mock, MagicMock, patch
from fastapi.testclient import TestClient
from typing import Generator

# Mock ROS2 modules if not available
def setup_ros2_mocks():
    """Setup mock ROS2 modules for testing."""
    # Mock rclpy
    if 'rclpy' not in sys.modules:
        rclpy = MagicMock()
        rclpy.init = Mock()
        rclpy.shutdown = Mock()
        rclpy.node = MagicMock()
        rclpy.node.Node = MagicMock
        rclpy.qos = MagicMock()
        rclpy.qos.QoSProfile = MagicMock
        rclpy.qos.QoSReliabilityPolicy = MagicMock()
        rclpy.executors = MagicMock()
        rclpy.executors.SingleThreadedExecutor = MagicMock
        sys.modules['rclpy'] = rclpy
        sys.modules['rclpy.node'] = rclpy.node
        sys.modules['rclpy.qos'] = rclpy.qos
        sys.modules['rclpy.executors'] = rclpy.executors
    
    # Mock ROS2 message types
    if 'std_msgs' not in sys.modules:
        std_msgs = MagicMock()
        std_msgs.msg = MagicMock()
        std_msgs.msg.String = MagicMock
        std_msgs.msg.Header = MagicMock
        sys.modules['std_msgs'] = std_msgs
        sys.modules['std_msgs.msg'] = std_msgs.msg
    
    if 'geometry_msgs' not in sys.modules:
        geometry_msgs = MagicMock()
        geometry_msgs.msg = MagicMock()
        geometry_msgs.msg.PoseStamped = MagicMock
        geometry_msgs.msg.Pose = MagicMock
        geometry_msgs.msg.Point = MagicMock
        geometry_msgs.msg.Quaternion = MagicMock
        sys.modules['geometry_msgs'] = geometry_msgs
        sys.modules['geometry_msgs.msg'] = geometry_msgs.msg
    
    if 'trajectory_msgs' not in sys.modules:
        trajectory_msgs = MagicMock()
        trajectory_msgs.msg = MagicMock()
        trajectory_msgs.msg.JointTrajectory = MagicMock
        trajectory_msgs.msg.JointTrajectoryPoint = MagicMock
        sys.modules['trajectory_msgs'] = trajectory_msgs
        sys.modules['trajectory_msgs.msg'] = trajectory_msgs.msg
    
    if 'builtin_interfaces' not in sys.modules:
        builtin_interfaces = MagicMock()
        builtin_interfaces.msg = MagicMock()
        builtin_interfaces.msg.Duration = MagicMock
        builtin_interfaces.msg.Time = MagicMock
        sys.modules['builtin_interfaces'] = builtin_interfaces
        sys.modules['builtin_interfaces.msg'] = builtin_interfaces.msg

# Setup mocks before importing app modules
setup_ros2_mocks()

# Now import app modules (they will use mocked ROS2 if needed)
from app.main import app
from app.services.ros2_bridge import ROS2FastapiBridgeNode


@pytest.fixture(scope="function", autouse=True)
def mock_ros2_globally():
    """
    Mock ROS2 initialization globally for all tests.
    This prevents ROS2 from actually initializing during tests.
    """
    with patch('rclpy.init') as mock_init, \
         patch('rclpy.shutdown') as mock_shutdown, \
         patch('rclpy.executors.SingleThreadedExecutor') as mock_executor:
        yield {
            'init': mock_init,
            'shutdown': mock_shutdown,
            'executor': mock_executor
        }


@pytest.fixture
def client():
    """
    Create a test client for the FastAPI app.
    This client bypasses the lifespan context manager for faster tests.
    """
    # Create a test client without lifespan
    from fastapi import FastAPI
    from app.api.v1.api import api_router
    
    test_app = FastAPI(
        title="Test FastAPI ROS2 Bridge",
        description="Test application",
        version="1.0.0"
    )
    test_app.include_router(api_router, prefix="/api/v1")
    
    # Add root endpoint
    @test_app.get("/")
    async def root():
        return {
            "message": "Test FastAPI ROS2 Bridge",
            "status": "running",
            "version": "1.0.0"
        }
    
    return TestClient(test_app)


@pytest.fixture
def client_with_ros2():
    """
    Create a test client with a mocked ROS2 node in app state.
    """
    from fastapi import FastAPI
    from app.api.v1.api import api_router
    
    test_app = FastAPI(
        title="Test FastAPI ROS2 Bridge",
        description="Test application",
        version="1.0.0"
    )
    test_app.include_router(api_router, prefix="/api/v1")
    
    # Add root endpoint
    @test_app.get("/")
    async def root():
        ros2_node = getattr(test_app.state, 'ros2_node', None)
        return {
            "message": "Test FastAPI ROS2 Bridge",
            "status": "running",
            "version": "1.0.0",
            "ros2_node": ros2_node.get_name() if ros2_node else "not initialized"
        }
    
    # Create mock ROS2 node
    mock_ros2_node = create_mock_ros2_node()
    test_app.state.ros2_node = mock_ros2_node
    
    return TestClient(test_app)


@pytest.fixture
def mock_ros2_node():
    """
    Create a mock ROS2 node for testing.
    """
    return create_mock_ros2_node()


def create_mock_ros2_node():
    """
    Helper function to create a mock ROS2 node.
    """
    mock_node = Mock(spec=ROS2FastapiBridgeNode)
    mock_node.get_name.return_value = "test_fastapi_control_node"
    mock_node.is_publisher_ready.return_value = True
    mock_node.send_command.return_value = True
    mock_node.send_moveit_pose.return_value = True
    mock_node.send_moveit_joint_trajectory.return_value = True
    mock_node.is_moveit_ready.return_value = True
    
    # Mock publisher attributes
    mock_publisher = Mock()
    mock_publisher.get_subscription_count.return_value = 1
    mock_publisher.topic_name = "/control/commands"
    mock_node.publisher = mock_publisher
    
    # Mock MoveIt2 publishers
    mock_moveit_pose_publisher = Mock()
    mock_moveit_pose_publisher.get_subscription_count.return_value = 1
    mock_moveit_pose_publisher.topic_name = "/move_group/goal_pose"
    mock_node.moveit_pose_publisher = mock_moveit_pose_publisher
    
    mock_moveit_joint_publisher = Mock()
    mock_moveit_joint_publisher.get_subscription_count.return_value = 1
    mock_moveit_joint_publisher.topic_name = "/joint_trajectory_controller/joint_trajectory"
    mock_node.moveit_joint_trajectory_publisher = mock_moveit_joint_publisher
    
    return mock_node


@pytest.fixture
def mock_ros2_node_no_moveit():
    """
    Create a mock ROS2 node without MoveIt2 publishers.
    """
    mock_node = create_mock_ros2_node()
    mock_node.moveit_pose_publisher = None
    mock_node.moveit_joint_trajectory_publisher = None
    return mock_node


@pytest.fixture
def mock_ros2_node_unavailable():
    """
    Create a mock ROS2 node that is unavailable (None).
    """
    return None

