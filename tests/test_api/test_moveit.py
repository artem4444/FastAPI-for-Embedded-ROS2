"""
Tests for MoveIt2 endpoints.
"""
import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch


@pytest.mark.api
class TestMoveIt2Endpoints:
    """Test MoveIt2 command endpoints."""
    
    def test_send_moveit_pose_success(self, client_with_ros2):
        """Test sending a MoveIt2 pose command successfully."""
        pose_data = {
            "x": 0.5,
            "y": 0.0,
            "z": 0.5,
            "frame_id": "base_link"
        }
        
        response = client_with_ros2.post("/api/v1/moveit/pose", json=pose_data)
        
        assert response.status_code == 200
        data = response.json()
        
        assert data["status"] == "sent"
        assert data["command_type"] == "pose"
        assert data["success"] is True
        assert "message" in data
        assert "topic" in data
        assert "timestamp" in data
    
    def test_send_moveit_pose_with_orientation(self, client_with_ros2):
        """Test sending a MoveIt2 pose with orientation."""
        pose_data = {
            "x": 0.5,
            "y": 0.0,
            "z": 0.5,
            "frame_id": "base_link",
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        }
        
        response = client_with_ros2.post("/api/v1/moveit/pose", json=pose_data)
        
        assert response.status_code == 200
        data = response.json()
        assert data["success"] is True
    
    def test_send_moveit_pose_invalid_position(self, client_with_ros2):
        """Test sending a pose with invalid position values."""
        pose_data = {
            "x": 15.0,  # Exceeds max (10.0)
            "y": 0.0,
            "z": 0.5,
            "frame_id": "base_link"
        }
        
        response = client_with_ros2.post("/api/v1/moveit/pose", json=pose_data)
        
        assert response.status_code == 422  # Validation error
    
    def test_send_moveit_pose_invalid_orientation(self, client_with_ros2):
        """Test sending a pose with invalid orientation."""
        pose_data = {
            "x": 0.5,
            "y": 0.0,
            "z": 0.5,
            "frame_id": "base_link",
            "orientation": {"x": 0.0, "y": 0.0}  # Missing z and w
        }
        
        response = client_with_ros2.post("/api/v1/moveit/pose", json=pose_data)
        
        assert response.status_code == 422  # Validation error
    
    def test_send_moveit_pose_ros2_node_unavailable(self, client):
        """Test sending pose when ROS2 node is not available."""
        pose_data = {
            "x": 0.5,
            "y": 0.0,
            "z": 0.5,
            "frame_id": "base_link"
        }
        
        response = client.post("/api/v1/moveit/pose", json=pose_data)
        
        assert response.status_code == 503
        data = response.json()
        assert "detail" in data
        assert "not initialized" in data["detail"].lower() or "not available" in data["detail"].lower()
    
    def test_send_moveit_pose_moveit_unavailable(self, client):
        """Test sending pose when MoveIt2 publisher is not available."""
        from fastapi import FastAPI
        from app.api.v1.api import api_router
        from tests.conftest import create_mock_ros2_node
        
        test_app = FastAPI()
        test_app.include_router(api_router, prefix="/api/v1")
        
        mock_node = create_mock_ros2_node()
        mock_node.moveit_pose_publisher = None
        test_app.state.ros2_node = mock_node
        
        test_client = TestClient(test_app)
        pose_data = {
            "x": 0.5,
            "y": 0.0,
            "z": 0.5,
            "frame_id": "base_link"
        }
        
        response = test_client.post("/api/v1/moveit/pose", json=pose_data)
        
        assert response.status_code == 503
        data = response.json()
        assert "detail" in data
        assert "not available" in data["detail"].lower()
    
    def test_send_moveit_pose_publish_failure(self, client):
        """Test handling when pose publishing fails."""
        from fastapi import FastAPI
        from app.api.v1.api import api_router
        from tests.conftest import create_mock_ros2_node
        
        test_app = FastAPI()
        test_app.include_router(api_router, prefix="/api/v1")
        
        mock_node = create_mock_ros2_node()
        mock_node.send_moveit_pose.return_value = False
        test_app.state.ros2_node = mock_node
        
        test_client = TestClient(test_app)
        pose_data = {
            "x": 0.5,
            "y": 0.0,
            "z": 0.5,
            "frame_id": "base_link"
        }
        
        response = test_client.post("/api/v1/moveit/pose", json=pose_data)
        
        assert response.status_code == 500
        data = response.json()
        assert "detail" in data
    
    def test_send_moveit_joint_trajectory_success(self, client_with_ros2):
        """Test sending a MoveIt2 joint trajectory command successfully."""
        trajectory_data = {
            "joint_names": ["joint1", "joint2", "joint3"],
            "positions": [0.5, 0.3, 0.1],
            "time_from_start": 2.0
        }
        
        response = client_with_ros2.post("/api/v1/moveit/joint_trajectory", json=trajectory_data)
        
        assert response.status_code == 200
        data = response.json()
        
        assert data["status"] == "sent"
        assert data["command_type"] == "joint_trajectory"
        assert data["success"] is True
        assert "message" in data
        assert "topic" in data
        assert "timestamp" in data
    
    def test_send_moveit_joint_trajectory_with_velocities(self, client_with_ros2):
        """Test sending a joint trajectory with velocities."""
        trajectory_data = {
            "joint_names": ["joint1", "joint2"],
            "positions": [0.5, 0.3],
            "velocities": [0.1, 0.1],
            "time_from_start": 2.0
        }
        
        response = client_with_ros2.post("/api/v1/moveit/joint_trajectory", json=trajectory_data)
        
        assert response.status_code == 200
        data = response.json()
        assert data["success"] is True
    
    def test_send_moveit_joint_trajectory_with_accelerations(self, client_with_ros2):
        """Test sending a joint trajectory with accelerations."""
        trajectory_data = {
            "joint_names": ["joint1", "joint2"],
            "positions": [0.5, 0.3],
            "velocities": [0.1, 0.1],
            "accelerations": [0.05, 0.05],
            "time_from_start": 2.0
        }
        
        response = client_with_ros2.post("/api/v1/moveit/joint_trajectory", json=trajectory_data)
        
        assert response.status_code == 200
        data = response.json()
        assert data["success"] is True
    
    def test_send_moveit_joint_trajectory_empty_joints(self, client_with_ros2):
        """Test sending a joint trajectory with empty joint names."""
        trajectory_data = {
            "joint_names": [],
            "positions": [],
            "time_from_start": 2.0
        }
        
        response = client_with_ros2.post("/api/v1/moveit/joint_trajectory", json=trajectory_data)
        
        assert response.status_code == 422  # Validation error
    
    def test_send_moveit_joint_trajectory_mismatched_lengths(self, client_with_ros2):
        """Test sending a joint trajectory with mismatched array lengths."""
        trajectory_data = {
            "joint_names": ["joint1", "joint2"],
            "positions": [0.5],  # Mismatched length
            "time_from_start": 2.0
        }
        
        response = client_with_ros2.post("/api/v1/moveit/joint_trajectory", json=trajectory_data)
        
        assert response.status_code == 422  # Validation error
    
    def test_send_moveit_joint_trajectory_invalid_time(self, client_with_ros2):
        """Test sending a joint trajectory with invalid time_from_start."""
        trajectory_data = {
            "joint_names": ["joint1"],
            "positions": [0.5],
            "time_from_start": 100.0  # Exceeds max (60.0)
        }
        
        response = client_with_ros2.post("/api/v1/moveit/joint_trajectory", json=trajectory_data)
        
        assert response.status_code == 422  # Validation error
    
    def test_send_moveit_joint_trajectory_ros2_node_unavailable(self, client):
        """Test sending joint trajectory when ROS2 node is not available."""
        trajectory_data = {
            "joint_names": ["joint1"],
            "positions": [0.5],
            "time_from_start": 2.0
        }
        
        response = client.post("/api/v1/moveit/joint_trajectory", json=trajectory_data)
        
        assert response.status_code == 503
        data = response.json()
        assert "detail" in data
    
    def test_send_moveit_joint_trajectory_moveit_unavailable(self, client):
        """Test sending joint trajectory when MoveIt2 publisher is not available."""
        from fastapi import FastAPI
        from app.api.v1.api import api_router
        from tests.conftest import create_mock_ros2_node
        
        test_app = FastAPI()
        test_app.include_router(api_router, prefix="/api/v1")
        
        mock_node = create_mock_ros2_node()
        mock_node.moveit_joint_trajectory_publisher = None
        test_app.state.ros2_node = mock_node
        
        test_client = TestClient(test_app)
        trajectory_data = {
            "joint_names": ["joint1"],
            "positions": [0.5],
            "time_from_start": 2.0
        }
        
        response = test_client.post("/api/v1/moveit/joint_trajectory", json=trajectory_data)
        
        assert response.status_code == 503
        data = response.json()
        assert "detail" in data
    
    def test_send_moveit_joint_trajectory_publish_failure(self, client):
        """Test handling when joint trajectory publishing fails."""
        from fastapi import FastAPI
        from app.api.v1.api import api_router
        from tests.conftest import create_mock_ros2_node
        
        test_app = FastAPI()
        test_app.include_router(api_router, prefix="/api/v1")
        
        mock_node = create_mock_ros2_node()
        mock_node.send_moveit_joint_trajectory.return_value = False
        test_app.state.ros2_node = mock_node
        
        test_client = TestClient(test_app)
        trajectory_data = {
            "joint_names": ["joint1"],
            "positions": [0.5],
            "time_from_start": 2.0
        }
        
        response = test_client.post("/api/v1/moveit/joint_trajectory", json=trajectory_data)
        
        assert response.status_code == 500
        data = response.json()
        assert "detail" in data
    
    def test_send_moveit_joint_trajectory_publish_exception(self, client):
        """Test handling when joint trajectory publishing raises an exception."""
        from fastapi import FastAPI
        from app.api.v1.api import api_router
        from tests.conftest import create_mock_ros2_node
        
        test_app = FastAPI()
        test_app.include_router(api_router, prefix="/api/v1")
        
        mock_node = create_mock_ros2_node()
        mock_node.send_moveit_joint_trajectory.side_effect = Exception("Publish error")
        test_app.state.ros2_node = mock_node
        
        test_client = TestClient(test_app)
        trajectory_data = {
            "joint_names": ["joint1"],
            "positions": [0.5],
            "time_from_start": 2.0
        }
        
        response = test_client.post("/api/v1/moveit/joint_trajectory", json=trajectory_data)
        
        assert response.status_code == 500
        data = response.json()
        assert "detail" in data

