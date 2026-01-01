"""
Tests for control endpoints.
"""
import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch
import json


@pytest.mark.api
class TestControlEndpoints:
    """Test control command endpoints."""
    
    def test_send_command_success(self, client_with_ros2):
        """Test sending a command successfully."""
        command_data = {
            "command": "move_forward",
            "command_type": "string"
        }
        
        response = client_with_ros2.post("/api/v1/control/command", json=command_data)
        
        assert response.status_code == 200
        data = response.json()
        
        assert data["status"] == "sent"
        assert data["success"] is True
        assert data["command"] == "move_forward"
        assert "message" in data
        assert "timestamp" in data
    
    def test_send_command_with_metadata(self, client_with_ros2):
        """Test sending a command with metadata."""
        command_data = {
            "command": "move_forward",
            "command_type": "string",
            "metadata": {"priority": "high", "timeout": 5}
        }
        
        response = client_with_ros2.post("/api/v1/control/command", json=command_data)
        
        assert response.status_code == 200
        data = response.json()
        assert data["success"] is True
    
    def test_send_command_json_type(self, client_with_ros2):
        """Test sending a JSON command."""
        json_command = json.dumps({"action": "move", "direction": "forward"})
        command_data = {
            "command": json_command,
            "command_type": "json"
        }
        
        response = client_with_ros2.post("/api/v1/control/command", json=command_data)
        
        assert response.status_code == 200
        data = response.json()
        assert data["success"] is True
    
    def test_send_command_invalid_json(self, client_with_ros2):
        """Test sending invalid JSON as command."""
        command_data = {
            "command": "{invalid json}",
            "command_type": "json"
        }
        
        response = client_with_ros2.post("/api/v1/control/command", json=command_data)
        
        assert response.status_code == 400
        data = response.json()
        assert "detail" in data
        assert "JSON" in data["detail"] or "json" in data["detail"].lower()
    
    def test_send_command_empty_string(self, client_with_ros2):
        """Test sending an empty command string."""
        command_data = {
            "command": "",
            "command_type": "string"
        }
        
        response = client_with_ros2.post("/api/v1/control/command", json=command_data)
        
        assert response.status_code == 422  # Validation error
    
    def test_send_command_whitespace_only(self, client_with_ros2):
        """Test sending a whitespace-only command."""
        command_data = {
            "command": "   ",
            "command_type": "string"
        }
        
        response = client_with_ros2.post("/api/v1/control/command", json=command_data)
        
        assert response.status_code == 422  # Validation error
    
    def test_send_command_ros2_node_unavailable(self, client):
        """Test sending command when ROS2 node is not available."""
        command_data = {
            "command": "move_forward",
            "command_type": "string"
        }
        
        response = client.post("/api/v1/control/command", json=command_data)
        
        assert response.status_code == 503
        data = response.json()
        assert "detail" in data
        assert "not initialized" in data["detail"].lower() or "not available" in data["detail"].lower()
    
    def test_send_command_publish_failure(self, client):
        """Test handling when command publishing fails."""
        from fastapi import FastAPI
        from app.api.v1.api import api_router
        from tests.conftest import create_mock_ros2_node
        
        test_app = FastAPI()
        test_app.include_router(api_router, prefix="/api/v1")
        
        mock_node = create_mock_ros2_node()
        mock_node.send_command.return_value = False
        test_app.state.ros2_node = mock_node
        
        test_client = TestClient(test_app)
        command_data = {
            "command": "move_forward",
            "command_type": "string"
        }
        
        response = test_client.post("/api/v1/control/command", json=command_data)
        
        assert response.status_code == 500
        data = response.json()
        assert "detail" in data
    
    def test_send_command_publish_exception(self, client):
        """Test handling when command publishing raises an exception."""
        from fastapi import FastAPI
        from app.api.v1.api import api_router
        from tests.conftest import create_mock_ros2_node
        
        test_app = FastAPI()
        test_app.include_router(api_router, prefix="/api/v1")
        
        mock_node = create_mock_ros2_node()
        mock_node.send_command.side_effect = Exception("Publish error")
        test_app.state.ros2_node = mock_node
        
        test_client = TestClient(test_app)
        command_data = {
            "command": "move_forward",
            "command_type": "string"
        }
        
        response = test_client.post("/api/v1/control/command", json=command_data)
        
        assert response.status_code == 500
        data = response.json()
        assert "detail" in data
    
    def test_send_simple_command_success(self, client_with_ros2):
        """Test sending a simple command string."""
        response = client_with_ros2.post(
            "/api/v1/control/command/simple",
            params={"command": "move_forward"}
        )
        
        assert response.status_code == 200
        data = response.json()
        
        assert data["status"] == "sent"
        assert data["success"] is True
        assert data["command"] == "move_forward"
    
    def test_send_simple_command_empty(self, client_with_ros2):
        """Test sending an empty simple command."""
        response = client_with_ros2.post(
            "/api/v1/control/command/simple",
            params={"command": ""}
        )
        
        assert response.status_code == 422  # Validation error
    
    def test_send_command_sanitization(self, client_with_ros2):
        """Test that commands are properly sanitized."""
        # Command with control characters
        command_data = {
            "command": "move\x00forward\x01test",
            "command_type": "string"
        }
        
        response = client_with_ros2.post("/api/v1/control/command", json=command_data)
        
        assert response.status_code == 200
        data = response.json()
        # Control characters should be removed
        assert "\x00" not in data["command"]
        assert "\x01" not in data["command"]
    
    def test_send_command_long_string(self, client_with_ros2):
        """Test sending a long command string."""
        long_command = "move_forward " * 1000
        command_data = {
            "command": long_command,
            "command_type": "string"
        }
        
        response = client_with_ros2.post("/api/v1/control/command", json=command_data)
        
        # Should succeed but may be truncated
        assert response.status_code in [200, 400]
        if response.status_code == 200:
            data = response.json()
            assert len(data["command"]) <= 10000  # Max length
    
    def test_send_command_raw_type(self, client_with_ros2):
        """Test sending a raw command type."""
        command_data = {
            "command": "raw_command_data",
            "command_type": "raw"
        }
        
        response = client_with_ros2.post("/api/v1/control/command", json=command_data)
        
        assert response.status_code == 200
        data = response.json()
        assert data["success"] is True

