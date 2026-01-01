"""
Tests for health check endpoints.
"""
import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch
from datetime import datetime


@pytest.mark.api
class TestHealthEndpoints:
    """Test health check endpoints."""
    
    def test_health_check_with_ros2_node(self, client_with_ros2):
        """Test health check endpoint when ROS2 node is available."""
        response = client_with_ros2.get("/api/v1/health")
        
        assert response.status_code == 200
        data = response.json()
        
        assert "status" in data
        assert "fastapi" in data
        assert "ros2" in data
        assert "timestamp" in data
        assert "version" in data
        
        # Check FastAPI status
        assert data["fastapi"]["running"] is True
        assert "uptime_seconds" in data["fastapi"]
        assert isinstance(data["fastapi"]["uptime_seconds"], int)
        
        # Check ROS2 status
        assert data["ros2"]["connected"] is True
        assert data["ros2"]["node_name"] == "test_fastapi_control_node"
        assert data["ros2"]["publisher_ready"] is True
    
    def test_health_check_without_ros2_node(self, client):
        """Test health check endpoint when ROS2 node is not available."""
        response = client.get("/api/v1/health")
        
        assert response.status_code == 200
        data = response.json()
        
        assert "status" in data
        assert data["status"] in ["unhealthy", "degraded"]
        assert data["ros2"]["connected"] is False
        assert data["ros2"]["node_name"] is None
        assert data["ros2"]["publisher_ready"] is False
    
    def test_health_check_ros2_node_not_ready(self, client):
        """Test health check when ROS2 node exists but publisher is not ready."""
        from fastapi import FastAPI
        from app.api.v1.api import api_router
        from tests.conftest import create_mock_ros2_node
        
        test_app = FastAPI()
        test_app.include_router(api_router, prefix="/api/v1")
        
        mock_node = create_mock_ros2_node()
        mock_node.is_publisher_ready.return_value = False
        test_app.state.ros2_node = mock_node
        
        test_client = TestClient(test_app)
        response = test_client.get("/api/v1/health")
        
        assert response.status_code == 200
        data = response.json()
        assert data["ros2"]["connected"] is True
        assert data["ros2"]["publisher_ready"] is False
        assert data["status"] in ["degraded", "unhealthy"]
    
    def test_readiness_check_ready(self, client_with_ros2):
        """Test readiness check when service is ready."""
        response = client_with_ros2.get("/api/v1/health/ready")
        
        assert response.status_code == 200
        data = response.json()
        
        assert "ready" in data
        assert data["ready"] is True
        assert "timestamp" in data
    
    def test_readiness_check_not_ready(self, client):
        """Test readiness check when service is not ready."""
        response = client.get("/api/v1/health/ready")
        
        assert response.status_code == 200
        data = response.json()
        
        assert "ready" in data
        assert data["ready"] is False
        assert "timestamp" in data
    
    def test_readiness_check_publisher_not_ready(self, client):
        """Test readiness check when publisher is not ready."""
        from fastapi import FastAPI
        from app.api.v1.api import api_router
        from tests.conftest import create_mock_ros2_node
        
        test_app = FastAPI()
        test_app.include_router(api_router, prefix="/api/v1")
        
        mock_node = create_mock_ros2_node()
        mock_node.is_publisher_ready.return_value = False
        test_app.state.ros2_node = mock_node
        
        test_client = TestClient(test_app)
        response = test_client.get("/api/v1/health/ready")
        
        assert response.status_code == 200
        data = response.json()
        assert data["ready"] is False
    
    def test_liveness_check(self, client):
        """Test liveness check endpoint."""
        response = client.get("/api/v1/health/live")
        
        assert response.status_code == 200
        data = response.json()
        
        assert "alive" in data
        assert data["alive"] is True
        assert "timestamp" in data
    
    def test_health_check_response_structure(self, client_with_ros2):
        """Test that health check response has correct structure."""
        response = client_with_ros2.get("/api/v1/health")
        
        assert response.status_code == 200
        data = response.json()
        
        # Validate all required fields
        required_fields = ["status", "fastapi", "ros2", "timestamp", "version"]
        for field in required_fields:
            assert field in data, f"Missing required field: {field}"
        
        # Validate ROS2 status structure
        ros2_fields = ["connected", "node_name", "topic_name", "publisher_ready", "subscribers_count"]
        for field in ros2_fields:
            assert field in data["ros2"], f"Missing ROS2 field: {field}"
        
        # Validate FastAPI status structure
        fastapi_fields = ["running", "uptime_seconds", "environment", "debug"]
        for field in fastapi_fields:
            assert field in data["fastapi"], f"Missing FastAPI field: {field}"

