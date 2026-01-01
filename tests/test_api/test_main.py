"""
Tests for main application endpoints.
"""
import pytest
from fastapi.testclient import TestClient


@pytest.mark.api
class TestMainApp:
    """Test main application endpoints."""
    
    def test_root_endpoint(self, client):
        """Test the root endpoint."""
        response = client.get("/")
        
        assert response.status_code == 200
        data = response.json()
        
        assert "message" in data
        assert "status" in data
        assert data["status"] == "running"
        assert "version" in data
    
    def test_root_endpoint_with_ros2(self, client_with_ros2):
        """Test the root endpoint when ROS2 node is available."""
        response = client_with_ros2.get("/")
        
        assert response.status_code == 200
        data = response.json()
        
        assert "message" in data
        assert "status" in data
        assert "version" in data
        assert "ros2_node" in data
        assert data["ros2_node"] == "test_fastapi_control_node"
    
    def test_root_endpoint_without_ros2(self, client):
        """Test the root endpoint when ROS2 node is not available."""
        response = client.get("/")
        
        assert response.status_code == 200
        data = response.json()
        
        assert "ros2_node" in data
        assert data["ros2_node"] == "not initialized"
    
    def test_api_docs_available(self, client):
        """Test that API documentation is available."""
        response = client.get("/docs")
        # Should return HTML for Swagger UI
        assert response.status_code == 200
        assert "text/html" in response.headers.get("content-type", "")
    
    def test_openapi_schema_available(self, client):
        """Test that OpenAPI schema is available."""
        response = client.get("/openapi.json")
        
        assert response.status_code == 200
        data = response.json()
        
        assert "openapi" in data
        assert "info" in data
        assert "paths" in data

