"""
Health check endpoints - Reports current status of FastAPI and ROS2 Node states.
"""
from fastapi import APIRouter, Request
from datetime import datetime
import logging
import time

from app.models.schemas import HealthResponse, HealthStatus, ROS2Status
from app.config import settings

logger = logging.getLogger(__name__)

router = APIRouter()

# Track application start time for uptime calculation
_app_start_time = time.time()


@router.get(
    "",
    response_model=HealthResponse,
    summary="Health check",
    description="Reports current status of FastAPI and ROS2 Node states when called"
)
async def health_check(http_request: Request) -> HealthResponse:
    """
    Health check endpoint.
    
    Reports:
    - FastAPI application status (running, uptime)
    - ROS2 node status (connected, node name, topic, publisher status)
    - Overall health status
    
    Args:
        http_request: FastAPI request object to access app state
        
    Returns:
        HealthResponse: Health status information
    """
    # Get ROS2 node from app state
    ros2_node = getattr(http_request.app.state, 'ros2_node', None)
    
    # Calculate FastAPI uptime
    uptime_seconds = int(time.time() - _app_start_time)
    
    # FastAPI status
    fastapi_status = {
        "running": True,
        "uptime_seconds": uptime_seconds,
        "environment": settings.environment,
        "debug": settings.debug
    }
    
    # ROS2 status
    ros2_connected = ros2_node is not None
    
    if ros2_connected:
        try:
            node_name = ros2_node.get_name()
            publisher_ready = ros2_node.is_publisher_ready()
            
            # Try to get subscriber count (may not be available in all ROS2 versions)
            try:
                subscribers_count = ros2_node.publisher.get_subscription_count()
            except AttributeError:
                subscribers_count = 0
            
            ros2_status = ROS2Status(
                connected=True,
                node_name=node_name,
                topic_name=settings.ros2_topic_name,
                publisher_ready=publisher_ready,
                subscribers_count=subscribers_count
            )
        except Exception as e:
            logger.error(f"Error getting ROS2 status: {e}", exc_info=True)
            ros2_status = ROS2Status(
                connected=True,
                node_name=None,
                topic_name=settings.ros2_topic_name,
                publisher_ready=False,
                subscribers_count=0
            )
    else:
        ros2_status = ROS2Status(
            connected=False,
            node_name=None,
            topic_name=settings.ros2_topic_name,
            publisher_ready=False,
            subscribers_count=0
        )
    
    # Determine overall health status
    if ros2_connected and ros2_status.publisher_ready:
        overall_status = HealthStatus.HEALTHY
    elif ros2_connected:
        overall_status = HealthStatus.DEGRADED
    else:
        overall_status = HealthStatus.UNHEALTHY
    
    return HealthResponse(
        status=overall_status,
        fastapi=fastapi_status,
        ros2=ros2_status,
        timestamp=datetime.now(),
        version=settings.app_version
    )


@router.get(
    "/ready",
    summary="Readiness check",
    description="Check if the service is ready to accept requests"
)
async def readiness_check(http_request: Request) -> dict:
    """
    Readiness check - verifies ROS2 node is initialized and ready.
    
    Args:
        http_request: FastAPI request object
        
    Returns:
        dict: Ready status
    """
    ros2_node = getattr(http_request.app.state, 'ros2_node', None)
    
    ready = (
        ros2_node is not None and
        ros2_node.is_publisher_ready()
    )
    
    return {
        "ready": ready,
        "timestamp": datetime.now().isoformat()
    }


@router.get(
    "/live",
    summary="Liveness check",
    description="Check if the service is alive"
)
async def liveness_check() -> dict:
    """
    Liveness check - verifies the service is running.
    
    Returns:
        dict: Alive status
    """
    return {
        "alive": True,
        "timestamp": datetime.now().isoformat()
    }

