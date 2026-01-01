import rclpy
from fastapi import FastAPI
from contextlib import asynccontextmanager
import threading
import logging

from app.services.ros2_bridge import ROS2FastapiBridgeNode
from app.config import settings
from app.api.v1.api import api_router

# Configure logging
logging.basicConfig(
    level=logging.DEBUG if settings.debug else logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Global reference to ROS2 node (will be initialized in lifespan)
ros2_node = None
ros2_executor_thread = None


# MUST be async function
# MUST have yield statement
# Code before yield = startup
# Code after yield = shutdown
@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Manages application lifecycle: startup and shutdown
    This runs when FastAPI starts and stops
    """
    global ros2_node, ros2_executor_thread
    
    # Startup: Initialize ROS2
    logger.info("Initializing ROS2...")
    rclpy.init()
    
    # Create ROS2 node with config settings
    ros2_node = ROS2FastapiBridgeNode(
        node_name=settings.ros2_node_name,
        topic_name=settings.ros2_topic_name,
        queue_size=settings.ros2_queue_size
    )
    logger.info(f"ROS2 node created: {settings.ros2_node_name}")
    
    # Start ROS2 executor in a separate thread
    # This allows ROS2 callbacks to run while FastAPI handles HTTP requests
    def spin_ros2():
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(ros2_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
    
    ros2_executor_thread = threading.Thread(target=spin_ros2, daemon=True)
    ros2_executor_thread.start()
    logger.info("ROS2 executor thread started")
    
    # Store ROS2 node in app state so endpoints can access it
    app.state.ros2_node = ros2_node
    
    yield  # Application runs here
    
    # Shutdown: Clean up ROS2
    logger.info("Shutting down ROS2...")
    if ros2_node is not None:
        ros2_node.destroy_node()
    rclpy.shutdown()
    logger.info("ROS2 shutdown complete")


# Create FastAPI app with lifespan context manager
app = FastAPI(
    title=settings.app_title,
    description=settings.app_description,
    version=settings.app_version,
    lifespan=lifespan
)

# Include API routers
app.include_router(api_router, prefix="/api/v1")


# Root endpoint
@app.get("/")
async def root():
    """
    Root endpoint - API information.
    """
    return {
        "message": settings.app_title,
        "status": "running",
        "version": settings.app_version,
        "ros2_node": ros2_node.get_name() if ros2_node else "not initialized",
        "api_docs": "/docs",
        "health_check": "/api/v1/health"
    }


# Run with: uvicorn app.main:app --reload
# Or: python -m uvicorn app.main:app --reload
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        app,
        host=settings.host,
        port=settings.port,
        reload=settings.reload
    )