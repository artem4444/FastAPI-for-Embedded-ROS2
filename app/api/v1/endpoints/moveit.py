"""
MoveIt2 endpoints - Convert HTTP requests to MoveIt2 message format.
"""
from fastapi import APIRouter, Request, HTTPException, status
import logging

from app.models.schemas import (
    MoveIt2PoseRequest,
    MoveIt2JointTrajectoryRequest,
    MoveIt2Response,
    ErrorResponse
)

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post(
    "/pose",
    response_model=MoveIt2Response,
    status_code=status.HTTP_200_OK,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid pose data"},
        500: {"model": ErrorResponse, "description": "MoveIt2 publishing error"},
        503: {"model": ErrorResponse, "description": "ROS2 node or MoveIt2 not available"}
    },
    summary="Send pose command to MoveIt2",
    description="Converts HTTP pose data into MoveIt2 PoseStamped message format and publishes to MoveIt2 topic"
)
async def send_moveit_pose(
    request: MoveIt2PoseRequest,
    http_request: Request
) -> MoveIt2Response:
    """
    Send a target pose to MoveIt2.
    
    This endpoint:
    1. Receives HTTP request with pose data (x, y, z, orientation)
    2. Validates the pose data
    3. Converts it to MoveIt2 PoseStamped message format
    4. Publishes to MoveIt2 topic that MoveIt2 listens to
    
    Args:
        request: MoveIt2 pose request with position and orientation
        http_request: FastAPI request object to access app state
        
    Returns:
        MoveIt2Response: Response with command status
    """
    # Get ROS2 node from app state
    ros2_node = getattr(http_request.app.state, 'ros2_node', None)
    
    if ros2_node is None:
        logger.error("ROS2 node not initialized")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="ROS2 node is not initialized. Please check the service status."
        )
    
    # Check if MoveIt2 publisher is available
    if not hasattr(ros2_node, 'moveit_pose_publisher') or ros2_node.moveit_pose_publisher is None:
        logger.error("MoveIt2 pose publisher not available")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="MoveIt2 pose publisher is not available. Ensure MoveIt2 packages are installed."
        )
    
    # Prepare pose data
    pose_data = {
        'x': request.x,
        'y': request.y,
        'z': request.z,
        'frame_id': request.frame_id
    }
    
    # Add orientation if provided
    if request.orientation:
        pose_data['orientation'] = request.orientation
    
    # Publish to MoveIt2
    try:
        success = ros2_node.send_moveit_pose(pose_data)
        
        if success:
            logger.info(
                f"MoveIt2 pose published successfully: "
                f"x={request.x:.3f}, y={request.y:.3f}, z={request.z:.3f}, "
                f"frame={request.frame_id}"
            )
            return MoveIt2Response(
                status="sent",
                command_type="pose",
                success=True,
                message=f"Pose published successfully to MoveIt2 topic",
                topic=ros2_node.moveit_pose_publisher.topic_name if hasattr(ros2_node.moveit_pose_publisher, 'topic_name') else "/move_group/goal_pose"
            )
        else:
            logger.error(f"Failed to publish MoveIt2 pose")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to publish pose to MoveIt2 topic"
            )
            
    except Exception as e:
        logger.error(f"Error publishing MoveIt2 pose: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error publishing pose to MoveIt2: {str(e)}"
        )


@router.post(
    "/joint_trajectory",
    response_model=MoveIt2Response,
    status_code=status.HTTP_200_OK,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid trajectory data"},
        500: {"model": ErrorResponse, "description": "MoveIt2 publishing error"},
        503: {"model": ErrorResponse, "description": "ROS2 node or MoveIt2 not available"}
    },
    summary="Send joint trajectory command to MoveIt2",
    description="Converts HTTP joint trajectory data into MoveIt2 JointTrajectory message format and publishes to MoveIt2 topic"
)
async def send_moveit_joint_trajectory(
    request: MoveIt2JointTrajectoryRequest,
    http_request: Request
) -> MoveIt2Response:
    """
    Send a joint trajectory to MoveIt2.
    
    This endpoint:
    1. Receives HTTP request with joint trajectory data
    2. Validates the trajectory data
    3. Converts it to MoveIt2 JointTrajectory message format
    4. Publishes to MoveIt2 topic that MoveIt2 listens to
    
    Args:
        request: MoveIt2 joint trajectory request
        http_request: FastAPI request object to access app state
        
    Returns:
        MoveIt2Response: Response with command status
    """
    # Get ROS2 node from app state
    ros2_node = getattr(http_request.app.state, 'ros2_node', None)
    
    if ros2_node is None:
        logger.error("ROS2 node not initialized")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="ROS2 node is not initialized. Please check the service status."
        )
    
    # Check if MoveIt2 publisher is available
    if not hasattr(ros2_node, 'moveit_joint_trajectory_publisher') or ros2_node.moveit_joint_trajectory_publisher is None:
        logger.error("MoveIt2 joint trajectory publisher not available")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="MoveIt2 joint trajectory publisher is not available. Ensure MoveIt2 packages are installed."
        )
    
    # Prepare trajectory data
    trajectory_data = {
        'joint_names': request.joint_names,
        'positions': request.positions,
        'time_from_start': request.time_from_start
    }
    
    # Add optional fields
    if request.velocities:
        trajectory_data['velocities'] = request.velocities
    if request.accelerations:
        trajectory_data['accelerations'] = request.accelerations
    
    # Publish to MoveIt2
    try:
        success = ros2_node.send_moveit_joint_trajectory(trajectory_data)
        
        if success:
            logger.info(
                f"MoveIt2 joint trajectory published successfully: "
                f"{len(request.joint_names)} joints, time_from_start={request.time_from_start:.2f}s"
            )
            return MoveIt2Response(
                status="sent",
                command_type="joint_trajectory",
                success=True,
                message=f"Joint trajectory published successfully to MoveIt2 topic",
                topic=ros2_node.moveit_joint_trajectory_publisher.topic_name if hasattr(ros2_node.moveit_joint_trajectory_publisher, 'topic_name') else "/joint_trajectory_controller/joint_trajectory"
            )
        else:
            logger.error(f"Failed to publish MoveIt2 joint trajectory")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to publish joint trajectory to MoveIt2 topic"
            )
            
    except Exception as e:
        logger.error(f"Error publishing MoveIt2 joint trajectory: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error publishing joint trajectory to MoveIt2: {str(e)}"
        )

