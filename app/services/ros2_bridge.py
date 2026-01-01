import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from typing import Optional, Union, Dict, List
import logging

from app.config import settings

logger = logging.getLogger(__name__)


class ROS2FastapiBridgeNode(Node):
    """
    ROS2 Node that bridges FastAPI HTTP requests to ROS2 topics.
    
    This node publishes control commands received via FastAPI to ROS2 topics.
    """
    
    def __init__(
        self,
        node_name: Optional[str] = None,
        topic_name: Optional[str] = None,
        queue_size: Optional[int] = None,
        qos_profile: Optional[QoSProfile] = None
    ):
        """
        Initialize ROS2 FastAPI Bridge Node.
        
        Args:
            node_name: Name of the ROS2 node (defaults to config)
            topic_name: ROS2 topic name (defaults to config)
            queue_size: Publisher queue size (defaults to config)
            qos_profile: Custom QoS profile (optional)
        """
        # Use config settings as defaults, allow override
        node_name = node_name or settings.ros2_node_name
        topic_name = topic_name or settings.ros2_topic_name
        queue_size = queue_size or settings.ros2_queue_size
        
        # Initialize parent Node
        super().__init__(node_name)
        
        # Set up QoS profile (default: reliable)
        if qos_profile is None:
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                depth=queue_size
            )
        
        # Create String publisher (for generic commands)
        try:
            self.publisher = self.create_publisher(
                String,
                topic_name,
                qos_profile
            )
            logger.info(
                f"ROS2 publisher created: topic='{topic_name}', "
                f"node='{node_name}', queue_size={queue_size}"
            )
        except Exception as e:
            logger.error(f"Failed to create ROS2 publisher: {e}")
            raise
        
        # Create MoveIt2 publishers
        try:
            # MoveIt2 pose publisher
            self.moveit_pose_publisher = self.create_publisher(
                PoseStamped,
                settings.moveit_pose_topic,
                qos_profile
            )
            logger.info(f"MoveIt2 pose publisher created: topic='{settings.moveit_pose_topic}'")
            
            # MoveIt2 joint trajectory publisher
            self.moveit_joint_trajectory_publisher = self.create_publisher(
                JointTrajectory,
                settings.moveit_joint_trajectory_topic,
                qos_profile
            )
            logger.info(f"MoveIt2 joint trajectory publisher created: topic='{settings.moveit_joint_trajectory_topic}'")
        except Exception as e:
            logger.warning(f"Failed to create MoveIt2 publishers (MoveIt2 may not be available): {e}")
            # Set to None so we can check later
            self.moveit_pose_publisher = None
            self.moveit_joint_trajectory_publisher = None
    
    def send_command(self, command_data: Union[str, bytes]) -> bool:
        """
        Publish a command to the ROS2 topic.
        
        Args:
            command_data: Command string or bytes to publish
            
        Returns:
            bool: True if published successfully, False otherwise
        """
        try:
            # Convert to string if needed
            if isinstance(command_data, bytes):
                command_data = command_data.decode('utf-8')
            
            # Create and publish message
            msg = String()
            msg.data = str(command_data)
            
            self.publisher.publish(msg)
            logger.debug(f"Published command to ROS2: {command_data}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to publish command: {e}")
            return False
    
    def is_publisher_ready(self) -> bool:
        """
        Check if publisher is ready (has subscribers or queue available).
        
        Returns:
            bool: True if publisher is ready
        """
        try:
            # Check if publisher exists and has capacity
            return (
                self.publisher is not None and
                self.publisher.get_subscription_count() >= 0
            )
        except Exception:
            return False
    
    def send_moveit_pose(self, pose_data: Dict) -> bool:
        """
        Convert pose data to PoseStamped and publish to MoveIt2.
        
        Args:
            pose_data: Dictionary with keys: x, y, z, frame_id, orientation (optional)
            
        Returns:
            bool: True if published successfully, False otherwise
        """
        if self.moveit_pose_publisher is None:
            logger.error("MoveIt2 pose publisher not initialized")
            return False
        
        try:
            # Create PoseStamped message
            pose = PoseStamped()
            
            # Set header
            pose.header = Header()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = pose_data.get('frame_id', settings.moveit_default_frame_id)
            
            # Set position
            pose.pose.position = Point()
            pose.pose.position.x = float(pose_data.get('x', 0.0))
            pose.pose.position.y = float(pose_data.get('y', 0.0))
            pose.pose.position.z = float(pose_data.get('z', 0.0))
            
            # Set orientation (quaternion)
            orientation = pose_data.get('orientation')
            if orientation:
                pose.pose.orientation = Quaternion()
                pose.pose.orientation.x = float(orientation.get('x', 0.0))
                pose.pose.orientation.y = float(orientation.get('y', 0.0))
                pose.pose.orientation.z = float(orientation.get('z', 0.0))
                pose.pose.orientation.w = float(orientation.get('w', 1.0))
            else:
                # Default to identity quaternion
                pose.pose.orientation = Quaternion()
                pose.pose.orientation.w = 1.0
            
            # Publish
            self.moveit_pose_publisher.publish(pose)
            logger.info(
                f"Published MoveIt2 pose: x={pose.pose.position.x:.3f}, "
                f"y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f}, "
                f"frame={pose.header.frame_id}"
            )
            return True
            
        except Exception as e:
            logger.error(f"Failed to publish MoveIt2 pose: {e}", exc_info=True)
            return False
    
    def send_moveit_joint_trajectory(self, trajectory_data: Dict) -> bool:
        """
        Convert joint trajectory data to JointTrajectory and publish to MoveIt2.
        
        Args:
            trajectory_data: Dictionary with keys: joint_names, positions, 
                            velocities (optional), accelerations (optional), time_from_start
            
        Returns:
            bool: True if published successfully, False otherwise
        """
        if self.moveit_joint_trajectory_publisher is None:
            logger.error("MoveIt2 joint trajectory publisher not initialized")
            return False
        
        try:
            # Create JointTrajectory message
            trajectory = JointTrajectory()
            
            # Set joint names
            joint_names = trajectory_data.get('joint_names', [])
            if not joint_names:
                raise ValueError("joint_names cannot be empty")
            trajectory.joint_names = [str(name) for name in joint_names]
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            
            # Set positions
            positions = trajectory_data.get('positions', [])
            if len(positions) != len(joint_names):
                raise ValueError(f"Number of positions ({len(positions)}) must match number of joints ({len(joint_names)})")
            point.positions = [float(p) for p in positions]
            
            # Set velocities (optional)
            velocities = trajectory_data.get('velocities')
            if velocities:
                if len(velocities) != len(joint_names):
                    raise ValueError(f"Number of velocities ({len(velocities)}) must match number of joints ({len(joint_names)})")
                point.velocities = [float(v) for v in velocities]
            
            # Set accelerations (optional)
            accelerations = trajectory_data.get('accelerations')
            if accelerations:
                if len(accelerations) != len(joint_names):
                    raise ValueError(f"Number of accelerations ({len(accelerations)}) must match number of joints ({len(joint_names)})")
                point.accelerations = [float(a) for a in accelerations]
            
            # Set time from start
            time_from_start = float(trajectory_data.get('time_from_start', 2.0))
            point.time_from_start = Duration()
            point.time_from_start.sec = int(time_from_start)
            point.time_from_start.nanosec = int((time_from_start - int(time_from_start)) * 1e9)
            
            # Add point to trajectory
            trajectory.points = [point]
            
            # Publish
            self.moveit_joint_trajectory_publisher.publish(trajectory)
            logger.info(
                f"Published MoveIt2 joint trajectory: {len(joint_names)} joints, "
                f"time_from_start={time_from_start:.2f}s"
            )
            return True
            
        except Exception as e:
            logger.error(f"Failed to publish MoveIt2 joint trajectory: {e}", exc_info=True)
            return False
    
    def is_moveit_ready(self) -> bool:
        """
        Check if MoveIt2 publishers are ready.
        
        Returns:
            bool: True if MoveIt2 publishers are initialized and ready
        """
        try:
            pose_ready = (
                self.moveit_pose_publisher is not None and
                self.moveit_pose_publisher.get_subscription_count() >= 0
            )
            trajectory_ready = (
                self.moveit_joint_trajectory_publisher is not None and
                self.moveit_joint_trajectory_publisher.get_subscription_count() >= 0
            )
            return pose_ready and trajectory_ready
        except Exception:
            return False

