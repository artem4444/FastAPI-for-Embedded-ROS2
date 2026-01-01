"""
Tests for ROS2 bridge service.
"""
import pytest
from unittest.mock import Mock, MagicMock, patch, call
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory


@pytest.mark.service
class TestROS2FastapiBridgeNode:
    """Test ROS2FastapiBridgeNode class."""
    
    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_node_initialization_defaults(self, mock_shutdown, mock_init):
        """Test node initialization with default settings."""
        from app.services.ros2_bridge import ROS2FastapiBridgeNode
        from rclpy.node import Node
        
        with patch.object(Node, '__init__', return_value=None), \
             patch.object(Node, 'create_publisher') as mock_create_pub:
            
            mock_publisher = Mock()
            mock_create_pub.return_value = mock_publisher
            
            node = ROS2FastapiBridgeNode()
            
            assert node is not None
            mock_init.assert_called_once()
            assert mock_create_pub.called
    
    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_node_initialization_custom_params(self, mock_shutdown, mock_init):
        """Test node initialization with custom parameters."""
        from app.services.ros2_bridge import ROS2FastapiBridgeNode
        from rclpy.node import Node
        
        with patch.object(Node, '__init__', return_value=None), \
             patch.object(Node, 'create_publisher') as mock_create_pub:
            
            mock_publisher = Mock()
            mock_create_pub.return_value = mock_publisher
            
            node = ROS2FastapiBridgeNode(
                node_name="custom_node",
                topic_name="/custom/topic",
                queue_size=20
            )
            
            assert node is not None
            # Verify create_publisher was called with custom topic
            calls = mock_create_pub.call_args_list
            assert any('/custom/topic' in str(call) for call in calls)
    
    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_send_command_success(self, mock_shutdown, mock_init):
        """Test sending a command successfully."""
        from app.services.ros2_bridge import ROS2FastapiBridgeNode
        from rclpy.node import Node
        
        with patch.object(Node, '__init__', return_value=None), \
             patch.object(Node, 'create_publisher') as mock_create_pub, \
             patch.object(Node, 'get_clock') as mock_get_clock:
            
            mock_publisher = Mock()
            mock_create_pub.return_value = mock_publisher
            
            node = ROS2FastapiBridgeNode()
            node.publisher = mock_publisher
            
            result = node.send_command("test_command")
            
            assert result is True
            assert mock_publisher.publish.called
            # Verify String message was published
            call_args = mock_publisher.publish.call_args[0][0]
            assert isinstance(call_args, String)
            assert call_args.data == "test_command"
    
    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_send_command_bytes(self, mock_shutdown, mock_init):
        """Test sending a command as bytes."""
        from app.services.ros2_bridge import ROS2FastapiBridgeNode
        from rclpy.node import Node
        
        with patch.object(Node, '__init__', return_value=None), \
             patch.object(Node, 'create_publisher') as mock_create_pub:
            
            mock_publisher = Mock()
            mock_create_pub.return_value = mock_publisher
            
            node = ROS2FastapiBridgeNode()
            node.publisher = mock_publisher
            
            result = node.send_command(b"test_command")
            
            assert result is True
            call_args = mock_publisher.publish.call_args[0][0]
            assert call_args.data == "test_command"
    
    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_send_command_failure(self, mock_shutdown, mock_init):
        """Test handling command send failure."""
        from app.services.ros2_bridge import ROS2FastapiBridgeNode
        from rclpy.node import Node
        
        with patch.object(Node, '__init__', return_value=None), \
             patch.object(Node, 'create_publisher') as mock_create_pub:
            
            mock_publisher = Mock()
            mock_publisher.publish.side_effect = Exception("Publish error")
            mock_create_pub.return_value = mock_publisher
            
            node = ROS2FastapiBridgeNode()
            node.publisher = mock_publisher
            
            result = node.send_command("test_command")
            
            assert result is False
    
    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_is_publisher_ready(self, mock_shutdown, mock_init):
        """Test checking if publisher is ready."""
        from app.services.ros2_bridge import ROS2FastapiBridgeNode
        from rclpy.node import Node
        
        with patch.object(Node, '__init__', return_value=None), \
             patch.object(Node, 'create_publisher') as mock_create_pub:
            
            mock_publisher = Mock()
            mock_publisher.get_subscription_count.return_value = 1
            mock_create_pub.return_value = mock_publisher
            
            node = ROS2FastapiBridgeNode()
            node.publisher = mock_publisher
            
            result = node.is_publisher_ready()
            
            assert result is True
            mock_publisher.get_subscription_count.assert_called()
    
    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_is_publisher_ready_no_publisher(self, mock_shutdown, mock_init):
        """Test checking publisher ready when publisher is None."""
        from app.services.ros2_bridge import ROS2FastapiBridgeNode
        from rclpy.node import Node
        
        with patch.object(Node, '__init__', return_value=None), \
             patch.object(Node, 'create_publisher'):
            
            node = ROS2FastapiBridgeNode()
            node.publisher = None
            
            result = node.is_publisher_ready()
            
            assert result is False
    
    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_send_moveit_pose_success(self, mock_shutdown, mock_init):
        """Test sending MoveIt2 pose successfully."""
        from app.services.ros2_bridge import ROS2FastapiBridgeNode
        from rclpy.node import Node
        from builtin_interfaces.msg import Time
        
        with patch.object(Node, '__init__', return_value=None), \
             patch.object(Node, 'create_publisher') as mock_create_pub, \
             patch.object(Node, 'get_clock') as mock_get_clock:
            
            mock_pose_publisher = Mock()
            mock_clock = Mock()
            mock_time = Mock()
            mock_time.to_msg.return_value = Time()
            mock_clock.now.return_value = mock_time
            mock_get_clock.return_value = mock_clock
            
            # Setup create_publisher to return different publishers
            def create_pub_side_effect(msg_type, topic, qos):
                if msg_type == PoseStamped:
                    return mock_pose_publisher
                return Mock()
            
            mock_create_pub.side_effect = create_pub_side_effect
            
            node = ROS2FastapiBridgeNode()
            node.moveit_pose_publisher = mock_pose_publisher
            node.get_clock = mock_get_clock
            
            pose_data = {
                'x': 0.5,
                'y': 0.0,
                'z': 0.5,
                'frame_id': 'base_link'
            }
            
            result = node.send_moveit_pose(pose_data)
            
            assert result is True
            assert mock_pose_publisher.publish.called
            call_args = mock_pose_publisher.publish.call_args[0][0]
            assert isinstance(call_args, PoseStamped)
            assert call_args.pose.position.x == 0.5
            assert call_args.pose.position.y == 0.0
            assert call_args.pose.position.z == 0.5
            assert call_args.header.frame_id == 'base_link'
    
    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_send_moveit_pose_with_orientation(self, mock_shutdown, mock_init):
        """Test sending MoveIt2 pose with orientation."""
        from app.services.ros2_bridge import ROS2FastapiBridgeNode
        from rclpy.node import Node
        from builtin_interfaces.msg import Time
        
        with patch.object(Node, '__init__', return_value=None), \
             patch.object(Node, 'create_publisher') as mock_create_pub, \
             patch.object(Node, 'get_clock') as mock_get_clock:
            
            mock_pose_publisher = Mock()
            mock_clock = Mock()
            mock_time = Mock()
            mock_time.to_msg.return_value = Time()
            mock_clock.now.return_value = mock_time
            mock_get_clock.return_value = mock_clock
            
            mock_create_pub.return_value = mock_pose_publisher
            
            node = ROS2FastapiBridgeNode()
            node.moveit_pose_publisher = mock_pose_publisher
            node.get_clock = mock_get_clock
            
            pose_data = {
                'x': 0.5,
                'y': 0.0,
                'z': 0.5,
                'frame_id': 'base_link',
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
            }
            
            result = node.send_moveit_pose(pose_data)
            
            assert result is True
            call_args = mock_pose_publisher.publish.call_args[0][0]
            assert call_args.pose.orientation.w == 1.0
    
    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_send_moveit_pose_no_publisher(self, mock_shutdown, mock_init):
        """Test sending MoveIt2 pose when publisher is not available."""
        from app.services.ros2_bridge import ROS2FastapiBridgeNode
        from rclpy.node import Node
        
        with patch.object(Node, '__init__', return_value=None), \
             patch.object(Node, 'create_publisher'):
            
            node = ROS2FastapiBridgeNode()
            node.moveit_pose_publisher = None
            
            pose_data = {
                'x': 0.5,
                'y': 0.0,
                'z': 0.5,
                'frame_id': 'base_link'
            }
            
            result = node.send_moveit_pose(pose_data)
            
            assert result is False
    
    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_send_moveit_joint_trajectory_success(self, mock_shutdown, mock_init):
        """Test sending MoveIt2 joint trajectory successfully."""
        from app.services.ros2_bridge import ROS2FastapiBridgeNode
        from rclpy.node import Node
        
        with patch.object(Node, '__init__', return_value=None), \
             patch.object(Node, 'create_publisher') as mock_create_pub:
            
            mock_trajectory_publisher = Mock()
            
            def create_pub_side_effect(msg_type, topic, qos):
                if msg_type == JointTrajectory:
                    return mock_trajectory_publisher
                return Mock()
            
            mock_create_pub.side_effect = create_pub_side_effect
            
            node = ROS2FastapiBridgeNode()
            node.moveit_joint_trajectory_publisher = mock_trajectory_publisher
            
            trajectory_data = {
                'joint_names': ['joint1', 'joint2'],
                'positions': [0.5, 0.3],
                'time_from_start': 2.0
            }
            
            result = node.send_moveit_joint_trajectory(trajectory_data)
            
            assert result is True
            assert mock_trajectory_publisher.publish.called
            call_args = mock_trajectory_publisher.publish.call_args[0][0]
            assert isinstance(call_args, JointTrajectory)
            assert call_args.joint_names == ['joint1', 'joint2']
            assert len(call_args.points) == 1
            assert call_args.points[0].positions == [0.5, 0.3]
    
    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_send_moveit_joint_trajectory_with_velocities(self, mock_shutdown, mock_init):
        """Test sending joint trajectory with velocities."""
        from app.services.ros2_bridge import ROS2FastapiBridgeNode
        from rclpy.node import Node
        
        with patch.object(Node, '__init__', return_value=None), \
             patch.object(Node, 'create_publisher') as mock_create_pub:
            
            mock_trajectory_publisher = Mock()
            mock_create_pub.return_value = mock_trajectory_publisher
            
            node = ROS2FastapiBridgeNode()
            node.moveit_joint_trajectory_publisher = mock_trajectory_publisher
            
            trajectory_data = {
                'joint_names': ['joint1', 'joint2'],
                'positions': [0.5, 0.3],
                'velocities': [0.1, 0.1],
                'time_from_start': 2.0
            }
            
            result = node.send_moveit_joint_trajectory(trajectory_data)
            
            assert result is True
            call_args = mock_trajectory_publisher.publish.call_args[0][0]
            assert call_args.points[0].velocities == [0.1, 0.1]
    
    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_send_moveit_joint_trajectory_empty_joints(self, mock_shutdown, mock_init):
        """Test sending joint trajectory with empty joint names."""
        from app.services.ros2_bridge import ROS2FastapiBridgeNode
        from rclpy.node import Node
        
        with patch.object(Node, '__init__', return_value=None), \
             patch.object(Node, 'create_publisher'):
            
            node = ROS2FastapiBridgeNode()
            node.moveit_joint_trajectory_publisher = Mock()
            
            trajectory_data = {
                'joint_names': [],
                'positions': [],
                'time_from_start': 2.0
            }
            
            result = node.send_moveit_joint_trajectory(trajectory_data)
            
            assert result is False
    
    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_send_moveit_joint_trajectory_mismatched_lengths(self, mock_shutdown, mock_init):
        """Test sending joint trajectory with mismatched array lengths."""
        from app.services.ros2_bridge import ROS2FastapiBridgeNode
        from rclpy.node import Node
        
        with patch.object(Node, '__init__', return_value=None), \
             patch.object(Node, 'create_publisher'):
            
            node = ROS2FastapiBridgeNode()
            node.moveit_joint_trajectory_publisher = Mock()
            
            trajectory_data = {
                'joint_names': ['joint1', 'joint2'],
                'positions': [0.5],  # Mismatched length
                'time_from_start': 2.0
            }
            
            result = node.send_moveit_joint_trajectory(trajectory_data)
            
            assert result is False
    
    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_send_moveit_joint_trajectory_no_publisher(self, mock_shutdown, mock_init):
        """Test sending joint trajectory when publisher is not available."""
        from app.services.ros2_bridge import ROS2FastapiBridgeNode
        from rclpy.node import Node
        
        with patch.object(Node, '__init__', return_value=None), \
             patch.object(Node, 'create_publisher'):
            
            node = ROS2FastapiBridgeNode()
            node.moveit_joint_trajectory_publisher = None
            
            trajectory_data = {
                'joint_names': ['joint1'],
                'positions': [0.5],
                'time_from_start': 2.0
            }
            
            result = node.send_moveit_joint_trajectory(trajectory_data)
            
            assert result is False
    
    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_is_moveit_ready(self, mock_shutdown, mock_init):
        """Test checking if MoveIt2 is ready."""
        from app.services.ros2_bridge import ROS2FastapiBridgeNode
        from rclpy.node import Node
        
        with patch.object(Node, '__init__', return_value=None), \
             patch.object(Node, 'create_publisher'):
            
            node = ROS2FastapiBridgeNode()
            
            mock_pose_pub = Mock()
            mock_pose_pub.get_subscription_count.return_value = 1
            node.moveit_pose_publisher = mock_pose_pub
            
            mock_traj_pub = Mock()
            mock_traj_pub.get_subscription_count.return_value = 1
            node.moveit_joint_trajectory_publisher = mock_traj_pub
            
            result = node.is_moveit_ready()
            
            assert result is True
    
    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_is_moveit_ready_not_ready(self, mock_shutdown, mock_init):
        """Test checking MoveIt2 ready when publishers are not available."""
        from app.services.ros2_bridge import ROS2FastapiBridgeNode
        from rclpy.node import Node
        
        with patch.object(Node, '__init__', return_value=None), \
             patch.object(Node, 'create_publisher'):
            
            node = ROS2FastapiBridgeNode()
            node.moveit_pose_publisher = None
            node.moveit_joint_trajectory_publisher = None
            
            result = node.is_moveit_ready()
            
            assert result is False

