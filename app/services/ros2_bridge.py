import rclpy
from rclpy.node import Node

from std_msgs.msg import String
#std_msgs is a ROS2 package with basic message types
    #can be changed to package-specific input with ROS2 msg packages


class ROS2FastapiBridgeNode(Node):
    def __init__(self):
        super().__init__("fastapi_control_node") 
        #super() - immediate parent class
        #Node class constructor inits ROS2 Node with name "fastapi_control_node"

        self.publisher = self.create_publisher(
            String,
            "/control/commands",
            10
        )

    def send_command(self, command_data):
        msg = String()
        msg.data = command_data
        self.publisher.publish(msg)



