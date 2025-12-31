from pydantic import BaseModel


class Config(BaseModel):
    ros2_node_name: str = ""