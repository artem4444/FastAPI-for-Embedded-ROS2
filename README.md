access to github repo via https
```bash
# Change remote to HTTPS
git remote set-url origin https://github.com/artem4444/FastAPI-for-Embedded-ROS2.git

# Push (will prompt for credentials)
git push -u origin main
```

# Steps
- code fastapi listener
-

- use existing wsl2 ubuntu 22.04; ros2 build 
- enter, pull gh repo to the wsl2
- run the Node in ros2 humble with topic printing in terminal: topic echo


# Transmitting data over internet
transmitted data format:


WiFi and LTE are mostly transparent since both use IP


Differences that appear in network configuration, reliability handling, and monitoring are handled in the Fastapi python directory that i code and that will run on the embedded cpu AND by driver of the modem or wifi module installed in the embedded CPU OS


daemon (or service on Windows) is a background process that runs continuously, usually without a user interface, and starts automatically at boot

# System-wide requirements: Windows 11 without Admin privileges
https://www.python.org/downloads/windows/
    C:\Users\arm1259\AppData\Local\Programs\Python\Python314

```
PS C:\Users\arm1259\FastAPI for Embedded ROS2> $env:Path += ";C:\Users\arm1259\AppData\Local\Programs\Python\Python314;C:\Users\arm1259\AppData\Local\Programs\Python\Python314\Scripts"
PS C:\Users\arm1259\FastAPI for Embedded ROS2> python --version 
Python 3.14.2
PS C:\Users\arm1259\FastAPI for Embedded ROS2> python -m venv venv    

```

# venv
vevn for development:
- track requirements: `pip freeze > requirements.txt`
- isolate projects installations

.\venv\Scripts\Activate.ps1
pip install fastapi uvicorn


# FastAPI Node
Python FastAPI Node that receives high-level control data for Embedded ROS2 via WIFI/LTE 



# Python Directory

GPT recommended directory structure

FastAPI for Embedded ROS2/
├── app/
│   ├── __init__.py
│   ├── main.py                 # FastAPI application entry point
│   ├── config.py               # Configuration settings
│   ├── models/
│   │   ├── __init__.py
│   │   └── schemas.py          # Pydantic models for request/response
│   ├── api/
│   │   ├── __init__.py
│   │   ├── v1/
│   │   │   ├── __init__.py
│   │   │   ├── api.py          # API router aggregator
│   │   │   └── endpoints/
│   │   │       ├── __init__.py
│   │   │       ├── control.py  # Control data endpoints
│   │   │       └── health.py   # Health check endpoints
│   ├── services/
│   │   ├── __init__.py
│   │   └── ros2_bridge.py      # ROS2 integration service
│   └── utils/
│       ├── __init__.py
│       └── validators.py       # Custom validators
├── tests/
│   ├── __init__.py
│   ├── test_api/
│   │   └── test_control.py
│   └── test_services/
│       └── test_ros2_bridge.py
├── requirements.txt
├── README.md
└── .env                        # Environment variables (optional)


# Architecture pattern

System/OS
    ↓
uvicorn (HTTP Server)
    ↓
main.py (FastAPI App) ← Entry Point
    ├── Creates ROS2FastapiBridgeNode (component)
    ├── Sets up routes
    ├── Configures middleware
    └── Manages lifecycle

# Files roles

`main.py` is the entry point that:
    Initializes ROS2 and creates the bridge node
    Creates and configures the FastAPI app
    Connects all the pieces (routers, middleware, ROS2)
    Handles startup/shutdown lifecycle
    Can be run directly or via uvicorn
`config.py`: Application settings used when the app runs

`ros2_bridge.py` `ROS2FastapiBridgeNode`: Publishes on-demand (when FastAPI receives a request), not on a schedule

`api.py`: Routes requests
`control.py`: Structures HTTP data into ROS2 message format
`health.py` : reports current status of FastAPI and ROS2 Node states when called

`validators.py` Helper functions used across the application

`schemas.py` defines:
    Request models — what data clients can send
    Response models — what data your API returns
    Validation rules — automatic data validation
    Type definitions — type safety and IDE support
    API documentation — auto-generated docs


# Node
the class inheriting from Node is the Node. The FastAPI app uses that Node instance to communicate with ROS2

publisher

ROS2 Interfaces. Messages
https://ros2-tutorial.readthedocs.io/en/humble/messages.html


## Interfaces for ROS2 Packages
For MoveIt2:
```python
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup

# Send target pose to MoveIt2
pose_msg = PoseStamped()
pose_msg.header.frame_id = "base_link"
pose_msg.pose.position.x = 0.5
pose_msg.pose.position.y = 0.0
pose_msg.pose.position.z = 0.5
```

For ROS2 Control:
```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

# Send joint trajectory
trajectory = JointTrajectory()
trajectory.joint_names = ["joint1", "joint2", "joint3"]
point = JointTrajectoryPoint()
point.positions = [0.5, 0.3, 0.1]
trajectory.points = [point]
```


# Topics 
Topics with timers (regular reads) are the standard approach for real-time systems with defined frequencies in ROS2


# Testing ROS2FastapiBridgeNode

1. **Build and launch ROS2** (ensure ROS2 environment is set up)
2. **Start the FastAPI server** (creates ROS2FastapiBridgeNode)
3. **Connect via WLAN/HTTP** (send HTTP requests to FastAPI)
4. **Echo the topic** (subscribe to see published messages)



# Moveit2
1. User/Application → MoveIt2
   ↓
2. MoveIt2 plans trajectory (collision-free path)
   ↓
3. MoveIt2 generates JointTrajectory message
   ↓
4. MoveIt2 → ROS2 Control (via FollowJointTrajectory action)
   ↓
5. ROS2 Control executes trajectory on hardware
   ↓
6. ROS2 Control reports status back to MoveIt2


# Th. sending HTTP packets through Public Network
Internet = The Global Network
Public Network = A Network Using Public IPs

http://203.0.113.45:8000/api/v1/health
│    │              │    │
│    │              │    └─ Path (route/endpoint)
│    │              └────── Port
│    └────────────────────── Host (IP address)
└─────────────────────────── Protocol/Scheme


[Client Device]
    ↓ (HTTP Request)
    User types URL: http://203.0.113.45:8000/api/v1/health
    Browser creates HTTP request
    OS wraps it in TCP/IP packet
[Client's Router] (NAT: client private IP → client public IP)
    ↓
[Client's ISP]
    ↓
[Internet Backbone] (Multiple routers, BGP routing)
    ↓
[Your ISP]
    ↓
[Your Router - Public Side] (203.0.113.45:8000)
    ↓ (Port Forwarding Rule)
[Your Router - NAT Translation] (203.0.113.45:8000 → 192.168.1.100:8000)
    ↓
[Your Local Network] (ARP lookup, MAC address resolution)
    ↓
[Embedded Device Network Interface] (192.168.1.100:8000)
    ↓
[OS Network Stack] (Port 8000 → uvicorn process)
    ↓
[uvicorn HTTP Server]
    ↓
[FastAPI Application]
    ↓
[Endpoint Handler] (e.g., /api/v1/health)
    ↓
[ROS2 Bridge] (if needed)
    ↓
[HTTP Response Generated]
    ↓ (Reverse path back to client)


# Desktop Windows 11 - fastapi server running in wsl2 ubuntu22: sending HTTP packets through Public Network

## Tailscale
VPN tunnel; virtual LAN