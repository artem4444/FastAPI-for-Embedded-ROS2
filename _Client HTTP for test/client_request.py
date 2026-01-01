"""
Production-ready FastAPI Client with Configuration Management
"""

import requests
import os
from typing import Optional, Dict, Any
from dataclasses import dataclass
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class ClientConfig:
    """Client configuration"""
    server_tailscale_ip: str
    port: int = 8000
    timeout: int = 10
    verify_ssl: bool = False  # Set True if using HTTPS
    retry_attempts: int = 3


class FastAPIClient:
    """Production-ready FastAPI client"""
    
    def __init__(self, config: Optional[ClientConfig] = None):
        if config is None:
            # Load from environment or use defaults
            config = ClientConfig(
                server_tailscale_ip=os.getenv("TAILSCALE_SERVER_IP", "100.98.109.13"),
                port=int(os.getenv("SERVER_PORT", "8000")),
                timeout=int(os.getenv("CLIENT_TIMEOUT", "10"))
            )
        self.config = config
        self.base_url = f"http://{config.server_tailscale_ip}:{config.port}"
    
    def _make_request(
        self, 
        method: str, 
        endpoint: str, 
        **kwargs
    ) -> Optional[requests.Response]:
        """Make HTTP request with retry logic"""
        url = f"{self.base_url}{endpoint}"
        
        for attempt in range(self.config.retry_attempts):
            try:
                response = requests.request(
                    method,
                    url,
                    timeout=self.config.timeout,
                    **kwargs
                )
                response.raise_for_status()
                return response
            except requests.exceptions.RequestException as e:
                if attempt == self.config.retry_attempts - 1:
                    logger.error(f"❌ Request failed after {self.config.retry_attempts} attempts: {e}")
                    return None
                logger.warning(f"⚠️ Attempt {attempt + 1} failed, retrying...")
        
        return None
    
    def health_check(self) -> Optional[Dict[str, Any]]:
        """Check server health"""
        response = self._make_request("GET", "/api/v1/health")
        return response.json() if response else None
    
    def send_control_command(self, command: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Send control command"""
        response = self._make_request(
            "POST",
            "/api/v1/control",
            json=command
        )
        return response.json() if response else None
    
    def send_moveit_pose(self, pose_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Send MoveIt2 pose command"""
        response = self._make_request(
            "POST",
            "/api/v1/moveit/pose",
            json=pose_data
        )
        return response.json() if response else None


# Usage
if __name__ == "__main__":
    # Option 1: Use defaults
    client = FastAPIClient()
    
    # Option 2: Custom config
    config = ClientConfig(
        server_tailscale_ip="100.98.109.13",
        port=8000,
        timeout=15
    )
    client = FastAPIClient(config)
    
    # Health check
    health = client.health_check()
    print(f"Health: {health}")