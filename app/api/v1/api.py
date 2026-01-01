from fastapi import APIRouter

from app.api.v1.endpoints import control, health, moveit

# Create API router for v1
api_router = APIRouter()

# Include endpoint routers
api_router.include_router(
    control.router,
    prefix="/control",
    tags=["control"]
)

api_router.include_router(
    health.router,
    prefix="/health",
    tags=["health"]
)

api_router.include_router(
    moveit.router,
    prefix="/moveit",
    tags=["moveit"]
)