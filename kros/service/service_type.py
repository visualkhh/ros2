from dataclasses import dataclass
from typing import Generic, TypeVar

Req = TypeVar("Req")
Res = TypeVar("Res")


@dataclass(frozen=True)
class ServiceType(Generic[Req, Res]):
    """ROS .srv 흉내: Request 타입과 Response 타입 한 쌍."""

    name: str
    request: type[Req]
    response: type[Res]
