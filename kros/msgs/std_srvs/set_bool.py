from dataclasses import dataclass

from kros.service.service_type import ServiceType


@dataclass(frozen=True)
class SetBoolRequest:
    data: bool


@dataclass(frozen=True)
class SetBoolResponse:
    success: bool
    message: str = ""


SetBool = ServiceType("std_srvs/srv/SetBool", SetBoolRequest, SetBoolResponse)
