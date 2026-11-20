from dataclasses import dataclass

from kros.service.service_type import ServiceType


@dataclass(frozen=True)
class TriggerRequest:
    pass


@dataclass(frozen=True)
class TriggerResponse:
    success: bool
    message: str = ""


Trigger = ServiceType("std_srvs/srv/Trigger", TriggerRequest, TriggerResponse)
