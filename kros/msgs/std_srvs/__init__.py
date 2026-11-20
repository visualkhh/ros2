"""ROS std_srvs 흉내: 인자 없이 "해라"만 보내는 서비스."""
from .trigger import Trigger, TriggerRequest, TriggerResponse

__all__ = ["Trigger", "TriggerRequest", "TriggerResponse"]
