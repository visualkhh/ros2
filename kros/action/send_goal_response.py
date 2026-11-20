from dataclasses import dataclass


@dataclass(frozen=True)
class SendGoalResponse:
    accepted: bool
