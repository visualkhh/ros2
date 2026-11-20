from dataclasses import dataclass


@dataclass(frozen=True)
class KeyValue:
    key: str
    value: str
