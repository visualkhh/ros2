"""rl_nav_controller 흉내: 학습한 정책으로 경로를 따라가는 controller 플러그인. import하면 controller_server의 플러그인 표에
"rl_nav_controller::RlNavController"로 들어간다 (pluginlib처럼)."""
from kros.nodes.nav2.controller.controller_server import PLUGINS

from .policy import OnnxPolicy
from .rl_nav_controller import FALLBACKS, RlNavController

PLUGINS["rl_nav_controller::RlNavController"] = RlNavController

__all__ = ["FALLBACKS", "OnnxPolicy", "RlNavController"]
