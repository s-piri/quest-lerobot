"""Official LeRobot configuration registration; no hardware configuration."""

from dataclasses import dataclass, field
from pathlib import Path

from lerobot.teleoperators.config import TeleoperatorConfig

from .contracts import finite


@TeleoperatorConfig.register_subclass("quest")
@dataclass(kw_only=True)
class QuestConfig(TeleoperatorConfig):
    host: str = "127.0.0.1"
    port: int = 8765
    timeout_s: float = 0.2
    origins: list[str | None] = field(
        default_factory=lambda: [
            None,
            "http://localhost",
            "https://localhost",
        ]
    )
    cert: Path | None = None
    key: Path | None = None

    def __post_init__(self):
        if type(self.port) is not int or not 0 <= self.port <= 65535:
            raise ValueError("port must be between 0 and 65535")
        if not isinstance(self.host, str) or not self.host:
            raise ValueError("host must be a non-empty string")
        if finite(self.timeout_s, "timeout_s") <= 0:
            raise ValueError("timeout_s must be positive")
        if (self.cert is None) != (self.key is None):
            raise ValueError("TLS requires both cert and key")
        if self.host not in ("localhost", "127.0.0.1", "::1") and self.cert is None:
            raise ValueError("non-loopback input requires TLS cert and key")
        if not self.origins or any(o is not None and not isinstance(o, str) for o in self.origins):
            raise ValueError("origins must explicitly list allowed WebXR origins")
