"""Quest input and SO-101 contracts. Core imports require only Python."""

from importlib.util import find_spec

from .contracts import So101State, Telemetry

__all__ = ["So101State", "Telemetry"]

# Official discovery imports the distribution name. Register the config when
# the integration extra is available; a core-only install stays Torch-free.
if find_spec("lerobot") is not None and find_spec("websockets") is not None:
    from .config import QuestConfig
    from .quest import Quest

    __all__ += ["Quest", "QuestConfig"]
