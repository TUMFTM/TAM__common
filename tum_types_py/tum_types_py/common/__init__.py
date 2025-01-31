from functools import total_ordering
from enum import Enum
from tum_types_py._common_binding import (  # noqa: F401
    Header,  # noqa: F401
    Vector3D,  # noqa: F401
    Vector2D,  # noqa: F401
    DataPerWheel,  # noqa: F401
    EulerYPR,  # noqa: F401
    TUMDebugContainer,  # noqa: F401
)  # noqa: F401
@total_ordering
class ErrorLvl(Enum):
    OK = 0
    WARN = 1
    ERROR = 2
    STALE = 3
    def __lt__(self, other):
        if self.__class__ is other.__class__:
            return self.value < other.value
        return NotImplemented

@total_ordering
class EmergencyLevel(Enum):
    NO_EMERGENCY = 0
    EMERGENCY_STOP = 1
    SOFT_EMERGENCY = 2
    HARD_EMERGENCY = 3
    def __lt__(self, other):
        if self.__class__ is other.__class__:
            return self.value < other.value
        return NotImplemented
