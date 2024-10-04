import enum


class ControllerOperationMode(enum.Enum):
    INIT = 0
    """ INIT-State init."""

    AUTO_CH = 1
    """ AUTO_CH-State change request for automatic mode."""

    MANF_CH = 2
    """ MANF_CH-State change request for manual mode & full speed."""

    MANR = 3

    MANF = 4
    """ MANF-State manual mode & full speed."""

    AUTO = 5
    """ AUTO-State automatic mode."""

    UNDEF = 6
    """ UNDEF-Undefined."""


class ControllerState(enum.Enum):
    init = 0
    """init- The robot is starting up. It will shift to state motors off when it has started."""

    motoroff = 1
    """ motoroff- The robot is in a standby state where there is no power to the robot's motors. 
    The state has to be shifted to motors on before the robot can move."""

    motoron = 2
    """ motoron- The robot is ready to move, either by jogging or by running programs."""

    guardstop = 3
    """ guardstop- The robot is stopped because the safety runchain is opened. For instance, a 
    door to the robot's cell might be open."""

    emergencystop = 4
    """ emergencystop- The robot is stopped because emergency stop was activated."""

    emergencystopreset = 5
    """ emergencystopreset- The robot is ready to leave emergency stop state. The emergency stop 
    is no longer activated, but the state transition isn't yet confirmed."""

    sysfail = 6
    """ sysfail- The robot is in a system failure state. Restart required."""
    #
    unknown = 7
