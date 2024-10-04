""" RWS Configurations"""


from dataclasses import dataclass


@dataclass
class RWSConfig:
    """RW Configurations"""

    auth: str = ""
    user = "Default User"
    password = "robotics"
    host_ip: str = "127.0.0.1"
    protocol: str = ""
    port: str = "80"
    ws_protocol: str = ""
    ws_port: str = "80"
    ws_sup_protocol: str = ""
    rw_version: int = 6
    ctrstate: str = ""

    def __init__(
        self,
        rw_version: int = 6,
        user: str = "Default User",
        password: str = "robotics",
        ip: str = "127.0.0.1",
        port: str = "80",
    ) -> None:
        self.user = user
        self.password = password
        self.host_ip = ip
        self.port = port
        self.ws_port = port
        self.rw_version = rw_version
        if rw_version == 6:
            self.auth = "digest"
            self.protocol = "http"
            self.ws_protocol = "ws"
            self.ws_sup_protocol = "robapi2_subscription"
            self.ctrstate = "ctrlstate"
        else:
            # self.port = "443"
            self.auth = "basic"
            self.protocol = "https"
            self.ws_protocol = "wss"
            # self.ws_port = "443"
            self.ws_sup_protocol = "rws_subscription"
            self.ctrstate = "ctrl-state"


# "subscription_urls": [
# 	"/rw/rapid/tasks/T_ROB1;excstate",
# 	"/rw/rapid/tasks/T_ROB1;syncstatechange",
# 	"/rw/rapid/tasks/T_ROB1/pcp;programpointerchange",
# 	"/rw/rapid/tasks/<taskname>/pcp;motionpointerchange",
# 	"/progress/1",
# 	"/ctrl/diagnostics",
# 	"/rw/iosystem/signals/Local/DRV_1/DRV1K1;state",
# 	"/rw/rapid/tasks/T_ROB1;taskchange",
# 	"/rw/rapid/tasks/T_ROB1;syncstatechange",
# 	"/rw/rapid/tasks/T_ROB1;excstate",
# 	"/rw/panel/ctrl-state",
# 	"/rw/motionsystem/errorstate;erroreventchange",
# 	"/rw/rapid/taskselection;taskpanelchange",
# 	"/rw/dipc/{queue}",
# 	"/rw/dipc",
# 	"/rw/rapid/execution;hdtrun",
# 	"/rw/rapid/tasks;buildlogchange",
# 	"/rw/elog/{domain}",
# 	"/rw/panel/opmode",
# 	"/rw/rapid/execution;ctrlexecstate",
# 	"/rw/panel/speedratio",
# 	"/rw/rapid/execution;rapidexeccycle",
# 	"/rw/rapid/symbol/RAPID/T_ROB1/uimsg/PNum1/data;value",
# 	"/rw/system/energy",
# 	"/rw/motionsystem/mechunits;mechunitmodechangecount",
# 	"/rw/cfg",
# 	"/rw/mastership/{motion|rapid}",
# 	"/rw/iosystem/networks/Local;state",
# 	"/rw/iosystem/devices/Local/PANEL;state",
# 	"/users/rmmp",
# 	"/rw/rapid/uiinstr;uievent"
# ]
