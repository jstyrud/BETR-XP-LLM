import numpy as np


def rapid_routine_define_log_signals(joint_signals, ndof=6):
    """
    only up to 19 total signals allowed (joint_signals * ndof + 1(time)  )
    """
    content = "\n        LOCAL PROC define_log_signals(num duration,num sample) \n\
            DataLogReset;\n"

    idx = 1
    for sig in joint_signals:
        for i in range(ndof):
            content += f"            DataLogDefine {idx}, {sig}, IRB, {i+1}\Sample := sample;\n"
            idx += 1
    content += "\
            DataLogStart duration;\n\
            DataLogTrigg 1000;! just some very large number\n\
        ENDPROC\n"
    return content


def rapid_tool_definition(name, tool_mass, tcp_pos, tool_cog):
    """
    Parameters
    ----------
    name: str
    payload_mass: float
        In Kg
    tcp_pos: list of float
        In meter
    tool_cog: list of float
        In meter
    """
    return f"PERS tooldata {name} := [TRUE, [[{tcp_pos[0]*1000}, {tcp_pos[1]*1000}, {tcp_pos[2]*1000}], [1, 0, 0, 0]], [{tool_mass}, [{tool_cog[0]*1000}, {tool_cog[1]*1000}, {tool_cog[2]*1000}],[1, 0, 0, 0], 0, 0, 0]];"


def rapid_move_abs_j_str(robax, speed, zone, tool, wobj="wobj0", extax=None, in_degree=False):
    """
    Parameters
    ----------
    qi, qf: list of floats
        In radians
    speed: str
        e.g. v10, vmax
    speed: str
        e.g. z0, fine
    """
    conv = 180 / np.pi
    if in_degree:
        conv = 1

    extax_str = "[9E+9, 9E+9, 9E+9, 9E+9, 9E+9, 9E+9]"
    if extax is not None and extax[0] != 9e9:
        extax_str = str([q * conv for q in extax])
    jtarg = f"{[q*conv for q in robax]}, {extax_str}]"
    return f"MoveAbsJ [{jtarg}, {speed}, {zone}, {tool}, \\WObj:={wobj};\n"
