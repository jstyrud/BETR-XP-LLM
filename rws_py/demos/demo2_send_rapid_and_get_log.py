"""
"""
import matplotlib.pyplot as plt
import numpy as np

from rws_py.rws_config import RWSConfig
from rws_py.rws_py_client import RWSPy
from rws_py.utils_data_parse import *
from rws_py.utils_rapid import *


def process_rw_data(data):
    out = {}
    out["t"] = data[:, 0]
    out["q"] = data[:, 1:7]
    out["qd"] = data[:, 7:13]
    dt = out["t"][1] - out["t"][0]
    out["qdd"] = np.vstack((np.diff(out["qd"], axis=0) / dt, [0] * 6))
    out["tau"] = data[:, 13:19]
    return out


def get_rapid_module(
    qi,
    qf,
    payload_mass,
    log_signals,
    log_duration,
    log_file_name,
    tcp_pos=[0] * 3,
    tool_cog=[0, 0, 0.001],
):
    """
    Parameters
    ----------
    qi, qf: list of floats
        In radians
    payload_mass: float
        In Kg
    tcp_pos: list of float
        In meter
    tool_cog: list of float
        In meter
    log_duration: float
        In second
    log_signals: list of integers
    log_file_name: str
    """
    tool_mass = 0.001
    if payload_mass != 0:
        tool_mass = payload_mass
    return f"\
    MODULE MyModule \n\
        {rapid_tool_definition('mytool', tool_mass, tcp_pos, tool_cog )} \n\
        PERS num duration; \n\
        PROC my_proc() \n\
            {rapid_move_abs_j_str(qi, 'vmax', zone='fine', tool='mytool')}\n\
            define_log_signals {log_duration}, 4;\n\
            {rapid_move_abs_j_str(qf, 'vmax', zone='fine', tool='mytool')}\n\
            DataLogSave \"{log_file_name}\";\n\
            DataLogReset;\n\
        ENDPROC \n\
            {rapid_routine_define_log_signals(log_signals, ndof=6)}\
    ENDMODULE \n\
    "


def main():
    with RWSPy(
        RWSConfig(
            rw_version=7,
            ip="127.0.0.1",
            port="80",
            user="Default User",
            password="robotics",
        ),
        debug_mode=True,
    ) as interface:

        # Generate the rapid program
        qi = [0, 0, 0, 0, 0, 0]
        qf = [-1, -1, -1, -1, -1, -1]
        content = get_rapid_module(
            qi,
            qf,
            payload_mass=0.0,
            log_signals=[7656, 5353],
            log_duration=1,
            log_file_name="mymy.log",
        )

        print(content)

        # Send the rapid program to the controller
        interface.execute_rapid_program(content, module_name="MyModule", proc_name="my_proc")

        # Get log file and parse it
        log_content = interface.files_get_file_content_from_controller("$Home", "mymy.log")
        log_data, _ = parse_rw_log_content(log_content)
        log_data2 = remove_zero_diff_start_and_end(
            log_data, columns_to_use_for_diff=[1, 2, 3, 4, 5, 6], remove_start=False
        )
        result = process_rw_data(log_data2)

        # Plot
        plt.plot(result["t"], result["qd"], ".-")
        plt.show()


if __name__ == "__main__":
    main()
