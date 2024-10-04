import pathlib
import time

import rws_py
from rws_py.rws_config import RWSConfig
from rws_py.rws_py_client import RWSPy
from rws_py.types import ControllerState

# VC
ip = "127.0.0.1"
port = "80"

# RC
# ip = "192.168.125.1"
# port = ""

f_dir = pathlib.Path(rws_py.__file__).parent.parent.resolve().joinpath("demos")
STATE_IDLE = 0
STATE_RUNNING = 1
STATE_FINISHED = 2

RAPID_MODULE_NAME_MOTION = "CFreeMotion"
RAPID_MODULE_NAME_LOADER = "CFreeLoader"
RAPID_TASK_NAME_MOTION = "T_ROB1"
RAPID_TASK_NAME_LOADER = "T_Loader"


class CFreePathToController:
    rws: RWSPy
    state: int = 0  # 0 is idle, 1 is running
    path_queue = []
    _run_path_sig_name = "CFREE_RUN_PATH"
    _load_nextpath_sig_name = "CFREE_LOAD_NEXT_PATH"
    has_path = False
    _is_path_1 = False

    def __init__(self, rws: RWSPy) -> None:
        self.rws = rws
        self.rws.subscribe_to_var(
            module_name=RAPID_MODULE_NAME_MOTION,
            variable_name="state",
            callback=self.state_callback,
            priority=2,
        )

    def state_callback(self, val):
        self.state = int(val["value"])
        print(f"--------------------  CFree path is running = {self.state}")

        if self.state == STATE_IDLE and self.has_path:
            self._execute_path()

        elif self.state == STATE_RUNNING:
            self._copy_next_path_to_ctr()

    def add_path_to_que(self, rapid_path: str):
        self.path_queue.append(rapid_path)
        print(f"Added path. queue size = {len(self.path_queue)}")

        if len(self.path_queue) == 1 and self.state == STATE_IDLE and not self.has_path:
            self._copy_next_path_to_ctr()
            self._execute_path()
        self.has_path = True

    def _execute_path(self):
        mod_name = f"CfreePath{int(self._is_path_1)}"
        self.rws.rapid_set_symbol(
            task_name=RAPID_TASK_NAME_MOTION,
            module_name=RAPID_MODULE_NAME_MOTION,
            variable_name="path_mod_name",
            value=f'"{mod_name}"',
        )
        self.rws.ios_set_signal_value(name=self._run_path_sig_name, value=0)
        self.rws.ios_set_signal_value(name=self._run_path_sig_name, value=1)

    def _copy_next_path_to_ctr(self):
        if len(self.path_queue) > 0:
            mod_name = f"CfreePath{int(self._is_path_1)}"

            self._is_path_1 = not self._is_path_1

            rapid_code = self._add_module_proc(
                code=self.path_queue[0], mod_name=mod_name, proc_name="path"
            )

            self.rws.files_upload_file_to_controller(
                path="$Home/", file_name=mod_name, content=rapid_code
            )

            self.rws.rapid_set_symbol(
                task_name=RAPID_TASK_NAME_LOADER,
                module_name=RAPID_MODULE_NAME_LOADER,
                variable_name="path_mod_name",
                value=f'"{mod_name}"',
            )
            self.rws.ios_set_signal_value(name=self._load_nextpath_sig_name, value=0)
            self.rws.ios_set_signal_value(name=self._load_nextpath_sig_name, value=1)

            self.path_queue.pop(0)
            print(f"Copy next path. queue size = {len(self.path_queue)}")
            self.has_path = True
        else:
            self.has_path = False

    def block_until_all_paths_done(self):

        while self.has_path or self.state == STATE_RUNNING:
            time.sleep(0.002)

    def _add_module_proc(self, code: str, mod_name: str, proc_name: str):
        all_code = ""
        all_code = f"MODULE {mod_name}\n"
        all_code += f"  PROC {proc_name}()\n"
        all_code += code
        all_code += "  ENDPROC\n"
        all_code += "ENDMODULE\n"
        return all_code


def main():
    with RWSPy(RWSConfig(rw_version=6, ip=ip, port=port), debug_mode=True) as rws:

        if not rws.ios_get_signal("CFREE_LOAD_NEXT_PATH") or not rws.ios_get_signal(
            "CFREE_RUN_PATH"
        ):
            print(
                "Run demo3_load_config first to define the signals CFREE_LOAD_NEXT_PATH and CFREE_RUN_PATH."
            )

        # set up the state machine once and start rapid execution
        # with open(f_dir.joinpath("CFreeRapid.mod"), encoding="UTF-8") as f:
        #     rapid_mod = f.read()

        # rws.files_upload_file_to_controller(
        #     path="$Home/", file_name="CfreeRapid.mod", content=rapid_mod
        # )
        rws.rapid_stop_execution()
        # rws.rapid_load_module(module_path="$Home/CfreeRapid.mod")
        rws.rapid_set_pp_to_routine(
            task_name=RAPID_TASK_NAME_MOTION, module_name="CFreeMotion", routine_name="entry"
        )
        rws.rapid_set_pp_to_routine(
            task_name=RAPID_TASK_NAME_LOADER, module_name="CFreeLoader", routine_name="main"
        )
        rws.controller_switch_motors(ControllerState.motoron)
        rws.rapid_start_execution()

        path_interface = CFreePathToController(rws)

        # --------- dynamic part

        rapid = "MoveAbsJ [[-5,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v400,fine,tool0\\WObj:=wobj0; \n\
                 MoveAbsJ [[5,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v400,fine,tool0\\WObj:=wobj0; \n"
        path_interface.add_path_to_que(rapid)

        rapid = "MoveAbsJ [[-10,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v400,fine,tool0\\WObj:=wobj0; \n\
                 MoveAbsJ [[10,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v400,fine,tool0\\WObj:=wobj0; \n"
        path_interface.add_path_to_que(rapid)

        rapid = "MoveAbsJ [[-15,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v400,fine,tool0\\WObj:=wobj0; \n\
                 MoveAbsJ [[15,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v400,fine,tool0\\WObj:=wobj0; \n"
        path_interface.add_path_to_que(rapid)

        rapid = "MoveAbsJ [[-20,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v400,fine,tool0\\WObj:=wobj0; \n\
                 MoveAbsJ [[20,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v400,fine,tool0\\WObj:=wobj0; \n"
        path_interface.add_path_to_que(rapid)

        path_interface.block_until_all_paths_done()

        rws.rapid_stop_execution()

        # i = 0
        # while i < 100:
        #     time.sleep(0.1)
        #     i += 1


if __name__ == "__main__":
    main()
