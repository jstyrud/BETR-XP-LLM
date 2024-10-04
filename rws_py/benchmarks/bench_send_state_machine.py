import pathlib
import time

import rws_py
from rws_py.rws_config import RWSConfig
from rws_py.rws_py_client import RWSPy
from rws_py.types import ControllerState

# VC
# ip = "127.0.0.1"
# port = "80"

# RC
ip = "192.168.125.1"
port = ""

f_dir = pathlib.Path(rws_py.__file__).parent.parent.resolve().joinpath("demos")
STATE_IDLE = 0
STATE_RUNNING = 1
STATE_FINISHED = 2


class CFreePathToController:
    rws: RWSPy
    state: int = 0  # 0 is idle, 1 is running
    path_queue = []
    sig_name = "RUN_CFREE_ROUTINE"
    has_path = False

    def __init__(self, rws: RWSPy) -> None:
        self.rws = rws
        self.rws.subscribe_to_var(
            module_name="CFreeRapid",
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
        self.rws.ios_set_signal_value(name=self.sig_name, value=0)
        self.rws.ios_set_signal_value(name=self.sig_name, value=1)

    def _copy_next_path_to_ctr(self):
        if len(self.path_queue) > 0:
            self.rws.files_upload_file_to_controller(
                path="$Home/", file_name="CfreePath.mod", content=self.path_queue[0]
            )
            self.path_queue.pop(0)
            print(f"Copy next path. queue size = {len(self.path_queue)}")
            self.has_path = True
        else:
            self.has_path = False

    def block_until_all_paths_done(self):

        while self.has_path or self.state == STATE_RUNNING:
            time.sleep(0.002)


def main():
    with RWSPy(RWSConfig(rw_version=6, ip=ip, port=port), debug_mode=False) as rws:

        all_signals = rws.ios_get_signals()
        sig_name = "RUN_CFREE_ROUTINE"

        # check if the signal defined in demo3_load_config is among the signals
        if sig_name not in all_signals:
            print("Run demo3_load_config first to define the signal.")

        # set up the state machine once and start rapid execution
        with open(f_dir.joinpath("CFreeRapid.mod"), encoding="UTF-8") as f:
            rapid_mod = f.read()

        rws.files_upload_file_to_controller(
            path="$Home/", file_name="CfreeRapid.mod", content=rapid_mod
        )
        rws.rapid_stop_execution()
        rws.rapid_load_module(module_path="$Home/CfreeRapid.mod")
        rws.rapid_set_pp_to_routine(module_name="CFreeRapid", routine_name="entry")
        rws.controller_switch_motors(ControllerState.motoron)
        rws.rapid_start_execution()

        path_interface = CFreePathToController(rws)

        rapid = "\
            MODULE CfreePath \n\
                PROC path() \n\
                    MoveAbsJ [[10,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v1500,fine,tool0\WObj:=wobj0; \n\
                    MoveAbsJ [[-10,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v1500,fine,tool0\WObj:=wobj0; \n\
                ENDPROC \n\
            ENDMODULE \n\
            "

        t0 = time.time()

        n_run = 4
        for i in range(n_run):
            path_interface.add_path_to_que(rapid)
        path_interface.block_until_all_paths_done()
        t_toal = time.time() - t0

        # v400
        motion_time = 1.128
        # v1500
        motion_time = 0.5
        print(f"motion time arounk {motion_time*n_run}")
        print(f"Total time was  {t_toal}")  # 5.30
        print(f"Time per send  {int((t_toal-motion_time*n_run)/n_run*1000)} ms")  # 5.30
        rws.rapid_stop_execution()

        # i = 0
        # while i < 100:
        #     time.sleep(0.1)
        #     i += 1


if __name__ == "__main__":
    main()
