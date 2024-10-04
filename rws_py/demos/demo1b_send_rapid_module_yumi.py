from rws_py.rws_config import RWSConfig
from rws_py.rws_py_client import RWSPy

# VC
ip = "127.0.0.1"
port = "80"

# RC
# ip = "192.168.125.1"
# port = ""

with RWSPy(
    RWSConfig(
        rw_version=6,
        ip=ip,
        port=port,
        user="Default User",
        password="robotics",
    ),
    debug_mode=True,
) as rws:

    # Left arm
    rapid_rob_l = "\
        MODULE MyModule \n\
            PROC my_proc() \n\
                MoveAbsJ [[0., -130.0, 30.0, 0.0, 40.0, 0.0],[135,9E+09,9E+09,9E+09,9E+09,9E+09]],v400,fine,tool0\WObj:=wobj0; \n\
                MoveAbsJ [[0., -130.0, 30.0, 0.0, 40.0, 0.0],[115,9E+09,9E+09,9E+09,9E+09,9E+09]],v400,fine,tool0\WObj:=wobj0; \n\
                MoveAbsJ [[0., -130.0, 30.0, 0.0, 40.0, 0.0],[135,9E+09,9E+09,9E+09,9E+09,9E+09]],v400,fine,tool0\WObj:=wobj0; \n\
            ENDPROC \n\
        ENDMODULE \n\
        "

    rapid_rob_r = "\
        MODULE MyModule \n\
            PROC my_proc() \n\
                MoveAbsJ [[0., -130.0, 30.0, 0.0, 40.0, 0.0],[-135,9E+09,9E+09,9E+09,9E+09,9E+09]],v400,fine,tool0\WObj:=wobj0; \n\
                MoveAbsJ [[0., -130.0, 30.0, 0.0, 40.0, 0.0],[-115,9E+09,9E+09,9E+09,9E+09,9E+09]],v400,fine,tool0\WObj:=wobj0; \n\
                MoveAbsJ [[0., -130.0, 30.0, 0.0, 40.0, 0.0],[-135,9E+09,9E+09,9E+09,9E+09,9E+09]],v400,fine,tool0\WObj:=wobj0; \n\
            ENDPROC \n\
        ENDMODULE \n\
        "

    # send and run the rapid program
    rws.rapid_stop_execution()
    rws.send_rapid_program(
        rapid_rob_l, module_name="MyModule", proc_name="my_proc", task_name="T_ROB_L"
    )

    rws.execute_rapid_program(
        rapid_rob_r,
        module_name="MyModule",
        proc_name="my_proc",
        unload_after=False,
        task_name="T_ROB_R",
    )
