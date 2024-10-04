from rws_py.rws_config import RWSConfig
from rws_py.rws_py_client import RWSPy

# VC
# ip = "127.0.0.1"
# port = "80"

# RC
ip = "192.168.125.1"
port = ""

with RWSPy(
    RWSConfig(
        rw_version=6,
        ip=ip,
        port=port,
        user="Default User",
        password="robotics",
    ),
    debug_mode=False,
) as rws:

    # --------- dynamic part
    rapid = "\
        MODULE MyModule \n\
            PROC my_proc() \n\
                MoveAbsJ [[10,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v1500,fine,tool0\WObj:=wobj0; \n\
                MoveAbsJ [[-10,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v1500,fine,tool0\WObj:=wobj0; \n\
            ENDPROC \n\
        ENDMODULE \n\
        "
    # 1.128

    import time

    n_run = 4
    t0 = time.time()
    for i in range(n_run):
        rws.execute_rapid_program(
            rapid, module_name="MyModule", proc_name="my_proc", unload_after=True
        )

    t_toal = time.time() - t0
    # v400
    motion_time = 1.128
    # v1500
    motion_time = 0.5

    print(f"motion time arounk {motion_time*n_run}")
    print(f"Total time was  {t_toal}")  # 5.30
    print(f"Time per send  {int((t_toal-motion_time*n_run)/n_run*1000)} ms")  # 5.30
