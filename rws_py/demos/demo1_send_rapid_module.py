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

    content = "\
        MODULE MyModule \n\
            PERS num duration; \n\
            VAR clock myclock;\n\
            PROC my_proc() \n\
                ClkReset myclock;\n\
                ClkStart myclock;\n\
                MoveAbsJ [[10,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v400,fine,tool0\WObj:=wobj0; \n\
                MoveAbsJ [[-10,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v400,fine,tool0\WObj:=wobj0; \n\
                ClkStop myclock; \n\
                duration:=ClkRead(myclock);\n\
            ENDPROC \n\
        ENDMODULE \n\
        "

    # send and run the rapid program
    rws.rapid_stop_execution()
    if rws.execute_rapid_program(
        content, module_name="MyModule", proc_name="my_proc", unload_after=False
    ):

        # get rapid variable and print it
        duration = float(rws.rapid_get_symbol("MyModule", "duration"))
        print(f"\n\nMotion duration = {duration }\n\n")
