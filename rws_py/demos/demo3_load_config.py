from rws_py.rws_config import RWSConfig
from rws_py.rws_py_client import RWSPy

# VC
# ip = "127.0.0.1"
# port = "80"

# RC
ip = "192.168.125.1"
port = ""

with RWSPy(RWSConfig(rw_version=6, ip=ip, port=port), debug_mode=True) as rws:

    # the content of the config file. Can be read from the disk also.
    fcont = '\
EIO:CFG_1.0:6:1::\n\
#\n\
EIO_SIGNAL:\n\
    -Name "CFREE_RUN_PATH" -SignalType "DI" -Access "All"\n\
    -Name "CFREE_LOAD_NEXT_PATH" -SignalType "DI" -Access "All"\n\
    -Name "CFREE_UNLOAD_PATH" -SignalType "DO" -Access "All"\n\
'

    # upload the file to the controller
    rws.files_upload_file_to_controller(path="$home", file_name="my_io_conf.cfg", content=fcont)

    # load the config
    rws.config_load(path="$home/my_io_conf.cfg", action_type="add", validate_first=True)
    rws.controller_restart()
