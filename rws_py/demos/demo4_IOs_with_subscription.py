from rws_py.rws_config import RWSConfig
from rws_py.rws_py_client import RWSPy

# VC
ip = "127.0.0.1"
port = "80"

# RC
# ip = "192.168.125.1"
# port = ""
import time


def io_callback(val):
    print(f"--------------------  Value of io changed to = {val}")


with RWSPy(RWSConfig(rw_version=6, ip=ip, port=port), debug_mode=True) as rws:

    # Get all io signals
    all_signals = rws.ios_get_signals()

    print("Controller has the following signals:")
    print("\n".join(map(str, all_signals)))

    # check if the signal defined in demo3_load_config is among the signals
    sig_name = "RUN_CFREE_ROUTINE"
    if sig_name not in all_signals:
        print("Run demo3_load_config first to define the signal.")

    # optionally you can subscribe to a signal to be notified when it changes.
    rws.subscribe_to_io(signal_name=sig_name, callback=io_callback, priority=2)

    # get the value of the signal
    val = int(rws.ios_get_signal_value(name=sig_name))
    print(f"Value of {sig_name} = {val}")

    # set the value of the signal. When the value is set the callback in the subscriber is called.
    rws.ios_set_signal_value(name=sig_name, value=1)

    time.sleep(0.5)

    rws.ios_set_signal_value(name=sig_name, value=0)

    time.sleep(0.5)

    rws.ios_set_signal_value(name=sig_name, value=0)
