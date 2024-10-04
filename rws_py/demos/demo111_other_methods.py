from rws_py.rws_config import RWSConfig
from rws_py.rws_py_client import RWSPy
from rws_py.types import ControllerState

with RWSPy(
    RWSConfig(
        rw_version=7,
        ip="127.0.0.1",
        port="80",
        user="Default User",
        password="robotics",
    ),
    debug_mode=True,
) as rws:

    # Switch motors on
    print(f"Controller state: {rws.controller_get_state()}")
    rws.controller_switch_motors(desired_state=ControllerState.motoron)
    print(f"Controller state: {rws.controller_get_state()}")

    # file operations
    f_path = "$HOME"
    f_dir = "my_folder"
    f_name = "my_file.modx"

    print(
        f"Befor create dir: Directories in the controller path {f_path}:\n{rws.files_list_dirs_in_controller_dir(f_path)}"
    )

    rws.files_create_directory_in_controller(f_path, f_dir)

    print(
        f"After create dir: Directories in the controller path {f_path}:\n\
            {rws.files_list_dirs_in_controller_dir(f_path)}"
    )

    content = "\
        MODULE MyModule \n\
            PROC my_proc() \n\
            ENDPROC \n\
        ENDMODULE \n\
        "
    print(
        f"Before create file: Files in the controller path {f_path}:\n\
            {rws.files_list_files_in_controller_dir(f_path)}"
    )
    rws.files_upload_file_to_controller(f_path + "/" + f_dir, f_name, content)
    print(
        f"After create file: Files in the controller path {f_path}:\n\
            {rws.files_list_files_in_controller_dir(f_path)}"
    )
    content = rws.files_get_file_content_from_controller(f_path + "/" + f_dir, f_name)
    print(f"\nReceived file content:\n{content}")

    print(f"\Rapid modules on the controller:\n{rws.rapid_list_modules()}")

    modlue_path = f"{f_path}/{f_dir}/{f_name}"
    if rws.rapid_load_module(module_path=modlue_path):
        print(f"Loaded rapid module {modlue_path}")

    # start/stop rapid exec
    rws.rapid_reset_pp_to_main()
    rws.rapid_set_pp_to_routine(module_name="MyModule", routine_name="my_proc")
    rws.rapid_start_execution()
    rws.rapid_stop_execution()

    if rws.rapid_unload_module(module_name="MyModule"):
        print(f"Unloaded rapid module {modlue_path}")

    rws.files_delete_file_in_controller(f_path + "/" + f_dir, f_name)
    rws.files_delete_directory_in_controller(f_path + "/" + f_dir)
    print(
        f"After delete dir: Directories in the controller path {f_path}:\n\
            {rws.files_list_dirs_in_controller_dir(f_path)}"
    )

    # import numpy as np

    # tcp_pos, tcp_quat, conf = interface.robot_forward_kinematics(
    #     rob_joints=[0, 0, 0, 0, 90 * np.pi / 180, 0]
    # )

    # rob_joints, ext_joints = interface.robot_inverse_kinematics(
    #     tcp_position=tcp_pos, tcp_orientation=tcp_quat, config=conf
    # )
