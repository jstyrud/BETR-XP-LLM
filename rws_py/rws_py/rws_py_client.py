import logging
import time
from typing import Callable, Dict, List, Optional

import numpy as np

from _version import __version__
from rws_py.rest_client import RESTClient
from rws_py.rws_config import RWSConfig
from rws_py.rws_subscriber import RWSSubscriber, SubscriptionInfo
from rws_py.types import ControllerOperationMode, ControllerState
from rws_py.utils_data_parse import join_file_path
from rws_py.utils_logging import configure_logging

logger = logging.getLogger(__name__)
configure_logging(logger)


class RWSPy:
    client: RESTClient

    is_rapid_running: bool = False
    controller_state: ControllerState = ControllerState.unknown
    controller_operation_mode: ControllerOperationMode = ControllerOperationMode.UNDEF
    mech_unit: str = ""
    task_name: str = ""
    n_joints: int = 6
    is_virtual_controller: bool = True
    debug_mode: bool = False
    _rws_request_block_loop_dt = 0.005  # [s]
    _timeout: float = 2.0
    _ctrl_error_list: Dict = {}
    _ctrl_is_restarting: bool = False

    def __init__(
        self,
        config: RWSConfig = RWSConfig(6),
        debug_mode: bool = True,
    ) -> None:
        self.cnf = config
        if debug_mode:
            logger.setLevel(logging.DEBUG)
        logger.info(config)

        self.client = RESTClient(config)
        if not self.client.login():
            exit()
        self.debug_mode = debug_mode
        self.initialize()

        self._subscriber = RWSSubscriber(
            self.client,
            logger_=logger,
        )
        self.subscribe(
            SubscriptionInfo(
                uri="/rw/rapid/execution;ctrlexecstate",
                title="ctrlexecstate",
                callback=self._callback_rapid_exec,
                priority=1,
            )
        )
        self.subscribe(
            SubscriptionInfo(
                uri="/rw/panel/opmode",
                title="opmode",
                callback=self._callback_ctrl_opmode,
                priority=1,
            )
        )
        self.subscribe(
            SubscriptionInfo(
                uri="/rw/panel/" + self.cnf.ctrstate,
                title="ctrlstate",
                callback=self._callback_ctrl_state,
                priority=1,
            )
        )

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self._subscriber.close()
        if not self._ctrl_is_restarting:
            self.release_mastership()
            self.client.logout()
        logger.info("Shutdown.")

    def initialize(self, print_ctrl_options=True) -> None:

        logger.info(f"rws-py version is {__version__}")

        self._get_mech_units()

        robot_type_resp = self.client.call_api("/rw/system/robottype", "GET")
        if len(robot_type_resp.data) != 0:
            robot_type = self.get_states(robot_type_resp.data)[0]["robot-type"]
            logger.info(f"\tRobot type:\t\t{robot_type}")

        # Info on controller operation mode
        self.controller_operation_mode = self.controller_get_operation_mode()
        logger.info(f"\tController mode:\t{self.controller_operation_mode}")

        # Info on controller state
        self.controller_state = self.controller_get_state()
        logger.info(f"\tController state:\t{self.controller_state}")

        # info on rapid execution
        rapid_exec_state, cycle = self.rapid_get_execution_status()
        if rapid_exec_state == "running":
            self.is_rapid_running = True
        else:
            self.is_rapid_running = False
        logger.info(f"\tRapid execution:\t{rapid_exec_state}")
        logger.info(f"\tRapid cycle:\t\t{cycle}")

        ctr_servs_resp = self.client.call_api("/ctrl", "GET")
        if self.cnf.rw_version == 7:
            ctr_servs_data = ctr_servs_resp.data["_embedded"]["resources"]
        else:
            ctr_servs_data = ctr_servs_resp.data["_embedded"]["_state"]

        ctr_servs = []
        for serv in ctr_servs_data:
            ctr_servs.extend([serv["_title"]])

        if "clock" in ctr_servs:
            resp = self.client.call_api("/ctrl/clock", "GET")
            if len(resp.data) != 0:
                logger.info(f"\tController clock:\t{self.get_states(resp.data)[0]['datetime']}")

        if "system" in ctr_servs:
            resp = self.client.call_api("/ctrl/system", "GET")
            if len(resp.data) != 0:
                systems = self.get_states(resp.data)
                for syst in systems:
                    logger.info(f"\tController system :\t{syst['_title']}")

        if "identity" in ctr_servs:
            ctr_iden_resp = self.client.call_api("/ctrl/identity", "GET")
            if len(ctr_iden_resp.data) != 0:
                ctr_iden = self.get_states(ctr_iden_resp.data)[0]

                if "ctrl-name" in ctr_iden:
                    logger.info(f"\tController name:\t{ctr_iden['ctrl-name']}")

                if "ctrl-type" in ctr_iden:
                    logger.info(f"\tController type:\t{ctr_iden['ctrl-type']}")
                    if ctr_iden["ctrl-type"] == "Virtual Controller":
                        self.is_virtual_controller = True

        # Check user grants
        user_grants_resp = self.client.call_api("/users/grants", "GET")
        if len(user_grants_resp.data) != 0:
            user_grants_data = self.get_states(user_grants_resp.data)
            user_grants = []
            for data in user_grants_data:
                user_grants.append(data["_title"])

            if "UAS_RAPID_EXECUTE" not in user_grants:
                logger.warning("\tUser does not have rapid execution grant. User grants are:")
                logger.warning("\n".join(map(str, user_grants)))

        if print_ctrl_options:
            options = self.get_system_options()
            logger.info("\tOptions:")
            for opt in options:
                logger.info(f"\t\t{opt}")

        # some useful warnings
        if self.controller_operation_mode != ControllerOperationMode.AUTO:
            logger.warning(
                f"Controller is not in auto mode. Current mode is {self.controller_operation_mode}"
            )
        if (
            self.controller_state != ControllerState.motoron
            and self.controller_state != ControllerState.motoroff
        ):
            logger.warning(f"Controller state is {self.controller_state}")

        self._get_controller_error_list()

        logger.info("Initialized.")

    def _get_controller_error_list(self) -> None:
        """Retrieves the lists of controller errors."""
        resp = self.client.call_api("/rw/retcode", "GET")

        errors_dict = resp.data["html"]["body"]["div"]["ul"]["li"]
        self._ctrl_error_list = {}
        for err in errors_dict:
            temp_error = {}
            for element in err["span"]:
                temp_error[element["@class"]] = element["#text"]
            self._ctrl_error_list[int(temp_error["code"])] = temp_error

    def _find_controller_error(self, msg: str) -> str:
        """Searches in the text to find error code. and prints name and description if available

        Args:
            msg (str): _description_
        """
        init = msg.find(" code:")
        if init != -1:
            code = int(msg[init + 6 : msg.find("icode:")])
            error = self._ctrl_error_list[code]
            return f"Error {error['name']}: {error['description']}"
        else:
            init = msg.find('"code"')
            if init != -1:
                code = int(msg[init + 7 : msg.find("</", init)])
                error = self._ctrl_error_list[code]
                return f"Error {error['name']}: {error['description']}"
        return ""

    def _callback_rapid_exec(self, val):
        value = val["ctrlexecstate"]
        if value == "running":
            self.is_rapid_running = True
        else:
            self.is_rapid_running = False
        if self.debug_mode:
            logger.debug(f"Rapid execution state chagned to {value}")

    def _callback_ctrl_state(self, val):
        value = val["ctrlstate"]
        self.controller_state = ControllerState[value]
        if self.debug_mode:
            logger.debug(f"Controller state chagned to {value}")

    def _callback_ctrl_opmode(self, val):
        value = val["opmode"]
        self.controller_operation_mode = ControllerOperationMode[value]
        if self.debug_mode:
            logger.debug(f"Controller operation mode chagned to {value}")

    def _add_action_to_url(self, url: str, action: str) -> str:
        if self.cnf.rw_version == 7:
            return url + "/" + action
        else:
            return url + "?action=" + action

    def subscribe(self, sub_info: SubscriptionInfo):
        self._subscriber.subscribe(sub_info)

    def subscribe_to_io(self, signal_name: str, callback: callable, priority: int = 1):
        self._subscriber.subscribe(
            SubscriptionInfo(
                f"/rw/iosystem/signals/{signal_name};state",
                title=signal_name,
                callback=callback,
                priority=priority,
            )
        )

    def subscribe_to_var(
        self,
        module_name: str,
        variable_name: str,
        callback: Callable,
        priority: int = 1,
        task_name: str = "T_ROB1",
    ):

        symbol_url = f"RAPID/{task_name}/{module_name}/{variable_name}"

        url = f"/rw/rapid/symbol/{symbol_url}/data"
        if self.cnf.rw_version == 6:
            url = f"/rw/rapid/symbol/data/{symbol_url};value"

        self._subscriber.subscribe(
            SubscriptionInfo(
                url,
                title=symbol_url,
                callback=callback,
                priority=priority,
            )
        )

    def get_mastership(self, domain="rapid") -> bool:
        """Requests mastership. Controller must be in auto mode.

        Returns:
            bool: True if successful, False otherwise.
        """
        if self.controller_operation_mode != ControllerOperationMode.AUTO:
            logger.error(
                f"Mastership can be requested only in auto mode. current operation mode is \
                    {self.controller_operation_mode}"
            )
            return False

        if self.cnf.rw_version == 7:
            dom = "edit" if domain == "rapid" else domain
            url = f"/rw/mastership/{dom}"
        else:
            url = f"/rw/mastership/{domain}"

        resp = self.client.call_api(self._add_action_to_url(url, "request"), "POST")

        if resp.status_code == 204:
            logger.debug("Got mastership")
            return True
        elif resp.status_code == 403:
            logger.error(
                "Failed to get mastership. It is held by someone else. If you are not sure by who,\
                     just switch the control mode to manual and back to auto mode."
            )
        else:
            logger.error(f"Failed to get mastership. {resp}")
        return False

    def release_mastership(self):

        resp = self.client.call_api(self._add_action_to_url("/rw/mastership", "release"), "POST")

        if resp.status_code == 204:
            logger.debug("Released mastership")
            return True
        else:
            logger.error(f"Failed to release mastership. {resp}")
        return False

    def controller_get_operation_mode(self) -> Optional[ControllerOperationMode]:
        """ Get current controller operation mode e.g. manual or auto.\n
        
        Returns:
            Optional[ControllerOperationMode]: The operation mode can be :{INIT | AUTO_CH | \
                MANF_CH | MANR | MANF | AUTO | UNDEF}. 
        """
        try:
            resp = self.get_states(self.client.call_api("/rw/panel/opmode", "GET").data)
            return ControllerOperationMode[resp[0]["opmode"]]
        except ValueError as err:
            logger.error(f"Failed to get operation mode - Reason: {err}")
        return None

    def controller_get_state(self) -> Optional[ControllerState]:
        """Gets current controller state, that can be any of the following:
            -init- The robot is starting up. It will shift to state motors off when it has started.
            -motoroff- The robot is in a standby state where there is no power to the robot's motors. The state has to be shifted to motors on before the robot can move.\n
            -motoron- The robot is ready to move, either by jogging or by running programs.
            -guardstop- The robot is stopped because the safety runchain is opened. For instance, a door to the robot's cell might be open.\n
            -emergencystop- The robot is stopped because emergency stop was activated.
            -emergencystopreset- The robot is ready to leave emergency stop state. The emergency stop is no longer activated, but the state transition isn't yet confirmed.\n
            -sysfail- The robot is in a system failure state. Restart required.

        Returns:
            Optional[ControllerState]: controller state e.g. 'motoron', 'motoroff'
        """
        try:
            resp = self.client.call_api(f"/rw/panel/{self.cnf.ctrstate}", "GET")
            return ControllerState[self.get_states(resp.data)[0]["ctrlstate"]]
        except Exception as err:
            logger.error(f"Failed to get controller state - Reason: {err}")
        return None

    def controller_switch_motors(
        self,
        desired_state: ControllerState = ControllerState.motoron,
        block_until_confirmed: bool = True,
    ):
        """
        Get current controller operation mode e.g. manual or auto

        Returns
        -------
        bool
            True on success, False otherwise.
        """
        try:
            if self.controller_state == desired_state:
                return True

            if desired_state == ControllerState.motoron:
                if self.controller_state != ControllerState.motoroff:
                    logger.warning(f"Cannot switch motors on in state {self.controller_state}")
                    return False
                state = "motoron"
            elif desired_state == ControllerState.motoroff:
                state = "motoroff"
            else:
                raise ValueError(
                    f"Can switch motors only to on and off states and not {desired_state}"
                )

            if self.cnf.rw_version == 7:
                if state == "motoroff":
                    logger.warning("No API is available to turn motors off in RWS2.")
                    return
                resp = self.client.call_api(
                    f"/rw/panel/{self.cnf.ctrstate}/keyless-{state}", "POST"
                )
            else:
                resp = self.client.call_api(
                    "/rw/panel/ctrlstate?action=setctrlstate",
                    "POST",
                    params={"ctrl-state": state},
                )

            if resp.status_code != 204:
                logger.error(
                    f"Failed to switch motors state. status_code = {resp.status_code} {resp.reason}"
                )
                return False
            if block_until_confirmed:
                init_time = time.time()
                while self.controller_state != desired_state:
                    time.sleep(self._rws_request_block_loop_dt)
                    if init_time + self._timeout < time.time():
                        logger.error("Timed out on controller switch motors")
                        return False
                if self.debug_mode:
                    logger.debug(f"Switched motors to {desired_state}")
                return True
            else:
                return True

        except Exception as err:
            logger.error(f"Failed to turn motors on - Reason: {err}")
        return False

    def controller_restart(self) -> None:
        """Sends a restart request to the controller."""

        if not self.get_mastership():
            return
        #    restart-mode = restart modes are {restart | istart | pstart | bstart} Required
        params = {"restart-mode": "restart"}
        url = self._add_action_to_url("/rw/panel", "restart")
        resp = self.client.call_api(url, "POST", params=params)

        if resp.status_code != 204:
            logger.error("Failed to restart")
            return

        self._ctrl_is_restarting = True
        logger.warning("Controller is restarting. You need tp rerun this client!")

    def get_system_options(self) -> List[str]:
        """Gets installed system options returns a list of strings.

        Returns:
            List[str]: List of options
        """
        resp = self.get_states(self.client.call_api("/rw/system/options", "GET").data)

        options = []
        for opt in resp:
            options += [opt["option"]]
        return options

    def _get_mech_units(self):
        try:
            resp = self.client.call_api("/rw/motionsystem/mechunits", "GET")
            if len(resp.data) == 0:
                return
            if "resources" in resp.data["_embedded"]:  # RWS2
                mechunits = resp.data["_embedded"]["resources"]
            elif "_state" in resp.data["_embedded"]:  # RWS1
                mechunits = resp.data["_embedded"]["_state"]
            else:
                return
        except Exception as err:
            logger.error(f"Failed to get mechunits - Reason: {err}")
            return

        for mech in mechunits:
            if mech["_title"] in ["ROB_1_7", "ROB_R_7", "ROB_L_7"]:
                # Special cases for YuMi and SAY
                continue

            mech_unit = mech["_title"]
            try:
                mechunit_info_resp = self.client.call_api(
                    f"/rw/motionsystem/mechunits/{mech_unit}", "GET"
                )
                if len(mechunit_info_resp.data) == 0:
                    return
                if "state" in mechunit_info_resp.data:  # RWS2
                    mechunit_info = mechunit_info_resp.data["state"][0]
                elif "_embedded" in resp.data:  # RWS2
                    mechunit_info = mechunit_info_resp.data["_embedded"]["_state"][0]
                else:
                    return

                path_sup_info_resp = self.client.call_api(
                    f"/rw/motionsystem/mechunits/{mech_unit}/pathsupervision",
                    "GET",
                )
                path_sup_info = None
                if len(path_sup_info_resp.data) != 0:
                    path_sup_info = path_sup_info_resp.data["state"][0]

                lead_thru_info_resp = self.client.call_api(
                    f"/rw/motionsystem/mechunits/{mech_unit}/lead-through", "GET"
                )
                lead_thru_info = None
                if len(lead_thru_info_resp.data) != 0:
                    lead_thru_info = lead_thru_info_resp.data["state"][0]

            except Exception as err:
                logger.error(f"Failed to get robot base frame of {mech_unit}  - Reason: {err}")
                return

            if "task-name" in mechunit_info:
                task_name = mechunit_info["task-name"]
            if "task" in mechunit_info:
                task_name = mechunit_info["task"]

            self.n_joints = mechunit_info["axes-total"]
            logger.info(f"Found mechanical unit {mech_unit}:")
            logger.info(f"\tTask name:\t\t{task_name}")
            logger.info(f"\tNum joints:\t\t{self.n_joints }")
            logger.info(f"\tUnit type:\t\t{mechunit_info['type']}")
            logger.info(f"\tToolname:\t\t{mechunit_info['tool-name']}")
            logger.info(f"\tWobj-name:\t\t{mechunit_info['wobj-name']}")
            logger.info(f"\tPayload name:\t\t{mechunit_info['payload-name']}")
            if path_sup_info is not None:
                logger.info(
                    f"\tPath supervision:\t{path_sup_info['mode']}, level: {path_sup_info['level']}"
                )
            if lead_thru_info is not None:
                logger.info(f"\tLeadThru :\t{lead_thru_info['status']}")
                np.set_printoptions(suppress=True)

            jpos = self.robot_get_joint_positions(mech_unit)
            logger.info(f"\tCurrent Joint pos:\t{np.round(jpos[0], 1)} {np.round(jpos[1], 1)}")

            cart = self.robot_get_cartesian(mech_unit)
            logger.info(f"\tCurrent Cartesian:\t{np.around(cart[0],1)} {cart[1]}")

    def get_states(self, data: Dict):
        if self.cnf.rw_version == 7:
            return data["state"]
        elif self.cnf.rw_version == 6:
            return data["_embedded"]["_state"]
        else:
            raise ValueError

    def rapid_get_execution_status(self):
        """
        Get current rapid execution status and cycle
        status: stopped, running
        cycle: e.g. forever
        """
        resp = self.get_states(self.client.call_api("/rw/rapid/execution", "GET").data)[0]
        return resp["ctrlexecstate"], resp["cycle"]

    def rapid_get_symbol(
        self, module_name: str, variable_name: str, task_name: str = "T_ROB1"
    ) -> str:

        symbol_url = f"RAPID/{task_name}/{module_name}/{variable_name}"

        url = f"/rw/rapid/symbol/{symbol_url}/data"
        if self.cnf.rw_version == 6:
            url = f"/rw/rapid/symbol/data/{symbol_url}"

        resp = self.client.call_api(url, "GET")
        if resp.status_code == 404:
            logger.error(f"Rapid symbol {symbol_url} was not found.")
            raise ValueError(f"Rapid symbol {symbol_url} was not found.")

        if resp.status_code != 200:
            raise ValueError("Failed in getting rapid symbol")

        val = self.get_states(resp.data)[0]["value"]
        if self.debug_mode:
            logger.debug(f"Rapid symbol {variable_name} was received with value = {val}")

        return val

    def rapid_set_symbol(
        self, module_name: str, variable_name: str, value, task_name: str = "T_ROB1"
    ) -> None:

        symbol_url = f"RAPID/{task_name}/{module_name}/{variable_name}"

        url = f"/rw/rapid/symbol/{symbol_url}/data"
        if self.cnf.rw_version == 6:
            url = f"/rw/rapid/symbol/data/{symbol_url}?action=set"

        if not self.get_mastership():
            logger.error(f"Failed to set variable {url} because mastership was not received.")

        resp = self.client.call_api(url, "POST", params={"value": value})
        self.release_mastership()

        if resp.status_code == 404:
            logger.error(f"Rapid symbol {symbol_url} was not found.")
            raise ValueError(f"Rapid symbol {symbol_url} was not found.")

        if resp.status_code != 204:
            err_str = self._find_controller_error(resp.content.decode("utf-8"))
            logger.error(
                f"Failed in setting rapid symbol {symbol_url}. text: {resp.reason}. {err_str}"
            )
            raise ValueError("Failed in setting rapid symbol")

        if self.debug_mode:
            logger.debug(f"Rapid symbol {url} set to {value}")

    def rapid_list_modules(
        self, mod_type: Optional[str] = "ProgMod", task_name: str = "T_ROB1"
    ) -> Optional[List[str]]:
        """Gets the list of all rapid modules of specified type.

        Args:
            mod_type (Optional[str], optional): Type of module 'ProgMod' or 'SysMod'. \
                Defaults to "ProgMod".
            task_name (Optional[str], optional): RAPID task name. Defaults to T_ROB1.

        Raises:
            ValueError: Failed to get list of modules

        Returns:
            Optional[List[str]]: a list of modules names as strings.

        """

        if self.cnf.rw_version == 7:
            resp = self.client.call_api(f"/rw/rapid/tasks/{task_name}/modules", "GET")
        else:
            resp = self.client.call_api(f"/rw/rapid/modules?task={task_name}", "GET")

        if resp.status_code != 200:
            logger.error(f"Failed to get list of modules of /{task_name} - reason: {resp.reason}")
            raise ValueError(f"Failed to get list of modules - reason: {resp.reason}")

        data = self.get_states(resp.data)
        modules = []
        for mod in data:
            if mod["type"] == mod_type:
                modules.append(mod["name"])
        return modules

    def rapid_load_module(
        self,
        module_path: str,
        task_name: str = "T_ROB1",
        get_mastership: bool = True,
    ) -> bool:
        """
        Loads a rapid module into a task

        Parameters
        ----------
        modlue_path: str
            The path of the module file on the controller
        task_name : str, optional
            RAPID task name e.g.: T_ROB1
        get_mastership: Wehther to request (and relseas) master ship. \n
                        This operation needs mastership, if get_mastership \n
                        is set to false it is meant that the mastership has \n
                        already been aquired and is not done here to save some time.

        Returns
        -------
        bool
            True on success, False otherwise.
        """
        if get_mastership:
            self.get_mastership()

        url = self._add_action_to_url(f"/rw/rapid/tasks/{task_name}", "loadmod")

        params = {"modulepath": module_path, "replace": "true"}

        resp = self.client.call_api(url, "POST", params=params)

        if get_mastership:
            self.release_mastership()

        if resp.status_code not in [200, 204]:
            err_str = self._find_controller_error(resp.content.decode("utf-8"))
            logger.error(f"Failed to load module {module_path} - reason: {resp.reason}. {err_str}")

            return False

        if self.debug_mode:
            logger.debug(f"Rapid module {module_path} loaded")
        return True

    def rapid_unload_module(self, module_name, task_name: str = "T_ROB1"):
        """
        Unloads a rapid module from a task

        Parameters
        ----------
        module_name: str
            The name of the module
        task_name : str, optional
            RAPID task name e.g.: T_ROB1

        Returns
        -------
        bool
            True on success, False otherwise.
        """

        if not self.get_mastership():
            return False

        url = self._add_action_to_url(f"/rw/rapid/tasks/{task_name}", "unloadmod")

        params = {"module": module_name}

        resp = self.client.call_api(url, "POST", params=params)

        if not self.release_mastership():
            return False

        if resp.status_code != 204:
            logger.error(
                f"Failed to unload module {module_name} from /{task_name} - reason: {resp.reason}"
            )
            return False

        if self.debug_mode:
            logger.debug("Rapid module unloaded")

        return True

    def rapid_reset_pp_to_main(self, get_mastership=True):

        if get_mastership:
            self.get_mastership()

        url = self._add_action_to_url("/rw/rapid/execution", "resetpp")

        resp = self.client.call_api(url, "POST")

        if get_mastership:
            self.release_mastership()

        if resp.status_code not in [200, 204]:
            logger.error(f"Reset PP to mai - reason: {resp.reason}")
            return False

        if self.debug_mode:
            logger.debug("Reset PP to main")
        return True

    def rapid_set_pp_to_routine(
        self, routine_name, module_name, task_name: str = "T_ROB1", get_mastership=True
    ):
        """
        Sets the program pointer to a rapid routine

        Parameters
        ----------
        routine_name: str
            The name of the routine
        task_name : str, optional
            RAPID task name e.g.: T_ROB1
        get_mastership: Wehther to request (and relseas) master ship. \n
                        This operation needs mastership, if get_mastership \n
                        is set to false it is meant that the mastership has \n
                        already been aquired and is not done here to save some time.
        Returns
        -------
        bool
            True on success, False otherwise.
        """

        if get_mastership:
            if not self.get_mastership():
                return False

        if self.cnf.rw_version == 7:
            url = f"/rw/rapid/tasks/{task_name}/pcp/routine"
        else:
            url = f"/rw/rapid/tasks/{task_name}/pcp?action=set-pp-routine"

        params = {"module": module_name, "routine": routine_name, "userlevel": "false"}

        resp = self.client.call_api(url, "POST", params=params)

        if get_mastership:
            if not self.release_mastership():
                return False

        if resp.status_code not in [200, 204]:
            err_str = self._find_controller_error(resp.content.decode("utf-8"))
            logger.error(
                f"Failed to set PP to {module_name}:{routine_name} - reason: {resp.reason}. {err_str}"
            )

            return False

        if self.debug_mode:
            logger.debug(f"PP was set to routine {routine_name} in {task_name}")

        return True

    def rapid_start_execution(self, cycle="once", block_until_confirmed=True) -> bool:
        """
        Start the execution of rapid.

        Parameters
        ----------
        cycle: str
            {forever,  asis,  once}

        Returns
        -------
        bool
            True on success, False otherwise.
        """
        if cycle not in ["forever", "asis", "once"]:
            logger.error(
                "Failed to start Rapid execution - Reason: cycle  must be one of \
                ['forever', 'asis', 'once']"
            )

        if self.is_rapid_running:
            if self.debug_mode:
                logger.debug("Rapid execution is already running. Request ignored.")
            return True

        if self.controller_state != ControllerState.motoron:
            logger.error(
                f"Cannot start RAPID execution when motors are not on. Current state is {self.controller_state}"
            )
            return False
        params = {
            "regain": "continue",
            "execmode": "continue",
            "cycle": cycle,
            "condition": "none",
            "stopatbp": "disabled",
            "alltaskbytsp": "false",
        }

        url = self._add_action_to_url("/rw/rapid/execution", "start")

        resp = self.client.call_api(url, "POST", params=params)

        if resp.status_code not in [200, 204]:
            err_str = self._find_controller_error(resp.content.decode("utf-8"))
            logger.error(f"Failed to start Rapid execution  - reason: {resp.reason}. {err_str}")
            return False

        if self.debug_mode:
            logger.debug("Rapid execution start request sent.")

        if block_until_confirmed:
            init_time = time.time()
            while not self.is_rapid_running:
                time.sleep(self._rws_request_block_loop_dt)
                if init_time + self._timeout < time.time():
                    logger.error("Timed out on rapid_start_execution")
                    return False
        logger.info("Rapid execution started.")
        return True

    def rapid_stop_execution(self) -> bool:
        """
        Stop the execution of rapid
        """
        if not self.is_rapid_running:
            if self.debug_mode:
                logger.debug("Rapid execution is already stopped. Request ignored.")
            return True

        url = self._add_action_to_url("/rw/rapid/execution", "stop")

        resp = self.client.call_api(url, "POST")
        if self.debug_mode:
            logger.debug("Rapid execution stop request sent.")

        if resp.status_code not in [200, 204]:
            logger.error(f"Failed to stop Rapid execution  - reason: {resp.reason}")
            return False

        logger.info("Stopped RAPID execution.")
        return True

    def robot_get_joint_positions(self, mech_unit: str = "") -> List[List[float]]:
        """
        Get the current joint positions of the robot
        """

        url = f"/rw/motionsystem/mechunits/{mech_unit}/jointtarget"

        resp = self.client.call_api(url, "GET")

        if resp.status_code != 200:
            logger.error(f"Failed to get JointTarget of {mech_unit} - reason: {resp.reason}")
            raise ValueError(f"Failed to get JointTarget - reason: {resp.reason}")

        d = self.get_states(resp.data)[0]

        return [
            [float(d[f"rax_{i+1}"]) for i in range(6)],
            [float(d[f"eax_{i}"]) for i in ["a", "b", "c", "d", "e", "f"]],
        ]

    def robot_get_cartesian(self, mech_unit=None):
        """
        Get the current Cartesian pose of the robot
        """

        url = f"/rw/motionsystem/mechunits/{mech_unit}/cartesian"

        resp = self.client.call_api(url, "GET")

        if resp.status_code != 200:
            logger.error(f"Failed to get the Cartesian pose of {mech_unit} - reason: {resp.reason}")
            raise ValueError(f"Failed to get Cartesian pose - reason: {resp.reason}")

        data = self.get_states(resp.data)[0]
        if self.cnf.rw_version == 7:
            cfg_str = "cf"
        else:
            cfg_str = "j"

        return [
            [float(data[i]) for i in ["x", "y", "z", "q1", "q2", "q3", "q4"]],
            [int(data[f"{cfg_str}{i}"]) for i in ["1", "4", "6", "x"]],
        ]

    def files_list_files_in_controller_dir(self, path) -> List[str]:

        files = self._files_get_dir_content(path)

        if path[-1] == "/":
            path = path[:-1]

        files_path = []
        for file in files:
            if file["_type"] == "fs-file":
                files_path.append(file["_title"].lower())
        return files_path

    def files_list_dirs_in_controller_dir(self, path) -> List[str]:

        files = self._files_get_dir_content(path)

        if path[-1] == "/":
            path = path[:-1]

        files_path = []
        for file in files:
            if file["_type"] == "fs-dir":
                files_path.append(file["_title"].lower())
        return files_path

    def _files_get_dir_content(self, path) -> Dict:

        if path[0] not in ["/", "$"]:
            raise ValueError("Failed to get directory. Path must start with / or $")

        resp = self.client.call_api(f"/fileservice/{path}", "GET")

        if resp.status_code != 200:
            logger.error(f"Failed to get the content of {path} - reason: {resp.reason}")
            raise ValueError(f"Failed to get directory content - reason: {resp.reason}")

        if self.cnf.rw_version == 7:
            return resp.data["_embedded"]["resources"]
        elif self.cnf.rw_version == 6:
            return resp.data["_embedded"]["_state"]

        return {}

    def files_create_directory_in_controller(self, path: str, dir_name: str) -> None:

        if path[0] not in ["/", "$"]:
            raise ValueError("Failed to create directory. Path must start with / or $")

        params = {"fs-newname": dir_name}

        if self.cnf.rw_version == 6:
            params["fs-action"] = "create"
            url = f"/fileservice/{path}"
        else:
            url = f"/fileservice/{path}/create"

        resp = self.client.call_api(url, "POST", params=params)

        if resp.status_code != 201:
            if resp.status_code == 409 or dir_name in self.files_list_dirs_in_controller_dir(path):
                logger.warning(f"Directory {dir_name} already exists in {path}")
            else:
                raise ValueError(f"Failed to create dir {dir_name} in {path}. Reason {resp.reason}")

        if self.debug_mode:
            logger.debug(f"Created directory {dir_name} in {path}")

    def files_delete_directory_in_controller(self, path: str) -> None:

        if path[0] not in ["/", "$"]:
            raise ValueError("Failed to delete directory. Path must start with / or $")

        resp = self.client.call_api(f"/fileservice/{path}", "DELETE")

        if resp.status_code == 404:
            logger.warning(f"Directory {path} does not exists. Nothing was deleted.")
            return
        if resp.status_code != 204:
            raise ValueError(f"Failed to delete dir {path}. Reason {resp.reason}")

        if self.debug_mode:
            logger.debug(f"Deleted directory {path}")

    def files_upload_file_to_controller(self, path: str, file_name: str, content: str) -> None:
        """ Uploads content to a file. Creates the file if it doesn't exists. other wise will \
            overwrite the content to the already existing file.

        Args:
            path (str): _description_
            file_name (str): _description_
            content (str): _description_

        Raises:
            ValueError: _description_
        """
        file_path = join_file_path(path, file_name)
        resp = self.client.call_api(f"/fileservice/{file_path}", "PUT", params=content)

        if resp.status_code not in [200, 201]:
            raise ValueError(f"Failed to create file {file_path}. Reason {resp.reason}")

        if self.debug_mode:
            logger.debug(f"Uploaded file {file_path}")

    def files_get_file_content_from_controller(self, path: str, file_name: str) -> str:
        file_path = join_file_path(path, file_name)
        resp = self.client.call_api(f"/fileservice/{file_path}", "GET")

        if resp.status_code != 200:
            raise ValueError(f"Failed to get file {file_path}. Reason {resp.reason}")

        if self.debug_mode:
            logger.debug(f"Retrieved file {file_path} from the controller.")

        return resp.data

    def files_delete_file_in_controller(self, path: str, file_name: str):

        file_path = join_file_path(path, file_name)
        resp = self.client.call_api(f"/fileservice/{file_path}", "DELETE")

        if resp.status_code == 404:
            logger.warning(f"File {file_path} does not exists. Nothing was deleted.")
            return
        if resp.status_code != 204:
            raise ValueError(f"Failed to delete dir {path}. Reason {resp.reason}")

        if self.debug_mode:
            logger.debug(f"Deleted {file_path}")

    def config_load(
        self, path: str, action_type: str = "replace", validate_first: bool = True
    ) -> bool:
        """Loads a config file to the controller. Restart is needed to take effect afterwards.

        Args:
            path (str): The path to the config file on the controller.
            action_type (str, optional): action-type=[add | replace | add-with-reset]. Defaults to\
                     "replace".
                add: Load a configuration file as external, i.e., instances will not be \
                    write-protected. If any instance in the file already exists, the file \
                    will not be loaded and an error code is returned.
                replace: As add, but the cfg-domain is reset before loading
                add-with-reset: Add the instances in the given file to the database. In case of \
                    instance name conflicts, the new instances replaces the existing.
            validate_first (bool, optional): Whether a validation request should be sent first to \
                see if the conifg contains errors.. Defaults to True.

        Returns:
            bool: True upon success. False otherwise.
        """

        supported_actions = ["add", "replace", "add-with-reset"]
        if action_type not in supported_actions:
            logger.error(
                f"Action type {action_type} is not supported. Supported actions are {supported_actions}"
            )
            return False
        if validate_first:
            if not self.config_validate(path, action_type):
                logger.error("Errors in the config file. Nothing loaded.")
                return False

        params = {"action-type": action_type, "filepath": path}

        url = self._add_action_to_url("/rw/cfg", "load")

        resp = self.client.call_api(url, "POST", params=params)
        if resp.status_code not in [200, 204]:
            logger.error("Failed to load the config file.")
            return False

        logger.info(f"Loaded config file {path}.")
        logger.warning("Changes will take effect after controller restart")
        return True

    def config_validate(self, path: str, action_type: str = "replace") -> bool:
        """Checks if a config file contains errors. No more details are provided at the moment.

        Args:
            path (str): The path to the config file on the controller.
            action_type (str, optional): action-type=[add | replace | add-with-reset]. Defaults to\
                     "replace".
                add: Load a configuration file as external, i.e., instances will not be \
                    write-protected. If any instance in the file already exists, the file \
                    will not be loaded and an error code is returned.
                replace: As add, but the cfg-domain is reset before loading
                add-with-reset: Add the instances in the given file to the database. In case of \
                    instance name conflicts, the new instances replaces the existing.
        Returns:
            bool: True upon success. False otherwise.
        """

        supported_actions = ["add", "replace", "add-with-reset"]
        if action_type not in supported_actions:
            logger.error(
                f"Action type {action_type} is not supported. Supported actions are {supported_actions}"
            )
            return False

        params = {"action-type": action_type, "filepath": path}

        url = self._add_action_to_url("/rw/cfg", "validate")

        resp = self.client.call_api(url, "POST", params=params)

        if resp.status_code != 200:
            err_str = self._find_controller_error(resp.content.decode("utf-8"))
            logger.error(f"Validation failed. Reason {resp.reason}. {err_str}")
            return False
        return True

    def ios_get_networks(self) -> Optional[List[str]]:
        """Returns a list of IO network resources defined in the robot controller.

        Returns:
            Optional[List[str]]: List of IO network names.
        """
        get_signals_resp = self.client.call_api("/rw/iosystem/networks", "GET")
        if len(get_signals_resp.data) == 0:
            return None

        networkss_resp = self.get_states(get_signals_resp.data)
        networks = []
        for network in networkss_resp:
            networks.append(network["name"])
        return networks

    def ios_get_signals(self) -> Dict:
        """Retrieves a list of IO signal resources defined in the robot controller.

        Returns:
            Dict: [signal-name: Dict]
        """
        get_signals_resp = self.client.call_api("/rw/iosystem/signals", "GET")
        if len(get_signals_resp.data) == 0:
            return None
        signals_resp = self.get_states(get_signals_resp.data)
        signals = {}
        for signal in signals_resp:
            signals[signal["name"]] = {
                "title": signal["_title"],
                "type": signal["type"],
                "category": signal["category"],
                "lvalue": signal["lvalue"],
                "lstate": signal["lstate"],
            }
        return signals

    def ios_get_signal(self, name: str) -> Optional[Dict]:
        """Retrieves the details of a signal.

        Args:
            name (str): the signal name.

        Returns:
            Dict: signal details including value and state.s
        """

        url = f"/rw/iosystem/signals/{name}"
        get_signal_resp = self.client.call_api(url, "GET")

        if get_signal_resp.status_code == 404:
            logger.error(f"Signal {name} not found.")
            return None

        if len(get_signal_resp.data) == 0:
            return None

        signal_data = self.get_states(get_signal_resp.data)[0]
        return {
            "name": signal_data["name"],
            "type": signal_data["type"],
            "category": signal_data["category"],
            "lvalue": signal_data["lvalue"],
            "lstate": signal_data["lstate"],
        }

    def ios_get_signal_value(self, name: str) -> str:
        """Retrieves the lvalue of a signal.

        Args:
            name (str): The name of the signal

        Returns:
            str: Value as a string.
        """
        sig_info = self.ios_get_signal(name)
        if sig_info is not None:
            return sig_info["lvalue"]

    def ios_set_signal_value(self, name: str, value) -> str:
        """Sets the lvalue of a signal.

        Args:
            name (str): The name of the signal

        Returns:
            bool: True upon success. False otherwise.
        """
        url = self._add_action_to_url(f"/rw/iosystem/signals/{name}", "set")

        params = {"lvalue": value}

        set_signal_resp = self.client.call_api(url, "POST", params=params)

        if set_signal_resp.status_code != 204:
            logger.error(f"Failed to set signal {name}")
            return False

        if self.debug_mode:
            logger.debug(f"Set signal {name} with value {value}")

        return True

    def send_rapid_program(
        self,
        prog,
        module_name,
        proc_name,
        task_name: str = "T_ROB1",
        create_before=True,
    ):
        """
        Creates a rapid module on the controller, loads it, and sets the pp to it

        Parameters
        ----------
        prog: str
            RAPID code as string
        module_nam: str, optional
            The name of the generated modul
        create_before: bool, optional
            When False the directory and the module file are not created, because they already \n
            exist on the controller. Used when saving a few milliseconds is of interest. \n
            If the directory or the file don't exists this method will fail.

        Returns
        -------
        bool
            True on success, False otherwise.
        """

        f_path = "$HOME"
        f_dir = "temp_dir"
        f_name = "temp.modx"

        if not self.controller_switch_motors(ControllerState.motoron):
            return False
        if create_before:
            self.files_create_directory_in_controller(path=f_path, dir_name=f_dir)
        self.files_upload_file_to_controller(f_path + "/" + f_dir, f_name, prog)

        if not self.get_mastership():
            return False
        if not self.rapid_load_module(
            f"{f_path}/{f_dir}/{f_name}", task_name=task_name, get_mastership=False
        ):
            self.release_mastership()
            return False
        if not self.rapid_set_pp_to_routine(
            module_name=module_name,
            routine_name=proc_name,
            task_name=task_name,
            get_mastership=False,
        ):
            self.release_mastership()
            return False
        if not self.release_mastership():
            return False

        return True

    def execute_rapid_program(
        self,
        prog,
        module_name,
        proc_name,
        task_name: str = "T_ROB1",
        block_until_execution_stops=True,
        unload_after=True,
        create_before=True,
        delete_after=True,
    ):
        """
        Creates a rapid module on the controller, loads it, sets the pp to it, and starts rapid \n
        execution

        Parameters
        ----------
        prog: str
            RAPID code as string
        module_nam: str, optional
            The name of the generated modul
        block_until_execution_stops : bool, optional
            Will block untill the proram execution is concluded
        unload_after:  bool, optional
            When False the module will not be unloaded after execution (and therefre not deleted \n
            either)
        delete_after:  bool, optional
            When False the created file and folder will not be deleted. Used when saving a few \n
            milliseconds is of interest
        create_before: bool, optional
            When False the directory and the module file are not created, because they already \n
            exist on the controller. Used when saving a few milliseconds is of interest. \n
            If the directory or the file don't exists this method will fail.

        Returns
        -------
        bool
            True on success, False otherwise.
        """

        f_path = "$HOME"
        f_dir = "temp_dir"
        f_name = "temp.modx"

        if not self.controller_switch_motors(ControllerState.motoron):
            return False
        if create_before:
            self.files_create_directory_in_controller(path=f_path, dir_name=f_dir)
        self.files_upload_file_to_controller(f_path + "/" + f_dir, f_name, prog)

        if not self.get_mastership():
            return False
        if not self.rapid_load_module(
            f"{f_path}/{f_dir}/{f_name}", task_name=task_name, get_mastership=False
        ):
            self.release_mastership()
            return False
        if not self.rapid_set_pp_to_routine(
            module_name=module_name,
            routine_name=proc_name,
            task_name=task_name,
            get_mastership=False,
        ):
            self.release_mastership()
            return False
        if not self.release_mastership():
            return False
        if not self.rapid_start_execution(cycle="once"):
            return False
        if block_until_execution_stops:
            while self.is_rapid_running:
                time.sleep(self._rws_request_block_loop_dt)
        if unload_after:
            if not self.rapid_unload_module(module_name=module_name, task_name=task_name):
                return False
        if delete_after and unload_after:
            if self.files_delete_file_in_controller(f_path + "/" + f_dir, f_name):
                return False
            if self.files_delete_directory_in_controller(f_path + "/" + f_dir):
                return False
        return True


if __name__ == "__main__":
    with RWSPy(RWSConfig(rw_version=7, ip="127.0.0.1", port="80"), debug_mode=True) as rws:
        # resp_opmode = rws.controller_get_operation_mode()
        # resp_options = rws.get_system_options()
        # resp_ctrlstate = rws.controller_get_state()
        # rws.controller_switch_motors(desired_state=ControllerState.motoron)
        # rws.controller_switch_motors(desired_state=ControllerState.motoroff)
        # jtarg = rws.robot_get_joint_positions()
        # robtarg = rws.robot_get_cartesian()
        # rws.controller_switch_motors(desired_state=ControllerState.motoron)
        # modls = rws.rapid_list_modules()
        # rws.rapid_load_module("$HOME/MyModule.modx")
        # rws.rapid_unload_module("MyModule")
        # rws.rapid_load_module("$HOME/MyModule.modx")
        # ctrl_files = rws.files_list_files_in_controller_dir("$HOME/")
        # ctrl_dirs = rws.files_list_dirs_in_controller_dir("$HOME/")
        # rws.files_create_directory_in_controller("$Home/", "new_dir")
        # rws.files_upload_file_to_controller("$home/new_dir", "test.txt", "Hello controller!")
        # content = rws.files_get_file_content_from_controller("$home/new_dir", "test.txt")
        # rws.files_delete_file_in_controller("$home/new_dir", "test.txt")
        # rws.files_delete_directory_in_controller("$Home/new_dir")
        # rws.files_delete_directory_in_controller("$Home/new_dir")
        rws.rapid_reset_pp_to_main()
        # rapid_content = "\
        #     MODULE MyModule \n\
        #         PROC my_proc() \n\
        #             MoveAbsJ [[20,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v1000,fine,tool0\WObj:=wobj0; \n\
        #             MoveAbsJ [[-20,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v1000,fine,tool0\WObj:=wobj0; \n\
        #         ENDPROC \n\
        #     ENDMODULE \n\
        #     "

        # # send and run the rapid program
        # rws.rapid_stop_execution()
        # rws.execute_rapid_program(rapid_content, module_name="MyModule", proc_name="my_proc")
