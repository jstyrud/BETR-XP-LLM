"""Interface to collision free path planning."""

# Copyright (c) 2024, ABB
# All rights reserved.
#
# Redistribution and use in source and binary forms, with
# or without modification, are permitted provided that
# the following conditions are met:
#
#   * Redistributions of source code must retain the
#     above copyright notice, this list of conditions
#     and the following disclaimer.
#   * Redistributions in binary form must reproduce the
#     above copyright notice, this list of conditions
#     and the following disclaimer in the documentation
#     and/or other materials provided with the
#     distribution.
#   * Neither the name of ABB nor the names of its
#     contributors may be used to endorse or promote
#     products derived from this software without
#     specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import numpy as np
from scipy.spatial.transform import Rotation
import cfree_py.utils_misc as ut
from cfree_py.client import CFreePyClient
from cfree_py.mesh import Mesh
from cfree_py.plotter_pv import PlotterPV
from cfree_py.pose import Pose
from cfree_py.types import JointTarget, PlanningQuery, RobotModel, RobTarget, CollisionBody

class CFreeInterface():
    """ Interface to the cfree server """
    def __init__(self, rws=None, table_offset=0):
        self.client = CFreePyClient(run_server_process=True)
        self.rws = rws
        self.obstacles = {}

        self.add_obstacle("table", position=[0.3, 0, -0.03 + table_offset], size=[1.5, 1.3, 0.06])
        rob_model = RobotModel.IRB14050
        print(f"CFree robot: \t\t{rob_model.name}")

        dual_arm_yumi_shoulder_pose = Pose(pos=[0.047607, -0.070008, 0.411486],
                                           orient=Rotation.from_euler('XYZ',
                                                                      [0.62923288658907, 0.95065732066284, 0.18426589256253]))
        single_arm_yumi_base_offset = Pose(pos=[0.0, 0.0, -0.1985])
        total_pose = dual_arm_yumi_shoulder_pose * single_arm_yumi_base_offset
        self.client.add_robot(model=rob_model, base_tf=total_pose, num_planning_threads=4)

        #Fingers
        self.client.add_robot_tool_geometry(body=CollisionBody(meshes=[Mesh.box([0.01, 0.01, 0.136])],
                                                               pose=Pose([0.0, 0.0, 0.136/2])),
                                                               tcp_transform=Pose([0.0, 0, 0.136]))
        
    def add_obstacle(self, name, position, size=None, variant=False):
        """ Add a an obstacle in the form of a mesh """
        if "centrifuge" in name:
            if variant:
                mesh = Mesh.from_file(f"BETR-XP-LLM/vision/models/open_centrifuge.obj")
            else:
                mesh = Mesh.from_file(f"BETR-XP-LLM/vision/models/closed_centrifuge.obj")
            self.obstacles[name] = self.client.add_obstacle(CollisionBody(meshes=[mesh], pose=Pose(position, Rotation.from_euler("x", 90, degrees=True))))
        else:
            mesh = Mesh.box(size)
            self.obstacles[name] = self.client.add_obstacle(CollisionBody(meshes=[mesh], pose=Pose(position)))

    def remove_obstacle(self, name):
        """ Removes an obstacle """
        try:
            self.client.remove_collision_body(self.obstacles[name])
            del self.obstacles[name]
        except KeyError:
            return

    def move_obstacle(self, name, position, size):
        """ Move obstacle to a new position """
        self.remove_obstacle(name)

        self.add_obstacle(name, position, size)

    def get_collfree_path(self, target_position, orientation=None,
                          start_position=None, start_orientation=None,
                          check_only=False, min_distance=0.03, show_debug_images=False):
        """ Plans a collision free path and runs it on the robot """
        # Specify goal target
        if orientation is None:
            orientation = Rotation.from_euler('xyz', [0, 180, 90], degrees=True)
        elif isinstance(orientation, np.ndarray):
            orientation = Rotation.from_quat(orientation, scalar_first=True)
        if start_orientation is None:
            start_orientation = Rotation.from_euler('xyz', [0, 180, 90], degrees=True)
        elif isinstance(start_orientation, np.ndarray):
            start_orientation = Rotation.from_quat(start_orientation, scalar_first=True)
        if start_position is not None:
            start = RobTarget(cf1=0, cf4=0, cf6=0, cfx=4, arm_angle=-np.pi,
                              pose=Pose(start_position, start_orientation))
        else:
            # get the current joint positions of the robot from the controller
            jpos_curr = self.rws.robot_get_joint_positions(mech_unit="ROB_R")
            start = JointTarget(robax=(np.array(jpos_curr[0]) * np.pi / 180).tolist(),
                                extax=[(np.array(jpos_curr[1][0]) * np.pi / 180).tolist()])
        goal = RobTarget(cf1=0, cf4=0, cf6=0, cfx=4, arm_angle=-np.pi,
                         pose=Pose(target_position, orientation))

        result_ok = False
        min_distance_temp = min_distance
        while not result_ok and min_distance_temp > 0:
            query = PlanningQuery(start, goal, timeout=100, min_distance_to_obstacles=min_distance_temp)
            result = self.client.plan_collision_free_paths(query)
            if result.status.value == 1:
                result_ok = True
            else:
                #Try again with tighter distance requirements (sometimes we start in a bad position for example)
                min_distance_temp -= 0.005

        if show_debug_images:
            vis = PlotterPV(self.client)
            vis.plot_cell_and_path(path=result, query=query, animate=False)
        if result.status.value == 1:
            if not check_only:
                program = ut.generate_rapid_program_from_path(result.best_path, speed=100, tool="MainTool", module_name="cfree_mod",
                                                              proc_name="collfree_path")
                #Fix program
                program = program.replace("MODULE cfree_mod\n  PROC collfree_path()\n", "")
                program = program.replace("  ENDPROC\nENDMODULE\n", "")
                program = program.replace("FINE", "z5")
                return program
            return True
        
        return None#result.status.value