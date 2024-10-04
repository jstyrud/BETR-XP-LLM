"""Interface to ABB robots and sensors."""

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
from copy import deepcopy
import numpy as np

from vision.vision_system import VisionSystem
from vision.kinect_camera import KinectCamera
import vision.perception_utils as utils
import vision.heuristics as heur
from interfaces.base_world_interface import BaseWorldInterface

class WorldInterface(BaseWorldInterface):
    """
    Class for handling the simple planning world
    """
    def __init__(self, cfree_interface, rws, movable_objects, graspable_objects=None, table_offset=0, use_vision=True):
        self.gripper_position = 0

        self.use_vision = use_vision
        if use_vision:
            self.vision_system = VisionSystem()
            self.vision_system.load_models()
            self.camera = KinectCamera()
            self.cropping = [1250, 2500, 1000, 3000]
            pos = np.array([0.117, -0.038, 0.647])
            quat = np.array([-0.645, 0.653, -0.273, 0.288])
            self.T_camera_in_robot = utils.homogeneous_matrix(pos, quat) #pylint:disable=invalid-name
        BaseWorldInterface.__init__(self, cfree_interface, rws, movable_objects, graspable_objects, table_offset)

    def get_feedback(self):
        # pylint: disable=no-self-use
        """ Get feedback from sensors to update world state """
        self.gripper_position = int(self.rws.ios_get_signal_value("hand_ActualPosition_R")) / 10000
        if self.gripper_position < 0.005:
            #This means we lost the object which allowed the fingers to close fully
            if self.grasped_object is not None:
                self.object_position_known[self.grasped_object] = False
                self.grasped_object = None
                self.stop() # Almost definitely we should stop here

        # run pose estimation pipeline
        if self.use_vision:
            rgb_img, depth_img, _  = self.camera.get_image(self.cropping)
            masks, labels = self.vision_system.run_pose_estimation_pipeline(
                image=rgb_img,
                texts=deepcopy(self.movable_objects),
                max_dets=2,
                score_thr=0.2,
                do_color_masking=True,
                show_debug_images=False
            )

            for i, label in enumerate(labels):
                for movable_object in self.movable_objects:
                    if movable_object != self.grasped_object:
                    
                        if movable_object.strip('"') in label:
                            position = None
                            if "cube" in movable_object:
                                position = heur.cube_heuristic(masks[i], depth_img, rgb_img,
                                                                self.camera, self.T_camera_in_robot,
                                                                WorldInterface.CUBE_SIZE, self.cropping)
                            elif "cup" in movable_object:
                                position, is_flipped = heur.cup_heuristic(masks[i], depth_img, self.camera, 
                                                              self.T_camera_in_robot, self.cropping)
                                if movable_object != self.manipulation_target: #Flip detection will not work once we start grasping
                                    self.object_upright[movable_object] = not is_flipped
                            elif "cap" in movable_object:
                                position = heur.cap_heuristic(masks[i], depth_img, self.camera, 
                                                                          self.T_camera_in_robot, self.cropping)
                            elif "centrifuge" in movable_object:
                                if movable_object != self.manipulation_target: # Centrifuge detection will not work if we are manipulating it
                                    centrifuge_opened = heur.centrifuge_heuristic(masks[i])
                                    if centrifuge_opened and self.object_opened[movable_object] != centrifuge_opened:
                                        self.object_opened[movable_object] = centrifuge_opened
                                        self.cfree_interface.remove_obstacle(movable_object)
                                        self.cfree_interface.add_obstacle(movable_object, [0.5, 0.075, self.table_offset + 0.025], variant=centrifuge_opened)
                            if position is not None:
                                if not self.object_position_known[movable_object] or self.calc_distance(movable_object, position) > 0.02:
                                    self.set_object_position(movable_object, position)

        return True

    def move_cfree(self, position, orientation=None):
        """ Move to position along a collision free path """
        BaseWorldInterface.update_cfree_objects(self)

        return self.cfree_interface.get_collfree_path(position, orientation)

    def move_linear(self, position, orientation=None, _target_object=None):
        """ Move linearly to position without checking for collisions """
        if orientation is None:
            orientation_str = '[0, 0.707107, -0.707107, 0]'
        else:
            orientation_str = ''.join(['[', str(orientation[0]), ',', str(orientation[1]), ',', str(orientation[2]), ',', str(orientation[3]), ']'])
        return "    MoveL [[" \
                        + str(position[0] * 1000) + "," + str(position[1] * 1000) + "," + str(position[2] * 1000) + \
                    "]," + orientation_str + ",[0,0,0,4],[-180,9E9,9E9,9E9,9E9,9E9]],v100,fine,MainTool\\WObj:=wobj0; \n"
    
    def move_joint(self, position, orientation=None, _target_object=None, _no_cfree_check=False):
        """ Move joints to position without checking for collisions """
        if orientation is None:
            orientation_str = '[0, 0.707107, -0.707107, 0]'
        else:
            orientation_str = ''.join(['[', str(orientation[0]), ',', str(orientation[1]), ',', str(orientation[2]), ',', str(orientation[3]), ']'])
        return "    MoveJ [[" \
                        + str(position[0] * 1000) + "," + str(position[1] * 1000) + "," + str(position[2] * 1000) + \
                    "]," + orientation_str + ",[0,0,0,4],[-180,9E9,9E9,9E9,9E9,9E9]],v50,z5,MainTool\\WObj:=wobj0; \n"
    
    def close_gripper(self):
        """ Closes the gripper """
        rapid_rob_r = "\
            MODULE MyModule \n\
                PROC my_proc() \n\
                    g_GripIn; \n\
                ENDPROC \n\
            ENDMODULE \n\
            "
        return self.run_program(rapid_rob_r)

    def open_gripper(self):
        """ Opens the gripper """
        rapid_rob_r = "\
            MODULE MyModule \n\
                PROC my_proc() \n\
                    g_GripOut \\NoWait; \n\
                ENDPROC \n\
            ENDMODULE \n\
            "
        return self.run_program(rapid_rob_r)

    def is_running(self):
        """ Returns true if RAPID is running, false otherwise """
        return self.rws.is_rapid_running

    def has_stopped(self):
        """ Returns true if RAPID is not running, false otherwise. Needs to be separate from is_running so it can be faked in simulation """
        return not self.rws.is_rapid_running

    def stop(self):
        """ Stops RAPID execution """
        self.rws.rapid_stop_execution()

    def run_program(self, program):
        """ Executes the input program"""
        return self.rws.execute_rapid_program(
            program,
            module_name="MyModule",
            proc_name="my_proc",
            block_until_execution_stops=False,
            unload_after=False,
            task_name="T_ROB_R"
        )
