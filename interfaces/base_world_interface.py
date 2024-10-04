""" Basic world interface for ABB robots and sensors to be inherited in for use in both planning and real robot stages."""

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

class BaseWorldInterface:
    """
    Base world interface class
    """
    CUBE_SIZE = 0.025
    CUP_HEIGHT = 0.05
    def __init__(self, cfree_interface, rws=None, movable_objects=None, graspable_objects=None, table_offset=0):
        self.cfree_interface = cfree_interface
        self.rws = rws
        self.grasped_object = None
        self.manipulation_target = None
        self.object_positions = {}
        self.object_position_known = {}
        self.object_upright = {}
        self.object_opened = {}
        self.object_unlocked = {}
        self.movable_objects = movable_objects
        if movable_objects is not None:
            for movable_object in movable_objects:
                self.object_positions[movable_object] = None
                self.object_position_known[movable_object] = False
                self.object_upright[movable_object] = True
                self.object_opened[movable_object] = False
                self.object_unlocked[movable_object] = False
        if graspable_objects is None:
            graspable_objects = movable_objects
        self.graspable_objects = graspable_objects
        self.table_offset = table_offset
        self.error_message = ''
        self.failed_behavior = ''

    def get_feedback(self):
        # pylint: disable=no-self-use
        """ Get feedback from sensors to update world state """
        return True

    def send_references(self):
        # pylint: disable=no-self-use
        """ Dummy to fit template """
        return

    def set_failed_behavior(self, failed_behavior):
        """ Sets memory of last failed behavior """
        self.failed_behavior = failed_behavior

    def get_failed_behavior(self):
        """ Returns memory of last failed behavior """
        return self.failed_behavior

    def set_cfree_error(self, cfree_error):
        """ Sets most recent cfree error and converts it to a message """
        if cfree_error == 202:
            self.set_error_message("Position not reachable due to collision")
        elif cfree_error == 203:
            self.set_error_message("Position not reachable due to collision")

    def set_error_message(self, error_message):
        """ Sets most recent error message """
        self.error_message = error_message

    def get_error_message(self):
        """ Returns most recent error message """
        return self.error_message

    def is_graspable(self, target_object):
        """ True if object is graspable """
        return target_object in self.graspable_objects

    def get_grasped_object(self):
        """ Returns grasped object"""
        return self.grasped_object

    def set_grasped_object(self, target_object):
        """ Set grasped object"""
        self.grasped_object = target_object
        self.object_position_known[target_object] = False

    def get_manipulation_target(self):
        """ Returns manipulation target"""
        return self.manipulation_target

    def set_manipulation_target(self, target_object):
        """ Set manipulation target"""
        self.manipulation_target = target_object

    def update_cfree_objects(self):
        """ Make sure all objects are in up to date positions """
        for movable_object in self.movable_objects:
            if "cube" in movable_object:
                if self.object_position_known[movable_object]:
                    self.cfree_interface.move_obstacle(movable_object,
                                                       self.object_positions[movable_object],
                                                       [BaseWorldInterface.CUBE_SIZE,
                                                        BaseWorldInterface.CUBE_SIZE,
                                                        BaseWorldInterface.CUBE_SIZE])
                else:
                    self.cfree_interface.remove_obstacle(movable_object)

    def close_gripper(self):
        """ No gripper simulation for now """
        return

    def open_gripper(self):
        """ No gripper simulation for now """
        return

    @staticmethod
    def get_close_gripper_program():
        """ Returns only the one program row needed to close gripper """
        return "    g_GripIn \\holdForce:=7; \n"

    @staticmethod
    def get_open_gripper_program(no_wait=False):
        """ Returns only the one program row needed to open gripper """
        if no_wait:
            return "    g_GripOut \\NoWait; \n"
        else:
            return "    g_GripOut; \n"
        
    def motionsupervision_off(self):
        """ Turns off motion supervision"""
        return "    MotionSup\Off; \n"
    
    def softservo_on(self):
        """ Turn on the soft servo """
        return "    SoftAct 1, 10; \n    SoftAct 2, 10; \n    SoftAct 3, 10; \n    SoftAct 4, 10; \n    SoftAct 5, 10; \n    SoftAct 6, 10; \n"

    def is_running(self):
        """ Program start assumed instantaneous so always true """
        return True

    def has_stopped(self):
        """ Program execution assumed instantaneous so always done """
        return True

    def stop(self):
        """ Program execution assumed instantaneous so always done """
        return

    def calc_distance(self, target_object, position):
        """ Calculates the distance between target object and given position """
        return np.linalg.norm(self.object_positions[target_object] - position)

    def get_position(self, target_object):
        """
        Returns position of object, 
        if target_object is table, find an empty spot on the table
        """
        if target_object == '"table"':
            for x in (0.5, 0.4, 0.6):
                for y in (0, 0.1, -0.1, 0.2, -0.2):
                    position = np.array([x, y, self.table_offset])
                    position_empty = True
                    for movable_object in self.movable_objects:
                        if self.object_position_known[movable_object]:
                            if self.calc_distance(movable_object, position) < BaseWorldInterface.CUBE_SIZE * 2:
                                position_empty = False
                    if position_empty:
                        return position
            return None
        else:
            return self.object_positions[target_object]

    def set_object_position(self, target_object, position):
        """ Sets position of object """
        self.object_positions[target_object] = position
        if position is not None:
            self.object_position_known[target_object] = True
        else:
            self.object_position_known[target_object] = False

    def object_at(self, target_object, relation, relative_object):
        """ Checks if object is at target_position"""
        if relation == "on":
            if self.object_position_known[target_object]:
                if relative_object == '"table"' and "cube" in target_object:
                    if abs(self.object_positions[target_object][2] - \
                           BaseWorldInterface.CUBE_SIZE / 2 - self.table_offset) < 0.01:
                        return True
                elif self.object_position_known[relative_object] and "cube" in target_object:
                    if abs(self.object_positions[target_object][0] - self.object_positions[relative_object][0]) < 0.01 and \
                       abs(self.object_positions[target_object][1] - self.object_positions[relative_object][1]) < 0.01 and \
                       abs(self.object_positions[target_object][2] - \
                           BaseWorldInterface.CUBE_SIZE - self.object_positions[relative_object][2]) < 0.01:
                        return True
        elif relation == "in":
            if self.object_position_known[target_object]:
                if self.object_position_known[relative_object]:
                    if abs(self.object_positions[target_object][0] - self.object_positions[relative_object][0]) < 0.01 and \
                       abs(self.object_positions[target_object][1] - self.object_positions[relative_object][1]) < 0.01 and \
                       abs(self.object_positions[target_object][2] - self.object_positions[relative_object][2]) < 0.03:
                        return True

        elif relation == "at" and isinstance(relative_object, np.ndarray):
            if self.object_position_known[target_object]:
                if self.calc_distance(target_object, relative_object) < 0.01:
                    return True

        return False

    def is_object_upright(self, target_object):
        """ Checks if target is standing upright or not """
        return self.object_upright[target_object]
    
    def set_object_upright(self, _target_object, _value):
        """ Set the object upright variable only set directly in planner"""
        pass

    def is_near_robot(self, target_object, distance=0.6):
        """ Checks if object is within reach """
        if self.object_position_known[target_object] and \
            self.calc_distance(target_object, np.array([0.047607, -0.070008, 0.411486])) < distance:
            return True
        return False

    def is_opened(self, target_object):
        """ Checks if object is opened """
        return self.object_opened[target_object]
    
    def set_object_opened(self, target_object, _value):
        """ Set the object opened variable only set directly in planner"""
        pass

    def is_unlocked(self, target_object):
        """ Checks if object is unlocked """
        return self.object_unlocked[target_object]

    @staticmethod
    def finalize_program(program):
        """ Wraps program in a module so that it can be loaded and executed """
        return "MODULE MyModule \n    PROC my_proc() \n" + program + \
               "   ENDPROC \nENDMODULE \n"

    def run_program(self, program): #pylint: disable=unused-argument
        """ Executes the input program"""
        return True
