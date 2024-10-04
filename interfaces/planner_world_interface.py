"""Simple planning interface simulating ABB robots and sensors."""

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
from interfaces.base_world_interface import BaseWorldInterface

class WorldInterface(BaseWorldInterface):
    """
    Class for handling the simple planning world
    """
    def __init__(self, cfree_interface, rws=None, movable_objects=None, graspable_objects=None, table_offset=0):
        self.robot_position = np.array([0.5, 0.0, 0.3])
        self.robot_orientation = np.array([0, 0.707107, -0.707107, 0])
        BaseWorldInterface.__init__(self, cfree_interface, rws, movable_objects, graspable_objects, table_offset)

    def move_cfree(self, position, orientation=None):
        """ Move to position along a collision free path """
        BaseWorldInterface.update_cfree_objects(self)
        cfree_response = self.cfree_interface.get_collfree_path(position, orientation,
                                                                self.robot_position, self.robot_orientation, check_only=True)
        if cfree_response is True:
            self.robot_position = position
            self.robot_orientation = orientation
            return ""
        else:
            self.set_cfree_error(cfree_response)
            return None

    def move_linear(self, position, orientation, target_object):
        """ Check if there is a collision free path, and if so teleports the robot there """
        self.cfree_interface.remove_obstacle(target_object)
        cfree_response = self.cfree_interface.get_collfree_path(position, orientation,
                                                                self.robot_position, self.robot_orientation, check_only=True)
        if cfree_response is True:
            self.robot_position = position
            self.robot_orientation = orientation
            return ""
        else:
            self.set_cfree_error(cfree_response)
            return None
        
    def move_joint(self, position, orientation=None, target_object=None, no_cfree_check=False):
        """ Check if there is a collision free path, and if so teleports the robot there """
        if no_cfree_check:
            cfree_response = True
        else:
            self.cfree_interface.remove_obstacle(target_object)
            cfree_response = self.cfree_interface.get_collfree_path(position, orientation,
                                                                    self.robot_position, self.robot_orientation, check_only=True)
        if cfree_response is True:
            self.robot_position = position
            self.robot_orientation = orientation
            return ""
        else:
            self.set_cfree_error(cfree_response)
            return None
        
    def set_object_upright(self, target_object, value):
        """ Set the object upright variable """
        self.object_upright[target_object] = value

    def set_object_opened(self, target_object, value):
        """ Set the object opened variable only set directly in planner"""
        self.cfree_interface.remove_obstacle(target_object)
        self.cfree_interface.add_obstacle(target_object, [0.5, 0.075, self.table_offset + 0.025], variant=value) # TODO position should be just kept as is, save and read
        self.object_opened[target_object] = value
