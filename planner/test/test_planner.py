"""Unit test for planner."""

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
from planner import planner
from interfaces.planner_world_interface import WorldInterface
from interfaces.cfreeinterface import CFreeInterface
from behaviors import behaviors
from behaviors.behavior_list_settings import get_behavior_list
from behaviors.common_behaviors import get_node

def test_pnp():
    """ Test run of pnp scenario """
    cfree_interface = CFreeInterface()
    objects = ['"blue cube"', '"green cube"', '"red cube"']
    world_interface = WorldInterface(cfree_interface=cfree_interface, movable_objects=objects)
    world_interface.set_object_position('"blue cube"', np.array([0.5, -0.1, 0.01]))
    world_interface.set_object_position('"green cube"', np.array([0.5, 0.0, 0.01]))
    world_interface.set_object_position('"yellow cube"', np.array([0.5, 0.1, 0.01]))
    goal_conditions = [behaviors.AtPos('', {"target_object": '"blue cube"', "relation": 'on', "relative_object": '"green cube"'},
                                       world_interface)]
    string_bt, _ = planner.plan(world_interface, behaviors, goal_conditions)
    assert string_bt == ['f(', '"blue cube" on "green cube"?',
                               's(', 'f(', 'grasped "blue cube"?', 
                                     's(', '~grasped "any object"?', 'grasp "blue cube"!', ')', ')', 
                                     'place "blue cube" on "green cube"!', ')', ')']

def test_remove():
    """ Test run of remove scenario """
    cfree_interface = CFreeInterface()
    objects = ['"blue cube"', '"green cube"', '"red cube"']
    world_interface = WorldInterface(cfree_interface, movable_objects=objects)
    world_interface.set_object_position('"green cube"', np.array([0.50, 0.0, 0.09]))
    world_interface.set_object_position('"blue cube"', np.array([0.55, 0.0, 0.09]))
    world_interface.set_object_position('"red cube"', np.array([0.55, 0.0, 0.115]))
    goal_conditions = ['~"any object" on "blue cube"', 'grasped "blue cube"']
    behavior_list = get_behavior_list(objects)
    behavior_list.convert_from_string(goal_conditions)
    for i, goal_condition in enumerate(goal_conditions):
        goal_conditions[i], _ = get_node(goal_condition, world_interface)

    string_bt, _ = planner.plan(world_interface, behaviors, goal_conditions)
    assert string_bt == ['s(', 'f(', '~"any object" on "blue cube"?',
                                     's(', '~grasped "any object"?', 'grasp "any object" from on "blue cube"!', ')', ')',
                               'f(', 'grasped "blue cube"?',
                                     's(', 'place "grasped object" on "table"!', 'grasp "blue cube"!', ')', ')', ')']
