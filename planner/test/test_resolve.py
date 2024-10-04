"""Unit test for error resolving."""

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
import py_trees as pt
from planner import planner
from interfaces.planner_world_interface import WorldInterface
from interfaces.cfreeinterface import CFreeInterface
from interfaces.py_trees_interface import PyTree
from behaviors import behaviors
from behaviors.behavior_list_settings import get_behavior_list
from behaviors.common_behaviors import get_node
from behaviors.behavior_tree import BT


def test_remove_from_source():
    """ Test run of remove scenario """
    cfree_interface = CFreeInterface()
    objects = ['"blue cube"', '"green cube"', '"red cube"']
    world_interface = WorldInterface(cfree_interface, movable_objects=objects)
    world_interface.set_object_position('"green cube"', np.array([0.50, 0.0, 0.02]))
    world_interface.set_object_position('"blue cube"', np.array([0.55, 0.0, 0.02]))
    world_interface.set_object_position('"red cube"', np.array([0.55, 0.0, 0.045]))
    goal_conditions = ['"blue cube" on "green cube"']
    behavior_list = get_behavior_list(objects)
    behavior_list.convert_from_string(goal_conditions)
    for i, goal_condition in enumerate(goal_conditions):
        goal_conditions[i], _ = get_node(goal_condition, world_interface)

    string_bt, tree = planner.plan(world_interface, behaviors, goal_conditions)
    if tree.status == pt.common.Status.FAILURE:
        failed_behavior = world_interface.get_failed_behavior()
        for i, node in enumerate(reversed(string_bt)):
            if failed_behavior == node:
                break
        resolve_conditions = '~"any object" on "blue cube"' # LLM would go here
        bt = BT(string_bt, behavior_list)
        bt.add_new_first_sibling(len(string_bt) - i, resolve_conditions)
        string_bt = bt.bt

        #Replan
        world_interface = WorldInterface(cfree_interface, movable_objects=objects)
        world_interface.set_object_position('"green cube"', np.array([0.50, 0.0, 0.02]))
        world_interface.set_object_position('"blue cube"', np.array([0.55, 0.0, 0.02]))
        world_interface.set_object_position('"red cube"', np.array([0.55, 0.0, 0.045]))
        behavior_list.convert_from_string(string_bt)
        tree = PyTree.create_from_list(string_bt, None, behaviors, world_interface)
        string_bt, tree = planner.plan(world_interface, behaviors, tree=tree)

    assert tree.status == pt.common.Status.SUCCESS
    assert string_bt == ['f(', '"blue cube" on "green cube"?',
                               's(', 'f(', 'grasped "blue cube"?', 
                                           's(', 'f(', '~"any object" on "blue cube"?', 
                                                       's(', '~grasped "any object"?', 'grasp "any object" from on "blue cube"!', ')', ')',
                                                 'place "grasped object" on "table"!',
                                                 'grasp "blue cube"!', ')', ')', 
                                     'place "blue cube" on "green cube"!', ')', ')']

def test_remove_from_destination():
    """ Test run of remove scenario """
    cfree_interface = CFreeInterface()
    objects = ['"blue cube"', '"green cube"', '"red cube"']
    world_interface = WorldInterface(cfree_interface, movable_objects=objects)
    world_interface.set_object_position('"green cube"', np.array([0.50, 0.0, 0.02]))
    world_interface.set_object_position('"blue cube"', np.array([0.55, 0.0, 0.02]))
    world_interface.set_object_position('"red cube"', np.array([0.50, 0.0, 0.045]))
    goal_conditions = ['"blue cube" on "green cube"']
    behavior_list = get_behavior_list(objects)
    behavior_list.convert_from_string(goal_conditions)
    for i, goal_condition in enumerate(goal_conditions):
        goal_conditions[i], _ = get_node(goal_condition, world_interface)

    string_bt, tree = planner.plan(world_interface, behaviors, goal_conditions)
    if tree.status == pt.common.Status.FAILURE:
        failed_behavior = world_interface.get_failed_behavior()
        for i, node in enumerate(reversed(string_bt)):
            if failed_behavior == node:
                break
        resolve_conditions = '~"any object" on "green cube"' # LLM would go here
        bt = BT(string_bt, behavior_list)
        bt.add_new_first_sibling(len(string_bt) - i, resolve_conditions)
        string_bt = bt.bt

        #Replan
        world_interface = WorldInterface(cfree_interface, movable_objects=objects)
        world_interface.set_object_position('"green cube"', np.array([0.50, 0.0, 0.02]))
        world_interface.set_object_position('"blue cube"', np.array([0.55, 0.0, 0.02]))
        world_interface.set_object_position('"red cube"', np.array([0.50, 0.0, 0.045]))
        behavior_list.convert_from_string(string_bt)
        tree = PyTree.create_from_list(string_bt, None, behaviors, world_interface)
        string_bt, tree = planner.plan(world_interface, behaviors, tree=tree)

    assert tree.status == pt.common.Status.SUCCESS
    assert string_bt == ['f(', '"blue cube" on "green cube"?',
                               's(', 'f(', '~"any object" on "green cube"?', 
                                           's(', '~grasped "any object"?', 'grasp "any object" from on "green cube"!', ')', ')',
                                     'f(', 'grasped "blue cube"?',
                                           's(', 'place "grasped object" on "table"!', 'grasp "blue cube"!', ')', ')',
                                     'place "blue cube" on "green cube"!', ')', ')']
    
def test_remove_both():
    """ Test run of double remove scenario """
    cfree_interface = CFreeInterface()
    objects = ['"blue cube"', '"green cube"', '"red cube"', '"yellow cube"']
    world_interface = WorldInterface(cfree_interface, movable_objects=objects)
    world_interface.set_object_position('"green cube"', np.array([0.50, 0.0, 0.02]))
    world_interface.set_object_position('"blue cube"', np.array([0.55, 0.0, 0.02]))
    world_interface.set_object_position('"red cube"', np.array([0.50, 0.0, 0.045]))
    world_interface.set_object_position('"yellow cube"', np.array([0.55, 0.0, 0.045]))
    goal_conditions = ['"blue cube" on "green cube"']
    behavior_list = get_behavior_list(objects)
    behavior_list.convert_from_string(goal_conditions)
    for i, goal_condition in enumerate(goal_conditions):
        goal_conditions[i], _ = get_node(goal_condition, world_interface)

    string_bt, tree = planner.plan(world_interface, behaviors, goal_conditions)
    if tree.status == pt.common.Status.FAILURE:
        failed_behavior = world_interface.get_failed_behavior()
        for i, node in enumerate(reversed(string_bt)):
            if failed_behavior == node:
                break
        resolve_conditions = '~"any object" on "blue cube"' # LLM would go here
        bt = BT(string_bt, behavior_list)
        bt.add_new_first_sibling(len(string_bt) - i, resolve_conditions)
        string_bt = bt.bt

        #Replan
        world_interface = WorldInterface(cfree_interface, movable_objects=objects)
        world_interface.set_object_position('"green cube"', np.array([0.50, 0.0, 0.02]))
        world_interface.set_object_position('"blue cube"', np.array([0.55, 0.0, 0.02]))
        world_interface.set_object_position('"red cube"', np.array([0.50, 0.0, 0.045]))
        world_interface.set_object_position('"yellow cube"', np.array([0.55, 0.0, 0.045]))
        behavior_list.convert_from_string(string_bt)
        tree = PyTree.create_from_list(string_bt, None, behaviors, world_interface)
        string_bt, tree = planner.plan(world_interface, behaviors, tree=tree)

    if tree.status == pt.common.Status.FAILURE:
        failed_behavior = world_interface.get_failed_behavior()
        for i, node in enumerate(reversed(string_bt)):
            if failed_behavior == node:
                break
        resolve_conditions = '~"any object" on "green cube"' # LLM would go here
        bt = BT(string_bt, behavior_list)
        bt.add_new_first_sibling(len(string_bt) - i, resolve_conditions)
        string_bt = bt.bt

        #Replan
        world_interface = WorldInterface(cfree_interface, movable_objects=objects)
        world_interface.set_object_position('"green cube"', np.array([0.50, 0.0, 0.02]))
        world_interface.set_object_position('"blue cube"', np.array([0.55, 0.0, 0.02]))
        world_interface.set_object_position('"red cube"', np.array([0.50, 0.0, 0.045]))
        world_interface.set_object_position('"yellow cube"', np.array([0.55, 0.0, 0.045]))
        behavior_list.convert_from_string(string_bt)
        tree = PyTree.create_from_list(string_bt, None, behaviors, world_interface)
        string_bt, tree = planner.plan(world_interface, behaviors, tree=tree)

    assert tree.status == pt.common.Status.SUCCESS
    assert string_bt == ['f(', '"blue cube" on "green cube"?',
                            's(', 'f(', '~"any object" on "green cube"?',
                                        's(', '~grasped "any object"?', 'grasp "any object" from on "green cube"!', ')', ')',
                                    'f(', 'grasped "blue cube"?',
                                        's(', 'f(', '~"any object" on "blue cube"?', 
                                                    's(', 'place "grasped object" on "table"!',
                                                          'grasp "any object" from on "blue cube"!', ')', ')',
                                              'place "grasped object" on "table"!',
                                              'grasp "blue cube"!', ')', ')',
                                    'place "blue cube" on "green cube"!', ')', ')']
