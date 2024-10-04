"""Main file for testing llm for resolves."""

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

from llm import llm_utilities

import py_trees as pt
from rws_py.rws_config import RWSConfig
from rws_py.rws_py_client import RWSPy
from interfaces.cfreeinterface import CFreeInterface
from interfaces.planner_world_interface import WorldInterface as PlannerWorldInterface
from interfaces.abb_world_interface import WorldInterface as ABBWorldInterface
from interfaces.py_trees_interface import PyTreeParameters, PyTree
from behaviors.behavior_list_settings import get_behavior_list
from behaviors.common_behaviors import get_node
from planner import planner
import behaviors.behaviors as behaviors
from behaviors.behavior_tree import BT

def run_planner(behavior_list, objects, goal_conditions, planner_world_interface, abb_world_interface):
    """ Runs the planner """
        #Convert from string to parameterized nodes
    behavior_list.convert_from_string(goal_conditions)
    print(goal_conditions)

    abb_world_interface.get_feedback()

    for object_name in objects:
        planner_world_interface.set_object_position(object_name, abb_world_interface.get_position(object_name))

    for i, goal_condition in enumerate(goal_conditions):
        goal_conditions[i], _ = get_node(goal_condition, planner_world_interface)

    # Run planner
    return planner.plan(planner_world_interface, behaviors, goal_conditions)


def run_planner_after_resolve(behavior_list, objects, string_bt, planner_world_interface, abb_world_interface, resolve_conditions):
    failed_behavior = planner_world_interface.get_failed_behavior()
    for i, node in enumerate(reversed(string_bt)):
        if failed_behavior == node:
            break

    bt = BT(string_bt, behavior_list)
    bt.add_new_first_sibling(len(string_bt) - i, resolve_conditions)
    string_bt = bt.bt

    #Replan
    planner_world_interface = PlannerWorldInterface(cfree_interface, None, objects, table_offset=TABLE_OFFSET)
    for object_name in objects:
        planner_world_interface.set_object_position(object_name, abb_world_interface.get_position(object_name))
    behavior_list.convert_from_string(string_bt)
    tree = PyTree.create_from_list(string_bt, None, behaviors, planner_world_interface)
    return planner.plan(planner_world_interface, behaviors, tree=tree)

def run_on_real_robot(behavior_lists, abb_world_interface, string_bt):
    behavior_lists.convert_from_string(string_bt)
    py_tree_parameters = PyTreeParameters(behavior_lists=behavior_lists, behaviors=behaviors, max_ticks=500, min_time_tick=0.1, verbose=True)
    py_tree = PyTree(
        string_bt[:],
        parameters=py_tree_parameters,
        world_interface=abb_world_interface)

    for _ in range(500):
        py_tree.step_bt()
        print(pt.display.unicode_tree(root=py_tree.root, show_status=True))
        if py_tree.root.status == pt.common.Status.SUCCESS:
            break

def red_on_blue(rws, cfree_interface, table_offset):
    """Runs the red on blue test """
    objects = ['"blue cube"', '"green cube"', '"red cube"']
    goal_conditions = ['"blue cube" on "green cube"']
    behavior_list = get_behavior_list()
    abb_world_interface = ABBWorldInterface(cfree_interface, rws, objects, table_offset=table_offset)
    planner_world_interface = PlannerWorldInterface(cfree_interface, None, objects, table_offset=table_offset)
    string_bt, tree = run_planner(behavior_list, objects, goal_conditions, planner_world_interface, abb_world_interface)


    if tree.status == pt.common.Status.FAILURE:
        resolve_conditions = '~"any object" on "blue cube"' # LLM would go here
        string_bt, _ = run_planner_after_resolve(behavior_list, objects, string_bt, planner_world_interface, abb_world_interface, resolve_conditions)

    print(string_bt)

    run_on_real_robot(behavior_list, abb_world_interface, string_bt)

def red_on_green(rws, cfree_interface, table_offset):
    """Runs the red on green test """
    objects = ['"blue cube"', '"green cube"', '"red cube"']
    goal_conditions = ['"blue cube" on "green cube"']
    behavior_list = get_behavior_list()
    abb_world_interface = ABBWorldInterface(cfree_interface, rws, objects, table_offset=table_offset)
    planner_world_interface = PlannerWorldInterface(cfree_interface, None, objects, table_offset=table_offset)
    string_bt, tree = run_planner(behavior_list, objects, goal_conditions, planner_world_interface, abb_world_interface)

    for object_name in objects:
        print(abb_world_interface.get_position(object_name))
    if tree.status == pt.common.Status.FAILURE:
        resolve_conditions = '~"any object" on "green cube"' # LLM would go here
        string_bt, _ = run_planner_after_resolve(behavior_list, objects, string_bt, planner_world_interface, abb_world_interface, resolve_conditions)

    print(string_bt)

    run_on_real_robot(behavior_list, abb_world_interface, string_bt)

def blue_not_visible(rws, cfree_interface, table_offset):
    """Runs blue not visible test """
    objects = ['"blue cube"', '"green cube"', '"red cube"']
    goal_conditions = ['"blue cube" on "green cube"']
    behavior_list = get_behavior_list()
    abb_world_interface = ABBWorldInterface(cfree_interface, rws, objects, table_offset=table_offset)
    planner_world_interface = PlannerWorldInterface(cfree_interface, None, objects, table_offset=table_offset)
    string_bt, tree = run_planner(behavior_list, objects, goal_conditions, planner_world_interface, abb_world_interface)

    for object_name in objects:
        print(abb_world_interface.get_position(object_name))
    if tree.status == pt.common.Status.FAILURE:
        resolve_conditions = '"blue cube" location known?' # LLM would go here
        string_bt, _ = run_planner_after_resolve(behavior_list, objects, string_bt, planner_world_interface, abb_world_interface, resolve_conditions)

    print(string_bt)

    run_on_real_robot(behavior_list, abb_world_interface, string_bt)
    
def flip_cup(rws, cfree_interface, table_offset):
    objects = ['"cup"']
    behavior_list = get_behavior_list()
    abb_world_interface = ABBWorldInterface(cfree_interface, rws, objects, table_offset=table_offset)
    planner_world_interface = PlannerWorldInterface(cfree_interface, None, objects, table_offset=table_offset)
    string_bt = ['flip "cup"']
    run_on_real_robot(behavior_list, abb_world_interface, string_bt)

if __name__ == "__main__":
    IP = "192.168.125.1"
    PORT = ""
    RW_VER = 6
    TABLE_OFFSET = 0.076

    rws = RWSPy(RWSConfig(rw_version=RW_VER, ip=IP, port=PORT, user="Default User", password="robotics"))
    client_handle = llm_utilities.get_gpt_client()
    

    cfree_interface = CFreeInterface(rws, TABLE_OFFSET)
    
    red_on_blue(rws, cfree_interface, TABLE_OFFSET)
    red_on_green(rws, cfree_interface, TABLE_OFFSET)
    blue_not_visible(rws, cfree_interface, TABLE_OFFSET)
    flip_cup(rws, cfree_interface, TABLE_OFFSET)
