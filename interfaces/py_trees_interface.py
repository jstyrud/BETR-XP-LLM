"""Interfaces to py_trees from behavior tree strings."""

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

from dataclasses import dataclass
import time
from typing import Any, List

from behaviors.common_behaviors import ParameterizedNode, get_node
from behaviors.behavior_tree import BT
import py_trees as pt

@dataclass
class PyTreeParameters:
    """Data class for parameters for the PyTree run."""

    behavior_lists: Any = None   # Lists of the types of behaviors
    behaviors: Any = None        # Module containing actual behaviors
    max_ticks: int = 200         # Maximum number of ticks to run
    max_time: float = 10000.0    # Maximum time ins s to run
    max_fails: int = 1           # Maximum number of failure states before breaking
    min_time_tick: float = 0.0   # Minimum time per tick if not handled by the communication
    successes_required: int = 2  # Number of success states required before breaking
    show_world: bool = False     # Animate the run
    verbose: bool = False        # Extra prints


class PyTree(pt.trees.BehaviourTree):
    """A class containing a behavior tree. Inherits from the py tree BehaviorTree class."""

    def __init__(
        self,
        node_list: List[ParameterizedNode],
        parameters: PyTreeParameters = None,
        world_interface: Any = None,
        root: Any = None
    ):
        if parameters is not None:
            self.par = parameters
        else:
            self.par = PyTreeParameters()

        if root is not None:
            self.root = root
            node_list = self.get_bt_from_root()

            self.bt = BT(node_list, self.par.behavior_lists)

        self.behaviors = self.par.behaviors
        self.world_interface = world_interface
        self.failed = False
        self.timeout = False

        if root is None:
            self.root, has_children = get_node(
                node_list[0], world_interface, self.par.verbose)
            node_list.pop(0)
        else:
            has_children = False

        super().__init__(root=self.root)
        if has_children:
            PyTree.create_from_list(node_list, self.root, self.behaviors, self.world_interface, self.par.verbose)

    def get_bt_from_root(self) -> List[str]:
        """
        Return bt string from py tree root by cleaning the ascii tree from py trees.

        Not complete or beautiful by any means but works for many trees.
        """
        string = pt.display.ascii_tree(self.root)
        string = string.replace('[o]', '')
        string = string.replace('[-]', '')
        string = string.replace('{o}', '')
        string = string.replace('{-}', '')
        string = string.replace('\t', '')
        string = string.replace('-->', '')
        string = string.replace('\x1b[0m', '')
        string = string.replace('\x1b[1m', '')
        string = string.replace('Fallback', 'f(')
        string = string.replace('Sequence', 's(')
        bt = string.split('\n')
        bt = bt[:-1]  # Remove empty element because of final newline

        prev_leading_spaces = 999999
        for i in range(len(bt) - 1, -1, -1):
            leading_spaces = len(bt[i]) - len(bt[i].lstrip(' '))
            bt[i] = bt[i].lstrip(' ')
            if leading_spaces > prev_leading_spaces:
                for _ in range(round((leading_spaces - prev_leading_spaces) / 4)):
                    bt.insert(i + 1, ')')
            prev_leading_spaces = leading_spaces

        bt_obj = BT(bt, self.par.behavior_lists)
        bt_obj.close()

        return bt_obj.bt

    @staticmethod
    def create_from_list(
        node_list: List[str],
        node: Any,
        behaviors: Any,
        world_interface: Any,
        verbose = False
    ) -> pt.composites.Composite:
        """Recursive function to generate the tree from a list."""
        if node is None:
            node, _ = behaviors.get_node(
                node_list[0], world_interface, verbose)
            node_list.pop(0)

        while len(node_list) > 0:
            if node_list[0] == ')':
                node_list.pop(0)
                return node

            new_node, has_children = behaviors.get_node(
                node_list[0], world_interface, verbose)
            node_list.pop(0)
            if has_children:
                # Node is a control node or decorator with children.
                # Add subtree via string and then add to parent
                new_node = PyTree.create_from_list(node_list, new_node, behaviors, world_interface, verbose)
                node.add_child(new_node)
            else:
                # Node is a leaf/action node - add to parent, then keep looking for siblings
                node.add_child(new_node)

        # This return is only reached if there are too few up nodes
        return node

    def run_bt(self):
        """Run the behavior tree."""
        ticks = 0
        straight_fails = 0
        successes = 0
        status_ok = True

        start = time.time()
        last_feedback_time = 0
        while (self.root.status is not pt.common.Status.FAILURE or
                straight_fails < self.par.max_fails) and\
              (self.root.status is not pt.common.Status.SUCCESS or
                successes < self.par.successes_required) and\
                ticks < self.par.max_ticks and status_ok:
            if time.time() - last_feedback_time < self.par.min_time_tick:
                time.sleep(self.par.min_time_tick - (time.time() - last_feedback_time))
            status_ok = self.world_interface.get_feedback()  # Wait for connection
            last_feedback_time = time.time()

            if status_ok:
                if self.par.verbose:
                    print('Tick', ticks)
                self.root.tick_once()
                self.world_interface.send_references()

                ticks += 1
                if self.root.status is pt.common.Status.SUCCESS:
                    successes += 1
                else:
                    successes = 0

                if self.root.status is pt.common.Status.FAILURE:
                    straight_fails += 1
                else:
                    straight_fails = 0

                if time.time() - start > self.par.max_time:
                    status_ok = False
                    print('Max time expired')

        if self.par.verbose:
            print('Total episode ticks: ', ticks)
            print('Total episode time: ', time.time() - start)

        if ticks >= self.par.max_ticks:
            self.timeout = True
        if straight_fails >= self.par.max_fails:
            self.failed = True
        return ticks, status_ok

    def step_bt(self):
        """Step the BT one step."""
        status_ok = True

        status_ok = self.world_interface.get_feedback()  # Wait for connection

        if status_ok:
            self.root.tick_once()
            self.world_interface.send_references()

        return status_ok

    def save_fig(
        self,
        path: str,
        name: str = 'Behavior tree',
        static: bool = True,
        blackboard: bool = False
    ):
        """Save the tree as a figure."""
        pt.display.render_dot_tree(
            self.root,
            name=name,
            target_directory=path,
            static=static,
            with_blackboard_variables=blackboard
        )
