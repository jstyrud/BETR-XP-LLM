"""List of behaviors to be used for the genetic programming."""

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
import random
from copy import deepcopy

from typing import List
from behaviors import common_behaviors


class BehaviorLists():
    """A list of all the available nodes."""

    def __init__(
        self,
        fallback_nodes: List[str] = None,
        sequence_nodes: List[str] = None,
        condition_nodes: List[str] = None,
        action_nodes: List[str] = None,
        root_nodes: List[str] = None
    ):
        # A list of all types of fallback nodes used, typically just one
        if fallback_nodes is not None:
            self.fallback_nodes = fallback_nodes
        else:
            self.fallback_nodes = ['f(']

        # A list of all types of sequence nodes used, typically just one
        if sequence_nodes is not None:
            self.sequence_nodes = sequence_nodes
        else:
            self.sequence_nodes = ['s(']

        # Control nodes are nodes that may have one or more children/subtrees.
        # Subsequent nodes will be children/subtrees until the related up character is reached.
        # List will contain fallback_nodes, sequence_nodes and any other control nodes.
        self.control_nodes = self.fallback_nodes + self.sequence_nodes

        # Conditions nodes are childless leaf nodes that never return RUNNING state.
        self.condition_nodes = []
        if condition_nodes is not None:
            self.condition_nodes = condition_nodes

        # Action nodes are also childless leaf nodes but may return RUNNING state.
        self.action_nodes = []
        if action_nodes is not None:
            self.action_nodes = action_nodes

        # A list of all allowed root node types
        self.root_nodes = root_nodes

        # Atomic fallback nodes are fallback nodes that have a predetermined set of
        # children/subtrees that cannot be changed.
        # They behave mostly like action nodes except that they may not be
        # the children of fallback nodes. Length is counted as one.
        self.atomic_fallback_nodes = []

        # Atomic sequence nodes are sequence nodes that have a predetermined set of
        # children/subtrees that cannot be changed.
        # They behave mostly like action nodes except that they may not be
        # the children of sequence nodes. Length is counted as one.
        self.atomic_sequence_nodes = []

        # The up node is not a node but a character that marks the end of a control nodes
        # set of children and subtrees.
        self.up_node = [')']

        self.behavior_nodes =\
            self.action_nodes + self.atomic_fallback_nodes + self.atomic_sequence_nodes
        self.leaf_nodes = self.condition_nodes + self.behavior_nodes
        self.nonleaf_nodes = self.control_nodes + self.up_node

    def merge_behaviors(self, other_bl: 'BehaviorLists') -> None:
        """Merge this behaviors with those of another BehaviorList."""
        self.action_nodes += [
            node for node in other_bl.action_nodes if node not in self.action_nodes]
        self.condition_nodes += [
            node for node in other_bl.condition_nodes if node not in self.condition_nodes]
        self.atomic_fallback_nodes += [
            node for node in other_bl.atomic_fallback_nodes
            if node not in self.atomic_fallback_nodes]
        self.atomic_sequence_nodes += [
            node for node in other_bl.atomic_sequence_nodes
            if node not in self.atomic_sequence_nodes]
        self.fallback_nodes += [
            node for node in other_bl.fallback_nodes if node not in self.fallback_nodes]
        self.sequence_nodes += [
            node for node in other_bl.sequence_nodes if node not in self.sequence_nodes]

    def convert_from_string(self, behavior_tree):
        """ Converts nodes in behavior tree from string format to parameterized nodes """
        for i, node in enumerate(behavior_tree):
            if isinstance(node, str):
                for parameterized_node_template in self.action_nodes + self.condition_nodes:
                    if parameterized_node_template.check_string_match(node):
                        behavior_tree[i] = deepcopy(parameterized_node_template)
                        behavior_tree[i].set_parameters_from_string(node)
                        break

    def is_fallback_node(self, node: str) -> bool:
        """Is node a fallback node."""
        if node in self.fallback_nodes:
            return True
        return False

    def is_sequence_node(self, node: str) -> bool:
        """Is node a sequence node."""
        if node in self.sequence_nodes:
            return True
        return False

    def is_control_node(self, node: str) -> bool:
        """Is node a control node."""
        if node in self.control_nodes:
            return True
        return False

    def is_root_node(self, node: str) -> bool:
        """Is node a valid root node."""
        if self.root_nodes is not None and node not in self.root_nodes:
            return False
        return True

    def get_random_control_node(self) -> common_behaviors.ParameterizedNode:
        """Return a random control node."""
        node = random.choice(self.control_nodes)

        return node

    def get_random_fallback_node(self) -> common_behaviors.ParameterizedNode:
        """Return a random control node."""
        node = random.choice(self.fallback_nodes)

        return node

    def get_random_sequence_node(self) -> common_behaviors.ParameterizedNode:
        """Return a random sequence node."""
        node = random.choice(self.sequence_nodes)

        return node

    def get_random_root_node(self) -> common_behaviors.ParameterizedNode:
        """Return a random root node."""
        if self.root_nodes is not None:
            node = random.choice(self.root_nodes)
        else:
            node = random.choice(self.control_nodes)
        return node

    def is_condition_node(self, node: common_behaviors.ParameterizedNode) -> bool:
        # pylint: disable=no-self-use
        """Is node a condition node."""
        if isinstance(node, common_behaviors.ParameterizedNode):
            return node.condition
        return False

    def get_random_condition_node(self) -> common_behaviors.ParameterizedNode:
        """Return a random condition node."""
        node = deepcopy(random.choice(self.condition_nodes))
        node.randomize_parameters()
        return node

    def is_action_node(self, node: common_behaviors.ParameterizedNode) -> bool:
        # pylint: disable=no-self-use
        """Is node an action node."""
        if isinstance(node, common_behaviors.ParameterizedNode):
            return not node.condition
        return False

    def get_random_action_node(self) -> common_behaviors.ParameterizedNode:
        """Return a random condition node."""
        node = deepcopy(random.choice(self.action_nodes))
        node.randomize_parameters()
        return node

    def is_behavior_node(self, node: common_behaviors.ParameterizedNode) -> bool:
        # pylint: disable=no-self-use
        """Is node a behavior node."""
        if isinstance(node, common_behaviors.ParameterizedNode):
            return not node.condition
        return False

    def get_random_behavior_node(self) -> common_behaviors.ParameterizedNode:
        """Return a random behavior node."""
        node = deepcopy(random.choice(self.behavior_nodes))
        node.randomize_parameters()
        return node

    def is_leaf_node(self, node: common_behaviors.ParameterizedNode) -> bool:
        """Is node a leaf node."""
        return isinstance(node, common_behaviors.ParameterizedNode)

    def get_random_leaf_node(self) -> common_behaviors.ParameterizedNode:
        """Return a random leaf node."""
        node = deepcopy(random.choice(self.leaf_nodes))
        node.randomize_parameters()
        return node

    def is_up_node(self, node: str) -> bool:
        """Is node an up node."""
        return node in self.up_node

    def get_up_node(self) -> str:
        """Return up node."""
        return self.up_node[0]

    def is_valid_node(self, node: common_behaviors.ParameterizedNode) -> bool:
        """Return True if node is valid node, False otherwise."""
        return isinstance(node, common_behaviors.ParameterizedNode) or node in self.nonleaf_nodes
