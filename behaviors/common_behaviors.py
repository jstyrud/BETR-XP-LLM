# pylint: disable=broad-exception-raised
"""Implementing various common py trees behaviors."""

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
from typing import Any, Tuple
from dataclasses import dataclass
from dataclasses import field
from copy import deepcopy
from enum import IntEnum
import string
import numpy as np

import py_trees as pt

class ParameterTypes(IntEnum):
    """Define the parameter types."""

    INDEX = 0  # Index of a list, typically in a list of objects
    POSITION = 1  # A position consists of three values, x y and z
    INTEGER = 2
    FLOAT = 3
    STRING = 4  # Used to describe object targets semantically
    TUPLE = 5  # Used for storing other parameters that don't fall in previous cases


def random_range(min_value: int, max_value: int, step: int) -> int:
    """
    Return a value from random range with some checks for step.

    Also includes max in the range unlike randrange.
    """
    if step == 0:
        return min_value
    return random.randrange(min_value, max_value + step, step)


def random_range_float(min_value, max_value, step):
    """
    Gives a float in random range with discretized steps
    Adding 0 avoids representing 0 as -0.0
    """
    if step == 0:
        return min_value
    n = 1 / step
    return np.round(np.random.randint(min_value * n, (max_value + step) * n) * step, 3) + 0


@dataclass
class NodeParameter():
    """Define a parameter for a parameterized node and how to handle it."""

    list_of_values: list = field(default_factory=list)
    min: Any = None
    max: Any = None
    step: Any = None
    placement: int = -1  # Placement within string, just for readability
    data_type: Any = ParameterTypes.INTEGER
    value: Any = None  # Current value of this parameter
    random_step: bool = False  # Whether to randomize with random step instead of complete reinitialization
    standard_deviation: Any = 1  # Standard deviation of random step length
    use_prior: bool = False  # Current value and standard_deviation can be used as prior

    def randomize_value(self):
        """Give the parameter a random value within the constraints."""
        if self.list_of_values:
            self.value = random.choice(self.list_of_values)
        elif self.data_type in (ParameterTypes.INDEX, ParameterTypes.INTEGER):
            self.value = random_range(self.min, self.max, self.step)
        elif self.data_type == ParameterTypes.POSITION:
            if isinstance(self.min[0], int):  # Integer values
                self.value = (random_range(self.min[0], self.max[0], self.step[0]),
                              random_range(self.min[1], self.max[1], self.step[1]),
                              random_range(self.min[2], self.max[2], self.step[2]))
            else:
                if not self.value:
                    self.value = [0.0, 0.0, 0.0]
                else:
                    self.value = list(self.value)
                for i in range(3):
                    if self.step[i] == 0 or self.step[i] is None:
                        self.value[i] = self.min[i]
                    elif self.random_step:
                        self.value[i] += np.random.normal(scale=self.standard_deviation)
                        self.value[i] = max(min(self.value[i], self.max[i]), self.min[i])
                        self.value[i] = np.round(np.round(self.value[i] / self.step[i], 0) *
                                                 self.step[i], 3) + 0  # Multiple of step
                    else:
                        self.value[i] = random_range_float(self.min[i], self.max[i], self.step[i])
                self.value = tuple(self.value)
        elif self.data_type == ParameterTypes.FLOAT:
            if self.step == 0 or self.step is None:
                self.value = self.min
            elif self.random_step:
                if not self.value:
                    self.value = 0.0

                self.value += np.random.normal(scale=self.standard_deviation)
                self.value = max(min(self.value, self.max), self.min)
                self.value = np.round(np.round(self.value / self.step, 0) * self.step, 3) + 0  # Multiple of step
            else:
                self.value = random_range_float(self.min, self.max, self.step)
        elif self.data_type == ParameterTypes.STRING:
            self.value = ''.join(random.choices(
                string.ascii_lowercase + string.digits, k=5))
        else:
            raise Exception('Unknown data_type: ', self.data_type)


class ParameterizedNode():
    """ A parameterized node is a node with parameters and how to handle those parameters """

    def __init__(
        self,
        name: str,
        behavior: Any = None,
        parameters: dict = None,
        condition: bool = True
    ):
        # pylint: disable=too-many-arguments
        self.name = name
        self.behavior = behavior
        self.parameters = deepcopy(parameters) if parameters else {}
        self.condition = condition
        self.print_floats = True

    def __eq__(self, other: Any) -> bool:
        if not isinstance(other, ParameterizedNode):
            # don't attempt to compare against unrelated types
            return False

        return self.name == other.name and self.parameters == other.parameters and \
            self.condition == other.condition

    def __repr__(self) -> str:
        """Return the string version of the node."""
        return ''.join(("'", self.to_string(), "'"))

    def check_string_match(self, node_string):
        """ Check if the string description of the node matches this node """
        try:
            return self.behavior.check_string_match(node_string, self.parameters)
        except AttributeError:
            return self.name in node_string

    def get_parameters(self):
        """Return parameter values."""
        parameters = {}
        if self.parameters:
            for name, parameter in self.parameters.items():
                parameters[name] = parameter.value
        return parameters

    def set_parameters_from_string(self, node_descriptor):
        """ Set parameter values of the node by parsing a string """
        parsed_parameters = self.behavior.parse_parameters(node_descriptor)
        for name, parameter in parsed_parameters.items():
            self.parameters[name].value = parameter

    def randomize_parameters(self):
        """ Randomize node parameters."""
        if self.parameters:
            for name, parameter in self.parameters.items():  # pylint: disable=not-an-iterable
                parameter_valid = False
                while not parameter_valid:
                    parameter.randomize_value()
                    if hasattr(self.behavior, "is_parameter_valid"):
                        parameter_valid = self.behavior.is_parameter_valid(self.parameters, name)
                    else:
                        parameter_valid = True

    def to_string(self) -> str:
        """Return string representation of node for printing/logging/hashing."""
        if hasattr(self.behavior, "to_string"):
            return self.behavior.to_string(self.get_parameters())
        else:
            string_node = self.name
            if self.parameters:
                for parameter in self.parameters:  # pylint: disable=not-an-iterable
                    parameter_string = ''
                    placement = self.parameters[parameter].placement

                    if self.print_floats:
                        parameter_string += str(self.parameters[parameter].value)

                        if placement == 0:
                            string_node = ''.join((parameter_string, ' ', string_node))
                        elif placement == -1:
                            string_node = ''.join((string_node, ' ', parameter_string))
                        else:
                            string_node = string_node[:placement] +\
                                ' ' + parameter_string + ' ' + \
                                string_node[placement:]
                    else:
                        string_node += parameter_string

            if self.condition:
                string_node += '?'
            else:
                string_node += '!'
            return string_node.strip()

    def set_behavior(self, behavior: Any):
        """Set the associated behavior."""
        self.behavior = behavior


def get_node(
    node_descriptor: Any = None,
    world_interface: Any = None,
    verbose: bool = False
) -> Tuple[Any, bool]:
    """Return a py_trees behavior or composite given the descriptor."""
    has_children = False

    if isinstance(node_descriptor, ParameterizedNode):
        node = node_descriptor.behavior(
            '', node_descriptor.get_parameters(), world_interface, verbose)
    else:
        if node_descriptor == 'f(':
            node = pt.composites.Selector('Fallback', memory=False)
            has_children = True
        elif node_descriptor == 'fm(':
            node = pt.composites.Selector('Fallback', memory=True)
            has_children = True
        elif node_descriptor == 'fr(':
            node = RandomSelector('RandomSelector')
            has_children = True
        elif node_descriptor == 's(':
            node = pt.composites.Sequence('Sequence', memory=False)
            has_children = True
        elif node_descriptor == 'sm(':
            node = pt.composites.Sequence('Sequence', memory=True)
            has_children = True
        elif node_descriptor == 'p(':
            node = pt.composites.Parallel(
                name='Parallel',
                policy=pt.common.ParallelPolicy.SuccessOnAll(synchronise=False))
            has_children = True
        else:
            print("Warning: Unrecognized node. Adding generic node")
            node = ActionBehavior(node_descriptor, {}, world_interface, [], [])

    return node, has_children

class Behavior(pt.behaviour.Behaviour):
    """ The general behavior implementation. """
    def __init__(self, name: str, parameters, world_interface: Any, verbose: bool = False):
        self.world_interface = world_interface
        self.verbose = verbose
        self.negated = False
        self.parameters = parameters.copy()
        try:
            if parameters["not"]:
                self.negated = True
        except KeyError:
            pass
        super().__init__(name)

    def __eq__(self, other) -> bool:
        if self.name != other.name:
            return False
        for parameter in self.parameters:
            if isinstance(self.parameters[parameter], np.ndarray):
                if not np.array_equal(self.parameters[parameter], other.parameters[parameter]):
                    return False
            else:
                if self.parameters[parameter] != other.parameters[parameter]:
                    return False
        return True

    @staticmethod
    def parse_parameters(node_descriptor):
        """ Standard parameter parsing only looking for target_object and not """
        parameters = {}
        n_marks = 2
        marks = [0] * n_marks
        marks[0] = node_descriptor.find('"')
        for i in range(1, n_marks):
            if marks[i - 1] >= 0:
                marks[i] = node_descriptor.find('"', marks[i - 1] + 1)
            else:
                print("Error, parameter parsing failed")
                return None

        parameters["target_object"] = node_descriptor[marks[0]: marks[1] + 1]
        parameters["not"] = node_descriptor[0] == "~"

        return parameters

    def get_parameters(self):
        """ Returns parameters of the condition """
        return self.parameters

    @staticmethod
    def common_string_rules(node_string, parameters):
        """ Changes string using common string rules """
        try:
            if parameters["not"]:
                node_string = ''.join(("~", node_string))
        except KeyError:
            pass

        return node_string

    def check_negated(self, success):
        """ Handle whether the condition should be negated or not """
        if self.negated and not success or \
            not self.negated and success:
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE


class ActionBehavior(Behavior):
    """
    Class template for action behaviors
    """
    def __init__(self, name, parameters, world_interface, preconditions, postconditions, max_ticks=50, verbose=False):
        self.state = None
        self.counter = 0
        self.max_ticks = max_ticks
        self.preconditions = preconditions
        self.postconditions = postconditions
        super().__init__(name, parameters, world_interface)

    def initialise(self) -> None:
        self.counter = 0
        self.state = pt.common.Status.RUNNING

    @staticmethod
    def parse_parameters(node_descriptor):
        """ Standard parameter parsing only looking for target_object and not """
        parameters = {}
        n_marks = 2
        marks = [0] * n_marks
        marks[0] = node_descriptor.find('"')
        for i in range(1, n_marks):
            if marks[i - 1] >= 0:
                marks[i] = node_descriptor.find('"', marks[i - 1] + 1)
            else:
                print("Error, parameter parsing failed")
                return None

        parameters["target_object"] = node_descriptor[marks[0]: marks[1] + 1]
        return parameters

    def update(self) -> None:
        self.counter += 1
        if self.state == pt.common.Status.RUNNING:
            if self.counter > self.max_ticks:
                self.failure()
        if self.verbose and self.state == pt.common.Status.RUNNING:
            print(self.name, ':', self.state)

    def success(self) -> None:
        """Set state success."""
        self.state = pt.common.Status.SUCCESS
        if self.verbose:
            print(self.name, ': SUCCESS')

    def failure(self) -> None:
        """Set state failure."""
        self.state = pt.common.Status.FAILURE
        self.world_interface.set_failed_behavior(self.name)
        if self.verbose:
            print(self.name, ': FAILURE')
        return self.state

    def cost(self) -> int:
        """Define the cost of the action."""
        return 1

    def get_preconditions(self):
        """ Returns list of preconditions """
        return self.preconditions

    def get_postconditions(self):
        """ Returns list of postconditions """
        return self.postconditions

    def has_postcondition_check(self):
        """
        Most behaviors check postconditions first and
        only execute if they are not already fulfilled
        """
        return True
    
    def terminate(self, _status):
        """ Cleanup after stop """
        self.world_interface.set_manipulation_target(None)

class ComparisonCondition(pt.behaviour.Behaviour):
    """Class template for conditions comparing against constants."""

    def __init__(
        self, name: str,
        parameters: list,
        world_interface: Any,
        _verbose: bool = False
    ):
        self.world_interface = world_interface
        self.larger_than = parameters["larger_than"]
        self.value = float(parameters["value"])
        super().__init__(name)

    def compare(self, variable: Any) -> pt.common.Status:
        """Compare input variable to stored value."""
        if (self.larger_than and variable > self.value) or \
           (not self.larger_than and variable < self.value):
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE


class RandomSelector(pt.composites.Selector):
    """
    Random selector node for py_trees
    """
    def __init__(self, name='RandomSelector', children=None):
        super().__init__(name=name, memory=False, children=children)

    def tick(self):
        """
        Run the tick behaviour for this selector.

        Note that the status of the tick is always determined by its children,
        not by the user customized update function.

        Yields
        ------
            class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children.

        """
        self.logger.debug('%s.tick()' % self.__class__.__name__)  # pylint: disable=consider-using-f-string
        # initialise
        if self.status == pt.common.Status.FAILURE or self.status == pt.common.Status.INVALID:
            # selector specific initialization - leave initialise() free for users to
            # re-implement without having to make calls to super()
            self.logger.debug(
                '%s.tick() [!RUNNING->reset current_child]' % self.__class__.__name__)  # pylint: disable=consider-using-f-string
            if len(self.children) > 1:
                # Select one child at random except the child we last tried executing.
                # If self.current_child is None we will choose a child entirely at random.
                self.current_child = random.choice(
                    [child for child in self.children if child is not self.current_child])
            elif len(self.children) == 1:
                # If there is only one child we should always execute it
                self.current_child = self.children[0]
            else:
                self.current_child = None

            # reset the children - don't need to worry since they will be handled
            # a) prior to a remembered starting point, or
            # b) invalidated by a higher level priority

            # user specific initialization
            self.initialise()

        for child in self.children:
            if child is not self.current_child:
                child.stop(new_status=pt.common.Status.SUCCESS)

        # customized work
        self.update()

        # nothing to do
        if not self.children:
            self.current_child = None
            self.stop(pt.common.Status.FAILURE)
            yield self
            return

        # actual work
        previous_children = []
        while len(previous_children) < len(self.children):
            for node in self.current_child.tick():
                yield node
                if node is self.current_child:
                    if node.status == pt.common.Status.RUNNING or\
                       node.status == pt.common.Status.SUCCESS:
                        self.status = node.status
                        yield self
                        return
            previous_children.append(self.current_child)
            children_left = [child for child in self.children if child not in previous_children]
            if len(children_left) > 0:
                # Don't set current_child in last loop so we remember the last
                # child that failed
                self.current_child = random.choice(children_left)
        # all children failed,
        # set failure ourselves and current child to the last bugger who failed us
        self.status = pt.common.Status.FAILURE
        yield self
