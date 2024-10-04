"""Unit test for behaviors.py module."""

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
import pytest
import py_trees as pt
from behaviors import common_behaviors


def test_random_range():
    """Test the random_range function."""
    assert common_behaviors.random_range(0, 1000, 0) == 0

    random_numbers = []
    for _ in range(100):
        random_numbers.append(common_behaviors.random_range(0, 10, 1))

    assert max(random_numbers) == 10
    assert min(random_numbers) == 0


def test_random_range_float():
    """Test the random_range_float function."""
    assert common_behaviors.random_range_float(0, 1000, 0) == 0

    random_numbers = []
    for _ in range(100):
        random_numbers.append(common_behaviors.random_range_float(0, 10, 1))

    assert max(random_numbers) == 10
    assert min(random_numbers) == 0

    random_numbers = []

    for _ in range(100):
        random_numbers.append(common_behaviors.random_range_float(-0.2, 0.2, 0.05))

    assert max(random_numbers) == 0.2
    assert min(random_numbers) == -0.2
    assert set(random_numbers).issubset([-0.2, -0.15, -0.1, -0.05, 0.0, 0.05, 0.1, 0.15, 0.2])


def test_randomize_value():
    """Test the randomize_value function."""
    parameter = common_behaviors.NodeParameter([], 0, 1, 1)
    parameter.randomize_value()
    assert parameter.value in (0, 1)

    parameter = common_behaviors.NodeParameter(
        [], (0, 1, 2), (1, 3, 5), (1, 2, 3), data_type=common_behaviors.ParameterTypes.POSITION)
    parameter.randomize_value()
    assert parameter.value[0] in (0, 1)
    assert parameter.value[1] in (1, 3)
    assert parameter.value[2] in (2, 5)

    parameter = common_behaviors.NodeParameter(
        [], (0, 1, 2), (1, 3, 5), (1, 1, 1), data_type=common_behaviors.ParameterTypes.POSITION)
    parameter.randomize_value()
    assert 0 <= parameter.value[0] <= 1
    assert 1 <= parameter.value[1] <= 3
    assert 2 <= parameter.value[2] <= 5

    parameter = common_behaviors.NodeParameter(
        [], (0.0, 1.0, 0.0), (1.0, 3.0, 0.0), (1.0, 1.0, 0.0),
        data_type=common_behaviors.ParameterTypes.POSITION, random_step=True)
    parameter.randomize_value()
    assert 0 <= parameter.value[0] <= 1
    assert 1 <= parameter.value[1] <= 3
    assert parameter.value[2] == 0

    parameter = common_behaviors.NodeParameter(
        [], 0, 1, 0.1, data_type=common_behaviors.ParameterTypes.FLOAT)
    parameter.randomize_value()
    assert 0 <= parameter.value <= 1

    parameter = common_behaviors.NodeParameter(
        [], 0, 1, 0.0, data_type=common_behaviors.ParameterTypes.FLOAT)
    parameter.randomize_value()
    assert parameter.value == 0

    random.seed(10)
    parameter = common_behaviors.NodeParameter(data_type=common_behaviors.ParameterTypes.STRING)
    parameter.randomize_value()
    assert 'upuh3' == parameter.value

    parameter = common_behaviors.NodeParameter([], data_type=-1)
    with pytest.raises(Exception):
        parameter.randomize_value()


def test_randomize_parameters():
    """Test the randomize_parameters function of ParameterizedNode."""
    parameters = {"value": common_behaviors.NodeParameter([0, 1]), "larger_than": common_behaviors.NodeParameter([True, False])}
    node = common_behaviors.ParameterizedNode('a', None, parameters=parameters)
    node.randomize_parameters()
    assert node.parameters["value"].value in (0, 1)
    assert node.parameters["larger_than"].value in (True, False)


def test_get_parameters():
    """Test the get_parameters function of ParameterizedNode."""
    parameters = {"value1": common_behaviors.NodeParameter(value=0), "value2": common_behaviors.NodeParameter(value=1)}
    node = common_behaviors.ParameterizedNode('a', None, parameters=parameters)
    parameters = node.get_parameters()

    assert parameters["value1"] == 0
    assert parameters["value2"] == 1

def test_to_string():
    """Test the to_string function of ParameterizedNode."""
    node = common_behaviors.ParameterizedNode('a')
    assert node.to_string() == 'a?'

    node = common_behaviors.ParameterizedNode('a', condition=False)
    assert node.to_string() == 'a!'

    parameters = {"value": common_behaviors.NodeParameter(value=0)}
    node = common_behaviors.ParameterizedNode('a', parameters=parameters)
    assert node.to_string() == 'a 0?'

    parameters = {"value": common_behaviors.NodeParameter(
        data_type=common_behaviors.ParameterTypes.FLOAT, value=0.1)}
    node = common_behaviors.ParameterizedNode('a', parameters=parameters)
    assert node.to_string() == 'a 0.1?'

    parameters = {"value": common_behaviors.NodeParameter(
        data_type=common_behaviors.ParameterTypes.POSITION, value=(0.1, 0.2, 0.3))}
    node = common_behaviors.ParameterizedNode('a', parameters=parameters)
    assert node.to_string() == 'a (0.1, 0.2, 0.3)?'

    parameters = {"value": common_behaviors.NodeParameter(value=0, placement=0)}
    node = common_behaviors.ParameterizedNode('a', parameters=parameters)
    assert node.to_string() == '0 a?'

    parameters = {"value": common_behaviors.NodeParameter(value=0, placement=1)}
    node = common_behaviors.ParameterizedNode('ab', parameters=parameters)
    assert node.to_string() == 'a 0 b?'


def test_sequence_with_memory():
    """Test sequence with memory."""
    root, _ = common_behaviors.get_node('sm(')
    root.add_child(pt.behaviours.TickCounter("", 1, pt.common.Status.SUCCESS))
    root.add_child(pt.behaviours.Success('Reached second child'))  # pylint: disable=abstract-class-instantiated
    root.tick_once()
    assert root.status.value == "RUNNING"
    root.tick_once()
    assert root.status.value == "SUCCESS"


def test_sequence_without_memory():
    """Test sequence without memory."""
    root, _ = common_behaviors.get_node('s(')
    root.add_child(pt.behaviours.TickCounter("", 1, pt.common.Status.SUCCESS))
    root.add_child(pt.behaviours.Success('Reached second child'))  # pylint: disable=abstract-class-instantiated
    root.tick_once()
    assert root.status.value == "RUNNING"
    root.tick_once()
    assert root.status.value == "SUCCESS"

    root, _ = common_behaviors.get_node('s(')
    root.add_child(pt.behaviours.TickCounter("", 1, pt.common.Status.SUCCESS))
    root.add_child(pt.behaviours.TickCounter("", 1, pt.common.Status.SUCCESS))

    for _ in range(6):
        root.tick_once()
        assert root.status.value == "RUNNING"


def test_fallback_with_memory():
    """Test fallback with memory."""
    root, _ = common_behaviors.get_node('fm(')
    root.add_child(pt.behaviours.TickCounter("ticker1", 1, pt.common.Status.FAILURE))
    root.add_child(pt.behaviours.TickCounter("ticker2", 1, pt.common.Status.SUCCESS))
    root.tick_once()
    assert root.status.value == "RUNNING"
    root.tick_once()
    assert root.status.value == "RUNNING"
    root.tick_once()
    assert root.status.value == "SUCCESS"


def test_fallback_without_memory():
    """Test fallback without memory."""
    root, _ = common_behaviors.get_node('f(')
    root.add_child(pt.behaviours.TickCounter("ticker1", 1, pt.common.Status.FAILURE))
    root.add_child(pt.behaviours.TickCounter("ticker2", 1, pt.common.Status.SUCCESS))

    for _ in range(6):
        root.tick_once()
        assert root.status.value == "RUNNING"


def test_fallback_random():
    """Test random selector."""
    root, _ = common_behaviors.get_node('fr(')
    root.add_child(pt.behaviours.TickCounter('first ticker', 1, pt.common.Status.FAILURE))
    root.add_child(pt.behaviours.TickCounter('second ticker', 1,  pt.common.Status.SUCCESS))

    # in case the random fallback ticks the FIRST child:
    #   since it returns FAILURE, the tree will then tick the SECOND child.
    #   the execution will thus be:
    #       RUNNING (tick first)
    #       RUNNING (first fails --> tick second)
    #       SUCCESS (second succeeds)
    # in case the random fallback ticks the SECOND child:
    #   since it returns SUCCESS, the execution will thus be:
    #       RUNNING (tick second)
    #       SUCCESS (second succeeds)
    #       RUNNING (root, then random child ticked again)
    status_sequence = []
    for _ in range(3):
        root.tick_once()
        status_sequence.append(root.status.value)

    assert status_sequence == ['RUNNING', 'SUCCESS', 'RUNNING'] or\
        status_sequence == ['RUNNING', 'RUNNING', 'SUCCESS']

    running_ctr = 0
    success_ctr = 0
    other_ctr = 0
    for _ in range(50):
        for _ in range(3):
            root.tick_once()
        if root.status == pt.common.Status.SUCCESS:
            success_ctr += 1
        elif root.status == pt.common.Status.RUNNING:
            running_ctr += 1
        else:
            other_ctr += 1
        root.stop()
    # both counter should be roughly around 25 (50% each)
    assert running_ctr >= 20 and success_ctr >= 20
    # never return FAILURE or INVALID
    assert other_ctr == 0


def test_parallel():
    """Test parallel node."""
    root, _ = common_behaviors.get_node('p(')
    root.add_child(pt.behaviours.TickCounter('first ticker', 1, pt.common.Status.SUCCESS))
    root.add_child(pt.behaviours.TickCounter('second ticker', 1, pt.common.Status.SUCCESS))
    for _ in range(2):
        root.tick_once()
    assert root.status == pt.common.Status.SUCCESS
