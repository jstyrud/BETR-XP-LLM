"""Unit test for behavior_lists.py module."""

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

from behaviors import behavior_list_test_settings, behavior_lists, common_behaviors

behavior_list = behavior_lists.BehaviorLists(
    condition_nodes=behavior_list_test_settings.get_condition_nodes(),
    action_nodes=behavior_list_test_settings.get_action_nodes()
)


def test_init():
    """Test the is_condition_node function."""
    _ = behavior_lists.BehaviorLists(fallback_nodes=['f('], sequence_nodes=['s('])


def test_merge():
    """Test the merge BehaviorLists function."""
    new_behavior_list = behavior_lists.BehaviorLists(
        condition_nodes=[common_behaviors.ParameterizedNode('condition1', None, [], True)],
        action_nodes=[common_behaviors.ParameterizedNode('action1', None, [], False)]
    )

    behavior_list.merge_behaviors(new_behavior_list)
    assert new_behavior_list.condition_nodes[0] in behavior_list.condition_nodes
    assert new_behavior_list.action_nodes[0] in behavior_list.action_nodes


def test_is_fallback_node():
    """Test the is _fallback_node function."""
    assert behavior_list.is_fallback_node('f(')
    assert not behavior_list.is_fallback_node('s(')


def test_is_sequence_node():
    """Test the is_sequence_node function."""
    assert not behavior_list.is_sequence_node('f(')
    assert behavior_list.is_sequence_node('s(')


def test_get_random_control_node():
    """Test the get_random_control_node function."""
    for _ in range(10):
        assert behavior_list.is_control_node(behavior_list.get_random_control_node())


def test_get_random_fallback_node():
    """Test the get_random_fallback_node function."""
    for _ in range(10):
        assert behavior_list.is_fallback_node(behavior_list.get_random_fallback_node())


def test_get_random_sequence_node():
    """Test the get_random_sequence_node function."""
    for _ in range(10):
        assert behavior_list.is_sequence_node(behavior_list.get_random_sequence_node())


def test_get_random_root_node():
    """Test the get_random_root_node function."""
    for _ in range(10):
        assert behavior_list.is_root_node(behavior_list.get_random_root_node())


def test_get_random_condition_node():
    """Test the get_random_condition_node function."""
    for _ in range(10):
        assert behavior_list.is_condition_node(behavior_list.get_random_condition_node())


def test_get_random_behavior_node():
    """Test the get_random_behavior_node function."""
    for _ in range(10):
        assert behavior_list.is_behavior_node(behavior_list.get_random_behavior_node())


def test_get_random_leaf_node():
    """Test the get_random_leaf_node function."""
    for _ in range(10):
        assert behavior_list.is_leaf_node(behavior_list.get_random_leaf_node())
