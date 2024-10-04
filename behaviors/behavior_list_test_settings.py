"""Test settings for behavior_lists.py module."""

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

from behaviors.common_behaviors import NodeParameter, ParameterizedNode

par1 = NodeParameter([], min=1, max=1, step=1, value=1)
par2 = NodeParameter([], min=1, max=1, step=1, value=1)


def get_condition_nodes():
    """Return a list of condition nodes."""
    return [
        ParameterizedNode('b', None, {}, True),
        ParameterizedNode('c', None, {}, True),
        ParameterizedNode('d', None, {"value": NodeParameter(['0', '100'])}, True),
        ParameterizedNode('e', None, {"value": NodeParameter([], 0, 100, 100)}, True),
        ParameterizedNode('value check', None, {"value": NodeParameter([], 0, 100, 100)}, True)
        ]


def get_action_nodes():
    """Return a list of action nodes."""
    return [
        ParameterizedNode('ab', None, {"value1": par1, "value2": par2}, False),
        ParameterizedNode('ac', None, {"value1": par1, "value2": par2}, False),
        ParameterizedNode('ad', None, {"value1": par1, "value2": par2}, False),
        ParameterizedNode('ae', None, {"value1": par1, "value2": par2}, False),
        ParameterizedNode('af', None, {"value1": par1, "value2": par2}, False),
        ParameterizedNode('ag', None, {}, False)
        ]
