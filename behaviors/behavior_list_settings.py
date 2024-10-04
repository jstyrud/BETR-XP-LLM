"""Define the behavior list for cube pnp tasks."""

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
from behaviors import behavior_lists as bl
from behaviors.common_behaviors import NodeParameter, ParameterizedNode
import behaviors.behaviors as behaviors


def get_behavior_list(objects=None,
                      relations=None,
                      sequence_nodes=None,
                      root_nodes=None) -> bl.BehaviorLists:
    """Return the behavior list."""
    if objects is None:
        objects = []
    if relations is None:
        relations = ['on', 'in']

    condition_nodes = []
    condition_nodes.append(ParameterizedNode(
        name='',
        behavior=behaviors.AtPos,
        parameters={"not": NodeParameter([True, False]),
                    "target_object": NodeParameter(['"any object"'] + objects),
                    "relation": NodeParameter(relations),
                    "relative_object": NodeParameter(['none'] + objects)},
        condition=True
        ))
    condition_nodes.append(ParameterizedNode(
        name='grasped',
        behavior=behaviors.Grasped,
        parameters={"not": NodeParameter([True, False]),
                    "target_object": NodeParameter(objects)},
        condition=True
        ))
    condition_nodes.append(ParameterizedNode(
        name='location known',
        behavior=behaviors.LocationKnown,
        parameters={"not": NodeParameter([True, False]),
                    "target_object": NodeParameter(objects)},
        condition=True
        ))
    condition_nodes.append(ParameterizedNode(
        name='upright',
        behavior=behaviors.Upright,
        parameters={"not": NodeParameter([True, False]),
                    "target_object": NodeParameter(objects)},
        condition=True
        ))
    condition_nodes.append(ParameterizedNode(
        name='near robot',
        behavior=behaviors.NearRobot,
        parameters={"not": NodeParameter([True, False]),
                    "target_object": NodeParameter(objects)},
        condition=True
        ))
    condition_nodes.append(ParameterizedNode(
        name='opened',
        behavior=behaviors.Opened,
        parameters={"not": NodeParameter([True, False]),
                    "target_object": NodeParameter(objects)},
        condition=True
        ))
    condition_nodes.append(ParameterizedNode(
        name='unlocked',
        behavior=behaviors.Unlocked,
        parameters={"not": NodeParameter([True, False]),
                    "target_object": NodeParameter(objects)},
        condition=True
        ))

    action_nodes = []

    action_nodes.append(
        ParameterizedNode(
            name='grasp',
            behavior=behaviors.Grasp,
            parameters={"target_object": NodeParameter(['"any object"'] + objects),
                        "relation": NodeParameter(relations),
                        "relative_object": NodeParameter(['none'] + objects)},
            condition=False
        ))
    action_nodes.append(
        ParameterizedNode(
            name='place',
            behavior=behaviors.Place,
            parameters={"target_object": NodeParameter(objects),
                        "relation": NodeParameter(relations),
                        "relative_object": NodeParameter(['none'] + objects)},
            condition=False
        ))
    action_nodes.append(
        ParameterizedNode(
            name='move home',
            behavior=behaviors.MoveHome,
            parameters={},
            condition=False
        ))
    action_nodes.append(
        ParameterizedNode(
            name='flip',
            behavior=behaviors.Flip,
            parameters={"target_object": NodeParameter(objects)},
            condition=False
        ))
    action_nodes.append(
        ParameterizedNode(
            name='open centrifuge',
            behavior=behaviors.OpenCentrifuge,
            parameters={},
            condition=False
        ))

    if root_nodes is None:
        root_nodes = ['s(']

    behavior_list = bl.BehaviorLists(sequence_nodes=sequence_nodes, condition_nodes=condition_nodes, action_nodes=action_nodes,
                                     root_nodes=root_nodes)

    return behavior_list
