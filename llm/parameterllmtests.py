"""Main file for testing llm for finding parameter values in a number of cases."""

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
from llm import llm_utilities


def egg(client_handle):
    """Runs the egg test """
    error_description = 'Failing action: Grasp "egg"\nError message: Grasping force not defined. Please define grasping force in N'
    experiment = "egg"
    reasoning, parameter_value = llm_utilities.resolve_error(client_handle, experiment, None, error_description, None, True, path='warallmtests/llm/parameter_prompts/')
    print("\n")
    print(reasoning)
    print(parameter_value)

def hammer(client_handle):
    """Runs the hammer test """
    error_description = 'Failing action: Grasp "hammer"\nError message: Grasping force not defined. Please define grasping force in N'
    experiment = "hammer"
    reasoning, parameter_value = llm_utilities.resolve_error(client_handle, experiment, None, error_description, None, True, path='warallmtests/llm/parameter_prompts/')
    print("\n")
    print(reasoning)
    print(parameter_value)

def baby(client_handle):
    """Runs the babe test """
    error_description = 'Failing action: Place "baby" in "crib" \nError message: Arm speed not defined. Please define arm speed in m/s'
    experiment = "baby"
    reasoning, parameter_value = llm_utilities.resolve_error(client_handle, experiment, None, error_description, None, True, path='warallmtests/llm/parameter_prompts/')
    print("\n")
    print(reasoning)
    print(parameter_value)

def pillow(client_handle):
    """Runs the pillow test """
    error_description = 'Failing action: Give "pillow" to "user"\nError message: Movement speed not defined. Please define movement speed in m/s'
    experiment = "pillow"
    reasoning, parameter_value = llm_utilities.resolve_error(client_handle, experiment, None, error_description, None, True, path='warallmtests/llm/parameter_prompts/')
    print("\n")
    print(reasoning)
    print(parameter_value)

def sand(client_handle):
    """Runs the sand test """
    error_description = 'Failing action: Lift "sand"\nError message: Tool not defined. Please define tool by name'
    experiment = "sand"
    reasoning, parameter_value = llm_utilities.resolve_error(client_handle, experiment, None, error_description, None, True, path='warallmtests/llm/parameter_prompts/')
    print("\n")
    print(reasoning)
    print(parameter_value)

def plate(client_handle):
    """Runs the plate test """
    error_description = 'Failing action: Clean "plate"\nError message: Tool not defined. Please define tool by name'
    experiment = "plate"
    reasoning, parameter_value = llm_utilities.resolve_error(client_handle, experiment, None, error_description, None, True, path='warallmtests/llm/parameter_prompts/')
    print("\n")
    print(reasoning)
    print(parameter_value)

def first_aid(client_handle):
    """Runs the first aid test """
    error_description = 'Failing action: MoveTo "first aid kit"\nError message: Movement speed not defined. Please define movement speed in m/s'
    experiment = "first_aid"
    reasoning, parameter_value = llm_utilities.resolve_error(client_handle, experiment, None, error_description, None, True, path='warallmtests/llm/parameter_prompts/')
    print("\n")
    print(reasoning)
    print(parameter_value)

if __name__ == "__main__":

    gpt_client = llm_utilities.get_gpt_client()

    egg(gpt_client)
    hammer(gpt_client)
    pillow(gpt_client)
    sand(gpt_client)
    plate(gpt_client)
    first_aid(gpt_client)
    baby(gpt_client)