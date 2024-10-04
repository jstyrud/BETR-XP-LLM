"""Main file for testing llm for resolves in a number of cases."""

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


def red_on_blue(client_handle):
    """Runs the red on blue test """
    error_description = 'Failing action: Grasp "blue cube"\nError message: No collision free path found'
    experiment = "red_on_blue"
    task = 'Please put "blue cube" on "green cube"'
    with llm_utilities.open_file("warallmtests/llm/resolve_prompts/" + "scene_descriptions_vlm/", experiment + ".txt", 'r', encoding="utf-8") as file:
        scene = file.read()
    scene_description = llm_utilities.get_scene_description(client_handle, "warallmtests/saved_images/red on blue.jpg", "warallmtests/saved_images/vlmlogs/redonblue", verbose=True, task=task)
    scene += scene_description
    reasoning, goal_conditions = llm_utilities.resolve_error(client_handle, experiment, scene, error_description, None, True)
    print("\n")
    print(reasoning)
    print(goal_conditions)

def red_on_blue_and_green(client_handle):
    """Runs the red on blue and green test """
    #error_description = 'Failing action: Grasp "blue cube"\nError message: No collision free path found'
    error_description = 'Failing action: Place "blue cube" on "green cube"\nError message: No collision free path found'
    experiment = "red_on_blue_and_green"
    task = 'Please put "blue cube" on "green cube"'
    with llm_utilities.open_file("warallmtests/llm/resolve_prompts/" + "scene_descriptions_vlm/", experiment + ".txt", 'r', encoding="utf-8") as file:
        scene = file.read()
    scene_description = llm_utilities.get_scene_description(client_handle, "warallmtests/saved_images/redongreenyellowonblue.jpg", "warallmtests/saved_images/vlmlogs/redonblue", verbose=True, task=task)
    scene += scene_description
    reasoning, goal_conditions = llm_utilities.resolve_error(client_handle, experiment, scene, error_description, None, True)
    print("\n")
    print(reasoning)
    print(goal_conditions)

def flip_cup(client_handle):
    """Runs the flip cup test """
    error_description = 'Failing action: Place "green cube" in "black cup"\nError message: No collision free path found'
    experiment = 'flip_cup'
    task = 'Please put "green cube" in "black cup"'
    with llm_utilities.open_file("warallmtests/llm/resolve_prompts/" + "scene_descriptions_vlm/", experiment + ".txt", 'r', encoding="utf-8") as file:
        scene = file.read()
    scene_description = llm_utilities.get_scene_description(client_handle, "warallmtests/saved_images/cup up down.jpg", "warallmtests/saved_images/vlmlogs/cupupdown", verbose=True, task=task)
    scene += scene_description
    reasoning, goal_conditions = llm_utilities.resolve_error(client_handle, "flip_cup", scene, error_description, None, True)
    print("\n")
    print(reasoning)
    print(goal_conditions)

def centrifuge(client_handle):
    """Runs the centrifuge test """
    error_description = 'Failing action: Place "test tube" in "centrifuge"\nError message: No collision free path found'
    experiment = 'centrifuge'
    task = 'Please put "test tube" inside "centrifuge"'
    task = None
    with llm_utilities.open_file("warallmtests/llm/resolve_prompts/" + "scene_descriptions_vlm/", experiment + ".txt", 'r', encoding="utf-8") as file:
        scene = file.read()
    scene_description = llm_utilities.get_scene_description(client_handle, "warallmtests/saved_images/centrifuge closed 2.jpg", "warallmtests/saved_images/vlmlogs/centrifuge", verbose=True, task=task)
    scene += scene_description
    reasoning, goal_conditions = llm_utilities.resolve_error(client_handle, "centrifuge", scene, error_description, None, True)
    print("\n")
    print(reasoning)
    print(goal_conditions)

def cupboard(client_handle):
    """Runs the cupboard test """
    error_description = 'Failing action: Open "cupboard"\nError message: Torque limit exceeded'
    reasoning, goal_conditions = llm_utilities.resolve_error(client_handle, "cupboard", None, error_description, None, True)
    print("\n")
    print(reasoning)
    print(goal_conditions)

def unknown(client_handle):
    """Runs the unknown banana test """
    error_description = 'Failing action: MoveTo "banana"\nError message: Object "banana" is not in the dictionary'
    reasoning, goal_conditions = llm_utilities.resolve_error(client_handle, "unknown", None, error_description, None, True)
    print("\n")
    print(reasoning)
    print(goal_conditions)

def unreachable(client_handle):
    """Runs the unreachable banana test """
    error_description = 'Failing action: Grasp "banana"\nError message: Position of out reach'
    reasoning, goal_conditions = llm_utilities.resolve_error(client_handle, "unreachable", None, error_description, None, True)
    print("\n")
    print(reasoning)
    print(goal_conditions)

def no_coffee(client_handle):
    """Runs the no coffee test """
    error_description = 'Failing action: Grasp Coffee\nError message: Coffee not found'
    reasoning, goal_conditions = llm_utilities.resolve_error(client_handle, "no_coffee", None, error_description, None, True, subfolder="obtea/")
    print("\n")
    print(reasoning)
    print(goal_conditions)

def find_fries(client_handle):
    """Runs the find fries test """
    error_description = 'Failing action: Grasp Chips\nError message: Position of out reach'
    reasoning, goal_conditions = llm_utilities.resolve_error(client_handle, "find_fries", None, error_description, None, True, subfolder="obtea/")
    print("\n")
    print(reasoning)
    print(goal_conditions)

def sweep(client_handle):
    """Runs the sweep test """
    error_description = 'Failing action: Sweep Floor\nError message: Postcondition IsClean_Floor not met after Sweep action completion'
    reasoning, goal_conditions = llm_utilities.resolve_error(client_handle, "sweep", None, error_description, None, True, subfolder="obtea/")
    print("\n")
    print(reasoning)
    print(goal_conditions)



if __name__ == "__main__":

    gpt_client = llm_utilities.get_gpt_client()

    red_on_blue(gpt_client)
    red_on_blue_and_green(gpt_client)
    flip_cup(gpt_client)
    centrifuge(gpt_client)
    cupboard(gpt_client)
    unknown(gpt_client)
    unreachable(gpt_client)
    no_coffee(gpt_client)
    find_fries(gpt_client)
    sweep(gpt_client)