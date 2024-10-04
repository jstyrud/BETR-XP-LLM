"""Main file for running llm obtea experiments."""

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
import re
from llm import llm_utilities


def goal_transfer_ls_set(goal):
    """ 
    Changes goal to follow format rules to make comparison between LLM answers possible to automate 
    From LLM-OBTEA paper
    """
    goal_dnf = str(llm_utilities.to_dnf(goal, simplify=True, force=True))
    goal_set = []
    goal_ls = goal_dnf.split("|")
    for g in goal_ls:
        g_set = set()
        g = g.replace(" ", "").replace("(", "").replace(")", "")
        g = g.split("&")
        for literal in g:
            if '_' in literal:
                first_part, rest = literal.split('_', 1)
                literal = first_part + '(' + rest
                literal += ')'
                literal = literal.replace('_', ',')
            literal=literal.replace('~', 'Not ')
            g_set.add(literal)
        goal_set.append(g_set)
    goal_set = [sorted(set(item)) for item in goal_set]
    return goal_set

def compile_prompts(file_path, experiment, verbose):
    """ Compiles prompt from files """
    main_prompt = ""
    instructions_list = []
    goals_list = []
    #with llm_utilities.open_file(file_path , "LLM_OBTEA_org_prompt.txt", 'r', encoding="utf-8") as file:
    with llm_utilities.open_file(file_path , "LLM_BTR_prompt.txt", 'r', encoding="utf-8") as file:
    #with llm_utilities.open_file(file_path , "LLM_BTR_prompt_cot.txt", 'r', encoding="utf-8") as file:
    #with llm_utilities.open_file(file_path , "LLM_BTR_prompt_no_desc.txt", 'r', encoding="utf-8") as file:
    #with llm_utilities.open_file(file_path , "LLM_BTR_prompt_no_obj_spec.txt", 'r', encoding="utf-8") as file:
    #with llm_utilities.open_file(file_path , "LLM_BTR_prompt_no_desc_no_obj_spec.txt", 'r', encoding="utf-8") as file:
    #with llm_utilities.open_file(file_path , "LLM_BTR_prompt_old_examples.txt", 'r', encoding="utf-8") as file:
        main_prompt += file.read()
    main_prompt += "\n"

    with llm_utilities.open_file(file_path + "datasets/" , experiment + ".txt", "r", encoding="utf-8") as file:
        data_set = file.read().strip()
    sections = re.split(r'\n\s*\n', data_set)

    for _, section in enumerate(sections):
        instruction, goals = section.strip().splitlines()
        instruction = instruction.strip()
        goals = goals.strip().replace("Goal: ", "")
        instructions_list.append(instruction)
        goals_list.append(goal_transfer_ls_set(goals))
    if verbose:
        print("Main prompt:\n")
        print(main_prompt)
        print("Instructions:")
        print(instructions_list)
        print("Goals:")
        print(goals_list)
    return main_prompt, instructions_list, goals_list

def parse_goal_response(response):
    """ Parses a gpt response and separates reasoning and goal """
    reasoning_first = True
    try:
        reasoning_index = response.index("Reasoning:") + 11
        conditions_index = response.index("Goal:", reasoning_index) + 5
    except ValueError:
        try:
            conditions_index = response.index("Goal:") + 5
            reasoning_index = response.index("Reasoning:", conditions_index) + 11
            reasoning_first = False
        except ValueError:
            try:
                question_index = response.index("Question:") + 10
                return response[question_index:], None
            except ValueError:
                #Usually means the response starts directly and there is no reasoning
                return "", response

    if reasoning_first:
        reasoning = response[reasoning_index:conditions_index - 12]

        #Remove preceding line break if there is one
        if response[conditions_index:conditions_index + 1] == "\n":
            conditions_index += 1

        goal_conditions = response[conditions_index:]
    else:
        reasoning = response[reasoning_index:]

        #Remove preceding line break if there is one
        if response[conditions_index:conditions_index + 1] == "\n":
            conditions_index += 1

        goal_conditions = response[conditions_index:reasoning_index - 12]

    return reasoning, goal_conditions

def get_goal_conditions(client, experiment, verbose):
    """ Gets goal condition by asking gpt and plans a tree to solve it """
    main_prompt, instructions_list, goals_list = compile_prompts("warallmtests/llm/LLM-OBTEA/", experiment, verbose)
    n_correct = 0
    skip_list = []
    if "easy" in experiment:
        skip_list = []
    elif "medium" in experiment:
        skip_list = []
    elif "hard" in experiment:
        skip_list = []
    for _ in range(1):
        for i, instruction in enumerate(instructions_list):
            if i in skip_list:
                continue

            prompt = main_prompt + "\n" + instruction
            completion = llm_utilities.get_completion(client, "warallmtests/llm/LLM-OBTEA/logs/",
                                                      experiment + "_" + str(i + 1), prompt, None, verbose)
            _reasoning, goal_conditions = parse_goal_response(completion.choices[0].message.content)
            goal = goal_conditions.strip().replace("Goal: ", "")
            goal = goal_transfer_ls_set(goal)
            print("Goal " + str(i + 1) + ":")
            if goal == goals_list[i]:
                print(goal)
                print("Correct!")
                n_correct += 1
            else:
                print("Wrong")

    print("Number of correct:")
    print(n_correct)


if __name__ == "__main__":
    client_handle = llm_utilities.get_gpt_client()

    get_goal_conditions(client_handle, "hard_instr_goal", verbose=True)
