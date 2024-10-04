""" Utility functions for running LLMs."""

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
import pickle
from pathlib import Path
import datetime
import dateutil.tz
import base64
from mimetypes import guess_type
from sympy import to_dnf
from openai import AzureOpenAI


def open_file(path, filename, mode, encoding=None):
    """
    Attempt to open file at path.

    Tried up to max_attempts times because of intermittent permission errors on windows.
    """
    max_attempts = 100
    f = None
    Path(path).mkdir(parents=True, exist_ok=True)
    for _ in range(max_attempts):  # pragma: no branch
        try:
            f = open(path + filename, mode, encoding=encoding)
        except PermissionError:  # pragma: no cover
            continue
        break
    return f

def get_new_filename():
    """ Returns new filename from date """
    return datetime.datetime.now(dateutil.tz.tzlocal()).strftime("%Y_%m_%d__%H_%M_%S")

def goal_transfer_ls_set(goal):
    """ 
    Changes goal to follow format rules to make comparison between LLM answers possible to automate 
    From LLM-OBTEA paper
    """
    goal_dnf = str(to_dnf(goal, simplify=True, force=True))
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

def compile_prompt(file_path, experiment, scene, instruction, verbose):
    """ Compiles prompt from files """
    prompt = ""
    with open_file(file_path, "intro_and_conditions.txt", 'r', encoding="utf-8") as file:
        prompt += file.read()
    prompt += "\n\n\n"
    try:
        with open_file(file_path, experiment + "_scene_information.txt", "r", encoding="utf-8") as file:
            prompt += file.read()
    except FileNotFoundError:
        pass
    prompt += scene
    prompt += "\n\n"
    with open_file(file_path, "specification.txt", "r", encoding="utf-8") as file:
        prompt += file.read()
    prompt += "\n"
    prompt += instruction
    if verbose:
        print("Prompt:\n")
        print(prompt)
    return prompt

def get_price(input_tokens, output_tokens, model_name):
    """ Returns openai price depending on tokens and model """
    if model_name == "gpt-3.5-turbo-1106" or model_name == "gpt-35-turbo-16k":
        return (input_tokens * 0.001 + output_tokens * 0.002) / 1000
    elif model_name == "gpt-4" or model_name == "gpt-4-1106-Preview" or \
        model_name == "gpt-4-1106-preview" or model_name == "gpt-4-1106-vision-preview":
        return (input_tokens * 0.01 + output_tokens * 0.03) / 1000
    elif model_name == "gpt-4-32k":
        return (input_tokens * 0.06 + output_tokens * 0.12) / 1000
    else:
        print('Pricing unknown because model is not known.')

def local_image_to_data_url(image_path):
    """
    Function to encode a local image into data URL 
    """
    # Guess the MIME type of the image based on the file extension
    mime_type, _ = guess_type(image_path)
    if mime_type is None:
        mime_type = 'application/octet-stream'  # Default MIME type if none is found

    # Read and encode the image file
    with open(image_path, "rb") as image_file:
        base64_encoded_data = base64.b64encode(image_file.read()).decode('utf-8')

    # Construct the data URL
    return f"data:{mime_type};base64,{base64_encoded_data}"

def get_scene_description(client, image_path, save_path, model="gpt-4o", verbose=False, task=None):
    """ Gets scene description from an image"""
    data_url = local_image_to_data_url(image_path)

    if task is None:
        prompt = 'Please describe the scene shown in the image'
    else:
        prompt = 'Please describe the scene shown in the image in a way that might help solve the task [' + task + '.] Do not suggest how to solve the task, just describe the scene'

    PROMPT_MESSAGES = [
        {"role": "user", "content": [{"type": "text", "text": prompt},
                                     {"type": "image_url", "image_url": {"url": data_url},}
                                    ]
        }
    ]

    completion = client.chat.completions.create(model=model,
                                                temperature=0.1,
                                                top_p=0.1,
                                                messages=PROMPT_MESSAGES)

    date_filename = get_new_filename()
    with open_file(save_path + "/" , date_filename + "_response.pickle", "wb") as f:
        pickle.dump(completion, f)
    with open_file(save_path + "/" , date_filename +  "_response.txt", "w", "utf-8") as file:
        file.write(completion.choices[0].message.content)
    with open_file(save_path + "/" , date_filename +  "_prompt.txt", "w", "utf-8") as file:
        file.write(prompt)

    if verbose:
        print(completion.model)
        print(completion.choices[0].message.content)

    return completion.choices[0].message.content

def get_gpt_client():
    """ 
    Returns the open ai gpt client
    gets the API Key from environment variable AZURE_OPENAI_API_KEY
    """
    return AzureOpenAI(
        # https://learn.microsoft.com/en-us/azure/ai-services/openai/reference#rest-api-versioning
        api_version="2023-09-01-preview",
        # https://learn.microsoft.com/en-us/azure/cognitive-services/openai/how-to/create-resource?pivots=web-portal#create-a-resource
        azure_endpoint="https://rda6-prj12683-nm.openai.azure.com" #azure_endpoint="https://example-endpoint.openai.azure.com",
    )

def get_completion(client, path, experiment, prompt, saved_completion, verbose):
    """ Gets completion of the prompt from either saved logs or asking gpt """
    if saved_completion is not None:
        with open_file(path, saved_completion + ".pickle", "rb") as f:
            completion = pickle.load(f)
    else:
        completion = run_gpt(client, path, experiment, prompt)

    if verbose:
        print(completion.model)
        price = get_price(completion.usage.prompt_tokens,
                           completion.usage.completion_tokens,
                           completion.model)
        print(f"This prompt cost: {price:f} $")
        print(completion.choices[0].message.content)

    return completion

def run_gpt(client, path, experiment, prompt):
    """ 
    Executes a gpt prompt and saves the results and logs 
    Available models are: # gpt-35-turbo-16k, gpt-4-32k, gpt-3.5-turbo-1106, 
    gpt-4-1106-Preview, gpt-4-1106-vision-preview 
    see: https://platform.openai.com/docs/models
    """
    completion = client.chat.completions.create(
            model="gpt-4-1106-Preview",
            temperature=0.1,
            top_p=0.1,
            messages=[
                {
                    "role": "user", "content": prompt,
                },
            ]
        )
    print("model created\n")
    date_filename = get_new_filename()
    with open_file(path + experiment + "/" , date_filename + "_response.pickle", "wb") as f:
        pickle.dump(completion, f)
    with open_file(path + experiment + "/" , date_filename +  "_response.txt", "w", "utf-8") as file:
        file.write(completion.choices[0].message.content)
    with open_file(path + experiment + "/" , date_filename +  "_prompt.txt", "w", "utf-8") as file:
        file.write(prompt)

    return completion

def parse_response(response):
    """ Parses a gpt response and separates reasoning and conditions """
    conditions_index = None
    try:
        reasoning_index = response.index("Reasoning:") + 11
        conditions_index = response.index("Conditions:", reasoning_index) + 12
        reasoning = response[reasoning_index:conditions_index - 12]
    except ValueError:
        try:
            question_index = response.index("Question:") + 10
            return response[question_index:], None
        except ValueError:
            reasoning_index = response.index("Reasoning:") + 11
            value_index = response.index("Parameter value:", reasoning_index) + 17
            reasoning = response[reasoning_index:value_index - 17]
            #Remove preceding line break if there is one
            if response[value_index:value_index + 1] == "\n":
                value_index += 1
            return reasoning, response[value_index:]

    #Remove preceding line break if there is one
    if response[conditions_index:conditions_index + 1] == "\n":
        conditions_index += 1

    goal_conditions = []
    new_condition = conditions_index
    try:
        line_break = conditions_index + response[conditions_index:].index("\n")
        new_condition = conditions_index
        while line_break > 0:
            goal_conditions.append(response[new_condition:line_break])
            new_condition = line_break + 1
            line_break = line_break + response[line_break + 1:].index("\n") + 1
    except ValueError:
        goal_conditions.append(response[new_condition:])

    #Remove numbering or other prefixes in the list
    for i, _ in enumerate(goal_conditions):
        if re.search("[0-9][.] ", goal_conditions[i][:3]) is not None:
            goal_conditions[i] = goal_conditions[i][3:]
        if re.search("- ", goal_conditions[i][:2]) is not None:
            goal_conditions[i] = goal_conditions[i][2:]

    return reasoning, goal_conditions

def get_goal_condition(client, experiment, scene, instruction, saved_completion=None, verbose=False):
    """ Gets goal condition by asking gpt and plans a tree to solve it """
    prompt = compile_prompt("warallmtests/llm/goal_prompts/", experiment, scene, instruction, verbose)

    completion = get_completion(client, "warallmtests/llm/goal_prompts/logs/", experiment, prompt, saved_completion, verbose)

    return parse_response(completion.choices[0].message.content)

def resolve_error(client, experiment, scene, instruction, saved_completion, verbose, path=None, subfolder=""):
    """ Resolves an error by asking gpt and plans a tree to solve it """
    if path is None:
        path = "warallmtests/llm/resolve_prompts/"

    if scene is None:
        with open_file(path + "scene_descriptions/", experiment + ".txt", 'r', encoding="utf-8") as file:
            scene = file.read()
    prompt = compile_prompt(path+subfolder, experiment, scene, instruction, verbose)

    completion = get_completion(client, path + "logs/", experiment, prompt, saved_completion, verbose)

    return parse_response(completion.choices[0].message.content)
