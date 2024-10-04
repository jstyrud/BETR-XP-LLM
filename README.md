# BETR-XP-LLM
This is the project repository for the paper 

**Automatic Behavior Tree Expansion with LLMs for Robotic Manipulation**

submitted to IEEE International Conference on Robotics and Automation (ICRA), 2025

A preview version can be found at:
https://arxiv.org/abs/2409.13356

## Installation

Create a project folder, clone this repo and install requirements
```
mkdir betr-xp-llm && cd betr-xp-llm
git clone git@github.com/jstyrud/BETR-XP-LLM.git
```

### Install Yolo-World, nanosam, py_trees
https://github.com/AILab-CVC/YOLO-World <br/>
https://github.com/NVIDIA-AI-IOT/nanosam <br/>
https://github.com/jstyrud/py_trees

### Install any missing requirements
pip install -r requirements.txt

### Install rws_py
1. In a terminal go to the directory rws_py
2. pip install -e . 

### Install cfree_py
This is to come, official ABB release is expected late 2024.
