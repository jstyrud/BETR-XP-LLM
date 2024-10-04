# coding: utf-8

"""
    RWS-PY: a package to communicate with OmniCore and IRC5 controllers using RWS2

    RWS2 OpenAPI spec version: 3HAC073675-001 Revision:D
"""


import re

from setuptools import find_packages, setup  # noqa: H301

NAME = "rws_py"
VERSION = "1.0.0"


def read_requirements(file):
    with open(file, encoding="UTF-8") as f:
        return f.read().splitlines()


requirements = read_requirements("requirements.txt")


# read version
VERSION_FILE = "_version.py"
ver_str_line = open(VERSION_FILE, "rt", encoding="UTF-8").read()
VSRE = r"^__version__ = ['\"]([^'\"]*)['\"]"
mo = re.search(VSRE, ver_str_line, re.M)
if mo:
    verstr = mo.group(1)
else:
    raise RuntimeError("Unable to find version string in %s." % (VERSION_FILE,))


setup(
    name=NAME,
    version=verstr,
    description="Python interface to ABB OmniCore and IRC5 controllers via RWS",
    author_email="nima.enayati@de.abb.com",
    url="",
    keywords=["ABB", "Robot Web Services"],
    install_requires=requirements,
    packages=find_packages(),
    include_package_data=True,
)
