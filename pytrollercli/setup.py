# Copyright 2023 ICube-Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from setuptools import find_packages
from setuptools import setup

package_name = "pytrollercli"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ],
    install_requires=["ros2cli"],
    zip_safe=True,
    author="Maciej Bednarczyk",
    author_email="mcbed.robotics@gmail.com",
    maintainer="Maciej Bednarczyk",
    maintainer_email="mcbed.robotics@gmail.com",
    url="https://github.com/mcbed/pytroller",
    keywords=[],
    classifiers=[
        "Environment :: Console",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
    ],
    description="Pytroller command interface.",
    long_description="""\
Pytroller command interface.""",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "ros2cli.command": [
            "pytroller = pytrollercli.command.pytroller:PytrollerCommand",
        ],
        "pytrollercli.verb": [
            "create = pytrollercli.verb.create:CreateVerb",

        ],
    },
    package_data={
        'pytrollercli': [
            'resource/**/*',
        ],
    },
)
