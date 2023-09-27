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

import os

from ros2cli.node.direct import add_arguments
from ros2cli.verb import VerbExtension

from pytrollercli.api.create import create_pytroller


class CreateVerb(VerbExtension):
  """Create a new pytroller for ros2_control."""

  def add_arguments(self, parser, cli_name):
    parser.add_argument(
      'pytroller_logic',
      help='Pytroller logic .py file')
    parser.add_argument(
      '--pytroller-name',
      default='pytroller',
      help='The pytroller name')
    parser.add_argument(
      '--destination-directory',
      default=os.curdir,
      help='Directory where to create the package directory')
    
  def main(self, *, args):
    create_pytroller(
      args.pytroller_name,
      args.destination_directory,
      args.pytroller_logic
    )
