# Copyright 2017 Open Source Robotics Foundation, Inc.
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

from io import StringIO
import os
import sys

import em
import importlib.resources as importlib_resources


def _expand_template(template_file, data, output_file):
  output = StringIO()
  interpreter = em.Interpreter(
    output=output,
    options={
      em.BUFFERED_OPT: True,
      em.RAW_OPT: True,
    },
    globals=data,
  )
  with open(template_file, 'r') as h:
    try:
      interpreter.file(h)
      content = output.getvalue()
    except Exception as e:
      if os.path.exists(output_file):
        os.remove(output_file)
      print("Exception when expanding '%s' into '%s': %s" %
          (template_file, output_file, e), file=sys.stderr)
      raise
    finally:
      interpreter.shutdown()

  if os.path.exists(output_file):
    with open(output_file, 'r') as h:
      if h.read() == content:
        return
  else:
    os.makedirs(os.path.dirname(output_file), exist_ok=True)

  with open(output_file, 'w') as h:
    h.write(content)


def _create_folder(folder_name, base_directory, exist_ok=True):
  folder_path = os.path.join(base_directory, folder_name)

  print('creating folder', folder_path)
  os.makedirs(folder_path, exist_ok=exist_ok)

  return folder_path


def _create_template_file(
  template_subdir, template_file_name, output_directory, output_file_name, template_config
):
  full_package = 'pytrollercli.resource.' + template_subdir
  with importlib_resources.path(full_package, template_file_name) as path:
    template_path = str(path)
  if not os.path.exists(template_path):
    raise FileNotFoundError('template not found:', template_path)

  output_file_path = os.path.join(output_directory, output_file_name)

  print('creating', output_file_path)
  _expand_template(template_path, template_config, output_file_path)


def create_package_environment(pytroller_name, destination_directory):
  package_directory = _create_folder(pytroller_name, destination_directory)

  package_xml_config = {
    'pytroller_name': pytroller_name,
    'maintainer_name': 'maintainer',
    'maintainer_email': 'maintainer@email.com'
  }
  _create_template_file(
    'package',
    'package.xml.em',
    package_directory,
    'package.xml',
    package_xml_config)

  source_directory = None
  include_directory = None
  print('creating source and include folder')
  source_directory = _create_folder('src', package_directory)
  source_directory = _create_folder('script', package_directory)
  source_directory = _create_folder('test', package_directory)
  include_directory = _create_folder(pytroller_name, package_directory + os.sep + 'include')

  return package_directory, source_directory, include_directory


def populate_ament_cmake(package_name, package_directory):
  class_name = package_name.replace('_', ' ').title()
  class_name = ''.join(x for x in class_name if not x.isspace())
  cmakelists_config = {
    'pytroller_name': package_name,
  }
  _create_template_file(
    'package',
    'CMakeLists.txt.em',
    package_directory,
    'CMakeLists.txt',
    cmakelists_config)
  
  plugin_config = {
    'pytroller_name': package_name,
    'pytroller_class': class_name,
  }
  _create_template_file(
    'package',
    'controller_plugin.xml.em',
    package_directory,
    'controller_plugin.xml',
    plugin_config)

  
def populate_logic(package_name, source_directory):
  
  cmakelists_config = {
    'pytroller_name': package_name,
  }
  _create_template_file(
    'src',
    'pytroller_logic.pyx.em',
    source_directory,
    package_name + '_logic.pyx',
    cmakelists_config)
  
def populate_logic_impl(package_name, script_directory):
  # os.popen('cp ' + logic_script + ' ' + source_directory + os.sep + package_name +'_logic_impl.py') 
  impl_config = {
    'pytroller_name': package_name,
  }
  _create_template_file(
    'script',
    'pytroller_logic_impl.py.em',
    script_directory,
    package_name + '_logic_impl.py',
    impl_config)

def populate_cpp_library(package_name, source_directory, include_directory):
  class_name = package_name.replace('_', ' ').title()
  class_name = ''.join(x for x in class_name if not x.isspace())
  cpp_header_config = {
    'pytroller_name': package_name,
    'pytroller_class': class_name,
  }
  _create_template_file(
    'include',
    'pytroller.hpp.em',
    include_directory,
    package_name + '.hpp',
    cpp_header_config)

  cpp_library_config = {
    'pytroller_name': package_name,
    'pytroller_class': class_name
  }
  _create_template_file(
    'src',
    'pytroller.cpp.em',
    source_directory,
    package_name + '.cpp',
    cpp_library_config)

  visibility_config = {
    'pytroller_name': package_name,
  }
  _create_template_file(
    'include',
    'visibility_control.h.em',
    include_directory,
    'visibility_control.h',
    visibility_config)
  
  parameters_config = {
    'pytroller_name': package_name,
  }
  _create_template_file(
    'src',
    'pytroller_parameters.yaml.em',
    source_directory,
    package_name + '_parameters.yaml',
    parameters_config)
  
def populate_test(package_name, source_directory):
  class_name = package_name.replace('_', ' ').title()
  class_name = ''.join(x for x in class_name if not x.isspace())
  load_test_config = {
    'pytroller_name': package_name,
    'pytroller_class': class_name
  }
  _create_template_file(
    'test',
    'test_load_pytroller.cpp.em',
    source_directory,
    'test_load_' + package_name + '.cpp',
    load_test_config)
  
  test_param_config = {
    'pytroller_name': package_name,
  }
  _create_template_file(
    'test',
    'test_params.yaml.em',
    source_directory,
    'test_params.yaml',
    test_param_config)
  
  test_header_config = {
    'pytroller_name': package_name,
    'pytroller_class': class_name
  }
  _create_template_file(
    'test',
    'test_pytroller.hpp.em',
    source_directory,
    'test_' + package_name + '.hpp',
    test_header_config)
  
  test_lib_config = {
    'pytroller_name': package_name,
    'pytroller_class': class_name
  }
  _create_template_file(
    'test',
    'test_pytroller.cpp.em',
    source_directory,
    'test_' + package_name + '.cpp',
    test_lib_config)

  
def create_pytroller(pytroller_name, destination_directory):
  create_package_environment(pytroller_name, destination_directory)
  populate_ament_cmake(pytroller_name, destination_directory+'/'+pytroller_name)
  populate_cpp_library(pytroller_name, 
                       destination_directory+'/'+pytroller_name+"/src",
                       destination_directory+'/'+pytroller_name+"/include/"+pytroller_name)
  populate_logic(pytroller_name,
                 destination_directory+'/'+pytroller_name+"/src")
  populate_logic_impl(pytroller_name,
                      destination_directory+'/'+pytroller_name+"/script")
  populate_test(pytroller_name,
                destination_directory+'/'+pytroller_name+"/test")