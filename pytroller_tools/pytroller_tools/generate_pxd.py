#!/usr/bin/env python3

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

import argparse
import sys


def extract_params_struct(content):
    struct_name = "struct Params {"
    struct2_name = "struct StackParams {"
    struct_start = content.find(struct_name)
    struct_end = content.find(struct2_name)
    return_struct = ''
    if struct_start != -1:
        struct_content = content[struct_start:struct_end]
        for line in struct_content.splitlines()[1:-4]:
            match line.strip().split(' = ')[0].split(' ')[0]:
                case 'bool':
                    return_struct += 'bool'
                case 'double':
                    return_struct += 'float'
                case 'int64_t':
                    return_struct += 'int'
                case 'std::string':
                    return_struct += 'string'
                case 'std::vector<bool>':
                    return_struct += 'vector[bool]'
                case 'std::vector<double>':
                    return_struct += 'vector[float]'
                case 'std::vector<int64_t>':
                    return_struct += 'vector[int64_t]'
                case 'std::vector<std::string>':
                    return_struct += 'vector[string]'

            return_struct += ' ' + line.strip().split(' = ')[0].split(' ')[1] + '\n'
        return return_struct

    return None


def read_cpp_file(file_path):
    with open(file_path) as file:
        return file.read()


def run(output_pxd_file, input_header_file):
    cpp_content = read_cpp_file(input_header_file)

    params_struct_content = extract_params_struct(cpp_content)
    namespace_start = cpp_content.find("namespace")
    namespace_end = cpp_content.find("{", namespace_start)
    namespace = cpp_content[namespace_start+10:namespace_end-1]

    if params_struct_content:
        pxd_file = '# distutils: language = c++\n\n'
        pxd_file += 'from libcpp.vector cimport vector\n'
        pxd_file += 'from libcpp.string cimport string\n\n'
        pxd_file += 'cdef extern from "' + input_header_file + '" namespace "' + namespace + '":\n'
        pxd_file += '\tcdef struct Params:\n'
        for line in params_struct_content.splitlines():
            pxd_file += '\t\t' + line + '\n'

        with open(output_pxd_file, "w") as text_file:
            text_file.write(pxd_file)

    else:
        print('Params struct not found in the file.')


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("output_pxd_file")
    parser.add_argument("input_cpp_header_file")
    return parser.parse_args()


def main():
    args = parse_args()
    output_file = args.output_pxd_file
    input_file = args.input_cpp_header_file

    run(output_file, input_file)


if __name__ == "__main__":
    sys.exit(main())
