# pytroller
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/ICube-Robotics/pytroller/actions/workflows/ci.yml/badge.svg)](https://github.com/ICube-Robotics/pytroller/actions/workflows/ci.yml)

Python controller generator for `ros2_control`.

## Usage

Generate the controller package using :
```shell
$ ros2 pytroller create my_pytroller --destination-directory controllers
```

The raw Python controller logic script `my_pytroller_logic_impl.py` to be implemented is located the `script` directory inside the newly created controller package.

This Python script containing the definition of the `pytroller_logic_impl` function that takes as input
- `state` : type `dict` {`joint/interface`, value} : current joint states on all available interfaces
- `commands` : type `dict` {`joint/interface`, value} : commands to be assigned to joint command interfaces
- `msg` : command message from subscriber
- `params` :  type `dict` {`parameter`, value} : node parameters

For example :

```python
# my_pytroller/script/my_pytroller_logic_impl.py

from math import cos, sin

def pytroller_logic_impl(period, states, commands, msg, params):

  commands['joint1/effort'.encode('ascii')] = msg.data[0]
  commands['joint2/effort'.encode('ascii')] = msg.data[1]

  return commands
```

Install all dependencies and build the controller using
```shell
$ rosdep install --ignore-src --from-paths . -y -r
$ colcon build
```

## Parameters
Pytrollers use parameters generated by the [generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library) package and are defined in the `my_pytroller_parameters.yaml` file :

```yaml
my_pytroller:
  interface_full_names: {
    type: string_array,
    default_value: [],
    description: "Full names of the interface(s) to command",
  }
  command_topic_name: {
    type: string,
    default_value: ""  # "~/commands",
    description: "Name of the subscribed command topic"
  }
  command_topic_type: {
    type: string,
    default_value: ""  # "std_msgs/msg/Float64MultiArray",
    description: "Type of the subscribed command topic"
  }
```
Additional parameters can be added to this configuration file. To do so, refer to the [generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library) package documentation.

The values of the defined parameters are set in the controller configuration file used for ros2-control.

## Contacts ##
![icube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)

[ICube Laboratory](https://icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Maciej Bednarczyk:__ [mcbed.robotics@gmail.com](mailto:mcbed.robotics@gmail.com), @github: [mcbed](https://github.com/mcbed)
