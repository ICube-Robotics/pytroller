# pytroller
Python controller generator for `ros2_control`

## Usage

Generate the controller package using :
```shell
$ ros2 pytroller create my_pytroller --destination-directory controllers
```

The raw Python controller logic script `my_pytroller_logic_impl.py` to be implemented is located the `script` directory inside the newly created controller package an is called.

This Python script containing the definition of the `pytroller_logic_impl` function that takes as input
- `state` : type `dict` {`joint/interface`, value} : current joint states on all available interfaces
- `commands` : type `dict` {`joint/interface`, value} : commands to be assigned to joint command interfaces
- `msg` : command message from subscriber
- `params` :  type `dict` {`parameter`, value} : node parameters

For example :

```python
# example_logic.py

from math import cos, sin

def pytroller_logic_impl(states, commands, msg, params):

  commands['joint1/effort'.encode('ascii')] = msg.data[0]
  commands['joint2/effort'.encode('ascii')] = msg.data[1]

  return commands
```
