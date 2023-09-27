# pytroller
Python controller generator for `ros2_control`

## Usage

Create a python script containing the definition of the `pytroller_logic_impl` function that takes as input 
- `state` : type `dict` {`joint/interface`, value} : current joint states on all available interfaces
- `commands` : type `dict` {`joint/interface`, value} : commands to be assigned to joint command interfaces
- `msg` : type `list` : bytearray of the serialized command message    

For example :

```python
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray
from math import cos, sin

def pytroller_logic_impl(states, commands, msg):
  
  command_message = deserialize_message(bytes(msg), type(Float64MultiArray()))

  commands['joint1/effort'.encode('ascii')] = command_message.data[0]
  commands['joint2/effort'.encode('ascii')] = command_message.data[1]

  return commands
```

```shell
$ ros2 pytroller create test_logic.py --pytroller-name my_pytroller --destination-directory controllers
```
