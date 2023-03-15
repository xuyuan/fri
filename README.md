# fri
Fuzzy ROS2 Introspection

## Demo Video
[![Demo Video](https://img.youtube.com/vi/S3CM47CnZ-0/mqdefault.jpg)](https://www.youtube.com/watch?v=S3CM47CnZ-0)

## Shortcuts

| Key          | Action                                |
| ------------ | ------------------------------------- |
| **Ctrl - a** | `ros2 node list` or `ros2 topic list` |
| **Ctrl - r** | reload                                |
| **Ctrl - e** | `ros2 topic echo --once <topic>` or `ros2 service call <service>`      |
| **Ctrl - h** | `ros2 topic hz <topic>`               |
| **Ctrl - i** | `ros2 topic info -v <topic>`          |
| **Ctrl - b** | `ros2 topic bw <topic>`               |
| **Ctrl - l** | `ros2 topic delay <topic>`            |
| **Ctrl - t** | `ros2 interface show <topic_type>`    |
| **Ctrl - y** | copy selection to clipboard*          |

## Installation
* [ROS2](https://docs.ros.org/) (tested with Humble)
* [fzf - command-line fuzzy finder](https://github.com/junegunn/fzf)
* [xclip](https://github.com/astrand/xclip) (*optional: copy selection to clipboard)