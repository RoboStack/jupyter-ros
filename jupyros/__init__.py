#############################################################################
# Copyright (c) Wolf Vollprecht, QuantStack                                 #
#                                                                           #
# Distributed under the terms of the BSD 3-Clause License.                  #
#                                                                           #
# The full license is in the file LICENSE, distributed with this software.  #
#############################################################################

from ._version import __version__
import os

try:
    ros_version = os.environ['ROS_VERSION']
    ros_distro = os.environ['ROS_DISTRO']
except KeyError:
    # print('No ROS environment detected.')
    # print('Defaulting to ROS noetic.')
    ros_version = '1'
    ros_distro = 'noetic'

if ros_version == '2':
    # Import ROS2 modules
    # print(f'ROS2 {ros_distro} environment detected.')
    from .ros2.publisher import *
    from .ros2.ros_widgets import *
    from .ros2.subscriber import *

else:
    # Default to ROS1
    # print(f'ROS {ros_distro} environment detected.')
    from .ros1.ipy import *
    from .ros1.pubsub import *
    from .ros1.ros_widgets import *
    from .ros1.ros3d import *
    from .ros1.server_extension import *
    from .ros1.turtle_sim import *


def _jupyter_nbextension_paths():
    return [{
        'section': 'notebook',
        'src': 'nbextension',
        'dest': '@robostack/jupyter-ros',
        'require': '@robostack/jupyter-ros/extension'
    }]

def _jupyter_labextension_paths():
    return [{
        'src': 'labextension',
        'dest': '@robostack/jupyter-ros',
    }]

def _jupyter_server_extension_paths():
    return [{
        "module": "jupyros"
    }]
