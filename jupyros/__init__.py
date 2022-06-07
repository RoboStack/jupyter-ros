from ._version import version_info, __version__

from .ros_widgets import *
from .pubsub import *
from .ipy import *
from .server_extension import *
from .ros3d import *

from .ros2 import *

"""
from .ros2.ros_widgets import *
from .ros2.pubsub import *
from .ros2.ipy import *
from .ros2.server_extension import *
from .ros2.ros3d import *
from .ros2.Subscription import *
from .ros2.Publisher import *
from .ros2.plot import *
from .ros2.keyboard_input import *
"""
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


