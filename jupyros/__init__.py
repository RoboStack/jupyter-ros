from ._version import version_info, __version__

from .ros_widgets import *
from .pubsub import *
from .ipy import *

from .ros3d import *

def _jupyter_nbextension_paths():
    return [{
        'section': 'notebook',
        'src': 'static',
        'dest': 'jupyter-ros',
        'require': 'jupyter-ros/extension'
    }]
