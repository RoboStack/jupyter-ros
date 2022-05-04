#############################################################################
# Copyright (c) Wolf Vollprecht, QuantStack                                 #
#                                                                           #
# Distributed under the terms of the BSD 3-Clause License.                  #
#                                                                           #
# The full license is in the file LICENSE, distributed with this software.  #
#############################################################################

from ._version import version_info, __version__

from .ros_widgets import *
from .pubsub import *
from .ipy import *
from .server_extension import *

from .ros3d import *

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