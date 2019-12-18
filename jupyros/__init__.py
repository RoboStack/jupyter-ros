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
        'src': 'static',
        'dest': 'jupyter-ros',
        'require': 'jupyter-ros/extension'
    }]
