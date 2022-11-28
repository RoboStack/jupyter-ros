#############################################################################
# Copyright (c) Wolf Vollprecht, QuantStack                                 #
#                                                                           #
# Distributed under the terms of the BSD 3-Clause License.                  #
#                                                                           #
# The full license is in the file LICENSE, distributed with this software.  #
#############################################################################

## Modified by Luigi Dania for Jupyter-Ros2
import sys
from threading import Thread

import ipywidgets as widgets
from IPython.core.magic import register_cell_magic

def executor(cell, gbls, lcls):
    exec(cell, gbls, lcls)

# @register_cell_magic is not available during jupyter nbextension enable ...
try:
    @register_cell_magic
    def thread_cell2(line, cell, local_ns=None):
        t = Thread(target=executor, args=(cell, globals(), sys._getframe(2).f_locals))
        out = widgets.Output(layout={'border': '1px solid gray'})
        t.start()
        return out
except:
    pass