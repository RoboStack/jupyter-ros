import sys
from threading import Thread
from IPython.core.magic import register_cell_magic
from jupyros.pubsub import output_registry
import ipywidgets as widgets

def executor(cell, gbls, lcls):
    exec(cell, gbls, lcls)

# @register_cell_magic is not available during jupyter nbextension enable ...
try:
    @register_cell_magic
    def thread_cell(line, cell, local_ns=None):
        t = Thread(target=executor, args=(cell, globals(), sys._getframe(2).f_locals))
        out = widgets.Output(layout={'border': '1px solid gray'})
        output_registry[t.name] = out
        t.start()
        return out
except:
    pass
