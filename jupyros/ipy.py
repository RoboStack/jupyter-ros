from IPython import get_ipython
from IPython.core.magic import register_line_magic, register_cell_magic, register_line_cell_magic
from threading import Thread
import time

def executor(cell, gbls, lcls):
	exec(cell, gbls, lcls)

@register_cell_magic
def thread_cell(line, cell):
	t = Thread(target=executor, args=(cell, globals(), locals()))
	t.start()
