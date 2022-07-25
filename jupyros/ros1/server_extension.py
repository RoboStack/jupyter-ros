#############################################################################
# Copyright (c) Wolf Vollprecht, QuantStack                                 #
#                                                                           #
# Distributed under the terms of the BSD 3-Clause License.                  #
#                                                                           #
# The full license is in the file LICENSE, distributed with this software.  #
#############################################################################
import os

from notebook.utils import url_path_join
from notebook.base.handlers import IPythonHandler

import rospkg

from .._version import __version__

__version__ = __version__

if os.getenv('JUPYROS_DEFAULT_WS'):
    envs = os.getenv('JUPYROS_DEFAULT_WS').split(';')
else:
    envs = None
r = rospkg.RosPack(envs)

class ROSStaticHandler(IPythonHandler):
    def get(self, *args, **kwargs):
        if not args:
            self.write("Error - no argument supplied")
        argslist = args[0].split('/')
        package, rest = argslist[0], '/'.join(argslist[1:])

        file = os.path.join(r.get_path(package), rest)
        try:
            with open(file, 'rb') as f:
                data = f.read()
                self.write(data)
        except:
            self.write("Error opening file %s" % file)
        self.finish()

def load_jupyter_server_extension(nb_server_app):
    """
    Called when the extension is loaded.

    Args:
        nb_server_app (NotebookWebApplication): handle to the Notebook webserver instance.
    """
    web_app = nb_server_app.web_app
    host_pattern = '.*$'
    route_pattern = url_path_join(web_app.settings['base_url'], '/rospkg/(.*)')
    web_app.add_handlers(host_pattern, [(route_pattern, ROSStaticHandler)])
