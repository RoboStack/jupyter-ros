Installation
============

jupyter-ros is distributed as a pip-package. We recommend installing it
inside a conda-environment.

To install conda, we recommend using the `Miniconda installer <https://docs.conda.io/en/latest/miniconda.html>`_.

Once installed, you should use conda to install jupyter:

.. code-block:: sh

  conda create -n jupyros_env python=2.7 bqplot pip -c conda-forge
  conda activate jupyros_env
  pip install jupyros

The less recommended alternative is to install jupyter-ros in a virtual environment
or globally (least recommended for compatibility reasons with rosbridge).

.. warning::
  
  Never start the rosbridge server in the same environment as Jupyter. They rely
  on incompatible versions of Tornado. The rosbridge server runs best with the
  default Tornado version from the Ubuntu repositories!

Once you have jupyter, and the jupyros-package installed, you can run the kernel
generator to install a special ROS Python kernel for Jupyter. This modifies a pre-
existing kernel in such a way that it knows about the catkin workspace.

You can find the available jupyter kernels by running:

.. code-block:: sh

  $ jupyter kernelspec list

  Available kernels:
  julia        ~/.local/share/jupyter/kernels/julia
  python3      ~/miniconda3/share/jupyter/kernels/python3
  python2      ~/miniconda2/share/jupyter/kernels/python2
  xcpp11       ~/miniconda3/share/jupyter/kernels/xcpp11
  xcpp14       ~/miniconda3/share/jupyter/kernels/xcpp14
  xcpp17       ~/miniconda3/share/jupyter/kernels/xcpp17
  xonsh        ~/miniconda3/share/jupyter/kernels/xonsh

For maximum compatibility with ROS 1 releases, we want to base the ROS kernel on
the existing Python 2 kernel.

To run the installed generator, run:

.. code-block:: sh

  $ ros_kernel_generator python2 /home/$USER/catkin_ws/devel/setup.bash

This will install a new kernel specification next to the installed python2 kernel 
(in this case at ``~/miniconda2/share/jupyter/kernels/ros_python2``) with ROS specific
environment variables set to the ones from the catkin workspace (such as an additional
Python path to find the ROS Python libraries).

The jupyter-ros server extension
--------------------------------

The jupyter-ros package contains an optional server extension which can be used
to serve static files from a catkin workspace to the web browser. For the 3D 
widgets we might have to load meshes to display robots correctly. These meshes
are usually part of a robot description package in ROS. In order to enable the 
web browser to access those meshes, we can enable the jupyter-ros server
extension:

.. code-block:: sh
  
  $ jupyter serverextension enable jupyros

  Enabling: jupyros
  - Writing config: /home/wolfv/.jupyter
      - Validating...
        jupyros 0.3.0 OK

The jupyter-ros server extension adds a handler to the Jupyter server that will
return contents from ROS packages. To check that it works, you can (re-)start the
Jupyter server and navigate to ``localhost:8888/rospkg/rospy/package.xml`` or any
other file that should exist in your catkin workspace.

.. warning::
  
  There are currently *no* security features in the jupyter-ros server extension.
  If you have sensitive ROS packages, all contents (including uncompiled source code)
  can be found through this extension.