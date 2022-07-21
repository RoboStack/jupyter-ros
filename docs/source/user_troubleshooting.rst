Troubleshooting
===============

When working with the official ROS packages, it may be necessary to modify
the Jupyter kernel.

Once you have Jupyter and the `jupyros` package installed, you can run the
kernel generator to install a special ROS Python kernel for Jupyter. This
modifies a pre-existing kernel in such a way that it knows about the catkin
workspace.

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

Run the installed generator:

.. code-block:: sh

  $ ros_kernel_generator python2 /home/$USER/catkin_ws/devel/setup.bash

This will install a new kernel specification next to the installed python2
kernel (in this case at ``~/miniconda2/share/jupyter/kernels/ros_python2``)
with ROS specific environment variables set to the ones from the catkin
workspace (such as an additional Python path to find the ROS Python libraries).

Websocket Failure
-----------------

For troubleshooting the ROS 3D widgets, we have prepared the following tips.

The 3D widgets of ROS communicate with the backend through websockets. To make
sure that a websocket connection is established, first, open the "Inspector" (in
FireFox or Chrome, right click -> Inspect Element) and navigate to the "Network"
panel of the Inspector. The network panel shows all requests. If the notebook
cells are now executed we should see a "Websocket" connection being established.

If the websocket connection remains at "Pending..." then you might be using the
wrong Tornado version for the rosbridge websocket server. Using the wrong
Tornado version results in a **silent failure**. In this case, it is very
important to **not** mix the Jupyter Python environment with the rosbridge
environment as Jupyter uses a more recent Tornado version than the default
rosbridge websocket installed from the official ROS packages.

Therefore, the fix is usually to reinstall the tornado version from the APT
packages:

.. code::
  
  sudo apt-get install python-tornado
  sudo apt-get install ros-melodic-rosbridge-suite --reinstall

You can also check the network panel to make sure that mesh files are loaded
correctly.
