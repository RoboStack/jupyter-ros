.. _usr-install-label:

Installation
============

Jupyter-ROS is distributed as a conda, pip, and npm package. We recommend 
installing it inside a conda environment. To install conda, we suggest using
the `Miniconda installer <https://docs.conda.io/en/latest/miniconda.html>`_.

.. code-block:: sh

   # Option 1. conda or mamba [Recommended]
   $ conda create -n jupyros_env python=3.9
   $ conda activate jupyros_env
   $ conda install jupyter-ros -c robostack

.. code-block:: sh

   # Option 2. pip
   $ pip install jupyros

.. code-block:: sh

   # Option 3. npm
   $ npm i jupyter-ros


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
  - Writing config: /home/user/.jupyter
      - Validating...
        jupyros 0.6.0 OK

The jupyter-ros server extension adds a handler to the Jupyter server that will
return contents from ROS packages. To check that it works, you can (re-)start
the Jupyter server and navigate to ``localhost:8888/rospkg/rospy/package.xml``
or any other file that should exist in your catkin workspace.

.. warning::
  
  There are currently **no** security features in the jupyter-ros server
  extension. If you have sensitive ROS packages, all contents (including
  uncompiled source code) can be found through this extension.