Getting Started
===============

The easiest way to get started is to create a ROS environment with conda or mamba with the following packages:

* ``jupyter-ros``
* ``jupyterlab``
* ``ros-noetic-desktop``

.. code-block:: sh
   
   $ conda create -n ros_env jupyter-ros jupyterlab ros-noetic-desktop -c conda-forge -c robostack
   $ conda activate ros_env

   # Launch JupyterLab
   $ jupyter lab

At this point, you should be able to use jupyter-ros from any Jupyter notebook in the same conda environment.

.. code-block:: python

   import jupyros

For alternative methods of installing Jupyter-ROS, please visit :ref:`usr-install-label` and :ref:`dev-install-label`.