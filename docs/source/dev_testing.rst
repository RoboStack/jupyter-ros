Testing
=======

The simplest way to test any additions to Jupyter-ROS is to create a fresh
environment and install the package in development mode.

1. Create new test environment with minimal packages

   .. code-block:: sh

      $ conda create -n test_env python=3.9 jupyterlab nodejs=15 jupyter-packaging ros-noetic-desktop -c conda-forge -c robostack

      $ conda activate test_env

2. Navigate to the root directory and install jupyter-ros

   .. code-block:: sh

      $ cd jupyter-ros/
      $ pip install -e .

   If there are any errors with this step, this indicates that the new
   additions are not configured correctly for installation. This will require
   some additional troubleshooting, but a common issue is to forget to include
   newly required dependencies in the `setup.py` file.

   .. code-block:: python

      setup_args = {
        'install_requires': [
            'ipywidgets>=7.0.0',
            'bqplot',
            'numpy',
            'rospkg',
            'ipycanvas'
        ],
      }

3. Symlink the JupyterLab extension and verify that `jupyros` can be imported.

   .. code-block:: sh
      
      $ jupyter labextension develop . --overwrite
      $ jupyter lab

   .. code-block:: python

      import jupyros

   This step may also require some troubleshooting depending on the changes
   made to Jupyter-ROS. If you see interference from other conda environments,
   e.g. additional lab extensions which should not be enabled in the test
   environment, it is often helpful to remove the ``~/.jupyter`` directory.

   .. code-block:: sh

      # List all the lab extensions
      $ jupyter labextension list

      # Remove jupyter directory [optional]
      $ rm -r ~/.jupyter


4. Once the setup is complete, it is now time to test the new additions. This
   type of testing will vary greatly depending on the changes, we suggest to
   use your best judgement. 

