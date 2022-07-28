.. _dev-install-label:

Developer Installation
======================


Install Jupyter-ROS
-------------------

1. Clone repository

   .. code-block:: sh

     $ git clone git@github.com:RoboStack/jupyter-ros.git
     $ cd jupyter-ros

2. Create a conda environment for development with the following packages

   * ``python = 3.9``
   * ``jupyterlab``
   * ``jupyter-packaging``
   * ``nodejs <= 15``
   * ``ros-noetic-desktop``

   .. code-block:: sh

      # You can use conda as well
      $ mamba create -n jupyros_env python=3.9 jupyterlab jupyter-packaging nodejs=15 ros-noetic-desktop -c conda-forge -c robostack

      $ mamba activate jupyros_end

3. Install *jupyter-ros* in editable mode
   
   .. code-block:: sh

      # From the jupyter-ros root directory
      $ pip install -e .

4. Symlink the JupyterLab extension

   .. code-block:: sh

      $ jupyter labextension develop . --overwrite

5. Verify installation with Python

   .. code-block:: python

      import jupyros
      print(jupyros.__file__)
      # Should return /home/user/jupyter-ros/jupyros/__init__.py



Build Documentation
-------------------

1. Create a new conda environment with the following dependencies:

   * ``sphinx``
   * ``myst-parser``
   * ``jinja2 <= 3.0``
   * ``sphinx-rtd-theme``

   .. code-block:: sh

    # You can use conda as well
    $ mamba create -n jupyros_docs sphinx myst-parser jinja2=3.0 -c conda-forge
    $ mamba activate jupyros_docs
    $ pip install sphinx-rtd-theme

2. **[Optional]** Install ``jupyter-ros`` in the environment. This is only necessary for the *References* page to display correctly; otherwise, there will be a few warnings in the next step.

3. Build the documents

   .. code-block:: sh

    $ cd jupyter-ros/docs/
    $ make html


4. Open the documentation locally

   .. code-block:: sh

    $ cd build/html/
    $ python -m http.server

5. From a web browser, navigate to ``localhost:8000``
