# Building Jupyter-ROS Documentation

1. Create a new environment with the following dependencies

   - `sphinx`
   - `myst-parser`
   - `jinja2=3.0`
   - `sphinx-rtd-theme`

   ```sh
   $ mamba create jupyros_docs sphinx myst-parser jinja2=3.0 -c conda-forge
   $ mamba activate jupyros_docs
   $ pip install sphinx-rtd-theme
   ```

1. [Optional] Install `jupyter-ros` in the environment. This is only necessary for the _References_ page to display correctly; otherwise, there will be a few warnings in the next step.

1. Build the documents

   ```sh
   $ cd jupyter-ros/docs/
   $ make html
   ```

1. Open the documentation locally

   ```sh
   $ cd build/html/
   $ python -m http.server
   ```

1. From a web browser, navigate to `localhost:8000`
