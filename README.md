# ROS Support for jupyter notebooks

While the Jupyter ecosystem has been widely adopted by
the Data Science and Machine Learning community, the
robotics community has not jumped on the band wagon yet!
Most tools around ROS, the Robot Operating System, are
built using Python and QT.

However, using QT seperates the user away from the code.
We've built an initial version of the ROS tools for jupyter
notebook, trying to promote a rich, interactive experience
for Robotics developers utilizing the power of the jupyter
notebook.

[![Video of jupyter-ros in JupyterLab](https://raw.githubusercontent.com/wolfv/jupyter-ros/master/docs/assets/jupyterlab-with-ros.gif)](https://www.youtube.com/watch?v=mPvYZango2E)

![](https://raw.githubusercontent.com/wolfv/jupyter-ros/master/docs/assets/screenshot.png)

With jupyter-ros, it's possible to easily create widgets for
custom message types to send messages.

In the future, we plan to bring simple and fast real-time
plotting from ROS topics to this library.

If you find this initial package useful, don't hesitate to
contribute!
You can also always reach out to w.vollprecht@gmail.com or
on twitter: https://twitter.com/wuoulf

## Installation and Dependencies

You need a ROS environment with rospy, obviously.
Also required are `numpy` and `bqplot` for the live-plotting
and `ipywidgets` for the interactive widgets

    python3 -m pip install jupyter bqplot pyyaml ipywidgets
    jupyter nbextension enable --py --sys-prefix ipywidgets

`pyyaml` is necessary for rospy.

To install use pip:

    python3 -m pip install jupyros
    jupyter nbextension enable --py --sys-prefix jupyros

To install the extension for **jupyterlab**, you also need to execute the following:

    $ jupyter labextension install jupyter-ros

##### Development installation

For a development installation (requires npm),

    git clone https://github.com/wolfv/jupyter-ros.git
    cd jupyter-ros
    python3 -m pip install -e .
    jupyter nbextension install --py --symlink --sys-prefix jupyros
    jupyter nbextension enable --py --sys-prefix jupyros


## Troubleshooting

You might see a warning like "The rospy package is not found in your $PYTHONPATH.
Subscribe and publish are not going to work. Do you need to activate your ROS environment?"

This is harmless during installation, but if you see this warning in a notebook, you should
check that your ROS environment is activated. You can also set the path from inside the notebook
using for Ubuntu 18.04:

```python
import sys
sys.path.append('/opt/ros/dashing/lib/python3.6/site-packages/')

# The next line should now work!
import jupyros
```

If you are using macOS, set the path to the directory where your source or binary ros2 installation is located as the following example:

```python
import sys
sys.path.append('/Users/some_user/path_to_your_ros2_install/lib/python3.7/site-packages/')

# The next line should now work!
import jupyros
```

If you got the following error when you run a cell ```failed to display Jupyter Widget of type VBox```,
you can solve it by running the following command ```jupyter nbextension enable --py --sys-prefix widgetsnbextension```
