# Jupyros2
Navigate to:
'''
|-- Jupyros
    |-- Ros2
        |-- README.md
'''
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
on twitter: https://twitter.com/wuoulf, or join us on [Gitter](https://gitter.im/RoboStack/Lobby)

## Installation and Dependencies

You need a ROS environment with rospy. We recommend using [Robostack](https://github.com/RoboStack/ros-noetic) (follow their installation instructions) which allows you to install ROS in a conda environment.
Also required are `numpy` and `bqplot` for the live-plotting
and `ipywidgets` for the interactive widgets

    conda install jupyter bqplot pyyaml ipywidgets

`pyyaml` is necessary for rospy.

To install use conda or mamba:

    conda install jupyter-ros -c robostack

##### Development installation

For a development installation (requires npm),

    git clone https://github.com/RoboStack/jupyter-ros.git
    cd jupyter-ros
    pip install -e .
    jupyter nbextension install --py --symlink --sys-prefix jupyros
    jupyter nbextension enable --py --sys-prefix jupyros


To update the `defaults.js` javascript you need to run `python jupyros/ros3d.py`.


## Troubleshooting

You might see a warning like "The rospy package is not found in your $PYTHONPATH.
Subscribe and publish are not going to work. Do you need to activate your ROS environment?"

This is harmless during installation, but if you see this warning in a notebook, you should
check that your ROS environment is activated. You can also set the path from inside the notebook
using

```
import sys
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages/')

# The next line should now work!
import jupyros
```

If you got the following error when you run a cell ```failed to display Jupyter Widget of type VBox```,
you can solve it by running the following command ```jupyter nbextension enable --py --sys-prefix widgetsnbextension```

## License

We use a shared copyright model that enables all contributors to maintain the copyright on their contributions.

This software is licensed under the BSD-3-Clause license. See the LICENSE file for details.
