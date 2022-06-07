This is Fork of the Robostack Jupyter-Ros library. It will be used to port the Jupyros library to allow Ros2 implementations and functions. The repo also uses previously developped software from a previous attempt to port Ros2. By zmk5 at https://github.com/zmk5/jupyter-ros2 .

The Ros2 port will be done by DoBots.


To Do:
- Separate the functions of Ros1 and Ros2 into submodules.
- Make the Submodules functional
- Relay data between node and Jupyter notebooks
- Display graphs of the previous recieved data
- (Future) Port Jupyros1 to Jupyros2 3D graphing

The Readme will be adjusted in time to match the newer functions of Ros2

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

![Screen_shot_Ros2](https://user-images.githubusercontent.com/27964546/170959530-820e6dc5-e23f-414e-baf4-23596fb53c10.png)



With jupyter-ros, it's possible to easily create widgets for
custom message types to send messages.

In the future, we plan to bring simple and fast real-time
plotting from ROS topics to this library.

If you find this initial package useful, don't hesitate to
contribute!

If you want to contact the maintaners and creators of the original Jupyros implementation please refer to the original repository of this fork. https://github.com/RoboStack/jupyterlab-ros 

You may also find them at:
You can also always reach out to w.vollprecht@gmail.com or
on twitter: https://twitter.com/wuoulf, or join us on [Gitter](https://gitter.im/RoboStack/Lobby)

If you are interested in the Ros2 port:
Please contact: info@dobots.com

## Installation and Dependencies
(At this moment the Jupyros Ros2 port does not function as a package, please do not attempt to install this repo as a package)

For the Ros 2, you need a Ros2 environment with Rclpy. The use of Simcloud https://github.com/dobots/simcloud is recommended for this. 


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
