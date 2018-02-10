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
Also required are `numpy` and `bqplot` for the live-plotting.
The easiest way to install `jupyter` and `bqplot` is with
conda. My recommendation is to set up a Miniconda 2 
environment, and then run:

```
conda install jupyter bqplot pyyaml
```

`pyyaml` is necessary for rospy.