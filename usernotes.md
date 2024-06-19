#### Notes from practical use

##### Background: 
* These should probably be merged into the wiki or readme
* I am writing these as I discover how to use this library
* YMMV

##### Jupyter Notebook
* Don't place the whole node in a single "cell"
* For example, place the code for the callback in its own Jupyter Notebook Cell

##### Jupyter Lab
* So far I have not managed to get things to work with Jupyter Lab

#### Ana- and Mini- Conda
* After a lot of experimentation (Ubuntu 16.04) I have come to the conclusion that Anaconda and ROS don't play nice together
* I wanted to avoid hacking with the default installation scripts for both
* I found that if I install conda first then ros, or the other way around, things would fail
* I am pretty sure it's because conda installs its own python and with two different pythons things get messed up.

##### RosCore
* Roscore is not automatically started
* To make things work in Jupyter Notebook you will need to start roscore separately

##### jr.subscribe()
* Is the jupyter-ros equivalent to rospy.Subscriber
* Sample: `jr.subscribe('/number', Int32, callback)`
* It does the same thing, but, it captures console output that the callback might produce

##### Don't put multiple nodes in one Notebook
* Still investigating the threading model
* But for now, it just doesnt work


