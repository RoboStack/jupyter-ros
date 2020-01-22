Publishing and Receiving
========================

Subscribing to a ROS topic
--------------------------

The jupyter-ros tools help publish and receive messages in the jupyter notebook.

For the publishing, the package contains tools to automatically generate widgets 
from message definitions. For receiving, the jupyter-ros package contains a helper
that redirects output to a specific output widget (instead of spamming the entire notebook).

.. code-block:: python

  import jupyros as jr
  import rospy
  from std_msgs.msg import String

  rospy.init_node('jupyter_node')
  jr.subscribe('/sometopic', String, lambda msg: print(msg))

This creates a output widget, and buttons to toggle (stop or start) receiving
messages. Internally, jupyter-ros will save a handle to the subscriber thread.
Note that we did not use the rospy-way of creating a subscriber but delegated this
to the jupyter-ros package.

If we now send a message from the command line to the Notebook, we see message being
printed to the output widget below the cell where we executed the `jr.subscribe`.

.. code-block:: shell

  $ rostopic pub /sometopic std_msgs/String "data: 'hello jupyter'" -r 10

Publishing to a ROS topic
-------------------------

In the same way we can publish to a ROS topic by using the `jr.publish` helper.

.. code-block:: python

  import jupyros as jr
  import rospy
  from std_msgs.msg import String

  rospy.init_node('jupyter_node')
  jr.publish('/sometopic', String)

This results in a jupyter widget where one can insert the desired message in the 
text field.
The form fields (jupyter widgets) are generated automatically from the message
definition. If we use a a different message type, we will get different fields.
For example, a ``Vector3`` message type contains three float fields (x,y, and z)
for which we will get 3 ``FloatTextField`` instances -- these can only hold float 
values (and not text).

