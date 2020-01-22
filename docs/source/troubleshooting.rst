Troubleshooting
===============

For trouble shooting the ROS 3D widgets, we have prepared the following tips.

The 3D widgets of ROS communicate with the backend through websockets. To make
sure that a websocket connection is established, first, open the "Inspector" (in
FireFox or Chrome, right click -> Inspect Element) and navigate to the "Network"
panel of the Inspector. The network panel shows all requests. If the notebook
cells are now executed we should see a "Websocket" connection being established.

If the websocket connection remains at "Pending..." then you might be using the
wrong Tornado version for the rosbridge websocket server. Using the wrong
Tornado version results in a **silent failure**. It's very important to not mix
the Jupyter Python environment with the rosbridge environment for the time
being, as Jupyter uses a more recent Tornado version than rosbridge websocket
(at least in the default version installed from the official ROS packages).

Therefore, the fix is usually to reinstall the tornado version from the APT packages:

.. code::
  
  sudo apt-get install python-tornado
  sudo apt-get install ros-melodic-rosbridge-suite --reinstall

You can also check the network panel to make sure that mesh files are loaded
correctly.