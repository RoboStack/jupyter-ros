ROS Server Extension
====================

The jupyter-ros package comes with a jupyter server extension to serve static
files (such as robot models) from a catkin workspace.

Once the server extension is installed, you can point the URDFModel URL
parameter to ``http://localhost:8888/rospkg/`` in order for it to search below
that URL for the meshes and other required assets.

The endpoint will use `rospkg` to find assets in your catkin workspace. For
example, the URL ``http://localhost:8888/rospkg/roscpp/CMakeLists.txt`` should
return the CMakeLists file of the roscpp package. This feature is mainly useful
for sending mesh files to the frontend.

.. warning::

  Currently, there is **no** mechanism in place to filter requests based on file
  type. That means, all your source files can be accessed through this endpoint.
