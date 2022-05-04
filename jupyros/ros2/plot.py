"""
Live Plot class for jupyter-ros2 Project

Author: zmk5 (Zahi Kakish)

"""
from typing import TypeVar
import numpy as np
import bqplot as bq
from jupyros.ros2.subscription import Subscription

try:
    import rclpy
    from rclpy.node import Node
except ModuleNotFoundError:
    print("The rclpy package is not found in your $PYTHONPATH. " +
          "Subscribe and publish are not going to work.")
    print("Do you need to activate your ros2 environment?")


# Used for documentation purposes only
MsgType = TypeVar('MsgType')


class LivePlot(Subscription):
    """
    LivePlot Class
    """
    def __init__(self, node: Node, msg_type: MsgType, topic: str,
                 history: int = 20) -> None:
        # Check if a ros2 node is provided.
        if (not isinstance(node, Node) or not issubclass(type(node), Node)):
            raise TypeError(
                "Input argument 'node' is not of type rclpy.node.Node!")

        self.fields = {
            "xlabel": "x",
            "ylabel": "y",
            "title": "LivePlot",
            "history": history,
            }
        self.data_fields = ["x", "y"]
        self.plot_data = []
        self.scale = {
            "x": bq.LinearScale(),
            "y": bq.LinearScale(),
            }
        self.axis = {
            "x": None,
            "y": None,
            }
        self.lines = None
        self.figure = None

        # Initialize the Subscription class
        super().__init__(node, msg_type, topic, self.plot_callback)

    def display(self) -> bq.Figure:
        """ Displays the plot figure on the jupyter notebook cell """
        # Create figure to display plot data.
        self.axis["x"] = bq.Axis(
            label=self.fields["xlabel"], scale=self.scale["x"],
            grid_lines="solid")
        self.axis["y"] = bq.Axis(
            label=self.fields["ylabel"], scale=self.scale["y"],
            orientation="vertical", grid_lines="solid")
        self.lines = bq.Lines(x=np.array([]), y=np.array([]), scales=self.scale)
        self.figure = bq.Figure(
            axes=[self.axis["x"], self.axis["y"]], marks=[self.lines],
            labels=[self.fields["xlabel"], self.fields["ylabel"]],
            display_legend=True, title=self.fields["title"])

        # Start subscription to begin rclpy.spin()
        self._start_subscription(None)

        return self.figure

    def plot_callback(self, msg: MsgType) -> None:
        """ default callback for LivePlot class """
        incoming_data = []
        for i in self.data_fields:
            incoming_data.append(getattr(msg, i))

        self.plot_data.append(incoming_data)
        self.plot_data = self.plot_data[-self.fields["history"]:]
        self.lines.x = np.arange(len(self.plot_data))
        self.lines.y = np.asarray(self.plot_data).T

    def change_title(self, new_title: str) -> None:
        """ Change title of live plot """
        self.fields["title"] = new_title

    def change_xlabel(self, new_xlabel: str) -> None:
        """ Change x-axis label of live plot """
        self.fields["xlabel"] = new_xlabel

    def change_ylabel(self, new_ylabel: str) -> None:
        """ Change y-axis label of live plot """
        self.fields["ylabel"] = new_ylabel
