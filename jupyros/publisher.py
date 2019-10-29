"""
Publisher class for jupyter-ros2 Project

Author: zmk5 (Zahi Kakish)

"""
import threading
import time
import ipywidgets as widgets

try:
    import rclpy
except ModuleNotFoundError:
    print("The rclpy package is not found in your $PYTHONPATH. " +
          "Subscribe and publish are not going to work.")
    print("Do you need to activate your ros2 environment?")


class Publisher():
    """
    Create a class containing the form widget for message type `msg_type`.
    This class analyzes the fields of msg_type and creates
    an appropriate widget.
    A publisher is automatically created which publishes to the
    topic given as topic parameter. This allows pressing the
    "Send Message" button to send the message.

    :param node: An rclpy node class to attach to the publisher.
    :param msg_type: The message type to publish.
    :param topic: The topic name on which to publish the message.

    """
    def __init__(self, node, msg_type, topic):
        # Check if a ros2 node is provided.
        if (not isinstance(node, rclpy.node.Node)
                or not issubclass(type(node), rclpy.node.Node)):
            raise TypeError(
                "Input argument 'node' is not of type rclpy.node.Node!")

        # Check if topic already created.
        for operating_publisher in node.publishers:
            if topic[0] != "/":
                if "/" + topic is operating_publisher.topic:
                    raise AttributeError(
                        f"Publisher for topic, /{topic}, already created!")

            if topic is operating_publisher.topic:
                raise AttributeError(
                    f"Publisher for topic, {topic}, already created!")

        # Set initial node and widget variables.
        self.node = node
        self.topic = topic
        self.msg_type = msg_type
        self.__publisher = self.node.create_publisher(msg_type, topic, 10)
        self.__thread_map = {}
        self.__widget_list = []
        # self.__widget_dict = {}
        self.__widgets = {
            "rate_field": widgets.IntText(description="Rate", value=5),
            "stop_btn": widgets.Button(description="Start"),
            "send_btn": widgets.Button(description="Send Message"),
            }

    def setup_widgets(self):
        """ Display's widgets within the Jupyter Cell for a ros2 Publisher """
        self.__widgets["send_btn"].on_click(self.__send_msg)
        self.__widgets["stop_btn"].on_click(self.__start_thread)
        btm_box = widgets.HBox((
            self.__widgets["send_btn"],
            self.__widgets["rate_field"],
            self.__widgets["stop_btn"],
            ))
        self.__widget_list.append(btm_box)
        vbox = widgets.VBox(children=self.__widget_list)

        return vbox

    def __send_msg(self, arg):
        """ Generic call to send message. """
        msg_to_send = self.msg_type()
        # widget_dict_to_msg(msg_to_send, widget_dict)
        self.__publisher.publish(msg_to_send)
        print("Message Sent!")

    def __thread_target(self):
        d = 1.0 / float(self.__widgets["rate_field"].value)
        while self.__thread_map[self.topic]:
            self.__send_msg(None)
            time.sleep(d)

    def __start_thread(self, click_args):
        self.__thread_map[self.topic] = not self.__thread_map[self.topic]
        if self.__thread_map[self.topic]:
            local_thread = threading.Thread(target=self.__thread_target)
            local_thread.start()
            self.__widgets["stop_btn"].description = "Stop"
        else:
            self.__widgets["stop_btn"].description = "Start"
