"""
Subscription class for jupyter-ros2 Project

Modified by: ldania (Luigi Dania)
Date: 22-July-2022


Original Author: zmk5 (Zahi Kakish)


"""
from typing import TypeVar
from typing import Callable
import threading
import functools
import ipywidgets as widgets

try:
    import rclpy
    from rclpy.node import Node
except ModuleNotFoundError:
    print("The rclpy package is not found in your $PYTHONPATH. " +
          "Subscribe and publish are not going to work.")
    print("Do you need to activate your ros2 environment?")


# Used for documentation purposes only
MsgType = TypeVar('MsgType')


class Subscriber():
    """
    Creates a class containing the form widget for message type `msg_type`.
    This class analyzes the fields of msg_type and creates
    an appropriate widget.

    A ros2 subscription is automatically created which spins for 10Hz waiting
    for an incoming message for a topic. The default position of the widgets is
    `Stop`. Callback execution will not occur until `Start` is pressed. It is
    suggested that the user presses `Stop` on all active jupyter-ros2
    Subscription widgets before restarting the kernel to assure proper exiting
    of the thread that is constantly spinning.

    :param node: An rclpy node class to attach to the publisher.
    :param msg_type: The message type to publish.
    :param topic: The topic name on which to publish the message.
    :param callback: The user-defined callback function
    :param print_incoming_msg: If the user wishes, the message can be outputted
        directly to the cell widget. By default this is turned off.

    """
    def __init__(self, node: Node, msg_type: MsgType, topic: str,
                 callback: Callable, print_incoming_msg: bool = False) -> None:
        # Check if a ros2 node is provided.
        if (not isinstance(node, Node) or not issubclass(type(node), Node)):
            raise TypeError(
                "Input argument 'node' is not of type rclpy.node.Node!")

        # Check if topic already created.
        for operating_subscription in node.subscriptions:
            if topic[0] != "/":
                if "/" + topic is operating_subscription.topic:
                    raise AttributeError(
                        f"Subscription for topic, /{topic}, already created!")

            if topic is operating_subscription.topic:
                raise AttributeError(
                    f"Subscription for topic, {topic}, already created!")

        self.node = node
        self.topic = topic
        self.msg_type = msg_type
        self.data = None
        self.__thread_state = False
        self.__widgets = {
            "start_btn": widgets.Button(description='Start'),
            "stop_btn": widgets.Button(description='Stop'),
            "out": widgets.Output(layout={'border': '1px solid gray'}),
            }

        if print_incoming_msg:
            self.__subscription = node.create_subscription(
                msg_type, topic, self.__print_msg(self.__data_msg(callback)),
                10)

        else:
            self.__subscription = node.create_subscription(
                msg_type, topic, callback, 10)

    def display(self) -> widgets.VBox:
        """ Display's widgets in the Jupyter Cell for a ros2 Subscription """
        self.__widgets["start_btn"].on_click(self._start_subscription)
        self.__widgets["stop_btn"].on_click(self._stop_subscription)
        btns = widgets.HBox((
            self.__widgets["start_btn"],
            self.__widgets["stop_btn"],
            ))
        vbox = widgets.VBox((
            btns,
            self.__widgets["out"]
            ))
        return vbox


    def __thread_target(self) -> None:
        while self.__thread_state:
            rclpy.spin_once(self.node, timeout_sec=1)
        self.__widgets["out"].append_stdout("Done!\n")

    def _stop_subscription(self, _) -> None:
        self.__thread_state = False

    def _start_subscription(self, _) -> None:
        self.__thread_state = True
        local_thread = threading.Thread(target=self.__thread_target)
        local_thread.start()

    def __data_msg(self, func: Callable) -> Callable:
        """
        Decorator for saving data from a callback.

        This is a simple decorator that forces returned data from the callback
        function to store jupyter-ros2 Subscription's `data` attribute.

        """
        @functools.wraps(func)
        def data_modifier(*args, **kargs):
            self.data = func(*args, **kargs)
            return self.data

        return data_modifier

    def __print_msg(self, func: Callable) -> Callable:
        """
        Decorator for printing incoming ros2 messages.

        This is a simple decorator that forces the first argument given to the
        callback (typically the ros2 msg) to display it's information to the
        Jupyter notebook's cell containing the Subscription widgets after
        executing the initial callback function.

        """
        @functools.wraps(func)
        def print_modifier(*args, **kargs):
            func(*args, **kargs)
            self.__widgets["out"].append_stdout(f"{args[0]}\n")

        return print_modifier
    
    
def subscribe(topic, msg_type, callback):
    """
    Subscribes to a specific topic in another thread, but redirects output!

    @param topic The topic
    @param msg_type The message type
    @param callback The callback

    @return Jupyter output widget
    """

    if subscriber_registry.get(topic):
        print("Removing previous callback, only one redirection possible right now", file=sys.stderr)
        subscriber_registry[topic].unregister()

    out = widgets.Output(layout={'border': '1px solid gray'})
    subscriber_registry[topic] = rcply.Subscriber(topic, msg_type, callback)
    output_registry[topic] = out

    btn = widgets.Button(description='Stop')

    def stop_start_subscriber(x):
        if output_registry.get(topic) is not None:
            subscriber_registry[topic].unregister()
            del output_registry[topic]
            btn.description = 'Start'
        else:
            output_registry[topic] = out
            subscriber_registry[topic] = rclpy.Subscriber(topic, msg_type, callback)
            btn.description = 'Stop'

    btn.on_click(stop_start_subscriber)
    btns = widgets.HBox((btn, ))
    vbox = widgets.VBox((btns, out))
    return vbox


