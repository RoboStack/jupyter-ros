import sys
import threading
import ipywidgets as widgets

try:
    import rclpy

except ModuleNotFoundError:
    print("The rclpy package is not found in your $PYTHONPATH. " +
          "Subscribe and publish are not going to work.")
    print("Do you need to activate your ros2 environment?")

OUTPUT_REGISTRY = {}
SUBSCRIBER_REGISTRY = {}


def callback_active():
    return threading.currentThread().name in active_callbacks


class OutputRedirector:
    def __init__(self, original):
        self.original = original

    def write(self, msg):
        thread_name = threading.currentThread().name
        if (thread_name != 'MainThread'
                and OUTPUT_REGISTRY.get(threading.currentThread().getName()) is not None):
            OUTPUT_REGISTRY[threading.currentThread().getName()].append_stdout(msg)
        else:
            self.original.write(msg)

    def flush(self):
        self.original.flush()

    # necessary for ipython, but **not** for xeus-python!
    def set_parent(self, parent):
        self.original.set_parent(parent)

sys.stdout = OutputRedirector(sys.stdout)


def subscribe(node, topic, msg_type, callback):
    """
    Subscribes to a specific topic in another thread, but redirects output!

    @param node An rclpy node class
    @param topic The topic
    @param msg_type The message type
    @param callback The callback

    @return Jupyter output widget
    """
    # Check if a ros2 node is provided.
    if (not isinstance(node, rclpy.node.Node)
            or not issubclass(type(node), rclpy.node.Node)):
        raise TypeError("Input argument 'node' is not of type rclpy.node.Node!")

    if SUBSCRIBER_REGISTRY.get(topic):
        raise RuntimeError("Already registerd...")

    out = widgets.Output(layout={'border': '1px solid gray'})
    SUBSCRIBER_REGISTRY[topic] = node.create_subscription(
        msg_type, topic, callback, 10)
    OUTPUT_REGISTRY[topic] = out

    btn = widgets.Button(description='Stop')

    def stop_start_subscriber(x):
        if OUTPUT_REGISTRY.get(topic) is not None:
            node.destroy_subscription(SUBSCRIBER_REGISTRY[topic])
            del OUTPUT_REGISTRY[topic]
            btn.description = 'Start'
        else:
            OUTPUT_REGISTRY[topic] = out
            SUBSCRIBER_REGISTRY[topic] = node.create_subscription(
                msg_type, topic, callback, 10)
            btn.description = 'Stop'

    btn.on_click(stop_start_subscriber)
    btns = widgets.HBox((btn, ))
    vbox = widgets.VBox((btns, out))
    return vbox
