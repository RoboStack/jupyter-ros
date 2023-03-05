"""
Publisher class for jupyter-ros2 Project

Modified by:    Luigi Dania
Email:          Luigi@dobots.nl
Github:         https://github.com/ldania

Company:        Dobots
Company Repo:   https://github.com/dobots/ 




Original Author: zmk5 (Zahi Kakish)



"""
from typing import TypeVar
import threading
import time
import ipywidgets as widgets
from .ros_widgets import add_widgets, rsetattr, rgetattr
import functools




try:
    import rclpy
    from rclpy.node import Node
except ModuleNotFoundError:
    print("The rclpy package is not found in your $PYTHONPATH. " +
          "Subscribe and publish are not going to work.")
    print("Do you need to activate your ros2 environment?")


# Used for documentation purposes only
MsgType = TypeVar('MsgType')


class Publisher():
    """
    Creates a class containing the form widget for message type `msg_type`.
    This class analyzes the fields of msg_type and creates
    an appropriate widget.

    A ros2 publisher is automatically created which publishes to the
    topic given as topic parameter. This allows pressing the
    "Send Message" button to send the message.

    :param node: An rclpy node class to attach to the publisher.
    :param msg_type: The message type to publish.
    :param topic: The topic name on which to publish the message.

    """
    def __init__(self, node: Node, msg_type: MsgType, topic: str, rate = None ) -> None:
        # Check if a ros2 node is provided.
        if (not isinstance(node, Node) or not issubclass(type(node), Node)):
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
        self.__thread_map[self.topic] = False
        self.__widget_list = []
        self.__widget_dict = {}
        self.__widgets = {
            "rate_field": widgets.IntText(description="Rate", value=5),
            "stop_btn": widgets.Button(description="Start"),
            "send_btn": widgets.Button(description="Send Message"),
            "txt_input": widgets.Text(description="Message", value="Something")
            }
        self.vbox =  None
        if(rate):
            self.node.create_timer(rate, self.__send_msg)
        self.widget_dict, self.widget_list = add_widgets(self.msg_type, self.__widget_dict, self.__widget_list)
    
    def widget_dict_to_msg(self):
        
        """
        Iterate over the widget data and assign them per attribute
        
        
        """
        for key in self.__widget_list:
            if(key.has_trait('children')):
                try:
                    attr_adress = ".".join([head_class, str(key.children[0].value)])
                    #rsetattr(bun,attr_adress, 0.0)
                    rsetattr(self.msg_inst, attr_adress, float(key.children[1].value))
                except:
                    next
            else:
                head_class = key.value
            
            
                
                
    def display(self) -> widgets.VBox:
        """ Display's widgets within the Jupyter Cell for a ros2 Publisher """
        self.__widgets["send_btn"].on_click(self.__send_msg)
        self.__widgets["stop_btn"].on_click(self.__start_thread)
        top_box = widgets.HBox((
            self.__widgets["txt_input"],
        ))
        btm_box = widgets.HBox((
            self.__widgets["send_btn"],
            self.__widgets["rate_field"],
            self.__widgets["stop_btn"],
            
            ))
        self.__widget_list.append(btm_box)
        vbox = widgets.VBox(children=self.__widget_list)
        self.vbox = vbox
        return vbox

    def send_msg(self, args):
        """Call to send message directly"""
        self._send_msg(args)
        
    
    def __send_msg(self, args):
         
        
        """ Generic call to send message. """
        self.msg_inst =  self.msg_type()
        if(self.vbox):
            self.widget_dict_to_msg()
            self.__publisher.publish(self.msg_inst)
        else:
            self.__publisher.publish(args)
    
    def send_msg(self, args):
        """Call to send message directly"""
        self.__send_msg(args)

    def __thread_target(self) -> None:
        d = 1.0 / float(self.__widgets["rate_field"].value)
        while self.__thread_map[self.topic]:
            self.__send_msg(None)
            time.sleep(d)

    def __start_thread(self, _) -> None:
        self.__thread_map[self.topic] = not self.__thread_map[self.topic]
        if self.__thread_map[self.topic]:
            local_thread = threading.Thread(target=self.__thread_target)
            local_thread.start()
            self.__widgets["stop_btn"].description = "Stop"
        else:
            self.__widgets["stop_btn"].description = "Start"

