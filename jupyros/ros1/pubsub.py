#############################################################################
# Copyright (c) Wolf Vollprecht, QuantStack                                 #
#                                                                           #
# Distributed under the terms of the BSD 3-Clause License.                  #
#                                                                           #
# The full license is in the file LICENSE, distributed with this software.  #
#############################################################################

from __future__ import print_function

import sys
import threading

import ipywidgets as widgets

try:
    import rospy
except:
    print("The actionlib package is not found in your $PYTHONPATH. Action clients are not going to work.")
    print("Do you need to activate your ROS environment?")
    pass

output_registry = {}
subscriber_registry = {}

def callback_active():
    return threading.currentThread().name in active_callbacks

class OutputRedirector:
    def __init__(self, original):
        self.original = original
    
    def isatty(self):
        return self.original.isatty()

    def write(self, msg):
        thread_name = threading.currentThread().name
        if thread_name != 'MainThread' and output_registry.get(threading.currentThread().getName()) is not None:
            output_registry[threading.currentThread().getName()].append_stdout(msg)
        else:
            self.original.write(msg)

    def flush(self):
        self.original.flush()

    # necessary for ipython, but **not** for xeus-python!
    def set_parent(self, parent):
        self.original.set_parent(parent)

sys.stdout = OutputRedirector(sys.stdout)

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
    subscriber_registry[topic] = rospy.Subscriber(topic, msg_type, callback)
    output_registry[topic] = out

    btn = widgets.Button(description='Stop')

    def stop_start_subscriber(x):
        if output_registry.get(topic) is not None:
            subscriber_registry[topic].unregister()
            del output_registry[topic]
            btn.description = 'Start'
        else:
            output_registry[topic] = out
            subscriber_registry[topic] = rospy.Subscriber(topic, msg_type, callback)
            btn.description = 'Stop'

    btn.on_click(stop_start_subscriber)
    btns = widgets.HBox((btn, ))
    vbox = widgets.VBox((btns, out))
    return vbox
