import rospy
import std_msgs
import bqplot as bq
import ipywidgets as widgets
import numpy as np
from genpy import Message

def add_widgets(msg_instance, widget_dict, widget_list, prefix=''):
    """
    Adds widgets.
    
    @param msg_type The message type
    @param widget_dict The form list
    @param widget_list The widget list
    
    @return widget_dict and widget_list
    """
    for idx, slot in enumerate(msg_instance.__slots__):
        attr = getattr(msg_instance, slot)
        s_t = msg_instance._slot_types[idx]
        w = None

        if s_t in ['float32', 'float64']:
            w = widgets.FloatSlider()
        if s_t in ['int8', 'uint8', 'int32', 'uint32', 'int64', 'uint64']:
            w = widgets.IntSlider()
        if s_t in ['string']:
            w = widgets.Text()

        if isinstance(attr, Message):
            widget_list.append(widgets.Label(value=slot))
            widget_dict[slot] = {}
            add_widgets(attr, widget_dict[slot], widget_list, slot)

        if w:
            widget_dict[slot] = w
            w_box = widgets.HBox([widgets.Label(value=slot), w])
            widget_list.append(w_box)

    return widget_dict, widget_list

def widget_dict_to_msg(msg_instance, d):
    for key in d:
        if isinstance(d[key], widgets.Widget):
            setattr(msg_instance, key, d[key].value)
        else:
            submsg = getattr(msg_instance, key)
            widget_dict_to_msg(submsg, d[key])

def publish(topic, msg_type):
    """
    Create a form widget for message type msg_type.
    This function analyzes the fields of msg_type and creates
    an appropriate widget.
    A publisher is automatically created which publishes to the
    topic given as topic parameter. This allows pressing the 
    "Send Message" button to send the message to ROS.
    
    @param msg_type The message type
    @param topic The topic name to publish to
    
    @return jupyter widget for display
    """
    publisher = rospy.Publisher(topic, msg_type, queue_size=10)

    widget_list = []
    widget_dict = {}

    add_widgets(msg_type(), widget_dict, widget_list)
    send_btn = widgets.Button(description="Send Message")
    
    def send_msg(arg):
        msg_to_send = msg_type()
        widget_dict_to_msg(msg_to_send, widget_dict)
        publisher.publish(msg_to_send)

    send_btn.on_click(send_msg)
    latch_check = widgets.Checkbox(description="Latch Message")
    rate_field = widgets.IntText(description="Rate")
    stop_btn = widgets.Button(description="Stop")

    btm_box = widgets.HBox((send_btn, latch_check, rate_field, stop_btn))
    widget_list.append(btm_box)
    vbox = widgets.VBox(children=widget_list)
    
    return vbox

def live_plot(plot_string, topic_type, history=100, title=None):
    topic = plot_string[:plot_string.find(':') - 1]
    title = title if title else topic
    fields = plot_string.split(':')[1:]
    x_sc = bq.LinearScale()
    y_sc = bq.LinearScale()

    ax_x = bq.Axis(label='X', scale=x_sc, grid_lines='solid')
    ax_y = bq.Axis(label='Y', scale=y_sc, orientation='vertical', grid_lines='solid')

    lines = bq.Lines(x=np.array([]), y=np.array([]), scales={'x': x_sc, 'y': y_sc})
    fig = bq.Figure(axes=[ax_x, ax_y], marks=[lines], labels=fields, display_legend=True, title=title)
    data = []

    def cb(msg, data=data):
        data_el = []
        for f in fields:
            data_el.append(getattr(msg, f))
        data.append(data_el)
        data = data[-history:]
        ndat = np.asarray(data).T
        if lines:
            lines.y = ndat
            lines.x = np.arange(len(data))

    rospy.Subscriber(topic, topic_type, cb)
    return fig