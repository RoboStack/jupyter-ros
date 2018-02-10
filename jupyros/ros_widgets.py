import rospy
import std_msgs
import bqplot as bq
import ipywidgets as widgets
import numpy as np


def _add_widgets(msg_type, form_list, widget_list):
    """
    Adds widgets.
    
    @param msg_type The message type
    @param form_list The form list
    @param widget_list The widget list
    
    @return form_list and widget_list
    """
    for idx, slot in enumerate(msg_type.__slots__):
        attr = getattr(msg_type, slot)
        s_t = msg_type._slot_types[idx]
        w = None

        if s_t in ['float32', 'float64']:
            w = widgets.FloatSlider()
        if s_t in ['int8', 'uint8', 'int32', 'uint32', 'int64', 'uint64']:
            w = widgets.IntSlider()
        if s_t in ['string']:
            w = widgets.Text()
        if w == None:
            add_widgets(s_t, form_list, widget_list)
        form_list.append(w)
        w_box = widgets.HBox([widgets.Label(value=slot), w])
        widget_list.append(w_box)
    return form_list, widget_list

def widget_for_msg(msg_type, publish_to):
    """
    Create a form widget for message type msg_type.
    This function analyzes the fields of msg_type and creates
    an appropriate widget.
    A publisher is automatically created which publishes to the
    topic given as publish_to parameter. This allows pressing the 
    "Send Message" button to send the message to ROS.
    
    @param msg_type The message type
    @param publish_to The topic name to publish to
    
    @return jupyter widget for display
    """
    widget_list, form_list = [], []
    publisher = rospy.Publisher(publish_to, msg_type, queue_size=10)

    form_list, widget_list = _add_widgets(msg_type, [], [])
    send_btn = widgets.Button(description="Send Message")
    
    def send_msg(arg):
        arg_list = [wdg.value for wdg in form_list]
        msg = msg_type(*arg_list)
        publisher.publish(msg)
    
    send_btn.on_click(send_msg)
    widget_list.append(send_btn)
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