import rosidl_runtime_py.utilities as rut
import ipywidgets as widgets
from ament_index_python.packages import get_package_share_directory
import rosidl_runtime_py.utilities as rut

def add_widgets(msg_instance, widget_dict, widget_list, prefix=''):
    """
    Adds widgets.

    @param msg_type The message type
    @param widget_dict The form list
    @param widget_list The widget list

    @return widget_dict and widget_list
    """
    # import only here so non ros env doesn't block installation
    #from genpy import Message
    """
    if msg_instance._type.split('/')[-1] == 'Image':
        w = widgets.Text()
        widget_dict['img'] = w
        w_box = widgets.HBox([widgets.Label(value='Image path:'), w])
        widget_list.append(w_box)
        return widget_dict, widget_list
    """
    if(rut.is_message(msg_instance) == False):
        return 0
    ## Type of msg
    msg_class = msg_instance._fields_and_field_types

  
    
    for idx, slot in enumerate(msg_instance.__slots__):
        attr = getattr(msg_instance, slot)
        print(attr)
        s_t = msg_instance.SLOT_TYPES[idx]
        print(s_t)
        w = None

        if s_t in ['float32', 'float64']:
            w = widgets.FloatText()
        if s_t in ['int8', 'uint8', 'int32', 'uint32', 'int64', 'uint64']:
            w = widgets.IntText()

        if s_t in ['string']:
            w = widgets.Text()
              
        #if isinstance(attr, Message):
        try:
            widget_list.append(widgets.Label(value=slot))
        
            widget_dict[slot] = {}
            add_widgets(attr, widget_dict[slot], widget_list, slot)
        except:
            print("next")
        try:
            widget_dict[slot] = w
            w_box = widgets.HBox([widgets.Label(value=slot, layout=widgets.Layout(width="100px")), w])
            widget_list.append(w_box)
        except:
            print("next one")
    return widget_dict, widget_list