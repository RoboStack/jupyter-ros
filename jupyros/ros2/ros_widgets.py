import rosidl_runtime_py.utilities as rut
import ipywidgets as widgets
from ament_index_python.packages import get_package_share_directory
import rosidl_runtime_py.utilities as rut
import functools






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
    #msg_class = msg_instance._fields_and_field_types

  
    
    for idx, slot in enumerate(msg_instance._fields_and_field_types):
        
        
        
        try:
            msg_inst = msg_instance()
        except:
            msg_inst = msg_instance
            
        attr = getattr(msg_inst, slot)

        
        
## Determine if the Message is the a basic form of Messages such as Point, String etc        
        if(type(attr) in [int, str, float]):
            msg_attr = msg_inst.get_fields_and_field_types()
            basic_flag = 1
            if(idx != 0):
                continue
            
        else:
            msg_attr = attr.get_fields_and_field_types()
            basic_flag = 0
        
            
        w = None
        
        if(rut.is_message(msg_instance)):
            if(basic_flag == 1):
                slot = type(msg_inst).__name__
            widget_list.append(widgets.Label(value=slot))

            widget_dict[slot] = {}
        
        for s_t in msg_attr:
            
  
            if  msg_attr[s_t] in ['float', 'float32', 'float64', 'double']:
                w = widgets.FloatText()

            if msg_attr[s_t] in ['int', 'int8', 'uint8', 'int32', 'uint32', 'int64', 'uint64']:
                w = widgets.IntText()

            if msg_attr[s_t] in ['string']:
                w = widgets.Text()
            

                
            if(w):

                widget_dict[slot] = w
                w_box = widgets.HBox([widgets.Label(value=s_t, layout=widgets.Layout(width="100px")), w])
                widget_list.append(w_box)

    return widget_dict, widget_list
            

                

            
            
"""
            if(rut.is_message(msg_instance)):
                
                widget_list.append(widgets.Label(value=slot))

                widget_dict[slot] = {}
            else:
                print("No inst")
"""

def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)

# using wonder's beautiful simplification: https://stackoverflow.com/questions/31174295/getattr-and-setattr-on-nested-objects/31174427?noredirect=1#comment86638618_31174427

def rgetattr(obj, attr, *args):
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)
    return functools.reduce(_getattr, [obj] + attr.split('.'))
