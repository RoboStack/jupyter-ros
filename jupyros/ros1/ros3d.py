#############################################################################
# Copyright (c) Wolf Vollprecht, QuantStack                                 #
#                                                                           #
# Distributed under the terms of the BSD 3-Clause License.                  #
#                                                                           #
# The full license is in the file LICENSE, distributed with this software.  #
#############################################################################

import os

from traitlets import *
import ipywidgets as widgets

from .._version import _fetchJSVersion

js_version = '^' + _fetchJSVersion()


def _quick_widget(package_name, version, has_view=True):
    def quick_widget_decorator(cls):
        from traitlets import Unicode
        name = cls.__name__
        if name.endswith('Model'):
            name = name[:-5]

        cls._model_name = Unicode(name + 'Model').tag(sync=True)
        cls._model_name.class_init(cls, '_model_name')

        cls._model_module = Unicode(package_name).tag(sync=True)
        cls._model_module.class_init(cls, '_model_module')

        cls._model_module_version = Unicode(version).tag(sync=True)
        cls._model_module_version.class_init(cls, '_model_module_version')

        if has_view:
            cls._view_module = Unicode(package_name).tag(sync=True)
            cls._view_module.class_init(cls, '_view_module')

            cls._view_module_version = Unicode(version).tag(sync=True)
            cls._view_module_version.class_init(cls, '_view_module_version')

            cls._view_name = Unicode(name + 'View').tag(sync=True)
            cls._view_name.class_init(cls, '_view_name')

        cls = widgets.register(cls)
        return cls

    return quick_widget_decorator


register = _quick_widget('@robostack/jupyter-ros', js_version)
register_noview = _quick_widget('@robostack/jupyter-ros', js_version, False)
sync_widget = {'sync': True}
sync_widget.update(widgets.widget_serialization)

@register_noview
class ROSConnection(widgets.Widget):
    """ Base ROS Connection

    The ROS Connection widget has the parameters with the websocket endpoint to 
    communicate with the rosbridge server.

    Parameters
    ----------

    url : Unicode
        URL endpoint of the websocket. Defaults to ``ws://{hostname}:9090``
        where ``{hostname}`` is replaced by the current hostname at runtime
        (e.g. ``localhost``). You can override the default value by setting the
        ``JUPYTER_WEBSOCKET_URL`` environment variable.
    """
    url = Unicode(os.environ.get("JUPYROS_WEBSOCKET_URL", "ws://{hostname}:9090")).tag(sync=True)

@register_noview
class TFClient(widgets.Widget):
    """ Base TF Client
    
    The TFClient keeps track of TF frames.

    Parameters
    ----------

    angular_treshold: Float
        The angular threshold for the TF republisher (default: 0.01)
    translational_treshold: Float
        The translation threshold for the TF republisher (default: 0.01)
    rate: Float
        Update and publish rate for the TF republisher (default: 10.0)
    fixed_frame: Unicode
        Fixed base frame for TF tree (default: '')
    """
    ros = Instance(ROSConnection).tag(**sync_widget)
    angular_treshold = Float(0.01).tag(sync=True)
    translational_treshold = Float(0.01).tag(sync=True)
    rate = Float(10.0).tag(sync=True)
    fixed_frame = Unicode('').tag(sync=True)

@register
class URDFModel(widgets.Widget):
    """ A URDFModel (Robot model)

    Displays a 3D robot model.
    
    Parameters
    ----------
    ros: ROSConnection instance
        Instance of the ROS Connection that should be used
    tf_client: TFClient instance
        Instance of the TF Client that should be used
    url: Unicode
        URL from which to fetch the _assets_ (mesh and texture files). This can 
        be either the jupyter-ros server extension (in this case one should use
        "http://{hostname}:{port}/rospkg/") or another server / URL where the 
        mesh files can be downloaded (default: "http://{hostname}:3000")
    """
    ros = Instance(ROSConnection).tag(**sync_widget)
    tf_client = Instance(TFClient).tag(**sync_widget)
    url = Unicode(
        os.environ.get("JUPYROS_ASSETS_URL", "http://{hostname}:3000")
    ).tag(sync=True)

@register
class GridModel(widgets.Widget):
    """ A simple GridModel

    Displays a 3D grid.
    
    Parameters
    ----------

    cell_size: Float
        Size of the cells in meters
    color: Unicode
        Color of the grid lines (e.g. a hex specifier ``#FF0000``)
    num_cells: Int
        Number of cells in length, and width (default: 2)0
    """
    cell_size = Float(0.5).tag(sync=True)
    color = Unicode("#0181c4").tag(sync=True)
    num_cells = Int(20).tag(sync=True)

@register
class OccupancyGrid(widgets.Widget):
    """ Displays an Occupancy Grid

    Parameters
    ----------

    ros: ROSConnection instance
        Instance of the ROS Connection that should be used
    tf_client: TFClient instance
        Instance of the TF Client that should be used
    topic: Unicode
        The topic to subscribe to (default: ``/basic_controls``)
    continuous: Book
        Wether the occupancy grid should continously update (default: False)
    compression: Unicode
        Compression mechanism (default: 'cbor')
    color: Unicode
        Color of the grid lines (e.g. a hex specifier ``#FFFFFF``)
    opacity: Float
        Opacity of the occupancy grid (default: 1.0)
    """
    ros = Instance(ROSConnection).tag(**sync_widget)
    tf_client = Instance(TFClient).tag(**sync_widget)
    topic = Unicode("/map").tag(sync=True)
    continuous = Bool(False).tag(sync=True)
    compression = Unicode('cbor').tag(sync=True)
    color = Unicode('#FFFFFF').tag(sync=True)
    opacity = Float(1.0).tag(sync=True)
    # rootObject =
    # offsetPose =

@register
class InteractiveMarker(widgets.Widget):
    """ Interactive Marker Widget

    Displays an interactive marker in the Viewer.

    Parameters
    ----------

    ros: ROSConnection instance
        Instance of the ROS Connection that should be used
    tf_client: TFClient instance
        Instance of the TF Client that should be used
    topic: Unicode
        The topic to subscribe to (default: ``/basic_controls``)
    menu_font_size: Unicode
        Menu font size (default: ``'0.8em'``)
    """

    ros = Instance(ROSConnection).tag(**sync_widget)
    tf_client = Instance(TFClient).tag(**sync_widget)
    topic = Unicode('/basic_controls').tag(sync=True)
    menu_font_size = Unicode('0.8em').tag(sync=True)
    # camera =
    # rootObject

@register
class Marker(widgets.Widget):
    """ Displays a Marker message

    Parameters
    ----------

    ros: ROSConnection instance
        Instance of the ROS Connection that should be used
    tf_client: TFClient instance
        Instance of the TF Client that should be used
    topic: Unicode
        Name of the topic (default: ``/pose_array``)
    path: Unicode
        Marker path (default: ``/``)
    lifetime: Float
        Lifetime of marker in seconds, 0 for infinity (default: 0.0)
    """
    ros = Instance(ROSConnection).tag(**sync_widget)
    tf_client = Instance(TFClient).tag(**sync_widget)
    topic = Unicode('/visualization_marker').tag(sync=True)
    path = Unicode('/').tag(sync=True)
    lifetime = Float(0.0).tag(sync=True)

@register
class PoseArray(widgets.Widget):
    """ Displays a PoseArray message 
    
    Parameters
    ----------

    ros: ROSConnection instance
        Instance of the ROS Connection that should be used
    tf_client: TFClient instance
        Instance of the TF Client that should be used
    topic: Unicode
        Name of the topic (default: ``/pose_array``)
    color: Unicode
        Hex string of the color for visualization (default: ``"#CC00FF"``)
    length: Float
        Length of pose arrows (default: 1m)
    """
    ros = Instance(ROSConnection).tag(**sync_widget)
    tf_client = Instance(TFClient).tag(**sync_widget)
    topic = Unicode('/pose_array').tag(sync=True)
    color = Unicode('#CC00FF').tag(sync=True)
    length = Float(1.0).tag(sync=True)

@register
class Pose(widgets.Widget):
    """ Displays a Pose message

    Parameters
    ----------

    ros: ROSConnection instance
        Instance of the ROS Connection that should be used
    tf_client: TFClient instance
        Instance of the TF Client that should be used
    topic: Unicode
        Name of the topic (default: ``/pose``)
    color: Unicode
        Hex string of the color for visualization (default: ``"#CC00FF"``)
    length: Float
        Length of pose arrows (default: 1m)
    """
    ros = Instance(ROSConnection).tag(**sync_widget)
    tf_client = Instance(TFClient).tag(**sync_widget)
    topic = Unicode('/pose').tag(sync=True)
    color = Unicode('#CC00FF').tag(sync=True)
    length = Float(1.0).tag(sync=True)

@register
class Polygon(widgets.Widget):
    """ Displays a Polygon message
    
    Parameters
    ----------

    ros: ROSConnection instance
        Instance of the ROS Connection that should be used
    tf_client: TFClient instance
        Instance of the TF Client that should be used
    topic: Unicode
        Name of the topic (default: ``/polygon``)
    color: Unicode
        Hex string of the color for visualization (default: ``"#CC00FF"``)
    """
    ros = Instance(ROSConnection).tag(**sync_widget)
    tf_client = Instance(TFClient).tag(**sync_widget)
    topic = Unicode('/polygon').tag(sync=True)
    color = Unicode('#CC00FF').tag(sync=True)

@register
class Path(widgets.Widget):
    """ Displays a Path message
    
    Parameters
    ----------

    ros: ROSConnection instance
        Instance of the ROS Connection that should be used
    tf_client: TFClient instance
        Instance of the TF Client that should be used
    topic: Unicode
        Name of the topic (default: ``/path``)
    color: Unicode
        Hex string of the color for visualization (default: ``"#CC00FF"``)
    """
    ros = Instance(ROSConnection).tag(**sync_widget)
    tf_client = Instance(TFClient).tag(**sync_widget)
    topic = Unicode('/path').tag(sync=True)
    color = Unicode('#CC00FF').tag(sync=True)

@register
class LaserScan(widgets.Widget):
    """ Displays a LaserScan message
    
    Parameters
    ----------

    ros: ROSConnection instance
        Instance of the ROS Connection that should be used
    tf_client: TFClient instance
        Instance of the TF Client that should be used
    topic: Unicode
        Name of the topic (default: ``/scan``)
    point_ratio: Float
        Ratio of points to send to the frontend (default: 1.0)
    message_ratio: Float
        Ratio of messages to send to the frontend (default: 1.0)
    max_points: Int
        Maximum number of points to display (default: 200000)
    color_source: Unicode
        Source field for the color information (default: 'intensities')
    color_map: Unicode
        Color map function (default: '')
    point_size: Float
        Point size (default: 0.05)
    static_color: Unicode
        Hex string of the color for visualization (default: ``"#FF0000"``)
    """

    ros = Instance(ROSConnection).tag(**sync_widget)
    tf_client = Instance(TFClient).tag(**sync_widget)
    topic = Unicode('/scan').tag(sync=True)
    point_ratio = Float(1.0).tag(sync=True)
    message_ratio = Float(1.0).tag(sync=True)
    max_points = Int(200000).tag(sync=True)
    color_source = Unicode('intensities').tag(sync=True)
    color_map = Unicode('').tag(sync=True)
    point_size = Float(0.05).tag(sync=True)
    static_color = Unicode("#FF0000").tag(sync=True)

@register
class MarkerArrayClient(widgets.Widget):
    """ MarkerArrayClient

    A client that listens to changes in a MarkerArray and triggers the update of 
    a visualization.

    Parameters
    ----------

    ros: ROSConnection instance
        Instance of the ROS Connection that should be used
    tf_client: TFClient instance
        Instance of the TF Client that should be used
    topic: Unicode
        Name of the topic (default: ``/marker_array``)
    path: Unicode
        The base path to any meshes that will be loaded (default: ``'/'``)
    """
    ros = Instance(ROSConnection).tag(**sync_widget)
    tf_client = Instance(TFClient).tag(**sync_widget)
    topic = Unicode('/marker_array').tag(sync=True)
    path = Unicode('/').tag(sync=True)

@register
class PointCloud(widgets.Widget):
    """ Displays a PointCloud message
    
    Parameters
    ----------

    ros: ROSConnection instance
        Instance of the ROS Connection that should be used
    tf_client: TFClient instance
        Instance of the TF Client that should be used
    topic: Unicode
        Name of the topic (default: ``/point_cloud``)
    point_ratio: Float
        Ratio of points to send to the frontend (default: 1.0)
    message_ratio: Float
        Ratio of messages to send to the frontend (default: 1.0)
    max_points: Int
        Maximum number of points to display (default: 200000)
    point_size: Float
        Point size (default: 0.05)
    static_color: Unicode
        Hex string of the color for visualization (default: ``"#FF0000"``)
    """

    ros = Instance(ROSConnection).tag(**sync_widget)    
    tf_client = Instance(TFClient).tag(**sync_widget)
    topic = Unicode('/point_cloud').tag(sync=True)
    message_ratio = Float(2.0).tag(sync=True)
    point_ratio = Float(3.0).tag(sync=True)
    max_points = Int(200000).tag(sync=True)
    point_size = Float(0.05).tag(sync=True)
    static_color = Unicode("#FF0000").tag(sync=True)
    # color_map = Unicode('').tag(sync=True)
    # color_source = Unicode('').tag(sync=True)

@register
class Viewer(widgets.DOMWidget):
    """ Viewer class

    This is the class that represents the actual 3D viewer widget. The viewer
    is derived from the ``ipywidgets.DOMWidget`` and therefore also implements
    the ``layout`` attribute which can be used to modify the CSS layout of the 
    viewer.

    Parameters
    ----------

    background_color: Unicode
        Background color of the viewer (default: ``'#FFFFFF'``)
    alpha: Float
        The alpha value of the background
    objects: List of ROS3D widget instances
        Objects to render in the viewer (e.g. Marker, PointCloud, ...)
    """
    background_color = Unicode('#FFFFFF').tag(sync=True)
    alpha = Float(1.0).tag(sync=True)
    height = Unicode('100%').tag(sync=True)
    objects = List(Instance(widgets.Widget)).tag(**sync_widget)

@register_noview
class DepthCloud(widgets.Widget):
    """
    Display a Depth Cloud for a RGB-D cloud (needs infrastructure on the server side.)
    """
    url = Unicode('').tag(sync=True)
    f = Float(525.0).tag(sync=True)

@register
class SceneNode(widgets.Widget):
    """ Scene Node (to be used in conjunction with DepthCloud)
    """
    frame_id = Unicode('/base_link').tag(sync=True)
    tf_client = Instance(TFClient).tag(**sync_widget)
    object = Instance(DepthCloud).tag(**sync_widget)

def js_formatter(d_in):
    import json
    from traitlets import utils as tut
    def remove_undefined(d):
        for key in d:
            if type(d[key]) is tut.sentinel.Sentinel:
                d[key] = '##UNDEFINED##'
            if type(d[key]) is dict:
                remove_undefined(d[key])

    remove_undefined(d_in)
    s = '{\n'
    for key in sorted(d_in.keys()):
        val = d_in[key] if d_in[key] is not None else "null"
        if type(val) is str:
            val = '"' + val + '"'
        if type(val) is bool:
            val = 'true' if val else 'false'

        s += '    {}: {},\n'.format(key, val)
    # s = json.dumps(d_in, indent=4)
    s += '}\n'
    s = s.replace('"##UNDEFINED##"', 'null')
    return s


def js_extract_cls(cls):

    template = """
var {class_name}Defaults = {json_defaults}
    """

    name = cls.__name__
    if not name.endswith('Model'):
        name += 'Model'

    defd = cls.__base__.class_trait_names()
    base_class_name = cls.__base__.__name__
    defaults = {}
    # trait_values = cls()._trait_values
    trait_values = cls.__dict__
    forward_traits = ['_model_module', '_model_name', '_model_module_version', '_view_name', '_view_module', '_view_module_version']
    for key, item in trait_values.items():
        if key not in defd or key in forward_traits:
            if not item is traitlets.Instance:
                if issubclass(item.__class__, traitlets.TraitType):
                    defaults[key] = item.default_value
            else:
                defaults[key] = traitlets.Undefined

    jd = js_formatter(defaults)
    jd = '\n'.join([" " * 4 + l for l in jd.split('\n')])

    return (name, template.format(class_name=name,
                          base_class=base_class_name,
                          json_defaults=jd))

def js_extract():
    import sys, inspect, json
    clsmembers = inspect.getmembers(sys.modules[__name__], inspect.isclass)
    # print(clsmembers)
    res = ""
    def modulify(d, suffix=''):
        js = {key + suffix: key + suffix for key in d}
        s = "{\n"
        for key in sorted(d.keys()):
            s += "    {key}: {key},\n".format(key=key + suffix)
        return s + '}\n'

    for_export = {}
    for cls_name, cls in clsmembers:
        if cls.__module__ == __name__:
            cls_name, text = js_extract_cls(cls)
            for_export[cls_name] = text

    for key in for_export:
        res += for_export[key] + "\n"

    export_template = """
module.exports = {exports_json}
    """
    return res + export_template.format(exports_json=modulify(for_export, 'Defaults'))

if __name__ == "__main__":
    import os
    filedir = os.path.dirname(os.path.realpath(__file__))

    defaults_js = js_extract()
    with open(os.path.join(filedir, "../js/lib/defaults.js"), "w") as fo:
        fo.write(defaults_js)