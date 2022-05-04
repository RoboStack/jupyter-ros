import sys
import inspect
import json
from typing import Tuple
import ipywidgets as widgets
from traitlets import Instance
from traitlets import TraitType
from traitlets import Undefined
from traitlets import Unicode
from traitlets import Float
from traitlets import Int
from traitlets import Bool
from traitlets import List
from traitlets import utils as tut
from ._version import version_info


def _quick_widget(package_name, version, has_view=True):
    def quick_widget_decorator(cls):
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

JS_VERSION = '^' + '.'.join([str(v) for v in version_info[:3]])

REGISTER = _quick_widget('jupyter-ros', JS_VERSION)
REGISTER_NOVIEW = _quick_widget('jupyter-ros', JS_VERSION, False)
SYNC_WIDGET = {'sync': True}
SYNC_WIDGET.update(widgets.widget_serialization)


@REGISTER_NOVIEW
class ROSConnection(widgets.Widget):
    """ ROSConnection """
    url = Unicode("ws://localhost:9090").tag(sync=True)


@REGISTER_NOVIEW
class TFClient(widgets.Widget):
    """ TFClient """
    ros = Instance(ROSConnection).tag(**SYNC_WIDGET)
    angular_treshold = Float(0.01).tag(sync=True)
    translational_treshold = Float(0.01).tag(sync=True)
    rate = Float(10.0).tag(sync=True)
    fixed_frame = Unicode('').tag(sync=True)


@REGISTER
class URDFModel(widgets.Widget):
    """ URDFModel """
    ros = Instance(ROSConnection).tag(**SYNC_WIDGET)
    tf_client = Instance(TFClient).tag(**SYNC_WIDGET)
    url = Unicode("http://localhost:3000").tag(sync=True)


@REGISTER
class GridModel(widgets.Widget):
    """ GridModel """
    cell_size = Float(0.5).tag(sync=True)
    color = Unicode("#0181c4").tag(sync=True)
    num_cells = Int(20).tag(sync=True)


@REGISTER
class OccupancyGrid(widgets.Widget):
    """ OccupancyGrid """
    ros = Instance(ROSConnection).tag(**SYNC_WIDGET)
    tf_client = Instance(TFClient).tag(**SYNC_WIDGET)
    topic = Unicode("/map").tag(sync=True)
    continuous = Bool(False).tag(sync=True)
    compression = Unicode('cbor').tag(sync=True)
    color = Unicode('#FFFFFF').tag(sync=True)
    opacity = Float(1.0).tag(sync=True)
    # rootObject =
    # offsetPose =


@REGISTER
class InteractiveMarker(widgets.Widget):
    """ InteractiveMarker """
    ros = Instance(ROSConnection).tag(**SYNC_WIDGET)
    tf_client = Instance(TFClient).tag(**SYNC_WIDGET)
    topic = Unicode('/basic_controls').tag(sync=True)
    menu_font_size = Unicode('0.8em').tag(sync=True)
    # camera =
    # rootObject


@REGISTER
class Marker(widgets.Widget):
    """ Marker """
    ros = Instance(ROSConnection).tag(**SYNC_WIDGET)
    tf_client = Instance(TFClient).tag(**SYNC_WIDGET)
    topic = Unicode('/visualization_marker').tag(sync=True)
    path = Unicode('/').tag(sync=True)
    lifetime = Float(0.0).tag(sync=True)


@REGISTER
class PoseArray(widgets.Widget):
    """ PoseArray """
    ros = Instance(ROSConnection).tag(**SYNC_WIDGET)
    tf_client = Instance(TFClient).tag(**SYNC_WIDGET)
    topic = Unicode('/particle_cloud').tag(sync=True)
    color = Unicode('#CC00FF').tag(sync=True)
    length = Float(1.0).tag(sync=True)


@REGISTER
class Pose(widgets.Widget):
    """ Pose """
    ros = Instance(ROSConnection).tag(**SYNC_WIDGET)
    tf_client = Instance(TFClient).tag(**SYNC_WIDGET)
    topic = Unicode('/particle_cloud').tag(sync=True)
    color = Unicode('#CC00FF').tag(sync=True)
    length = Float(1.0).tag(sync=True)


@REGISTER
class Polygon(widgets.Widget):
    """ Polygon """
    ros = Instance(ROSConnection).tag(**SYNC_WIDGET)
    tf_client = Instance(TFClient).tag(**SYNC_WIDGET)
    topic = Unicode('/path').tag(sync=True)
    color = Unicode('#CC00FF').tag(sync=True)


@REGISTER
class Path(widgets.Widget):
    """ Path """
    ros = Instance(ROSConnection).tag(**SYNC_WIDGET)
    tf_client = Instance(TFClient).tag(**SYNC_WIDGET)
    topic = Unicode('/path').tag(sync=True)
    color = Unicode('#CC00FF').tag(sync=True)


@REGISTER
class LaserScan(widgets.Widget):
    """ LaserScan """
    ros = Instance(ROSConnection).tag(**SYNC_WIDGET)
    tf_client = Instance(TFClient).tag(**SYNC_WIDGET)
    topic = Unicode('/path').tag(sync=True)
    point_ratio = Float(1.0).tag(sync=True)
    message_ratio = Float(1.0).tag(sync=True)
    max_points = Int(200000).tag(sync=True)
    color_source = Unicode('intensities').tag(sync=True)
    color_map = Unicode('').tag(sync=True)
    point_size = Float(0.05).tag(sync=True)
    static_color = Unicode("#FF0000").tag(sync=True)


@REGISTER
class MarkerArrayClient(widgets.Widget):
    """ MarkerArrayClient """
    ros = Instance(ROSConnection).tag(**SYNC_WIDGET)
    tf_client = Instance(TFClient).tag(**SYNC_WIDGET)
    topic = Unicode('/basic_controls').tag(sync=True)
    path = Unicode('/').tag(sync=True)


@REGISTER
class PointCloud(widgets.Widget):
    """ PointCloud """
    ros = Instance(ROSConnection).tag(**SYNC_WIDGET)
    tf_client = Instance(TFClient).tag(**SYNC_WIDGET)
    topic = Unicode('').tag(sync=True)
    message_ratio = Float(2.0).tag(sync=True)
    point_ratio = Float(3.0).tag(sync=True)
    max_points = Int(200000).tag(sync=True)
    point_size = Float(0.05).tag(sync=True)
    static_color = Unicode("#FF0000").tag(sync=True)
    # color_map = Unicode('').tag(sync=True)
    # color_source = Unicode('').tag(sync=True)


@REGISTER
class Viewer(widgets.DOMWidget):
    """ Viewer """
    background_color = Unicode('#FFFFFF').tag(sync=True)
    alpha = Bool(True).tag(sync=True)
    height = Unicode('100%').tag(sync=True)
    objects = List(Instance(widgets.Widget)).tag(**SYNC_WIDGET)


@REGISTER_NOVIEW
class DepthCloud(widgets.Widget):
    """ DepthCloud """
    url = Unicode('').tag(sync=True)
    f = Float(525.0).tag(sync=True)


@REGISTER
class SceneNode(widgets.Widget):
    """ SceneNode """
    frame_id = Unicode('/base_link').tag(sync=True)
    tf_client = Instance(TFClient).tag(**SYNC_WIDGET)
    object = Instance(DepthCloud).tag(**SYNC_WIDGET)


def js_formatter(d_in: dict) -> str:
    def remove_undefined(d: dict) -> None:
        for key in d:
            if isinstance(d[key], tut.sentinel.Sentinel):
                d[key] = '##UNDEFINED##'
            if isinstance(d[key], dict):
                remove_undefined(d[key])

    remove_undefined(d_in)
    ret_s = '{\n'
    for key in sorted(d_in.keys()):
        val = d_in[key] if d_in[key] is not None else "null"
        if isinstance(val, str):
            val = '"' + val + '"'
        if isinstance(val, bool):
            val = 'true' if val else 'false'

        ret_s += '    {}: {},\n'.format(key, val)
    # ret_s = json.dumps(d_in, indent=4)
    ret_s += '}\n'
    ret_s = ret_s.replace('"##UNDEFINED##"', 'null')
    return ret_s


def js_extract_cls(cls) -> Tuple[str, str]:
    template = "\nvar {class_name}Defaults = {json_defaults}\n"

    name = cls.__name__
    if not name.endswith('Model'):
        name += 'Model'

    defd = cls.__base__.class_trait_names()
    base_class_name = cls.__base__.__name__
    defaults = {}
    # trait_values = cls()._trait_values
    trait_values = cls.__dict__
    forward_traits = ['_model_module', '_model_name', '_model_module_version',
                      '_view_name', '_view_module', '_view_module_version']
    for key, item in trait_values.items():
        if key not in defd or key in forward_traits:
            if not item is Instance:
                if issubclass(item.__class__, TraitType):
                    defaults[key] = item.default_value
            else:
                defaults[key] = Undefined

    jd = js_formatter(defaults)
    jd = '\n'.join([" " * 4 + l for l in jd.split('\n')])

    return (name, template.format(class_name=name,
                                  base_class=base_class_name,
                                  json_defaults=jd))


def js_extract() -> None:
    clsmembers = inspect.getmembers(sys.modules[__name__], inspect.isclass)
    # print(clsmembers)
    def modulify(d: dict, suffix: str = '') -> str:
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
        print(for_export[key])

    export_template = "\nmodule.exports = {exports_json}\n"
    print(export_template.format(exports_json=modulify(for_export, 'Defaults')))
