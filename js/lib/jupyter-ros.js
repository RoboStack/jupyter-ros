///////////////////////////////////////////////////////////////////////////////
// Copyright (c) Wolf Vollprecht, QuantStack                                 //
//                                                                           //
// Distributed under the terms of the BSD 3-Clause License.                  //
//                                                                           //
// The full license is in the file LICENSE, distributed with this software.  //
///////////////////////////////////////////////////////////////////////////////

window.ws = window.WebSocket;
var widgets = require('@jupyter-widgets/base');
var _ = require('lodash');

var ROSLIB = require('roslib');
var ROS3D = require('ros3d');
var THREE = require('three');

var defaults = require('./defaults.js')

var widget_defaults = widgets.WidgetModel.prototype.defaults;
var domwidget_defaults = widgets.DOMWidgetModel.prototype.defaults;

var default_serializers = function(names) {
    names = names || ['ros', 'tf_client']

    var named_serializers = {}
    for (let idx in names)
    {
        named_serializers[names[idx]] = { deserialize: widgets.unpack_models }
    }
    return {serializers: _.extend(named_serializers, widgets.WidgetModel.serializers)};
}

var fixup_url = function(url) {
    url = url
        .replace("{hostname}", window.location.hostname)
        .replace("{port}", window.location.port);
    return url;
}

var ROSConnectionModel = widgets.WidgetModel.extend({
    defaults: _.extend(widget_defaults(), defaults.ROSConnectionModelDefaults),
    initialize: function() {
        ROSConnectionModel.__super__.initialize.apply(this, arguments);
        this.connection = new ROSLIB.Ros({
          url: fixup_url(this.get('url'))
        });
    },
    get_connection: function() {
        return this.connection;
    }
});

var TFClientModel = widgets.WidgetModel.extend({
    defaults: _.extend(widget_defaults(), defaults.TFClientModelDefaults),
    initialize: function() {
        TFClientModel.__super__.initialize.apply(this, arguments);
        this.client = new ROSLIB.TFClient({
          ros: this.get('ros').get_connection(),
          angularThres: this.get('angular_treshold'),
          transThres: this.get('translational_treshold'),
          rate: this.get('rate'),
          fixedFrame: this.get('fixed_frame')
        });
    },
    get_client: function() {
        return this.client;
    },
}, default_serializers(['ros'])
);

var PointCloudModel = widgets.WidgetModel.extend({
    defaults: _.extend(widgets.WidgetModel.prototype.defaults(), defaults.PointCloudModelDefaults),
}, default_serializers());

var OccupancyGridModel = widgets.WidgetModel.extend({
    defaults: _.extend(widget_defaults(), defaults.OccupancyGridDefaults),
}, default_serializers());

var SceneNodeModel = widgets.WidgetModel.extend({
    defaults: _.extend(widgets.WidgetModel.prototype.defaults(), defaults.SceneNodeModelDefaults),
}, default_serializers(['tf_client', 'object']));

var OccupancyGridView = widgets.WidgetView.extend({
    initialize: function(args) {
        OccupancyGridView.__super__.initialize.apply(this, arguments);
        this.viewer = this.options.viewer;
    },
    render: function() {
        this.three_color = new THREE.Color(this.model.get('color'));
        this.rgb_color = {r: this.three_color.r * 255, 
                          g: this.three_color.g * 255,
                          b: this.three_color.b * 255};

        this.view = new ROS3D.OccupancyGridClient({
            rootObject: this.viewer.scene,
            ros: this.model.get('ros').get_connection(),
            tfClient: this.model.get('tf_client').get_client(),
            topic: this.model.get('topic'),
            color: this.rgb_color,
            compression: this.model.get('compression'),
            continuous: this.model.get('continuous'),
            opacity: this.model.get('opacity')
        });
    }
});

var InteractiveMarkerModel = widgets.WidgetModel.extend({
    defaults: _.extend(widget_defaults(), defaults.InteractiveMarkerDefaults),
},
default_serializers()
);

var InteractiveMarkerView = widgets.WidgetView.extend({
    initialize: function(args) {
        InteractiveMarkerView.__super__.initialize.apply(this, arguments);
        this.viewer = this.options.viewer;
    },
    render: function() {
        this.view = new ROS3D.InteractiveMarkerClient({
            ros: this.model.get('ros').get_connection(),
            tfClient: this.model.get('tf_client').get_client(),
            topic: this.model.get('topic'),
            menuFontSize: this.model.get('menu_font_size'),
            camera: this.viewer.camera,
            rootObject: this.viewer.selectableObjects,
        });
    }
});

var PoseArrayModel = widgets.WidgetModel.extend({
    defaults: _.extend(widget_defaults(), defaults.PoseArrayDefaults),
},
default_serializers()
);

var PoseArrayView = widgets.WidgetView.extend({
    initialize: function(args) {
        PoseArrayView.__super__.initialize.apply(this, arguments);
        this.viewer = this.options.viewer;
        this.model.on("change:length", () => { this.view.length = this.model.get('length') });
        this.model.on("change:color", () => { this.view.color = this.model.get('color'); });
    },
    render: function() {
        this.view = new ROS3D.PoseArray({
            ros: this.model.get('ros').get_connection(),
            tfClient: this.model.get('tf_client').get_client(),
            topic: this.model.get('topic'),
            color: this.model.get('color'),
            length: this.model.get('length'),
            rootObject: this.viewer.scene,
        });
    }
});

var PoseModel = widgets.WidgetModel.extend({
    defaults: _.extend(widget_defaults(), defaults.PoseModelDefaults),
},
default_serializers()
);

var PoseView = widgets.WidgetView.extend({
    initialize: function(args) {
        PoseView.__super__.initialize.apply(this, arguments);
        this.viewer = this.options.viewer;
        this.model.on("change:length", () => { this.view.length = this.model.get('length') });
        this.model.on("change:color", () => { this.view.color = this.model.get('color'); });
    },
    render: function() {
        this.view = new ROS3D.Pose({
            ros: this.model.get('ros').get_connection(),
            tfClient: this.model.get('tf_client').get_client(),
            topic: this.model.get('topic'),
            color: this.model.get('color'),
            length: this.model.get('length'),
            rootObject: this.viewer.scene,
        });
    }
});

var PathModel = widgets.WidgetModel.extend({
    defaults: _.extend(widget_defaults(), defaults.PathModelDefaults),
},
default_serializers()
);

var PathView = widgets.WidgetView.extend({
    initialize: function(args) {
        PathView.__super__.initialize.apply(this, arguments);
        this.viewer = this.options.viewer;
        this.model.on("change:color", () => { this.view.color = this.model.get('color'); });
    },
    render: function() {
        this.view = new ROS3D.Path({
            ros: this.model.get('ros').get_connection(),
            tfClient: this.model.get('tf_client').get_client(),
            topic: this.model.get('topic'),
            color: this.model.get('color'),
            rootObject: this.viewer.scene,
        });
    }
});

var PolygonModel = widgets.WidgetModel.extend({
    defaults: _.extend(widget_defaults(), defaults.PolygonModelDefaults),
},
default_serializers()
);

var PolygonView = widgets.WidgetView.extend({
    initialize: function(args) {
        PolygonView.__super__.initialize.apply(this, arguments);
        this.viewer = this.options.viewer;
        this.model.on("change:color", () => { this.view.color = this.model.get('color'); });
    },
    render: function() {
        this.view = new ROS3D.Polygon({
            ros: this.model.get('ros').get_connection(),
            tfClient: this.model.get('tf_client').get_client(),
            topic: this.model.get('topic'),
            color: this.model.get('color'),
            rootObject: this.viewer.scene,
        });
    }
});

var LaserScanModel = widgets.WidgetModel.extend({
    defaults: _.extend(widget_defaults(), defaults.LaserScanModelDefaults),
},
default_serializers()
);

var toMaterial = function(pointSize, color) {
    return { size: pointSize, color: new THREE.Color(color) };
};

var LaserScanView = widgets.WidgetView.extend({
    initialize: function(args) {
        LaserScanView.__super__.initialize.apply(this, arguments);
        this.viewer = this.options.viewer;
        this.model.on("change", this.trigger_rerender, this);
    },
    render: function() {
        if (this.model.get('color_map')) {
            this.color_map_function = eval(this.model.get('color_map'))
        }
        this.view = new ROS3D.LaserScan({
            ros: this.model.get('ros').get_connection(),
            tfClient: this.model.get('tf_client').get_client(),
            topic: this.model.get('topic'),
            rootObject: this.viewer.scene,
            messageRatio: this.model.get('message_ratio'),
            max_pts: this.model.get('max_points'),
            pointRatio: this.model.get('point_ratio'),
            material: toMaterial(this.model.get('point_size'), this.model.get('static_color')),
            colorsrc: this.model.get('color_source'),
            colormap: this.color_map_function || undefined
        });
    },
    remove: function() {
        this.viewer.scene.remove(this.view.points.sn);
    },
    trigger_rerender: function() {
        this.remove();
        this.render();
    }
});

var MarkerModel = widgets.WidgetModel.extend({
    defaults: _.extend(widget_defaults(), defaults.MarkerDefaults),
},
default_serializers()
);

var MarkerView = widgets.WidgetView.extend({
    initialize: function(args) {
        MarkerView.__super__.initialize.apply(this, arguments);
        this.viewer = this.options.viewer;
    },
    render: function() {
        this.view = new ROS3D.MarkerClient({
            ros: this.model.get('ros').get_connection(),
            tfClient: this.model.get('tf_client').get_client(),
            topic: this.model.get('topic'),
            path: this.model.get('path'),
            lifetime: this.model.get('lifetime'),
            rootObject: this.viewer.scene,
        });
    }
});

var MarkerArrayModel = widgets.WidgetModel.extend({
    defaults: _.extend(widget_defaults(), defaults.MarkerArrayDefaults),
},
default_serializers()
);

var MarkerArrayView = widgets.WidgetView.extend({
    initialize: function(args) {
        MarkerArrayView.__super__.initialize.apply(this, arguments);
        this.viewer = this.options.viewer;
        // this.model.on("change", this.trigger_rerender, this);
    },
    render: function() {
        this.view = new ROS3D.MarkerArrayClient({
            ros: this.model.get('ros').get_connection(),
            tfClient: this.model.get('tf_client').get_client(),
            topic: this.model.get('topic'),
            path: this.model.get('path'),
            rootObject: this.viewer.scene,
        });
    }
});

var GridModel = widgets.WidgetModel.extend({
    defaults: _.extend(widgets.WidgetModel.prototype.defaults(), defaults.GridModelDefaults)
});

var URDFModel = widgets.WidgetModel.extend({
    defaults: _.extend(widgets.WidgetModel.prototype.defaults(), defaults.URDFModelDefaults),
},
default_serializers()
);

var URDFView = widgets.WidgetView.extend({
    initialize: function(parms) {
        URDFView.__super__.initialize.apply(this, arguments);
        this.viewer = this.options.viewer;
        // this.model.on('change', this.trigger_rerender, this);
    },
    render: function() {
        this.view = new ROS3D.UrdfClient({
            ros: this.model.get('ros').get_connection(),
            tfClient: this.model.get('tf_client').get_client(),
            rootObject: this.viewer.scene,
            path: fixup_url(this.model.get('url'))
        });
    },
    trigger_rerender: function() {
        this.remove();
        this.render();
    },
    remove: function() {
        this.viewer.scene.remove(this.view);
    }
});

var ViewerModel = widgets.DOMWidgetModel.extend({
    defaults: _.extend(widgets.DOMWidgetModel.prototype.defaults(), defaults.ViewerModelDefaults)
},
default_serializers(['objects', 'layout'])
);

var PointCloudView = widgets.WidgetView.extend({
    initialize: function(parms) {
        PointCloudView.__super__.initialize.apply(this, arguments);
        this.viewer = this.options.viewer;
    },
    render: function() {
        // if (this.model.get('color_map')) {
        //     this.color_map_function = new Function(this.model.get('color_source'), 'THREE', this.model.get('color_map'));
        //     this.color_map_apply = function() { args = arguments; args.push(THREE); this.color_map_apply.apply(null, args); }
        //     material = { size: this.model.get('point_size'), sizeAttenuation: false };
        // }
        // else {
        //     material = toMaterial(this.model.get('point_size'), this.model.get('static_color'));
        // }

        this.view = new ROS3D.PointCloud2({
            ros: this.model.get('ros').get_connection(),
            tfClient: this.model.get('tf_client').get_client(),
            topic: this.model.get('topic'),
            rootObject: this.viewer.scene,
            messageRatio: this.model.get('message_ratio'),
            max_pts: this.model.get('max_points'),
            pointRatio: this.model.get('point_ratio'),
            material: toMaterial(this.model.get('point_size'), this.model.get('static_color'))
            // colorsrc: this.model.get('color_source'),
            // colormap: this.color_map_apply || undefined
        });
    },
    // remove: function() {
    //     this.viewer.scene.remove(this.view.points.sn);
    // }
});

var DepthCloudModel = widgets.WidgetModel.extend({
    defaults: _.extend(widget_defaults(), defaults.DepthCloudModelDefaults),
    initialize: function() {
        DepthCloudModel.__super__.initialize.apply(this, arguments);
        this.depth_cloud = new ROS3D.DepthCloud({
            url: this.get('url'),
            f: this.get('f')
        });
        this.depth_cloud.startStream();
    },
    get_threejs_obj: function() {
        return this.depth_cloud;
    },
});

var SceneNodeView = widgets.WidgetView.extend({
    initialize: function(parms) {
        SceneNodeView.__super__.initialize.apply(this, arguments);
        this.viewer = this.options.viewer;
        this.model.on('change', this.trigger_rerender, this);
    },
    render: function() {
        this.view = new ROS3D.SceneNode({
            frameID: this.model.get('frame_id'),
            tfClient: this.model.get('tf_client').get_client(),
            object: this.model.get('object').get_threejs_obj()
        });
        this.viewer.scene.add(this.view);
    },
    trigger_rerender: function() {
        this.remove();
        this.render();
    },
    remove: function() {
        this.viewer.scene.remove(this.view);
    }
});

var GridView = widgets.WidgetView.extend({
    initialize: function(parms) {
        GridView.__super__.initialize.apply(this, arguments);
        this.viewer = this.options.viewer;
        this.model.on('change', this.trigger_rerender, this);
    },
    render: function() {
        this.grid_view = new ROS3D.Grid({
            color: this.model.get('color'),
            cellSize: this.model.get('cell_size'),
            num_cells: this.model.get('num_cells')
        });
        this.viewer.addObject(this.grid_view);
    },
    trigger_rerender: function() {
        this.remove();
        this.render();
    },
    remove: function() {
        this.viewer.scene.remove(this.grid_view);
    }
});

// Custom View. Renders the widget model.
var ViewerView = widgets.DOMWidgetView.extend({
    render: function() {
        var unique_id = (new Date).getTime().toString() + Math.floor(Math.random() * Math.floor(9999)).toString();
        this.el.id = unique_id + 'ROS_VIEWER';

        this.model.on("change:background_color", this.background_color_change, this);
        this.model.on("change:alpha", this.background_color_change, this);

        this.displayed.then(() => {
           this.init_viewer();
        });
    },

    add_object: function (model) {
        return this.create_child_view(model, {
            viewer: this.viewer,
        });
    },
    remove_object: function (view) {
        view.remove();
    },
    objects_changed: function(msg) {
        this.object_views.update(msg.changed.objects);
    },
    trigger_resize: function() {
        this.viewer.resize(this.el.clientWidth, this.el.clientHeight)
    },
    processPhosphorMessage: function(msg) {
        ViewerView.__super__.processPhosphorMessage.apply(this, arguments);
        if (msg.type == 'resize') {
            this.trigger_resize();
        }
    },
    background_color_change: function() {
        this.viewer.renderer.setClearColor(this.model.get('background_color'), this.model.get('alpha'))
    },
    init_viewer: function() {
        var height = this.model.get('layout').get('height');
        if (height === null || height == 'auto') {
            height = 400;
        }
        else {
            height = parseInt(height)
        }

        this.viewer = new ROS3D.Viewer({
            divID: this.el.id,
            width: this.el.clientWidth,
            height: height,
            antialias: this.model.get('antialias'),
            background: this.model.get('background_color')
        });

        window.addEventListener("resize", () => {
           this.viewer && this.trigger_resize();
        });

        this.model.on("change:objects", this.objects_changed, this);
        this.object_views = new widgets.ViewList(this.add_object, this.remove_object, this);
        this.object_views.update(this.model.get('objects'));
     }
});


module.exports = {
    ROSConnectionModel: ROSConnectionModel,
    TFClientModel: TFClientModel,
    PointCloudModel: PointCloudModel,
    PointCloudView: PointCloudView,
    MarkerModel: MarkerModel,
    MarkerView: MarkerView,
    MarkerArrayModel: MarkerArrayModel,
    MarkerArrayView: MarkerArrayView,
    OccupancyGridModel: OccupancyGridModel,
    OccupancyGridView: OccupancyGridView,
    InteractiveMarkerModel: InteractiveMarkerModel,
    InteractiveMarkerView: InteractiveMarkerView,
    GridModel: GridModel,
    GridView: GridView,
    URDFModel: URDFModel,
    URDFView: URDFView,
    PoseArrayModel: PoseArrayModel,
    PoseArrayView: PoseArrayView,
    PoseModel: PoseModel,
    PoseView: PoseView,
    PathModel: PathModel,
    PathView: PathView,
    PolygonModel: PolygonModel,
    PolygonView: PolygonView,
    LaserScanModel: LaserScanModel,
    LaserScanView: LaserScanView,
    SceneNodeModel: SceneNodeModel,
    SceneNodeView: SceneNodeView,
    DepthCloudModel: DepthCloudModel,
    ViewerModel: ViewerModel,
    ViewerView: ViewerView
};
