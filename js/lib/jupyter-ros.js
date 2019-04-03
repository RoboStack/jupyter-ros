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

var ROSConnectionModel = widgets.WidgetModel.extend({
    defaults: _.extend(widget_defaults(), defaults.ROSConnectionModelDefaults),
    initialize: function() {
        ROSConnectionModel.__super__.initialize.apply(this, arguments);
        this.connection = new ROSLIB.Ros({
          url : this.get('url') + ':' + this.get('port')
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
    defaults: _.extend(widgets.WidgetModel.prototype.defaults(), defaults.PointCloudModelDefaults,
    {
        material: { size: 0.05, color: 0xff00ff }
    }),
}, default_serializers());

var OccupancyGridModel = widgets.WidgetModel.extend({
    defaults: _.extend(widget_defaults(), defaults.OccupancyGridDefaults),
},
default_serializers()
);

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
    defaults: _.extend(widget_defaults(), defaults.LaserScanModelDefaults, 
        {
            material: { size: 0.05, color: 0xff0000 }
        }),
},
default_serializers()
);

var LaserScanView = widgets.WidgetView.extend({
    initialize: function(args) {
        LaserScanView.__super__.initialize.apply(this, arguments);
        this.viewer = this.options.viewer;
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
            material: this.model.get('material'),
            colorsrc: this.model.get('color_source'),
            colormap: this.color_map_function || undefined
        });
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

var GridModel = widgets.DOMWidgetModel.extend({
    defaults: _.extend(widgets.DOMWidgetModel.prototype.defaults(), defaults.GridModelDefaults)
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
            path: this.model.get('path')
            // colorsrc: 'z',
            // colormap: function(z) { z=z+2; return new THREE.Color(z,0,1-z); }
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
default_serializers(['objects'])
);

var PointCloudView = widgets.WidgetView.extend({
    initialize: function(parms) {
        PointCloud2View.__super__.initialize.apply(this, arguments);
        this.viewer = this.options.viewer;
        this.model.on('change', this.trigger_rerender, this);
    },
    render: function() {
        this.view = new ROS3D.PointCloud2({
            ros: this.model.get('ros').get_connection(),
            tfClient: this.model.get('tf_client').get_client(),
            topic: this.model.get('topic'),
            rootObject: this.viewer.scene,
            messageRatio: this.model.get('message_ratio'),
            max_pts: this.model.get('max_points'),
            pointRatio: this.model.get('point_ratio'),
            material: this.model.get('material')
            // colorsrc: 'z',
            // colormap: function(z) { z=z+2; return new THREE.Color(z,0,1-z); }
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

var GridView = widgets.WidgetView.extend({
    initialize: function(parms) {
        GridView.__super__.initialize.apply(this, arguments);
        console.log(this.options)
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
        this.el.style.height = this.model.get('height');

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
        height = this.model.get('height')
        if (height == 'auto') {
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
        this.model.on("change:height", () => {
           this.el.style.height = this.model.get('height');
           this.trigger_resize();
        });
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
    ViewerModel: ViewerModel,
    ViewerView: ViewerView
};
