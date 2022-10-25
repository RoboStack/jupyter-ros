
var DepthCloudModelDefaults =     {
        _model_module: "@robostack/jupyter-ros",
        _model_module_version: "^0.6.1",
        _model_name: "DepthCloudModel",
        f: 526.1,
        url: "",
    }
    
    

var GridModelDefaults =     {
        _model_module: "@robostack/jupyter-ros",
        _model_module_version: "^0.6.1",
        _model_name: "GridModel",
        _view_module: "@robostack/jupyter-ros",
        _view_module_version: "^0.6.1",
        _view_name: "GridView",
        cell_size: 0.5,
        color: "#0181c4",
        num_cells: 20,
    }
    
    

var InteractiveMarkerModelDefaults =     {
        _model_module: "@robostack/jupyter-ros",
        _model_module_version: "^0.6.1",
        _model_name: "InteractiveMarkerModel",
        _view_module: "@robostack/jupyter-ros",
        _view_module_version: "^0.6.1",
        _view_name: "InteractiveMarkerView",
        menu_font_size: "0.8em",
        ros: null,
        tf_client: null,
        topic: "/basic_controls",
    }
    
    

var LaserScanModelDefaults =     {
        _model_module: "@robostack/jupyter-ros",
        _model_module_version: "^0.6.1",
        _model_name: "LaserScanModel",
        _view_module: "@robostack/jupyter-ros",
        _view_module_version: "^0.6.1",
        _view_name: "LaserScanView",
        color_map: "",
        color_source: "intensities",
        max_points: 200000,
        message_ratio: 1.0,
        point_ratio: 1.0,
        point_size: 0.05,
        ros: null,
        static_color: "#FF0000",
        tf_client: null,
        topic: "/scan",
    }
    
    

var MarkerModelDefaults =     {
        _model_module: "@robostack/jupyter-ros",
        _model_module_version: "^0.6.1",
        _model_name: "MarkerModel",
        _view_module: "@robostack/jupyter-ros",
        _view_module_version: "^0.6.1",
        _view_name: "MarkerView",
        lifetime: 0.0,
        path: "/",
        ros: null,
        tf_client: null,
        topic: "/visualization_marker",
    }
    
    

var MarkerArrayClientModelDefaults =     {
        _model_module: "@robostack/jupyter-ros",
        _model_module_version: "^0.6.1",
        _model_name: "MarkerArrayClientModel",
        _view_module: "@robostack/jupyter-ros",
        _view_module_version: "^0.6.1",
        _view_name: "MarkerArrayClientView",
        path: "/",
        ros: null,
        tf_client: null,
        topic: "/marker_array",
    }
    
    

var OccupancyGridModelDefaults =     {
        _model_module: "@robostack/jupyter-ros",
        _model_module_version: "^0.6.1",
        _model_name: "OccupancyGridModel",
        _view_module: "@robostack/jupyter-ros",
        _view_module_version: "^0.6.1",
        _view_name: "OccupancyGridView",
        color: "#FFFFFF",
        compression: "cbor",
        continuous: false,
        opacity: 1.0,
        ros: null,
        tf_client: null,
        topic: "/map",
    }
    
    

var PathModelDefaults =     {
        _model_module: "@robostack/jupyter-ros",
        _model_module_version: "^0.6.1",
        _model_name: "PathModel",
        _view_module: "@robostack/jupyter-ros",
        _view_module_version: "^0.6.1",
        _view_name: "PathView",
        color: "#CC00FF",
        ros: null,
        tf_client: null,
        topic: "/path",
    }
    
    

var PointCloudModelDefaults =     {
        _model_module: "@robostack/jupyter-ros",
        _model_module_version: "^0.6.1",
        _model_name: "PointCloudModel",
        _view_module: "@robostack/jupyter-ros",
        _view_module_version: "^0.6.1",
        _view_name: "PointCloudView",
        max_points: 200000,
        message_ratio: 2.0,
        point_ratio: 3.0,
        point_size: 0.05,
        ros: null,
        static_color: "#FF0000",
        tf_client: null,
        topic: "/point_cloud",
    }
    
    

var PolygonModelDefaults =     {
        _model_module: "@robostack/jupyter-ros",
        _model_module_version: "^0.6.1",
        _model_name: "PolygonModel",
        _view_module: "@robostack/jupyter-ros",
        _view_module_version: "^0.6.1",
        _view_name: "PolygonView",
        color: "#CC00FF",
        ros: null,
        tf_client: null,
        topic: "/polygon",
    }
    
    

var PoseModelDefaults =     {
        _model_module: "@robostack/jupyter-ros",
        _model_module_version: "^0.6.1",
        _model_name: "PoseModel",
        _view_module: "@robostack/jupyter-ros",
        _view_module_version: "^0.6.1",
        _view_name: "PoseView",
        color: "#CC00FF",
        length: 1.0,
        ros: null,
        tf_client: null,
        topic: "/pose",
    }
    
    

var PoseArrayModelDefaults =     {
        _model_module: "@robostack/jupyter-ros",
        _model_module_version: "^0.6.1",
        _model_name: "PoseArrayModel",
        _view_module: "@robostack/jupyter-ros",
        _view_module_version: "^0.6.1",
        _view_name: "PoseArrayView",
        color: "#CC00FF",
        length: 1.0,
        ros: null,
        tf_client: null,
        topic: "/pose_array",
    }
    
    

var ROSConnectionModelDefaults =     {
        _model_module: "@robostack/jupyter-ros",
        _model_module_version: "^0.6.1",
        _model_name: "ROSConnectionModel",
        url: "ws://{hostname}:9090",
    }
    
    

var SceneNodeModelDefaults =     {
        _model_module: "@robostack/jupyter-ros",
        _model_module_version: "^0.6.1",
        _model_name: "SceneNodeModel",
        _view_module: "@robostack/jupyter-ros",
        _view_module_version: "^0.6.1",
        _view_name: "SceneNodeView",
        frame_id: "/base_link",
        object: null,
        tf_client: null,
    }
    
    

var TFClientModelDefaults =     {
        _model_module: "@robostack/jupyter-ros",
        _model_module_version: "^0.6.1",
        _model_name: "TFClientModel",
        angular_treshold: 0.01,
        fixed_frame: "",
        rate: 10.0,
        ros: null,
        translational_treshold: 0.01,
    }
    
    

var URDFModelDefaults =     {
        _model_module: "@robostack/jupyter-ros",
        _model_module_version: "^0.6.1",
        _model_name: "URDFModel",
        _view_module: "@robostack/jupyter-ros",
        _view_module_version: "^0.6.1",
        _view_name: "URDFView",
        ros: null,
        tf_client: null,
        url: "http://{hostname}:3000",
    }
    
    

var ViewerModelDefaults =     {
        _model_module: "@robostack/jupyter-ros",
        _model_module_version: "^0.6.1",
        _model_name: "ViewerModel",
        _view_module: "@robostack/jupyter-ros",
        _view_module_version: "^0.6.1",
        _view_name: "ViewerView",
        alpha: 1.0,
        background_color: "#FFFFFF",
        height: "100%",
        objects: null,
    }
    
    

module.exports = {
    DepthCloudModelDefaults: DepthCloudModelDefaults,
    GridModelDefaults: GridModelDefaults,
    InteractiveMarkerModelDefaults: InteractiveMarkerModelDefaults,
    LaserScanModelDefaults: LaserScanModelDefaults,
    MarkerArrayClientModelDefaults: MarkerArrayClientModelDefaults,
    MarkerModelDefaults: MarkerModelDefaults,
    OccupancyGridModelDefaults: OccupancyGridModelDefaults,
    PathModelDefaults: PathModelDefaults,
    PointCloudModelDefaults: PointCloudModelDefaults,
    PolygonModelDefaults: PolygonModelDefaults,
    PoseArrayModelDefaults: PoseArrayModelDefaults,
    PoseModelDefaults: PoseModelDefaults,
    ROSConnectionModelDefaults: ROSConnectionModelDefaults,
    SceneNodeModelDefaults: SceneNodeModelDefaults,
    TFClientModelDefaults: TFClientModelDefaults,
    URDFModelDefaults: URDFModelDefaults,
    ViewerModelDefaults: ViewerModelDefaults,
}

    