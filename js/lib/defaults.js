
var TFClientModelDefaults =     {
        _model_module: "jupyter-ros",
        _model_module_version: "^0.1.0",
        _model_name: "TFClientModel",
        angular_treshold: 0.01,
        fixed_frame: "",
        rate: 10.0,
        ros: undefined,
        translational_treshold: 0.01,
    }
    
    

var URDFModelDefaults =     {
        _model_module: "jupyter-ros",
        _model_module_version: "^0.1.0",
        _model_name: "URDFModel",
        _view_module: "jupyter-ros",
        _view_module_version: "^0.1.0",
        _view_name: "URDFView",
        path: "http://localhost:3000",
        ros: undefined,
        tf_client: undefined,
    }
    
    

var LaserScanModelDefaults =     {
        _model_module: "jupyter-ros",
        _model_module_version: "^0.1.0",
        _model_name: "LaserScanModel",
        _view_module: "jupyter-ros",
        _view_module_version: "^0.1.0",
        _view_name: "LaserScanView",
        ros: undefined,
        tf_client: undefined,
        topic: "/path",
    }
    
    

var InteractiveMarkerClientModelDefaults =     {
        _model_module: "jupyter-ros",
        _model_module_version: "^0.1.0",
        _model_name: "InteractiveMarkerClientModel",
        _view_module: "jupyter-ros",
        _view_module_version: "^0.1.0",
        _view_name: "InteractiveMarkerClientView",
        menu_font_size: "0.8em",
        ros: undefined,
        tf_client: undefined,
        topic: "/basic_controls",
    }
    
    

var PoseModelDefaults =     {
        _model_module: "jupyter-ros",
        _model_module_version: "^0.1.0",
        _model_name: "PoseModel",
        _view_module: "jupyter-ros",
        _view_module_version: "^0.1.0",
        _view_name: "PoseView",
        color: "#CC00FF",
        length: 1.0,
        ros: undefined,
        tf_client: undefined,
        topic: "/particle_cloud",
    }
    
    

var PathModelDefaults =     {
        _model_module: "jupyter-ros",
        _model_module_version: "^0.1.0",
        _model_name: "PathModel",
        _view_module: "jupyter-ros",
        _view_module_version: "^0.1.0",
        _view_name: "PathView",
        color: "#CC00FF",
        ros: undefined,
        tf_client: undefined,
        topic: "/path",
    }
    
    

var ROSConnectionModelDefaults =     {
        _model_module: "jupyter-ros",
        _model_module_version: "^0.1.0",
        _model_name: "ROSConnectionModel",
        port: "9090",
        url: "ws://localhost",
    }
    
    

var PolygonModelDefaults =     {
        _model_module: "jupyter-ros",
        _model_module_version: "^0.1.0",
        _model_name: "PolygonModel",
        _view_module: "jupyter-ros",
        _view_module_version: "^0.1.0",
        _view_name: "PolygonView",
        color: "#CC00FF",
        ros: undefined,
        tf_client: undefined,
        topic: "/path",
    }
    
    

var PointCloud2ModelDefaults =     {
        _model_module: "jupyter-ros",
        _model_module_version: "^0.1.0",
        _model_name: "PointCloud2Model",
        _view_module: "jupyter-ros",
        _view_module_version: "^0.1.0",
        _view_name: "PointCloud2View",
        ros: undefined,
        tf_client: undefined,
        topic: "",
    }
    
    

var MarkerClientModelDefaults =     {
        _model_module: "jupyter-ros",
        _model_module_version: "^0.1.0",
        _model_name: "MarkerClientModel",
        _view_module: "jupyter-ros",
        _view_module_version: "^0.1.0",
        _view_name: "MarkerClientView",
        lifetime: 0.0,
        path: "/",
        ros: undefined,
        tf_client: undefined,
        topic: "/visualization_marker",
    }
    
    

var MarkerArrayClientModelDefaults =     {
        _model_module: "jupyter-ros",
        _model_module_version: "^0.1.0",
        _model_name: "MarkerArrayClientModel",
        _view_module: "jupyter-ros",
        _view_module_version: "^0.1.0",
        _view_name: "MarkerArrayClientView",
        path: "/",
        ros: undefined,
        tf_client: undefined,
        topic: "/basic_controls",
    }
    
    

var GridModelDefaults =     {
        _model_module: "jupyter-ros",
        _model_module_version: "^0.1.0",
        _model_name: "GridModel",
        _view_module: "jupyter-ros",
        _view_module_version: "^0.1.0",
        _view_name: "GridView",
        cell_size: 0.5,
        color: "#0181c4",
        num_cells: 20,
    }
    
    

var OccupancyGridClientModelDefaults =     {
        _model_module: "jupyter-ros",
        _model_module_version: "^0.1.0",
        _model_name: "OccupancyGridClientModel",
        _view_module: "jupyter-ros",
        _view_module_version: "^0.1.0",
        _view_name: "OccupancyGridClientView",
        color: "#FFFFFF",
        compression: "cbor",
        continuous: false,
        opacity: 1.0,
        ros: undefined,
        tf_client: undefined,
        topic: "/map",
    }
    
    

var PoseArrayModelDefaults =     {
        _model_module: "jupyter-ros",
        _model_module_version: "^0.1.0",
        _model_name: "PoseArrayModel",
        _view_module: "jupyter-ros",
        _view_module_version: "^0.1.0",
        _view_name: "PoseArrayView",
        color: "#CC00FF",
        length: 1.0,
        ros: undefined,
        tf_client: undefined,
        topic: "/particle_cloud",
    }
    
    

var ViewerModelDefaults =     {
        _model_module: "jupyter-ros",
        _model_module_version: "^0.1.0",
        _model_name: "ViewerModel",
        _view_module: "jupyter-ros",
        _view_module_version: "^0.1.0",
        _view_name: "ViewerView",
        alpha: true,
        background_color: "#FFFFFF",
        height: "auto",
        objects: undefined,
    }
    
    

module.exports = {
    GridModelDefaults: GridModelDefaults,
    InteractiveMarkerClientModelDefaults: InteractiveMarkerClientModelDefaults,
    LaserScanModelDefaults: LaserScanModelDefaults,
    MarkerArrayClientModelDefaults: MarkerArrayClientModelDefaults,
    MarkerClientModelDefaults: MarkerClientModelDefaults,
    OccupancyGridClientModelDefaults: OccupancyGridClientModelDefaults,
    PathModelDefaults: PathModelDefaults,
    PointCloud2ModelDefaults: PointCloud2ModelDefaults,
    PolygonModelDefaults: PolygonModelDefaults,
    PoseArrayModelDefaults: PoseArrayModelDefaults,
    PoseModelDefaults: PoseModelDefaults,
    ROSConnectionModelDefaults: ROSConnectionModelDefaults,
    TFClientModelDefaults: TFClientModelDefaults,
    URDFModelDefaults: URDFModelDefaults,
    ViewerModelDefaults: ViewerModelDefaults,
}

    
