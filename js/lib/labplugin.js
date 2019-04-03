var index_module = require('./index.js');
var base = require('@jupyter-widgets/base');

module.exports = {
  id: 'jupyter-ros',
  requires: [base.IJupyterWidgetRegistry],
  activate: function(app, widgets) {
      widgets.registerWidget({
          name: 'jupyter-ros',
          version: index_module.version,
          exports: index_module
      });
  },
  autoStart: true
};

