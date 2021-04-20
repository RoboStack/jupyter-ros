///////////////////////////////////////////////////////////////////////////////
// Copyright (c) Wolf Vollprecht, QuantStack                                 //
//                                                                           //
// Distributed under the terms of the BSD 3-Clause License.                  //
//                                                                           //
// The full license is in the file LICENSE, distributed with this software.  //
///////////////////////////////////////////////////////////////////////////////

var index_module = require('./index.js');
var base = require('@jupyter-widgets/base');

module.exports = {
  id: '@robostack/jupyter-ros',
  requires: [base.IJupyterWidgetRegistry],
  activate: function(app, widgets) {
    widgets.registerWidget({
        name: '@robostack/jupyter-ros',
        version: index_module.version,
        exports: index_module
    });
  },
  autoStart: true
};

