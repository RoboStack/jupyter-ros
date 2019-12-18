///////////////////////////////////////////////////////////////////////////////
// Copyright (c) Wolf Vollprecht, QuantStack                                 //
//                                                                           //
// Distributed under the terms of the BSD 3-Clause License.                  //
//                                                                           //
// The full license is in the file LICENSE, distributed with this software.  //
///////////////////////////////////////////////////////////////////////////////

// Export widget models and views, and the npm package version number.
module.exports = require('./jupyter-ros.js');
module.exports['version'] = require('../package.json').version;
