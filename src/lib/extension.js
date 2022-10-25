///////////////////////////////////////////////////////////////////////////////
// Copyright (c) Wolf Vollprecht, QuantStack                                 //
//                                                                           //
// Distributed under the terms of the BSD 3-Clause License.                  //
//                                                                           //
// The full license is in the file LICENSE, distributed with this software.  //
///////////////////////////////////////////////////////////////////////////////

// This file contains the javascript that is run when the notebook is loaded.
// It contains some requirejs configuration and the `load_ipython_extension`
// which is required for any notebook extension.
//
// Some static assets may be required by the custom widget javascript. The base
// url for the notebook is not known at build time and is therefore computed
// dynamically.
__webpack_public_path__ = document.querySelector('body').getAttribute('data-base-url') + 'nbextensions/@robostack/jupyter-ros';


// Configure requirejs
if (window.require) {
    window.require.config({
        map: {
            "*" : {
                "@robostack/jupyter-ros": "nbextensions/@robostack/jupyter-ros/index",
            }
        }
    });
}

// Export the required load_ipython_extension
module.exports = {
    load_ipython_extension: function() {}
};
