module.exports = [
    {
      id: 'jupyter-ros',
      autoStart: true,
      activate: function (app) {
        console.log(
          'JupyterLab extension jupyter-ros is activated!'
        );
        console.log(app.commands);
      }
    }
  ];
  