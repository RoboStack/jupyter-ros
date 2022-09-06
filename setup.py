import json
import sys
from pathlib import Path

import setuptools

HERE = Path(__file__).parent.resolve()

# The name of the project
name = 'jupyros'
nb_ext_name = '@robostack/jupyter-ros'
lab_ext_name = '@robostack/jupyter-ros'

lab_path = (HERE / name / "labextension")
nb_path = (HERE / name / "nbextension")

# Representative files that should exist after a successful build
ensured_targets = [
    str(lab_path / "package.json"),
    str(lab_path / "static/style.js")
]

data_files_spec = [
    ('share/jupyter/nbextensions/' + nb_ext_name, str(nb_path.relative_to(HERE)), '*.*'),
    ("share/jupyter/labextensions/" + lab_ext_name, str(lab_path.relative_to(HERE)), "**"),
    ("share/jupyter/labextensions/" + lab_ext_name, str("."), "install.json"),
    ('etc/jupyter/nbconfig/notebook.d', "jupyros", 'jupyter-ros.json'),
    ('etc/jupyter/jupyter_notebook_config.d', "jupyros", 'jupyros_server_extension.json')
]

long_description = (HERE / "README.md").read_text()

# Get the package info from package.json
pkg_json = json.loads((HERE / "js/package.json").read_bytes())
version = (
    pkg_json["version"]
    .replace("-alpha.", "a")
    .replace("-beta.", "b")
    .replace("-rc.", "rc")
)

setup_args = dict(
    name=name,
    version=version,
    url=pkg_json["homepage"],
    author=pkg_json["author"]["name"],
    author_email=pkg_json["author"]["email"],
    description=pkg_json["description"],
    license=pkg_json["license"],
    license_file="LICENSE",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=setuptools.find_packages(),
    install_requires = [
        'ipywidgets>=7.7.2,<8.0.0',
        'bqplot',
        'numpy',
        'rospkg',
        'ipycanvas'
    ],
    extras_require = {
        'dev': ['click','jupyter_releaser==0.22']
    },
    zip_safe = False,
    include_package_data = True,
    scripts = ['scripts/ros_kernel_generator'],
    python_requires=">=3.7",
    platforms="Linux, Mac OS X, Windows",
    keywords = ['ipython', 'jupyter', 'widgets'],
    classifiers = [
        'Development Status :: 4 - Beta',
        'Framework :: IPython',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'Topic :: Multimedia :: Graphics',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.3',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
    ],
)

try:
    from jupyter_packaging import (
        wrap_installers,
        npm_builder,
        get_data_files
    )
    post_develop = npm_builder(
        path="js", build_cmd="install:extension", source_dir="lib", build_dir=lab_path,  npm='jlpm'
    )
    setup_args["cmdclass"] = wrap_installers(post_develop=post_develop, ensured_targets=ensured_targets)
    setup_args["data_files"] = get_data_files(data_files_spec)
except ImportError as e:
    import logging
    logging.basicConfig(format="%(levelname)s: %(message)s")
    logging.warning("Build tool `jupyter-packaging` is missing. Install it with pip or conda.")
    if not ("--name" in sys.argv or "--version" in sys.argv):
        raise e

if __name__ == "__main__":
    setuptools.setup(**setup_args)