from os import path

from jupyter_packaging import (
    create_cmdclass, install_npm, ensure_targets,
    combine_commands, ensure_python,
    get_version
)

from setuptools import setup, find_packages

# The name of the project
name = 'jupyros'
nb_ext_name = '@robostack/jupyter-ros'
ext_name = '@robostack/jupyter-ros'

HERE = path.dirname(path.abspath(__file__))
ext_path = path.join(HERE, 'js')

# Ensure a valid python version
ensure_python('>=3.6')

# Get our version
version = get_version(path.join(name, '_version.py'))

# def extract_defaults():
#     import sys
#     sys.path.append("jupyros")

#     import ros3d

#     print("Extracting defaults.js")
#     with open("./js/lib/defaults.js", "w") as fo:
#         s = ros3d.js_extract()
#         fo.write(s)
#     sys.path.pop()

# extract_defaults()

# Extensions' path
module_path = path.join(HERE, name)
nb_path = path.join(HERE, name, 'nbextension')
lab_path = path.join(HERE, name, 'labextension')


cmdclass = create_cmdclass(
    'js',
    package_data_spec = {
        name: [
            'nbextension/*',
            'labextension/*'
        ]
    },
    data_files_spec = [
        ('share/jupyter/nbextensions/' + nb_ext_name, "jupyros/nbextension", '*.*'),
        ("share/jupyter/labextensions/" + ext_name, "jupyros/labextension", "**"),
        ("share/jupyter/labextensions/" + ext_name, ".", "install.json"),
        ('etc/jupyter/nbconfig/notebook.d', "jupyros", 'jupyter-ros.json'),
        ('etc/jupyter/jupyter_notebook_config.d', "jupyros", 'jupyros_server_extension.json')
    ]
)

cmdclass['js'] = combine_commands(
    install_npm(
        path=ext_path,
        npm=["jlpm"],
        build_cmd="build:labextension",
        source_dir=path.join(ext_path, 'lib')
    ),
    # Representative files that should exist after a successful build
    ensure_targets([
        path.join(nb_path, 'index.js'),
        path.join(lab_path, 'package.json'),
    ]),
)

setup_args = {
    'name': name,
    'version': version,
    'description': 'ROS 3D Jupyter widget',
    'long_description': 'ROS 3D Jupyter widget',
    'author': 'Wolf Vollprecht',
    'author_email': 'w.vollprecht@gmail.com',
    'url': 'https://github.com/RoboStack/jupyter-ros',
    'keywords': [
        'ipython',
        'jupyter',
        'widgets',
    ],
    'include_package_data': True,
    'scripts': ['scripts/ros_kernel_generator'],
    'install_requires': [
        'ipywidgets>=7.0.0',
        'bqplot',
        'numpy',
        'rospkg'
    ],
    'packages': find_packages(),
    'zip_safe': False,
    'cmdclass': cmdclass,
    'classifiers': [
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
}

setup(**setup_args)
