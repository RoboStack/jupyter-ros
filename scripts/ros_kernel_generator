#!/usr/bin/env python

import sys
import json
import subprocess
import shutil
import os

if len(sys.argv) < 3:
    print("Usage: ros_kernel_generator SOURCE_KERNEL TARGET_WORKSPACE/setup.bash\nWhere source kernel is the name of a Python kernel such as python2 (cf. ``jupyter kernelspec list``).")

keys = {
    "CMAKE_PREFIX_PATH",
    "LD_LIBRARY_PATH",
    "PATH",
    "PYTHONPATH",
    "PKG_CONFIG_PATH",
    "ROSLISP_PACKAGE_DIRECTORIES",
    "ROS_DISTRO",
    "ROS_ETC_DIR",
    "ROS_MASTER_URI",
    "ROS_PACKAGE_PATH",
    "ROS_PYTHON_VERSION",
    "ROS_ROOT",
    "ROS_VERSION",
    # "SBCL_HOME"
}


def diff(first_file, second_file, json_file):

    first_file.seek(0)

    without_ros_dict = {}

    for el in first_file:
        partition = el.partition("=")
        without_ros_dict[partition[0]] = partition[2]

    second_file.seek(0)

    with_ros_dict = {}

    for el in second_file:
        partition = el.partition("=")
        with_ros_dict[partition[0]] = partition[2].rstrip()

    diff_dict = {k: with_ros_dict.get(k) for k in keys}

    for k in keys:
        if diff_dict[k] is None:
            del diff_dict[k]

    with json_file:
        json_data = json.load(json_file)
        json_data["env"] = diff_dict
        json_file.seek(0)
        json.dump(
            json_data, json_file, sort_keys=True, indent=4, separators=(",", ": ")
        )
        json_file.truncate()


if sys.version_info >= (3, 0):
    input_func = input
else:
    input_func = raw_input

argv = sys.argv
python_version = argv[1]
ros_command = "source " + argv[2]

subprocess.call(
    "printenv > /tmp/without_ros.txt;" + ros_command + ";printenv > /tmp/with_ros.txt",
    shell=True,
    executable="/bin/bash",
)

without_ros = open("/tmp/without_ros.txt")
with_ros = open("/tmp/with_ros.txt")

python_spec_list = subprocess.check_output(
    ["jupyter kernelspec list --json"], shell=True
)
python_spec_list = json.loads(python_spec_list)

python_path = python_spec_list["kernelspecs"][python_version]["resource_dir"]
partition = python_path.partition("kernels/")
ros_python_path = partition[0] + partition[1] + "ros_python/"

try:
    os.mkdir(ros_python_path)
except:
    pass

with open(python_path + "/kernel.json", "r") as python_spec:
    python_spec = json.load(python_spec)

with open(ros_python_path + "kernel.json", "w") as ros_python_spec:
    python_spec["display_name"] = "ROS " + python_spec["display_name"]
    json.dump(
        python_spec, ros_python_spec, sort_keys=True, indent=4, separators=(",", ": ")
    )

diff(without_ros, with_ros, open(ros_python_path + "kernel.json", "r+"))