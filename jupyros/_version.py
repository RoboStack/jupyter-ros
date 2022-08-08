#############################################################################
# Copyright (c) Wolf Vollprecht, QuantStack                                 #
#                                                                           #
# Distributed under the terms of the BSD 3-Clause License.                  #
#                                                                           #
# The full license is in the file LICENSE, distributed with this software.  #
#############################################################################

import json
from pathlib import Path

__all__ = ["__version__"]

def _fetchVersion():
    HERE = Path(__file__).parent.parent.resolve()

    for settings in HERE.rglob("package.json"):
        try:
            with settings.open() as f:
                version = json.load(f)["version"]
                return (
                    version.replace("-alpha.", "a")
                    .replace("-beta.", "b")
                    .replace("-rc.", "rc")
                )
        except FileNotFoundError:
            pass

    raise FileNotFoundError(f"Could not find package.json under dir {HERE!s}")

def _fetchJSVersion():
    HERE = Path(__file__).parent.parent.resolve()

    for settings in HERE.rglob("package.json"):
        try:
            with settings.open() as f:
                return json.load(f)["version"]
        except FileNotFoundError:
            pass

    raise FileNotFoundError(f"Could not find package.json under dir {HERE!s}")

__version__ = _fetchVersion()
