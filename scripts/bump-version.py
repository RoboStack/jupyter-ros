#############################################################################
# Copyright (c) 2018, QuantStack                                            #
#                                                                           #
# Distributed under the terms of the BSD 3-Clause License.                  #
#                                                                           #
# The full license is in the file LICENSE, distributed with this software.  #
#############################################################################

import json
import click
from pathlib import Path
from jupyter_releaser.util import get_version, run
from pkg_resources import parse_version

LERNA_CMD = "jlpm run lerna version --no-push --force-publish --no-git-tag-version"

def _update_version(version):
    HERE = Path(__file__).parent.resolve()

    for settings in HERE.rglob("package.json"):
        try:
            with settings.open() as f:
                version = json.load(f)["version"]
                print(version)
        except FileNotFoundError:
            pass

    raise FileNotFoundError(f"Could not find package.json under dir {HERE!s}")

@click.command()
@click.option("--force", default=False, is_flag=True)
@click.argument("spec", nargs=1)
def bump(force, spec):
    status = run("git status --porcelain").strip()
    if len(status) > 0:
        raise Exception("Must be in a clean git state with no untracked files")

    curr = parse_version(get_version())
    if spec == 'next':
        spec = f"{curr.major}.{curr.minor}."
        if curr.pre:
            p, x = curr.pre
            spec += f"{curr.micro}{p}{x + 1}"
        else:
            spec += f"{curr.micro + 1}"
    
    elif spec == 'patch':
        spec = f"{curr.major}.{curr.minor}."
        if curr.pre:
            spec += f"{curr.micro}"
        else:
            spec += f"{curr.micro + 1}"


    version = parse_version(spec)

    # convert the Python version
    js_version = f"{version.major}.{version.minor}.{version.micro}"
    if version.pre:
        p, x = version.pre
        p = p.replace("a", "alpha").replace("b", "beta")
        js_version += f"-{p}.{x}"

    # bump the JS packages
    lerna_cmd = f"{LERNA_CMD} {js_version}"
    if force:
        lerna_cmd += " --yes"
    #run(lerna_cmd)

    _update_version(js_version)
    print(js_version)


if __name__ == "__main__":
    bump()
