"""Locate the DeltaSeek Clearpath configuration without hard-coding a machine.

The Clearpath generators read ``robot.yaml`` from a *setup path*.  This project
keeps that file in the repository rather than in the system location, so the
launch files have to discover it.  Resolving it here keeps every absolute path
out of the launch files, which is what lets the same checkout run on a
workstation and on a shared server.
"""

import os
from pathlib import Path


ENV_VAR = 'DELTASEEK_SETUP_PATH'
SYSTEM_SETUP_PATH = '/etc/clearpath'


def repository_root():
    """Return the repository checkout containing this module, if reachable.

    ``colcon build --symlink-install`` installs this package by symlink, so
    resolving the module reaches the source tree and therefore the sibling
    ``clearpath/`` directory.  A plain ``colcon build`` copies the module into
    ``install/`` instead, where no such sibling exists.
    """
    return Path(os.path.realpath(__file__)).parents[4]


def default_setup_path():
    """Return the Clearpath setup directory to use when none was given.

    Resolution order: the ``DELTASEEK_SETUP_PATH`` override, then the
    repository's own ``clearpath/`` directory, then Clearpath's system
    location for an installed robot.
    """
    override = os.environ.get(ENV_VAR)
    if override:
        return override
    candidate = repository_root() / 'clearpath'
    if (candidate / 'robot.yaml').is_file():
        return str(candidate)
    return SYSTEM_SETUP_PATH
