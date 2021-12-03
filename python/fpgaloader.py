#!/usr/bin/env python3
"""
Usage: fpgaloader.py [OPTIONS] COMMAND [ARGS]...

Options:
  -h, --help  Show this message and exit.

Commands:
  current  Returns the current personality.
  list     Returns a list of available personalities.
  reload   Reloads the currently running personality.
  switch   Switches to a personality.
"""

import sys
import shutil
import re
import os.path as osp
import os
import errno
from enum import IntEnum

# @todo: This is a hack. Yocto doesn't set these or default to UTF-8.. so
#        click prints an error. Try to resolve this in a cleaner way.
os.environ['LANG'] = 'en_US'
os.environ['LC_ALL'] = 'en_US'

import click

DTBO_BASE_DIR = '/configfs/device-tree/overlays/'
FW_BASE_DIR = '/lib/firmware/'
FPGA_MGR_DIR = '/sys/class/fpga_manager/fpga0/'
VAR_STATE_DIR = '/var/run/rwt/'

class Status(IntEnum):
    SUCCESS = 0
    MISSING_CMDLINE = 1
    BITFILE_NOT_FOUND = 2
    OVERLAY_NOT_FOUND = 3
    BITFILE_LOAD_FAILED = 4
    ERROR_APPLYING_OVERLAY = 5
    NO_PERSONALITY_LOADED = 6
    ERROR_MOUNTING_CONFIGFS = 7
    PERSONALITY_NOT_FOUND = 8

    @staticmethod
    def tostr(error):
        if error == Status.SUCCESS:
            return "Success"
        elif error == Status.MISSING_CMDLINE:
            return "Device name missing from /proc/cmdline"
        elif error == Status.BITFILE_NOT_FOUND:
            return "Bitfile does not exist"
        elif error == Status.OVERLAY_NOT_FOUND:
            return "Overlay does not exist"
        elif error == Status.BITFILE_LOAD_FAILED:
            return "Bitfile failed to load"
        elif error == Status.ERROR_APPLYING_OVERLAY:
            return "Error applying overlay"
        elif error == Status.NO_PERSONALITY_LOADED:
            return "No current personality loaded"
        elif error == Status.ERROR_MOUNTING_CONFIGFS:
            return "Error trying to mount configfs"
        elif error == Status.PERSONALITY_NOT_FOUND:
            return "Personality not found"
        return "Unknown Error"


def _writefile(msg, filename):
    "Read a filename and return the contents"

    with open(filename, 'w') as fp:
        fp.write(msg)

def _readfile(filename):
    "Read a filename and return the contents"

    with open(filename, 'r') as fp:
        return fp.read().strip()

def _set_current(personality):
    """
    Set the current personality.
    """

    os.makedirs(VAR_STATE_DIR, exist_ok=True)
    _writefile(personality, osp.join(VAR_STATE_DIR, 'current'))

def get_current():
    """
    Return the currently running personality.

    Returns None if no personality is loaded.
    """

    try:
        personality = _readfile(osp.join(VAR_STATE_DIR, 'current')).strip()
    except:
        return None

    if personality == '':
        return None

    return personality


def reload():
    """
    Reload the currently running overlay.
    """

    personality = get_current()
    if personality is None:
        return Status.NO_PERSONALITY_LOADED

    return switch(personality, True)

def list_personalities():
    """
    List the available personalities.
    """
    personalities = []

    fwdir = osp.join(FW_BASE_DIR, 'rwt')
    try:
        dirs = os.listdir(fwdir)
    except:
        dirs = []

    for d in dirs:
        dtbo = osp.join(fwdir, d, 'overlay.dtbo')

        if osp.exists(dtbo):
            personalities.append(d)

    return personalities

def switch(personality, force=True):
    """
    Switch to the specified personality.

    If force is False and the current running personality is requested,
    the personality is not reloaded.
    """

    if not force:
        current = get_current()
        if current == personality:
            return Status.SUCCESS

    cmdline = _readfile('/proc/cmdline')
    match = re.search('fpgadev=([^ \n]+)', cmdline)
    if match is None:
        return Status.MISSING_CMDLINE
    chip = match.group(1)

    fwdir = osp.join('rwt', personality)
    overlaydir = osp.join(DTBO_BASE_DIR, 'rwt')
    dtbo = osp.join(fwdir, 'overlay.dtbo')
    bitfile = osp.join(fwdir, chip, 'download.bin')

    if not osp.exists(osp.join(FW_BASE_DIR, fwdir)):
        return Status.PERSONALITY_NOT_FOUND

    if not osp.exists(osp.join(FW_BASE_DIR, bitfile)):
        return Status.BITFILE_NOT_FOUND

    if not osp.exists(osp.join(FW_BASE_DIR, dtbo)):
        return Status.OVERLAY_NOT_FOUND

    if not osp.exists('/configfs/device-tree'):
        os.makedirs('/configfs', exist_ok=True)
        ret = os.system('mount -t configfs configfs /configfs')
        if ret != 0:
            return Status.ERROR_MOUNTING_CONFIGFS

    # Set the current personality to None in case of a failure.
    _set_current('')

    # Remove the old overlay
    shutil.rmtree(overlaydir, ignore_errors=True)
    os.makedirs(overlaydir, exist_ok=True)

    # Load the bitfile
    _writefile("0\n", osp.join(FPGA_MGR_DIR, "flags"))
    _writefile(bitfile, osp.join(FPGA_MGR_DIR, "firmware"))

    # Check bitfile status
    status = _readfile(osp.join(FPGA_MGR_DIR, 'state'))
    if not status.startswith('operating'):
        return Status.BITFILE_LOAD_FAILED

    # Apply the overlay
    _writefile(dtbo, osp.join(overlaydir, 'path'))

    # Check overlay status
    status = _readfile(osp.join(overlaydir, 'status'))
    if not status.startswith('applied'):
        return Status.ERROR_APPLYING_OVERLAY

    _set_current(personality)

    return Status.SUCCESS


CONTEXT_SETTINGS = dict(help_option_names=['-h', '--help'])
@click.group(context_settings=CONTEXT_SETTINGS)
def cli():
    ""
    pass


@cli.command('switch')
@click.option('-f', '--force', is_flag=True, help="Force a reload.")
@click.argument('personality')
def cli_switch(force, personality):
    """
    Switches to a personality.

    PERSONALITY is the personality to switch
    """
    ret = switch(personality, force)
    print(Status.tostr(ret))
    sys.exit(int(ret))


@cli.command('reload')
def cli_reload():
    """
    Reloads the currently running personality.
    """

    ret = reload()
    print(Status.tostr(ret))
    sys.exit(int(ret))

@cli.command('current')
def cli_current():
    """
    Returns the current personality.
    """

    ret = get_current()
    if ret is None:
        ret = ''
    print(ret)

@cli.command('list')
def cli_list():
    """
    Returns a list of available personalities.
    """
    print(' '.join(list_personalities()))

if __name__ == "__main__":
    cli()
