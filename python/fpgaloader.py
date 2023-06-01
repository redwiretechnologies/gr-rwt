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
import json
import smbus
from enum import IntEnum

# @todo: This is a hack. Yocto doesn't set these or default to UTF-8.. so
#        click prints an error. Try to resolve this in a cleaner way.
os.environ['LANG'] = 'en_US'
os.environ['LC_ALL'] = 'en_US'

import click

DTBO_BASE_DIR = '/configfs/device-tree/overlays/'
FW_BASE_DIR = '/lib/firmware/'
JSON_LOC = '/home/root/board_id'
FPGA_MGR_DIR = '/sys/class/fpga_manager/fpga0/'
VAR_STATE_DIR = '/var/run/rwt/'
PC_CARDS = {'PC0'  : 0x52,
            'PC1'  : 0x53,
            'CPC1' : 0x55,
            'CPC2' : 0x3C,
            'CPC3' : 0x2C}

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

def _remove_overlay(overlaydir):
#    print("Remove overlay: {}".format(overlaydir))
#    return
    shutil.rmtree(overlaydir, ignore_errors=True)
    os.makedirs(overlaydir, exist_ok=True)

def _apply_overlay(overlaydir, overlayfile):
#    print("Overlay File: {}".format(overlayfile))
#    print("Overlay Directory: {}".format(overlaydir))
#    return Status.SUCCESS
    _writefile(overlayfile, osp.join(overlaydir, 'path'))

    # Check overlay status
    status = _readfile(osp.join(overlaydir, 'status'))
    if not status.startswith('applied'):
        return Status.ERROR_APPLYING_OVERLAY

    return Status.SUCCESS

def _remove_pc_cards():
    for p in PC_CARDS.keys():
        overlaydir = osp.join(DTBO_BASE_DIR, p)
        _remove_overlay(overlaydir)

def _apply_pc_cards():
    returns = []
    for i, p in enumerate(PC_CARDS):
        overlaydir = osp.join(DTBO_BASE_DIR, p)
        ret = _detect_pc_card(PC_CARDS[p])
        if ret != -1:
            returns.append(_apply_overlay(overlaydir, "rwt/{}-{}.dtbo".format(ret, i)))
        else:
            returns.append(Status.SUCCESS)
    return returns

def _detect_pc_card(p):

    bus = smbus.SMBus(0)

    try:
        bus.read_byte_data(p, 0x00)

        # Open the JSON file which defines all of the boards
        f = open("{}/board_id.json".format(JSON_LOC))
        board_id_lookup = json.load(f)
        f.close()

        short_form = bus.read_i2c_block_data(p, 0x7d, 3)

        t  = short_form[0]
        n  = short_form[1]
        r  = short_form[2]
        r  = str(r>>4) + "_" + str(r & 15)

        board_name = board_id_lookup["id"][board_id_lookup["type"][t]][n]

        if board_id_lookup["type"][t] != "Personality":
            print("Invalid short form!")
            print("  ", t)
            print("  ", n)
            print("  ", r)
            return -1

        if board_name in board_id_lookup["no_dts"]:
            print("Found board {} on no_dts list".format(board_name))
            return -1

        return "{}-{}".format(board_name, r)
    except:
        return -1

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

    # Remove any applied DTS for personality cards
    _remove_pc_cards()

    # Remove the old overlay
    _remove_overlay(overlaydir)

    # Load the bitfile
    _writefile("0\n", osp.join(FPGA_MGR_DIR, "flags"))
    _writefile(bitfile, osp.join(FPGA_MGR_DIR, "firmware"))

    # Check bitfile status
    status = _readfile(osp.join(FPGA_MGR_DIR, 'state'))
    if not status.startswith('operating'):
        return Status.BITFILE_LOAD_FAILED

    # Apply the overlay
    status = _apply_overlay(overlaydir, dtbo)
    if status != Status.SUCCESS:
        return status

    statuses = _apply_pc_cards()
    for status in statuses:
        if status != Status.SUCCESS:
            return status

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
