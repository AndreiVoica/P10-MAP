"""
Planar Motor Python API Basic Example

(c) Planar Motor Inc 2022
"""
from pmclib import system_commands as sys   # PMC System related commands
from pmclib import xbot_commands as bot     # PMC Mover related commands
from pmclib import pmc_types                # PMC API Types

import time

# %% Connect to the PMC
# To start sending commands to the system, the library must
# first connect to the Planar Motor Controller (PMC) via TCP/IP.
# this can be done by explicitly connecting to a known PMC IP
# address, or by scanning the network using an auto-connect
# command. By default, the PMC IP address is 192.168.10.100.
sys.connect_to_pmc("192.168.10.100")
# or
# sys.auto_connect_to_pmc()

# %% Activating the system
# On bootup, all the movers within the system will be in a
# "Deactivated" state. Which means that they are not actively
# position controlled. To start controlling the system, the
# "activate" command must be sent.
bot.activate_xbots()

# Now we wait for the movers to be levitated and fully controlled.
# This can be done by periodically polling for the PMC status.
maxTime = time.time() + 60  # Set timeout of 60s
while sys.get_pmc_status() is not pmc_types.PmcStatus.PMC_FULLCTRL:
    time.sleep(0.5)
    if time.time() > maxTime:
        raise TimeoutError("PMC Activation timeout")


# %% Basic mover commands
# Now that the mover are levitated, they are now ready to receive
# motion commands.

bot.linear_motion_si(xbot_id=1, target_x=0.06, target_y=0.06,
                     max_speed=1.0, max_accel=10.0)

# The commands will return as soon the PMC receives and acknowledges
# that the command is valid/invalid. Multiple motion commands can be
# buffered to the same mover, and they will execute continuously in the
# order that the command was sent. Let's define a simple sample motion:

def sample_motions(input_id):
    bot.linear_motion_si(xbot_id=input_id, target_x=0.18, target_y=0.06,
                         max_speed=1.0, max_accel=10.0)
    bot.linear_motion_si(xbot_id=input_id, target_x=0.18, target_y=0.18,
                         max_speed=1.0, max_accel=10.0)
    bot.linear_motion_si(xbot_id=input_id, target_x=0.06, target_y=0.18,
                         max_speed=1.0, max_accel=10.0)
    bot.linear_motion_si(xbot_id=input_id, target_x=0.06, target_y=0.06,
                         max_speed=1.0, max_accel=10.0)

# Sending all commands, will buffer into the movers "Motion Buffer"
sample_motions(input_id=1)

# To check if all buffered motions are complete, we poll for
# the xbot information, and check that it's state is IDLE.
# Let's define a helper function for this:

def wait_for_xbot_done(xid):
    while bot.get_xbot_status(xbot_id=xid).xbot_state is not pmc_types.XbotState.XBOT_IDLE:
        time.sleep(0.5)

# Now we can wait for all motions buffered to an mover to be
# complete
wait_for_xbot_done(xid=1)

# %% Macros
# We can also save a series of motion commands as a "macro", which
# can be re-used for different movers. Macros are programmed by sending
# commands to mover ID 128 - 191

# First we clear the macro
bot.edit_motion_macro(
    option=pmc_types.MotionMacroOptions.CLEAR_MACRO, macro_id=128)

# The the commands can be programmed to the macro
sample_motions(input_id=128)

# Then the macro can be saved,and run
bot.edit_motion_macro(
    option=pmc_types.MotionMacroOptions.SAVE_MACRO, macro_id=128)
bot.run_motion_macro(macro_id=128, xbot_id=1)

wait_for_xbot_done(xid=1)

# Macros can be infinitely looped/ chained together by sending
# run_motion_macro commands to macro IDs. i.e. to run macro id
# 128 in a loop:

# First clear the macro so we can edit it again
bot.edit_motion_macro(
    option=pmc_types.MotionMacroOptions.CLEAR_MACRO, macro_id=128)

# Send some motion commands
sample_motions(input_id=128)

# Then send run_macro 128 to macro ID 128
bot.run_motion_macro(macro_id=128, xbot_id=128)
bot.edit_motion_macro(
    option=pmc_types.MotionMacroOptions.SAVE_MACRO, macro_id=128)

# Now when we run, the macro will infinitely loop
bot.run_motion_macro(macro_id=128, xbot_id=1)

time.sleep(5)

# To stop the motion, we can send the stop motion command
bot.stop_motion(xbot_id=1)

