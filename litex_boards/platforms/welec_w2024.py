#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2014-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litex.build.generic_platform import *
from litex.build.altera import AlteraPlatform
from litex.build.altera.programmer import USBBlaster

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Clk
    ("clk25", 0, Pins("M1"), IOStandard("LVDS")),

    # F1 & F2 from frontpanel are directly connected
    # Button 
    ("key", 0, Pins("AB12"), IOStandard("3.3-V LVTTL")),
    ("key", 1, Pins("E9"),  IOStandard("3.3-V LVTTL")),

    # Serial
    ("serial", 0,
        Subsignal("tx", Pins("D11"), IOStandard("3.3-V LVTTL")),
        Subsignal("rx", Pins("E11"), IOStandard("3.3-V LVTTL"))
    ),

    # VGA.
    ("vga", 0,
        Subsignal("en",  Pins("W8")),
#        Subsignal("blank_n", Pins("AH3")),
        Subsignal("clk",     Pins("AA6")),
        Subsignal("hsync_n", Pins("AB7")),
        Subsignal("vsync_n", Pins("AB6")),
        Subsignal("r", Pins("W7 AB3 Y7")),
        Subsignal("g", Pins("AA3 AB4 AB5")),
        Subsignal("b", Pins("AA5 Y6 AA4")),
        IOStandard("3.3-V LVTTL")
    ),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(AlteraPlatform):
    default_clk_name   = "clk25"
    default_clk_period = 1e9/25e6

    def __init__(self):
        AlteraPlatform.__init__(self, "EP2C35F484C8", _io)

    def create_programmer(self):
        return USBBlaster(cable_name="Arrow-USB-Blaster")

    def do_finalize(self, fragment):
        AlteraPlatform.do_finalize(self, fragment)
        self.add_period_constraint(self.lookup_request("clk25", loose=True), 1e9/25e6)
