#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2023 Fabien Caura <fabien@acathla.tk>
# SPDX-License-Identifier: BSD-2-Clause

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxSpartan3Platform
from litex.build.xilinx.programmer import XC3SProg

# IOs ----------------------------------------------------------------------------------------------
_io = [
    # clk
    ("clk50",  0, Pins("E12"),  IOStandard("LVCMOS33")),

    # leds
    ("user_led",  0, Pins("R20"),  IOStandard("LVCMOS33")),
    ("user_led",  1, Pins("T19"),  IOStandard("LVCMOS33")),
    ("user_led",  2, Pins("U20"),  IOStandard("LVCMOS33")),
    ("user_led",  3, Pins("U19"),  IOStandard("LVCMOS33")),
    ("user_led",  4, Pins("V19"),  IOStandard("LVCMOS33")),
    ("user_led",  5, Pins("V20"), IOStandard("LVCMOS33")),
    ("user_led",  6, Pins("Y22"), IOStandard("LVCMOS33")),
    ("user_led",  7, Pins("W21"), IOStandard("LVCMOS33")),

    # Buttons
    ("user_btn", 0, Pins("U15"),  IOStandard("LVCMOS33")), # west
    ("user_btn", 1, Pins("T15"),  IOStandard("LVCMOS33")), # south
    ("user_btn", 2, Pins("T14"), IOStandard("LVCMOS33")), # north
    ("user_btn", 3, Pins("T16"), IOStandard("LVCMOS33")), #  east

    # Switches
    ("user_dip", 0, Pins("V8"), IOStandard("LVCMOS33")),
    ("user_dip", 1, Pins("U10"), IOStandard("LVCMOS33")),
    ("user_dip", 2, Pins("U8"), IOStandard("LVCMOS33")),
    ("user_dip", 3, Pins("T9"), IOStandard("LVCMOS33")),

    # Serial DCE (pc)
    ("serial", 0,
        Subsignal("rx", Pins("E16"), IOStandard("LVCMOS33")),
        Subsignal("tx", Pins("F15"), IOStandard("LVCMOS33")),
    ),

    # Serial DTE (device)
    ("serial", 1,
        Subsignal("rx", Pins("F16"), IOStandard("LVCMOS33")),
        Subsignal("tx", Pins("E15"), IOStandard("LVCMOS33")),
    ),

]

class Platform(XilinxSpartan3Platform):
    default_clk_name   = "clk50"
    default_clk_period = 1e9/50e6

    def __init__(self, toolchain="ise"):
        #XC3S700A/AN-4FGG484C
        #XilinxSpartan3Platform.__init__(self, "xc3s700an-4fgg484c-3", _io, _connectors, toolchain=toolchain)
        XilinxSpartan3Platform.__init__(self, "xc3s700an-4-fgg484", _io, toolchain=toolchain)
        #self.toolchain.bitgen_opt += " -g Compress -g ConfigRate:6"

    def create_programmer(self):
        return XC3SProg(cable="xpc")

    def do_finalize(self, fragment):
        XilinxSpartan3Platform.do_finalize(self, fragment)
        self.add_period_constraint(self.lookup_request("clk50", loose=True), 1e9/50e6)
