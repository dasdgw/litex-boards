#!/usr/bin/env python3

# This file is Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

# Disclaimer: This SoC is still a Proof of Concept with large timings violations on the IP/UDP and
# Etherbone stack that need to be optimized. It was initially just used to validate the reversed
# pinout but happens to work on hardware...

# Build/Use:
# ./colorlight_5a_75b.py --uart-name=crossover --with-etherbone --csr-csv=csr.csv
# ./colorlight_5a_75b.py --load
# ping 192.168.1.50
# Get and install wishbone tool from: https://github.com/litex-hub/wishbone-utils/releases
# wishbone-tool --ethernet-host 192.168.1.50 --server terminal --csr-csv csr.csv
# You should see the LiteX BIOS and be able to interact with it.
#
# Build/Use without ethernet:
# ./colorlight_5a_75b.py --uart-name=stub --csr-csv=csr.csv
# ./colorlight_5a_75b.py --load

import argparse
import sys

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex_boards.platforms import colorlight_5a_75b

from litex.build.lattice.trellis import trellis_args, trellis_argdict

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from liteeth.phy.ecp5rgmii import LiteEthPHYRGMII
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone

# LED ----------------------------------------------------------------------------------------------

class _LED(Module):
    def __init__(self, platform, sys_clk_freq):

        # Led --------------------------------------------------------------------------------------
        led_counter = Signal(32)
        self.sync += led_counter.eq(led_counter + 1)
        self.comb += platform.request("user_led_n", 0).eq(led_counter[26])

        #self.comb += platform.request("debug", 0).eq(led_counter[26])

        #self.comb += platform.request("j1", 0).eq(led_counter[27])
        #self.comb += platform.request("j1", 1).eq(led_counter[26])
        #self.comb += platform.request("j1", 2).eq(led_counter[25])

        hub75 = platform.request("hub75", 0)
        self.comb += [
            hub75.R0.eq(led_counter[27]),
            hub75.G0.eq(led_counter[26]),
            hub75.B0.eq(led_counter[25])
        ]
# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()

        # # #

        # Clk / Rst
        clk25 = platform.request("clk25")
        rst_n = platform.request("user_btn_n", 0)
        platform.add_period_constraint(clk25, 1e9/25e6)

        # PLL
        self.submodules.pll = pll = ECP5PLL()

        pll.register_clkin(clk25, 25e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~pll.locked | ~rst_n)

# NoSoC ------------------------------------------------------------------------------------------
class NoSoC(SoCCore):
    def __init__(self, revision, **kwargs):
        platform     = colorlight_5a_75b.Platform(revision=revision)
        sys_clk_freq = int(25e6)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq, **kwargs)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        self.submodules.led = _LED(platform, sys_clk_freq)


# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, revision, **kwargs):
        platform     = colorlight_5a_75b.Platform(revision=revision)
        sys_clk_freq = int(25e6)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq, **kwargs)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        self.submodules.led = _LED(platform, sys_clk_freq)


# EtherboneSoC -------------------------------------------------------------------------------------

class EtherboneSoC(BaseSoC):
    def __init__(self, eth_phy=0, **kwargs):
        BaseSoC.__init__(self, **kwargs)

        # Ethernet ---------------------------------------------------------------------------------
        # phy
        self.submodules.ethphy = LiteEthPHYRGMII(
            clock_pads = self.platform.request("eth_clocks", eth_phy),
            pads       = self.platform.request("eth", eth_phy),
            tx_delay   = 0e-9, # 0ns FPGA delay (Clk delay added by PHY)
            rx_delay   = 2e-9) # 2ns FPGA delay to compensate Clk routing to IDDRX1F
        self.add_csr("ethphy")
        # core
        self.submodules.ethcore = LiteEthUDPIPCore(
            phy         = self.ethphy,
            mac_address = 0x10e2d5000000,
            ip_address  = "192.168.1.50",
            clk_freq    = self.clk_freq)
        # etherbone
        self.submodules.etherbone = LiteEthEtherbone(self.ethcore.udp, 1234)
        self.add_wb_master(self.etherbone.wishbone.bus)
        # timing constraints
        self.platform.add_period_constraint(self.ethphy.crg.cd_eth_rx.clk, 1e9/25e6)
        self.platform.add_period_constraint(self.ethphy.crg.cd_eth_tx.clk, 1e9/25e6)
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.ethphy.crg.cd_eth_rx.clk,
            self.ethphy.crg.cd_eth_tx.clk)

# Load ---------------------------------------------------------------------------------------------

def load():
    import os
    f = open("openocd.cfg", "w")
    f.write(
"""
interface ftdi
ftdi_vid_pid 0x0403 0x6010
ftdi_channel 0
ftdi_layout_init 0x0098 0x008b
reset_config none
adapter_khz 25000
jtag newtap ecp5 tap -irlen 8 -expected-id 0x41111043
""")
    f.close()
    # ! the output path depends on the build target
    # with-etherbone: soc_etherbonesoc_colorlight_5a_75b/gateware/top.svf
    # no-soc: svf build/top.svf
    #
    # os.system("openocd -f openocd.cfg -c \"transport select jtag; init; svf build/top.svf; exit\"")
    os.system("openocd -f openocd.cfg -c \"transport select jtag; init; svf soc_etherbonesoc_colorlight_5a_75b/gateware/top.svf; exit\"")
    exit()

# sim ----------------------------------------------------------------------------------------------

def sim():
    print("sim")
    exit()

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on Colorlight 5A-75B")
    builder_args(parser)
    soc_core_args(parser)
    trellis_args(parser)
    parser.add_argument("--revision", default="7.0", type=str, help="Board revision 7.0 (default) or 6.1")
    parser.add_argument("--with-etherbone", action="store_true", help="enable Etherbone support")
    parser.add_argument("--eth-phy", default=0, type=int, help="Ethernet PHY 0 or 1 (default=0)")
    parser.add_argument("--load", action="store_true", help="load bitstream")
    parser.add_argument("--sim", action="store_true", help="sim led (WIP)")
    parser.add_argument("--no-soc", action="store_true", help="without SoC")
    args = parser.parse_args()

    if args.load:
        load()

    if args.sim:
        sim()

    if args.no_soc:
        platform     = colorlight_5a_75b.Platform(revision=args.revision)
        sys_clk_freq = int(25e6)
        dut = _LED(platform, sys_clk_freq)
        platform.build(dut)
        exit()

    if args.with_etherbone:
        soc = EtherboneSoC(eth_phy=args.eth_phy, revision=args.revision, **soc_core_argdict(args))
    else:
        soc = BaseSoC(args.revision, **soc_core_argdict(args))
    builder = Builder(soc, **builder_argdict(args))
    builder.build(**trellis_argdict(args))

if __name__ == "__main__":
    main()
