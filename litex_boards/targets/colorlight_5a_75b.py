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

import os
import argparse
import sys

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.doc import generate_docs, generate_svd

from litex_boards.platforms import colorlight_5a_75b

from litex.build.lattice.trellis import trellis_args, trellis_argdict

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litedram.modules import M12L16161A
from litedram.phy import GENSDRPHY

from liteeth.phy.ecp5rgmii import LiteEthPHYRGMII

# LED ----------------------------------------------------------------------------------------------

class _led(Module):
    def __init__(self, platform, sys_clk_freq):

        # Led --------------------------------------------------------------------------------------
        led_counter = Signal(32)
        self.sync += led_counter.eq(led_counter + 1)
        #self.comb += platform.request("user_led_n", 0).eq(led_counter[26])

        #self.comb += platform.request("debug", 0).eq(led_counter[26])
        self.comb += platform.request("j1", 0).eq(led_counter[27])
        self.comb += platform.request("j1", 1).eq(led_counter[26])
        self.comb += platform.request("j1", 2).eq(led_counter[25])


# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys    = ClockDomain()
        self.clock_domains.cd_sys_ps = ClockDomain()
        #self.clock_domains.cd_sys_125 = ClockDomain(reset_less=True)


        # # #

        # Clk / Rst
        clk25 = platform.request("clk25")
        #rst_n = platform.request("user_btn_n", 0)
        platform.add_period_constraint(clk25, 1e9/25e6)

        # PLL
        self.submodules.pll = pll = ECP5PLL()

        pll.register_clkin(clk25, 25e6)
        pll.create_clkout(self.cd_sys,    sys_clk_freq)
        pll.create_clkout(self.cd_sys_ps, sys_clk_freq, phase=90)
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~pll.locked)

        # SDRAM clock
        self.comb += platform.request("sdram_clock").eq(self.cd_sys_ps.clk)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, revision, toolchain, with_ethernet=False, with_etherbone=False, **kwargs):
        platform     = colorlight_5a_75b.Platform(revision=revision, toolchain=toolchain)
        sys_clk_freq = int(125e6)

        # serial
        platform.add_extension(colorlight_5a_75b.serial)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq, **kwargs)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # LED --------------------------------------------------------------------------------------
        self.submodules.led = _led(platform, sys_clk_freq)

        # SDR SDRAM --------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            self.submodules.sdrphy = GENSDRPHY(platform.request("sdram"), cl=2)
            self.add_sdram("sdram",
                phy                     = self.sdrphy,
                module                  = M12L16161A(sys_clk_freq, "1:1"),
                origin                  = self.mem_map["main_ram"],
                size                    = kwargs.get("max_sdram_size", 0x40000000),
                l2_cache_size           = kwargs.get("l2_size", 8192),
                l2_cache_min_data_width = kwargs.get("min_l2_data_width", 128),
                l2_cache_reverse        = True
            )

        # Ethernet ---------------------------------------------------------------------------------
        if with_ethernet:
            self.submodules.ethphy = LiteEthPHYRGMII(
                clock_pads = self.platform.request("eth_clocks"),
                pads       = self.platform.request("eth"))
            self.add_csr("ethphy")
            self.add_ethernet(phy=self.ethphy)

        # Etherbone --------------------------------------------------------------------------------
        if with_etherbone:
            self.submodules.ethphy = LiteEthPHYRGMII(
                clock_pads = self.platform.request("eth_clocks"),
                pads       = self.platform.request("eth"))
            self.add_csr("ethphy")
            self.add_etherbone(phy=self.ethphy)


# Load / Flash -------------------------------------------------------------------------------------

def openocd_run_svf(filename):
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
    os.system("openocd -f openocd.cfg -c \"transport select jtag; init; svf {}; exit\"".format(filename))
    os.system("rm openocd.cfg")

def load():
    openocd_run_svf("soc_basesoc_colorlight_5a_75b/gateware/top.svf")

def flash():
    import os
    os.system("cp bit_to_flash.py soc_basesoc_colorlight_5a_75b/gateware/")
    os.system("cd soc_basesoc_colorlight_5a_75b/gateware && ./bit_to_flash.py top.bit top.svf.flash")
    openocd_run_svf("soc_basesoc_colorlight_5a_75b/gateware/top.svf.flash")


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
    parser.add_argument("--gateware-toolchain", dest="toolchain", default="trellis",
        help="gateware toolchain to use, trellis (default) or diamond")
    parser.add_argument("--with-ethernet",  action="store_true", help="enable Ethernet support")
    parser.add_argument("--with-etherbone", action="store_true", help="enable Etherbone support")
    parser.add_argument("--eth-phy", default=0, type=int, help="Ethernet PHY 0 or 1 (default=0)")
    parser.add_argument("--load", action="store_true", help="load bitstream")
    parser.add_argument("--flash", action="store_true", help="flash bitstream")
    parser.add_argument("--sim", action="store_true", help="sim led (WIP)")
    args = parser.parse_args()

    assert not (args.with_ethernet and args.with_etherbone)
    soc = BaseSoC(revision = args.revision,
        toolchain = args.toolchain,
        with_ethernet  = args.with_ethernet,
        with_etherbone = args.with_etherbone,
        **soc_core_argdict(args))

    #builder = Builder(soc, **builder_argdict(args))
    ## für diamond toolchain ohne trellis_argdict?
    #if args.toolchain == "trellis":
    #    vns=builder.build(**trellis_argdict(args))
    #else:
    #    vns=builder.build()
    #soc.do_exit(vns)
    #generate_docs(soc, "build/documentation")
    ##generate_svd(soc, "build/software")

    if args.load:
        load()

    if args.flash:
        flash()

    if args.sim:
        sim()


if __name__ == "__main__":
    main()
