#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2023 Pano Logic G2 reverse-engineering community
# SPDX-License-Identifier: BSD-2-Clause

# The Pano Logic Zero Client G2 is a thin commercial client that can be repurposed as a generic
# development board, see https://github.com/tomverbeure/panologic-g2.
#
# Three things make this board special, and this target is written around all of them:
#
# 1. CLOCKING. The FPGA has *no* oscillator of its own. The only one on the board (25 MHz) feeds the
#    Marvell 88E1119R Ethernet PHY, whose recovered clock appears on `clk125` (Y13) as **125 MHz when
#    the PHY is out of reset and 25 MHz when it is in reset**. `eth_rst_n` (R11) is therefore *always*
#    driven high here, whatever the Ethernet is used for: with the PHY in reset the whole SoC would be
#    running at 1/5th of its constrained frequency, silently.
#
# 2. DDR2. Two MT47H32M16 (x16, 64 MB each) with **independent** address/command pins per chip -- that
#    is not a standard multi-rank wiring (litedram shares cmd/addr between ranks), so one chip is
#    driven at a time: --sdram-chip=a|b, default "b" (the configuration litex-buildenv proved on this
#    board, with rd_bitslip=0 wr_bitslip=4 dqs_ddr_alignment="C0"). The dm/dqs/dqs_n byte lanes of the
#    platform file are ordered low-lane-first, matching `dq` (see the note there: the historical order
#    was swapped and litedram pairs those lists by index).
#
# 3. The S6 half-rate PHY needs 3 clock domains (sdram_half = 2x sys, sdram_full = 4x sys) generated
#    by BUFIO2 + PLL_ADV + BUFPLL, so the VCO has to be an integer multiple of 4*sys_clk_freq *and*
#    inside the Spartan-6 range (400-1000 MHz on a -2 part). The PLL is computed from those
#    constraints below; sys_clk_freq=50e6 lands on the proven litex-buildenv setting (VCO=1000 MHz,
#    sdram_full=200/5, sdram_half=100/10, sys=50/20).
#
# Build (rev C = XC6SLX100, Linux-capable SoC, DDR2 chip B, Ethernet) --------------------------------
#
#   python3 -m litex_boards.targets.pano_logic_g2 --revision=c --cpu-type=vexriscv --cpu-variant=linux \
#       --with-ethernet --build --csr-csv=csr.csv
#
# Load over JTAG (openFPGALoader, see the platform's create_programmer) / write to the SPI flash:
#
#   ... --load                     # SRAM, volatile
#   ... --flash --flash-ext=.mcs   # generate the .mcs with impact first (unambiguous byte order)
#
# Console: --serial-index selects which DDC pair is the UART (0 = micro-HDMI AB19/AA21, 1 = DVI
# C14/C17). litex_term /dev/ttyUSBX --speed=115200.

import os
from fractions import Fraction

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex_boards.platforms import pano_logic_g2

from litex.soc.cores.clock import *
from litex.soc.integration.soc import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser

from liteeth.phy import LiteEthPHY

from litedram.modules import MT47H32M16
from litedram.phy import s6ddrphy
from litedram.core import ControllerSettings

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq,
        sdram_chip      = "b",
        sdram_half_phase = 270.,
        sdram_clk_phase  = 250.,
        cpu_reset_pin    = None):
        self.rst            = Signal()
        self.cd_sys         = ClockDomain()
        self.cd_sdram_half  = ClockDomain()
        self.cd_sdram_full_wr = ClockDomain()
        self.cd_sdram_full_rd = ClockDomain()

        # # #

        # Keep the Ethernet PHY out of reset: it is what generates clk125 (see header comment). This is
        # a standalone platform resource, not part of the `eth` group -- liteeth never drives reset.
        self.comb += platform.request("eth_rst_n").eq(1)

        # PLL parameters ---------------------------------------------------------------------------
        # VCO = 4 * sys_clk_freq * q, with the OSERDES2/ISERDES2 (4:1) and SDRAM (2x) relations all
        # derived from it. -2 speed grade: VCO in [400, 1000] MHz (DS181; 1000 MHz is what
        # litex-buildenv used successfully on this exact board).
        clk_freq_in = 1e9/platform.default_clk_period # = 125 MHz, PHY out of reset.
        sys_hz = Fraction(int(round(sys_clk_freq)))
        ref_hz = Fraction(int(round(clk_freq_in)))
        best   = None
        for q in range(1, 65):
            vco = 4*sys_hz*q
            if not (Fraction(400e6) <= vco <= Fraction(1000e6)):
                continue
            m_over_d = vco/ref_hz
            m, d = m_over_d.numerator, m_over_d.denominator
            if m > 256 or d > 128:  # CLKFBOUT_MULT / DIVCLK_DIVIDE ranges.
                continue
            if not (Fraction(19e6) <= ref_hz/d <= Fraction(500e6)): # PFD frequency.
                continue
            cost = abs(vco - Fraction(1000e6)) # Prefer the highest legal VCO (== proven config).
            if best is None or cost < best[0]:
                best = (cost, q, m, d, vco)
        if best is None:
            raise ValueError(
                f"Cannot synthesize sys_clk_freq={sys_clk_freq/1e6} MHz from a {clk_freq_in/1e6} MHz "
                f"reference: no VCO = 4*sys*q in [400, 1000] MHz reachable with M<=256, D<=128.")
        _, q, m, d, vco = best
        print("[CRG] {} MHz ref /{} x{} -> VCO {} MHz | sdram_full(4x) {} MHz /{} | "
              "sdram_half(2x) {} MHz /{} | sys {} MHz /{}".format(
            ref_hz/1e6, d, m, vco/1e6, 4*sys_hz/1e6, q, 2*sys_hz/1e6, 2*q, sys_hz/1e6, 4*q))

        # Input clock ------------------------------------------------------------------------------
        # BUFIO2 (bypassed) so that clk125 can feed the PLL *and* stay available for I/O use; the
        # PLL's CLKIN1 must come from BUFIO2's DIVCLK when the same clock drives DDR outputs. Feed it
        # from the pad directly (like saanlima_pipistrello): an explicit IBUFG here makes XST insert a
        # second input buffer on the pad and par then fails with ERROR:Pack:1107 (NOMERGE).
        clk125    = platform.request("clk125")
        clk125_b  = Signal()
        self.specials += Instance("BUFIO2",
            p_DIVIDE = 1, p_DIVIDE_BYPASS = "TRUE", p_I_INVERT = "FALSE",
            i_I = clk125, o_DIVCLK = clk125_b)

        # PLL --------------------------------------------------------------------------------------
        pll_lckd           = Signal()
        pll_fb             = Signal()
        pll_sdram_full     = Signal()
        pll_sdram_half_a   = Signal()
        pll_sdram_half_b   = Signal()
        pll_unused         = Signal()
        pll_sys            = Signal()

        self.specials.pll = Instance(
            "PLL_ADV",
            name = "crg_pll_adv",
            p_SIM_DEVICE = "SPARTAN6", p_BANDWIDTH = "OPTIMIZED", p_COMPENSATION = "INTERNAL",
            p_REF_JITTER = .01,
            i_DADDR=0, i_DCLK=0, i_DEN=0, i_DI=0, i_DWE=0, i_RST=0, i_REL=0,
            p_DIVCLK_DIVIDE = d,
            # Input clock (125 MHz, see above).
            i_CLKIN1 = clk125_b,
            p_CLKIN1_PERIOD = 1e9/clk_freq_in,
            i_CLKIN2 = 0,
            p_CLKIN2_PERIOD = 0.,
            i_CLKINSEL = 1,
            # Feedback.
            i_CLKFBIN = pll_fb, o_CLKFBOUT = pll_fb, o_LOCKED = pll_lckd,
            p_CLK_FEEDBACK = "CLKFBOUT",
            p_CLKFBOUT_MULT = m, p_CLKFBOUT_PHASE = 0.,
            # VCO/{} = 4x sys: OSERDES2/ISERDES2 clock (via BUFPLL).
            o_CLKOUT0 = pll_sdram_full, p_CLKOUT0_DUTY_CYCLE = .5,
            p_CLKOUT0_PHASE = 0., p_CLKOUT0_DIVIDE = q,
            # Unused (kept at the same frequency; was used for a video clock in earlier revisions).
            o_CLKOUT1 = pll_unused, p_CLKOUT1_DUTY_CYCLE = .5,
            p_CLKOUT1_PHASE = 0., p_CLKOUT1_DIVIDE = 2*q,
            # VCO/{} = 2x sys: sdram_half (DFI/cmd/DQS logic).
            o_CLKOUT2 = pll_sdram_half_a, p_CLKOUT2_DUTY_CYCLE = .5,
            p_CLKOUT2_PHASE = sdram_half_phase, p_CLKOUT2_DIVIDE = 2*q,
            # VCO/{} = 2x sys, phase-shifted: the SDRAM CK output driver.
            o_CLKOUT3 = pll_sdram_half_b, p_CLKOUT3_DUTY_CYCLE = .5,
            p_CLKOUT3_PHASE = sdram_clk_phase, p_CLKOUT3_DIVIDE = 2*q,
            # Unused.
            o_CLKOUT4 = pll_unused, p_CLKOUT4_DUTY_CYCLE = .5,
            p_CLKOUT4_PHASE = 0., p_CLKOUT4_DIVIDE = 2*q,
            # VCO/{} = sys.
            o_CLKOUT5 = pll_sys, p_CLKOUT5_DUTY_CYCLE = .5,
            p_CLKOUT5_PHASE = 0., p_CLKOUT5_DIVIDE = 4*q,
        )

        # Power on reset ---------------------------------------------------------------------------
        # The G2 has no usable front-panel reset for us, so the default is a boot counter only
        # (--cpu-reset-pin=user_btn_n|rst_n adds the pin if your setup really has one; a floating
        # input as reset would hold the SoC in reset forever).
        reset = self.rst
        if cpu_reset_pin is not None:
            reset = reset | ~platform.request(cpu_reset_pin)
        self.cd_por = ClockDomain()
        por = Signal(max=1 << 11, reset=(1 << 11) - 1)
        self.sync.por += If(por != 0, por.eq(por - 1))
        self.specials += AsyncResetSynchronizer(self.cd_por, reset)

        # System clock -----------------------------------------------------------------------------
        self.specials += Instance("BUFG", i_I=pll_sys, o_O=self.cd_sys.clk)
        self.comb += self.cd_por.clk.eq(self.cd_sys.clk)
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~pll_lckd | (por > 0))
        platform.add_period_constraint(self.cd_sys.clk, 1e9/sys_clk_freq)
        platform.add_false_path_constraints(self.cd_sys.clk, clk125)

        # SDRAM clocks -----------------------------------------------------------------------------
        self.clk4x_wr_strb = Signal()
        self.clk4x_rd_strb = Signal()

        # SDRAM full clock (4x sys): BUFPLL turns the PLL's 4x clock into the SERDES clock + strobe.
        self.specials += Instance("BUFPLL", name="sdram_full_bufpll",
            p_DIVIDE  = 4,
            i_PLLIN   = pll_sdram_full, i_GCLK=self.cd_sys.clk,
            i_LOCKED  = pll_lckd,
            o_IOCLK   = self.cd_sdram_full_wr.clk,
            o_SERDESSTROBE = self.clk4x_wr_strb)
        self.comb += [
            self.cd_sdram_full_rd.clk.eq(self.cd_sdram_full_wr.clk),
            self.clk4x_rd_strb.eq(self.clk4x_wr_strb),
        ]
        # SDRAM half clock (2x sys): one for the PHY logic, one phase-shifted for the CK driver.
        self.specials += Instance("BUFG", name="sdram_half_a_bufg",
            i_I=pll_sdram_half_a, o_O=self.cd_sdram_half.clk)
        clk_sdram_half_shifted = Signal()
        self.specials += Instance("BUFG", name="sdram_half_b_bufg",
            i_I=pll_sdram_half_b, o_O=clk_sdram_half_shifted)

        output_clk = Signal()
        clk = platform.request(f"ddram_clock_{sdram_chip}")
        self.specials += Instance("ODDR2", p_DDR_ALIGNMENT="NONE",
            p_INIT=0, p_SRTYPE="SYNC",
            i_D0=1, i_D1=0, i_S=0, i_R=0, i_CE=1,
            i_C0=clk_sdram_half_shifted,
            i_C1=~clk_sdram_half_shifted,
            o_Q=output_clk)
        self.specials += Instance("OBUFDS", i_I=output_clk, o_O=clk.p, o_OB=clk.n)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, revision="c", sys_clk_freq=50e6,
        with_ethernet      = False,
        with_etherbone     = False,
        eth_ip             = "192.168.1.50",
        remote_ip          = None,
        eth_dynamic_ip     = False,
        with_led_chaser    = True,
        sdram_chip         = "b",
        sdram_rd_bitslip   = 0,
        sdram_wr_bitslip   = 4,
        sdram_dqs_alignment = "C0",
        sdram_half_phase   = 270.,
        sdram_clk_phase    = 250.,
        cpu_reset_pin      = None,
        serial_index       = 0,
        **kwargs):
        platform = pano_logic_g2.Platform(revision=revision)
        assert sdram_chip in ["a", "b"]

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq,
            sdram_chip       = sdram_chip,
            sdram_half_phase = sdram_half_phase,
            sdram_clk_phase  = sdram_clk_phase,
            cpu_reset_pin    = cpu_reset_pin)

        # SoCCore ----------------------------------------------------------------------------------
        # serial_index: the console lives on a DDC pair, 0 = micro-HDMI, 1 = DVI.
        #
        # DDR bring-up builds must be cacheless: the L2 added below is a *write-back* cache
        # (litex/soc/interconnect/wishbone.py: "write-back wishbone cache") and so is VexRiscv's L1 D
        # cache, i.e. byte writes get absorbed and only full-line writebacks ever reach the DRAM's DM
        # pins -- a byte-lane/mask fault is then invisible to a CPU-side test. Use
        #   --cpu-variant=lite --l2-size=0
        # for the test bitstream (the BIOS executes from block RAM, so it still runs when the SDRAM is
        # dead); see tools/ddr_bytelane.py.
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident     = "LiteX SoC on Pano Logic G2",
            uart_pads = platform.request("serial", serial_index),
            **kwargs)

        # DDR2 SDRAM -------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            # Single chip: the two MT47H32M16 have independent addr/cmd pins, which is not the
            # multi-rank topology litedram expects (see header comment).
            self.ddrphy = s6ddrphy.S6HalfRateDDRPHY(platform.request(f"ddram_{sdram_chip}"),
                memtype           = "DDR2",
                rd_bitslip        = sdram_rd_bitslip,
                wr_bitslip        = sdram_wr_bitslip,
                dqs_ddr_alignment = sdram_dqs_alignment)
            self.comb += [
                self.ddrphy.clk4x_wr_strb.eq(self.crg.clk4x_wr_strb),
                self.ddrphy.clk4x_rd_strb.eq(self.crg.clk4x_rd_strb),
            ]
            self.add_sdram("sdram",
                phy               = self.ddrphy,
                module            = MT47H32M16(sys_clk_freq, "1:2"),
                controller_settings = ControllerSettings(with_bandwidth=True),
                l2_cache_size     = kwargs.get("l2_size", 8192),
            )

        # Ethernet / Etherbone ---------------------------------------------------------------------
        if with_ethernet or with_etherbone:
            self.ethphy = LiteEthPHY(
                clock_pads         = self.platform.request("eth_clocks"),
                pads               = self.platform.request("eth"),
                clk_freq           = sys_clk_freq,
                with_hw_init_reset = False) # PHY already taken out of reset by the CRG.
            if with_etherbone:
                self.add_etherbone(phy=self.ethphy, ip_address=eth_ip, with_ethmac=with_ethernet)
            if with_ethernet:
                self.add_ethernet(phy=self.ethphy, dynamic_ip=eth_dynamic_ip, local_ip=eth_ip, remote_ip=remote_ip)
            # The GMII clocks come from the PHY and go to a BUFG inside LiteEth's PHY CRG; tell the
            # tools the routing is not the dedicated one (same as the other Spartan-6 GMII targets).
            self.ethphy.crg.cd_eth_rx.clk.attr.add("keep")
            self.ethphy.crg.cd_eth_tx.clk.attr.add("keep")
            self.platform.add_platform_command("""
NET "{eth_clocks_rx}" CLOCK_DEDICATED_ROUTE = FALSE;
NET "{eth_clocks_tx}" CLOCK_DEDICATED_ROUTE = FALSE;
""",
                eth_clocks_rx = platform.lookup_request("eth_clocks").rx,
                eth_clocks_tx = platform.lookup_request("eth_clocks").tx,
            )

        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                sys_clk_freq = sys_clk_freq)

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=pano_logic_g2.Platform, description="LiteX SoC on Pano Logic G2.")
    parser.add_target_argument("--revision",      default="c",              help="Board revision (b or c).")
    parser.add_target_argument("--sys-clk-freq",  default=50e6, type=float, help="System clock frequency.")
    parser.add_target_argument("--prog",          default="openfpgaloader", help="Programmer (openfpgaloader | openocd).")
    parser.add_target_argument("--flash",         action="store_true",      help="Write bitstream to the SPI flash.")
    parser.add_target_argument("--flash-ext",     default=".bin",           help="File for --flash: .bin/.bit, or .mcs generated by impact (unambiguous byte order, see RESEARCH.md §1).")
    # DDR2.
    parser.add_target_argument("--sdram-chip",           default="b",          help="DDR2 chip to drive (a or b).")
    parser.add_target_argument("--sdram-rd-bitslip",     default=0,    type=int, help="S6HalfRateDDRPHY rd_bitslip.")
    parser.add_target_argument("--sdram-wr-bitslip",     default=4,    type=int, help="S6HalfRateDDRPHY wr_bitslip.")
    parser.add_target_argument("--sdram-dqs-alignment",  default="C0",         help="S6HalfRateDDRPHY dqs_ddr_alignment.")
    parser.add_target_argument("--sdram-half-phase",     default=270., type=float, help="PLL CLKOUT2 phase (sdram_half).")
    parser.add_target_argument("--sdram-clk-phase",      default=250., type=float, help="PLL CLKOUT3 phase (SDRAM CK driver).")
    # Console / reset.
    parser.add_target_argument("--serial-index",  default=0, type=int, help="Console DDC pair (0=micro-HDMI, 1=DVI).")
    parser.add_target_argument("--cpu-reset-pin", default=None,
        help="Also reset from this platform resource (active low, e.g. user_btn_n). Default: boot counter only.")
    ethopts = parser.target_group.add_mutually_exclusive_group()
    ethopts.add_argument("--with-ethernet",  action="store_true", help="Enable Ethernet support.")
    ethopts.add_argument("--with-etherbone", action="store_true", help="Enable Etherbone support.")
    parser.add_target_argument("--eth-ip",         default="192.168.1.50",  help="Ethernet/Etherbone IP address.")
    parser.add_target_argument("--eth-dynamic-ip", action="store_true",     help="Enable dynamic Ethernet IP assignment.")
    parser.add_target_argument("--remote-ip",      default="192.168.1.100", help="Remote IP address of TFTP server.")
    args = parser.parse_args()

    soc = BaseSoC(
        revision            = args.revision,
        sys_clk_freq        = args.sys_clk_freq,
        with_ethernet       = args.with_ethernet,
        with_etherbone      = args.with_etherbone,
        eth_ip              = args.eth_ip,
        remote_ip           = args.remote_ip,
        eth_dynamic_ip      = args.eth_dynamic_ip,
        sdram_chip          = args.sdram_chip,
        sdram_rd_bitslip    = args.sdram_rd_bitslip,
        sdram_wr_bitslip    = args.sdram_wr_bitslip,
        sdram_dqs_alignment = args.sdram_dqs_alignment,
        sdram_half_phase    = args.sdram_half_phase,
        sdram_clk_phase     = args.sdram_clk_phase,
        cpu_reset_pin       = args.cpu_reset_pin,
        serial_index        = args.serial_index,
        **parser.soc_argdict
    )
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer(name=args.prog)
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    if args.flash:
        prog = soc.platform.create_programmer(name=args.prog)
        prog.flash(0, builder.get_bitstream_filename(mode="flash", ext=args.flash_ext))

if __name__ == "__main__":
    main()
