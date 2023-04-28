#!/usr/bin/env python3

# python3 -m pano_logic_g2 --csr-csv=csr.csv --sys-clk-freq 125e6 --with-video-terminal --video-timing=1920x1080@30Hz --with-ethernet --build
#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
from fractions import Fraction

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex_boards.platforms import pano_logic_g2

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.video import VideoGenericPHY
from litex.soc.cores.video import video_timings
from litex.soc.cores.bitbang import I2CMaster
from litex.soc.cores.led import LedChaser

from liteeth.phy import LiteEthPHY

from litedram.modules import MT47H32M16
from litedram.phy import s6ddrphy

from litescope import LiteScopeAnalyzer

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_ethernet=False, video_timing="800x600@75Hz"):
        self.rst     = Signal()
        self.cd_sys  = ClockDomain()
        self.cd_hdmi = ClockDomain()
        self.cd_hdmi_90 = ClockDomain()

        self.cd_sdram_half    = ClockDomain()
        self.cd_sdram_full_wr = ClockDomain()
        self.cd_sdram_full_rd = ClockDomain()

#        if not with_ethernet:
            # Take Ethernet PHY out of reset to enable 125MHz on clk125 (25MHz otherwise).
            # See https://github.com/tomverbeure/panologic-g2#fpga-external-clocking-architecture
        # always keep phy out of reset
        self.comb += platform.request("eth_rst_n").eq(1)

        self.reset = Signal()

        # # #

        # Input clock ------------------------------------------------------------------------------
        clk125_freq = int(125e6)
        clk125      = platform.request("clk125")
        clk125a = Signal()
        self.specials += Instance("IBUFG", i_I=clk125, o_O=clk125a)

        clk125b     = Signal()
        self.specials += Instance("BUFIO2",
            p_DIVIDE=1, p_DIVIDE_BYPASS="TRUE",
            p_I_INVERT="FALSE",
#            i_I=clk125, o_DIVCLK=clk125b)
            i_I=clk125a, o_DIVCLK=clk125b)

        # PLL --------------------------------------------------------------------------------------
        pll_lckd           = Signal()
        pll_fb             = Signal()
        pll_sdram_full   = Signal()
        pll_sdram_half_a = Signal()
        pll_sdram_half_b = Signal()
        pll_unused       = Signal()
        pll_sys          = Signal()
        pll_periph       = Signal()

        p = 12
        #p = 8
        f = Fraction(int(sys_clk_freq*p), clk125_freq)
        n, d = f.numerator, f.denominator
        print(f'pll fractions n:{n}, d:{d}')
        assert 19e6 <= clk125_freq/d <= 500e6  # pfd
        assert 400e6 <= clk125_freq*n/d <= 1080e6  # vco

        self.specials.pll = Instance(
            "PLL_ADV",
            name="crg_pll_adv",
            p_SIM_DEVICE="SPARTAN6", p_BANDWIDTH="OPTIMIZED", p_COMPENSATION="INTERNAL",
            p_REF_JITTER=.01,
            i_DADDR=0, i_DCLK=0, i_DEN=0, i_DI=0, i_DWE=0, i_RST=0, i_REL=0,
            p_DIVCLK_DIVIDE=d,
            # Input Clocks (125MHz)
            i_CLKIN1=clk125b,
            p_CLKIN1_PERIOD=1e9/clk125_freq,
            i_CLKIN2=0,
            p_CLKIN2_PERIOD=0.,
            i_CLKINSEL=1,
            # Feedback
            i_CLKFBIN=pll_fb, o_CLKFBOUT=pll_fb, o_LOCKED=pll_lckd,
            p_CLK_FEEDBACK="CLKFBOUT",
            p_CLKFBOUT_MULT=n, p_CLKFBOUT_PHASE=0.,
            # (333MHz) sdram wr rd
            o_CLKOUT0=pll_sdram_full, p_CLKOUT0_DUTY_CYCLE=.5,
            p_CLKOUT0_PHASE=0., p_CLKOUT0_DIVIDE=p//4,
            # ddr2_a
            o_CLKOUT1=pll_sdram_half_b_a, p_CLKOUT1_DUTY_CYCLE=.5,
            p_CLKOUT1_PHASE=257., p_CLKOUT1_DIVIDE=p//2,
            # (166MHz) sdram_half - sdram dqs adr ctrl
            o_CLKOUT2=pll_sdram_half_a, p_CLKOUT2_DUTY_CYCLE=.5,
            p_CLKOUT2_PHASE=270., p_CLKOUT2_DIVIDE=p//2,
            # (166MHz) off-chip ddr
            o_CLKOUT3=pll_sdram_half_b, p_CLKOUT3_DUTY_CYCLE=.5,
            #p_CLKOUT3_PHASE=250., p_CLKOUT3_DIVIDE=p//2,
            # no errors with this phase
            p_CLKOUT3_PHASE=255., p_CLKOUT3_DIVIDE=p//2,
            # ( 50MHz) periph
            o_CLKOUT4=pll_periph, p_CLKOUT4_DUTY_CYCLE=.5,
            p_CLKOUT4_PHASE=0., p_CLKOUT4_DIVIDE=20,
            # ( 83MHz) sysclk
            o_CLKOUT5=pll_sys, p_CLKOUT5_DUTY_CYCLE=.5,
            p_CLKOUT5_PHASE=0., p_CLKOUT5_DIVIDE=p//1,
        )

        # Power on reset
        reset = ~platform.request("user_btn_n") | self.reset | self.rst
        self.cd_por = ClockDomain()
        por = Signal(max=1 << 11, reset=(1 << 11) - 1)
        self.sync.por += If(por != 0, por.eq(por - 1))
        self.specials += AsyncResetSynchronizer(self.cd_por, reset)

        # System clock
        self.specials += Instance("BUFG", i_I=pll_sys, o_O=self.cd_sys.clk)
        self.comb += self.cd_por.clk.eq(self.cd_sys.clk)
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~pll_lckd | (por > 0))
        platform.add_period_constraint(self.cd_sys.clk, 1e9/sys_clk_freq)

        # SDRAM clocks -----------------------------------------------------------------------------
        self.clk4x_wr_strb = Signal()
        self.clk4x_rd_strb = Signal()

        # SDRAM full clock
        self.specials += Instance("BUFPLL", name="sdram_full_bufpll",
            p_DIVIDE       = 4,
            i_PLLIN        = pll_sdram_full, i_GCLK=self.cd_sys.clk,
            i_LOCKED       = pll_lckd,
            o_IOCLK        = self.cd_sdram_full_wr.clk,
            o_SERDESSTROBE = self.clk4x_wr_strb)
        self.comb += [
            self.cd_sdram_full_rd.clk.eq(self.cd_sdram_full_wr.clk),
            self.clk4x_rd_strb.eq(self.clk4x_wr_strb),
        ]
        # SDRAM_half clock
        self.specials += Instance("BUFG", name="sdram_half_a_bufpll",
            i_I=pll_sdram_half_a, o_O=self.cd_sdram_half.clk)
        clk_sdram_half_shifted = Signal()
        self.specials += Instance("BUFG", name="sdram_half_b_bufpll",
            i_I=pll_sdram_half_b, o_O=clk_sdram_half_shifted)

        output_clk = Signal()
        clk = platform.request("ddram_clock_b")
        self.specials += Instance("ODDR2", p_DDR_ALIGNMENT="NONE",
            p_INIT=0, p_SRTYPE="SYNC",
            i_D0=1, i_D1=0, i_S=0, i_R=0, i_CE=1,
            i_C0=clk_sdram_half_shifted,
            i_C1=~clk_sdram_half_shifted,
            o_Q=output_clk)
        self.specials += Instance("OBUFDS", i_I=output_clk, o_O=clk.p, o_OB=clk.n)



        # video -----------------------------------------------
        self.video_pll = video_pll = S6PLL(speedgrade=-2)
        video_pll.register_clkin(clk125b, 125e6)
        video_pix_clk = video_timings[video_timing]['pix_clk']
        video_pll.create_clkout(self.cd_hdmi, video_pix_clk,  margin=0.1)
        video_pll.create_clkout(self.cd_hdmi_90, video_pix_clk,  margin=0.1, phase=90)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, revision="c", sys_clk_freq=50e6,
        with_ethernet          = False,
        with_etherbone         = False,
        eth_ip                 = "192.168.1.50",
        with_led_chaser        = True,
        with_video_terminal    = False,
        with_video_framebuffer = False,
        with_video_colorbars   = False,
        video_timing           = "800x600@75Hz",
        **kwargs):
        platform = pano_logic_g2.Platform(revision=revision)
        if with_etherbone:
            sys_clk_freq = int(125e6)

        # CRG --------------------------------------------------------------------------------------
        with_ethernet = (with_ethernet or with_etherbone)
        self.crg = _CRG(platform, sys_clk_freq, with_ethernet=with_ethernet, video_timing=video_timing)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on Pano Logic G2", **kwargs)

        # Ethernet / Etherbone ---------------------------------------------------------------------
        if with_ethernet or with_etherbone:
            self.ethphy = LiteEthPHY(
                clock_pads         = self.platform.request("eth_clocks"),
                pads               = self.platform.request("eth"),
                clk_freq           = sys_clk_freq,
                with_hw_init_reset = False)
            if with_ethernet:
                self.add_ethernet(phy=self.ethphy)
            if with_etherbone:
                self.add_etherbone(phy=self.ethphy, ip_address=eth_ip)

        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                sys_clk_freq = sys_clk_freq)

        # DDR2 SDRAM -------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            self.ddrphy = s6ddrphy.S6HalfRateDDRPHY(platform.request("ddram_b"),
                memtype           = "DDR2",
                rd_bitslip        = 0,
                wr_bitslip        = 4,
                dqs_ddr_alignment = "C0")
            self.comb += [
                self.ddrphy.clk4x_wr_strb.eq(self.crg.clk4x_wr_strb),
                self.ddrphy.clk4x_rd_strb.eq(self.crg.clk4x_rd_strb),
            ]
            self.add_sdram("sdram",
                phy           = self.ddrphy,
                module        = MT47H32M16(sys_clk_freq, "1:2"),
                l2_cache_size = kwargs.get("l2_size", 8192),
            )

        #analyzer_signals = []
        # HDMI Options -----------------------------------------------------------------------------
        #output_mode = "chrontel_idf_0"
        if (with_video_colorbars or with_video_framebuffer or with_video_terminal):
            hdmi_pads = platform.request("hdmi")
            self.videophy = VideoGenericPHY(hdmi_pads, clock_domain="hdmi", output_mode = "chrontel_idf_0", clk_clock_domain="hdmi_90")
            #analyzer_signals += [ self.videophy.sink ]
            videoi2c_pads = platform.request("videoi2c")
            self.videoi2c = I2CMaster(videoi2c_pads)
            # https://github.com/tomverbeure/panologic-g2/blob/master/spinal/sw/dvi.c
            # hdmi: 0x75 dvi: 0x76
            self.videoi2c.add_init(addr=0x75, init=[
            #self.videoi2c.add_init(addr=0x76, init=[
                (0x1c, 0x00),              # 1x DDR clocking
                (0x1d, 0x40),              # internal clock delay
                (0x1f, 0x80),              # input data format. Bit 4: vsp, bit 4: hsp
                (0x21, 0x09),              # Enable VGA
                # clk <= 65 MHz
                (0x33, 0x08),              # charge pump settings. See table 10
                (0x34, 0x16),              # charge pump settings. See table 10
                (0x36, 0x60),              # charge pump settings. See table 10
                # clk > 65 MHz
                #(0x33, 0x06),              # charge pump settings. See table 10
                #(0x34, 0x26),              # charge pump settings. See table 10
                #(0x36, 0xa0),              # charge pump settings. See table 10
                (0x48, 0x18),              # Test Image: none
               #(0x48, 0x19),              # Test Image: color bars
               #(0x48, 0x1e),              # Test Image: luminance ramp
                (0x49, 0xc0),              # DVI on
                (0xff, 0xff)               # The end
            ])

            if with_video_colorbars:
                #self.add_video_colorbars(phy=self.videophy, timings="640x480@60Hz", clock_domain="hdmi")
                self.add_video_colorbars(phy=self.videophy, timings=video_timing, clock_domain="hdmi")
            if with_video_terminal:
                #self.add_video_terminal(phy=self.videophy, timings="640x480@60Hz", clock_domain="hdmi")
                self.add_video_terminal(phy=self.videophy, timings=video_timing, clock_domain="hdmi")
            if with_video_framebuffer:
                #self.add_video_framebuffer(phy=self.videophy, timings="640x480@60Hz", clock_domain="hdmi")
                self.add_video_framebuffer(phy=self.videophy, timings=video_timing, clock_domain="hdmi")

        platform.add_period_constraint(self.crg.cd_sys.clk, 1e9/sys_clk_freq)
        #platform.add_period_constraint(self.crg.cd_sdram_full.clk, 1e9/(4 * sys_clk_freq))
        video_pix_clk = video_timings[video_timing]['pix_clk']
        print(f"{video_timing} / {video_timings[video_timing]['pix_clk']/1e6:3.2f}MHz.")
        platform.add_period_constraint(self.crg.cd_hdmi.clk, 1e9/video_pix_clk)
        platform.add_period_constraint(self.crg.cd_hdmi_90.clk, 1e9/video_pix_clk)

        # LiteScope Analyzer -----------------------------------------------------------------------
        #self.hdmi_sigs = Signal(1)
        #self.comb += self.hdmi_sigs.eq(platform.request("eth_rst_n"))
        #self.comb += self.hdmi_sigs.eq(self.crg.cd_hdmi.clk)
        # videophy.comb.clk

        #count = Signal(8)
        #self.sync += count.eq(count + 1)
        #analyzer_signals += [
        #    #self.crg.cd_hdmi.clk,
        #    #self.videophy.sink,
        #    #self.
        #    count,
        #]
        #self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals,
        #    depth        = 1024,
        #    clock_domain = "sys",
        #    samplerate   = self.sys_clk_freq,
        #    csr_csv      = "analyzer.csv",
        #    register     = True)
        #self.add_csr("analyzer")

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=pano_logic_g2.Platform, description="LiteX SoC on Pano Logic G2.")
    parser.add_target_argument("--revision",        default="c",              help="Board revision (b or c).")
    parser.add_target_argument("--sys-clk-freq",    default=50e6, type=float, help="System clock frequency.")
    ethopts = parser.target_group.add_mutually_exclusive_group()
    ethopts.add_argument("--with-ethernet",  action="store_true",    help="Enable Ethernet support.")
    ethopts.add_argument("--with-etherbone", action="store_true",    help="Enable Etherbone support.")
    parser.add_target_argument("--eth-ip",   default="192.168.1.50", help="Ethernet/Etherbone IP address.")
    viopts = parser.target_group.add_mutually_exclusive_group()
    viopts.add_argument("--with-video-terminal",    action="store_true", help="Enable Video Terminal (HDMI).")
    viopts.add_argument("--with-video-framebuffer", action="store_true", help="Enable Video Framebuffer (HDMI).")
    viopts.add_argument("--with-video-colorbars",   action="store_true", help="Enable Video Colorbars (HDMI).")
    parser.add_target_argument("--video-timing",  default="800x600@75Hz",help="Video resolution and refresh rate")
    #for video_timing in video_timings.keys():
    #    print(f" - {video_timing} / {video_timings[video_timing]['pix_clk']/1e6:3.2f}MHz.")

    args = parser.parse_args()

    soc = BaseSoC(
        revision               = args.revision,
        sys_clk_freq           = args.sys_clk_freq,
        with_ethernet          = args.with_ethernet,
        with_etherbone         = args.with_etherbone,
        eth_ip                 = args.eth_ip,
        with_video_terminal    = args.with_video_terminal,
        with_video_framebuffer = args.with_video_framebuffer,
        with_video_colorbars   = args.with_video_colorbars,
        video_timing           = args.video_timing,
        **parser.soc_argdict
    )
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
