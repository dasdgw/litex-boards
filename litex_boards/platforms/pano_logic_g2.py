#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2019 Tom Keddie <git@bronwenandtom.com>
# Copyright (c) 2020 Antmicro <www.antmicro.com>
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# The Pano Logic Zero Client G2 is a thin commercial client from Pano Logic that can be repurposed
# as a generic development board thanks to reverse engineering efforts than can be found at:
# https://github.com/tomverbeure/panologic-g2

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxSpartan6Platform
from litex.build.openocd import OpenOCD
from litex.build.openfpgaloader import OpenFPGALoader

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Clk / Rst
    ("clk125",    0, Pins("Y13"),  IOStandard("LVCMOS33")),
    ("rst_n",     0, Pins("AB14"), IOStandard("LVCMOS33")),

    # Leds
    ("user_led", 0, Pins("E12"),  IOStandard("LVCMOS33")),
    ("user_led", 1, Pins("H13"),  IOStandard("LVCMOS33")),
    ("user_led", 2, Pins("F13"),  IOStandard("LVCMOS33")),

    # Buttons
    ("user_btn_n", 0, Pins("H12"), IOStandard("LVCMOS33")),

    # Serial
    ("serial", 0, # hdmi
        Subsignal("tx", Pins("AB19")),
        Subsignal("rx", Pins("AA21")),
        IOStandard("LVCMOS33")
    ),

    # Serial
    ("serial", 1, # dvi
        Subsignal("tx", Pins("C14")),
        Subsignal("rx", Pins("C17")),
        IOStandard("LVCMOS33")
    ),

    # USB ULPI (SMSC/USB3300 PHY -> USB2514 hub -> 4x USB-A)
    #
    # Ported from the 2023-era fork branch `legacy-2023` of this file: this group exists *there* and
    # nowhere else (not on `pano-linux`, not upstream), and it is the only handle we have on the PHY.
    #
    # DATA BIT ORDER -- a known suspect, checked rather than trusted, and it is CORRECT:
    #   LiteX `Pins("A7 B8 A8 D6 C6 B6 A6 A4")` assigns by index, i.e. data[0]=A7 ... data[7]=A4.
    #   tomverbeure's `Pano.ucf` (table reproduced in RESEARCH.md, "confirmed identical in
    #   litex-buildenv") lists ULPI_DATA[0]=A7, [1]=B8, [2]=A8, [3]=D6, [4]=C6, [5]=B6, [6]=A6,
    #   [7]=A4 -- the *same* mapping read in the opposite textual order. The two sources agree; do
    #   not "fix" this by reversing the list, that would be the actual bug. If ULPI traffic ever
    #   shows a bit-permuted byte, re-derive from the UCF, do not start here.
    #
    # Reset polarity on `reset` (net USB3300_RESET) is NOT settled: ULPI RESET is active-high by
    # spec, tomverbeure drives `io.ulpi.reset := False` for "not in reset", and the 2023 core wrapper
    # held pads.reset at a constant 0 with a `# todo remove`. It is a CSR polarity bit upstream, so
    # the board decides; nothing here bakes in a direction.
    ("ulpi", 0,
        Subsignal("clk",   Pins("C12")),  # 60 MHz FROM the PHY (input): present == PHY alive
        Subsignal("stp",   Pins("A5")),
        Subsignal("dir",   Pins("C7")),
        Subsignal("nxt",   Pins("C5")),
        Subsignal("reset", Pins("C9")),
        Subsignal("data",  Pins("A7 B8 A8 D6 C6 B6 A6 A4")),
        IOStandard("LVCMOS33")
    ),

    # USB reference clock: the 24 MHz the USB3300 (and per tomverbeure the USB2514 hub too) needs to
    # start. Direction is UNPROVEN -- `USB_CLK` appears in Pano.ucf only as a commented-out pin under
    # "USB Clock Generation", and the board's only oscillator (25 MHz) feeds the Marvell PHY, not USB.
    # Whether the FPGA must drive it is TODO (a); treat every use as measure-before-drive.
    ("usb_refclk", 0, Pins("W12"), IOStandard("LVCMOS33")),

    # Hub reset (net USB_RST_N, the on-board USB2514). Independent of the PHY reset above; the two
    # are never driven as one signal.
    ("usb_hub_rst_n", 0, Pins("W11"), IOStandard("LVCMOS33")),

    # SPIFlash
    ("spiflash", 0,
        Subsignal("cs_n", Pins("T5"),   IOStandard("LVCMOS33")),
        Subsignal("clk",  Pins("Y21"),  IOStandard("LVCMOS33")),
        Subsignal("mosi", Pins("AB20"), IOStandard("LVCMOS33")),
        Subsignal("miso", Pins("AA20"), IOStandard("LVCMOS33"))
    ),

    # DDR2 SDRAM
    #
    # ⚠ BYTE LANES: `dq` is listed low-byte-first (dq0-7 = first 8 pins = the *lower* physical byte
    # lane of the chip) while `dm`/`dqs`/`dqs_n` were historically listed upper-lane-first, and
    # litedram pairs these lists **by index** -- so lane 0 used lane 1's strobe/mask. Audited against
    # tomverbeure's `Pano.ucf` (see RESEARCH.md §3 of the pano project notes): the order below is the
    # corrected one (low lane first everywhere). Invisible to command training and to a word-wise
    # memtest; corrupts byte/masked accesses. Do not "restore" the old order.
    ("ddram_clock_a", 0,
        Subsignal("p", Pins("H20")),
        Subsignal("n", Pins("J19")),
        IOStandard("DIFF_SSTL18_II"), Misc("IN_TERM=NONE")
    ),
    ("ddram_a", 0,
        Subsignal("a", Pins(
            "F21 F22 E22 G20 F20 K20 K19 E20",
            "C20 C22 G19 F19 D22"),
            IOStandard("SSTL18_II")),
        Subsignal("ba",    Pins("J17 K17 H18"), IOStandard("SSTL18_II")),
        Subsignal("ras_n", Pins("H21"), IOStandard("SSTL18_II")),
        Subsignal("cas_n", Pins("H22"), IOStandard("SSTL18_II")),
        Subsignal("we_n",  Pins("H19"), IOStandard("SSTL18_II")),
        Subsignal("dm",    Pins("L19 M20"), IOStandard("SSTL18_II")),
        Subsignal("dq",    Pins(
            "N20 N22 M21 M22 J20 J22 K21 K22",
            "P21 P22 R20 R22 U20 U22 V21 V22"),
            IOStandard("SSTL18_II")),
        Subsignal("dqs",   Pins("L20 T21"), IOStandard("DIFF_SSTL18_II")),
        Subsignal("dqs_n", Pins("L22 T22"), IOStandard("DIFF_SSTL18_II")),
        Subsignal("cke", Pins("D21"), IOStandard("SSTL18_II")),
        Subsignal("odt", Pins("G22"), IOStandard("SSTL18_II")),
    ),
    ("ddram_clock_b", 0,
        Subsignal("p", Pins("H4")),
        Subsignal("n", Pins("H3")),
        IOStandard("DIFF_SSTL18_II"), Misc("IN_TERM=NONE")
    ),
    ("ddram_b", 0,
        Subsignal("a", Pins(
            "H2 H1 H5 K6 F3 K3 J4 H6",
            "E3 E1 G4 C1 D1"),
            IOStandard("SSTL18_II")),
        Subsignal("ba",    Pins("G3 G1 F1"), IOStandard("SSTL18_II")),
        Subsignal("ras_n", Pins("K5"), IOStandard("SSTL18_II")),
        Subsignal("cas_n", Pins("K4"), IOStandard("SSTL18_II")),
        Subsignal("we_n",  Pins("F2"), IOStandard("SSTL18_II")),
        Subsignal("dm",    Pins("L4 M3"), IOStandard("SSTL18_II")),
        Subsignal("dq",    Pins(
            "N3 N1 M2 M1 J3 J1 K2 K1",
            "P2 P1 R3 R1 U3 U1 V2 V1"),
            IOStandard("SSTL18_II")),
        Subsignal("dqs",   Pins("L3 T2"), IOStandard("DIFF_SSTL18_II")),
        Subsignal("dqs_n", Pins("L1 T1"), IOStandard("DIFF_SSTL18_II")),
        Subsignal("cke", Pins("D2"), IOStandard("SSTL18_II")),
        Subsignal("odt", Pins("J6"), IOStandard("SSTL18_II")),
    ),

    # GMII Ethernet
    ("eth_rst_n",  0, Pins("R11"), IOStandard("LVCMOS33")),
    ("eth_clocks", 0,
        Subsignal("tx",  Pins("Y11")),
        Subsignal("gtx", Pins("AA12")),
        Subsignal("rx",  Pins("AB11")),
        IOStandard("LVCMOS33")
    ),
    ("eth", 0,
        # rst_n is the standalone `eth_rst_n` resource above (the PHY's reset is not a LiteEth signal:
        # liteeth never drives it, and the CRG must own it because the PHY also generates clk125).
        # Keeping it here too would place two PADs on R11 -> ERROR:Pack:2811 at par.
        Subsignal("int_n",   Pins("AA4")),
        Subsignal("mdio",    Pins("AA2")),
        Subsignal("mdc",     Pins("AB6")),
        Subsignal("rx_dv",   Pins("Y7")),
        Subsignal("rx_er",   Pins("Y8")),
        Subsignal("rx_data", Pins("Y3 Y4 R9 R7 V9 R8 U9 Y9")),
        Subsignal("tx_en",   Pins("AA8")),
        Subsignal("tx_er",   Pins("AB8")),
        Subsignal("tx_data", Pins("AB2 AB3 AB4 AB7 AB9 AB10 T7 Y10")),
        Subsignal("col",     Pins("V7")),
        Subsignal("crs",     Pins("W4")),
        IOStandard("LVCMOS33")
    ),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(XilinxSpartan6Platform):
    default_clk_name   = "clk125"
    default_clk_period = 1e9/125e6

    # openFPGALoader "fpga part" = device + package *without* speed grade. The Pano has no entry in
    # openFPGALoader's board list, so --fpga-part is mandatory; it also selects the spiOverJtag
    # (JTAG -> SPI) bridge it ships, and spiOverJtag_xc6slx100fgg484.bit.gz exists for rev C -- that
    # is why openFPGALoader is the default here: SPI flash access needs no external proxy bitstream
    # (bscan_spi_xc6slx100.bit has to be downloaded/built for both openocd and xc3sprog).
    # Rev B caveat: openFPGALoader knows only the *T* LX150 (IDCODE 0x4403d093) and ships bridges
    # for xc6slx150t{fgg484,csg484} => on rev B use --prog=openocd or build the bridge yourself
    # (spiOverJtag/build.py in that repo).
    ofl_parts = {"b": "xc6slx150fgg484", "c": "xc6slx100fgg484"}

    def __init__(self, revision="c", toolchain="ise"):
        assert revision in ["b", "c"]
        self.revision = revision
        device = {"b": "xc6slx150-2-fgg484", "c": "xc6slx100-2-fgg484"}[revision]
        XilinxSpartan6Platform.__init__(self, device, _io, toolchain=toolchain)
        # Do not remove: the ULPI bank's VCCAUX is 2.5 V on this board, which is also what openFPGALoader's
        # own spiOverJtag proxy bitfiles for xc6slx100fgg484 are built at. (The ULPI *IOs* are LVCMOS33 --
        # tomverbeure's UCF -- and that pairing is unverified by meter; see the ULPI Probe failure tree.)
        self.add_platform_command("""CONFIG VCCAUX="2.5";""")
        self.add_period_constraint(self.lookup_request("clk125", loose=True), 1e9/125e6)
        # NOTE for whoever adds the USB logic: `ulpi_clk` (C12, 60 MHz from the PHY) is constrained in the
        # *SoC*, with the `cd_usb` domain that buffers it -- not here. `lookup_request()` only resolves
        # resources a design has already *requested* (it walks `matched`, not `available`), so a
        # `add_period_constraint(lookup_request("ulpi_clk", loose=True), ...)` here silently resolves to
        # None and returns without constraining anything (measured 2026-10-30: it prints None on this
        # platform even with the group present).

    def create_programmer(self, name="openfpgaloader"):
        # digilent_hs2 = 0x0403:0x6014 interface A == xc3sprog's `jtaghs2` cable. 6 MHz is
        # openFPGALoader's own default (DEFAULT_FREQ); lower it if the hand-soldered adapter is sad.
        if name == "openfpgaloader":
            return OpenFPGALoader(
                cable     = "digilent_hs2",
                fpga_part = self.ofl_parts[self.revision],
                freq      = 6e6,
            )
        elif name == "openocd":
            return OpenOCD("openocd_xc6_ft232.cfg")
        else:
            raise ValueError(f"Unsupported programmer: {name}")
