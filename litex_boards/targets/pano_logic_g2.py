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
from litex.soc.cores.freqmeter import FreqMeter

from liteeth.phy import LiteEthPHY

from litedram.modules import MT47H32M16
from litedram.phy import s6ddrphy
from litedram.core import ControllerSettings
from litex.soc.interconnect.csr import CSRStatus, CSRStorage
from migen.genlib.cdc import MultiReg
from migen.fhdl.specials import Tristate

from litex_boards.extensions.pano_ulpi_csr import UlpiCsr
from litex_boards.extensions.pano_ulpi_scope import UlpiUsbScope

# ULPI Probe ---------------------------------------------------------------------------------------


# Board skew, in ns, added to BOTH directions of the ULPI constraint. Sourced, not invented: Microchip
# AN19.17 (ULPI Design Guide) §3 asks for 50-60 ohm controlled-impedance traces, each under 3 inches, "routed
# to within 0.5 inches of CLKOUT", which at ~150-180 ps/in on FR-4 is ~0.1 ns between CLKOUT and a data line.
# We do NOT know that the Pano's layout meets that -- there are no layout files and it is a 2009 thin client,
# not a reference design -- so this allowance is the reference rule plus a factor rather than the rule itself.
# The honest correction is to measure it: drive a square wave on one data line, probe CLKOUT and that line,
# pass the number to --ulpi-skew-ns. 0.0 would claim matched traces; 0.3 claims nothing we have not seen.
#
# It lives at module scope, not as a class attribute, because `get_argparser()` runs before the SoC class is
# defined at import time -- a `SocPanoLogicG2.ULPI_SKEW_NS_DEFAULT` default is a NameError at --help.
ULPI_SKEW_NS_DEFAULT = 0.3


class RefclkPLL(LiteXModule):
    """24.000 MHz for the USB PHY's reference clock, derived from `cd_sys` in a PLL of its own.

    **Why not a spare output of the CRG's PLL:** the CRG deliberately takes the *highest legal VCO*
    (1000 MHz on this -2 part, which is also what litex-buildenv proved on this exact board) because the
    DRAM PHY needs 4x and 2x taps of it -- and 1000/24 is not an integer, so no divider on that VCO can
    make 24 MHz. From the 50 MHz sys clock: VCO = 50x12 = 600 MHz, /25 = **exactly 24 MHz**, PFD 50 MHz
    (inside the 19-500 window). The accuracy is inherited from the Marvell-PHY oscillator that the 125 MHz
    GbE reference also comes from, which is well inside USB's +-500 ppm.

    Fed from `cd_sys` (already BUFG'd) instead of the CRG's BUFIO2 so that this PLL cannot perturb the DRAM
    clock tree: adding a load there is the one change in this milestone that could break memory we already
    proved working. If par objects to a BUFG driving PLL_ADV's CLKIN1, the fallback is `clk125_b`.
    """

    def __init__(self, ref_freq = 50e6, out_freq = 24e6):
        # Solved, not hardcoded: a build at another sys_clk_freq must fail here with a clear message rather
        # than quietly emit 24.something MHz to a PHY that expects 24.
        best = None
        for m in range(1, 257):
            vco = ref_freq*m
            if not (400e6 <= vco <= 1000e6):
                continue
            n = vco/out_freq
            if abs(n - round(n)) > 1e-9 or not (1 <= n <= 128):
                continue
            if best is None or m < best[0]:
                best = (m, int(round(n)), vco)
        if best is None:
            raise ValueError(
                f"Cannot synthesize {out_freq/1e6} MHz from {ref_freq/1e6} MHz on a Spartan-6 PLL: no VCO "
                f"in [400, 1000] MHz is an integer multiple of the target with N<=128.")
        m, n, vco = best
        print("[refclk] {} MHz x{} -> VCO {} MHz -> {} MHz (N={})".format(
            ref_freq/1e6, m, vco/1e6, out_freq/1e6, n))

        self.clk    = Signal()
        self.lock   = Signal()
        # **A declared clock domain, not just a wire, and that is the whole Score-173 story.**
        #
        # `ulpi_t08` failed with `Timing Score: 173 (Setup: 173)`. trce named exactly one endpoint:
        # `keep_timer<6> (cd_sys) -> ODDR2_6` (the pad's tri-state cell), **requirement 0.833 ns** --
        # XST had auto-derived `TS_refclk_pll_clk = TSsys_clk / 0.48` and therefore analysed every
        # cd_sys -> refclk path as a *synchronous* path with a sub-nanosecond requirement. Nothing was
        # wrong with the logic; the relationship between the two clocks had simply never been declared,
        # so the tool guessed the wrong one. Proven minimally: a 60-line design (PLL + keep-timer + the
        # two ODDR2s) reproduces it as Score 1509 with the identical endpoint, and goes green on the
        # same netlist with only LiteX's `add_false_path_constraints()` added
        # (`~/fpga/forkbuild/repro_w12/loop.sh`, 18 s per run).
        #
        # Declared here rather than patched with a hand-written TIG so that the constraint and the logic
        # that relies on it live together, exactly like `cd_usb` below (same file, same reason).
        self.cd_refclk = ClockDomain(reset_less=True)   # reset-less: the ODDR2s use S/R tied to 0 and
                                                        # MultiReg's flops are reset_less by construction.
        self.out_freq  = out_freq
        fb        = Signal()
        spare     = Signal()
        self.specials += Instance("PLL_ADV",
            p_SIM_DEVICE = "SPARTAN6", p_BANDWIDTH = "OPTIMIZED",
            p_COMPENSATION = "INTERNAL", p_REF_JITTER = .01,
            i_DADDR = 0, i_DCLK = 0, i_DEN = 0, i_DI = 0, i_DWE = 0, i_RST = 0, i_REL = 0,
            i_CLKIN1 = ClockSignal("sys"), p_CLKIN1_PERIOD = 1e9/ref_freq,
            i_CLKIN2 = 0, p_CLKIN2_PERIOD = 0., i_CLKINSEL = 1,
            p_DIVCLK_DIVIDE = 1,
            i_CLKFBIN = fb, o_CLKFBOUT = fb, o_LOCKED = self.lock,
            p_CLK_FEEDBACK = "CLKFBOUT", p_CLKFBOUT_MULT = m, p_CLKFBOUT_PHASE = 0.,
            o_CLKOUT0 = self.clk, p_CLKOUT0_DUTY_CYCLE = .5,
            p_CLKOUT0_PHASE = 0., p_CLKOUT0_DIVIDE = n,
            # Spares, kept the way the CRG keeps its own.
            o_CLKOUT1 = spare, p_CLKOUT1_DUTY_CYCLE = .5, p_CLKOUT1_PHASE = 0., p_CLKOUT1_DIVIDE = n,
        )
        # A PLL output that drives an IO goes through a global buffer: without it the pad clock takes
        # ordinary routing, and asking a PHY to accept unknown skew on its reference input is not a
        # measurement, it is a coin flip.
        self.specials += Instance("BUFG", i_I = self.clk, o_O = self.cd_refclk.clk)


class ULPIProbe(LiteXModule):
    """The instrument half of the ULPI Probe: count clocks over a **gate window**, and prove the meter
    is not lying before anything it says about the PHY is believed.

    Everything is in `cd_sys`. `cd_usb` cannot hold state because the PHY's clock stops when the PHY is
    in reset, so a counter there would freeze exactly when the answer is "no clock".

    Registers are **edges per gate window**, not Hz: with the default 1 ms window the numbers read as
    Hz/1000 (60 MHz -> ~60 000). Keep the gate in one place (`gate_ms`) so the harness can divide.

    The self-test channel measures `cd_sdram_half` -- 2x sys, a clock this design already generates and
    that is *not* on the far side of a PHY. It is a genuine clock-domain crossing (unlike measuring sys
    with sys, which would exercise no CDC at all), and it is what distinguishes "the PHY is dead" from
    "the counter is dead", which are otherwise indistinguishable and lead to opposite, expensive
    conclusions. Deviation from the spec's wording ("a sys-derived *divided* clock") recorded here: a
    multiplied clock we already have is a stronger self-test than a divided one and needs no new clock.
    """
    def __init__(self, sys_clk_freq, ulpi_pads, refclk_pads = None, refclk_clk = None,
        refclk_lock = None, hub_rst_pad = None, gate_ms = 1,
        clk_ok_min_hz = 45e6, clk_ok_max_hz = 75e6, with_analyzer = True, bus_owner = None):
        gate = int(sys_clk_freq*gate_ms/1e3)
        # **Who drives the ULPI pads, and why that is a constructor argument.**
        #
        # Spin 1 of the probe only *listened*: it drove `data` at the ULPI idle level while the PHY was in
        # reset and released the bus otherwise, and held `stp` low because it transmitted nothing. The Link
        # (spec decision 8) takes the pads now, and its idle state *is* that rule -- `oe = phy_reset |
        # master_driving`, `data` 00h when not driving -- so handing them over loses nothing. What must not
        # happen is two blocks attaching a tristate to the same pads: that is not a warning, it is either a
        # build error or a bus fight, and the second kind is invisible from a register read. Hence one flag
        # that moves the driver, the `probe_oe` CSR's source, and the analyzer's `data` tap together --
        # they are three views of one fact and cannot be allowed to disagree.
        #
        # `bus_owner=None` is spin 1 exactly as measured. `--with-ulpi-master` passes the Link.
        drives_pads = bus_owner is None

        # --- measurement: the ULPI clock (C12 via cd_usb; may be absent) -----------------------------
        # width=8: the gray counter wraps every 256 edges and the sampler accumulates the differences, so
        # what matters is that the per-sys-tick increment stays far below half the range (2x sys at 50 MHz
        # sys ticks is ~2, not ~128).
        self.rsv_spin2      = Signal(5, name="rsv_spin2")   # see RESERVED_CSR_NAMES below
        # High bits of the ULPI edge counter, as a named signal of its own: a slice (`status[8:24]`) is a
        # _Slice, which is unhashable, and LiteScope de-duplicates its group with a dict -- it refuses
        # slices outright. Named, it also reads as something in the VCD instead of `x[8:24]`. Bits 8..23
        # are the interesting ones: 60 MHz over a 1 ms gate is ~60 000, so they move once per window.
        self.ctr_hi         = Signal(16, name="ulpi_ctr_hi")
        self.fm_ulpi     = FreqMeter(period=gate, width=8, clk=ClockSignal("usb"))
        # --- measurement: the self-test channel (known clock, real CDC) ------------------------------
        self.fm_selftest = FreqMeter(period=gate, width=8, clk=ClockSignal("sdram_half"))

        # --- the derived claim `clk_ok` --------------------------------------------------------------
        # A band, not equality: the PHY's 60 MHz is only as good as its 24 MHz reference's tolerance, and
        # a gate window that starts/stops on a sys edge quantises by +-1 edge. Registered so the value is
        # stable for the whole window instead of glitching while the counter is being latched.
        lo = int(clk_ok_min_hz*gate_ms/1e3)
        hi = int(clk_ok_max_hz*gate_ms/1e3)
        self.clk_ok = CSRStatus(description=f"1 = ULPI clock within {clk_ok_min_hz/1e6:.0f}-"
                                            f"{clk_ok_max_hz/1e6:.0f} MHz over the last {gate_ms} ms.")
        self.sync += self.clk_ok.status.eq((self.fm_ulpi.value.status >= lo) &
                                           (self.fm_ulpi.value.status <= hi))
        self.comb += self.ctr_hi.eq(self.fm_ulpi.value.status[8:24])
        self.gate_ms = CSRStatus(16, reset=gate_ms,
            description=f"Gate window in ms; divide any *_freq by this to get Hz.")

        # --- resets: registers, with the polarity included in software ------------------------------
        #
        # The two sources disagree and no probe reaches the pin: ULPI `RESET` is active-high by spec,
        # tomverbeure drives `io.ulpi.reset := False` for "not in reset", the hub net is spelled
        # `USB_RST_N`, and the 2023 wrapper held `pads.reset` at a constant 0 with a `# todo remove`
        # (which, if the PHY is active-high, held the PHY permanently in reset). So the *logical*
        # meaning lives in software and the polarity is a bit: assert, watch `clk_ok`, and the board
        # decides. `*_inv=1` means the net is active-low, i.e. the pad carries the complement.
        self.phy_rst     = CSRStorage(reset=1, description="1 = assert PHY reset (logical).")
        self.phy_rst_inv = CSRStorage(reset=0, description="1 = the PHY reset net is active low.")
        self.hub_rst     = CSRStorage(reset=1, description="1 = assert hub reset (logical).")
        self.hub_rst_inv = CSRStorage(reset=1, description="1 = the hub reset net is active low "
                                                         "(the net is named USB_RST_N).")

        # --- the sequence: identical edges in every run, so the analyzer can trigger on one ---------
        # `auto=0` gives manual stepping (the *_rst bits drive the pads directly); `auto=1` lets this
        # FSM do it, and the wait is a register so the edge timing is not baked into a bitstream.
        self.seq_start    = CSRStorage(description="Write 1 to run assert -> wait -> release.")
        # reset=0: at power-up the **registers** own the resets, and `phy_rst`/`hub_rst` reset to 1, so the
        # board comes up with both resets asserted and the bus driven idle. An auto-sequencer that took
        # the resets over at power-up would release the PHY before anyone decided to.
        self.seq_auto     = CSRStorage(reset=0, description="1 = the sequencer owns the resets.")
        self.seq_wait_ms  = CSRStorage(16, reset=100, description="ms to hold each reset state.")
        self.seq_state    = CSRStatus(3, description="0=IDLE (register-owned) 1=ASSERT 2=RELEASE 3=DONE.")
        IDLE, ASSERT, RELEASE, DONE = [0, 1, 2, 3]
        state    = Signal(3, reset=IDLE)
        timer    = Signal(32)
        target   = Signal(32)
        asserted = Signal(reset=1)                              # logical, before polarity
        # Exposed for the Link (`--with-ulpi-master`), and deliberately the **logical** level rather than
        # `rst_phy_net`: the master must gate its `oe` on "the PHY is in reset" in the sense ULPI means, and
        # that is decided here, once, in software. Handing it the pad net would re-apply the polarity hint a
        # second time in a different place, and the whole point of the hint being a CSR is that the bench can
        # still be wrong about it -- two places to correct is two places to disagree.
        self.rst_level_logical = asserted
        self.sync += [
            target.eq(self.seq_wait_ms.storage*int(sys_clk_freq/1e3)),
            timer.eq(timer + 1),
        ]
        # A write, not a level: starting from the current state means the harness can re-run the same
        # edges indefinitely, and DONE is stable (it does NOT fall back to the register) so a released
        # PHY stays released for the measurements that follow instead of being reset again.
        self.sync += If(self.seq_auto.storage & self.seq_start.wr_stb, state.eq(ASSERT), timer.eq(0))
        self.sync += Case(state, {
            IDLE    : [],
            ASSERT  : [If(timer >= target, state.eq(RELEASE), timer.eq(0))],
            RELEASE : [If(timer >= target, state.eq(DONE),    timer.eq(0))],
            DONE    : [],
        })
        # Ownership is explicit and one-way: `auto=1` hands the resets to the FSM, `auto=0` gives them to
        # the registers. The first version instead let state==DONE hold the release forever, which made
        # **manual re-assertion impossible after one sequence** -- the "re-assert PHY reset" acceptance
        # row, the one that proves the clock comes from the PHY, could never be run twice. Found by the
        # offline model, not on the board, which is what the model is for.
        self.comb += asserted.eq(Mux(self.seq_auto.storage, state == ASSERT, self.phy_rst.storage))
        self.comb += self.seq_state.status.eq(state)

        # --- drive the pads, polarity applied last ---------------------------------------------------
        # A Signal, not the Mux expression: LiteScope's format_groups() de-duplicates with a dict, and a
        # migen operator (_Operator) is unhashable -- the analyzer refuses to tap an expression. Register
        # it and tap the register (which is also what you want in a capture: one name, one net).
        rst_phy_net = Signal(name="rst_phy_net")
        self.comb += [
            rst_phy_net.eq(Mux(self.phy_rst_inv.storage, ~asserted, asserted)),
            ulpi_pads.reset.eq(rst_phy_net),
        ]
        if hub_rst_pad is not None:
            self.comb += hub_rst_pad.eq(Mux(self.hub_rst_inv.storage, ~self.hub_rst.storage,
                                            self.hub_rst.storage))

        # --- the ULPI bus: observed, never driven by anything but the idle level ---------------------
        #
        # `oe` here is a **level from the sequence state**, not the standards' `rst_asserted | ~dir_in`:
        # with no `cd_usb` logic sampling `dir` synchronously, that expression would make an output
        # enable depend on an asynchronous pad input on a bus we transmit nothing on. The standards line
        # was amended in the same change (CODING_STANDARDS.md) so reviewer and code agree.
        # The two bus taps, so the pad driver, `probe_oe` and analyzer group 1 all read the same net. A
        # named Signal and not `self.tri.o`: LiteScope de-duplicates its groups through a dict, so an
        # expression is unhashable and refuses to be tapped (spin 1 measured that), and it is also what
        # you want in a VCD -- one name, one net, one meaning.
        self.bus_oe = Signal(name = "ulpi_bus_oe")
        self.bus_o  = Signal(8, name = "ulpi_bus_o")
        if drives_pads:
            self.tri = TSTriple(8)
            self.specials += self.tri.get_tristate(ulpi_pads.data)
            self.comb += [
                self.tri.oe.eq(asserted),    # drive the bus idle while the PHY is in reset (ULPI asks
                self.tri.o.eq(0),            #   for exactly this), release it the moment it is not
                ulpi_pads.stp.eq(0),         # never a strobe here: spin 1 transmits nothing
            ]
            self.comb += [self.bus_oe.eq(asserted), self.bus_o.eq(self.tri.o)]
        else:
            # The Link owns `data`, `oe` and `stp`. This block keeps every one of its registers and every
            # one of its analyzer slots -- but they now observe the master rather than a driver that is no
            # longer there. `bus_oe`/`bus_o` are assigned by the target from the master's **registered**
            # views (`oe_out`, `tx_byte`), which is the same information the pad cell sees, one clock
            # earlier in cd_usb: the same substitution the W12 group already makes for the ODDR2's D inputs
            # and for the same reason (ERROR:Pack:2531 -- pulling an output cell's Q side into fabric makes
            # the packer refuse the design).
            self.tri = None
        self.oe = CSRStatus(description="Current output-enable level on the ULPI data pads. With "
                             "--with-ulpi-probe alone this is the probe's own idle rule; with "
                             "--with-ulpi-master it is the Link's, which is the same fact about the same "
                             "pad and the only one still worth reporting under this name.")
        self.comb += self.oe.status.eq(self.bus_oe)

        # dir/nxt are the PHY's; a 2-FF copy *and* the raw wire are both tapped so a metastable sample
        # in the capture is distinguishable from a real transition (they differ for one sample only).
        self.dir_sync = Signal(1)
        self.nxt_sync = Signal(1)
        self.specials += [MultiReg(ulpi_pads.dir, self.dir_sync, n=2),
                          MultiReg(ulpi_pads.nxt, self.nxt_sync, n=2)]
        self.dir_status = CSRStatus(description="`dir` after the 2-FF synchroniser (raw is in group 0).")
        self.comb += self.dir_status.status.eq(self.dir_sync)

        # --- the analyzer: ONE instance, groups multiplexed -----------------------------------------
        # Only group 0 is captured at full rate, so the cause-and-effect set has to live there together:
        # sequence state, both resets, oe, clk_ok, raw+sync dir, and high bits of the clock counter. A
        # second analyzer would halve the rate of both.
        # The groups, exposed for the SoC to hand to LiteScope. Built here (where the signals are) but
        # instantiated at SoC level, because `litescope_cli`'s driver looks the analyzer's own CSRs up by
        # fixed name: nested inside `probe` they came out as `probe_analyzer_trigger_enable` and every
        # capture died with "'LiteScopeAnalyzerDriver' object has no attribute 'trigger_enable'" (measured
        # on the board, 2026-10-30 -- the CSRs were all present, just not under the name the client uses).
        # --- W12 / refclk: measure before drive -------------------------------------------------------
        #
        # TODO (a) is the question "does the FPGA have to generate the 24 MHz", and the only instrument
        # is this FPGA. So W12 is first an input whose level and edge activity are counted with the PHY
        # held in reset and nothing driven; driving comes later, behind a human gate.
        #
        # ⚠ **The pull-up is NOT a CSR bit, and cannot be.** The spec's `w12_mode` list has
        # "1 = input with pull-up", but on Spartan-6 PULLUP/KEEPER are **IOB attributes burned into the
        # bitstream**, not fabric signals -- there is no runtime path to them. Asking for one would have
        # produced a register that silently does nothing, which is the worst possible outcome for a safety
        # knob. So the pull-up is a *build* option (`--w12-pullup`, a `NET ... PULLUP = TRUE` constraint)
        # and the two readings are two spins. Value 1 is then free, and it is used for something better
        # than the thing it replaced -- see the mode list on `w12_mode`.
        if refclk_pads is not None:
            # The modes are ordered by how much they can hurt, and the harness walks them in that order:
            #   0 input            -- cannot hurt, and the only state at power-up
            #   1 output static 1  -- the *discriminator*: if the net does not read 1 while we hold it at
            #                         1 with the weakest driver, something is holding it low and driving a
            #                         clock would be driving into it. Static, so the exposure carries no
            #                         edges and no state machine depends on it.
            #   3 output static 0  -- cannot contend with a low holder (both sides drive the same level)
            #   2 output 24 MHz    -- the actual experiment, and the only mode that also needs the PLL lock
            self.w12_mode = CSRStorage(2, reset=0, description="0=input (drive OFF, the power-up value), "
                                     "1=output static 1 (does the net follow our weakest driver?), "
                                     "3=output static 0, 2=output 24 MHz (needs PLL lock). Pull-up is "
                                     "--w12-pullup at build time, NOT here: PULLUP is a Spartan-6 IOB "
                                     "attribute with no fabric path to it.")
            self.w12_keep = CSRStorage(description="Refresh: write 1 to keep the output enable. It drops "
                                     "itself `w12_keep_ms` after the last write -- a crashed harness, a "
                                     "closed laptop or an agent that stops mid-experiment must not leave a "
                                     "net driven whose other driver was never ruled out.")
            # **In transactions, not in seconds.** Measured 2026-10-30: one CSR round trip over the DDC
            # serial link costs ~12-20 ms, so the original fixed 50 ms window is only ~2.5 transactions
            # wide -- a harness that writes the keep bit and then reads three registers *reliably* sees the
            # last read fall outside its own window (measured: keep_ok={0:39,1:1} while drive={1:39,0:1} in
            # the same loop). That made a healthy board look like it was dropping the enable at random, and
            # it voided good rows. 500 ms is still a short leash (a dead host leaves the pad driven for at
            # most half a second) and buys ~25 transactions of margin.
            self.w12_keep_ms = CSRStorage(16, reset=500, description="Keep-bit window in ms (default 500: "
                                     "~25 serial transactions. Do not set it near one transaction's cost, "
                                     "measured ~20 ms here).")
            # ⚠ This description used to say "the real net level, **even while driven**". It cannot be true:
            # the tri-state is an ODDR2 because ERROR:LIT:675 requires it, and using the IOLOGIC that way
            # "eliminate[s] the use of the IOB as a bidirectional IO" -- the input path stops observing the
            # pin exactly when the output engages, and the value you read is the one sampled *before* the
            # pad was enabled. Measured on the same firmware twice: float spin + drive static 1 reads
            # `1,0,0,0,0,0,0` (frozen at the floating net's 0) and pull-up spin + drive static 0 reads
            # `0,1,1,1,1,1` (frozen at the pull-up's 1). It is what produced a phantom "something is holding
            # W12 low" on 2026-10-30, so the pad's own view is no longer trusted silently: `w12_level_valid`
            # says whether the number in front of you was observed or remembered.
            self.w12_level   = CSRStatus(description="W12 as the pad's input path last observed it. "
                                     "**Only meaningful while w12_level_valid=1**: while the output is "
                                     "enabled the IOB is not bidirectional and this is the pre-drive value.")
            self.w12_edges   = CSRStatus(16, description="W12 transitions per gate window. NOT a frequency: "
                                         "sampled in cd_sys, so a 24 MHz net aliases -- 'many' means "
                                         "'something is driving it', which is the actual question.")
            self.w12_drive   = CSRStatus(description="1 = the FPGA is driving W12 right now.")
            self.w12_pll_lock = CSRStatus(description="1 = the refclk PLL is locked. Mode 2 will not drive "
                                     "without it: an unlocked PLL can sit at any frequency, and 'the PHY "
                                     "did not answer' would then be a measurement of nothing.")
            # --- the two seams that were invisible, added 2026-10-30 after the board refused to answer ---
            #
            # `w12_drive` is cd_sys's *intent*. What arms the pad is `drive_on` after it has been carried
            # into cd_refclk -- and if that domain has no clock, the intent is 1, the enable is 0, the pad
            # sits at the tri-state ODDR2's INIT=1 (hi-Z), and every register still says "driving". That is
            # not a hypothetical: driving static 1 on the board read the net at 0 (7 of 8 samples) while
            # `w12_drive` and `w12_keep_ok` both read 1, and the same net followed a 50 kOhm pull-up --
            # which a 4 mA driver cannot lose to, physically. So the pad's own view gets a register.
            self.w12_drive_r = CSRStatus(description="1 = cd_refclk itself holds the enable up, i.e. what the "
                                     "pad cell actually sees. If `w12_drive`=1 while this is 0, cd_refclk is "
                                     "not clocking and the pad never left hi-Z (INIT=1 on the tri-state "
                                     "ODDR2) -- the failure mode that makes every other register lie.")
            self.w12_tog     = CSRStatus(description="1 = the refclk-domain heartbeat toggle is moving. 0 "
                                     "while w12_pll_lock=1 means the PLL claims lock but its clock is not "
                                     "reaching the fabric (BUFG/routing), which is the same fault seen from "
                                     "the other side.")
            # An independent count of the *internal* 24 MHz: not the pad, not the PHY, just PLL -> BUFG ->
            # fabric. 24000 per 1 ms gate when alive. Cheap, and it separates 'the clock does not exist'
            # from 'the clock exists but never got to the pin' -- the two answers that need opposite fixes.
            self.fm_refclk = FreqMeter(period=gate, width=8, clk=ClockSignal("refclk"))
            self.w12_keep_ok = CSRStatus(description="1 = the keep bit was refreshed inside its window.")
            # Not a nice-to-have: without this, `w12_level` is indistinguishable from a live reading, and a
            # live reading is what a safety gate needs. 1 = the pad is hi-Z, so `w12_level`/`w12_edges` are
            # observations of the net. 0 = the pad is driving, so they are a memory of before it did.
            self.w12_level_valid = CSRStatus(description="1 = the pad is tri-stated, so w12_level and "
                                     "w12_edges are observing the net. 0 = we are driving and the IOB's "
                                     "input path is disabled (DDR tri-state => not a bidirectional IOB): "
                                     "those registers then hold the value from *before* the enable, which "
                                     "is not a measurement of anything current.")

            w12_in   = Signal(name="w12_in")
            keep_ok  = Signal(name="w12_keep_live")
            drive_on = Signal(name="w12_drive_on")

            # The pad: one IOBUF, both directions, and that is deliberate. The read-back (`o`) is the
            # *net*, not our own output register, so "the enable dropped and the net is really not ours"
            # is something the harness can verify instead of assume -- and `w12_level` means something in
            # every mode, including while driving (a drive fight shows up as the level not following us).
            # A Signal, not `Constant(0)`: XST's Verilog reader refuses an assignment into a constant on
            # the IOBUF's I pin (ERROR:HDLCompiler:1532 "Cannot assign to a constant value").
            # There is no `w12_out` any more. The combinational mux that used to sit here was meant to be
            # the pad's driven value and *cannot* be one: Spartan-6 refuses a clock into fabric
            # (Place:1136), so the driven value is an ODDR2 output -- `w12_drv` in the netlist. The modes
            # are decoded from synchronised bits below, in the pad's own domain.
            # ⚠ **How we drive this pad, and why it is not a mux into a buffer.**
            #
            # First attempt: `Tristate(pad, i=w12_out, o=w12_in)`. Wrong on argument order -- migen is
            # `Tristate(pad, o, oe, i)`, where `o` is what gets DRIVEN (IOBUF `.I`) and `i` is what gets
            # READ (`.O`). The result drove the pad from an undriven wire and let the read-back net fight
            # the mux; it synthesized, placed and timed clean, so nothing said so. Every `w12_level` /
            # `w12_edges` reading from earlier on 2026-10-30 is therefore not a measurement of the net --
            # including the "stays low against the internal pull-up" that looked like a holder. Do not
            # cite those numbers; re-measure.
            #
            # Second attempt: a mux on a BUFG'd clock, which Spartan-6 refuses outright
            # (ERROR:Place:1136, a global buffer driving a non-clock load). So the pad is driven the way
            # the CRG drives SDRAM CK: **ODDR2**, with the clock on C0 (a legal clock load) and the mode
            # choosing D0/D1 -- 1/0 gives a copy of the clock, 1/1 a static 1, 0/0 a static 0. One output
            # stage, no fabric in the clock path, and the same three modes the CSR advertises.
            #
            # DDR_ENABLE must be TRUE: with FALSE the primitive ignores D1 and the "clock" mode would
            # silently output a constant 1.
            # --- what feeds the pad cells, and from which domain ---------------------------------------
            #
            # Both ODDR2s below are clocked by `cd_refclk`, so everything into their D inputs must belong
            # to `cd_refclk` too. The first version fed them `d0/d1` decoded straight from
            # `w12_mode.storage`, and `~drive_on` decoded from a 32-bit cd_sys watchdog: two multi-bit
            # asynchronous cones into the data and tri-state inputs of the cell that decides whether the
            # PHY sees a clock. trce caught the timing half of that (Score 173, one endpoint:
            # `keep_timer<6> -> ODDR2_6`). The functional half is silent, and it is precisely the
            # "we believed we had stopped driving" case the watchdog exists to prevent.
            #
            # So: decode in `cd_sys` (where the register lives) into **one-bit predicates**, synchronise
            # each with its own MultiReg, decode again in `cd_refclk`. One bit per net means a
            # synchroniser can only emit the old or the new value, never a third -- and the pad's
            # (D0,D1) pair can then only ever combine into:
            #
            #   (1,0) clock   (1,1) static 1   (0,0) static 0
            #
            # The clock combination needs `sel_clock`, i.e. `w12_mode.storage == 2`, so **a clock cannot
            # appear on the pad on the way between two other modes**. That is why this is two predicates
            # rather than the obvious "ship the 2-bit mode across and decode there", which can land on the
            # clock code mid-transition (mode 0 -> 3 changes both bits).
            sel_clock = Signal(name="w12_sel_clock")   # mode 2: put the refclk on the pad
            sel_one   = Signal(name="w12_sel_one")     # mode 1: hold the pad at 1 (the discriminator)
            mode      = self.w12_mode.storage
            self.comb += [
                sel_clock.eq(mode == 2),
                sel_one.eq(mode == 1),
            ]

            # The watchdog, moved above the pad cells so the enable is defined before it is used (it was
            # below, which is how the async cone got wired in the first place).
            # reset=0, deliberately: **expired at power-up** -- see the note in the original place below.
            keep_timer = Signal(32, reset=0)
            # `keep_timer` itself, in ~1 ms units (>>10 of sys cycles: 65535*1024/50000 = 1342 ms of range).
            # Added 2026-10-30 because the board produced a row where `w12_drive` read 0 while its own
            # complement `w12_level_valid` and its crossed copy `w12_drive_r` both said the enable was UP --
            # three reads each, so not a one-off. Either the window is really expiring between the keep
            # write and the read, or the read is wrong, and those need opposite fixes. This register ends
            # the argument: it is the countdown, not an inference from a status bit.
            self.w12_keep_left = CSRStatus(16, description="keep_timer >> 10, i.e. ~ms of window left "
                                     "(1024 sys cycles per count). 0 = expired.")
            # **A second way to hold the pin driven, and deliberately not another value of the first one.**
            #
            # A keep-bit watchdog is the right instrument for a probe: it fails safe, and a spin that stops
            # refreshing it stops driving. It is the wrong instrument for the Link, which is not probing any
            # more -- it needs a refclock for as long as it owns the PHY, and a 20 ms window whose refresh
            # rides on a link where one CSR round-trip costs ~20 ms is a design that drops the clock whenever
            # the host blinks. (Measured: the PHY emits exactly 60.000 MHz on C12 **only** while we drive,
            # and free-runs at ~29.45 MHz otherwise -- TODO (a) closed, RESEARCH.md.)
            #
            # So `arm` is a second input to the enable alongside `keep_ok`, not a third mode value. A mode
            # number would have cost `w12_mode` its meaning ("how is the oscillator routed"), and the two
            # questions are independent: a long-lived Link driving mode 1's direct copy is a different fact
            # from "the host refreshed the window 4 ms ago". Both old modes behave exactly as before; the
            # power-up value is 0 so a spin that never writes it is bit-for-bit spin 1; and the watchdog
            # still runs, still expires and still reports, because `w12_keep_left`/`w12_keep_ok` must mean
            # the same thing in an armed build as in an un-armed one.
            self.w12_arm = CSRStorage(description="Hold the W12 output enable up independently of the keep "
                                 "window: write 1 once at boot and the pad stays driven for the life of this "
                                 "bitstream. The Link's refclock cannot ride a 20 ms watchdog when one CSR "
                                 "round-trip costs ~20 ms. Independent of w12_mode (oscillator routing) and "
                                 "of w12_keep_ms (window length); 0 at power up, so an unwritten arm never "
                                 "silently drives a pin, and the watchdog alone decides as before.")
            self.sync += If(self.w12_keep.wr_stb,
                            keep_timer.eq(self.w12_keep_ms.storage*int(sys_clk_freq/1e3)))
            self.sync += If(keep_timer != 0, keep_timer.eq(keep_timer - 1))

            # `LOCKED` is an asynchronous level as far as the fabric is concerned, and it gates mode 2.
            # Synchronised **in cd_sys, not cd_refclk**, on purpose: while the PLL is unlocked its clock
            # output may not be running, and a synchroniser in the domain that clock would drive can never
            # release. (Independent bug in the tree before any of this: the raw level went straight into a
            # CSRStatus and into the enable logic.)
            pll_lock = Signal(name="w12_pll_lock_sync")
            if refclk_lock is not None:
                self.specials += MultiReg(refclk_lock, pll_lock, odomain="sys", n=2)
            else:
                self.comb += pll_lock.eq(1)

            # **Registered, and that is a measured decision, not a style preference.** As combinational
            # signals the CSR bridge reads them mid-glitch: `keep_timer != 0` is a 32-bit reduction that
            # re-evaluates on all 50 million decrements a second, and `drive_on` inherits every one of them.
            # The symptom on the board (2026-10-30): a status bit wrong ~1 read in 25 with nothing having
            # changed -- caught because `w12_drive` and `w12_level_valid` are *complements* and came back
            # AGREED (both 0) in the same row. That is impossible for the logic and entirely possible for a
            # read. One such read voided a healthy drive row and the harness refused, correctly, to build a
            # clock step on it. A flop can be read one cycle stale; it cannot be read mid-transition.
            self.sync += [
                keep_ok.eq(keep_timer != 0),
                # The watchdog gates the *enable*, not the mode register: `w12_mode` may say 2 or 3 and the
                # pin still is not driven unless the keep bit is live -- or unless `w12_arm` holds it up.
                # `keep_ok` deliberately reports the watchdog and nothing else, so this bit is the only
                # place the two sources of permission meet, and `w12_keep_left` stays honest about expiry.
                drive_on.eq((((mode == 3) | (mode == 1)) | ((mode == 2) & pll_lock))
                            & (keep_ok | self.w12_arm.storage)),
            ]

            # The crossings into the pad's domain. migen's Xilinx builder puts `ASYNC_REG` on a MultiReg's
            # flops (which is why MultiReg rather than two hand-written flops), so the pair stays in one
            # slice; the false-path constraint for the crossing itself belongs to the domain, see
            # RefclkPLL + add_false_path_constraints in BaseSoC.
            # A heartbeat in the pad's domain: if cd_refclk has no clock this never toggles, which is a
            # *positive* statement about the clock rather than the absence of one.
            tog = Signal(name="w12_refclk_tog")
            self.sync.refclk += tog.eq(~tog)   # the pad domain, not cd_sys
            tog_r = Signal(name="w12_refclk_tog_sys")
            sel_clock_r = Signal(name="w12_sel_clock_r")
            sel_one_r   = Signal(name="w12_sel_one_r")
            drive_on_r  = Signal(name="w12_drive_on_r")
            self.specials += [
                MultiReg(sel_clock, sel_clock_r, odomain="refclk", n=2),
                MultiReg(sel_one,   sel_one_r,   odomain="refclk", n=2),
                MultiReg(drive_on,  drive_on_r,  odomain="refclk", n=2),
            ]
            # Decode again, in the pad's own domain: from here to the ODDR2 data pins is a *synchronous*
            # path with 41.7 ns to spare, so it is not the thing that can fail timing.
            d0 = Signal()
            d1 = Signal()
            self.comb += [
                d0.eq(sel_clock_r | sel_one_r),
                d1.eq(sel_one_r),
            ]
            w12_drv   = Signal(name="w12_drv")
            w12_t_r   = Signal(name="w12_t_r")    # ~drive_on through an ODDR2: LIT:675 demands it
            self.specials += [
                # Port/parameter names copied from the CRG's own ODDR2 in this file rather than from the
                # 7-series datasheet: Spartan-6 has no DDR_ENABLE (that is `DDR_MODE`) and no T on ODDR2,
                # and XST says so with "Module ODDR2 does not have a parameter named DDR_ENABLE" /
                # "Cannot find port T". C1 = ~C0 is how the CRG gets a full-rate edge pair too.
                Instance("ODDR2", p_DDR_ALIGNMENT = "NONE", p_INIT = 0, p_SRTYPE = "SYNC",
                    i_D0 = d0, i_D1 = d1, i_S = 0, i_R = 0, i_CE = 1,
                    i_C0 = (refclk_clk if refclk_clk is not None else 0),
                    i_C1 = ~(refclk_clk if refclk_clk is not None else 0),
                    o_Q = w12_drv),
                # IOBUF, not OBUF: `w12_in` is the net, so "the enable dropped and the pin is not ours"
                # and "we drove 1 and the level did not follow" stay things the harness can verify.
                # ERROR:LIT:675 spells it out: "when the data path on a TBUF is driven by an ODDR2, the
                # tri-state path of the same TBUF must also be driven by an ODDR2". So the enable goes
                # through a second ODDR2 with D0=D1 (a level, clocked through) rather than a plain flop --
                # and it fails safe: INIT=1 means hi-Z until something clocks it, so if the refclk PLL
                # never locks, the pad is never driven.
                Instance("ODDR2", p_DDR_ALIGNMENT = "NONE", p_INIT = 1, p_SRTYPE = "SYNC",
                    i_D0 = ~drive_on_r, i_D1 = ~drive_on_r, i_S = 0, i_R = 0, i_CE = 1,
                    i_C0 = (refclk_clk if refclk_clk is not None else 0),
                    i_C1 = ~(refclk_clk if refclk_clk is not None else 0),
                    o_Q = w12_t_r),
                Instance("IOBUF",
                    i_I = w12_drv, i_T = w12_t_r, o_O = w12_in, io_IO = refclk_pads),
            ]

            # Count *transitions of the synchronised copy*: the pad is only ever sampled by the first flop
            # of the 2-FF, so a metastable read cannot itself be counted as an edge.
            w12_sync = Signal(name="w12_sync")
            self.specials += MultiReg(w12_in, w12_sync, n=2)
            w12_d = Signal()
            edge  = Signal()
            self.sync += w12_d.eq(w12_sync)
            self.comb  += edge.eq(w12_sync ^ w12_d)
            edges      = Signal(16)
            gate_count = Signal(32)
            gate       = int(sys_clk_freq*gate_ms/1e3)
            self.sync += [
                gate_count.eq(gate_count + 1),
                If(edge, edges.eq(edges + 1)),
                If(gate_count >= gate, gate_count.eq(0), edges.eq(0)),
            ]
            self.comb += [self.w12_level.status.eq(w12_sync), self.w12_edges.status.eq(edges)]

            # (The watchdog and the PLL-lock synchroniser moved up next to the pad cells they gate; this is
            #  now only the read-back.)
            #
            # ⚠ Residual exposure, stated rather than fixed, because there is no way to fix it here: the
            # enable is held in a cell clocked by the refclk, so **if the refclk disappeared while we were
            # driving, the watchdog could no longer drop the enable** -- the ODDR2 would simply stop
            # clocking and the pad would freeze at whatever level it held. That residual is a *static
            # level*, not a clock, and the refclk's only source is this PLL fed from cd_sys: if it stops,
            # cd_sys stopped too and the board is not being driven by anything any more. Power-cycling
            # clears it. Worth knowing before the drive step, and worth a meter if the pin is ever reachable.
            # Reading the pad domain's state back into cd_sys for the CSRs (levels, both quasi-static).
            drive_on_r_sys = Signal(name="w12_drive_on_r_sys")
            self.specials += [MultiReg(drive_on_r, drive_on_r_sys, odomain="sys", n=2),
                              MultiReg(tog, tog_r, odomain="sys", n=2)]
            self.comb += [
                self.w12_drive.status.eq(drive_on),
                self.w12_keep_ok.status.eq(keep_ok),
                self.w12_pll_lock.status.eq(pll_lock),
                self.w12_level_valid.status.eq(~drive_on),
                self.w12_keep_left.status.eq(keep_timer >> 10),
                self.w12_drive_r.status.eq(drive_on_r_sys),
                self.w12_tog.status.eq(tog_r),
            ]
        else:
            refclk_pads, w12_in = None, Signal()
            self.w12_mode   = CSRStorage(2)
            self.w12_keep   = CSRStorage()
            self.w12_level  = CSRStatus()
            self.w12_edges  = CSRStatus(16)
            self.w12_drive  = CSRStatus()
            self.w12_keep_ok = CSRStatus()
            # Present so a build without the refclock pads still has the name spin 2 writes, and fails as
            # "this revision has no W12 to arm" rather than as a missing CSR in the middle of a bench run.
            self.w12_arm = CSRStorage(description="No refclock pads on this revision: nothing to arm.")

        # ---- USB3300 §6.1.11: releasing reset is a conditional act, and we detect rather than gate -------
        # The datasheet (DS00001783 §6.1.11): "reset must only be de-asserted when the USB3300 clkout signal is
        # active", and "no other PHY digital input signals may change state for two clkout clock cycles after the
        # de-assertion of the reset signal".
        #
        # The second half we satisfy by scale, not by construction: `UlpiClear`'s relock tail holds the master
        # cleared for `relock_ms` (10 ms ≈ 600 000 cd_usb cycles) after the clock returns, so the Link cannot
        # touch STP or DATA inside two cycles of a release. Worth naming, because it turns a guessed constant
        # into one with a citation.
        #
        # The first half is NOT enforced here, deliberately. AN19.17 §4.2 says asserting RESETB *stops* the
        # 60 MHz output clock, so "hold reset until clk_ok" is a deadlock the moment it is written literally:
        # CLKOUT only comes back after the release. §6.1.11 meanwhile requires CLKOUT active *at* the release.
        # The two documents disagree on whether the PLL or only the pad buffer stops during hardware reset --
        # that is a bench question (row D14 of ticket 05), and the wrong HDL choice here would brick the very
        # session that answers it. So: count it, report it, gate nothing.
        #
        # A counter rather than a sticky bit: `phy_rst` comes out of configuration asserted (`reset=1`), so a
        # sticky bit would be set by the first boot sequence and then report nothing forever. Read it, release,
        # read it again -- an increment means YOUR release happened blind.
        prev_asserted = Signal(reset = 1, name = "prev_phy_rst")
        self.rst_blind = CSRStatus(8, description="PHY-reset RELEASES that happened while `clk_ok` was 0 "
                             "(USB3300 DS 6.1.11 wants CLKOUT active at release). Saturating, never cleared, and "
                             "NOTHING is gated on it -- see the deadlock note above. Read before and after a "
                             "release: an increment means that release was blind.")
        self.sync += prev_asserted.eq(asserted)
        self.sync += If(prev_asserted & ~asserted & ~self.clk_ok.status, [
            If(self.rst_blind.status != 0xFF, self.rst_blind.status.eq(self.rst_blind.status + 1)),
        ])
        # Created HERE on purpose: inside one CSR region LiteX assigns addresses in *creation* order, so a
        # register inserted where it "logically belongs" shifts every later one -- and this counter is new
        # since the spin-1 map was frozen, so `tools/test_ulpi_csr_map.py`'s golden file (rightly) refuses the
        # build. Same reason `probe_w12_arm` sits last. (The other level, measured later the same day: the
        # regions themselves are laid out alphabetically, which is why the 60 MHz capture's module is named
        # `usb_scope` and not `analyzer_usb` -- see ADR-0007 and extensions/pano_ulpi_scope.py.) Appending
        # costs nothing except that a name is not adjacent to the signals it belongs to -- which is what the
        # comment is for.

        # The groups, built last (they reference everything above) and instantiated by the SoC.
        #
        # ONE instance here because its groups are **multiplexed**: only one group is captured at a time, so
        # group 0 is where cause and effect have to live together on a single time axis.
        #
        # ⚠ This comment used to give the reason as "a second instance would halve the sampling rate of both",
        # and that is not what a second instance does: measured 2026-10-31 with `usb_scope` built alongside it
        # (bench/2026-10-31_ulpi_t17), `analyzer.csv` still reports samplerate 50e6 / RLE on / 25 bits, and the
        # `TIMESPEC`s are unchanged. An extra analyzer costs BRAM (360 Kb -> RAMB16 31 -> 53 of 268) and, if it
        # taps pads, input *hold* margin (ULPI `dir` +0.706 -> +0.146 ns) -- not rate. Rate is what a second
        # *group* would not cost either; what a second group costs is the one-pipeline sharing that this line
        # is actually about.
        self.analyzer_groups = {
            # group 0: sequence state, both resets, oe, clk_ok, raw + synchronised `dir`, and the high bits
            # of the clock counter -- cause and effect on a single time axis.
            0 : [state, rst_phy_net, self.hub_rst.storage, self.oe.status, self.clk_ok.status,
                 ulpi_pads.dir, self.dir_sync, self.ctr_hi],
            # group 1: slow/secondary, deliberately not time-critical.
            1 : [self.nxt_sync, ulpi_pads.nxt, self.hub_rst_inv.storage, self.phy_rst_inv.storage,
                 self.seq_auto.storage, self.bus_o],
            # group 2: W12 -- what we asked for, whether we are actually driving, and what the net says.
            # Separate from group 0 because "we enabled the driver and the level did not follow" is a
            # different question from the reset/clock coupling, and merging them costs rate to both.
            # ⚠ Tap the ODDR2's **D inputs**, never its Q: pulling `w12_drv`/`w12_t_r` into fabric logic
            # makes the packer refuse the design (`ERROR:Pack:2531 - The dual data rate register "ODDR2_5"
            # failed to join the OLOGIC/IOSWITCH pad`, measured 2026-10-30 on the first attempt at exactly
            # this instrumentation). The D side is the same information one clock earlier, and `d0/d1` are
            # already the decoded (D0,D1) pair, so nothing is lost: `d0=1,d1=1` + enable up + `w12_in`=0 is
            # a NET problem (something holds it), while the enable up and `d0/d1` not moving is a FABRIC
            # problem (the pad domain never clocked). Those two look identical from the CSR side and need
            # opposite fixes, which is the whole reason this group carries them.
            2 : [self.w12_mode.storage, self.w12_drive.status, self.w12_keep_ok.status,
                 self.w12_level.status, w12_in, self.w12_pll_lock.status, d0, d1, tog],
            # group 3: RESERVED for spin 2's ULPI bus capture (see RESERVED_* below); tied to 0 now so the
            # slot exists in the CSV and the client can be pointed at it.
            3 : [self.rsv_spin2],
        }

    # Spin 2 (the ULPI ID read) must land without renaming anything measured here or moving a group.
    # It needed: ulpi_cmd/ulpi_addr/ulpi_wdata/ulpi_rdata + ulpi_go, a CDC bridge into cd_usb, and the
    # 60 MHz bus capture. The CSRs were built under these names; the 60 MHz capture is a SEPARATE analyzer
    # (`usb_scope`, extensions/pano_ulpi_scope.py), not group 3 of this one -- group 3 stays tied to 0 here
    # precisely so this analyzer's groups 0/1/2 keep the positions every spin-1 capture was read at. Of the
    # names below, three are exact in `usb_scope.csv`; `ulpi_clk_ok` carries its module's prefix because
    # litex.gen prefixes module-owned signals, and `ulpi_data` becomes `ulpi_scope_data` because a
    # bidirect pad net may not drive fabric at all (ERROR:NgdBuild:924) so the tap is the Link's IOBUF
    # output. Both reasons are in extensions/pano_ulpi_scope.py; pinned row-by-row by
    # tools/test_ulpi_csr_map.py.
    RESERVED_CSR_NAMES  = ["ulpi_cmd", "ulpi_addr", "ulpi_wdata", "ulpi_rdata", "ulpi_go"]
    RESERVED_GRP_NAMES  = ["ulpi_data", "ulpi_dir", "ulpi_nxt", "ulpi_stp", "ulpi_clk_ok"]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq,
        sdram_chip      = "b",
        ulpi_pads       = None,
        # sdram_clk_phase = PLL CLKOUT3, the clock handed to the DRAM CK driver (CLKOUT2/270 deg runs the
        # sdram_half domain that drives DFI and captures read data). The DRAM launches DQ against ITS clock,
        # so this is a pure read-eye position control and it is NOT flat: measured on a rev C board with the
        # cacheless diagnostic SoC (10 240 test bytes + 16 byte-lane checks per point, chip b, 50 MHz sys /
        # 200 MT/s), the clean span is 252-268 deg with 0 errors, while 250 deg -- the value inherited from
        # the 2023-era litex-buildenv target -- already corrupts reads (BIOS 2 MiB memtest: 23078 data errors
        # of 524288, always the same bit: lane 1, bit 2 = DQ10, even words only, spurious 1s only), and
        # >=270 deg is catastrophic (8040-10240 bad bytes). 260 deg = centre of the window; 3/3 confirmation
        # runs clean (Memtest OK + 0/4096 B + 0/16 lanes). Treat it as per-board tuning, not a constant:
        # re-measure with tools/ddr_sweep.sh + tools/ddr_confirm.sh in ~/llm/tests/panologic_g2.
        sdram_half_phase = 270.,
        sdram_clk_phase  = 260.,
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

        # ULPI clock domain (`cd_usb`) ---------------------------------------------------------------
        # C12 is the 60 MHz the PHY outputs when it is out of reset. `reset_less` **because** that clock
        # can vanish: there is no usable synchronous reset in a domain whose clock may be absent, and the
        # boot-counter pattern is the only one that is (CODING_STANDARDS.md). Spin 1 of the ULPI Probe puts
        # no synchronous logic in here beyond FreqMeter's gray counter, so the boot counter is not needed
        # yet -- the domain is declared so the CDC boundary and the constraint exist once, not twice.
        if ulpi_pads is not None:
            self.cd_usb = ClockDomain(reset_less=True)
            # IBUFG, not a bare input: this is a clock and must reach the global network. If par refuses
            # this pin, that is itself the finding (it would mean the PHY's 60 MHz cannot clock fabric at
            # all) -- do not "fix" it by dropping the buffer.
            self.specials += Instance("IBUFG", name="ulpi_clk_bufg",
                i_I = ulpi_pads.clk, o_O = self.cd_usb.clk)
            platform.add_period_constraint(self.cd_usb.clk, 1e9/60e6)
            # The crossing to cd_usb is gray-code and deliberately asynchronous (FreqMeter does the CDC),
            # so no timing relationship is claimed between the two domains.
            platform.add_false_path_constraints(self.cd_sys.clk, self.cd_usb.clk)

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
        sdram_clk_phase    = 260.,   # see _CRG: read-eye centre measured on rev C, default 250 corrupts reads
        cpu_reset_pin      = None,
        serial_index       = 0,
        with_ulpi_probe    = False,
        with_analyzer      = True,
        w12_pullup         = False,
        with_ulpi_master   = False,
        with_ulpi_scope    = True,
        ulpi_skew_ns       = ULPI_SKEW_NS_DEFAULT,
        ulpi_stall_timeout = 1 << 16,
        ulpi_retry_limit   = 3,
        ulpi_io_offsets    = True,
        **kwargs):
        platform = pano_logic_g2.Platform(revision=revision)
        assert sdram_chip in ["a", "b"]

        # CRG --------------------------------------------------------------------------------------
        # Requested once here and handed down: `platform.request()` consumes the resource, so a second
        # request for the same group anywhere else fails with "Resource not found".
        ulpi_pads   = platform.request("ulpi") if (with_ulpi_probe or with_ulpi_master) else None
        refclk_pads = platform.request("usb_refclk") if (with_ulpi_probe or with_ulpi_master) else None

        if with_ulpi_probe:
            # Built before the probe so the probe can be handed the clock and, more importantly, its lock.
            self.refclk_pll = RefclkPLL(ref_freq = sys_clk_freq)
            # The weakest driver the part offers, on the one pin where we might be fighting a holder we
            # cannot see: contention current is set by the output impedance, and this is the only knob for
            # it. SLOW slew too -- a fast edge into an unknown load is a ringing experiment on top of an
            # electrical one.
            # Two statements, not `DRIVE = 4; SLEW = SLOW;` on one netspec: UCF repeats the netspec per
            # attribute and answers the shorthand with ERROR:ConstraintSystem:300 ("a value was expected
            # for the SLEW constraint's 'resspec'"), which reads like a typo in a line that looks fine.
            platform.add_platform_command('NET "{pad}" DRIVE = 4;',  pad=refclk_pads)
            platform.add_platform_command('NET "{pad}" SLEW = SLOW;', pad=refclk_pads)

        self.crg = _CRG(platform, sys_clk_freq,
            sdram_chip       = sdram_chip,
            ulpi_pads        = ulpi_pads,
            sdram_half_phase = sdram_half_phase,
            sdram_clk_phase  = sdram_clk_phase,
            cpu_reset_pin    = cpu_reset_pin)

        if with_ulpi_probe:
            # The refclk domain's constraints. Both are load-bearing and this is where the Score-173 build
            # was missing them:
            #   * the **period**, so the tool treats the pad clock as 24.000 MHz instead of inventing a
            #     phase relationship with sys_clk (it auto-derived `TSsys_clk / 0.48`, which is the right
            #     period for the wrong reason: a *derived* timespec tells XST the clocks are related), and
            #   * the **false paths**, because every cd_sys <-> cd_refclk path in this design is a MultiReg
            #     (see the pad-cell block in ULPIProbe) and an analysed 2-FF synchroniser is a bug report,
            #     not a design.
            # Registered here rather than inside RefclkPLL because cd_sys's clock net is only known once the
            # CRG exists. Same pair as `cd_usb` in the CRG, same reason.
            platform.add_period_constraint(self.refclk_pll.cd_refclk.clk, 1e9/self.refclk_pll.out_freq)
            platform.add_false_path_constraints(self.crg.cd_sys.clk, self.refclk_pll.cd_refclk.clk)

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
        # The probe build (see ~/llm/tests/panologic_g2/.scratch/ulpi-probe/spec.md) is CPU-less on
        # purpose: the DDC pair is either a **console** or the **remote channel**, never both, because
        # they are the same two wires. No BIOS => no console => the host can read CSRs and run the logic
        # analyzer over UARTBone, and the LED chaser is the only "alive" signal.
        if with_ulpi_probe:
            kwargs["with_uart"]     = False
            kwargs["with_uartbone"] = False   # added below, from the pads this target actually requested

        # Requested once and reused: platform.request() consumes the resource, so asking for the same
        # index twice dies with "Resource not found: serial:1".
        uart_pads = platform.request("serial", serial_index)
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident     = "LiteX SoC on Pano Logic G2",
            uart_pads = uart_pads,
            **kwargs)

        # UARTBone from the *requested* pads ---------------------------------------------------------
        #
        # Why this is hand-built instead of `--with-uartbone`: SoCCore.add_uartbone() ignores the pads a
        # target passes and re-requests the platform resource by *name* --
        #     uartbone_phy = uart.UARTPHY(self.platform.request(uart_name), ...)
        # with uart_name defaulting to "serial" (litex/soc/integration/soc.py, verified against the
        # pinned tree 2026-10-30). On this board `serial` has TWO indexes and _lookup() takes the first
        # match when number is None: index 0 = the micro-HDMI DDC pair. So the stock helper puts the
        # remote channel on wires nobody cabled, silently, while `uart_pads` (the DVI pair we are wired
        # to) is dropped. Same class of trap as the console's --serial-index (JTAG_CABLE.md §4).
        if with_ulpi_probe:
            from litex.soc.cores.uart import UARTPHY, UARTBone
            uartbone_phy = UARTPHY(uart_pads,
                clk_freq              = sys_clk_freq,
                baudrate              = kwargs.get("uart_baudrate", 115200),
                with_dynamic_baudrate = kwargs.get("uart_with_dynamic_baudrate", False))
            uartbone = UARTBone(
                phy           = uartbone_phy,
                clk_freq      = sys_clk_freq,
                cd            = "sys",
                address_width = self.bus.address_width)
            # add_module() sets self.uartbone itself and asserts the name is free -- do not assign it first.
            self.add_module(name="uartbone", module=uartbone)
            self.bus.add_master(name="uartbone", master=uartbone.wishbone)

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

        # ULPI Probe -------------------------------------------------------------------------------
        if with_ulpi_probe:
            self.probe = ULPIProbe(sys_clk_freq = sys_clk_freq,
                ulpi_pads    = ulpi_pads,
                refclk_pads  = refclk_pads,
                refclk_clk   = self.refclk_pll.cd_refclk.clk if with_ulpi_probe else None,
                refclk_lock  = self.refclk_pll.lock  if with_ulpi_probe else None,
                hub_rst_pad  = platform.request("usb_hub_rst_n"),
                with_analyzer= with_analyzer,
                # With the Link present it owns the pads; see the flag's comment in ULPIProbe.
                bus_owner    = with_ulpi_master)
            if w12_pullup and refclk_pads is not None:
                # The pull-up half of "floating or actively held". An attribute on the NET, because that
                # is all PULLUP is on this FPGA -- see the W12 block in ULPIProbe.
                platform.add_platform_command('NET "{pad}" PULLUP = TRUE;', pad=refclk_pads)

            if with_analyzer:
                # Named `analyzer` at the TOP level: LiteScope's client looks up `analyzer_trigger_enable`
                # & co by fixed name, so a prefix breaks it (see ULPIProbe.analyzer_groups).
                from litescope import LiteScopeAnalyzer
                self.analyzer = LiteScopeAnalyzer(self.probe.analyzer_groups,
                    depth        = 2**14,
                    samplerate   = sys_clk_freq,  # sets the VCD time base; wrong here = wrong axis later
                    clock_domain = "sys",
                    with_rle     = True, rle_length = 256,  # stretches the window over the quiet holds
                    csr_csv      = "analyzer.csv",
                )

        # The ULPI Link (ticket 02): the `ulpi_*` CSR door, behind it a bridge into the PHY's clock domain,
        # and behind that the master that puts one RegRead/RegWrite on the bus.
        #
        # Built AFTER the probe and its analyzer on purpose: it takes the pads the probe used to hold
        # (decision 8), and its analyzer taps are the probe's registered views of those pads rather than
        # `TSTriple` internals -- pulling an output cell's Q side back into fabric is ERROR:Pack:2531, which
        # is the same lesson the W12 group already learned the hard way.
        if with_ulpi_master:
            assert with_ulpi_probe, "--with-ulpi-master needs --with-ulpi-probe: the probe owns phy_reset, " \
                                    "clk_ok and the W12 refclock, and the Link has no business re-deriving " \
                                    "any of them (spec decision 8, decision 14)."
            self.ulpi = UlpiCsr(ulpi_pads,
                sys_clk_freq   = sys_clk_freq,
                stall_timeout  = ulpi_stall_timeout,
                retry_limit    = ulpi_retry_limit,
            )
            self.comb += [
                # The probe's *logical* reset levels, already polarity-corrected in software -- which is why
                # this is safe to hand to combinational logic in cd_sys. Feeding it the raw pad would be the
                # async-reset mistake decision 6 refuses, and feeding it the master's own guess would be a
                # second opinion where we want one.
                self.ulpi.phy_reset.eq(self.probe.rst_level_logical),
                # Spin 1's clock meter, registered in cd_sys, is the only thing that can tell us the PHY
                # stopped clocking us: cd_usb cannot observe the absence of its own clock (decision 14).
                self.ulpi.clk_ok.eq(self.probe.clk_ok.status),
                # `bus_grant` is a real interlock with one degenerate input. It is *not* dead code: when the
                # 2023 `usb_host` core joins, or a second spin wants `stp`, the arbitration lands here, and
                # `UlpiMaster` keeps `oe` low on a denied request (ULPI 1.1 §3.8.4.2). Tying it to 1 states
                # the one fact that makes that legal today: this SoC has exactly one ULPI bus master, and it
                # is us. A future second owner must fail here rather than share the pads silently.
                self.ulpi.bus_grant.eq(1),
                # What the probe reports as `probe_oe` / analyzer group 1's `data`, now driven by the Link.
                # Registered views (`oe_out`, `tx_byte`), not the pad cell's Q: same information, and the
                # packer stays happy.
                self.probe.bus_oe.eq(self.ulpi.link.master.oe_out),
                self.probe.bus_o.eq(self.ulpi.link.master.tx_byte),
            ]
            # The 60 MHz capture of the bus (ticket 03) ----------------------------------------------
            # A 50 MHz sample of a 60 MHz bus answers state questions and not byte questions: a ULPI byte is
            # ONE CLKOUT cycle, so `analyzer` can show that a transaction happened and cannot show which bit
            # of which byte is wrong. This is the instrument for that, in `cd_usb_scope` -- the PHY's clock
            # with a boot-counter reset, because LiteScope's storage/RLE FSMs need a reset and `cd_usb` is
            # reset-less by necessity. All of the reasoning, including the `reg scope_rst = 1'd0;` with no
            # driver that this replaces, is in extensions/pano_ulpi_scope.py.
            if with_analyzer and with_ulpi_scope:
                from litescope import LiteScopeAnalyzer
                self.ulpi_scope = UlpiUsbScope(ulpi_pads, link = self.ulpi.link)
                # Top-level, and `csr_csv` matching the instance name: `litescope_cli` derives the analyzer's
                # CSR prefix from the BASENAME OF --csv (`litescope_cli.py`, and `driver/analyzer.py` matches
                # `name + "_"` against the remote's register names), so prefix and CSV name are ONE decision.
                # The name is `usb_scope` and not the spec's `analyzer_usb`: LiteX lays CSR regions out
                # alphabetically, so `analyzer_usb` sorted in between `analyzer` and `ctrl` and moved all 71
                # addresses the golden map pins -- the guard refused the build, and this name appends. See
                # the last section of extensions/pano_ulpi_scope.py.
                self.usb_scope = LiteScopeAnalyzer(self.ulpi_scope.groups,
                    depth        = 2**14,          # 273 us at 60 MHz: longer than any ULPI access, shorter
                                                   #   than the ~20 ms it takes the host to notice anything.
                    samplerate   = 60e6,           # sets the VCD time base; wrong here = a lying axis later
                    clock_domain = "usb_scope",
                    register     = True,           # one register stage on the taps, in cd_usb_scope: the
                                                   #   pad->flop cone is then all the OFFSET = IN budget has
                                                   #   to cover, instead of mux + trigger + FIFO-write.
                    with_rle     = False,          # decision 22: one new suspect per spin.
                    csr_csv      = "usb_scope.csv",
                )
                # The line the whole domain exists for: LiteScope drives `cd_scope.clk` from our domain but
                # NEVER drives `cd_scope.rst`, so without this the storage FSM's reset mux reads an undriven
                # net (`ulpi_t16`'s netlist has exactly that: declared, read once, zero drivers).
                self.comb += self.usb_scope.cd_scope.rst.eq(self.ulpi_scope.scope_rst)

            if ulpi_io_offsets:
                # Relative to the PHY's clock, exactly as the spec's decision 9 requires. They do not all close
                # at the slow corner (~1 ns on `data`, ~3.3 ns on `stp`, spin `ulpi_t16`), and that deficit is an
                # accepted deviation with a trigger attached: ADR-0005 + ticket 06. `--no-ulpi-io-offsets` exists
                # to isolate which constraint a failure belongs to on a bench, never to make a report prettier.
                self._add_ulpi_io_offsets(platform, skew_ns = ulpi_skew_ns,
                                          cd_sys = self.crg.cd_sys.clk)

        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                sys_clk_freq = sys_clk_freq)

    def _add_ulpi_io_offsets(self, platform, clk_net = "usb_clk", skew_ns = ULPI_SKEW_NS_DEFAULT,
                             cd_sys = None):
        """The ULPI pad input/output timing, as raw UCF.

        **Why raw.** LiteX in this tree has no `add_input_delay_constraint`/`add_output_delay_constraint`
        (checked: no such name exists anywhere in `litex/`, and there is no SDC backend for ISE), so the
        documented helper cannot be used and `add_platform_command` is the only door. Net names are taken
        from the requested pads rather than hardcoded, because they are LiteX's naming, not ours.

        **The numbers, and what they are made of.** Read off **ULPI v1.1 §3.7.2 Table 5, the "Output clock"
        group** (`datasheets/ULPI_v1_1.pdf`, verified 2026-10-31; Microchip AN19.17 §5.1 restates the same
        table and adds "it is expected that any SoC that integrates a link meets the setup and hold times
        defined here"). Two things that citation settles, and that an earlier version of this docstring got
        wrong: the *other* group in that table, "Input clock (optional)", is where the Link supplies the clock
        and `TSD` is only 3.0 ns -- not this board, where the PHY outputs CLKOUT; and every parameter is
        measured "with respect to the clock as seen at the PHY clock pin", which is exactly why skew is a term
        of the constraint and not a rounding error.

        * ``OFFSET IN  = TDD_max + skew`` -- `TDC/TDD` is the PHY's worst clock-to-output on `data`/`dir`/`nxt`
          (9.0 ns max), plus the board skew above.
        * ``OFFSET OUT = TSD_min + skew`` -- **a margin, not a budget.** Xilinx checks an output offset as "the
          pad must stay quiet for d before the NEXT clock edge", so the budget it enforces is `period - d`
          (= 10.667 ns at 60 MHz) and `d` is ULPI's `TSD_min` (6.0 ns). Writing `period - TSD` into this field
          is the bug this file had, and `trce`'s own `Requirement:` line is what caught it: it asked for
          clock-to-pad <= 6 ns and invented 4.67 ns of violation on every output. See `ulpi_io_windows` and
          `docs/NOTES.md`; `tools/test_ulpi_ucf.py` now refuses the complement on the ground that *a margin
          cannot depend on the clock while a budget must*.
        * ``VALID = period - TDD_max = 7.667 ns`` -- the honest window: the PHY's byte is valid at the pin
          from TDD_max after its own edge until the next byte replaces it one period later. (The §7.4
          "capture at 7.7 ns" figure is the *internal* setup budget; it happens to be the same number here
          because the budget is the complement of the same TDD_max, but it is not what a VALID window means,
          and an earlier draft of this method computed ``period - 2*TDD_max`` and emitted a **negative**
          window -- which Xilinx accepts without complaint. A constraint the tool will take silently is a
          constraint that needs a test, so see `tools/test_ulpi_ucf.py`.)

        **And the measured fact about this file, which is why it is not silently turned off.** Measured on
        this part (xc6slx100fgg484-**2**, the fastest grade it ships in) with a throwaway that copies this
        constraint block exactly, and reported rather than tuned away:

        | constraints                       | trce result                                  |
        |-----------------------------------|----------------------------------------------|
        | ``VALID 7.667`` (the honest one)  | **HOLD** fails, ~4.8 ns, on `dir`/`nxt`      |
        | ``VALID 16.667`` (widened)        | **SETUP** fails instead                      |

        That is not the constraint being wrong; it is the constraint *working*. With the pads registered in
        fabric, the clock reaches the flop ~3.5-4.8 ns after it reaches the pin while the data path from the
        same pin is ~1.5 ns, so the *next* byte (launched 0-9 ns after the edge) reaches the D pin before the
        capturing edge does. Widening `VALID` does not remove the race, it moves it onto the setup axis. The
        fix order is therefore fixed, in this order:

        1. **IO-register the pads** (ILOGIC/OLOGIC, where clock and data arrive together). This is the only
           fix that keeps the spec's numbers, and it is a gateware change in `pano_ulpi.py`.
        2. Never "fix" it by relaxing `OFFSET`, `VALID` or the clock: a relaxed OFFSET is how a Link that
           corrupts one byte in 10^9 ships as green.
        """
        # The arithmetic and the text live in pano_ulpi.py as pure functions, so they can be asserted
        # without running ISE -- see `ulpi_io_windows` for why that mattered within the hour of writing it.
        # It raises if the window cannot close at this clock, which is a build error and not a UCF line.
        from litex_boards.extensions.pano_ulpi import ulpi_ucf_lines, ulpi_wake_false_path_lines
        for line in ulpi_ucf_lines(clk_net = clk_net, clk_hz = 60e6, skew_ns = skew_ns):
            platform.add_platform_command(line)
        # ... and the one path that must NOT be timed against them. `ulpi_wake` is a `cd_sys` level driven
        # combinationally to the `stp` pad, because every synchroniser into `cd_usb` is built from the clock
        # a suspend stops (`pano_ulpi.py`, decision A); leaving it timed asks Xilinx to meet an `OFFSET`
        # across two clocks that have no phase relationship, on a path with no register to move.
        # `ulpi_wake_false_path_lines` explains why the documented `add_false_path_constraints()` cannot be
        # used here (one endpoint is a pad, not a clock) and why the net name is LiteX's `{cd_sys}`
        # placeholder rather than a string we typed.
        # `cd_sys` is passed as a SIGNAL and not spelled, because `add_platform_command` resolves each
        # `{name}` from its kwargs (`generic_platform.resolve_signals`: `template.format(**name_dict)` with
        # the names coming out of the VerilogNetlist). Passing the bare word raised `KeyError: 'cd_sys'` at
        # build time -- which is the loud failure; the silent one would be a NET line naming a net that does
        # not exist, and Xilinx accepts that with a warning nobody reads while the path stays timed. That is
        # why `tools/test_ulpi_csr_map.py` asserts no brace survives into the emitted `.ucf`.
        if cd_sys is not None:
            for line in ulpi_wake_false_path_lines(stp_net = "ulpi_stp"):
                platform.add_platform_command(line, cd_sys = cd_sys)


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
    parser.add_target_argument("--sdram-clk-phase",      default=260., type=float,
        help="PLL CLKOUT3 phase (SDRAM CK driver). Clean read window on rev C = 252-268 deg; 250 corrupts reads.")
    # Console / reset.
    parser.add_target_argument("--serial-index",  default=0, type=int, help="Console DDC pair (0=micro-HDMI, 1=DVI).")
    parser.add_target_argument("--with-ulpi-probe", action="store_true",
        help="Add the ULPI Probe (PHY/hub reset CSRs, frequency measurement, analyzer). Permanent: the\n"
             "product SoC keeps it. Pair with --cpu-type=None for the bring-up spin (no BIOS, so the DDC\n"
             "pair is the remote channel and not a console); see ~/llm/tests/panologic_g2/MILESTONE1.md.")
    parser.add_target_argument("--w12-pullup", action="store_true",
        help="Constrain usb_refclk (W12) with PULLUP for the 'does the level follow our pull' half of "
             "TODO (a). On Spartan-6 PULLUP is an IOB attribute, so floating and pulled are TWO builds; "
             "it cannot be a CSR bit.")
    parser.add_target_argument("--with-ulpi-master", action="store_true",
        help="Add the ULPI Link and its `ulpi_*` CSRs (RegRead/RegWrite only, spec decision 4). Needs "
             "--with-ulpi-probe, which keeps owning phy_reset, clk_ok and the W12 refclock; the Link takes "
             "the data/stp pads from the probe (decision 8) and needs `probe_w12_arm=1` to keep that clock "
             "alive without a host-side watchdog.")
    parser.add_target_argument("--no-ulpi-io-offsets", action="store_true",
        help="Do not emit the raw-UCF OFFSET constraints on the ULPI pads. A bench diagnostic for which "
             "constraint a timing failure belongs to -- NOT a way to get a green report. (This help used to "
             "say the failure's \"only real fix is IO-registering the pads\"; measured 2026-10-31 that is "
             "worth ~0.3 ns, see bench/2026-10-31_ulpi_iob/ and ADR-0005.)")
    parser.add_target_argument("--ulpi-skew-ns", default=ULPI_SKEW_NS_DEFAULT, type=float,
        help="Board skew allowance, ns, added to OFFSET IN and to the OFFSET OUT margin. Default 0.3 comes "
             "from Microchip AN19.17 s3 (50-60 ohm, <3 in, within 0.5 in of CLKOUT => ~0.1 ns) plus a factor, "
             "because the Pano's layout is unknown to us; 0.0 would claim matched traces. Measure it with a "
             "scope on CLKOUT and one data line, then pass the number.")
    parser.add_target_argument("--ulpi-stall-timeout", default=1 << 16, type=int,
        help="cd_usb cycles a ULPI access waits before it declares itself stuck (2^16 = ~1.1 ms). The PHY "
             "may hold nxt indefinitely per ULPI 1.1, so this bound is ours and is never evidence of a PHY "
             "spec violation.")
    parser.add_target_argument("--ulpi-retry-limit", default=3, type=int,
        help="How many times an access retries a dir-aborted transfer before err_dir_abort (ULPI 1.1 "
             "Table 10 mandates at least one). Always read ulpi_cnt_dir_abort as well: a pass after three "
             "aborts is a different fact from a clean pass.")
    parser.add_target_argument("--no-analyzer", action="store_true",
        help="Drop the LiteScope analyzer from the probe (faster rebuild; the CSR matrix still works).")
    parser.add_target_argument("--no-ulpi-scope", action="store_true",
        help="Drop `usb_scope` (the second LiteScope instance, `--csv usb_scope.csv`), which samples the ULPI "
             "bus at the PHY's own 60 MHz (needs --with-ulpi-master). It costs BRAM-heavy FSMs and puts the "
             "first fabric loads "
             "on the ULPI input pads in the PHY's clock domain, so it is the thing to remove when a timing "
             "failure on those pads needs an owner. Not a way to get a greener report: without it a wrong "
             "byte costs another spin instead of another capture.")
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
        with_ulpi_probe     = args.with_ulpi_probe,
        with_analyzer       = not args.no_analyzer,
        w12_pullup          = args.w12_pullup,
        with_ulpi_master    = args.with_ulpi_master,
        with_ulpi_scope     = not args.no_ulpi_scope,
        ulpi_skew_ns        = args.ulpi_skew_ns,
        ulpi_stall_timeout  = args.ulpi_stall_timeout,
        ulpi_retry_limit    = args.ulpi_retry_limit,
        ulpi_io_offsets     = not args.no_ulpi_io_offsets,
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
