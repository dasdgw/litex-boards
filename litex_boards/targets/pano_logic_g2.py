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

# ULPI Probe ---------------------------------------------------------------------------------------

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
    def __init__(self, sys_clk_freq, ulpi_pads, refclk_pads = None, hub_rst_pad = None, gate_ms = 1,
        clk_ok_min_hz = 45e6, clk_ok_max_hz = 75e6, with_analyzer = True):
        gate = int(sys_clk_freq*gate_ms/1e3)

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
        self.tri = TSTriple(8)
        self.specials += self.tri.get_tristate(ulpi_pads.data)
        self.comb += [
            self.tri.oe.eq(asserted),        # drive the bus idle while the PHY is in reset (ULPI asks
            self.tri.o.eq(0),                #   for exactly this), release it the moment it is not
            ulpi_pads.stp.eq(0),             # never a strobe here: spin 1 transmits nothing
        ]
        self.oe = CSRStatus(description="Current output-enable level (the above, observed).")
        self.comb += self.oe.status.eq(asserted)

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
        # and the two readings are two spins; `w12_mode` keeps only what the fabric can actually switch:
        # 0 = input (drive off), 3 = output static 0, 2 = output 24 MHz (added with the refclk PLL).
        if refclk_pads is not None:
            self.w12_mode = CSRStorage(2, reset=0, description="0=input (drive OFF, the power-up value), "
                                     "3=output static 0, 2=output 24 MHz. Pull-up is --w12-pullup at build "
                                     "time: PULLUP is a Spartan-6 IOB attribute, not runtime-selectable.")
            self.w12_keep = CSRStorage(description="Refresh: write 1 to keep the output enable. It drops "
                                     "itself ~50 ms after the last write -- a crashed harness, a closed "
                                     "laptop or an agent that stops mid-experiment must not leave a net "
                                     "driven whose other driver was never ruled out.")
            self.w12_level   = CSRStatus(description="W12 as read (the real net level, even while driven).")
            self.w12_edges   = CSRStatus(16, description="W12 transitions per gate window. NOT a frequency: "
                                         "sampled in cd_sys, so a 24 MHz net aliases -- 'many' means "
                                         "'something is driving it', which is the actual question.")
            self.w12_drive   = CSRStatus(description="1 = the FPGA is driving W12 right now.")
            self.w12_keep_ok = CSRStatus(description="1 = the keep bit was refreshed inside its window.")

            w12_in   = Signal(name="w12_in")
            keep_ok  = Signal(name="w12_keep_live")
            drive_on = Signal(name="w12_drive_on")

            # The pad: one IOBUF, both directions, and that is deliberate. The read-back (`o`) is the
            # *net*, not our own output register, so "the enable dropped and the net is really not ours"
            # is something the harness can verify instead of assume -- and `w12_level` means something in
            # every mode, including while driving (a drive fight shows up as the level not following us).
            # A Signal driven to 0, not `Constant(0)`: XST's Verilog reader refuses an assignment into a
            # constant on the IOBUF's I pin (ERROR:HDLCompiler:1532 "Cannot assign to a constant value").
            # It also has to be a signal for ticket 06, which puts the 24 MHz clock here.
            self.w12_out = Signal(name="w12_out")     # 0 in spin 1; the refclk PLL in ticket 06
            self.comb  += self.w12_out.eq(0)
            self.specials += Tristate(refclk_pads, i=self.w12_out, o=w12_in, oe=drive_on)

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

            # The watchdog: reloaded by any write to w12_keep. ~50 ms is far above a CSR round trip and
            # far below "somebody walked away". Note it gates the *enable*, not the mode register: the
            # mode can say 2 or 3 and the pin still must not be driven unless the keep is live.
            # reset=0, deliberately: **expired at power-up**. Giving the timer its full window at boot
            # would mean "off at power-up" was only true for the first 50 ms, and the whole point of the
            # watchdog is that a bitstream nobody is talking to cannot drive the net.
            keep_timer = Signal(32, reset=0)
            self.sync += If(self.w12_keep.wr_stb, keep_timer.eq(int(sys_clk_freq*50e-3)))
            self.sync += If(keep_timer != 0, keep_timer.eq(keep_timer - 1))
            self.comb += [
                keep_ok.eq(keep_timer != 0),
                # The watchdog gates the *enable*, not the mode register: `w12_mode` may say 2 or 3 and the
                # pin still is not driven unless the keep bit is live.
                drive_on.eq(((self.w12_mode.storage == 3) | (self.w12_mode.storage == 2)) & keep_ok),
                self.w12_drive.status.eq(drive_on),
                self.w12_keep_ok.status.eq(keep_ok),
            ]
        else:
            refclk_pads, w12_in = None, Signal()
            self.w12_mode   = CSRStorage(2)
            self.w12_keep   = CSRStorage()
            self.w12_level  = CSRStatus()
            self.w12_edges  = CSRStatus(16)
            self.w12_drive  = CSRStatus()
            self.w12_keep_ok = CSRStatus()

        # The groups, built last (they reference everything above) and instantiated by the SoC.
        #
        # ONE analyzer because groups are **multiplexed**: a second instance would halve the sampling rate
        # of both, and group 0 is the one that has to hold the whole cause-and-effect story.
        self.analyzer_groups = {
            # group 0: sequence state, both resets, oe, clk_ok, raw + synchronised `dir`, and the high bits
            # of the clock counter -- cause and effect on a single time axis.
            0 : [state, rst_phy_net, self.hub_rst.storage, self.oe.status, self.clk_ok.status,
                 ulpi_pads.dir, self.dir_sync, self.ctr_hi],
            # group 1: slow/secondary, deliberately not time-critical.
            1 : [self.nxt_sync, ulpi_pads.nxt, self.hub_rst_inv.storage, self.phy_rst_inv.storage,
                 self.seq_auto.storage, self.tri.o],
            # group 2: W12 -- what we asked for, whether we are actually driving, and what the net says.
            # Separate from group 0 because "we enabled the driver and the level did not follow" is a
            # different question from the reset/clock coupling, and merging them costs rate to both.
            2 : [self.w12_mode.storage, self.w12_drive.status, self.w12_keep_ok.status,
                 self.w12_level.status, w12_in],
            # group 3: RESERVED for spin 2's ULPI bus capture (see RESERVED_* below); tied to 0 now so the
            # slot exists in the CSV and the client can be pointed at it.
            3 : [self.rsv_spin2],
        }

    # Spin 2 (the ULPI ID read) must land without renaming anything measured here or moving a group.
    # It needs: ulpi_cmd/ulpi_addr/ulpi_wdata/ulpi_rdata + ulpi_go, a CDC bridge into cd_usb, and the
    # 60 MHz bus capture. Those slots are group 2 and the names below; they are documented, not built.
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
        **kwargs):
        platform = pano_logic_g2.Platform(revision=revision)
        assert sdram_chip in ["a", "b"]

        # CRG --------------------------------------------------------------------------------------
        # Requested once here and handed down: `platform.request()` consumes the resource, so a second
        # request for the same group anywhere else fails with "Resource not found".
        ulpi_pads   = platform.request("ulpi") if with_ulpi_probe else None
        refclk_pads = platform.request("usb_refclk") if with_ulpi_probe else None

        self.crg = _CRG(platform, sys_clk_freq,
            sdram_chip       = sdram_chip,
            ulpi_pads        = ulpi_pads,
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
                hub_rst_pad  = platform.request("usb_hub_rst_n"),
                with_analyzer= with_analyzer)
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
    parser.add_target_argument("--no-analyzer", action="store_true",
        help="Drop the LiteScope analyzer from the probe (faster rebuild; the CSR matrix still works).")
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
