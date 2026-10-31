#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2026 Pano Logic G2 reverse-engineering community
# SPDX-License-Identifier: BSD-2-Clause

"""The ULPI **Link** for the Pano Logic G2: ``RegRead`` + ``RegWrite``, nothing else.

Why this block exists and stays (it is not a bring-up throwaway): Track A's PHY path
(``ultraembedded/core_ulpi_wrapper`` -> ``ulpi_wrapper.v``) writes Function Control (``0x04``) and OTG
Control (``0x0A``) only, so Interface Control, the Debug register (the only pre-enumeration witness of the
USB ``link state``) and a soft PHY reset have no door. Every later PHY question needs this door.

Every encoding, address and timing figure is owned by ``RESEARCH.md`` §7 in the project docs and cited
there rather than restated. The two that appear below because they *are* the state machine:
``TX CMD = 0x80|addr`` (RegWrite) and ``0xC0|addr`` (RegRead) — ULPI 1.1 Table 6, bits 7:6 = ``10b``/``11b``.

Choreography, quoted rather than remembered (ULPI 1.1 §3.8.3.1, §3.8.4)
----------------------------------------------------------------------

*RegWrite* — "the Link sends a register write command and waits for nxt to assert. In the cycle after nxt
asserts, the Link sends the register write data and waits for nxt to assert again. When nxt asserts the
second time, the Link asserts stp in the following cycle to complete the operation." **Two bytes on the
wire plus a one-cycle ``stp``.** A "3-byte write" is wrong, and this project's own milestone doc used to
say one.

*``stp`` is an END marker, not a start marker* — "stp … If the Link is sending data to the PHY, stp
indicates the last byte of data was on the bus in the previous cycle"; "the Link must assert stp to signal
the end of a USB transmit packet or a register write operation". The TX CMD cycle is therefore driven with
``stp`` **low**; asserting it early tells the PHY the transfer ended before it began.

*RegRead* — "the Link sends a register read command and waits for nxt to assert. In the cycle after nxt
asserts, the PHY asserts dir to gain control of the data bus. In the cycle after dir asserts, the PHY must
return the register read data. The PHY does not assert nxt when dir is asserted during the register read
operation." So: **no ``nxt`` on a read, no ``stp`` on a read, and the byte arrives one cycle after
``dir``** — which is why the states are named ``RD_TURN``/``RD_WAIT_DIR``/``RD_DATA`` and why the
testbench asserts the arrival cycle instead of just the happy value. Reading that gap as a stall is the
likeliest way to make a healthy PHY look broken.

*Bus idle first* — "Anytime the Link wants to write or read a ULPI register, the Link will need to wait
until DIR is low" (USB3300), so ``go`` while ``dir`` is high **waits and counts** (``BUSY_WAIT``,
``result_bus_waited``) rather than erroring: the PHY legitimately holds ``dir`` after reset until its PLL
settles, and an immediate error would cost a 60 ms round trip per retry.

*Abort* — "the PHY can abort the Link by asserting dir" (§3.8.4.1) and "the Link must retry the RegWrite
when the bus is idle" (§3.8.3.1): a ``dir`` abort is retried up to ``retry_limit`` **and still reported**,
because "passed after three aborts" and "passed cleanly" are different facts. The Link aborts a PHY holding
``dir`` by pulsing ``stp`` (§3.8.4.2) — that is ``abort``; §3.8.4.2's own caveat ("If the Link does not
immediately drive a TX CMD, the PHY is allowed to re-assert dir") is why it releases the bus and promises
nothing more.

Interface (tickets 02 and 04 code against this — keep it in sync)
----------------------------------------------------------------

``UlpiMaster``, in the PHY's clock domain. The request is a **level** and the completion a **one-cycle**
``ack``, because that is the shape ``ReqAckCDC`` presents on its far side; ``UlpiLink`` is the ``cd_sys``
side and is what a CSR front-end or a testbench drives.

::

    req             i 1  level: a request is offered (rw/addr/wdata valid with it)
    rw              i 1  0 = RegWrite, 1 = RegRead
    addr            i 6  immediate register address (ULPI's 6-bit space; RESEARCH.md §7.2)
    wdata           i 8  byte to write
    ack             o 1  one cycle at the end of the access; ack_data valid with it
    ack_data        o 30 packed result, see ACK_FIELDS / ack_unpack — never read a piece of it alone
    xfer_active     o 1  level: high for the whole span, INCLUDING an access that never ends
    state           o 4  the FSM, for the analyzer (STATE below)
    phy_reset       i 1  level: PHY reset asserted now — drives the bus-idle level
    clear           i 1  level: hold this master off the bus and clear its FSM (see "The clear")
    abort_req       i 1  pulse: release a PHY that is holding `dir` (the CSR behind it is
                     `ulpi_abort`; the `_req` suffix is because a Verilated C++ model cannot have a
                     port named `abort` -- it collides with libc's abort())
    bus_grant       i 1  permission to drive data — tied to 1 in spin 2, see the comment at `bus_grant`
    dir_in, nxt_in  o 1  registered view of the pads, for the analyzer
    oe_out, stp_out o 1  registered view of what *we* drive, for the analyzer
    tx_byte         o 8  the byte we are driving, whether or not `oe` says so

``UlpiLink`` adds the ``cd_sys`` side (``h_req``/``h_ack``/``h_busy``, one cycle each, ``h_ack_data`` the same
30-bit bundle) and the latched status described on that class.

``STATE`` (4 bits, 11 states): ``0 IDLE · 1 BUSY_WAIT · 2 WR_CMD · 3 WR_DATA · 4 WR_STP · 5 RD_CMD ·
6 RD_TURN · 7 RD_WAIT_DIR · 8 RD_DATA · 9 ABORT · 10 FINISH``. The analyzer taps ``state``, so these numbers
are what a saved VCD and every bench note that quotes a state means: renumbering one invalidates them all.

``stall_timeout`` (default ``2**16`` ≈ 1.1 ms at 60 MHz) is **ours, not the spec's**: "A register read is
the only instance where ULPI does not use nxt to throttle data", and ULPI 1.1 bounds ``nxt`` throttling not
at all (searched: there is no "three-strike rule" in it). A timeout here must sit far above any legal
throttle, must be counted, and must never be reported as a PHY spec violation. ``retry_limit`` default
**3**: §3.8.3.1 says *must retry*, and a forever-retry is a hang with a nicer name.

The clear, and why it is a level rather than ``always @(posedge clk or posedge rst)``
------------------------------------------------------------------------------------

Spec decision 6 asked for an async clear from the PHY-reset net OR'd with a boot counter, because that is
the only reset that works with the clock **stopped** — and this domain's clock *is* the PHY's, which stops
whenever the PHY is in reset. migen's generator emits synchronous resets only, so the *guarantee* is
rebuilt from two expressible things, both load-bearing:

1. ``driving`` is qualified with ``~clear`` **combinationally**, so while ``clear`` is high the pads sit at
   the ULPI idle level (``data = 00h``, ``stp`` low) whatever the frozen FSM was doing. A PHY reset can
   never leave us mid-byte on a bus we are no longer clocked to see.
2. :func:`UlpiClear` — in ``cd_sys``, which always has a clock — holds ``clear`` asserted for
   ``relock_ms`` **after** PHY reset releases, long after the PHY's clock can return. So the first edge a
   frozen FSM ever takes is an edge with ``clear`` high: an access stuck in ``RD_TURN`` cannot resume a
   transaction that no longer exists. The host learns of it from the *bridge* (``err_lost``), the side that
   still has a clock to report with.

That is the recovery property decision 6 asked for, and ``reset_mid`` in ``tools/ulpi_vectors.tsv`` proves
it offline. The literal async-clear flip-flop is what ``ulpi_wrapper.v`` does; if a spin ever needs it,
this file is the place to change and the testbench is already here.

``ulpi_status``, and why its layout is a table and not a sentence
----------------------------------------------------------------

Ticket 02 builds the CSRs and ticket 04 builds the harness that reads them, so a layout written only in
prose is a layout that can silently disagree with itself. ``STATUS_FIELDS`` is the list of
``(name, width)`` pairs ``UlpiLink.status()`` cats together, LSB first, and :func:`status_decode` is its
inverse; the CSR block, the offline python twin and the bench harness all read that one table. A status
word whose bits were documented rather than *derived* is how a bench note and a bitstream end up disagreeing
about which bit is ``busy`` -- and only one of them is wrong, at the bench, at 1 am.
"""

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import LiteXModule

from litex_boards.extensions.pano_cdc import CdcLevel, ReqAckCDC

__all__ = ["STATE", "REQ_FIELDS", "ACK_FIELDS", "req_pack", "ack_unpack",
           "STATUS_FIELDS", "status_decode", "ulpi_io_windows", "ulpi_output_budget", "ulpi_ucf_lines",
           "UlpiMaster", "UlpiLink", "UlpiClear"]

# The analyzer taps `state`, so these numbers are part of what a capture *means*: renumbering one
# invalidates every saved .gtkwave layout and every bench note that quotes a state.
(IDLE, BUSY_WAIT, WR_CMD, WR_DATA, WR_STP, RD_CMD, RD_TURN, RD_WAIT_DIR, RD_DATA, ABORT,
 FINISH) = range(11)
STATE = dict(IDLE=IDLE, BUSY_WAIT=BUSY_WAIT, WR_CMD=WR_CMD, WR_DATA=WR_DATA, WR_STP=WR_STP,
             RD_CMD=RD_CMD, RD_TURN=RD_TURN, RD_WAIT_DIR=RD_WAIT_DIR, RD_DATA=RD_DATA, ABORT=ABORT,
             FINISH=FINISH)

# The `ulpi_status` word: (name, width), LSB first, and the *only* place its layout is written down.
# `UlpiLink.status_signals()` returns the signals in exactly this order; `status_decode` is the inverse,
# shared by the python twin and the bench harness so that neither re-derives an offset by hand.
STATUS_FIELDS = [("busy", 1), ("done", 1), ("err_dir_abort", 1), ("err_nxt_timeout", 1),
                 ("err_no_clock", 1), ("err_reset", 1), ("dir", 1), ("oe", 1), ("state", 4)]
STATUS_WIDTH  = sum(w for _, w in STATUS_FIELDS)          # 12


def status_decode(word):
    """``ulpi_status`` as read -> {name: value}. Pure, and shared by every reader of the word."""
    out, off = {}, 0
    for name, width in STATUS_FIELDS:
        out[name] = (word >> off) & ((1 << width) - 1)
        off += width
    return out


# ---------------------------------------------------------------------------------------------
# The pad timing, as arithmetic -- because a constraint the tool accepts silently is a constraint
# that needs a test. `tools/test_ulpi_ucf.py` in ~/llm/tests/panologic_g2 asserts these numbers.
# ---------------------------------------------------------------------------------------------

# USB 2.0 ULPI 1.1 Table 5, the 192.6..255 MHz column that a 60 MHz `C12` selects. Sourced once, in
# RESEARCH.md §7.3; the numbers are the *wire* budget, not an internal setup budget.
ULPI_TDD_MAX_NS = 9.0    # CLKOUT rising -> PHY drives data/stp (max)
ULPI_TSD_MIN_NS = 6.0    # the PHY must see our data this far before the next CLKOUT rising edge


def ulpi_io_windows(clk_hz = 60e6, skew_ns = 0.0, tdd_ns = ULPI_TDD_MAX_NS, tsd_ns = ULPI_TSD_MIN_NS):
    """``(offset_in, valid_in, offset_out)`` in ns for the ULPI pads, relative to the PHY's clock.

    * ``offset_in``  = TDD_max + skew -- our worst-case arrival, so the internal setup budget is the rest
      of the period.
    * ``valid_in``   = period - offset_in -- the byte is valid at the pin from TDD_max after the PHY's edge
      until the next byte replaces it one period later. **Not** ``period - 2*TDD_max``, which is what an
      earlier draft of this file said and what it then *emitted*: at 60 MHz that is -1.333 ns, a window
      smaller than nothing, and Xilinx wrote it into the UCF without a word of complaint. That is the whole
      reason this function exists separately from the code that formats it.
    * ``offset_out``  = **TSD_min + skew = 6.0 ns, and it does not depend on the clock.** This is the line
      that was wrong before, and wrong in the way that looks like a hardware problem: `OFFSET = OUT d` in
      Xilinx's model is the **quiet margin the data must hold before the NEXT clock edge**, so the tool's own
      requirement is ``period - d`` (measured: `trce` printed ``Requirement: 5.999ns`` for
      ``OFFSET = OUT 10.667`` at a 16.667 ns period). Emitting the *budget* (10.667) instead of the *margin*
      (6.0) therefore demanded clock-to-pad <= 6.0 ns when ULPI only asks for <= 10.667 ns -- a phantom
      4.67 ns violation on every output, on top of the real one. The complement was inherited from the
      ticket's wording ("OFFSET_OUT ~ 10.5 ns = period - TSD - margin") and nobody checked it against the
      tool's arithmetic; the fact that both numbers are complements of the same two quantities at this one
      clock is what made it survivable in prose. A margin is a margin at any clock: that invariance is now
      asserted in `tools/test_ulpi_ucf.py`, because it is the only cheap thing that distinguishes the two
      readings.

    Raises rather than emitting a window that cannot exist: at a clock where the PHY's own TDD exceeds the
    period there is no constraint to write, and the build must say so instead of synthesising a race.
    """
    period = 1e9/clk_hz
    offset_in  = tdd_ns + skew_ns
    valid_in   = period - offset_in
    offset_out = tsd_ns + skew_ns        # a MARGIN, not a budget -- see the docstring
    if valid_in <= 0:
        raise ValueError(f"ULPI input window cannot close at {clk_hz/1e6:.1f} MHz: TDD_max "
                         f"{offset_in:.2f} ns leaves {valid_in:.2f} ns of valid window. A 60 MHz C12 is "
                         f"the fastest this budget supports; check --sys-clk-freq and the PHY's CLKOUT mode.")
    if period - offset_out <= 0:
        raise ValueError(f"ULPI output budget cannot close at {clk_hz/1e6:.1f} MHz: the PHY needs TSD_min "
                         f"{tsd_ns:.2f} ns of the {period:.2f} ns period, leaving nothing for our clock "
                         f"path. This PHY cannot run this fast relative to CLKOUT.")
    return offset_in, valid_in, offset_out


def ulpi_output_budget(clk_hz = 60e6, skew_ns = 0.0, tsd_ns = ULPI_TSD_MIN_NS):
    """The clock-to-pad budget the tool will actually check, in ns: ``period - (TSD_min + skew)``.

    Separate function on purpose. It is the number people want when they ask "how much room do we have",
    and it is NOT the number that goes in the UCF -- confusing them is the 4.67 ns mistake described in
    :func:`ulpi_io_windows`. `trce`'s ``Requirement:`` line for an ``OFFSET = OUT`` is this value.
    """
    return 1e9/clk_hz - tsd_ns - skew_ns


def ulpi_ucf_lines(clk_net = "usb_clk", clk_hz = 60e6, skew_ns = 0.0, data_net = "ulpi_data",
                    in_nets = ("ulpi_dir", "ulpi_nxt"), out_nets = ("ulpi_stp",), bits = 8):
    """The UCF text for the ULPI pads, as data.

    A function rather than f-strings inside the target, so the emitted text is assertable without running
    ISE (which is the only way this block gets checked on every commit). One line per net and per bit, no
    wildcards: `trce` reports failures per bit, and `NET "ulpi_data[*]"` would hide *which* bit failed --
    the difference between "a byte lane is bad" and "the whole bus is bad".
    """
    offset_in, valid_in, offset_out = ulpi_io_windows(clk_hz = clk_hz, skew_ns = skew_ns)
    lines = []
    # `data` is bidirectional and ULPI puts a budget on each half, so it appears in both lists.
    for net in list(in_nets) + [f"{data_net}[{i}]" for i in range(bits)]:
        lines.append(f'NET "{net}" OFFSET = IN {offset_in:.3f} ns VALID {valid_in:.3f} ns '
                     f'BEFORE "{clk_net}";')
    for net in list(out_nets) + [f"{data_net}[{i}]" for i in range(bits)]:
        lines.append(f'NET "{net}" OFFSET = OUT {offset_out:.3f} ns BEFORE "{clk_net}";')
    return lines


CMD_REG_WRITE = 0x80      # TX CMD bits 7:6 = 10b, immediate 6-bit address in 5:0
CMD_REG_READ  = 0xC0      # TX CMD bits 7:6 = 11b
# Deliberately absent: 0x00 NOOP (that is the idle *level*, not something to send), 0x40|PID packet
# transmit (Track A's job; doing it here would mean a second, worse ulpi_wrapper), and the 0x2F extended
# forms (the USB3300 tells the Link not to read or write anything outside its register table).

# The ack carries the whole result bundle rather than one byte, because the result is not interpretable in
# pieces: `rdata` without its `bus_waited` is a number you cannot judge — "passed after waiting 3 cycles"
# and "passed after waiting 40 000" are the same verdict today and a different diagnosis next month.
REQ_FIELDS = ["rw", "addr", "wdata"]                                        # 1 + 6 + 8       = 15
ACK_FIELDS = ["dir_abort", "nxt_timeout", "rdata", "bus_waited", "retry"]   # 1 + 1 + 8 + 16 + 4 = 30


def req_pack(rw, addr, wdata):
    return Cat(rw, addr, wdata)


def ack_unpack(sig):
    """Field views of an ack payload, in the order named by ``ACK_FIELDS``."""
    return dict(dir_abort=sig[0], nxt_timeout=sig[1], rdata=sig[2:10],
                bus_waited=sig[10:26], retry=sig[26:30])


def _inc_sat(sig):
    return sig.eq(Mux(sig != (2**len(sig) - 1), sig + 1, sig))




class UlpiClear(Module):
    """``cd_sys``: the level that holds the Link off the bus and clears its FSM.

    ``clear = ~boot_done | phy_reset | relock``, and ``relock`` keeps it asserted for ``relock_ms``
    **after** PHY reset releases. That tail is not caution, it is the mechanism -- see "The clear" in this
    file's docstring: without it a PHY reset asserted while ``C12`` was already stopped is a reset the FSM
    never observes, and an access frozen in ``RD_WAIT_DIR`` simply resumes when the clock comes back. With
    it, the first edge a frozen FSM ever takes is an edge with ``clear`` high.

    A boot counter rather than a config pin: ``cd_sys`` itself only starts clocking when the Ethernet PHY's
    125 MHz appears (``RESEARCH.md``), so "counted N cycles of the sys clock" is the only definition of
    *booted* that means anything on this board.
    """
    def __init__(self, sys_clk_freq, phy_reset, boot_ms = 1, relock_ms = 10):
        self.clear     = Signal()
        self.boot_done = Signal()
        boot_cycles    = int(sys_clk_freq*boot_ms/1e3)
        relock_cycles  = int(sys_clk_freq*relock_ms/1e3)
        boot, hold, rst_d = Signal(32), Signal(32), Signal()
        self.sync += [
            rst_d.eq(phy_reset),
            If(boot < boot_cycles, boot.eq(boot + 1)),
            If(phy_reset, hold.eq(relock_cycles))      # asserted, and every cycle since it was
             .Elif(hold != 0, hold.eq(hold - 1)),
        ]
        self.comb += [
            self.boot_done.eq(boot >= boot_cycles),
            self.clear.eq((boot < boot_cycles) | phy_reset | (hold != 0)),
        ]


class UlpiMaster(Module):
    """One register path into a ULPI PHY. Lives in the PHY's clock domain (``domain``, default ``usb``).

    The request is a **level** (``req``) and the completion a **one-cycle** ``ack``, which is the shape
    :class:`~litex_boards.extensions.pano_cdc.ReqAckCDC` presents on its far side; ``req_taken`` makes sure
    a request still held high after its own ack cannot start a second transaction.

    Cycle convention, stated once because every state depends on it -- and **split in two, because the
    bench proved one half of it false.** ``dir_in``/``nxt_in`` are registered, so in any cycle the FSM sees
    what the PHY drove in the **previous** cycle. For the **read** that registration is not a convenience,
    it is the alignment: §3.8.3.1 has the PHY assert ``dir`` and return the byte in the cycle after, so a
    one-cycle-late ``dir`` is exactly the cycle the byte is valid (measured that way on ``ulpi_t17``:
    ``dir`` 8333 ns, byte 8349 ns, ``bench/2026-10-31_capture_fresh_00.vcd``).

    For the **write** the same registration was a bug. §3.8.3.1: "the Link sends a register write command
    and waits for nxt to assert. **In the cycle after nxt asserts, the Link sends the register write
    data**". Reacting to a *registered* ``nxt`` puts the byte two cycles after the assertion, and the
    USB3300 does not wait: it samples the bus in the cycle after its own ``nxt`` and takes whatever is
    there. Measured on this board -- a write of ``0x3c`` to Scratch stored ``0x96``, which is the TX CMD
    ``0x80|0x16`` still on the bus when it sampled, and the access reported **no error**
    (``bench/2026-10-31_capture_write16_0x3C.vcd``, and the byte read back for twelve different written
    values in ``bench/2026-10-31_ulpi_scratch_walk.tsv``). The claim that justified the old wiring -- "the
    PHY holds ``nxt`` until the Link's next byte appears" -- is what the capture falsifies: on a write
    ``nxt`` is high for two consecutive cycles and then it samples.

    So the write states read ``pads.nxt`` **raw**, and they are the only states that do. That is the same
    pad-to-flop class the ``oe`` turnaround term already lives in, so ``OFFSET = IN`` covers it (slice 02's
    constraint row, worst input slack +3.380 ns on ``ulpi_t17``). ``nxt_in`` stays registered and stays the
    analyzer's tap: the capture must show what the PHY drove, not what our pipeline made of it.
    """

    def __init__(self, pads, stall_timeout = 1 << 16, retry_limit = 3, domain = "usb",
                 bus_waited_width = 16, name = "ulpi"):
        assert 1 <= retry_limit <= 15, "retry_limit has to fit the 4-bit retry counter"
        assert stall_timeout >= 1024, "a shorter timeout can fire during a legal nxt throttle (§3.8.3: unbounded)"

        # --- host-facing (the bridge's far side) --------------------------------------------------------
        self.req         = Signal()
        self.rw          = Signal()
        self.addr        = Signal(6)
        self.wdata       = Signal(8)
        self.ack         = Signal()
        self.ack_data    = Signal(30)
        self.xfer_active = Signal()
        self.state       = Signal(4)
        self.phy_reset   = Signal()
        self.clear       = Signal()
        self.abort_req   = Signal()
        # DO NOT fold this into `req`, and do not delete it because spin 2 ties it to 1. The next owner of
        # these pads is Track A's `ulpi_wrapper.v`, which raises `stp` on its own the moment the HCD
        # transmits: two engines on one bus cannot be separated by a *convention*, so the arbitration
        # promise has to live in the interface. One input now, versus a bus fight nobody would see from the
        # host side. (Spec decision 9.)
        self.bus_grant   = Signal()

        # --- analyzer taps, all registered --------------------------------------------------------------
        # LiteScope's format_groups() de-duplicates through a dict and a migen operator is unhashable, so
        # anything worth capturing needs a name of its own (spin 1 measured: the analyzer refuses to tap
        # `~dir`). It is also what you want in the VCD: one name, one net, one meaning.
        self.dir_in  = Signal()
        self.nxt_in  = Signal()
        self.oe_out  = Signal()
        self.stp_out = Signal()
        self.tx_byte = Signal(8)

        sync = self.sync if domain == "sync" else getattr(self.sync, domain)

        # --- pads -----------------------------------------------------------------------------------------
        self.tri = TSTriple(8, name = name + "_tri")
        self.specials += self.tri.get_tristate(pads.data)

        # dir/nxt are registered **one stage, on purpose, and this must not become a MultiReg.** They are
        # synchronous here -- the PHY drives them from the same CLKOUT that clocks this domain, inside the
        # 9 ns TDC/TDD budget of `RESEARCH.md` §7.4, which slice 02 closes with OFFSET_IN -- so the problem
        # is propagation and not metastability, and a 2-FF synchroniser would push the read-byte sample one
        # cycle past the only cycle in which the PHY drives it. The alignment is worth seeing: the PHY
        # asserts `dir` in cycle D and returns the byte in D+1 (§3.8.3.1), so our registered copy is high
        # *during* D+1, which is exactly when RD_WAIT_DIR samples the pad. One stage is what makes that line
        # correct rather than a cycle late.
        sync += [self.dir_in.eq(pads.dir), self.nxt_in.eq(pads.nxt)]

        # Every one of these is named on purpose: the synth report, the LiteScope VCD and the iverilog
        # dump are the only debug interface this block has, and an unnamed Signal elaborates as
        # `ulpimaster7` -- which is what you then have to translate by hand at 1 am from a .gtkwave file.
        state     = Signal(4, reset = IDLE, name = "state")
        driving   = Signal(name = "driving")
        stp_drive = Signal(name = "stp_drive")
        tx_byte   = Signal(8,   name = "tx_byte")
        rw_r      = Signal(name = "rw_r")
        addr_r    = Signal(6,   name = "addr_r")
        wdata_r   = Signal(8,   name = "wdata_r")
        waited    = Signal(bus_waited_width, name = "waited")
        retries   = Signal(4,   name = "retries")
        aborted   = Signal(name = "aborted")
        timed_out = Signal(name = "timed_out")
        req_taken = Signal(name = "req_taken")
        tmo       = Signal(32,  name = "tmo")
        tmo_hit   = Signal(name = "tmo_hit")

        self.comb += [
            self.state.eq(state), self.tx_byte.eq(tx_byte), self.stp_out.eq(stp_drive),
            self.oe_out.eq(self.tri.oe),
            self.xfer_active.eq(state != IDLE),
        ]

        # DO NOT register the `dir` term below, and DO NOT delete it either.
        #   * Delete it and two drivers can own the data bus at once: the result is a byte that is neither
        #     ours nor the PHY's and nothing on the host side says so. §3.8.4.2: "In the next cycle, the
        #     PHY must de-assert dir", so a registered `dir` keeps us driving exactly one cycle into the
        #     PHY's ownership of the bus. Both prior references for this exact board do it combinationally
        #     and both flag the path: `ulpi_wrapper.v` `pads_data.oe.eq(~pads.dir)`, `UlpiCtrl.scala`
        #     `data.writeEnable := !direction`.
        #   * Register it and it stops doing its job: the PHY takes the bus in a cycle our own pipeline has
        #     not reached yet.
        # `~clear` is the other half of "The clear" in this file's docstring -- combinational *because* the
        # clock may already have stopped, leaving this the only logic able to return the bus to the ULPI
        # idle level (data 00h, stp low). `phy_reset` in the same term is ULPI's own requirement that the
        # Link drive the idle value while the PHY is in reset.
        # `stp` is a dedicated output and is deliberately NOT qualified by `dir`: §3.8.4.2 asks for it
        # precisely while the PHY is holding `dir`.
        drive_now = driving & ~self.clear & self.bus_grant
        # TWO command bytes, because there are two moments a command is issued and they do not have the same
        # information available:
        #   `cmd_new` at accept, from the request itself -- `rw_r`/`addr_r` are assigned on that very edge,
        #     so building the byte from them there yields the *previous* access's rw. That is not a
        #     hypothetical: it was this module's first bug, it sent every RegRead out as a RegWrite, and the
        #     simulation caught it in one vector (`tx=0x80` while `rw_r=1`).
        #   `cmd_r` on a retry out of BUSY_WAIT, where the request pins have long since moved on and only
        #     the latched copy is the right answer.
        cmd_new, cmd_r = Signal(8, name = "cmd_new"), Signal(8, name = "cmd_r")
        self.comb += [
            cmd_new.eq(Mux(self.rw, CMD_REG_READ | self.addr, CMD_REG_WRITE | self.addr)),
            cmd_r.eq(Mux(rw_r, CMD_REG_READ | addr_r, CMD_REG_WRITE | addr_r)),
            self.tri.oe.eq(self.phy_reset | (drive_now & ~pads.dir)),
            self.tri.o.eq(Mux(drive_now & ~self.phy_reset, tx_byte, 0)),
            # `stp` is a separate pad, not part of the tri-state, and it is qualified by `~clear` for the
            # same reason the data pad is: the ULPI idle level is `stp` low with `data` at 00h, and that is
            # what must be on the pins whenever the PHY is in reset or the Link has been cleared. It is NOT
            # qualified by `bus_grant`: §3.8.4.2 asks the Link to raise `stp` while the PHY holds `dir`, so
            # the release path must work in exactly the moment the arbitration is telling us not to talk.
            pads.stp.eq(stp_drive & ~self.clear),
        ]

        # A `dir` abort (§3.8.4.1) is not a failure the first `retry_limit` times: "the Link must retry the
        # RegWrite when the bus is idle" (§3.8.3.1). Retry through BUSY_WAIT, because by definition the bus
        # is *not* idle while `dir` is high -- and keep `aborted` set, so a successful retry reports "passed
        # with a note" rather than a clean pass. Exhausting the budget ends the access with the bit set too.
        def on_dir_abort():
            return [
                aborted.eq(1), driving.eq(0), tx_byte.eq(0),
                If(retries < retry_limit, [_inc_sat(retries), state.eq(BUSY_WAIT)]).Else(
                    state.eq(FINISH)),
            ]

        issue = Signal(name = "issue")
        self.comb += issue.eq(self.bus_grant & ~self.dir_in)

        # The request is a LEVEL from the bridge and stays high for a few cycles after its own ack (the
        # 2-phase loop has to carry the ack home). `req_taken` is what stops a request servicing itself
        # twice; it is released when the level drops, never at accept time -- clearing it at accept is the
        # bug where the *next* access is swallowed, because the level has not dropped yet then.
        sync += If(~self.req | self.clear, req_taken.eq(0))

        sync += If(self.clear, [
            # Everything except the result bundle: `clear` is also how a PHY reset mid-transaction gets
            # reported, and the host must still be able to read *which* access died when it comes back.
            state.eq(IDLE), driving.eq(0), stp_drive.eq(0), tx_byte.eq(0),
            waited.eq(0), retries.eq(0), aborted.eq(0), timed_out.eq(0), req_taken.eq(0), tmo.eq(0),
        ]).Elif(1, [
            stp_drive.eq(0),
            If(self.abort_req & (state != IDLE), [                 # §3.8.4.2: release a PHY holding `dir`
                state.eq(ABORT), driving.eq(0), tx_byte.eq(0), aborted.eq(1),
            ]).Else(
                Case(state, {
                    IDLE: [
                        driving.eq(0),
                        If(self.req & ~req_taken, [
                            rw_r.eq(self.rw), addr_r.eq(self.addr), wdata_r.eq(self.wdata),
                            req_taken.eq(1), waited.eq(0), retries.eq(0), aborted.eq(0),
                            timed_out.eq(0), tmo.eq(0),
                            If(issue, [
                                driving.eq(1), tx_byte.eq(cmd_new),
                                state.eq(Mux(self.rw, RD_CMD, WR_CMD)),
                            ]).Else(
                                # "Anytime the Link wants to write or read a ULPI register, the Link will
                                # need to wait until DIR is low" (USB3300 §6.1). Count the wait, never
                                # error on it: the PHY legitimately holds `dir` after reset until its PLL
                                # settles, and an error here would cost a 60 ms round trip per retry.
                                state.eq(BUSY_WAIT),
                            ),
                        ]),
                    ],
                    BUSY_WAIT: [
                        driving.eq(0), _inc_sat(waited),
                        If(issue, [
                            driving.eq(1), tx_byte.eq(cmd_r),
                            state.eq(Mux(rw_r, RD_CMD, WR_CMD)),
                        ]),
                    ],
                    # RAW `pads.nxt`, deliberately, and one cycle earlier than the registered copy would
                    # allow: §3.8.3.1 puts the data byte "in the cycle after nxt asserts", and a Link that
                    # reacts to `nxt_in` puts it in the cycle after that -- which is how `0x80|addr` got
                    # stored in place of the payload on silicon. The `dir` abort check stays on the
                    # registered copy: an abort is retried, so its exact cycle is not load-bearing, and
                    # routing it raw would put a second combinational load on the same pad for no gain.
                    WR_CMD: [                                  # TX CMD 0x80|addr, `stp` low
                        If(self.dir_in, on_dir_abort()).Elif(pads.nxt, [
                            state.eq(WR_DATA), tx_byte.eq(wdata_r),
                        ]),
                    ],
                    WR_DATA: [                                 # the byte, then `stp` next cycle
                        If(self.dir_in, on_dir_abort()).Elif(pads.nxt, [
                            state.eq(WR_STP), stp_drive.eq(1), tx_byte.eq(0),
                        ]),
                    ],
                    WR_STP: [                                  # exactly one cycle: 1-cycle END marker
                        driving.eq(0), stp_drive.eq(0), tx_byte.eq(0), state.eq(FINISH),
                    ],
                    RD_CMD: [                                  # TX CMD 0xC0|addr, `stp` low
                        If(self.dir_in, on_dir_abort()).Elif(self.nxt_in, [
                            state.eq(RD_TURN), driving.eq(0), tx_byte.eq(0),
                        ]),
                    ],
                    RD_TURN: [
                        driving.eq(0), tx_byte.eq(0),
                        If(self.dir_in, state.eq(RD_DATA)).Else(state.eq(RD_WAIT_DIR)),
                    ],
                    RD_WAIT_DIR: [
                        driving.eq(0),
                        If(self.dir_in, state.eq(RD_DATA)),
                    ],
                    RD_DATA: [
                        driving.eq(0), tx_byte.eq(0), state.eq(FINISH),
                    ],
                    ABORT: [
                        stp_drive.eq(1), driving.eq(0), tx_byte.eq(0), state.eq(FINISH),
                    ],
                    FINISH: [
                        state.eq(IDLE), driving.eq(0), stp_drive.eq(0), tx_byte.eq(0),
                    ],
                }),
            ),
        ])

        # Only 11 of the 16 encodings of `state` exist. Anything else comes home: the alternative is an
        # unrouted encoding that leaves the master holding the data bus forever, and `verilator -Wall` says
        # the same thing (CASEINCOMPLETE). It is checked ahead of the case, not as a case default, because
        # every waiting state relies on `state` *holding* and a default assignment in the block would clear
        # that.
        sync += If(state > FINISH, state.eq(IDLE), driving.eq(0), stp_drive.eq(0), tx_byte.eq(0))

        # The stall timeout: **ours**, not the spec's (see this file's docstring). It bounds the whole
        # access rather than one wait, which is what makes "waited for bus idle", "waited for nxt" and
        # "waited for dir" all fail the same way instead of hanging -- and it is armed only while an access
        # is live, so an idle master can never time out.
        sync += If((state != IDLE) & ~self.clear, [
            If(tmo < stall_timeout, tmo.eq(tmo + 1)).Else(tmo_hit.eq(1)),
        ]).Else(tmo.eq(0), tmo_hit.eq(0))
        # `state != FINISH` is the fix, and it is load-bearing: without it this **later** `sync +=` outranks
        # the `Case` above it, whose only exit from FINISH is `state.eq(IDLE)`. A timed-out access set
        # `tmo_hit`, `tmo_hit` is cleared only in the `.Else` branch above (i.e. only once `state == IDLE`),
        # and so the timeout re-forced FINISH every cycle and the master could never leave it. `ack` is
        # combinational on FINISH, so the bridge acked every subsequent request with the frozen result
        # bundle and no pad activity at all: one bad access cost the link until a PHY reset. Found on
        # silicon, not in the suite -- `apply_pre()` writes `phy_reset` before *every* vector, so no vector
        # ever followed a timeout without the very thing that clears it (`phy_reset`) in between. Rows
        # `alive_after_stall` / `alive_after_silence` in `tools/ulpi_vectors.tsv` are that boundary; the
        # bench evidence is `bench/2026-10-31_ulpi_wedge_probe.tsv` (state stuck at 10 across 200 ms and
        # across `ulpi_abort`, 2/2, and a PHY reset the only thing that returned `0x24`).
        sync += If(tmo_hit & (state != IDLE) & (state != FINISH) & ~self.clear, [
            timed_out.eq(1), state.eq(FINISH), driving.eq(0), stp_drive.eq(0), tx_byte.eq(0),
        ])

        # The read byte, and the one cycle it can be taken in.
        #
        # The PHY announces with `dir` in cycle D and drives the byte in D+1 (§3.8.3.1). Our `dir_in` is
        # registered, so it is high *during* D+1 -- which is therefore the cycle whose pad value is the
        # answer, and `RD_TURN` is entered in that same cycle (it is one cycle after we see `nxt`). Waiting
        # for `RD_WAIT_DIR` before sampling would read the cycle *after* the PHY let go: an all-zero byte,
        # or worse the byte of the previous access. Hence `read_active` spans both states, and the capture
        # is on the *first* cycle `dir_in` is high.
        #
        # It is latched here rather than sampled at `ack` because "whatever is on the bus when `done` is
        # read" is the stale-read failure this whole spin is built to avoid reporting as a pass (spec
        # testing decision 8).
        rdata_c     = Signal(8, name = "rdata_c")
        read_active = Signal(name = "read_active")
        self.comb += read_active.eq((state == RD_TURN) | (state == RD_WAIT_DIR))
        sync += If(read_active & self.dir_in & ~self.clear, rdata_c.eq(self.tri.i))
        # ...and it is **emptied whenever the master is idle**, so that an access which never reached its
        # read cycle -- a `nxt` stall we timed out, a `dir` abort that exhausted the retry budget -- acks with
        # 00 rather than with the *previous* access's byte. `rdata_c` is a register and `ack_data` carries it
        # on every access, so without this a failed read reports whatever byte was last read: measured
        # 2026-10-31, `stall_rd` acked with `rdata=24`, the ID byte of a row three accesses earlier. That is
        # spec testing decision 8's stale-read failure sitting one layer below the harness rule the
        # testbench already enforces, and the harness cannot see it because a timed-out access really does
        # ack.
        sync += If(state == IDLE, rdata_c.eq(0))

        self.comb += [
            self.ack.eq(state == FINISH),
            self.ack_data.eq(Cat(aborted, timed_out, rdata_c, waited, retries)),
        ]




class UlpiLink(LiteXModule):
    """The register path as one block: :class:`ReqAckCDC` (``cd_sys`` -> ``cd_usb``) + :class:`UlpiMaster`.

    This is what the SoC instantiates and what the simulator runs. No CSRs in here, deliberately: the CSR
    front-end is the integration layer's (ticket 02), and keeping it out means the simulation exercises *the
    logic* rather than a LiteX CSR model.

    Host side (``cd_sys``)
    ----------------------
    ``h_req`` for one cycle with ``h_rw``/``h_addr``/``h_wdata``; ``h_ack`` for one cycle when it finishes;
    ``h_ack_data`` is the master's result bundle (``ack_unpack``). ``h_busy`` is the bridge's and is the only
    ``busy`` the host should ever read: it clears on the ack **or** on ``lost``, so a PHY reset cannot strand
    it (spec testing decision 5).

    What lives on which side, and why
    ---------------------------------
    ``err_no_clock`` is derived **here**, in ``cd_sys``, from the master's ``xfer_active`` level while
    ``clk_ok`` is low. The master cannot derive it: a domain cannot observe the absence of its own clock,
    which is the same reason spin 1's clock meter sits in ``cd_sys`` (spec decision 14). Sampled across the
    *span* of the access, not at the instant of ``go``, because "the clock died during the transaction" is
    the failure that actually happens on this board.

    ``err_reset`` is the bridge's ``err_lost`` under the name the acceptance matrix uses. Both halves of the
    argument matter: ``clear`` cannot cross from ``cd_usb`` to ``cd_sys`` as a pulse (there may be no
    ``cd_usb`` cycles left in which to generate one), but it crosses fine as a level -- which is what
    ``ReqAckCDC.dst_clear`` is.

    The ``ulpi_status`` bit layout is the contract tickets 02 and 04 code against, so it is stated once,
    here, and repeated by the :meth:`status` helper rather than re-derived at each use:

    ::

        bit 0  busy             (cd_sys bridge)
        bit 1  done             (last access completed; sticky until the next h_req)
        bit 2  err_dir_abort    (this access was aborted -- it may still have succeeded)
        bit 3  err_nxt_timeout  (our stall bound expired: ours, never a PHY spec violation)
        bit 4  err_no_clock     (xfer_active while clk_ok=0; derived here, in cd_sys)
        bit 5  err_reset        (far side cleared mid-access; from the bridge)
        bit 6  dir              (live, as the master sees it -- "is the PHY holding the bus?")
        bit 7  oe               (live output enable)
        bits 11:8  state        (the master's FSM; see STATE)

    The counters are separate and saturating, and are *not* in the status word, because a counter's whole
    purpose is to survive being read: ``cnt_access``, ``cnt_dir_abort``, ``cnt_timeout``, ``cnt_reset``,
    ``cnt_no_clock``, plus the last access's ``last_bus_waited``/``last_retry``.
    """
    def __init__(self, pads, sys_clk_freq, stall_timeout = 1 << 16, retry_limit = 3,
                 boot_ms = 1, relock_ms = 10, watchdog_ms = 20):
        # --- host side, cd_sys --------------------------------------------------------------------------
        self.h_req      = Signal()
        self.h_rw       = Signal()
        self.h_addr     = Signal(6)
        self.h_wdata    = Signal(8)
        self.h_ack      = Signal()
        self.h_ack_data = Signal(30)
        self.h_busy     = Signal()
        self.abort_req  = Signal()          # one-cycle, cd_sys
        self.bus_grant  = Signal()
        self.phy_reset  = Signal()          # logical level, 1 = PHY reset asserted (already cd_sys-clean)
        self.clk_ok     = Signal()          # spin 1's clock meter, already in cd_sys

        # --- latched status and counters, cd_sys --------------------------------------------------------
        self.done            = Signal()
        self.err_dir_abort   = Signal()
        self.err_nxt_timeout = Signal()
        self.err_no_clock    = Signal()
        self.err_reset       = Signal()
        self.dir_now         = Signal()
        self.oe_now          = Signal()
        self.state           = Signal(4)
        self.last_bus_waited = Signal(16)
        self.last_retry      = Signal(4)
        self.cnt_access      = Signal(16)
        self.cnt_dir_abort   = Signal(8)
        self.cnt_timeout     = Signal(8)
        self.cnt_reset       = Signal(8)
        self.cnt_no_clock    = Signal(8)

        # --- the clear ----------------------------------------------------------------------------------
        self.ulpi_clear = UlpiClear(sys_clk_freq = sys_clk_freq, phy_reset = self.phy_reset,
                                    boot_ms = boot_ms, relock_ms = relock_ms)

        # --- the crossing -------------------------------------------------------------------------------
        # watchdog_ms = 20, not 10: `docs/NOTES.md` measured one CSR transaction on this serial link at
        # 12-20 ms, so a bridge that has been busy for less than that is not yet news, and a watchdog that
        # fires inside the noise of normal traffic is worse than no watchdog.
        self.bridge = ReqAckCDC(req_width = 15, ack_width = 30,
                                src_domain = "sys", dst_domain = "usb")
        self.bridge.watchdog.reset = int(sys_clk_freq*watchdog_ms/1e3)

        # --- the master ---------------------------------------------------------------------------------
        self.master = UlpiMaster(pads, stall_timeout = stall_timeout, retry_limit = retry_limit,
                                 domain = "usb")

        # --- wiring -------------------------------------------------------------------------------------
        m = ack_unpack(self.bridge.dst_ack_data)
        self.comb += [
            self.bridge.req.eq(self.h_req & ~self.h_busy),
            self.bridge.req_data.eq(req_pack(self.h_rw, self.h_addr, self.h_wdata)),
            self.bridge.dst_clear.eq(self.ulpi_clear.clear),
            self.h_ack.eq(self.bridge.ack),
            self.h_ack_data.eq(self.bridge.ack_data),
            self.h_busy.eq(self.bridge.busy),

            self.master.req.eq(self.bridge.dst_req),
            self.master.rw.eq(self.bridge.dst_data[0]),
            self.master.addr.eq(self.bridge.dst_data[1:7]),
            self.master.wdata.eq(self.bridge.dst_data[7:15]),
            self.bridge.dst_ack.eq(self.master.ack),
            self.bridge.dst_ack_data.eq(self.master.ack_data),

            self.master.phy_reset.eq(self.phy_reset),
            self.master.clear.eq(self.ulpi_clear.clear),
            self.master.bus_grant.eq(self.bus_grant),
        ]

        # `abort` is a cd_sys pulse and the master consumes a cd_usb level. A *pulse* cannot cross into a
        # domain that may have stopped, so what crosses is a level the link holds until the access that saw
        # it ends (ack) or is declared dead (lost). The master acts on it once per access for the same
        # reason: a held level must not abort the access after the one the host meant.
        abort_level = Signal()
        self.sync  += If(self.abort_req, abort_level.eq(1))
        self.sync  += If(self.bridge.ack | self.bridge.err_lost | self.ulpi_clear.clear,
                         abort_level.eq(0))
        abort_usb = Signal()
        self.specials += MultiReg(abort_level, abort_usb, odomain = "usb")
        self.comb  += self.master.abort_req.eq(abort_usb)

        # --- latched status, cd_sys ---------------------------------------------------------------------
        # Sticky **until the next `h_req`**, never self-clearing: the host is ~20 ms away on this link and a
        # bit that clears before anybody can read it is a bit that was never there (spec decision 12).
        # Registered, never combinational: spin 1 measured a combinational status read in the glitch.
        a = ack_unpack(self.bridge.ack_data)
        self.sync += If(self.bridge.ack, [
            self.done.eq(1),
            self.err_dir_abort.eq(a["dir_abort"]),
            self.err_nxt_timeout.eq(a["nxt_timeout"]),
            self.last_bus_waited.eq(a["bus_waited"]),
            self.last_retry.eq(a["retry"][:4]),
            _inc_sat(self.cnt_access),
        ])
        self.sync += If(self.bridge.ack & a["dir_abort"], _inc_sat(self.cnt_dir_abort))
        self.sync += If(self.bridge.ack & a["nxt_timeout"], _inc_sat(self.cnt_timeout))
        # `err_reset` and `err_no_clock` are set on the *events*, not on the ack: the access that lost the
        # clock or the far side never produces an ack at all, and that is precisely the failure being
        # guarded. Latching them off the ack is how a dead link reports success.
        self.sync += If(self.bridge.err_lost, [self.err_reset.eq(1), _inc_sat(self.cnt_reset)])
        # Derived from the BRIDGE's `busy`, not from the master's `xfer_active`, and that is not an
        # oversight: a master with no clock never leaves IDLE, so `xfer_active` stays low in exactly the
        # case this bit exists to report (spec decision 14, "an access was asked for and the PHY clock is not
        # there"). `h_busy` is already in cd_sys, needs no synchroniser, and is true whether the access is
        # running, stalled, or blocked behind `clear` because the PHY is in reset. The master's
        # `xfer_active` stays what it is for -- the analyzer's gate for "is there a transaction to capture".
        self.sync += If(self.h_busy & ~self.clk_ok, [self.err_no_clock.eq(1), _inc_sat(self.cnt_no_clock)])
        # The sticky clear, and it is the LAST assignment to these bits on purpose.
        #
        # ``err_reset`` is set from ``bridge.err_lost``, which is itself a level held until the next
        # ``start`` -- so on the very edge that accepts a new ``h_req`` both the clear and the set are
        # alive, the set reads the old level, and a bit written off a level wins: every access following a
        # lost one reported ``err_reset`` for one access too many. Found 2026-10-31 by the row *after*
        # `reset_mid_noclk_rd`, which read a stale `err_reset` as "this access died" and gave up on one that
        # was running fine. ``err_no_clock`` is the same shape (an access asked for with no clock holds
        # ``h_busy`` high, so its clear and its set are alive together by construction); re-asserting on the
        # next cycle is the right answer there, not a bug.
        #
        # Order also *is* the contract: "sticky until the next `go`" (spec decision 12) means the `go` wins.
        self.sync += If(self.h_req & ~self.h_busy, [
            self.done.eq(0), self.err_dir_abort.eq(0), self.err_nxt_timeout.eq(0),
            self.err_no_clock.eq(0), self.err_reset.eq(0),
        ])
        self.specials += [
            MultiReg(self.master.dir_in, self.dir_now, odomain = "sys"),
            MultiReg(self.master.oe_out, self.oe_now,  odomain = "sys"),
            MultiReg(self.master.state,  self.state,   odomain = "sys"),
        ]

    def status_signals(self):
        """The ``ulpi_status`` word, LSB first -- **the** definition of the layout.

        ``STATUS_FIELDS`` is this list's names and widths, in this order, and ``status_decode`` is its
        inverse. Change one without the other and the gateware and every reader of a captured word
        disagree about which bit is ``busy`` -- silently, at the bench, which is the one place a
        bit-field disagreement is expensive instead of merely wrong.
        """
        return [self.h_busy, self.done, self.err_dir_abort, self.err_nxt_timeout,
                self.err_no_clock, self.err_reset, self.dir_now, self.oe_now, self.state]

    def status(self):
        """The ``ulpi_status`` word, in the bit order documented on the class."""
        return Cat(*self.status_signals())
