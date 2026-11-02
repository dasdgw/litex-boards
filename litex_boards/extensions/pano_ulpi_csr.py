#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2026 Pano Logic G2 reverse-engineering community
# SPDX-License-Identifier: BSD-2-Clause

"""The ``ulpi_*`` CSR door into the ULPI Link (ticket 02).

Why this file exists separately from :mod:`pano_ulpi`
----------------------------------------------------

``pano_ulpi.py`` is the **logic**, and its simulator runs it with plain signals on every commit. If the
CSR front-end lived there, the sim would be exercising a LiteX CSR model as much as our choreography --
which is the wrong thing to spend a simulation on, and the wrong thing to have to delete when Track A
reaches the same two domains by Wishbone. So: ``pano_ulpi`` owns the Link, this file owns the register
map, and the target owns wiring the two to a board.

That split had a cost, found on 2026-11-01: the Link had a simulator and the register map had **none**, so the
rules that decide whether a bench reading is trustworthy -- ``go`` is an edge, a ``go`` against a busy link is
dropped and witnessed, ``rdata`` moves only on the ack -- had only ever been read, never executed. Hence
:class:`UlpiCsrRules`: those rules are a plain-Signal ``Module``, simulated against their generated netlist by
``tools/ulpi_csr_rules_sim.py`` (docs repo), and ``UlpiCsr`` is what is left -- names, descriptions, and
``.storage``/``.status`` plumbing that cannot fail interestingly.

The map, and why it looks like this
-----------------------------------

One CSR per thing the ULPI transfer has, none packed (spec decision 11)::

    ulpi_cmd      w  RW:  0 = RegWrite (wire TX CMD 0x80|addr), 1 = RegRead (0xC0|addr)
    ulpi_addr     w  RW:  the 6-bit immediate register address (RESEARCH.md §7.2)
    ulpi_wdata    w  RW:  the byte to write; ignored on a read
    ulpi_go       w  write 1 to start one access; reads back what you wrote
    ulpi_rdata    r  the byte the last *completed* read returned
    ulpi_status   r  the 12-bit word defined by ``STATUS_FIELDS`` in pano_ulpi.py
    ulpi_abort    w  write 1 to pulse `stp` for one cd_usb cycle and release the bus
    ulpi_refused  r  writes to ulpi_go that arrived while busy and were dropped
    ulpi_cnt_access / _dir_abort / _timeout / _reset / _no_clock    r  saturating counters
    ulpi_bus_waited / ulpi_retry   r  the last access's wait and retry counts

Packing these into one 32-bit word would have halved ~60 ms per access to ~30 ms. It was rejected because
the register map is the instrument's face: ``litex_cli --filter=ulpi_addr`` must stay a lookup, and a
bit-field hunt is what a tired engineer at a bench gets wrong. An *alias* can be added later without
renaming anything; a rename cannot be.

``ulpi_cmd`` carries the direction, not the wire byte, and that is deliberate
----------------------------------------------------------------------------

ULPI's first byte is ``0x80|addr`` or ``0xC0|addr``. Writing that byte literally into a CSR looks like it
"reads like the wire" until someone writes ``0x40`` -- a *packet* command, which is not implemented here
and which the PHY would answer with USB data rather than a register value -- or ``0x00``, the idle level.
That is not hypothetical: ``PLAN.md`` M2.3 told the implementer to use "burst-set cmd ``0x3``/burst-get
cmd ``0x4``", which are Linux driver internals and not ULPI at all, and a spin built from them would have
emitted a packet command where a register read belonged (``RESEARCH.md`` §7.1). So the CSR holds one
direction bit and the Link composes the byte. **A packet command is now not expressible through the
register map at all**, which is a stronger guarantee than a runtime check on a value nobody should have
written.

Two disciplines that come from measurements, not taste
-----------------------------------------------------

* **``ulpi_go`` triggers on a rising edge of the stored bit, not on ``wr_stb``.** Over this remote channel
  a host that times out and retries a write must not start a second transaction, and ``wr_stb`` fires on
  every write including a rewrite of the same value. Writing 1 twice in a row is therefore one access;
  the harness writes 0 then 1 to force a second one. And unlike a self-clearing register, the bit reads
  back what was written, which is what the "every write is read back" rule (the CSR bridge does not order
  a read after a write -- ``docs/NOTES.md``) needs in order to notice a lost write at all.
* **A ``go`` that arrives while ``busy`` is dropped, and counted** (``ulpi_refused``). It cannot be
  queued: the bridge is single-outstanding by design (``pano_cdc.py``), and a queue is the FIFO failure
  mode spec decision 10 rejects. Dropping it *silently* would be the worst outcome, because the status
  word would then still describe the *previous* access and read as a fresh success.
"""

from migen import *

from litex.gen import LiteXModule
from litex.soc.interconnect.csr import CSRStatus, CSRStorage

# `_inc_sat` lives in pano_ulpi.py because it is what the Link's own error counters use; the wake
# refuse counter is the fifth user and the sixth would be a copy of a saturating adder in a file that
# cannot be tested against the other five. Import it, do not re-derive it.
from litex_boards.extensions.pano_ulpi import UlpiLink, ack_unpack, _inc_sat

__all__ = ["UlpiCsr", "UlpiCsrRules", "ACK_WIDTH"]

# The width of the Link's ack payload, in one place. `ack_unpack` slices `sig[26:30]`, so a rules module
# that declared a 16-bit `h_ack_data` would still *work* for `rdata` and silently truncate `bus_waited`/
# `retry`; the sim top and `tools/test_ulpi_csr_rules.py` both compare this number against the real
# `link.h_ack_data`, so the disagreement is a red check instead of a bench mystery.
ACK_WIDTH = 30


def _edge(prev, cur):
    """One cycle, on the 0->1 transition of a CSR-held level."""
    return cur & ~prev


class UlpiCsrRules(Module):
    """Every *rule* of the ``ulpi_*`` map, with no CSR object anywhere in it.

    Why a module and not ten lines inside :class:`UlpiCsr`: those ten lines are the whole interface
    contract of the instrument -- ``go`` is an edge and not a level, a ``go`` against a busy link is dropped
    and witnessed, ``rdata`` moves only on the ack -- and until now not one of them had ever been executed.
    `tools/ulpi_rtl_sim.py` stops at ``UlpiLink`` on purpose (a Link sim that also needed a CSR model would
    be testing LiteX's `CSRStorage` instead of our choreography), so the rules were proven by reading them.
    That is how this repo has already been wrong twice: a `MultiReg(i, Signal(2))` that was a two-bit net
    and not a synchroniser, and a `dst_req` whose only release was `dst_ack` (`docs/STATUS.md` 2026-10-31).
    Both were readable and both were wrong.

    The seam is plain Signals, which is what makes it simulatable: the inputs are what a CSR *holds*
    (`go`/`abort`/`cmd`/`addr`/`wdata`) plus the Link's own ``cd_sys`` host side (`h_busy`/`h_ack`/
    `h_ack_data`), and the outputs are what the Link receives and the host reads back. `UlpiCsr` therefore
    keeps only plumbing, and the drift guard (`tools/test_ulpi_csr_rules.py`) is what keeps the fake honest:
    it builds a real `UlpiLink` and refuses a build in which a name the rules are wired to has disappeared
    or changed width.

    Three behaviours it owns, each with a bench reason:

    * **edge, not level.** Over a ~20 ms/transaction remote channel a host that timed out and retried its
      write must not start a second transaction, so ``go`` triggers on 0->1 of the *stored bit*. Rewriting
      1 is not an access; 0 then 1 is (the harness's documented way to force one).
    * **a dropped ``go`` is counted.** `h_req` carries `~h_busy` twice over (here, and again inside
      `UlpiLink`, whose bridge is single-outstanding by decision 10). Repeating the guard here is not
      defensive coding, it is the only place the event can be *witnessed* rather than lost -- and an
      uncounted drop leaves the status word describing the previous access, which reads as a fresh success.
      `refused` is a **sticky bit, not a counter**: nothing clears it until the bitstream reloads, so its
      meaning is "at least one ``go`` was dropped since boot", which is exactly what acceptance row A0 asks
      and what per-phase accounting must come from `ulpi_cnt_*` instead.
    * **``rdata`` on the ack, and nowhere else** (spec testing decision 8). It is a register, not a wire, so
      a host reading it while ``done`` = 0 gets the previous byte and the ``done`` bit says so. A combinational
      `rdata` would make ``done`` decorative.
    """
    def __init__(self):
        # --- in: what the host wrote, as levels held by the CSR block ----------------------------------
        self.go    = Signal()
        # `abort_in`, not `abort`: a Verilated C++ model cannot have a port called `abort` (libc), which is
        # the same reason `UlpiMaster` names its input `abort_req`. The CSR behind it stays `ulpi_abort`.
        self.abort_in = Signal()
        self.cmd   = Signal()
        self.addr  = Signal(6)
        self.wdata = Signal(8)
        # `wake_req`: what `ulpi_wake.storage` holds. `wake_applied`: what the pad is allowed to see. They
        # are different signals because they must be -- see the refuse rule below, and spec decision C.
        self.wake_req = Signal()
        self.stp_drive_req = Signal()
        # --- in: the Link's cd_sys host side -------------------------------------------------------------
        self.h_busy     = Signal()
        self.h_ack      = Signal()
        self.h_ack_data = Signal(ACK_WIDTH)
        # --- out: what the Link receives ------------------------------------------------------------------
        self.h_req     = Signal()
        self.h_rw      = Signal()
        self.h_addr    = Signal(6)
        self.h_wdata   = Signal(8)
        self.abort_req = Signal()
        # --- out: what the host reads back -----------------------------------------------------------------
        self.rdata   = Signal(8)
        self.refused = Signal()
        self.wake_applied      = Signal()
        # **reset = 1, and it is the CSR's reset value, not a default.** `stp_drive_applied` is what reaches
        # the pad's output enable; 0 there TRI-STATES A5. Every spin before this ticket drove `stp` low as the
        # ULPI idle level requires (ULPI 1.1 §2.3.2), so a rules module that comes out of reset with 0 would
        # hand the pad to whatever is on the board the moment the FPGA configures, on a build whose CSR reads
        # "driving". It must also agree with `UlpiMaster.stp_drive_en`'s own reset, or the two halves of one
        # promise disagree during the boot clear.
        self.stp_drive_applied = Signal(reset = 1)
        self.pad_refused       = Signal(8)

        # ---- `wake`, and why the stored level is not the applied level ------------------------------------
        # `ulpi_wake` asserts `stp` asynchronously into a PHY that may be holding a transaction open. ULPI
        # §3.8.4.2 makes that a *packet-ending* event ("the PHY decodes stp as the end of a transmit"), so an
        # applied-in-the-middle `wake` would end a transfer the host never asked to end -- the same class of
        # fault as bug 4, where an `abort` written on an idle link aborted the NEXT access. Decision C: apply
        # the level only while the bridge reports nothing in flight, and count the refusal.
        #
        # The predicate is `h_busy` and nothing else, for the reason decision C states: `bridge.busy` is
        # `cd_sys`, so it is a real answer in the state the wake exists for. Anything from `cd_usb` (`state`,
        # `oe`, `xfer_active`) is FROZEN there, not false, and a refusal predicate built on a frozen signal is
        # a policy that quietly stops working when it is needed. `phy_reset` is deliberately NOT part of the
        # predicate: §6.1.9.3 asks for `stp` asserted while a hardware reset is applied, and ADR-0006's rule
        # for a rule about the far side is detect-and-report, not gate (that is ticket 01's open question 1,
        # answered here: a `wake` is legal during a PHY reset).
        #
        # Both directions of the level go through the same gate. Dropping `wake` is the §3.9.4 *false-resume*
        # case -- release `stp` before the PHY has had its clock and it goes back to sleep -- and a drop that
        # arrived mid-access is exactly as unwelcome as an assert that did; the harness drops it when the link
        # is idle, or not at all.
        # Both are *edges of the stored bit*, not levels, and that is a decision rather than a detail. Two
        # wrong shapes were available:
        #   * `If(~busy, applied := req)` follows the level whenever the link goes quiet, so a `wake` written
        #     during an access is applied ~ms later to a board whose host has moved on -- `stp` asserting by
        #     itself, from a write nobody now remembers making. Spec decision C says DROPPED, and a dropped
        #     write must stay dropped.
        #   * counting "while busy and different" counts every `cd_sys` cycle of a stalled access, so the
        #     counter saturates in 5 us and the delta the harness reads means nothing.
        # So: a change of the stored bit is one attempted write, exactly like `go`'s edge, and it is either
        # applied (link idle) or dropped-and-counted (link busy). A refused write is not queued and not
        # retried: the operator writes 0 then 1, which is already this map's documented way to force an event
        # (`ulpi_go`), and `ulpi_stp_pad` is the register that says what is really on the pin when the stored
        # bit and the pad disagree.
        prev_wake, prev_stp_drive = Signal(), Signal()
        self.sync += [prev_wake.eq(self.wake_req), prev_stp_drive.eq(self.stp_drive_req)]
        wake_change, stp_change = Signal(), Signal()
        self.comb += [wake_change.eq(self.wake_req ^ prev_wake),
                      stp_change.eq(self.stp_drive_req ^ prev_stp_drive)]
        self.sync += If(wake_change & ~self.h_busy, self.wake_applied.eq(self.wake_req))
        self.sync += If(stp_change & ~self.h_busy,
                        self.stp_drive_applied.eq(self.stp_drive_req))
        self.sync += If((wake_change | stp_change) & self.h_busy, _inc_sat(self.pad_refused))
        # And the converse door: a `go` while `wake` is applied does not start an access. §6.1.9.2's own
        # figure has the Link driving *idle* for the whole resume window, because the PHY is running its
        # start-up sequence and cannot answer a CMD byte; a Link that drove a TX CMD into that window would
        # get an abort or a corrupt byte and would report it as a link fault. Refused here (ticket 01 open
        # question 2, answered: refuse) and witnessed in `refused`, whose description names both causes -- the
        # host that wrote `wake` knows which one it caused, and one bit with two documented meanings is a
        # better object than a second bit nobody reads.

        go_edge, abort_edge = Signal(), Signal()
        prev_go, prev_abort = Signal(), Signal()
        self.sync += [
            prev_go.eq(self.go),
            prev_abort.eq(self.abort_in),
        ]
        self.comb += [
            go_edge.eq(_edge(prev_go, self.go)),
            abort_edge.eq(_edge(prev_abort, self.abort_in)),
        ]
        # `h_req` is one cycle in cd_sys and the bridge ignores it while busy -- so "while busy" here is
        # the bridge's own guard, restated only so it can be *counted* rather than lost.
        self.comb += self.h_req.eq(go_edge & ~self.h_busy & ~self.wake_applied)
        self.sync += If(go_edge & (self.h_busy | self.wake_applied), self.refused.eq(1))
        self.comb += [
            self.h_rw.eq(self.cmd),
            self.h_addr.eq(self.addr),
            self.h_wdata.eq(self.wdata),
            self.abort_req.eq(abort_edge),
        ]

        # The result bundle is copied on the ack and nowhere else. `a` is the field view of the Link's own
        # `ack_unpack`, so a field offset cannot disagree with the Link that produced the payload.
        a = ack_unpack(self.h_ack_data)
        self.sync += If(self.h_ack, self.rdata.eq(a["rdata"]))


class UlpiCsr(LiteXModule):
    """The ``ulpi_*`` register map, in ``cd_sys``, in front of :class:`~pano_ulpi.UlpiLink`.

    Instantiate it as ``self.ulpi`` and the CSRs land under exactly the names spin 1 reserved
    (LiteX prefixes a CSR with its module's final name). Everything host-visible is here; the Link stays
    CSR-free so the simulator never needs a CSR model.

    The three inputs are the board's opinion, not the Link's:

    ``phy_reset``  the *logical* PHY-reset level (1 = in reset) as the ULPI Probe drives it, polarity
                   already applied in software -- ``cd_sys``-clean by construction, so it is safe both for
                   ``UlpiClear``'s synchronous tail and for the master's combinational ``oe`` term.
    ``clk_ok``     spin 1's clock meter, registered in ``cd_sys``. ``err_no_clock`` is derived from it in
                   ``cd_sys`` because a domain cannot observe the absence of its own clock (decision 14).
    ``bus_grant``  permission to drive ``data``. See the comment at the tie in the target: it is a real
                   interlock, tied to 1 only because this spin has exactly one bus master.
    """
    def __init__(self, pads, sys_clk_freq, stall_timeout = 1 << 16, retry_limit = 3,
                 boot_ms = 1, relock_ms = 10, watchdog_ms = 20):
        # --- the Link (bridge + master + clear), all of it inside this module ---------------------------
        self.link = UlpiLink(pads, sys_clk_freq = sys_clk_freq, stall_timeout = stall_timeout,
                             retry_limit = retry_limit, boot_ms = boot_ms, relock_ms = relock_ms,
                             watchdog_ms = watchdog_ms)
        self.phy_reset = Signal()
        self.clk_ok    = Signal()
        self.bus_grant = Signal()
        self.comb += [
            self.link.phy_reset.eq(self.phy_reset),
            self.link.clk_ok.eq(self.clk_ok),
            self.link.bus_grant.eq(self.bus_grant),
        ]

        # --- the five names spin 1 reserved ------------------------------------------------------------
        self.cmd = CSRStorage(description="The direction, not the wire byte: 0 = RegWrite (the Link puts "
                             "TX CMD 0x80|addr on the bus), 1 = RegRead (0xC0|addr). ULPI 1.1 Table 6. "
                             "The command byte is composed by the Link rather than accepted from software "
                             "because 0x40 is a *packet* command and 0x00 is the idle level, and a map "
                             "that can express neither cannot send one by accident (RESEARCH.md §7.1 -- "
                             "our own PLAN.md used to ask for exactly that).")
        self.addr = CSRStorage(6, description="Immediate 6-bit register address. The ID is 0x00..0x03; "
                             "0x15 is Debug (link state), 0x16 Scratch is the safe write target. Do not "
                             "touch 0x30..0x3F or the extended forms: the USB3300 tells the Link not to "
                             "read or write registers outside its table.")
        self.wdata = CSRStorage(8, description="Byte to write. Ignored when ulpi_cmd = 1.")
        self.go = CSRStorage(description="Write 1 to start ONE access. Reads back what was written, so a "
                             "lost write is visible: the CSR bridge does not order a read after a write. "
                             "The trigger is the 0->1 transition, NOT any write -- a host retrying a write "
                             "it could not see must not start a second transaction. Write 0, then 1, to "
                             "force another access. Ignored while ulpi_status.busy = 1, and witnessed by "
                             "ulpi_refused.")
        self.rdata = CSRStatus(8, description="The byte of the last access that COMPLETED a read. Only "
                             "meaningful together with ulpi_status.done = 1 for this access: read it while "
                             "done = 0 and you are reading the previous access's answer, which is the "
                             "failure this spin exists to make impossible to report as a pass.")

        # --- status, abort, and the counters -----------------------------------------------------------
        self.status = CSRStatus(12, description="bit0 busy, 1 done, 2 err_dir_abort, 3 err_nxt_timeout, "
                             "4 err_no_clock, 5 err_reset, 6 dir (live), 7 oe (live), 11:8 state. "
                             "The sticky bits hold until the NEXT go, never self-clearing: the host is "
                             "~20 ms away on this link. Layout source: STATUS_FIELDS in pano_ulpi.py.")
        self.abort = CSRStorage(description="Write 1 to pulse stp for one cd_usb cycle: the Link's way of "
                             "releasing a PHY that is holding dir (ULPI 1.1 §3.8.4.2), so 'the bus is "
                             "stuck' does not need a power cycle. Caveat from the same clause: unless the "
                             "Link drives a TX CMD immediately after the turnaround the PHY may simply "
                             "re-assert dir -- this releases the bus and promises nothing more, which is "
                             "why go waits for idle and counts the wait instead of erroring. "
                             "IGNORED unless an access is in flight (ulpi_status.busy = 1), and silently: a "
                             "Link abort ends a transaction and with none running there is nothing to "
                             "release. That is a measured choice, not an oversight -- before it, an abort "
                             "written on an idle link latched and aborted the NEXT access, so the innocent "
                             "read after a stray ulpi_abort came back 00h with dir_abort+nxt_timeout "
                             "(bug 4, bench/2026-11-01_ulpi_abort_idle/, proven red-before-green by "
                             "extensions/usb/sim/ulpi_abort_idle_tb.v and kept red-able by tools/ulpi_mutate.py "
                             "'s abort_idle mutation.")

        self.refused = CSRStatus(description="STICKY BIT, not a counter: once one ulpi_go has been DROPPED "
                             "this reads 1 and stays 1 until the bitstream reloads. The bridge is "
                             "single-outstanding on purpose, so a go cannot be queued -- but a dropped go "
                             "must never be silent, because the status word it left behind still describes "
                             "the PREVIOUS access and reads as a fresh success.\n"
                             "TWO causes, one bit, since 2026-11-02: a go against a busy link, and a go "
                             "while ulpi_wake = 1 (the second is §6.1.9.2's rule that the Link drives IDLE "
                             "during a resume, not a command). It is not split, because the question it "
                             "answers is 'has this map ever dropped a request since boot' (acceptance row "
                             "A0) and not 'which', and the harness that wrote ulpi_wake knows which it "
                             "caused. A per-write count is ulpi_cnt_access and ulpi_cnt_pad_refused.")

        self.cnt_access    = CSRStatus(16, description="Completed accesses (ack), saturating.")
        self.cnt_dir_abort = CSRStatus(8,  description="Accesses that saw a dir abort. A successful "
                             "retry still counts here: 'passed after three aborts' and 'passed cleanly' "
                             "are different facts (ULPI 1.1 §3.8.3.1 makes the retry mandatory).")
        self.cnt_timeout   = CSRStatus(8,  description="Accesses ended by OUR stall bound (2^16 cd_usb "
                             "cycles). Ours, not the PHY's: nxt throttling is unbounded in ULPI 1.1, so "
                             "this is never evidence of a PHY spec violation.")
        self.cnt_reset     = CSRStatus(8,  description="Accesses lost because the far side was cleared "
                             "(PHY reset) -- the bridge's err_lost, under the name the acceptance matrix "
                             "uses.")
        self.cnt_no_clock  = CSRStatus(8,  description="Accesses that were running while clk_ok was low: "
                             "'the PHY stopped clocking us', which needs the opposite fix from 'our "
                             "choreography is wrong'.")
        self.bus_waited = CSRStatus(16, description="cd_usb cycles the LAST access spent waiting for bus "
                             "idle before it drove. 'Passed after 3' and 'passed after 40000' are the same "
                             "verdict today and a different diagnosis next month.")
        self.retry = CSRStatus(4, description="dir aborts the LAST access retried (0 = clean).")

        # --- the wake path and the `stp` pad readings, APPENDED LAST ---------------------------------------
        # Appended, never inserted: ADR-0007, and `tools/test_ulpi_csr_map.py` refuses a build where a
        # spin-1 address moved. These four are the whole `ulpi_*` addition of the wake ticket and the four
        # addresses ticket 02 hard-codes; the order here IS the map.
        self.wake = CSRStorage(description="1 = hold the ULPI `stp` pad high, asynchronously, with no "
                             "cd_usb edge required. This is the ONLY way out of a PHY in Low Power Mode: "
                             "USB3300 §6.1.9 says clearing `SuspendM` (bit 6 of Function Control, 0x04) "
                             "stops the PHY's clock and asserts dir, and §6.1.9.2 / ULPI 1.1 §3.9.3 say the "
                             "exit is the Link asserting stp -- which every other writer of stp in this "
                             "design cannot do, because `stp_drive` lives in the clock that just stopped.\n"
                             "A LEVEL, not a pulse: T_START is 2.25-3.5 ms (RESEARCH.md §7.6) and one "
                             "transaction on this link costs ~20 ms, so a host-side pulse-and-poll is a race "
                             "that is lost by construction. Assert it, walk away, come back.\n"
                             "REFUSED while ulpi_status.busy = 1 (and counted by ulpi_cnt_pad_refused): "
                             "asserting stp in the middle of someone's transaction is §3.8.4.2's end-of-"
                             "packet event. A refused write is DROPPED, not queued -- write 0 then 1 to try "
                             "again, the same way ulpi_go works.\n"
                             "RELEASE IT ONLY WHEN THE LINK IS IDLE AND THE CLOCK IS BACK (§3.9.4). "
                             "Releasing stp before the PHY drops dir returns it to Low Power Mode, so a "
                             "timeout is not a reason to drop this bit; read ulpi_stp_pad/dir and decide."
                             "\nAlso refused: a ulpi_go written while this reads 1 (witnessed in "
                             "ulpi_refused). §6.1.9.2's figure has the Link driving idle for the whole resume "
                             "window, and a CMD byte driven into a PHY that is still starting up is a fault "
                             "report, not a read.")
        self.stp_drive = CSRStorage(reset = 1, description="1 (the default, and every spin before this one) "
                             "= we drive the stp pad, which is what ULPI requires of a Link: the idle level "
                             "is stp low with data at 00h. Write 0 to TRI-STATE A5 so that ulpi_stp_pad "
                             "becomes a reading of the BOARD instead of a reading of our own output driver -- "
                             "that is TODO (e), whether anything pulls the wake line up.\n"
                             "This changes what the PHY sees on a running interface: a floating stp is not "
                             "the ULPI idle level. Read it, then write 1 back. It is refused while "
                             "ulpi_status.busy = 1 for that reason (ulpi_cnt_pad_refused), and `wake` "
                             "outranks it: asserting wake drives the pad whatever this says, because a wake "
                             "that a stray 0 could switch off is the same bug as a wake that `clear` could "
                             "switch off.")
        self.stp_pad = CSRStatus(2, description="bits [0] the level on the stp pad (A5), as its own input "
                             "receiver sees it; [1] = WE are driving it. Read them together, and read [1] "
                             "first: with [1]=1 this says what our flop drove and NOTHING about the board "
                             "(docs/NOTES.md, learned the expensive way on W12 -- a pad's level read-back "
                             "while its output driver is enabled is not a measurement of the pin). With "
                             "[1]=0 it is a reading: 0 with no driver = no pull-up, TODO (e) answered.\n"
                             "While [1]=0 the pad is floating and the PHY may see anything on it; see "
                             "ulpi_stp_drive.")
        self.cnt_pad_refused = CSRStatus(8, description="Writes to ulpi_wake or ulpi_stp_drive that arrived "
                             "while an access was running and were DROPPED (saturating). One counter for "
                             "both, because they are one rule -- 'a write that changes what the PHY sees on "
                             "stp, while the link owns a transaction'; the harness knows which write it "
                             "issued, and the alternative was a register nobody reads. Not the same witness "
                             "as ulpi_refused, which is sticky and about go: this one answers 'how many, and "
                             "did the one I just tried land', which is what a bench operator asks while "
                             "holding a dark board.\nIf this moves and the clock did not come back, the "
                             "write never reached the pad: retry with the link idle (write 0, then 1).")

        # --- request: the CSR map -> the Link's cd_sys host side ----------------------------------------
        # All of the behaviour lives in `UlpiCsrRules`, whose ports are plain Signals so that
        # `tools/ulpi_csr_rules_sim.py` can drive it without a CSR model. What is left here is plumbing:
        # `.storage` in, `.status` out. The rules module is the DUT the simulation runs, so this is the
        # code the vectors have actually exercised -- not a copy of it.
        self.rules = UlpiCsrRules()
        self.comb += [
            self.rules.go.eq(self.go.storage),
            self.rules.abort_in.eq(self.abort.storage),
            self.rules.cmd.eq(self.cmd.storage[0]),
            self.rules.addr.eq(self.addr.storage[:6]),
            self.rules.wdata.eq(self.wdata.storage[:8]),
            self.rules.wake_req.eq(self.wake.storage[0]),
            self.rules.stp_drive_req.eq(self.stp_drive.storage[0]),
            self.link.wake.eq(self.rules.wake_applied),
            self.link.stp_drive.eq(self.rules.stp_drive_applied),
            self.stp_pad.status[0].eq(self.link.stp_pad),
            self.stp_pad.status[1].eq(self.link.stp_driving),
            self.cnt_pad_refused.status.eq(self.rules.pad_refused),
            self.rules.h_busy.eq(self.link.h_busy),
            self.rules.h_ack.eq(self.link.h_ack),
            self.rules.h_ack_data.eq(self.link.h_ack_data),
            self.link.h_req.eq(self.rules.h_req),
            self.link.h_rw.eq(self.rules.h_rw),
            self.link.h_addr.eq(self.rules.h_addr),
            self.link.h_wdata.eq(self.rules.h_wdata),
            self.link.abort_req.eq(self.rules.abort_req),
            # Registers in the rules module, wired out combinationally: a second register here would put a
            # cycle between the ack and what the host reads, which is the one thing `rdata` must not do.
            self.rdata.status.eq(self.rules.rdata),
            self.refused.status.eq(self.rules.refused),
        ]

        # --- result: the Link's ack -> the CSRs, mirrored -----------------------------------------------
        # (`rdata` itself is the rules module's register; see above.)
        self.comb += [
            self.status.status.eq(self.link.status()),
            self.cnt_access.status.eq(self.link.cnt_access),
            self.cnt_dir_abort.status.eq(self.link.cnt_dir_abort),
            self.cnt_timeout.status.eq(self.link.cnt_timeout),
            self.cnt_reset.status.eq(self.link.cnt_reset),
            self.cnt_no_clock.status.eq(self.link.cnt_no_clock),
            self.bus_waited.status.eq(self.link.last_bus_waited),
            self.retry.status.eq(self.link.last_retry),
        ]
