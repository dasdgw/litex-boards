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

from litex_boards.extensions.pano_ulpi import UlpiLink, ack_unpack

__all__ = ["UlpiCsr"]


def _edge(prev, cur):
    """One cycle, on the 0->1 transition of a CSR-held level."""
    return cur & ~prev


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
                             "force another access. Ignored while ulpi_status.busy = 1, and counted in "
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
                             "why go waits for idle and counts the wait instead of erroring.")
        self.refused = CSRStatus(description="Writes to ulpi_go that arrived while an access was running "
                             "and were dropped. The bridge is single-outstanding on purpose, so a go "
                             "cannot be queued -- but a dropped go must never be silent, because the "
                             "status word it left behind still describes the PREVIOUS access and reads as "
                             "a fresh success.")

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

        # --- request: the CSR map -> the Link's cd_sys host side ----------------------------------------
        go_edge, abort_edge = Signal(), Signal()
        prev_go, prev_abort = Signal(), Signal()
        self.sync += [
            prev_go.eq(self.go.storage),
            prev_abort.eq(self.abort.storage),
        ]
        self.comb += [
            go_edge.eq(_edge(prev_go, self.go.storage)),
            abort_edge.eq(_edge(prev_abort, self.abort.storage)),
        ]
        # `h_req` is one cycle in cd_sys and the bridge ignores it while busy -- so "while busy" here is
        # the bridge's own guard, restated only so it can be *counted* rather than lost.
        self.comb += self.link.h_req.eq(go_edge & ~self.link.h_busy)
        self.sync += If(go_edge & self.link.h_busy, self.refused.status.eq(1))
        self.comb += [
            self.link.h_rw.eq(self.cmd.storage[0]),
            self.link.h_addr.eq(self.addr.storage[:6]),
            self.link.h_wdata.eq(self.wdata.storage[:8]),
            self.link.abort_req.eq(abort_edge),
        ]

        # --- result: the Link's ack -> the CSRs, sampled ONLY on the ack --------------------------------
        # The rule this honours (spec testing decision 8): the result bundle is copied on the ack and
        # nowhere else. `rdata` is a register written on the ack edge, so a host that reads it while
        # `done` = 0 gets the previous byte and the `done` bit says so -- the pair is the contract, and
        # neither half alone can be mistaken for a fresh answer.
        a = ack_unpack(self.link.h_ack_data)
        self.sync += If(self.link.h_ack, self.rdata.status.eq(a["rdata"]))
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
