#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2026 Pano Logic G2 reverse-engineering community
# SPDX-License-Identifier: BSD-2-Clause

"""Clock-domain-crossing primitives for the Pano's ULPI Link, reusable by Track A.

Why a module of its own instead of three calls to ``MultiReg``: the one crossing that has ever silently
broken a Pano build is a **request that outlives the far side**. ``cd_usb`` is the PHY's own clock
(``C12``), so it stops whenever the PHY is in reset -- and a handshake whose far side can be erased
mid-flight leaves a ``busy`` bit that never clears, which over a remote channel that costs ~20 ms per CSR
transaction is indistinguishable from a dead link (measured 2026-10-30, ``docs/NOTES.md``). So the reset
path is part of this primitive's interface (``dst_clear``) rather than something every user re-invents.

Two rules, both load-bearing here:

* **Levels cross; edges are made locally.** A ``MultiReg`` on a level, and each domain creates its own
  rising-edge detect. An edge generated in the far domain can be missed entirely when that clock stops --
  which on this board is a normal event, not a corner case.
* **A cleared far side is a reportable outcome, not a hang.** See ``ReqAckCDC.err_lost``.

Rejected: crossing with a FIFO (what ``UlpiCtrl`` does -- tomverbeure, Apache-2.0, formally checked for
this board). A FIFO can hold a command across a PHY reset, and that command would then be issued to a PHY
that has forgotten the transaction that earned it. Rejected in spec decision 10.

Track A reuse is the reason this is not inlined: ``core_usb_host``'s AXI4-Lite ``cfg_*`` window lives on
the PHY's 60 MHz and needs the same two domains crossed, single-outstanding is enough there too, and the
plan is a ``Wishbone2AXILiteCDC`` built on :class:`ReqAckCDC`. Keep this protocol ULPI-free.
"""

from migen import *
from migen.genlib.cdc import MultiReg

__all__ = ["CdcLevel", "ReqAckCDC"]


class CdcLevel(Module):
    """A level from one domain to another (2-FF).

    Only signals that are *tested* as levels may cross this way: a bit that pulses in the source domain can
    be shorter than two destination cycles and is missed **silently**. ``cd_usb`` pulses are especially
    easy to write, because that clock is a normal-looking 60 MHz right up until the PHY is reset.
    """
    def __init__(self, i, o, dst_domain="sys"):
        self.specials += MultiReg(i, o, odomain=dst_domain)


class ReqAckCDC(Module):
    """Single-outstanding 2-phase request/ack handshake: src -> dst -> src.

    Protocol. Levels cross; every edge below is generated inside the domain that uses it.

    1. **src**, idle: ``req`` for one cycle with the payload valid latches the payload and raises
       ``req_level``; ``busy`` is set on the *same* edge, so there is no window in which a transaction is
       in flight and ``busy`` reads 0.
    2. **dst**: ``req_level`` arrives through a 2-FF, is edge-detected there, and ``dst_req`` goes high for
       as long as the request is outstanding. The payload is *copied* into ``dst_data`` on that edge, so
       the destination owns a stable value instead of watching the source's registers.
    3. **dst**, on completion: ``dst_ack_data`` is latched, ``ack_level`` rises, ``dst_req`` falls.
    4. **src**: ``ack_level`` arrives through a 2-FF and is edge-detected there -> one-cycle ``ack`` with
       ``ack_data``; ``req_level`` falls.
    5. **dst**: ``req_level`` low is the only thing that releases ``ack_level``. That closes the loop and
       is what makes the handshake 2-phase rather than a pulse that can be lost.

    ``busy`` clears on the ack **or** on ``lost``, and ``lost`` is the reason this class exists:

    * ``dst_clear`` -- a level, asserted for as long as the destination's state is being erased (the Link
      passes its ``phy_reset | boot_counter`` clear, which by construction stays asserted until the
      destination's clock has come back). 2-FF'd to src: ``err_lost``, ``busy`` clear.
    * ``active`` -- the destination asserting "I am servicing something". If it stops saying so while we
      are busy and no ack came, the far side was erased in a way ``dst_clear`` never showed us. Same
      verdict, same bit, and it costs one more 2-FF.
    * ``watchdog`` -- a bound in **source** cycles, ours rather than the far side's, so it also catches a
      mistake in *this file*. The Link arms it at >10 ms.

    Sticky, and counted: ``err_lost`` holds until the next ``req`` (a status bit that a later transaction
    clears is a bit nobody 20 ms away can see), ``xfer`` counts completions and ``lost_count`` counts
    losses, both saturating.

    Payloads are opaque on purpose: ``req_width``/``ack_width``. The Link puts ``{rw, addr[6], wdata[8]}``
    in and ``{rdata[8], err_dir_abort, err_nxt_timeout, err_reset}`` back.
    """
    def __init__(self, req_width=15, ack_width=11, src_domain="sys", dst_domain="usb"):
        # --- source side (host / CSR side) ------------------------------------------------------------
        self.req         = Signal()              # 1 cycle, payload valid that cycle, ignored while busy
        self.req_data    = Signal(req_width)
        self.ack         = Signal()              # 1 cycle in src_domain
        self.ack_data    = Signal(ack_width)
        self.busy        = Signal()
        self.err_lost    = Signal()              # sticky until the next req
        self.xfer        = Signal(16)
        self.lost_count  = Signal(8)
        self.dst_clear   = Signal()              # level: the far side is being cleared
        self.watchdog    = Signal(32)            # source cycles; 0 disables it

        # --- destination side -------------------------------------------------------------------------
        self.dst_req      = Signal()             # level: request outstanding, dst_data valid
        self.dst_data     = Signal(req_width)
        self.dst_ack      = Signal()             # 1 cycle in dst_domain
        self.dst_ack_data = Signal(ack_width)
        self.dst_active   = Signal()             # level: exported to src as the `active` witness
        self.dst_dropped  = Signal()             # 1 cycle in dst_domain: a request was lost

        src = self.sync if src_domain == "sync" else getattr(self.sync, src_domain)
        dst = self.sync if dst_domain == "sync" else getattr(self.sync, dst_domain)

        # --- src: take the request ----------------------------------------------------------------------
        # `~req_level` in the guard is what makes this single-outstanding: a second go while one is in
        # flight is dropped rather than queued. Queuing is the FIFO failure mode this design rejects.
        req_level   = Signal()
        req_payload = Signal(req_width)
        src += self.ack.eq(0)          # a pulse needs its default: a register holds its value
        # `start` is combinational on purpose: as a registered signal it would fire a cycle late, and the
        # `~req_level` guard would then be comparing against a payload that has already been taken.
        start = Signal()
        self.comb += start.eq(self.req & ~req_level & ~self.busy)
        src += If(start, [
            req_level.eq(1),
            req_payload.eq(self.req_data),
            self.busy.eq(1),
            self.err_lost.eq(0),
        ])

        # --- src -> dst ---------------------------------------------------------------------------------
        req_x = Signal(name = "req_x")   # 1 bit: MultiReg's stage count is its `n` argument, not the width
        self.specials += MultiReg(req_level, req_x, odomain=dst_domain)
        req_high = req_x
        req_was  = Signal()
        dst += req_was.eq(req_high)
        dst += If(req_high & ~req_was & ~self.dst_req, [      # rising edge, detected where it is used
            self.dst_req.eq(1),
            self.dst_data.eq(req_payload),
        ])
        dst += self.dst_dropped.eq(0)
        dst += If(self.dst_ack, self.dst_req.eq(0))
        # Observation only, for the analyzer and for `dst_dropped`. Nothing may derive `lost` from
        # this dropping: see the comment at `lost`.
        self.comb += self.dst_active.eq(self.dst_req)

        # --- dst -> src ---------------------------------------------------------------------------------
        ack_level = Signal()
        ack_hold  = Signal(ack_width)
        dst += If(self.dst_ack, [
            ack_level.eq(1),
            ack_hold.eq(self.dst_ack_data),
        ])
        # Step 5: only the source releasing the request closes the loop.
        dst += If(~req_high, ack_level.eq(0))

        ack_x = Signal(name = "ack_x")
        self.specials += MultiReg(ack_level, ack_x, odomain=src_domain)
        ack_high = ack_x
        ack_was  = Signal()
        src += ack_was.eq(ack_high)
        ack_edge = Signal()
        src += ack_edge.eq(ack_high & ~ack_was)
        src += If(ack_edge, [
            self.ack.eq(1),
            self.ack_data.eq(ack_hold),
            req_level.eq(0),
            self.busy.eq(0),
            self.xfer.eq(Mux(self.xfer != 0xffff, self.xfer + 1, self.xfer)),
        ])

        # --- lost: a destination that cannot answer ------------------------------------------------------
        clear_x = Signal(name = "clear_x")
        active_x = Signal(name = "active_x")
        self.specials += [MultiReg(self.dst_clear, clear_x, odomain=src_domain),
                          MultiReg(self.dst_active, active_x, odomain=src_domain)]
        # `seen_clear` exists because a PHY reset can arrive and leave between two of our slow cycles, and a
        # level that comes and goes is exactly what a synchroniser misses. It is a *witness for the current
        # access*, so it is cleared when an access starts and when one is declared lost: left sticky, it
        # marks every later access lost the moment the first PHY reset happens -- which is what it did, on
        # the vector immediately after `reset_mid`, and the failure looked like a Link that hung.
        seen_clear = Signal(name = "seen_clear")
        src += If(clear_x & self.busy, seen_clear.eq(1))
        # Reloaded while idle, counts only while a request is outstanding: a bound on *an access*, not a
        # free-running timer that would fire during the idle gap between two accesses 20 ms apart.
        timer   = Signal(32)
        expired = Signal()
        self.comb += expired.eq((timer == 0) & (self.watchdog != 0))
        src += If(self.busy, If(timer != 0, timer.eq(timer - 1)))
        src += If(~self.busy, timer.eq(self.watchdog))
        lost = Signal()
        # What counts as lost, and what does NOT.
        #
        #   `clear_x`      -- the far side says it was erased. This is the PHY-reset path, and the reason it
        #                     travels as a level: a pulse cannot be generated in a domain that may have no
        #                     clock left.
        #   `seen_clear`   -- the clear came and went between two of our cycles (a PHY reset shorter than
        #                     ~40 ns is not a thing we can hope to catch, but the level plus this witness is
        #                     what catches the ordinary case without a pulse synchroniser).
        #   `expired`      -- our own bound. Catches a bug in this file or a destination that vanished.
        #
        # Deliberately absent: "the destination stopped claiming work". `dst_active` is the request level at
        # the far end, so it drops in the *same* crossing window as the ack rises, and a detector on it fires
        # on a perfectly successful completion -- it was this bridge's second bug, and it declared every
        # completed access lost. `busy & ~ack_edge` cannot save it: `ack_edge` is a single cycle and the two
        # levels are asynchronous to each other. A destination that really does forget a transaction is
        # caught by `expired` instead, which is slower and always correct.
        self.comb += lost.eq(self.busy & ~ack_edge & (clear_x | (seen_clear & ~clear_x) | expired))
        # ...and the **withdraw**. The source gives up on a lost access by dropping `req_level`, and a
        # destination being erased cannot answer at all; either way the far side's `dst_req` has to go with
        # it. Only `dst_ack` released it, so a master cleared by a PHY reset left `dst_req` stuck high, and
        # the `~self.dst_req` guard in the edge detect above then swallowed the *next* access: the failure
        # was one row late and in a different row, which is how the freeze vector (`reset_mid_noclk_rd`)
        # found it on 2026-10-31 -- that row clears a master whose clock was gone before it could ever ack.
        # Harmless in the normal flow: `req_level` falls only on `ack_edge` or on `lost`, and by the
        # `ack_edge` case the destination has long since released `dst_req` itself.
        dst += If(~req_high | clear_x, self.dst_req.eq(0))
        # The witness is per-access: cleared when one starts and when one is declared lost. Left sticky, it
        # marks every later access lost from the first PHY reset onward (found on the vector right after
        # `reset_mid`, where it looked like a Link that hung).
        src += If(start | lost, seen_clear.eq(0))
        src += If(lost, [
            self.err_lost.eq(1),
            self.busy.eq(0),
            req_level.eq(0),
            self.lost_count.eq(Mux(self.lost_count != 0xff, self.lost_count + 1, self.lost_count)),
        ])
        # A request that is lost never produced dst traffic at all, and the master is the one place that
        # can say so -- it is the only holder of `dst_req`.
        dst += If(clear_x & self.dst_req, self.dst_dropped.eq(1))
        src += If(self.busy, timer.eq(Mux(timer != 0, timer - 1, timer)))
        src += If(~self.busy, timer.eq(self.watchdog))
