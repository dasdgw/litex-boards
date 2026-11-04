#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2026 Pano Logic G2 reverse-engineering community
# SPDX-License-Identifier: BSD-2-Clause

"""``Wishbone2AXILiteCDC``: the ``cd_sys`` Wishbone side of a peripheral whose registers live on the
PHY's 60 MHz.

Why this exists at all
----------------------

``core_usb_host``'s register window (``cfg_aw*/cfg_w*/cfg_b*/cfg_ar*/cfg_r*``) is documented as
synchronous to the UTMI/ULPI clock: *"The core requires a 48MHz/60MHz clock input, which the AXI4-Lite and
UTMI interfaces are expected to be synchronous to"* (``core_usb_host/README.md``, Instantiation), and
``PLAN.md`` M3.2 refuses the alternative -- running the core on ``cd_sys`` would leave every ULPI pad
launch undefined against the PHY's forwarded clock. So the two domains are a fact, and something has to
sit between the CPU's bus and a register file that is clocked by a signal that **can stop** (PHY reset,
Low Power Mode -- ``RESEARCH.md`` §7.10).

That last clause is the whole reason this is not ``stream.ClockDomainCrossing`` or one of LiteX's
AXI bridges. A FIFO would happily hold a write across the event that erases the far side, and that write
would then be delivered to a core that has forgotten the transaction that earned it. It is the same
argument as ``pano_cdc.py``'s rejection of a FIFO for the ULPI Link (spec decision 10), and it is why this
module is built on :class:`~litex_boards.extensions.pano_cdc.ReqAckCDC` rather than beside it: that
primitive already answers *the only question that matters here*, "did the far side survive my access?",
with ``err_lost``, and its docstring names Track A as the reason it is generic.

What "lost" means to the CPU, and why it is ``err`` and not ``ack``
-------------------------------------------------------------------

``err_lost`` drives ``wishbone.err``. VexRiscv connects that pin (``i_dBusWishbone_ERR``,
``litex/soc/cores/cpu/vexriscv/core.py``), so a lost access becomes a load/store access fault with a
register dump on the console. The alternative -- ``ack`` with ``dat_r = 0`` -- was rejected, and it is
worth being explicit because this repo has been burned from both directions:

* ``ack``-with-zero is the failure mode ``docs/NOTES.md`` calls *"litex answers a bridge timeout with
  ``[0] * length`` -- silent"*. On this bus ``0x00`` is a **plausible** core reading (``USB_STATUS`` with
  no packet seen), so a lost read would be reported as a healthy device that happens to be idle.
* An access fault is loud, and at this stage of the bring-up loud is what we are paying for. It is also
  recoverable by reloading the bitstream, which is what we would do anyway.

The counter-weight is recorded rather than hidden: **the BIOS enumeration test (M3.3) must tolerate a
fault**, and the CDC's own counters (``xfer``/``lost_count``, visible through the ``usb_host_*`` CSRs of
the parent module) are the only way to see a *transient* loss, since the fault itself resets the program.

The core's own ``cfg_*`` handshake, read before writing this (``usbh_host.v:150-152, 685-723``)
----------------------------------------------------------------------------------------------

* ``cfg_arready_o = ~cfg_rvalid_o``; ``cfg_awready_o = ~cfg_bvalid_o & ~cfg_arvalid_i & ~awvalid_q``. So a
  write and a read may **not** be presented together, and this module never does (single-outstanding).
* ``cfg_wstrb_i`` is declared and **never read** anywhere in the file. A byte write is a whole-register
  write. Nothing here pretends otherwise: ``sel`` crosses and is driven out, so the wire tells the truth
  and the core ignores it, and the ``usb_host_*`` CSR does not claim byte granularity it cannot deliver.
* ``cfg_bresp_o``/``cfg_rresp_o`` are ``assign ... = 2'b0`` -- tied OKAY. They still cross, so that a core
  that ever learns to report an error is not silently swallowed here, and ``resp != OKAY`` becomes a
  ``wishbone.err`` exactly like a lost access.
* ``cfg_bready_i``/``cfg_rready_i`` are tied to **1** by the destination FSM, and that is load-bearing, not
  laziness: ``bvalid_q``/``rvalid_q`` clear on ``cfg_bready_i``/``cfg_rready_i``, so a response is always
  consumed even when this FSM has already withdrawn (the ``dst_req``-dropped path below). A master that
  could not accept a response would leave ``bvalid`` asserted forever, and ``awready`` is ``~bvalid`` -- a
  dead core with no error bit. **There is no back-pressure to have**, and the FSM is written for that.

Addressing
----------

LiteX Wishbone addresses in **32-bit words**; the core decodes ``cfg_awaddr_i[7:0]`` against byte offsets
(``USB_CTRL 0x00`` … ``USB_RD_DATA 0x20``, ``usbh_host_defs.v``). So ``byte_addr = {adr, 2'b00}`` and only
``adr[5:0]`` can change the core's decode: the core ignores ``cfg_awaddr_i[31:8]`` itself, so crossing six
address bits is **bit-faithful to the core's own decode** for any region size, including a 64 KB one
(an access at byte ``0x244`` decodes as ``0x44`` on the core and as ``0x44`` here -- both match nothing).
Crossing fewer bits (4, say, because "the registers end at 0x20") would alias ``0x44`` onto ``0x04``,
which is not what the core does and would turn a driver bug into a plausible-looking status read.

Why the base class is ``LiteXModule`` and not migen's ``Module`` (measured, 2026-11-04)
-------------------------------------------------------------------------------------

This class was first written as ``class Wishbone2AXILiteCDC(Module)``, assigned its bridge with
``self.bridge = ReqAckCDC(...)``, and **generated a netlist in which the bridge's ports were declared and
nobody drove them**. Plain migen registers a submodule only through ``self.submodules.x = ...`` or
``self.submodules += ...`` (migen/fhdl/module.py's ``__setattr__`` refuses nothing and adds nothing); the
automatic ``self.x = <Module>`` behaviour is LiteXModule's (``litex/gen/fhdl/module.py``, "``m.module_x = ..``
equivalent of Migen's ``m.submodules.module_x = ..``"). The symptom is precisely the kind this project keeps
meeting: it elaborates, it builds, the crossing simply does not exist. Nothing in the toolchain objects --
the nets are used, they are just never written.

The first run of ``wb_axil_cdc_tb.v`` after the fix is the artifact: **19 of 27 rows red**, and the eight
that were green are the five that assert *nothing happens* (`idle-quiet`, `read-no-aw-w`,
`read-no-early-ack`, `lost-drove-nothing`, `no-ack-on-dead-clock`, `no-timeout-while-idle`) plus the three
that only need the generator to have run (`pre-timeout-nonzero`, `bready-rready-tied-high`, and
`read-no-early-ack`'s companion). **A suite whose green rows are all negative rows has not met its DUT.**
That is worth remembering the next time a suite is quoted as "8 of 27 passing" -- the count alone would
have looked like a partially-working bridge rather than a nonexistent one.

Simulation
----------

Generated to Verilog and executed by ``tools/wb_axil_cdc_sim.py` (docs repo, category ``cdc``) against a
behavioural AXI-Lite slave, and mutation-tested by ``tools/wb_axil_cdc_mutate.py`` (category
``cdc-mutate``). Nothing in this file is proven by having been written.
"""

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import LiteXModule

from litex.soc.interconnect import wishbone

from litex_boards.extensions.pano_cdc import ReqAckCDC

__all__ = ["Wishbone2AXILiteCDC", "REQ_FIELDS", "ACK_FIELDS", "req_unpack", "ack_unpack"]

# Payload layout, one place, named. ``pano_ulpi.py`` does the same for the Link's req/ack so that a field
# is a name in a test as well as a slice in hardware.
REQ_FIELDS = ["we", "adr", "wdata", "strb"]     # 1 + 6 + 32 + 4 = 43
ACK_FIELDS = ["resp", "rdata"]                  # 2 + 32         = 34

ADR_BITS   = 6                                  # see "Addressing" in the module docstring
REQ_WIDTH  = 1 + ADR_BITS + 32 + 4
ACK_WIDTH  = 2 + 32

AXI_OKAY = 0


def req_pack(we, adr, wdata, strb):
    return Cat(we, adr, wdata, strb)


def req_unpack(sig):
    """Field views of a crossed request, in the order named by ``REQ_FIELDS``."""
    return dict(we=sig[0], adr=sig[1:1 + ADR_BITS],
                wdata=sig[1 + ADR_BITS:1 + ADR_BITS + 32],
                strb=sig[1 + ADR_BITS + 32:REQ_WIDTH])


def ack_unpack(sig):
    """Field views of a crossed response, in the order named by ``ACK_FIELDS``."""
    return dict(resp=sig[0:2], rdata=sig[2:ACK_WIDTH])


class Wishbone2AXILiteCDC(LiteXModule):
    """``cd_sys`` Wishbone slave -> ``cd_usb`` AXI4-Lite master, one access in flight.

    Latency: two crossings plus the register access, so ~4-6 ``cd_sys`` cycles of handshake per access.
    Irrelevant next to a full-speed USB transaction (a SETUP packet is ~1 µs on the wire) and next to the
    ~20 ms a CSR transaction costs on this board's serial link.

    ``usb_ok`` is the **``cd_sys`` level** that says the far side exists and is not being held in reset:
    the parent builds it from the probe's clock meter and its PHY reset (see
    :class:`~litex_boards.extensions.pano_usb_host.UsbReset`). It is passed straight to
    ``ReqAckCDC.dst_clear``, which is what makes an access against a reset core report ``err_lost`` rather
    than waiting out the watchdog.
    """

    def __init__(self, sys_clk_freq=50e6, timeout_ms=10, timeout_cycles=None,
                 src_domain="sys", dst_domain="usb"):
        # --- the CPU side -------------------------------------------------------------------------------
        self.wishbone = wishbone.Interface(data_width=32)

        # --- the core side (dst_domain), plain signals on purpose ---------------------------------------
        # ``axi.AXILiteInterface`` would give us ``aw.prot``/``ar.prot`` (and, in this LiteX, an ``id`` on
        # the full-AXI variants) with nowhere to go: the vendor port list has neither. A record field with
        # no destination is a net with no driver, which is exactly the ``reg scope_rst = 1'd0;`` class of
        # finding (docs/NOTES.md). The names below are the core's own port names so that the generated
        # Verilog reads like the core's README.
        self.cfg_awvalid = Signal()
        self.cfg_awaddr  = Signal(32)
        self.cfg_wvalid  = Signal()
        self.cfg_wdata   = Signal(32)
        self.cfg_wstrb   = Signal(4)
        self.cfg_bready  = Signal()
        self.cfg_arvalid = Signal()
        self.cfg_araddr  = Signal(32)
        self.cfg_rready  = Signal()

        self.cfg_awready = Signal()
        self.cfg_wready  = Signal()
        self.cfg_bvalid  = Signal()
        self.cfg_bresp   = Signal(2)
        self.cfg_arready = Signal()
        self.cfg_rvalid  = Signal()
        self.cfg_rdata   = Signal(32)
        self.cfg_rresp   = Signal(2)

        # --- diagnostics, for the parent's CSRs ---------------------------------------------------------
        self.xfer       = Signal(16)     # completed accesses (saturating)
        self.lost_count = Signal(8)      # accesses declared lost (saturating)
        self.err_lost   = Signal()       # sticky until the next request
        self.aborted    = Signal(8)      # destination FSMs withdrawn mid-AXI (saturating)

        # # #

        # "The far side is alive", as a ``cd_sys`` level, attached by the parent (:class:`
        # ~litex_boards.extensions.pano_usb_host.UsbHostCore`). This module deliberately does not invent its
        # own definition: the probe's clock meter is the only witness of that on this board (decision 14),
        # and a second opinion here would be a second, weaker one. Instance-level, not class-level -- a
        # ``Signal()`` in the class body would be **shared by every instance** of this bridge.
        self.usb_ok = Signal()

        wb = self.wishbone
        src = self.sync if src_domain == "sync" else getattr(self.sync, src_domain)
        dst = self.sync if dst_domain == "sync" else getattr(self.sync, dst_domain)

        # --- source: one Wishbone access per bridge request --------------------------------------------
        # Two flags, because one is not enough and the difference is a row each.
        #
        # ``pending`` -- an access is outstanding in the bridge. Not ``bridge.busy``: ``bridge.busy`` falls
        # the cycle the access is *declared lost*, and a Wishbone master keeps ``cyc``/``stb`` high until it
        # is answered, so a ``cyc`` that straddles a loss would start a second bridge request whose answer
        # belongs to a different access than the address the CPU is still holding.
        #
        # ``claimed`` -- this assertion of ``cyc``/``stb`` has already been answered. Cleared **only when the
        # master drops ``cyc``/``stb``, never by our own ack.** Without it the bridge re-issues: ``ack`` is
        # registered, so the master cannot see it before the following cycle, and any master that holds
        # ``stb`` one cycle past the ack (which is what LiteX's own Wishbone masters do --
        # ``spi_bone.py``'s ``BUS-MMAP-READ`` holds ``stb`` through the ack cycle and leaves the state after)
        # gets a **second AXI transaction with the same address**. ``core_usb_host`` makes that a real
        # hazard rather than a curiosity: ``USB_XFER_TOKEN`` *starts a transfer*, and ``USB_WR_DATA`` pushes
        # a FIFO byte, so a doubled write is a doubled command, not an idempotent rewrite.
        #
        # Found by the suite, not by reading: ``read-addr-word-to-byte`` went red on the first run in which
        # reads actually completed, because the access it was judging had happened twice.
        #
        # ``claimed`` is deliberately NOT named ``sent`` and does not copy an idiom from this LiteX: grepping
        # the pinned tree for a re-issue guard finds none, because LiteX's own slaves answer combinationally
        # or in one cycle and never need one. A bridge to a 60 MHz domain is the first thing in this tree
        # that answers many cycles late, so the rule comes from Wishbone's "a claim, once answered, is
        # over" and not from a neighbour.
        bridge = ReqAckCDC(req_width=REQ_WIDTH, ack_width=ACK_WIDTH,
                           src_domain=src_domain, dst_domain=dst_domain)
        self.bridge = bridge

        pending = Signal()
        armed   = Signal()
        start   = Signal()
        src += [
            bridge.req.eq(0),
            wb.ack.eq(0),
            wb.err.eq(0),
        ]
        claimed = Signal()
        self.comb += start.eq(wb.cyc & wb.stb & ~pending & ~claimed)
        src += If(~(wb.cyc & wb.stb), claimed.eq(0))
        src += If(start, [
            bridge.req.eq(1),
            bridge.req_data.eq(req_pack(
                we    = wb.we,
                adr   = wb.adr[:ADR_BITS],
                wdata = wb.dat_w,
                strb  = wb.sel,
            )),
            pending.eq(1),
        ])

        # The answer. ``err`` rather than ``ack`` when the far side did not survive or did not like it.
        #
        # ``armed`` exists because ``ReqAckCDC.err_lost`` is STICKY -- deliberately, so that a loss can be
        # read by someone 20 ms away -- and sticky means the loss of access N is still standing when access
        # N+1 starts. The bridge clears it on its own ``start``, which is one cycle after ours (our ``req``
        # is a registered pulse, so the bridge sees it a cycle later), and in that one cycle a bare
        # ``pending & err_lost`` reports the PREVIOUS access's loss against the new access's address: every
        # access after the first loss fails instantly, which is exactly the failure pano_cdc.py already
        # documents for ``seen_clear`` ("left sticky, it marks every later access lost the moment the first
        # PHY reset happens"). Re-committing a documented mistake one file away is the reason this comment
        # is long: ``next-access-works-after-loss`` and ``recovers-after-clock-returns`` were both red for
        # this one cause, and the pair only exists because the 2026-10-31 withdraw bug was one-row-late too.
        #
        # `armed` therefore means "the bridge has taken THIS request" (it asserts ``busy`` no later than the
        # cycle it clears ``err_lost``), and it is cleared by our own ``start``, so a stale verdict can never
        # be attributed to a fresh access.
        src += If(bridge.busy, armed.eq(1))
        src += If(start,     armed.eq(0))

        lost    = Signal()
        badresp = Signal()
        ack_ok  = Signal()
        self.comb += [
            lost.eq(pending & armed & bridge.err_lost),
            badresp.eq(pending & bridge.ack &
                       (ack_unpack(bridge.ack_data)["resp"] != AXI_OKAY)),
            ack_ok.eq(pending & bridge.ack & ~badresp),
        ]
        src += If(ack_ok, [
            wb.ack.eq(1),
            wb.dat_r.eq(ack_unpack(bridge.ack_data)["rdata"]),
            pending.eq(0),
            claimed.eq(1),
        ])
        src += If(lost | badresp, [
            wb.err.eq(1),
            pending.eq(0),
            claimed.eq(1),
        ])

        src += [
            self.err_lost.eq(bridge.err_lost),
            self.xfer.eq(bridge.xfer),
            self.lost_count.eq(bridge.lost_count),
        ]
        # Our own bound, in *source* cycles, and **non-zero by default**: ReqAckCDC treats 0 as "watchdog
        # disabled", so a default of 0 here would be a silent removal of the only protection against a
        # destination that cannot answer for a reason ``dst_clear`` never showed. 10 ms is 500 000 cd_sys
        # cycles: ~600x a full-speed USB frame, and ~10^4x a healthy crossing. It is NOT sized against the
        # ~20 ms a CSR transaction costs **on the serial console** (docs/NOTES.md) -- that is the human's
        # link, not this hardware path, and a watchdog that measured the difference between two console
        # commands would never fire.
        # `timeout_cycles` exists so the simulation can test a timeout without 500 000 edges, and is kept as
        # an attribute so a runner can print the bound it actually built rather than the one the caller asked
        # for -- the difference between those two is how a test can be green around a disabled watchdog.
        if timeout_cycles is None:
            timeout_cycles = int(sys_clk_freq * timeout_ms / 1e3)
        self.timeout_cycles = timeout_cycles
        src += bridge.watchdog.eq(timeout_cycles)
        # The probe's ``usb_ok`` doubles as the bridge's "the far side is being erased" level.
        self.comb += bridge.dst_clear.eq(~self.usb_ok)

        # --- destination: drive one AXI4-Lite access, then ack ------------------------------------------
        # ``bready``/``rready`` tied high, for the reason in the module docstring: the core's own valids
        # clear on them, so a response can never wedge a master that has already withdrawn.
        dst += [
            self.cfg_bready.eq(1),
            self.cfg_rready.eq(1),
            self.cfg_awvalid.eq(0),
            self.cfg_wvalid.eq(0),
            self.cfg_arvalid.eq(0),
            bridge.dst_ack.eq(0),
        ]

        IDLE, WR, AR, WB_WAIT, R_WAIT = 0, 1, 2, 3, 4
        state = Signal(3, reset=IDLE)
        req   = req_unpack(bridge.dst_data)

        # ``dst_req`` is a LEVEL that stays high until the ack, and the ack is what returns this FSM to
        # IDLE -- so on the ack cycle itself the level is still high and a level-triggered start re-issues
        # the whole AXI transaction with the same address. `n_aw`/`n_w` == 2 for one Wishbone write is what
        # that looks like, and it is what the first run of `write-one-of-each-channel` measured. The fix is
        # the rule pano_cdc.py states in its own header -- levels cross, edges are made locally -- applied
        # here, where "locally" means "in the domain that has to know a request has begun".
        #
        # `state == IDLE` stays in the guard even though an edge should imply it. If they ever disagree the
        # request stalls instead of double-issuing, and a stall is what the watchdog reports -- visible --
        # where a double issue is what a USB core receives as a second transfer -- invisible.
        dst_req_was = Signal()
        dst += dst_req_was.eq(bridge.dst_req)
        dst_req_edge = Signal()
        self.comb += dst_req_edge.eq(bridge.dst_req & ~dst_req_was)

        # A withdrawn request is not a completed one. ``dst_req`` falls when the source gives up (lost) or
        # when the destination is being cleared, and either way this FSM must return to IDLE *and say so*:
        # an abandoned AXI transaction is the one case where the core may still be answering, and a silent
        # abandonment would read later as a core that answers twice.
        withdrawn = Signal()
        self.comb += withdrawn.eq(~bridge.dst_req & (state != IDLE))
        dst += If(withdrawn, [
            state.eq(IDLE),
            self.aborted.eq(Mux(self.aborted != 0xff, self.aborted + 1, self.aborted)),
        ])

        dst += If(dst_req_edge & (state == IDLE), [
            # Byte address = word address << 2; see "Addressing" above.
            self.cfg_awaddr.eq(Cat(C(0, 2), req["adr"])),
            self.cfg_araddr.eq(Cat(C(0, 2), req["adr"])),
            self.cfg_wdata.eq(req["wdata"]),
            self.cfg_wstrb.eq(req["strb"]),
            If(req["we"], [
                self.cfg_awvalid.eq(1),
                self.cfg_wvalid.eq(1),
                state.eq(WR),
            ]).Else(
                self.cfg_arvalid.eq(1),
                state.eq(AR),
            ),
        ])

        dst += If(state == WR, [
            self.cfg_awvalid.eq(1),
            self.cfg_wvalid.eq(1),
            If(self.cfg_awready & self.cfg_wready, [
                self.cfg_awvalid.eq(0),
                self.cfg_wvalid.eq(0),
                state.eq(WB_WAIT),
            ]),
        ])
        dst += If(state == AR, [
            self.cfg_arvalid.eq(1),
            If(self.cfg_arready, [
                self.cfg_arvalid.eq(0),
                state.eq(R_WAIT),
            ]),
        ])
        dst += If(state == WB_WAIT, [
            If(self.cfg_bvalid, [
                bridge.dst_ack_data.eq(Cat(self.cfg_bresp, C(0, 32))),
                bridge.dst_ack.eq(1),
                state.eq(IDLE),
            ]),
        ])
        dst += If(state == R_WAIT, [
            If(self.cfg_rvalid, [
                bridge.dst_ack_data.eq(Cat(self.cfg_rresp, self.cfg_rdata)),
                bridge.dst_ack.eq(1),
                state.eq(IDLE),
            ]),
        ])
