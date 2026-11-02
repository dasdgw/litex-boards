#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2026 Pano Logic G2 reverse-engineering community
# SPDX-License-Identifier: BSD-2-Clause

"""``analyzer_usb``: the 60 MHz capture of the ULPI bus, and the clock domain it cannot work without.

Why a second analyzer at all
----------------------------

The ``cd_sys`` analyzer measures 50 MHz samples of a 60 MHz bus. That answers *state* questions (spin 1
used it exactly that way) and cannot answer *byte* questions: a ULPI byte occupies one ``CLKOUT`` cycle, so
a 50 MHz sample of it is a coin flip. When the ID comes back wrong the only thing that says *whose* byte is
wrong is a waveform taken at the PHY's own rate, and the serial link is ~20 ms per CSR transaction -- there
is no probe to put on these pins and no way to re-run the transaction faster than a human reads a register.
Spec decision 20.

Why a second *clock domain*, which is the part that looks like style
-------------------------------------------------------------------

``cd_usb`` is ``reset_less`` **because** the PHY's clock stops when the PHY is in reset (see ``_CRG``): a
domain with no clock cannot hold a synchronous reset. But LiteScope's ``_Storage``/``_RLE`` are FSMs written
with ``reset_state=`` (`litescope/core.py`), and an FSM in a reset-less domain gets its reset *silently
dropped*. That is not a hypothesis: the netlist of `ulpi_t13p`/`ulpi_t16` -- the build that has been on the
board -- contains ::

    reg scope_rst = 1'd0;        // declared, INIT 0
    ...
    if (scope_rst) begin         // and read by the storage FSM's reset mux

with **no assignment to ``scope_rst`` anywhere in the file** (checked: one declaration, one use, zero
drivers). The spin-1 analyzer's reset has always been tied to 0 by its power-up value; it worked because a
power-up of all-zeros happens to be that FSM's ``IDLE``. "It worked" is not the same sentence as "the reset
exists", and a spin whose capture is suspect cannot afford the difference.

So this module creates ``cd_usb_scope``: the same buffered ``C12`` net (no second ``BUFG`` -- Spartan-6 has
eight global buffers and this design already uses most of them; the *domain* is new, the clock net is not),
with a **boot-counter reset** (spec decision 21). The consequences are all deliberate:

* the reset releases only after ``boot_done`` **and** two edges of ``C12`` -- so if the PHY is not clocking
  us the analyzer stays in reset, which is correct: you cannot capture what is not clocking;
* "boot" has one definition in a build, because ``boot_done`` comes from the Link's own
  :class:`~litex_boards.extensions.pano_ulpi.UlpiClear`, not from a second counter that could disagree with
  it about when the FPGA was configured;
* ``cd_scope`` (LiteScope's own internal domain) is handed the same reset by the target, which is the whole
  point of creating the domain here rather than passing ``clock_domain="usb"``.

What is in the group
--------------------

One group, ``samplerate=60e6``, ``with_rle=False`` (decision 22: ``core.py`` documents an RLE memory bug
"shortly after reset or re-arming", and a new reset domain plus RLE in one spin leaves a bad capture with
two suspects). The slots are spin 1's ``ULPIProbe.RESERVED_GRP_NAMES`` made real, plus the Link's own state:

The name column is what ``usb_scope.csv`` actually says, measured from a build, not what was asked for:
litex.gen prefixes a signal with its owning module path, so a tap declared **in here** is ``ulpi_scope_*``
while a tap that *is a pad* keeps the platform's name. That asymmetry is the CSV, so the trigger strings on
the bench have to match it; ``tools/test_ulpi_csr_map.py`` pins every row of this table verbatim.

============================  ====  ======================================================
``ulpi_scope_data``           8     the data PADS (via the Link's IOBUFs), not ``tx_byte``:
                                    the PHY's byte and ours appear here indistinguishably,
                                    which is the point -- see the NgdBuild:924 note below
``ulpi_dir``                  1     raw pad (the PHY claiming the bus)
``ulpi_nxt``                  1     raw pad (the throttle)
``ulpi_stp``                  1     the wire driving the pad, i.e. our strobe
``ulpi_scope_clk_ok``         1     spin 1's clock meter, synchronised in: "was C12 there?"
``ulpi_scope_state``          4     the master's FSM, in the domain that runs it
``ulpi_scope_oe``             1     our output enable at that moment
``ulpi_scope_busy``           1     the ``go`` anchor: set by ``go``, held for the whole access
``ulpi_scope_err``            4     dir_abort, nxt_timeout, no_clock, reset -- the arm-on-error row
============================  ====  ======================================================

Three of the five names ``ULPIProbe.RESERVED_GRP_NAMES`` reserved are exact (``ulpi_dir``, ``ulpi_nxt``,
``ulpi_stp``). The other two carry the module prefix, for two different and both unfixable reasons:
``ulpi_scope_data`` because the pad net owns the name ``ulpi_data`` and a bidirect pad net may not drive
fabric at all, and ``ulpi_scope_clk_ok`` because litex.gen prefixes a signal with its owning module and
building the taps in the target to dodge a prefix is decision 5 refusing to happen for a cosmetic reason.

22 bits, which is also a hard limit worth knowing: above 32 the ``_Storage`` read path instantiates a
``stream.Converter`` in ``cd_sys`` and the capture's word order becomes a second thing to debug.

Every tap is a **named** ``Signal``, never an expression. ``LiteScopeAnalyzer.format_groups()``
de-duplicates through a dict and a migen ``_Operator`` is unhashable, so the analyzer refuses to tap
``~dir`` outright; and in the VCD an unnamed tap is ``ulpimaster7`` at 1 am (spin 1 learned both).

The ``go`` anchor is ``ulpi_scope_busy`` and not the ``go`` edge itself, and that is a correctness choice rather
than a convenience: ``go`` is one cycle of ``cd_sys`` (20 ns) sampled by a 60 MHz clock that is *asynchronous
to it* -- such a pulse is seen usually but not always, and a trigger that misses is a capture that looks like
a dead instrument. ``busy`` is raised by the same event, spans the whole access, and cannot be missed.

The cross-domain CSR caveat (spec decision 24), written here so it is found before it bites
-------------------------------------------------------------------------------------------

``LiteScopeAnalyzer`` writes its ``storage``/``trigger``/``length`` CSRs from ``cd_sys`` and reads them in
the **scope** domain through plain ``MultiReg``s, with no handshake. Configuration must therefore be
settled **before** arming, and one capture must not be re-armed from software mid-flight. That is fine for
us -- the harness configures, then enables, then triggers, and a capture at 60 MHz with ``depth=2^14`` is
273 us, far shorter than the ~20 ms it takes the host to do anything. It would be a real bug for anyone who
later tries to re-arm while samples are flowing: they would get a capture whose ``length``/``offset``
changed underneath it, and nothing in the data says so.

What this module does NOT do
----------------------------

It does not instantiate the ``LiteScopeAnalyzer``, for two reasons that are really one:

* the client derives the analyzer's CSR prefix from the **basename of ``--csv``**
  (``litescope/software/litescope_cli.py``: ``basename = splitext(basename(args.csv))[0]``; then
  ``driver/analyzer.py`` matches ``name + "_"`` against the remote's register names). So the prefix and the
  CSV name are one decision, taken once, and nested in here the instance would answer to
  ``ulpi_scope_analyzer_trigger_enable`` and die exactly the way spin 1's first attempt died (measured
  2026-10-30: "'LiteScopeAnalyzerDriver' object has no attribute 'trigger_enable'").
* **which** top-level name is not free. LiteX allocates one CSR region per module and lays the regions out
  **alphabetically** (0x800 apart), so a name decides which measured addresses move. The spec asked for
  ``analyzer_usb``; built under that name it sorted between ``analyzer`` and ``ctrl`` and shifted **all 71**
  addresses the golden map pins -- `probe_rst_blind` 0x2070 -> 0x2870, every ``leds``/``sdram``/``timer``
  name too -- and ``tools/test_ulpi_csr_map.py`` refused the build. The instance is therefore ``usb_scope``:
  it sorts after every existing region (``ulpi`` is the last), so nothing spin 1 measured moves, and it is
  named after the clock domain this whole module exists to create. ``--csv usb_scope.csv``.
"""

from migen import *

from migen.genlib.cdc import MultiReg

from litex.gen import LiteXModule

__all__ = ["UlpiUsbScope"]


class UlpiUsbScope(LiteXModule):
    """``cd_usb_scope`` + the ULPI bus taps, as ``groups`` for a ``LiteScopeAnalyzer``.

    ``pads``      the ``ulpi`` pad group, read (never driven) here.
    ``link``      the :class:`~litex_boards.extensions.pano_ulpi.UlpiLink` whose FSM, ``oe`` and latched
                  errors are worth seeing. Its ``ulpi_clear.boot_done`` is the build's one definition of
                  "configured and counted", so the scope's reset and the Link's clear cannot disagree.
    ``domain``    the name of the domain created here (``usb_scope`` -> ``cd_usb_scope``).
    """
    def __init__(self, pads, link, domain = "usb_scope"):
        # --- the domain ----------------------------------------------------------------------------
        # Same clock *net* as cd_usb, new *domain*, because the reset is what this is for. Driving a
        # domain's clk from ClockSignal() is what LiteScope itself does for cd_scope, so it is a shape the
        # backends know; and re-using cd_usb's buffered net rather than a second IBUFG/BUFG pair is what
        # keeps this off the global-clock-buffer budget.
        self.cd_usb_scope = ClockDomain()
        self.comb += self.cd_usb_scope.clk.eq(ClockSignal("usb"))

        # --- the boot-counter reset ------------------------------------------------------------------
        # A level from cd_sys (`UlpiClear.boot_done`), asserted on its rising edge with no clock required
        # and released only by TWO edges of C12. Both halves matter:
        #   * async assert: `boot_done` can drop the instant the sys clock stops, and the analyzer must be
        #     in reset whenever the design is not booted;
        #   * synchronised release: an asynchronous *de-assertion* into a domain that may have stopped is
        #     exactly the recovery race this whole file exists to avoid -- and it is what makes "no PHY
        #     clock" mean "still in reset" rather than "half-configured FSM" (decision 21).
        # Written out instead of `AsyncResetSynchronizer` because that is a `Special` whose lowering lives
        # in the platform back-end; here the two flip-flops are the whole design and they survive a
        # `--no-compile` audit of the generated Verilog.
        # `boot_done` is `UlpiClear`'s own cd_sys signal, and the *only* thing this module takes from the
        # clear: the crossing that needs synchronising is into cd_usb_scope, which is what rst_ff below is.
        boot_done = link.ulpi_clear.boot_done
        rst_ff = Signal(2, name = "usb_scope_rst_ff")
        scope  = getattr(self.sync, domain)
        scope += rst_ff.eq(Cat(rst_ff[1], ~boot_done))
        self.comb += self.cd_usb_scope.rst.eq(rst_ff[1])
        self.scope_rst = rst_ff[1]      # the target hands it to LiteScope's own cd_scope.

        # --- the taps ------------------------------------------------------------------------------
        # `ulpi_clk_ok` and the Link's cd_sys status bits cross on MultiRegs: they are *levels* that live
        # for milliseconds, and one metastable sample in a debug capture is a dot on a waveform rather than
        # a decision. The pad taps and the master's own state need none -- they are already synchronous to
        # C12, which is the entire reason this capture can be honest about a byte.
        # `data` is the ONE tap that cannot be the pad itself. `ulpi_data` is a *bidirectional* top-level
        # net, and ngdbuild refuses a bidirect pad net that drives fabric:
        #     ERROR:NgdBuild:924 - bidirect pad net 'ulpi_data<0>' is driving non-buffer
        # (all eight bits, measured on the first `ulpi_t17` attempt, 2026-10-31 -- the tap that looks most
        # obviously right is the one the tool forbids, and it is a build error rather than a warning, so the
        # spin simply does not exist). The pad's input buffer already exists for a different reason: the
        # Link's own `TSTriple`. Its `.i` is the same eight bits one buffer later, it is what Track A's
        # receiver will read too, and `OFFSET = IN` still applies to it -- the constraint is on the pad, not
        # on which of its buffers the tap hangs off.
        #
        # **`stp` joined `data` on 2026-11-02 and it is the same error, not an exception.** `stp` used to be a
        # plain output, so tapping the pad net was legal and spin 1 always did. It is an `IOBUF` now, because
        # `ulpi_stp_pad` (TODO (e): is A5 pulled up?) cannot be answered from an `OBUF` -- and the moment the
        # pad is bi-directional, ngdbuild's rule applies to it too. Tapping `pads.stp` here after that change
        # is not a warning to read later; it is a build that does not exist, with `924` naming eight... one
        # bit. So: `link.master.stp_pad`, which is that `IOBUF`'s own `I`, one buffer later and the same
        # level. `dir`/`nxt` stay the raw pads, because those really are single-direction.
        data_t = Signal(8, name = "data")
        self.comb += data_t.eq(link.master.tri.i)
        stp_t  = Signal(name = "stp")
        self.comb += stp_t.eq(link.master.stp_pad)

        clk_ok_t = Signal(name = "clk_ok")
        self.specials += MultiReg(link.clk_ok, clk_ok_t, odomain = domain)

        state_t = Signal(4, name = "state")
        self.comb += state_t.eq(link.master.state)
        oe_t = Signal(name = "oe")
        self.comb += oe_t.eq(link.master.oe_out)

        busy_t = Signal(name = "busy")
        self.specials += MultiReg(link.h_busy, busy_t, odomain = domain)
        err_t = Signal(4, name = "err")        # dir_abort, nxt_timeout, no_clock, reset
        for i, s in enumerate([link.err_dir_abort, link.err_nxt_timeout,
                               link.err_no_clock, link.err_reset]):
            self.specials += MultiReg(s, err_t[i], odomain = domain)

        # One group; the analyzer's `register=True` puts a register stage on all of it, in this domain
        # (`sd = getattr(self.sync, clock_domain)` in core.py), so the pad taps' D cone is pad -> IBUF ->
        # mux(1:1) -> flop and the OFFSET = IN budget is the only thing it has to meet.
        self.groups = {
            0 : [ data_t, pads.dir, pads.nxt, stp_t,
                  clk_ok_t, state_t, oe_t, busy_t, err_t ],
        }
