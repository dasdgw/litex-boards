#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2026 Pano Logic G2 reverse-engineering community
# SPDX-License-Identifier: BSD-2-Clause

"""Track A's USB host: ``core_usb_host`` + ``core_ulpi_wrapper`` behind a Wishbone window.

What this file is, and what it is not
-------------------------------------

It is the **integration** of two vendor Verilog modules (``ultraembedded/core_usb_host@81eb9f1`` and
``ultraembedded/core_ulpi_wrapper``) into this board's clocking and pad policy, plus the one piece of new
logic the integration genuinely needs (:class:`~litex_boards.extensions.pano_wb_axil.Wishbone2AXILiteCDC`).
It is **not** a re-implementation of either core, and it does not touch the ULPI PHY reset, the 24 MHz
reference clock or the hub reset: those belong to the probe
(:class:`~litex_boards.extensions.pano_ulpi.ULPIProbe`), which is decision 8 of the ULPI-ID-read spec and
stays in force here -- ``--with-usb-host`` requires ``--with-ulpi-probe`` and refuses to duplicate any of
them. See also *Pad ownership* below, which is the decision this build adds.

Every choice below is traceable to a port list or a measurement, and the two that are traceable to a
measurement are named as such. Where this file says "the core ignores ``cfg_wstrb_i``", that is a grep of
``usbh_host.v``, not an assumption about AXI.

The 2023 wrapper this replaces, and the four things it got wrong
----------------------------------------------------------------

``litex/soc/cores/usb_host.py`` on our ``litex`` fork's ``pano-usb-host`` branch (``PLAN.md`` §0.1 says
"Phase M3 starts from *your* ``usb_host.py``") is the starting point, and it is not reusable as written:

1. **``p_USB_CLK_FREQ = "60e6"`` is a string, and Verilog accepts it.** migen emits a python ``str``
   parameter verbatim: ``.USB_CLK_FREQ("60e6")`` (reproduced with the pinned migen, 2026-11-04 -- not a
   guess, see ticket 01). The core then does arithmetic on it::

       localparam CLKS_PER_BIT  = (USB_CLK_FREQ / 12000000);   // 0x36306536/12e6 = **75**, not 5
       localparam SOF_THRESHOLD = (USB_CLK_FREQ/1000)-1;        // 909196, not 59999
       localparam RX_TIMEOUT_FS = (USB_CLK_FREQ == 60000000) ? 12'd511 : 12'd255;   // false -> 255

   A 32-bit string constant is ``909196854``, so that core would bit-bang at **75 clocks per bit instead of
   5** (800 kbit/s instead of 12 Mbit/s), send a SOF every ~15 ms instead of every 1 ms, and time out on
   every reply at half the intended window. **No device would ever answer and nothing would report an
   error** -- which is worth writing down because it is the failure shape this project keeps meeting: a
   wrong constant that produces a plausible-looking silence. Here the parameter is ``int(usb_clk_freq)``.
2. **No CDC.** ``s_axil`` was handed straight to a core on ``cd_usb`` from whatever bus the integrator
   connected, i.e. a Wishbone/AXI-Lite interface on ``cd_sys`` driving a register file on ``C12``. That is
   the entire subject of :mod:`pano_wb_axil`.
3. **``self.comb += pads.reset.eq(0)`` with a ``# todo remove``.** The PHY reset pad on *this* board is
   active-high and owned by the probe (measured 2026-11-01, ``RESEARCH.md`` §7.8/§7.9 -- pad high = deaf),
   so tying it to a constant is both a second owner of a pad the probe already drives and an accident
   waiting for a polarity flip. This module does not touch ``pads.reset``.
4. **``os.system("git clone …")`` at build time**, into the build directory, on every build. Sources come
   from ``$CORE_USB_HOST``/``$CORE_ULPI_WRAPPER`` (see ``add_core_sources``) or the build refuses.

Pad ownership -- the decision this build adds, and why it refuses rather than arbitrates
----------------------------------------------------------------------------------------

Three things want the ULPI pads: the probe (observes, and drives the idle level), the CSR Link
(``--with-ulpi-master``, RegRead/RegWrite) and now the core's ``ulpi_wrapper`` (packets). ``pano_ulpi.py``
prepared for this with ``bus_grant`` ("when the 2023 ``usb_host`` core joins… the arbitration lands here"),
and this build **does not use it**. Two reasons, one electrical and one about what we can verify:

* ``ulpi_wrapper`` has **no output-enable port**. Its ULPI data direction is produced outside it, by
  ``oe = ~dir`` (that is what the 2023 wrapper did, and CODING_STANDARDS.md's ``oe`` rule permits exactly
  the *turnaround release* form of it). So the core is always driving whenever ``dir`` is low, and cannot
  be told to release without inventing a gate that changes what the vendor module means. A mux in front of
  ``data``/``stp`` to share the pads with the Link would sit directly on the path that already fails
  ``OFFSET = OUT`` by 3.3 ns (ADR-0005, ``bench/2026-10-31_ulpi_t16/``): the cost is measurable and the
  benefit is a second master we cannot test against the first.
* The Link's arbitration is a ``bus_grant`` input that its testbench proves is honoured *when it is 0*.
  The wrapper has no such input, so an arbiter would have one polite participant and one that cannot hear.

So ``--with-usb-host`` and ``--with-ulpi-master`` are **mutually exclusive in the target**, with a message
that says why, and the probe keeps ``bus_owner`` semantics for the analyzer taps. The ULPI ID-read
instruments are not lost: they live in another spin, and a spin that owns the pads differently has a
different ``csr.csv`` anyway.

Reset, in a domain whose clock can stop
---------------------------------------

Both vendor modules reset asynchronously (``always @(posedge clk_i or posedge rst_i)``), which is the one
kind of reset that works when ``C12`` may be absent (CODING_STANDARDS.md, second bullet). What they do
**not** have is any way to know when it is safe to leave reset: USB3300 §6.1.11 requires ``CLKOUT`` to be
active at reset release, and ADR-0006's ``probe_rst_blind`` exists because we could only ever *count*
releases that happened while the clock meter said nothing. :class:`UsbReset` makes that a construction
instead of a counter: the core's reset is ``~clk_ok``-derived, so a release cannot happen while the meter
is unhappy, and ``probe_rst_blind`` should read 0 for the rest of this board's life **for a structural
reason**. That is a claim about hardware we have not built yet; ticket 03 says how to falsify it.
"""

import os
import subprocess

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import LiteXModule

from litex.soc.interconnect.csr import CSRField, CSRStatus, CSRStorage

from litex_boards.extensions.pano_wb_axil import Wishbone2AXILiteCDC

__all__ = ["UsbReset", "UsbHostCore", "core_source_dirs", "USBH_REGS"]

# The core's register file, as names. Owned here because the BIOS test (M3.3) and this file must not
# disagree with each other about an offset, and the vendor header is not on the include path of either.
# Byte offsets, exactly as `usbh_host_defs.v` decodes `cfg_awaddr_i[7:0]`.
# Bit fields below are `usbh_host_defs.v@81eb9f1`, restated because a wrong one here is how a bench session
# writes the wrong value: an earlier version of this comment claimed `[5] SOF_EN`, `[1] USB_RST` and
# `[0] CORE_RST`, none of which exist, and a `USB_CTRL` built from it (`0xC0`) selects the PHY's **Host
# Hi-Speed** signaling instead of Host Full Speed -- measured on `usb_t3`, where `0xE8` tracks the hub's line
# state and `0xC0` does not (panologic_g2 `bench/2026-11-05_usb_ctrl_hostfs_t3/`). Core reset is the
# `usb_host_core_rst` CSRStorage below, not a bit in this register file.
USBH_REGS = {
    "USB_CTRL":      0x00,   # [8] TX_FLUSH (auto-clear, NOT in the read mux) [7] DM_PD [6] DP_PD
                             # [5] TERMSELECT [4:3] XCVRSELECT (01b = FS) [2:1] OPMODE [0] ENABLE_SOF
                             # Host Full Speed = 0xE8 (usb3300.pdf Table 6-8; ue11-hcd usbhw_hub_enable)
    "USB_STATUS":    0x04,   # [31:16] SOF_TIME: counts usb_clk, HOLDS at USB_CLK_FREQ/1000-1 (0xEA5F @60MHz)
                             # and resets only on send_sof_w -- so a pinned 0xEA5F means "no SOF ever fired",
                             # not "the field is unused". [2] RX_ERROR [1:0] LINESTATE (a LATCH: written only
                             # by the wrapper's RX CMD decode, reset 0, so 00 = "no status byte since reset")
    "USB_IRQ_ACK":   0x08,   # WRITE-ONLY, every bit a one-cycle strobe (auto_clr), and the read mux has no
                             # arm for it -- a read is always 0, so NEVER verify an ACK by read-back; verify it
                             # by re-reading USB_IRQ_STS (measured: 0x1 -> ACK -> 0x0).
                             # [3] DEVICE_DETECT [2] ERR [1] DONE [0] SOF
    "USB_IRQ_STS":   0x0c,   # sticky until ACKed, and readable with USB_IRQ_MASK at 0. [3] DEVICE_DETECT IS A
                             # HARDWIRED 1'b0 (usbh_host.v:1076 -- device_det_q feeds only intr_o), so that bit
                             # can never read 1 in any configuration: do not put it in an acceptance list.
                             # [2] ERR [1] DONE [0] SOF. DONE cannot fire for a core-generated SOF at all
                             # (usbh_sie.v writes status_tx_done_q only in STATE_TX_CRC2, which the SOF route
                             # TOKEN1->TOKEN2->TOKEN3->TX_IFS->IDLE never enters).
    "USB_IRQ_MASK":  0x10,   # [3:0] gates ONLY intr_o; the status bits above are live regardless of the mask.
                             # Keep it 0 on a bench where no handler owns that CPU interrupt line.
    "USB_XFER_DATA": 0x14,   # [15:0] TX_LEN
    "USB_XFER_TOKEN":0x18,   # [31] START [30] IN [29] ACK [28] PID_DATAX [23:16] PID [15:9] DEV_ADDR [8:5] EP
    "USB_RX_STAT":   0x1c,   # [31] START_PEND [30] CRC_ERR [29] RESP_TIMEOUT [28] IDLE (the SIE's own idle)
                             # [23:16] RESP_BITS [15:0] COUNT_BITS
    "USB_WR_DATA":   0x20,   # Tx FIFO (write) -- same offset as USB_RD_DATA
    "USB_RD_DATA":   0x20,   # Rx FIFO (read)
}


# The commits the low-speed patch set (`tools/core_usb_host-*.patch`), `tools/sync_pin.txt` and every bench
# note are written against. Not enforced by the build (see `vendor_provenance`), but named by it.
CORE_SOURCE_PINS = {"core_usb_host": "81eb9f1", "core_ulpi_wrapper": "3c20296"}


def _git_state(path):
    """``(short_head, dirty_or_na)`` for a vendor checkout, never raising.

    ``dirty_or_na`` is ``"clean"``, ``"DIRTY(+n)"`` (n = changed/untracked paths that would be compiled) or
    ``"not a git tree"``. Read-only: ``rev-parse`` and ``status --porcelain`` write nothing, so a build can
    ask this of a tree it is about to consume. Failures return ``("?", "git unavailable")`` rather than an
    exception, because a provenance line must never be the reason a bench build dies.
    """
    def git(*args):
        try:
            r = subprocess.run(["git", "-C", path] + list(args), capture_output=True, text=True,
                               timeout=20)
        except Exception:
            return None
        return r.stdout if r.returncode == 0 else None

    head = git("rev-parse", "--short", "HEAD")
    if head is None:
        return "?", "not a git tree"
    status = git("status", "--porcelain", "--untracked-files=normal")
    if status is None:
        return head.strip(), "git unavailable"
    n = len([line for line in status.splitlines() if line.strip()])
    return head.strip(), ("clean" if n == 0 else "DIRTY(+%d)" % n)


def vendor_provenance(roots=None):
    """One line per vendor tree: which commit the build is about to compile, and whether it is unmodified.

    **Why this exists, measured:** on 2026-11-05 a spin was built, loaded and pressed on the bench while
    ``core_usb_host``'s working tree held both low-speed patch sets (``git status``: three modified files,
    stamped the previous evening), and the evidence for that session cited the bare pinned commit. The tree
    *was* the pin at HEAD, but the compiled text was not the pin's text, and **nothing in the build said
    so** -- `ls-timing` is immune because it `git archive`s, and the build is what reaches a board. The rule
    this implements is the cheap half of reproducibility: a build cannot be attributed after the fact unless
    it records what it compiled, so it now records it. Loud, **not refusing** -- refusing would stop a bench
    build for a condition the operator may well intend, and this repo has decided (ADR-0010) that a
    measurement plus a printed verdict beats a signature nobody reads.
    """
    lines = []
    for name, path in (roots if roots is not None else core_source_dirs()).items():
        head, state = _git_state(path)
        pin = CORE_SOURCE_PINS.get(name, "?")
        off_pin = "" if head == pin else "  <<< NOT THE PIN (%s)" % pin
        lines.append("CORE PROVENANCE: %-18s @ %-8s %-16s (pinned %s)%s  %s"
                     % (name, head, state, pin, off_pin, path))
    return lines


def core_source_dirs():
    """The two vendor ``src_v`` directories, or a refusal naming the environment variables.

    The 2023 wrapper ran ``os.system("git clone …")`` at build time: a build that fetches its own inputs
    cannot be reproduced, cannot be audited (the thing it fetched may have moved), and fails at the worst
    possible moment -- offline, at a bench. Both trees are already cloned here (``~/fpga/upstream/``) and
    ``core_usb_host`` is **pinned** at ``81eb9f1`` by ``tools/sync_pin.txt`` and by
    ``tools/core_usb_host-ls_timing.patch``, so this function takes the pin seriously: if a directory
    exists but is not the pinned commit, it still builds (that is the developer's call) but says so.
    """
    roots = {
        "core_usb_host":    os.environ.get("CORE_USB_HOST",
                            os.path.expanduser("~/fpga/upstream/core_usb_host")),
        "core_ulpi_wrapper": os.environ.get("CORE_ULPI_WRAPPER",
                            os.path.expanduser("~/fpga/upstream/core_ulpi_wrapper")),
    }
    missing = [f"{name}: {path}" for name, path in roots.items()
               if not os.path.isdir(os.path.join(path, "src_v"))]
    if missing:
        raise FileNotFoundError(
            "Track A needs the two vendor cores and cannot find them -- refusing to `git clone` at build "
            "time (a build that fetches its own inputs is not reproducible). Missing:\n  " +
            "\n  ".join(missing) +
            "\nSet $CORE_USB_HOST / $CORE_ULPI_WRAPPER. core_usb_host should be at 81eb9f1 "
            "(the commit the low-speed patch set and tools/sync_pin.txt are written against).")
    # Say what is about to be compiled. Deliberately after the missing-tree refusal (that one is fatal and
    # must be the first thing read) and before any build artefact exists, so the line is at the top of the
    # log the spin's evidence will quote.
    for line in vendor_provenance(roots):
        print(line, flush=True)
    return roots


class UsbReset(LiteXModule):
    """``cd_usb``: a reset for the vendor cores that cannot be released before ``C12`` is vouched for.

    ``clk_ok`` and ``phy_reset`` are both ``cd_sys`` levels that already exist: the probe's frequency meter
    on ``cd_usb`` (exactly 60000 edges per gate when the PHY is healthy; 0 when it is in reset or in Low
    Power Mode -- measured 2026-11-03, ``bench/2026-11-03_ulpi_wake_t21/``) and its logical PHY reset.

    Two halves, and each covers the other's blind spot:

    * ``~ok_sys`` acts **asynchronously**, so the cores are pulled into reset the moment the clock meter
      or the PHY reset says so, even though the event that announces it is a ``cd_sys`` event and the
      cores live on a clock that may be about to stop. This is the only kind of reset that can win that
      race (CODING_STANDARDS.md: an async clear from the reset net "is the only reset that works *with the
      clock stopped*").
    * ``~ok_sync`` (a 2-FF of the same level into ``cd_usb``) is what **releases** it: the release can
      only happen on an edge of ``C12``, so it cannot happen while there is no clock to sample it, and
      cannot happen for two cycles after one appears. That is USB3300 §6.1.11 by construction.

    What it does **not** claim: a PHY reset that begins and ends while ``C12`` is stopped is invisible to
    ``cd_usb``, exactly as :class:`~litex_boards.extensions.pano_ulpi.UlpiClear`'s ``relock`` tail documents
    for the Link. Here the meter covers it (a clock that stopped and came back re-passes through "not ok",
    so the cores re-enter reset), and the ``usb_host_state`` CSR is where a bench operator checks that
    rather than trusting this paragraph.
    """
    def __init__(self, clk_ok, phy_reset, domain="usb"):
        # ``ok`` is a cd_sys-level AND of two cd_sys registered levels: combining them combinationally
        # crosses nothing, it just names the condition.
        self.ok = Signal()
        self.comb += self.ok.eq(clk_ok & ~phy_reset)

        ok_sync = Signal()
        self.specials += MultiReg(self.ok, ok_sync, odomain=domain)

        self.reset = Signal()
        self.comb += self.reset.eq(~self.ok | ~ok_sync)


class UsbHostCore(LiteXModule):
    """``cd_sys`` Wishbone slave at ``usb_host_ctrl`` -> CDC -> ``usbh_host`` + ``ulpi_wrapper`` on ``cd_usb``.

    Pads: takes ``data`` and ``stp`` only. ``clk`` is ``cd_usb`` itself (the platform's IBUFG), ``dir`` and
    ``nxt`` are inputs, and ``reset`` is the probe's -- see the module docstring's *Pad ownership*.

    ``clk_ok``/``phy_reset`` come from the probe, which is why the target builds this module **after** it:
    decision 8 (one owner of PHY reset, refclock and clock meter) is enforced by a function signature here
    rather than by a comment, because a comment does not stop a second ``platform.request("ulpi")``.
    """
    def __init__(self, pads, clk_ok, phy_reset,
                 sys_clk_freq = 50e6, usb_clk_freq = 60e6,
                 timeout_ms   = 10):
        # --- the crossed register window -------------------------------------------------------------
        self.cdc = Wishbone2AXILiteCDC(
            sys_clk_freq = sys_clk_freq, timeout_ms = timeout_ms,
            src_domain   = "sys",        dst_domain = "usb")
        self.wishbone = self.cdc.wishbone

        # ``irq`` is what ``SoCCore._finalize_irq()`` looks for on a submodule whose name is in
        # ``self.irq.locs`` (litex/soc/integration/soc.py). The target registers it as
        # ``self.irq.add("usb_host", 0)`` -- location 0 is not free by luck, it is what the device tree in
        # PLAN.md M4.1 will say, and ``M3.4``'s build check is what notices if another master takes it.
        self.irq = Signal()

        # --- reset -----------------------------------------------------------------------------------
        self.usb_reset = UsbReset(clk_ok = clk_ok, phy_reset = phy_reset)

        # A second, deliberate lever: the BIOS test (M3.3) has to be able to hold the cores in reset while
        # it checks the clock, and then release. Held by default, and the question it makes answerable on
        # the bench is the first one worth asking of a new integration: **does the Wishbone window work
        # before the USB core has ever been released?** It does, because the bridge is not in this reset --
        # which is also why ``usb_host_state`` can be read while ``usb_host_core_rst`` is 1.
        self.core_rst = CSRStorage(reset = 1, description =
            "Hold the USB host core and its UTMI/ULPI wrapper in reset (default 1 = held). Not the only "
            "reset: the cores are also held while the PHY's 60 MHz is not vouched for by the probe's clock "
            "meter, so writing 0 releases them only if C12 is healthy (USB3300 §6.1.11). The Wishbone "
            "window and the CDC are NOT in this reset and stay readable while the core is held.")

        # --- what this module is doing, as far as the CPU is concerned --------------------------------
        # One register per fact, in the same style as the ulpi_* map (spec decision 11): a bit-field hunt
        # is what gets wrong at a bench, and an alias can be added later without renaming anything.
        self.state = CSRStatus(fields = [
            CSRField("clk_ok",    size = 1, description = "Probe's cd_usb meter says a healthy 60 MHz."),
            CSRField("phy_reset", size = 1, description = "Probe holds the PHY in reset (logical level)."),
            CSRField("core_ok",   size = 1, description = "clk_ok & ~phy_reset: the condition UsbReset releases on."),
            CSRField("core_rst",  size = 1, description = "The level actually applied to the cores (usb_reset | core_rst.storage)."),
            CSRField("cdc_busy",  size = 1, description = "A Wishbone access is in flight across the bridge."),
            CSRField("cdc_lost",  size = 1, description = "Last crossed access was declared lost (sticky until the next one)."),
        ])
        self.xfer      = CSRStatus(16, description = "Completed crossed accesses (saturating).")
        self.lost      = CSRStatus(8,  description = "Crossed accesses declared lost (saturating).")
        self.aborted   = CSRStatus(8,  description = "Crossed accesses whose destination FSM was withdrawn "
                                                     "mid-AXI (the source gave up, or the far side was cleared). "
                                                     "Grows with every lost access; a nonzero here with a dead "
                                                     "core is expected, a nonzero here with a healthy one is not.")

        # --- the pads, created before the instances that drive them -----------------------------------
        # ``oe = ~dir``: the wrapper has no output-enable port (module docstring, *Pad ownership*), and this
        # is the turnaround-release form CODING_STANDARDS.md permits -- a registered ``dir`` here would keep
        # driving one cycle into the PHY's turnaround, which is a bus fight, not a safety margin.
        #
        # What drives the pads is the wrapper's own ``ulpi_data_q``, whose reset value and every
        # transaction-end value are ``8'b0`` = the ULPI idle byte. That is bug 3's lesson
        # (ADR-0008: "the Link must continuously drive 00h while it owns the bus") arriving already paid
        # for: the 2023 probe build failed this by *holding the PHY's last byte*, and the wrapper cannot,
        # because its data register has an explicit idle value at every end-of-packet.
        self.tri = TSTriple(8)
        self.specials += self.tri.get_tristate(pads.data)

        # The three nets the two instances drive and the pad connections read. Declared here, before the
        # `Instance(...)` calls, because migen evaluates those keyword arguments **now**: the first version of
        # this module referenced `self.bus_o` inside the wrapper's `Instance` and migen answered
        # `AttributeError: 'UsbHostCore' object has no attribute 'bus_o'` -- loud, at least, and a reminder
        # that an `Instance` is not a declaration of nets, only a use of them.
        #
        # Named with a `usbh_` prefix and not `ulpi_`: the probe already owns `ulpi_bus_oe`/`ulpi_bus_o`, and
        # LiteScope's group de-duplication is by *name*. Two signals that both mean "our driver is on the
        # pad" in two modules must not arrive in the CSV as one row.
        self.stp    = Signal(name = "usbh_stp")
        self.bus_oe = Signal(name = "usbh_bus_oe")
        self.bus_o  = Signal(8,  name = "usbh_bus_o")

        # --- UTMI, point to point --------------------------------------------------------------------
        # A Record and not a stream: the two vendor modules share these wires combinationally, with no
        # handshake to speak of on the Rx side (rxvalid is a one-cycle pulse), and a stream.Endpoint would
        # imply a valid/ready contract the wrapper does not implement on that half.
        utmi = Record([
            ("data_in",    8), ("txready", 1), ("rxvalid", 1), ("rxactive", 1), ("rxerror", 1),
            ("linestate",  2),
            ("data_out",   8), ("txvalid", 1), ("op_mode", 2), ("xcvrselect", 2),
            ("termselect", 1), ("dppulldown", 1), ("dmpulldown", 1),
        ])
        self.utmi = utmi

        core_rst = Signal()
        self.comb += core_rst.eq(self.usb_reset.reset | self.core_rst.storage)

        # --- the host core ---------------------------------------------------------------------------
        self.specials += Instance("usbh_host",
            # **An int, not "60e6".** See the module docstring, item 1: the 2023 wrapper passed a string,
            # migen emitted it verbatim, and the core's `CLKS_PER_BIT` became 75 instead of 5. Nothing
            # would have failed to build; nothing would have answered on the wire.
            p_USB_CLK_FREQ = int(usb_clk_freq),

            i_clk_i = ClockSignal("usb"),
            i_rst_i = core_rst,

            i_cfg_awvalid_i = self.cdc.cfg_awvalid,
            i_cfg_awaddr_i  = self.cdc.cfg_awaddr,
            i_cfg_wvalid_i  = self.cdc.cfg_wvalid,
            i_cfg_wdata_i   = self.cdc.cfg_wdata,
            i_cfg_wstrb_i   = self.cdc.cfg_wstrb,
            i_cfg_bready_i  = self.cdc.cfg_bready,
            i_cfg_arvalid_i = self.cdc.cfg_arvalid,
            i_cfg_araddr_i  = self.cdc.cfg_araddr,
            i_cfg_rready_i  = self.cdc.cfg_rready,

            o_cfg_awready_o = self.cdc.cfg_awready,
            o_cfg_wready_o  = self.cdc.cfg_wready,
            o_cfg_bvalid_o  = self.cdc.cfg_bvalid,
            o_cfg_bresp_o   = self.cdc.cfg_bresp,
            o_cfg_arready_o = self.cdc.cfg_arready,
            o_cfg_rvalid_o  = self.cdc.cfg_rvalid,
            o_cfg_rdata_o   = self.cdc.cfg_rdata,
            o_cfg_rresp_o   = self.cdc.cfg_rresp,

            o_intr_o = self.irq,

            i_utmi_data_in_i    = utmi.data_in,
            i_utmi_txready_i    = utmi.txready,
            i_utmi_rxvalid_i    = utmi.rxvalid,
            i_utmi_rxactive_i   = utmi.rxactive,
            i_utmi_rxerror_i    = utmi.rxerror,
            i_utmi_linestate_i  = utmi.linestate,
            o_utmi_data_out_o   = utmi.data_out,
            o_utmi_txvalid_o    = utmi.txvalid,
            o_utmi_op_mode_o    = utmi.op_mode,
            o_utmi_xcvrselect_o = utmi.xcvrselect,
            o_utmi_termselect_o = utmi.termselect,
            o_utmi_dppulldown_o = utmi.dppulldown,
            o_utmi_dmpulldown_o = utmi.dmpulldown,
        )

        # --- the UTMI -> ULPI wrapper ----------------------------------------------------------------
        self.specials += Instance("ulpi_wrapper",
            i_ulpi_clk60_i     = ClockSignal("usb"),
            i_ulpi_rst_i       = core_rst,
            i_ulpi_data_out_i  = self.tri.i,
            i_ulpi_dir_i       = pads.dir,
            i_ulpi_nxt_i       = pads.nxt,
            o_ulpi_data_in_o   = self.bus_o,
            o_ulpi_stp_o       = self.stp,

            i_utmi_data_out_i   = utmi.data_out,
            i_utmi_txvalid_i    = utmi.txvalid,
            i_utmi_op_mode_i    = utmi.op_mode,
            i_utmi_xcvrselect_i = utmi.xcvrselect,
            i_utmi_termselect_i = utmi.termselect,
            i_utmi_dppulldown_i = utmi.dppulldown,
            i_utmi_dmpulldown_i = utmi.dmpulldown,
            o_utmi_data_in_o    = utmi.data_in,
            o_utmi_txready_o    = utmi.txready,
            o_utmi_rxvalid_o    = utmi.rxvalid,
            o_utmi_rxactive_o   = utmi.rxactive,
            o_utmi_rxerror_o    = utmi.rxerror,
            o_utmi_linestate_o  = utmi.linestate,
        )

        # --- the pad connections ----------------------------------------------------------------------
        self.comb += [
            self.tri.o.eq(self.bus_o),
            self.tri.oe.eq(~pads.dir),
            pads.stp.eq(self.stp),
            self.bus_oe.eq(~pads.dir),
        ]

        # --- diagnostics ------------------------------------------------------------------------------
        self.comb += [
            self.cdc.usb_ok.eq(self.usb_reset.ok),
            self.state.fields.clk_ok.eq(clk_ok),
            self.state.fields.phy_reset.eq(phy_reset),
            self.state.fields.core_ok.eq(self.usb_reset.ok),
            self.state.fields.core_rst.eq(core_rst),
            self.state.fields.cdc_busy.eq(self.cdc.bridge.busy),
            self.state.fields.cdc_lost.eq(self.cdc.err_lost),
            self.xfer.status.eq(self.cdc.xfer),
            self.lost.status.eq(self.cdc.lost_count),
            self.aborted.status.eq(self.cdc.aborted),
        ]
