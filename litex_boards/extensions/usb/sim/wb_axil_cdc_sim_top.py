#!/usr/bin/env python3
#
# Generate the synthesizable Verilog of the Wishbone -> AXI4-Lite CDC for **simulation only**.
#
# Same reasoning as ``ulpi_link_sim_top.py`` next to this file: the target needs a platform, a CRG, LiteDRAM
# and a LiteEth MAC to elaborate, so a testbench that depends on the target cannot run in a check suite.
# ``Wishbone2AXILiteCDC`` is the block whose handshake M3.2 is about, so the top here is that block, its two
# clock domains, and nothing else.
#
# Every port is a **plain Signal created in this file** and then connected to the module's own field. That
# is not decoration: litex.gen names a generated net after the module that *owns* the signal, so a port
# taken straight from ``cdc.wishbone`` or ``cdc.bridge`` arrives in the netlist with a prefix nobody wrote
# down, and a testbench that references the name from the source rather than the name from the netlist
# drives a wire that goes nowhere -- which in iverilog is not an error, it is a row that passes for the
# wrong reason. (The same lesson is in pano_ulpi_scope.py's "the CSV is the contract" section.)
#
# Usage: wb_axil_cdc_sim_top.py  (import ``generate``; nothing to run)

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from migen import ClockDomain, Module, Signal

from litex.build.generic_platform import GenericPlatform
from litex.gen import LiteXModule

from litex_boards.extensions.pano_wb_axil import Wishbone2AXILiteCDC


class WbAxilCdcSimTop(LiteXModule):
    """The bridge with both sides brought out, plus the three counters that witness its verdicts."""
    def __init__(self, sys_clk_freq = 50e6, timeout_cycles = 200, adr_bits = 8):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_usb = ClockDomain(reset_less = True)   # C12 can stop; see pano_wb_axil

        # The watchdog is a **cycle count**, and the board's default is 500 000 (10 ms at 50 MHz); a row that
        # waited for the real bound would run 500 000 sys edges for one verdict. So the sim top asks for a
        # short one and the TB says which number it is waiting for. `timeout_cycles=0` is the other setting
        # under test, and it is a row of its own: ReqAckCDC treats 0 as "disabled", so a bridge that cannot
        # be configured to never time out is a bridge whose bound nobody can switch off for a long
        # transfer-heavy debug session.
        cdc = Wishbone2AXILiteCDC(sys_clk_freq = sys_clk_freq, timeout_cycles = timeout_cycles)
        self.cdc           = cdc
        self.timeout_cycles = Signal(32, name = "timeout_cycles")
        self.comb += self.timeout_cycles.eq(cdc.timeout_cycles)

        # --- Wishbone side (cd_sys) -------------------------------------------------------------------
        self.wb_cyc   = Signal(name = "wb_cyc")
        self.wb_stb   = Signal(name = "wb_stb")
        self.wb_we    = Signal(name = "wb_we")
        self.wb_adr   = Signal(adr_bits, name = "wb_adr")
        self.wb_dat_w = Signal(32, name = "wb_dat_w")
        self.wb_sel   = Signal(4,  name = "wb_sel")
        self.wb_ack   = Signal(name = "wb_ack")
        self.wb_err   = Signal(name = "wb_err")
        self.wb_dat_r = Signal(32, name = "wb_dat_r")

        # --- AXI4-Lite side (cd_usb), the core's own port names ---------------------------------------
        self.cfg_awvalid = Signal(name = "cfg_awvalid")
        self.cfg_awaddr  = Signal(32, name = "cfg_awaddr")
        self.cfg_wvalid  = Signal(name = "cfg_wvalid")
        self.cfg_wdata   = Signal(32, name = "cfg_wdata")
        self.cfg_wstrb   = Signal(4,  name = "cfg_wstrb")
        self.cfg_bready  = Signal(name = "cfg_bready")
        self.cfg_arvalid = Signal(name = "cfg_arvalid")
        self.cfg_araddr  = Signal(32, name = "cfg_araddr")
        self.cfg_rready  = Signal(name = "cfg_rready")
        self.cfg_awready = Signal(name = "cfg_awready")
        self.cfg_wready  = Signal(name = "cfg_wready")
        self.cfg_bvalid  = Signal(name = "cfg_bvalid")
        self.cfg_bresp   = Signal(2,  name = "cfg_bresp")
        self.cfg_arready = Signal(name = "cfg_arready")
        self.cfg_rvalid  = Signal(name = "cfg_rvalid")
        self.cfg_rdata   = Signal(32, name = "cfg_rdata")
        self.cfg_rresp   = Signal(2,  name = "cfg_rresp")

        # --- the far-side witness ----------------------------------------------------------------------
        # `usb_ok` is a cd_sys level in the real design (`UsbReset.ok`: the probe's clock meter AND-noted
        # with its PHY reset). It is a port here because the TB has to be able to say "the far side is gone"
        # independently of the bridge's own opinion, exactly as `clk_ok` is a port of the Link's sim top.
        self.usb_ok     = Signal(name = "usb_ok")

        # --- witnesses ---------------------------------------------------------------------------------
        self.xfer       = Signal(16, name = "xfer")
        self.lost       = Signal(8,  name = "lost_count")
        self.aborted    = Signal(8,  name = "aborted")
        self.err_lost   = Signal(name = "err_lost")
        self.busy       = Signal(name = "bridge_busy")

        # # #

        wb = cdc.wishbone
        self.comb += [
            wb.cyc.eq(self.wb_cyc),
            wb.stb.eq(self.wb_stb),
            wb.we.eq(self.wb_we),
            wb.adr.eq(self.wb_adr),
            wb.dat_w.eq(self.wb_dat_w),
            wb.sel.eq(self.wb_sel),
            self.wb_ack.eq(wb.ack),
            self.wb_err.eq(wb.err),
            self.wb_dat_r.eq(wb.dat_r),
        ]
        cfg_pairs = ["awvalid", "awaddr", "wvalid", "wdata", "wstrb", "bready", "arvalid", "araddr",
                     "rready", "awready", "wready", "bvalid", "bresp", "arready", "rvalid", "rdata",
                     "rresp"]
        for name in cfg_pairs:
            top  = getattr(self,  "cfg_" + name)
            cdcx = getattr(cdc,   "cfg_" + name)
            # Direction follows the bridge: what the bridge DRIVES becomes an output port of the generated
            # module (the TB must observe it), and what the core drives becomes an input (the TB must be able
            # to say it). Written as a rule and not seventeen lines so that an eighth `cfg_` signal cannot be
            # added to the bridge and quietly left out of the sim top. The first version of this list had the
            # two branches swapped; iverilog is content to simulate a master whose outputs are inputs, so the
            # direction of every cfg_ port is asserted by the runner (`ports`), not assumed here.
            if name in ("awvalid", "awaddr", "wvalid", "wdata", "wstrb", "bready",
                        "arvalid", "araddr", "rready"):
                self.comb += top.eq(cdcx)
            else:
                self.comb += cdcx.eq(top)
        self.comb += [
            cdc.usb_ok.eq(self.usb_ok),
            self.xfer.eq(cdc.xfer),
            self.lost.eq(cdc.lost_count),
            self.aborted.eq(cdc.aborted),
            self.err_lost.eq(cdc.err_lost),
            self.busy.eq(cdc.bridge.busy),
        ]

    @property
    def ios(self):
        s = self
        return {
            s.cd_sys.clk, s.cd_usb.clk, s.cd_sys.rst,
            s.wb_cyc, s.wb_stb, s.wb_we, s.wb_adr, s.wb_dat_w, s.wb_sel,
            s.wb_ack, s.wb_err, s.wb_dat_r,
            s.cfg_awvalid, s.cfg_awaddr, s.cfg_wvalid, s.cfg_wdata, s.cfg_wstrb, s.cfg_bready,
            s.cfg_arvalid, s.cfg_araddr, s.cfg_rready,
            s.cfg_awready, s.cfg_wready, s.cfg_bvalid, s.cfg_bresp,
            s.cfg_arready, s.cfg_rvalid, s.cfg_rdata, s.cfg_rresp,
            s.usb_ok, s.xfer, s.lost, s.aborted, s.err_lost, s.busy, s.timeout_cycles,
        }


if __name__ == "__main__" or True:
    def generate(path = "wb_axil_cdc.v", top = "wb_axil_cdc", sys_clk_freq = 50e6, **kw):
        dut = WbAxilCdcSimTop(sys_clk_freq = sys_clk_freq, **kw)
        v = GenericPlatform("xilinx", "xc6slx100", []).get_verilog(dut, ios = dut.ios, name = top)
        with open(path, "w") as f:
            f.write(str(v))
        return path
