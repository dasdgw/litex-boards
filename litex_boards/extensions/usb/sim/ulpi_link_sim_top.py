#!/usr/bin/env python3
#
# Generate the synthesizable Verilog of the ULPI Link for **simulation only**.
#
# Why this file exists instead of "just run the target": the target needs a platform, a CRG, LiteDRAM and
# a LiteEth MAC to elaborate, and a testbench that depends on all of that cannot run in a check suite (it
# would take minutes and fail on a machine with no LiteDRAM checkout). `UlpiLink` is the block whose
# choreography we are actually testing, so the top here is that block plus its two clock domains and
# nothing else -- which is also the honest description of what the sim proves: the Link, not the SoC.
#
# The clock ports are passed as `ios` explicitly. migen will not export a clock it does not know is a
# top-level input, and a testbench that drives a signal the generated module declared as an internal reg
# does not fail -- it just sits at 0 and every timeout vector "passes" for the wrong reason.
#
# Usage: ulpi_link_sim_top.py [-o out.v] [--top ulpi_link]

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from migen import ClockDomain, ClockSignal, ResetSignal
from migen.genlib.record import Record

from litex.build.generic_platform import GenericPlatform
from litex.gen import LiteXModule

from litex_boards.extensions.pano_ulpi import UlpiLink


def ulpi_pad_record(name = "ulpi"):
    """The shape `platform.request("ulpi")` gives the master, without a platform.

    Kept as its own function because the *directions* are the contract: `data` is a tri-state record and
    `dir`/`nxt` are plain inputs. If the platform resource for `ulpi` ever changes shape, the simulation
    top and the target must move together, and this is the one place to look.
    """
    # `stp` is a tri-state record too as of the wake ticket, and the reason is the same one that made `data`
    # one: the pad is now an `IOBUF`, so its level has to come back in (`probe_stp_pad`, spec decision D).
    # `dir`/`nxt` stay plain inputs. The testbench is the only other place this shape appears
    # (`ulpi_link_tb.v` wires `ulpi_stp_o/_oe` into a resolved net the PHY model reads).
    return Record([
        ("data", [("o", 8), ("oe", 8), ("i", 8)]),
        ("stp",  [("o", 1), ("oe", 1), ("i", 1)]),
        ("dir",  1),
        ("nxt",  1),
    ], name = name)


class UlpiLinkSimTop(LiteXModule):
    """`UlpiLink` with every host-side signal promoted to a port, and the pads brought out plainly.

    `clk_ok` is a port rather than being derived inside: on the board it comes from spin 1's clock meter,
    and the testbench has to be able to say "the reference is there but the PHY has stopped" independently
    of the master's own opinion, which is the entire content of the clock-absent vector.
    """
    def __init__(self, sys_clk_freq = 50e6, **kw):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_usb = ClockDomain(reset_less = True)   # ULPI has no reset pin; see pano_ulpi
        self.pads = ulpi_pad_record()
        self.link = UlpiLink(self.pads, sys_clk_freq = sys_clk_freq, **kw)

    @property
    def ios(self):
        # A set, because that is what LiteX's convert() unions. Plain Signals only: a ClockSignal proxy is
        # unhashable *and* has no `name_override`, so the clocks come in as the domain's own Signals.
        l, p = self.link, self.pads
        return {
            # The domain's own clock/reset Signals, not ClockSignal(): LiteX's convert() reads
            # `name_override` off every entry of `ios`, and the proxy objects migen hands back for a clock
            # do not have one. Same wires, and they carry the names the testbench must use -- `sys_clk`,
            # `usb_clk`, `sys_rst`.
            self.cd_sys.clk, self.cd_usb.clk, self.cd_sys.rst,
            l.h_req, l.h_rw, l.h_addr, l.h_wdata,
            l.h_ack, l.h_ack_data, l.h_busy,
            l.abort_req, l.bus_grant, l.phy_reset, l.clk_ok, l.ulpi_clear.clear,
            # `wake` and `stp_drive` are host-side LEVELS on purpose: the testbench holds them across a clock
            # stop, which is the whole content of the wake rows. They are not pulsed and they are not
            # synchronised -- `cd_usb` is the clock the row stops.
            l.wake, l.stp_drive, l.stp_pad, l.stp_driving,
            l.done, l.err_dir_abort, l.err_nxt_timeout, l.err_no_clock, l.err_reset,
            l.dir_now, l.oe_now, l.state, l.last_bus_waited, l.last_retry,
            l.cnt_access, l.cnt_dir_abort, l.cnt_timeout, l.cnt_reset, l.cnt_no_clock,
            p.data.o, p.data.oe, p.data.i, p.stp.o, p.stp.oe, p.stp.i, p.dir, p.nxt,
        }


if __name__ == "__main__" or True:
    def generate(path = "ulpi_link_sim.v", top = "ulpi_link", sys_clk_freq = 50e6, **kw):
        dut = UlpiLinkSimTop(sys_clk_freq = sys_clk_freq, **kw)
        v = GenericPlatform("xilinx", "xc6slx100", []).get_verilog(dut, ios = dut.ios, name = top)
        with open(path, "w") as f:
            f.write(str(v))
        return path
