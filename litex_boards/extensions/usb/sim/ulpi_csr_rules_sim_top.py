#!/usr/bin/env python3
#
# Generate the synthesizable Verilog of the ``ulpi_*`` **rule logic** for simulation only.
#
# Sister file to ``ulpi_link_sim_top.py``, and for the same reason: the target needs a platform, a CRG,
# LiteDRAM and a LiteEth MAC to elaborate, so a testbench that depends on it cannot run in a check suite.
# The difference is what each top proves. ``ulpi_link_sim_top`` stops at ``UlpiLink`` precisely *because* a
# Link simulation must not exercise a LiteX CSR model. That was always true, and it is also why the rules in
# front of the Link -- "``go`` is an edge", "a ``go`` against a busy link is dropped and witnessed",
# "``rdata`` moves only on the ack" -- had never been executed by anything before 2026-11-01: they lived in
# ``UlpiCsr``, which nothing simulated.
#
# So the seam was moved, not faked. :class:`UlpiCsrRules` is a plain-Signal ``Module`` holding exactly those
# rules; ``UlpiCsr`` instantiates it and keeps names, descriptions and ``.storage``/``.status`` plumbing.
# This file is the simulation handle on that module, and ``tools/test_ulpi_csr_rules.py`` (project repo) is
# the drift guard that refuses a build where ``UlpiCsr`` no longer connects these ports to the Link signals
# they are supposed to reach.
#
# The clock ports are passed as ``ios`` explicitly, for the reason spelled out in ``ulpi_link_sim_top.py``:
# migen will not export a clock it does not know is a top-level input, and a testbench driving a signal the
# generated module declared as an internal register does not fail -- it just sits at 0.
#
# Usage: generate("ulpi_csr_rules.v")

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from migen import ClockDomain, Signal

from litex.build.generic_platform import GenericPlatform
from litex.gen import LiteXModule

from litex_boards.extensions.pano_ulpi_csr import UlpiCsrRules   # ACK_WIDTH: the ack payload's width, pinned by the drift guard


class UlpiCsrRulesSimTop(LiteXModule):
    """``UlpiCsrRules`` with ``cd_sys`` and every port promoted to a top-level input or output.

    Nothing else. No CSR bank, no ``CSRStorage``, no address decode: the point of this suite is the rules,
    and every extra block in the DUT is a block whose bugs the suite would report as ours.

    One thing the simulation must model by hand instead of by instance: ``CSRStorage``'s own reset value.
    ``ulpi_stp_drive`` is ``CSRStorage(reset = 1)``, so the testbench boots its ``stp_drive`` input at 1 and
    ``stp_drive_applied`` must agree with it at reset -- a top that left the request at 0 would be simulating a
    register map that does not exist, and would make the first write of 0 a non-edge.
    """
    def __init__(self):
        self.clock_domains.cd_sys = ClockDomain()
        self.rules = UlpiCsrRules()

    @property
    def ios(self):
        r = self.rules
        # A set, because that is what LiteX's convert() unions, and the domain's own Signals rather than
        # ClockSignal()/ResetSignal() proxies for the reason in ulpi_link_sim_top.py.
        return {
            self.cd_sys.clk, self.cd_sys.rst,
            # inputs
            r.go, r.abort_in, r.cmd, r.addr, r.wdata,
            r.wake_req, r.stp_drive_req,
            r.h_busy, r.h_ack, r.h_ack_data,
            # outputs
            r.h_req, r.h_rw, r.h_addr, r.h_wdata, r.abort_req,
            r.rdata, r.refused,
            # The wake gate's four, added 2026-11-02 because a testbench cannot audit a net it cannot name.
            # Without these the generated module kept `wake_req` / `stp_drive_req` / `pad_refused` as INTERNAL
            # registers -- the wake logic still synthesized, the vectors still passed at the Link level, and the
            # rules module's own new behaviour was unobservable, which is exactly the gap ticket 01's exit note
            # called out by name. `*_applied` is what the pad receives; `*_req` is what the CSR holds; the
            # distance between them is the design.
            r.wake_applied, r.stp_drive_applied, r.pad_refused,
        }


def generate(path = "ulpi_csr_rules.v", top = "ulpi_csr_rules"):
    dut = UlpiCsrRulesSimTop()
    v = GenericPlatform("xilinx", "xc6slx100", []).get_verilog(dut, ios = dut.ios, name = top)
    with open(path, "w") as f:
        f.write(str(v))
    return path


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out", default = "ulpi_csr_rules.v")
    ap.add_argument("--top", default = "ulpi_csr_rules")
    a = ap.parse_args()
    print(generate(a.out, a.top))
