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

    Nothing else. No CSR bank, no ``CSRStorage``, no address decode: the point of this suite is the three
    rules, and every extra block in the DUT is a block whose bugs the suite would report as ours.
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
            r.h_busy, r.h_ack, r.h_ack_data,
            # outputs
            r.h_req, r.h_rw, r.h_addr, r.h_wdata, r.abort_req,
            r.rdata, r.refused,
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
