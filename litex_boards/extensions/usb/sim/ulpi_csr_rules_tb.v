`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// ulpi_csr_rules_tb.v -- the ``ulpi_*`` register map's RULES, executed.
//
// Until 2026-11-01 nothing had ever run these three behaviours:
//
//   * ``ulpi_go`` starts an access on the 0->1 edge of the stored bit, not on a level and not on a write;
//   * a ``go`` that arrives while the link is busy is dropped AND witnessed (`ulpi_refused`), and the drop
//     is permanent -- it is not queued and it does not fire when busy releases;
//   * ``ulpi_rdata`` moves only on the ack, so ``done``=0 means "this byte is the previous access's".
//
// They were proven by reading them. This repo's own history says what that is worth: a `MultiReg(i,
// Signal(2))` that was a two-bit net and not a synchroniser, and a `dst_req` whose only release was
// `dst_ack`, both readable and both wrong (`docs/STATUS.md` 2026-10-31). Each rule above has a bench
// consequence -- a retried write starting a second ~20 ms transaction, a dropped `go` whose status word
// reads as a fresh success, a stale byte reported as a read `24 04 04 00` -- so each gets rows here.
//
// Expectations live in this file, in the style of `ulpi_phy_selfcheck.v` (which guards the PHY model for the
// same reason: a suite cannot audit its own oracle). What is NOT here is any opinion about ULPI wire
// behaviour -- that is `ulpi_link_tb.v` + `ulpi_phy_model.v`, 35 rows. This DUT has no pads.
//
//   iverilog -g2012 -o rules.out ulpi_csr_rules_tb.v ulpi_csr_rules.v
//   vvp rules.out
//
// Two conventions worth knowing before editing:
//
//  * **The witness is a registered consumer, not a `#1` probe.** `h_req` is combinational and one cycle
//    wide; the `ReqAckCDC` bridge that eats it samples it with a clock. So `req_seen` counts `h_req` at
//    `posedge sys_clk`, exactly as the bridge does. A probe that sampled a delta after the edge would read
//    0 always and every row would "pass".
//  * **Every stimulus change happens 1 ns AFTER an edge, and every read happens 1 ns after an edge.**
//    `ulpi_go.storage` is a register the CSR bank writes, so the level the rules see changes on an edge --
//    but *when* the testbench changes it decides who wins. Driving it in the same timestep as the
//    `posedge` loses: `prev_go <= go` then samples the NEW level, the 0->1 edge is never observed, and
//    `h_req` never pulses. That is not a hypothetical -- this file's first version did exactly that and
//    reported 12 failures for a DUT that is correct (mini2.vcd: `go` and `prev_go` both change at 50 ns,
//    `h_req` stays 0 for the whole run). A `#1` after the edge is the difference between modelling a CSR
//    write and modelling a race, and it also puts every *read* away from the edge where the DUT's own
//    registers are updating. `wait_cycles` ends at +1 ns for that reason; do not "simplify" it to a bare
//    `repeat (n) @(posedge clk)`.
// -----------------------------------------------------------------------------

module ulpi_csr_rules_tb;

    reg sys_clk = 1'b0;
    reg sys_rst = 1'b1;
    always #10 sys_clk = ~sys_clk;              // 50 MHz, the board's cd_sys

    // --- host side (what a CSRStorage holds) -----------------------------------------------------------
    reg         go = 1'b0, abort_in = 1'b0, cmd = 1'b0;
    reg  [ 5:0] addr = 6'd0;
    reg  [ 7:0] wdata = 8'd0;
    // --- Link side (what the bridge shows the map) -------------------------------------------------------
    reg         h_busy = 1'b0, h_ack = 1'b0;
    reg  [29:0] h_ack_data = 30'd0;

    wire        h_req, h_rw, abort_req, refused;
    wire [ 5:0] h_addr;
    wire [ 7:0] h_wdata, rdata;

    ulpi_csr_rules dut (
        .sys_clk(sys_clk), .sys_rst(sys_rst),
        .go(go), .abort_in(abort_in), .cmd(cmd), .addr(addr), .wdata(wdata),
        .h_busy(h_busy), .h_ack(h_ack), .h_ack_data(h_ack_data),
        .h_req(h_req), .h_rw(h_rw), .h_addr(h_addr), .h_wdata(h_wdata),
        .abort_req(abort_req), .rdata(rdata), .refused(refused)
    );

    // The bridge's point of view: an access "started" when a clock edge found h_req high.
    integer req_seen = 0, abort_seen = 0;
    always @(posedge sys_clk) begin
        if (h_req)     req_seen   = req_seen   + 1;
        if (abort_req) abort_seen = abort_seen + 1;
    end

    integer nchecks, nfail;
    initial begin nchecks = 0; nfail = 0; end

    // `===`, never `if (!ok)`. The first version of this task counted an UNKNOWN as a pass: deleting the
    // only writer of `refused` makes migen re-declare it as an *input* (a read-only net has no driver), the
    // TB's wire floats to x, `!x` is x, the branch is not taken, and `busy-go-counted` reported PASS on a
    // register nobody drives. An x verdict is not a pass and it is not a fail either -- it means the harness
    // lost sight of the DUT, so it gets its own word and its own count, and the runner treats it as red.
    task chk(input [8*26-1:0] id, input ok, input [8*96-1:0] detail);
        begin
            nchecks = nchecks + 1;
            if (ok !== 1'b1) nfail = nfail + 1;
            $display("CSRCHK %-26s %s %0s", id,
                     ok === 1'b1 ? "PASS" : (ok === 1'b0 ? "FAIL" : "AMB"),
                     ok === 1'bx || ok === 1'bz ? "verdict is x/z: the DUT port moved or lost its driver" : detail);
        end
    endtask

    // Ends 1 ns AFTER the n-th edge: see the timing rule in the header. Everything that reads the DUT does
    // it from here, and everything that drives the DUT drives from here too.
    task wait_cycles(input integer n);
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) @(posedge sys_clk);
            #1;
        end
    endtask

    // A CSR write lands on an edge, one nanosecond later, so `prev_go` cannot sample the new level in the
    // same timestep it appeared (see the header). The pulse is then a full cycle wide, as on the board.
    task csr_go(input v);
        begin
            @(posedge sys_clk);
            #1;
            go = v;
        end
    endtask

    task csr_abort(input v);
        begin
            @(posedge sys_clk);
            #1;
            abort_in = v;
        end
    endtask

    task apply_reset;
        begin
            @(posedge sys_clk); #1;
            sys_rst = 1'b1; go = 1'b0; abort_in = 1'b0; cmd = 1'b0; addr = 6'd0; wdata = 8'd0;
            h_busy = 1'b0; h_ack = 1'b0; h_ack_data = 30'd0;
            wait_cycles(4);
            sys_rst = 1'b0;
            wait_cycles(2);
            req_seen = 0; abort_seen = 0;
        end
    endtask

    // 30-bit ack payload in ACK_FIELDS order: dir_abort, nxt_timeout, rdata, bus_waited, retry.
    // Every field gets a different pattern on purpose: a slice that is off by two bits still returns *a*
    // byte, and only distinct neighbours make that visible.
    function [29:0] ack_word(input [7:0] b, input [15:0] waited, input [3:0] retry,
                             input dir_abort, input nxt_timeout);
        begin
            ack_word = {retry, waited, b, nxt_timeout, dir_abort};
        end
    endfunction

    integer b_walk;
    reg     walk_ok;

    // ---------------------------------------------------------------------------------
    initial begin : main

        // ---- 1. quiet at boot ----------------------------------------------------------------
        apply_reset;
        wait_cycles(10);
        chk("idle-quiet", (req_seen == 0) && (abort_seen == 0) && (refused == 1'b0),
            "a CSR map at rest must not ask the Link for anything");

        // ---- 2/3/4. `go` is an edge, not a level, and not a write ---------------------------------
        csr_go(1'b1);
        wait_cycles(2);
        chk("go-single-access", req_seen == 1, "one 0->1 must start exactly one access");
        csr_go(1'b1);                                    // the retried write a ~20 ms link produces
        wait_cycles(3);
        chk("go-rewrite-no-access", req_seen == 1,
            "writing 1 again must not start a second transaction (that is why it is not wr_stb)");
        wait_cycles(8);
        chk("go-held-no-more", req_seen == 1, "holding go high is a level, not a stream of accesses");

        // ---- 5. the documented way to force a second access ---------------------------------------
        csr_go(1'b0);
        wait_cycles(2);
        csr_go(1'b1);
        wait_cycles(2);
        chk("go-0-then-1-starts", req_seen == 2, "0 then 1 is how the harness forces a second access");
        chk("go-when-idle-not-refused", refused == 1'b0,
            "two clean accesses must leave refused at 0, or A0's 'refused=0 means a quiet board' is noise");

        // ---- 6/7. a `go` against a busy link: dropped, and witnessed ------------------------------
        csr_go(1'b0); wait_cycles(2);
        @(posedge sys_clk); #1; h_busy = 1'b1;      // busy BEFORE the go, or the row proves nothing
        csr_go(1'b1);
        wait_cycles(4);
        chk("busy-drops-go", req_seen == 2,
            "busy must gate h_req: the bridge is single-outstanding by decision 10");
        chk("busy-go-counted", refused == 1'b1,
            "a dropped go that is not counted leaves the PREVIOUS access's status reading as a pass");

        // ---- 8/9. the drop is permanent, and the sticky bit stays set -------------------------------
        @(posedge sys_clk); #1; h_busy = 1'b0;
        wait_cycles(12);
        chk("no-replay-after-busy", req_seen == 2,
            "a dropped go is not queued: the harness writes 0 then 1, and the TSV has to show it did");
        chk("refused-sticky", refused == 1'b1,
            "sticky by design: it answers 'dropped since boot' (row A0), never 'how many'");

        // ---- 10. reset really clears it (A0's premise, not luck) ------------------------------------
        apply_reset;
        wait_cycles(4);
        chk("reset-clears-refused", refused == 1'b0,
            "row A0 reads ulpi_refused BEFORE any go and requires 0 -- that needs a reset, not luck");

        // ---- 11/12/13. pass-through widths ----------------------------------------------------------
        @(posedge sys_clk); #1;
        cmd = 1'b1; addr = 6'h3F; wdata = 8'hFF;
        wait_cycles(2);
        chk("passthrough-all-ones", (h_rw == 1'b1) && (h_addr == 6'h3F) && (h_wdata == 8'hFF),
            "cmd=1 addr=3F wdata=FF must arrive intact -- the direction bit and both fields");
        walk_ok = 1'b1;
        for (b_walk = 0; b_walk < 6; b_walk = b_walk + 1) begin
            @(posedge sys_clk); #1; addr = 6'd1 << b_walk;
            wait_cycles(2);
            if (h_addr !== ((6'd1 << b_walk))) walk_ok = 1'b0;
        end
        chk("passthrough-addr-walking1", walk_ok,
            "every addr bit reaches the Link: the mini version of A2b's pin-map proof");
        @(posedge sys_clk); #1; cmd = 1'b0;
        wait_cycles(2);
        chk("cmd-only-drives-rw", (h_rw == 1'b0) && (h_addr == 6'h20) && (h_wdata == 8'hFF),
            "a read must not be a write, and turning it off must not disturb addr/wdata");

        // ---- 14/15. abort is an edge too -------------------------------------------------------------
        abort_seen = 0;
        csr_abort(1'b1);
        wait_cycles(2);
        chk("abort-single-cycle", abort_seen == 1, "ulpi_abort = 1 pulses stp once");
        csr_abort(1'b1);
        wait_cycles(8);
        chk("abort-held-no-more", abort_seen == 1, "a held abort must not spam stp at the PHY");

        // ---- 16/17/18/19. rdata moves on the ack and nowhere else -------------------------------------
        apply_reset;
        @(posedge sys_clk); #1; h_ack_data = ack_word(8'h5A, 16'hBEEF, 4'h3, 1'b1, 1'b0);
        wait_cycles(3);
        chk("rdata-holds-without-ack", rdata == 8'h00,
            "a result on the wire is not a result the host may read until the ack");
        @(posedge sys_clk); #1; h_ack = 1'b1;
        @(posedge sys_clk); #1; h_ack = 1'b0;
        wait_cycles(2);
        // One row, two claims: the ack is the only thing that may move `rdata`, AND the byte it moves in is
        // `ack_data[9:2]`. The neighbour fields are non-zero (dir_abort=1, waited=BEEF, retry=3) so a slice
        // that starts at bit 0 -- `h_ack_data[7:0]`, which would read 0x68 here, since ack_unpack puts
        // dir_abort and nxt_timeout BELOW the byte -- cannot pass by coincidence either.
        chk("rdata-updates-on-ack", rdata == 8'h5A,
            "the ack moves rdata, and moves it the ack_unpack byte (5Ah, not 68h/2Dh from a wrong slice)");
        @(posedge sys_clk); #1; h_ack_data = ack_word(8'hA5, 16'h0001, 4'h0, 1'b0, 1'b0);
        wait_cycles(4);
        chk("rdata-holds-after-ack", rdata == 8'h5A,
            "the stale-byte contract: after the ack, the NEXT access's payload must not appear early");
        @(posedge sys_clk); #1; h_ack_data = ack_word(8'h00, 16'h0000, 4'h0, 1'b1, 1'b1);
        @(posedge sys_clk); #1; h_ack = 1'b1;
        @(posedge sys_clk); #1; h_ack = 1'b0;
        wait_cycles(2);
        chk("rdata-zero-is-a-real-byte", rdata == 8'h00,
            "a read that returns 00h must land as 00h -- rdata=0 is an answer, not an absence");

        // ---- 20. a go in the same cycle as an ack: both happen -----------------------------------------
        req_seen = 0;
        csr_go(1'b1);
        @(posedge sys_clk); #1; h_ack = 1'b1; h_ack_data = ack_word(8'h24, 16'h0007, 4'h1, 1'b0, 1'b0);
        @(posedge sys_clk); #1; h_ack = 1'b0;
        wait_cycles(3);
        chk("go-during-ack-both", (req_seen == 1) && (rdata == 8'h24),
            "the map must not lose an access because a result was landing, nor lose the result");

        // ---- summary -----------------------------------------------------------------------------------
        $display("SUMMARY checks=%0d failed=%0d", nchecks, nfail);
        $finish;
    end

    // A hang must be reported as a hang. Nothing here should take more than a few thousand cycles, and a
    // check that waits forever is indistinguishable from a simulator that has stopped.
    initial begin
        #2_000_000;
        $display("CSRCHK watchdog                  FAIL no SUMMARY within 2 ms of sim time");
        $display("SUMMARY checks=%0d failed=%0d", nchecks + 1, nfail + 1);
        $finish;
    end

endmodule
