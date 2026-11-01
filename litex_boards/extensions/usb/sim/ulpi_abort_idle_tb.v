`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// ulpi_abort_idle_tb.v -- "what does `ulpi_abort` mean when there is nothing to abort?"
//
// THE QUESTION. `ulpi_abort` is a control input that means "stop the thing in flight". Every such input has
// to say what happens when nothing is in flight, in code and not in a comment, because the two possible
// answers -- drop it, or hold it for the next victim -- differ by exactly one `&`, and the wrong one reports
// as a *later, innocent* access. This file is bug 4 and its neighbour, found on the bench 2026-11-01:
//
//   * `id_vid_low_after_abort` returned `00` with `errors=dir_abort+nxt_timeout` (`bench/2026-11-01_ulpi_id_t19.tsv`)
//     one row after `abort_csr_idle` reported the link IDLE with `busy`=0. The `abort` latch was
//     `If(abort_req, level.eq(1))` with clears that all mean "an access ended", so on an idle link nothing
//     ever cleared it: it crossed into cd_usb and aborted the NEXT command.
//   * The other answer, "abort is inert", is the same bug in the opposite direction and is invisible to any
//     test that only asks whether the *next* access survives -- which is why row 3 exists.
//
// So there are four rows, and the middle two are the pair: an `abort` with nothing to abort must be dropped
// (row 2), and an `abort` with something to abort must still reach the pad (row 3). Neither alone proves the
// gate exists.
//
//   iverilog -g2012 -I. -o abort_idle.out ulpi_abort_idle_tb.v ulpi_phy_model.v ulpi_link.v && vvp abort_idle.out
//     +vcd=abort_idle.vcd          (optional; no dump unless asked, so a check-suite run writes nothing)
//
// Expect 4 RESULT lines, all PASS, then `SUMMARY 4 passed, 0 failed`. Run by `tools/ulpi_rtl_sim.py` (the
// `rtl` category of `tools/check.sh`) on every commit; `tools/ulpi_mutate.py` proves the rows can fail.
// Original reproduction: `bench/2026-11-01_ulpi_abort_idle/` in the docs repo, ticket 08.
// -----------------------------------------------------------------------------
`include "ulpi_phy_defs.vh"

module ulpi_abort_idle_tb;

    reg sys_clk = 1'b0;
    reg usb_clk = 1'b0;
    always #10.0  sys_clk = ~sys_clk;
    always #8.333 usb_clk = ~usb_clk;

    reg sys_rst = 1'b1;

    reg         h_req = 0, h_rw = 0, abort_req = 0, bus_grant = 1, phy_reset = 1, clk_ok = 1;
    reg  [ 5:0] h_addr = 0;
    reg  [ 7:0] h_wdata = 0;
    wire        h_ack, h_busy;
    wire [29:0] h_ack_data;
    wire        done, err_dir_abort, err_nxt_timeout, err_no_clock, err_reset;
    wire        dir_now, oe_now;
    wire [ 3:0] state;
    wire [15:0] last_bus_waited;
    wire [ 3:0] last_retry;
    // Widths are the DUT's, from `UlpiLink.__init__`: cnt_access is 16 and every error counter is 8. The
    // first copy of this file declared all of them 16 and iverilog said so only as a warning, which is the
    // kind of warning that outlives the signal it is about.
    wire [15:0] cnt_access;
    wire [ 7:0] cnt_dir_abort, cnt_timeout, cnt_reset, cnt_no_clock;
    wire        ulpi_clear;

    wire [ 7:0] ulpi_data;
    wire [ 7:0] dut_o, dut_oe;
    wire        dut_stp;
    wire        phy_dir, phy_nxt, phy_oe;
    wire [ 7:0] phy_data;

    ulpi_link dut (
        .sys_clk(sys_clk), .usb_clk(usb_clk), .sys_rst(sys_rst),
        .h_req(h_req), .h_rw(h_rw), .h_addr(h_addr), .h_wdata(h_wdata),
        .h_ack(h_ack), .h_ack_data(h_ack_data), .h_busy(h_busy),
        .abort_req(abort_req), .bus_grant(bus_grant), .phy_reset(phy_reset), .clk_ok(clk_ok),
        .clear(ulpi_clear),
        .done(done), .err_dir_abort(err_dir_abort), .err_nxt_timeout(err_nxt_timeout),
        .err_no_clock(err_no_clock), .err_reset(err_reset),
        .dir_now(dir_now), .oe_now(oe_now), .state(state),
        .last_bus_waited(last_bus_waited), .last_retry(last_retry),
        .cnt_access(cnt_access), .cnt_dir_abort(cnt_dir_abort), .cnt_timeout(cnt_timeout),
        .cnt_reset(cnt_reset), .cnt_no_clock(cnt_no_clock),
        .ulpi_data_o(dut_o), .ulpi_data_oe(dut_oe), .ulpi_data_i(ulpi_data),
        .ulpi_stp(dut_stp), .ulpi_dir(phy_dir), .ulpi_nxt(phy_nxt)
    );

    reg        knop_stall_nxt = 0, knop_wrong_cmd = 0, knop_rev_bus = 0, knop_ignore_write = 0;
    reg  [3:0] knop_abort_n = 0;
    reg        knop_abort_forever = 0;
    reg  [1:0] linestate = 2'b01;
    wire [7:0] seen_cmd, seen_wdata;
    wire       saw_stp, cmd_is_write, nxt_cycles_too_long;
    wire [3:0] abort_count;
    wire       unsolicited_tx, unsolicited_cmd;
    wire [7:0] unsolicited_byte;

    ulpi_phy_model phy (
        .clk(usb_clk), .phy_reset(phy_reset),
        .link_data(dut_o), .link_oe(dut_oe[0]), .bus(ulpi_data),
        .knop_ignore_float(1'b0),
        .phy_data(phy_data), .phy_dir(phy_dir), .phy_nxt(phy_nxt), .phy_stp(dut_stp),
        .phy_oe(phy_oe),
        .knop_stall_nxt(knop_stall_nxt), .knop_abort_n(knop_abort_n),
        .knop_abort_forever(knop_abort_forever), .knop_wrong_cmd(knop_wrong_cmd),
        .knop_rev_bus(knop_rev_bus), .knop_ignore_write(knop_ignore_write),
        .clr_seen(1'b0),
        .linestate(linestate),
        .seen_cmd(seen_cmd), .seen_wdata(seen_wdata), .saw_stp(saw_stp),
        .cmd_is_write(cmd_is_write), .nxt_cycles_too_long(nxt_cycles_too_long),
        .abort_count(abort_count),
        .unsolicited_tx(unsolicited_tx), .unsolicited_cmd(unsolicited_cmd),
        .unsolicited_byte(unsolicited_byte)
    );

    assign ulpi_data = dut_oe[0] ? dut_o  : 8'hzz;
    assign ulpi_data = phy_oe    ? phy_data : 8'hzz;

    // The `stp` witness, counted on the PAD rather than from any internal signal: rows 2 and 3 ask opposite
    // questions about the same pin ("nothing", "at least one pulse"), and an internal `stp_drive` would
    // answer about our FSM instead of about the wire the PHY sees. `pads.stp` is `stp_drive & ~clear`, which
    // is what §3.8.1.2's "assert stp for only one cycle" is a statement about.
    integer      stp_cycles = 0;
    reg          stp_watch  = 0;
    always @(posedge usb_clk) if (stp_watch && dut_stp) stp_cycles = stp_cycles + 1;

    localparam integer ACCESS_BOUND = 200_000;
    reg        acc_ok;
    reg  [7:0] acc_rdata;
    reg        acc_abort, acc_timeout;

    // One access, and `abort_after` >= 0 means "pulse `abort` that many cd_sys cycles into it" -- which is
    // the whole difference between row 2 and row 3. A negative value never pulses.
    task automatic do_access(input rw, input [5:0] a, input [7:0] d, input integer abort_after);
        integer guard;
        begin
            acc_ok = 0; acc_rdata = 8'hxx; acc_abort = 0; acc_timeout = 0;
            guard = 0;
            while (h_busy && (guard < ACCESS_BOUND)) begin @(posedge sys_clk); guard = guard + 1; end
            @(posedge sys_clk);
            h_rw <= rw; h_addr <= a; h_wdata <= d; h_req <= 1'b1;
            @(posedge sys_clk);
            h_req <= 1'b0;
            repeat (2) @(posedge sys_clk);
            if (abort_after >= 0) begin
                repeat (abort_after) @(posedge sys_clk);
                abort_req <= 1'b1;
                @(posedge sys_clk);
                abort_req <= 1'b0;
            end
            guard = 0;
            while (!h_ack && (guard < ACCESS_BOUND)) begin
                @(posedge sys_clk);
                guard = guard + 1;
                if (err_reset || err_no_clock) guard = ACCESS_BOUND;
            end
            acc_ok = h_ack;
            if (h_ack) begin
                acc_rdata   = h_ack_data[9:2];
                acc_abort   = h_ack_data[0];
                acc_timeout = h_ack_data[1];
            end else begin
                acc_rdata = 8'hxx; acc_abort = 1'bx; acc_timeout = 1'bx;
            end
            @(posedge sys_clk);
        end
    endtask

    reg stale_level = 1'bx;
    integer pass_n = 0, fail_n = 0;
    // `why` is 200 bytes because a 96-byte one silently LOST its head on the first run of this file: a
    // fixed-width Verilog string truncates from the left, so the printed reason for bug 4 read
    // "<-- illed THIS access ...". A judge that cannot say what it judged is the failure this repo keeps
    // hitting, so the field is wide and the strings are not.
    task automatic row(input [8*24-1:0] name, input ok, input [8*200-1:0] why);
        begin
            if (ok) begin
                pass_n = pass_n + 1;
                $display("RESULT %0s PASS rdata=%h abort=%b tmo=%b ack=%b err_dir_abort=%b err_tmo=%b stp=%0d stale=%b state=%b abort_level=%b",
                         name, acc_rdata, acc_abort, acc_timeout, acc_ok, err_dir_abort, err_nxt_timeout,
                         stp_cycles, stale_level, state, dut.abort_level);
            end else begin
                fail_n = fail_n + 1;
                // `stale=` is the sampled `abort_level`, on BOTH lines deliberately: a row can be red with
                // every other field green, because the ack of the very access the row reads is what clears a
                // level that should never have been latched. Without this field that red line reads as a
                // passing row -- the trap `bench/2026-11-01_ulpi_abort_idle/README.md` documents for bug 4's
                // original output.
                $display("RESULT %0s FAIL rdata=%h abort=%b tmo=%b ack=%b err_dir_abort=%b err_tmo=%b stp=%0d stale=%b state=%b abort_level=%b  <-- %0s",
                         name, acc_rdata, acc_abort, acc_timeout, acc_ok, err_dir_abort, err_nxt_timeout,
                         stp_cycles, stale_level, state, dut.abort_level, why);
            end
        end
    endtask

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("abort_idle.vcd");
            $dumpvars(0, ulpi_abort_idle_tb);
        end
        repeat (5) @(posedge sys_clk);
        sys_rst = 0;
        // Boot in reset (the registers' power-up state), then release exactly like the harness' link_reset.
        repeat (200) @(posedge sys_clk);
        phy_reset = 0;
        repeat (2000) @(posedge sys_clk);          // let UlpiClear's relock tail expire

        // --- row 1: control. A read works, so anything red below is about the abort and not the link. ----
        // `stale_level` is printed as `x` on the rows that did not sample it, so a reader cannot mistake one
        // row's sample for another's -- the value is only meaningful where an `abort` was just pulsed.
        stale_level = 1'bx;
        do_access(1'b1, 6'h00, 8'h00, -1);
        row("baseline_rd",  acc_ok && (acc_rdata === 8'h24) && !acc_abort && !acc_timeout,
            "expected 24 with no error: if THIS row is red the abort rows prove nothing");

        // --- row 2 (bug 4): `abort` with NO outstanding access. -----------------------------------------
        // h_busy is 0, so there is nothing for the Link to release. The request must be dropped, leave no
        // trace on the pads, and above all must not be held for the next victim.
        if (h_busy) $display("NOTE: link was busy at the abort pulse -- the row is not measuring idleness");
        $display("PRE-ABORT idle: h_busy=%b state=%b oe_now=%b cnt_access=%0d", h_busy, state, oe_now, cnt_access);
        stp_cycles = 0; stp_watch = 1;
        @(posedge sys_clk);
        abort_req <= 1'b1;
        @(posedge sys_clk);
        abort_req <= 1'b0;
        repeat (20) @(posedge sys_clk);
        stale_level = dut.abort_level;          // the latch itself: was the request HELD?
        repeat (30) @(posedge sys_clk);
        stp_watch = 0;
        do_access(1'b1, 6'h00, 8'h00, -1);
        row("read_after_idle_abort",
                acc_ok && (acc_rdata === 8'h24) && !acc_abort && !acc_timeout
                && (stp_cycles == 0) && (stale_level === 1'b0),
            "bug 4: an abort on an idle link was latched and killed THIS access (00 + dir_abort+nxt_timeout); it must be ignored, in the latch and on the pads");

        // --- row 3: the other direction. `abort` DURING an access must still reach `stp`. ----------------
        // Without this row, "row 2 passes" is also true of a Link whose abort input is wired to nothing, and
        // the fix could have been made by deleting the feature. The PHY is told not to answer (`nxt` never
        // asserts), so the access is genuinely in flight -- and the abort, not the 2048-cycle stall bound of
        // the SIM build, must be the thing that ends it. `rdata` must stay 00: an access that never reached
        // its read cycle reports no byte (spec testing decision 8), and a byte here would be some earlier
        // access's. `stp` must be ONE cycle, because §3.8.1.2 says "The Link must assert stp for only one
        // cycle", and a level re-firing the release is exactly how bug 5's fix could come back as a pulse train.
        //
        // THE PREAMBLE IS THE TEST: pulse an ignored `abort` first, and let NO access complete between it and
        // the justified one. Row 2 cannot judge whether an ignored request left a stale level, because its own
        // following read produces the ack that clears whatever the idle pulse latched -- which is exactly why
        // the `abort_idle` mutation came back BLIND the first time this row was written against a suite shaped
        // like row 2. A dropped request must be dropped, not merely harmless-until-the-next-ack: leave the
        // level high and the justified abort below has no rising edge left to fire, so the door is dead until
        // someone resets the PHY -- the same "cure hides it" shape as bugs 3 and 4.
        @(posedge sys_clk);
        abort_req <= 1'b1;
        @(posedge sys_clk);
        abort_req <= 1'b0;
        repeat (20) @(posedge sys_clk);
        $display("PRE-INFLIGHT h_busy=%b abort_level=%b  (a held level here steals the edge this row asks for)",
                 h_busy, dut.abort_level);
        knop_stall_nxt = 1;
        stp_cycles = 0; stp_watch = 1;
        do_access(1'b1, 6'h00, 8'h00, 40);      // abort 40 cd_sys cycles in: far inside the bound, inside busy
        stp_watch = 0;
        knop_stall_nxt = 0;
        row("abort_in_flight_stp",
                acc_ok && acc_abort && !acc_timeout && (acc_rdata === 8'h00) && (stp_cycles == 1),
            "abort during an access must pulse stp EXACTLY ONCE (ULPI 1.1 3.8.1.2) and end THAT access with dir_abort, not time out");
        // What this row caught, and why it is not merely row 2's mirror image. Written before the fix, it
        // failed with `stp=0 tmo=1`: the master's abort branch outranked the FSM's own `ABORT` state, so while
        // the crossed level was high the `ABORT` case -- the only writer of `stp_drive` -- never ran, and the
        // access ended 33 us later by OUR stall bound. That is bug 5: `ulpi_abort` could not release anything,
        // it could only time something out (`aborted` was set by a branch that had never touched the bus).
        // Row 2 cannot see it (an ignored abort does not have to reach a pad) and row 2 alone would therefore
        // have been satisfied by deleting the feature -- which is the whole reason the pair is one test.
        // Row 2 also proves something row 3 depends on: a dropped idle abort must leave no latched level, or
        // there would be no rising edge left to fire this one.

        // --- row 4: control. Does a PHY reset still work afterwards? (The bench's own recovery door.) ----
        stale_level = 1'bx;                    // not sampled here: row 3's abort is long since acked
        phy_reset = 1; repeat (200) @(posedge sys_clk); phy_reset = 0;
        repeat (2000) @(posedge sys_clk);
        do_access(1'b1, 6'h00, 8'h00, -1);
        row("read_after_phy_reset",  acc_ok && (acc_rdata === 8'h24),
            "the link must survive all of the above once the PHY is reset");

        $display("SUMMARY %0d passed, %0d failed  (abort_latched=%b state=%b err_dir_abort=%b err_tmo=%b unsolicited_tx=%b)",
                 pass_n, fail_n, dut.abort_level, state, err_dir_abort, err_nxt_timeout, unsolicited_tx);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("RESULT global_watchdog FAIL the simulation itself stopped making progress");
        $finish;
    end
endmodule
