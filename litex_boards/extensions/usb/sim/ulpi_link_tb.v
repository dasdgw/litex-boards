`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// ulpi_link_tb.v -- run tools/ulpi_vectors.tsv against the real RTL.
//
// This testbench has one job: take the shared vector list, drive the generated `ulpi_link` and the
// behavioural PHY model together, and say per row whether the Link did what the vectors say. It is
// deliberately thin. Anything clever here is a place a wrong expectation can hide, and the entire value of
// this suite over a bench run is that the expectations live in a file both PHY models read.
//
// Run it through `tools/ulpi_rtl_sim.py` in the project repo, which generates the DUT from migen, converts
// the TSV to the `$readmemh` word list below, and re-labels `RESULT n` with the vector's `id`. The
// conversion is that script's job and not this file's: Verilog's `$sscanf` on a TSV with empty fields reads
// the wrong columns silently, and a silently-shifted expectation column is the exact failure this repo has
// been built to catch.
//
//   iverilog -g2012 -o sim.out ulpi_link_tb.v ulpi_phy_model.v ulpi_link.v
//   vvp sim.out +hex=vectors.hex +nvec=<count>
//
// Three things this file does on purpose:
//
//  * The data bus is a **real tri-state net with two drivers**. Resolving to x when both drive is the only
//    way a Link that forgets to release `oe` shows up as a failure instead of as a byte that happens to
//    look right.
//  * Every access has a wall-clock bound *in this file*, independent of the DUT's own timeout. A Link that
//    hangs must be reported as a hang; a check that waits forever for a `done` that cannot come is
//    indistinguishable from a simulator that has stopped.
//  * `RESULT n PASS|FAIL <detail>` on one line per vector, with the byte expected, the byte seen, and the
//    state the FSM finished in. A harness that prints only "FAIL" is a harness you end up reading waveforms
//    for anyway.
// -----------------------------------------------------------------------------

`include "ulpi_phy_defs.vh"

module ulpi_link_tb;

    // ---------------------------------------------------------------------------------------------
    // Clocks: 50 MHz sys, 60 MHz usb, unrelated phases. Nothing about ULPI cares about the phase
    // relationship between the SoC clock and the PHY clock, and making them unrelated is what forces the
    // crossing to be a real crossing rather than a lucky alignment.
    // ---------------------------------------------------------------------------------------------
    reg sys_clk = 1'b0;
    reg usb_clk = 1'b0;
    reg usb_clk_en = 1'b1;              // 0 = the PHY has stopped ticking
    always #10.0  sys_clk = ~sys_clk;
    always #8.333 usb_clk = (usb_clk_en ? ~usb_clk : 1'b0);

    reg sys_rst = 1'b1;

    // ---------------------------------------------------------------------------------------------
    // Host side.
    // ---------------------------------------------------------------------------------------------
    reg         h_req = 0, h_rw = 0, abort_req = 0, bus_grant = 1, phy_reset = 0, clk_ok = 1;
    reg  [ 5:0] h_addr = 0;
    reg  [ 7:0] h_wdata = 0;
    wire        h_ack, h_busy;
    wire [29:0] h_ack_data;
    wire        done, err_dir_abort, err_nxt_timeout, err_no_clock, err_reset;
    wire        dir_now, oe_now;
    wire [ 3:0] state;
    wire [15:0] last_bus_waited;
    wire [ 3:0] last_retry;
    wire [15:0] cnt_access, cnt_dir_abort, cnt_timeout, cnt_reset, cnt_no_clock;
    wire        ulpi_clear;

    wire [ 7:0] ulpi_data;
    wire [ 7:0] dut_o, dut_oe;
    wire        dut_stp;

    ulpi_link dut (
        .sys_clk(sys_clk), .usb_clk(usb_clk), .sys_rst(sys_rst),
        .h_req(h_req), .h_rw(h_rw), .h_addr(h_addr), .h_wdata(h_wdata),
        .h_ack(h_ack), .h_ack_data(h_ack_data), .h_busy(h_busy),
        .abort_req(abort_req), .bus_grant(bus_grant), .phy_reset(phy_reset), .clk_ok(clk_ok),
        .clear(ulpi_clear),   // migen flattens `UlpiClear.clear` to `clear` at the top
        .done(done), .err_dir_abort(err_dir_abort), .err_nxt_timeout(err_nxt_timeout),
        .err_no_clock(err_no_clock), .err_reset(err_reset),
        .dir_now(dir_now), .oe_now(oe_now), .state(state),
        .last_bus_waited(last_bus_waited), .last_retry(last_retry),
        .cnt_access(cnt_access), .cnt_dir_abort(cnt_dir_abort), .cnt_timeout(cnt_timeout),
        .cnt_reset(cnt_reset), .cnt_no_clock(cnt_no_clock),
        .ulpi_data_o(dut_o), .ulpi_data_oe(dut_oe), .ulpi_data_i(ulpi_data),
        .ulpi_stp(dut_stp), .ulpi_dir(phy_dir), .ulpi_nxt(phy_nxt)
    );

    initial begin watch_oe = 1'b0; drove_after_reset = 1'b0; end
    // **A NONZERO pattern**, not merely `oe`. Re-derived 2026-11-01 when the Link began driving the ULPI idle
    // level (bug 3): `oe` high with `data`=00h is what ULPI 1.1 §2.3.2 *requires* of a Link that owns the bus,
    // so watching the enable alone would now fail the row for obeying the spec. What the row is for is the
    // command byte surviving on the pads into a transaction the host was told is dead -- a nonzero pattern --
    // and `tools/ulpi_mutate.py no_relock_tail` still reddens exactly that.
    always @(posedge sys_clk) if (watch_oe && dut_oe[0] && (dut_o != 8'h00)) drove_after_reset = 1'b1;

    // ---------------------------------------------------------------------------------------------
    // The model, and the knobs the vector's `fault` column turns.
    // ---------------------------------------------------------------------------------------------
    reg         clr_seen = 0;
    reg         knop_stall_nxt = 0, knop_wrong_cmd = 0, knop_rev_bus = 0, knop_ignore_write = 0;
    reg  [ 3:0] knop_abort_n = 0;
    reg         knop_abort_forever = 0;
    reg  [ 1:0] linestate = 2'b01;      // SE0 with D+ low: nothing plugged in, hub up

    wire [ 7:0] phy_data;
    wire        phy_dir, phy_nxt, phy_oe;
    wire [ 7:0] seen_cmd, seen_wdata;
    wire        saw_stp, cmd_is_write, nxt_cycles_too_long;
    wire [ 3:0] abort_count;

    ulpi_phy_model phy (
        .clk(usb_clk), .phy_reset(phy_reset),
        .link_data(dut_o), .link_oe(dut_oe[0]),
        // The resolved net, which is the only thing a real PHY's data receiver has. Wiring the model to
        // `dut_o`/`dut_oe` alone is what let this suite report 33/33 green against a Link that tri-states the
        // bus and leaves the PHY's own answer for the PHY to decode as a command (bug 3, 2026-11-01).
        .bus(ulpi_data),
        .knop_ignore_float(1'b0),      // this PHY decodes what it samples; the board does, so the model must
        .phy_data(phy_data), .phy_dir(phy_dir), .phy_nxt(phy_nxt), .phy_stp(dut_stp),
        .phy_oe(phy_oe),
        .knop_stall_nxt(knop_stall_nxt), .knop_abort_n(knop_abort_n),
        .knop_abort_forever(knop_abort_forever), .knop_wrong_cmd(knop_wrong_cmd),
        .knop_rev_bus(knop_rev_bus), .knop_ignore_write(knop_ignore_write),
        .clr_seen(clr_seen),
        .linestate(linestate),
        .seen_cmd(seen_cmd), .seen_wdata(seen_wdata), .saw_stp(saw_stp),
        .abort_count(abort_count), .cmd_is_write(cmd_is_write),
        .nxt_cycles_too_long(nxt_cycles_too_long),
        .unsolicited_tx(unsolicited_tx), .unsolicited_cmd(unsolicited_cmd),
        .unsolicited_byte(unsolicited_byte)
    );

    wire unsolicited_tx, unsolicited_cmd;
    wire [7:0] unsolicited_byte;

    // Two continuous drivers on one net, which Verilog resolves. When both drive different values the
    // result is x -- and that is the outcome a check must never be able to mistake for 0x24.
    assign ulpi_data = dut_oe[0] ? dut_o  : 8'hzz;
    assign ulpi_data = phy_oe    ? phy_data : 8'hzz;

    // ---------------------------------------------------------------------------------------------
    // Vectors: `tools/ulpi_vectors.tsv`, converted by the runner to 40-bit words.
    //
    //   [39:34] unused  [33] has_exp  [32:25] exp  [24:17] wdata  [16:11] addr  [10] op
    //   [9:6] fault  [5:3] pre  [2:0] exp_err
    //
    // The encodings live in tools/ulpi_rtl_sim.py (FAULT/PRE/ERR). This file is the only other place they
    // appear, and the names below are the reason a runner generates the words rather than this file parsing
    // the TSV.
    // ---------------------------------------------------------------------------------------------
    localparam F_NONE = 0, F_ABORT_ONCE = 1, F_ABORT_3 = 2, F_ABORT_FOREVER = 3, F_STALL = 4,
               F_SILENCE = 5, F_RESET_MID = 6, F_BUS_REV = 7, F_IGNORE_WRITE = 8,
               F_RESET_MID_NOCLK = 9;

    // The freeze row's witness (see the sequence in the main loop): did the Link put anything on the data
    // pad *after* the host was told the access was dead? `dut_oe` is the TSTriple's own enable, so this is
    // the pad, not the FSM's opinion about it.
    reg watch_oe, drove_after_reset;
    localparam P_READY = 0, P_PHY_RESET = 1, P_HUB_RELEASED = 2, P_HUB_ASSERTED = 3;
    localparam E_NONE = 0, E_TIMEOUT = 1, E_ABORT = 2, E_NO_CLOCK = 3, E_RESET = 4;

    reg [39:0] vecs [0:63];
    integer    nvec, i;
    integer    pass_n, fail_n;

    // One access as the DUT reported it.
    reg        acc_ok;
    reg  [7:0] acc_rdata;
    reg        acc_abort, acc_timeout;
    reg  [15:0] acc_waited;
    reg  [3:0]  acc_retry;

    // The bound is in sys cycles: 200 000 = 4 ms, above every vector's real duration and far below forever.
    // It exists because the DUT's own bound is the thing under test -- a check that trusts only it cannot
    // tell "correctly timed out" from "hung".
    localparam integer ACCESS_BOUND = 200_000;

    task automatic do_access(input rw, input [5:0] a, input [7:0] d);
        integer guard;
        begin
            acc_ok = 0; acc_rdata = 8'hxx; acc_abort = 0; acc_timeout = 0;
            acc_waited = 0; acc_retry = 0;
            // The bridge refuses a request while busy, so waiting here is not politeness: issuing into
            // `busy` would silently drop the vector.
            guard = 0;
            while (h_busy && (guard < ACCESS_BOUND)) begin @(posedge sys_clk); guard = guard + 1; end
            @(posedge sys_clk);
            h_rw <= rw; h_addr <= a; h_wdata <= d; h_req <= 1'b1;
            @(posedge sys_clk);
            h_req <= 1'b0;
            // Two cycles before the bail-out check becomes legal: the link clears the sticky error bits on
            // the edge it accepts `h_req`, and reading them before that edge sees the *previous* access's
            // error -- which is precisely how the mutation row after `reset_mid_rd` reported a hang it did
            // not have. The bits are sticky by design; a harness that forgets that inherits the last row's
            // verdict.
            repeat (2) @(posedge sys_clk);
            guard = 0;
            while (!h_ack && (guard < ACCESS_BOUND)) begin
                @(posedge sys_clk);
                guard = guard + 1;
                // An access killed by a PHY reset never acks -- that is the design (the bridge reports it
                // and drops busy). Sitting here for an ack that cannot come would report every reset vector
                // as a hang and hide the one bug this row is looking for.
                if (err_reset || err_no_clock) guard = ACCESS_BOUND;
            end
            // Copy the result bundle **only when there is one**. `h_ack_data` is a wire that keeps the last
            // completed access's payload, so sampling it unconditionally makes an access that never finished
            // print the *previous* byte -- and a harness that prints 24 for a hung access is the exact
            // stale-read failure the vector suite is written to catch (spec testing decision 8). `xx` in the
            // output is the honest answer for "it did not return a byte".
            acc_ok = h_ack;
            if (h_ack) begin
                acc_rdata   = h_ack_data[9:2];
                acc_abort   = h_ack_data[0];
                acc_timeout = h_ack_data[1];
                acc_waited  = h_ack_data[25:10];
                acc_retry   = h_ack_data[29:26];
            end else begin
                acc_rdata   = 8'hxx;
                acc_abort   = 1'bx;
                acc_timeout = 1'bx;
                acc_waited  = 16'hxxxx;
                acc_retry   = 4'hx;
            end
            @(posedge sys_clk);
        end
    endtask

    task automatic apply_pre(input [2:0] pre);
        begin
            phy_reset = 0; usb_clk_en = 1; clk_ok = 1; linestate = 2'b01;
            if (pre == P_PHY_RESET) begin
                phy_reset = 1;  clk_ok = 0;      // no CLKOUT while the PHY is in reset -- see the vector
            end
            // The two hub rows are the W11 pin, not a PHY register: what the Debug register reports is the
            // line state, which is the only way to see a hub reset work before enumeration.
            if (pre == P_HUB_RELEASED) linestate = 2'b01;
            if (pre == P_HUB_ASSERTED) linestate = 2'b00;
            // Wait for the clear to release. The boot and relock counters ARE the mechanism, so starting a
            // vector before `ulpi_clear` drops is testing a state the board can never be in.
            guard_clear;
        end
    endtask

    integer gguard;
    task automatic guard_clear;
        begin
            gguard = 0;
            while (ulpi_clear && (gguard < ACCESS_BOUND)) begin @(posedge sys_clk); gguard = gguard + 1; end
        end
    endtask

    // TB_HOLE, closed. `seen_cmd`/`seen_wdata`/`saw_stp` are the PHY's memory of the *file*, and the file is
    // run down one shared model, so a row that decodes nothing used to inherit the previous row's command and
    // PASS on it. Pulse the model's clear on an idle bus before every row. One `usb_clk`, not one `sys_clk`:
    // the model is clocked by the PHY domain and a sys-domain pulse is 0.8 of a usb cycle, which is exactly
    // the kind of thing that works until it does not.
    task automatic clear_witnesses;
        begin
            @(posedge usb_clk); clr_seen = 1'b1;
            @(posedge usb_clk); clr_seen = 1'b0;
        end
    endtask

    task automatic apply_fault(input [3:0] fault);
        begin
            knop_stall_nxt = 0; knop_wrong_cmd = 0; knop_rev_bus = 0; knop_ignore_write = 0;
            knop_abort_n = 0; knop_abort_forever = 0;
            case (fault)
                F_NONE:         ;
                // abort_once is n=1 and abort_3 is n=3 rather than two separate knobs: the difference is
                // only how many retries the Link had to make, and one knob is one thing less to get wrong
                // in the model.
                F_ABORT_ONCE:    knop_abort_n = 4'd1;
                F_ABORT_3:       knop_abort_n = 4'd3;
                F_ABORT_FOREVER: knop_abort_forever = 1'b1;
                // `stall` and `silence` both leave nxt low forever. The difference between them (a model
                // that decodes the command and never throttles, vs one that ignores it) is not visible to
                // the Link, and both must end in the same place: our own bound.
                F_STALL:         knop_stall_nxt = 1'b1;
                F_SILENCE:       knop_stall_nxt = 1'b1;
                F_BUS_REV:       knop_rev_bus = 1'b1;
                F_IGNORE_WRITE:  knop_ignore_write = 1'b1;
                default:         ;
            endcase
        end
    endtask

    task automatic check_row(input integer n, input has_exp, input [7:0] exp, input [2:0] exp_err,
                             input [3:0] fault, input [2:0] pre, input op);
        reg ok;
        reg [8*200:1] why;
        begin
            ok  = 1;
            why = "";
            // Every row also checks the **encoding the PHY actually decoded** -- `exp_cmd`, set by whoever
            // ran last: the command of a write access for a write row, and of the read-back for every row
            // that reads. This is how a wrong TX CMD gets *named* rather than merely failing: a master that
            // sent 0x40|PID (a packet -- Track A's encoding, never ours), or with a stuck rw bit, or an
            // address field off by one, would otherwise read as "no ack" and cost a waveform session.
            // The model decoded it independently from Table 6, so the cross-check is not a tautology.
            //
            // First, deliberately, ahead of the byte comparison: when a row's byte *and* its encoding are
            // both wrong, the encoding is the cause, and naming the symptom is what costs the evening. The
            // main loop clears `check_cmd` on the reset rows (a PHY in reset decodes nothing, and the
            // reset-mid row's `seen_cmd` belongs to a transaction that no longer exists).
            if (ok && (fault == F_RESET_MID_NOCLK) && drove_after_reset) begin
                // Everything else this row expects -- `err_reset`, `busy` clear -- is the BRIDGE's doing, and
                // `cd_sys` never stopped clocking, so those hold with or without the relock tail. Without
                // this check the tail, which is the mechanism decision 6 was written for, was covered by no
                // vector at all: measured 2026-10-31, deleting it left 28/28 green
                // (`tools/ulpi_mutate.py no_relock_tail`).
                ok  = 0;
                why = "the Link drove the data bus for an access the host had been told was dead";
            end
            if (check_cmd !== 1'b0) begin
                if (seen_cmd !== exp_cmd) begin
                    ok  = 0;
                    why = "the PHY decoded the wrong TX CMD encoding";
                end
            end


            // `if (ok)` on both branches, and that is load-bearing rather than tidiness: these compute the
            // verdict, so assigning to `ok` unconditionally here would erase the encoding verdict above and
            // put the suite back to "wrong byte" for a wrong-command bug.
            if (ok && !has_exp) begin
                // The row is about an error, not a byte. The vector file's rule: it passes only if the
                // named error is the one that latched.
                case (exp_err)
                    E_TIMEOUT:  ok = err_nxt_timeout;
                    E_ABORT:    ok = err_dir_abort;
                    E_NO_CLOCK: ok = err_no_clock;
                    E_RESET:    ok = err_reset;
                    default:    ok = 0;
                endcase
                if (!ok) begin
                    why = "the named error never latched";
                end else if (acc_ok && (acc_rdata !== 8'h00)) begin
                    // A row with no byte expectation is a row whose access was *not* supposed to deliver
                    // one. If it acked anyway and hands back a byte, that byte is the previous access's:
                    // `rdata` is a register, and a timed-out or abort-exhausted access reports through the
                    // same ack. "0x24, and by the way it timed out" is the lie this line refuses -- the
                    // harness-side half of the same rule is the `if (h_ack)` in `do_access` above, and it
                    // cannot see this case because a failure that acks is indistinguishable from a success
                    // by the ack alone. (spec testing decision 8; found 2026-10-31, `stall_rd` read 24.)
                    ok  = 0;
                    why = "a failed access returned a byte (stale rdata)";
                end else if (h_busy) begin
                    // Half of what these rows test is that the Link comes back: an error bit latched behind
                    // a permanently asserted busy is a hang the host cannot distinguish from a slow access
                    // (spec testing decision 5). The `ok = 0` is the whole line and it was missing until
                    // 2026-10-31: the message existed, `why` was assigned, and the row still printed PASS.
                    // What reddened the mutation was the *next* vector, blocked behind the stuck busy -- a
                    // true failure reported in the wrong row, which is how this became a verdict instead of
                    // a rumour. See `tools/ulpi_mutate.py`'s `lost_keeps_busy`.
                    ok  = 0;
                    why = "error latched but busy never cleared";
                end
            end else if (ok) begin
                if (!acc_ok) begin
                    ok  = 0;
                    why = "no ack inside the bound (hung, or died for a reason the row did not name)";
                    // `ustx`/`ubyte` on the RESULT line carries it, not this string: `why` is a fixed-width
                    // reg and a long sentence here silently truncates the reason the row exists.
                    
                end else if (acc_rdata !== exp) begin
                    ok  = 0;
                    why = "byte mismatch";
                end else if ((exp_err == E_ABORT) && !err_dir_abort) begin
                    // A retry that worked still has to report the aborts: "passed after 3 aborts" and
                    // "passed cleanly" are different facts (spec decision 12).
                    ok  = 0;
                    why = "byte correct, abort not reported";
                end
            end
            if (ok) begin
                pass_n = pass_n + 1;
                // A PASS carries the same evidence as a FAIL. It used to carry less, which meant the four
                // error bits were only ever visible on a failing row -- and the cross-model diff between
                // this suite and the python twin (`tools/test_ulpi_phy_model.py`, section 6) compares
                // readings on rows where both models passed, so it needs them here.
                $display("RESULT %0d PASS rdata=%h ack=%b abort=%b tmo=%b noclk=%b reset=%b busy=%b waited=%0d retry=%0d state=%0d",
                         n, acc_rdata, acc_ok, acc_abort, acc_timeout, err_no_clock, err_reset, h_busy,
                         acc_waited, acc_retry, state);
            if (unsolicited_tx)
                $display("     WITNESS row %0d: unsolicited USB Transmit, decoded %h off the pads (bug 3)",
                         n, unsolicited_byte);
            end else begin
                fail_n = fail_n + 1;
                // One line, one format string, no concatenation: `check_row` is the only place a verdict is
                // formed, and a verdict that needs a grep across three lines is a verdict nobody reads.
                // `phycmd` is what the PHY says it decoded and `expcmd` what this row expected it to decode:
                // the two fields that turn "FAIL" into "the master sent a packet command", which is the
                // difference between a 2-minute fix and an evening in GTKWave.
                $display("RESULT %0d FAIL exp=%h seen=%h ack=%b abort=%b tmo=%b noclk=%b reset=%b busy=%b waited=%0d retry=%0d state=%0d phycmd=%h expcmd=%h why=%0s", n, exp, acc_rdata, acc_ok, acc_abort, acc_timeout, err_no_clock, err_reset, h_busy, acc_waited, acc_retry, state, seen_cmd, exp_cmd, why);
            end
        end
    endtask

    // ---------------------------------------------------------------------------------------------
    // Main.
    // ---------------------------------------------------------------------------------------------
    reg [8*256:1] hex_path;
    reg [8*256:1] vcd_path;
    integer       have_vcd;
    reg           check_cmd;          // does this row's last access have an expected encoding?
    reg  [7:0]    exp_cmd;            // the TX CMD byte the PHY must have decoded on that access
    reg           has_exp;
    reg  [7:0]    addr, wdata, exp;
    reg           op;
    reg  [3:0]    fault;
    reg  [2:0]    pre, exp_err;

    initial begin
        pass_n = 0; fail_n = 0;
        if (!$value$plusargs("hex=%s", hex_path)) begin
            $display("ulpi_link_tb: +hex=<vectors.hex> required (see tools/ulpi_rtl_sim.py)");
            $fatal(2);
        end
        if (!$value$plusargs("nvec=%d", nvec)) begin
            $display("ulpi_link_tb: +nvec=<count> required");
            $fatal(2);
        end
        have_vcd = $value$plusargs("vcd=%s", vcd_path);
        if (have_vcd) begin
            $dumpfile(vcd_path);
            $dumpvars(0, ulpi_link_tb);
        end

        $readmemh(hex_path, vecs);

        sys_rst = 1;
        repeat (10) @(posedge sys_clk);
        sys_rst = 0;
        repeat (60) @(posedge sys_clk);          // the boot counter has to expire before any access

        for (i = 0; i < nvec; i = i + 1) begin
            exp_err = vecs[i][2:0];
            pre     = vecs[i][5:3];
            fault   = vecs[i][9:6];
            op      = vecs[i][10];
            addr    = {2'b00, vecs[i][16:11]};
            wdata   = vecs[i][24:17];
            exp     = vecs[i][32:25];
            has_exp = vecs[i][33];

            apply_pre(pre);
            apply_fault(fault);
            clear_witnesses();          // TB_HOLE: a row may not pass on the previous row's decode

            // Two rows have no encoding to check: a PHY held in reset decodes nothing, and the reset-mid row
            // decodes a command whose transaction is then destroyed, so its `seen_cmd` belongs to an access
            // that no longer exists.
            check_cmd = (pre != P_PHY_RESET) && (fault != F_RESET_MID) && (fault != F_RESET_MID_NOCLK);
            exp_cmd   = 8'h00;
            drove_after_reset = 1'b0;
            if (fault == F_RESET_MID_NOCLK) begin
                // Decision 6's *other* half, the one `reset_mid_rd` cannot see. That row resets a domain
                // that is still being clocked, so the FSM takes the clear itself and the relock tail never
                // matters. Here the clock goes away first: a PHY reset asserted against a frozen `cd_usb`
                // is a reset the FSM never observes, and when CLKOUT returns the FSM would simply resume a
                // transaction whose bridge state no longer exists -- driving a TX CMD into a PHY that has
                // already forgotten it. `UlpiClear`'s `relock` tail is what makes the first edge back an
                // edge with `clear` high, and this is the vector that would notice its absence.
                fork
                    do_access(1'b1, addr, 8'h00);
                    begin
                        // Freeze it *while it is driving*, not N cycles into the access: a register read is
                        // over in five cd_usb cycles, so a counted delay freezes whatever happens to be
                        // there -- which is what this row originally did, freezing an idle master and
                        // passing for the wrong reason. The wait is event-driven: it fires in the
                        // delta where the pad enable rises, which is by construction the cycle the master
                        // holds the command byte, and `driving` is therefore 1 when the clock stops.
                        // **`oe` AND a nonzero byte**, not `oe` alone, and bug 3 is why: since the Link
                        // drives the ULPI idle level whenever it owns the bus (ULPI 1.1 2.3.2, `idle_now`
                        // in pano_ulpi.py), `|dut_oe` is true in EVERY idle cycle, so the original trigger
                        // froze the clock the moment the master got the bus -- with the command not yet on
                        // the pads, and the row silently measuring an idle master. `tools/ulpi_mutate.py
                        // no_relock_tail` went BLIND exactly that way, and this is the general lesson:
                        // once `oe` is a level and not an event, every trigger written as "the enable rose"
                        // has to be re-read. A pattern is the event.
                        //
                        // And a comment warning nobody to read, in this file, by me, on the day this broke:
                        // the first edit here replaced the `wait` line with this commentary and left NO wait
                        // statement behind, so the clock froze the instant the fork started, with the master
                        // still in IDLE. The symptom was not an error; it was `no_relock_tail` going BLIND and
                        // the row still PASSING. A fork branch that does nothing looks exactly like a row
                        // whose mechanism held.
                        wait (dut_oe[0] && (dut_o != 8'h00));
                        usb_clk_en = 1'b0;                   // C12 dies underneath the transaction
                        repeat (60) @(posedge sys_clk);      // cd_sys counts; cd_usb is frozen mid-byte
                        phy_reset = 1'b1;                    // the operator's recovery, per the failure tree
                        repeat (30) @(posedge sys_clk);
                        phy_reset = 1'b0;
                        // From here the pad must be quiet, and the interesting part is that it has to go
                        // quiet *combinationally*, with no cd_usb edge to do it: `clear` is a cd_sys signal
                        // and the idle level is a combinational function of it. With the relock tail, `oe`
                        // drops the cycle `phy_reset` does. Without it, the frozen `driving` keeps the
                        // command byte on the bus in front of a PHY that has just forgotten the transaction
                        // that put it there.
                        watch_oe = 1'b1;
                        repeat (10) @(posedge sys_clk);
                        usb_clk_en = 1'b1;                   // the clock returns: the frozen FSM's first edge
                        repeat (400) @(posedge sys_clk);
                        watch_oe = 1'b0;
                    end
                join
                repeat (200) @(posedge sys_clk);
            end else if (fault == F_RESET_MID) begin
                // The only fault whose content is *when*: the clock is taken away from an access that has
                // already started. What must come back is `busy`, and what must latch is `err_reset`; the
                // access itself is expected to die, and a Link that reports success here is the bug.
                fork
                    do_access(1'b1, addr, 8'h00);
                    begin
                        repeat (12) @(posedge usb_clk);
                        phy_reset = 1'b1;
                        repeat (30) @(posedge sys_clk);
                        phy_reset = 1'b0;
                    end
                join
                repeat (200) @(posedge sys_clk);     // let relock expire so the next row starts clean
            end else begin
                do_access(op, addr, wdata);
                // Remember what the PHY decoded for *this* access: the read-back below overwrites `seen_cmd`.
                // **`(pre != P_PHY_RESET)` and not `1'b1`**, which is a bug this file had since it was
                // written and which only became visible when `clr_seen` started clearing the witnesses per
                // row: a PHY held in reset decodes NOTHING, so the main loop had already decided this row
                // has no encoding to check -- and this line overrode that. It passed anyway, on the
                // previous row's `seen_cmd`, which is precisely the hole (`TB_HOLE`) the clear closes. Two
                // blind spots were covering for each other, and the second one only showed up when the
                // first was fixed.
                check_cmd = (pre != P_PHY_RESET); exp_cmd = op ? (`ULPI_BYTE_RD | addr)
                                                              : (`ULPI_BYTE_WR | addr);
                if (!op) begin
                    // A write vector's `exp` is the read-back, per the TSV's column definition: a write that
                    // "succeeded" without changing the register is the failure the row exists for, and only
                    // a read can tell.
                    do_access(1'b1, addr, 8'h00);
                    check_cmd = (pre != P_PHY_RESET); exp_cmd = `ULPI_BYTE_RD | addr;
                end
            end

            check_row(i, has_exp, exp, exp_err, fault, pre, op);
        end

        $display("ulpi_link_tb: %0d vectors, %0d passed, %0d failed", nvec, pass_n, fail_n);
        if (nxt_cycles_too_long)
            $display("ulpi_link_tb: note: the PHY model held nxt beyond its own bound");
        // Status is the failure count. `check.sh` judges this suite by exit code, and a suite that prints
        // FAIL and exits 0 is a suite that will be reported green tomorrow.
        if (fail_n != 0) $fatal(1);
        $finish;
    end

    // A wall for the whole run: if the loop above ever deadlocks, this ends the sim nonzero instead of
    // leaving the caller to time out.
    initial begin
        #200_000_000;
        $display("ulpi_link_tb: GLOBAL TIMEOUT after 200 ms of sim time -- the run deadlocked");
        $fatal(1);
    end

endmodule
