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
    // The two levels the wake ticket adds to the Link's interface: `ulpi_wake`'s stored bit as it arrives
    // from the CSR rules, and `ulpi_stp_drive`'s. Defaults are the reset state of the CSRs, so a row that
    // never touches them sees exactly what every pre-wake spin saw.
    reg         tb_wake = 1'b0, tb_stp_drive_en = 1'b1;
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
    wire        dut_stp_o, dut_stp_oe, dut_stp_pad, dut_stp_driving;
    // What the PHY presents on C12. `usb_clk` above is the PHY's *reference*, which in Low Power Mode keeps
    // running while `phy_clk` goes flat, and `cd_usb` is C12 and not the reference: a DUT clocked by the
    // reference keeps working through a suspend and the wake vectors would then test a Link that was never
    // suspended. Every `@(posedge usb_clk)` still in this file is a PHY-domain delay used while the PHY is
    // awake, which is the case for every pre-existing row; the LPM rows wait on `phy_clk` explicitly.
    wire        phy_clk;
    wire        in_lpm, lpm_false_exit;
    wire [15:0] lpm_ticks;

    // The `stp` pad, resolved: the Link's tri-state, the PHY's LPM hold, and the board pull-up TODO (e) asks
    // about. Two drivers and a weak one on one net is what Verilog resolves, and `x` from two strong drivers
    // is the outcome a check must not be able to mistake for a 1.
    wire        stp_net;
    reg         tb_stp_drive = 1'b0;    // the TB's own driver, for TODO (e)'s two states
    reg         tb_stp_oe    = 1'b0;
    // Two real tri-states on one net, not a mux: a mux cannot express the one thing worth expressing here,
    // which is *both drivers active with different levels* (`x`, and a check must not be able to mistake that
    // for a 1). Neither drives by default, and `stp_drive_en=0` with the TB quiet leaves the net floating --
    // `z` into `ulpi_stp_pad`, which is the honest answer for TODO (e) on a board no pull-up has been
    // measured on, and better than a `1` this file invented.
    assign stp_net = dut_stp_oe ? dut_stp_o     : 1'bz;
    assign stp_net = tb_stp_oe  ? tb_stp_drive  : 1'bz;
    // There is deliberately NO `pullup(stp_net)` here and no `stp_pullup` knob either: `stp` is a Link output,
    // a suspended PHY does nothing to this net, and whether the *board* pulls it is TODO (e) -- a question for
    // `ulpi_stp_pad` on silicon, not for a testbench to assume. The `chk_bits` helper prints `bin(...)`, so a
    // floating pad shows up in the evidence as such instead of as a level nobody measured.

    ulpi_link dut (
        .sys_clk(sys_clk), .usb_clk(phy_clk), .sys_rst(sys_rst),
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
        .ulpi_stp_o(dut_stp_o), .ulpi_stp_oe(dut_stp_oe), .ulpi_stp_i(stp_net),
        .ulpi_dir(phy_dir), .ulpi_nxt(phy_nxt),
        .wake(tb_wake), .stp_drive(tb_stp_drive_en),
        .stp_pad(dut_stp_pad), .stp_driving(dut_stp_driving)
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

    // T_START_CYCLES: 600 cycles = 10 us of a 60 MHz reference, against the datasheet's 2.25 ms minimum.
    // Short on purpose (a resume row would otherwise spend 3.5 ms of sim per wake) and named, because the
    // number is a modelling budget and not a measurement: `ulpi_phy_model.v`'s default IS the cited value, so
    // a TB that forgets this override runs the datasheet. The vectors' `wake` rows wait on the clock returning
    // rather than on a counted delay, so shortening it changes cost and not meaning.
    ulpi_phy_model #(.T_START_CYCLES(600)) phy (
        .clk(usb_clk), .phy_reset(phy_reset),
        .link_data(dut_o), .link_oe(dut_oe[0]),
        // The resolved net, which is the only thing a real PHY's data receiver has. Wiring the model to
        // `dut_o`/`dut_oe` alone is what let this suite report 33/33 green against a Link that tri-states the
        // bus and leaves the PHY's own answer for the PHY to decode as a command (bug 3, 2026-11-01).
        .bus(ulpi_data),
        .knop_ignore_float(1'b0),      // this PHY decodes what it samples; the board does, so the model must
        .phy_clk_out(phy_clk), .in_lpm(in_lpm), .lpm_false_exit(lpm_false_exit),
        .lpm_ticks(lpm_ticks),
        .phy_data(phy_data), .phy_dir(phy_dir), .phy_nxt(phy_nxt), .phy_stp(stp_net),
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
               F_RESET_MID_NOCLK = 9, F_WAKE_FALSE = 10;

    // The freeze row's witness (see the sequence in the main loop): did the Link put anything on the data
    // pad *after* the host was told the access was dead? `dut_oe` is the TSTriple's own enable, so this is
    // the pad, not the FSM's opinion about it.
    reg watch_oe, drove_after_reset;
    localparam P_READY = 0, P_PHY_RESET = 1, P_HUB_RELEASED = 2, P_HUB_ASSERTED = 3, P_CLK_OOB = 4,
               P_RESET_PULSED = 5, P_LPM = 6, P_RESUME = 7;

    // `P_LPM`: Low Power Mode, entered the way the board entered it on 2026-11-01 -- a real RegWrite of
    // Function Control that clears `SuspendM`, through this same Link, and then a wait for the PHY to stop
    // the clock. It is entered by a *transaction* and not by poking the model, because "the write that
    // suspends the PHY still gets its ack" is itself the claim of the `suspend_enter` row, and a `pre` that
    // conjured the state would test nothing about how the state is arrived at.
    // `P_RESUME`: the same, plus `wake` held. The clock comes back T_START later, i.e. DURING the row's own
    // access, which is the state §6.1.9.2's figure draws: the Link holding `stp`, the PHY running its
    // start-up, and a master that is frozen mid-byte on a clock that no longer exists.
    //
    // The last two values of the 3-bit `pre`. A 9th would need the runner's word packing widened, which is
    // why §3.9.4's *false* resume is a `fault` (`F_WAKE_FALSE`) and not a third pre-state: it is a sequence
    // that happens while the access runs, which is exactly what `fault` already means here (`F_RESET_MID`
    // takes the clock away mid-access; this one offers a resume and snatches it back).
    reg         pre_witness_bad;
    reg [8*140:1] pre_why;

    // `P_CLK_OOB` = the meter says the clock is out of band while the domain is still being clocked. That is
    // NOT a fiction invented to give `err_no_clock` a row: it is the one way this board loses its clock, and
    // it was measured doing it (`bench/2026-11-01_access_without_clock/`: our 24 MHz off -> the PHY free-runs
    // at ~29.45 MHz -> `probe_clk_ok`=0, `probe_fm_ulpi_value`=29413, and a RegRead still returns 0x24 with
    // `err_no_clock` latched). 3 bits of `pre`, 5 used: a 6th still fits.

    localparam E_NONE = 0, E_TIMEOUT = 1, E_ABORT = 2, E_NO_CLOCK = 3, E_RESET = 4, E_HUNG = 5;

    // `hung` is a new word in a vocabulary that was built for failures that *report*: abort, timeout, reset.
    // It exists because Low Power Mode is the first state on this board where the failure is that nothing
    // happens at all. The TB's own bound (`ACCESS_BOUND`, 4 ms of `cd_sys`) is what observes it, and the
    // claim is deliberately about the absence: the access neither acks nor latches an error, because the
    // choreography it would run on has no clock. `docs/NOTES.md`'s W12 lesson is the precedent for naming
    // this rather than reusing `nxt_timeout`: an error that "should" have fired did not, and a suite that
    // files a silence under a noise is a suite that sends the engineer to the wrong register.

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
                //
                // `err_no_clock` is NOT in this condition, and it used to be. That was the same fiction as
                // `apply_pre`'s `clk_ok = 0`: it assumed a low clock meter means the far side never starts,
                // so no ack can come. The board says an ack DOES come -- with the PHY free-running at
                // ~29.45 MHz and `probe_clk_ok`=0, a RegRead returned `0x24` with `done`=1 and `busy`=0
                // (`bench/2026-11-01_access_without_clock/`). Bailing here would have made `freerun_rd`
                // report a hang on a link that answered, which is a false red of exactly the kind this suite
                // is built to refuse; the row's own `exp_err` is what proves the meter was reported.
                if (err_reset) guard = ACCESS_BOUND;
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
        integer g;
        begin
            phy_reset = 0; usb_clk_en = 1; clk_ok = 1; linestate = 2'b01;
            // Every row starts with the wake released and the `stp` driver enabled -- the reset state of
            // `ulpi_wake` / `ulpi_stp_drive`, so a row that never touches them sees exactly the pad state
            // every pre-wake spin had.
            tb_wake = 1'b0; tb_stp_drive_en = 1'b1; tb_stp_oe = 1'b0;
            // Knob-off here is NOT redundant with the `apply_fault(fault)` each row calls afterwards: the LPM
            // pre-states WRITE THROUGH THE LINK (they suspend with a real RegWrite of Function Control), and
            // a fault left over from the previous row would act on a precondition the vector file never
            // asked for. With `ignore_write` carried in from `igwrite_wr`, the suspending write is dropped,
            // the PHY never sleeps, and every row below fails for a reason nobody wrote. The RTL did not
            // show this hole on its own -- the row ahead of `suspend_deaf_rd` happens not to set that knob,
            // which is luck and not a test -- the python model found it via `ignore_write_knob_dead`, and
            // both models now clear the knobs before building a pre-state.
            knop_stall_nxt = 0; knop_wrong_cmd = 0; knop_rev_bus = 0; knop_ignore_write = 0;
            knop_abort_n = 0; knop_abort_forever = 0;
            pre_witness_bad = 1'b0; pre_why = "";
            // Row isolation, wake edition. Every row before this one ran on a PHY that could answer, so no
            // pre-state ever had to recover the link; the LPM rows leave it suspended and with `bridge.busy`
            // set by an access that has no clock to finish. Recovering is the same thing an operator is told
            // to do (`.scratch/ulpi-wake/spec.md`'s failure tree): assert `stp`, wait for the clock, then
            // take the PHY reset that releases the bridge -- and it is done HERE, before the row that follows
            // asks anything, because a row that starts behind a stuck `busy` does not fail on its own terms.
            // SCOPE, and it is a measured scope, not a stylistic one: this recovery runs only for the rows
            // whose pre-state is a suspended PHY. The first version also fired on a stuck `h_busy`, which is
            // a sensible-looking thing to want and which DELETED a witness -- `tools/ulpi_mutate.py`'s
            // `no_dst_req_withdraw` is noticed precisely because the row before `freerun_rd` leaves a stale
            // request asserted, so a pre-state that resets any busy before every row cleans up the evidence
            // and reports that mutation BLIND on a tree where the bug is present (measured, both ways, on
            // this file). Recovery therefore happens in the one state that genuinely cannot be left alone --
            // a sleeping PHY poisons every later row and is not itself a symptom -- and a stuck `busy`
            // anywhere else stays as visible as it was before this ticket. Generalised, because it is not
            // about this row: **isolation is a cure, and a cure applied before every measurement also removes
            // the disease from the record.**
            if ((pre == P_LPM) || (pre == P_RESUME)) begin
                if (in_lpm === 1'b1) begin
                    tb_wake = 1'b1;
                    g = 0;
                    while ((in_lpm === 1'b1) && (g < 4000)) begin @(posedge usb_clk); g = g + 1; end
                    tb_wake = 1'b0;
                    if (in_lpm === 1'b1) begin
                        pre_witness_bad = 1'b1;
                        pre_why = "row isolation FAILED: a held stp did not bring the clock back either";
                    end
                end
                // The bridge is stuck on the access that had no clock, and a PHY reset is what releases it
                // (`bridge.err_lost`). `UlpiClear` then holds `clear` for its whole relock tail -- 10 ms by
                // default -- and issuing the next row's write into that window is how row 39 came back
                // `acc_ok=0` on its first run: the write was never started, so the row's own `pre` looked like
                // a PHY that would not suspend. Wait for the clear to drop, with a bound above `relock_ms`.
                phy_reset = 1'b1; repeat (30) @(posedge sys_clk); phy_reset = 1'b0;
                g = 0;
                while (ulpi_clear && (g < 800_000)) begin @(posedge sys_clk); g = g + 1; end
                repeat (20) @(posedge sys_clk);
            end
            if ((pre == P_LPM) || (pre == P_RESUME)) begin
                // Suspend, through the bus and not the model's internals: `0x41 -> 0x01` clears `SuspendM`
                // and leaves `XcvrSelect`=01b (full speed) exactly as POR set it. This is the byte the 2026-
                // 11-01 incident wrote, and the reason the row `suspend_enter` exists separately: a `pre`
                // that suspends silently would leave "did the suspending write ever complete?" unanswered.
                // The spec's `suspend_enter` row, which cannot be a vector row and should not pretend: a
                // `wr` vector reads the byte back, and there is no clock left to read it with. So the enter
                // claim lives here, as two witnesses rather than as a green row -- (a) the suspending write
                // COMPLETED, which is the datasheet's "the PHY stops its clock when it has retired this
                // transaction" and not a PHY dying mid-write, and (b) the clock then really stopped.
                do_access(1'b0, 6'h04, 8'h01);   // WRITE, not read: `do_access(rw, ...)` and 0 is a write
                if (!acc_ok) begin
                    pre_witness_bad = 1'b1;
                    pre_why = "the write that clears SuspendM never acked: the PHY stopped its clock mid-write";
                end
                // Wait for the delivered clock to actually go flat. Waiting on `in_lpm` and then *asserting*
                // that we waited is the whole difference between a row that tests a suspended PHY and a row
                // that tests a PHY which never suspended -- the second shape is green, and it is what a
                // mutation of the model's clock gate would leave behind.
                g = 0;
                while ((in_lpm !== 1'b1) && (g < 4000)) begin @(posedge usb_clk); g = g + 1; end
                if (in_lpm !== 1'b1) begin
                    pre_witness_bad = 1'b1;
                    pre_why = "the PHY never entered Low Power Mode: there is no suspended PHY in this row";
                end
                // The meter, honestly: with C12 stopped, spin 1's clock meter reads nothing, so `clk_ok` is
                // 0 here because the clock really has stopped -- not because a spec note said reset stops it
                // (ticket 09 deleted that fiction for the reset pad; this one is the suspend, where the board
                // and the datasheet agree). It goes back on the clock's return, see the block below.
                clk_ok = 1'b0;
                if (pre == P_RESUME) begin
                    // §3.9.3's exit, then §6.1.9.2's sequence: assert `stp`, wait out T_START, and only THEN
                    // ask for the byte -- with `wake` released first, which is what ticket 03's bench
                    // discipline does and what `UlpiCsrRules` forces in hardware (a `go` while `wake_applied`
                    // is refused).
                    //
                    // The first draft issued the access immediately and waited for it to complete after the
                    // resume. It cannot work, and the reason is a property of the design worth writing down
                    // rather than a TB slip: measured through the whole resume, `bridge_req_x` and
                    // `bridge_req_was` are BOTH 1 when `cd_usb` starts ticking again, because the level
                    // crossed while the domain was frozen and the edge detector needs an edge. The request is
                    // simply gone, `busy` stays asserted, and no amount of waiting changes that. A suspended
                    // clock loses a request; it does not queue one. That is the mechanism under decision C's
                    // converse, and it is why the wake's promise is narrower than it first sounds: **the wake
                    // recovers the LINK, not a transaction that was in flight when the clock went away.**
                    tb_wake = 1'b1;
                    g = 0;
                    while ((in_lpm === 1'b1) && (g < 4000)) begin @(posedge usb_clk); g = g + 1; end
                    if (in_lpm === 1'b1) begin
                        pre_witness_bad = 1'b1;
                        pre_why = "stp was held past T_START and the PHY still did not return its clock";
                    end
                    repeat (60) @(posedge usb_clk);      // §6.1.9.2's idle window, clock up
                    // §6.1.9.3's sentence, pressed while it is still cheap: "if the Link requests a Resume
                    // while the PHY is in Reset, the PHY will complete the Resume handshaking and then enter
                    // Reset". That is only a statement about the ORDER unless `wake` reaches the pad while
                    // `clear` is asserted, and `clear` is exactly what `UlpiClear` drives from `phy_reset` --
                    // in `cd_usb`, frozen, which is why the pad equation lets `wake` past it. So press it:
                    // hold the reset, and require the pad to stay up. The first draft of the ticket's
                    // mutation list had `wake_blocked_by_clear` in it with no row to catch it, and a fault
                    // nobody can observe is not a tested decision -- this eight-line press is what makes
                    // that mutation's expectation honest rather than hopeful.
                    phy_reset = 1'b1;
                    repeat (40) @(posedge sys_clk);
                    if (stp_net !== 1'b1) begin
                        pre_witness_bad = 1'b1;
                        pre_why = "with phy_reset asserted, `wake` did not hold the pad high: the pad equation lets `clear` win, so §6.1.9.3's resume-during-reset is unreachable and the 2026-10-31 wedge returns any time the recovery wants a reset too";
                    end
                    phy_reset = 1'b0;
                    // The reset pulse costs a relock tail, and the row's access must not land inside it: that
                    // would report our own clear as the answer and call it a wake.
                    g = 0;
                    while ((ulpi_clear === 1'b1) && (g < 800000)) begin @(posedge sys_clk); g = g + 1; end
                    if (ulpi_clear === 1'b1) begin
                        pre_witness_bad = 1'b1;
                        pre_why = "the relock tail after §6.1.9.3's press never expired";
                    end
                    tb_wake = 1'b0;                     // release, and the link is usable again
                    repeat (60) @(posedge usb_clk);
                end
            end
            if (pre == P_PHY_RESET) begin
                // **`clk_ok` stays 1 here, and that is a measurement, not an oversight.** Until ticket 09 this
                // line read `phy_reset = 1; clk_ok = 0;` because AN19.17 §4.2 says asserting reset stops
                // CLKOUT. It does not stop it on this part: with the pad at the level that makes the PHY
                // ignore every command, `probe_fm_ulpi_value` reads 60000 exactly and `probe_clk_ok` reads 1,
                // in every reading and in both polarity settings (`bench/2026-11-01_ulpi_rst_polarity/`), and
                // `ulpi_cnt_no_clock` has read 0 on four spins. Driving it low here made `phyreset_rd` pass
                // by *constructing* the failure it claimed to observe -- the model agreed with a board that
                // was never asked. What really ends that access is our own `UlpiClear` + `bridge.err_lost`.
                phy_reset = 1;  clk_ok = 1;
            end
            // The clock-absent case, now on the side of the interface that can actually see it: `cd_usb`
            // keeps running (a free-running PHY still clocks it) and only the METER says no. `usb_clk_en`
            // deliberately stays 1 -- stopping it would test `reset_mid_noclk_rd`'s domain, not this one.
            if (pre == P_CLK_OOB)   clk_ok = 0;
            // The two hub rows are the W11 pin, not a PHY register: what the Debug register reports is the
            // line state, which is the only way to see a hub reset work before enumeration.
            if (pre == P_HUB_RELEASED) linestate = 2'b01;
            if (pre == P_HUB_ASSERTED) linestate = 2'b00;
            // A PULSE, not a hold -- the difference is the row. `P_PHY_RESET` asks what the PHY answers WHILE
            // it is in reset (nothing); `P_RESET_PULSED` asks what SURVIVES the pulse, which is the question
            // ticket 10 was opened for because both PHY models had just been changed to answer "the register
            // array" on the strength of a bench measurement no vector covered (`bench/2026-11-01_reset_array/`).
            // `clk_ok` is left at 1 on purpose: a reset pulse does not stop CLKOUT on this part (D14), so
            // blinking the meter here would re-introduce the very fiction ticket 09 removed and make the row
            // judge the meter instead of the array. 5 is the last value a 3-bit `pre` holds -- the 6th does not
            // fit, and `parse_vectors` in tools/ulpi_vectors.py is the file that would notice first.
            if (pre == P_RESET_PULSED) begin
                // Long by ULPI's own rule (USB3300 wants >=1 us asserted), and grossly long in sim cycles so
                // that no reader has to wonder whether the model saw it.
                @(posedge sys_clk); phy_reset = 1;
                repeat (200) @(posedge sys_clk);
                @(posedge sys_clk); phy_reset = 0;
            end
            // Wait for the clear to release. The boot and relock counters ARE the mechanism, so starting a
            // vector before `ulpi_clear` drops is testing a state the board can never be in.
            guard_clear;
        end
    endtask

    // `clk_ok` is an INPUT to the Link (spin 1's meter lives in cd_sys, outside this DUT), so during a
    // suspend the TB owns it. It goes high again on the edge the PHY's clock returns -- `in_lpm` falling is
    // the model's own statement that CLKOUT is back -- and only for the rows that stopped it: for every
    // other row `in_lpm` never rises, so this block never fires and the pre-existing `clk_ok` handling is
    // untouched. A `clk_ok` that stayed low after a successful resume would latch `err_no_clock` on the NEXT
    // row too, and the next row would then be judging a clock that came back fine.
    always @(negedge in_lpm) clk_ok = 1'b1;

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
            // A `pre` that failed to build the state it names is a red, not a note. Every LPM row above this
            // line would otherwise PASS on a board that never suspended: a read against a not-suspended PHY
            // that hangs for some *other* reason still matches `exp_err = hung`.
            if (pre_witness_bad) begin
                ok  = 0;
                why = pre_why;
            end
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
                    // The silence, and it must be a *total* silence: no ack, and none of the four latching
                    // bits. If a suspended link really did latch `nxt_timeout`, the right row would be
                    // `nxt_timeout` and this one would be deleted -- keeping the negation explicit is what
                    // makes the difference visible the day the gateware changes.
                    // No ack, and no CHOREOGRAPHY error -- but `err_no_clock` is allowed, and required to be
                    // read separately rather than folded in. It is the meter's own sentence ("this ran on a
                    // clock we cannot vouch for"), it latches from `clk_ok`, and with C12 stopped the meter is
                    // telling the truth. `freerun_rd` already established that the flag is about OUR clock
                    // and not about the far side; demanding it clear here would demand that the board lie.
                    // What must stay clear are `dir_abort`, `nxt_timeout` and `reset`: each of those is a
                    // statement that some cd_usb choreography ran, and on a stopped clock none can.
                    E_HUNG:     ok = !acc_ok && !err_dir_abort && !err_nxt_timeout && !err_reset;
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
                end else if ((exp_err != E_HUNG) && h_busy) begin
                    // Half of what these rows test is that the Link comes back: an error bit latched behind
                    // a permanently asserted busy is a hang the host cannot distinguish from a slow access
                    // (spec testing decision 5). The `ok = 0` is the whole line and it was missing until
                    // 2026-10-31: the message existed, `why` was assigned, and the row still printed PASS.
                    // What reddened the mutation was the *next* vector, blocked behind the stuck busy -- a
                    // true failure reported in the wrong row, which is how this became a verdict instead of
                    // a rumour. See `tools/ulpi_mutate.py`'s `lost_keeps_busy`.
                    //
                    // `E_HUNG` is the one exemption in this file, and the exemption is the definition: a
                    // suspended link holding `busy` with no ack is what "the clock is gone" looks like from
                    // `cd_sys`, so reddening it would reddish the only row that models the wedge correctly.
                    // What keeps that exemption from being a free pass is `pre_witness_bad` above -- the row
                    // must have a PHY that really suspended, or the silence means nothing -- plus the row's
                    // own `!err_*` negations, so a silence caused by anything else has to be a silence with
                    // no error bits and no ack on a link the `pre` proved was asleep.
                    ok  = 0;
                    why = "error latched but busy never cleared";
                end
                // A hung row must ALSO still be busy: `!acc_ok` with `busy` low and no error bit is a
                // different claim -- the bridge dropped the request without ever starting it, which is
                // bug 4's shape and deserves its own red rather than a pass under "silence".
                if (ok && (exp_err == E_HUNG) && !h_busy) begin
                    ok  = 0;
                    why = "no ack and no error, but busy already clear: the access was dropped, not suspended";
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
                end else if ((exp_err != E_NONE) && !(
                        (exp_err == E_ABORT    && err_dir_abort)    ||
                        (exp_err == E_TIMEOUT  && err_nxt_timeout)  ||
                        (exp_err == E_NO_CLOCK && err_no_clock)     ||
                        (exp_err == E_RESET    && err_reset))) begin
                    // A retry that worked still has to report the aborts: "passed after 3 aborts" and
                    // "passed cleanly" are different facts (spec decision 12). It was written for the abort
                    // rows only (`exp_err == E_ABORT`) and is now any named error, because `freerun_rd` is a
                    // row whose byte is RIGHT and whose whole content is the bit that must ride with it; a
                    // check that only enforced the bit on rows with no byte would let that row pass on the
                    // byte alone, which is the difference between "reported the clock" and "got an answer".
                    ok  = 0;
                    why = "byte correct, the named error not reported";
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
            // Three kinds of row have no command encoding to check: a PHY held in reset (decodes nothing),
            // the two reset-mid rows (the transaction they started no longer exists), and -- new with the
            // wake ticket -- every row that runs against a SUSPENDED PHY. That one is `pre == P_LPM`: the
            // access never reaches the pads at all, so `seen_cmd` is still the byte from the *suspending*
            // write (`0x84`), and a row about a silent link would be reddened by the very silence it exists
            // to prove. `P_RESUME` is NOT in this list, and that asymmetry is the point: by the time that
            // row's access runs, the clock is back and the PHY really did decode our `0xC0` -- checking it
            // there is a claim, not a courtesy.
            check_cmd = (pre != P_PHY_RESET) && (pre != P_LPM)
                     && (fault != F_RESET_MID) && (fault != F_RESET_MID_NOCLK);
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
            end else if (fault == F_WAKE_FALSE) begin
                // ULPI 1.1 §3.9.4: if `stp` is deasserted before the resume completes, the PHY does not
                // complete it -- it goes back to sleep. The row therefore expects the SAME outcome as
                // `suspend_deaf_rd` (a hung access), and the witness that makes it a §3.9.4 row and not a
                // duplicate is `lpm_false_exit`, checked below rather than in the vector file: the 8 columns
                // have no slot for a PHY-internal witness, and inventing one to hold a single sentence is
                // how a contract file becomes a dump.
                fork
                    do_access(1'b1, addr, 8'h00);
                    begin
                        repeat (200) @(posedge usb_clk);      // assert a resume...
                        tb_wake = 1'b1;
                        repeat (200) @(posedge usb_clk);      // ...and release it well inside T_START(600)
                        tb_wake = 1'b0;
                        repeat (2000) @(posedge usb_clk);     // long enough that a PHY which resumed would
                    end                                       // have answered by now
                join
                if (lpm_false_exit !== 1'b1) begin
                    pre_witness_bad = 1'b1;
                    pre_why = "stp rose and fell inside T_START and the model never saw 3.9.4's false resume";
                end
                if (in_lpm !== 1'b1) begin
                    pre_witness_bad = 1'b1;
                    pre_why = "the false resume left the PHY AWAKE: 3.9.4 says it goes back to sleep";
                end
                tb_wake = 1'b0;
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
                // Same three exclusions as the main loop's line above -- and this is the line that actually
                // sets the expected byte, so gating only the other one changed nothing (measured: three
                // wake rows red with `phycmd=84`, the *suspending* write's command, on rows whose own
                // command never reached a PHY with no clock).
                check_cmd = (pre != P_PHY_RESET) && (pre != P_LPM);
                exp_cmd   = op ? (`ULPI_BYTE_RD | addr) : (`ULPI_BYTE_WR | addr);
                if (!op) begin
                    // A write vector's `exp` is the read-back, per the TSV's column definition: a write that
                    // "succeeded" without changing the register is the failure the row exists for, and only
                    // a read can tell.
                    do_access(1'b1, addr, 8'h00);
                    check_cmd = (pre != P_PHY_RESET) && (pre != P_LPM);
                    exp_cmd   = `ULPI_BYTE_RD | addr;
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
