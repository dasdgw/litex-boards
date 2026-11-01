// ulpi_phy_selfcheck.v — does the PHY model still have the capability bug 3 needed it to have?
//
// WHY THIS FILE EXISTS
//
// On 2026-10-31 the board said: a RegRead of Function Control or Scratchpad succeeds and leaves the PHY
// unable to answer the next access (ticket 05, `bench/2026-10-31_ulpi_t18_poison_sweep.tsv`). The offline
// suite said 33/33. Both were telling the truth: the suite was green because `ulpi_phy_model.v` decoded
// commands as `link_oe && link_data` — it was handed the Link's output enable and believed it. A PHY has no
// such signal. It samples a LEVEL and decodes bits 7:6 (USB3300 Table 6-4), and that is the entire mechanism
// of the bug: ULPI 1.1 §2.3.2 ("the Link continuously drives the data bus to 00h during idle") exists
// because USB3300 §6.1.5.1 ("the Link must drive a ULPI Idle (00h) on the data bus **or the USB3300 may
// decode the bus value as a ULPI command**") says what happens when it does not.
//
// The fix is `idle_now` in `pano_ulpi.py`, and `tools/ulpi_mutate.py no_idle_drive` proves the vectors now
// see it. But *nothing* would notice the model being made permissive again, and a vector file cannot test its
// own oracle: revert the model to `link_oe && link_data` and all 35 rows go green around a Link that has
// never driven an idle level, exactly as they did last week. `tools/ulpi_mutate.py` mutates the gateware;
// `tools/test_ulpi_phy_model.py` mutates the python twin. This is the third thing, and it is the only one
// that points at the Verilog model: assertions about how the model behaves, run by
// `tools/ulpi_rtl_sim.py` before a single vector is loaded, needing no generated netlist and no board.
//
// Every check is a sentence about a datasheet, not about our FSM. If one fails, the suite is blind again and
// the green it reports afterwards means nothing.
//
//   run: iverilog -g2012 -I. -o sc ulpi_phy_selfcheck.v ulpi_phy_model.v && ./sc   (exit 0 = all pass)

`timescale 1ns/100ps

module ulpi_phy_selfcheck;

    localparam integer TX_PERIOD = 40;      // the measured offer cadence; SC6 re-derives it

    reg         clk = 0;
    reg         rst = 1;
    reg         oe_cmd = 0;                 // the Link driving a byte on purpose (a CMD, or `stp` context)
    reg  [7:0]  drv = 8'h00;
    reg         stp = 0;
    // The thing under test, as a scenario rather than as a wire: 1 = the Link drives ULPI Idle while it owns
    // the bus (`idle_now` in pano_ulpi.py), 0 = the 2026-10-31 Link that tri-states the pads. Both are run
    // against the SAME model, because the point is the model's difference in behaviour, not ours.
    reg         idle_drive = 0;

    wire        link_oe   = oe_cmd | (idle_drive & ~phy_dir & ~dir_d);
    wire [7:0]  link_byte = oe_cmd ? drv : 8'h00;
    reg         dir_d = 0;                  // one-cycle-late `dir`, the same term `~self.dir_in` is

    wire [7:0]  bus;
    wire [7:0]  phy_data, seen_cmd, seen_wdata, unsol_byte;
    wire        phy_dir, phy_nxt, phy_oe;
    wire        unsol_tx, unsol_cmd, saw_stp, nxt_too_long;
    wire [3:0]  abort_count;
    wire        cmd_is_write;
    reg         clr_seen = 0;

    integer errors = 0;
    integer ans      = 0;
    integer n_ok   = 0;
    integer guard;

    initial forever #8.333 clk = ~clk;

    ulpi_phy_model #(.TX_NXT_PERIOD(TX_PERIOD)) phy (
        .clk(clk), .phy_reset(rst),
        .link_data(link_byte), .link_oe(link_oe),
        .bus(bus),
        .knop_ignore_float(1'b0), .clr_seen(clr_seen),
        .knop_stall_nxt(1'b0), .knop_abort_n(4'd0), .knop_abort_forever(1'b0),
        .knop_wrong_cmd(1'b0), .knop_rev_bus(1'b0), .knop_ignore_write(1'b0),
        .linestate(2'b01),
        .seen_cmd(seen_cmd), .seen_wdata(seen_wdata), .saw_stp(saw_stp),
        .abort_count(abort_count), .cmd_is_write(cmd_is_write),
        .nxt_cycles_too_long(nxt_too_long),
        .unsolicited_tx(unsol_tx), .unsolicited_cmd(unsol_cmd), .unsolicited_byte(unsol_byte),
        .phy_data(phy_data), .phy_dir(phy_dir), .phy_nxt(phy_nxt), .phy_stp(stp), .phy_oe(phy_oe));

    always @(posedge clk) dir_d <= phy_dir;

    // The pads, resolved the way the testbench resolves them: two tri-state drivers, one net. The model is
    // given `bus` and nothing else, because that is all a real PHY's receiver has.
    assign bus = link_oe  ? link_byte : 8'hzz;
    assign bus = phy_oe   ? phy_data  : 8'hzz;

    task automatic bad(input [8*160:1] msg);
        begin
            errors = errors + 1;
            $display("SELFCHK FAIL  %0s   (t=%0t)", msg, $time);
        end
    endtask

    task automatic good(input [8*160:1] msg);
        begin
            n_ok = n_ok + 1;
            $display("SELFCHK ok    %0s", msg);
        end
    endtask

    // The master's own timing, so these checks measure the model and not an invented choreography: hold the
    // TX CMD for the cycles UlpiMaster holds it, then release. What happens AFTER that release is the whole
    // difference between `idle_drive` 1 and 0.
    task automatic read_cmd(input [5:0] a);
        begin
            @(posedge clk); oe_cmd <= 1'b1; drv <= 8'hC0 | a;
            @(posedge clk); oe_cmd <= 1'b1; drv <= 8'hC0 | a;
            @(posedge clk); oe_cmd <= 1'b0; drv <= 8'h00;
        end
    endtask

    // Pulse the model's "forget the last row", so a later check cannot pass on an earlier row's decode.
    task automatic clear_seen;
        begin
            @(posedge clk); clr_seen <= 1'b1;
            @(posedge clk); clr_seen <= 1'b0;
        end
    endtask

    // Wait for the PHY's answer byte to have been on the pads. `ok` comes back 0 on a guard timeout, and a
    // timeout is itself a finding -- it means the model never entered the read at all. A `task` with an
    // output and not a `function`, because iverilog is right that a function may not consume time.
    task automatic wait_answer(output integer ok);
        integer g;
        begin
            g = 0;
            ok = 1;
            while (!phy_dir && (g < 200)) begin @(posedge clk); g = g + 1; end
            if (!phy_dir) ok = 0;
            @(posedge clk); @(posedge clk);
        end
    endtask

    // Wait for one `nxt` offer of the transmit throttle, so a phase can be stated relative to an offer and
    // not relative to simulation zero.
    task automatic sc_wait_offer(input integer bound);
        integer g;
        begin
            g = 0;
            while (!phy_nxt && (g < bound)) begin @(posedge clk); g = g + 1; end
        end
    endtask

    task automatic phy_reset_pulse;
        begin
            @(posedge clk); rst <= 1'b1; oe_cmd <= 1'b0; drv <= 8'h00; stp <= 1'b0;
            repeat (6) @(posedge clk);
            rst <= 1'b0;
            repeat (6) @(posedge clk);
        end
    endtask

    // Drive the bus idle, then reset, so the model starts a scenario the way a fixed Link starts one: with
    // 00h on the pads rather than with somebody else's leftovers.
    task automatic start_scenario(input drive_idle);
        begin
            idle_drive = drive_idle;
            phy_reset_pulse;
            repeat (4) @(posedge clk);
        end
    endtask

    // ---------------------------------------------------------------------------------------------
    initial begin
        repeat (20) @(posedge clk);

        // SC1 — the model still decodes a command the Link drives. Every check below is a capability claim,
        // and a capability claim about a model that answers nothing is noise.
        start_scenario(1'b0);
        read_cmd(6'h00);
        wait_answer(ans);
        if (ans == 0)                  bad("SC1: a driven RegRead CMD did not decode at all");
        else if (seen_cmd !== 8'hC0) bad("SC1: the model decoded the wrong TX CMD byte");
        else                         good("SC1: the model decodes a driven RegRead (seen_cmd=C0, byte answered)");

        // SC2 — THE ONE THAT MATTERS. Function Control answers 41h, whose bits 7:6 are 01b = USB Transmit,
        // and a PHY that sees only levels cannot tell that byte from a command the Link drove. Release the
        // pads the way the pre-fix Link did and the model MUST enter the unsolicited transmit. If this ever
        // reports FAIL, the model has gone back to trusting `link_oe` and every green row in this repo is
        // void — which is the state the suite was in for a week.
        start_scenario(1'b0);
        read_cmd(6'h04);                       // Function Control, POR 41h (USB3300 Table 6.3)
        wait_answer(ans);
        if (ans == 0)                  bad("SC2: the read of 04h was never answered, so nothing is being measured");
        else begin
            repeat (TX_PERIOD) @(posedge clk); // the float the board measured at ~1 us = 60+ cycles
            if (!unsol_tx)                    bad("SC2: a floating 41h decoded as NOTHING  --  the model trusts link_oe again and cannot see bug 3");
            else if (unsol_byte !== 8'h41)    bad("SC2: entered the unsolicited transmit, but not from 41h");
            else                              good("SC2: a floating 41h is decoded as a USB Transmit (bug 3 is reproducible offline)");
        end

        // SC3 — the same read with the Link doing what ULPI 1.1 §2.3.2 says. `link_oe` here is
        // `idle_drive & ~phy_dir & ~dir_d`, literally `idle_now`'s expression, so the one cycle the PHY is
        // handed the bus (§2.3.1: "neither Link nor PHY can drive the bus ... must be ignored by both") is
        // still a float. A model that poisoned THIS would punish the fix and send us chasing the board; a
        // model that poisons neither is the one that can tell the two designs apart, which is the point.
        start_scenario(1'b1);
        read_cmd(6'h04);
        wait_answer(ans);
        if (ans == 0)                  bad("SC3: the read of 04h was never answered with the Link driving idle");
        else begin
            repeat (TX_PERIOD + 10) @(posedge clk);
            if (unsol_tx)  bad("SC3: the Link drove ULPI Idle and the PHY entered a transmit anyway  --  the model cannot tell the fix from the bug");
            else           good("SC3: a Link that drives ULPI Idle after a read is NOT poisoned (the fix is accepted, sec.2.3.1's turnaround included)");
        end

        // SC4 — the cure is not "any stp". USB3300 §6.1.7: "The USB Transmit ends when the Link asserts STP
        // while NXT is asserted. (Note that the Link cannot assert STP with NXT de-asserted since the
        // USB3300 is expecting to fetch another byte from the Link in this state.)" That is the sentence that
        // explains `.scratch/ulpi-id-read/diag_cure.py` on the bench (`cure_stdout.txt` in
        // `bench/2026-11-01_ulpi_t18_poison/`), where a bare `stp` did NOT cure a poisoned PHY and a RegWrite
        // did. That is the half §6.1.7 settles; *why* the RegWrite worked is not settled (its END marker in the
        // window, or §3.8.1's "ends when the Link stops driving"), and SC5's exit-on-in-window-`stp` is a
        // mechanism, not a reproduction of that row. What no reading allows is what a model exiting on ANY `stp`
        // would have prescribed: the abort path, which the board then refused.
        //
        // SC5 runs on the same poisoned model immediately after, so the state is not manufactured twice.
        start_scenario(1'b0);
        read_cmd(6'h04);
        wait_answer(ans);
        if (ans == 0)                  bad("SC4: could not poison the model to test the cure on");
        else begin
            // SC4 — a SINGLE `stp` at an offset of 7 cycles after an offer, repeated four times. Seven, and
            // not a round number, because the offer period is 40: an offset sharing a factor with 40 walks
            // into the offer phase by accident (5 does exactly that — the first version of this check pulsed
            // every 5 cycles and so landed on every 8th offer, which is a claim about a different datasheet
            // reading than the one this row makes). 40k+7 is never 40k (the offer) and never 40k+1 (this
            // model's END window, see `nxt_window`), so the assertion is "an out-of-window stp does not
            // cure" — USB3300 6.1.7's sentence, and what `.scratch/ulpi-id-read/diag_cure.py` measured with a
            // bare `stp` on the board.
            for (guard = 0; guard < 4; guard = guard + 1) begin
                sc_wait_offer(TX_PERIOD * 3);
                repeat (7) @(posedge clk);
                @(posedge clk); stp <= 1'b1;
                @(posedge clk); stp <= 1'b0;
                repeat (3) @(posedge clk);
            end
            clear_seen;
            read_cmd(6'h00);                   // ask for the ID byte: only a model that left Tx answers
            repeat (TX_PERIOD * 2) @(posedge clk);
            if (seen_cmd === 8'hC0)  bad("SC4: an out-of-window stp ended the transmit  --  the model contradicts USB3300 6.1.7 and would prescribe the abort path as bug 3's cure");
            else                     good("SC4: a single out-of-window stp does NOT end the unsolicited transmit (USB3300 6.1.7, as measured)");

            // SC5 — and an exit exists. `stp` HELD rather than pulsed, so the testbench does not have to
            // predict an offer a cycle ahead (a Link cannot see an offer before the cycle it appears in) and
            // so this file does not bet on whether the window is the offer cycle, as 6.1.7 reads literally,
            // or the cycle after it, which is what our registered `stp` makes it (`nxt_window`). What SC5
            // guards is the failure that matters: a model with NO exit at all turns every row after the first
            // read red for the wrong reason, and the bench then chases a Link that was already fixed.
            stp <= 1'b1;
            guard = 0;
            while ((phy.state == 9) && (guard < TX_PERIOD * 3)) begin @(posedge clk); guard = guard + 1; end
            @(posedge clk); stp <= 1'b0;
            if (phy.state == 9)       bad("SC5: stp held across every offer never ended the transmit  --  the model has no exit at all");
            else begin
                clear_seen;
                read_cmd(6'h00);
                wait_answer(ans);
                if ((ans == 0) || (seen_cmd !== 8'hC0))
                                      bad("SC5: the model left the transmit but still will not answer a RegRead");
                else                  good("SC5: an in-window stp ends the transmit and a later RegRead is answered again");
            end
        end


        // SC7 — the asymmetry the whole sweep rests on: a WRITE leaves the idle command on the pads.
        // `bench/2026-10-31_ulpi_t18_poison_sweep.tsv` is 6 poisoned / 42 clean, and the 6 are reads. The
        // reason is not magic: UlpiMaster drives `tx_byte`=0 with the END marker *before* it releases the
        // bus, so the level a released pad holds is 00h. This model has to reproduce that, or SC2's "a level
        // left behind is decoded" would say every access poisons and the sweep's 42 clean rows would be
        // evidence for nothing. (This is also why the python twin's bug-3 knob gates on `rw`.)
        //
        // Driven here with `idle_drive` = 0, i.e. the pre-fix Link: after the write's END marker the pads are
        // released, and the model must NOT enter a transmit.
        start_scenario(1'b0);
        begin : write_leaves_idle
            integer g;
            @(posedge clk); oe_cmd <= 1'b1; drv <= 8'h80 | 6'h16;   // RegWrite of Scratch
            repeat (2) @(posedge clk);
            drv <= 8'h5A;                                           // the payload, in the cycle nxt is up
            @(posedge clk);
            drv <= 8'h00; stp <= 1'b1;                               // ULPI's END marker
            @(posedge clk);
            stp <= 1'b0; oe_cmd <= 1'b0;                             // and the pre-fix release: no idle drive
            g = 0;
            while (phy_nxt && (g < 10)) begin @(posedge clk); g = g + 1; end
            repeat (TX_PERIOD + 10) @(posedge clk);
        end
        if (unsol_tx)       bad("SC7: a WRITE poisoned the link — the model contradicts the sweep, whose 42 clean rows are writes and Idle-typed reads");
        else                good("SC7: a write leaves 00h on the pads, so only a read's byte can poison (the sweep's 6/42 split, modelled)");

        // SC6 — the offer cadence, because the number is quoted in `pano_ulpi.py` and `docs/STATUS.md` as a
        // measurement (666.6 ns = 40 x 16.667 ns, from
        // `bench/2026-10-31_t18_after_scratchread_00.vcd`). A model whose throttle drifted from the
        // measurement would quietly stop resembling the board, and any timeout argued from that number would
        // be argued from a guess.
        begin : cadence
            integer seen_high, off, wrong;
            seen_high = 0; off = 0; wrong = 0;
            start_scenario(1'b0);
            read_cmd(6'h04);
            wait_answer(ans);
            if (ans != 0) begin
                for (guard = 0; guard < TX_PERIOD * 5; guard = guard + 1) begin
                    @(posedge clk);
                    if (phy_nxt) begin
                        if (off != 0 && off != TX_PERIOD) wrong = wrong + 1;
                        seen_high = seen_high + 1;
                        off = 1;
                    end else if (seen_high != 0) off = off + 1;
                end
            end
            if (seen_high < 3)   bad("SC6: no nxt offers in Tx mode  --  the throttle is gone, so a Link that waits for nxt cannot be timed");
            else if (wrong)      bad("SC6: the offer period is not TX_NXT_PERIOD; the quoted 666.6 ns is now a guess");
            else begin
                // Not through `good()`: a task argument is a fixed-width reg and iverilog refuses a
                // $sformatf into it. The counter and the line are the same thing either way.
                n_ok = n_ok + 1;
                $display("SELFCHK ok    SC6: Tx-mode offers are one cycle every %0d (the measured 666.6 ns)",
                         TX_PERIOD);
            end
        end

        if (nxt_too_long) bad("the model flagged its OWN nxt violation (nxt_cycles_too_long) on a clean run");

        $display("ulpi_phy_selfcheck: %0d checks, %0d failed", n_ok + errors, errors);
        if (errors != 0) $fatal(1, "the PHY model lost a capability the vectors depend on");
        $finish;
    end

endmodule
