`timescale 1ns / 1ps
//=============================================================================
// wb_axil_cdc_tb.v -- the cd_sys -> cd_usb register bridge of PLAN.md M3.2, executed.
//
// DUT: wb_axil_cdc_sim_top.py (same directory), which is
// litex_boards/extensions/pano_wb_axil.py and nothing else.
//
// The far side is the behavioural AXI4-Lite slave below. Its handshake rules are
// copied from the core it stands in for (usbh_host.v:150-152, 685-723), not invented:
//
//     cfg_arready_o = ~cfg_rvalid_o
//     cfg_awready_o = ~cfg_bvalid_o & ~cfg_arvalid_i & ~awvalid_q
//     cfg_wready_o  = ~cfg_bvalid_o & ~cfg_arvalid_i & ~wvalid_q
//     bvalid_q / rvalid_q clear on cfg_bready_i / cfg_rready_i
//
// The `~cfg_arvalid_i` term is why a master may not present a write and a read
// together, and it is the reason a model that accepted either would let a mutation
// that drives both valids "pass". This model refuses.
//
// What the rows are for. The risk in a register bridge whose far side runs on the
// PHY's clock is not arithmetic, it is a far side that can STOP (PHY reset, Low
// Power Mode -- RESEARCH.md §7.10). So half the rows are about a far side that does
// not answer, and the interesting failure is not "the bridge hangs" (loud, and a
// reloading human) but "the bridge reports the wrong thing": an `ack` on a lost
// access hands the CPU whatever is on the read bus, and zero is a plausible
// USB_STATUS value. `lost-is-err-not-ack` is the row that exists for that, and
// docs/NOTES.md's "litex answers a bridge timeout with [0]*length -- silent" is the
// same failure mode one layer up, already met once.
//
// Verdict protocol: one `CDCCHECK <name> <PASS|FAIL|AMB> <detail>` per row and
// a final `SUMMARY checks=N failed=M amb=A`, plus a `RESULT` line. The runner
// (docs repo, tools/wb_axil_cdc_sim.py) requires the exact row SET -- a testbench
// that errors out halfway and exits 0 is a red, not a smaller number -- and treats
// AMB as red, so a row cannot quietly stop running.
//
// `AMB` exists because the first version of the sister task counted an x as a pass
// (ulpi_csr_rules_tb.v: deleting a signal's only writer makes migen re-declare the
// port as an input, the TB's wire floats, `!x` is x, and a row reported PASS on a
// register nobody drives). Keep `chk` multi-valued.
//
// Why `chk` takes a STRING LITERAL and the numbers come from `ctx1/ctx2/ctx3`:
// iverilog refuses to implicitly cast `$sformatf(...)` to a sized task argument
// ("cannot be implicitly cast to the target type", measured 2026-11-04), so either
// the numbers go or the rows do. They go as their own indented line under the row,
// which is better in a log anyway: the explanation of a row is constant, the numbers
// are what changed.
//=============================================================================

module wb_axil_cdc_tb;

    // --- clocks -------------------------------------------------------------------------------
    // cd_sys 50 MHz (20 ns), cd_usb = the PHY's 60 MHz (16.667 ns). Deliberately NOT harmonised:
    // the 3.333 ns beat between them is what the board has, and a TB whose two clocks share
    // edges never shows a bridge sampling the far side mid-transition.
    reg sys_clk = 1'b0;
    always #10 sys_clk = ~sys_clk;

    reg usb_clk_run = 1'b1;
    reg usb_clk     = 1'b0;
    always begin
        #8.333;
        if (usb_clk_run) usb_clk = ~usb_clk;
    end

    reg sys_rst = 1'b1;

    // --- Wishbone master (the CPU's stand-in) ---------------------------------------------------
    reg         wb_cyc = 1'b0, wb_stb = 1'b0, wb_we = 1'b0;
    reg  [ 7:0] wb_adr = 8'd0;
    reg  [31:0] wb_dat_w = 32'd0;
    reg  [ 3:0] wb_sel = 4'hf;

    wire        wb_ack, wb_err;
    wire [31:0] wb_dat_r;

    // --- AXI4-Lite (the bridge masters, the model answers) --------------------------------------
    wire        cfg_awvalid, cfg_wvalid, cfg_bready, cfg_arvalid, cfg_rready;
    wire [31:0] cfg_awaddr, cfg_araddr, cfg_wdata;
    wire [ 3:0] cfg_wstrb;
    wire        cfg_awready, cfg_wready, cfg_bvalid, cfg_arready, cfg_rvalid;
    wire [ 1:0] cfg_bresp, cfg_rresp;
    wire [31:0] cfg_rdata;

    // --- witnesses ------------------------------------------------------------------------------
    reg         usb_ok = 1'b1;
    wire [15:0] xfer;
    wire [ 7:0] lost_count, aborted;
    wire        err_lost, bridge_busy;
    wire [31:0] timeout_cycles;

    // --- the behavioural core -------------------------------------------------------------------
    reg  [31:0] areg [0:7];              // byte offsets 0x00..0x1c
    integer     dly_r  = 0;              // cd_usb cycles before R_VALID (0 = the cycle after)
    integer     dly_b  = 0;              // cd_usb cycles before B_VALID
    reg         deaf   = 1'b0;           // 1 = never ready, never valid: the far side is not there
    // The real core ties both to OKAY (usbh_host.v:707,723), so these exist to test OUR plumbing:
    // a resp that ever means something must not be swallowed here.
    reg  [ 1:0] bresp_in = 2'b00, rresp_in = 2'b00;

    integer     n_ar = 0, n_aw = 0, n_w = 0, n_ack = 0, n_err = 0;

    // Counted where they happen. Every "exactly one" claim reads these, because a row that reads a
    // LEVEL can be satisfied by a pulse that occurred twice between two of its samples.
    always @(posedge sys_clk) begin
        if (wb_ack) n_ack = n_ack + 1;
        if (wb_err) n_err = n_err + 1;
    end
    always @(posedge usb_clk) begin
        if (cfg_arvalid & cfg_arready) n_ar = n_ar + 1;
        if (cfg_awvalid & cfg_awready) n_aw = n_aw + 1;
        if (cfg_wvalid  & cfg_wready ) n_w  = n_w  + 1;
    end

    // "The bridge never touched the core" needs the OR of every cycle, not a sample of the level.
    reg saw_awvalid = 1'b0, saw_arvalid = 1'b0, saw_wvalid = 1'b0;
    always @(posedge usb_clk) begin
        if (cfg_awvalid) saw_awvalid <= 1'b1;
        if (cfg_arvalid) saw_arvalid <= 1'b1;
        if (cfg_wvalid ) saw_wvalid  <= 1'b1;
    end

    wb_axil_cdc dut (
        .sys_clk(sys_clk), .sys_rst(sys_rst), .usb_clk(usb_clk),
        .wb_cyc(wb_cyc), .wb_stb(wb_stb), .wb_we(wb_we), .wb_adr(wb_adr),
        .wb_dat_w(wb_dat_w), .wb_sel(wb_sel),
        .wb_ack(wb_ack), .wb_err(wb_err), .wb_dat_r(wb_dat_r),
        .cfg_awvalid(cfg_awvalid), .cfg_awaddr(cfg_awaddr),
        .cfg_wvalid(cfg_wvalid), .cfg_wdata(cfg_wdata), .cfg_wstrb(cfg_wstrb),
        .cfg_bready(cfg_bready),
        .cfg_arvalid(cfg_arvalid), .cfg_araddr(cfg_araddr), .cfg_rready(cfg_rready),
        .cfg_awready(cfg_awready), .cfg_wready(cfg_wready),
        .cfg_bvalid(cfg_bvalid), .cfg_bresp(cfg_bresp),
        .cfg_arready(cfg_arready), .cfg_rvalid(cfg_rvalid), .cfg_rdata(cfg_rdata),
        .cfg_rresp(cfg_rresp),
        .usb_ok(usb_ok), .xfer(xfer), .lost_count(lost_count), .aborted(aborted),
        .err_lost(err_lost), .bridge_busy(bridge_busy), .timeout_cycles(timeout_cycles)
    );

    // --- the core's handshake, copied from usbh_host.v ------------------------------------------
    // core_aw_q/core_w_q are the core's own "address came without data" latches (usbh_host.v:94-121).
    reg core_aw_q = 1'b0, core_w_q = 1'b0, bvalid_q = 1'b0, rvalid_q = 1'b0;
    reg [31:0] core_rdata_q = 32'd0;
    integer r_timer = 0, b_timer = 0;

    assign cfg_awready = deaf ? 1'b0 : (~bvalid_q & ~cfg_arvalid & ~core_aw_q);
    assign cfg_wready  = deaf ? 1'b0 : (~bvalid_q & ~cfg_arvalid & ~core_w_q);
    assign cfg_arready = deaf ? 1'b0 : (~rvalid_q);
    assign cfg_bvalid  = bvalid_q;
    assign cfg_bresp   = bresp_in;
    assign cfg_rvalid  = rvalid_q;
    assign cfg_rdata   = core_rdata_q;
    assign cfg_rresp   = rresp_in;

    // Accepted write = both channels presented, and not already answering.
    wire core_wr = (~deaf) & (cfg_awvalid | core_aw_q) & (cfg_wvalid | core_w_q) & ~bvalid_q;
    wire core_rd = (~deaf) & cfg_arvalid & cfg_arready;
    wire [7:0] wr_addr = core_aw_q ? 8'h00 : cfg_awaddr[7:0];

    // "The response timer just expired", computed OUTSIDE the always block because inside it the clear
    // below would shadow the set (see the note where bvalid_q is assigned).
    wire b_set = core_wr ? (dly_b == 0) : (b_timer == 1);
    wire r_set = core_rd ? (dly_r == 0) : (r_timer == 1);

    always @(posedge usb_clk) begin
        // awvalid_q / wvalid_q, as the core does.
        if (cfg_awvalid & cfg_awready & ~(cfg_wvalid & cfg_wready)) core_aw_q <= 1'b1;
        else if ((cfg_awvalid & cfg_awready) | core_aw_q)           core_aw_q <= 1'b0;

        if (cfg_wvalid & cfg_wready & ~(cfg_awvalid & cfg_awready)) core_w_q <= 1'b1;
        else if ((cfg_wvalid & cfg_wready) | core_w_q)              core_w_q <= 1'b0;

        // The write lands. cfg_wstrb is IGNORED here because usbh_host.v never reads cfg_wstrb_i
        // (declared at :58, used nowhere in the file). A model that honoured the strobes would make
        // the bridge look byte-accurate on a core that is not, and then no artifact anywhere would
        // carry the byte-write fact -- so `write-strb-reaches-the-wire` and
        // `write-whole-word-despite-strb` are one fact in two rows, on purpose.
        if (core_wr) begin
            if (wr_addr[7:2] < 8) areg[wr_addr[7:2]] <= cfg_wdata;
            if (dly_b != 0) b_timer <= dly_b;
        end else if (b_timer != 0) begin
            b_timer <= b_timer - 1;
        end

        if (core_rd) core_rdata_q <= (cfg_araddr[7:2] < 8) ? areg[cfg_araddr[7:2]] : 32'hdeadbeef;
        if (core_rd && (dly_r != 0)) r_timer <= dly_r;
        else if (r_timer != 0)       r_timer <= r_timer - 1;

        // BVALID / RVALID, and the reason they are a separate statement with a `b_set`/`r_set` wire:
        // the bridge ties bready/rready HIGH, so a naive `if (cfg_bready) bvalid_q <= 1'b0;` placed
        // after the set wins in non-blocking order and the response NEVER appears. That is not a
        // hypothetical -- it was this model's first bug, and it made the bridge look dead while every
        // read timed out on the watchdog. With rready tied high a response is consumed the cycle it
        // appears (which is exactly why the bridge cannot wedge: usbh_host.v:719,690), so the faithful
        // shape is a ONE-CYCLE pulse, set in preference to being cleared.
        if (b_set)          bvalid_q <= 1'b1;
        else if (cfg_bready) bvalid_q <= 1'b0;

        if (r_set)          rvalid_q <= 1'b1;
        else if (cfg_rready) rvalid_q <= 1'b0;
    end

    // --- checking -------------------------------------------------------------------------------
    integer nchecks = 0, nfail = 0, namb = 0;

    // id: <= 34 chars. A string assigned to a narrower formal keeps its TAIL, which is how a sister
    // suite once printed a verdict whose sentence started 40 characters earlier.
    task chk(input [8*34-1:0] id, input ok, input [8*170-1:0] detail);
        begin
            nchecks = nchecks + 1;
            if (ok === 1'bx || ok === 1'bz) begin
                namb = namb + 1;
                $display("CDCCHECK %-34s AMB  verdict is x/z: a port moved or lost its driver -- %0s", id, detail);
            end else if (ok !== 1'b1) begin
                nfail = nfail + 1;
                $display("CDCCHECK %-34s FAIL %0s", id, detail);
            end else begin
                $display("CDCCHECK %-34s PASS %0s", id, detail);
            end
        end
    endtask

    // The numbers a row rests on, printed under it. Call BEFORE chk so that the context describes the
    // state the verdict was taken on, not whatever the verdict's evaluation left behind.
    task ctx1(input [8*90-1:0] label, input integer a);
        begin $display("     . %-90s a=%0d", label, a); end
    endtask
    task ctx2(input [8*90-1:0] label, input integer a, input integer b);
        begin $display("     . %-90s a=%0d b=%0d", label, a, b); end
    endtask
    task ctx3(input [8*90-1:0] label, input integer a, input integer b, input integer c);
        begin $display("     . %-90s a=%0d b=%0d c=%0d", label, a, b, c); end
    endtask
    // Hex context, because `%h` of a register is the sentence "which register answered", and %0d of
    // 0x55aa55aa is not.
    task ctxh(input [8*90-1:0] label, input [31:0] a);
        begin $display("     . %-90s h=%h", label, a); end
    endtask

    // Ends 1 ns AFTER the n-th cd_sys rising edge, so a read sees the state settled after that edge and
    // stimulus written from here is sampled by the edge after. Same discipline as
    // ulpi_csr_rules_tb.v, where driving stimulus in the same timestep as the clock edge made 12 rows
    // fail on a correct DUT.
    task wait_sys(input integer n);
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                @(posedge sys_clk);
                #1;
            end
        end
    endtask

    // Resets the per-access witnesses so a row can say "exactly one" without owing anything to the row
    // before it. A row that forgets this is a row that passes on its neighbour's traffic.
    task clear_counts;
        begin
            n_ar = 0; n_aw = 0; n_w = 0; n_ack = 0; n_err = 0;
            saw_awvalid = 1'b0; saw_arvalid = 1'b0; saw_wvalid = 1'b0;
        end
    endtask

    // One Wishbone access, host-style: cyc+stb held with the address, released once answered.
    // `bound` is in cd_sys cycles; a row that wants "it never completed" asks for got_ack == 0.
    task do_access(input we, input [7:0] addr, input [31:0] wdata, input [3:0] sel,
                   input integer bound, output integer waited, output integer got_ack,
                   output integer got_err);
        integer n, ack0;
        begin
            wb_we    <= we;
            wb_adr   <= addr;
            wb_dat_w <= wdata;
            wb_sel   <= sel;
            wb_cyc   <= 1'b1;
            wb_stb   <= 1'b1;
            wait_sys(1);
            ack0 = n_ack;
            n    = 0;
            while ((n < bound) && (n_ack == ack0) && (n_err == 0)) begin
                wait_sys(1);
                n = n + 1;
            end
            wait_sys(1);                     // let a same-cycle ack/err land in the counters
            got_ack = (n_ack > ack0) ? 1 : 0;
            got_err = (n_err > 0)    ? 1 : 0;
            waited  = n;
            wb_cyc   <= 1'b0;
            wb_stb   <= 1'b0;
            wait_sys(1);
        end
    endtask

    integer waited, got_ack, got_err;
    integer ack0, err0, lost0, ab0, xfer0, n;

    initial begin
        // Values a reader recognises: 0x0424 is the SMSC VID every ULPI ID read on this bench answers
        // with, and 0x24 is the byte itself. A value that MEANS something is what makes "the wrong
        // register answered" visible instead of merely different.
        areg[0] = 32'h00000424;
        areg[1] = 32'h00000004;
        areg[2] = 32'h11223344;
        areg[3] = 32'h00000000;
        areg[4] = 32'h55aa55aa;
        areg[5] = 32'h00000024;
        areg[6] = 32'h00000000;
        areg[7] = 32'h00000000;

        wait_sys(5);
        sys_rst <= 1'b0;
        wait_sys(5);

        // -----------------------------------------------------------------------------------------
        // Preconditions. The 2026-11-03 bench run left the sister suite at 41/41 while three of its
        // sentences were contradicted by the board, and the shape of the miss was that all 41 mutants
        // attacked a row's PREDICATE and none attacked its PRECONDITION. So the numbers this suite
        // depends on are rows, read out of the DUT rather than repeated from the generator.
        // -----------------------------------------------------------------------------------------
        ctx1("the watchdog bound the DUT was built with (0 would make every watchdog row vacuous)", timeout_cycles);
        chk("pre-timeout-nonzero", (timeout_cycles !== 32'hx) && (timeout_cycles != 0),
            "the bridge has a watchdog at all, and its bound comes from the DUT and not from this file");

        chk("idle-quiet", (bridge_busy === 1'b0) && (n_ack == 0) && (n_err == 0)
                          && !saw_awvalid && !saw_arvalid && !saw_wvalid,
            "no traffic on either side and no busy before the first access");

        chk("bready-rready-tied-high", (cfg_bready === 1'b1) && (cfg_rready === 1'b1),
            "the core's valids clear on these; a master that cannot accept a response leaves bvalid high forever, and awready is ~bvalid (usbh_host.v:151)");

        // -----------------------------------------------------------------------------------------
        // Reads.
        // -----------------------------------------------------------------------------------------
        clear_counts;
        do_access(1'b0, 8'd5, 32'd0, 4'hf, 200, waited, got_ack, got_err);
        ctxh("wb_dat_r (expect 24, the byte this board always answers with)", wb_dat_r);
        ctx2("cd_sys cycles waited, acks seen", waited, n_ack);
        chk("read-acks", (got_ack == 1) && (got_err == 0) && (wb_dat_r === 32'h00000024),
            "a read returns the far side's word, in a handful of cycles");
        ctxh("cfg_araddr (expect 14: wb.adr=5 is a WORD, the core decodes cfg_awaddr_i[7:0])", cfg_araddr);
        chk("read-addr-word-to-byte", (cfg_araddr === 32'h14) && (n_ar == 1),
            "the word-to-byte shift is there; a missing <<2 is a plausible read of the wrong register, forever");
        ctx3("aw accepted, w accepted, ar accepted", n_aw, n_w, n_ar);
        chk("read-no-aw-w", (n_aw == 0) && (n_w == 0) && !saw_awvalid && !saw_wvalid,
            "a read must not present AW/W: cfg_awready is ~cfg_arvalid_i, so doing both deadlocks the core");

        clear_counts;
        do_access(1'b0, 8'h51, 32'd0, 4'hf, 200, waited, got_ack, got_err);
        ctxh("cfg_araddr for wb.adr=0x51 (byte 0x144): expect 44", cfg_araddr);
        chk("read-addr-above-core-decode", (cfg_araddr === 32'h44),
            "faithful to the core, which ignores cfg_awaddr_i[31:8] itself; crossing fewer bits would ALIAS 0x44 onto 0x04");

        // A slow far side: the ack waits for the FAR SIDE and for nothing else, and `busy` says so the
        // whole time. (A BIOS test polls nothing -- it blocks -- but `usb_host_state.cdc_busy` is the
        // register a human reads when a console access disappears.)
        dly_r = 40;
        clear_counts;
        wb_we <= 1'b0; wb_adr <= 8'd1; wb_cyc <= 1'b1; wb_stb <= 1'b1;
        wait_sys(15);
        ctx1("acks after 15 cd_sys cycles with dly_r=40 cd_usb cycles (expect 0)", n_ack);
        chk("read-no-early-ack", (n_ack == 0),
            "the bridge does not answer for a far side that has not answered");
        chk("read-busy-while-waiting", (bridge_busy === 1'b1) && (n_ack == 0),
            "busy is what a polling host would read; high for the whole crossing, not just at the start");
        n = 0;
        while ((n < 400) && (n_ack == 0)) begin wait_sys(1); n = n + 1; end
        ctxh("wb_dat_r after the delayed far side (expect 4)", wb_dat_r);
        ctx1("cd_sys cycles until the ack", n);
        chk("read-completes-after-delay", (n_ack == 1) && (wb_dat_r === 32'h00000004),
            "the wait ends when the far side ends it, with the far side's data");
        wb_cyc <= 1'b0; wb_stb <= 1'b0;
        wait_sys(3);
        dly_r = 0;

        // -----------------------------------------------------------------------------------------
        // Writes.
        // -----------------------------------------------------------------------------------------
        clear_counts;
        do_access(1'b1, 8'd2, 32'hcafe0000, 4'hf, 200, waited, got_ack, got_err);
        ctxh("areg[2] (expect cafe0000)", areg[2]);
        chk("write-lands", (got_ack == 1) && (got_err == 0) && (areg[2] === 32'hcafe0000),
            "a write reaches the far side's register, not just the wire");
        ctx3("aw accepted, w accepted, ar accepted", n_aw, n_w, n_ar);
        chk("write-one-of-each-channel", (n_aw == 1) && (n_w == 1) && (n_ar == 0),
            "exactly one AW and one W per Wishbone write; two would be two writes to a register like USB_XFER_TOKEN");

        clear_counts;
        do_access(1'b1, 8'd3, 32'hffeeddcc, 4'b0110, 200, waited, got_ack, got_err);
        ctxh("cfg_wstrb as it reached the wire (expect 6)", {28'd0, cfg_wstrb});
        chk("write-strb-reaches-the-wire", (cfg_wstrb === 4'b0110),
            "the CORE ignores cfg_wstrb_i (usbh_host.v:58, used nowhere) -- the bridge must not, so the fact stays visible on the wire");
        ctxh("areg[3] (expect ffeeddcc: the whole word, strobes notwithstanding)", areg[3]);
        chk("write-whole-word-despite-strb", (areg[3] === 32'hffeeddcc),
            "the documented consequence, witnessed rather than asserted: a byte write is a whole-register write on this core");

        // -----------------------------------------------------------------------------------------
        // The far side is gone before the access: `dst_clear`, and the answer must be `err`.
        // -----------------------------------------------------------------------------------------
        clear_counts;
        usb_ok <= 1'b0;                 // == "PHY in reset or C12 gone" (UsbReset.ok)
        deaf   <= 1'b1;
        wait_sys(6);                    // let the 2-FF of usb_ok cross into cd_sys and become clear_x
        lost0 = lost_count;
        ack0  = n_ack;
        do_access(1'b0, 8'd5, 32'd0, 4'hf, 400, waited, got_ack, got_err);
        ctx2("acks during the loss, lost_count", n_ack - ack0, lost_count);
        chk("lost-is-err-not-ack", (got_err == 1) && (got_ack == 0) && (n_ack == ack0),
            "zero acks: an ack here hands the CPU a zero it would read as an idle core, which is a device that is not there");
        ctx2("lost_count before -> after", lost0, lost_count);
        chk("lost-counted", (lost_count == lost0 + 1) && (err_lost === 1'b1),
            "and witnessed: a loss nobody can read after the fact is a story about a dead board");
        chk("lost-drove-nothing", !saw_arvalid && !saw_awvalid && !saw_wvalid,
            "the clear was already high when the request arrived, so the core must never have been touched");

        // -----------------------------------------------------------------------------------------
        // The withdraw, with the clock running: an access reaches a deaf far side and the clear
        // arrives while the destination FSM is mid-AXI. This is the ReqAckCDC bug of 2026-10-31
        // (`dst_req` released only by `dst_ack`, so the NEXT access was swallowed): the row that
        // matters is the one AFTER, and it is written as a pair on purpose.
        // -----------------------------------------------------------------------------------------
        usb_ok <= 1'b1;
        wait_sys(10);
        clear_counts;
        ab0 = aborted;
        do_access(1'b0, 8'd5, 32'd0, 4'hf, 100, waited, got_ack, got_err);   // deaf: will not complete
        ctx2("accepted AR (expect 0: deaf), arvalid ever seen (expect 1)", n_ar, saw_arvalid);
        chk("inflight-before-clear", (bridge_busy === 1'b1) && (got_err == 0) && saw_arvalid,
            // `saw_arvalid` and not `n_ar`: a DEAF far side never accepts the request, so an accepted-AR
            // count here is 0 by construction. Asking for n_ar==1 would have made this row red for the
            // wrong reason, and a row that is red for the wrong reason trains nobody to read it.
            "an access outstanding at a far side that will not answer -- the state the withdraw has to survive");
        usb_ok <= 1'b0;
        wait_sys(10);
        ctx2("aborted before -> after", ab0, aborted);
        chk("withdraw-counted", (aborted == ab0 + 1),
            "the destination FSM left its AXI wait and counted it: an abandoned AXI transaction is the one case where the core may answer later");
        usb_ok <= 1'b1;
        deaf   <= 1'b0;
        wait_sys(10);
        err0 = n_err;

        clear_counts;
        do_access(1'b0, 8'd5, 32'd0, 4'hf, 400, waited, got_ack, got_err);
        ctxh("wb_dat_r on the access AFTER the loss (expect 24)", wb_dat_r);
        chk("next-access-works-after-loss", (got_ack == 1) && (got_err == 0) && (wb_dat_r === 32'h00000024),
            "the ReqAckCDC withdraw bug made THIS row fail, not the row that lost the access");

        // -----------------------------------------------------------------------------------------
        // The watchdog: present by our own witness, and silent anyway. `dst_clear` structurally cannot
        // see this, and on the board it is "C12 stopped before the meter noticed" -- so the clock is
        // stopped HERE, with usb_ok left high, rather than a model that merely pretends to be deaf.
        // -----------------------------------------------------------------------------------------
        clear_counts;
        lost0 = lost_count;
        usb_clk_run = 1'b0;                      // C12 stops; usb_ok still says the far side is fine
        wait_sys(3);
        do_access(1'b0, 8'd5, 32'd0, 4'hf, timeout_cycles + 200, waited, got_ack, got_err);
        ctx2("lost_count before -> after (bound is a cd_sys-cycle count)", lost0, lost_count);
        chk("watchdog-fires-on-silent-far-side", (got_err == 1) && (lost_count == lost0 + 1),
            "the bound is ours, so it also catches a far side our own witness failed to disbelieve");
        ctx1("acks on a dead clock (expect 0)", n_ack);
        chk("no-ack-on-dead-clock", (n_ack == 0),
            "this is the row that keeps a dead clock from looking like an idle device");
        usb_clk_run = 1'b1;
        wait_sys(20);

        clear_counts;
        do_access(1'b0, 8'd5, 32'd0, 4'hf, 400, waited, got_ack, got_err);
        ctxh("wb_dat_r after the clock returns (expect 24)", wb_dat_r);
        chk("recovers-after-clock-returns", (got_ack == 1) && (wb_dat_r === 32'h00000024) && (got_err == 0),
            "a bridge that stays lost after the clock comes back is a bridge that needs a reload");

        // The timer reloads while idle (ReqAckCDC: `If(~busy, timer.eq(watchdog))`), so a bridge cannot
        // idle its way into a spurious loss. Without that reload every idle board would report losses.
        clear_counts;
        wait_sys(timeout_cycles * 3);
        ctx2("errs, acks across three watchdog bounds with no access", n_err, n_ack);
        chk("no-timeout-while-idle", (n_err == 0) && (n_ack == 0),
            "the bound counts an access, not the time between two of them");

        // -----------------------------------------------------------------------------------------
        // A far side that reports an error. The real core ties bresp/rresp to OKAY, so these rows are
        // about our plumbing: a resp that ever means something must not be swallowed here.
        // -----------------------------------------------------------------------------------------
        bresp_in = 2'b10;
        clear_counts;
        do_access(1'b1, 8'd6, 32'h00000001, 4'hf, 200, waited, got_ack, got_err);
        ctx1("acks after a SLVERR write (expect 0)", n_ack);
        chk("bresp-nonokay-is-err", (got_err == 1) && (n_ack == 0),
            "a write acked after an error response is silent data loss");
        bresp_in = 2'b00;

        rresp_in = 2'b10;
        clear_counts;
        do_access(1'b0, 8'd5, 32'd0, 4'hf, 200, waited, got_ack, got_err);
        ctx1("acks after a SLVERR read (expect 0)", n_ack);
        chk("rresp-nonokay-is-err", (got_err == 1) && (n_ack == 0),
            "same on the read half: a resp that is silently dropped here is a resp that can never be reported");
        rresp_in = 2'b00;

        // -----------------------------------------------------------------------------------------
        // The counters are counters: one per access, not two, and not a level a row can misread.
        // -----------------------------------------------------------------------------------------
        clear_counts;
        xfer0 = xfer;
        do_access(1'b0, 8'd5, 32'd0, 4'hf, 200, waited, got_ack, got_err);
        ctx2("xfer before -> after one access", xfer0, xfer);
        chk("xfer-increments-by-one", (xfer == xfer0 + 1),
            "a bridge that re-issued the access would move this by two -- and the wire would show it too");

        // -----------------------------------------------------------------------------------------
        // One answer per claim. LiteX's own Wishbone masters hold `stb` THROUGH the ack cycle and leave
        // the state after it (spi_bone.py's BUS-MMAP-READ), so a bridge that became eligible again on its
        // own ack would issue a SECOND AXI transaction with the same address. On this core that is not
        // cosmetic: `USB_XFER_TOKEN` starts a transfer and `USB_WR_DATA` pushes a FIFO byte, so a doubled
        // write is a doubled command. The row that found this was `read-addr-word-to-byte`, and it went
        // red on the first run in which reads completed at all -- n_ar was 2 where 1 was expected.
        // -----------------------------------------------------------------------------------------
        clear_counts;
        wb_we <= 1'b1; wb_adr <= 8'd2; wb_dat_w <= 32'h0000beef; wb_cyc <= 1'b1; wb_stb <= 1'b1;
        n = 0;
        while ((n < 200) && (n_aw == 0)) begin wait_sys(1); n = n + 1; end
        // Ten cd_sys cycles, not four: the ack has to cross back (2-FF), and the row is about what
        // happens AFTER the ack while the claim is still held, so it has to be sure it is after it.
        wait_sys(10);
        ctx3("aw accepted, w accepted, acks seen (expect 1, 1, 1)", n_aw, n_w, n_ack);
        chk("stb-held-after-ack-no-reissue", (n_aw == 1) && (n_w == 1) && (n_ack == 1),
            "one AXI write per claim, even from a master that keeps stb asserted past the ack");
        wb_cyc <= 1'b0; wb_stb <= 1'b0;
        wait_sys(3);

        clear_counts;
        do_access(1'b0, 8'd2, 32'd0, 4'hf, 200, waited, got_ack, got_err);
        ctxh("areg[2] after the held claim, then a fresh read of it", areg[2]);
        chk("new-claim-after-release-works", (got_ack == 1) && (wb_dat_r === 32'h0000beef) && (n_ar == 1),
            "the guard is one-per-claim and not one-per-boot: dropping cyc/stb re-arms it");

        wait_sys(2);
        $display("SUMMARY checks=%0d failed=%0d amb=%0d", nchecks, nfail, namb);
        if (nfail != 0 || namb != 0) $display("RESULT FAILED");
        else                         $display("RESULT PASSED");
        $finish;
    end

    // A bound on the WHOLE run, so a hung handshake prints something instead of leaving the runner to a
    // wall-clock timeout with no rows at all ("a hang becomes a red, not a false green" -- the rule from
    // tools/ulpi_rtl_sim.py, and the reason the runner also refuses a missing SUMMARY).
    initial begin
        #5_000_000;
        $display("SUMMARY checks=%0d failed=%0d amb=%0d", nchecks, nfail + 1, namb);
        $display("RESULT TIMEOUT -- the run never reached its own end");
        $finish;
    end

endmodule
