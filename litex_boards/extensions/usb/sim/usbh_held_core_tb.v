`timescale 1ns/1ps
// ---------------------------------------------------------------------------------------------
// **Is the register window readable while `usb_host_core_rst` is 1?** No, and this file is the row set that
// says so -- which is worth more than the sentence it replaces, because a sentence in a comment is what a
// later session trusts, and this one was wrong.
//
// `pano_usb_host.py` used to claim, about the reset this CSR controls: *"The Wishbone window and the CDC are
// NOT in this reset and stay readable while the core is held."* The 2026-11-05/06 bench found the opposite:
// with `phy_reset` released and `core_rst` 1, every read of `0xc0000000` returned `0xffffffff` and
// `usb_host_lost` climbed in lockstep with LiteX's `ctrl_bus_errors` (panologic_g2
// `bench/2026-11-05_usb_host_window_t3/`, `bench/2026-11-06_hubclass_t3/`, RESEARCH §7.16.6). The sentence
// was not harmless: M3.3's BIOS test was designed to hold the cores in reset, check the clock and read the
// window in between, and that plan reads `0xffffffff` and reports a bridge fault that is not there.
//
// THE DUT IS THE COMPOSITION, not the bridge alone. `wb_axil_cdc_tb.v` proves what our bridge does with a
// far side that *chooses* not to answer; that slave is behavioural. Here the far side is
// `core_usb_host` itself, held by the level the CSR drives -- because the thing inside that reset is the
// vendor's own AXI4-Lite response half: `rvalid_q`/`bvalid_q` are cleared by `rst_i`
// (`usbh_host.v:683-691`, `:711-723`) while its *accept* signals are combinational and stay high
// (`cfg_arready_o = ~cfg_rvalid_o`, `:150-152`). A held core therefore **shakes hands and never answers**:
// `held-read-is-accepted-by-a-deaf-core` is that mechanism as a row, and it is why the old comment was
// unfalsifiable in the direction that mattered -- the bridge is exempt, the slave is not.
//
// HOW IT RUNS. Not by hand and not from this repo: it is stage `held` of
// `tools/wb_axil_cdc_sim.py` in the panologic_g2 repo, so `check.sh cdc` runs it on every commit there
// (`--no-held` skips it, `-v` prints every row plus the CTX lines). It needs `$CORE_USB_HOST` to point at a
// `core_usb_host` checkout — the runner **skips this stage by name** without it, because the part, not a model
// of it, is what the rows judge — and it expects the exact row set below (`EXPECT_HELD` in that file: a row
// that stops printing is a red, never a smaller green).
//
// `usb_ok` is tied 1 on purpose. In the design it is `UsbReset.ok` = `clk_ok & ~phy_reset`, which does NOT
// include this CSR: the bridge has no idea the far side is deaf, so the only thing that can report the state
// is the watchdog. That is the bench's situation exactly, and it is why the counters and not the value are
// the witness.
//
// THE OTHER HALF OF THE DECISION, and the reason this is not just "it hangs": an access a held core
// swallows is **dropped, not delayed**. The rows `no-phantom-rvalid-after-release`,
// `no-phantom-bvalid-after-release` and `held-write-did-not-land-late` say that releasing the core delivers
// no stale transaction earned before the reset, and retroforms no write. Without those three rows the
// honest answer to "is a dead window acceptable?" would be "no -- I cannot release a core I may have left a
// write in", and a bench would be told to reload the bitstream instead of writing 0 to a CSR.
//
// Rows are `HOLDCHECK <name> PASS|FAIL`, plus `CTX` lines the runner echoes with -v, and
// `tools/wb_axil_cdc_sim.py` (category `cdc`) demands this file's exact set. Row names are 512 bits wide
// because the 128-bit field of the one-off this came from silently truncated every name to 16 characters in
// its own log, which is how a row becomes an anecdote.
// ---------------------------------------------------------------------------------------------
`default_nettype none
module usbh_held_core_tb;
    reg sys_clk = 0, usb_clk = 0, sys_rst = 0;
    always #10    sys_clk = ~sys_clk;    // 50 MHz: cd_sys
    always #8.333 usb_clk = ~usb_clk;    // 60 MHz: cd_usb, the PHY's forwarded clock

    localparam WATCHDOG = 200;           // the sim top's bound; row 1 checks we did not assume it

    // ---- Wishbone side ---------------------------------------------------------------------------
    reg         wb_cyc = 0, wb_stb = 0, wb_we = 0;
    reg  [7:0]  wb_adr = 0;
    reg  [31:0] wb_dat_w = 0;
    reg  [3:0]  wb_sel = 4'hf;
    wire        wb_ack, wb_err;
    wire [31:0] wb_dat_r;

    // ---- AXI4-Lite seam, the core's own port names ------------------------------------------------
    wire        cfg_awvalid, cfg_wvalid, cfg_bready, cfg_arvalid, cfg_rready;
    wire [31:0] cfg_awaddr, cfg_wdata, cfg_araddr;
    wire [3:0]  cfg_wstrb;
    wire        cfg_awready, cfg_wready, cfg_bvalid, cfg_arready, cfg_rvalid;
    wire [1:0]  cfg_bresp, cfg_rresp;
    wire [31:0] cfg_rdata;

    wire        usb_ok = 1'b1;           // clock healthy, PHY released: ONLY the core is held
    wire [15:0] xfer;
    wire [7:0]  lost, aborted;
    wire        err_lost, bridge_busy;
    wire [31:0] timeout_cycles;

    wb_axil_cdc dut(
        .sys_clk(sys_clk), .usb_clk(usb_clk), .sys_rst(sys_rst),
        .wb_cyc(wb_cyc), .wb_stb(wb_stb), .wb_we(wb_we), .wb_adr(wb_adr),
        .wb_dat_w(wb_dat_w), .wb_sel(wb_sel), .wb_ack(wb_ack), .wb_err(wb_err), .wb_dat_r(wb_dat_r),
        .cfg_awvalid(cfg_awvalid), .cfg_awaddr(cfg_awaddr), .cfg_wvalid(cfg_wvalid),
        .cfg_wdata(cfg_wdata), .cfg_wstrb(cfg_wstrb), .cfg_bready(cfg_bready),
        .cfg_arvalid(cfg_arvalid), .cfg_araddr(cfg_araddr), .cfg_rready(cfg_rready),
        .cfg_awready(cfg_awready), .cfg_wready(cfg_wready), .cfg_bvalid(cfg_bvalid),
        .cfg_bresp(cfg_bresp), .cfg_arready(cfg_arready), .cfg_rvalid(cfg_rvalid),
        .cfg_rdata(cfg_rdata), .cfg_rresp(cfg_rresp),
        .usb_ok(usb_ok), .xfer(xfer), .lost_count(lost), .aborted(aborted),
        .err_lost(err_lost), .bridge_busy(bridge_busy), .timeout_cycles(timeout_cycles));

    // ---- the part we ship, held by the level `usb_host_core_rst` drives -----------------------------
    reg core_rst = 1;
    wire [7:0] utmi_data_out; wire txvalid, termselect, dppulldown, dmpulldown;
    wire [1:0] op_mode, xcvrselect;
    usbh_host core(.clk_i(usb_clk), .rst_i(core_rst),
        .cfg_awvalid_i(cfg_awvalid), .cfg_awaddr_i(cfg_awaddr), .cfg_wvalid_i(cfg_wvalid),
        .cfg_wdata_i(cfg_wdata), .cfg_wstrb_i(cfg_wstrb), .cfg_bready_i(cfg_bready),
        .cfg_arvalid_i(cfg_arvalid), .cfg_araddr_i(cfg_araddr), .cfg_rready_i(cfg_rready),
        .cfg_awready_o(cfg_awready), .cfg_wready_o(cfg_wready), .cfg_bvalid_o(cfg_bvalid),
        .cfg_bresp_o(cfg_bresp), .cfg_arready_o(cfg_arready), .cfg_rvalid_o(cfg_rvalid),
        .cfg_rdata_o(cfg_rdata), .cfg_rresp_o(cfg_rresp), .intr_o(),
        // A PHY idling at J: SE0 would look like a reset, J is what a released hub shows.
        .utmi_data_in_i(8'h0), .utmi_txready_i(1'b1), .utmi_rxvalid_i(1'b0),
        .utmi_rxactive_i(1'b0), .utmi_rxerror_i(1'b0), .utmi_linestate_i(2'b01),
        .utmi_data_out_o(utmi_data_out), .utmi_txvalid_o(txvalid),
        .utmi_op_mode_o(op_mode), .utmi_xcvrselect_o(xcvrselect),
        .utmi_termselect_o(termselect), .utmi_dppulldown_o(dppulldown),
        .utmi_dmpulldown_o(dmpulldown));

    // Count the CORE's responses, so "nothing arrived after the release" is an observation and not a
    // silence. The bridge ties rready/bready to 1, so a handshake is a cycle with valid high.
    integer rvalid_hits = 0, bvalid_hits = 0, ar_accepted = 0;
    always @(posedge usb_clk) begin
        if (cfg_rvalid && cfg_rready) rvalid_hits = rvalid_hits + 1;
        if (cfg_bvalid && cfg_bready) bvalid_hits = bvalid_hits + 1;
        if (cfg_arvalid && cfg_arready) ar_accepted = ar_accepted + 1;
    end

    integer checks = 0, fails = 0;
    task chk(input [0:511] name, input ok, input [0:1023] why); begin
        checks = checks + 1;
        if (ok) $display("HOLDCHECK %0s PASS", name);
        else begin fails = fails + 1; $display("HOLDCHECK %0s FAIL  %0s", name, why); end
    end endtask

    // One Wishbone access, bounded. A real CPU master would hang and LiteX's interconnect timeout would
    // answer it with all-ones -- which is exactly the `0xffffffff` every bench row here carries.
    task access(input we, input [7:0] a, input [31:0] d, input integer bound,
                output got_ack, output got_err); begin
        integer k;
        @(posedge sys_clk); wb_adr = a; wb_dat_w = d; wb_we = we; wb_cyc = 1; wb_stb = 1;
        got_ack = 0; got_err = 0;
        for (k = 0; k < bound; k = k + 1) begin
            @(posedge sys_clk);
            if (wb_ack) begin got_ack = 1; k = bound; end
            if (wb_err) begin got_err = 1; k = bound; end
        end
        wb_cyc = 0; wb_stb = 0;
        @(posedge sys_clk);
    end endtask

    localparam [7:0]  USB_CTRL_W   = 8'd0;      // byte 0x00
    localparam [7:0]  USB_STATUS_W = 8'd1;      // byte 0x04
    localparam integer HAMMER      = 20;

    integer ha, he, wa, we_;
    integer expect_lost = 0;                    // presses made against the HELD core, and only those
    integer rv_before, bv_before, ar_before;

    initial begin
        $display("CTX watchdog=%0d usb_ok=%b core_rst=%b", timeout_cycles, usb_ok, core_rst);
        sys_rst = 1; repeat (5) @(posedge sys_clk); sys_rst = 0; repeat (5) @(posedge sys_clk);

        chk("pre-watchdog-is-the-duts", timeout_cycles == WATCHDOG,
            "every bound below is WATCHDOG+500, so a row that waited for a watchdog of a length it did not ask for would be judging a different design");

        // ============================== HELD IN RESET =============================================
        ar_before = ar_accepted; rv_before = rvalid_hits; bv_before = bvalid_hits;
        access(1'b0, USB_STATUS_W, 32'd0, WATCHDOG + 500, ha, he);
        expect_lost = expect_lost + 1;
        $display("CTX held-read ack=%0d err=%0d dat_r=%h ar_accepted=%0d rvalid_hits=%0d xfer=%0d lost=%0d aborted=%0d",
                 ha, he, wb_dat_r, ar_accepted - ar_before, rvalid_hits - rv_before, xfer, lost, aborted);

        // THE MECHANISM. `cfg_arready_o = ~cfg_rvalid_o` is combinational and the response register is held
        // at 0, so the deaf core ACCEPTS the read. The bridge cannot know it handed the transaction to a
        // slave that will never answer, and nothing but its watchdog can report that.
        chk("held-read-is-accepted-by-a-deaf-core", (ar_accepted - ar_before) >= 1 && he == 1,
            "had the core refused arvalid instead, the bridge would have waited on a handshook-less request and the old comment would have been defensible");
        chk("held-read-answers-nothing", (ha == 0) && (he == 1),
            "the old comment's claim, in RTL: a held core reported as an ack");
        chk("held-read-rvalid-never-rose", (rvalid_hits - rv_before) == 0,
            "the vendor clears rvalid_q with rst_i, so a read accepted while held can never complete");
        chk("held-read-is-err-not-zero-ack", (wb_dat_r != 32'h0) || (he == 1),
            "the shape this design refuses is an ack whose data is 0: 0 is a PLAUSIBLE USB_CTRL/USB_STATUS");
        chk("held-read-counts-lost", lost == expect_lost,
            "over a serial bridge a lost access and an all-ones register are the SAME value, so the counter is the only witness and it must move");
        chk("held-read-xfer-frozen", xfer == 16'd0,
            "xfer counts completions; a press that never completed must not move it");
        chk("held-err-lost-sticky", err_lost == 1'b1,
            "usb_host_state.cdc_lost is what a later read sees, and it must survive the press");

        // A write, held: a driver must not be allowed to post configuration blind.
        access(1'b1, USB_CTRL_W, 32'h0000_00e9, WATCHDOG + 500, wa, we_);
        expect_lost = expect_lost + 1;
        chk("held-write-is-err-too", (wa == 0) && (we_ == 1),
            "an ack here would let a driver believe its USB_CTRL landed while the core never took it");
        chk("held-write-never-completed", bvalid_hits == 0,
            "bvalid_q is cleared by rst_i too, so no write response can exist while held");

        // The shape a bench ladder actually produces: N presses against a window it never verified.
        begin : hammer_held
            integer i;
            for (i = 0; i < HAMMER; i = i + 1) begin
                access(1'b0, USB_STATUS_W, 32'd0, WATCHDOG + 500, ha, he);
                expect_lost = expect_lost + 1;
            end
            $display("CTX held-hammer expect_lost=%0d xfer=%0d lost=%0d aborted=%0d",
                     expect_lost, xfer, lost, aborted);
            chk("held-hammer-lost-counts-every-press", lost == expect_lost,
                "lost is the per-press witness: every press against a held core is exactly one loss");
            chk("held-hammer-xfer-still-zero", xfer == 16'd0,
                "any xfer here would mean the held core answered something");
            // `aborted` counts the destination FSM being withdrawn mid-AXI, and where that lands relative to
            // the destination clock is decided by the 50/60 MHz beat: measured 0, 1 or 2 per press, with the
            // total conserved. A bench note promising `aborted == presses` would make a healthy bridge read
            // as instrument-broken, so this row promises only that it tracks, and CTX prints the pair.
            chk("held-aborted-tracks-but-is-bounded", ((aborted >= 8'd1) && (aborted <= (expect_lost + 2))),
                "aborted must count the withdrawals without pretending to be a press counter");
        end

        // ============================== RELEASE, NO NEW ACCESS ====================================
        // The three rows the DECISION needs. Was anything *queued* by the swallow? If it was, releasing the
        // core would deliver a transaction earned before the reset, and "accept a dead window" would be
        // unsafe rather than merely inconvenient.
        rv_before = rvalid_hits; bv_before = bvalid_hits;
        core_rst = 0;
        repeat (400) @(posedge sys_clk);
        chk("no-phantom-rvalid-after-release", rvalid_hits == rv_before,
            "a held read completing after the release is a stale value attributed to a live core");
        chk("no-phantom-bvalid-after-release", bvalid_hits == bv_before,
            "a held write acking after the release is a write that landed with nobody watching it");

        access(1'b0, USB_CTRL_W, 32'd0, WATCHDOG + 500, ha, he);
        $display("CTX released-read-ctrl ack=%0d err=%0d dat_r=%h xfer=%0d lost=%0d",
                 ha, he, wb_dat_r, xfer, lost);
        chk("released-read-acks", (ha == 1) && (he == 0),
            "same window, same bridge, one changed CSR: the split that made the comment false");
        chk("held-write-did-not-land-late", wb_dat_r == 32'h0,
            "the 0xe9 written while held must NOT be in USB_CTRL now: it died with the reset, so releasing and re-initialising does not fight a write from before the reset");

        begin : hammer_free
            integer base, i;
            base = xfer;
            for (i = 0; i < HAMMER; i = i + 1)
                access(1'b0, USB_STATUS_W, 32'd0, WATCHDOG + 500, ha, he);
            $display("CTX free-hammer xfer=%0d (base %0d) lost=%0d aborted=%0d",
                     xfer, base, lost, aborted);
            chk("free-hammer-xfer-tracks-1to1", (xfer - base) == HAMMER,
                "the 1:1 accounting ticket 01 asked for in the HELD state belongs here, in the state where it can be true");
            chk("free-hammer-no-new-loss", lost == expect_lost,
                "the lost meter must freeze once the core answers, else a bench cannot tell a settled link from a livelock");
        end

        access(1'b1, USB_CTRL_W, 32'h0000_00e9, WATCHDOG + 500, wa, we_);
        chk("released-write-acks", (wa == 1) && (we_ == 0), "the write that was err while held now lands");
        access(1'b0, USB_CTRL_W, 32'd0, WATCHDOG + 500, ha, he);
        chk("released-write-reads-back", (ha == 1) && (wb_dat_r == 32'h0000_00e9),
            "and reads back, which is the WCHK a bench ladder depends on; bit 8 has no read-mux case, so 0xe9 (not 0x1e9) is the expectation to write");

        $display("SUMMARY checks=%0d fails=%0d presses_lost=%0d xfer=%0d rvalid_hits=%0d bvalid_hits=%0d",
                 checks, fails, expect_lost, xfer, rvalid_hits, bvalid_hits);
        $finish;
    end
endmodule
