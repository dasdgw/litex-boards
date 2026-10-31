`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// ulpi_phy_model.v -- a ULPI PHY that knows the spec and nothing about our FSM.
//
// Why this file exists at all: the only other model of this choreography is the one inside our own head,
// and a testbench written against that proves nothing. What this model implements is ULPI 1.1 §3.8.3.1
// (RegRead/RegWrite), §3.8.4.1 (abort) and §3.8.4.2 (disconnect) as written, with the USB3300's register
// values. Where the spec leaves a choice, the choice and the reason are commented at the line.
//
// Deliberately NOT modelled: burst, OTG, drive-strength, clock modes, line states, or any of the 12
// registers past the ones the vectors touch. A model that implements everything is a second design to
// debug; this one implements the choreography and the failure modes we have a vector for.
// -----------------------------------------------------------------------------

`include "ulpi_phy_defs.vh"

module ulpi_phy_model #(
    // Set to 1 to have the model print what it decodes. Off by default: the TB diffs outcomes, and a
    // chatty model turns a 200-line failure into a 4 000-line one.
    parameter DEBUG = 0
) (
    input  wire       clk,          // 60 MHz, the same clock the Link is running on
    input  wire       phy_reset,    // the reset pin, active high (our `phy_reset` CSR level)

    // Bus, as the Link sees it. The TB resolves the contention: `bus` is wired-OR of the two drivers and
    // both driving nonzero resolves to x, which is the point -- a Link that forgets to release does not
    // quietly win the bus, it fails the byte check.
    input  wire [7:0] link_data,
    input  wire       link_oe,
    output reg  [7:0] phy_data,
    output reg        phy_dir,
    output reg        phy_nxt,
    // The model's side of the tri-state. It is a separate signal and not `phy_dir` because the model
    // asserts dir one cycle before it drives the byte (§3.8.3.1), and a TB that wired `oe` to `dir` would
    // have the PHY sitting on the bus during its own announcement cycle -- which reads as a Link bug.
    output wire       phy_oe,
    input  wire       phy_stp,

    // --- failure knobs, one per vector in tools/ulpi_vectors.tsv ----------------
    input wire       knop_stall_nxt,     // accept the command, never assert nxt
    input wire [3:0] knop_abort_n,       // abort this many commands before cooperating (0 = never);
                                         // `abort_once` is n=1, `abort_3` is n=3, and `abort_3` exhausting
                                         // a budget of 2 is a different vector with a different name
    input wire       knop_abort_forever, // abort every command: the retry budget has to expire
    input wire       knop_wrong_cmd,     // only respond to the wrong TX CMD encoding (0xE3)
    input wire       knop_rev_bus,       // drive read data bit-reversed: the wiring-swap fingerprint
    input wire       knop_ignore_write,  // answer reads, swallow writes (a stuck write path)

    // The two line-state receivers, so the Link can read them back out of the Debug register the way the
    // bench does with a hub reset on the other pin. `01` = SE0/DP-low, which is what "nothing plugged in"
    // means electrically; the value lives here rather than in the register table because the truth of a
    // line state is not storage.
    input wire [1:0] linestate,

    // --- witnesses, so a vector can assert the PHY really saw what we think it saw
    output reg  [7:0] seen_cmd,          // last TX CMD byte decoded
    output reg  [7:0] seen_wdata,        // last write data byte accepted
    output reg        saw_stp,           // a 1-cycle stp was observed after the data byte
    output reg  [3:0] abort_count,       // dir aborts this model has issued
    output reg        cmd_is_write,      // decoding of seen_cmd
    output reg        nxt_cycles_too_long// model's own violation detector, see the nxt throttle note
);

    // The register table. Only the four ID registers and the handful the vectors write are meaningful;
    // everything else reads 00h like the USB3300's unimplemented space, which is what makes a stray address
    // vector able to say "0x08 must NOT read 0x24".
    reg [7:0] regs [0:63];
    integer i;

    // One more state than spin 1 had, and the extra 4th bit with it: §3.8.3.1's write is a THREE-cycle
    // story (nxt for the command, the byte in the cycle after it, then the END marker), and the three-state
    // version could only express "hold nxt" and "wait for stp" -- which is how this model ended up holding
    // nxt until a late byte arrived instead of sampling the cycle the spec owns to the byte.
    localparam S_IDLE    = 4'd0,
               S_NXT     = 4'd1,   // the cycle before `nxt` rises (this model is registered: the assertion
                                    // cycle is the one AFTER the state that assigns phy_nxt)
               S_WDATA   = 4'd2,   // C1: `nxt` asserted for the command
               S_WACK    = 4'd3,   // C2: the data byte is due; sample the bus, drop `nxt`
               S_WSTP    = 4'd4,   // waiting for the END marker
               S_RD_DIR  = 4'd5,   // announcing: dir up this cycle
               S_RD_DATA = 4'd6,   // driving the byte, one cycle
               S_RD_END  = 4'd7,   // releasing it: the cycle that bites a Link with a registered `dir`
               S_ABORT   = 4'd8;   // dir held; released on stp, or on the model's own bound

    reg [3:0]  state;
    reg [7:0]  byte_r;
    reg [5:0]  addr;
    reg        is_write;
    reg [7:0]  nxt_hold;      // cycles nxt has been up: the throttle bookkeeping
    reg [7:0]  abort_hold;

    task automatic set_id_regs;
        begin
            // USB3300 §6.2/§6.3.1: the vendor/product ID split low-byte-first across 0x00..0x03, so an
            // endianness bug in the master shows up as a *swap* and not as garbage.
            regs[0]  = 8'h24;  // VendorID low
            regs[1]  = 8'h04;  // VendorID high   -> 0x0424 = SMSC/Microchip
            regs[2]  = 8'h04;  // ProductID low
            regs[3]  = 8'h00;  // ProductID high  -> 0x0004 = USB3300
            for (i = 4; i < 64; i = i + 1) regs[i] = 8'h00;
            // Every value below comes from RESEARCH.md §7.2 (USB3300 datasheet Table 6.3), which is this
            // repo's single home for them, and `tools/test_ulpi_phy_model.py` diffs this list against the
            // python table in `tools/ulpi_phy_model.py` address by address. That check exists because the
            // two tables used to disagree here -- `0x06` said 0x02 ("Host Configuration") and `0x0A` said
            // 0x01 ("minimally OTG-compliant"), both of them remembered rather than cited, both of them
            // wrong against the doc, and **no vector reads either address**, so no vector could have said
            // so. A register the vectors do not read is exactly where a model is allowed to lie, which is
            // why the cross-check compares tables and not just outcomes.
            regs[6'h04] = 8'h41;   // Function Control POR: OTG_EN|INDIRECT_ACCESS|DPDM_DIS
            // §6.1.4's set/clear aliases read back the register they alias. They are modelled as separate
            // storage here (a write to `0x04` does not appear at `0x05`), which is honest because the Link
            // never uses the aliases (spec decision 2) and no vector reads them.
            regs[6'h05] = 8'h41;
            regs[6'h06] = 8'h41;
            regs[6'h0A] = 8'h06;   // OTG Control POR
            regs[6'h15] = 8'h00;   // Debug: the linestates are overlaid at read time, not stored
        end
    endtask

    reg phy_oe_r;   // registered beside phy_data, never decoded from `state`
    assign phy_oe = phy_oe_r;

    initial begin
        byte_r = 8'h00; phy_oe_r = 1'b0;
        phy_data = 8'h00; phy_dir = 1'b0; phy_nxt = 1'b0;
        state = S_IDLE; addr = 6'h00; is_write = 1'b0;
        seen_cmd = 8'h00; seen_wdata = 8'h00; saw_stp = 1'b0; abort_count = 4'd0;
        cmd_is_write = 1'b0; nxt_cycles_too_long = 1'b0;
        nxt_hold = 0; abort_hold = 0;
        set_id_regs;
    end

    // A PHY reset is not our `clear`: it wipes the register table back to defaults, which is precisely why
    // the reset-mid-transaction vector has to be followed by a fresh read to see anything at all.
    always @(posedge clk or posedge phy_reset) begin
        if (phy_reset) begin
            phy_data <= 8'h00; phy_dir <= 1'b0; phy_nxt <= 1'b0;
            state    <= S_IDLE;
            nxt_hold <= 0; abort_hold <= 0;
            set_id_regs;
        end else begin

            case (state)
                S_IDLE: begin
                    phy_nxt  <= 1'b0;
                    phy_dir  <= 1'b0;
                    phy_data <= 8'h00;
                    // A command is a byte the Link newly drives whose 7:6 is a TX CMD. Sampling the bus
                    // rather than trusting an "is this a command" hint is the PHY's job, and it is why the
                    // model can catch a master that sends a *packet* command by mistake: 0x40|PID decodes
                    // here as a packet, which this model ignores, and the vector fails.
                    if (link_oe && ((link_data[7:6] == 2'b10) || (link_data[7:6] == 2'b11))) begin
                        seen_cmd     <= link_data;
                        cmd_is_write <= (link_data[7:6] == 2'b10);
                        addr         <= link_data[5:0];
                        is_write     <= (link_data[7:6] == 2'b10);
                        if (knop_wrong_cmd) begin
                            // The `cmd_wrong` vector: the Link must not accept 0xE3 as a valid encoding,
                            // so this model pretends 0xE3 is the only thing worth acknowledging. Passing
                            // this vector the way a naive master would (by retrying forever) is what the
                            // timeout bound is for.
                            if (link_data == 8'hE3) begin
                                state    <= S_NXT;
                                nxt_hold <= 0;
                            end
                        end else if ((abort_count < knop_abort_n) || knop_abort_forever) begin
                            state       <= S_ABORT;
                            abort_hold  <= 8'd6;
                            abort_count <= abort_count + 4'd1;
                        end else if (knop_abort_forever) begin
                            state       <= S_ABORT;
                            abort_hold  <= 8'd6;
                            if (abort_count != 4'hf) abort_count <= abort_count + 4'd1;
                        end else if (!knop_stall_nxt) begin
                            state    <= S_NXT;
                            nxt_hold <= 0;
                        end
                        // knop_stall_nxt: fall through, stay IDLE, never assert nxt. The Link's own bound
                        // is the only thing that ends that.
                    end
                end

                // ---- the write byte is SAMPLED, not waited for ----------------------------------------
                // This is the state that was wrong, and the bench is what proved it. The previous version
                // held `nxt` up until the Link's byte *appeared*, quoting §3.8.2.2's "the cycle after nxt is
                // **detected** as high" to bless a master that registers `nxt` before acting. §3.8.3.1 is a
                // waveform, not a detection: "the Link sends a register write command and waits for nxt to
                // assert. **In the cycle after nxt asserts, the Link sends the register write data**" -- so
                // the PHY takes the bus value one cycle after the assertion cycle, whatever happens to be on
                // it. "Detected" describes the Link's view; it does not entitle the Link to be late.
                //
                // Measured against that on 2026-10-31 with `ulpi_t17`: `nxt` high for two consecutive
                // cycles, the byte the PHY took = the **command** byte of a Link that answers a cycle late,
                // and Scratch read back `0x96` = `0x80|0x16` for a write of `0x3c`
                // (`bench/2026-10-31_capture_write16_0x3C.vcd`). A model that waits cannot see that, and
                // 31/31 vectors passed a Link that has never once written a PHY.
                //
                // So: hold `nxt` for the command's assertion cycle (C1), keep it up for C2 -- the second
                // assertion, which is what the USB3300's two consecutive high cycles are -- and latch the
                // bus at the end of C2. No waiting for the byte, no permissive tail.
                S_NXT: begin
                    phy_nxt    <= 1'b1;
                    nxt_hold   <= nxt_hold + 8'd1;
                    if (nxt_hold > 8'd100) nxt_cycles_too_long <= 1'b1;   // model bug detector
                    if (is_write) begin
                        state <= S_WDATA;             // C2: nxt stays up, the byte is due on the bus
                    end else if (nxt_hold >= 8'd1) begin
                        // A read: there is no second byte from the Link to wait for -- it releases the bus.
                        // §3.8.3.1 then has the PHY assert dir in the cycle after nxt and drive the byte in
                        // the cycle after that, which is the alignment the Link's RD_TURN/RD_WAIT_DIR pair is
                        // built around. Waiting here for a Link byte is how this model used to hang every
                        // read vector, and a hung model looks exactly like a hung Link.
                        phy_nxt  <= 1'b0;
                        nxt_hold <= 0;
                        state    <= S_RD_DIR;
                    end
                end

                S_WDATA: begin
                    // C1, the command's `nxt` cycle. Keep nxt up for C2: the USB3300's two consecutive high
                    // cycles on a write are this assertion plus the second one §3.8.3.1 asks for
                    // ("waits for nxt to assert again"), not a throttle held open for a late Link.
                    phy_nxt <= 1'b1;
                    state   <= S_WACK;
                end

                S_WACK: begin
                    // C2, the cycle §3.8.3.1 owns to the data byte: latch **whatever is on the bus** and
                    // drop nxt for C3. A Link that answers a cycle late latches its own TX CMD here, which
                    // is precisely what the silicon did (`0x96` stored for a write of `0x3c`); a model that
                    // stores the intended value instead is a model that hides the bug.
                    seen_wdata <= link_data;
                    phy_nxt    <= 1'b0;
                    nxt_hold   <= 8'd0;
                    state      <= S_WSTP;
                end

                S_WSTP: begin
                    // §3.8.1.2: "The Link must assert stp for only one cycle to indicate End of Packet" --
                    // and §3.8.3.1 asks for it after the last write byte even though this is not a packet.
                    // A write is not committed until the model sees it, so a Link that drives the byte and
                    // never marks the end is caught.
                    if (phy_stp) begin
                        saw_stp <= 1'b1;
                        // Commit the byte captured in S_NXT, **not `link_data` now**: `stp` is asserted the
                        // cycle after the last data byte, and a Link that has already released the bus
                        // presents 00h (per the idle level), so committing `link_data` here writes 00 into
                        // every register -- which is exactly what this model did, and it looked like a Link
                        // that could not write. A real PHY latches the byte when it takes it.
                        if (!knop_ignore_write) regs[addr] <= seen_wdata;
                        state   <= S_IDLE;
                    end
                end

                // ---- the read, cycle by cycle, because this is where the byte is lost or found -------
                // §3.8.3.1: `dir` is asserted in the cycle after nxt; the byte follows in the cycle after
                // `dir`; "In the next cycle, the PHY must de-assert dir". Three registered states, so the
                // net is: dir high in D, data high (with dir still high) in D+1, both low in D+2.
                //
                // The byte and `phy_oe` have to be registered *together*. An `oe` decoded from the state
                // while the data register is loaded one cycle later -- which is what this file had -- puts
                // the enable up a cycle before the byte exists, and a Link that samples correctly reads the
                // idle level and reports 00h. That is a model bug wearing a Link bug's clothes, and it is
                // the reason this comment is longer than the code.
                S_RD_DIR: begin
                    phy_dir   <= 1'b1;
                    phy_nxt   <= 1'b0;
                    phy_data  <= 8'h00;              // §3.8.3.1: the idle level while `dir` goes up
                    state     <= S_RD_DATA;
                end
                S_RD_DATA: begin
                    byte_r = regs[addr];
                    if (addr == 6'h15) byte_r[1:0] = linestate;   // live, not stored -- see above
                    phy_data  <= knop_rev_bus ? { byte_r[0], byte_r[1], byte_r[2], byte_r[3],
                                                  byte_r[4], byte_r[5], byte_r[6], byte_r[7] }
                                              : byte_r;
                    phy_oe_r  <= 1'b1;               // with the data, on the same edge
                    state     <= S_RD_END;
                end
                S_RD_END: begin
                    phy_dir   <= 1'b0;
                    phy_data  <= 8'h00;
                    phy_oe_r  <= 1'b0;
                    state     <= S_IDLE;
                end

                // ---- §3.8.4.2: PHY-initiated disconnect ---------------------------------------------
                // "The PHY drives dir high for a minimum of one cycle ... and must hold dir asserted until
                // it detects stp asserted or it is ready to receive again." Both prior references call a
                // Link that ignores this "permanently wedged"; the vectors exist so we cannot.
                S_ABORT: begin
                    phy_dir  <= 1'b1;
                    phy_nxt  <= 1'b0;
                    phy_data <= 8'h00;
                    if (phy_stp) begin
                        phy_dir <= 1'b0;
                        state   <= S_IDLE;
                    end else if (abort_hold == 8'd0) begin
                        phy_dir <= 1'b0;
                        state   <= S_IDLE;
                    end else begin
                        abort_hold <= abort_hold - 8'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // §3.8.4.2 again, from the other side: "the PHY must not assert stp or dir at any other time". Watch
    // for our own model violating it, because a model that drives dir randomly makes every abort vector
    // meaningless.
    always @(posedge clk) begin
        if (!phy_reset && (state == S_IDLE) && phy_dir) begin
            $display("ulpi_phy_model: MODEL BUG: dir asserted while idle at time %0t", $time);
        end
    end

endmodule
