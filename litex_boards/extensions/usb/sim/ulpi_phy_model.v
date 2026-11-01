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
    parameter DEBUG = 0,
    // The `nxt` offer period of a USB3300 sitting in a USB Transmit it was never asked for: one cycle high
    // every **40** cycles of the 60 MHz interface, measured as 666.6 ns on this board and this PHY
    // (`bench/2026-10-31_t18_after_scratchread_00.vcd`, and the same cadence in
    // `bench/2026-10-31_t18_fresh_16.vcd`). A named parameter because a magic 40 in a comparison is the kind
    // of number a later reader assumes was invented, and it was not.
    parameter integer TX_NXT_PERIOD = 40
) (
    input  wire       clk,          // 60 MHz, the same clock the Link is running on
    input  wire       phy_reset,    // the reset pin, active high (our `phy_reset` CSR level)

    // Bus, as the Link sees it. The TB resolves the contention: `bus` is wired-OR of the two drivers and
    // both driving nonzero resolves to x, which is the point -- a Link that forgets to release does not
    // quietly win the bus, it fails the byte check.
    input  wire [7:0] link_data,
    input  wire       link_oe,
    // The resolved data NET, as the PHY's input receiver actually sees it (the TB's wired tri-state). It is
    // a separate port from `link_data`/`link_oe` for one reason: **a PHY has no view of the Link's output
    // enable.** It samples a level every cycle, whatever produced it. See `rx_r` below.
    input  wire [7:0] bus,
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
    // One-cycle "forget what the last row made me see". The TB used to compare `seen_cmd` across a whole
    // file without clearing it, so **a row whose PHY decoded nothing at all passed on the previous row's
    // command** -- and a wrong-command bug on a reset row could, and would, pass. Recorded in this repo as
    // `TB_HOLE` (handoff 2026-10-31, unfixed there); cleared per row here.
    input wire       clr_seen,
    // 1 = this PHY ignores a level left on the bus with nobody driving it, i.e. it is the permissive model
    // this file was before bug 3. 0 (the default, wired so by the TB) = it decodes what it samples.
    // The whole of bug 3 is the difference between those two sentences.
    input wire       knop_ignore_float,

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
    output reg        nxt_cycles_too_long,// model's own violation detector, see the nxt throttle note
    // Bug 3's witness: the PHY entered a USB Transmit it was never asked for, because of the LEVEL the Link
    // left on the data bus, and `how` says which level. Named rather than folded into `state` because a
    // vector that only says "the next access died" sends the engineer to the choreography, and the fault is
    // on the pads.
    output reg        unsolicited_tx,
    output reg        unsolicited_cmd,   // a RegRead/RegWrite decoded while `link_oe` was low
    output reg [7:0]  unsolicited_byte
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
               S_ABORT   = 4'd8,   // dir held; released on stp, or on the model's own bound
               // ---- bug 3: the state a floating data bus puts a real PHY into -------------------------
               // ULPI 1.1 Table 1: the data bus is "driven low by the Link during idle", and "the Link and
               // PHY initiate data transfers by driving a non-zero pattern onto the data bus". §2.3.2 is
               // stronger: "the Link CONTINUOUSLY drives the data bus to 00h during idle". USB3300 §6.1.5.1
               // spells out the consequence of not doing it -- "the Link must drive a ULPI Idle (00h) on the
               // data bus **or the USB3300 may decode the bus value as a ULPI command**" -- and §6.1.5.2
               // names the exact cycle our Link was missing: after a register READ's turnaround, "the Link
               // must drive a ULPI Idle command at T6". Our Link tri-states the pads instead, so after a
               // read the PHY's OWN answer stays on the pads and is decoded as if the Link had driven it.
               // Measured on this board: `bench/2026-10-31_t18_fresh_16.transitions.csv` holds `5a` there and
               // lets it decay `52 -> 50 -> 40 -> 0` over ~1.17 us = **60-70 interface cycles**, and every
               // one of those values has bits 7:6 = 01b, a **USB Transmit** (USB3300 Table 6-4). From then on
               // the PHY is fetching packet bytes and ignores every TXD CMD, which is
               // `bench/2026-10-31_ulpi_t18_poison_sweep.tsv` read properly: poisoned for exactly the 6 rows
               // whose BYTE was a non-Idle TXD type, clean for exactly the 42 whose byte was the Idle
               // command. A byte's address explains nothing; a byte's TYPE explains all 48 rows.
               S_TX      = 4'd9;   // unsolicited USB Transmit; see the state body

    reg [3:0]  state;
    reg [7:0]  byte_r;
    reg [7:0]  rx_r;        // the PHY's receiver: the last level actually driven onto the net
    reg [7:0]  tx_tick;     // Tx-mode throttle bookkeeping, see S_TX
    reg        nxt_window;  // §6.1.7's END window: the cycle after a Tx-mode `nxt` offer
    // ULPI 1.1 §2.3.1: "A change in dir causes a turnaround cycle on the bus during which, neither Link nor
    // PHY can drive the bus. Data during the turnaround cycle is undefined and must be ignored by both Link
    // and PHY." One cycle, armed whenever this model lets go of the bus. WITHOUT it the model would decode
    // the PHY's own byte during the very cycle it is handing the bus back, and would then poison a Link that
    // has already started driving 00h -- a model bug that looks exactly like the board's bug 3 and would
    // have made the fix look wrong. The bug is the LONG float afterwards (measured 60 cycles), not this one.
    reg        turn;
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

    // The receiver, and the retention. When neither side drives, a real CMOS net KEEPS its last level for a
    // long time (measured: 70 interface cycles above), so `rx_r` holds. Holding it FOREVER is the harsher
    // bound than the measured decay, and the harsher bound is the one that cannot flatter a bad Link: a
    // Link that drives 00h makes `rx_r` 00h one cycle after it takes the bus, whatever the board's
    // capacitance does afterwards.
    always @(posedge clk or posedge phy_reset) begin
        if (phy_reset)          rx_r <= 8'h00;
        else if (!$isunknown(bus)) rx_r <= bus;
    end

    // **Combinational**, with the retention behind it, and the reason is an off-by-one that would otherwise
    // hide the entire bug: the PHY samples its receiver at the rising edge, so what it decides in cycle C is
    // the level that was on the pads *during* C. Reading the registered `rx_r` inside the clocked block
    // reads the level of cycle C-1, which is the cycle the PHY was still driving itself -- so a Link that
    // takes the bus and drives 00h one cycle after `dir` falls (exactly what USB3300 §6.1.5.2's T6 asks
    // for) would still be decoded as the PHY's own previous byte. `rx_now` is what the pad presents.
    wire [7:0] rx_now = $isunknown(bus) ? rx_r : bus;

    initial begin
        byte_r = 8'h00; phy_oe_r = 1'b0; rx_r = 8'h00; tx_tick = 8'h00;
        phy_data = 8'h00; phy_dir = 1'b0; phy_nxt = 1'b0;
        state = S_IDLE; addr = 6'h00; is_write = 1'b0;
        seen_cmd = 8'h00; seen_wdata = 8'h00; saw_stp = 1'b0; abort_count = 4'd0;
        cmd_is_write = 1'b0; nxt_cycles_too_long = 1'b0;
        nxt_hold = 0; abort_hold = 0;
        // Every new receiver state, initialised, because `!turn` with `turn` unknown is unknown and an
        // unknown in the decode condition is not "false": iverilog takes neither branch, and the observable
        // symptom was this model refusing to decode ANY command (33/33 rows failing with `phycmd=00`, while
        // the same file passed a minute earlier). A state register added to a model without an initial value
        // does not fail loudly; it silently deletes the model's behaviour.
        unsolicited_tx = 1'b0; unsolicited_cmd = 1'b0; unsolicited_byte = 8'h00;
        nxt_window = 1'b0; turn = 1'b0;
        set_id_regs;
    end

    // A PHY reset is not our `clear`: it wipes the register table back to defaults, which is precisely why
    // the reset-mid-transaction vector has to be followed by a fresh read to see anything at all.
    always @(posedge clk or posedge phy_reset) begin
        if (phy_reset) begin
            phy_data <= 8'h00; phy_dir <= 1'b0; phy_nxt <= 1'b0;
            state    <= S_IDLE;
            nxt_hold <= 0; abort_hold <= 0;
            // Bug 3's state dies here and nowhere else, and that IS the bench result: `diag_cure.py` row
            // "wait 100 ms" does not cure it and row "PHY reset" does. Everything else in this list is a
            // register; this one is the reason the cure row reads the way it reads.
            unsolicited_tx <= 1'b0; unsolicited_byte <= 8'h00; nxt_window <= 1'b0; turn <= 1'b0;
            set_id_regs;
        end else begin
            // BEFORE the case, so a decode in the same cycle wins: `clr_seen` is asserted while the bus is
            // idle, and a row that then decodes a command must still be able to report it.
            if (clr_seen) begin
                seen_cmd <= 8'h00; seen_wdata <= 8'h00; saw_stp <= 1'b0;
                unsolicited_cmd <= 1'b0;
            end

            case (state)
                S_IDLE: begin
                    phy_nxt  <= 1'b0;
                    phy_dir  <= 1'b0;
                    phy_data <= 8'h00;
                    if (turn) turn <= 1'b0;          // §2.3.1: the hand-back cycle is not a command
                    // ---- bug 3, the entry, and the reason this model used to be blind ---------------------
                    // A real PHY has no view of the Link's output enable; it samples a LEVEL and decodes
                    // bits 7:6 (USB3300 Table 6-4). So a Link that tri-states the pads after driving them does
                    // not present "nothing" -- it presents the last byte, held. ULPI 1.1 §2.3.2: "the Link
                    // continuously drives the data bus to 00h during idle". USB3300 §6.1.5.1: "the Link must
                    // drive a ULPI Idle (00h) on the data bus OR THE USB3300 MAY DECODE THE BUS VALUE AS A
                    // ULPI COMMAND". Decoding `link_oe && link_data` instead -- which is what this model did,
                    // and what every offline check in this repo was built on -- builds a PHY that is
                    // physically incapable of the failure the board shows: 33/33 passed a Link that had never
                    // driven an idle level in its life.
                    //
                    // `unsolicited_tx` is deliberately NOT cleared by `clr_seen`: it is the state the next row
                    // inherits, and a per-row clear would erase the only witness of a fault whose entire
                    // symptom is that it appears one access later.
                    if (!knop_ignore_float && !turn && (rx_now[7:6] == 2'b01)) begin
                        state                <= S_TX;
                        tx_tick              <= 8'd0;
                        nxt_window           <= 1'b0;
                        if (!unsolicited_tx) unsolicited_byte <= rx_now;
                        unsolicited_tx       <= 1'b1;
                    // A command is a byte whose 7:6 is a TX CMD, taken off the pads. Sampling the bus rather
                    // than trusting an "is this a command" hint is the PHY's job, and it is why the model can
                    // catch a master that sends a *packet* command by mistake: 0x40|PID decodes here as a
                    // packet, which this model ignores, and the vector fails.
                    end else if (!turn && ((rx_now[7:6] == 2'b10) || (rx_now[7:6] == 2'b11))) begin
                        seen_cmd     <= rx_now;
                        cmd_is_write <= (rx_now[7:6] == 2'b10);
                        addr         <= rx_now[5:0];
                        is_write     <= (rx_now[7:6] == 2'b10);
                        // The witness this row exists for: a command decoded from a level the Link was NOT
                        // driving. On a Link that holds 00h it can only ever be a real command.
                        if (!link_oe) unsolicited_cmd <= 1'b1;
                        if (knop_wrong_cmd) begin
                            // The `cmd_wrong` vector: the Link must not accept 0xE3 as a valid encoding,
                            // so this model pretends 0xE3 is the only thing worth acknowledging. Passing
                            // this vector the way a naive master would (by retrying forever) is what the
                            // timeout bound is for.
                            if (rx_now == 8'hE3) begin
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
                    seen_wdata <= rx_now;
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
                    turn      <= 1'b1;               // the Link gets this cycle to take the bus (§6.1.5.2 T6)
                    state     <= S_IDLE;
                end

                // ---- bug 3: the unsolicited USB Transmit ----------------------------------------------
                // Entered from a LEVEL, not from a command: the PHY believes the Link is streaming a packet
                // at it (Table 6-4, 01b). Consequences, all of them the measured bench symptom:
                //   * a level on the pads is now DATA, so the next RegRead's TXD CMD is payload and is never
                //     answered -- "succeeded and leaves the interface unable to answer the next access";
                //   * the throttle is the transmit throttle, and on this PHY the offer is one `nxt` cycle
                //     every 40 (`bench/2026-10-31_t18_after_scratchread_00.vcd`: 666.6 ns = 40 x 16.667 ns),
                //     which is why a `RD_CMD` that waits on `nxt` advances and then hangs waiting for a `dir`
                //     that cannot come;
                //   * the only way out is the END marker IN A TRANSMIT CONTEXT, USB3300 §6.1.7: "The USB
                //     Transmit ends when the Link asserts STP while NXT is asserted. (Note that the Link
                //     cannot assert STP with NXT de-asserted since the USB3300 is expecting to fetch another
                //     byte from the Link in this state.)" It explains the half of
                //     `bench/2026-11-01_ulpi_t18_poison/cure_stdout.txt` that says a bare `stp` does NOT cure
                //     it, and it is *a* mechanism for the other half (the RegWrite's END marker landing in the
                //     window) rather than a measured match: §3.8.1's "a transmit ends when the Link stops
                //     driving" explains that row just as well, and nothing here tells them apart. Recorded
                //     because the difference matters to whoever reuses this model, not to the fix — driving the
                //     idle level means there is no unsolicited transmit left to cure. What no reading supports
                //     is what a model that exited on ANY `stp` would have prescribed: the abort path.
                // The window is the cycle AFTER an offer, because that is the cycle in which a Link that
                // *reacts* to `nxt` can have driven `stp`; it is one cycle wide out of forty, so an
                // out-of-context `stp` is 1-in-40 likely to land in it by luck. That co-incidence is a fact
                // about the model and is written here rather than hidden in the comparison.
                S_TX: begin
                    phy_dir  <= 1'b0;
                    phy_data <= 8'h00;
                    phy_oe_r <= 1'b0;              // the PHY is a receiver now: it does not own the pads
                    nxt_window <= (phy_nxt == 1'b1);
                    if (tx_tick >= TX_NXT_PERIOD - 8'd1) begin
                        phy_nxt <= 1'b1;           // the offer: one cycle, once every TX_NXT_PERIOD
                        tx_tick <= 8'd0;
                    end else begin
                        phy_nxt <= 1'b0;
                        tx_tick <= tx_tick + 8'd1;
                    end
                    if (phy_stp && nxt_window) begin
                        state <= S_IDLE;
                        tx_tick <= 8'd0;
                    end
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
                        turn    <= 1'b1;
                        state   <= S_IDLE;
                    end else if (abort_hold == 8'd0) begin
                        phy_dir <= 1'b0;
                        turn    <= 1'b1;
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
