`ifndef ULPI_PHY_DEFS_VH
`define ULPI_PHY_DEFS_VH
// -----------------------------------------------------------------------------
// The three numbers both sides of this simulation need, spelled once.
//
// They are *not* copied from our Python: they are copied from ULPI 1.1 Table 6 and the USB3300 datasheet,
// which is the point of having a second model. If a future edit "fixes" one of these to match whatever the
// migen constant happens to be, the simulation stops being a check and becomes a mirror.
// -----------------------------------------------------------------------------

// ULPI 1.1 Table 6, TX CMD bits 7:6.
`define ULPI_CMD_IDLE   2'b00   // the idle *level*, not something to send
`define ULPI_CMD_PACKET 2'b01   // NextData + PID -- Track A's job, never ours
`define ULPI_CMD_WR     2'b10   // RegWrite:  10b | 6-bit address
`define ULPI_CMD_RD     2'b11   // RegRead:   11b | 6-bit address

// The same two commands as a whole byte, for code that must name the byte rather than the field.
// `ULPI_CMD_WR` is 2 bits *by design* -- that is the width Table 6 defines -- and the temptation to write
// `ULPI_CMD_WR | addr` is a real trap: `2'b10 | 8'h00` is `8'h02`, not `8'h80`. The first version of the
// testbench's encoding cross-check did exactly that and failed every row in the file.
`define ULPI_BYTE_WR  8'h80   // RegWrite: 10b in 7:6, immediate address in 5:0
`define ULPI_BYTE_RD  8'hC0   // RegRead:  11b in 7:6, immediate address in 5:0

// The USB3300's own ID registers (datasheet Table 6.3 / §6.1.4), low byte first.
`define ULPI_REG_VID_LO 6'h00
`define ULPI_REG_VID_HI 6'h01
`define ULPI_REG_PID_LO 6'h02
`define ULPI_REG_PID_HI 6'h03
`define ULPI_REG_FUNC   6'h04   // Function Control,  POR 0x41
`define ULPI_REG_IFC    6'h07   // Interface Control, POR 0x00
`define ULPI_REG_DEBUG  6'h15   // bits 1:0 = Linestate0/1 -- the pre-enumeration witness
`define ULPI_REG_SCRATCH 6'h16  // "read/write it, PHY functionality will not be affected"

`endif
