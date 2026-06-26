// =============================================================
//  kmeans_axi_wrapper.v  -  AXI4-Lite Slave Wrapper
//
//  Wraps kmeans_top so the ARM Cortex-A9 PS can:
//    - Write 8 point coordinates via AXI-Lite registers
//    - Pulse START to begin clustering
//    - Poll STATUS / read centroid results
//
//  Register Map (byte address, 32-bit words):
//  +---------+----+------------------------------------------+
//  | Offset  | RW | Description                              |
//  +---------+----+------------------------------------------+
//  | 0x00    | RW | CTRL  [0]=start (self-clearing)          |
//  | 0x04    | RO | STATUS[0]=done, [4:1]=iter_count,        |
//  |         |    |        [7:5]=state_dbg, [15:8]=clust_ids |
//  | 0x08    | RW | PX_LO  points 0-3 x-coords (8b each)    |
//  | 0x0C    | RW | PX_HI  points 4-7 x-coords (8b each)    |
//  | 0x10    | RW | PY_LO  points 0-3 y-coords (8b each)    |
//  | 0x14    | RW | PY_HI  points 4-7 y-coords (8b each)    |
//  | 0x18    | RO | RESULT {cy1,cx1,cy0,cx0} (8b each)       |
//  | 0x1C    | RO | DIST0  dist0_dbg[17:0]                   |
//  +---------+----+------------------------------------------+
//
//  Target : xc7z020clg400-1 (Zynq-7000)
// =============================================================
`timescale 1ns / 1ps

module kmeans_axi_wrapper #(
    parameter C_S_AXI_DATA_WIDTH = 32,
    parameter C_S_AXI_ADDR_WIDTH = 5    // 2^5 = 32 bytes = 8 registers
)(
    // ---- AXI4-Lite Slave Interface ----------------------------
    input  wire                              S_AXI_ACLK,
    input  wire                              S_AXI_ARESETN,  // active-low

    // Write address channel
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]    S_AXI_AWADDR,
    input  wire [2:0]                        S_AXI_AWPROT,
    input  wire                              S_AXI_AWVALID,
    output wire                              S_AXI_AWREADY,

    // Write data channel
    input  wire [C_S_AXI_DATA_WIDTH-1:0]    S_AXI_WDATA,
    input  wire [C_S_AXI_DATA_WIDTH/8-1:0]  S_AXI_WSTRB,
    input  wire                              S_AXI_WVALID,
    output wire                              S_AXI_WREADY,

    // Write response channel
    output wire [1:0]                        S_AXI_BRESP,
    output wire                              S_AXI_BVALID,
    input  wire                              S_AXI_BREADY,

    // Read address channel
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]    S_AXI_ARADDR,
    input  wire [2:0]                        S_AXI_ARPROT,
    input  wire                              S_AXI_ARVALID,
    output wire                              S_AXI_ARREADY,

    // Read data channel
    output wire [C_S_AXI_DATA_WIDTH-1:0]    S_AXI_RDATA,
    output wire [1:0]                        S_AXI_RRESP,
    output wire                              S_AXI_RVALID,
    input  wire                              S_AXI_RREADY
);

    // ============================================================
    // AXI handshake registers
    // ============================================================
    reg  axi_awready, axi_wready, axi_bvalid;
    reg  axi_arready, axi_rvalid;
    reg  [C_S_AXI_DATA_WIDTH-1:0] axi_rdata;
    reg  [1:0] axi_bresp, axi_rresp;

    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = axi_wready;
    assign S_AXI_BRESP   = axi_bresp;
    assign S_AXI_BVALID  = axi_bvalid;
    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RDATA   = axi_rdata;
    assign S_AXI_RRESP   = axi_rresp;
    assign S_AXI_RVALID  = axi_rvalid;

    // ============================================================
    // Internal registers  (mirror of register map)
    // ============================================================
    reg [31:0] reg_ctrl;    // 0x00
    // reg_status is read-only from hardware signals (0x04)
    reg [31:0] reg_px_lo;   // 0x08  points 0-3 x
    reg [31:0] reg_px_hi;   // 0x0C  points 4-7 x
    reg [31:0] reg_py_lo;   // 0x10  points 0-3 y
    reg [31:0] reg_py_hi;   // 0x14  points 4-7 y

    // ============================================================
    // kmeans_top wires
    // ============================================================
    wire [63:0] points_x_flat = {reg_px_hi, reg_px_lo};
    wire [63:0] points_y_flat = {reg_py_hi, reg_py_lo};

    wire        start_pulse;
    wire [7:0]  cx0_out, cy0_out, cx1_out, cy1_out;
    wire        done;
    wire [3:0]  iter_count;
    wire [2:0]  state_dbg;
    wire [7:0]  cluster_ids_out;
    wire [17:0] dist0_dbg;

    // start is bit [0] of CTRL register; self-clears next cycle
    reg start_r;
    assign start_pulse = start_r;

    always @(posedge S_AXI_ACLK or negedge S_AXI_ARESETN) begin
        if (!S_AXI_ARESETN)
            start_r <= 1'b0;
        else
            start_r <= reg_ctrl[0]; // pulse for one cycle then ARM clears bit
    end

    // ============================================================
    // kmeans_top instantiation
    // ============================================================
    kmeans_top #(
        .DATA_W   (8),
        .N_POINTS (8),
        .DIST_W   (18),
        .ACC_W    (11),
        .MAX_ITER (4'd15)
    ) u_kmeans (
        .clk             (S_AXI_ACLK),
        .rst_n           (S_AXI_ARESETN),
        .start           (start_pulse),
        .points_x_flat   (points_x_flat),
        .points_y_flat   (points_y_flat),
        .cx0_out         (cx0_out),
        .cy0_out         (cy0_out),
        .cx1_out         (cx1_out),
        .cy1_out         (cy1_out),
        .done            (done),
        .iter_count      (iter_count),
        .state_dbg       (state_dbg),
        .cluster_ids_out (cluster_ids_out),
        .dist0_dbg       (dist0_dbg)
    );

    // ============================================================
    // STATUS register (read-only, assembled from hw signals)
    // ============================================================
    wire [31:0] reg_status;
    assign reg_status = {
        8'b0,                // [31:24] reserved
        cluster_ids_out,     // [23:16] one bit per point
        1'b0,                // [15]    reserved
        state_dbg,           // [14:12] FSM state
        iter_count,          // [11:8]  iterations done
        7'b0,                // [7:1]   reserved
        done                 // [0]     clustering complete
    };

    wire [31:0] reg_result;
    assign reg_result = {cy1_out, cx1_out, cy0_out, cx0_out};

    wire [31:0] reg_dist0;
    assign reg_dist0 = {14'b0, dist0_dbg};

    // ============================================================
    // Address decode helpers (word-addressed from bits [4:2])
    // ============================================================
    wire [2:0] wr_addr = S_AXI_AWADDR[4:2];
    wire [2:0] rd_addr = S_AXI_ARADDR[4:2];

    // ============================================================
    // AXI Write Logic
    // ============================================================
    reg aw_active, w_active;
    reg [C_S_AXI_ADDR_WIDTH-1:0] awaddr_latch;

    always @(posedge S_AXI_ACLK or negedge S_AXI_ARESETN) begin
        if (!S_AXI_ARESETN) begin
            axi_awready <= 1'b0;
            axi_wready  <= 1'b0;
            axi_bvalid  <= 1'b0;
            axi_bresp   <= 2'b0;
            aw_active   <= 1'b0;
            w_active    <= 1'b0;
            awaddr_latch<= 0;
            reg_ctrl    <= 32'h0;
            reg_px_lo   <= 32'h0;
            reg_px_hi   <= 32'h0;
            reg_py_lo   <= 32'h0;
            reg_py_hi   <= 32'h0;
        end else begin
            // Default de-assert every cycle
            axi_awready <= 1'b0;
            axi_wready  <= 1'b0;
            // Auto-clear start bit every cycle unless ARM is actively writing it.
            // This is the ONLY driver of reg_ctrl[0] - no second always block.
            reg_ctrl[0] <= 1'b0;

            // Accept write address
            if (S_AXI_AWVALID && !aw_active) begin
                axi_awready  <= 1'b1;
                awaddr_latch <= S_AXI_AWADDR;
                aw_active    <= 1'b1;
            end

            // Accept write data
            if (S_AXI_WVALID && !w_active) begin
                axi_wready <= 1'b1;
                w_active   <= 1'b1;
            end

            // Perform register write when both address and data are ready
            // Non-blocking: the case write overrides the default clear above
            // on the same cycle (last NBA assignment wins).
            if (aw_active && w_active) begin
                aw_active  <= 1'b0;
                w_active   <= 1'b0;
                axi_bvalid <= 1'b1;
                axi_bresp  <= 2'b00; // OKAY

                case (awaddr_latch[4:2])
                    3'd0: reg_ctrl  <= S_AXI_WDATA; // sets [0]=start for 1 cycle
                    3'd2: reg_px_lo <= S_AXI_WDATA;
                    3'd3: reg_px_hi <= S_AXI_WDATA;
                    3'd4: reg_py_lo <= S_AXI_WDATA;
                    3'd5: reg_py_hi <= S_AXI_WDATA;
                    default: ; // ignore writes to RO regs
                endcase
            end

            if (axi_bvalid && S_AXI_BREADY)
                axi_bvalid <= 1'b0;
        end
    end

    // NOTE: reg_ctrl[0] auto-clear is handled inside the write always block above.
    // A second always block here would create a multiple-driver (MDRV) error.

    // ============================================================
    // AXI Read Logic
    // ============================================================
    always @(posedge S_AXI_ACLK or negedge S_AXI_ARESETN) begin
        if (!S_AXI_ARESETN) begin
            axi_arready <= 1'b0;
            axi_rvalid  <= 1'b0;
            axi_rresp   <= 2'b0;
            axi_rdata   <= 32'h0;
        end else begin
            axi_arready <= 1'b0;

            if (S_AXI_ARVALID && !axi_rvalid) begin
                axi_arready <= 1'b1;
                axi_rvalid  <= 1'b1;
                axi_rresp   <= 2'b00; // OKAY

                case (S_AXI_ARADDR[4:2])
                    3'd0: axi_rdata <= reg_ctrl;
                    3'd1: axi_rdata <= reg_status;
                    3'd2: axi_rdata <= reg_px_lo;
                    3'd3: axi_rdata <= reg_px_hi;
                    3'd4: axi_rdata <= reg_py_lo;
                    3'd5: axi_rdata <= reg_py_hi;
                    3'd6: axi_rdata <= reg_result;
                    3'd7: axi_rdata <= reg_dist0;
                    default: axi_rdata <= 32'hDEAD_BEEF;
                endcase
            end

            if (axi_rvalid && S_AXI_RREADY)
                axi_rvalid <= 1'b0;
        end
    end

endmodule
