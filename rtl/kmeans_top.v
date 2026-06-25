// =============================================================
//  kmeans_top.v  –  2D K-Means Clustering Hardware Accelerator
//
//  Top-level integration:
//    - 8 input data points (unsigned 8-bit x,y coordinates)
//    - Auto-selects points[0] and points[1] as initial centroids
//    - 8 parallel Processing Elements (distance_pe) for assignment
//    - centroid_updater for accumulation & shift-divide averaging
//    - kmeans_fsm for iteration control & convergence detection
//    - Outputs final two centroid coordinates + done flag
//
//  Target  : xc7z020clg400-1 (Zynq-7000)
//  Clock   : 100 MHz (10 ns period) recommended
// =============================================================
`timescale 1ns / 1ps
`include "kmeans_pkg.vh"

module kmeans_top #(
    parameter DATA_W   = `DATA_W,
    parameter N_POINTS = `N_POINTS,
    parameter DIST_W   = `DIST_W,
    parameter ACC_W    = `ACC_W,
    parameter MAX_ITER = 4'd15
)(
    // ---- Clock & Reset -----------------------------------------
    input                          clk,
    input                          rst_n,  // active-low synchronous reset
    input                          start,  // pulse to begin clustering

    // ---- Input Data Points (flat buses) ------------------------
    // points_x_flat[7:0] = point_0_x, [15:8] = point_1_x, ...
    input  [N_POINTS*DATA_W-1:0]   points_x_flat,
    input  [N_POINTS*DATA_W-1:0]   points_y_flat,

    // ---- Output: Final Centroids -------------------------------
    output [DATA_W-1:0]            cx0_out,  // centroid 0 x
    output [DATA_W-1:0]            cy0_out,  // centroid 0 y
    output [DATA_W-1:0]            cx1_out,  // centroid 1 x
    output [DATA_W-1:0]            cy1_out,  // centroid 1 y

    // ---- Status ------------------------------------------------
    output                         done,        // clustering complete
    output [3:0]                   iter_count,  // iterations performed
    output [2:0]                   state_dbg,   // FSM state (waveform debug)

    // ---- Debug: Cluster ID per point ---------------------------
    output [N_POINTS-1:0]          cluster_ids_out,

    // ---- Debug: dist0 from PE[0] (prevents opt_design pruning) ----
    output [DIST_W-1:0]            dist0_dbg
);

    // ============================================================
    // Internal wires – point coordinate unpack
    // ============================================================
    wire [DATA_W-1:0] px [0:N_POINTS-1];
    wire [DATA_W-1:0] py [0:N_POINTS-1];

    genvar gi;
    generate
        for (gi = 0; gi < N_POINTS; gi = gi+1) begin : gen_unpack
            assign px[gi] = points_x_flat[gi*DATA_W +: DATA_W];
            assign py[gi] = points_y_flat[gi*DATA_W +: DATA_W];
        end
    endgenerate

    // ============================================================
    // Centroid registers  (current & new)
    // ============================================================
    reg [DATA_W-1:0] cx0, cy0;   // current centroid 0
    reg [DATA_W-1:0] cx1, cy1;   // current centroid 1

    wire [DATA_W-1:0] new_cx0, new_cy0;
    wire [DATA_W-1:0] new_cx1, new_cy1;

    assign cx0_out = cx0;
    assign cy0_out = cy0;
    assign cx1_out = cx1;
    assign cy1_out = cy1;

    // ============================================================
    // FSM control signals
    // ============================================================
    wire fsm_assign_en;
    wire fsm_update_en;
    wire fsm_latch_init;
    wire fsm_latch_new;
    wire updater_done;

    // ============================================================
    // PE cluster ID outputs + dist debug
    // ============================================================
    wire [N_POINTS-1:0] cluster_ids;
    assign cluster_ids_out = cluster_ids;

    // dist0 bus from all PEs – index [0] is routed to dist0_dbg top-level
    // output so opt_design cannot prune the distance computation.
    wire [DIST_W-1:0] dist0_arr [0:N_POINTS-1];
    assign dist0_dbg = dist0_arr[0];

    // ============================================================
    // Parallel Processing Elements (one per point)
    // ============================================================
    generate
        for (gi = 0; gi < N_POINTS; gi = gi+1) begin : gen_pe
            distance_pe #(
                .DATA_W (DATA_W),
                .DIST_W (DIST_W)
            ) u_pe (
                .xi         (px[gi]),
                .yi         (py[gi]),
                .cx0        (cx0),
                .cy0        (cy0),
                .cx1        (cx1),
                .cy1        (cy1),
                .cluster_id (cluster_ids[gi]),
                .dist0      (dist0_arr[gi]),  // routed out; prevents pruning
                .dist1      ()                // unused, OK to leave open
            );
        end
    endgenerate

    // ============================================================
    // Centroid Updater
    // ============================================================
    centroid_updater #(
        .DATA_W   (DATA_W),
        .N_POINTS (N_POINTS),
        .ACC_W    (ACC_W)
    ) u_updater (
        .clk            (clk),
        .rst_n          (rst_n),
        .update_en      (fsm_update_en),
        .points_x_flat  (points_x_flat),
        .points_y_flat  (points_y_flat),
        .cluster_ids    (cluster_ids),
        .prev_cx0       (cx0),
        .prev_cy0       (cy0),
        .prev_cx1       (cx1),
        .prev_cy1       (cy1),
        .new_cx0        (new_cx0),
        .new_cy0        (new_cy0),
        .new_cx1        (new_cx1),
        .new_cy1        (new_cy1),
        .done           (updater_done)
    );

    // ============================================================
    // Control FSM
    // ============================================================
    kmeans_fsm #(
        .DATA_W   (DATA_W),
        .MAX_ITER (MAX_ITER)
    ) u_fsm (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start),
        .cur_cx0        (cx0),
        .cur_cy0        (cy0),
        .cur_cx1        (cx1),
        .cur_cy1        (cy1),
        .new_cx0        (new_cx0),
        .new_cy0        (new_cy0),
        .new_cx1        (new_cx1),
        .new_cy1        (new_cy1),
        .updater_done   (updater_done),
        .assign_en      (fsm_assign_en),
        .update_en      (fsm_update_en),
        .latch_init     (fsm_latch_init),
        .latch_new      (fsm_latch_new),
        .done           (done),
        .iter_count     (iter_count),
        .state_dbg      (state_dbg)
    );

    // ============================================================
    // Centroid Register Update Logic
    // Auto-init: points[0] → centroid0, points[1] → centroid1
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cx0 <= {DATA_W{1'b0}};
            cy0 <= {DATA_W{1'b0}};
            cx1 <= {DATA_W{1'b0}};
            cy1 <= {DATA_W{1'b0}};
        end else begin
            if (fsm_latch_init) begin
                // Use first two distinct data points as seeds
                cx0 <= px[0];
                cy0 <= py[0];
                cx1 <= px[1];
                cy1 <= py[1];
            end else if (fsm_latch_new) begin
                // Accept converged new centroids
                cx0 <= new_cx0;
                cy0 <= new_cy0;
                cx1 <= new_cx1;
                cy1 <= new_cy1;
            end
        end
    end

endmodule
