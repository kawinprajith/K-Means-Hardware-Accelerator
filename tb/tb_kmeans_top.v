// =============================================================
//  tb_kmeans_top.v  –  Testbench for 2D K-Means Accelerator
//
//  Test dataset (8 points forming 2 clear clusters):
//    Cluster A (near 12, 14):  (10,12), (14,10), (12,16), (10,14)
//    Cluster B (near 50, 52):  (48,50), (52,48), (50,54), (52,52)
//
//  Expected final centroids after convergence:
//    Centroid 0 ≈ (11, 13)  [cluster A average: shift-divide >>2]
//    Centroid 1 ≈ (50, 51)  [cluster B average: shift-divide >>2]
//
//  Self-checking: $error if centroids deviate by more than ±3
//
//  Target : xc7z020clg400-1 (Zynq-7000)
// =============================================================
`timescale 1ns / 1ps

module tb_kmeans_top;

    // --------------------------------------------------------
    // Parameters matching DUT defaults
    // --------------------------------------------------------
    localparam DATA_W   = 8;
    localparam N_POINTS = 8;
    localparam CLK_HALF = 5;        // 10 ns period = 100 MHz

    // --------------------------------------------------------
    // DUT signals
    // --------------------------------------------------------
    reg                          clk;
    reg                          rst_n;
    reg                          start;
    reg  [N_POINTS*DATA_W-1:0]   points_x_flat;
    reg  [N_POINTS*DATA_W-1:0]   points_y_flat;

    wire [DATA_W-1:0]            cx0_out, cy0_out;
    wire [DATA_W-1:0]            cx1_out, cy1_out;
    wire                         done;
    wire [3:0]                   iter_count;
    wire [2:0]                   state_dbg;
    wire [N_POINTS-1:0]          cluster_ids_out;

    // --------------------------------------------------------
    // DUT instantiation
    // --------------------------------------------------------
    kmeans_top #(
        .DATA_W   (DATA_W),
        .N_POINTS (N_POINTS),
        .MAX_ITER (4'd15)
    ) dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .start           (start),
        .points_x_flat   (points_x_flat),
        .points_y_flat   (points_y_flat),
        .cx0_out         (cx0_out),
        .cy0_out         (cy0_out),
        .cx1_out         (cx1_out),
        .cy1_out         (cy1_out),
        .done            (done),
        .iter_count      (iter_count),
        .state_dbg       (state_dbg),
        .cluster_ids_out (cluster_ids_out)
    );

    // --------------------------------------------------------
    // Clock generation  (100 MHz)
    // --------------------------------------------------------
    initial clk = 1'b0;
    always #CLK_HALF clk = ~clk;

    // --------------------------------------------------------
    // State name for display
    // --------------------------------------------------------
    function [63:0] state_name;
        input [2:0] s;
        begin
            case (s)
                3'd0: state_name = "IDLE   ";
                3'd1: state_name = "INIT   ";
                3'd2: state_name = "ASSIGN ";
                3'd3: state_name = "UPDATE ";
                3'd4: state_name = "CHECK  ";
                3'd5: state_name = "DONE   ";
                default: state_name = "UNKNOWN";
            endcase
        end
    endfunction

    // --------------------------------------------------------
    // Monitor: print centroid values each iteration
    // --------------------------------------------------------
    always @(posedge clk) begin
        if (state_dbg == 3'd5) begin   // DONE state
            $display("[%0t ns] DONE after %0d iteration(s)", $time, iter_count);
            $display("  Final Centroid 0 : (%0d, %0d)", cx0_out, cy0_out);
            $display("  Final Centroid 1 : (%0d, %0d)", cx1_out, cy1_out);
            $display("  Cluster IDs      : %08b", cluster_ids_out);
        end
    end

    // Monitor state transitions
    reg [2:0] prev_state;
    always @(posedge clk) begin
        if (state_dbg !== prev_state) begin
            $display("[%0t ns] FSM -> %s  (iter=%0d)", $time, state_name(state_dbg), iter_count);
            prev_state <= state_dbg;
        end
    end

    // --------------------------------------------------------
    // Helper task: absolute difference for checking
    // --------------------------------------------------------
    function integer abs_diff;
        input integer a, b;
        begin
            abs_diff = (a > b) ? (a - b) : (b - a);
        end
    endfunction

    // --------------------------------------------------------
    // Main test sequence
    // --------------------------------------------------------
    integer timeout;

    initial begin
        $display("==============================================");
        $display("  2D K-Means Hardware Accelerator Testbench  ");
        $display("  Target: xc7z020clg400-1  N=%0d  DATA_W=%0d",
                  N_POINTS, DATA_W);
        $display("==============================================");

        // ---- Load 8 data points into flat buses ----
        // Cluster A: centroid should converge near (11, 13)
        //   points[0]=(10,12)  points[1]=(14,10)
        //   points[2]=(12,16)  points[3]=(10,14)
        // Cluster B: centroid should converge near (50, 51)
        //   points[4]=(48,50)  points[5]=(52,48)
        //   points[6]=(50,54)  points[7]=(52,52)

        points_x_flat = {
            8'd52, 8'd50, 8'd52, 8'd48,   // points [7:4] x
            8'd10, 8'd12, 8'd14, 8'd10    // points [3:0] x  (MSB=point7, LSB=point0)
        };
        points_y_flat = {
            8'd52, 8'd54, 8'd48, 8'd50,   // points [7:4] y
            8'd14, 8'd16, 8'd10, 8'd12    // points [3:0] y
        };

        // ---- Initial reset ----
        rst_n  = 1'b0;
        start  = 1'b0;
        prev_state = 3'd7;  // invalid, forces first display

        repeat(4) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        // ---- Pulse start ----
        $display("[%0t ns] Asserting START", $time);
        start = 1'b1;
        @(posedge clk);
        start = 1'b0;

        // ---- Wait for done with timeout ----
        timeout = 0;
        while (!done && timeout < 500) begin
            @(posedge clk);
            timeout = timeout + 1;
        end

        if (timeout >= 500) begin
            $error("TIMEOUT: clustering did not complete within 500 cycles");
        end else begin
            // ---- Self-checking assertions ----
            $display("");
            $display("--- Self-Check Results ---");

            // Determine which output corresponds to which cluster
            // (seed order may swap depending on which seed comes first)
            if (cx0_out < 8'd30) begin
                // cx0 = cluster A, cx1 = cluster B
                if (abs_diff(cx0_out, 11) > 3 || abs_diff(cy0_out, 13) > 3)
                    $error("FAIL: Centroid 0 (%0d,%0d) deviates > 3 from expected (11,13)",
                            cx0_out, cy0_out);
                else
                    $display("PASS: Centroid 0 (%0d,%0d) within tolerance of (11,13)",
                              cx0_out, cy0_out);

                if (abs_diff(cx1_out, 50) > 3 || abs_diff(cy1_out, 51) > 3)
                    $error("FAIL: Centroid 1 (%0d,%0d) deviates > 3 from expected (50,51)",
                            cx1_out, cy1_out);
                else
                    $display("PASS: Centroid 1 (%0d,%0d) within tolerance of (50,51)",
                              cx1_out, cy1_out);
            end else begin
                // cx0 = cluster B, cx1 = cluster A
                if (abs_diff(cx0_out, 50) > 3 || abs_diff(cy0_out, 51) > 3)
                    $error("FAIL: Centroid 0 (%0d,%0d) deviates > 3 from expected (50,51)",
                            cx0_out, cy0_out);
                else
                    $display("PASS: Centroid 0 (%0d,%0d) within tolerance of (50,51)",
                              cx0_out, cy0_out);

                if (abs_diff(cx1_out, 11) > 3 || abs_diff(cy1_out, 13) > 3)
                    $error("FAIL: Centroid 1 (%0d,%0d) deviates > 3 from expected (11,13)",
                            cx1_out, cy1_out);
                else
                    $display("PASS: Centroid 1 (%0d,%0d) within tolerance of (11,13)",
                              cx1_out, cy1_out);
            end

            $display("  Iterations used : %0d", iter_count);
            $display("  Cluster IDs     : %08b (expected: 00001111 or 11110000)", cluster_ids_out);
        end

        $display("");
        $display("==============================================");
        $display("  Simulation complete");
        $display("==============================================");

        repeat(10) @(posedge clk);
        $finish;
    end

    // --------------------------------------------------------
    // Dump waveforms (Vivado Simulator)
    // --------------------------------------------------------
    initial begin
        $dumpfile("kmeans_sim.vcd");
        $dumpvars(0, tb_kmeans_top);
    end

endmodule
