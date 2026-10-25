//
// CPU Throttle
// Fixed-clock execution-rate limiter; memory and peripherals keep running at full speed
//
module cpu_throttle #(
    parameter logic [6:0] CLOCK_RATE_MHZ = 7'd85
)(
    input  logic       clk,
    input  logic       reset_n,
    input  logic [2:0] speed_sel,
    input  logic       active_cycle,
    output logic       hold,
    output logic       release_cycle,
    output logic       full_speed
);

logic [15:0] debt;
logic  [2:0] speed_r;
logic        release_r;

// Keep the original ao486 SYSCTL encodings in 0..3; bit 2 extends them.
logic [6:0] target_mhz;
always_comb begin
    case (speed_r)
        3'd1: target_mhz = 7'd15;
        3'd2: target_mhz = 7'd30;
        3'd3: target_mhz = 7'd56;
        3'd4: target_mhz = 7'd6;
        3'd5: target_mhz = 7'd5;
        3'd6: target_mhz = 7'd4;
        3'd7: target_mhz = 7'd3;
        default: target_mhz = CLOCK_RATE_MHZ;
    endcase
end
wire [15:0] target = {9'd0, target_mhz};
wire  [6:0] charge = CLOCK_RATE_MHZ - target_mhz;
wire [16:0] target_twice = {target, 1'b0};
wire [16:0] debt_sum = {1'b0, debt} + {10'd0, charge};
wire [15:0] debt_repaid = debt - target;

assign full_speed = speed_r == 3'd0 || target_mhz >= CLOCK_RATE_MHZ;
assign release_cycle = release_r;

always_ff @(posedge clk) begin
    if (!reset_n) begin
        debt      <= 16'd0;
        speed_r   <= 3'd0;
        hold      <= 1'b0;
        release_r <= 1'b0;
    end else if (speed_sel != speed_r) begin
        debt      <= 16'd0;
        speed_r   <= speed_sel;
        hold      <= 1'b0;
        release_r <= 1'b0;
    end else if (full_speed) begin
        debt      <= 16'd0;
        hold      <= 1'b0;
        release_r <= 1'b0;
    end else if (active_cycle) begin
        debt      <= debt_sum[16] ? 16'hffff : debt_sum[15:0];
        hold      <= debt_sum >= {1'b0, target};
        release_r <= debt_sum >= {1'b0, target} &&
                     debt_sum < target_twice;
    end else if (hold) begin
        debt      <= debt_repaid;
        hold      <= debt_repaid >= target;
        release_r <= debt_repaid >= target &&
                     {1'b0, debt_repaid} < target_twice;
    end
end

endmodule
