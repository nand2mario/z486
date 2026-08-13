// Fixed-clock architectural execution-rate controller. Memory and peripherals
// continue at clk speed while execution cycles accrue and repay rate debt.
module cpu_throttle #(
    parameter logic [6:0] CLOCK_RATE_MHZ = 7'd85
)(
    input  logic       clk,
    input  logic       reset_n,
    input  logic [1:0] speed_sel,
    input  logic       active_cycle,
    output logic       hold,
    output logic       release_cycle,
    output logic       full_speed
);

logic [15:0] debt;
logic  [1:0] speed_r;
logic        release_r;

wire  [6:0] target_mhz = speed_r == 2'd1 ? 7'd15 :
                         speed_r == 2'd2 ? 7'd30 : 7'd56;
wire [15:0] target = {9'd0, target_mhz};
wire  [6:0] charge = CLOCK_RATE_MHZ - target_mhz;
wire [16:0] target_twice = {target, 1'b0};
wire [16:0] debt_sum = {1'b0, debt} + {10'd0, charge};
wire [15:0] debt_repaid = debt - target;

assign full_speed = speed_r == 2'd0 || target_mhz >= CLOCK_RATE_MHZ;
assign release_cycle = release_r;

always_ff @(posedge clk) begin
    if (!reset_n) begin
        debt      <= 16'd0;
        speed_r   <= 2'd0;
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
