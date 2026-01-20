`timescale 1 ns / 1 ps

module top (
    input  wire clk, rst_n,
    output wire ja_cs, ja_mosi, ja_sclk, ja_cs_m, ja_cs_alt,
    input  wire ja_miso,
    // 生データ出力（既存）
    output wire [15:0] acc_x, acc_y, acc_z, 
    output wire [15:0] gyr_x, gyr_y, gyr_z, 
    output wire [15:0] temp,
    output wire [5:0]  is_minus_out,
    // PL計算済み 物理量出力 (32bit固定小数点: [31]符号 [30:16]整数 [15:0]小数)
    output wire [31:0] fixed_ax, fixed_ay, fixed_az,
    output wire [31:0] fixed_gx, fixed_gy, fixed_gz,
    output wire [31:0] fixed_temp
);
    assign ja_cs_m = 1'b1; 
    assign ja_cs_alt = 1'b1;

    wire [15:0] rx_reg;
    reg  [15:0] tx_reg;
    wire busy;
    reg  start_sig;
    reg  [7:0]  tmp_l;
    reg  [15:0] r_ax, r_ay, r_az, r_gx, r_gy, r_gz, r_temp;

    // --- 既存の出力アサイン ---
    assign acc_x = r_ax; assign acc_y = r_ay; assign acc_z = r_az;
    assign gyr_x = r_gx; assign gyr_y = r_gy; assign gyr_z = r_gz;
    assign temp  = r_temp;
    assign is_minus_out = {r_gz[15], r_gy[15], r_gx[15], r_az[15], r_ay[15], r_ax[15]};

    // --- 物理量計算ロジック (Fixed Point) ---
    // 係数定義: 2^16 (65536) を掛けて整数化
    // Accel: 0.000061g/LSB * 65536 = 3.997... ≒ 4
    // Gyro: 0.00875dps/LSB * 65536 = 573.44 ≒ 573
    
    function [31:0] calc_fixed;
        input signed [15:0] raw;
        input [15:0] factor;
        reg [47:0] product;
        reg [15:0] abs_raw;
        begin
            abs_raw = (raw[15]) ? -raw : raw;
            product = abs_raw * factor;
            // [31]符号, [30:16]整数部, [15:0]小数部
            calc_fixed = {raw[15], product[30:0]};
        end
    endfunction

    assign fixed_ax = calc_fixed(r_ax, 16'd4);   // 0.061mg ≒ 4/65536
    assign fixed_ay = calc_fixed(r_ay, 16'd4);
    assign fixed_az = calc_fixed(r_az, 16'd4);
    assign fixed_gx = calc_fixed(r_gx, 16'd573); // 8.75mdps ≒ 573/65536
    assign fixed_gy = calc_fixed(r_gy, 16'd573);
    assign fixed_gz = calc_fixed(r_gz, 16'd573);

// --- 温度計算の修正 ---
    // 1) 16bitの2の補数（符号付き）として解釈
    wire signed [15:0] s_temp_raw = r_temp;
    
    // 2) 物理量 [degC] = 25 + (raw / 16) の計算
    // raw/16 を行う際、固定小数点(2^16倍)を維持するため 65536/16 = 4096 を掛けます
    wire signed [47:0] t_full_res = (25 * 48'sd65536) + (s_temp_raw * 48'sd4096);
    
    // 3) [31]符号, [30:16]整数部, [15:0]小数部 の形式に整形
    assign fixed_temp = {t_full_res[47], t_full_res[30:0]};

    // --- 以下、以前提供いただいたステートマシン (修正なし) ---
    localparam S_INIT = 0, S_WAKE_ACC = 1, S_WAKE_GYR = 2;
    localparam S_AXL_CMD=3, S_AXL_WAIT=4, S_AXH_CMD=5, S_AXH_WAIT=6;
    localparam S_AYL_CMD=7, S_AYL_WAIT=8, S_AYH_CMD=9, S_AYH_WAIT=10;
    localparam S_AZL_CMD=11, S_AZL_WAIT=12, S_AZH_CMD=13, S_AZH_WAIT=14;
    localparam S_GXL_CMD=15, S_GXL_WAIT=16, S_GXH_CMD=17, S_GXH_WAIT=18;
    localparam S_GYL_CMD=19, S_GYL_WAIT=20, S_GYH_CMD=21, S_GYH_WAIT=22;
    localparam S_GZL_CMD=23, S_GZL_WAIT=24, S_GZH_CMD=25, S_GZH_WAIT=26;
    localparam S_TL_CMD=27, S_TL_WAIT=28, S_TH_CMD=29, S_TH_WAIT=30;

    reg [5:0] state = S_INIT;
    reg [27:0] timer = 0;
    reg [7:0] wait_cnt = 0;

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_INIT; timer <= 0; wait_cnt <= 0; start_sig <= 0;
            {r_ax, r_ay, r_az, r_gx, r_gy, r_gz, r_temp} <= 0;
        end else begin
            case (state)
                S_INIT: begin
                    if (timer < 28'd25_000_000) timer <= timer + 1;
                    else begin timer <= 0; state <= S_WAKE_ACC; end
                end
                S_WAKE_ACC: begin
                    if (!busy && !start_sig && wait_cnt == 0) begin tx_reg <= 16'h1060; start_sig <= 1; end
                    else if (busy) begin start_sig <= 0; wait_cnt <= 1; end
                    else if (!busy && wait_cnt > 0) begin
                        if (wait_cnt < 8'd100) wait_cnt <= wait_cnt + 1;
                        else begin wait_cnt <= 0; state <= S_WAKE_GYR; end
                    end
                end
                S_WAKE_GYR: begin
                    if (!busy && !start_sig && wait_cnt == 0) begin tx_reg <= 16'h1160; start_sig <= 1; end
                    else if (busy) begin start_sig <= 0; wait_cnt <= 1; end
                    else if (!busy && wait_cnt > 0) begin
                        if (wait_cnt < 8'd100) wait_cnt <= wait_cnt + 1;
                        else begin wait_cnt <= 0; state <= S_AXL_CMD; end
                    end
                end
                S_AXL_CMD: begin tx_reg <= 16'hA800; start_sig <= 1; state <= S_AXL_WAIT; end
                S_AXL_WAIT: if (busy) start_sig <= 0; else if (!busy && !start_sig) begin
                    if (wait_cnt < 8'd50) wait_cnt <= wait_cnt + 1;
                    else begin tmp_l <= rx_reg[7:0]; wait_cnt <= 0; state <= S_AXH_CMD; end
                end
                S_AXH_CMD: begin tx_reg <= 16'hA900; start_sig <= 1; state <= S_AXH_WAIT; end
                S_AXH_WAIT: if (busy) start_sig <= 0; else if (!busy && !start_sig) begin
                    if (wait_cnt < 8'd50) wait_cnt <= wait_cnt + 1;
                    else begin r_ax <= {rx_reg[7:0], tmp_l}; wait_cnt <= 0; state <= S_AYL_CMD; end
                end
                S_AYL_CMD: begin tx_reg <= 16'hAA00; start_sig <= 1; state <= S_AYL_WAIT; end
                S_AYL_WAIT: if (busy) start_sig <= 0; else if (!busy && !start_sig) begin
                    if (wait_cnt < 8'd50) wait_cnt <= wait_cnt + 1;
                    else begin tmp_l <= rx_reg[7:0]; wait_cnt <= 0; state <= S_AYH_CMD; end
                end
                S_AYH_CMD: begin tx_reg <= 16'hAB00; start_sig <= 1; state <= S_AYH_WAIT; end
                S_AYH_WAIT: if (busy) start_sig <= 0; else if (!busy && !start_sig) begin
                    if (wait_cnt < 8'd50) wait_cnt <= wait_cnt + 1;
                    else begin r_ay <= {rx_reg[7:0], tmp_l}; wait_cnt <= 0; state <= S_AZL_CMD; end
                end
                S_AZL_CMD: begin tx_reg <= 16'hAC00; start_sig <= 1; state <= S_AZL_WAIT; end
                S_AZL_WAIT: if (busy) start_sig <= 0; else if (!busy && !start_sig) begin
                    if (wait_cnt < 8'd50) wait_cnt <= wait_cnt + 1;
                    else begin tmp_l <= rx_reg[7:0]; wait_cnt <= 0; state <= S_AZH_CMD; end
                end
                S_AZH_CMD: begin tx_reg <= 16'hAD00; start_sig <= 1; state <= S_AZH_WAIT; end
                S_AZH_WAIT: if (busy) start_sig <= 0; else if (!busy && !start_sig) begin
                    if (wait_cnt < 8'd50) wait_cnt <= wait_cnt + 1;
                    else begin r_az <= {rx_reg[7:0], tmp_l}; wait_cnt <= 0; state <= S_GXL_CMD; end
                end
                S_GXL_CMD: begin tx_reg <= 16'h9800; start_sig <= 1; state <= S_GXL_WAIT; end
                S_GXL_WAIT: if (busy) start_sig <= 0; else if (!busy && !start_sig) begin
                    if (wait_cnt < 8'd50) wait_cnt <= wait_cnt + 1;
                    else begin tmp_l <= rx_reg[7:0]; wait_cnt <= 0; state <= S_GXH_CMD; end
                end
                S_GXH_CMD: begin tx_reg <= 16'h9900; start_sig <= 1; state <= S_GXH_WAIT; end
                S_GXH_WAIT: if (busy) start_sig <= 0; else if (!busy && !start_sig) begin
                    if (wait_cnt < 8'd50) wait_cnt <= wait_cnt + 1;
                    else begin r_gx <= {rx_reg[7:0], tmp_l}; wait_cnt <= 0; state <= S_GYL_CMD; end
                end
                S_GYL_CMD: begin tx_reg <= 16'h9A00; start_sig <= 1; state <= S_GYL_WAIT; end
                S_GYL_WAIT: if (busy) start_sig <= 0; else if (!busy && !start_sig) begin
                    if (wait_cnt < 8'd50) wait_cnt <= wait_cnt + 1;
                    else begin tmp_l <= rx_reg[7:0]; wait_cnt <= 0; state <= S_GYH_CMD; end
                end
                S_GYH_CMD: begin tx_reg <= 16'h9B00; start_sig <= 1; state <= S_GYH_WAIT; end
                S_GYH_WAIT: if (busy) start_sig <= 0; else if (!busy && !start_sig) begin
                    if (wait_cnt < 8'd50) wait_cnt <= wait_cnt + 1;
                    else begin r_gy <= {rx_reg[7:0], tmp_l}; wait_cnt <= 0; state <= S_GZL_CMD; end
                end
                S_GZL_CMD: begin tx_reg <= 16'h9C00; start_sig <= 1; state <= S_GZL_WAIT; end
                S_GZL_WAIT: if (busy) start_sig <= 0; else if (!busy && !start_sig) begin
                    if (wait_cnt < 8'd50) wait_cnt <= wait_cnt + 1;
                    else begin tmp_l <= rx_reg[7:0]; wait_cnt <= 0; state <= S_GZH_CMD; end
                end
                S_GZH_CMD: begin tx_reg <= 16'h9D00; start_sig <= 1; state <= S_GZH_WAIT; end
                S_GZH_WAIT: if (busy) start_sig <= 0; else if (!busy && !start_sig) begin
                    if (wait_cnt < 8'd50) wait_cnt <= wait_cnt + 1;
                    else begin r_gz <= {rx_reg[7:0], tmp_l}; wait_cnt <= 0; state <= S_TL_CMD; end
                end
                S_TL_CMD: begin tx_reg <= 16'h9500; start_sig <= 1; state <= S_TL_WAIT; end
                S_TL_WAIT: if (busy) start_sig <= 0; else if (!busy && !start_sig) begin
                    if (wait_cnt < 8'd50) wait_cnt <= wait_cnt + 1;
                    else begin tmp_l <= rx_reg[7:0]; wait_cnt <= 0; state <= S_TH_CMD; end
                end
                S_TH_CMD: begin tx_reg <= 16'h9600; start_sig <= 1; state <= S_TH_WAIT; end
                S_TH_WAIT: if (busy) start_sig <= 0; else if (!busy && !start_sig) begin
                    if (wait_cnt < 8'd50) wait_cnt <= wait_cnt + 1;
                    else begin r_temp <= {rx_reg[7:0], tmp_l}; wait_cnt <= 0; state <= S_AXL_CMD; end
                end
                default: state <= S_INIT;
            endcase
        end
    end

    spi_master u_spi (
        .clk(clk), .rst_n(rst_n), .start(start_sig), .tx_data(tx_reg), .rx_data(rx_reg),
        .busy(busy), .sclk(ja_sclk), .mosi(ja_mosi), .miso(ja_miso), .cs(ja_cs)
    );
endmodule