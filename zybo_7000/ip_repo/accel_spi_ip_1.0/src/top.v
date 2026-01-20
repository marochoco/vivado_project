`timescale 1 ns / 1 ps

module top (
    input  wire clk, rst_n,
    output wire ja_cs, ja_mosi, ja_sclk, ja_cs_m, ja_cs_alt,
    input  wire ja_miso,
    // AXIレジスタに渡すための出力ポート
    output wire [15:0] accel_x_out,
    output wire [15:0] accel_g_int_out,
    output wire [15:0] accel_g_frac_out,
    output wire is_minus_out
);
    // 常時有効化
    assign ja_cs_m = 1'b1; 
    assign ja_cs_alt = 1'b1;

    // 内部信号
    wire [15:0] rx_reg;
    reg  [15:0] tx_reg;
    wire busy;
    reg  start_sig;

    reg [7:0]  accel_x_low;
    reg [15:0] accel_x_combined;

    localparam STATE_INIT   = 0;
    localparam STATE_WAKEUP = 1;
    localparam STATE_READ_L = 2;
    localparam STATE_READ_H = 3;
    
    reg [1:0]  main_state = STATE_INIT;
    reg [27:0] timer = 0;
    reg [3:0]  wait_cnt = 0; 

    // --- 加速度変換ロジック (assignによる組み合わせ回路) ---
    // 0x8000 (32768) 以上をマイナスと判定
    wire is_val_minus = (accel_x_combined >= 16'd32768);
    // マイナスの場合は2の補数を取って絶対値にする
    wire [15:0] abs_val = is_val_minus ? (~accel_x_combined + 1'b1) : accel_x_combined;

    // AXIポートへの出力接続
    assign accel_x_out      = accel_x_combined;
    assign is_minus_out     = is_val_minus;
    assign accel_g_int_out  = abs_val / 16'd16384; // 1g = 16384 LSB

    // 【重要】32bit明示により計算途中のオーバーフロー（16,383,000）を回避
    assign accel_g_frac_out = ( (abs_val % 32'd16384) * 32'd1000 ) / 32'd16384;

    // --- メインの状態マシン ---
    always @(posedge clk) begin 
        if (!rst_n) begin
            main_state <= STATE_INIT; 
            timer <= 0; 
            wait_cnt <= 0; 
            start_sig <= 0;
            accel_x_low <= 8'd0;
            accel_x_combined <= 16'd0;
        end else begin
            case (main_state)
                // 電源投入後のウェイト
                STATE_INIT: begin
                    if (timer < 28'd25_000_000) 
                        timer <= timer + 1; 
                    else begin 
                        timer <= 0; 
                        main_state <= STATE_WAKEUP; 
                    end
                end

                // 加速度センサをアクティブにするコマンド送信
                STATE_WAKEUP: begin
                    if (!busy && !start_sig && wait_cnt == 0) begin 
                        tx_reg <= 16'h1020; // CTRL_REG1 に 0x20 を書き込み
                        start_sig <= 1; 
                    end else if (busy) begin 
                        start_sig <= 0; 
                        wait_cnt <= 1; 
                    end else if (!busy && !start_sig && wait_cnt != 0) begin 
                        if (wait_cnt < 4'd15) wait_cnt <= wait_cnt + 1; 
                        else begin wait_cnt <= 0; main_state <= STATE_READ_L; end 
                    end
                end

                // X軸の下位バイト読み出し
                STATE_READ_L: begin
                    if (timer < 28'd12_500_000) 
                        timer <= timer + 1;
                    else if (!busy && !start_sig && wait_cnt == 0) begin 
                        tx_reg <= 16'hA800; // OUT_X_L 読み出しコマンド
                        start_sig <= 1; 
                    end else if (busy) begin 
                        start_sig <= 0; 
                        wait_cnt <= 1; 
                    end else if (!busy && !start_sig && wait_cnt != 0) begin 
                        if (wait_cnt < 4'd15) wait_cnt <= wait_cnt + 1; 
                        else begin 
                            accel_x_low <= rx_reg[7:0]; 
                            wait_cnt <= 0; 
                            main_state <= STATE_READ_H; 
                        end 
                    end
                end

                // X軸の上位バイト読み出し
                STATE_READ_H: begin
                    if (!busy && !start_sig && wait_cnt == 0) begin 
                        tx_reg <= 16'hA900; // OUT_X_H 読み出しコマンド
                        start_sig <= 1; 
                    end else if (busy) begin 
                        start_sig <= 0; 
                        wait_cnt <= 1; 
                    end else if (!busy && !start_sig && wait_cnt != 0) begin 
                        if (wait_cnt < 4'd15) wait_cnt <= wait_cnt + 1; 
                        else begin 
                            accel_x_combined <= {rx_reg[7:0], accel_x_low}; 
                            wait_cnt <= 0; 
                            timer <= 0; 
                            main_state <= STATE_READ_L; // 繰り返し読み出し
                        end 
                    end
                end
            endcase
        end
    end

    // SPIマスタモジュールのインスタンス
    spi_master u_spi (
        .clk(clk), 
        .rst_n(rst_n), 
        .start(start_sig), 
        .tx_data(tx_reg), 
        .rx_data(rx_reg), 
        .busy(busy), 
        .sclk(ja_sclk), 
        .mosi(ja_mosi), 
        .miso(ja_miso), 
        .cs(ja_cs)
    );

endmodule