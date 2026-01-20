module spi_master (
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire [15:0] tx_data,
    output reg  [15:0] rx_data,
    output reg  busy,
    output reg  sclk,
    output reg  mosi,
    input  wire miso,
    output reg  cs
);

    // 速度設定: 約250kHz
    localparam DIV_COUNT = 8'd250; 
    localparam IDLE = 0, CS_LEAD = 1, SCLK_LOW = 2, SCLK_HIGH = 3, CS_LAG = 4, DONE = 5;

    reg [2:0] state;
    reg [7:0] clk_div;
    reg [4:0] bit_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; busy <= 0; sclk <= 0; mosi <= 0; cs <= 1; rx_data <= 0;
            clk_div <= 0; bit_cnt <= 0;
        end else begin
            case (state)
                IDLE: begin
                    busy <= 0;
                    cs <= 1;
                    sclk <= 0;
                    clk_div <= 0;
                    if (start) begin
                        busy <= 1;
                        mosi <= tx_data[15]; // 最初のビットを準備
                        state <= CS_LEAD;
                    end
                end

                // CSを下げてからクロック開始までの待ち
                CS_LEAD: begin
                    cs <= 0;
                    if (clk_div < DIV_COUNT) clk_div <= clk_div + 1;
                    else begin
                        clk_div <= 0;
                        state <= SCLK_LOW;
                        bit_cnt <= 5'd15;
                    end
                end

                // クロックLow区間：ここでMOSIを安定させる
                SCLK_LOW: begin
                    if (clk_div < DIV_COUNT) clk_div <= clk_div + 1;
                    else begin
                        clk_div <= 0;
                        sclk    <= 1; // 立ち上げ
                        state   <= SCLK_HIGH;
                    end
                end

                // クロックHigh区間：立ち上がり直後（データが安定している間）に取り込む
                SCLK_HIGH: begin
                    if (clk_div == DIV_COUNT/2) begin
                        // ★立ち上がり後、少し時間が経過してからMISOを取り込む（ノイズ対策）
                        rx_data[bit_cnt] <= miso;
                    end

                    if (clk_div < DIV_COUNT) clk_div <= clk_div + 1;
                    else begin
                        clk_div <= 0;
                        sclk <= 0; // 立ち下げ
                        if (bit_cnt == 0) begin
                            state <= CS_LAG;
                        end else begin
                            bit_cnt <= bit_cnt - 1;
                            mosi    <= tx_data[bit_cnt-1]; // 次の送信データを準備
                            state   <= SCLK_LOW;
                        end
                    end
                end

                // 通信終了後の余韻
                CS_LAG: begin
                    if (clk_div < DIV_COUNT) clk_div <= clk_div + 1;
                    else begin
                        clk_div <= 0;
                        cs <= 1;
                        state <= DONE;
                    end
                end

                DONE: begin
                    busy <= 0;
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule