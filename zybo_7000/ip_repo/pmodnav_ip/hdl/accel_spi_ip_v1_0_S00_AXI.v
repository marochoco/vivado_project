`timescale 1 ns / 1 ps

module pmodnav_v1_0_S00_AXI #
(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 7
)
(
    output wire ja_cs, ja_mosi, ja_sclk, ja_cs_m, ja_cs_alt, input wire ja_miso,
    input wire S_AXI_ACLK, S_AXI_ARESETN,
    input wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_AWADDR,
    input wire [2 : 0] S_AXI_AWPROT,
    input wire S_AXI_AWVALID, output wire S_AXI_AWREADY,
    input wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_WDATA,
    input wire [(C_S_AXI_DATA_WIDTH/8)-1 : 0] S_AXI_WSTRB,
    input wire S_AXI_WVALID, output wire S_AXI_WREADY,
    output wire [1 : 0] S_AXI_BRESP, output wire S_AXI_BVALID,
    input wire S_AXI_BREADY,
    input wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_ARADDR,
    input wire [2 : 0] S_AXI_ARPROT,
    input wire S_AXI_ARVALID, output wire S_AXI_ARREADY,
    output wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_RDATA,
    output wire [1 : 0] S_AXI_RRESP, output wire S_AXI_RVALID,
    input wire S_AXI_RREADY
);
    reg [C_S_AXI_ADDR_WIDTH-1 : 0] axi_awaddr, axi_araddr;
    reg axi_awready, axi_wready, axi_bvalid, axi_arready, axi_rvalid;
    reg [C_S_AXI_DATA_WIDTH-1:0] axi_rdata;

    // レジスタ実体
    reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg [0:31];

    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = axi_wready;
    assign S_AXI_BRESP   = 2'b00;
    assign S_AXI_BVALID  = axi_bvalid;
    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RDATA   = axi_rdata;
    assign S_AXI_RRESP   = 2'b00;
    assign S_AXI_RVALID  = axi_rvalid;

    // AXI制御ロジック
    always @( posedge S_AXI_ACLK ) begin
        if ( S_AXI_ARESETN == 1'b0 ) begin
            axi_awready <= 0; axi_wready <= 0; axi_bvalid <= 0; axi_arready <= 0; axi_rvalid <= 0;
        end else begin
            if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID) axi_awready <= 1; else axi_awready <= 0;
            if (~axi_wready && S_AXI_WVALID && S_AXI_AWVALID) axi_wready <= 1; else axi_wready <= 0;
            if (axi_awready && S_AXI_AWVALID && axi_wready && S_AXI_WVALID && ~axi_bvalid) axi_bvalid <= 1;
            else if (S_AXI_BREADY && axi_bvalid) axi_bvalid <= 0;
            if (~axi_arready && S_AXI_ARVALID) axi_arready <= 1; else axi_arready <= 0;
            if (axi_arready && S_AXI_ARVALID && ~axi_rvalid) axi_rvalid <= 1;
            else if (axi_rvalid && S_AXI_RREADY) axi_rvalid <= 0;
        end
    end

    // アドレスラッチ
    always @( posedge S_AXI_ACLK ) begin
        if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID) axi_awaddr <= S_AXI_AWADDR;
        if (~axi_arready && S_AXI_ARVALID) axi_araddr <= S_AXI_ARADDR;
    end

    // --- top からのデータ接続用ワイヤ ---
    wire [15:0] w_ax, w_ay, w_az, w_gx, w_gy, w_gz, w_temp;
    wire [5:0]  w_minus;
    wire [31:0] w_f_ax, w_f_ay, w_f_az, w_f_gx, w_f_gy, w_f_gz, w_f_tp;

    // レジスタへの流し込み
    always @( posedge S_AXI_ACLK ) begin
        if ( S_AXI_ARESETN == 1'b0 ) begin
            // リセット時は全レジスタクリア（必要に応じて）
        end else begin
            // 0-7: 生データおよび符号情報
            slv_reg[0] <= {16'b0, w_ax};    // 0x00
            slv_reg[1] <= {16'b0, w_ay};    // 0x04
            slv_reg[2] <= {16'b0, w_az};    // 0x08
            slv_reg[3] <= {16'b0, w_gx};    // 0x0C
            slv_reg[4] <= {16'b0, w_gy};    // 0x10
            slv_reg[5] <= {16'b0, w_gz};    // 0x14
            slv_reg[6] <= {16'b0, w_temp};  // 0x18
            slv_reg[7] <= {26'b0, w_minus}; // 0x1C

            // 8-14: PL計算済みの固定小数点データ (Base + 0x20 〜)
            slv_reg[8]  <= w_f_ax;          // 0x20
            slv_reg[9]  <= w_f_ay;          // 0x24
            slv_reg[10] <= w_f_az;          // 0x28
            slv_reg[11] <= w_f_gx;          // 0x2C
            slv_reg[12] <= w_f_gy;          // 0x30
            slv_reg[13] <= w_f_gz;          // 0x34
            slv_reg[14] <= w_f_tp;          // 0x38
        end
    end

    always @(*) begin
        axi_rdata <= slv_reg[axi_araddr[C_S_AXI_ADDR_WIDTH-1:2]];
    end

    // topモジュールのインスタンス化
    top u_top (
        .clk(S_AXI_ACLK), .rst_n(S_AXI_ARESETN),
        .ja_cs(ja_cs), .ja_mosi(ja_mosi), .ja_sclk(ja_sclk), .ja_cs_m(ja_cs_m), .ja_cs_alt(ja_cs_alt), .ja_miso(ja_miso),
        .acc_x(w_ax), .acc_y(w_ay), .acc_z(w_az),
        .gyr_x(w_gx), .gyr_y(w_gy), .gyr_z(w_gz),
        .temp(w_temp), .is_minus_out(w_minus),
        // 計算済み物理量の接続
        .fixed_ax(w_f_ax), .fixed_ay(w_f_ay), .fixed_az(w_f_az),
        .fixed_gx(w_f_gx), .fixed_gy(w_f_gy), .fixed_gz(w_f_gz),
        .fixed_temp(w_f_tp)
    );

endmodule