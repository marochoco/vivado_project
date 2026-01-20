`timescale 1 ns / 1 ps

module accel_spi_ip_v1_0_S00_AXI #
(
    parameter integer C_S_AXI_DATA_WIDTH    = 32,
    parameter integer C_S_AXI_ADDR_WIDTH    = 4
)
(
    output wire ja_cs, output wire ja_mosi, output wire ja_sclk, input wire ja_miso,
    input wire  S_AXI_ACLK,
    input wire  S_AXI_ARESETN,
    input wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_AWADDR,
    input wire [2 : 0] S_AXI_AWPROT,
    input wire  S_AXI_AWVALID,
    output wire  S_AXI_AWREADY,
    input wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_WDATA,
    input wire [(C_S_AXI_DATA_WIDTH/8)-1 : 0] S_AXI_WSTRB,
    input wire  S_AXI_WVALID,
    output wire  S_AXI_WREADY,
    output wire [1 : 0] S_AXI_BRESP,
    output wire  S_AXI_BVALID,
    input wire  S_AXI_BREADY,
    input wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_ARADDR,
    input wire [2 : 0] S_AXI_ARPROT,
    input wire  S_AXI_ARVALID,
    output wire  S_AXI_ARREADY,
    output wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_RDATA,
    output wire [1 : 0] S_AXI_RRESP,
    output wire  S_AXI_RVALID,
    input wire  S_AXI_RREADY
);

    reg [C_S_AXI_ADDR_WIDTH-1 : 0]  axi_awaddr;
    reg  axi_awready;
    reg  axi_wready;
    reg  axi_bvalid;
    reg [C_S_AXI_ADDR_WIDTH-1 : 0]  axi_araddr;
    reg  axi_arready;
    reg [C_S_AXI_DATA_WIDTH-1 : 0]  axi_rdata;
    reg  axi_rvalid;

    reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg0, slv_reg1, slv_reg2, slv_reg3;

    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = axi_wready;
    assign S_AXI_BRESP   = 2'b00;
    assign S_AXI_BVALID  = axi_bvalid;
    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RDATA   = axi_rdata;
    assign S_AXI_RRESP   = 2'b00;
    assign S_AXI_RVALID  = axi_rvalid;

    // AXI書き込み/読み出し制御 (標準的なLiteプロトコル)
    always @( posedge S_AXI_ACLK ) begin
        if ( S_AXI_ARESETN == 1'b0 ) begin
            axi_awready <= 1'b0; axi_wready <= 1'b0; axi_bvalid <= 1'b0;
            axi_arready <= 1'b0; axi_rvalid <= 1'b0;
        end else begin    
            if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID) axi_awready <= 1'b1; else axi_awready <= 1'b0;
            if (~axi_wready && S_AXI_WVALID && S_AXI_AWVALID) axi_wready <= 1'b1; else axi_wready <= 1'b0;
            if (axi_awready && S_AXI_AWVALID && axi_wready && S_AXI_WVALID && ~axi_bvalid) axi_bvalid <= 1'b1;
            else if (S_AXI_BREADY && axi_bvalid) axi_bvalid <= 1'b0;
            if (~axi_arready && S_AXI_ARVALID) axi_arready <= 1'b1; else axi_arready <= 1'b0;
            if (axi_arready && S_AXI_ARVALID && ~axi_rvalid) axi_rvalid <= 1'b1;
            else if (axi_rvalid && S_AXI_RREADY) axi_rvalid <= 1'b0;
        end
    end

    always @( posedge S_AXI_ACLK ) begin
        if (S_AXI_ARESETN == 1'b0) begin axi_awaddr <= 0; axi_araddr <= 0; end
        else begin
            if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID) axi_awaddr <= S_AXI_AWADDR;
            if (~axi_arready && S_AXI_ARVALID) axi_araddr <= S_AXI_ARADDR;
        end
    end

    // topモジュールとの接続用wire
    wire [15:0] w_raw_x, w_g_int, w_g_frac;
    wire w_is_minus;

    // 加速度データをレジスタに格納
    always @( posedge S_AXI_ACLK ) begin
        if ( S_AXI_ARESETN == 1'b0 ) begin
            slv_reg0 <= 0; slv_reg1 <= 0; slv_reg2 <= 0; slv_reg3 <= 0;
        end else begin
            slv_reg0 <= {16'd0, w_raw_x};
            slv_reg1 <= {31'd0, w_is_minus};
            slv_reg2 <= {16'd0, w_g_int};
            slv_reg3 <= {16'd0, w_g_frac};
        end
    end

    // 読み出しデータ選択
    always @(*) begin
        case ( axi_araddr[3:2] )
            2'h0: axi_rdata <= slv_reg0;
            2'h1: axi_rdata <= slv_reg1;
            2'h2: axi_rdata <= slv_reg2;
            2'h3: axi_rdata <= slv_reg3;
            default: axi_rdata <= 0;
        endcase
    end

    // topモジュールのインスタンス化
    top u_top (
        .clk(S_AXI_ACLK), .rst_n(S_AXI_ARESETN),
        .ja_cs(ja_cs), .ja_mosi(ja_mosi), .ja_sclk(ja_sclk), .ja_miso(ja_miso),
        .ja_cs_m(), .ja_cs_alt(),
        .accel_x_out(w_raw_x), .accel_g_int_out(w_g_int), .accel_g_frac_out(w_g_frac), .is_minus_out(w_is_minus)
    );

endmodule