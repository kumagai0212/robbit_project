/* CFU Proving Ground since 2025-02    Copyright(c) 2025 Archlab. Science Tokyo /
/ Released under the MIT license https://opensource.org/licenses/mit           */

`resetall `default_nettype none


`include "config.vh"

module main (
    input  wire clk_i       ,
    output wire scl         , // I2C Gyroscope sensor
`ifdef SYNTHESIS             
    inout  wire sda         , // I2C Gyroscope sensor
`else
    input  wire sda         , // I2C Gyroscope sensor
`endif             
    output wire motor_stby  , // motor driver
    output wire motor_ain1  , // motor driver
    output wire motor_ain2  , // motor driver
    output wire motor_pwma  , // motor driver
    output wire st7789_SDA  ,
    output wire st7789_SCL  ,
    output wire st7789_DC   , 
    output wire st7789_RES  ,
    input  wire [1:0] button     , //button
    output reg  [1:0] led
);
    //    wire rst_ni = 1;
    reg rst_ni = 0; initial #15 rst_ni = 1;
    wire clk, locked;

`ifdef SYNTHESIS
    clk_wiz_0 clk_wiz_0 (
        .clk_out1(clk),      // output clk_out1
        .reset   (!rst_ni),  // input reset
        .locked  (locked),   // output locked
        .clk_in1 (clk_i)     // input clk_in1
    );
`else
    assign clk    = clk_i;
    assign locked = 1'b1;
`endif

///// Parameter Tuning
    wire [31:0] w_vio0_target, w_vio1_Pgain, w_vio2_Igain;
    wire [31:0] w_vio3_Dgain, w_vio4_Vmin, w_vio5_Vmax;
    reg  [31:0] r_VIO_trgt = 40;
    reg  [31:0] r_VIO_pgin = 1500;
    reg  [31:0] r_VIO_igin = 16000; 
    reg  [31:0] r_VIO_dgin = 50;
    reg  [31:0] r_VIO_vmin = 45;
    reg  [31:0] r_VIO_vmax = 255;

/*
`ifdef SYNTHESIS
    vio_0 vio_0 (
        .clk                (clk                ),
        .probe_out0         (w_vio0_target      ),
        .probe_out1         (w_vio1_Pgain       ),
        .probe_out2         (w_vio2_Igain       ),
        .probe_out3         (w_vio3_Dgain       ), 
        .probe_out4         (w_vio4_Vmin        ), 
        .probe_out5         (w_vio5_Vmax        )
    );
    always @(posedge clk) begin
        r_VIO_trgt <= w_vio0_target;
        r_VIO_pgin <= w_vio1_Pgain;
        r_VIO_igin <= w_vio2_Igain;
        r_VIO_dgin <= w_vio3_Dgain;
        r_VIO_vmin <= w_vio4_Vmin;
        r_VIO_vmax <= w_vio5_Vmax;
    end
`endif
*/
    

`ifdef SYNTHESIS    
    wire [31:0] w_MPU_ayax, w_MPU_gxaz, w_MPU_gzgy;
    mpu6050 mpu6050(clk, rst, w_MPU_ayax, w_MPU_gxaz, w_MPU_gzgy, scl, sda);
`else
    wire [31:0] w_MPU_ayax = {16'd100, 16'd100};
    wire [31:0] w_MPU_gxaz = {16'd100, 16'd100};
    wire [31:0] w_MPU_gzgy = {16'd100, 16'd100};
`endif

    reg [31:0] r_MPU_ayax = 0;
    reg [31:0] r_MPU_gxaz = 0;
    reg [31:0] r_MPU_gzgy = 0;
    always@(posedge clk) begin
        r_MPU_ayax <= w_MPU_ayax;
        r_MPU_gxaz <= w_MPU_gxaz;
        r_MPU_gzgy <= w_MPU_gzgy;
    end
    
    wire                        rst             = !rst_ni || !locked;
    wire [`IBUS_ADDR_WIDTH-1:0] imem_raddr  ;
    wire [`IBUS_DATA_WIDTH-1:0] imem_rdata  ;
    wire                        dbus_we     ;
    wire [`DBUS_ADDR_WIDTH-1:0] dbus_addr   ;
    wire [`DBUS_DATA_WIDTH-1:0] dbus_wdata  ;
    wire [`DBUS_STRB_WIDTH-1:0] dbus_wstrb  ;
    wire [`DBUS_DATA_WIDTH-1:0] dbus_rdata  ;

    //----------------balancing-car-------------------------------
    reg [31:0] r_dmem_addr = 0;
    always@(posedge clk) r_dmem_addr <= dmem_addr;

    reg [31:0] r_timer_cnt = 1;
    reg [31:0] r_timer = 0;
    always@(posedge clk) begin
        r_timer_cnt <= (r_timer_cnt==(`CLK_FREQ_MHZ*10)) ? 1 : r_timer_cnt + 1;
        if(r_timer_cnt==1) r_timer <= r_timer + 1;
    end

////ditect Horizontal state
    wire [31:0] w_mem_roll_1 = dbus_we && (dbus_addr == 32'h3000_0048);
    always @(posedge clk) begin
        if(w_mem_roll_1 == 1) led[1] <= dbus_wdata[0];
        if(w_mem_roll_1 == 1) led[0] <= dbus_wdata[1];

    end

////check button pushing
    reg [31:0] r_button = 0;
    always @(posedge clk) begin
        r_button <= {30'd0, button[1:0]};
    end

    wire [31:0] w_mmio_data = (r_dmem_addr[7:0]==8'h00) ? r_MPU_ayax :
                              (r_dmem_addr[7:0]==8'h04) ? r_MPU_gxaz :
                              (r_dmem_addr[7:0]==8'h08) ? r_MPU_gzgy :
                              (r_dmem_addr[7:0]==8'h10) ? r_timer    :
                              (r_dmem_addr[7:0]==8'h20) ? r_VIO_trgt :
                              (r_dmem_addr[7:0]==8'h24) ? r_VIO_pgin :
                              (r_dmem_addr[7:0]==8'h28) ? r_VIO_igin :
                              (r_dmem_addr[7:0]==8'h2c) ? r_VIO_dgin :
                              (r_dmem_addr[7:0]==8'h30) ? r_VIO_vmin :
                              (r_dmem_addr[7:0]==8'h34) ? r_VIO_vmax :
                              (r_dmem_addr[7:0]==8'h44) ? r_button   : 0;

    //main.v
    //reg rdata_sel = 0; always @(posedge clk) rdata_sel <= dbus_addr[30];
    //assign dbus_rdata = (rdata_sel) ? perf_rdata : dmem_rdata;

    //main1.v
    assign dbus_rdata = (r_dmem_addr[31:28]==4'h4) ? perf_rdata : (r_dmem_addr[31:28]==4'h3) ? w_mmio_data: dmem_rdata;

    wire pwm_we = (dbus_addr==32'h3000_0040 & dbus_we);
    wire [31:0] pwm_ctrl = dbus_wdata;
    tb6612fng tb6612fng (
        .clk_i              (clk            ), // 100MHz
        .we_i               (pwm_we         ),
        .ctrl_i             (pwm_ctrl       ), // 32bit, [17]: in1, [16]: in2, [7:0]: duty
        .stby_o             (motor_stby     ),
        .in1_o              (motor_ain1     ),
        .in2_o              (motor_ain2     ),
        .pwm_o              (motor_pwma     )  // 100kHz
    );

    cpu cpu (
        .clk_i        (clk),         // input  wire
        .rst_i        (rst),         // input  wire
        .ibus_araddr_o(imem_raddr),  // output wire [`IBUS_ADDR_WIDTH-1:0]
        .ibus_rdata_i (imem_rdata),  // input  wire [`IBUS_DATA_WIDTH-1:0]
        .dbus_addr_o  (dbus_addr),   // output wire [`DBUS_ADDR_WIDTH-1:0]
        .dbus_wvalid_o(dbus_we),     // output wire
        .dbus_wdata_o (dbus_wdata),  // output wire [`DBUS_DATA_WIDTH-1:0]
        .dbus_wstrb_o (dbus_wstrb),  // output wire [`DBUS_STRB_WIDTH-1:0]
        .dbus_rdata_i (dbus_rdata)   // input  wire [`DBUS_DATA_WIDTH-1:0]
    );

    m_imem imem (
        .clk_i  (clk),         // input  wire
        .raddr_i(imem_raddr),  // input  wire [ADDR_WIDTH-1:0]
        .rdata_o(imem_rdata)   // output reg  [DATA_WIDTH-1:0]
    );

    wire dmem_we = dbus_we & (dbus_addr[28]) & !(dbus_addr[29]);
    wire [31:0] dmem_addr = dbus_addr;
    wire [31:0] dmem_wdata = dbus_wdata;
    wire [3:0] dmem_wstrb = dbus_wstrb;
    wire [31:0] dmem_rdata;
    m_dmem dmem (
        .clk_i  (clk),         // input  wire
        .we_i   (dmem_we),     // input  wire                  
        .addr_i (dmem_addr),   // input  wire [ADDR_WIDTH-1:0] 
        .wdata_i(dmem_wdata),  // input  wire [DATA_WIDTH-1:0] 
        .wstrb_i(dmem_wstrb),  // input  wire [STRB_WIDTH-1:0] 
        .rdata_o(dmem_rdata)   // output reg  [DATA_WIDTH-1:0] 
    );

    wire vmem_we = dbus_we & (dbus_addr[29]);
    wire [15:0] vmem_addr = dbus_addr[15:0];
    wire [2:0] vmem_wdata = dbus_wdata[2:0];
    wire [15:0] vmem_raddr;
    wire [2:0] vmem_rdata_t;
    vmem vmem (
        .clk_i  (clk),          // input wire
        .we_i   (vmem_we),      // input wire
        .waddr_i(vmem_addr),    // input wire [15:0]
        .wdata_i(vmem_wdata),   // input wire [15:0]
        .raddr_i(vmem_raddr),   // input wire [15:0]
        .rdata_o(vmem_rdata_t)  // output wire [15:0]
    );

    wire perf_we = dbus_we & (dbus_addr[30]);
    wire [3:0] perf_addr = dbus_addr[3:0];
    wire [2:0] perf_wdata = dbus_wdata[2:0];
    wire [31:0] perf_rdata;
    perf_cntr perf (
        .clk_i  (clk),         // input  wire
        .addr_i (perf_addr),   // input  wire [3:0]
        .wdata_i(perf_wdata),  // input  wire [2:0]
        .w_en_i (perf_we),     // input  wire
        .rdata_o(perf_rdata)   // output wire [31:0]
    );

    wire [15:0] vmem_rdata = {{5{vmem_rdata_t[2]}}, {6{vmem_rdata_t[1]}}, {5{vmem_rdata_t[0]}}};
    m_st7789_disp st7789_disp (
        .w_clk     (clk),         // input  wire
        .st7789_SDA(st7789_SDA),  // output wire
        .st7789_SCL(st7789_SCL),  // output wire
        .st7789_DC (st7789_DC),   // output wire
        .st7789_RES(st7789_RES),  // output wire
        .w_raddr   (vmem_raddr),  // output wire [15:0]
        .w_rdata   (vmem_rdata)   // input  wire [15:0]
    );

endmodule

module m_imem (
    input  wire        clk_i,
    input  wire [31:0] raddr_i,
    output wire [31:0] rdata_o
);

    (* ram_style = "block" *) reg [31:0] imem[0:`IMEM_ENTRIES-1];
    `include "memi.txt"

    wire [`IMEM_ADDRW-1:0] valid_raddr = raddr_i[`IMEM_ADDRW+1:2];

    reg [31:0] rdata;
    always @(posedge clk_i) begin
        rdata <= imem[valid_raddr];
    end
    assign rdata_o = rdata;
endmodule

module m_dmem (
    input  wire        clk_i,
    input  wire        we_i,
    input  wire [31:0] addr_i,
    input  wire [31:0] wdata_i,
    input  wire [ 3:0] wstrb_i,
    output wire [31:0] rdata_o
);

    (* ram_style = "block" *) reg [31:0] dmem[0:`DMEM_ENTRIES-1];
    `include "memd.txt"

    wire [`DMEM_ADDRW-1:0] valid_addr = addr_i[`DMEM_ADDRW+1:2];

    reg [31:0] rdata = 0;
    always @(posedge clk_i) begin
        if (we_i) begin  ///// data bus
            if (wstrb_i[0]) dmem[valid_addr][7:0] <= wdata_i[7:0];
            if (wstrb_i[1]) dmem[valid_addr][15:8] <= wdata_i[15:8];
            if (wstrb_i[2]) dmem[valid_addr][23:16] <= wdata_i[23:16];
            if (wstrb_i[3]) dmem[valid_addr][31:24] <= wdata_i[31:24];
        end
        rdata <= dmem[valid_addr];
    end
    assign rdata_o = rdata;
endmodule

module perf_cntr (
    input  wire        clk_i,
    input  wire [ 3:0] addr_i,
    input  wire [ 2:0] wdata_i,
    input  wire        w_en_i,
    output wire [31:0] rdata_o
);
    reg [63:0] mcycle = 0;
    reg [ 1:0] cnt_ctrl = 0;
    reg [31:0] rdata = 0;

    always @(posedge clk_i) begin
        rdata <= (addr_i[2]) ? mcycle[31:0] : mcycle[63:32];
        if (w_en_i && addr_i == 0) cnt_ctrl <= wdata_i[1:0];
        case (cnt_ctrl)
            0: mcycle <= 0;
            1: mcycle <= mcycle + 1;
            default: ;
        endcase
    end

    assign rdata_o = rdata;
endmodule

module vmem (
    input  wire        clk_i,
    input  wire        we_i,
    input  wire [15:0] waddr_i,
    input  wire [ 2:0] wdata_i,
    input  wire [15:0] raddr_i,
    output wire [ 2:0] rdata_o
);

    reg [2:0] vmem_lo[0:32767];  // vmem
    reg [2:0] vmem_hi[0:32767];  // vmem
    integer i;
    initial
        for (i = 0; i < 32768; i = i + 1) begin
            vmem_lo[i] = 0;
            vmem_hi[i] = 0;
        end

    reg        we;
    reg        top;
    reg [ 2:0] wdata;
    reg [14:0] waddr;

    reg        rtop;
    reg [14:0] raddr;
    reg [ 2:0] rdata_lo;
    reg [ 2:0] rdata_hi;
    reg        sel;

    localparam ADDR_MASK = 16'h7FFF;

    always @(posedge clk_i) begin
        we <= we_i;
        top <= waddr_i[15];
        waddr <= waddr_i[14:0];
        wdata <= wdata_i;

        rtop <= raddr_i[15];
        raddr <= raddr_i[14:0];

        if (we) begin
            if (top) vmem_hi[waddr&ADDR_MASK] <= wdata;
            else vmem_lo[waddr&ADDR_MASK] <= wdata;
        end

        sel <= rtop;
        rdata_lo <= vmem_lo[raddr&ADDR_MASK];
        rdata_hi <= vmem_hi[raddr&ADDR_MASK];
    end

    assign rdata_o = (sel) ? rdata_hi : rdata_lo;


`ifndef SYNTHESIS
    reg  [15:0] r_adr_p = 0;
    reg  [15:0] r_dat_p = 0;

    wire [15:0] data = {{5{wdata_i[2]}}, {6{wdata_i[1]}}, {5{wdata_i[0]}}};
    always @(posedge clk_i)
        if (we_i) begin
            case (waddr_i[15])
                0:
                if (vmem_lo[waddr_i&ADDR_MASK] != wdata_i) begin
                    r_adr_p <= waddr_i;
                    r_dat_p <= data;
                    $write("@D%0d_%0d\n", waddr_i ^ r_adr_p, data ^ r_dat_p);
                    $fflush();
                end
                1:
                if (vmem_hi[waddr_i&ADDR_MASK] != wdata_i) begin
                    r_adr_p <= waddr_i;
                    r_dat_p <= data;
                    $write("@D%0d_%0d\n", waddr_i ^ r_adr_p, data ^ r_dat_p);
                    $fflush();
                end
            endcase
        end
`endif

endmodule

module m_st7789_disp (
    input wire w_clk,  // main clock signal (100MHz)
    output wire st7789_SDA,
    output wire st7789_SCL,
    output wire st7789_DC,
    output wire st7789_RES,
    output wire [15:0] w_raddr,
    input wire [15:0] w_rdata
);
    reg [31:0] r_cnt = 1;
    always @(posedge w_clk) r_cnt <= (r_cnt == 0) ? 0 : r_cnt + 1;
    reg r_RES = 1;
    always @(posedge w_clk) begin
        r_RES <= (r_cnt == 100000) ? 0 : (r_cnt == 200000) ? 1 : r_RES;
    end
    assign st7789_RES = r_RES;

    wire busy;
    reg r_en = 0;
    reg init_done = 0;
    reg [4:0] r_state = 0;
    reg [19:0] r_state2 = 0;
    reg [8:0] r_dat = 0;
    reg [15:0] r_c = 16'hf800;

    reg [31:0] r_bcnt = 0;
    always @(posedge w_clk) r_bcnt <= (busy) ? 0 : r_bcnt + 1;

    always @(posedge w_clk)
        if (!init_done) begin
            r_en <= (r_cnt > 1000000 && !busy && r_bcnt > 1000000);
        end else begin
            r_en <= (!busy);
        end

    always @(posedge w_clk) if (r_en && !init_done) r_state <= r_state + 1;

    always @(posedge w_clk)
        if (r_en && init_done) begin
            r_state2 <= (r_state2==115210) ? 0 : r_state2 + 1; // 11 + 240x240*2 = 11 + 115200 = 115211
        end

    reg [7:0] r_x = 0;
    reg [7:0] r_y = 0;
    always @(posedge w_clk)
        if (r_en && init_done && r_state2[0] == 1) begin
            r_x <= (r_state2 < 11 || r_x == 239) ? 0 : r_x + 1;
            r_y <= (r_state2 < 11) ? 0 : (r_x == 239) ? r_y + 1 : r_y;
        end

    wire [7:0] w_nx = 239 - r_x;
    wire [7:0] w_ny = 239 - r_y;
    assign w_raddr = (`LCD_ROTATE == 0) ? {r_y, r_x} :  // default
        (`LCD_ROTATE == 1) ? {r_x, w_ny} :  // 90 degree rotation
        (`LCD_ROTATE == 2) ? {w_ny, w_nx} : {w_nx, r_y};  //180 degree, 240 degree rotation

    reg [15:0] r_color = 0;
    always @(posedge w_clk) r_color <= w_rdata;

    always @(posedge w_clk) begin
        case (r_state2)  /////
            0: r_dat <= {1'b0, 8'h2A};  // Column Address Set
            1: r_dat <= {1'b1, 8'h00};  // [0]
            2: r_dat <= {1'b1, 8'h00};  // [0]
            3: r_dat <= {1'b1, 8'h00};  // [0]
            4: r_dat <= {1'b1, 8'd239};  // [239]
            5: r_dat <= {1'b0, 8'h2B};  // Row Address Set
            6: r_dat <= {1'b1, 8'h00};  // [0]
            7: r_dat <= {1'b1, 8'h00};  // [0]
            8: r_dat <= {1'b1, 8'h00};  // [0]
            9: r_dat <= {1'b1, 8'd239};  // [239]
            10: r_dat <= {1'b0, 8'h2C};  // Memory Write
            default: r_dat <= (r_state2[0]) ? {1'b1, r_color[15:8]} : {1'b1, r_color[7:0]};
        endcase
    end

    reg [8:0] r_init = 0;
    always @(posedge w_clk) begin
        case (r_state)  /////
            0: r_init <= {1'b0, 8'h01};  // Software Reset, wait 120msec
            1: r_init <= {1'b0, 8'h11};  // Sleep Out, wait 120msec
            2: r_init <= {1'b0, 8'h3A};  // Interface Pixel Format
            3: r_init <= {1'b1, 8'h55};  // [65K RGB, 16bit/pixel]
            4: r_init <= {1'b0, 8'h36};  // Memory Data Accell Control
            5: r_init <= {1'b1, 8'h00};  // [000000]
            6: r_init <= {1'b0, 8'h21};  // Display Inversion On
            7: r_init <= {1'b0, 8'h13};  // Normal Display Mode On
            8: r_init <= {1'b0, 8'h29};  // Display On
            9: init_done <= 1;
        endcase
    end

    wire [8:0] w_data = (init_done) ? r_dat : r_init;
    m_spi spi0 (
        w_clk,
        r_en,
        w_data,
        st7789_SDA,
        st7789_SCL,
        st7789_DC,
        busy
    );
endmodule

/****** SPI send module,  SPI_MODE_2, MSBFIRST                                           *****/
/*********************************************************************************************/
module m_spi (
    input  wire       w_clk,  // 100MHz input clock !!
    input  wire       en,     // write enable
    input  wire [8:0] d_in,   // data in
    output wire       SDA,    // Serial Data
    output wire       SCL,    // Serial Clock
    output wire       DC,     // Data/Control
    output wire       busy    // busy
);
    reg [5:0] r_state = 0;
    reg [7:0] r_cnt = 0;
    reg r_SCL = 1;
    reg r_DC = 0;
    reg [7:0] r_data = 0;
    reg r_SDA = 0;

    always @(posedge w_clk) begin
        if (en && r_state == 0) begin
            r_state <= 1;
            r_data  <= d_in[7:0];
            r_DC    <= d_in[8];
            r_cnt   <= 0;
        end else if (r_state == 1) begin
            r_SDA   <= r_data[7];
            r_data  <= {r_data[6:0], 1'b0};
            r_state <= 2;
            r_cnt   <= r_cnt + 1;
        end else if (r_state == 2) begin
            r_SCL   <= 0;
            r_state <= 3;
        end else if (r_state == 3) begin
            r_state <= 4;
        end else if (r_state == 4) begin
            r_SCL   <= 1;
            r_state <= (r_cnt == 8) ? 0 : 1;
        end
    end

    assign SDA  = r_SDA;
    assign SCL  = r_SCL;
    assign DC   = r_DC;
    assign busy = (r_state != 0 || en);
endmodule
/*********************************************************************************************/

/******************************************************************************************/
// (0,0) stop, (0,1) normal rotation, (1,0) reverse rotation, (1,1) brake
/******************************************************************************************/
module tb6612fng (
    input  wire         clk_i,    // 100MHz input
    input  wire         we_i,     // dbus_addr==32'h3000_0040 & dbus_we
    input  wire [31:0]  ctrl_i,   // in1, in2, duty[7:0]
    output wire         stby_o,   // after cnt == 0x0100_0000 , high
    output wire         in1_o,    //
    output wire         in2_o,    //
    output wire         pwm_o     // 100kHz PWM
);

    reg [31:0] r_motor_ctrl = 0;
    always@(posedge clk_i) begin
        if (we_i) r_motor_ctrl <= ctrl_i;
    end
    assign in1_o  = r_motor_ctrl[17];
    assign in2_o  = r_motor_ctrl[16];

    reg [31:0] r_cnt = 0;
    always@(posedge clk_i) r_cnt <= r_cnt + 1;

    reg r_motor_stby = 0;
    always@(posedge clk_i) if(r_cnt[27]) r_motor_stby <= 1;
    assign stby_o = r_motor_stby;

    // 100MHz -> 100kHz,  2048 cycle
    //wire [9:0] dutyx4 = {r_motor_ctrl[7:0], 2'b11};
    wire [10:0] dutyx8 = {r_motor_ctrl[7:0], 3'b111};
    assign pwm_o  = (r_cnt[10:0] <= dutyx8);
    
endmodule

`resetall
