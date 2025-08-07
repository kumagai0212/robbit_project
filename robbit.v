
module robbit_control (
    input  wire                        clk_i,
    input  wire                        rst_i,
    input  wire [1:0]                  button_i,
    input  wire  [31:0]                 r_dmem_addr_i,
    input  wire                        dbus_we_i,
    input  wire [`DBUS_ADDR_WIDTH-1:0] dbus_addr_i,
    input  wire [`DBUS_DATA_WIDTH-1:0] dbus_wdata_i,
    output wire [1:0]                  led_o,
    output wire [31:0]                 w_mmio_data_o,
    output wire                        scl_io,
    `ifdef SYNTHESIS             
        inout  wire sda_io          // I2C Gyroscope sensor
    `else
        input  wire sda_io          // I2C Gyroscope sensor
    `endif   
);


    ///// Parameter Tuning
    wire [31:0] w_vio0_target, w_vio1_Pgain, w_vio2_Igain;
    wire [31:0] w_vio3_Dgain, w_vio4_Vmin, w_vio5_Vmax;
    reg  [31:0] r_VIO_trgt = 40;
    reg  [31:0] r_VIO_pgin = 1500;
    reg  [31:0] r_VIO_igin = 16000; 
    reg  [31:0] r_VIO_dgin = 50;
    reg  [31:0] r_VIO_vmin = 45;
    reg  [31:0] r_VIO_vmax = 255;

    `ifdef SYNTHESIS    
    wire [31:0] w_MPU_ayax, w_MPU_gxaz, w_MPU_gzgy;
    mpu6050 mpu6050(clk_i, rst_i, w_MPU_ayax, w_MPU_gxaz, w_MPU_gzgy, scl_io, sda_io);
`else
    wire [31:0] w_MPU_ayax = {16'd100, 16'd100};
    wire [31:0] w_MPU_gxaz = {16'd100, 16'd100};
    wire [31:0] w_MPU_gzgy = {16'd100, 16'd100};
`endif

    reg [31:0] r_MPU_ayax = 0;
    reg [31:0] r_MPU_gxaz = 0;
    reg [31:0] r_MPU_gzgy = 0;
    always@(posedge clk_i) begin
        r_MPU_ayax <= w_MPU_ayax;
        r_MPU_gxaz <= w_MPU_gxaz;
        r_MPU_gzgy <= w_MPU_gzgy;
    end


    reg [31:0] r_timer_cnt = 1;
    reg [31:0] r_timer = 0;
    always@(posedge clk_i) begin
        r_timer_cnt <= (r_timer_cnt==(`CLK_FREQ_MHZ*10)) ? 1 : r_timer_cnt + 1;
        if(r_timer_cnt==1) r_timer <= r_timer + 1;
    end

    ////ditect Horizontal state
    wire [31:0] w_mem_roll_1 = dbus_we_i && (dbus_addr_i == 32'h3000_0048);

    reg [1:0] led_reg;
    assign led_o = led_reg;
    always @(posedge clk_i) begin
        if(w_mem_roll_1 == 1) begin
            led_reg[1] <= dbus_wdata_i[0];
            led_reg[0] <= dbus_wdata_i[1];
        end
    end

////check button pushing
    reg [31:0] r_button = 0;
    always @(posedge clk_i) begin
        r_button <= {30'd0, button_i[1:0]};
    end

    assign w_mmio_data_o =    (r_dmem_addr_i[7:0]==8'h00) ? r_MPU_ayax :
                              (r_dmem_addr_i[7:0]==8'h04) ? r_MPU_gxaz :
                              (r_dmem_addr_i[7:0]==8'h08) ? r_MPU_gzgy :
                              (r_dmem_addr_i[7:0]==8'h10) ? r_timer    :
                              (r_dmem_addr_i[7:0]==8'h20) ? r_VIO_trgt :
                              (r_dmem_addr_i[7:0]==8'h24) ? r_VIO_pgin :
                              (r_dmem_addr_i[7:0]==8'h28) ? r_VIO_igin :
                              (r_dmem_addr_i[7:0]==8'h2c) ? r_VIO_dgin :
                              (r_dmem_addr_i[7:0]==8'h30) ? r_VIO_vmin :
                              (r_dmem_addr_i[7:0]==8'h34) ? r_VIO_vmax :
                              (r_dmem_addr_i[7:0]==8'h44) ? r_button   : 0;

    //main.v
    //reg rdata_sel = 0; always @(posedge clk) rdata_sel <= dbus_addr[30];
    //assign dbus_rdata = (rdata_sel) ? perf_rdata : dmem_rdata;

endmodule

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