`define ENABLE_CLOCK1
`define ENABLE_CLOCK2
`define ENABLE_SDRAM
`define ENABLE_KEY
`define ENABLE_LED
`define ENABLE_SW
`define ENABLE_VGA
`define ENABLE_GPIO

module DE10_LITE_Golden_Top
  (
    `ifdef ENABLE_ADC_CLOCK
      input ADC_CLK_10,
    `endif

    `ifdef ENABLE_CLOCK1
      input MAX10_CLK1_50,
    `endif

    `ifdef ENABLE_CLOCK2
      input MAX10_CLK_2_50,
    `endif

    `ifdef ENABLE_SDRAM
      output [12:0] DRAM_ADDR,
      output [1:0]  DRAM_BA,
      output        DRAM_CAS_N,
      output        DRAM_CKE,
      output        DRAM_CLK,
      output        DRAM_CS_N,
      inout  [15:0] DRAM_DQ,
      output        DRAM_LDQM,
      output        DRAM_RAS_N,
      output        DRAM_UDQM,
      output        DRAM_WE_N,
    `endif

    `ifdef ENABLE_HEX0
      output [7:0] HEX0,
    `endif

    `ifdef ENABLE_HEX1
      output [7:0] HEX1,
    `endif

    `ifdef ENABLE_HEX2
      output [7:0] HEX2,
    `endif

    `ifdef ENABLE_HEX3
      output [7:0] HEX3,
    `endif

    `ifdef ENABLE_HEX0
      output [7:0] HEX4,
    `endif

    `ifdef ENABLE_HEX5
      output [7:0] HEX5,
    `endif

    `ifdef ENABLE_KEY
      input [1:0] KEY,
    `endif

    `ifdef ENABLE_LED
      output [9:0] LEDR,
    `endif

    `ifdef ENABLE_SW
      input [9:0] SW,
    `endif

    `ifdef ENABLE_VGA
      output [3:0] VGA_B,
      output [3:0] VGA_G,
      output       VGA_HS,
      output [3:0] VGA_R,
      output       VGA_VS,
    `endif

    `ifdef ENABLE_ACCELEROMETER
      output       GSENSOR_CS_N,
      input  [2:1] GSENSOR_INT,
      output       GSENSOR_SCLK,
      inout        GSENSOR_SDI,
      inout        GSENSOR_SDO,
    `endif

    `ifdef ENABLE_ARDUINO
      inout [15:0] ARDUINO_IO,
      inout        ARDUINO_RESET_N,
    `endif

    `ifdef ENABLE_GPIO
      inout [35:0] GPIO,
    `endif

	  output        camReset,
	  output        PWRDownCam,
	  output        camXCLK,	
	  input         camPCLK,
	  input         camVSYNC,
	  input         HREF,
	  input  [7:0]  pixData,
	  output [15:0] pixOutput,
	  output        buffClear1,
	  output        buffClear2,
	  output        writeBuff1,
	  output        writeBuff2
  );

  wire CLK24MHz, CLK25MHz, CLK100MHz;
	wire writeBuffSelect;

	wire [15:0] dataFIFO1;
	wire [15:0] dataFIFO2;

	wire DRAMWriteAck, DRAMWriteReq, DRAMReadAck, DRAMReadReq;
	wire [12:0] rowAddress;
	wire [1:0]  bankAddress;
	wire [15:0] dataToDRAM;
	wire        inBuffRd1, inBuffRd2;

  assign camReset   = 1;
  assign PWRDownCam = 0;
  assign camXCLK    = CLK24MHz;

  refCLKPLL refCLKPLLInstant(.areset(!KEY[0]),
                             .inclk0(MAX10_CLK1_50),
                             .c0(CLK24MHz),
                             .c1(CLK25MHz),
                             .c2(CLK100MHz),
                             .locked()
                            );

  dataRegistering dataRegesteringInstant(.camPCLK(camPCLK),
                                         .camVSYNC(camVSYNC),
                                         .HREF(HREF),
                                         .pixData(pixData),
                                         .pixOutput(pixOutput[15:0]),
                                         .buffClear1(buffClear1),
                                         .buffClear2(buffClear2),
                                         .writeBuff1(writeBuff1),
                                         .writeBuff2(writeBuff2),
                                         .buffSelect(writeBuffSelect)
                                        );

  lineBuffer lineBufferInstant1(.aclr(buffClear1),
                                .data(pixOutput[15:0]),
                                .rdclk(CLK100MHz),
                                .rdreq(inBuffRd1),
                                .wrclk(camPCLK),
                                .wrreq(writeBuff1),
                                .q(dataFIFO1),
                                .rdempty(),
                                .rdfull(),
                                .wrempty(),
                                .wrfull()
                               );

  lineBuffer lineBufferInstant2(.aclr(buffClear2),
                                .data(pixOutput[15:0]),
                                .rdclk(CLK100MHz),
                                .rdreq(inBuffRd2),
                                .wrclk(camPCLK),
                                .wrreq(writeBuff2),
                                .q(dataFIFO2),
                                .rdempty(),
                                .rdfull(),
                                .wrempty(),
                                .wrfull()
                               );

  buffCapControl buffCapControlInstant(.CLK100MHz(CLK100MHz),
                                       .resetN(KEY[1]),
                                       .VSYNC(camVSYNC),
                                       .writeBuffSelect(writeBuffSelect),
                                       .dataFIFO1(dataFIFO1),
                                       .dataFIFO2(dataFIFO2),
                                       .DRAMWriteAck(DRAMWriteAck),
                                       .inBuffRd1(inBuffRd1),
                                       .inBuffRd2(inBuffRd2),
                                       .DRAMWriteReq(DRAMWriteReq),
                                       .rowAddress(rowAddress),
                                       .bankAddress(bankAddress),
                                       .dataToDRAM(dataToDRAM)
                                      );

  DRAMControl DRAMControlInstant(.CLK100MHz(CLK100MHz),
                                 .resetN(KEY[1]),
                                 .DRAMWriteReq(DRAMWriteReq),
                                 .DRAMReadReq(DRAMReadReq),	 
                                 .rowAddress(rowAddress),
                                 .bankAddress(bankAddress),
                                 .dataToDRAM(dataToDRAM),
                                 .DRAMWriteAck(DRAMWriteAck),
                                 .DRAMReadAck(DRAMReadAck)
                                );

  vgaGen vgaGenInstant(.pixClock(CLK25MHz),
                       .resetN(KEY[1]), 
                       .VSync(VGA_VS), 
                       .HSync(VGA_HS), 
                       .red(VGA_R), 
                       .green(VGA_G), 
                       .blue(VGA_B) 
                      );
endmodule

