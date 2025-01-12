//`define ENABLE_ADC_CLOCK
`define ENABLE_CLOCK1
`define ENABLE_CLOCK2
`define ENABLE_SDRAM
//`define ENABLE_HEX0
//`define ENABLE_HEX1
//`define ENABLE_HEX2
//`define ENABLE_HEX3
//`define ENABLE_HEX4
//`define ENABLE_HEX5
`define ENABLE_KEY
`define ENABLE_LED
`define ENABLE_SW
`define ENABLE_VGA
//`define ENABLE_ACCELEROMETER
//`define ENABLE_ARDUINO
//`define ENABLE_GPIO

`timescale 1ns/10ps

module testBench;

	//////////// ADC CLOCK: 3.3-V LVTTL //////////
`ifdef ENABLE_ADC_CLOCK
	reg 		          		ADC_CLK_10;
`endif
	//////////// CLOCK 1: 3.3-V LVTTL //////////
`ifdef ENABLE_CLOCK1
	reg 		          		MAX10_CLK1_50;
`endif
	//////////// CLOCK 2: 3.3-V LVTTL //////////
`ifdef ENABLE_CLOCK2
	wire 		          		MAX10_CLK2_50;
`endif
	//////////// SDRAM: 3.3-V LVTTL //////////
`ifdef ENABLE_SDRAM
	wire		    [12:0]		DRAM_ADDR;
	wire		     [1:0]		DRAM_BA;
	wire		          		DRAM_CAS_N;
	wire		          		DRAM_CKE;
	wire		          		DRAM_CLK;
	wire		          		DRAM_CS_N;
	wire		    [15:0]		DRAM_DQ;
	wire		          		DRAM_LDQM;
	wire		          		DRAM_RAS_N;
	wire		          		DRAM_UDQM;
	wire		          		DRAM_WE_N;
`endif
	//////////// SEG7: 3.3-V LVTTL //////////
`ifdef ENABLE_HEX0
	wire		     [7:0]		HEX0;
`endif
`ifdef ENABLE_HEX1
	wire		     [7:0]		HEX1;
`endif
`ifdef ENABLE_HEX2
	wire		     [7:0]		HEX2;
`endif
`ifdef ENABLE_HEX3
	wire		     [7:0]		HEX3;
`endif
`ifdef ENABLE_HEX4
	wire		     [7:0]		HEX4;
`endif
`ifdef ENABLE_HEX5
	wire		     [7:0]		HEX5;
`endif
	//////////// KEY: 3.3 V SCHMITT TRIGGER //////////
`ifdef ENABLE_KEY
	reg 		     [1:0]		KEY;
`endif
	//////////// LED: 3.3-V LVTTL //////////
`ifdef ENABLE_LED
	wire	     [9:0]		LEDR;
`endif
	//////////// SW: 3.3-V LVTTL //////////
`ifdef ENABLE_SW
	reg 		     [9:0]		SW;
`endif
	//////////// VGA: 3.3-V LVTTL //////////
`ifdef ENABLE_VGA
	wire		     [3:0]		VGA_B;
	wire		     [3:0]		VGA_G;
	wire		          		VGA_HS;
	wire		     [3:0]		VGA_R;
	wire		          		VGA_VS;
`endif
	//////////// Accelerometer: 3.3-V LVTTL //////////
`ifdef ENABLE_ACCELEROMETER
	wire		          		GSENSOR_CS_N;
	reg 		     [2:1]		GSENSOR_INT;
	wire		          		GSENSOR_SCL;
	wire		          		GSENSOR_SDI;
	wire		          		GSENSOR_SDO;
`endif
	//////////// Arduino: 3.3-V LVTTL //////////
`ifdef ENABLE_ARDUINO
	wire		    [15:0]		ARDUINO_IO;
	wire		          		ARDUINO_RESET_N;
`endif
	//////////// GPIO, GPIO connect to GPIO Default: 3.3-V LVTTL //////////
`ifdef ENABLE_GPIO
	wire		    [35:0]		GPIO;
`endif
// creating pins which are on cam
	wire		          		CamReset;
	wire		          		PWRDownCam;
	wire		          		CamXCLK;
	wire 		          		CamPCLK;
	reg 		          		CamVSYNC;
	reg 		          		HREF;
	reg 		     [7:0]     PixData;
	//for theouput of dataregistering
	wire	[15:0]pixOutput;
	wire		          		buffClear1;
	wire		          		buffClear2;
	wire		          		writeBuff1;
	wire		          		writeBuff2;
	//wire dataValid;

	
initial
   begin
	//ADC_CLK_10		= 0;
	MAX10_CLK1_50	= 0;
	//MAX10_CLK2_50	= 0;
	KEY				= 2'b00;
	SW					= 0;
	//GSENSOR_INT		= 0;
	//CamPCLK			= 0;
	CamVSYNC			= 0;
	HREF				= 0;
	PixData			= 0;
	end

	//Generate clock signals to FPGA
	//always #50 ADC_CLK_10    = !ADC_CLK_10;
	always #10 MAX10_CLK1_50 = !MAX10_CLK1_50;
	assign MAX10_CLK2_50     = MAX10_CLK1_50;
	assign CamPCLK = CamXCLK;

   reg [10:0] pix_count;
   reg [9:0]  line_count;

initial
   begin
   pix_count   = 0;
   line_count  = 0;
   end

//Pixel and Line counters
always @(posedge CamPCLK)
	begin
   if (pix_count == 1539)
      begin
   	pix_count <= 0;
      if (line_count == 529)
         begin
   	   line_count <= 0;
         end
      else
         begin
   	   line_count <= line_count + 1;
         end
      end
   else
      begin
   	pix_count <= pix_count + 1;
      end
   end

//VSYNC and HBLANK generation
always @(posedge CamPCLK)
	begin
   //VSYNC
   if ((line_count >= 0) && (line_count <= 2))
      begin
   	CamVSYNC <= 1;
      end
   else
      begin
   	CamVSYNC <= 0;
      end
   //HREF and pixel data
   if ((line_count >= 25) && (line_count <= 504))
      begin
      if ((pix_count >= 130) && (pix_count <= 1409))
         begin
      	HREF <= 1;
       	PixData <= PixData + 1;
         end
      else
         begin
      	HREF <= 0;
       	PixData <= 0;
         end
      end
   else
      begin
 	   HREF <= 0;
    	PixData <= 0;
      end
   end

DE10_LITE_Golden_Top uut_device(
	//////////// ADC CLOCK: 3.3-V LVTTL //////////
`ifdef ENABLE_ADC_CLOCK
		          		ADC_CLK_10,
`endif
	//////////// CLOCK 1: 3.3-V LVTTL //////////
`ifdef ENABLE_CLOCK1
			          		MAX10_CLK1_50,
`endif
	//////////// CLOCK 2: 3.3-V LVTTL //////////
`ifdef ENABLE_CLOCK2
	          		MAX10_CLK2_50,
`endif
	//////////// SDRAM: 3.3-V LVTTL //////////
`ifdef ENABLE_SDRAM
	DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_LDQM,
	DRAM_RAS_N,
	DRAM_UDQM,
	DRAM_WE_N,
`endif
	//////////// SEG7: 3.3-V LVTTL //////////
`ifdef ENABLE_HEX0
	HEX0,
`endif
`ifdef ENABLE_HEX1
	HEX1,
`endif
`ifdef ENABLE_HEX2
	HEX2,
`endif
`ifdef ENABLE_HEX3
	HEX3,
`endif
`ifdef ENABLE_HEX4
	HEX4,
`endif
`ifdef ENABLE_HEX5
	HEX5,
`endif
	//////////// KEY: 3.3 V SCHMITT TRIGGER //////////
`ifdef ENABLE_KEY
	KEY,
`endif
	//////////// LED: 3.3-V LVTTL //////////
`ifdef ENABLE_LED
	LEDR,
`endif
	//////////// SW: 3.3-V LVTTL //////////
`ifdef ENABLE_SW
	SW,
`endif
	//////////// VGA: 3.3-V LVTTL //////////
`ifdef ENABLE_VGA
	VGA_B,
	VGA_G,
	VGA_HS,
	VGA_R,
	VGA_VS,
`endif
	//////////// Accelerometer: 3.3-V LVTTL //////////
`ifdef ENABLE_ACCELEROMETER
	GSENSOR_CS_N,
	GSENSOR_INT,
	GSENSOR_SCLK,
	GSENSOR_SDI,
	GSENSOR_SDO,
`endif
	//////////// Arduino: 3.3-V LVTTL //////////
`ifdef ENABLE_ARDUINO
	ARDUINO_IO,
	ARDUINO_RESET_N,
`endif
	//////////// GPIO, GPIO connect to GPIO Default: 3.3-V LVTTL //////////
`ifdef ENABLE_GPIO
	GPIO
`endif
// creating pins which are on cam
	CamReset,
	PWRDownCam,
	CamXCLK,	
	CamPCLK,
	CamVSYNC,
	HREF,
	PixData,
	//for theouput of dataregistering
	pixOutput,
	buffClear1,
	buffClear2,
	writeBuff1,
	writeBuff2
//	dataValid
);

initial
	begin
	#1000
   KEY[0] = 1;
	#1000
	KEY[1] = 1;
	$display("\nStarting simulation\n");
	#100000000
	$display("\nEnd of simulation\n");
	$finish;
	end

endmodule
