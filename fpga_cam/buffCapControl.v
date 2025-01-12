// Module Name: data
// Description: module for handling databus
// Author: Jeremie Hews
// Date: 12/27
module  buffCapControl(

	input CLK100MHz,
	input resetN,
	
	//below is from linebuffer
	input VSYNC,
	input writeBuffSelect,
	input [15:0] dataFIFO1,
	input [15:0] dataFIFO2,
	
	//from DRAM
	input DRAMWriteAck,
	
	//to line buffer
	output reg inBuffRd1,
	output reg inBuffRd2,
	
	
	//to dram
	output reg DRAMWriteReq,
	output reg [12:0] rowAddress,
	output reg [1:0] bankAddress,	
	output reg [15:0] dataToDRAM
	
	);
	
	reg [4:0] VSYNCDelay, writeBuffSelDel;
	reg	VSYNCNegEdge;
	reg	writeBuffEdge;
	reg lineBuffSel;
	
	
	
	always@(posedge CLK100MHz or negedge resetN)
		begin
		if (!resetN)
			begin
			VSYNCNegEdge <= 0;
			writeBuffEdge <= 0;
			VSYNCDelay[4:0]		<= 0;
			writeBuffSelDel[4:0] <= 0;
			lineBuffSel <= 0;
			dataToDRAM			<=0;
			end
			
		else
			begin
			VSYNCDelay[4:0]		<= {VSYNCDelay[3:0], VSYNC};
			writeBuffSelDel[4:0] <= {writeBuffSelDel[3:0], writeBuffSelect};
			dataToDRAM			<=0;
			
			if(!VSYNCDelay[3] && VSYNCDelay[4])
				begin
				VSYNCNegEdge <= 1;
				end
			else
				begin
				VSYNCNegEdge <= 0;
				end
			
			if(writeBuffSelDel[3] != writeBuffSelDel[4])
				begin
				writeBuffEdge <= 1;
				lineBuffSel		<= writeBuffSelDel[4];
				end
			else
				begin
				writeBuffEdge <= 0;
				end

			if (!lineBuffSel)
				begin
				dataToDRAM <= dataFIFO1;
				end
			else
				begin
				dataToDRAM <= dataFIFO2;				
				end
			
			end
		end
	
	reg [3:0] writeBuffState;
	reg [9:0] pixelCount;
//	reg [8:0] lineCount;
	
	
//state assignments
localparam [3:0]
   IDLE            = 4'b0000,
   WAIT_ACK     = 4'b0001,
   WRITE_DRAM     = 4'b0010;


	//Write-port address generator
	always@(posedge CLK100MHz or negedge resetN)
		begin
		if (!resetN)
			begin
			inBuffRd1      <= 0;
			inBuffRd2      <= 0;
			DRAMWriteReq      <= 0;
			rowAddress      <= 0;
			bankAddress      <= 0;
//			dataToDRAM			<=0;
			pixelCount 			<= 0;
//			lineCount 			<= 0;
			writeBuffState	 <= IDLE;
			end
			
		else if (VSYNCNegEdge)
			begin
			inBuffRd1      <= 0;
			inBuffRd2      <= 0;
			DRAMWriteReq      <= 0;
			rowAddress      <= 0;
			bankAddress      <= 0;
			pixelCount 			<= 0;
//			lineCount 			<= 0;
			writeBuffState	 <= IDLE;
			end
			
		else
			begin
			
			case (writeBuffState)
				IDLE        : begin
									
					if (writeBuffEdge)
					begin
					DRAMWriteReq <= 1;
					writeBuffState <= WAIT_ACK;
					end						
								 
				end
				
				WAIT_ACK : begin
					if (DRAMWriteAck)
						begin
						inBuffRd1 <= !lineBuffSel;
						inBuffRd2 <= lineBuffSel;
						writeBuffState <= WRITE_DRAM;
						pixelCount <= 0;	
						end
				end
				
				WRITE_DRAM : begin
					if (pixelCount == 639)
						begin
						writeBuffState <= IDLE;
						rowAddress      <= rowAddress + 1;
						inBuffRd1 <= 0;
						inBuffRd2 <= 0;
						DRAMWriteReq <= 0;
						end
					else
						begin
						pixelCount <= pixelCount + 1;	
						end
   					
				end

				default   : begin
					inBuffRd1      <= 0;
					inBuffRd2      <= 0;
					DRAMWriteReq      <= 0;
					rowAddress      <= 0;
					bankAddress      <= 0;
					dataToDRAM			<=0;
					writeBuffState	 <= IDLE;
				end
			
			endcase
			end
		end

	
	
	
	
endmodule