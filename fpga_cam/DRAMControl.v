// Module Name: data
// Description: module for handling databus
// Author: Jeremie Hews
// Date: 12/27
module  DRAMControl(

	 
	 input CLK100MHz,
	 input resetN,
	 input DRAMWriteReq,
	 input [12:0] rowAddress,
	 input [1:0] bankAddress,
	 input [15:0] dataToDRAM,
	 input DRAMReadReq,
	 
	 output reg DRAMWriteAck,
	 output reg DRAMReadAck,
	 
	 //pins directly to dram
	 output	reg	    [12:0]		DRAM_ADDR,
	 output	reg	     [1:0]		DRAM_BA,
	 output	reg	          		DRAM_CAS_N,
	 output	reg	          		DRAM_CKE,
	 output	wire	          		DRAM_CLK,
	 output	reg	          		DRAM_CS_N,
	 inout 	wire	    [15:0]		DRAM_DQ,
	 output	reg	          		DRAM_LDQM,
	 output	reg	          		DRAM_RAS_N,
	 output	reg	          		DRAM_UDQM,
	 output	reg	          		DRAM_WE_N
);
	
	
assign DRAM_CLK = CLK100MHz;

reg [15:0] DRAM_DQ_0;

	
	
reg [4:0] DRAMState, prevState; // Updated to 5 bits to accommodate additional states

// State assignments
localparam [4:0]
    INIT0   = 5'b00000,
    INIT1   = 5'b00001,
    INIT2   = 5'b00010,
    INIT3   = 5'b00011,
    INIT4   = 5'b00100,
    INIT5   = 5'b00101,
    INIT6   = 5'b00110,
    INIT7   = 5'b00111,
    IDLE    = 5'b01000,
    WRITE0  = 5'b01001,
    WRITE1  = 5'b01010,
    WRITE2  = 5'b01011,
    WRITE3  = 5'b01100,
    READ0   = 5'b01101,
    READ1   = 5'b01110,
    READ2   = 5'b01111,
    READ3   = 5'b10000,
    REFRESH0 = 5'b10001, // Added refresh states
    REFRESH1 = 5'b10010,
    REFRESH2 = 5'b10011;
	 
		 
	wire refreshReq;
	assign refreshReq = 0;
	
	
	//Write-port address generator
	always@(posedge CLK100MHz or negedge resetN)
		begin
		if(!resetN)
			begin
			DRAMWriteAck <= 0;
			DRAMState <= INIT0;
			prevState <= INIT0;
			DRAM_ADDR <= 0;
			DRAM_BA <= 0;
			DRAM_CAS_N <= 1;
			DRAM_CKE <= 1;
			DRAM_CS_N <= 1;
			DRAM_DQ_0 <= 0;
			DRAM_LDQM <= 0;
			DRAM_UDQM <= 0;
			DRAM_RAS_N <= 1;
			DRAM_WE_N <= 1;

			end
			
		else
			begin
			
			case (DRAMState)
			
            INIT0 : begin
                    DRAMState <= INIT1;
                    end
                    
            INIT1 : begin
                    DRAMState <= INIT2;
                    end
                    
            INIT2 : begin
                    DRAMState <= INIT3;
                    end
                    
            INIT3 : begin
                    DRAMState <= INIT4;
                    end
                    
            INIT4 : begin
                    DRAMState <= INIT5;
                    end
                    
            INIT5 : begin
                    DRAMState <= INIT6;
                    end
                    
            INIT6 : begin
                    DRAMState <= INIT7;
                    end
                    
            INIT7 : begin
                    DRAMState <= IDLE;
                    end
                    
            IDLE  : begin
						  if(refreshReq)
								begin
								DRAMState <= REFRESH0;
								end
                  
				 	   	else if (DRAMReadReq)
							   begin
							   DRAMState <= READ0;
								DRAMReadAck <= 1;
								
 							   end
                  
				   		else if(DRAMWriteReq)
							   begin 
							   DRAMState <= WRITE0;
								DRAMWriteAck <= 1;
							   end
							  
                    end
                    
            WRITE0 : begin
                    DRAMState <= WRITE1;
                    end
                    
            WRITE1 : begin
                    DRAMState <= WRITE2;
                    end
                    
            WRITE2 : begin
                    DRAMState <= WRITE3;
                    end
                    
            WRITE3 : begin
						  if(!DRAMWriteReq)
							  begin
							  DRAMWriteAck <= 0;
							  DRAMState <= IDLE; 
							  end
						  end
                    
            READ0 : begin
                    DRAMState <= READ1;
                    end
                    
            READ1 : begin
                    DRAMState <= READ2;
                    end
                    
            READ2 : begin
                    DRAMState <= READ3;
                    end
                    
            READ3 : begin
						  if(!DRAMReadReq)
							  begin
							  DRAMReadAck <= 0;
							  DRAMState <= IDLE; 
							  end
                    end		
            
				REFRESH0 : begin
                    DRAMState <= REFRESH1;
                    end
                    
            REFRESH1 : begin
                    DRAMState <= REFRESH2;
                    end
                    
            REFRESH2 : begin
                    DRAMState <= IDLE;
                    end
                    
			 default : begin
                    DRAMWriteAck <= 0;
                    DRAMState <= INIT0;
						  end
			endcase
			end
		end
	
	
endmodule