module DRAMControl
  #(
    parameter [4:0] INIT0     = 5'b00000,
    parameter [4:0] INIT1     = 5'b00001,
    parameter [4:0] INIT2     = 5'b00010,
    parameter [4:0] INIT3     = 5'b00011,
    parameter [4:0] INIT4     = 5'b00100,
    parameter [4:0] INIT5     = 5'b00101,
    parameter [4:0] INIT6     = 5'b00110,
    parameter [4:0] INIT7     = 5'b00111,
    parameter [4:0] IDLE      = 5'b01000,
    parameter [4:0] WRITE0    = 5'b01001,
    parameter [4:0] WRITE1    = 5'b01010,
    parameter [4:0] WRITE2    = 5'b01011,
    parameter [4:0] WRITE3    = 5'b01100,
    parameter [4:0] READ0     = 5'b01101,
    parameter [4:0] READ1     = 5'b01110,
    parameter [4:0] READ2     = 5'b01111,
    parameter [4:0] READ3     = 5'b10000,
    parameter [4:0] REFRESH0  = 5'b10001,
    parameter [4:0] REFRESH1  = 5'b10010,
    parameter [4:0] REFRESH2  = 5'b10011
  )(
    input             CLK100MHz,
    input             resetN,
    input             DRAMWriteReq,
    input      [12:0] rowAddress,
    input      [1:0]  bankAddress,
    input      [15:0] dataToDRAM,
    input             DRAMReadReq,
    output reg        DRAMWriteAck,
    output reg        DRAMReadAck,
    output reg [12:0] DRAM_ADDR,
    output reg [1:0]  DRAM_BA,
    output reg        DRAM_CAS_N,
    output reg        DRAM_CKE,
    output            DRAM_CLK,
    output reg        DRAM_CS_N,
    inout      [15:0] DRAM_DQ,
    output reg        DRAM_LDQM,
    output reg        DRAM_RAS_N,
    output reg        DRAM_UDQM,
    output reg        DRAM_WE_N
  );

  
  reg [15:0] DRAM_DQ_0;
  reg [4:0]  DRAMState, prevState; /* Updated to 5 bits to accommodate additional states */
  wire       refreshReq;

  assign DRAM_CLK   = CLK100MHz;
 	assign refreshReq = 0;

  /* Write-port address generator */
  always @(posedge CLK100MHz or negedge resetN)
  begin
    if (!resetN) begin
      DRAMWriteAck <= 0;
      DRAMState    <= INIT0;
      prevState    <= INIT0;
      DRAM_ADDR    <= 0;
      DRAM_BA      <= 0;
      DRAM_CAS_N   <= 1;
      DRAM_CKE     <= 1;
      DRAM_CS_N    <= 1;
      DRAM_DQ_0    <= 0;
      DRAM_LDQM    <= 0;
      DRAM_UDQM    <= 0;
      DRAM_RAS_N   <= 1;
      DRAM_WE_N    <= 1;
    end else begin
      case (DRAMState)
        INIT0:    DRAMState <= INIT1;
        INIT1:    DRAMState <= INIT2;
        INIT2:    DRAMState <= INIT3;
        INIT3:    DRAMState <= INIT4;
        INIT4:    DRAMState <= INIT5;
        INIT5:    DRAMState <= INIT6;
        INIT6:    DRAMState <= INIT7;
        INIT7:    DRAMState <= IDLE;
        IDLE:     begin
                    if (refreshReq) begin
                      DRAMState <= REFRESH0;
                    end else if (DRAMReadReq) begin
                      DRAMState   <= READ0;
                      DRAMReadAck <= 1;
                    end else if (DRAMWriteReq) begin 
                      DRAMState    <= WRITE0;
                      DRAMWriteAck <= 1;
                    end
                  end
        WRITE0:   DRAMState <= WRITE1;
        WRITE1:   DRAMState <= WRITE2;
        WRITE2:   DRAMState <= WRITE3;
        WRITE3:   begin
                    if (!DRAMWriteReq) begin
                      DRAMWriteAck <= 0;
                      DRAMState    <= IDLE;
                    end
                  end
        READ0:    DRAMState <= READ1;
        READ1:    DRAMState <= READ2;
        READ2:    DRAMState <= READ3;
        READ3:    begin
                    if (!DRAMReadReq) begin
                      DRAMReadAck <= 0;
                      DRAMState   <= IDLE; 
                    end
                  end
        REFRESH0: DRAMState <= REFRESH1;
        REFRESH1: DRAMState <= REFRESH2;
        REFRESH2: DRAMState <= IDLE;
        default:  begin
                    DRAMWriteAck <= 0;
                    DRAMState <= INIT0;
                  end
      endcase
    end
  end
endmodule
