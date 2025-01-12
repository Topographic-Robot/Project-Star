module vgaGen(

pixClock, resetN, VSync, HSync, red, green, blue 

);

input pixClock;
input resetN;
output VSync;
output HSync;
output [3:0] red;
output [3:0] green;
output [3:0] blue;



reg VSync;
reg HSync;
reg [3:0] red;
reg [3:0] green;
reg [3:0] blue;
 

   reg [10:0] pix_count;
   reg [9:0]  line_count;

initial
   begin
   pix_count   = 0;
   line_count  = 0;
   end

//Pixel and Line counters
always @(posedge pixClock, negedge resetN)
	begin
   if (~resetN)
	   begin
		line_count <= 0;
		pix_count <= 0;	
	   end
	
   else if (pix_count == 799)
      begin
   	pix_count <= 0;
      if (line_count == 524)
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

//VSYNC and HSYNC generation

always @(posedge pixClock, negedge resetN)
	begin
	if (~resetN)
		begin
		VSync <= 0;
		HSync <= 0;	
		end
   else
		begin
		//VSYNC
		if ((line_count >= 0) && (line_count <= 1))
			begin
			VSync <= 0;
			end
		else
			begin
			VSync <= 1;
			end
		//HSync
		if ((pix_count >= 0) && (pix_count <= 95))
			begin
      	HSync <= 0;
			end
		else
         begin
      	HSync <= 1;
         end
      end
   end
   

//color bar generator
always @(posedge pixClock, negedge resetN)
	begin
	if (~resetN)
      begin
		red   <= 0;
		green <= 0;
		blue  <= 0;
      end
   //VSYNC
   else if ((line_count >= 514) || (line_count <= 34) || (pix_count >= 783) || (pix_count <= 143))
      begin
		red   <= 0;
		green <= 0;
		blue  <= 0;
      end
   else
      begin
		case(pix_count)
			144 : begin
				   red  <= 15;
				   green <= 0;
				   blue  <= 0;
			      end
			235 : begin
				   red   <= 7;
				   green <= 8;
				   blue  <= 0;
			      end
			
			326 : begin
				red <= 7;
				green <= 0;
				blue <= 8;
			end
			
			417 : begin
				red <= 0;
				green <= 15;
				blue <= 0;
			end
			
			508 : begin
				red <= 0;
				green <= 7;
				blue <= 8;
			end
			
			599 : begin
				red <= 15;
				green <= 15;
				blue <= 15;
			end
		endcase
      
		end
   //HSync
   end
   

endmodule