module Final_Project(SW, KEY, CLOCK_50, VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK, VGA_SYNC,	VGA_R, VGA_G, VGA_B, LEDG, LEDR);

input [17:0] SW;
input CLOCK_50;
input [3:0] KEY;
output [7:0] LEDG;
output [17:0] LEDR;
output			VGA_CLK;   				//	VGA Clock
output			VGA_HS;					//	VGA H_SYNC
output			VGA_VS;					//	VGA V_SYNC
output			VGA_BLANK;				//	VGA BLANK
output			VGA_SYNC;				//	VGA SYNC
output	[9:0]	VGA_R;   				//	VGA Red[9:0]
output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
output	[9:0]	VGA_B;   				//	VGA Blue[9:0]

reg [7:0] x;
reg [6:0] y;
wire [2:0] colour;
reg wireEn;
wire hP, vP; //create pulses for drawing a slope and drawing a straight line
reg GEnd; //signals to leave the g state

reg [2:0] col;

wire [2:0] col0; //retrieve colours from each picture module
wire [2:0] col1;
wire [2:0] col2;
wire [2:0] col3;
wire [2:0] col4;
wire [2:0] col5;
wire [2:0] col6;
wire [2:0] col7;
wire [2:0] col8;
wire [2:0] col9;



reg peakReached;

wire [7:0] xstart;
wire [6:0] ystart;

assign xstart = 9; // assign x and y starting positions to (9, 60)
assign ystart = 60;

reg [7:0] xcount ;
reg [6:0] ycount ;


//assign colour = 3'b010; //Make the colour always green

wire enable;
assign enable = !KEY[0]; 
wire Heart_In;
assign Heart_In = !GPIO_0[0]; //Get input signal from external circuit actually checking for user's heartbeat

wire [7:0] XMAX;
wire [6:0] YMAX;
wire [6:0] YSTART;

assign XMAX = 160;
assign YMAX = 50; //temporary
assign YSTART = 60;

wire resetn;
assign resetn = KEY[3];



reg [7:0] beatCount;

reg [7:0] xpic;
reg [7:0] ypic;
 

//Now create the states
parameter A = 4'b0000, B = 4'b0001, C = 4'b0010, D = 4'b0011, E = 4'b0100, F = 4'b0101, G = 4'b0110, H =4'b0111, I = 4'b1000, J = 4'b1001, K = 4'b1010, L = 4'b1011; 

reg [4:0]ynext, ycurrent; //Y and ycurrent corresponding to the next-state and current-state variables of the FSM, respectively

reg [6:0] numbercol; //Stores location of image

reg [2:0] endcolour; //get the colour of the image

reg [2:0]digitnum; 

wire [7:0] getAdd; //get address for picture
wire [7:0] getAdd2; //get address for second picture



zeero u21(numbercol,CLOCK_50, 0, 0, col0); //instantiations of modules 
one u5(numbercol, CLOCK_50, 0, 0, col1);
two u23(numbercol, CLOCK_50, 0, 0, col2);
three u24(numbercol, CLOCK_50, 0, 0, col3);
four u25(numbercol, CLOCK_50, 0, 0, col4);
five u26(numbercol, CLOCK_50, 0, 0, col5);
six u27(numbercol, CLOCK_50, 0, 0, col6);
seven u28(numbercol, CLOCK_50, 0, 0, col7);
eight u29(numbercol, CLOCK_50, 0, 0, col8);
nein u30(numbercol, CLOCK_50, 0, 0, col9);


vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(wireEn),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK),
			.VGA_SYNC(VGA_SYNC),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120"; //OR 320 by 240, we haven't yet decided
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "display.mif";
		
		
		
FiftyHurts u1(hP, CLOCK_50,vP); //get the pulses
		
		//Finite state machine control
always@(*)
begin

	case(ycurrent) 

		A : if(enable)ynext = B;  
			else ynext = A;

//B: ynext = C; //Skip the reset state

		B: if(x > 159) 
		begin
		ynext=H; /*if(x == 160 & y == 60)*/  //reset state
		digitnum = 0;
		xpic <= 80;
ypic <= 6;
		end

			else if(Heart_In == 1) ynext = C;
			else ynext = B;
		C: if(ycount == 10)
				ynext = D; 
			else 
				ynext = C;
		D: if(ycount == 20)
				ynext = E;
			else 
				ynext = D;
		E: if(col == 3'b000)
				ynext = F;
			else 
				ynext = E;
		F: if(ycount == 8)
				ynext = G;
			else 
				ynext = F;
		G: if(y == 59)
				ynext = B;
			else 
				ynext = G;
		H: if (xcount == 9)
				ynext = I;
			else 
				ynext = H;
		I: if (ycount == 9 & digitnum == 0)
			begin
				digitnum = 1;
				ynext = J;
				xpic <= 90;
ypic <= 6;
			end
			else
			begin
				ynext = H;
				
			end
		J: if (xcount == 9)
				ynext = K;
			else 
				ynext = J;
		K: if (ycount == 9)
				ynext = L;
			else 
				ynext = J;
			

	endcase

end

always @ (posedge CLOCK_50)

begin

ycurrent <= ynext;


if (ycurrent == A)
begin
col <= 3'b010;
wireEn <= 0;


end


else if (ycurrent == B)
begin
col <= 3'b010;
ycount <= 0;
y <= ystart -ycount;

if (x > 159)
begin
	ycount <= 0;
	xcount <= 0;
	
end

if (hP)
begin
x <= xstart + xcount;
		y <= ystart + ycount;
		wireEn <= 1;
		xcount <= xcount + 1;
end
end // end B state

else if (ycurrent == C)   //      C state
begin
col <= 3'b010;
if (vP)
begin
	ycount <= ycount + 1;
	y <= ystart - ycount;
end

	if (ycount == 10)
	begin
		
		xcount <= xcount + 1;
		x <= xstart + xcount;
		beatCount <= beatCount + 1;
	end // end the x-increment condition
	
end

else if (ycurrent == D)   //      D state
begin
	col <= 3'b010;
	if (vP)
begin
	ycount <= ycount + 1;
	y <= ystart - ycount;
end
	
	if (ycount == 20)
	begin
		
		xcount <= xcount + 1;
		x <= xstart + xcount;
		
	end // end the x-increment condition
	
end

else if (ycurrent == E)   //      E state
begin
	
if (vP)
begin
	ycount <= ycount + 1;
	y <= ystart - ycount;
end
	
	if (ycount == 30)
	begin
	wireEn <= 0;
		ycount <= 19;
		xcount <= xcount + 1;
		y <= ystart - ycount;
		x <= xstart + xcount;
		col <= 3'b000;
		
	end // end the x-increment condition

end

else if (ycurrent == F)   //      F state
begin
	col <= 3'b010;
if (vP)
begin
	ycount <= ycount - 1;
	y <= ystart - ycount;
	wireEn <= 1;

end
	
	if (ycount == 8)
	begin
		
		xcount <= xcount + 1;
		x <= xstart + xcount;
		
	end // end the x-increment condition
	
end

else if (ycurrent == G)   //      G state
begin
	col <= 3'b010;
	if (vP)
begin

	ycount <= ycount - 1;
y <= ystart - ycount;


end
	
	if (y == 60)
	begin
		
		
	end // end the x-increment condition
	
end

else if (ycurrent == H) //H state
begin
	
	
	col <= endcolour;
	
	x <= xcount + xpic;
	y <= ycount + ypic;
	
	xcount <= xcount + 1;
	numbercol <= numbercol + 1;
	
	
	
end //end the H state

else if (ycurrent == I) // I state
begin
	
	ycount <= ycount + 1;
	xcount <= 0;
	
	if (ycount == 9)
	begin
		ycount <= 0;
		xcount <= 0;
		numbercol <= 0;
	
	end
	

end //end the I state

else if (ycurrent == J) //J state`
begin



col <= endcolour;

x <= xcount + xpic;
y <= ycount + ypic;
	
xcount <= xcount + 1;
numbercol <= numbercol + 1;
	


end // End the J state


else if (ycurrent == K) // K state

begin

	xcount <= 0;
	ycount <= ycount + 1;


end //End the K state



end //end the always block

assign colour = col;

assign LEDR [12:0] = xpic;
assign LEDG = ycurrent;
assign LEDR[17] = digitnum;

always @ (*)
begin
	
	if (beatCount == 0 & digitnum == 0)
		endcolour <= col0;
	else if (beatCount == 0 & digitnum == 1)
		endcolour <= 3'b000;
	else if (beatCount == 20 & digitnum == 0)
		endcolour <= col8;
	else if (beatCount == 20 & digitnum == 1)
		endcolour <= col0;
	else if (beatCount == 1 & digitnum == 0)
		endcolour <= col4;
	else if (beatCount == 1 & digitnum == 1)
		endcolour <= 3'b000;
	else if (beatCount == 2 & digitnum == 0)
		endcolour <= col8;
	else if (beatCount == 2 & digitnum == 1)
		endcolour <= 3'b000;
	else if (beatCount == 3 & digitnum == 0)
		endcolour <= col1;
	else if (beatCount == 3 & digitnum == 1)
		endcolour <= col2;
	else if (beatCount == 4 & digitnum == 0)
		endcolour <= col1;
	else if (beatCount == 4 & digitnum == 1)
		endcolour <= col6;	
	else if (beatCount == 5 & digitnum == 0)
		endcolour <= col2;
	else if (beatCount == 5 & digitnum == 1)
		endcolour <= col0;
	else if (beatCount == 6 & digitnum == 0)
		endcolour <= col2;
	else if (beatCount == 6 & digitnum == 1)
		endcolour <= col4;
	else if (beatCount == 7 & digitnum == 0)
		endcolour <= col2;
	else if (beatCount == 7 & digitnum == 1)
		endcolour <= col8;
	else if (beatCount == 8 & digitnum == 0)
		endcolour <= col3;
	else if (beatCount == 8 & digitnum == 1)
		endcolour <= col2;
	else if (beatCount == 9 & digitnum == 0)
		endcolour <= col3;
	else if (beatCount == 9 & digitnum == 1)
		endcolour <= col6;
	else if (beatCount == 10 & digitnum == 0)
		endcolour <= col4;
	else if (beatCount == 10 & digitnum == 1)
		endcolour <= col0;
	else if (beatCount == 11 & digitnum == 0)
		endcolour <= col4;
	else if (beatCount == 11 & digitnum == 1)
		endcolour <= col4;
	else if (beatCount == 12 & digitnum == 0)
		endcolour <= col4;
	else if (beatCount == 12 & digitnum == 1)
		endcolour <= col8;
	else if (beatCount == 13 & digitnum == 0)
		endcolour <= col5;
	else if (beatCount == 13 & digitnum == 1)
		endcolour <= col2;
	else if (beatCount == 14 & digitnum == 0)
		endcolour <= col5;
	else if (beatCount == 14 & digitnum == 1)
		endcolour <= col6;
	else if (beatCount == 15 & digitnum == 0)
		endcolour <= col6;
	else if (beatCount == 15 & digitnum == 1)
		endcolour <= col0;
	else if (beatCount == 16 & digitnum == 0)
		endcolour <= col6;
	else if (beatCount == 16 & digitnum == 1)
		endcolour <= col4;
	else if (beatCount == 17 & digitnum == 0)
		endcolour <= col6;
	else if (beatCount == 17 & digitnum == 1)
		endcolour <= col8;
	else if (beatCount == 18 & digitnum == 0)
		endcolour <= col7;
	else if (beatCount == 18 & digitnum == 1)
		endcolour <= col2;
	else if (beatCount == 19 & digitnum == 0)
		endcolour <= col7;
	else if (beatCount == 19 & digitnum == 1)
		endcolour <= col6;
	else if (beatCount == 21 & digitnum == 0)
		endcolour <= col8;
	else if (beatCount == 21 & digitnum == 1)
		endcolour <= col4;
	else if (beatCount == 22 & digitnum == 0)
		endcolour <= col8;
	else if (beatCount == 22 & digitnum == 1)
		endcolour <= col8;
	else if (beatCount == 23 & digitnum == 0)
		endcolour <= col9;
	else if (beatCount == 23 & digitnum == 1)
		endcolour <= col2;
	else if (beatCount == 24 & digitnum == 0)
		endcolour <= col9;
	else if (beatCount == 24 & digitnum == 1)
		endcolour <= col6;
	else if (beatCount > 24 & digitnum == 0)
		endcolour <= 3'b111;
	else if (beatCount > 24 & digitnum == 1)
		endcolour <= 3'b111;
	
end


endmodule

 //--------------------------------------------------------------- END OF THE MAIN MODULE -------------------------------------------


module FiftyHurts (horizontalPulse,clk,slopePulse);
		
output horizontalPulse;
output slopePulse;

input clk;
reg [25:0] count;
reg [25:0] count2;

wire cond1,cond2,cond3,cond4;


always @ (posedge clk)
begin

if (slopePulse)
count2 <= 26'b0;
else
count2 <= count2 + 1;
if (horizontalPulse)
count <= 26'b0;
else
count <= count + 1'b1;
end

assign horizontalPulse = count == 26'd4999999;
assign slopePulse = count2 == 26'd499999;

endmodule