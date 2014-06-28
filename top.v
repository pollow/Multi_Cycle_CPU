`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   19:56:12 06/23/2014
// Design Name:   Top_Muliti_IOBUS
// Module Name:   C:/Users/Deus/Windows Sync/Xilinx Workspace/Multi_Cycle_CPU/top.v
// Project Name:  M_C_C
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: Top_Muliti_IOBUS
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module top;

	// Inputs
	reg clk_50mhz;
	reg [3:0] BTN;
	reg [7:0] SW;

	// Outputs
	wire [7:0] LED;
	wire [7:0] SEGMENT;
	wire [3:0] AN_SEL;

	// Instantiate the Unit Under Test (UUT)
	Top_Muliti_IOBUS uut (
		.clk_50mhz(clk_50mhz), 
		.BTN(BTN), 
		.SW(SW), 
		.LED(LED), 
		.SEGMENT(SEGMENT), 
		.AN_SEL(AN_SEL)
	);
		integer i;

	initial begin
		// Initialize Inputs
		clk_50mhz = 0;
		BTN = 0;
		SW = 8'b00100001;

		// Wait 100 ns for global reset to finish
		#100;
		BTN[3] = 1;
		#10;
		for( i = 0; i < 5; i = i+1) #10 clk_50mhz = ~clk_50mhz;
		BTN[3] = 0;
		// Add stimulus here
	end
	
	always #5 clk_50mhz = ~clk_50mhz;
      
endmodule

