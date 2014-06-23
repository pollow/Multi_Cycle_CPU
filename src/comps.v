module BTN_Anti_jitter(
   // Outputs
   button_out, 
   SW_OK,
   // Inputs
   clk, 
   button, 
   SW
);
	input  clk;
	input  [3:0] button;
	input  [7:0] SW;
	output reg [3:0] button_out;
	output reg [7:0] SW_OK;

endmodule

module seven_seg_dev(input wire [31:0] disp_num,
	input wire clk,
	input wire clr,
	input wire [1:0]SW,
	input wire [1:0] Scanning,
	output wire [7:0] SEGMENT,
	output reg [3:0] AN
);


endmodule

module Regs(
	clk, 
	rst,
	reg_R_addr_A,
	reg_R_addr_B,
	reg_W_addr,
	wdata,
	reg_we,
	rdata_A,
	rdata_B
);
	input clk, rst, reg_we;
	input [4:0] reg_R_addr_A, reg_R_addr_B, reg_W_addr;
	input [31:0] wdata;
	output [31:0] rdata_A, rdata_B;
	reg [31:0] register [1:31];// r1 - r31
	integer i;

	assign rdata_A = (reg_R_addr_A == 0) ? 0 : register[reg_R_addr_A];//read
	assign rdata_B = (reg_R_addr_B == 0) ? 0 : register[reg_R_addr_B];//read

	always @(posedge clk or posedge rst) begin
		if (rst==1) begin// reset
			for (i=1; i<32; i=i+1)
				register[i] <= 0;//i;
		end else begin
			if ((reg_W_addr != 0) && (reg_we == 1))
			register[reg_W_addr] <= wdata;
		end
	end

endmodule


module alu(A, B, ALU_operation, res, zero, overflow );
	input signed [31:0] A, B;
	input [2:0] ALU_operation;
	output [31:0] res;
	output zero, overflow ;
	wire [31:0] res_and, res_or, res_add, res_sub, res_nor, res_slt, res_xor, res_srl;
	reg [31:0] res;
	parameter one = 32'h00000001, zero_0 = 32'h00000000;

	assign res_and = A&B;
	assign res_or = A|B;
	assign res_add = A+B;
	assign res_sub = A-B;
	assign res_nor=~(A | B);
	assign res_slt =(A < B) ? one : zero_0;
	assign res_xor = A^B;
	assign res_srl = B>>1;

	always @*
	// (A or B or ALU_operation)
		case (ALU_operation)
			3'b000: res=res_and;
			3'b001: res=res_or;
			3'b010: res=res_add;
			3'b110: res=res_sub;
			3'b100: res=res_nor;
			3'b111: res=res_slt;
			3'b011: res=res_xor;
			3'b101: res=res_srl;
			default: res=res_add;
		//32'hx;
		endcase
	assign zero = (res==0)? 1: 0;

endmodule

module clk_div(
	input wire clk,
	input wire rst,
	input wire SW2,
	output reg [31:0] clkdiv,
	output wire Clk_CPU
);
	always @ (posedge clk or posedge rst) begin
		if (rst) begin
			clkdiv <= 0;
		end else begin
			clkdiv <= clkdiv + 1'b1;
		end
	end
	assign Clk_CPU = SW2 ? clkdiv[25] : clkdiv[1];
endmodule



module seven_seg_Dev_IO( input wire clk,
	input wire rst,
	input wire GPIOe0000000_we,
	input wire [2:0] Test,
	input wire [31:0] disp_cpudata,
	input wire [31:0] Test_data0,
	input wire [31:0] Test_data1,
	input wire [31:0] Test_data2,
	input wire [31:0] Test_data3,
	input wire [31:0] Test_data4,
	input wire [31:0] Test_data5,
	input wire [31:0] Test_data6,
	output wire [31:0] disp_num
);
endmodule

module MIO_BUS( input wire clk, input wire rst,
	input wire [3:0] BTN,
	input wire [7:0]SW,
	input wire mem_w,
	input wire [31:0] Cpu_data2bus,
	input wire [31:0] addr_bus, //data from CPU
	input wire [31:0] ram_data_out,
	input wire [7:0] led_out,
	input wire [31:0] counter_out,
	input wire counter0_out,
	input wire counter1_out,
	input wire counter2_out,
	output wire [31:0] Cpu_data4bus, //write to CPU
	output wire [31:0] ram_data_in, //from CPU write to Memory
	output wire [9: 0] ram_addr, //Memory Address signals
	output wire data_ram_we, 
	output wire GPIOf0000000_we,
	output wire GPIOe0000000_we,
	output wire counter_we,
	output wire [31: 0] Peripheral_in
);

endmodule

module Counter_x( input wire clk,
	input wire 	     rst,
	input wire 	     clk0,
	input wire 	     clk1,
	input wire 	     clk2,
	input wire 	     counter_we,
	input wire [31:0]  counter_val,
	input wire [1:0]   counter_ch,
	output wire 	     counter0_OUT,
	output wire 	     counter1_OUT,
	output wire 	     counter2_OUT,
	output wire [31:0] counter_out
	);
endmodule

module led_Dev_IO( input wire clk ,
input wire rst,
input wire GPIOf0000000_we,
input wire [31:0] Peripheral_in,
output wire [1:0] counter_set,
output wire [7:0] led_out,
output wire [21:0] GPIOf0
);
endmodule

