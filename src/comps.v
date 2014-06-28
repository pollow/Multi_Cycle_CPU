module glitch_filter(
	clk,
	s_in,
	s_out
);
	input clk;
	input s_in;
	output s_out;
	
	reg s_tmp;
	reg [31:0]counter_low, counter_high;
	
	initial begin
		counter_low <= 0;
		counter_high <= 0;
	end
	
	assign s_out = s_tmp;
	
	always @(posedge clk) begin
		if(s_in == 1'b0) 
			counter_low <= counter_low + 1;
		else 
			counter_low <= 0;
	end
	
	always @(posedge clk) begin
		if(s_in == 1'b1) 
			counter_high <= counter_high + 1;
		else
			counter_high <= 0;
	end
	
	always @(posedge clk) begin
		if (counter_low == 4) s_tmp <= 0;
		else if (counter_high == 4) s_tmp <= 1;
	end
	
endmodule

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
	output [3:0] button_out;
	output [7:0] SW_OK;

	glitch_filter G0(clk, button[0], button_out[0]);
	glitch_filter G1(clk, button[1], button_out[1]);
	glitch_filter G2(clk, button[2], button_out[2]);
	glitch_filter G3(clk, button[3], button_out[3]);
	glitch_filter G4(clk, SW[0], SW_OK[0]);
	glitch_filter G5(clk, SW[1], SW_OK[1]);
	glitch_filter G6(clk, SW[2], SW_OK[2]);
	glitch_filter G7(clk, SW[3], SW_OK[3]);
	glitch_filter G8(clk, SW[4], SW_OK[4]);
	glitch_filter G9(clk, SW[5], SW_OK[5]);
	glitch_filter G10(clk, SW[6], SW_OK[6]);
	glitch_filter G11(clk, SW[7], SW_OK[7]);

endmodule


module display(
    input clk, rst, mode,
    input [31:0]disp_num,
    output reg [7:0]seg,
    output reg [3:0]anode
);

    reg [26:0]tick;
    reg [1:0]an;
    reg [3:0]num;
    reg t;
	reg [7:0]dots;
    
    initial begin
        an <= 2'b00;
        tick <= 0;
		dots <= 0;
		num <= 0;
    end
    
    always @(posedge clk or posedge rst) begin
        if (rst == 1'b1) tick <= 0;
		else tick <= tick+1;
	end
    
    always @(posedge tick[16] or posedge rst) begin
        if (rst == 1'b1) an <= 0;
		else an <= an + 1;
	end
	
    always @(an) begin
        if (rst == 1'b1) begin
			anode <= 4'b1111;
			num <= 0;
			dots <= 0;
		end else begin 
		anode <= ~(4'b1<<an);
			case(an) 
				2'b00: begin 
					num <= disp_num[3:0]; 
					dots <= {disp_num[24], disp_num[0], disp_num[4], disp_num[16], disp_num[25], disp_num[17], disp_num[5], disp_num[12]}; 
				end
				2'b01: begin 
					num <= disp_num[7:4]; 
					dots <= {disp_num[26], disp_num[1], disp_num[6], disp_num[18], disp_num[27], disp_num[19], disp_num[7], disp_num[13]}; 
				end
				2'b10: begin 
					num <= disp_num[11:8]; 
					dots <= {disp_num[28], disp_num[2], disp_num[8], disp_num[20], disp_num[29], disp_num[21], disp_num[9], disp_num[14]}; 
				end
				2'b11: begin 
					num <= disp_num[15:12]; 
					dots <= {disp_num[30], disp_num[3], disp_num[10], disp_num[22], disp_num[31], disp_num[23], disp_num[11], disp_num[15]}; 
				end
				default:;
			endcase
		end
    end

    always @(*) begin
        if (rst == 1'b1) seg <= 0;
		else begin
			if(mode==1'b1) begin
				case(num)
					4'h0 : seg[7:0] <= 8'b10000001;
					4'h1 : seg[7:0] <= 8'b11001111;
					4'h2 : seg[7:0] <= 8'b10010010;
					4'h3 : seg[7:0] <= 8'b10000110;
					4'h4 : seg[7:0] <= 8'b11001100;
					4'h5 : seg[7:0] <= 8'b10100100;
					4'h6 : seg[7:0] <= 8'b10100000;
					4'h7 : seg[7:0] <= 8'b10001111;
					4'h8 : seg[7:0] <= 8'b10000000;
					4'h9 : seg[7:0] <= 8'b10000100;
					4'hA : seg[7:0] <= 8'b10001000;
					4'hB : seg[7:0] <= 8'b11100000;
					4'hC : seg[7:0] <= 8'b10110001;
					4'hD : seg[7:0] <= 8'b11000010;
					4'hE : seg[7:0] <= 8'b10110000;
					default : seg[7:0] <= 8'b10111000;
				endcase
			end else seg[7:0] <= dots;
		end
	end

endmodule

module seven_seg_dev(
	input wire [31:0] disp_num,
	input wire clk,
	input wire clr,
	input wire [1:0]SW,
//	input wire [1:0] Scanning,
	output wire [7:0] SEGMENT,
	output wire [3:0] AN
);
	reg [31:0] number;
	
	initial number <= 0;
	display D0(clk, clr, SW[0], number, SEGMENT, AN);
	always @(*) begin
		case (SW) 
			2'b01 : number <= { 16'b0, disp_num[15:0] };
			2'b11 : number <= { 16'b0, disp_num[31:16] };
			default : number <= disp_num;
		endcase
	end
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

	initial begin
		for (i=1; i<32; i=i+1)
			register[i] <= 0;//i;
	end
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
	initial clkdiv <= 0;
	always @ (posedge clk or posedge rst) begin
		if (rst) begin
			clkdiv <= 0;
		end else begin
			clkdiv <= clkdiv + 1'b1;
		end
	end
	assign Clk_CPU = SW2 ? clkdiv[22] : clkdiv[1];
endmodule

module seven_seg_Dev_IO( 
	input wire clk,
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
	output reg [31:0] disp_num
);

	always @(negedge clk or posedge rst) begin
		if (rst) disp_num <= 32'h0000;
		else begin
			case (Test) 
				0: 	begin 
						if(GPIOe0000000_we) disp_num <= disp_cpudata;
						else disp_num <= disp_num;
					end
				1:	disp_num <= Test_data0;
				2:	disp_num <= Test_data1;
				3:	disp_num <= Test_data2;
				4:	disp_num <= Test_data3;
				5:	disp_num <= Test_data4;
				6:	disp_num <= Test_data5;
				7:	disp_num <= Test_data6;
			endcase
		end
	end
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
	output reg [31:0] Cpu_data4bus, //write to CPU
	output reg [31:0] ram_data_in, //from CPU write to Memory
	output reg [9: 0] ram_addr, //Memory Address signals
	output reg data_ram_we, 
	output reg GPIOf0000000_we,
	output reg GPIOe0000000_we,
	output reg counter_we,
	output reg [31: 0] Peripheral_in
);

	reg [7:0] led_in;

	always @(*) begin
		data_ram_we = 0;
		counter_we = 0;
		GPIOe0000000_we = 0;
		GPIOf0000000_we = 0;
		ram_addr = 10'h0;
		ram_data_in = 32'h0;
		Peripheral_in = 32'h0;
		Cpu_data4bus = 32'h0;
		case(addr_bus[31:28])
			4'h0: begin
				data_ram_we = mem_w	;
				ram_addr = addr_bus[11:2];
				ram_data_in = Cpu_data2bus;
				Cpu_data4bus = ram_data_out;
			end
			4'he: begin
				GPIOf0000000_we = mem_w;
				Peripheral_in = Cpu_data2bus;
				Cpu_data4bus = counter_out;
			end
			4'hf: begin
				if(addr_bus[2]) begin
					counter_we = mem_w;
					Peripheral_in = Cpu_data2bus;
					Cpu_data4bus = counter_out;
				end else begin
					GPIOf0000000_we = mem_w;
					Peripheral_in = Cpu_data2bus;
					Cpu_data4bus = {counter0_out, counter1_out, counter2_out, 9'h00, led_out, BTN, SW};
				end
			end
		endcase
	end
endmodule

module Counter_x( input wire clk,
	input wire 	      rst,
	input wire 	      clk0,
	input wire 	      clk1,
	input wire 	      clk2,
	input wire 	      counter_we,
	input wire  [31:0]counter_val,
	input wire  [1:0] counter_ch,
	output wire 	   counter0_OUT,
	output wire 	   counter1_OUT,
	output wire 	   counter2_OUT,
	output wire [31:0]counter_out
	);
endmodule

module led_Dev_IO( 
	input wire clk ,
	input wire rst,
	input wire GPIOf0000000_we,
	input wire [31:0] Peripheral_in,
	output reg [1:0] counter_set,
	output wire [7:0] led_out,
	output reg [21:0] GPIOf0
);
	
	reg [7:0]LED;

	assign led_out = LED;

	always @(negedge clk or posedge rst) begin
		if (rst) begin
			LED <= 8'hAA;
			counter_set <= 2'b00;
		end else begin
			if (GPIOf0000000_we) {GPIOf0[21:0], LED, counter_set} <= Peripheral_in;
			else begin
				LED <= LED;
				counter_set <= counter_set;
			end
		end
	end
endmodule
