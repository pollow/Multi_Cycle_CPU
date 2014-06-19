//module mux2to1_5(
//    input [4:0] a,
//    input [4:0] b,
//    input sel,
//    output [4:0] o
//    );
//	assign o = sel? a : b;
//
//endmodule

module mux2to1_32(
    input [31:0] a,
    input [31:0] b,
    input sel,
    output [31:0] o
    );
assign o = sel ? a : b;

endmodule

module mux4to1_32(
    input [31:0] a,
	 input [31:0] b,
	 input [31:0] c,
	 input [31:0] d,
    input [1:0] sel,
    output [31:0] o
    );
assign o = sel[1] ? (sel[0]? d: c) : (sel[0]? b: a) ;

endmodule

module mux4to1_5(
    input [5:0] a,
	 input [5:0] b,
	 input [5:0] c,
	 input [5:0] d,
    input [1:0] sel,
    output [5:0] o
    );
assign o = sel[1] ? (sel[0]? d: c) : (sel[0]? b: a) ;

endmodule
