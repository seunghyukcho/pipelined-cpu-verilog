`timescale 1ns / 100ps

`define	NumBits	16

module FullAdder_1bit (A, B, S, Cin, Cout);
	input A;
	input B;
	input Cin;
	output S;
	output Cout;

	assign S = Cin ^ (A ^ B);
	assign Cout = (A & B) | ((Cin & A) | (Cin & B));
endmodule

module FullAdder_4bit (A, B, S, Cin, Cout);
	input [3:0] A;
	input [3:0] B;
	input Cin;
	output [3:0] S;
	output Cout;

	wire Carry0, Carry1, Carry2;

	FullAdder_1bit adder0(A[0], B[0], S[0], Cin, Carry0);
	FullAdder_1bit adder1(A[1], B[1], S[1], Carry0, Carry1);
	FullAdder_1bit adder2(A[2], B[2], S[2], Carry1, Carry2);
	FullAdder_1bit adder3(A[3], B[3], S[3], Carry2, Cout);
endmodule

module FullAdder_16bit (A, B, S, Cin, Cout);
	input [15:0] A;
	input [15:0] B;
	input Cin;
	output [15:0] S;
	output Cout;

	wire Carry0, Carry1, Carry2;
	
	FullAdder_4bit adder0(A[3:0], B[3:0], S[3:0], Cin, Carry0);
	FullAdder_4bit adder1(A[7:4], B[7:4], S[7:4], Carry0, Carry1);
	FullAdder_4bit adder2(A[11:8], B[11:8], S[11:8], Carry1, Carry2);
	FullAdder_4bit adder3(A[15:12], B[15:12], S[15:12], Carry2, Cout);
endmodule

module Not_16bit (A, S);
	input [15:0] A;
	output [15:0] S;
	
	genvar i;
	for(i = 0; i < 16; i = i + 1)
		assign S[i] = ~A[i];
endmodule

module And_16bit (A, B, S);
	input [15:0] A;
	input [15:0] B;
	output [15:0] S;
	
	genvar i;
	for(i = 0; i < 16; i = i + 1)
		assign S[i] = A[i] & B[i];
endmodule

module Or_16bit (A, B, S);
	input [15:0] A;
	input [15:0] B;
	output [15:0] S;
	
	genvar i;
	for(i = 0; i < 16; i = i + 1)
		assign S[i] = A[i] | B[i];
endmodule

module ComplementOfTwo_16bit (A, S);
	input [15:0] A;
	output [15:0] S;

	wire [15:0] B;
	wire Cout;
	Not_16bit notA(A, B);
	FullAdder_16bit adder(B, 16'b1, S, 1'b0, Cout);
endmodule

module Lshift_16bit (A, S);
	input [15:0] A;
	output [15:0] S;

	genvar i;
	for(i = 1; i < 16; i = i + 1) begin
		assign S[i] = A[i - 1];
	end
	assign S[0] = 0;
endmodule

module ArithmeticRshift_16bit (A, S);
	input [15:0] A;
	output [15:0] S;

	genvar i;
	for(i = 0; i < 15; i = i + 1) begin
		assign S[i] = A[i + 1];
	end
	assign S[15] = A[15];
endmodule

module Subtract_16bit (A, B, S, Ov);
	input [15:0] A;
	input [15:0] B;
	output [15:0] S;
	output Ov;
	wire Cout;
	wire [15:0] minusB;

	ComplementOfTwo_16bit complementer(B, minusB);
	FullAdder_16bit adder(A, minusB, S, 1'b0, Cout);
	assign Ov = (~(A[15] ^ minusB[15])) & (A[15] ^ S[15]);
endmodule

module ALU (A, B, FuncCode, C, BCond);
	input [`NumBits-1:0] A;
	input [`NumBits-1:0] B;
	input [3:0] FuncCode;
	output [`NumBits-1:0] C;
	output BCond;

	reg [`NumBits-1:0] C;
	reg BCond;

	// You can declare any variables as needed.
	wire [`NumBits-1:0] adderSumOut;
	wire [`NumBits-1:0] subtractOutput;
	wire subtracterOverflow;
	wire [`NumBits-1:0] notOutput;
	wire [`NumBits-1:0] orOutput;
	wire [`NumBits-1:0] andOutput;
	wire [`NumBits-1:0] nandOutput;
	wire [`NumBits-1:0] norOutput;
	wire [`NumBits-1:0] complementOutput;
	wire [`NumBits-1:0] xorOutput;
	wire [`NumBits-1:0] xnorOutput;
	wire [`NumBits-1:0] LshiftOutput;
	wire [`NumBits-1:0] LogicRshiftOutput;
	wire [`NumBits-1:0] ArithmeticRshiftOutput;

	wire adderCarryOut;

	FullAdder_16bit adder(A, B, adderSumOut, 1'b0, adderCarryOut);
	Subtract_16bit subtracter(A, B, subtractOutput, subtracterOverflow);
	Not_16bit notter(A, notOutput);
	And_16bit ander(A, B, andOutput);
	Or_16bit orer(A, B, orOutput);
	ComplementOfTwo_16bit complementer(A, complementOutput);
	Lshift_16bit lshifter(A, LshiftOutput);
	ArithmeticRshift_16bit arithmeticshifter(A, ArithmeticRshiftOutput);

	initial begin
		C = 0;
		BCond = 0;
	end   	

	always @(A or B or FuncCode) begin
		case(FuncCode)
			4'b0000 : begin
				// 0. Signed addition
				C = adderSumOut;
				BCond = 0;
			end
			4'b0001 : begin
				// 1. Signed subtraction
				C = subtractOutput;
				BCond = 0;
			end
			4'b0010 : begin
				// 2. Identity A
				C = A;
				BCond = 0;
			end
			4'b0011 : begin
				// 3. Bitwise NOT
				C = notOutput;
				BCond = 0;
			end
			4'b0100 : begin
				// 4. Bitwise AND
				C = andOutput;
				BCond = 0;
			end
			4'b0101 : begin
				// 5. Bitwise OR
				C = orOutput;
				BCond = 0;
			end
			4'b0110 : begin
				// 6. A != B check
				C = A;
				BCond = (A != B);
			end
			4'b0111 : begin
				// 7. A == B check
				C = A;
				BCond = (A == B);
			end
			4'b1000 : begin
				// 8. A > 0 check
				C = A;
				BCond = (A[15] == 0 && A != 0 ? 1 : 0);
			end
			4'b1001 : begin
				// 9. A < 0 check
				C = A;
				BCond = (A[15] == 1 ? 1 : 0);
			end
			4'b1010 : begin
				// 10. Identity B
				C = B;
				BCond = 0;
			end
			4'b1011 : begin
				// 11. Empty
				C = A;
				BCond = 0;
			end
			4'b1100 : begin
				// 12. Arithmetic left shift
				C = LshiftOutput;
				BCond = 0;
			end
			4'b1101 : begin
				// 13. Arithmetic right shift
				C = ArithmeticRshiftOutput;
				BCond = 0;
			end
			4'b1110 : begin
				// 14. Two's complement
				C = complementOutput;
				BCond = 0;
			end
			default : begin
				// 15. Zero
				C = 0;
				BCond = 0;
			end
		endcase
	end

endmodule

