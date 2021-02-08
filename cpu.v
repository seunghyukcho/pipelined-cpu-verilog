`timescale 1ns/1ns
`define WORD_SIZE 16    // data and address word size
`include "opcodes.v"
`include "ALU.v"

module Register(clk, rs1, rs2, rd, writeData, readData1, readData2, regWrite, reset_n);
	input clk;
	input [1:0] rs1;
	input [1:0] rs2;
	input [2:0] rd;
	input [`WORD_SIZE-1:0] writeData;
	input regWrite;
	input reset_n;
	output [`WORD_SIZE-1:0] readData1;
	output [`WORD_SIZE-1:0] readData2;

	reg [`WORD_SIZE-1:0] r[3:0];

	initial begin
		r[0] = 0;
		r[1] = 0;
		r[2] = 0;
		r[3] = 0;
	end

	assign readData1 = r[rs1];
	assign readData2 = r[rs2];

	always @(posedge reset_n) begin
		r[0] = 0;
		r[1] = 0;
		r[2] = 0;
		r[3] = 0;
	end

	always @(rd or writeData or regWrite) begin
		if(regWrite == 1) begin
			r[rd] = writeData;
		end
		else begin
			r[rd] = r[rd];
		end
	end
endmodule

module ImmediateGenerator(instruction, immediate);
	input [`WORD_SIZE-1:0] instruction;
	output [`WORD_SIZE-1:0] immediate;

	reg [`WORD_SIZE-1:0] immediate;

	always @(instruction) begin
		case(instruction[15:12])
			// Concat 0x00
			`LHI_OP: begin
				immediate[7:0] = 0;
				immediate[15:8] = instruction[7:0];
			end
			// Zero-extend
			`ORI_OP: begin
				immediate[15:8] = 0;
				immediate[7:0] = instruction[7:0];
			end
			`JMP_OP: begin
				immediate[11:0] = instruction[11:0];
				immediate[15:12] = 4'b0;
			end
			`JAL_OP: begin
				immediate[11:0] = instruction[11:0];
				immediate[15:12] = 4'b0;
			end
			// Sign-extend
			default: begin
				immediate[7:0] = instruction[7:0];
				immediate[8] = instruction[7];
				immediate[9] = instruction[7];
				immediate[10] = instruction[7];
				immediate[11] = instruction[7];
				immediate[12] = instruction[7];
				immediate[13] = instruction[7];
				immediate[14] = instruction[7];
				immediate[15] = instruction[7];
			end
		endcase
	end
endmodule

module ImmExtender(PC, inputImmediate, outputImmediate);
	input [`WORD_SIZE-1:0] PC;
	input [`WORD_SIZE-1:0] inputImmediate;
	output [`WORD_SIZE-1:0] outputImmediate;

	assign outputImmediate = inputImmediate[11:0] + (PC & `WORD_SIZE'b1111000000000000);
endmodule

module MUX2to1(input0, input1, selector, output0);
	input [`WORD_SIZE-1:0] input0;
	input [`WORD_SIZE-1:0] input1;
	input selector;
	output [`WORD_SIZE-1:0] output0;

	assign output0 = selector ? input1 : input0;
endmodule

module MUX4to1(input0, input1, input2, input3, selector, output0);
	input [`WORD_SIZE-1:0] input0;
	input [`WORD_SIZE-1:0] input1;
	input [`WORD_SIZE-1:0] input2;
	input [`WORD_SIZE-1:0] input3;
	input [1:0] selector;

	output [`WORD_SIZE-1:0] output0;

	assign output0 = (selector != 0 ? 
					(selector != 1 ? 
					(selector != 2 ? input3 : input2)
						: input1)
						: input0);
endmodule

module MUXPC(input0, input1, input2, selector, output0);
	input [2:0] input0, input1, input2;
	input [1:0] selector;

	output [2:0] output0;

	assign output0 = (selector == 3'b11 ? 3'b100 : 
					(selector == 3'b00 ? input0 : 
					(selector == 3'b01 ? input1 : input2)));
endmodule

module PC(writePC, outputPC, clk, reset_n, writeFlag);
	input reset_n;
	input clk;
	input [`WORD_SIZE-1:0] writePC;
	input writeFlag;

	output [`WORD_SIZE-1:0] outputPC;

	reg [`WORD_SIZE-1:0] outputPC;

	initial begin
		outputPC = 0;
	end

	always @(posedge reset_n) begin
		outputPC = 0;
	end

	always @(posedge clk) begin
		if(writeFlag)
			outputPC = writePC;
		else
			outputPC = outputPC;
	end
endmodule

module ALUControl(opCode, funcCode, ALUOp);
	input [3:0] opCode;
	input [5:0] funcCode;
	output [3:0] ALUOp;
	reg [3:0] ALUOp;

	always @(opCode or funcCode) begin
		case(opCode)
			// Signed addition
			`ADI_OP, `LWD_OP, `SWD_OP : begin
				ALUOp = 4'b0000;
			end
			// Bitwise OR
			`ORI_OP : begin
				ALUOp = 4'b0101;
			end
			// Select B
			`LHI_OP : begin
				ALUOp = 4'b1010;
			end
			// A != B
			`BNE_OP : begin
				ALUOp = 4'b0110;
			end
			// A == B
			`BEQ_OP : begin
				ALUOp = 4'b0111;
			end
			// A > 0
			`BGZ_OP : begin
				ALUOp = 4'b1000;
			end
			// A < 0
			`BLZ_OP : begin
				ALUOp = 4'b1001;
			end
			// ALU Operations
			`ALU_OP : begin
				case(funcCode)
					`INST_FUNC_ADD : begin
						ALUOp = 4'b0000;
					end
					`INST_FUNC_SUB : begin
						ALUOp = 4'b0001;
					end
					`INST_FUNC_AND : begin
						ALUOp = 4'b0100;
					end
					`INST_FUNC_NOT : begin
						ALUOp = 4'b0011;
					end
					`INST_FUNC_ORR : begin
						ALUOp = 4'b0101;
					end
					`INST_FUNC_TCP : begin
						ALUOp = 4'b1110;
					end
					`INST_FUNC_SHL : begin
						ALUOp = 4'b1100;
					end
					`INST_FUNC_SHR : begin
						ALUOp = 4'b1101;
					end
				endcase
			end
			// Default case - Select A
			default : begin
				ALUOp = 4'b0010;
			end
		endcase
	end
endmodule

module IFControl(opCode, funcCode, rdSelector, writeFlag);
	input [3:0] opCode;
	input [5:0] funcCode;
	input writeFlag;

	output [1:0] rdSelector;
	reg [1:0] rdSelector;

	initial begin
		rdSelector = 0;
	end

	always @(opCode or funcCode or writeFlag) begin
		if(writeFlag)
			rdSelector = (opCode == `JAL_OP || (opCode == `JRL_OP && funcCode == `INST_FUNC_JRL)) ? 2 :
						(opCode != `ALU_OP && opCode != `SWD_OP ? 1 : 0);
		else
			rdSelector = 0;
	end
endmodule

module EXControl(opCode, funcCode, memRead, ALUSrc, writeFlag);
	input [3:0] opCode;
	input [5:0] funcCode;
	input writeFlag;

	output memRead;
	output ALUSrc;
	reg memRead;
	reg ALUSrc;

	initial begin
		memRead = 0;
		ALUSrc = 0;
	end

	always @(opCode or funcCode or writeFlag) begin
		if(writeFlag) begin
			memRead = (opCode == `LWD_OP);
			ALUSrc = (opCode == `ADI_OP || opCode == `ORI_OP || opCode == `LHI_OP || opCode == `LWD_OP || opCode == `SWD_OP);
		end
		else begin
			memRead = 0;
			ALUSrc = 0;
		end
	end
endmodule

module MEMControl(opCode, funcCode, memWrite, regWrite, writeFlag);
	input [3:0] opCode;
	input [5:0] funcCode;
	input writeFlag;

	output memWrite;
	output regWrite;

	reg memWrite, regWrite;

	initial begin
		memWrite = 0;
		regWrite = 0;
	end

	always @(opCode or funcCode or writeFlag) begin
		if(writeFlag) begin
			memWrite = (opCode == `SWD_OP);
			regWrite = (opCode == `JAL_OP || opCode == `LWD_OP || (opCode == `ALU_OP && funcCode != `INST_FUNC_JPR && funcCode != 6'h1c) || opCode == `LHI_OP || opCode == `ADI_OP || opCode == `ORI_OP) ? 1 : 0;
		end
		else begin
			memWrite = 0;
			regWrite = 0;
		end
	end
endmodule

module WBControl(opCode, funcCode, dataSelector, regWrite, isHalt, isWWD, writeFlag);
	input [3:0] opCode;
	input [5:0] funcCode;
	input writeFlag;

	output [1:0] dataSelector;
	output regWrite;
	output isHalt;
	output isWWD;

	reg [1:0] dataSelector;
	reg regWrite;
	reg isHalt;
	reg isWWD;

	initial begin
		dataSelector = 0;
		regWrite = 0;
		isHalt = 0;
		isWWD = 0;
	end

	always @(opCode or funcCode or writeFlag) begin
		if(writeFlag) begin
			if(opCode == `LWD_OP)
				dataSelector = 1;
			else if(opCode == `JAL_OP || (opCode == `JRL_OP && funcCode == `INST_FUNC_JRL))
				dataSelector = 2;
			else if(opCode == `LHI_OP)
				dataSelector = 3;
			else
				dataSelector = 0;

			regWrite = (opCode == `JAL_OP || opCode == `LWD_OP || (opCode == `ALU_OP && funcCode != `INST_FUNC_JPR && funcCode != 6'h1c) || opCode == `LHI_OP || opCode == `ADI_OP || opCode == `ORI_OP) ? 1 : 0;
			isHalt = (opCode == `ALU_OP && funcCode == 29 ? 1 : 0);
			isWWD = (opCode == 4'd15 && funcCode == 6'd28);
		end
		else begin
			dataSelector = 0;
			regWrite = 0;
			isHalt = 0;
			isWWD = 0;
		end
	end
endmodule

module GeneralPipeline(clk, inPC, inNumInst, inOpCode, inFuncCode, inRS1, inRS2, inRD, inWriteFlag,
					PC, numInst, opCode, funcCode, rs1, rs2, rd, writeFlag);
	input clk;
	input [3:0] inOpCode;
	input [5:0] inFuncCode;
	input [1:0] inRS1;
	input [1:0] inRS2;
	input [2:0] inRD;
	input inWriteFlag;
	input [`WORD_SIZE-1:0] inPC;
	input inNumInst;

	output [3:0] opCode;
	output [5:0] funcCode;
	output [1:0] rs1;
	output [1:0] rs2;
	output [2:0] rd;
	output [`WORD_SIZE-1:0] PC;
	output numInst;
	output writeFlag;

	reg [3:0] opCode;
	reg [5:0] funcCode;
	reg [1:0] rs1;
	reg [1:0] rs2;
	reg [2:0] rd;
	reg [`WORD_SIZE-1:0] PC;
	reg numInst;
	reg writeFlag;

	initial begin
		opCode = 0;
		funcCode = 0;
		rs1 = 0;
		rs2 = 0;
		rd = 4;
		PC = 0;
		numInst = 0;
	end

	always @(posedge clk) begin
		writeFlag = inWriteFlag;
		opCode = inOpCode;
		funcCode = inFuncCode;
		rs1 = inRS1;
		rs2 = inRS2;
		rd = inRD;
		PC = inPC;
		numInst = inNumInst;
	end
endmodule

module IDPipeline(clk, inInstruction, inPC, instruction, PC, rdSelector, writeFlag, flushFlag);
	input clk;
	input [`WORD_SIZE-1:0] inInstruction;
	input [`WORD_SIZE-1:0] inPC;
	input writeFlag;
	input flushFlag;

	output [`WORD_SIZE-1:0] instruction;
	output [`WORD_SIZE-1:0] PC;
	output [1:0] rdSelector;

	reg [`WORD_SIZE-1:0] instruction;
	reg [`WORD_SIZE-1:0] PC;
	reg [1:0] rdSelector;

	reg [3:0] opCode;
	reg [5:0] funcCode;

	initial begin
		instruction = 0;
		PC = 0;
		rdSelector = 3;
	end
	
	always @(posedge clk) begin
		if(writeFlag) begin
			instruction = inInstruction;
			opCode = instruction[15:12];
			funcCode = instruction[5:0];
			PC = inPC;

			if((opCode == `ALU_OP && funcCode != `INST_FUNC_JPR && funcCode != 6'd29 && funcCode != 6'd28) || opCode == `JAL_OP || opCode == `LWD_OP || opCode == `ADI_OP || opCode == `ORI_OP || opCode == `LHI_OP)
				rdSelector =
					(opCode == `JAL_OP || (opCode == `JRL_OP && funcCode == `INST_FUNC_JRL)) ? 2 :
					(opCode != `ALU_OP && opCode != `SWD_OP ? 1 : 0);
			else
				rdSelector = 3;
		end
		
		if(flushFlag) begin
			instruction = 0;
			PC = 0;
			rdSelector = 3;
		end
	end
endmodule

module EXPipeline(clk, inPC, inNumInst, inOpCode, inFuncCode, inRS1, inRS2, inRD, inImm, inReadData1, inReadData2, inWriteFlag,
					memRead, ALUSrc, PC, numInst, opCode, funcCode, rs1, rs2, rd, imm, readData1, readData2, PCCtrl, writeFlag);
	input clk;
	input [3:0] inOpCode;
	input [5:0] inFuncCode;
	input [1:0] inRS1;
	input [1:0] inRS2;
	input [2:0] inRD;
	input [`WORD_SIZE-1:0] inImm;
	input [`WORD_SIZE-1:0] inReadData1, inReadData2;
	input [`WORD_SIZE-1:0] inPC;
	input inNumInst;
	input inWriteFlag;

	output memRead;
	output ALUSrc;
	output [3:0] opCode;
	output [5:0] funcCode;
	output [1:0] rs1;
	output [1:0] rs2;
	output [2:0] rd;
	output [`WORD_SIZE-1:0] imm;
	output [`WORD_SIZE-1:0] readData1, readData2;
	output [`WORD_SIZE-1:0] PC;
	output numInst;
	output [1:0] PCCtrl;
	output writeFlag;

	reg [`WORD_SIZE-1:0] imm;
	reg [`WORD_SIZE-1:0] readData1, readData2;
	reg [1:0] PCCtrl;

	GeneralPipeline EXGeneralPipeline(clk, inPC, inNumInst, inOpCode, inFuncCode, inRS1, inRS2, inRD, inWriteFlag, PC, numInst, opCode, funcCode, rs1, rs2, rd, writeFlag);
	EXControl Control(opCode, funcCode, memRead, ALUSrc, writeFlag);

	initial begin
		imm = 0;
		readData1 = 0;
		readData2 = 0;
		PCCtrl = 0;
	end

	always @(posedge clk) begin
		imm = inImm;
		readData1 = inReadData1;
		readData2 = inReadData2;
		if(writeFlag == 1) begin
			PCCtrl = 
				(inOpCode == `BNE_OP || inOpCode == `BEQ_OP || inOpCode == `BGZ_OP || inOpCode == `BLZ_OP) ? 2'b01 :
				(inOpCode == `JMP_OP || inOpCode == `JAL_OP) ? 2'b10 :
				(
					inOpCode == `JPR_OP && inFuncCode == `INST_FUNC_JPR ||
					inOpCode == `JRL_OP && inFuncCode == `INST_FUNC_JRL
				) ? 2'b11 : 2'b00;
		end
		else begin
			PCCtrl = 0;
		end
	end
endmodule

module MEMPipeline(clk, inPC, inNumInst, inOpCode, inFuncCode, inRS1, inRS2, inRD, inImmExtend, inALUResult, inReadData1, inReadData2, inMemRead, inWriteFlag,
					memRead, memWrite, regWrite, PC, numInst, opCode, funcCode, rs1, rs2, rd, immExtend, ALUResult, readData1, readData2, writeFlag);
	input clk;
	input [3:0] inOpCode;
	input [5:0] inFuncCode;
	input [1:0] inRS1;
	input [1:0] inRS2;
	input [2:0] inRD;
	input [`WORD_SIZE-1:0] inImmExtend;
	input [`WORD_SIZE-1:0] inALUResult;
	input [`WORD_SIZE-1:0] inPC;
	input inNumInst;
	input [`WORD_SIZE-1:0] inReadData1;
	input [`WORD_SIZE-1:0] inReadData2;
	input inMemRead;
	input inWriteFlag;

	output memRead;
	output memWrite;
	output regWrite;
	output [3:0] opCode;
	output [5:0] funcCode;
	output [1:0] rs1;
	output [1:0] rs2;
	output [2:0] rd;
	output [`WORD_SIZE-1:0] immExtend;
	output [`WORD_SIZE-1:0] ALUResult;
	output [`WORD_SIZE-1:0] PC;
	output numInst;
	output [`WORD_SIZE-1:0] readData1;
	output [`WORD_SIZE-1:0] readData2;
	output writeFlag;

	reg memRead;
	reg [`WORD_SIZE-1:0] immExtend;
	reg [`WORD_SIZE-1:0] ALUResult;
	reg [`WORD_SIZE-1:0] readData1;
	reg [`WORD_SIZE-1:0] readData2;

	GeneralPipeline MEMGeneralPipeline(clk, inPC, inNumInst, inOpCode, inFuncCode, inRS1, inRS2, inRD, inWriteFlag, PC, numInst, opCode, funcCode, rs1, rs2, rd, writeFlag);
	MEMControl Control(opCode, funcCode, memWrite, regWrite, writeFlag);

	initial begin
		immExtend = 0;
		ALUResult = 0;
		readData1 = 0;
		readData2 = 0;
		memRead = 0;
	end

	always @(posedge clk) begin
		immExtend = inImmExtend;
		ALUResult = inALUResult;
		readData1 = inReadData1;
		readData2 = inReadData2;

		if(writeFlag == 1) begin
			memRead = inMemRead;
		end
		else begin
			memRead = 0;
		end
	end
endmodule

module WBPipeline(clk, inPC, inNumInst, inOpCode, inFuncCode, inRS1, inRS2, inRD, inImmExtend, inALUResult, inReadData, inReadData1, inWriteFlag,
					dataSelector, regWrite, isHalt, isWWD, PC, numInst, opCode, funcCode, rs1, rs2, rd, immExtend, ALUResult, readData, readData1, writeFlag);
	input clk;
	input [3:0] inOpCode;
	input [5:0] inFuncCode;
	input [1:0] inRS1;
	input [1:0] inRS2;
	input [2:0] inRD;
	input [`WORD_SIZE-1:0] inImmExtend;
	input [`WORD_SIZE-1:0] inALUResult;
	input [`WORD_SIZE-1:0] inPC;
	input inNumInst;
	input [`WORD_SIZE-1:0] inReadData;
	input [`WORD_SIZE-1:0] inReadData1;
	input inWriteFlag;

	output [1:0] dataSelector;
	output regWrite;
	output isHalt;
	output isWWD;
	output [3:0] opCode;
	output [5:0] funcCode;
	output [1:0] rs1;
	output [1:0] rs2;
	output [2:0] rd;
	output [`WORD_SIZE-1:0] immExtend;
	output [`WORD_SIZE-1:0] ALUResult;
	output [`WORD_SIZE-1:0] PC;
	output numInst;
	output [`WORD_SIZE-1:0] readData;
	output [`WORD_SIZE-1:0] readData1;
	output writeFlag;

	reg [`WORD_SIZE-1:0] immExtend;
	reg [`WORD_SIZE-1:0] readData;
	reg [`WORD_SIZE-1:0] readData1;
	reg [`WORD_SIZE-1:0] ALUResult;

	GeneralPipeline WBGeneralPipeline(clk, inPC, inNumInst, inOpCode, inFuncCode, inRS1, inRS2, inRD, inWriteFlag, PC, numInst, opCode, funcCode, rs1, rs2, rd, writeFlag);
	WBControl Control(opCode, funcCode, dataSelector, regWrite, isHalt, isWWD, writeFlag);

	initial begin
		immExtend = 0;
		ALUResult = 0;
		readData = 0;
		readData1 = 0;
	end

	always @(posedge clk) begin
		immExtend = inImmExtend;
		ALUResult = inALUResult;
		readData = inReadData;
		readData1 = inReadData1;
	end
endmodule

module HazardUnit(reset_n, rs1ID, rs2ID, rdEX, memRead, PCWrite, IDWrite, IDFlushFlag, EXWrite, PCCtrl);
	input [1:0] rs1ID;
	input [1:0] rs2ID;
	input [2:0] rdEX;
	input memRead;
	input reset_n;
	input [1:0] PCCtrl;

	output PCWrite;
	output IDWrite;
	output IDFlushFlag;
	output EXWrite;

	reg PCWrite;
	reg IDWrite;
	reg IDFlushFlag;
	reg EXWrite;

	initial begin
		PCWrite = 0;
		IDWrite = 0;
		IDFlushFlag = 0;
		EXWrite = 0;
	end

	always @(reset_n or rs1ID or rs2ID or rdEX or memRead or PCCtrl) begin
		if(reset_n == 0) begin
			PCWrite = 0;
			IDWrite = 0;
			IDFlushFlag = 1;
			EXWrite = 0;
		end
		else if(PCCtrl != 0) begin
			PCWrite = 1;
			IDWrite = 0;
			IDFlushFlag = 1;
			EXWrite = 0;
		end
		else if(memRead == 1 && rdEX != 3 && (rs1ID == rdEX || rs2ID == rdEX)) begin
			PCWrite = 0;
			IDWrite = 0;
			IDFlushFlag = 0;
			EXWrite = 0;
		end
		else begin
			PCWrite = 1;
			IDWrite = 1;
			IDFlushFlag = 0;
			EXWrite = 1;
		end
	end
endmodule

module ForwardUnit(rs1EX, rs2EX, rdMEM, rdWB, opCode, funcCode, RegWriteMEM, RegWriteWB, forwardA, forwardB);
	input [1:0] rs1EX;
	input [1:0] rs2EX;
	input [2:0] rdMEM;
	input [2:0] rdWB;
	input [3:0] opCode;
	input [5:0] funcCode;
	input RegWriteMEM;
	input RegWriteWB;

	output [1:0] forwardA;
	output [1:0] forwardB;

	reg [1:0] forwardA;
	reg [1:0] forwardB;

	wire use_rs1;
	wire use_rs2;

	assign use_rs1 = (opCode == `ALU_OP) || (opCode == `BNE_OP) || (opCode == `BEQ_OP) || (opCode == `ADI_OP) ||
	 			(opCode == `ORI_OP) || (opCode == `LWD_OP) || (opCode == `SWD_OP) || (opCode == `BGZ_OP) || (opCode == `BLZ_OP);
	assign use_rs2 = ((opCode == `ALU_OP) && (funcCode == `INST_FUNC_ADD || funcCode == `INST_FUNC_SUB || funcCode == `INST_FUNC_AND || funcCode == `INST_FUNC_ADD || funcCode == `INST_FUNC_ORR)) ||
				(opCode == `BNE_OP) || (opCode == `BEQ_OP);

	initial begin
		forwardA = 0;
		forwardB = 0;
	end

	always @(rs1EX or rs2EX or rdMEM or rdWB or RegWriteMEM or RegWriteWB) begin
		if((rs1EX == rdMEM) && RegWriteMEM == 1 && use_rs1)
			forwardA = 2;
		else if((rs1EX == rdWB) && RegWriteWB == 1 && use_rs1)
			forwardA = 1;
		else
			forwardA = 0;

		if((rs2EX == rdMEM) && RegWriteMEM == 1 && use_rs2)
			forwardB = 1;
		else if((rs2EX == rdWB) && RegWriteWB == 1 && use_rs2)
			forwardB = 2;
		else
			forwardB = 0;
	end
endmodule

module cpu(clk, reset_n, readM1, address1, data1, readM2, writeM2, address2, data2, num_inst, output_port, is_halted);
	input clk;
	wire clk;
	input reset_n;
	wire reset_n;

	output readM1;
	wire readM1;
	output [`WORD_SIZE-1:0] address1;
	wire [`WORD_SIZE-1:0] address1;
	output readM2;
	wire readM2;
	output writeM2;
	wire writeM2;
	output [`WORD_SIZE-1:0] address2;
	wire [`WORD_SIZE-1:0] address2;

	input [`WORD_SIZE-1:0] data1;
	wire [`WORD_SIZE-1:0] data1;
	inout [`WORD_SIZE-1:0] data2;
	wire [`WORD_SIZE-1:0] data2;

	output [`WORD_SIZE-1:0] num_inst;
	reg [`WORD_SIZE-1:0] num_inst;
	output [`WORD_SIZE-1:0] output_port;
	reg [`WORD_SIZE-1:0] output_port;
	output is_halted;
	wire is_halted;
	// TODO : Implement your pipelined CPU!

	wire [`WORD_SIZE-1:0] IF_writePC, IF_outputPC, IF_instruction, IF_PC;
	wire [`WORD_SIZE-1:0] ID_instruction, ID_PC, ID_immediate, ID_readData1, ID_readData2;
	wire [`WORD_SIZE-1:0] EX_PC, EX_readData1, EX_readData2, EX_immediate, EX_immExtend, EX_ALUSrc2, EX_ALUResult, EX_ALUInput1, EX_ALUInput2;
	wire [`WORD_SIZE-1:0] MEM_PC, MEM_immExtend, MEM_immediate, MEM_ALUResult, MEM_readData, MEM_readData1, MEM_readData2;
	wire [`WORD_SIZE-1:0] WB_PC, WB_immExtend, WB_immediate, WB_writeData, WB_ALUResult, WB_readData, WB_readData1;

	wire IF_PCWrite;
	wire ID_IDWrite, ID_FlushFlag, ID_numInst, ID_EXWrite;
	wire EX_WriteFlag, EX_ALUSrc, EX_bCond, EX_numInst, EX_memRead;
	wire MEM_memRead, MEM_memWrite, MEM_regWrite, MEM_numInst, MEM_WriteFlag;
	wire WB_regWrite, WB_isHalt, WB_isWWD, WB_numInst, WB_WriteFlag;

	wire [1:0] ID_rs1, ID_rs2, ID_rdSelector;
	wire [1:0] EX_rs1, EX_rs2, EX_forwardA, EX_forwardB, EX_PCCtrl;
	wire [1:0] MEM_rs1, MEM_rs2;
	wire [1:0] WB_rs1, WB_rs2, WB_dataSelector;
	wire [1:0] PCCtrl;

	wire [2:0] ID_rd, EX_rd, MEM_rd, WB_rd;

	wire [3:0] ID_opCode, EX_opCode, EX_ALUOp, MEM_opCode, WB_opCode;
	wire [5:0] ID_funcCode, EX_funcCode, MEM_funcCode, WB_funcCode;

	assign readM1 = reset_n;
	assign readM2 = MEM_memRead;
	assign writeM2 = MEM_memWrite;
	assign address1 = IF_outputPC;
	assign address2 = MEM_ALUResult;
	assign data2 = MEM_memWrite ? MEM_readData2 : `WORD_SIZE'bz;
	assign IF_instruction = data1;
	assign MEM_readData = data2;
	assign is_halted = WB_isHalt;
	assign PCCtrl = (EX_bCond ? EX_PCCtrl : 
					(EX_PCCtrl == 2'b01 ? 2'b00 : EX_PCCtrl));

	// Extra
	HazardUnit hazardUnit(reset_n, ID_rs1, ID_rs2, EX_rd, EX_memRead, IF_PCWrite, ID_IDWrite, ID_FlushFlag, ID_EXWrite, PCCtrl);
	ForwardUnit forwardUnit(EX_rs1, EX_rs2, MEM_rd, WB_rd, EX_opCode, EX_funcCode, MEM_regWrite, WB_regWrite, EX_forwardA, EX_forwardB);

	// IF stage
	MUX4to1 PCSelector(IF_outputPC + 1'd1, EX_PC + 1'd1 + EX_immediate, EX_immExtend, EX_ALUInput1, PCCtrl, IF_writePC);
	PC pc(IF_writePC, IF_outputPC, clk, reset_n, IF_PCWrite);
	// instruction memeory access

	// ID stage
	assign ID_rs1 = ID_instruction[11:10];
	assign ID_rs2 = ID_instruction[9:8];
	assign ID_opCode = ID_instruction[15:12];
	assign ID_funcCode = ID_instruction[5:0];
	assign ID_numInst = (ID_EXWrite && reset_n);
	IDPipeline Pipeline_ID(clk, IF_instruction, IF_outputPC, ID_instruction, ID_PC, ID_rdSelector, ID_IDWrite, ID_FlushFlag);
	MUXPC MUX_ID_rd(ID_instruction[7:6] + 3'b000, ID_instruction[9:8] + 3'b000, 3'b10, ID_rdSelector, ID_rd);
	
	Register register(clk, ID_rs1, ID_rs2, WB_rd, WB_writeData, ID_readData1, ID_readData2, WB_regWrite, reset_n);
	ImmediateGenerator immGen(ID_instruction, ID_immediate);

	// EX stage
	EXPipeline Pipeline_EX(clk, ID_PC, ID_numInst, ID_opCode, ID_funcCode, ID_rs1, ID_rs2, ID_rd, ID_immediate, ID_readData1, ID_readData2, (ID_EXWrite && reset_n), 
							EX_memRead, EX_ALUSrc, EX_PC, EX_numInst, EX_opCode, EX_funcCode, EX_rs1, EX_rs2, EX_rd, EX_immediate, EX_readData1, EX_readData2, EX_PCCtrl, EX_WriteFlag);
	ImmExtender immExtender(EX_PC, EX_immediate, EX_immExtend);
	MUX2to1 MUX_EX_ALUSrc2(EX_readData2, EX_immediate, EX_ALUSrc, EX_ALUSrc2);
	MUX4to1 MUX_EX_ALUInput1(EX_readData1, WB_writeData, MEM_ALUResult, `WORD_SIZE'b0, EX_forwardA, EX_ALUInput1);
	MUX4to1 MUX_EX_ALUInput2(EX_ALUSrc2, MEM_ALUResult, WB_writeData, `WORD_SIZE'b0, EX_forwardB, EX_ALUInput2);
	ALUControl aluContol(EX_opCode, EX_funcCode, EX_ALUOp);
	ALU alu(EX_ALUInput1, EX_ALUInput2, EX_ALUOp, EX_ALUResult, EX_bCond);

	// MEM stage
	MEMPipeline Pipeline_MEM(clk, EX_PC, (PCCtrl != 0 ? 1'b0 : EX_numInst), EX_opCode, EX_funcCode, EX_rs1, EX_rs2, EX_rd, EX_immediate, EX_ALUResult, EX_ALUInput1, EX_readData2, EX_memRead, EX_WriteFlag,
							MEM_memRead, MEM_memWrite, MEM_regWrite, MEM_PC, MEM_numInst, MEM_opCode, MEM_funcCode, MEM_rs1, MEM_rs2, MEM_rd, MEM_immediate, MEM_ALUResult, MEM_readData1, MEM_readData2, MEM_WriteFlag);

	// WB stage
	WBPipeline Pipeline_WB(clk, MEM_PC, MEM_numInst, MEM_opCode, MEM_funcCode, MEM_rs1, MEM_rs2, MEM_rd, MEM_immediate, MEM_ALUResult, MEM_readData, MEM_readData1, MEM_WriteFlag,
							WB_dataSelector, WB_regWrite, WB_isHalt, WB_isWWD, WB_PC, WB_numInst, WB_opCode, WB_funcCode, WB_rs1, WB_rs2, WB_rd, WB_immediate, WB_ALUResult, WB_readData, WB_readData1, WB_WriteFlag);
	MUX4to1 MUX_WB_writeData(WB_ALUResult, WB_readData, WB_PC + `WORD_SIZE'b1, WB_immediate, WB_dataSelector, WB_writeData);

	initial begin
		num_inst = -1;
		output_port = 0;
	end

	always @(posedge reset_n) begin
		num_inst = -1;
		output_port = 0;
	end

	always @(posedge clk) begin
		num_inst = num_inst + WB_numInst;
	end

	always @(posedge WB_isWWD) begin
		output_port = WB_readData1;
	end

endmodule
