`timescale 1ns/1ns
`define PERIOD1 100
`define WORD_SIZE 16

`define NUM_TEST 56
`define TESTID_SIZE 5

module cpu_TB();
	reg reset_n;    // active-low RESET signal
	reg clk;        // clock signal	
	
	wire readM1;
	wire [`WORD_SIZE-1:0] address1;
	wire [`WORD_SIZE-1:0] data1;
	wire readM2;
	wire writeM2;
	wire [`WORD_SIZE-1:0] address2;
	wire [`WORD_SIZE-1:0] data2;

	// for debuging purpose
	wire [`WORD_SIZE-1:0] num_inst;		// number of instruction during execution
	wire [`WORD_SIZE-1:0] output_port;	// this will be used for a "WWD" instruction
	wire is_halted;				// set if the cpu is halted

	// instantiate the unit under test
	cpu UUT (clk, reset_n, readM1, address1, data1, readM2, writeM2, address2, data2, num_inst, output_port, is_halted);
	Memory NUUT(!clk, reset_n, readM1, address1, data1, readM2, writeM2, address2, data2);

	// initialize inputs
	initial begin
		clk = 0;           // set initial clock value	
		
		reset_n = 1;       // generate a LOW pulse for reset_n
		#(`PERIOD1/4) reset_n = 0;
		#(`PERIOD1 + `PERIOD1/2) reset_n = 1;
	end

	// generate the clock
	always #(`PERIOD1/2)clk = ~clk;  // generates a clock (period = `PERIOD1)
		
	event testbench_finish;	// This event will finish the testbench.
	initial #(`PERIOD1*10000) -> testbench_finish; // Only 10,000 cycles are allowed.
		
	reg [`TESTID_SIZE*8-1:0] TestID[`NUM_TEST-1:0];
	reg [`WORD_SIZE-1:0] TestNumInst [`NUM_TEST-1:0];
	reg [`WORD_SIZE-1:0] TestAns[`NUM_TEST-1:0];
	reg TestPassed[`NUM_TEST-1:0];	
	initial begin
		TestID[0] <= "1-1";		TestNumInst[0] <= 16'h0003;	TestAns[0] <= 16'h0000;		TestPassed[0] <= 1'bx;
		TestID[1] <= "1-2";		TestNumInst[1] <= 16'h0005;	TestAns[1] <= 16'h0000;		TestPassed[1] <= 1'bx;
		TestID[2] <= "1-3";		TestNumInst[2] <= 16'h0007;	TestAns[2] <= 16'h0000;		TestPassed[2] <= 1'bx;
		TestID[3] <= "1-4";		TestNumInst[3] <= 16'h0009;	TestAns[3] <= 16'h0000;		TestPassed[3] <= 1'bx;
		TestID[4] <= "2-1";		TestNumInst[4] <= 16'h000b;	TestAns[4] <= 16'h0001;		TestPassed[4] <= 1'bx;
		TestID[5] <= "2-2";		TestNumInst[5] <= 16'h000d;	TestAns[5] <= 16'h0002;		TestPassed[5] <= 1'bx;
		TestID[6] <= "3-1";		TestNumInst[6] <= 16'h000f;		TestAns[6] <= 16'h0001;		TestPassed[6] <= 1'bx;
		TestID[7] <= "3-2";		TestNumInst[7] <= 16'h0011;	TestAns[7] <= 16'h0003;		TestPassed[7] <= 1'bx;
		TestID[8] <= "3-3";		TestNumInst[8] <= 16'h0013;	TestAns[8] <= 16'h0003;		TestPassed[8] <= 1'bx;
		TestID[9] <= "4-1";		TestNumInst[9] <= 16'h0015;	TestAns[9] <= 16'h0002;		TestPassed[9] <= 1'bx;
		TestID[10] <= "4-2";		TestNumInst[10] <= 16'h0017;	TestAns[10] <= 16'h0003;		TestPassed[10] <= 1'bx;
		TestID[11] <= "4-3";		TestNumInst[11] <= 16'h0019;	TestAns[11] <= 16'h0005;		TestPassed[11] <= 1'bx;
		TestID[12] <= "5-1";		TestNumInst[12] <= 16'h001b;	TestAns[12] <= 16'h0002;		TestPassed[12] <= 1'bx;
		TestID[13] <= "5-2";		TestNumInst[13] <= 16'h001d;	TestAns[13] <= 16'hFFFE;	TestPassed[13] <= 1'bx;
		TestID[14] <= "5-3";		TestNumInst[14] <= 16'h001f;	TestAns[14] <= 16'h0003;		TestPassed[14] <= 1'bx;
		TestID[15] <= "5-4";		TestNumInst[15] <= 16'h0021;	TestAns[15] <= 16'hFFFD;	TestPassed[15] <= 1'bx;
		TestID[16] <= "5-5";		TestNumInst[16] <= 16'h0023;	TestAns[16] <= 16'hFFFF;		TestPassed[16] <= 1'bx;
		TestID[17] <= "5-6";		TestNumInst[17] <= 16'h0025;	TestAns[17] <= 16'h0001;		TestPassed[17] <= 1'bx;
		TestID[18] <= "6-1";		TestNumInst[18] <= 16'h0027;	TestAns[18] <= 16'h0000;		TestPassed[18] <= 1'bx;
		TestID[19] <= "6-2";		TestNumInst[19] <= 16'h0029;	TestAns[19] <= 16'h0000;		TestPassed[19] <= 1'bx;
		TestID[20] <= "6-3";		TestNumInst[20] <= 16'h002b;	TestAns[20] <= 16'h0002;		TestPassed[20] <= 1'bx;
		TestID[21] <= "7-1";		TestNumInst[21] <= 16'h002d;	TestAns[21] <= 16'h0002;		TestPassed[21] <= 1'bx;
		TestID[22] <= "7-2";		TestNumInst[22] <= 16'h002f;	TestAns[22] <= 16'h0003;		TestPassed[22] <= 1'bx;
		TestID[23] <= "7-3";		TestNumInst[23] <= 16'h0031;	TestAns[23] <= 16'h0003;		TestPassed[23] <= 1'bx;
		TestID[24] <= "8-1";		TestNumInst[24] <= 16'h0033;	TestAns[24] <= 16'hFFFD;	TestPassed[24] <= 1'bx;
		TestID[25] <= "8-2";		TestNumInst[25] <= 16'h0035;	TestAns[25] <= 16'hFFFC;	TestPassed[25] <= 1'bx;
		TestID[26] <= "8-3";		TestNumInst[26] <= 16'h0037;	TestAns[26] <= 16'hFFFF;		TestPassed[26] <= 1'bx;
		TestID[27] <= "9-1";		TestNumInst[27] <= 16'h0039;	TestAns[27] <= 16'hFFFE;	TestPassed[27] <= 1'bx;
		TestID[28] <= "9-2";		TestNumInst[28] <= 16'h003b;	TestAns[28] <= 16'hFFFD;	TestPassed[28] <= 1'bx;
		TestID[29] <= "9-3";		TestNumInst[29] <= 16'h003d;	TestAns[29] <= 16'h0000;		TestPassed[29] <= 1'bx;
		TestID[30] <= "10-1";	TestNumInst[30] <= 16'h003f;	TestAns[30] <= 16'h0004;		TestPassed[30] <= 1'bx;
		TestID[31] <= "10-2";	TestNumInst[31] <= 16'h0041;	TestAns[31] <= 16'h0006;		TestPassed[31] <= 1'bx;
		TestID[32] <= "10-3";	TestNumInst[32] <= 16'h0043;	TestAns[32] <= 16'h0000;		TestPassed[32] <= 1'bx;
		TestID[33] <= "11-1";	TestNumInst[33] <= 16'h0045;	TestAns[33] <= 16'h0001;		TestPassed[33] <= 1'bx;
		TestID[34] <= "11-2";	TestNumInst[34] <= 16'h0047;	TestAns[34] <= 16'h0001;		TestPassed[34] <= 1'bx;
		TestID[35] <= "11-3";	TestNumInst[35] <= 16'h0049;	TestAns[35] <= 16'h0000;		TestPassed[35] <= 1'bx;
		TestID[36] <= "12-1";	TestNumInst[36] <= 16'h004b;	TestAns[36] <= 16'h0001;		TestPassed[36] <= 1'bx;
		TestID[37] <= "12-2";	TestNumInst[37] <= 16'h004d;	TestAns[37] <= 16'hFFFF;		TestPassed[37] <= 1'bx;
		TestID[38] <= "13-1";	TestNumInst[38] <= 16'h0051;	TestAns[38] <= 16'hFFFF;		TestPassed[38] <= 1'bx;
		TestID[39] <= "13-2";	TestNumInst[39] <= 16'h0053;	TestAns[39] <= 16'h0001;		TestPassed[39] <= 1'bx;
		TestID[40] <= "14-1";	TestNumInst[40] <= 16'h0055;	TestAns[40] <= 16'hFFFF;		TestPassed[40] <= 1'bx;
		TestID[41] <= "14-2";	TestNumInst[41] <= 16'h0057;	TestAns[41] <= 16'h0001;		TestPassed[41] <= 1'bx;
		TestID[42] <= "15-1";	TestNumInst[42] <= 16'h005a;	TestAns[42] <= 16'hFFFF;		TestPassed[42] <= 1'bx;
		TestID[43] <= "15-2";	TestNumInst[43] <= 16'h005c;	TestAns[43] <= 16'h0001;		TestPassed[43] <= 1'bx;
		TestID[44] <= "16-1";	TestNumInst[44] <= 16'h005f;	TestAns[44] <= 16'hFFFF;		TestPassed[44] <= 1'bx;
		TestID[45] <= "16-2";	TestNumInst[45] <= 16'h0061;	TestAns[45] <= 16'h0001;		TestPassed[45] <= 1'bx;
		TestID[46] <= "17-1";	TestNumInst[46] <= 16'h0064;	TestAns[46] <= 16'hFFFF;		TestPassed[46] <= 1'bx;
		TestID[47] <= "17-2";	TestNumInst[47] <= 16'h0066;	TestAns[47] <= 16'h0001;		TestPassed[47] <= 1'bx;
		TestID[48] <= "17-3";	TestNumInst[48] <= 16'h0069;	TestAns[48] <= 16'hFFFF;		TestPassed[48] <= 1'bx;
		TestID[49] <= "18-1";	TestNumInst[49] <= 16'h006b;	TestAns[49] <= 16'h0001;		TestPassed[49] <= 1'bx;
		TestID[50] <= "18-2";	TestNumInst[50] <= 16'h006e;	TestAns[50] <= 16'hFFFF;		TestPassed[50] <= 1'bx;
		TestID[51] <= "18-3";	TestNumInst[51] <= 16'h0071;	TestAns[51] <= 16'h0001;		TestPassed[51] <= 1'bx;
		TestID[52] <= "19-1";	TestNumInst[52] <= 16'h0074;	TestAns[52] <= 16'hFFFF;		TestPassed[52] <= 1'bx;
		TestID[53] <= "19-2";	TestNumInst[53] <= 16'h0078;	TestAns[53] <= 16'h0001;		TestPassed[53] <= 1'bx;
		TestID[54] <= "19-3";	TestNumInst[54] <= 16'h0116;	TestAns[54] <= 16'h0008;		TestPassed[54] <= 1'bx;
		TestID[55] <= "20";		TestNumInst[55] <= 16'h03d5;	TestAns[55] <= 16'h0022;		TestPassed[55] <= 1'bx;
	end		   
	
	reg [`WORD_SIZE-1:0] i;	
	reg [`WORD_SIZE-1:0] num_clock;
		
	always @ (posedge clk) begin 
		if (reset_n == 1) begin
			num_clock = num_clock+1;
			for(i=0; i<`NUM_TEST; i=i+1) begin
				if (num_inst == TestNumInst[i]) begin
					if (output_port == TestAns[i]) begin
						TestPassed[i] = 1'b1;
					end
					else begin
						TestPassed[i] = 1'b0;
						$display("Test #%s has been failed!", TestID[i]);
						$display("output_port = 0x%0x (Ans : 0x%0x)", output_port, TestAns[i]);
						-> testbench_finish;
					end
				end
			end
			if (is_halted == 1)
				-> testbench_finish;
		end
		else begin
			num_clock = 0;
		end
	end											
	
	reg [`WORD_SIZE-1:0] Passed;
	
	initial begin
		Passed <= 0;
	end
	always @(testbench_finish) begin
		
		$display("Clock #%d", num_clock);
		$display("The testbench is finished. Summarizing...");
		for(i=0; i<`NUM_TEST; i=i+1) begin
			if (TestPassed[i] == 1)
				Passed=Passed+1;
			else									   
				$display("Test #%s : %s", TestID[i], (TestPassed[i] === 0)?"Wrong" : "No Result");
		end
		if (Passed == `NUM_TEST)
			$display("All Pass!");
		else
			$display("Pass : %0d/%0d", Passed, `NUM_TEST);
		$finish;
	end

endmodule