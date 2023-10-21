`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/20/2023 10:38:55 AM
// Design Name: 
// Module Name: fir_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module fir_tb
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11,
    parameter Data_Num    = 600
)();
    wire                        awready;
    wire                        wready;
    reg                         awvalid;
    reg   [(pADDR_WIDTH-1): 0]  awaddr;
    reg                         wvalid;
    reg signed [(pDATA_WIDTH-1) : 0] wdata;
    wire                        arready;
    reg                         rready;
    reg                         arvalid;
    reg         [(pADDR_WIDTH-1): 0] araddr;
    wire                        rvalid;
    wire signed [(pDATA_WIDTH-1): 0] rdata;
    reg                         ss_tvalid;
    reg signed [(pDATA_WIDTH-1) : 0] ss_tdata;
    reg                         ss_tlast;
    wire                        ss_tready;
    reg                         sm_tready;
    wire                        sm_tvalid;
    wire signed [(pDATA_WIDTH-1) : 0] sm_tdata;
    wire                        sm_tlast;
    reg                         axis_clk;
    reg                         axis_rst_n;

// ram for tap
    wire [3:0]               tap_WE;
    wire                     tap_EN;
    wire [(pDATA_WIDTH-1):0] tap_Di;
    wire [(pADDR_WIDTH-1):0] tap_A;
    wire [(pDATA_WIDTH-1):0] tap_Do;

// ram for data RAM
    wire [3:0]               data_WE;
    wire                     data_EN;
    wire [(pDATA_WIDTH-1):0] data_Di;
    wire [(pADDR_WIDTH-1):0] data_A;
    wire [(pDATA_WIDTH-1):0] data_Do;



    fir fir_DUT(
        .awready(awready),
        .wready(wready),
        .awvalid(awvalid),
        .awaddr(awaddr),
        .wvalid(wvalid),
        .wdata(wdata),
        .arready(arready),
        .rready(rready),
        .arvalid(arvalid),
        .araddr(araddr),
        .rvalid(rvalid),
        .rdata(rdata),
        .ss_tvalid(ss_tvalid),
        .ss_tdata(ss_tdata),
        .ss_tlast(ss_tlast),
        .ss_tready(ss_tready),
        .sm_tready(sm_tready),
        .sm_tvalid(sm_tvalid),
        .sm_tdata(sm_tdata),
        .sm_tlast(sm_tlast),

        // ram for tap
        .tap_WE(tap_WE),
        .tap_EN(tap_EN),
        .tap_Di(tap_Di),
        .tap_A(tap_A),
        .tap_Do(tap_Do),

        // ram for data
        .data_WE(data_WE),
        .data_EN(data_EN),
        .data_Di(data_Di),
        .data_A(data_A),
        .data_Do(data_Do),

        .axis_clk(axis_clk),
        .axis_rst_n(axis_rst_n)

        );
    
    // RAM for tap
    bram11 tap_RAM (
        .CLK(axis_clk),
        .WE(tap_WE),
        .EN(tap_EN),
        .Di(tap_Di),
        .A(tap_A),
        .Do(tap_Do)
    );

    // RAM for data: choose bram11 or bram12
    bram11 data_RAM(
        .CLK(axis_clk),
        .WE(data_WE),
        .EN(data_EN),
        .Di(data_Di),
        .A(data_A),
        .Do(data_Do)
    );
	
	integer RUN_TIME = 3;
	integer latency = 0;
	integer save_latency = 0;

    reg signed [(pDATA_WIDTH-1):0] Din_list[0:(Data_Num-1)];
    reg signed [(pDATA_WIDTH-1):0] golden_list[0:(Data_Num-1)];
	
	// dump waveform
    initial begin
        $dumpfile("fir.vcd");
        $dumpvars();
    end
	
	// clock
	initial begin
		axis_clk = 0;
        forever begin
            #5 axis_clk = (~axis_clk);
        end
    end
	
	// Prevent hang
    integer timeout = (100000);
    initial begin
        while(timeout > 0) begin
            @(posedge axis_clk);
            timeout = timeout - 1;
			latency = latency + 1;
        end
        $display($time, "Simualtion Hang ....");
        $finish;
    end
	
	// read input data and golden data file
	reg [31:0]  data_length;
    integer Din, golden, input_data, golden_data, m;
    initial begin
        data_length = 0;
        Din = $fopen("./samples_triangular_wave.dat","r");
        golden = $fopen("./out_gold.dat","r");
        for(m=0;m<Data_Num;m=m+1) begin
            input_data = $fscanf(Din,"%d", Din_list[m]);
            golden_data = $fscanf(golden,"%d", golden_list[m]);
            data_length = data_length + 1;
        end
		$display("Successful Read Golden data file, data length = %3d", data_length);
    end
	
	// fill in coef data
	reg signed [31:0] coef[0:10];
    initial begin
        coef[0]  =  32'd0;
        coef[1]  = -32'd10;
        coef[2]  = -32'd9;
        coef[3]  =  32'd23;
        coef[4]  =  32'd56;
        coef[5]  =  32'd63;
        coef[6]  =  32'd56;
        coef[7]  =  32'd23;
        coef[8]  = -32'd9;
        coef[9]  = -32'd10;
        coef[10] =  32'd0;
    end

	// 1. Check FIR is idle, if not, wait until FIR is idle
	// 2. Program length, and coef parameters
	// 3. Program ap_start -> 1
	// 4. Fork
	//     (1) Transmit Xn
	//     (2) Receive Yn
	//     (3) Polling ap_done
	// 5. When ap_done is sampled, compare Yn with golden data

    reg error;
    reg status_error;
    reg error_coef;
	reg transmit_finish_flag;
	reg receive_finish_flag;
	integer e;
	
    initial begin
		initial_task;
		for (e = 0; e < RUN_TIME;e=e+1) begin
			wait_idle_task;
			program_param_task;
			program_ap_start_task;
			fork
				transmit_xn_task;
				receive_yn_task;
				polling_ap_done_task;
			join
		end
		repeat(10) @(negedge axis_clk);
		$finish;
	end
	
	// reset signal assert
	task initial_task; begin
		$display("Assert reset signal.");
        awvalid = 0;
        awaddr = 'dx;
        wvalid = 0;
        wdata = 'dx;
        rready = 0;
        arvalid = 0;
        araddr = 'dx;
        axis_rst_n = 1;
		ss_tdata = 'dx;
		ss_tvalid = 0;
		ss_tlast = 0;
		sm_tready = 0;
        @(negedge axis_clk);
		axis_rst_n = 0;
		@(negedge axis_clk);
        axis_rst_n = 1;
		$display("Reset completed.");
	end
	endtask
	 
		
	reg wait_flag;
	task wait_idle_task; begin
		$display("Wiating for FIR to be in idle state...");
		wait_flag = 1;
		transmit_finish_flag = 0;
		receive_finish_flag = 0;
        while(wait_flag == 1)
			begin
				arvalid = 0;
				@(negedge axis_clk);
				arvalid = 1;
				araddr = 12'h00;
				rready = 1;
				@(negedge axis_clk);
				while (!arready) @(negedge axis_clk);
				arvalid = 0;
				araddr = 'dx;
				while (!rvalid) @(negedge axis_clk);
				if( rdata[2] === 1 && rdata[1] === 0 && rdata[0] === 0) begin
					$display("FIR in idle, reset complete.");
					wait_flag = 0;
				end else begin
					wait_flag = 1;
				end
			end
		end
    endtask
	
	integer i;
	task program_param_task; begin
		$display("---------Start programming parameters--------");
		write_axi_task(12'h10, data_length);
		for(i=0; i < Tape_Num; i=i+1) begin
            write_axi_task(12'h20+4*i, coef[i]);
        end
		$display("--------Finish programming parameters--------");
		$display("--------Start checking tap parameters--------");
		for(i=0; i < Tape_Num; i=i+1) begin
			read_axi_and_check_task(12'h20+4*i, coef[i], 32'hffffffff);
        end
		$display("--------Finish checking tap parameters--------");
		end
    endtask
	
	task program_ap_start_task; begin
		write_axi_task(12'h0, 32'b101);
		save_latency = latency;
		$display("---------Start executing kernel--------");
	end
	endtask
	
	task write_axi_task;
        input [11:0]    addr;
        input [31:0]    data;
		begin
			@(negedge axis_clk);
			fork
			send_write_requset_task(addr);
			send_write_data_task(data);
			join
		end
    endtask
	
	task send_write_requset_task;
        input [11:0]    addr;
		begin
			awvalid = 1;
			awaddr = addr;
			@(negedge axis_clk);
			while(!awready) @(negedge axis_clk);
			awvalid = 0;
			awaddr = 'dx;
		end
	endtask
	
	task send_write_data_task;
        input [31:0]    data;
		begin
			wvalid = 1;
			wdata = data;
			@(negedge axis_clk);
			while(!wready) @(negedge axis_clk);
			wvalid = 0;
			wdata = 'dx;
		end
	endtask

    task read_axi_and_check_task;
        input [11:0]        addr;
        input signed [31:0] exp_data;
        input [31:0]        mask;
        begin
            arvalid = 0;
			@(negedge axis_clk);
			arvalid = 1;
			araddr = addr;
			rready = 1;
			@(negedge axis_clk);
            while (!arready) @(negedge axis_clk);
			arvalid = 0;
			araddr = 'dx;
			while (!rvalid) @(negedge axis_clk);
            if( (rdata & mask) != (exp_data & mask)) begin
                $display("ERROR: exp = %d, rdata = %d", exp_data, rdata);
                error_coef <= 1;
            end else begin
                $display("OK: exp = %d, rdata = %d", exp_data, rdata);
            end
        end
    endtask
	
	task transmit_single_data_stream;
	input  signed [31:0] in; begin
		ss_tvalid = 1;
		ss_tdata = in;
		@(negedge axis_clk);
		while (!ss_tready) begin
            @(negedge axis_clk);
        end
	end
	endtask
	
	integer num;
	task transmit_xn_task; begin
		for(i=0;i<(data_length-1);i=i+1) begin
            ss_tlast = 0; transmit_single_data_stream(Din_list[i]);
        end
        ss_tlast = 1;
		transmit_single_data_stream(Din_list[(Data_Num-1)]);
		transmit_finish_flag = 1;
		ss_tlast = 0;
		$display("--------Finish transmitting data--------");
	end
	endtask
	
	task recerve_single_data_stream;
		input  signed [31:0] in; // golden data
        input         [31:0] pcnt; // pattern count
		begin
		@(negedge axis_clk)
        while(!sm_tvalid) @(negedge axis_clk);
        if (sm_tdata !== in) begin
            $display("[ERROR] [Pattern %d] Golden answer: %d, Your answer: %d", pcnt, in, sm_tdata);
        end else begin
            $display("[PASS] [Pattern %d] Golden answer: %d, Your answer: %d", pcnt, in, sm_tdata);
        end
	end
	endtask
	
	integer k;
	task receive_yn_task; begin
		error = 0;
		status_error = 0;
        sm_tready = 1;
        wait (sm_tvalid);
        for(k=0;k < data_length;k=k+1) begin
            recerve_single_data_stream(golden_list[k],k);
        end
		receive_finish_flag = 1;
		$display("--------Finish receiving data--------");
	end
	endtask
	
	task polling_ap_done_task; begin
		wait_flag = 1;
        while(wait_flag == 1)
			begin
				arvalid = 0;
				@(negedge axis_clk);
				arvalid = 1;
				araddr = 12'h00;
				rready = 1;
				@(negedge axis_clk);
				while (!arready) @(negedge axis_clk);
				arvalid = 0;
				araddr = 'dx;
				while (!rvalid) @(negedge axis_clk);
				if( rdata[2] === 1 && rdata[1] === 1 && rdata[0] === 0) begin
					$display("AP done is observed, engine finish, latency = %5d", (latency - save_latency));
					if(receive_finish_flag != 1 || transmit_finish_flag != 1) begin
						$display("[ERROR] AP done is observed, but transmit task or receive task not done yet!!!");
						$finish;
					end
					wait_flag = 0;
				end else begin
					wait_flag = 1;
				end
			end
	end
	endtask
	
	
endmodule

