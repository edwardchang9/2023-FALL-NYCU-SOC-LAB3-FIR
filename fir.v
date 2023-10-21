`timescale 1ns / 1ps
module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    // axi lite
	// ap_start, ap_done, ap_idle
	// coef
	// len
    output  reg                      awready,
    output  reg                      wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    output  reg                      arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  reg                      rvalid,
    output  reg  [(pDATA_WIDTH-1):0] rdata, 
    
    // axi stream
	// data_in, data_out
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  reg                      ss_tready, 
    input   wire                     sm_tready, 
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  reg  [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);
begin
	// main FSM
	reg [2:0] current_state;
	reg [2:0] next_state;
	// parameter declaration
	parameter S_IDLE = 'd0;
	// input first 11 data
	parameter S_INIT_SRAM = 'd1;
	// get stream input data
	parameter S_GET_DATA = 'd2;
	// calculate multiply, add and shift sram simultaneously
	parameter S_CAL = 'd3;
	// shift the pointer and stream output result
	parameter S_SHIFT = 'd4;
	parameter S_WAIT = 'd5;
	// reset all counter
	parameter S_WAIT_AP_DONE = 'd6;
	
	reg [31:0] pattern_count;
	// length
	reg [31:0] length;
	// ap_ctrl register
	reg ap_idle; 
	reg ap_done;
	reg ap_start;
	// ap_idle: r/w
	// set to 1 when reset
	// set to 0 when ao_start is sampled
	// set to 1 when process completed and last data is transfered
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			ap_idle <= 1;
		end else if (ap_start) begin
			ap_idle <= 0;
		end else if(current_state == S_WAIT_AP_DONE) begin
			ap_idle <= 1;
		end else begin
			ap_idle <= ap_idle;
		end
	end
	// ap_done: r/w
	// set to 0 when reset
	// set to 0 when ap_done is read
	// set to 1 when process completed and last data is transfered
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			ap_done <= 0;
		end else if(current_state == S_WAIT_AP_DONE)begin
			ap_done <= 1;
		end else if(current_state == S_IDLE)begin
			ap_done <= 0;
		end else begin
			ap_done <= ap_done;
		end
	end
	
	// ap_start: read only
	// set to 1 by testbench means start proggram
	// set to 0 when 1 is observed
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			ap_start <= 0;
		end else begin
			if(awvalid && wvalid && awaddr == 12'h00)
				ap_start <= wdata[0];
			else if(ss_tready && ss_tvalid)
				ap_start <= 0;
			else
				ap_start <= ap_start;
		end
	end
	
	// length parameter
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			length <= 0;
		end else begin
			if(awvalid && wvalid && awaddr == 12'h10)
				length <= wdata;
			else
				length <= length;
		end
	end
	
	//------------------------------------------------------------AXI Lite Read--------------------------------------------------------
    // read channel
	parameter AXI_READ_IDLE = 2'd0;
	parameter AXI_READ_REG = 2'd1;
	parameter AXI_READ_SRAM = 2'd2;
	parameter AXI_READ_OUT = 2'd3;
	
	reg [1:0] axi_current_state;
	reg [1:0] axi_next_state;
	
	reg [(pADDR_WIDTH-1):0] save_araddr;
	
	// save araddr
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			save_araddr <= 0;
		end else begin
			if(arvalid)
				save_araddr <= araddr;
			else
				save_araddr <= save_araddr;
		end
	end
	
	// FSM
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			axi_current_state <= AXI_READ_IDLE;
		end else begin
			axi_current_state <= axi_next_state;
		end
	end
	
	always@(*) begin
		case(axi_current_state)
		AXI_READ_IDLE:
			if(arvalid)
				if(araddr >= 12'h020)
					axi_next_state = AXI_READ_SRAM;
				else
					axi_next_state = AXI_READ_OUT;
			else
				axi_next_state = AXI_READ_IDLE;
		AXI_READ_OUT:
			if(rready)
				axi_next_state = AXI_READ_IDLE;
			else
				axi_next_state = AXI_READ_OUT;
		AXI_READ_SRAM:
			axi_next_state = AXI_READ_OUT;
		endcase
	end
	
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			arready <= 1;
		end else begin
			if(axi_current_state == AXI_READ_IDLE)
				arready <= 1;
			else
				arready <= 0;
		end
	end
	
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			rvalid <= 0;
		end else begin
			if (axi_current_state == AXI_READ_OUT)
				rvalid <= 1;
			else
				rvalid <= 0;
		end
	end
	
	reg [(pDATA_WIDTH-1):0] next_rdata;
	always@(*) begin
		case(axi_current_state)
		AXI_READ_IDLE: next_rdata = 0;
		AXI_READ_SRAM: next_rdata = 0;
		AXI_READ_OUT:
			if(save_araddr == 12'h00)
				next_rdata = {29'd0,ap_idle,ap_done,ap_start};
			else if(save_araddr == 12'h10)
				next_rdata = length;
			else
				next_rdata = tap_Do;
		default: next_rdata = 32'hFFFF;
		endcase
	end
	
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			rdata <= 0;
		end else begin
			rdata <= next_rdata;
		end
	end
	
	//---------------------------------------------------------------Main Process FSM-----------------------------------------------------------

	// counter
	reg [3:0] counter;
	reg [3:0] next_counter;
	// shift sram pointer
	// current data pointer
	reg [(pADDR_WIDTH-1):0] data_address_pointer;
	reg [(pADDR_WIDTH-1):0] data_address_saver;
	// save the first position pointer
	reg [(pADDR_WIDTH-1):0] next_data_address_pointer;
	reg [(pADDR_WIDTH-1):0] next_data_address_saver;
	// sram valid flag
	reg sram_valid_flag;
	
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			sram_valid_flag <= 0;
		end else if(current_state == S_CAL) begin
			sram_valid_flag <= 1;
		end else begin
			sram_valid_flag <= 0;
		end
	end
	
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			counter <= 0;
		end else begin
			counter <= next_counter;
		end
	end
	
	always@(*) begin
		if(current_state == S_INIT_SRAM || current_state == S_CAL)
			next_counter = counter + 1;
		else
			next_counter = 0;
	end
	
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			pattern_count <= 0;
		end else if(sm_tready && sm_tvalid) begin
			pattern_count <= pattern_count + 1;
		end else if(current_state == S_WAIT_AP_DONE) begin
			pattern_count <= 0;
		end else begin
			pattern_count <= pattern_count;
		end
	end
	
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			data_address_pointer <= 0;
		end else begin
			data_address_pointer <= next_data_address_pointer;
		end
	end
	
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			data_address_saver <= 0;
		end else begin
			data_address_saver <= next_data_address_saver;
		end
	end
	
	always@(*) begin
	case(current_state)
	S_IDLE: next_data_address_pointer = 0;
	S_INIT_SRAM:
		if(counter == 'd10)
			next_data_address_pointer = data_address_saver;
		else
			next_data_address_pointer = data_address_pointer + 4;
	S_CAL: 
		if(data_address_pointer == 'd40)
			next_data_address_pointer = 0;
		else
			next_data_address_pointer = data_address_pointer + 4;
	S_SHIFT: next_data_address_pointer = next_data_address_saver;
	default: next_data_address_pointer = data_address_pointer;
	endcase
	end
	
	always@(*) begin
	case(current_state)
	S_INIT_SRAM: next_data_address_saver = 0;
	S_SHIFT:
		if(data_address_saver == 'd0)
			next_data_address_saver = 'd40;
		else
			next_data_address_saver = data_address_saver - 4;
	default: next_data_address_saver = data_address_saver;
	endcase
	end
	
	
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			current_state <= S_IDLE;
		end else begin
			current_state <= next_state;
		end
	end
	
	always@(*) begin
	case(current_state)
	S_IDLE:
		if(ap_start)
			next_state = S_INIT_SRAM;
		else
			next_state = S_IDLE;
	S_INIT_SRAM:
		if(counter == 'd10)
			next_state = S_GET_DATA;
		else
			next_state = S_INIT_SRAM;
	S_GET_DATA: 
		if(ss_tvalid)
			next_state = S_CAL;
		else
			next_state = S_GET_DATA;
	S_CAL: 
		if(counter == 'd10)
			next_state = S_SHIFT;
		else
			next_state = S_CAL;
	S_SHIFT: 
		if(sm_tready)
			if(pattern_count == length-1)
				next_state = S_WAIT_AP_DONE;
			else
				next_state = S_GET_DATA;
		else
			next_state = S_WAIT;
	S_WAIT: if(sm_tready)
				if(pattern_count == length-1)
					next_state = S_WAIT_AP_DONE;
				else
					next_state = S_GET_DATA;
			else
				next_state = S_WAIT;
	S_WAIT_AP_DONE:
		if(rvalid && rdata[1])
			next_state = S_IDLE;
		else
			next_state = S_WAIT_AP_DONE;
	default: next_state = S_IDLE;
	endcase
	end
	
	reg signed [(pDATA_WIDTH-1):0] cal_result;
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			cal_result <= 0;
		end else if(sram_valid_flag) begin
			cal_result <= ( $signed(data_Do) * $signed(tap_Do) ) + $signed(cal_result);
		end else if(current_state == S_GET_DATA) begin
			cal_result <= 0;
		end else begin
			cal_result <= cal_result;
		end
	end
	
	//------------------------------------------------------------AXI Lite Write--------------------------------------------------------
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			awready <= 0;
		end else begin
			awready <= (wvalid && awvalid);
		end
	end
	always@(posedge axis_clk) begin
		if(!axis_rst_n) begin
			wready <= 0;
		end else begin
			wready <= (wvalid && awvalid);
		end
	end

	//---------------------------------------------------------------Tap SRAM-----------------------------------------------------------
	assign tap_WE = {4{(awvalid && wvalid && awaddr >= 12'h20)}};
	assign tap_EN = 1;
	assign tap_Di = wdata;
    always@(*) begin
		if(awvalid && wvalid && awaddr >= 12'h20)
			tap_A = awaddr - 12'h20;
		else if(axi_current_state == AXI_READ_SRAM)
			tap_A = save_araddr - 12'h20;
		else if(current_state == S_CAL) begin
			tap_A = counter << 2;
		end else
			tap_A = 0;
	end
		
	//---------------------------------------------------------------Data SRAM-----------------------------------------------------------
	assign data_WE = {4{current_state == S_GET_DATA || current_state == S_INIT_SRAM}};
	assign data_EN = 1;
	assign data_Di= (current_state == S_INIT_SRAM) ? 0 : ss_tdata;
	assign data_A = data_address_pointer;
	
	//---------------------------------------------------------------AXI stream-----------------------------------------------------------
	always@ (posedge axis_clk) begin
		ss_tready <= current_state == S_GET_DATA;
	end
	
	assign sm_tvalid = current_state == S_SHIFT;
	assign sm_tlast = current_state == S_SHIFT && pattern_count == length-1;
	assign sm_tdata = cal_result;
	
end
endmodule

