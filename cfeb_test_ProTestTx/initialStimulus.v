`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   12:52:57 06/06/2014
// Design Name:   dcfeb_test
// Module Name:   C:/xilinx_proj/cfeb_test/initialStimulus.v
// Project Name:  cfeb_test
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: dcfeb_test
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module initialStimulus;

	// Inputs
	reg ck125n;
	reg ck125p;
	reg ck160n;
	reg ck160p;
	reg lhc_ckn;
	reg lhc_ckp;
	reg [8:7] sw;
	reg tmb_clock0;
	reg pb;
	reg vme_cmd10;
	reg qpll_lock;
	reg [4:1] cfebemul_in;
	reg [3:0] vstat;
	reg [28:1] alct_rx;
	reg jtag_usr0_tdo;
	reg gp_io4;
	reg rpc_dsn;
	reg rpc_smbrx;
	reg prom_d3;
	reg prom_d7;
	reg jtag_fpga3;
	reg sda0;
	reg tmb_sn;
	reg t_crit;
	reg [50:0] _ccb_rx;
	reg [5:0] dmb_rx;
	reg [15:8] dmb_i1tx;
	reg [31:24] dmb_i2tx;
	reg [43:38] dmb_i3tx;
	reg [9:0] rpc_i1rx;
	reg [37:26] rpc_i2rx;
	reg t12_fault;
	reg r12_fok;
	reg [7:1] rxn;
	reg [7:1] rxp;

	// Outputs
	wire [3:0] sel_usr;
	wire [3:1] jtag_usr;
	wire [17:5] alct_txa;
	wire [23:19] alct_txb;
	wire [7:0] dmb_tx;
	wire [25:10] rpc_orx;
	wire [23:16] dmb_o1tx;
	wire [37:32] dmb_o2tx;
	wire [48:44] dmb_o3tx;
	wire smb_clk;
	wire alct_loop;
	wire alct_txoe;
	wire alct_clock_en;
	wire alct_rxoe;
	wire smb_data;
	wire gtl_loop;
	wire dmb_loop;
	wire rpc_loop;
	wire ccb_status_oe;
	wire _dmb_oe;
	wire [26:0] _ccb_tx;
	wire _hard_reset_tmb_fpga;
	wire _hard_reset_alct_fpga;
	wire rst_qpll;
	wire fcs;
	wire [3:0] rpc_tx;
	wire [6:0] vme_reply;
	wire [4:0] step;
	wire [7:0] led_low;
	wire [15:8] led_hi;
	wire [10:5] test_led;
	wire [7:1] txn;
	wire [7:1] txp;
	wire t12_rst;
	wire t12_sclk;
	wire r12_sclk;

	// Instantiate the Unit Under Test (UUT)
	dcfeb_test uut (
		.ck125n(ck125n), 
		.ck125p(ck125p), 
		.ck160n(ck160n), 
		.ck160p(ck160p), 
		.lhc_ckn(lhc_ckn), 
		.lhc_ckp(lhc_ckp), 
		.sw(sw), 
		.tmb_clock0(tmb_clock0), 
		.pb(pb), 
		.vme_cmd10(vme_cmd10), 
		.qpll_lock(qpll_lock), 
		.cfebemul_in(cfebemul_in), 
		.vstat(vstat), 
		.alct_rx(alct_rx), 
		.jtag_usr0_tdo(jtag_usr0_tdo), 
		.gp_io4(gp_io4), 
		.rpc_dsn(rpc_dsn), 
		.rpc_smbrx(rpc_smbrx), 
		.prom_d3(prom_d3), 
		.prom_d7(prom_d7), 
		.jtag_fpga3(jtag_fpga3), 
		.sda0(sda0), 
		.tmb_sn(tmb_sn), 
		.t_crit(t_crit), 
		._ccb_rx(_ccb_rx), 
		.dmb_rx(dmb_rx), 
		.dmb_i1tx(dmb_i1tx), 
		.dmb_i2tx(dmb_i2tx), 
		.dmb_i3tx(dmb_i3tx), 
		.rpc_i1rx(rpc_i1rx), 
		.rpc_i2rx(rpc_i2rx), 
		.sel_usr(sel_usr), 
		.jtag_usr(jtag_usr), 
		.alct_txa(alct_txa), 
		.alct_txb(alct_txb), 
		.dmb_tx(dmb_tx), 
		.rpc_orx(rpc_orx), 
		.dmb_o1tx(dmb_o1tx), 
		.dmb_o2tx(dmb_o2tx), 
		.dmb_o3tx(dmb_o3tx), 
		.smb_clk(smb_clk), 
		.alct_loop(alct_loop), 
		.alct_txoe(alct_txoe), 
		.alct_clock_en(alct_clock_en), 
		.alct_rxoe(alct_rxoe), 
		.smb_data(smb_data), 
		.gtl_loop(gtl_loop), 
		.dmb_loop(dmb_loop), 
		.rpc_loop(rpc_loop), 
		.ccb_status_oe(ccb_status_oe), 
		._dmb_oe(_dmb_oe), 
		._ccb_tx(_ccb_tx), 
		._hard_reset_tmb_fpga(_hard_reset_tmb_fpga), 
		._hard_reset_alct_fpga(_hard_reset_alct_fpga), 
		.rst_qpll(rst_qpll), 
		.fcs(fcs), 
		.rpc_tx(rpc_tx), 
		.vme_reply(vme_reply), 
		.step(step), 
		.led_low(led_low), 
		.led_hi(led_hi), 
		.test_led(test_led), 
		.t12_fault(t12_fault), 
		.r12_fok(r12_fok), 
		.rxn(rxn), 
		.rxp(rxp), 
		.txn(txn), 
		.txp(txp), 
		.t12_rst(t12_rst), 
		.t12_sclk(t12_sclk), 
		.r12_sclk(r12_sclk)
	);

	initial begin
		// Initialize Inputs
		ck125n = 0;
		ck125p = 0;
		ck160n = 0;
		ck160p = 0;
		lhc_ckn = 0;
		lhc_ckp = 0;
		sw = 0;
		tmb_clock0 = 0;
		pb = 0;
		vme_cmd10 = 0;
		qpll_lock = 0;
		cfebemul_in = 0;
		vstat = 0;
		alct_rx = 0;
		jtag_usr0_tdo = 0;
		gp_io4 = 0;
		rpc_dsn = 0;
		rpc_smbrx = 0;
		prom_d3 = 0;
		prom_d7 = 0;
		jtag_fpga3 = 0;
		sda0 = 0;
		tmb_sn = 0;
		t_crit = 0;
		_ccb_rx = 0;
		dmb_rx = 0;
		dmb_i1tx = 0;
		dmb_i2tx = 0;
		dmb_i3tx = 0;
		rpc_i1rx = 0;
		rpc_i2rx = 0;
		t12_fault = 0;
		r12_fok = 0;
		rxn = 0;
		rxp = 0;

		// Wait 100 ns for global reset to finish
		#100;
		lhc_clock=1;
		#12.5
		lhc_clock=0;
				#12.5
		lhc_clock=0;
      		#12.5
		lhc_clock=0;
		#12.5
		lhc_clock=0;
		#12.5
		lhc_clock=0;		
		// Add stimulus here

	end
      
endmodule

