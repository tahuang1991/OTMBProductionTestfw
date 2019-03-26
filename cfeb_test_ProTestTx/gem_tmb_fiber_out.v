`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name:    gem_tmb_fiber_out 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//////////////////////////////////////////////////////////////////////////////////
module gem_tmb_fiber_out #(
	parameter SIM_SPEEDUP = 0
)
(
	output TMB_TX0N, // these are the raw differential signals going to the SFPs
	output TMB_TX0P,
	output TMB_TX1N,
	output TMB_TX1P,
	input [47:0] out0data,  // 48-bit data to transmit on link0. IMPORTANT: sync reg to negedge "LHC_CLK" (~40 MHz)
	input [47:0] out1data,  // 48-bit data to transmit on link1. IMPORTANT: sync reg to negedge "LHC_CLK" (~40 MHz)
	input TMB_TX_REFCLK, // this is the 160 MHz clock phase-locked to the LHC clock
	input TMB_TXUSRCLK,  // for our GTX, here we use the 160 MHz clock phase-locked to the LHC clock
	input TMB_CLK80,     // this is the 80 MHz derived from the LHC clock (so not exactly 80 MHz)
	input TMB_GTPTXRST,  // for this input use the "DCM !lock" signal, where DCM is reset by Jitter cleaner !lock signal
	input TMB_TX_PLLRST, // we usually set this to logic 0, but better to use Jitter cleaner !lock signal
	input TMB_RST,       // for this use !TX_SYNC_DONE | !DCM_locked | "LinkResync" (if GEM has it)... sync with posedge "LHC_CLK"
	output TMB_TX_PLL_LOCK, // just for monitoring and debug use
	output TMB_TXRESETDONE, // just for monitoring and debug use
	output TX_SYNC_DONE, // used for monitoring and debug, also a reset for the GTP module (see above)
//	input INJ_ERR,    // old, for OTMB PRBS module, ignore for now
//	output STRT_LTNCY,   // old, for OTMB PRBS module, ignore for now
	output reg LTNCY_TRIG   // for monitoring: may be useful to measure data latency later on scope
	);

// Note that "LHC_CLK" and TMB_CLK80 can be phase-shifted relative to the LHC clock, but they must be in-phase with each other, and must exactly match the LHC frequency. 


//Inputs to TMB GTP transmitter
   wire [3:0] 	   tmb_tx0isk;
   wire [31:0] 	   tmb_tx0data;
   wire [3:0] 	   tmb_tx1isk;
   wire [31:0] 	   tmb_tx1data;
//   wire 	   tx_dlyaligndisable0;
//   wire 	   tx_dlyalignreset0;
//   wire 	   tx_dlyaligndisable1;
//   wire 	   tx_dlyalignreset1;
   wire 	   tx_enpmaphasealign0;
   wire 	   tx_pmasetphase0;
   wire 	   tx_enpmaphasealign1;
   wire 	   tx_pmasetphase1;
   reg 		   tmb_txresetdone_r;
   reg 		   tmb_txresetdone_r2;
   reg 		   tmb_txresetdone1r;
   reg 		   tmb_txresetdone1r2;
   reg [15:0] 	   frm_sep;
   reg [7:0] 	   trgcnt;
   reg 		   lt_trg;
   reg 		   rst_tx;

   wire TMB_TXRESETDONE0, TMB_TXRESETDONE1;
   wire TX_SYNC_DONE0, TX_SYNC_DONE1;
   wire TMB_TX_PLL_LOCK0, TMB_TX_PLL_LOCK1;
   reg  tx_sel;
   reg  tx_sel_bar;


         csc_gtp #
	   ( .WRAPPER_SIM_GTPRESET_SPEEDUP   (SIM_SPEEDUP)  // Set this to 1 for simulation
	   ) ucsc_gtp (
	  //_____________________________________________________________________
	  //      TILE0  (X1Y1)

	  //---------------------- Loopback and Powerdown Ports ----------------------
	    .TILE0_LOOPBACK0_IN (),  // in [2:0]  **not used by CSC
	    .TILE0_LOOPBACK1_IN (),  // in [2:0]  **not used by CSC
	  //------------------------------- PLL Ports --------------------------------
	    .TILE0_CLK00_IN (TMB_TX_REFCLK),  // in  * the 160 MHz REF Clock, common
	    .TILE0_CLK01_IN (TMB_TX_REFCLK),  // in
	    .TILE0_GTPRESET0_IN (TMB_GTPTXRST),  // in
	    .TILE0_GTPRESET1_IN (TMB_GTPTXRST),  // in
	    .TILE0_PLLLKDET0_OUT (TMB_TX_PLL_LOCK0),  // out
	    .TILE0_PLLLKDET1_OUT (TMB_TX_PLL_LOCK1),  // out
	    .TILE0_RESETDONE0_OUT (TMB_TXRESETDONE0),  // out
	    .TILE0_RESETDONE1_OUT (TMB_TXRESETDONE1),  // out
	  //--------------------- Receive Ports - 8b10b Decoder ----------------------
	    .TILE0_RXCHARISK0_OUT (),  // out [3:0]  **not used by CSC
	    .TILE0_RXCHARISK1_OUT (),  // out [3:0]  **not used by CSC
	    .TILE0_RXDISPERR0_OUT (),  // out [3:0]  **not used by CSC
	    .TILE0_RXDISPERR1_OUT (),  // out [3:0]  **not used by CSC
	    .TILE0_RXNOTINTABLE0_OUT (),  // out [3:0]  **not used by CSC
	    .TILE0_RXNOTINTABLE1_OUT (),  // out [3:0]  **not used by CSC
	  //------------- Receive Ports - Comma Detection and Alignment --------------
	    .TILE0_RXCOMMADET0_OUT (),  // out  **not used by CSC
	    .TILE0_RXCOMMADET1_OUT (),  // out  **not used by CSC
	    .TILE0_RXENMCOMMAALIGN0_IN (),  // in  **not used by CSC
	    .TILE0_RXENMCOMMAALIGN1_IN (),  // in  **not used by CSC
	    .TILE0_RXENPCOMMAALIGN0_IN (),  // in  **not used by CSC
	    .TILE0_RXENPCOMMAALIGN1_IN (),  // in  **not used by CSC
	  //----------------- Receive Ports - RX Data Path interface -----------------
	    .TILE0_RXDATA0_OUT (),  // out [31:0]  **not used by CSC
	    .TILE0_RXDATA1_OUT (),  // out [31:0]  **not used by CSC
	    .TILE0_RXRECCLK0_OUT (),  // out  **not used by CSC
	    .TILE0_RXRECCLK1_OUT (),  // out  **not used by CSC
	    .TILE0_RXUSRCLK0_IN (),  // in  **not used by CSC
	    .TILE0_RXUSRCLK1_IN (),  // in  **not used by CSC
	    .TILE0_RXUSRCLK20_IN (),  // in  **not used by CSC
	    .TILE0_RXUSRCLK21_IN (),  // in  **not used by CSC
	  //----- Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR ------
	    .TILE0_RXN0_IN (),  // in  **not used by CSC
	    .TILE0_RXN1_IN (),  // in  **not used by CSC
	    .TILE0_RXP0_IN (),  // in  **not used by CSC
	    .TILE0_RXP1_IN (),  // in  **not used by CSC
	  //-------------------------- TX/RX Datapath Ports --------------------------
	    .TILE0_GTPCLKFBEAST_OUT (),  // out [1:0]  **not used by CSC?
	    .TILE0_GTPCLKFBWEST_OUT (),  // out [1:0]  **not used by CSC?
	    .TILE0_GTPCLKOUT0_OUT (),  // out [1:0]  **not used by CSC?
	    .TILE0_GTPCLKOUT1_OUT (),  // out [1:0]  **not used by CSC?
	  //----------------- Transmit Ports - 8b10b Encoder Control -----------------
	    .TILE0_TXCHARISK0_IN (tmb_tx0isk),  // in [3:0]
	    .TILE0_TXCHARISK1_IN (tmb_tx1isk),  // in [3:0]
	  //------------- Transmit Ports - TX Buffer and Phase Alignment -------------
	    .TILE0_TXENPMAPHASEALIGN0_IN (tx_enpmaphasealign0),  // in
	    .TILE0_TXENPMAPHASEALIGN1_IN (tx_enpmaphasealign1),  // in
	    .TILE0_TXPMASETPHASE0_IN (tx_pmasetphase0),  // in
	    .TILE0_TXPMASETPHASE1_IN (tx_pmasetphase1),  // in
	  //---------------- Transmit Ports - TX Data Path interface -----------------
	    .TILE0_TXDATA0_IN (tmb_tx0data),  // in [31:0]
	    .TILE0_TXDATA1_IN (tmb_tx1data),  // in [31:0]
	    .TILE0_TXUSRCLK0_IN (TMB_TXUSRCLK),  // in
	    .TILE0_TXUSRCLK1_IN (TMB_TXUSRCLK),  // in
	    .TILE0_TXUSRCLK20_IN (TMB_CLK80),  // in
	    .TILE0_TXUSRCLK21_IN (TMB_CLK80),  // in
	  //------------- Transmit Ports - TX Driver and OOB signalling --------------
	    .TILE0_TXN0_OUT (TMB_TX0N),  // out
	    .TILE0_TXN1_OUT (TMB_TX1N),  // out
	    .TILE0_TXP0_OUT (TMB_TX0P),  // out
	    .TILE0_TXP1_OUT (TMB_TX1P)  // out
	    );
   
   
	 //---------------------------- TXSYNC module ------------------------------
	 // Since you are bypassing the TX Buffer in your wrapper, you will need to drive
	 // the phase alignment ports to align the phase of the TX Datapath. Include
	 // this module in your design to have phase alignment performed automatically as
	 // it is done in the example design.

	 always @(posedge TMB_CLK80 or negedge TMB_TXRESETDONE0) begin
		if(!TMB_TXRESETDONE0) begin
			tmb_txresetdone_r  <= 1'b0;
			tmb_txresetdone_r2 <= 1'b0;
		end
		else begin
			tmb_txresetdone_r  <= TMB_TXRESETDONE0;
			tmb_txresetdone_r2 <= tmb_txresetdone_r;
		end
	 end
	 always @(posedge TMB_CLK80 or negedge TMB_TXRESETDONE1) begin
		if(!TMB_TXRESETDONE1) begin
			tmb_txresetdone1r  <= 1'b0;
			tmb_txresetdone1r2 <= 1'b0;
		end
		else begin
			tmb_txresetdone1r  <= TMB_TXRESETDONE1;
			tmb_txresetdone1r2 <= tmb_txresetdone1r;
		end
	 end
   
	 TX_SYNC #(
		  .SIM_TXPMASETPHASE_SPEEDUP   (SIM_SPEEDUP)      // Set this to 1 for simulation
	 )
	 mgt_txsync0 (
		  .TXENPMAPHASEALIGN  (tx_enpmaphasealign0),  // out to GTP module
		  .TXPMASETPHASE      (tx_pmasetphase0),      // out to GTP module
//		  .TXDLYALIGNDISABLE  (tx_dlyaligndisable0),  // out  **GTP does not have this
//		  .TXDLYALIGNRESET    (tx_dlyalignreset0),    // out  **GTP does not have this
		  .SYNC_DONE          (TX_SYNC_DONE0),        // out for monitoring or Reset control
		  .USER_CLK           (TMB_CLK80),           // in, it is USRCLK2 used in GTP
		  .RESET              (!tmb_txresetdone_r2)  // in
	 );

   	 TX_SYNC #(
		  .SIM_TXPMASETPHASE_SPEEDUP   (SIM_SPEEDUP)      // Set this to 1 for simulation
	 )
	 mgt_txsync1 (
		  .TXENPMAPHASEALIGN  (tx_enpmaphasealign1),  // out to GTP module
		  .TXPMASETPHASE      (tx_pmasetphase1),      // out to GTP module
//		  .TXDLYALIGNDISABLE  (tx_dlyaligndisable1),  // out  **GTP does not have this
//		  .TXDLYALIGNRESET    (tx_dlyalignreset1),    // out  **GTP does not have this
		  .SYNC_DONE          (TX_SYNC_DONE1),        // out for monitoring or Reset control
		  .USER_CLK           (TMB_CLK80),           // in, it is USRCLK2 used in GTP
		  .RESET              (!tmb_txresetdone1r2)  // in
	 );

         assign TMB_TX_PLL_LOCK = TMB_TX_PLL_LOCK0 & TMB_TX_PLL_LOCK1;
         assign TMB_RESETDONE = TMB_TXRESETDONE0 & TMB_TXRESETDONE1;
         assign TX_SYNC_DONE = TX_SYNC_DONE0 & TX_SYNC_DONE1;

//
// Transmit data frame
//  
	assign tmb_tx0data = rst_tx ? 32'h50BC50BC : (tx_sel ? out0data[47:16] : {out0data[15:0],frm_sep});
	assign tmb_tx0isk  = rst_tx ?  4'b0101 :     (tx_sel ?               4'b0000     :  4'b0001);

	assign tmb_tx1data = rst_tx ? 32'h50BC50BC : (tx_sel ? out1data[47:16] : {out1data[15:0],frm_sep});
	assign tmb_tx1isk  = rst_tx ?  4'b0101 :     (tx_sel ?               4'b0000     :  4'b0001);


// Doing this rst correctly is critical for getting the data phase right:
	always @(posedge TMB_CLK80) begin
		rst_tx <= TMB_RST;
		LTNCY_TRIG <= lt_trg;
	end
	always @(posedge TMB_CLK80 or posedge TMB_RST) begin
		if(TMB_RST) begin
			tx_sel <= 1'b1;
			tx_sel_bar <= 1'b1;
		end
		else begin
			tx_sel <= ~tx_sel;
			tx_sel_bar <= tx_sel;
		end
	end

// This is to trigger the latency measurement signal; not required for normal operation
	always @(posedge TMB_CLK80 or posedge rst_tx) begin
		if(rst_tx) begin
			trgcnt <= 8'h00;
		end
		else begin
			trgcnt <= trgcnt + 1;
		end
	end
	always @* begin
		if(!rst_tx && (trgcnt==8'h00)) begin
			frm_sep = 16'h50FC;
			lt_trg = 1'b1;
		end
		else begin
			frm_sep = 16'h50BC;
			lt_trg = 1'b0;
		end
	end
	

endmodule
