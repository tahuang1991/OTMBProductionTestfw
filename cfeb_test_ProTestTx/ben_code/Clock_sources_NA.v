`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    13:07:31 03/10/2011 
// Design Name: 
// Module Name:    Clock_sources 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module Clock_sources(
    input CMS_CLK_N,
    input CMS_CLK_P,
    input CMS80_N,
    input CMS80_P,
    input QPLL_CLK_AC_N,
    input QPLL_CLK_AC_P,
    input XO_CLK_AC_N,
    input XO_CLK_AC_P,
    input ALTXO_CLK_AC_N,
    input ALTXO_CLK_AC_P,
    input TMB_CLK_N,
    input TMB_CLK_P,
    input GC0N,
    input GC0P,
    input GC1N,
    input GC1P,
    input GC2N,
    input GC2P,
    output TP_B35_0N,
    output TP_B35_0P,
    output CMS80,
    output DAQ_TX_125_REFCLK,
    output TRG_TX_160_REFCLK,
    output DAQ_TX_200_REFCLK,
    output COMP_CLK,
    output COMP_CLK80,
	 input  DAQ_TXOUTCLK,
    output DAQ_DATA_CLK,
	 input  TRG_TXOUTCLK,
	 input  TRG_TX_PLL_LOCK,
	 input  TRG_CLK_SRC_TMB,
    output TRG_TXUSRCLK,
    output TRG_MMCM_LOCK,
    output CLK200,
    output CLK160,
    output CLK120,
    output CLK80,
    output CLK40,
    output CLK20,
    output CLK20_180,
    output CLK10,
    output ADC_CLK,
    input DAQ_MMCM_RST,
    output DAQ_MMCM_LOCK,
    output STRTUP_CLK,
    output EOS
    );

     //---------------------Dedicated GTX Reference Clock Inputs ---------------
    // Each dedicated refclk you are using in your design will need its own IBUFDS_GTXE1 instance
    
  wire daq_tx_125_refclk_dv2;
  wire trg_tx_160_refclk_dv2;
  wire daq_tx_200_refclk_dv2;
  
  wire cms_clk;
  wire tmb_clk;
  
  wire gc0,gc1,gc2;
  wire tp_b35_0;
  wire c20_phase_sel;
  wire dmy_cclk, dmy_din, dmy_tck, preq;
  
  
  assign tp_b35_0 = 1'b0;
  assign c20_phase_sel = 1'b0;
  assign ADC_CLK = c20_phase_sel ? CLK20_180 : CLK20;
  
  
	IBUFGDS #(.DIFF_TERM("TRUE"),.IOSTANDARD("DEFAULT")) IBUFGDS_CMS_CLK (.O(cms_clk),.I(CMS_CLK_P),.IB(CMS_CLK_N));
	IBUFGDS #(.DIFF_TERM("TRUE"),.IOSTANDARD("DEFAULT")) IBUFGDS_CMS80 (.O(CMS80),.I(CMS80_P),.IB(CMS80_N));
	IBUFDS_GTXE1 q3_clk0_refclk_ibufds_i (.O(DAQ_TX_125_REFCLK),.ODIV2(daq_tx_125_refclk_dv2),.CEB(1'b0),.I(XO_CLK_AC_P),.IB(XO_CLK_AC_N));
	IBUFDS_GTXE1 q3_clk1_refclk_ibufds_i (.O(TRG_TX_160_REFCLK),.ODIV2(trg_tx_160_refclk_dv2),.CEB(1'b0),.I(QPLL_CLK_AC_P),.IB(QPLL_CLK_AC_N));
	IBUFDS_GTXE1 q4_clk0_refclk_ibufds_i (.O(DAQ_TX_200_REFCLK),.ODIV2(daq_tx_200_refclk_dv2),.CEB(1'b0),.I(ALTXO_CLK_AC_P),.IB(ALTXO_CLK_AC_N));
	IBUFGDS #(.DIFF_TERM("TRUE"),.IOSTANDARD("DEFAULT")) IBUFGDS_TMB_CLK (.O(tmb_clk),.I(TMB_CLK_P),.IB(TMB_CLK_N));
	IBUFGDS #(.DIFF_TERM("TRUE"),.IOSTANDARD("DEFAULT")) IBUFGDS_GC0 (.O(gc0),.I(GC0P),.IB(GC0N));
	IBUFGDS #(.DIFF_TERM("TRUE"),.IOSTANDARD("DEFAULT")) IBUFGDS_GC1 (.O(gc1),.I(GC1P),.IB(GC1N));
	IBUFGDS #(.DIFF_TERM("TRUE"),.IOSTANDARD("DEFAULT")) IBUFGDS_GC2 (.O(gc2),.I(GC2P),.IB(GC2N));
	OBUFDS #(.IOSTANDARD("DEFAULT")) OBUFDS_TP_B35_0 (.O(TP_B35_0P),.OB(TP_B35_0N),.I(tp_b35_0));
   BUFG BUFG_daq_clk (.O(DAQ_DATA_CLK),.I(DAQ_TXOUTCLK));
   BUFG BUFG_200_clk (.O(CLK200),.I(DAQ_TX_200_REFCLK));
   BUFGMUX comp_clk_mux   (.O(COMP_CLK),  .I0(trg_clk40),.I1(tmb_clk40),.S(TRG_CLK_SRC_TMB));
   BUFGMUX comp_clk80_mux (.O(COMP_CLK80),.I0(trg_clk80),.I1(tmb_clk80),.S(TRG_CLK_SRC_TMB));


//----------------------------------------------------------------------------
// Output     Output      Phase    Duty Cycle   Pk-to-Pk     Phase
// Clock     Freq (MHz)  (degrees)    (%)     Jitter (ps)  Error (ps)
//----------------------------------------------------------------------------
// CLK_OUT1    40.000      0.000      50.0      247.096    196.976
// CLK_OUT2    80.000      0.000      50.0      200.412    196.976
// CLK_OUT3    20.000      0.000      50.0      298.160    196.976
// CLK_OUT4    20.000    180.000      50.0      298.160    196.976
// CLK_OUT5    10.000      0.000      50.0      342.202    196.976
// CLK_OUT6   160.000      0.000      50.0      169.112    196.976
// CLK_OUT7   120.000      0.000      50.0      180.794    196.976
//
//----------------------------------------------------------------------------
// Input Clock   Input Freq (MHz)   Input Jitter (UI)
//----------------------------------------------------------------------------
// primary          40.000            0.010
  daq_mmcm daq_mmc1(.CLK_IN1(cms_clk),.CLK_OUT1(CLK40),.CLK_OUT2(CLK80),.CLK_OUT3(CLK20),.CLK_OUT4(CLK20_180),.CLK_OUT5(CLK10),.CLK_OUT6(CLK160),.CLK_OUT7(CLK120),.RESET(DAQ_MMCM_RST),.LOCKED(DAQ_MMCM_LOCK));

//----------------------------------------------------------------------------
// Output     Output      Phase    Duty Cycle   Pk-to-Pk     Phase
// Clock     Freq (MHz)  (degrees)    (%)     Jitter (ps)  Error (ps)
//----------------------------------------------------------------------------
// CLK_OUT1   160.000    270.000      50.0      116.326     95.014
// CLK_OUT2    80.000    315.000      50.0      132.221     95.014
// CLK_OUT3    40.000      0.000      50.0      153.625     95.014
//
//----------------------------------------------------------------------------
// Input Clock   Input Freq (MHz)   Input Jitter (UI)
//----------------------------------------------------------------------------
// primary          80.000            0.010

  trg_mmcm trg_mmcm1(.CLK_IN1(TRG_TXOUTCLK),.CLK_OUT1(TRG_TXUSRCLK),.CLK_OUT2(trg_clk80),.CLK_OUT3(trg_clk40),.RESET(!TRG_TX_PLL_LOCK),.LOCKED(TRG_MMCM_LOCK));
//  BUFG BUFG_160_clk (.O(CLK160),.I(TRG_TXUSRCLK));


//----------------------------------------------------------------------------
// Output     Output      Phase    Duty Cycle   Pk-to-Pk     Phase
// Clock     Freq (MHz)  (degrees)    (%)     Jitter (ps)  Error (ps)
//----------------------------------------------------------------------------
// CLK_OUT1    40.000      0.000      50.0      247.096    196.976
// CLK_OUT2    80.000      0.000      50.0      200.412    196.976
//
//----------------------------------------------------------------------------
// Input Clock   Input Freq (MHz)   Input Jitter (UI)
//----------------------------------------------------------------------------
// primary           40.00            0.010

//comp_mmcm comp_mmcm1(.CLK_IN1(tmb_clk),.CLK_OUT1(tmb_clk40),.CLK_OUT2(tmb_clk80));
wire tmb_clkfb_out, tmb_clkfp_in;
comp_mmcm comp_mmcm1(.CLK_IN1(tmb_clk),.CLKFB_IN(tmb_clkfb_in),.CLK_OUT1(tmb_clk40),.CLK_OUT2(tmb_clk80),.CLKFB_OUT(tmb_clkfb_out));
  BUFG BUFG_tmb_fb_clk (.O(tmb_clkfb_in),.I(tmb_clkfb_out));


//
// configuration clock for Power On state machines
//  
   STARTUP_VIRTEX6 #(
      .PROG_USR("FALSE")  // Activate program event security feature
   )
   strt_up_v6 (
      .CFGCLK(dmy_cclk),       // 1-bit output Configuration main clock output
      .CFGMCLK(STRTUP_CLK),     // 1-bit output Configuration internal oscillator clock output
      .DINSPI(dmy_din),       // 1-bit output DIN SPI PROM access output
      .EOS(EOS),             // 1-bit output Active high output signal indicating the End Of Configuration.
      .PREQ(preq),           // 1-bit output PROGRAM request to fabric output
      .TCKSPI(dmy_tck),       // 1-bit output TCK configuration pin access output
      .CLK(1'b0),             // 1-bit input User start-up clock input
      .GSR(1'b0),             // 1-bit input Global Set/Reset input (GSR cannot be used for the port name)
      .GTS(1'b0),             // 1-bit input Global 3-state input (GTS cannot be used for the port name)
      .KEYCLEARB(1'b0), // 1-bit input Clear AES Decrypter Key input from Battery-Backed RAM (BBRAM)
      .PACK(1'b0),           // 1-bit input PROGRAM acknowledge input
      .USRCCLKO(1'b0),   // 1-bit input User CCLK input
      .USRCCLKTS(1'b1), // 1-bit input User CCLK 3-state enable input
      .USRDONEO(1'b1),   // 1-bit input User DONE pin output control
      .USRDONETS(1'b0)  // 1-bit input User DONE 3-state enable output
   );
endmodule
