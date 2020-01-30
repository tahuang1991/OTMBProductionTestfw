`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Create Date:    12:22 8/20/11
// Design Name: 
// Module Name:    mezz2012
//
// Mezz2012 test version info
//  5.1:  begin production tests for Mezz2012, starting from cfeb_test v3.5 with corrected 2012 pinouts
//  5.2:  tune qpll_locklost startup; switch to -1 speed grade, no problems!
//  5.3:  add single fiber test/debug features for Prod.Test software
//  5.4:  en_cabletests is not cleared by Reset.  enable fiber_count.
//  5.5:  ForceErr affects Tx output as well as Rx now.  fiber_count_r gets Reset now.  TMB_SoftRst clears Fiber errors.
//        Replaced _gtl_oe output with qpll_err input.  Removed GTX LOC constraints in fiber code.
//  5.6:  removed GbE and other unneeded stuff.  Added testLED signals for Loopback debug.  Fix step4 loopback error cases.
//  5.7:  replace ck125 with qpll_ck40 everywhere, redo LOCK monitoring logic, remove ck125 everywhere.  Rearrange testLEDs.
//  5.8:  put all Rx comp_fiber logic into a separate module "rcv_compfiber"
//  5.9:  expanded to 7 rcv-Comp fiber modules, enabled software checking for them. Remove MMCMs from cfeb_fiber_in.
//  5.10: add FCS driven HIGH! Also in UCF.  Make trig_start/stop load results register with "en_test" status, etc.
//        - reversed the two swapped" Rx Fibers, added a "pol_swap" argument in the cmp/rcv modules.
//  5.11: put bc0 out on clct_stat1 (inverse on stat3)
//  5.12: added sw[8] requirement to set fiber_stat 1-7 bits (PRBS is enabled), invert clct_stat1 to "_bc0"
//        - use 2 STAT bits to control Tx Board (via CFEB Emulator):
//                              _reset (on clct_stat3), ccb_code7c = _ForceError (clct_stat2)
//        - send INVERSE of these (inc BC0) to be High True on Tx Board, so no change when cable is not plugged
//        - receive 24-bit data from 5 CFEB cables at 80 MHz, check that it INCREMENTS and they are EQUAL
//        - LEDs setup for cable test, with results reported in dedicated Results Register
//  5.13: added control for cfeb_oe and cfeb_clock_en (drive them all HIGH)
//  5.14: for NEW Rev-4 Mez boards... the QPLL & LHC clock sources changes on-board (was tmb_clk05p & tmb_clk0==lhc_ck==lhc_clk}
//        - but NOW they must be tmb_clk0 & qpll_ck40(differential)==lhc_clk
//        - was tmb_clk0 -> lhc_ck, but now it's tmb_clk05p==unused?  Also tmb_clock1 is unused?  Add a check for these later!
//        - set command D5 for Hard Reset test (some constant bits & some variable)
//  5.15: Added checks for lhc_ck (tmb_clock05p) and tmb_clock1 connectivity in register D5 (the Hard Reset test register).
//  5.16: Add I/O for SCL[2:0] & SDA0 (3D delay chip, 60-bit serial register controls)... usually load all FF, but 00 for en_fibertests
//        - changed UCF to correct for on-board swap of rpc_rx36 & 37.
//  5.17: rearranged !sw[7] TPs to check ccb cable controls bc0, cmd7c, reset {= !_ccb_rx[1] || (!sw[7] & !pb)... ccb_res4 == L1reset}
//        - now both-Off has debug for ratloop[29] errors (ratloop3stat[5]), also signals for DDD chips SDin and SDout
//  5.18: make err_ratloop[29] similar to 27 & 30 (killed during en_fibertests) and make [4] the opposite (killed with !en_fibertests)
//        - this should help avoid SMB data line conflicts with LM84BIMQA temp sensor by killing the SMB clock
//        - need to make sure software only checks these bits during appropriate en_fibertests phase!
//  5.19: bring ksync out of rcv_compfiber & send to test_led TPs
//  5.20: add "ksynclost" monitor logic, show "data" on LEDs when "AD" is detected
//  5.21: add monitoring for NonZero data & compfifo_dav to LEDs, tune logic for ksynclost monitoring
//  5.22: fix ksynclost logic in comp_fifo_in
//  5.23: make 3 AD registers to cover bad-phase case, load d_reg for |nzdat[1]
//  5.24: pick out d_reg GEM bits 30:24 rather than 6:0 for now...
//  5.25: add logic for reading DSN chip ds2401 (1-wire protocol), modify LEDs (include fiber latency monitor)
//  5.26: fix bug in DSN IOBUF control
//  5.27: fix bug in DSN bitcount during state=2
//  5.28: fix bug in DSN command, reverse bit transmit sequence (LSB last)
//  5.29: For Production Mez Boards: only change UCF, swap bits for DMB_TX39 & 40
//  5.30: change ODDR for step[1]/dmbfifo_step1ck to run off of lhc_clk90
//  5.31: changed trg_tx_ & cmp_rx_buf_bypass "powersave" GTX bits to work wirh ISE 14; also changed "div25" MMCM (slwclk) params to keep FVCO over 600 MHz to meet speed grade -1 limitations... but now it is only divide by 8.533333, so "bufg_div8p5clk" module is created. Finally, for LHC_clk MMCM, try using the FB clock to drive logic (saves a bufg)
//  5.33: extend the 7 fibers to 12 fibers 
//  
// 
//  -- also bring out some data bits to show they are random and all channels are equal, and perhaps phase shifted.
//  
//  
//     OLD, General about clock uses:
//  tmb_clock0 --> lhc_ck --> mmcm"lhc4phase": lhc_clk,lhc_clk90  [lhc_locked, no RESET]  !lhc_locked to Reset div25 MMCM
//    lhc_clk90 --> lout_ratloops
//    lhc_clk --> communication logic & mmcm"div25clk": step1-dmbfifoclk,slwclk  [stopped & locked]
//      slwclk --> time_count, slowloops & debounce
//  tmb_clock05p --> qpll --> qpll_ck40 --> nowhere!
//  qpll_ck40 --> monitors other clocks: locked160...ck160_locklost. Checked by lhc_clk
//     Fiber GTX clock uses:
//  tmb_clock05p --> qpll --> ck160 --> GTX ref_clk --> gtx_txpll: tx_clk_out@80MHz [ck160_locked]
//    tx_clk_out --> mmcm"x2div2": snap_clk2@160MHz, tx_clk@80MHz, ck40 (to nowhere)    [locked160]
//      tx_clk --> gtx Tx logic & fiber force_err logic
//      snap_clk2 --> gtx Tx logic
//  rx_clk ("recovered Rx clock"--> triad fifo & data handling logic
//  
//  
//    procedure: en_cabletests, L1rst, en_fibertests, reset Tx board, send TMB_SoftRst, check errors (1a, RST, 1b, PB, 70)
//
//  
// - Old dcfeb_test started with (Post)RadTest v3.4 then take away the snap12 logic, replace with Ben's fiber clct code
//    two outputs to testpoints: Valid and Match.  Add a wire to PCB for access inside VME crate.
//    GbE needs two modes (use switch 8 for control): 
//       dump an error count packet at ~1 Hz
//       dump a packet every time we get a non-zero triad 
//    Use switch 7 to force a transmit error (like we do now).
//
//
//  OLDER about clocks:
//   QPLL links with tmb_clock05p (no delay), which is disabled when step4=Hi and prevents QPLL lock (bad)
//     -- this is dumb; QPLL should use tmb_clock0 (io600 on B31)? Only disabled by mez_clock_en from TMB/VME boot reg. b12
//     -- tmb_clock1 comes in on K24, has delay AND it stops when step4=Hi
//   qpll_ck40 comes directly from QPLL40, derived from tmb_clk05p that gets stopped bt step4; goes nowhere
//   lhc_ck NOW comes directly from tmb_clock0; slwclk is this divided by 25
//     -- "locked" indicates lock OK from slwclk div25 function
//   rx_clk is gtx_Rx 160 MHz reconstructed receiver USR clock
//     -- "ck160_locked" indicates gtx_Tx lock OK, but not Rx!
//     -- it's not clear that rx_clk has any bufg or lock implemented at all!
//   ck125 is not really used, just to monitor other clocks   --REPLACED w/QPLL_CK40
//  
// 
// 
//  
//  MTP Fiber Mapping to Signal Name, FPGA GTX channels, Diff. Pin Numbers, verilog name
//    1:   Tx0-Rx0   GTX3-GTX0    AK1/AK2=Q0 - AP5/AP6=Q0     txp/n[0]-rxp/n[0] 
//    2:   Tx1-Rx1   GTX4-GTX1    AH1/AH2=Q1 - AM5/AM6=Q0     txp/n[1]-rxp/n[1]
//    3:   Tx2-Rx2   GTX7-GTX2    AB1/AB2=Q1 - AL3/AL4=Q0     txp/n[2]-rxp/n[2]
//    4:   Tx3-Rx3   GTX8-GTX3    Y1/Y2=Q2   - AJ3/AJ4=Q0     txp/n[3]-rxp/n[3]
//    9:   Tx4-Rx8   GTX9-GTX8    V1/V2=Q2   - AA3/AA4=Q2     txp/n[4]-rxp/n[4]
//    10:  Tx5-Rx9   GTX10-GTX9   T1/T2=Q2   - W3/W4=Q2       txp/n[5]-rxn/p[5] --RX swapped!
//    11:  Tx6-Rx10  GTX11-GTX10  P1/P2=Q2   - U3/U4=Q2       txp/n[6]-rxn/p[6] --RX swapped!
//    12:  Tx7-Rx11  GTX2-GTX11   AM1/AM2=Q0 - R3/R4=Q2       txp/n[7]-rxp/n[7] <<-- used this @OSU
//    5:    Rx4        GTX4           AG3/AG4
//    6:    Rx5        GTX5           AF5/AF6
//    7:    Rx6        GTX6           AE3/AE4
//    8:    Rx7        GTX7           AC3/AC4
//      QPLL 160 refclk comes into pins AB6/AB5, Quad 113 refclk 1 (Q1,C1)
//   Q0=quad_112, Q1=quad_113, Q2=quad_114, Q3=quad_115, Q4=quad_116
// 
//////////////////////////////////////////////////////////////////////////////////
module mezz2019(
//    input 	      ck125n, ck125p,
    input 	      ck160n, ck160p,
    input 	      lhc_ckn, lhc_ckp,
    input  [8:7]      sw,
    input 	      tmb_clock0, tmb_clock1, pb, vme_cmd10,
    input 	      qpll_lock,
    input  [3:0]      vstat, // +1.5V TMB, Vcore RPC (driven via loopback), +3.3V TMB, +5V TMB
    input [28:1]      alct_rx,
    input 	      jtag_usr0_tdo, gp_io4, rpc_dsn, rpc_smbrx,
    input 	      prom_d3, prom_d7, jtag_fpga3, sda0, tmb_sn, t_crit,
    input [50:0]      _ccb_rx,  // add 42-47
// ccb_rx1=L1rst; 7-2=CMD, 8,9=ev/bcntrst; 10=cmdstr, 11=bc0, 12=L1A, 13=datstr, 21-14=DAT
    input [5:0]       dmb_rx,
    input [15:8]      dmb_i1tx,  // yes, input for loopback tests, only use on MODIFIED TMBs!
    input [31:24]     dmb_i2tx,  // yes, input for loopback tests, only use on MODIFIED TMBs!
    input [43:38]     dmb_i3tx,  // yes, input for loopback tests, only use on MODIFIED TMBs!
    input [9:0]       rpc_i1rx,
    input [37:26]     rpc_i2rx,
    input [23:0]      cfeb0_rx,
    input [23:0]      cfeb1_rx,
    input [23:0]      cfeb2_rx,
    input [23:0]      cfeb3_rx,
    input [23:0]      cfeb4_rx,
    inout             mez_sn,   // bidir signal to DSN chip via iobuf
    output [3:0]      sel_usr,
    output [3:1]      jtag_usr, // [0] is Input, see above
    output [17:5]     alct_txa,
    output [23:19]    alct_txb,
    output [7:0]      dmb_tx,
    output [25:10]    rpc_orx,
    output [23:16]    dmb_o1tx,
    output [37:32]    dmb_o2tx,
    output [48:44]    dmb_o3tx,
    output            smb_clk, alct_loop, alct_txoe, alct_clock_en, alct_rxoe, smb_data, // == smb0,alct_tx31,rpc_smbtx_out,SMD,DTA,io_269
    output            gtl_loop, dmb_loop, rpc_loop, ccb_status_oe, _dmb_oe, // set normal safe bidir bus modes
    output [26:0]     _ccb_tx,  // add 20-26
    output            _hard_reset_tmb_fpga, _hard_reset_alct_fpga,
    output 	      rst_qpll, fcs, cfeb_oe,
    output [3:0]      rpc_tx, // [3] is alct_tx29 on TMB!
    output [6:0]      vme_reply,
    output [4:0]      cfeb_clock_en, step, // step4 enables STEP bits 3:0 = cfeb, rpc, dmb, alct. step4 Low makes all free-running clocks.
    output [2:0]  scl,  // SLA,SCK,SIN.  Make these a Reg for logical control...
    output reg [7:0]  led_low,
    output reg [15:8] led_hi,
    output reg [10:1] test_led,
    input 	      t12_fault, r12_fok, qpll_err, // _gtl_oe is now qpll_err on Mezz 2012
    input             rxn0, rxp0,
    input  [7:1]      rxn, rxp,
    input  [3:0]      gemrxn, gemrxp,
    //output [1:1]      txn, txp,
    output 	      t12_rst, t12_sclk, r12_sclk
   )/* synthesis syn_useioff = 1 */;


// snap12 GTX signals
   //wire        all_tx_ready;
   //wire        snap_clk2, ck160_locked;
   //wire        snap_wait;
   //wire [7:0]  check_ok_snapr, check_bad_snapr;
   //wire [7:0]  rxdv_snapr, rxcomma_snapr, synced_snapt;
   //wire [7:0]  rxdv_diff, rxcomma_diff;
   //wire [7:0]  lgood_snapr, lbad_snapr, llost_snapr;
   //wire [15:0] snap_rxdat;
   //wire [1:0]  snap_rxk;
   //reg  [7:0]  time_r_snap;
   //reg [15:0]  snap_tx_dat;
    
   parameter Nfibers = 4'd12;
   wire [Nfibers:1] rxn_12, rxp_12;

   assign rxn_12[7:1]   = rxn[7:1];
   assign rxp_12[7:1]   = rxp[7:1];
   assign rxn_12[11:8]  = gemrxn[3:0];
   assign rxp_12[11:8]  = gemrxp[3:0];
   assign rxn_12[12]    = rxn0;
   assign rxp_12[12]    = rxp0;

   parameter DDDCODE_ON  = 64'h00E03F0E03F0E03F;
   parameter DDDCODE_OFF = 64'h0FC0A0FC0A0FC0A0;

   parameter SEEDBASE = 64'h8731c6ef4a5b0d29;
   parameter SEEDSTEP = 16'hc01d;
   reg [5:0]  ireg;
   reg [22:0] free_count;
   reg 	      free_tc, free_tc_r;
   reg 	      slow_tc, slow_tc_r;
   reg [7:0]  time_count;
   reg [7:0]  time_r_i;
   reg [7:0]  time_r;
   reg [7:0]  time_40i;
   reg [7:0]  time_40r;
   wire  gtx_ready;
//   wire [13:0] test_in;
   reg 	       l_locked160, ck160_locklost, l_qpll_lock, qpll_locklost;
   wire reset, gtx_reset;

   wire stopped, locked, locked160, dmbfifo_step1ck;
   assign locked160 = 1'b1;// not used
   wire ck160, lhc_ck, lhc_clk, qpll_ck40, slwclk;   // ref QPLL160, ccb_ck40, QPLL40, ccb_ck40/25=1.6MHz
   wire lhc_ck0, lhc_ck90, lhc_ck180, lhc_ck270, lhcckfbout, lhcckfbout_buf, lhc_clk90;
   wire zero, one;
   wire [1:0] zero2;
   wire [31:0] zero32;
   reg [3:0]  lhc_ckreg, tmb_ck1reg;

   wire [12:0] low;
   wire [3:0]  ignore;  // outputs from GTX we don't care about

   reg [23:0]  cfeb0rx,cfeb1rx,cfeb2rx,cfeb3rx,cfeb4rx, cfeb0rx_r,cfeb1rx_r,cfeb2rx_r,cfeb3rx_r,cfeb4rx_r;
   reg [23:0]  cfeb0rxin,cfeb1rxin,cfeb2rxin,cfeb3rxin,cfeb4rxin;
   reg [9:0]   cfeb_start, cfeb_err;
   reg [15:0]  cfeb0errcnt, cfeb1errcnt, cfeb2errcnt, cfeb3errcnt, cfeb4errcnt;

// JGhere, Add-ons for Ben dCFEB testing:
   //wire        tx_clk_out, tx_clk;
   //wire        tx_begin, tx_fc; // JGnew
/*
    output reg rx_start_r, rx_fc_r, rx_valid_r, rx_match_r, synced_snapr_r,   // bring to fabric domain!
    output reg compfifo_dav_r, compfifo_overflow_r, err_r,   // bring to fabric domain!
    output reg [15:0] err_count_r,  // bring to fabric domain!
    output reg [47:0] comp_dout,  // bring to fabric domain!
    output [47:0] compfifo_dout
*/
   // ================================
   //one per fiber !!, Tao
   // ================================
   wire [Nfibers:1]  synced_snapr, rx_strt, rx_valid, rx_match, rx_fc, compfifo_dav, compfifo_overflow, err_f, ksync, ksynclost, rx_pll_locked;
   wire [15:0] err_count_f[Nfibers:1];
   wire [47:0] comp_data[Nfibers:1], compfifo_data[Nfibers:1];
   wire [3:1]  nzdat[Nfibers:1];
   reg  [23:0] triad_word, triad_word_r;
   reg  [Nfibers:0]  fiber_stat, fiber_invalid; // these are synched to fabric clock
   reg [11:0]  error_f1count, error_f2count, error_f3count, error_f4count, error_f5count, error_f6count, error_f7count, error_f8count,error_f9count,error_f10count,error_f11count, error_f12count;
   reg [3:0]   f1stat, f2stat, f3stat, f4stat, f5stat, f6stat, f7stat, f8stat, f9stat, f10stat, f11stat, f12stat;

   assign ck160_locked = rx_pll_locked[1]; // not used
   // to flag any bad fiber
   reg fiber_stat_all, fiber_invalid_all;
   reg [6:0]   d_reg, o_reg;
   reg [3:1]   ad_reg;
   wire [3:1]  ad;


// 27 registers for inputs from DMB Loopback, 47 for RPCloop, plus 5 (7?) for SLOWloop
   reg  [46:0] shft_seq, rnd_word;  // slow seq .1 sec shift, loads to flag on pb_pulse  shft_seq = 32'h00000001;
   reg 	       debounced_bit;    // sets one pulse for 200 ns  (5 MHz clock)
   reg 	       pb_pulse;  //  <- sw7 & !pb, clears on pb high after 2nd seq. shift (debounce), lasts a while!
   reg 	       err_wait;   // pb_pulse & tc & !wait -> load rnd_word, set wait.  !pb_pulse & tc & wait -> clear wait
   //reg 	       ferr_i, ferr_r, ferr_rr, ferr_done;


   assign cfeb_oe = 1'b1;
   assign cfeb_clock_en = 5'h1f;
/*  these used to be for testing translator chip SEUs, now unused:
   assign test_in[0] = sda0;
   assign test_in[1] = tmb_sn;
   assign test_in[2] = t_crit;
   assign test_in[3] = jtag_fpga3;
   assign test_in[4] = prom_d3;
   assign test_in[5] = prom_d7;
*/
   assign vme_reply[0] = 1'b1;   // OE_b, low true
   assign vme_reply[1] = 1'b1;   // DIR
   assign vme_reply[2] = 1'b0;   // DTACK, inverted on TMB board
   assign vme_reply[3] = ~vme_cmd10; // IACKOUT = IACKIN, inverted on TMB board?
   assign vme_reply[4] = 1'b0;   // BERR, inverted on TMB board
   assign vme_reply[5] = 1'b0;   // IRQ, inverted on TMB board
   assign vme_reply[6] = 1'b0;   // vme_ready, High enables 0-5 above?
//   assign _gtl_oe = 1'b0;   // SEU danger. Now disconnected from FPGA, io_311 shorted to GND on Mezz board
//  /gtl_oe now = GND on board (R140), gtl_loop is Open (R137), dmb_loop=Jumper (R139), ccb_status_oe is Open (R138), 
//  set rpc_loop=0 (1 for tests), /dmb_oe=0          normally Open^^^ Closed only at TAMU, normal TMBs will kill FPGA!
   assign gtl_loop = 1'b0;  // JRG: always set LOW (SEU danger, make OPEN --PD)
   assign dmb_loop = 1'b1;  // JRG: set HIGH for SPECIAL TMB ONLY! LOW for normal CMS operation (SEU danger, S5 OPEN --PD)
   assign rpc_loop = 1'b1;  // JRG: io_306, set HIGH for Production Test, LOW for normal CMS operation (SEU safe --PD)
   assign ccb_status_oe = 1'b1;  // JRG: io_310, set HI for Prod Test & normal CMS operation? (SEU danger, R138 OPEN --PU)
   assign _dmb_oe = 1'b0;
   assign _hard_reset_tmb_fpga = 1'b1;
   
   assign low=0;
   assign zero=0;
   assign zero2=2'b0;
   assign zero32=0;
   assign one=1'b1;

   assign fcs   = 1'b1;   // must drive HIGH to XCF128 Prom
   assign t12_rst  = 1'b1;  // low-true signal for Snap12 Transmitter
   assign t12_sclk = 1'b1;  // to Snap12 Transmitter
   assign r12_sclk = 1'b1;  // to Snap12 Receiver
   assign rst_qpll = 1'b1;  // reset is low-true, but in ExtControl mode (sw6 On) it becomes fSelect[5]
   //                                                 and autoRestart (sw5) becomes fSelect[4]
   //        note that sw1-4 are fSelect[0:3] but they only function in ExtControl mode (sw6 On),
   //        and fSelect sets the VCXO center frequency (because automatic calibration is Off)
   //   qpll_lock is probably random in ExtControl mode (sw6 On)

   wire  ccb_cken; // global clock signal to use for ccb_rx[0] "clock"
   reg [11:0]  ccbrxzero_count; // count toggles on ccb_rx0
   reg [12:0]  pulse_count; // count triggers, 10 low-true sources ANDed together
   reg [3:0]  hard_count; // like pulse_count triggers, only cleared by hard reset
   reg [11:0]  pulses_fired;
   reg [11:0]  in_pulse_r;
   wire [11:0] in_pulse;
   wire      bc0, rst_errcnt;
   reg       trigger, rst_errcnt_r;

// - pulse counter (pulse is CE); send pulses & read back count via status LEDs (11 ccb_rx)
//      > 1 reset signal (L1Reset=ccb_reserved4, to clear) and triggered by 11 different pulses:
//          BC0, L1A, tmb_soft_reset=tmb_reserved1, clct/alct_external_trigger, dmb_cfeb_calibrate[2:0], 
// 	    adb_pulse sync/async, alct_hard_reset,
//         - verify that all single CMD & DATA bits work reliably (avoid CMD/DATA = 0C,0D,0E,0F; 10,11,12,13; 40,41,42,43)
//         - need to check LEDs at least one time too, to verify status bus works

// unless noted otherwise, these pulses are only 25ns long and count just one time:
   assign bc0 = !_ccb_rx[11]; // BC0, CCB base+52
   assign rst_errcnt  = !_ccb_rx[29]; // TMB_SoftRst (tmb_res1), CCB base+6c or 6a
   assign in_pulse[0] = !_ccb_rx[11]; // BC0, CCB base+52
   assign in_pulse[1] = !_ccb_rx[12]; // L1A, CCB base+54
   assign in_pulse[2] = !_ccb_rx[29]; // TMB_SoftRst (tmb_res1), CCB base+6c or 6a
   assign in_pulse[3] = !_ccb_rx[32]; // clct_ext_trig, CCB base+86
   assign in_pulse[4] = !_ccb_rx[33]; // alct_ext_trig, CCB base+88
   assign in_pulse[5] = !_ccb_rx[39]; // dmb_cfeb_calib0, CCB base+8a
   assign in_pulse[6] = !_ccb_rx[40]; // dmb_cfeb_calib1, CCB base+8c
   assign in_pulse[7] = !_ccb_rx[41]; // dmb_cfeb_calib2, CCB base+8e
   assign in_pulse[8] = !_ccb_rx[27]; // alct_hard_reset_ccb, CCB base+66:  500 ns long pulse
   assign in_pulse[9] = !_ccb_rx[30]; // alct_adb_pulse_sync, CCB base+82:  500 ns long pulse
   assign in_pulse[10] = !_ccb_rx[31]; // alct_adb_pulse_async, CCB base+84: long pulse
   assign in_pulse[11] = !_ccb_rx[28]; // tmb_reserved0, Not Really a Pulse!  I will think of a better way to test this.
      //  -- right now we access this with CCB base+2a (CSRB6, write a 1 then a 0 to bit2): we get a random count each time

   assign _ccb_tx[26:22] = _ccb_rx[47:43];  // returns DMB_Reserved_Out[4:0] from CCB back to the CCB on TMB_Reserved_In[4:0]
   // CCB can Write DMB_Reserved_Out[4:0] (to all TMBs & DMBs) on base+2a (CSRB6, bits 14:10).  ccb_rx[47-43]
   // CCB can Read TMB_Reserved_In[4:0] from TMB on base+34 (CSRB11, bits 7:3).  ccb_tx[26-22]
   //   --> For this test, TMB will return the value we set on DMB_Reserved_Out, back to the CCB via TMB_Reserved_In.

//  For these I am not sure how best to test them, still thinking...
// ccb_rx0 should be a clock... count it to see it toggle, and send some bits via CCB.
// CCB can Write TMB_Reserved_Out[2:0] (to all TMBs) on base+2a (CSRB6, bits 9:7).  ccb_rx[38-36]
// CCB can Write TMB_Reserved0 (to all TMBs) on base+2a (CSRB6, bit2).   ccb_rx28
// CCB can Write CCB_Reserved[3:2] (to all TMBs) on base+2a (CSRB6, bits 1:0).  ccb_rx[25-24]
//   -- ccb_reserved(1:0) are for QPLL & TTC status... just try to read them back via CCB.  ccb_rx[23-22]
// TMB_L1A_Release/Request can generate L1As at the CCB... try this and count the L1As via CCB.  ccb_tx[21-20]
   
   wire [2:0]  tmb_res_out;
   wire [5:0]  ccb_unused;
   wire [7:0]  ccb_data;
   wire [7:0]  ccb_cmd;
   wire        ccb_cmdstrb, ccb_datstrb;
   wire        _alct_adb_pulse_async, _alct_adb_pulse_sync;
   reg [7:0]  ccb_data_r;
   reg [7:0]  ccb_cmd_r, last_cmd;
   reg        ccb_cmdstrb_r, ccb_datstrb_r, ccb_code7c;
   reg        alct_cfg_out, tmb_cfg_out, results_hold, late_load_done;
   reg [4:0]  get_bit_ptr;
   reg [19:0] results_r;
   reg [1:0]  ccb_rsv_r;
   reg [1:0]  tmb_l1a_relreq;
   assign ccb_data[7:0] = ~_ccb_rx[21:14];
   assign ccb_datstrb = !_ccb_rx[13];
   assign ccb_cmd[7:0] = { (~_ccb_rx[7:2]), (!_ccb_rx[8]), (!_ccb_rx[9])};
   assign ccb_cmdstrb = !_ccb_rx[10];
   assign tmb_res_out[2:0] = ~_ccb_rx[38:36];
   assign ccb_unused[4:0] = ~_ccb_rx[28:24];
   assign ccb_unused[5]   = ~_ccb_rx[42];

// clct_status: temp for testing,  _ccb_tx[8:0]
   assign _ccb_tx[8] = pb;
   assign _ccb_tx[7] = !pb;
   assign _ccb_tx[6] = pb;
   assign _ccb_tx[5] = !pb;
   assign _ccb_tx[4] = pb;
   assign _ccb_tx[3] = !reset; // cfebemul[2], set by ccb_rx1==ccb_res4==L1reset OR pb, as L1Reset for Tx board
   assign _ccb_tx[2] = !ccb_code7c; // cfebemul[3], used to force an error at the Tx board
   assign _ccb_tx[1] = !bc0;   // cfebemul[1], used to reset the Tx PRBS system sequence
   assign _ccb_tx[0] = !reset;     // cfebemul[4], set by ccb_rx1==ccb_res4==L1reset OR pb, as L1Reset for Tx board

// These tx bits are outputs for TMB_L1A_Release/Request. Create pulses using CCB_Reserved[3:2] from CCB:
//   when TMB L1A Release & Request go out, CCB should send an L1A, check!  May need to enable that on CCB.
   assign _ccb_tx[21:20] = tmb_l1a_relreq[1:0];  // must be 25ns pulses from TMB to CCB
   // CCB can Write CCB_Reserved[3:2] (to all TMBs) on base+2a (CSRB6, bits 1:0).  ccb_rx[25-24]

// take these to TMB-FP LEDs for observation:
   assign _alct_adb_pulse_async = !_ccb_rx[31];
   assign _alct_adb_pulse_sync = !_ccb_rx[30];
   assign ccb_ttcrx_rdy = !_ccb_rx[22];
   assign ccb_qpll_lck = !_ccb_rx[23];
//   assign mpc_in0 = !_ccb_rx[34];
//   assign mpc_in1 = !_ccb_rx[35];

   reg [38:0] init_dmbloop[26:0];
   reg en_loopbacks, en_loopbacks_r, en_loopbacks_rr, lhc_tog_err;
   reg lhc_tog, en_lhcloop, en_lhcloop_r, lhcloop_tog, lhcloop_tog_r;
   reg [26:0] in_dmbloop, lout_dmbloop, llout_dmbloop;
   wire [26:0] rawin_dmbloop, out_dmbloop;
   wire [27:0] good_dmbloop, err_dmbloop;
   wire [15:0] count_dmbloop[26:0];    // Not Used: 27 16-bit wide elements, error counts for DMB Loop signals
   wire [15:0] count_ratloop[46:0];    // Not Used: 47 16-bit wide elements, error counts for RAT Loop signals
   wire [15:0] count_slowloop[4:0];    // Not Used: 5 16-bit wide elements, error counts for SLOW Loop signals
   reg [11:0]  dmbloop_errcnt, dmbloop1_stat, dmbloop2_stat, dmbloop3_stat, fiber_count_r, cable_count_r;
   reg [31:0]  loop_count, fiber_count, cable_count;
   reg 	       en_cabletests, en_fibertests, en_cabletests_r, en_fibertests_r;
   reg [38:0]  init_ratloop[46:0];
   reg [38:0]  init_slowloop[4:0];
   reg [4:0]   in_slowloop, lout_slowloop, llout_slowloop;
   reg [46:0]  in_ratloop, lout_ratloop, loutpos_ratloop, llout_ratloop;
   wire [4:0]  out_slowloop, good_slowloop, err_slowloop;
   wire [46:0] rawin_ratloop, out_ratloop, good_ratloop, err_ratloop;
   reg [11:0]  ratloop_errcnt, ratloop1_stat, ratloop2_stat, ratloop3_stat, ratloop4_stat;
   reg [26:0]  slowloop_count;
   reg [11:0]  slowloop_errcnt, hzloop_count;
   reg [7:0]   slowloop_err, slowloop_stat;
   reg [3:0]   selusr;
   integer     i;

//   DMB Loop:  27 pairs + one clock.    lout_dmbloop goes out, then in_dmbloop comes back from DMB-Loopback
// dmbfifo_step1ck -> dmb_rx0   This is CCB clock, 40 MHz. But NOT a CLK pin!  Div2 via Flop and send to bufg?
//    Note: step4 selects STEP mode for 3:0 = cfeb, rpc, dmb, alct.  step4 Low makes all free-running clocks.
// 0 dmb_tx33 -> dmb_rx1
// 1 dmb_tx47 -> dmb_rx2
// 2 dmb_tx48 -> dmb_rx3
// 3 dmb_tx45 -> dmb_rx4
// 4 dmb_tx46 -> dmb_rx5
// 5 dmb_tx0  -> dmb_tx12
// 6 dmb_tx1  -> dmb_tx13
// 7 dmb_tx2  -> dmb_tx14
// 8 dmb_tx3  -> dmb_tx15
// 9 dmb_tx4  -> dmb_tx8
//10 dmb_tx5  -> dmb_tx9
//11 dmb_tx6  -> dmb_tx10
//12 dmb_tx7  -> dmb_tx11
//13 dmb_tx16 -> dmb_tx28
//14 dmb_tx17 -> dmb_tx29
//15 dmb_tx18 -> dmb_tx30
//16 dmb_tx19 -> dmb_tx31
//17 dmb_tx20 -> dmb_tx24
//18 dmb_tx21 -> dmb_tx25
//19 dmb_tx22 -> dmb_tx26
//20 dmb_tx23 -> dmb_tx27
//21 dmb_tx32 -> dmb_tx42
//22 dmb_tx34 -> dmb_tx38
//23 dmb_tx35 -> dmb_tx39
//24 dmb_tx36 -> dmb_tx40
//25 dmb_tx37 -> dmb_tx41
//26 dmb_tx44 -> dmb_tx43
   assign dmbfifo_step1ck = dmb_rx[0];
   assign dmb_o2tx[33] = lout_dmbloop[0];
   assign rawin_dmbloop[0] = dmb_rx[1];
   assign dmb_o3tx[47] = lout_dmbloop[1];
   assign rawin_dmbloop[1] = dmb_rx[2];
   assign dmb_o3tx[48] = lout_dmbloop[2];
   assign rawin_dmbloop[2] = dmb_rx[3];
   assign dmb_o3tx[45] = lout_dmbloop[3];
   assign rawin_dmbloop[3] = dmb_rx[4];
   assign dmb_o3tx[46] = lout_dmbloop[4];
   assign rawin_dmbloop[4] = dmb_rx[5];

   assign dmb_tx[7:0] = lout_dmbloop[12:5];
   assign rawin_dmbloop[12:5] = {dmb_i1tx[11:8],dmb_i1tx[15:12]};
   assign dmb_o1tx[23:16] = lout_dmbloop[20:13];
   assign rawin_dmbloop[20:13] = {dmb_i2tx[27:24],dmb_i2tx[31:28]};
   assign dmb_o2tx[32] = lout_dmbloop[21];
   assign dmb_o2tx[37:34] = lout_dmbloop[25:22];
   assign dmb_o3tx[44] = lout_dmbloop[26];
   assign rawin_dmbloop[26:21] = {dmb_i3tx[43],dmb_i3tx[41:38],dmb_i3tx[42]};
// JGhere, RAT_Loopback signals:
   assign rpc_tx[3:0] = lout_ratloop[3:0]; // [3] is actually alct_tx29
   assign smb_clk = !en_fibertests_r | lout_ratloop[4]; // JGhere, this is only active during en_fibertests, otherwise it's HIGH
   assign alct_txa[17:5] = lout_ratloop[17:5];
   assign _hard_reset_alct_fpga = lout_ratloop[18]; // this goes to alct_tx18
   assign alct_txb[23:19] = lout_ratloop[23:19];
   assign alct_loop = lout_ratloop[24];
   assign alct_txoe = lout_ratloop[25];
   assign alct_clock_en = lout_ratloop[26];
   assign step[0] = lout_ratloop[27];  // requires step[4] to be set or this goes nowhere!
   assign alct_rxoe = lout_ratloop[28];
   assign smb_data = !en_fibertests_r & lout_ratloop[29]; // JGhere, this is inactive -LOW- during en_fibertests.  similar to bits 27,30
   assign step[2] = lout_ratloop[30];  // requires step[4] to be set or this goes nowhere!
   assign rpc_orx[25:10]  = lout_ratloop[46:31];

   assign rawin_ratloop[0] = rpc_i2rx[30]; // rpc_sync="rpc_tx"0
   assign rawin_ratloop[1] = rpc_i2rx[29]; // ERROR! rpc_posneg="rpc_tx"1
   assign rawin_ratloop[2] = rpc_i2rx[31]; // rpc_loop_tm="rpc_tx"2
   assign rawin_ratloop[3] = alct_rx[4];   // rpc_free0="rpc_tx"3=alct_tx29   bubble
   assign rawin_ratloop[4] = rpc_i2rx[27]; // smb_clk
   assign rawin_ratloop[5] = alct_rx[25];  // alct_txa5-17
   assign rawin_ratloop[6] = alct_rx[26];  //  bubble
   assign rawin_ratloop[7] = alct_rx[24];  //  2*bubble
   assign rawin_ratloop[8] = alct_rx[21];
   assign rawin_ratloop[9] = alct_rx[22];  // ERROR alct_tx9   2*bubble
   assign rawin_ratloop[10] = alct_rx[23]; // ERROR alct_tx10
   assign rawin_ratloop[11] = alct_rx[20]; //  2*bubble
   assign rawin_ratloop[12] = alct_rx[18]; // ERROR alct_tx12   bubble
   assign rawin_ratloop[13] = alct_rx[19]; // ERROR! alct_tx13  bubble
   assign rawin_ratloop[14] = alct_rx[17];
   assign rawin_ratloop[15] = alct_rx[14]; //  2*bubble
   assign rawin_ratloop[16] = alct_rx[15]; // ERROR alct_tx16
   assign rawin_ratloop[17] = alct_rx[2];  //  bubble
   assign rawin_ratloop[18] = alct_rx[1];  // typo-error, fixed  // alct_tx18 = hard_reset_alct_fpga    bubble
   assign rawin_ratloop[19] = alct_rx[16]; // ERROR   // alct_txb19-23   2*bubble
   assign rawin_ratloop[20] = alct_rx[12]; // ERROR alct_tx20  bubble
   assign rawin_ratloop[21] = alct_rx[9];  // ERROR alct_tx21  bubble
   assign rawin_ratloop[22] = alct_rx[10]; //  bubble
   assign rawin_ratloop[23] = alct_rx[8];  //  2*bubble
   assign rawin_ratloop[24] = alct_rx[5];  // Fixed error?  // alct_loop
   assign rawin_ratloop[25] = alct_rx[7];  // alct_txoe
   assign rawin_ratloop[26] = alct_rx[11]; // alct_clock_en
   assign rawin_ratloop[27] = alct_rx[13]; // step0
   assign rawin_ratloop[28] = alct_rx[6];  // alct_rxoe   bubble
   assign rawin_ratloop[29] = alct_rx[3];  // ERROR   // smb_data   bubble
   assign rawin_ratloop[30] = rpc_i2rx[28];// step2   bubble
   assign rawin_ratloop[31] = rpc_i1rx[7]; // rpc_rx10-25
   assign rawin_ratloop[32] = rpc_i1rx[8];
   assign rawin_ratloop[33] = rpc_i1rx[9];
   assign rawin_ratloop[34] = rpc_i1rx[6];
   assign rawin_ratloop[35] = rpc_i1rx[3];
   assign rawin_ratloop[36] = rpc_i1rx[4];
   assign rawin_ratloop[37] = rpc_i1rx[5];
   assign rawin_ratloop[38] = rpc_i1rx[0];
   assign rawin_ratloop[39] = rpc_i1rx[1];
   assign rawin_ratloop[40] = rpc_i1rx[2];
   assign rawin_ratloop[41] = alct_rx[28]; // 20   bubble
   assign rawin_ratloop[42] = rpc_i2rx[32];
   assign rawin_ratloop[43] = rpc_i2rx[33];
   assign rawin_ratloop[44] = rpc_i2rx[34];
   assign rawin_ratloop[45] = alct_rx[27];
   assign rawin_ratloop[46] = rpc_i2rx[26];

   assign err_dmbloop[27] = lhc_tog_err; // JGhere, Fixed. uses lhc_clk 40, but could use prbs data?
   assign good_dmbloop[27] = !(|err_dmbloop);

//   assign step[4] = en_loopbacks; // ~sw[8];  // step4 Low makes free clocks from TMB; Hi allows logic signals for clocks
   assign step[4] = (en_loopbacks & (~en_fibertests)); // ~sw[8];  // step4 Low makes free clocks from TMB; Hi allows logic signals for clocks
   assign step[3] = 1'b0;   // this is cfeb step signal
//   assign step[2] = 1'b0;   // this is rpc step signal
//   assign step[1] = lhc_clk;  // this is dmb step signal... now uses ODDR below.
//   assign step[0] = 1'b0;   // this is alct step signal
   assign sel_usr[3:0] = selusr[3:0];  // if !en_fibertests selusr <= 4'b1101
   assign jtag_usr[3] = lout_slowloop[2]; // vstat[2] = slowloop2 is SLOW!!  Only ~3 HZ max from power-sense chip
   assign jtag_usr[1] = lout_slowloop[0];
   assign jtag_usr[2] = lout_slowloop[1];
   always @(*)
     begin
	if (!en_fibertests) selusr = 4'b1101;    // rpc_jtag active, 3 bits only, includes ~1 Hz Vstat2 test
	else selusr = {2'b00,lout_slowloop[4],lout_slowloop[3]}; // alct_jtag active, 5 bits under test
     end

   ODDR #(.DDR_CLK_EDGE("OPPOSITE_EDGE"), .INIT(1'b0), .SRTYPE("ASYNC")) DMB_FIFO_CLK (.Q(step[1]), .C(lhc_clk90), .CE(1'b1), .D1(1'b1), .D2(1'b0), .R(1'b0), .S(1'b0));  // make step[1] an image of lhc_clk, as it goes out and loops back as dmbfifo_step1ck
//   ODDR #(.DDR_CLK_EDGE("OPPOSITE_EDGE"), .INIT(1'b0), .SRTYPE("ASYNC")) DMB_FIFO_CLK (.Q(step[1]), .C(lhc_clk), .CE(1'b1), .D1(1'b1), .D2(1'b0), .R(1'b0), .S(1'b0));  // make step[1] an image of lhc_clk, as it goes out and loops back as dmbfifo_step1ck
   

   initial begin
      l_locked160 = 0;
      ck160_locklost = 0;
      l_qpll_lock = 0;
      qpll_locklost = 0;
      time_r = 8'h00;
      rst_errcnt_r  = 0; // TMB_SoftRst (tmb_res1), CMDbus h70 or h78, CCB base+6c or 6a.  use to clear fiber & cable errors
      free_tc_r = 1'b0;
      slow_tc_r = 1'b0;
      pulse_count = 0;
      hard_count = 0;
      loop_count = 0;
      fiber_count = 0;
      fiber_count_r = 0;
      cable_count = 0;
      cable_count_r = 0;
      en_loopbacks = 1'b0;
      en_loopbacks_r = 0;
      en_loopbacks_rr = 0;
      en_cabletests_r = 0;
      en_cabletests = 1'b0;
      en_fibertests_r = 0;
      en_fibertests = 1'b0;
      llout_dmbloop = 0;
      lout_dmbloop = 0;
      in_dmbloop = 0;
      dmbloop_errcnt = 0;
      dmbloop1_stat = 0;
      dmbloop2_stat = 0;
      dmbloop3_stat = 0;
      lhc_tog = 0;
      en_lhcloop = 0;
      en_lhcloop_r = 0;
      lhcloop_tog = 0;
      lhcloop_tog_r = 0;
      lout_ratloop = 0;
      loutpos_ratloop = 0;
      llout_ratloop = 0;
      in_ratloop = 0;
      ratloop_errcnt = 0;
      ratloop1_stat = 0;
      ratloop2_stat = 0;
      ratloop3_stat = 0;
      ratloop4_stat = 0;
      slowloop_count = 0;
      slowloop_err = 0;
      slowloop_stat = 0;
      lout_slowloop = 0;
      llout_slowloop = 0;
      in_slowloop = 0;
      slowloop_errcnt = 0;
      selusr = 4'b1101;
      error_f1count = 0;
      error_f2count = 0;
      error_f3count = 0;
      error_f4count = 0;
      error_f5count = 0;
      error_f6count = 0;
      error_f7count = 0;
      error_f8count = 0;
      error_f9count = 0;
      error_f10count = 0;
      error_f11count = 0;
      error_f12count = 0;
      f1stat = 0;
      f2stat = 0;
      f3stat = 0;
      f4stat = 0;
      f5stat = 0;
      f6stat = 0;
      f7stat = 0;
      f8stat = 0;
      f9stat = 0;
      f10stat = 0;
      f11stat = 0;
      f12stat = 0;
      fiber_stat = 0;
      fiber_invalid = 0;
      fiber_stat_all = 0;
      fiber_invalid_all  = 0;
      for (i = 0; i < 27; i = i + 1) begin
	 init_dmbloop[i] = 39'd15 + 11401017*i;
      end
      for (i = 0; i < 47; i = i + 1) begin
	 init_ratloop[i] = 39'd68 + 1301017*i;
      end
      for (i = 0; i < 5; i = i + 1) begin
	 init_slowloop[i] = 39'd89 + 11901017*i;
      end
      shft_seq = 47'h000000000001;
      rnd_word = 0;
      debounced_bit = 0;
      pb_pulse = 0;
      ccb_cmdstrb_r = 0;
      ccb_cmd_r = 0;
      ccb_code7c = 0;
      dsn_state = 0;    // states 0-4: noop, initial, write, read, endwait
      dsn_val = 0;      // 64-bit serial data input collected from DSN chip
      bitcount = 0;     // 7 bits, tracks the # of bits transferred
      scount = 0;       // 12-bit slwclk step counter "stopwatch"
      scount_rst = 0;   // reset for stopwatch
      drive_low = 0;    // float the dsn output
   end  // initial begin

    genvar u;
    generate
       for (u=0; u<27; u=u+1) begin:prbs39dmbgen
	  prbs39_test dmb_loops(init_dmbloop[u], en_loopbacks_r, in_dmbloop[u], out_dmbloop[u], good_dmbloop[u], err_dmbloop[u], count_dmbloop[u], err_wait&rnd_word[u], (!locked)|reset, lhc_clk);
       end
    endgenerate

    genvar x;
    generate
       for (x=0; x<47; x=x+1) begin:prbs39ratgen
	  prbs39_test rat_loops(init_ratloop[x], en_loopbacks_r, in_ratloop[x], out_ratloop[x], good_ratloop[x], err_ratloop[x], count_ratloop[x], err_wait&rnd_word[x], (!locked)|reset, lhc_clk); // slwclk?
       end
    endgenerate

    genvar w;
    generate
       for (w=0; w<5; w=w+1) begin:prbs39slowgen
	  prbs39_test slow_loops(init_slowloop[w], en_loopbacks_r, in_slowloop[w], out_slowloop[w], good_slowloop[w], err_slowloop[w], count_slowloop[w], err_wait&rnd_word[w], (!locked)|reset, slwclk); // slwclk?
       end
    endgenerate


   BUFG tmbclock05p(.I(tmb_clock0), .O(lhc_ck));  // now tmb_clk05p! just for tests of lhc_ckreg
   BUFG tmb_clock_1(.I(tmb_clock1), .O(tmb_ck1)); // now tmb_clk1! just for tests of tmb_ck1reg. comes via DDD chip, often OFF

   IBUFGDS #(.DIFF_TERM("TRUE"),.IOSTANDARD("LVDS_25"))  qpll40(.I(lhc_ckp) , .IB(lhc_ckn) , .O(qpll_ck40)); // from QPLL
//   -- goes to mmcm now for 4-phase generation and used for clock/lock monitoring
   
   IBUFDS_GTXE1  clock160(.I(ck160p) , .IB(ck160n) , .O(ck160), .ODIV2(), .CEB(zero));
//   -- from QPLL, goes to TxOutClk -> tx_clk, snap_clk2 & ck40

//   bufg_div25clk clk1p6(lhc_clk,!lhc_locked,slwclk,stopped,locked); // slwclk is now 1.6 MHz (was 5 MHz using ck125)  [40 -> 1.6]
   bufg_div8p5clk clk4p7(lhc_clk,!lhc_locked,slwclk,stopped,locked); // slwclk is now 4.6875 MHz (was 1.6, change forced by speed grade -1)

   BUFR ccbrx0_clock(.I(_ccb_rx[0]), .O(ccb_cken));


// MMCM for 4-phase LHC clock
  MMCM_ADV
  #(.BANDWIDTH            ("OPTIMIZED"),
    .CLKOUT4_CASCADE      ("FALSE"),
    .CLOCK_HOLD           ("FALSE"),
    .COMPENSATION         ("ZHOLD"),
    .STARTUP_WAIT         ("FALSE"),
    .DIVCLK_DIVIDE        (1),
    .CLKFBOUT_MULT_F      (25.000),
    .CLKFBOUT_PHASE       (0.000),
    .CLKFBOUT_USE_FINE_PS ("FALSE"),
    .CLKOUT0_DIVIDE_F     (25.000),
    .CLKOUT0_PHASE        (0.000),
    .CLKOUT0_DUTY_CYCLE   (0.500),
    .CLKOUT0_USE_FINE_PS  ("FALSE"),
    .CLKOUT1_DIVIDE       (25),
    .CLKOUT1_PHASE        (90.000),
    .CLKOUT1_DUTY_CYCLE   (0.500),
    .CLKOUT1_USE_FINE_PS  ("FALSE"),
    .CLKOUT2_DIVIDE       (25),
    .CLKOUT2_PHASE        (180.000),
    .CLKOUT2_DUTY_CYCLE   (0.500),
    .CLKOUT2_USE_FINE_PS  ("FALSE"),
    .CLKOUT3_DIVIDE       (25),
    .CLKOUT3_PHASE        (270.000),
    .CLKOUT3_DUTY_CYCLE   (0.500),
    .CLKOUT3_USE_FINE_PS  ("FALSE"),
    .CLKIN1_PERIOD        (25.0),
    .REF_JITTER1          (0.010))
  mmcm_lhc4phase
    // Output clocks
   (.CLKFBOUT            (lhcckfbout),
    .CLKFBOUTB           (),
    .CLKOUT0             (lhc_ck0),
    .CLKOUT0B            (),
    .CLKOUT1             (lhc_ck90),
    .CLKOUT1B            (),
    .CLKOUT2             (lhc_ck180),
    .CLKOUT2B            (),
    .CLKOUT3             (lhc_ck270),
    .CLKOUT3B            (),
    .CLKOUT4             (),
    .CLKOUT5             (),
    .CLKOUT6             (),
     // Input clock control
    .CLKFBIN             (lhc_clk),     // JRG FB experiment... was (lhcckfbout_buf),
    .CLKIN1              (qpll_ck40),  // was lhc_ck!  Still no delays and unaffected by step4
    .CLKIN2              (1'b0),
     // Tied to always select the primary input clock
    .CLKINSEL            (1'b1),
    // Ports for dynamic reconfiguration
    .DADDR               (7'h0),
    .DCLK                (1'b0),
    .DEN                 (1'b0),
    .DI                  (16'h0),
    .DO                  (),
    .DRDY                (),
    .DWE                 (1'b0),
    // Ports for dynamic phase shift
    .PSCLK               (1'b0),
    .PSEN                (1'b0),
    .PSINCDEC            (1'b0),
    .PSDONE              (),
    // Other control and status signals
    .LOCKED              (lhc_locked),
    .CLKINSTOPPED        (),
    .CLKFBSTOPPED        (),
    .PWRDWN              (1'b0),
    .RST                 (1'b0));  // drive this somehow?  use sr16 trick?
  // Output buffering
  //-----------------------------------
// JRG FB experiment... 
  BUFG lhcclkout0_buf
   (.O   (lhc_clk),
    .I   (lhcckfbout));
/*  JRG FB experiment... 
  BUFG lhcclkf_buf
   (.O (lhcckfbout_buf),
    .I (lhcckfbout));
  BUFG lhcclkout0_buf
   (.O   (lhc_clk),
    .I   (lhc_ck0));
*/

  BUFG lhcclkout90_buf
   (.O   (lhc_clk90),
    .I   (lhc_ck90));

// JGhere, add TPs to check this:
   assign scl[0] = !en_fibertests; // SIN
   assign scl[1] = lhc_clk;        // SCK
   assign scl[2] = en_loopbacks;   // AL


   assign reset = !_ccb_rx[1] || (!sw[7] & !pb);  // careful, this ccb_rx signal can screw up on-bench tests...

   assign gtx_ready = locked160 & ck160_locked & ( &synced_snapr); // JGhere, need to AND or OR 7 synced_snapr?  Only used for time_count startup delay.
   assign gtx_reset = reset | !gtx_ready;  // JGhere, is this a good idea?  Think SEU & mechanical troubles...ugh.


//   always @(posedge _ccb_rx[0] or posedge reset) begin
   always @(posedge ccb_cken or posedge reset) begin
      if(reset) begin
	 ccbrxzero_count <= 0;
      end
      else begin
	 ccbrxzero_count <= ccbrxzero_count + 1'b1; // count toggles on ccb_rx0
      end
   end // always @ (posedge _ccb_rx[0] or posedge reset)

   always @(posedge lhc_ck or posedge reset or posedge rst_errcnt) begin // only for testing tmb_clk05p connection to FPGA
      if(reset | rst_errcnt) begin
	 lhc_ckreg <= 0;
      end
      else begin
	 lhc_ckreg[3:0] <= {lhc_ckreg[2:0],1'b1};  // JGhere: send lhc_ckreg[3] to a monitoring bit
      end
   end

   always @(posedge tmb_ck1 or posedge reset or posedge rst_errcnt) begin // only for testing tmb_clk1 connection to FPGA, use SoftRst to check!
      if(reset | rst_errcnt) begin
	 tmb_ck1reg <= 0;
      end
      else begin
	 tmb_ck1reg[3:0] <= {tmb_ck1reg[2:0],1'b1};  // JGhere: send tmb_ck1reg[3] to a monitoring bit
      end
   end


   //always @(posedge tx_clk or posedge reset) begin
   //   if(reset) begin
   //      ferr_i <= 0;
   //      ferr_r <= 0;
   //      ferr_rr <= 0;	      
   //      triad_word_r <= 0;
   //   end
   //   else begin   // syncing forced single-clock effects (debounced, for bit error or triad word)
   //      ferr_i <= debounced_bit;  // normally zero
   //      ferr_r <= ferr_i;
   //      if (!sw[8]) triad_word <= rnd_word[23:0];  // normally zero
   //      if (!ferr_done & ferr_r) begin  // begin the Forced Error sequence, sync with snap tx_clk
   //         ferr_rr <= ferr_r;  // true for exactly one tx_clk cycle
   //         triad_word_r <= triad_word;  // loaded for exactly one tx_clk cycle
   //      end
   //      else begin  // end the Forced Error sequence when PB is released (ferr_i goes low)
   //         ferr_rr <= 0;
   //         triad_word_r <= 0;
   //      end
   //      ferr_done <= ferr_r;

   //   end

   //end // always @ (posedge tx_clk or posedge reset)


// start of DSN control & readout logic
   reg [3:0]   dsn_state;  // states 0-4: noop, initial, write, read, endwait
   reg [63:0]  dsn_val;    // 64-bit serial data input collected from DSN chip
   reg [6:0]   bitcount;   // 7 bits, tracks the # of bits transferred
   reg [11:0]  scount;     // 12-bit slwclk step counter "stopwatch"
   reg 	       scount_rst; // reset for stopwatch
   reg 	       drive_low;  // drive the dsn output (enable tri-state)
   wire        dsn_in;     // incoming signal from the mez_sn tri-state

   IOBUF mez_dsn (
		.O(dsn_in),   // incoming from DSN chip
		.IO(mez_sn),  // pin assigned for DSN signal
		.I(1'b0),     // only drive DSN low, or float
		.T(~drive_low) // Tbuf control, low=output, high=input, so I have to invert it here!
	   );
   
   always @(posedge slwclk or posedge reset) // DSN read block, will be activated for one full cycle after every reset
     begin
	if (reset) begin
	   dsn_state <= 0;  // states 0-4: noop, initial, write, read, endwait
	   dsn_val <= 0;    // 64-bit serial data input collected from DSN chip
	   bitcount <= 0;   // 7 bits, tracks the # of bits transferred
	   scount <= 0;     // 12-bit slwclk step counter "stopwatch"
	   scount_rst <= 0; // reset for stopwatch
	   drive_low <= 1'b0; // float the dsn output
	end
	else begin

	   if (scount_rst) begin // clear scount & move to next dsn_state
	      scount <= 0;
	      dsn_state <= dsn_state + 1'b1;
	      bitcount <= 0;   // 7 bits, tracks the # of bits transferred
	      drive_low <= 1'b0; // float the dsn output
	   end
	   else if (time_r[7]) scount <= scount + 1'b1; // wait 80 usec after reset_done state to begin

	   if ((dsn_state==0) && time_r[7]) begin // wait cycle, at least 1 ms = 1600 sclk after reset_done to begin
	      scount_rst <= (scount[11:0] == 12'h700); // reset stopwatch @1792 = 1120 usec, move to next dsn_state
	      drive_low <= 1'b0; // float the dsn output
	   end
// drive_low: 0 for a long time... 1100 usec min
// then 1 for 640 usec & 0 for 160 usec
	   if (dsn_state==1) begin // initialize cycle
	      if (scount[11:0]<12'h400) drive_low <= 1'b1;  // this is 640 usec Hi [meas 639]
	      else drive_low <= 1'b0;      // this is min 160 usec Low  [meas 160]
	      scount_rst <= (scount[11:0] == 12'h500); // reset stopwatch @1280 = 800 usec, move to next dsn_state
	   end

// drive_low:  1 for 5 usec then 0 for 155 usec; repeat one more time (to send a binary "11")
//   set  1 for 80 usec then 0 for 80 usec; repeat one more time (to send a binary "00")
// repeat both of the above steps one more time; combined it sends 0x33
	   if (dsn_state==2) begin // write command cycle, send 8'h33
	      // send 1 for bitcount == 0,1,4,5 -->  "1" = drive_low for 8 sclk, then float for 246
	      // send 0 for bitcount == 2,3,6,7 -->  "0" = drive_low for 128 sclk, then float for 128
	      if (scount[7:0]<8'h08) drive_low <= 1'b1;  // this is 5 usec Hi  [meas 5]
	      else if (!bitcount[1] && (scount[7:0]<8'h80)) drive_low <= 1'b1; // drives low 80 usec for 0,1,4,5
	      else drive_low <= 1'b0; // drive low is shorter for 2,3,6,7
	      if (scount[7:0]==8'hfc) bitcount <= bitcount + 1'b1; // increment bit counter @252 (total 256 slwclk per bit = 160 usec)
	      scount_rst <= (bitcount[3] & scount[0]); // reset stopwatch @8 bits = 1280 usec, move to next dsn_state @253
	   end

// drive_low:  1 for 1.2 usec then 0 for 18.8 usec; repeat 63 times
	   if (dsn_state==3) begin // input cycle, send dsn_val to readback register
	      if (scount[4:0]<5'd2) drive_low <= 1'b1;   // this is min 1.2 usec Hi
	      else drive_low <= 1'b0;
	      if (scount[4:0]==5'h16) begin // increment bit counter @22 (total 32 slwclk per bit == 20 usec)
		 dsn_val[63:0] <= {dsn_val[62:0],dsn_in};
		 bitcount <= bitcount + 1'b1;
	      end
	      scount_rst <= ((bitcount == 7'h40) && scount[0]); // reset stopwatch @64 bits = 1280 usec, move to next dsn_state
	   end

	   if (dsn_state>3) begin // cycle takes ~4500 usec to get here
	      drive_low <= 1'b0;  // this is the final "parked" state
	      bitcount <= 0;
	      scount <= 0;
	   end

	end
     end
   

   always @(posedge slwclk or posedge reset) // everything that uses 1.6 MHz clock with simple Reset (from lhc_clk)
     begin
	if (reset) begin
	   free_count <= 0;
	   time_count <= 0;
	   free_tc <= 0;
	   slow_tc <= 0;
	   shft_seq <= 47'h000000000001;
	   rnd_word <= 0;
	   debounced_bit <= 0;
	   err_wait <= 0;
	   pb_pulse <= 0;
	   slowloop_count <= 0;
	   slowloop_err <= 0;
	   slowloop_stat <= 0;
	   lout_slowloop <= 0;
	   llout_slowloop <= 0;
	   in_slowloop <= 0;
	   slowloop_errcnt <= 0;
	end
	else begin
	   free_count <= free_count + 1'b1;
// JGhere, add slow loop checking logic:
	   lout_slowloop[4] <= out_slowloop[4];
	   lout_slowloop[3] <= out_slowloop[3];
	   lout_slowloop[1] <= out_slowloop[1];
	   lout_slowloop[0] <= out_slowloop[0];
	   if (!en_fibertests) begin  // oe_rpc jtag active, usually only for short tests
	      lout_slowloop[2] <= free_count[19]; // vstat[2] = slowloop2 is SLOW!!  Only ~3 HZ max from power-sense chip
	      in_slowloop[0] <= rpc_i2rx[36]; // rawin_slowloop[5], driven by rpc jtag_usr/bus[1] (TDI)
	      in_slowloop[1] <= rpc_smbrx;    // rawin_slowloop[6]
	      in_slowloop[2] <= vstat[2]; // rawin_slowloop[7]  SLOW!!  Only ~3 HZ max from power-sense chip
	      in_slowloop[3] <= 1'b1; // ok, but only a real test for alct_jtag case!
	      in_slowloop[4] <= 1'b1; //  only a real test for alct_jtag case
	      if (en_loopbacks_r) begin
		 slowloop_err[6] <= err_slowloop[1];
		 slowloop_err[5] <= err_slowloop[0];
		 slowloop_err[4] <= 0;
		 slowloop_err[3] <= 0;
		 slowloop_err[2] <= 0;
		 slowloop_err[1] <= 0;
		 slowloop_err[0] <= 0;
		 if (free_count[18:0] == 19'h50000) begin
		    slowloop_err[7] <= (in_slowloop[2]^free_count[19]);
		    hzloop_count <= hzloop_count + 1'b1;
		 end
	      end
	      else slowloop_err <= 0;
	   end
	   else begin     //  oe_alct jtag active, expect to run this for "long term" tests
	      lout_slowloop[2] <= out_slowloop[2]; // gp_io4, no speed problem here...
	      in_slowloop[0] <= rpc_i2rx[37]; // rawin_slowloop[0], driven by alct jtag_usr/bus[1] (TDI)
	      in_slowloop[1] <= rpc_i2rx[35]; // rawin_slowloop[1]  alct_tx1=tms_alct  bubble
	      in_slowloop[2] <= gp_io4;   // rawin_slowloop[2]      alct_rx29
	      in_slowloop[3] <= rpc_dsn;  // rawin_slowloop[3]      alct_tx3=sel0_alct -> alct_rx30, real test   bubble
	      in_slowloop[4] <= jtag_usr0_tdo; // rawin_slowloop[4] alct_rx0, real test
	      if (en_loopbacks_r) begin
		 slowloop_err[7] <= 0;
		 slowloop_err[6] <= 0;
		 slowloop_err[5] <= 0;
		 slowloop_err[4] <= err_slowloop[4];
		 slowloop_err[3] <= err_slowloop[3];
		 slowloop_err[2] <= err_slowloop[2];
		 slowloop_err[1] <= err_slowloop[1];
		 slowloop_err[0] <= err_slowloop[0];
	      end
	      else slowloop_err <= 0;
	   end
	   if (locked & en_loopbacks_r) slowloop_count <= slowloop_count + 1'b1;
	   slowloop_stat <= slowloop_stat | slowloop_err;
	   if ( |slowloop_err ) slowloop_errcnt <= ((slowloop_errcnt[11:0] + 1'b1) | (slowloop_errcnt[11]<<11));

	   if (free_count == 23'h7fffff) slow_tc <= 1;  // ~.6 Hz or 1.664 sec cycle, for GbE looping
	   else slow_tc <= 0;

	   if (free_count[15:0] == 16'hffff) free_tc <= 1;  // ~76 Hz or .013 sec cycle, for debounce
	   else free_tc <= 0;

	   if (free_tc) shft_seq[46:0] <= {shft_seq[45:0],shft_seq[46]}; // shift a bit over time for "random" word
	   if (!pb_pulse) begin
	      pb_pulse <= sw[7] & !pb; // guaranteed true at least one full "free_tc" cycle
	      err_wait <= 0;
	      debounced_bit <= 0;
	   end
	   else begin
	      if (free_tc & !err_wait) begin
		 if (!sw[8]) rnd_word <= shft_seq; // use this for one "random" word when NOT PRBS testing.  one SLWCLK
//		 debounced_bit <= sw[8];   // use this for a simple bit error during PRBS testing.  lasts one SLWCLK period.
		 debounced_bit  <= 1;  // use this for a debounced test control pulse.  lasts one SLWCLK period.
		 err_wait <= 1;  // use this to sync with Xmit clocks & load rnd_word in their domains?
	      end
	      else if (free_tc & err_wait) begin
		 pb_pulse <= sw[7] & !pb;
	      end
	      else begin
		 rnd_word <= 0;
		 debounced_bit <= 0;
	      end
	   end
	   if (gtx_ready && time_count < 8'hfe) time_count <= time_count + 1'b1; // use to delay startup; ends.
	end
     end

   always @(posedge dmbfifo_step1ck or posedge reset) // used by DmbLoop[27]
     begin
	if (reset) begin
	   en_lhcloop <= 0;
	   lhcloop_tog <= 0;
	end
	else begin   // this is for dmbloop[27] testing
	   en_lhcloop <= en_loopbacks & !en_fibertests;  // like step[4]. JGhere, enables dmbloop[27], disable this if en_fibertests is set
	   if (en_lhcloop) lhcloop_tog <= !lhcloop_tog & !en_fibertests;;
	end
     end

   always @(negedge lhc_clk90 or posedge reset) // everything that uses neg_edge lhc_clk.
     begin
	if (reset) begin
	   lout_ratloop <= 0;
	end
	else begin
	   lout_ratloop <= out_ratloop;
	end
     end


   always @(posedge lhc_clk) // things that use lhc_clk, NO reset, put in IOBUF!  Tune CLK phase!
     begin
	cfeb0rxin <= cfeb0_rx;
	cfeb1rxin <= cfeb1_rx;
	cfeb2rxin <= cfeb2_rx;
	cfeb3rxin <= cfeb3_rx;
	cfeb4rxin <= cfeb4_rx;
     end

   assign ad[3] = (comp_data[1][47:40]==8'hAD); // data on bits 30:24
   assign ad[2] = (comp_data[1][23:16]==8'hAD); // data on bits 6:0
   assign ad[1] = (comp_data[1][15:8]==8'hAD); // data on bits 38:32?  bad sync case...
   always @(posedge lhc_clk or posedge reset) // things that use lhc_clk w/Reset (was ck40)
     begin
	if (reset) begin
	   lhcloop_tog_r <= 0;
	   en_lhcloop_r <= 0;
	   l_qpll_lock <= 0;    // use lhc_clk to monitor qpll_lock
	   qpll_locklost <= 0;
	   pulse_count <= 0;
	   in_pulse_r <= 0;
	   pulses_fired <= 0;
	   ccb_cmdstrb_r <= 0;
	   ccb_datstrb_r <= 0;
	   ccb_cmd_r <= 8'h00;
	   ccb_data_r <= 8'h00;
	   get_bit_ptr <= 0;
	   late_load_done <= 1'b0;
	   results_r <= 0;
	   results_hold <= 1'b0;
	   tmb_cfg_out <= 1'b0;
	   alct_cfg_out <= 1'b0;
	   last_cmd <= 8'h00;
	   ccb_rsv_r <= 0;
	   tmb_l1a_relreq <= 0;
//	   l1a_count <= 0;
	   en_loopbacks <= 0;
	   en_cabletests <= 1'b0; // this does nothing so far, just an alternative to sw[7]
	   en_fibertests <= 1'b0;
	   en_cabletests_r <= 1'b0; // this does nothing so far
	   en_fibertests_r <= 1'b0;
// were always in lhc_clk domain:
	   en_loopbacks_r <= 0;
	   en_loopbacks_rr <= 0;
	   lhc_tog <= 0;
	   lhc_tog_err <= 0;
	   lout_dmbloop <= 0;
	   llout_dmbloop <= 0;
	   in_dmbloop <= 0;
	   dmbloop_errcnt <= 0;
	   dmbloop1_stat <= 0;
	   dmbloop2_stat <= 0;
	   dmbloop3_stat <= 0;
	   loop_count <= 0;
	   cable_count <= 0;
	   cable_count_r <= 0;
	   loutpos_ratloop <= 0;
	   llout_ratloop <= 0;
	   in_ratloop <= 0;
	   ratloop_errcnt <= 0;
	   ratloop1_stat <= 0;
	   ratloop2_stat <= 0;
	   ratloop3_stat <= 0;
	   ratloop4_stat <= 0;
	   f1stat <= 0;
	   f2stat <= 0;
	   f3stat <= 0;
	   f4stat <= 0;
	   f5stat <= 0;
	   f6stat <= 0;
	   f7stat <= 0;
	   f8stat <= 0;
	   f9stat <= 0;
	   f10stat <= 0;
	   f11stat <= 0;
	   f12stat <= 0;
	   fiber_stat <= 0;
	   fiber_invalid <= 0;
	   fiber_stat_all <= 0;
	   fiber_invalid_all <= 0;
	   fiber_count <= 0;
	   fiber_count_r <= 0;
	   error_f1count <= 0;
	   error_f2count <= 0;
	   error_f3count <= 0;
	   error_f4count <= 0;
	   error_f5count <= 0;
	   error_f6count <= 0;
	   error_f7count <= 0;
	   error_f8count <= 0;
	   error_f9count <= 0;
	   error_f10count <= 0;
	   error_f11count <= 0;
	   error_f12count <= 0;
	   ccb_code7c <= 0;
	   cfeb_start <= 0;
	   cfeb_err <= 0;
	   cfeb0errcnt <= 0;
	   cfeb1errcnt <= 0;
	   cfeb2errcnt <= 0;
	   cfeb3errcnt <= 0;
	   cfeb4errcnt <= 0;
	   ad_reg <= 0;
	   d_reg <= 0;
	   o_reg <= 0;
	end
	else begin
	   time_40i <= time_count;
	   time_40r <= time_40i;  // transfer slow slwclk time counter to lhc_clk domain
	   lhcloop_tog_r <= lhcloop_tog; // this is for dmbloop[27] testing
	   if (time_40r[7] & qpll_lock) l_qpll_lock <= 1;
	   if (l_qpll_lock & (!qpll_lock)) qpll_locklost <= 1;
	   en_cabletests_r <= en_cabletests;
	   en_fibertests_r <= en_fibertests;
	   en_lhcloop_r <=  en_lhcloop;  // JGhere, disable this if en_fibertests is set?
	   in_pulse_r <= in_pulse;
	   pulses_fired <= (pulses_fired|in_pulse_r);
	   ccb_rsv_r <= ~_ccb_rx[25:24]; // this is ccb_reserved[3:2] controlled by CSRB6
	   tmb_l1a_relreq <= _ccb_rx[25:24]; // removed pulse logic; probably CCB won't send an L1A, check? UN-inverted this in v2p2

	   ad_reg <= ad;
	   if (|nzdat[1]) begin
	      d_reg <= (comp_data[1][30:24]);
	      o_reg <= (comp_data[1][6:0]);  // the "other" d_reg
	   end
//	   if (ad[2]) d_reg <= (comp_data[1][6:0]);
//	   else if (ad[3]) d_reg <= (comp_data[1][30:24]);
//	   else if (ad[1]) d_reg <= (comp_data[1][38:32]);
//	   else d_reg <= 0;

	   cfeb0rx <= cfeb0rxin;
	   cfeb1rx <= cfeb1rxin;
	   cfeb2rx <= cfeb2rxin;
	   cfeb3rx <= cfeb3rxin;
	   cfeb4rx <= cfeb4rxin;
	   cfeb0rx_r <= cfeb0rx;
	   cfeb1rx_r <= cfeb1rx;
	   cfeb2rx_r <= cfeb2rx;
	   cfeb3rx_r <= cfeb3rx;
	   cfeb4rx_r <= cfeb4rx;
	   if ( ~|cfeb0rx[23:0] ) cfeb_start[0] <= 1'b1;
	   if ( ~|cfeb1rx[23:0] ) cfeb_start[1] <= 1'b1;
	   if ( ~|cfeb2rx[23:0] ) cfeb_start[2] <= 1'b1;
	   if ( ~|cfeb3rx[23:0] ) cfeb_start[3] <= 1'b1;
	   if ( ~|cfeb4rx[23:0] ) cfeb_start[4] <= 1'b1;
	   if (cfeb_start[0]) begin
	      cfeb_err[0] <= |(cfeb2rx_r^cfeb0rx_r);
	      cfeb0errcnt <= cfeb0errcnt + cfeb_err[0];
	      if (|cfeb0rx[23:0]) cfeb_start[5] <= 1'b1;
	      if (cfeb_err[0]) cfeb_err[5] <= 1'b1;
	   end
	   if (cfeb_start[1]) begin
	      cfeb_err[1] <= |(cfeb2rx_r^cfeb1rx_r);
	      cfeb1errcnt <= cfeb1errcnt + cfeb_err[1];
	      if (|cfeb1rx[23:0]) cfeb_start[6] <= 1'b1;
	      if (cfeb_err[1]) cfeb_err[6] <= 1'b1;
	   end
	   if (cfeb_start[2]) begin
	      cfeb_err[2] <= |((cfeb2rx -1'b1) - cfeb2rx_r);
	      cfeb2errcnt <= cfeb2errcnt + cfeb_err[2];
	      if (|cfeb2rx[23:0]) cfeb_start[7] <= 1'b1;
	      if (cfeb_err[2]) cfeb_err[7] <= 1'b1;
	   end
	   if (cfeb_start[3]) begin
	      cfeb_err[3] <= |(cfeb2rx_r^cfeb3rx_r);
	      cfeb3errcnt <= cfeb3errcnt + cfeb_err[3];
	      if (|cfeb3rx[23:0]) cfeb_start[8] <= 1'b1;
	      if (cfeb_err[3]) cfeb_err[8] <= 1'b1;
	   end
	   if (cfeb_start[4]) begin
	      cfeb_err[4] <= |(cfeb2rx_r^cfeb4rx_r);
	      cfeb4errcnt <= cfeb4errcnt + cfeb_err[4];
	      if (|cfeb4rx[23:0]) cfeb_start[9] <= 1'b1;
	      if (cfeb_err[4]) cfeb_err[9] <= 1'b1;
	   end

	   if(ccb_cmdstrb & !ccb_cmdstrb_r) begin
	      ccb_cmd_r <= ccb_cmd;
	      ccb_cmdstrb_r <= 1'b1;
	   end
	   else ccb_cmdstrb_r <= 1'b0;
           if (ccb_cmdstrb_r && (ccb_cmd_r[7:2]==6'h1f)) begin  // CCB "Inj.MS" CMD used to ForceError on Tx Board fibers
	      ccb_code7c <= 1'b1;
	   end
	   else ccb_code7c <= 0;
	   ccb_datstrb_r <= (ccb_datstrb & !ccb_datstrb_r);
	   if(ccb_datstrb & !ccb_datstrb_r) ccb_data_r <= ccb_data;

// things that were always in lhc_clk domain...
	   if (en_cabletests_r) cable_count <= cable_count + 1'b1;
	   cable_count_r[11:0] <= cable_count[31:20];  //  sync to fabric clock
	   en_loopbacks_rr <= en_loopbacks_r;
	   en_loopbacks_r <= en_loopbacks;
	   llout_dmbloop <= lout_dmbloop;
	   lout_dmbloop <= out_dmbloop;
	   in_dmbloop <= rawin_dmbloop;
	   if (locked & en_loopbacks_r) loop_count <= loop_count + 1'b1;
	   if (en_loopbacks) lhc_tog <= ~lhc_tog & !en_fibertests;  // this is for dmbloop[27] testing
	   if (en_loopbacks && en_loopbacks_r && en_lhcloop_r && !en_fibertests) lhc_tog_err <= (lhcloop_tog ^ lhc_tog);  // JGhere, disable this if en_fibertests is set
	   else lhc_tog_err <= 0;
	   dmbloop1_stat <= dmbloop1_stat | err_dmbloop[11:0];
	   dmbloop2_stat <= dmbloop2_stat | err_dmbloop[23:12];
	   dmbloop3_stat <= dmbloop3_stat | err_dmbloop[27:24];
	   if ( |err_dmbloop ) dmbloop_errcnt <= ((dmbloop_errcnt[11:0] + 1'b1) | (dmbloop_errcnt[11]<<11));
//    ^^^^^^^^^^^^^^^^
	   llout_ratloop <= loutpos_ratloop;
	   loutpos_ratloop <= out_ratloop;
	   in_ratloop <= rawin_ratloop; // JGhere, consider moving this to lhc90ck domain...
	   ratloop4_stat[10:0] <= ratloop4_stat[10:0] | err_ratloop[46:36];
	   ratloop3_stat[11:0] <= ratloop3_stat[11:0] | {err_ratloop[35:31], (!en_fibertests_r & err_ratloop[30]), (!en_fibertests_r & err_ratloop[29]), err_ratloop[28], (!en_fibertests_r & err_ratloop[27]), err_ratloop[26:24]}; // bits 27 & 30 get killed by step4/en_fibetests, similar for 29 due to SMB control issues
	   ratloop2_stat[11:0] <= ratloop2_stat[11:0] | err_ratloop[23:12];
	   ratloop1_stat[11:0] <= ratloop1_stat[11:0] | {err_ratloop[11:5], (en_fibertests_r & err_ratloop[4]), err_ratloop[3:0]}; // bit 4 gets killed in !en_fibertests due to SMB control issues
	   f1stat[3:0] <= {f1stat[2:0],rx_strt[1]}; // pipeline to confirm 200 ns Start signal is here (now outside en_fibertests)
	   f2stat[3:0] <= {f2stat[2:0],rx_strt[2]};
	   f3stat[3:0] <= {f3stat[2:0],rx_strt[3]};
	   f4stat[3:0] <= {f4stat[2:0],rx_strt[4]};
	   f5stat[3:0] <= {f5stat[2:0],rx_strt[5]};
	   f6stat[3:0] <= {f6stat[2:0],rx_strt[6]};
	   f7stat[3:0] <= {f7stat[2:0],rx_strt[7]};
	   f8stat[3:0] <= {f8stat[2:0],rx_strt[8]};
	   f9stat[3:0] <= {f9stat[2:0],rx_strt[9]};
	   f10stat[3:0] <= {f10stat[2:0],rx_strt[10]};
	   f11stat[3:0] <= {f11stat[2:0],rx_strt[11]};
	   f12stat[3:0] <= {f12stat[2:0],rx_strt[12]};
	   if (&f1stat & sw[8]) fiber_stat[1] <= !rx_strt[1]; // declare fiber ALIVE after ~200 ns StartPatt sequence (outside en_fibertests)
	   if (&f2stat & sw[8]) fiber_stat[2] <= !rx_strt[2];
	   if (&f3stat & sw[8]) fiber_stat[3] <= !rx_strt[3];
	   if (&f4stat & sw[8]) fiber_stat[4] <= !rx_strt[4];
	   if (&f5stat & sw[8]) fiber_stat[5] <= !rx_strt[5];
	   if (&f6stat & sw[8]) fiber_stat[6] <= !rx_strt[6];
	   if (&f7stat & sw[8]) fiber_stat[7] <= !rx_strt[7];
	   if (&f8stat & sw[8]) fiber_stat[8] <= !rx_strt[8];
	   if (&f9stat & sw[8]) fiber_stat[9] <= !rx_strt[9];
	   if (&f10stat & sw[8]) fiber_stat[10] <= !rx_strt[10];
	   if (&f11stat & sw[8]) fiber_stat[11] <= !rx_strt[11];
	   if (&f12stat & sw[8]) fiber_stat[12] <= !rx_strt[12];
	   fiber_stat_all <= ( &fiber_stat[12:1] ); // flag for all 7 fibers got the 200ns start sequence
	   
// JGhere, can add SCL & SDA0 load/read tests here:
	   if (en_fibertests) begin  // tests are disabled for step 0 & 2 (ratloop 27 & 30) so ignore
	      fiber_stat[0] <= 1'b1; // set Hi if en_fibertests, otherwise Lo
	      if (time_40r[7] & en_fibertests_r) fiber_count <= fiber_count + 1'b1;
	      fiber_count_r[11:0] <= fiber_count[31:20];  // #1.05M counts of word[0] fiber test cycles
	      error_f1count <= err_count_f[1][11:0]; // **MUST Reset/init Tx after enabling fibertests for Rx**
	      error_f2count <= err_count_f[2][11:0];
	      error_f3count <= err_count_f[3][11:0];
	      error_f4count <= err_count_f[4][11:0];
	      error_f5count <= err_count_f[5][11:0];
	      error_f6count <= err_count_f[6][11:0];
	      error_f7count <= err_count_f[7][11:0];
	      error_f8count <= err_count_f[8][11:0];
	      error_f9count <= err_count_f[9][11:0];
	      error_f10count <= err_count_f[10][11:0];
	      error_f11count <= err_count_f[11][11:0];
	      error_f12count <= err_count_f[12][11:0];

	      if (rst_errcnt) fiber_invalid <= 0;
	      else begin
		 if (fiber_stat[1] & (~|f1stat)) fiber_invalid[1] <= !rx_valid[1]; // rx_valid = tripwire for StartPatt from RcvCompFiber
		 if (fiber_stat[2] & (~|f2stat)) fiber_invalid[2] <= !rx_valid[2];
		 if (fiber_stat[3] & (~|f3stat)) fiber_invalid[3] <= !rx_valid[3];
		 if (fiber_stat[4] & (~|f4stat)) fiber_invalid[4] <= !rx_valid[4];
		 if (fiber_stat[5] & (~|f5stat)) fiber_invalid[5] <= !rx_valid[5];
		 if (fiber_stat[6] & (~|f6stat)) fiber_invalid[6] <= !rx_valid[6];
		 if (fiber_stat[7] & (~|f7stat)) fiber_invalid[7] <= !rx_valid[7];
		 if (fiber_stat[8] & (~|f8stat)) fiber_invalid[8] <= !rx_valid[8];
		 if (fiber_stat[9] & (~|f9stat)) fiber_invalid[9] <= !rx_valid[9];
		 if (fiber_stat[10] & (~|f10stat)) fiber_invalid[10] <= !rx_valid[10];
		 if (fiber_stat[11] & (~|f11stat)) fiber_invalid[11] <= !rx_valid[11];
		 if (fiber_stat[12] & (~|f12stat)) fiber_invalid[12] <= !rx_valid[12];
	         fiber_invalid_all <= ( |fiber_invalid[12:1] ); // flag if any of 12 fibers have a not-valid frame
	      end
	      if ( (|err_ratloop[26:0]) || ( err_ratloop[28]) || ( |err_ratloop[46:31]) ) ratloop_errcnt <= ((ratloop_errcnt[11:0] + 1'b1) | (ratloop_errcnt[11]<<11)); // bit 29 gets killed in en_fibertests due to SMB control issues, bits 27 & 30 due to step4 conflicts
	   end
	   else begin  // note that step tests are ON when fibertests are Off.
	      fiber_stat[0] <= 1'b0; // set Hi if en_fibertests, otherwise Lo
	      if ( (|err_ratloop[46:5]) || ( |err_ratloop[3:0]) ) ratloop_errcnt <= ((ratloop_errcnt[11:0] + 1'b1) | (ratloop_errcnt[11]<<11)); // bit 4 gets killed in !en_fibertests due to SMB control issues
	   end



// this is for our "CCB Pulsed Signal" checks: the "fired" register and a pulse counter based on in_pulse and "trigger"
	   trigger <= ( ( in_pulse_r == 12'h001)||( in_pulse_r == 12'h002)||( in_pulse_r == 12'h004)||( in_pulse_r == 12'h008)||( in_pulse_r == 12'h010)||( in_pulse_r == 12'h020)||( in_pulse_r == 12'h040)||( in_pulse_r == 12'h080)||( in_pulse_r == 12'h100)||( in_pulse_r == 12'h200)||( in_pulse_r == 12'h800)||( in_pulse_r == 12'h400) );
	   if (trigger) begin
	      pulse_count <= pulse_count + 1'b1;
	      hard_count <= hard_count + 1'b1;
	   end
// JG, check for Result Register instructions for CCB tests (CC plus CD, CE, CF or C0):
	   if (ccb_cmdstrb_r && ccb_cmd_r[7:0]==8'hCC) begin  // check all 8 bits here for "CC"
	      if (results_hold == 1'b0) begin   // first pass for CC cmd
		 tmb_cfg_out <= last_cmd[0]; // put out bit0 on the first CC cmd
// lock results immediately on CMD CC for some commands:    add  qpll_locklost etc in Data Bus check CE
		 if (last_cmd[7:0]==8'hCD) results_r <= {pulse_count[11:0],last_cmd[7:0]};    // count of CCB pulsed signals
		 else if(last_cmd[7:0]==8'hCE) results_r <= {ck160_locklost,!locked160,qpll_locklost,!qpll_lock ,ccb_data_r[7:0],last_cmd[7:0]}; // CCB data bus content
		 else if(last_cmd[7:0]==8'hCF) results_r <= {pulses_fired[11:0],last_cmd[7:0]};   // observed CCB test pulses
		 else if(last_cmd[7:0]==8'hC0) results_r <= {ccbrxzero_count[11:0],last_cmd[7:0]};   // check ccb_rx0 clocking
		 else if(last_cmd[7:0]==8'hC7) results_r <= {8'h0,dsn_val[63:60],last_cmd[7:0]};   // DSN output, MSB
		 else if(last_cmd[7:0]==8'hC6) results_r <= {dsn_val[59:48],last_cmd[7:0]};   // DSN output
		 else if(last_cmd[7:0]==8'hC5) results_r <= {dsn_val[47:36],last_cmd[7:0]};   // DSN output
		 else if(last_cmd[7:0]==8'hC4) results_r <= {dsn_val[35:24],last_cmd[7:0]};   // DSN output
		 else if(last_cmd[7:0]==8'hC3) results_r <= {dsn_val[23:12],last_cmd[7:0]};   // DSN output
		 else if(last_cmd[7:0]==8'hC2) results_r <= {dsn_val[11:0],last_cmd[7:0]};    // DSN output, LSB
		 else  results_r <= {!_ccb_rx[42],~_ccb_rx[50:48],tmb_res_out[2:0],!_ccb_rx[28],~_ccb_rx[25:22],last_cmd[7:0]};   // Use 00 or C1 for this read! it'ss also the default, sends last cmd along with a bunch of CCB signals to check, select an unused CMD to see this!  C2-7, D6-7, E6-F, F9-B
	      end

	      else begin    // CC cmd: second pass and beyond
		 tmb_cfg_out <= results_r[get_bit_ptr]; // put out bit[i] on later CC cmds
// lock test results as late as possible for some commands, at end of last_cmd register transmission:
		 if (!late_load_done && get_bit_ptr == 5'h07) begin  // the final bit of "last_cmd" field before sending "results"
		    late_load_done <= 1'b1;  // prevents re-update of Results during an over-looped series of CC cmds
		    //if ((last_cmd[7:0]&8'hf8)==8'h18) results_r <= {en_fibertests, en_cabletests, en_loopbacks,1'b0, ccb_data_r[7:0],last_cmd[7:0]}; // show "en_test" status and also CCB data bus content ---> eventually this reads "testLED switch control" setting
		    if ((last_cmd[7:0]&8'hf8)==8'h18) results_r <= {en_fibertests, en_cabletests, en_loopbacks, fiber_stat[0], ccb_data_r[7:0],last_cmd[7:0]}; // show "en_test" status and also CCB data bus content ---> eventually this reads "testLED switch control" setting

		    if(last_cmd[7:0]==8'hD0) results_r <= {dmbloop_errcnt[11:0],last_cmd[7:0]}; // total errors in 27 DMB loops, bit 11 = rollover
		    if(last_cmd[7:0]==8'hD1) results_r <= {dmbloop1_stat[11:0],last_cmd[7:0]};  // DMB loop1 signals with error
		    if(last_cmd[7:0]==8'hD2) results_r <= {dmbloop2_stat[11:0],last_cmd[7:0]};  // DMB loop2 signals with error
		    if(last_cmd[7:0]==8'hD3) results_r <= {dmbloop3_stat[11:0],last_cmd[7:0]};  // DMB loop3 signals with error
		    //                                      ^^^^ 28 bits here, only last 3:0 active for DMB loop tests
//		    if(last_cmd[7:0]==8'hD4) results_r <= {dmbloop4_stat[11:0],last_cmd[7:0]};  // DMB unused signals
		    if(last_cmd[7:0]==8'hD5) results_r <= {tmb_ck1reg[3],lhc_ckreg[3],6'h1E,hard_count[3:0],last_cmd[7:0]};  // DMB unused. Hijack for HardReset check!  Should give "DEX,LastCmd" where X is the # of pulses sent since Hard Reset

		    if(last_cmd[7:0]==8'hD8) results_r <= {ratloop_errcnt[11:0],last_cmd[7:0]}; // total errors in 47 fast RPC loops, bit 11=rollover
		    if(last_cmd[7:0]==8'hD9) results_r <= {ratloop1_stat[11:0],last_cmd[7:0]};  // RPC loop1 signals with error
		    if(last_cmd[7:0]==8'hDA) results_r <= {ratloop2_stat[11:0],last_cmd[7:0]};  // RPC loop2 signals with error
		    if(last_cmd[7:0]==8'hDB) results_r <= {ratloop3_stat[11:0],last_cmd[7:0]};  // RPC loop3 signals with error
		    if(last_cmd[7:0]==8'hDC) results_r <= {ratloop4_stat[11:0],last_cmd[7:0]};  // RPC loop4 signals with error
		    //                                      ^^^^ 47 bits here, only last 10:0 active, these are fast RPC loop tests
		    if(last_cmd[7:0]==8'hDD) results_r <= {~vstat[3],1'b0,~vstat[1:0],slowloop_stat[7:0],last_cmd[7:0]};  // RPC slowloop signals with error
		    //                                      ^^^^ 8 bits here, only last 7:0 active, these are the SLOW RPC loop tests
		    if(last_cmd[7:0]==8'hDE) results_r <= {hzloop_count[11:0],last_cmd[7:0]}; // counts VERY slow 1.5 Hz Loopback cycles
		    if(last_cmd[7:0]==8'hDF) results_r <= {slowloop_errcnt[11:0],last_cmd[7:0]}; // total errors in 8 slow RPC loops, bit 11=rollover

		    if(last_cmd[7:0]==8'hE0) results_r <= {2'b10,cfeb_start[9:0],last_cmd[7:0]};   // all skewclear cables status
		    if(last_cmd[7:0]==8'hE1) results_r <= {cfeb0errcnt[11:0],last_cmd[7:0]};  // for cable 1
		    if(last_cmd[7:0]==8'hE2) results_r <= {cfeb1errcnt[11:0],last_cmd[7:0]};  // for cable 2
		    if(last_cmd[7:0]==8'hE3) results_r <= {cfeb2errcnt[11:0],last_cmd[7:0]};  // for cable 3
		    if(last_cmd[7:0]==8'hE4) results_r <= {cfeb3errcnt[11:0],last_cmd[7:0]};  // for cable 4
		    if(last_cmd[7:0]==8'hE5) results_r <= {cfeb4errcnt[11:0],last_cmd[7:0]};  // for cable 5
		    if(last_cmd[7:0]==8'hF0) results_r <= {fiber_stat[12:1],last_cmd[7:0]};  // all fibers link + enable status
		    if(last_cmd[7:0]==8'hF1) results_r <= {error_f1count[11:0],last_cmd[7:0]};  // for fiber 1
		    if(last_cmd[7:0]==8'hF2) results_r <= {error_f2count[11:0],last_cmd[7:0]};  // for fiber 2
		    if(last_cmd[7:0]==8'hF3) results_r <= {error_f3count[11:0],last_cmd[7:0]};  // for fiber 3
		    if(last_cmd[7:0]==8'hF4) results_r <= {error_f4count[11:0],last_cmd[7:0]};  // for fiber 4
		    if(last_cmd[7:0]==8'hF5) results_r <= {error_f5count[11:0],last_cmd[7:0]};  // for fiber 5
		    if(last_cmd[7:0]==8'hF6) results_r <= {error_f6count[11:0],last_cmd[7:0]};  // for fiber 6
		    if(last_cmd[7:0]==8'hF7) results_r <= {error_f7count[11:0],last_cmd[7:0]};  // for fiber 7
                    //Tao add 5 more fibers  to test all 12 channels
		    if(last_cmd[7:0]==8'hE6) results_r <= {error_f8count[11:0],last_cmd[7:0]};  // for fiber 8
		    if(last_cmd[7:0]==8'hE7) results_r <= {error_f9count[11:0],last_cmd[7:0]};  // for fiber 9
		    if(last_cmd[7:0]==8'hE8) results_r <= {error_f10count[11:0],last_cmd[7:0]};  // for fiber 10
		    if(last_cmd[7:0]==8'hE9) results_r <= {error_f11count[11:0],last_cmd[7:0]};  // for fiber 11
		    if(last_cmd[7:0]==8'hEA) results_r <= {error_f12count[11:0],last_cmd[7:0]};  // for fiber 12
		    if(last_cmd[7:0]==8'hF8) results_r <= {fiber_invalid[12:1],last_cmd[7:0]};  // all fibers invalid trips
		    if(last_cmd[7:0]==8'hFC) results_r <= {cable_count[11:0],last_cmd[7:0]};   // count of cable test cycles
		    if(last_cmd[7:0]==8'hFD) results_r <= {loop_count[31:20],last_cmd[7:0]}; // #1.05M counts @40MHz Loopback test cycles
		    if(last_cmd[7:0]==8'hFE) results_r <= {slowloop_count[26:15],last_cmd[7:0]}; // #32K counts @1.6 MHz Loopback cycles
		    if(last_cmd[7:0]==8'hFF) results_r <= {fiber_count_r[11:0],last_cmd[7:0]};   // #1.05M counts of word[0] fiber test cycles
		 end
		 
	      end

// end of each CC cmd:
	      if (get_bit_ptr == 5'd19) begin
		 alct_cfg_out <= 1'b1;
		 get_bit_ptr <= 5'h00;
	      end
	      else alct_cfg_out <= 1'b0;

	      results_hold <= 1'b1;
	      get_bit_ptr <= get_bit_ptr + 1'b1;
	   end

	   else if (ccb_cmdstrb_r) begin   // handle CMDs that actually DO something... not CC, clear all CC registers
	      get_bit_ptr <= 5'h00;
	      results_hold <= 1'b0;
	      late_load_done <= 1'b0;  // allows update of Results just once during a series of CC cmds
	      tmb_cfg_out <= ccb_cmd_r[0];
	      alct_cfg_out <= 1'b0;

	      last_cmd <= ccb_cmd_r;
// here we must decode what the CMDs actually DO... e.g. Controls for TMB and the slave boards
              if (ccb_cmd_r[7:2]==6'h07) begin  // this is the CCB "Stop Trig" CMD, use it to freeze tests
	         if (ccb_cmd_r[1:0]==2'h1) en_loopbacks <= 1'b0;  // stop DMB & RPC loopback tests
	         else if (ccb_cmd_r[1:0]==2'h2) en_cabletests <= 1'b0;  // stop skewclear pattern tests
	         else if (ccb_cmd_r[1:0]==2'h3) en_fibertests <= 1'b0;  // stop fiber pattern tests
	         else begin  // disable all tests by default on 2'h0?  Give datastrb the same function?
		    en_loopbacks <= 1'b0;
		    en_cabletests <= 1'b0;
		    en_fibertests <= 1'b0;
		 end
	      end

//   before "Start" of tests, might need to send L1 Reset... let software control that!
              else if (ccb_cmd_r[7:2]==6'h06) begin  // this is the CCB "Start Trig" CMD, use it to start tests
		 if (ccb_cmd_r[1:0]==2'h1) en_loopbacks <= 1'b1;  // start DMB & RPC loopback tests
	         else if (ccb_cmd_r[1:0]==2'h2) en_cabletests <= 1'b1;  // start skewclear pattern tests
	         else if (ccb_cmd_r[1:0]==2'h3) en_fibertests <= 1'b1;  // start fiber pattern tests
	         else begin  // enable all tests by default on 2'h0?  Give datastrb the same function?
		    en_loopbacks <= 1'b1;
		    en_cabletests <= 1'b1;
		    en_fibertests <= 1'b1;
		 end
	      end
	   end // else if (ccb_cmdstrb_r)

	end // else: !if(reset)
     end // always @ (posedge lhc_clk or posedge reset)


   assign _ccb_tx[19] = alct_cfg_out;  // JG, don't invert
   assign _ccb_tx[18] = tmb_cfg_out;   // JG, don't invert
   assign _ccb_tx[17:9] = pulse_count[8:0];  // 9 bit alct_status, to CCB Front Panel (our LED plug)
   

   always @(posedge qpll_ck40) // everything that uses RAW qpll_40 clock (pre-MMCM lhc_clk), no Reset
     begin
	time_r_i  <= time_count;
     end

   always @(posedge qpll_ck40 or posedge reset) // everything using RAW qpll_40 clock w/simple Reset
     begin
	if (reset) begin
	   l_locked160 <= 0;    // use qpll_ck40 to monitor ck40(160)
	   ck160_locklost <= 0;
	   time_r <= 8'h00;
	end
	else begin
	   if (time_r[7] & locked160) l_locked160 <= 1; // wait 80 usec after reset_done state to begin
	   if (l_locked160 & (!locked160)) ck160_locklost <= 1;
	   time_r <= time_r_i;  // transfer slow slwclk time counter to qpll_40 clock domain
	end
     end



/*  switch modes
   sw7 & sw8:   send err_count at 1.6 Hz; PB forces an error  (use w/Tx tester, OK at TAMU)
   !sw7 & sw8:  send err_count at 1.6 Hz; PB is Reset  (good to use at OSU & w/tester at TAMU)

   sw7 & !sw8:  send comp_data when non-zero; PB forces a triad pattern  (for Tx tester at TAMU)
   !sw7 & !sw8: send comp_data when non-zero; PB is Reset  (good to use at OSU & w/tester at TAMU)
*/
   always @(*)
     begin
	led_low[0] = !time_40r[7] | !lhc_locked | reset; // -OFF
	led_low[1] = locked;    // -USUALLY! ON  (sometime not Mezz#12)
	led_low[2] = locked160; // -ON
	led_low[3] = qpll_lock;     // always ON!
	led_low[4] = qpll_locklost; // -OFF
	led_low[5] = ck160_locked;  // always ON!  // Tx GTX PLL Ref lock
	led_low[6] = ck160_locklost;// -OFF
	led_low[7] = (&synced_snapr & en_fibertests); // -OFF/ON with en_fibertests?

	if (!sw[7]) begin
	   if (sw[8] == 1) begin // sw8 ON, 7 OFF:  send err_count at 1.6 Hz; PB is Reset  (good to use at OSU)
	      led_hi[8] = !reset;  // Mezz1:ON.  --> off, but ON during reset
	      led_hi[9] = !gtx_reset; // M1: Good.   == (!gtx_ready | reset)
 	      led_hi[10] = sda0;  // SD back from DDD chips
	      led_hi[11] = reset; // --FREE--
	      led_hi[12] = qpll_ck40; // M1: Good.  was gbe_fok --> ON
	      led_hi[13] = locked;    // M1: ON.  --> off
	      led_hi[14] = ck160_locked;  //  M1: OFF.   --> on sometimes  // Tx GTX PLL Ref lock
	      led_hi[15] = locked160;    // M1: OFF.     --> on ~always // from Tx GTX PLL out clk
	      test_led[1] = (&rx_match);// 
	      test_led[2] = lhc_clk;    // * used to check lhc_clk freq.
	      test_led[3] = (|rx_fc);   // M1+M11: 3.2usec between 25ns pulses.  == LTNCY_TRIG
	      test_led[4] = fiber_invalid_all; // flag if any of 7 fibers have a not-valid frame
	      test_led[5] = fiber_stat_all;   // all fibers got the 200ns start sequence
// for each dcfeb fiber, latch the START pulse:  catch 4 in a row, then latch when it goes Low.
	      test_led[6] = (&rx_strt);   // M1+M11: always LO; 200/225ns pulse after RST.  CHECK.
// for each dcfeb fiber, latch any NOT valid?
	      test_led[7] = (&rx_valid);  // M1+M11: always HI; Lo @RST.  CHECK.
	      test_led[8] = (|rx_strt); //  --FREE--
	      test_led[9] = (|rx_valid); //  --FREE--
	      test_led[10] = (comp_data[1] == 48'hFFFFFF000000);//  --FREE--
//	      test_led[8] = bc0;    //
//	      test_led[9] = ccb_code7c; //
//	      test_led[10]  = reset;    //
	   end

	   else begin            // Both OFF: send comp_dout when non-zero; PB is Reset  (good to use at OSU)
	      led_hi[8] = !reset;  //  --> off, but ON during reset
	      led_hi[9] = !gtx_reset;  //  == (!gtx_ready | reset)  --> off, but ON during reset
 	      led_hi[10] = time_r[7];  // * off, but ON during reset
	      led_hi[11] = reset;  //  --FREE--
	      led_hi[12] = qpll_ck40;  //  was gbe_fok --> ON
	      led_hi[13] = locked;     //  --> off
	      led_hi[14] = ck160_locked; //  --> off  // Tx GTX PLL Ref lock
	      led_hi[15] = locked160;    //  --> off // from Tx GTX PLL out clk
// add rpc_loop3stat[5] debug here...  err_ratloop[29]  
	      test_led[1] = |nzdat[1]; //  --FREE--
	      test_led[2] = |nzdat[2]; //  --FREE--
	      test_led[3] = scl[0]; // SDin for DDD chips
	      //test_led[4] = ck40;       // used to check ck40 freq. from snap gtx rx_clk
	      test_led[4] = qpll_ck40;       // used to check ck40 freq. from snap gtx rx_clk
	      test_led[5] = |nzdat[3];    //  --FREE--
	      test_led[6] = sda0;  // SDout back from DDD chips
	      test_led[7] = |nzdat[4];//  --FREE--
	      test_led[8] = |nzdat[5]; //  --FREE--
	      test_led[9] = |nzdat[6]; //  --FREE--
	      test_led[10] = |nzdat[7];//  --FREE--

/*	      test_led[8:1]   = rxdvr_snap[7:0];
	      test_led[9] = snap_wait; //

  	      test_led[8:1] = l_gbe_rxdat[7:0]; // counter[7:0];
	      test_led[9]   = kchar_r[0]; // gbe_fok;
 */
	   end
	end
// JGnew for LEDs:   dsn_in, drive_low, rx_fc[6:0], dsn_state[3:0], scount_rst, dsn_val
	else if (sw[8] == 0) begin  // *** sw7 ON from here on: send comp_dout when non-zero; PB forces a triad pattern
	   //	                               (for Tx tester at TAMU)
// Mezz LEDs are inverted...TMB FP LEDs are not...test_leds are not.
  	   led_hi[14:8]  = ~d_reg[6:0]; // show Samir's data if ad_reg[2|1] is true
	   led_hi[15] = ~|ksynclost;
	   test_led[1] = dsn_in;  //  --FREE--
	   test_led[2] = dsn_state[0];  //  --FREE--
	   test_led[3] = dsn_state[1];  //  --FREE--
	   test_led[4] = scount_rst;  //  --FREE--
	   test_led[5] = dsn_state[2];  //  --FREE--
	   test_led[6] = dsn_state[3];  //  --FREE--
	   test_led[7] = drive_low;  //  --FREE--
	   test_led[8] = dsn_val[0]; //  --FREE--
	   test_led[9] = ad_reg[1];  // gets a single 25ns pulse high
	   test_led[10] = ad_reg[2]; // gets a single 25ns pulse high
	end    // if (sw[8] == 0)
	else begin  // Both ON: send err_count at 1.6 Hz; PB forces an error (use w/Tx tester at TAMU)
  	   led_hi[8]  = lhc_clk; // should be dim
	   led_hi[15:9]  = ~o_reg[6:0]; // the "other" d_reg
	   test_led[1] = rx_fc[1];  // 
	   test_led[2] = rx_fc[2];  // 
	   test_led[3] = rx_fc[3];  // 
	   test_led[4] = rx_fc[4];  // 
	   test_led[5] = rx_fc[5];  // 
	   test_led[6] = rx_fc[6];  // 
	   test_led[7] = rx_fc[7];  // 
	   test_led[8] = nzdat[7][1]; // v5: always off   v6: gets a single 25ns pulse high
	   test_led[9] = nzdat[7][2]; // v5: gets a single 25ns pulse high   v6: always off
	   test_led[10] = nzdat[7][3];// v5: gets a single 25ns pulse high   v6: always off
	end
     end

   
//   mmcm from 80 MHz:         In,    out80,  out160,   out40, reset,         locked
   //bufg_x2div2 snap_mmcm (tx_clk_out, tx_clk, snap_clk2, ck40, !ck160_locked, locked160); // from Tx GTX PLL out clk

// JGhere:  was Snap12 module, not cfeb-tmb test code
   //tmb_fiber_out  dcfeb_out (
   //     .RST (reset),   // Manual only
   //     .TRG_SIGDET (), // from IPAD to IBUF.  N/A?
   //     .TRG_RX_N (),   // empty
   //     .TRG_RX_P (),   // empty
   //     .TRG_TDIS (),   // OBUF output, for what?  N/A?
   //     .TRG_TX_N (txn[1]),   // pick a fiber, match LOC constraint in module
   //     .TRG_TX_P (txp[1]),   // pick a fiber
   //     .G1C (triad_word_r[7:0]),  // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
   //     .G2C (triad_word_r[15:8]),  //   if ENA_TEST_PAT then it's prbs so these don't matter...
   //     .G3C (triad_word_r[23:16]),  //   unless I want to test low-rate non-zero triad data:
   //     .G4C (triad_word_r[7:0]),
   //     .G5C (triad_word_r[15:8]),
   //     .G6C (triad_word_r[23:16]),
   //     .TRG_TX_REFCLK (ck160),  // QPLL 160 from MGT clk
   //     .TRG_TXUSRCLK (snap_clk2),  // get 160 from TXOUTCLK (times 2)
   //     .TRG_CLK80 (tx_clk),     // get 80 from TXOUTCLK
   //     .TRG_GTXTXRST (0),   // maybe Manual "reset" only
   //     .TRG_TX_PLLRST (0),  //  Tie LOW.
   //     .TRG_RST (gtx_reset),       // gtx_reset = PBrst | !RxSyncDone
   //     .ENA_TEST_PAT (sw[8]),   // HIGH for PRBS!  (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
   //     .INJ_ERR (ferr_rr & sw[8]),  // use my switch/PB combo logic for this, high-true? Pulse high once.
   //     .TRG_SD (),          // from IBUF, useless output. N/A
   //     .TRG_TXOUTCLK (tx_clk_out),   // 80 MHz; This has to go to MCM to generate 160/80
   //     .TRG_TX_PLL_LOCK (ck160_locked), // inverse holds the MCM in Reset  // Tx GTX PLL Ref lock
   //     .TRG_TXRESETDONE (), // N/A
   //     .TX_SYNC_DONE (synced_snapt[1]), // NOT used... inverse could hold logic in Reset someplace, via gtx_ready?
   //     .STRT_LTNCY (tx_begin),  // after every Reset, to TP for debug only  -- !sw7 ?
   //     .LTNCY_TRIG (tx_fc),     // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
   //     .MON_TX_SEL (),      //  N/A
   //     .MON_TRG_TX_ISK (),  //  N/A returns 4 bits
   //     .MON_TRG_TX_DATA ()  //  N/A returns 32 bits
   //     );

    genvar r;  //  need to generate receive logic for comparator fibers 1-12.  remember to Swap Rx Polarity for 5 & 6.
    generate
       for (r=1; r<=Nfibers; r=r+1) begin:rcv_comp_gen
	  rcv_compfiber  #(.iFIBER(r)) comp_in ( rxn_12[r], rxp_12[r], ck160, lhc_clk, ck160_locked, reset, gtx_reset, rst_errcnt,
				  time_40r[7], sw[8], en_fibertests, (r==5 || r==6),
				  rx_strt[r], rx_fc[r], rx_valid[r], rx_match[r], synced_snapr[r],
				  compfifo_dav[r], compfifo_overflow[r], err_f[r], err_count_f[r],
				  comp_data[r], compfifo_data[r], rx_pll_locked[r], ksync[r], ksynclost[r], nzdat[r] );
       end
    endgenerate



/*
 module rcv_compfiber(
    input  rxn, rxp,
    input  ref_clk, fabric_clk, ref_locked,
    input  reset, gtx_reset, rst_errcount, startup_ready, en_prbstest, en_dataread,
    output reg rx_start_r, rx_fc_r, rx_valid_r, rx_match_r, synced_snapr_r,   // bring to fabric domain!
    output reg compfifo_dav_r, compfifo_overflow_r, err_r,   // bring to fabric domain!
    output reg [15:0] err_count_r,  // bring to fabric domain!
    output reg [47:0] comp_dout,  // bring to fabric domain!
    output [47:0] compfifo_dout
   )
// rx module reqs, IN:  rxp/n[i], reset, gtx_reset, refclk=ck160, lhc_clk, ck160_locked,
//                      time_40r[7], rst_errcount, "sw[8]"=en_PRBStest?
// rx module reqs, OUT: comp_dat_out[i], err[i], err_count[i], rx_strt[i], rx_fc[i], rx_valid[i], rx_match[i]?
*/

// rx module reqs, IN:  reset, lhc_clk, time_40r[7], rst_errcnt, "sw[8]"=en_PRBStest?  snap_wait[i]?
// rx module reqs, OUT: comp_dat_out[i], err[i], err_count[i]

endmodule // mezz2012




// Lower-level modules HERE (in lieu of includes)



module bufg_div8p5clk
 (// Clock in ports
  input         CLK_IN1,
  input         RESET,
  // Clock out ports
  output        CLK_OUT1,  // [40 MHz -> 4.875 MHz]
  // Status and control signals
  output        INPUT_CLK_STOPPED,
  output        LOCKED
 );

/*
  // Input buffering
  //------------------------------------
  BUFG clkin1_buf
   (.O (clkin1),
    .I (CLK_IN1));
*/
   assign clkin1 = CLK_IN1;

  // Clocking primitive
  //------------------------------------
  // Instantiation of the MMCM primitive
  //    * Unused inputs are tied off
  //    * Unused outputs are labeled unused
  wire [15:0] do_unused;
  wire        drdy_unused;
  wire        psdone_unused;
  wire        clkfbout;
  wire        clkfbout_buf;
  wire        clkfboutb_unused;
  wire        clkout0b_unused;
  wire        clkout1_unused;
  wire        clkout1b_unused;
  wire        clkout2_unused;
  wire        clkout2b_unused;
  wire        clkout3_unused;
  wire        clkout3b_unused;
  wire        clkout4_unused;
  wire        clkout5_unused;
  wire        clkout6_unused;
  wire        clkfbstopped_unused;

  MMCM_ADV
  #(.BANDWIDTH            ("OPTIMIZED"),
    .CLKOUT4_CASCADE      ("FALSE"),
    .CLOCK_HOLD           ("FALSE"),
    .COMPENSATION         ("ZHOLD"),
    .STARTUP_WAIT         ("FALSE"),
    .DIVCLK_DIVIDE        (4),       // JRG was 10, reduce. allowed range is 1-80 (integers)
    .CLKFBOUT_MULT_F      (60.000),  // JRG was 51.000, increase. allowed range: 5-64 (integers)
    .CLKFBOUT_PHASE       (0.000),
    .CLKFBOUT_USE_FINE_PS ("FALSE"),
    .CLKOUT0_DIVIDE_F     (128.000), // JRG was 127.500, increase?  allowed range: 2-128, 0.125 increments (integer only for out[1:6])
    .CLKOUT0_PHASE        (0.000),
    .CLKOUT0_DUTY_CYCLE   (0.500),
    .CLKOUT0_USE_FINE_PS  ("FALSE"),
    .CLKIN1_PERIOD        (25.0),
    .REF_JITTER1          (0.010))
  mmcm_adv_inst
    // Output clocks
   (.CLKFBOUT            (clkfbout),
    .CLKFBOUTB           (clkfboutb_unused),
    .CLKOUT0             (clkout0),
    .CLKOUT0B            (clkout0b_unused),
    .CLKOUT1             (clkout1_unused),
    .CLKOUT1B            (clkout1b_unused),
    .CLKOUT2             (clkout2_unused),
    .CLKOUT2B            (clkout2b_unused),
    .CLKOUT3             (clkout3_unused),
    .CLKOUT3B            (clkout3b_unused),
    .CLKOUT4             (clkout4_unused),
    .CLKOUT5             (clkout5_unused),
    .CLKOUT6             (clkout6_unused),
     // Input clock control
    .CLKFBIN             (clkfbout_buf),
    .CLKIN1              (clkin1),
    .CLKIN2              (1'b0),
     // Tied to always select the primary input clock
    .CLKINSEL            (1'b1),
    // Ports for dynamic reconfiguration
    .DADDR               (7'h0),
    .DCLK                (1'b0),
    .DEN                 (1'b0),
    .DI                  (16'h0),
    .DO                  (do_unused),
    .DRDY                (drdy_unused),
    .DWE                 (1'b0),
    // Ports for dynamic phase shift
    .PSCLK               (1'b0),
    .PSEN                (1'b0),
    .PSINCDEC            (1'b0),
    .PSDONE              (psdone_unused),
    // Other control and status signals
    .LOCKED              (LOCKED),
    .CLKINSTOPPED        (INPUT_CLK_STOPPED),
    .CLKFBSTOPPED        (clkfbstopped_unused),
    .PWRDWN              (1'b0),
    .RST                 (RESET));

  // Output buffering
  //-----------------------------------
  BUFG clkf_buf
   (.O (clkfbout_buf),
    .I (clkfbout));


  BUFG clkout1_buf
   (.O   (CLK_OUT1),
    .I   (clkout0));
endmodule





///////////////////////////////////////////////////////////////////////////////
//   ____  ____
//  /   /\/   /
// /___/  \  /    Vendor: Xilinx 
// \   \   \/     Version : 1.8
//  \   \         Application : Virtex-6 FPGA GTX Transceiver Wizard
//  /   /         Filename : double_reset.v
// /___/   /\           --workaround from core_project examples--
// \   \  /  \ 
//  \___\/\___\
//
//
// Module DOUBLE_RESET
// Generated by Xilinx Virtex-6 FPGA GTX Transceiver Wizard
// 
`define DLY #1

//    --workaround from core_project examples--
module DOUBLE_RESET
(
	input  	CLK,
	input  	PLLLKDET,
        output  GTXTEST_DONE,
	output 	GTXTEST_BIT1
);

   reg        	    plllkdet_sync;
   reg              plllkdet_r;
   reg 	  [10:0]    reset_dly_ctr;
   reg        	    reset_dly_done;
   reg    [3:0]	    testdone_f;


	always @(posedge CLK)
	begin
   	  plllkdet_r    	<= `DLY PLLLKDET;
   	  plllkdet_sync 	<= `DLY plllkdet_r;
   	end

	assign GTXTEST_BIT1  = reset_dly_done; 
        assign GTXTEST_DONE  = (reset_dly_ctr == 11'd0)? testdone_f[0] : 1'b0;

	always @(posedge CLK)
        begin
    	   if (!plllkdet_sync) 
              reset_dly_ctr 	<= `DLY 11'h7FF;
    	   else if (reset_dly_ctr != 11'h000)
              reset_dly_ctr 	<= `DLY reset_dly_ctr - 1'b1;
        end

	always @(posedge CLK)
        begin
    	   if (!plllkdet_sync) 
              reset_dly_done 	<= `DLY 1'b0;
    	   else if (reset_dly_ctr[10] == 1'b0) 
              reset_dly_done 	<= `DLY reset_dly_ctr[8];
        end

	always @(posedge CLK)
        begin
     	   if (reset_dly_ctr != 11'd0)
       	      testdone_f     	<= `DLY 4'b1111;
           else
              testdone_f     	<= `DLY {1'b0, testdone_f[3:1]};
        end

endmodule



///////////////////////////////////////////////////////////////////////////////
//   ____  ____
//  /   /\/   / 
// /___/  \  /    Vendor: Xilinx 
// \   \   \/     Version : 1.8
//  \   \         Application :  Virtex-6 FPGA GTX Transceiver Wizard
//  /   /         Filename : mgt_usrclk_source_mmcm.v
// /___/   /\     Timestamp : 
// \   \  /  \ 
//  \___\/\___\ 
//
//
// Module MGT_USRCLK_SOURCE (for use with GTX Transceivers)
// Generated by Xilinx Virtex-6 FPGA GTX Transceiver Wizard
// 
// 
// (c) Copyright 2009-2010 Xilinx, Inc. All rights reserved.
// 
// This file contains confidential and proprietary information
// of Xilinx, Inc. and is protected under U.S. and
// international copyright and other intellectual property
// laws.
// 
// DISCLAIMER
// This disclaimer is not a license and does not grant any
// rights to the materials distributed herewith. Except as
// otherwise provided in a valid license issued to you by
// Xilinx, and to the maximum extent permitted by applicable
// law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
// WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
// AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
// BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
// INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
// (2) Xilinx shall not be liable (whether in contract or tort,
// including negligence, or under any other theory of
// liability) for any loss or damage of any kind or nature
// related to, arising under or in connection with these
// materials, including for any direct, or any indirect,
// special, incidental, or consequential loss or damage
// (including loss of data, profits, goodwill, or any type of
// loss or damage suffered as a result of any action brought
// by a third party) even if such damage or loss was
// reasonably foreseeable or Xilinx had been advised of the
// possibility of the same.
// 
// CRITICAL APPLICATIONS
// Xilinx products are not designed or intended to be fail-
// safe, or for use in any application requiring fail-safe
// performance, such as life-support or safety devices or
// systems, Class III medical devices, nuclear facilities,
// applications related to the deployment of airbags, or any
// other applications that could lead to death, personal
// injury, or severe property or environmental damage
// (individually and collectively, "Critical
// Applications"). Customer assumes the sole risk and
// liability of any use of Xilinx products in Critical
// Applications, subject only to applicable laws and
// regulations governing limitations on product liability.
// 
// THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
// PART OF THIS FILE AT ALL TIMES. 


`timescale 1ns / 1ps

//***********************************Entity Declaration*******************************
module MGT_USRCLK_SOURCE_MMCM #
(
    parameter   MULT            =   2,
    parameter   DIVIDE          =   2,
    parameter   CLK_PERIOD      =   6.4,
    parameter   OUT0_DIVIDE     =   2,
    parameter   OUT1_DIVIDE     =   2,
    parameter   OUT2_DIVIDE     =   2,
    parameter   OUT3_DIVIDE     =   2   
)
(
    output          CLK0_OUT,
    output          CLK1_OUT,
    output          CLK2_OUT,
    output          CLK3_OUT,
    input           CLK_IN,
    output          MMCM_LOCKED_OUT,
    input           MMCM_RESET_IN
);


`define DLY #1

//*********************************Wire Declarations**********************************

    wire    [15:0]  tied_to_ground_vec_i;
    wire            tied_to_ground_i;
    wire            clkout0_i;
    wire            clkout1_i;
    wire            clkout2_i;
    wire            clkout3_i;
    wire            clkfbout_i;

//*********************************** Beginning of Code *******************************

    //  Static signal Assigments    
    assign tied_to_ground_i             = 1'b0;
    assign tied_to_ground_vec_i         = 16'h0000;

    // Instantiate a MMCM module to divide the reference clock. Uses internal feedback
    // for improved jitter performance, and to avoid consuming an additional BUFG
    MMCM_ADV #
    (
         .COMPENSATION      ("ZHOLD"),
         .CLKFBOUT_MULT_F   (MULT),
         .DIVCLK_DIVIDE     (DIVIDE),
         .CLKFBOUT_PHASE    (0),
         
         .CLKIN1_PERIOD     (CLK_PERIOD),
         .CLKIN2_PERIOD     (10),   //Not used
         
         .CLKOUT0_DIVIDE_F  (OUT0_DIVIDE),
         .CLKOUT0_PHASE     (0),
         
         .CLKOUT1_DIVIDE    (OUT1_DIVIDE),
         .CLKOUT1_PHASE     (0),

         .CLKOUT2_DIVIDE    (OUT2_DIVIDE),
         .CLKOUT2_PHASE     (0),
         
         .CLKOUT3_DIVIDE    (OUT3_DIVIDE),
         .CLKOUT3_PHASE     (0),
         .CLOCK_HOLD        ("TRUE")        
    )
    mmcm_adv_i   
    (
         .CLKIN1            (CLK_IN),
         .CLKIN2            (1'b0),
         .CLKINSEL          (1'b1),
         .CLKFBIN           (clkfbout_i),
         .CLKOUT0           (clkout0_i),
         .CLKOUT0B          (),
         .CLKOUT1           (clkout1_i),
         .CLKOUT1B          (),         
         .CLKOUT2           (clkout2_i),
         .CLKOUT2B          (),         
         .CLKOUT3           (clkout3_i),
         .CLKOUT3B          (),         
         .CLKOUT4           (),
         .CLKOUT5           (),
         .CLKOUT6           (),
         .CLKFBOUT          (clkfbout_i),
         .CLKFBOUTB         (),
         .CLKFBSTOPPED      (),
         .CLKINSTOPPED      (),
         .DO                (),
         .DRDY              (),
         .DADDR             (7'd0),
         .DCLK              (1'b0),
         .DEN               (1'b0),
         .DI                (16'd0),
         .DWE               (1'b0),
         .LOCKED            (MMCM_LOCKED_OUT),
         .PSCLK             (1'b0),
         .PSEN              (1'b0),         
         .PSINCDEC          (1'b0), 
         .PSDONE            (),         
         .PWRDWN            (1'b0),         
         .RST               (MMCM_RESET_IN)
    );
    
    BUFG clkout0_bufg_i  
    (
        .O              (CLK0_OUT), 
        .I              (clkout0_i)
    ); 


    BUFG clkout1_bufg_i
    (
        .O              (CLK1_OUT),
        .I              (clkout1_i)
    );


    BUFG clkout2_bufg_i 
    (
        .O              (CLK2_OUT),
        .I              (clkout2_i)
    );
    
    
    BUFG clkout3_bufg_i
    (
        .O              (CLK3_OUT),
        .I              (clkout3_i)
    );    

endmodule




///////////////////////////////////////////////////////////////////////////////
//   ____  ____
//  /   /\/   /
// /___/  \  /    Vendor: Xilinx 
// \   \   \/     Version : 1.8
//  \   \         Application : Virtex-6 FPGA GTX Transceiver Wizard
//  /   /         Filename : tx_sync.v
// /___/   /\     Timestamp : 
// \   \  /  \ 
//  \___\/\___\
//
//
// Module TX_SYNC
// Generated by Xilinx Virtex-6 FPGA GTX Transceiver Wizard
// (c) Copyright 2009-2010 Xilinx, Inc. All rights reserved.
`timescale 1ns / 1ps
`define DLY #1

module TX_SYNC #
(
    parameter       SIM_TXPMASETPHASE_SPEEDUP   = 0
)
(
    output          TXENPMAPHASEALIGN,
    output          TXPMASETPHASE,
    output          TXDLYALIGNDISABLE,
    output          TXDLYALIGNRESET,
    output          SYNC_DONE,
    input           USER_CLK,
    input           RESET
);


//*******************************Register Declarations************************

    reg            begin_r;
    reg            phase_align_r;
    reg            ready_r;
    reg   [15:0]   sync_counter_r;
    reg   [5:0]    wait_before_setphase_counter_r;
    reg   [4:0]    align_reset_counter_r;
    reg            align_reset_r;
    reg            wait_before_setphase_r;
    
//*******************************Wire Declarations****************************
    
    wire           count_setphase_complete_r;
    wire           count_32_complete_r;
    wire           count_align_reset_complete_r;
    wire           next_phase_align_c;
    wire           next_ready_c;
    wire           next_align_reset_c;
    wire           next_wait_before_setphase_c;

//*******************************Main Body of Code****************************

    //________________________________ State machine __________________________
    // This state machine manages the TX phase alignment procedure of the GTX.
    // The module is held in reset till TXRESETDONE is asserted. Once TXRESETDONE 
    // is asserted, the state machine goes into the align_reset_r state, asserting
    // TXDLYALIGNRESET for 20 TXUSRCLK2 cycles. After this, it goes into the 
    // wait_before_setphase_r state for 32 cycles. After asserting TXENPMAPHASEALIGN and 
    // waiting 32 cycles, it goes into the phase_align_r state where the last 
    // part of the alignment procedure is completed. This involves asserting 
    // TXPMASETPHASE for 8192 (TXPLL_DIVSEL_OUT=1), 16384 (TXPLL_DIVSEL_OUT=2), 
    // or 32768 (TXPLL_DIVSEL_OUT=4) clock cycles. After completion of the phase 
    // alignment procedure, TXDLYALIGNDISABLE is deasserted.
    
    // State registers
    always @(posedge USER_CLK)
        if(RESET)
            {begin_r,align_reset_r,wait_before_setphase_r,phase_align_r,ready_r}  <=  `DLY    5'b10000;
        else
        begin
            begin_r                <=  `DLY    1'b0;
            align_reset_r          <=  `DLY    next_align_reset_c;
            wait_before_setphase_r <=  `DLY    next_wait_before_setphase_c;
            phase_align_r          <=  `DLY    next_phase_align_c;
            ready_r                <=  `DLY    next_ready_c;
        end

    // Next state logic    
    assign  next_align_reset_c          =   begin_r |
                                            (align_reset_r & !count_align_reset_complete_r);
    
    assign  next_wait_before_setphase_c =   (align_reset_r & count_align_reset_complete_r) |
                                            (wait_before_setphase_r & !count_32_complete_r);
                                        
    assign  next_phase_align_c          =   (wait_before_setphase_r & count_32_complete_r) |
                                            (phase_align_r & !count_setphase_complete_r);
                                        
    assign  next_ready_c                =   (phase_align_r & count_setphase_complete_r) |
                                            ready_r;

    //______ Counter for holding TXDLYALIGNRESET for 20 TXUSRCLK2 cycles ______
    always @(posedge USER_CLK)
    begin
        if (!align_reset_r)
            align_reset_counter_r <= `DLY 5'b00000;
        else
            align_reset_counter_r <= `DLY align_reset_counter_r +1'b1;
    end
    
    assign count_align_reset_complete_r = align_reset_counter_r[4]
                                        & align_reset_counter_r[2];

    //_______ Counter for waiting 32 clock cycles before TXPMASETPHASE ________
    always @(posedge USER_CLK)
    begin
        if (!wait_before_setphase_r)
           wait_before_setphase_counter_r  <= `DLY  6'b000000;
        else
           wait_before_setphase_counter_r  <= `DLY  wait_before_setphase_counter_r + 1'b1;
    end

    assign count_32_complete_r = wait_before_setphase_counter_r[5];

    //_______________ Counter for holding SYNC for SYNC_CYCLES ________________
    always @(posedge USER_CLK)
    begin
        if (!phase_align_r)
            sync_counter_r <= `DLY  16'h0000;
        else
            sync_counter_r <= `DLY  sync_counter_r + 1'b1;
    end

generate
if(SIM_TXPMASETPHASE_SPEEDUP==1)
begin:fast_simulation
    // 64 cycles of setphase for simulation 
    assign count_setphase_complete_r = sync_counter_r[6];
end
else
begin:no_fast_simulation
    // 8192 cycles of setphase for output divider of 1
    assign count_setphase_complete_r = sync_counter_r[13];
end
endgenerate

    //_______________ Assign the phase align ports into the GTX _______________

    assign TXDLYALIGNRESET   = align_reset_r;
    assign TXENPMAPHASEALIGN = !begin_r & !align_reset_r;
    assign TXPMASETPHASE     = phase_align_r;
    assign TXDLYALIGNDISABLE = !ready_r;

    //_______________________ Assign the sync_done port _______________________
    
    assign SYNC_DONE = ready_r;
    
    
endmodule



//
// PRBS39_TEST 4/20/11
//-----------------------------------------------------------------------------
// 39-bit PRBS module, refs:
//   http://www.eng.auburn.edu/~strouce/polycorrect2.pdf
//   http://www.xilinx.com/support/documentation/application_notes/xapp052.pdf
//   http://www.eecs.berkeley.edu/~newton/Classes/CS150sp97/labs/lab5/lab5.html#primPolynomials
//   http://en.wikipedia.org/wiki/Linear_feedback_shift_register
//   http://www.springerlink.com/content/u590n42r72568803/
// Note that for any primitive polynomial, the reverse polynomial is also primitive!
//-----------------------------------------------------------------------------

module prbs39_test(
	input [38:0] init_dat,  // ==0 will disable the circuit (set constant)
	input        en,    // en allows data to go out and checks on the return
	input        in,    // this is the data bit received, Tpd < 10 ns?  Fmax=50Mhz
	output       out,   // this is the data bit to send out, on pos-clk
	output reg   good, err,   // these indicate a match/mismatch was detected
	output reg [15:0]  count, // this is a 16-bit counter for # of errors
        input  force_error,  // this needs to trigger a  _single_  bit-flip
	input        rst,
	input        clk);  // assume 10-20 MHz operation...NOW 40 Mhz!  later 80?

        reg 	    in_r, out_r, out_rr, out_3r, ferr_r, ferr_rr, ferr_done;
        reg  [3:0]  en_r;
        wire 	    rawout;
        assign out = rawout^ferr_rr;

        prbs39 randombit(init_dat, en, rawout, rst, clk);
	always @(posedge clk or posedge rst) begin
	   if(rst) begin
	      en_r <= 0;
	      count  <= 0;
	      ferr_r <= 0;
	      ferr_rr <= 0;	      
	      in_r <= 0;
	      out_r <= 0;
	      out_rr <= 0;
	      out_3r <= 0;
	      err <= 0;
	      good <= 0;
	   end

	   else begin
	      in_r <= in;
	      out_3r <= out_rr;
	      out_rr <= out_r^ferr_rr;
	      out_r <= rawout;
	      if (init_dat != 39'h0000000000) en_r[3:0] <= {en_r[2:0],en};
	      if ( (&en_r) && (in_r != out_3r)) begin
		 count <= count + 1'b1;
		 err <= 1'b1;
	      end
	      else err <= 0;

	      if ( (&en_r) && (in_r == out_3r)) good <= 1'b1;
	      else good <= 0;


	      ferr_r <= force_error;
	      if (!ferr_done) begin
		 ferr_rr <= ferr_r;
		 ferr_done <= ferr_r;
	      end
	      else begin
		 ferr_rr <= 0;
		 ferr_done <= force_error;
	      end

	   end
	end // always @ (posedge clk, posedge rst)

endmodule




//
// PRBS39 4/20/11
//-----------------------------------------------------------------------------
// 39-bit PRBS module, refs:
//   http://www.eng.auburn.edu/~strouce/polycorrect2.pdf
//   http://www.xilinx.com/support/documentation/application_notes/xapp052.pdf
//   http://www.eecs.berkeley.edu/~newton/Classes/CS150sp97/labs/lab5/lab5.html#primPolynomials
//   http://en.wikipedia.org/wiki/Linear_feedback_shift_register
//   http://www.springerlink.com/content/u590n42r72568803/
// Note that for any primitive polynomial, the reverse polynomial is also primitive!
//-----------------------------------------------------------------------------

module prbs39(
	input [38:0] init_dat,
	input        en,
	output reg   out,
	input        rst,
	input        clk);

	reg  [39:1] lfsr;

	always @(posedge clk or posedge rst) begin
	   if(rst) begin
	      lfsr[39:1]  <= init_dat[38:0];
	      out <= 0;
	   end
	   else begin
	      out <= lfsr[38];
	      if (en) lfsr[39:1] <= {lfsr[38:1],lfsr[39]^lfsr[4]};
//	      else lfsr  <= lfsr;  // why do this???
	   end
	end // always
endmodule
