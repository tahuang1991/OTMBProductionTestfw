`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Create Date:    12:22 8/20/11
// Design Name: 
// Module Name:    dcfeb_test
//
// - Start with (Post)RadTest v3.4 then take away the snap12 logic, replace with Ben's fiber clct code
//    two outputs to testpoints: Valid and Match.  Add a wire to PCB for access inside VME crate.
//    GbE needs two modes (use switch 8 for control): 
//       dump an error count packet at ~1 Hz
//       dump a packet every time we get a non-zero triad 
//    Use switch 7 to force a transmit error (like we do now).
//    Fibers to use (must have LC connector):
//
// dCFEB test version info
//  1.1: just get PRBS test working, with err_count sent to GbE at ~1 Hz. sw8: Force err on PB & sw7. why groups of 4?
//        -- use MTP fiber 12
//  1.2: try to add triad word inject and readout via GbE. !sw8: load data on PB & sw7. -- bug in gbe tx_dat logic
//  1.3: fixed bug, works well.  Maybe too many triad words get injected... tuned LED assignments
//  1.4: constrain inject triad to single clk80 period, hijack ferr_r logic.   d001f004?
//        -- I don't see any cause for groups of 4 forced errors...
//  1.5: bring rx_fc to test_led10.  also qpll_lock, rx_sync_done.  changed these 4 TPs are set same (5,7,9,10).
//  1.6: send sw[8:7] to FP LEDs, also QPLL_Lock, rst/pb...etc.
//  1.7: add NullVMEop logic, mod UCF.  debug triad transmit function...use to test triad RECEIVE function!
//  1.8: edit UCF, lots of backplane probing logic for ccb_rx1,12,22-35 to LEDs and ccb_tx0-19 to CCB.
//        -- remove Translator Loopback test & set on-board TMB bidir bus control signals to good state:
//         /gtl_oe=0, gtl_loop=0, dmb_loop=0, rpc_loop=0, ccb_status_oe=1, /dmb_oe=0
//  1.9: adjust some output assign values, make fp-LED7 OR of !pb + !L1reset
//  1.10: add control for both hard_reset_fpga signals, invert some fp-LED signals,
//        add trigger counter for 10 CCB pulses and CCB L1Reset function
//  1.11: add 16-bit-serialized readout of results register via TMB_CFG_DONE, with ALCT_CFG_DONE as handshake
//        -- activated with CCB_CMD 33h (load CCB VME "CMD bus" register with CCh)
//        -- any non-33 CMD will clear the results register
//        -- currently sends only the results for pulse_count
//  1.12: increase Results register to 20 bits, decode & return all 8 bits of the CMD code
//        -- establish a few new active functions: readback data_reg & pulses_fired, include ALCTadbs & HardRst, also TMBres0
//        -- defined a set of futire functions that will be needed
//  1.13: make some fixes in pulses_fired and results_r
//  1.14: make some fixes in cmd CC results & get_bit_ptr.  Added logic to check most CCB signals to TMB using CMDs CC & C0.
//        -- also trying 5-bit tmb_reserved_in data bus to the CCB & added a test for TMB_L1Arelease/request lines
//  1.15: fix serial results_reg "end" signal, change tmb_l1a_relreq logic, tweak ccb_cmd_r control logic
//  1.16: add Lock monitor logic for QPLL and ck160, put ccb_rx[0] into a BUFR for improved performance, but did not help!
//  2.0:  TMB mainboard is now loopback-safe (2.5 V) so enable DMB-Loop & RPC-Loop.  Enable Loops for CCB test signals.
//  2.1:  Add all DMB-Loop logic & load readback registers for software checks.  NO UCF entries for ccb_rx[50:48], yikes!
//  2.2:  Added UCF entries for ccb_rx[50:48], UN-inverted tmb_l1a_relreq inputs from ccb_rx[25:24].
//        -- Add TrigStop/Start functions for test control, but defaults to ON at Reset, Start not normally needed?
//  2.3:  Add fixes for dmbfifo_step1ck tests.  No clock, looks frozen; try Step1 next....
//  2.4:  Add STEP control, use ODDR to drive step1 as mirror of lhc_ck to dmbfifo_ck via DMB_Loopback
//  2.5:  Put all 40 MHz functions into lhc_ck domain, add 7 ccb_cmd diagnostic signals & qpll_lock to test_leds
//  2.6:  Use tmb_clock0 now for CCB bus communications, so Step4 can be used in other tests
//        -- LONG TERM: FIX the clocking to the QPLL, better to use tmb_clock1 instead of tmb_clock05p (FPGA SEU can screw QPLL)
//        -- fixed all DMB Loop logic to run on slwclk, bring loop[3] out&in to LEDs
//  2.7:  add another EN delay inside PRBS39 & initialize ALL registers in there; add llout_dmbloop for testLEDs;
//        add good_dmbloop monitoring too
//        -- force bit err_dmbloop[27] to zero, but add debug for its lhc_tog_err in sw[7] testLEDs
//  2.8:  take dmbloop back to 40 MHz; fix trig_stop/start 07/06 encoding; activate step4 with en_loopbacks (no sw[8] now)
//  2.9:  en_loopbacks defails to 0 now; add a lhcloop_ck-lhc_ck sync step for en_lhcloop before toggle error check
//  2.10: bring err_dmbloop[3] to sw7 Low test_LED; put LHC_CK into PRBS39!
//  
//  
//  about clocks:
//   QPLL links with tmb_clock05p (no delay), which is disabled when step4=Hi and prevents QPLL lock (bad)
//     -- this is dumb; QPLL should use tmb_clock0 (io600 on B31)? Only disabled by mez_clock_en from TMB/VME boot reg. b12
//     -- tmb_clock1 comes in on K24, has delay AND it stops when step4=Hi
//   qpll_ck40 comes directly from QPLL40, derived from tmb_clk05p that gets stopped bt step4; goes nowhere
//   lhc_ck NOW comes directly from tmb_clock0; slwclk is this divided by 25
//     -- "locked" indicates lock OK from slwclk div25 function
//   ck40 comes from gtx_Tx based on QPLL160
//     -- "lock40" indicates lock OK, and it depends upon "ck160_locked"
//     -- has random phase relative to lhc_ck
//   rx_clk is gtx_Rx 160 MHz reconstructed receiver USR clock
//     -- "ck160_locked" indicates gtx_Tx lock OK, but not Rx!
//     -- it's not clear that rx_clk has any bufg or lock implemented at all!
//   ck125 is not really used, just to monitor other clocks
//   ckgbe is 125 MHz gtx-based clock for GbE functions
//   gbe_txclk2 comes from gtx_Tx based on ckgbe, used for all GbE RadTest functions
//     -- also used in GbE readout test of Comparator data from DCFEB
//     -- mostly removable, delete it all?  Later replace GbE comparator readback with VME function?
//  
//  
//  MTP Fiber Mapping to Signal Name, FPGA GTX channels, Diff. Pin Numbers, verilog name
//    1:   Tx0-Rx0   GTX3-GTX0    AK1/AK2 - AP5/AP6             txp/n[0]-rxp/n[0]
//    2:   Tx1-Rx1   GTX4-GTX1    AH1/AH2 - AM5/AM6             txp/n[1]-rxp/n[1]  <<-- no errors? test!
//    3:   Tx2-Rx2   GTX7-GTX2    AB1/AB2 - AL3/AL4             txp/n[2]-rxp/n[2]  <<-- no errors? test!
//    4:   Tx3-Rx3   GTX8-GTX3    Y1/Y2   - AJ3/AJ4             txp/n[3]-rxp/n[3]
//    9:   Tx4-Rx8   GTX9-GTX8    V1/V2   - AA3/AA4             txp/n[4]-rxp/n[4]
//    10:  Tx5-Rx9   GTX10-GTX9   T1/T2   - W3/W4 --swapped!    txp/n[5]-rxp/n[5]
//    11:  Tx6-Rx10  GTX11-GTX10  P1/P2   - U3/U4 --swapped!    txp/n[6]-rxp/n[6]
//    12:  Tx7-Rx11  GTX2-GTX11   AM1/AM2 - R3/R4               txp/n[7]-rxp/n[7]   <<-- use this one!
//    5:    Rx4        GTX4           AG3/AG4
//    6:    Rx5        GTX5           AF5/AF6
//    7:    Rx6        GTX6           AE3/AE4
//    8:    Rx7        GTX7           AC3/AC4
//      QPLL 160 refclk comes into pins AB6/AB5, Quad 113 refclk 1
// 
//  
//    old RadTest version info
//  Full RadTest system: GbE packet commands & data, BRAM readout, Snap12 and Translator loopbacks
//  2.6: add "Force Error" feature for 32 loopbacks (sw7 & !pb)
//  2.7: add Function4 capability to send ~100 GbE packets/sec and ignore commands from GbE (sw8 & !sw7)
//
//  3:   start from v2.7, modify translator tests to 80 MHz for 22 channels, 2 special ch. DC 1,0
//        -- use outreg and inreg, need to double pipeline the data for checking logic
//        -- tests may need to force errors to make sure it works
//  3.1: add 90 MHz clocking for TI translator tests     ** Post Rad Testing **
//  3.2: reduce BRAMs & CRAMs for fast compiling, try to force obuf_ff use.   --> snapr[6] out
//  3.3: slow translator test to 84 MHz & use IOB FFs (no help);  fixed (!) snap12_gtx ferr_r logic,
//         bring rxdvr[7:0] to test LEDs.  --> snapr[1] out to scope
//  3.4: add explicit IOB=TRUE code but still uses SLICEs;  fixed (!) F4 packet count logic (no skip)
//         --> snapr[2] out to scope
//
// 
//////////////////////////////////////////////////////////////////////////////////
module dcfeb_test(
    input 	      ck_gben, ck_gbep,
    input 	      ck125n, ck125p,
    input 	      ck160n, ck160p,
    input 	      lhc_ckn, lhc_ckp,
    input 	      gbe_rxn, gbe_rxp,
    input  [8:7]      sw,
    input 	      tmb_clock0, pb, vme_cmd10,
    input 	      gbe_fok, qpll_lock,
    input [11:7]      alct_rx, // not used now, was 3.3V I/O translation test on Mezz #1
    input 	      alct_rx13, alct_rx19, alct_rx23, // not used now ^^^^
    input 	      prom_d3, prom_d7, jtag_fpga3, sda0, tmb_sn, t_crit, // not used now ^^^
//    input [28:5]      rpc_rx, // 24 inputs from TI Level Shifters loopback
    input [50:0]      _ccb_rx,  // add 42-47
// ccb_rx1=L1rst; 7-2=CMD, 8,9=ev/bcntrst; 10=cmdstr, 11=bc0, 12=L1A, 13=datstr, 21-14=DAT
    input [5:0]       dmb_rx,
    input [15:8]      dmb_i1tx,  // yes, input for loopback tests, only use on MODIFIED TMBs!
    input [31:24]     dmb_i2tx,  // yes, input for loopback tests, only use on MODIFIED TMBs!
    input [43:38]     dmb_i3tx,  // yes, input for loopback tests, only use on MODIFIED TMBs!
    output [7:0]      dmb_tx,
    output [23:16]    dmb_o1tx,
    output [37:32]    dmb_o2tx,
    output [48:44]    dmb_o3tx,
    output            _gtl_oe, gtl_loop, dmb_loop, rpc_loop, ccb_status_oe, _dmb_oe, // set normal safe bidir bus modes
    output [26:0]     _ccb_tx,  // add 20-26
    output            _hard_reset_tmb_fpga, _hard_reset_alct_fpga,
    output 	      gbe_txn, gbe_txp,
    output 	      f_sclk, rst_qpll,
    output [6:0]      vme_reply,
    output [4:0]      step, // step4 enables STEP mode for 3:0 = cfeb, rpc, dmb, alct.  step4 Low makes all free-running clocks.
    output reg [7:0]  led_low,
    output reg [15:8] led_hi,
    output reg [10:1] test_led,
    input 	      t12_fault, r12_fok,
    input  [7:7]      rxn, rxp,
    output [7:7]      txn, txp,
    output 	      t12_rst, t12_sclk, r12_sclk
   )/* synthesis syn_useioff = 1 */;


// snap12 GTX signals
   wire [15:0] send_lim;
   wire        synced_snapr, all_tx_ready;
   wire        snap_clk2, ck160_locked;
   wire        snap_wait;
   wire [7:0]  check_ok_snapr, check_bad_snapr;
   wire [7:0]  rxdv_snapr, rxcomma_snapr, synced_snapt;
   wire [7:0]  rxdv_diff, rxcomma_diff;
   wire [7:0]  lgood_snapr, lbad_snapr, llost_snapr;
   wire [15:0] snap_rxdat;
   wire [1:0]  snap_rxk;
   reg  [7:0]  check_okr_snapr, check_badr_snapr;
   reg  [7:0]  rxdvr_snapr, rxcommar_snapr;
   reg [15:0]  snap_rxdat_r;
   reg  [1:0]  snap_rxk_r;
   reg [15:0]  err_count;
   reg [15:0]  lerr_count;
   reg  [7:0]  time_r_snap;
   reg  [7:0]  time_snap;
   reg [15:0]  snap_tx_dat;
   reg  [1:0]  snap_kout;
   reg 	       snap_comma_align;
   reg 	       snap_rstdone_r2, snap_rstdone_r3;
   wire [15:0] snap_bram;
   wire [1:0]  snapp_bram;

   wire  ck15125, stopped, locked, stop40, lock40, ck90, ck84, dmbfifo_step1ck;
   wire  gbe_refck; // GTXE1 ref clk, used for MGTref and DoubleReset clock
   wire  gbe_tx_outclk; // out from Tx PLL
   wire  gbe_txclk2;   // drives logic for Tx, the "internal" choice for USR clock
   wire  gbe_ready, all_ready, txoutclk_mmcm_lk;

   parameter SEEDBASE = 64'h8731c6ef4a5b0d29;
   parameter SEEDSTEP = 16'hc01d;
   parameter BRAM_LIM = 12'hb07; // bff: Try 256 Brom's.  these are Dout from BRAMs
     // make sure BRAM_LIM matches rad_testXXXbroms.h file size!
   parameter CRAM_LIM = 8191;  // put all CROM pages into one big block, 2048 words each.
     // 4 is OK, now try 32.  64 pages is 131071:0, 32 is 65535:0, 16 is 32767:0, 4 is 8191:0
   parameter GBE_LIM = 16'h080b; // WORDcount limit. allow extra bytes for MAC preamble, addr's...
   reg [15:0] pkt_lim; // BytecountLimit/2. Plus extra 22 bytes for MAC preamble etc.
   reg [15:0] counter;  // ^^^ 1st data word @ counter == 12; need 11 extra word counts, 22 bytes.
   reg [5:0]  ireg;
   reg [10:0] tx_adr;
   reg [16:0] tx_cadr;
   reg [15:0] gbe_rxcount;
   reg [22:0] pkt_time, pkt_time_r;
   reg [22:0] free_count;
   reg 	      free_tc, free_tc_r;
   reg 	      slow_tc, slow_tc_r;
   reg [7:0]  time_count;
   reg [7:0]  time_r_i;
   reg [7:0]  time_r;
   reg [7:0]  time_40i;
   reg [7:0]  time_40r;
   reg [7:0]  pkt_id;
   reg 	      loop_command;  // send big packets to GbE, ignore rxdv commands from PC
   reg [15:0] cmd_code;
   reg [11:0] bk_adr;
   reg   tx_resetdone_r, rx_resetdone_r, rx_resetdone_r2, rx_resetdone_r3, pkt_send;
   wire  gtx_ready, tx_resetdone, rx_resetdone;
   wire [13:0] test_in;
   reg [7:0]   ovfl_packet;
   reg 	       rx_timeout;
   reg 	       good_rx_cmd;
   reg 	       l_lock40, ck160_locklost, qpll_lock_lost;
   reg 	       l_lock125, ck125_locklost;
   wire reset, gtx_reset;
   wire ckgbe, ck125, ck160, lhc_ck, qpll_ck40, slwclk;   // 125 ref, ext 125 usr, QPLL160, QPLL40, QPLL40/25=1.6MHz
   wire zero, one;
   wire [1:0] zero2;
   wire [31:0] zero32;
   
   wire [12:0] low;
   wire [3:0]  ignore;  // outputs from GTX we don't care about
   wire        rxpll_lk;
   wire [2:0]  rx_bufstat;
   wire [1:0]  rx_comma;
   wire [1:0]  rx_disp;  // other signals for GTX
   wire [1:0]  rx_lostsync;
   reg 	[1:0]  gbe_kout, tx_kout, tx_kout_r;
   reg 	       comma_align;
   wire [5:0]  rxer;
   reg  [15:0] gbe_txdat, tx_out, tx_out_r;
   wire [15:0] gbe_rxdat;
   reg  [15:0] l_gbe_rxdat, ll_gbe_rxdat;
   reg 	       l_rxdv, l_kchar, ll_kchar, mac_seek, mac_sync, mac_ack, counter_send;
   reg 	[1:0]  kchar_r;
   reg 	[1:0]  sync_state, data_state;
   wire        rxdv;

   reg [15:0] data_bram, data_bram_r;  // these are the BRAM MUX bus & register
   wire [15:0] data_rom[BRAM_LIM:12'hb00]; 
   wire [1:0]  datap_bram;

   reg [15:0] data_cram, data_cram_r;  // these are the CRAM MUX bus & register

   reg         crc_en, crc_rst;
   wire [31:0] crc_out;
   reg  [15:0] byte_count;

// Add-ons for Ben dCFEB testing:
   wire    tx_clk_out, tx_clk, tx_begin, tx_fc;
   wire    rx_clk, rx_strt, rx_valid, rx_match, rx_fc;
   wire [3:0]  word;
   wire [3:1]  nz;
   wire [47:0] comp_dat, comp_dout;
   reg  [47:0] comp_dat_r, comp_dat_2r, comp_dat_3r, comp_dat_4r, comp_dat_5r;
   reg [3:0]   itriad;
   reg [2:0]   comp_phase;
   reg 	       save_triad, comp_dav_r, send_triad;
   wire        push_fifo, no_comp_dav, comp_overflow;
   reg  [23:0] triad_word, triad_word_r;

//  set itriad=9  if ( !sw[8] && itriad==0 && |comp_dat_r > 0 )  can we pick a few specific bits?  TRIAD FORMAT!
//      -- comp_dat is 48 bits, takes 3-step logic....pipeline some before _r and finish before _2r
//  48-bit wide, 5-deep pipeline; word0 is EN.  comp_dat_r, comp_dat_2r, .... comp_dat_5r
//      -- goes to 48-bit FIFO with  PUSH=word0&itriad>0; runs on rx_clk (160 MHz)
//      -- PUSH controls itriad-1 countdown
//  FIFO output is 16-bits wide(?) for GbE tx_dat...NOT ALLOWED!   runs on clk 
//      -- !empty triggers GbE data dump of three triads (3 * 48 bits each...54 bytes total)


// 27 registers for inputs from DMB Loopback
   reg  [26:0] shft_seq, rnd_word;  // slow seq .1 sec shift, loads to flag on pb_pulse  shft_seq = 32'h00000001;
   reg 	       debounced_bit;    // sets one pulse for 200 ns  (5 MHz clock)
   reg 	       pb_pulse;  //  <- sw7 & !pb, clears on pb high after 2nd seq. shift (debounce), lasts a while!
   reg 	       err_wait;   // pb_pulse & tc & !wait -> load rnd_word, set wait.  !pb_pulse & tc & wait -> clear wait
   reg 	       ferr_i, ferr_r, ferr_rr, ferr_done;

   assign test_in[11:7] = alct_rx[11:7];
   assign test_in[13] = alct_rx13;
   assign test_in[12] = alct_rx19;
   assign test_in[0] = sda0;
   assign test_in[1] = tmb_sn;
   assign test_in[2] = t_crit;
   assign test_in[3] = jtag_fpga3;
   assign test_in[4] = prom_d3;
   assign test_in[5] = prom_d7;
   assign test_in[6] = alct_rx23;

// clct_status: temp for testing,  _ccb_tx[8:0] was 9'h0aa, now toggle with push-button
   assign _ccb_tx[8] = pb;
   assign _ccb_tx[7] = !pb;
   assign _ccb_tx[6] = pb;
   assign _ccb_tx[5] = !pb;
   assign _ccb_tx[4] = pb;
   assign _ccb_tx[3] = !pb;
   assign _ccb_tx[2] = pb;
   assign _ccb_tx[1] = !pb;
   assign _ccb_tx[0] = pb;


   assign vme_reply[0] = 1'b1;   // OE_b, low true
   assign vme_reply[1] = 1'b1;   // DIR
   assign vme_reply[2] = 1'b0;   // DTACK, inverted on TMB board
   assign vme_reply[3] = ~vme_cmd10; // IACKOUT = IACKIN, inverted on TMB board?
   assign vme_reply[4] = 1'b0;   // BERR, inverted on TMB board
   assign vme_reply[5] = 1'b0;   // IRQ, inverted on TMB board
   assign vme_reply[6] = 1'b0;   // vme_ready, High enables 0-5 above?
   assign _gtl_oe = 1'b0;   // JRG: always set LOW (SEU danger, short to GND --PU)
   assign gtl_loop = 1'b0;  // JRG: always set LOW (SEU danger, make OPEN --PD)
   assign dmb_loop = 1'b1;  // JRG: set HIGH for SPECIAL TMB ONLY! LOW for normal CMS operation (SEU danger, make OPEN --PD)
   assign rpc_loop = 1'b1;  // JRG: set HIGH for Produtcion Test, LOW for normal CMS operation (SEU safe --PD)
   assign ccb_status_oe = 1'b1;  // JRG:  set HIGH for Produtcion Test and for normal CMS operation (SEU danger, make OPEN --PU)
   assign _dmb_oe = 1'b0;
   assign _hard_reset_tmb_fpga = 1'b1;
   assign _hard_reset_alct_fpga = 1'b1;
   
   assign low=0;
   assign zero=0;
   assign zero2=2'b0;
   assign zero32=0;
   assign one=1'b1;

   assign t12_rst  = 1'b1;  // low-true signal for Snap12 Transmitter
   assign t12_sclk = 1'b1;  // to Snap12 Transmitter
   assign r12_sclk = 1'b1;  // to Snap12 Receiver
   assign f_sclk   = 1'b1;  // to Finisar GbE Transceiver
   assign rst_qpll = 1'b1;  // reset is low-true, but in ExtControl mode (sw6 On) it becomes fSelect[5]
   //                                                 and autoRestart (sw5) becomes fSelect[4]
   //        note that sw1-4 are fSelect[0:3] but they only function in ExtControl mode (sw6 On),
   //        and fSelect sets the VCXO center frequency (because automatic calibration is Off)
   //   wire qpll_lock; // probably random in ExtControl mode (sw6 On)

//   reg [11:0]  l1a_count; // count L1accepts
   wire  ccb_cken; // global clock signal to use for ccb_rx[0] "clock"
   reg [11:0]  ccbrxzero_count; // count toggles on ccb_rx0
   reg [12:0]  pulse_count; // count triggers, 10 low-true sources ANDed together
   reg [11:0]  pulses_fired;
   reg [11:0]  in_pulse_r;
   wire [11:0] in_pulse;
   reg       trigger;

// - pulse counter (pulse is CE); send pulses & read back count via status LEDs (11 ccb_rx)
//      > 1 reset signal (L1Reset=ccb_reserved4, to clear) and triggered by 11 different pulses:
//          BC0, L1A, tmb_soft_reset=tmb_reserved1, clct/alct_external_trigger, dmb_cfeb_calibrate[2:0], 
// 	    adb_pulse sync/async, alct_hard_reset,
//         - verify that all single CMD & DATA bits work reliably (avoid CMD/DATA = 0C,0D,0E,0F; 10,11,12,13; 40,41,42,43)
//         - need to check LEDs at least one time too, to verify status bus works

// unless noted otherwise, these pulses are only 25ns long and count just one time:
   assign in_pulse[0] = !_ccb_rx[11]; // BC0, CCB base+52
   assign in_pulse[1] = !_ccb_rx[12]; // L1A, CCB base+54
   assign in_pulse[2] = !_ccb_rx[29]; // TMB_SoftRst (tmb_res1), CCB base+6c or 6a
   assign in_pulse[3] = !_ccb_rx[32]; // clct_ext_trig, CCB base+86
   assign in_pulse[4] = !_ccb_rx[33]; // alct_ext_trig, CCB base+88
   assign in_pulse[5] = !_ccb_rx[39]; // dmb_cfeb_calib0, CCB base+8a
   assign in_pulse[6] = !_ccb_rx[40]; // dmb_cfeb_calib1, CCB base+8c
   assign in_pulse[7] = !_ccb_rx[41]; // dmb_cfeb_calib2, CCB base+8e
   assign in_pulse[8] = !_ccb_rx[27]; // alct_hard_reset, CCB base+66:  500 ns long pulse
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
   reg        ccb_cmdstrb_r, ccb_datstrb_r;
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
   wire [15:0] count_dmbloop[26:0];    // 27 16-bit wide elements, error counts for DMB Loop signals
   reg [11:0]  dmbloop_errcnt, dmbloop1_stat, dmbloop2_stat, dmbloop3_stat;
   reg [31:0]  loop_count;
   reg 	       en_cabletests, en_fibertests, en_cabletests_r, en_fibertests_r;
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

   assign err_dmbloop[27] = 0; // lhc_tog_err;
   assign good_dmbloop[27] = !(|err_dmbloop);

   assign step[4] = en_loopbacks; // ~sw[8];  // step4 Low makes free clocks from TMB; Hi allows logic signals for clocks
   assign step[3] = 1'b0;   // this is cfeb step signal
   assign step[2] = 1'b0;   // this is rpc step signal
//   assign step[1] = lhc_ck;  // this is dmb step signal... now uses ODDR below.
   assign step[0] = 1'b0;   // this is alct step signal
   ODDR #(.DDR_CLK_EDGE("OPPOSITE_EDGE"), .INIT(1'b0), .SRTYPE("ASYNC")) DMB_FIFO_CLK (.Q(step[1]), .C(lhc_ck), .CE(1'b1), .D1(1'b1), .D2(1'b0), .R(1'b0), .S(1'b0));  // make step[1] an image of lhc_ck, as it goes out and loops back as dmbfifo_step1ck

   

   (* ram_style = "distributed" *)
   reg [15:0] cram [CRAM_LIM:0];  // put all CROM pages into one big block

   initial begin
      l_rxdv=0;
      l_kchar=0;
      kchar_r=0;
      tx_resetdone_r = 0;
      rx_resetdone_r = 0;
      rx_resetdone_r2 = 0;
      rx_resetdone_r3 = 0;
      comma_align = 0;
      snap_kout = 2'h0;
      snap_comma_align = 0;
      snap_rstdone_r2 = 0;
      snap_rstdone_r3 = 0;
      gbe_kout = 2'h1;
      gbe_txdat = 16'h50bc;
      sync_state = 2'h0;
      data_state = 2'h0;
      mac_seek=0;
      mac_sync=0;
      mac_ack=0;
      l_lock40 = 0;
      ck160_locklost = 0;
      l_lock125 = 0;
      ck125_locklost = 0;
      time_r = 8'h00;
      pkt_time = 23'h00000001;
      pkt_time_r = 23'h00000001;
      pkt_lim = 16'd64;
      gbe_rxcount = 16'h0000;
      good_rx_cmd = 1'b0;
      rx_timeout = 1'b0;
      free_tc_r = 1'b0;
      slow_tc_r = 1'b0;
      loop_command = 1'b0;
      pkt_id = 8'h0000;
      loop_count = 0;
      en_loopbacks = 1'b0;
      en_loopbacks_r = 0;
      en_loopbacks_rr = 0;
      en_cabletests_r = 0;
      en_cabletests = 1'b1;
      en_fibertests_r = 0;
      en_fibertests = 1'b1;
      llout_dmbloop = 0;
      lout_dmbloop = 0;
      lhc_tog = 0;
      en_lhcloop = 0;
      en_lhcloop_r = 0;
      lhcloop_tog = 0;
      lhcloop_tog_r = 0;
      for (i = 0; i < 27; i = i + 1) begin
	 init_dmbloop[i] = 39'd15 + 11401017*i;
      end
//     $readmemh ("cram_rand64.mem", cram); // rand contents: compile takes 50+ hrs.  30% SLICEs used.
//     $readmemh ("cram_init64.mem", cram); // make sure this matches with CRAM_LIM!  fast but few SLICEs
      $readmemh ("cram_init.mem", cram, 0, 8191); // 4 pages here; 2048 each
//     $readmemh ("cram_rand16.mem", cram); // SMALL random file contents: took 4 hours!
      shft_seq = 27'h0000001;
      pb_pulse = 0;
   end

    genvar u;
    generate
       for (u=0; u<27; u=u+1) begin:prbs39gen
	  prbs39_test dmb_loop(init_dmbloop[u], en_loopbacks_r, in_dmbloop[u], out_dmbloop[u], good_dmbloop[u], err_dmbloop[u], count_dmbloop[u], err_wait&rnd_word[u], (!locked)|reset, lhc_ck); // slwclk?
       end
    endgenerate

   BUFG lhcck(.I(tmb_clock0), .O(lhc_ck));
   IBUFGDS #(.DIFF_TERM("TRUE"),.IOSTANDARD("LVDS_25"))  qpll40(.I(lhc_ckp) , .IB(lhc_ckn) , .O(qpll_ck40));
   IBUFGDS #(.DIFF_TERM("FALSE"),.IOSTANDARD("LVDS_25"))  clock125(.I(ck125p) , .IB(ck125n) , .O(ck125));
//   IBUFDS_GTXE1 #(.DIFF_TERM("TRUE"),.IOSTANDARD("LVDS_25"))  clock_gbe(.I(ck_gbep) , .IB(ck_gben) , .O(ckgbe));
   IBUFDS_GTXE1  clock_gbe(.I(ck_gbep) , .IB(ck_gben) , .O(ckgbe), .ODIV2(), .CEB(zero));
   IBUFDS_GTXE1  clock160(.I(ck160p) , .IB(ck160n) , .O(ck160), .ODIV2(), .CEB(zero));

   BUFG gbe_refclock(.I(ckgbe), .O(gbe_refck));
   bufg_div25clk clk1p6(lhc_ck,slwclk,stopped,locked); // slwclk is now 1.6 MHz (was 5 MHz using ck125)

   BUFR ccbrx0_clock(.I(_ccb_rx[0]), .O(ccb_cken));

   wire        clkfbout;
   wire        clkfbout_buf;
  MMCM_ADV
  #(.BANDWIDTH            ("OPTIMIZED"),
    .CLKOUT4_CASCADE      ("FALSE"),
    .CLOCK_HOLD           ("FALSE"),
    .COMPENSATION         ("ZHOLD"),
    .STARTUP_WAIT         ("FALSE"),
    .DIVCLK_DIVIDE        (4),
    .CLKFBOUT_MULT_F      (43.000),
    .CLKFBOUT_PHASE       (0.000),
    .CLKFBOUT_USE_FINE_PS ("FALSE"),
    .CLKOUT0_DIVIDE_F     (21.500),
    .CLKOUT0_PHASE        (0.000),
    .CLKOUT0_DUTY_CYCLE   (0.500),
    .CLKOUT0_USE_FINE_PS  ("FALSE"),
    .CLKOUT1_DIVIDE       (15),
    .CLKOUT1_PHASE        (0.000),
    .CLKOUT1_DUTY_CYCLE   (0.500),
    .CLKOUT1_USE_FINE_PS  ("FALSE"),
    .CLKOUT2_DIVIDE       (16),
    .CLKOUT2_PHASE        (0.000),
    .CLKOUT2_DUTY_CYCLE   (0.500),
    .CLKOUT2_USE_FINE_PS  ("FALSE"),
    .CLKOUT3_DIVIDE       (89),
    .CLKOUT3_PHASE        (0.000),
    .CLKOUT3_DUTY_CYCLE   (0.500),
    .CLKOUT3_USE_FINE_PS  ("FALSE"),
    .CLKIN1_PERIOD        (8.0),
    .REF_JITTER1          (0.010))
  txusrclk_mmcm
    // Output clocks
   (.CLKFBOUT            (clkfbout),
    .CLKFBOUTB           (),
    .CLKOUT0             (gbe_txclk2),
    .CLKOUT0B            (),
    .CLKOUT1             (ck90),
    .CLKOUT1B            (),
    .CLKOUT2             (ck84),
    .CLKOUT2B            (),
    .CLKOUT3             (ck15125),
    .CLKOUT3B            (),
    .CLKOUT4             (),
    .CLKOUT5             (),
    .CLKOUT6             (),
     // Input clock control
    .CLKFBIN             (clkfbout_buf),
    .CLKIN1              (ck125),  // use ck125 or gbe_tx_outclk...or gbe_refck?
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
    .LOCKED              (txoutclk_mmcm_lk),
    .CLKINSTOPPED        (),
    .CLKFBSTOPPED        (),
    .PWRDWN              (1'b0),
    .RST                 (!rxpll_lk));

    BUFG clkf_buf(.O (clkfbout_buf), .I (clkfbout));

   `include "rad_test8broms.h"
// `include "rad_test256broms.h"
// `include "random_256broms.h"

    genvar v;
    generate
       for (v=12'hb00; v<=BRAM_LIM; v=v+1'b1) begin:bromgen
 // make sure BRAM_LIM matches XXXbroms.h file size!
          BRAM_SINGLE_MACRO    #(
	.BRAM_SIZE("36Kb"),
        .DEVICE("VIRTEX6"), // Target Device: "VIRTEX5", "VIRTEX6", "SPARTAN6"
        .DO_REG(0),         // Optional output register (0 or 1)
        .INIT(36'h000000000),  // Initial values on output port
        .INIT_FILE ("NONE"),
        .WRITE_WIDTH(16),   // Valid values are 1-72 (37-72 only valid when BRAM_SIZE="36Kb")
        .READ_WIDTH(16),    // Valid values are 1-72 (37-72 only valid when BRAM_SIZE="36Kb")
        .SRVAL(36'h000000000), // Set/Reset value for port output
        .WRITE_MODE("WRITE_FIRST"),  // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE"
        .INIT_00(MEM_A[v]),
        .INIT_01(MEM_01),
        .INIT_02(MEM_02),
        .INIT_03(MEM_03),
        .INIT_04(MEM_04),
        .INIT_05(MEM_05),
        .INIT_06(MEM_06),
        .INIT_07(MEM_07),
        .INIT_08(MEM_08),
        .INIT_09(MEM_09),
        .INIT_0A(MEM_0A),
        .INIT_0B(MEM_0B),
        .INIT_0C(MEM_0C),
        .INIT_0D(MEM_0D),
        .INIT_0E(MEM_0E),
        .INIT_0F(MEM_0F),
        .INIT_10(MEM_10),
        .INIT_11(MEM_11),
        .INIT_12(MEM_12),
        .INIT_13(MEM_13),
        .INIT_14(MEM_14),
        .INIT_15(MEM_15),
        .INIT_16(MEM_16),
        .INIT_17(MEM_17),
        .INIT_18(MEM_18),
        .INIT_19(MEM_19),
        .INIT_1A(MEM_1A),
        .INIT_1B(MEM_1B),
        .INIT_1C(MEM_1C),
        .INIT_1D(MEM_1D),
        .INIT_1E(MEM_1E),
        .INIT_1F(MEM_1F),
        .INIT_20(MEM_20),
        .INIT_21(MEM_21),
        .INIT_22(MEM_22),
        .INIT_23(MEM_23),
        .INIT_24(MEM_24),
        .INIT_25(MEM_25),
        .INIT_26(MEM_26),
        .INIT_27(MEM_27),
        .INIT_28(MEM_28),
        .INIT_29(MEM_29),
        .INIT_2A(MEM_2A),
        .INIT_2B(MEM_2B),
        .INIT_2C(MEM_2C),
        .INIT_2D(MEM_2D),
        .INIT_2E(MEM_2E),
        .INIT_2F(MEM_2F),
        .INIT_30(MEM_30),
        .INIT_31(MEM_31),
        .INIT_32(MEM_32),
        .INIT_33(MEM_33),
        .INIT_34(MEM_34),
        .INIT_35(MEM_35),
        .INIT_36(MEM_36),
        .INIT_37(MEM_37),
        .INIT_38(MEM_38),
        .INIT_39(MEM_39),
        .INIT_3A(MEM_3A),
        .INIT_3B(MEM_3B),
        .INIT_3C(MEM_3C),
        .INIT_3D(MEM_3D),
        .INIT_3E(MEM_3E),
        .INIT_3F(MEM_3F),
        .INIT_40(MEM_40),
        .INIT_41(MEM_41),
        .INIT_42(MEM_42),
        .INIT_43(MEM_43),
        .INIT_44(MEM_44),
        .INIT_45(MEM_45),
        .INIT_46(MEM_46),
        .INIT_47(MEM_47),
        .INIT_48(MEM_48),
        .INIT_49(MEM_49),
        .INIT_4A(MEM_4A),
        .INIT_4B(MEM_4B),
        .INIT_4C(MEM_4C),
        .INIT_4D(MEM_4D),
        .INIT_4E(MEM_4E),
        .INIT_4F(MEM_4F),
        .INIT_50(MEM_50),
        .INIT_51(MEM_51),
        .INIT_52(MEM_52),
        .INIT_53(MEM_53),
        .INIT_54(MEM_54),
        .INIT_55(MEM_55),
        .INIT_56(MEM_56),
        .INIT_57(MEM_57),
        .INIT_58(MEM_58),
        .INIT_59(MEM_59),
        .INIT_5A(MEM_5A),
        .INIT_5B(MEM_5B),
        .INIT_5C(MEM_5C),
        .INIT_5D(MEM_5D),
        .INIT_5E(MEM_5E),
        .INIT_5F(MEM_5F),
        .INIT_60(MEM_60),
        .INIT_61(MEM_61),
        .INIT_62(MEM_62),
        .INIT_63(MEM_63),
        .INIT_64(MEM_64),
        .INIT_65(MEM_65),
        .INIT_66(MEM_66),
        .INIT_67(MEM_67),
        .INIT_68(MEM_68),
        .INIT_69(MEM_69),
        .INIT_6A(MEM_6A),
        .INIT_6B(MEM_6B),
        .INIT_6C(MEM_6C),
        .INIT_6D(MEM_6D),
        .INIT_6E(MEM_6E),
        .INIT_6F(MEM_6F),
        .INIT_70(MEM_70),
        .INIT_71(MEM_71),
        .INIT_72(MEM_72),
        .INIT_73(MEM_73),
        .INIT_74(MEM_74),
        .INIT_75(MEM_75),
        .INIT_76(MEM_76),
        .INIT_77(MEM_77),
        .INIT_78(MEM_78),
        .INIT_79(MEM_79),
        .INIT_7A(MEM_7A),
        .INIT_7B(MEM_7B),
        .INIT_7C(MEM_7C),
        .INIT_7D(MEM_7D),
        .INIT_7E(MEM_7E),
        .INIT_7F(MEM_7F),
        .INITP_00(MEMP_00),
        .INITP_01(MEMP_01),
        .INITP_02(MEMP_02),
        .INITP_03(MEMP_03),
        .INITP_04(MEMP_04),
        .INITP_05(MEMP_05),
        .INITP_06(MEMP_06),
        .INITP_07(MEMP_07),
        .INITP_08(MEMP_08),
        .INITP_09(MEMP_09),
        .INITP_0A(MEMP_0A),
        .INITP_0B(MEMP_0B),
        .INITP_0C(MEMP_0C),
        .INITP_0D(MEMP_0D),
        .INITP_0E(MEMP_0E),
        .INITP_0F(MEMP_0F)
    )
    block_rom_ab 
    (
        .ADDR (tx_adr[10:0]),
        .DI   (zero32),
        .DO   (data_rom[v]),
        .WE   (zero),       //  Alfke: "WE off" is not sufficient to protect Init data,
        .EN   (gtx_ready),  //  need to ensure safe setup time for ADR when EN is Enabled!
        .RST   (reset),     //  So require stable clocks before EN is set.
        .REGCE   (one),
        .CLK  (gbe_txclk2)
    );
       end
    endgenerate

   mac_crc mac_crc32(gbe_txdat, crc_en, crc_out, crc_rst, gbe_txclk2);

   assign gbe_ready = txoutclk_mmcm_lk & locked & tx_resetdone_r & rx_resetdone_r;
   assign gtx_ready = lock40 & ck160_locked & synced_snapr;
   assign all_ready = gbe_ready & gtx_ready;
   assign rxdv = ~(|rxer | rx_bufstat[2]) & gbe_fok; // idle is K28.5,D16.2  =  BC,50 in time order
   assign reset = !_ccb_rx[1] || (!sw[7] & !pb);  // JGhere, this ccb_rx signal can screw up a bench test...
   assign gtx_reset = reset | !gtx_ready;


//   always @(posedge _ccb_rx[0] or posedge reset) begin
   always @(posedge ccb_cken or posedge reset) begin
      if(reset) begin
	 ccbrxzero_count <= 0;
      end
      else begin
	 ccbrxzero_count <= ccbrxzero_count + 1'b1; // count toggles on ccb_rx0
      end
   end // always @ (posedge _ccb_rx[0] or posedge reset)



   always @(posedge tx_clk or posedge reset) begin
      if(reset) begin
	 ferr_i <= 0;
	 ferr_r <= 0;
	 ferr_rr <= 0;	      
	 triad_word_r <= 0;
      end
      else begin   // syncing forced single-clock effects (debounced, for bit error or triad word)
	 ferr_i <= debounced_bit;  // normally zero
	 ferr_r <= ferr_i;
	 if (!sw[8]) triad_word <= rnd_word[23:0];  // normally zero
	 if (!ferr_done & ferr_r) begin  // begin the Forced Error sequence, sync with snap tx_clk
	    ferr_rr <= ferr_r;  // true for exactly one tx_clk cycle
	    triad_word_r <= triad_word;  // loaded for exactly one tx_clk cycle
	 end
	 else begin  // end the Forced Error sequence when PB is released (ferr_i goes low)
	    ferr_rr <= 0;
	    triad_word_r <= 0;
	 end
	 ferr_done <= ferr_r;

      end

   end // always @ (posedge tx_clk or posedge reset)


   always @(posedge slwclk or posedge reset) // everything that uses 1.6 MHz clock with simple Reset (from lhc_ck)
     begin
	if (reset == 1) begin
	   tx_resetdone_r <= 0;
	   rx_resetdone_r <= 0;
	   free_count <= 0;
	   time_count <= 0;
	   pkt_time <= 23'h00000001;
	   free_tc <= 0;
	   slow_tc <= 0;
	   shft_seq <= 27'h0000001;
	   rnd_word <= 27'h0000000;
	   debounced_bit <= 0;
	   err_wait <= 0;
	   pb_pulse <= 0;
	end
	else begin
	   tx_resetdone_r <= tx_resetdone;
	   rx_resetdone_r <= rx_resetdone;
	   free_count <= free_count + 1'b1;
	   if (free_count == 23'h7fffff) slow_tc <= 1;  // ~.6 Hz or 1.664 sec cycle, for GbE looping
	   else slow_tc <= 0;

	   if (free_count[15:0] == 16'hffff) free_tc <= 1;  // ~76 Hz or .013 sec cycle, for debounce
	   else free_tc <= 0;

	   if (free_tc) shft_seq[26:0] <= {shft_seq[25:0],shft_seq[26]}; // shift a bit over time for "random" word
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
		 rnd_word <= 27'h0000000;
		 debounced_bit <= 0;
	      end
	   end

	   if (gtx_ready && time_count < 8'hfe) time_count <= time_count + 1'b1; // use to delay startup; ends.
	   if (gtx_ready) pkt_time <= pkt_time + 1'b1; // use to space packets 1.67 sec apart; looping.

	end
     end

   always @(posedge dmbfifo_step1ck or posedge reset) // used by one bit on DmbLoopback
     begin
	if (reset) begin
	   en_lhcloop <= 0;
	   lhcloop_tog <= 0;
	end
	else begin
	   en_lhcloop <= en_loopbacks;
	   if (en_lhcloop) lhcloop_tog <= !lhcloop_tog;
	end
     end

   always @(negedge lhc_ck) // everything that uses lhc_ck, no Reset.
     begin
	time_40i <= time_count;
	time_40r <= time_40i;  // transfer slow slwclk time counter to lhc_ck domain
     end


   always @(posedge lhc_ck or posedge reset) // everything that uses lhc_ck w/simple Reset (was ck40)
     begin
	if (reset) begin
	   lhcloop_tog_r <= 0;
	   en_lhcloop_r <= 0;
	   l_lock125 <= 0;    // use lhc_ck to monitor ck125
	   ck125_locklost <= 0;
	   qpll_lock_lost <= 0;
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
	   en_cabletests <= 1'b1; // this does nothing so far
	   en_fibertests <= 1'b1; // this does nothing so far
	   en_cabletests_r <= 1'b0;
	   en_fibertests_r <= 1'b0;
// were always in lhc_ck domain:
	   en_loopbacks_r <= 0;
	   en_loopbacks_rr <= 0;
	   lout_dmbloop <= 0;
	   llout_dmbloop <= 0;
	   in_dmbloop <= 0;
	   lhc_tog <= 0;
	   lhc_tog_err <= 0;
	   dmbloop_errcnt <= 0;
	   dmbloop1_stat <= 0;
	   dmbloop2_stat <= 0;
	   dmbloop3_stat <= 0;
	   loop_count <= 0;
	end
	else begin
	   lhcloop_tog_r <= lhcloop_tog; // maybe try negedge?  careful about Reset...
	   if (time_40r[7] & locked) l_lock125 <= 1;
	   if (l_lock125 & (!locked)) ck125_locklost <= 1;
	   qpll_lock_lost <= !qpll_lock | qpll_lock_lost;
	   in_pulse_r <= in_pulse;
	   pulses_fired <= (pulses_fired|in_pulse_r);
//	   ccb_cmdstrb_r <= (ccb_cmdstrb & !ccb_cmdstrb_r);
//	   if(ccb_cmdstrb & !ccb_cmdstrb_r) ccb_cmd_r <= ccb_cmd;
//	   else ccb_cmd_r <= 8'h00;
	   if(ccb_cmdstrb & !ccb_cmdstrb_r) begin
	      ccb_cmd_r <= ccb_cmd;
	      ccb_cmdstrb_r <= 1'b1;
	   end
	   else ccb_cmdstrb_r <= 1'b0;

	   ccb_datstrb_r <= (ccb_datstrb & !ccb_datstrb_r);
	   if(ccb_datstrb & !ccb_datstrb_r) ccb_data_r <= ccb_data;
//	   else ccb_data_r <= 8'h00;

	   ccb_rsv_r <= ~_ccb_rx[25:24]; // this is ccb_reserved[3:2] controlled by CSRB6
//	   tmb_l1a_relreq <= ~ccb_rsv_r & ~_ccb_rx[25:24]; // when TMB L1A Release & Request go out, CCB should send an L1A, check!
	   tmb_l1a_relreq <= _ccb_rx[25:24]; // removed pulse logic; probably CCB won't send an L1A, check? UN-inverted this in v2p2

	   en_cabletests_r <= en_cabletests;
	   en_fibertests_r <= en_fibertests;
	   en_lhcloop_r <=  en_lhcloop;

// things that were always in lhc_ck domain...
	   en_loopbacks_rr <= en_loopbacks_r;
	   en_loopbacks_r <= en_loopbacks;
	   llout_dmbloop <= lout_dmbloop;
	   lout_dmbloop <= out_dmbloop;
	   in_dmbloop <= rawin_dmbloop;
	   if (en_loopbacks) lhc_tog <= ~lhc_tog;
	   if (en_loopbacks && en_loopbacks_r && en_lhcloop_r) lhc_tog_err <= (lhcloop_tog ^ lhc_tog);
	   if (locked & en_loopbacks_r) loop_count <= loop_count + 1'b1;
	   dmbloop1_stat <= dmbloop1_stat | err_dmbloop[11:0];
	   dmbloop2_stat <= dmbloop2_stat | err_dmbloop[23:12];
	   dmbloop3_stat <= dmbloop3_stat | err_dmbloop[27:24];
	   if ( |err_dmbloop ) dmbloop_errcnt <= ((dmbloop_errcnt[11:0] + 1'b1) | (dmbloop_errcnt[11]<<11));
//    ^^^^^^^^^^^^^^^^

// this is for our "CCB Pulsed Signal" checks: the "fired" register and a pulse counter based on in_pulse and "trigger"
	   trigger <= ( ( in_pulse_r == 12'h001)||( in_pulse_r == 12'h002)||( in_pulse_r == 12'h004)||( in_pulse_r == 12'h008)||( in_pulse_r == 12'h010)||( in_pulse_r == 12'h020)||( in_pulse_r == 12'h040)||( in_pulse_r == 12'h080)||( in_pulse_r == 12'h100)||( in_pulse_r == 12'h200)||( in_pulse_r == 12'h800)||( in_pulse_r == 12'h400) );
	   if (trigger) pulse_count <= pulse_count + 1'b1;
// JG, check for Result Register instructions for CCB tests (CC plus CD, CE, CF or C0):
	   if (ccb_cmdstrb_r && ccb_cmd_r[7:0]==8'hCC) begin  // check all 8 bits here for "CC"
	      if (results_hold == 1'b0) begin   // first pass for CC cmd
		 tmb_cfg_out <= last_cmd[0]; // put out bit0 on the first CC cmd
// lock results immediately on CMD CC for some commands:    add  qpll_lock_lost etc in Data Bus check CE
		 if (last_cmd[7:0]==8'hCD) results_r <= {pulse_count[11:0],last_cmd[7:0]};    // count of CCB pulsed signals
		 else if(last_cmd[7:0]==8'hCE) results_r <= {ck160_locklost,!lock40,qpll_lock_lost,!qpll_lock ,ccb_data_r[7:0],last_cmd[7:0]}; // CCB data bus content
		 else if(last_cmd[7:0]==8'hCF) results_r <= {pulses_fired[11:0],last_cmd[7:0]};   // observed CCB test pulses
		 else if(last_cmd[7:0]==8'hC0) results_r <= {ccbrxzero_count[11:0],last_cmd[7:0]};   // check ccb_rx0 clocking
		 else  results_r <= {!_ccb_rx[42],~_ccb_rx[50:48],tmb_res_out[2:0],!_ccb_rx[28],~_ccb_rx[25:22],last_cmd[7:0]};   // default is send last cmd along with a bunch of CCB signals to check
	      end

	      else begin    // CC cmd: second pass and beyond
		 tmb_cfg_out <= results_r[get_bit_ptr]; // put out bit[i] on later CC cmds
// lock test results as late as possible for some commands, at end of last_cmd register transmission:
		 if (!late_load_done && get_bit_ptr == 5'h07) begin  // the final bit of "last_cmd" field before sending "results"
		    late_load_done <= 1'b1;  // prevents re-update of Results during an over-looped series of CC cmds
		 if(last_cmd[7:0]==8'hD0) results_r <= {dmbloop_errcnt[11:0],last_cmd[7:0]}; // total errors in DMB loops
		 if(last_cmd[7:0]==8'hD1) results_r <= {dmbloop1_stat[11:0],last_cmd[7:0]};  // DMB loop1 signals with error
		 if(last_cmd[7:0]==8'hD2) results_r <= {dmbloop2_stat[11:0],last_cmd[7:0]};  // DMB loop2 signals with error
		 if(last_cmd[7:0]==8'hD3) results_r <= {dmbloop3_stat[11:0],last_cmd[7:0]};  // DMB loop3 signals with error
//		 if(last_cmd[7:0]==8'hD4) results_r <= {dmbloop4_stat[11:0],last_cmd[7:0]};  // DMB loop4 signals with error
//		 if(last_cmd[7:0]==8'hD5) results_r <= {dmbloop5_stat[11:0],last_cmd[7:0]};  // DMB loop5 signals with error

//		 if(last_cmd[7:0]==8'hD8) results_r <= {rpcloop_errcnt[11:0],last_cmd[7:0]}; // total errors in RPC loops
//		 if(last_cmd[7:0]==8'hD9) results_r <= {rpcloop1_stat[11:0],last_cmd[7:0]};  // RPC loop1 signals with error
//		 if(last_cmd[7:0]==8'hDA) results_r <= {rpcloop2_stat[11:0],last_cmd[7:0]};  // RPC loop2 signals with error
//		 if(last_cmd[7:0]==8'hDB) results_r <= {rpcloop3_stat[11:0],last_cmd[7:0]};  // RPC loop3 signals with error
//		 if(last_cmd[7:0]==8'hDC) results_r <= {rpcloop4_stat[11:0],last_cmd[7:0]};  // RPC loop4 signals with error
//		 if(last_cmd[7:0]==8'hDD) results_r <= {rpcloop5_stat[11:0],last_cmd[7:0]};  // RPC loop5 signals with error
//		 if(last_cmd[7:0]==8'hDE) results_r <= {rpcloop6_stat[11:0],last_cmd[7:0]};  // RPC loop6 signals with error
//		 if(last_cmd[7:0]==8'hDF) results_r <= {rpcloop7_stat[11:0],last_cmd[7:0]};  // RPC loop7 signals with error

//		 if(last_cmd[7:0]==8'hE0) results_r <= {cable_stat[11:0],last_cmd[7:0]};   // all skewclear cables status
//		 if(last_cmd[7:0]==8'hE1) results_r <= {error1count[11:0],last_cmd[7:0]};  // for cable 1
//		 if(last_cmd[7:0]==8'hE2) results_r <= {error2count[11:0],last_cmd[7:0]};  // for cable 2
//		 if(last_cmd[7:0]==8'hE3) results_r <= {error3count[11:0],last_cmd[7:0]};  // for cable 3
//		 if(last_cmd[7:0]==8'hE4) results_r <= {error4count[11:0],last_cmd[7:0]};  // for cable 4
//		 if(last_cmd[7:0]==8'hE5) results_r <= {error5count[11:0],last_cmd[7:0]};  // for cable 5
//		 if(last_cmd[7:0]==8'hF0) results_r <= {fiber_stat[11:0],last_cmd[7:0]};  // all fibers link status
//		 if(last_cmd[7:0]==8'hF1) results_r <= {error_f1count[11:0],last_cmd[7:0]};  // for fiber 1
//		 if(last_cmd[7:0]==8'hF2) results_r <= {error_f2count[11:0],last_cmd[7:0]};  // for fiber 2
//		 if(last_cmd[7:0]==8'hF3) results_r <= {error_f3count[11:0],last_cmd[7:0]};  // for fiber 3
//		 if(last_cmd[7:0]==8'hF4) results_r <= {error_f4count[11:0],last_cmd[7:0]};  // for fiber 4
//		 if(last_cmd[7:0]==8'hF5) results_r <= {error_f5count[11:0],last_cmd[7:0]};  // for fiber 5
//		 if(last_cmd[7:0]==8'hF6) results_r <= {error_f6count[11:0],last_cmd[7:0]};  // for fiber 6
//		 if(last_cmd[7:0]==8'hF7) results_r <= {error_f7count[11:0],last_cmd[7:0]};  // for fiber 7
//		 if(last_cmd[7:0]==8'hFC) results_r <= {cable_count[11:0],last_cmd[7:0]};   // count of cable test cycles
		 if(last_cmd[7:0]==8'hFD) results_r <= {loop_count[31:20],last_cmd[7:0]};// #1.05M counts of Loopback test cycles
//		 if(last_cmd[7:0]==8'hFE) results_r <= {--unused--, last_cmd[7:0]};   //
//		 if(last_cmd[7:0]==8'hFF) results_r <= {fiber_count[11:0],last_cmd[7:0]};   // count of fiber test cycles
		 end
		 
	      end

// end of each CC cmd:
	      if (get_bit_ptr == 5'd19) alct_cfg_out <= 1'b1;
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
              if (ccb_cmd_r[7:2]==6'h06) begin  // this is the CCB "Start Trig" CMD, use it to start tests
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
     end // always @ (posedge lhc_ck or posedge reset)


   assign _ccb_tx[19] = alct_cfg_out;  // JG, don't invert
   assign _ccb_tx[18] = tmb_cfg_out;   // JG, don't invert
   assign _ccb_tx[17:9] = pulse_count[8:0];  // 9 bit alct_status, to CCB Front Panel (our LED plug)
   

   always @(posedge gbe_txclk2) // everything that uses GbE USR clock, no Reset
     begin
	tx_out_r  <= tx_out;
	tx_kout_r <= tx_kout;
	time_r_i  <= time_count;
	data_bram_r  <= data_bram;
	data_cram_r  <= data_cram;
	if (bk_adr[11:8] == 4'hc) tx_cadr <= {bk_adr[5:0],tx_adr[10:0]};  // get 4 kB CROM pages from one big block
	lerr_count <= err_count;
     end

   always @(posedge gbe_txclk2 or posedge reset) // everything using GbE USR clock w/simple Reset
     begin
	if (reset) begin
	   l_lock40 <= 0;    // use gbe ck125 to monitor ck40(160)
	   ck160_locklost <= 0;
	   time_r <= 8'h00;
	   pkt_time_r <= 23'h00000001;
	end
	else begin
	   if (time_r[7] & lock40) l_lock40 <= 1;
	   if (l_lock40 & (!lock40)) ck160_locklost <= 1;
	   time_r <= time_r_i;  // transfer slow slwclk time counter to GbE USR clock domain
	   pkt_time_r <= pkt_time;
	end
     end

   always @(posedge gbe_txclk2 or posedge gtx_reset) // everything using GbE USR clock w/GTX_Reset
     begin
	if (gtx_reset) begin
	   comma_align <= 0;
	   counter_send <= 0;
	   l_rxdv <= 0;
	   l_kchar <= 0;
	   ll_kchar <= 0;
	   l_gbe_rxdat <= 0;
	   ll_gbe_rxdat <= 0;
	   kchar_r <= 0;
	   ovfl_packet <= 0;
	   counter <= 16'h0000;
	   ireg <= 0;
	   tx_adr <= 0;
	   free_tc_r <= 1'b0;
	   loop_command <= 1'b0;
	   pkt_id <= 8'h0000;
	   gbe_rxcount <= 16'h0000;
	   cmd_code <= 16'h0000;
	   comp_phase <= 3'h1;
	   comp_dav_r <= 0;
	   send_triad <= 0;
	   bk_adr <= 12'h000;
	   gbe_txdat <= 16'h50bc;  // <-- idle 28.5/16.2
	   gbe_kout <= 2'h1;
	   sync_state <= 2'h0;
	   data_state <= 2'h0;
	   mac_seek <= 0;
	   mac_sync <= 0;
	   mac_ack  <= 0;
	   rx_resetdone_r2 <= 0;
	   rx_resetdone_r3 <= 0;
	   crc_rst <= 1;
	   crc_en <= 0;
	   pkt_send <= 0; // 1st data word @counter == 12 --> 11 extra words, so subtract 22 bytes.
	   good_rx_cmd <= 1'b0;
	   rx_timeout <= 1'b0;
	end

	else begin // Not Reset case
	   if (sw[8]) slow_tc_r <= slow_tc;   // sync to gbe_clk
	   else comp_dav_r <= ( !no_comp_dav ); // sync to gbe_clk.  JG: add free_tc to slow it down?
	   if (cmd_code == 16'h0000) begin
	      loop_command <= slow_tc_r;
	      send_triad <= comp_dav_r;
	   end

	   rx_resetdone_r2 <= rx_resetdone_r;
	   rx_resetdone_r3 <= rx_resetdone_r2;
	   if (rx_resetdone_r3) comma_align <= 1; // maybe use time_r > 8'h10 ?

	   crc_rst <= (data_state == 2'h2)? 1'b1 : 1'b0;  // needs one cycle during state 3

//	   if (cmd_code == 16'hf3f3 || loop_command) begin  // load counter2 to READ registers and send it out
	   if (cmd_code == 16'hf3f3) begin  // load counter2 to READ registers and send it out
	      pkt_lim <= 16'd2059;   // 16'd61 was special test size 100 bytes. Final size is 4 KB.
	   end
	   else if (loop_command | send_triad) pkt_lim <= 16'd38; // JG: increase to 3 triad size (38=54 bytes)
	   else pkt_lim <= 16'd36; // 36 was the old default 50-byte size.

	   byte_count <= 16'hffff&((pkt_lim*2) - 16'd22); // Need BYTES; subtract preamble bytes etc.

	   if (data_state[1]) begin  // end of sent data packet
	      cmd_code <= 16'h0000;
	      bk_adr <= 12'h000;
	      pkt_send <= 1'b0;
	      counter <= 0;
	      loop_command <= 1'b0;
	      send_triad   <= 1'b0;
	      if (data_state == 2'h2) pkt_id <= pkt_id + 1'b1;
	   end
	   else if (!sync_state[0] && pkt_send) begin
	      counter <= counter + 1'b1;  // verified that sync[0] ends OFF
	   end
 	   else if (!sync_state[0] && time_r[7] && (cmd_code > 16'h0000)) pkt_send <= 1'b1;

	   if (loop_command && (cmd_code == 16'h0000) ) cmd_code <= 16'hf4f4;
	   if (send_triad && (cmd_code == 16'h0000) )  cmd_code <= 16'hf5f5;

	   if ((counter > 0) && (counter <= pkt_lim)) begin  // put data on gbe_txdat after preamble, MACaddr's & Length
	      counter_send <= 1;
	      if (counter == 16'd1) begin
		 gbe_txdat <= 16'h55fb;
		 gbe_kout <= 2'h1;
		 tx_adr <= 0;
	      end
	      else if (counter < 16'd5) begin
		 gbe_txdat <= {counter[2],15'h5555};
		 gbe_kout <= 2'h0;
	      end
	      else if (counter < 16'd7) begin
		 crc_en <= 1;
		 gbe_txdat <= 16'hffff; // MAC32 No Longer inverts gbe_txdat input w/counter == 5 or 6; just send ones.
	      end
	      else if (counter < 16'd11) begin
		 gbe_txdat <= 16'h0000;
		 if (counter == 16'd10) tx_adr <= 11'h001; // load tx_bram pipeline, set address#1; data#0 goes in next clock
	      end
	      else if (counter < 16'd12) begin  // only one cycle for this step!  counter == 11
		 gbe_txdat <= {byte_count[7:0],byte_count[15:8]};
		 tx_adr <= 11'h002; // shifting 2-deep tx_bram pipeline, set address#2; data#1 goes in next clock
		 ireg <= 0;
	      end
	      else begin // if (counter >= 16'd12).  first data word @counter == 12 --> 11 counts ahead!
		 tx_adr <= tx_adr + 1'b1;
		 if (counter > 16'd12) ireg <= ireg + cmd_code[1] + (~counter[0]&cmd_code[0]); // 16b F2 & 32b F1 counts
		 if(!cmd_code[2] & (cmd_code[1]^cmd_code[0])) begin  // generalized for f1f1 or f2f2 ( == !f3f3 && !f4f4)
		    if (counter == 16'd12) begin
		       gbe_txdat <= cmd_code;    //  <-- first word returns the cmd_code
		    end
		    else if (cmd_code == 16'hf1f1 && ireg < 8) begin  //  "en_snapshift" signal for the Snap12s
		       if (counter[0]) gbe_txdat <= 0; // was 8 32-bit wide elements in this array
		       else gbe_txdat <= 0;
		    end
		    else if (ireg < 24) gbe_txdat <= 0; // was 24 16-bit wide elements in counting array
		    else gbe_txdat <= 16'h0000; // ireg out of range
		 end
		 else if (cmd_code[2] & cmd_code[0]) begin  // Function5 is active
		    comp_phase <= {comp_phase[1:0],comp_phase[2]};
		    if (comp_phase[0]) gbe_txdat <= comp_dout[15:0];
		    else if (comp_phase[1]) gbe_txdat <= comp_dout[31:16];
		    else gbe_txdat <= comp_dout[47:32];
		 end

		 else if (cmd_code[2]) begin  // Function4 is active
//		   gbe_txdat <= {pkt_id[7:0],tx_adr[7:0]};  // send the sequential packet ID for each packet & word counter
		   if (tx_adr[10:0] < 11'h004) begin
		      gbe_txdat <= {pkt_id[7:0],lerr_count[7:0]};  // send the seq. packet ID for each packet & err count
		   end
		    else begin
		      gbe_txdat <= lerr_count[15:0];  // send the full error count
		    end
		 end

		 else if (bk_adr < 12'hc00) begin  // this is data_bram range
		   gbe_txdat <= data_bram_r[15:0];  // added a pipe register for data_bram, allows better timing?
		 end
		 else gbe_txdat <= data_cram_r[15:0];  // this is data_cram range

	      end // if (counter >= 16'd12)
	   end // if (counter > 0 && counter <= GBE_LIM && pkt_send)
//jg note: the MAC_ACK etc still don't work (v11p5)...do we care?
	   else if (!time_r[7] || sync_state[0]) begin  // send Sync "pairs" for a while after Reset
	      if (time_r[6] && time_r[1]) begin
		 mac_seek <= (ll_kchar && !l_kchar && ll_gbe_rxdat == 16'h42bc);
		 if (mac_seek && l_kchar && l_gbe_rxdat == 16'hb5bc) mac_sync <= 1'b1;
		 if (mac_seek && mac_sync && !ll_kchar && ll_gbe_rxdat == 16'h4000) mac_ack <= 1'b1;
	      end
	      else mac_seek <= 0;

	      case (sync_state)   // was casez
		2'b00: gbe_txdat <= 16'h42bc;
		2'b10: gbe_txdat <= 16'hb5bc;
		default: gbe_txdat <= {1'b0,mac_sync,14'h0000}; // <-- we want this to be the final sync state.
	      endcase
	      gbe_kout  <= ((~sync_state) & 2'b01);
	      sync_state <= sync_state + 1'b1;
	   end
	   else  begin // otherwise send Idles, allows time for Rx Checking
	      counter_send <= 0;
	      crc_en <= 0;  // make one clock sooner?  Looks ok here...
	      gbe_txdat <= 16'h50bc;  // <-- idle 28.5/16.2
	      gbe_kout <= 2'h1;
	      tx_adr <= 0;
	      comp_phase <= 3'h1;
	      if (counter_send) data_state <= 2'h1;
	      else if (data_state > 2'h0) data_state <= data_state + 2'h1;
	   end

	   l_rxdv <= rxdv;  // note, rxdv may stay True for over .5 sec after Reset!  Why...?
	   ll_kchar <= l_kchar;
	   l_kchar <= rxer[0]|rxer[1];
	   kchar_r <= rxer[1:0];
	   ll_gbe_rxdat <= l_gbe_rxdat;
	   l_gbe_rxdat <= gbe_rxdat;

	   if (time_r[7] && (rxdv > 0) && (sw[7] || !sw[8]) ) begin // get command from first 1-2 data bytes of packet
	      if (gbe_rxcount == 16'd4407) begin  // tuned to handle weird .5 sec rxdv case after Reset.
		 rx_timeout <= 1'b1;
		 ovfl_packet <= ovfl_packet + 1'b1; // tracks when a rx packet times-out, over jumbo GbE limit
		 gbe_rxcount <= 16'h0000;  // ^^^ change to ovfl_packet
		 cmd_code <= 16'h0000;
		 bk_adr <= 12'h000;
		 counter <= 0;
		 pkt_send <= 1'b0;
		 tx_adr <= 0;
	      end
	      else if (!rx_timeout) gbe_rxcount <= gbe_rxcount + 1'b1;

	      if ( (gbe_rxcount == 16'h0003) && (cmd_code == 16'h0000) && (sw[7] || !sw[8]) ) begin
//   else if cmd_code != 0) next_cmd <= gbe_rxdat;   to allow stacking up commands?
		 if ( (gbe_rxdat&16'hf0f0)==16'hf0f0 && (gbe_rxdat&16'h0f0f)!=16'h0f0f ) begin
		    cmd_code <= gbe_rxdat; //   ^^^ make sure cmd code looks valid ^^^
		    good_rx_cmd <= 1'b1;
		 end
	      end
	      else if (gbe_rxcount == 16'h0004 && cmd_code == 16'hf3f3) begin
	        //     **> hex b00 = 2816 dec.   hex c00 = 3072 dec.
		 if (gbe_rxdat[11:8] == 4'hb || gbe_rxdat[11:8] == 4'hc) bk_adr <= gbe_rxdat[11:0]; // test gbe_rxdat for b & c
	        // use "ABxx" for block ROMs and "ACxx" for CLB ROMs.
	      end
	      else begin
		 good_rx_cmd <= 1'b0;
	      end // else: !if(gbe_rxcount == 3 or 4)
	   end  // if (time_r[7] & (rxdv > 0))

	   else  begin  // if (rxdv == 0) or time_r[7]==0
	      gbe_rxcount <= 16'h0000;
	      rx_timeout <= 1'b0;
	   end

	end // else: !if(gtx_reset)
     end

   always @(*)
     begin  // select what goes on the BRAM bus.  Later add the CLBram bus.
	if (bk_adr < 12'hb00 || bk_adr > BRAM_LIM) data_bram = 16'hba0f; // limited range of bk_adr space
	else if (cmd_code == 16'hf3f3) data_bram = data_rom[bk_adr];
	else data_bram = 16'h0000;

	if ( bk_adr < 12'hc00 || bk_adr > (12'hbff + (CRAM_LIM+1)/2048) ) data_cram = 16'hca0f; // limit bk_adr space
	else if (cmd_code == 16'hf3f3) data_cram = cram[tx_cadr];
	else data_cram = 16'h0000;

  	if (data_state == 2'h1) begin  // <-- last "counter_send" is in time with data_state 1
	   tx_out[15:0] = crc_out[15:0];  // send out CRC "MSB-first"; it's already inverted and backwards
	   tx_kout = 2'h0;
	end
	else if (data_state == 2'h2) begin
	   tx_out[15:0] = crc_out[31:16];
	   tx_kout = 2'h0;
	end
	else if (data_state == 2'h3) begin
	   tx_out[15:0] = 16'hf7fd; // eof code
	   tx_kout = 2'h3;
	end
	else begin    // tx_out control returns to gbe_txdat w/Idles
	   tx_out[15:0] = gbe_txdat[15:0];
	   tx_kout = gbe_kout;
	end
     end  // always @ (*)


   always @(posedge rx_clk) // everything that uses 160 MHz snap USR clock, w/o Reset
     begin
	time_r_snap <= time_count;
	time_snap <= time_r_snap;  // transfer slow slwclk time counter to snap USR clock domain
     end

   assign push_fifo = ( word[0] && save_triad );
   always @(posedge rx_clk or posedge reset or posedge snap_wait) // things that uses 160 MHz snap USR clock, w/Reset
     begin
	if (reset | snap_wait) begin
	   err_count <= 0;  // use this to count PRBS errors?
	   comp_dat_r  <= 0;
	   comp_dat_2r <= 0;
	   comp_dat_3r <= 0;
	   comp_dat_4r <= 0;
	   comp_dat_5r <= 0;
	   save_triad  <= 0;
	   itriad  <= 0;
	end

	else begin // Not Reset case
//  set itriad=9  if ( !sw[8] && itriad==0 && |comp_dat_r > 0 )  can we pick a few specific bits?  TRIAD FORMAT!
// push FIFO if (word[0] && itriad > 0 )
	   if (!sw[8] && !save_triad && |nz && word[0]) begin  // 6-input logic
	      itriad <= 9;
	      save_triad <= 1;
	   end
	   else if (push_fifo) begin
	      itriad <= itriad - 1'b1;
	      if(itriad == 4'h1)save_triad <= 0;
	   end

	   if (word[0]) begin
	      comp_dat_r   <= comp_dat;
	      comp_dat_2r   <= comp_dat_r;
	      comp_dat_3r   <= comp_dat_2r;
	      comp_dat_4r   <= comp_dat_3r;
//	      comp_dat_5r   <= comp_dat_4r;
	   end

	   if ( !snap_wait & time_snap[7] ) begin  // wait 3000 clocks after Reset
	      if ( !rx_match & rx_valid ) err_count <= err_count + 1'b1;  // send to GbE ~1 Hz
	   end
        end

     end



/*  switch modes
   sw7 & sw8:   send err_count at 1.6 Hz; PB forces an error  (use w/Tx tester, OK at TAMU)
   !sw7 & sw8:  send err_count at 1.6 Hz; PB is Reset  (good to use at OSU & w/tester at TAMU)

   sw7 & !sw8:  send comp_dat when non-zero; PB forces a triad pattern  (for Tx tester at TAMU)
   !sw7 & !sw8: send comp_dat when non-zero; PB is Reset  (good to use at OSU & w/tester at TAMU)
*/
   always @(*)
     begin   // JG, v1p16: swap LED 3<>4, notes and all
	led_low[0] = !_ccb_rx[22]; // ccb_ttcrx_rdy, always OFF
	led_low[1] = !_ccb_rx[23]; // ccb_qpll_lck, OFF but often blinks ON (very short)
	led_low[2] = qpll_lock;    // always ON!   was !_ccb_rx[31] == _alct_adb_pulse_async
	led_low[3] = qpll_lock_lost; // always OFF?  was _ccb_rx[35] == mpc_in1, always ON
	led_low[4] = _ccb_rx[34]; // mpc_in0, always ON
//	led_low[5] = !_ccb_rx[12];   // L1accept
//	led_low[6] = trigger;      // OR of several trigger pulse inputs
	led_low[7] = (!pb | !_ccb_rx[1]);    // just reset, includes ccb_rx[1]==L1reset
//	led_low[0] = sw[8];  // on    to TMB FP LEDs.  is this LED[7:0]? Prom D[7:0] should control on-TMB LEDs.
//	led_low[1] = sw[7];  // off
//	led_low[2] = !pb;     // on w/pb.   change to reset?  can we use CCB_Resync?
//	led_low[3] = gbe_fok;     // on
//	led_low[4] = synced_snapr; // on  // == RX_SYNC_DONE.  was all_ready
	led_low[5] = ck160_locked; // always ON!  // Tx GTX PLL Ref lock
	led_low[6] = !ck160_locklost; // --> ON!  comes from mmcm driven by tx_pll_ck160 in FPGA
//	led_low[7] = qpll_lock;    // on
	if (sw[7] == 0) begin
	   if (sw[8] == 1) begin // sw8 ON, 7 OFF:  send err_count at 1.6 Hz; PB is Reset  (good to use at OSU)
	      led_hi[8] = all_ready;  //  --> off, but ON during reset
	      led_hi[9] = !gtx_reset;   //  == (!gtx_ready | reset)  --> off, but ON during reset
 	      led_hi[10] = gbe_ready; //  --> off, but ON during reset
	      led_hi[11] = !pkt_send;  // off
	      led_hi[12] = qpll_ck40;   // was gbe_fok --> ON
	      led_hi[13] = locked;    //  --> off
	      led_hi[14] = ck160_locked;  //  --> on sometimes  // Tx GTX PLL Ref lock
	      led_hi[15] = lock40;    //  --> on ~always // from Tx GTX PLL out clk
	      test_led[1] = tx_begin;  // CHECK.  best for !sw7 case
	      test_led[2] = ck40;      // used to check ck40 freq. from snap gtx rx_clk
	      test_led[3] = tx_fc;     // CHECK.
	      test_led[4] = synced_snapr; // == RX_SYNC_DONE.  was all_ready
	      test_led[5] = rx_strt;   // CHECK.
	      test_led[6] = !ck160_locklost; // this comes from mmcm driven by tx_pll_ck160
	      test_led[7] = rx_valid;  // CHECK.
	      test_led[8] = qpll_lock; //   was snap_wait
	      test_led[9] = rx_match;  // CHECK.
	      test_led[10]  = rx_fc;   // == LTNCY_TRIG    was == 1
//	      led_low = 0;
	   end
// tx_begin (!sw7), tx_fc, rx_strt, rx_valid, rx_match

// JGhere, add 7 signals to 0,0 case: ccb_cmdstrb & lhc_ck & ccb_cmd[0], tmb_cfg_out, alct_cfg_out, ccb_datstrb & ccb_data[0]?
//                                    
	   else begin            // Both OFF: send comp_dout when non-zero; PB is Reset  (good to use at OSU)
	      led_hi[8] = all_ready;  //  --> off, but ON during reset
	      led_hi[9] = !gtx_reset;   //  == (!gtx_ready | reset)  --> off, but ON during reset
 	      led_hi[10] = time_r[7];  // * off, but ON during reset
	      led_hi[11] = !pkt_send;  //  --> off
	      led_hi[12] = qpll_ck40;   //  was gbe_fok --> ON
	      led_hi[13] = locked;    //  --> off
	      led_hi[14] = ck160_locked; //  --> off  // Tx GTX PLL Ref lock
	      led_hi[15] = lock40;    //  --> off // from Tx GTX PLL out clk
	      test_led[1] = in_dmbloop[3];  // tx_begin;  // CHECK.  best for !sw7 case HI    **ALWAYS 0
	      test_led[2] = lhc_ck;      // * used to check lhc_ck freq.
	      test_led[3] = ccb_cmdstrb;  // tx_fc;     // CHECK.  13ns high pulse w/3.2usec period
	      test_led[4] = ccb_cmd[0];   // all_ready; // HI
	      test_led[5] = dmbfifo_step1ck;  // rx_strt;   // CHECK. LOW
	      test_led[6] = good_dmbloop[3]; // !ck125_locklost;  // *
	      test_led[7] = ccb_datstrb;  // rx_valid;  // CHECK. LOW
	      test_led[8] = ccb_data[0];  // comp_overflow; // LOW
	      test_led[9] = err_dmbloop[3];    // rx_match;  // CHECK.  LOW
	      test_led[10]  = llout_dmbloop[3]; // rx_fc;    // was comp_phase[2]. Trigger all the time!

/*	      test_led[8:1]   = rxdvr_snap[7:0];
	      test_led[9] = snap_wait; //

  	      test_led[8:1] = l_gbe_rxdat[7:0]; // counter[7:0];
	      test_led[9]   = kchar_r[0]; // gbe_fok;
	      test_led[10]  = l_rxdv;  // all_ready;  
 */
//	      led_low = 8'b11111111;   // Prom D[7:0], go to TMB LEDs
	   end
	end
	else if (sw[8] == 0) begin  // *** sw7 ON from here on: send comp_dout when non-zero; PB forces a triad pattern
	   //	                               (for Tx tester at TAMU)
// Mezz LEDs are inverted...TMB FP LEDs are not...test_leds are not.
  	   led_hi[8]  = !save_triad;// off
	   led_hi[9]  = !crc_en;    // off  should go True at end of 2nd h55, False at end of hFE.
	   led_hi[10] = !comp_overflow; //  --> off
	   led_hi[11] = !pkt_send;  //  --> off
	   led_hi[13:12] = ~sync_state[1:0];    // after PROG, sometimes ON/always OFF
	   led_hi[15:14] = ~data_state[1:0];    // ALWAYS OFF/OFF!
	   test_led[1] = err_dmbloop[3];   // push_fifo;  // 2nd.  9 ~7ns Hi pulses @25ns spacing   used at OSU
	   test_led[2] = good_dmbloop[3];  // save_triad; // 1st.  Hi for 225ns, starting ~20ns before PushFIFO  used at OSU
	   test_led[3] = good_dmbloop[27]; // send_triad; // 3rd.  Hi ~702ns, starting ~158ns after SaveTriad.   used at OSU
//	   test_led[1] = push_fifo;  // 2nd.  9 ~7ns Hi pulses @25ns spacing   used at OSU
	     // ^^  Add WIRE.  Trigger all the time!
//	   test_led[2] = save_triad; // 1st.  Hi for 225ns, starting ~20ns before PushFIFO   used at OSU
//	   test_led[3] = send_triad; // 3rd.  Hi ~702ns, starting ~158ns after SaveTriad.   used at OSU
	     // ^^  CHECK.  32ns low pulse every 730ns!
//	   test_led[4] = all_ready; // HI   used at OSU
	   test_led[4] = lhc_ck; // clk 40 from Mezz QPLL, CLEAN clk, random phase from LHC Clock
	   test_led[5] = lhc_tog;
//	   test_led[5] = rx_strt;   // change?  LOW   used at OSU
//	   test_led[6] = no_comp_dav;  // * LOW    used at OSU
	   test_led[6] = en_loopbacks_r;
	   test_led[7] = lhc_tog_err; // usually = err_dmbloop[27]
//	   test_led[7] = rx_valid;  // CHECK.  LOW   used at OSU
//	   test_led[8] = comp_phase[0]; // used at OSU
	   test_led[8] = dmbfifo_step1ck; // clk from DMB mainboard via Loopback
	   test_led[9] = lhcloop_tog; // ^^compare  lhcloop_tog  to  lhc_tog
//	   test_led[9] = rx_match; // used at OSU;  was comp_phase[1];  // CHECK.  LOW
//	   test_led[10]  = rx_fc; // used at OSU;  was comp_phase[2];    
	   test_led[10]  = lhcloop_tog_r;
/*
	   test_led[8:1] = snap_rxdat_r[7:0]; // tx_out or l_gbe_rxdat[15:8];
	   test_led[9]   = snap_rxk_r[0];  // tx_kout_r or kchar_r[1];
	   test_led[10]  = rxdvr_snapr[2];  // counter_send
	      test_led[4:1] = l_gbe_rxdat[3:0]; // counter[7:0];
	      test_led[8:5] = cmd_code[3:0]; // counter[7:0];
	      test_led[9]   = good_rx_cmd; // gbe_fok;
	      test_led[10]  = l_rxdv;  // all_ready;  */
	end    // if (sw[8] == 0)
	else begin  // Both ON: send err_count at 1.6 Hz; PB forces an error (use w/Tx tester at TAMU)
	      led_hi[8] = all_ready;  // --> off
	      led_hi[9] = !gtx_reset;   //  == !gtx_ready | reset --> off
 	      led_hi[10] = time_r[7];  // * off
	      led_hi[11]  = !crc_en;   // "off"  goes True at end of 2nd h55, False at end of hFE?
	      led_hi[12] = !err_count[2]; // toggles with PB
	      led_hi[13] = locked;    //  --> off
	      led_hi[14] = ck160_locked; //  --> off  // Tx GTX PLL Ref lock
	      led_hi[15] = lock40;    //  --> off // from Tx GTX PLL out clk
	      test_led[1] = tx_begin;  //  CHECK.  best for !sw7 case
	      test_led[2] = ck40;    // * used to check ck40 freq. from gtx_Tx, from QPLL 160 MHz clock
	      test_led[3] = tx_fc;     // CHECK.
	      test_led[4] = all_ready; // 
	      test_led[5] = rx_strt;   // CHECK.
	      test_led[6] = !ck125_locklost;  // *
	      test_led[7] = rx_valid;  // CHECK.
	      test_led[8] = snap_wait; // 
	      test_led[9] = rx_match;  // CHECK.
	      test_led[10]  = rx_fc;    // was High
//	      led_low = 8'b11111111;   // Prom D[7:0], go to TMB LEDs
	end
     end

   
//   mmcm from 80 MHz:         In,    out80,  out160,   out40, reset,         locked
   bufg_x2div2 snap_mmcm (tx_clk_out, tx_clk, snap_clk2, ck40, !ck160_locked, lock40); // from Tx GTX PLL out clk

// jghere:  was Snap12 module, not cfeb-tmb test code
   tmb_fiber_out  dcfeb_out (
	.RST (reset),   // Manual only
	.TRG_SIGDET (), // from IPAD to IBUF.  N/A?
	.TRG_RX_N (),   // empty
	.TRG_RX_P (),   // empty
	.TRG_TDIS (),   // OBUF output, for what?  N/A?
	.TRG_TX_N (txn[7]),   // pick a fiber, match LOC constraint in module
	.TRG_TX_P (txp[7]),   // pick a fiber
	.G1C (triad_word[7:0]),  // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
	.G2C (triad_word[15:8]),  //   if ENA_TEST_PAT then it's prbs so these don't matter...
	.G3C (triad_word[23:16]),  //   unless I want to test low-rate non-zero triad data:
	.G4C (triad_word[7:0]),
	.G5C (triad_word[15:8]),
	.G6C (triad_word[23:16]),
	.TRG_TX_REFCLK (ck160),  // QPLL 160 from MGT clk
	.TRG_TXUSRCLK (snap_clk2),  // get 160 from TXOUTCLK (times 2)
	.TRG_CLK80 (tx_clk),     // get 80 from TXOUTCLK
	.TRG_GTXTXRST (0),   // maybe Manual "reset" only
	.TRG_TX_PLLRST (0),  //  Tie LOW.
	.TRG_RST (gtx_reset),       // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
	.ENA_TEST_PAT (sw[8]),   // HIGH for PRBS!  (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
	.INJ_ERR (ferr_rr & sw[8]),  // use my switch/PB combo logic for this, high-true? Pulse high once.
	.TRG_SD (),          // from IBUF, useless output. N/A
	.TRG_TXOUTCLK (tx_clk_out),   // 80 MHz; This has to go to MCM to generate 160/80
	.TRG_TX_PLL_LOCK (ck160_locked), // inverse holds the MCM in Reset  // Tx GTX PLL Ref lock
	.TRG_TXRESETDONE (), // N/A
	.TX_SYNC_DONE (synced_snapt), // use inverse to hold logic in Reset in many places
	.STRT_LTNCY (tx_begin),  // after every Reset, to TP for debug only  -- !sw7 ?
	.LTNCY_TRIG (tx_fc),     // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
	.MON_TX_SEL (),      //  N/A
	.MON_TRG_TX_ISK (),  //  N/A returns 4 bits
	.MON_TRG_TX_DATA ()  //  N/A returns 32 bits
	);

   comp_fiber_in dcfeb_in (
	.CMP_RX_VIO_CNTRL (), // empty or Delete from code
	.CMP_RX_LA_CNTRL (),  // empty or Delete from code
	.RST (gtx_reset),     // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
        .CMP_SIGDET (),    //  N/A
        .CMP_RX_N (rxn[7]),      // pick a fiber, set LOC constraint
        .CMP_RX_P (rxp[7]),      // pick a fiber
	.CMP_TDIS (),      //  N/A
	.CMP_TX_N (),      // empty
	.CMP_TX_P (),      // empty
        .CMP_RX_REFCLK (ck160), // QPLL 160 via GTX Clock
        .CMP_SD (),        // from IBUF, useless output. N/A
	.CMP_RX_CLK160 (rx_clk), // Rx recovered clock out.  Use for now as Logic Fabric clock
			           //   Needed to sync all 7 CFEBs with Fabric clock!
	.STRT_MTCH (rx_strt), // gets set when the Start Pattern is present, N/A for me.  To TP for debug only.  --sw8,7
	.VALID (rx_valid),    // send this output to TP (only valid after StartMtch has come by)
	.MATCH (rx_match),    // send this output to TP  AND use for counting errors
			           // VALID="should match" when true, !MATCH is an error
        .RCV_DATA (comp_dat),  // 48 bit comp. data output, send to GbE if |48  > 0
		 	 //  keep 3 48-bit words now,  3 48-bit words before,  plus 3 48-bit words after
        .NONZERO_WORD (nz),
        .CEW0 (word[0]),        // access four phases of 40 MHz cycle...frame sep. out from GTX
        .CEW1 (word[1]),
        .CEW2 (word[2]),
        .CEW3 (word[3]),        // on CEW3_r (== CEW3 + 1) the RCV_DATA is valid, use to clock into pipeline
        .LTNCY_TRIG (rx_fc),  // flags when RX sees "FC" for latency msm't.  Send raw to TP  --sw8,7
        .RX_SYNC_DONE (synced_snapr) // use inverse of this as Reset for GTXs etc?
	);

   fifo_48b_256_async comp_fifo (
	.rst    (reset),
	.wr_clk (rx_clk),
	.rd_clk (gbe_txclk2),
	.din    (comp_dat_4r),
	.wr_en  (push_fifo),
	.rd_en  (comp_phase[2]),
	.dout   (comp_dout),
	.full   (comp_overflow),
	.empty  (no_comp_dav)
	);

   assign send_lim = 16'h0800; // h0800=dec 2048, h0300=dec 768  --now try to ignore this!
   assign snap_wait = !(synced_snapr & ck160_locked);  // allows pattern checks when RX is ready


// logic for rxdv False:  rxer[0-5]
GBE_T20R20 gbe_gtx (
        gbe_refck,     // GTX0_DOUBLE_RESET_CLK_IN,
    //---------------------- Loopback and Powerdown Ports ----------------------
        low[2:0],//  GTX0_LOOPBACK_IN,
        low[1:0],//  GTX0_RXPOWERDOWN_IN,
    //--------------------- Receive Ports - 8b10b Decoder ----------------------
        rx_comma,  //  GTX0_RXCHARISCOMMA_OUT,  ***1
        rxer[1:0], //  GTX0_RXCHARISK_OUT,  ***2
        rxer[3:2], //  GTX0_RXDISPERR_OUT,  ***3
        rxer[5:4], //  GTX0_RXNOTINTABLE_OUT,  ***4
        rx_disp,   //  GTX0_RXRUNDISP_OUT,  ***5
    //----------------- Receive Ports - Clock Correction Ports -----------------
        ignore[2:0], //  GTX0_RXCLKCORCNT_OUT,
    //------------- Receive Ports - Comma Detection and Alignment --------------
        comma_align,   //   GTX0_RXENMCOMMAALIGN_IN,
        comma_align,   //   GTX0_RXENPCOMMAALIGN_IN,
    //----------------- Receive Ports - RX Data Path interface -----------------
        gbe_rxdat,    // GTX0_RXDATA_OUT,  compare to count_out
        !txoutclk_mmcm_lk,   //   GTX0_RXRESET_IN,  // hold reset until rxPLL and tx_usrclk are locked
        gbe_txclk2,   //   GTX0_RXUSRCLK2_IN,
    //----- Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR ------
        ignore[3], //  GTX0_RXELECIDLE_OUT,
        gbe_rxn,   //  GTX0_RXN_IN,
        gbe_rxp,   //  GTX0_RXP_IN,
    //------ Receive Ports - RX Elastic Buffer and Phase Alignment Ports -------
        reset,       // GTX0_RXBUFRESET_IN,  // don't need this one
        rx_bufstat,  // GTX0_RXBUFSTATUS_OUT,  --ignore these in RxDV?
    //------------- Receive Ports - RX Loss-of-sync State Machine --------------
        rx_lostsync, //  GTX0_RXLOSSOFSYNC_OUT,  *** 6    Use this where???
    //---------------------- Receive Ports - RX PLL Ports ----------------------
        reset,     //  GTX0_GTXRXRESET_IN,  // should use this one
        ckgbe,     //  GTX0_MGTREFCLKRX_IN,
        zero,      //  GTX0_PLLRXRESET_IN,  // should not use this one
        rxpll_lk,  //  GTX0_RXPLLLKDET_OUT,
        rx_resetdone,  //  GTX0_RXRESETDONE_OUT,
    //-------------- Transmit Ports - 8b10b Encoder Control Ports --------------
        zero2,       //  GTX0_TXCHARDISPMODE_IN,  ***7
        zero2,       //  GTX0_TXCHARDISPVAL_IN,  ***8
        tx_kout_r,   //  GTX0_TXCHARISK_IN,  ***9
    //----------------------- Transmit Ports - GTX Ports -----------------------
        low[12:0],   //  GTX0_GTXTEST_IN,
    //---------------- Transmit Ports - TX Data Path interface -----------------
        tx_out_r,    //  GTX0_TXDATA_IN,
        gbe_tx_outclk,   //  GTX0_TXOUTCLK_OUT,
        !rxpll_lk,   //  GTX0_TXRESET_IN,  // hold reset until rxPLL is locked!!!
        gbe_txclk2,     //  GTX0_TXUSRCLK2_IN,
    //-------------- Transmit Ports - TX Driver and OOB signaling --------------
        gbe_txn,   //  GTX0_TXN_OUT,
        gbe_txp,   //  GTX0_TXP_OUT,
    //--------------------- Transmit Ports - TX PLL Ports ----------------------
        reset,     //  GTX0_GTXTXRESET_IN,  // should use this one
        tx_resetdone //  GTX0_TXRESETDONE_OUT
	);

endmodule




// Lower-level modules HERE (in lieu of includes)


(* CORE_GENERATION_INFO = "bufg_div25clk,clk_wiz_v1_8,{component_name=bufg_div25clk,use_phase_alignment=true,use_min_o_jitter=false,use_max_i_jitter=false,use_dyn_phase_shift=false,use_inclk_switchover=false,use_dyn_reconfig=false,feedback_source=FDBK_AUTO,primtype_sel=MMCM_ADV,num_out_clk=1,clkin1_period=8.0,clkin2_period=10.0,use_power_down=false,use_reset=false,use_locked=true,use_inclk_stopped=true,use_status=false,use_freeze=false,use_clk_valid=false,feedback_type=SINGLE,clock_mgr_type=MANUAL,manual_override=false}" *)
module bufg_div25clk
 (// Clock in ports
  input         CLK_IN1,
  // Clock out ports
  output        CLK_OUT1,
  // Status and control signals
  output        INPUT_CLK_STOPPED,
  output        LOCKED
 );

// User entered comments
//----------------------------------------------------------------------------
// Assumes 125 MHz in clk from bufg, then 5 MHz out
//
//----------------------------------------------------------------------------
// Output     Output      Phase    Duty Cycle   Pk-to-Pk     Phase
// Clock     Freq (MHz)  (degrees)    (%)     Jitter (ps)  Error (ps)
//----------------------------------------------------------------------------
// CLK_OUT1     5.000      0.000      50.0      780.112    580.812
//
//----------------------------------------------------------------------------
// Input Clock   Input Freq (MHz)   Input Jitter (UI)
//----------------------------------------------------------------------------
// primary         125.000            0.010
//----------------------------------------------------------------------------

  // Input buffering
  //------------------------------------
  BUFG clkin1_buf
   (.O (clkin1),
    .I (CLK_IN1));

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
    .DIVCLK_DIVIDE        (10),
    .CLKFBOUT_MULT_F      (51.000),
    .CLKFBOUT_PHASE       (0.000),
    .CLKFBOUT_USE_FINE_PS ("FALSE"),
    .CLKOUT0_DIVIDE_F     (127.500),
    .CLKOUT0_PHASE        (0.000),
    .CLKOUT0_DUTY_CYCLE   (0.500),
    .CLKOUT0_USE_FINE_PS  ("FALSE"),
    .CLKIN1_PERIOD        (8.0),
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
    .RST                 (1'b0));

  // Output buffering
  //-----------------------------------
  BUFG clkf_buf
   (.O (clkfbout_buf),
    .I (clkfbout));


  BUFG clkout1_buf
   (.O   (CLK_OUT1),
    .I   (clkout0));
endmodule



(* CORE_GENERATION_INFO = "bufg_div4clk,clk_wiz_v1_8,{component_name=bufg_div4clk,use_phase_alignment=true,use_min_o_jitter=false,use_max_i_jitter=false,use_dyn_phase_shift=false,use_inclk_switchover=false,use_dyn_reconfig=false,feedback_source=FDBK_AUTO,primtype_sel=MMCM_ADV,num_out_clk=1,clkin1_period=6.25,clkin2_period=10.0,use_power_down=false,use_reset=false,use_locked=true,use_inclk_stopped=true,use_status=false,use_freeze=false,use_clk_valid=false,feedback_type=SINGLE,clock_mgr_type=MANUAL,manual_override=false}" *)
module bufg_div4clk
 (// Clock in ports
  input         CLK_IN1,
  // Clock out ports
  output        CLK_OUT1,
  // Status and control signals
  output        INPUT_CLK_STOPPED,
  output        LOCKED
 );

//----------------------------------------------------------------------------
// User entered comments
//----------------------------------------------------------------------------
// Bufg 160MHz input to mmcm, 40MHz out to Bufg.
//
//----------------------------------------------------------------------------
// Output     Output      Phase    Duty Cycle   Pk-to-Pk     Phase
// Clock     Freq (MHz)  (degrees)    (%)     Jitter (ps)  Error (ps)
//----------------------------------------------------------------------------
// CLK_OUT1    40.000      0.000      50.0      207.443    191.950
//
//----------------------------------------------------------------------------
// Input Clock   Input Freq (MHz)   Input Jitter (UI)
//----------------------------------------------------------------------------
// primary         160.000            0.010

  // Input buffering
  //------------------------------------
  BUFG clkin1_buf
   (.O (clkin1),
    .I (CLK_IN1));


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
    .DIVCLK_DIVIDE        (4),
    .CLKFBOUT_MULT_F      (25.000),
    .CLKFBOUT_PHASE       (0.000),
    .CLKFBOUT_USE_FINE_PS ("FALSE"),
    .CLKOUT0_DIVIDE_F     (25.000),
    .CLKOUT0_PHASE        (0.000),
    .CLKOUT0_DUTY_CYCLE   (0.500),
    .CLKOUT0_USE_FINE_PS  ("FALSE"),
    .CLKIN1_PERIOD        (6.25),
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
    .RST                 (1'b0));

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
//  /   /         Filename : gbe_t20r20.v
// /___/   /\     
// \   \  /  \ 
//  \___\/\___\
//
//
// Module GBE_T20R20 (a GTX Wrapper)
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


//***************************** Entity Declaration ****************************

(* CORE_GENERATION_INFO = "GBE_T20R20,v6_gtxwizard_v1_8,{protocol_file=gigabit_ethernet}" *)
module GBE_T20R20 #
(
    // Simulation attributes
    parameter   WRAPPER_SIM_GTXRESET_SPEEDUP    = 0    // Set to 1 to speed up sim reset
)
(
    
    //_________________________________________________________________________
    //_________________________________________________________________________
    //GTX0  (X0Y19)

    input          GTX0_DOUBLE_RESET_CLK_IN,
    //---------------------- Loopback and Powerdown Ports ----------------------
    input   [2:0]   GTX0_LOOPBACK_IN,
    input   [1:0]   GTX0_RXPOWERDOWN_IN,
    //--------------------- Receive Ports - 8b10b Decoder ----------------------
    output  [1:0]   GTX0_RXCHARISCOMMA_OUT,
    output  [1:0]   GTX0_RXCHARISK_OUT,
    output  [1:0]   GTX0_RXDISPERR_OUT,
    output  [1:0]   GTX0_RXNOTINTABLE_OUT,
    output  [1:0]   GTX0_RXRUNDISP_OUT,
    //----------------- Receive Ports - Clock Correction Ports -----------------
    output  [2:0]   GTX0_RXCLKCORCNT_OUT,
    //------------- Receive Ports - Comma Detection and Alignment --------------
    input           GTX0_RXENMCOMMAALIGN_IN,
    input           GTX0_RXENPCOMMAALIGN_IN,
    //----------------- Receive Ports - RX Data Path interface -----------------
    output  [15:0]  GTX0_RXDATA_OUT,
    input           GTX0_RXRESET_IN,
    input           GTX0_RXUSRCLK2_IN,
    //----- Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR ------
    output          GTX0_RXELECIDLE_OUT,
    input           GTX0_RXN_IN,
    input           GTX0_RXP_IN,
    //------ Receive Ports - RX Elastic Buffer and Phase Alignment Ports -------
    input           GTX0_RXBUFRESET_IN,
    output  [2:0]   GTX0_RXBUFSTATUS_OUT,
    //------------- Receive Ports - RX Loss-of-sync State Machine --------------
    output  [1:0]   GTX0_RXLOSSOFSYNC_OUT,
    //---------------------- Receive Ports - RX PLL Ports ----------------------
    input           GTX0_GTXRXRESET_IN,
    input           GTX0_MGTREFCLKRX_IN,
    input           GTX0_PLLRXRESET_IN,
    output          GTX0_RXPLLLKDET_OUT,
    output          GTX0_RXRESETDONE_OUT,
    //-------------- Transmit Ports - 8b10b Encoder Control Ports --------------
    input   [1:0]   GTX0_TXCHARDISPMODE_IN,
    input   [1:0]   GTX0_TXCHARDISPVAL_IN,
    input   [1:0]   GTX0_TXCHARISK_IN,
    //----------------------- Transmit Ports - GTX Ports -----------------------
    input   [12:0]  GTX0_GTXTEST_IN,
    //---------------- Transmit Ports - TX Data Path interface -----------------
    input   [15:0]  GTX0_TXDATA_IN,
    output          GTX0_TXOUTCLK_OUT,
    input           GTX0_TXRESET_IN,
    input           GTX0_TXUSRCLK2_IN,
    //-------------- Transmit Ports - TX Driver and OOB signaling --------------
    output          GTX0_TXN_OUT,
    output          GTX0_TXP_OUT,
    //--------------------- Transmit Ports - TX PLL Ports ----------------------
    input           GTX0_GTXTXRESET_IN,
    output          GTX0_TXRESETDONE_OUT


);

//***************************** Wire Declarations *****************************

    // ground and vcc signals
    wire            tied_to_ground_i;
    wire    [63:0]  tied_to_ground_vec_i;
    wire            tied_to_vcc_i;
    wire    [63:0]  tied_to_vcc_vec_i;
    wire            gtx0_gtxtest_bit1;
    wire            gtx0_gtxtest_done;
    wire    [12:0]  gtx0_gtxtest_i;
    wire            gtx0_txreset_i;
    wire            gtx0_rxreset_i;
    wire            gtx0_rxplllkdet_i;
 
//********************************* Main Body of Code**************************

    assign tied_to_ground_i             = 1'b0;
    assign tied_to_ground_vec_i         = 64'h0000000000000000;
    assign tied_to_vcc_i                = 1'b1;
    assign tied_to_vcc_vec_i            = 64'hffffffffffffffff;

    assign gtx0_gtxtest_i          = {11'b10000000000,gtx0_gtxtest_bit1,1'b0};
    assign gtx0_txreset_i          = gtx0_gtxtest_done || GTX0_TXRESET_IN;
    assign gtx0_rxreset_i          = gtx0_gtxtest_done || GTX0_RXRESET_IN;
    assign GTX0_RXPLLLKDET_OUT     = gtx0_rxplllkdet_i;

//------------------------- GTX Instances  -------------------------------



    //_________________________________________________________________________
    //_________________________________________________________________________
    //GTX0  (X0Y19)

    GBE_T20R20_GTX #
    (
        // Simulation attributes
        .GTX_SIM_GTXRESET_SPEEDUP   (WRAPPER_SIM_GTXRESET_SPEEDUP),
        
        // Share RX PLL parameter
        .GTX_TX_CLK_SOURCE           ("RXPLL"),
        // Save power parameter
        .GTX_POWER_SAVE              (10'b0000110100)
    )
    gtx0_gbe_t20r20_i
    (
        //---------------------- Loopback and Powerdown Ports ----------------------
        .LOOPBACK_IN                    (GTX0_LOOPBACK_IN),
        .RXPOWERDOWN_IN                 (GTX0_RXPOWERDOWN_IN),
        //--------------------- Receive Ports - 8b10b Decoder ----------------------
        .RXCHARISCOMMA_OUT              (GTX0_RXCHARISCOMMA_OUT),
        .RXCHARISK_OUT                  (GTX0_RXCHARISK_OUT),
        .RXDISPERR_OUT                  (GTX0_RXDISPERR_OUT),
        .RXNOTINTABLE_OUT               (GTX0_RXNOTINTABLE_OUT),
        .RXRUNDISP_OUT                  (GTX0_RXRUNDISP_OUT),
        //----------------- Receive Ports - Clock Correction Ports -----------------
        .RXCLKCORCNT_OUT                (GTX0_RXCLKCORCNT_OUT),
        //------------- Receive Ports - Comma Detection and Alignment --------------
        .RXENMCOMMAALIGN_IN             (GTX0_RXENMCOMMAALIGN_IN),
        .RXENPCOMMAALIGN_IN             (GTX0_RXENPCOMMAALIGN_IN),
        //----------------- Receive Ports - RX Data Path interface -----------------
        .RXDATA_OUT                     (GTX0_RXDATA_OUT),
        .RXRESET_IN                     (gtx0_rxreset_i),
        .RXUSRCLK2_IN                   (GTX0_RXUSRCLK2_IN),
        //----- Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR ------
        .RXELECIDLE_OUT                 (GTX0_RXELECIDLE_OUT),
        .RXN_IN                         (GTX0_RXN_IN),
        .RXP_IN                         (GTX0_RXP_IN),
        //------ Receive Ports - RX Elastic Buffer and Phase Alignment Ports -------
        .RXBUFRESET_IN                  (GTX0_RXBUFRESET_IN),
        .RXBUFSTATUS_OUT                (GTX0_RXBUFSTATUS_OUT),
        //------------- Receive Ports - RX Loss-of-sync State Machine --------------
        .RXLOSSOFSYNC_OUT               (GTX0_RXLOSSOFSYNC_OUT),
        //---------------------- Receive Ports - RX PLL Ports ----------------------
        .GTXRXRESET_IN                  (GTX0_GTXRXRESET_IN),
        .MGTREFCLKRX_IN                 ({tied_to_ground_i , GTX0_MGTREFCLKRX_IN}),
        .PLLRXRESET_IN                  (GTX0_PLLRXRESET_IN),
        .RXPLLLKDET_OUT                 (gtx0_rxplllkdet_i),
        .RXRESETDONE_OUT                (GTX0_RXRESETDONE_OUT),
        //-------------- Transmit Ports - 8b10b Encoder Control Ports --------------
        .TXCHARDISPMODE_IN              (GTX0_TXCHARDISPMODE_IN),
        .TXCHARDISPVAL_IN               (GTX0_TXCHARDISPVAL_IN),
        .TXCHARISK_IN                   (GTX0_TXCHARISK_IN),
        //----------------------- Transmit Ports - GTX Ports -----------------------
        .GTXTEST_IN                     (gtx0_gtxtest_i),
        //---------------- Transmit Ports - TX Data Path interface -----------------
        .TXDATA_IN                      (GTX0_TXDATA_IN),
        .TXOUTCLK_OUT                   (GTX0_TXOUTCLK_OUT),
        .TXRESET_IN                     (gtx0_txreset_i),
        .TXUSRCLK2_IN                   (GTX0_TXUSRCLK2_IN),
        //-------------- Transmit Ports - TX Driver and OOB signaling --------------
        .TXN_OUT                        (GTX0_TXN_OUT),
        .TXP_OUT                        (GTX0_TXP_OUT),
        //--------------------- Transmit Ports - TX PLL Ports ----------------------
        .GTXTXRESET_IN                  (GTX0_GTXTXRESET_IN),
        .MGTREFCLKTX_IN                 ({tied_to_ground_i , GTX0_MGTREFCLKRX_IN}),
        .PLLTXRESET_IN                  (tied_to_ground_i),
        .TXPLLLKDET_OUT                 (),
        .TXRESETDONE_OUT                (GTX0_TXRESETDONE_OUT)

    );


    //--------------------------Logic to drive GTXTEST[1] -------------------------------
     DOUBLE_RESET gtx0_double_reset_i
     (
        .CLK(GTX0_DOUBLE_RESET_CLK_IN),
        .PLLLKDET(gtx0_rxplllkdet_i),
        .GTXTEST_DONE(gtx0_gtxtest_done),
        .GTXTEST_BIT1(gtx0_gtxtest_bit1)
     );
    

endmodule

    



///////////////////////////////////////////////////////////////////////////////
//   ____  ____ 
//  /   /\/   /
// /___/  \  /    Vendor: Xilinx
// \   \   \/     Version : 1.8
//  \   \         Application : Virtex-6 FPGA GTX Transceiver Wizard
//  /   /         Filename : gbe_t20r20_gtx.v
// /___/   /\     Timestamp :
// \   \  /  \ 
//  \___\/\___\
//
//
// Module GBE_T20R20_GTX (a GTX Wrapper)
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


//***************************** Entity Declaration ****************************

module GBE_T20R20_GTX #
(
    // Simulation attributes
    parameter   GTX_SIM_GTXRESET_SPEEDUP   =   0,      // Set to 1 to speed up sim reset
    
    // Share RX PLL parameter
    parameter   GTX_TX_CLK_SOURCE          =   "TXPLL",
    // Save power parameter
    parameter   GTX_POWER_SAVE             =   10'b0000000000
)
(
    //---------------------- Loopback and Powerdown Ports ----------------------
    input   [2:0]   LOOPBACK_IN,
    input   [1:0]   RXPOWERDOWN_IN,
    //--------------------- Receive Ports - 8b10b Decoder ----------------------
    output  [1:0]   RXCHARISCOMMA_OUT,
    output  [1:0]   RXCHARISK_OUT,
    output  [1:0]   RXDISPERR_OUT,
    output  [1:0]   RXNOTINTABLE_OUT,
    output  [1:0]   RXRUNDISP_OUT,
    //----------------- Receive Ports - Clock Correction Ports -----------------
    output  [2:0]   RXCLKCORCNT_OUT,
    //------------- Receive Ports - Comma Detection and Alignment --------------
    input           RXENMCOMMAALIGN_IN,
    input           RXENPCOMMAALIGN_IN,
    //----------------- Receive Ports - RX Data Path interface -----------------
    output  [15:0]  RXDATA_OUT,
    input           RXRESET_IN,
    input           RXUSRCLK2_IN,
    //----- Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR ------
    output          RXELECIDLE_OUT,
    input           RXN_IN,
    input           RXP_IN,
    //------ Receive Ports - RX Elastic Buffer and Phase Alignment Ports -------
    input           RXBUFRESET_IN,
    output  [2:0]   RXBUFSTATUS_OUT,
    //------------- Receive Ports - RX Loss-of-sync State Machine --------------
    output  [1:0]   RXLOSSOFSYNC_OUT,
    //---------------------- Receive Ports - RX PLL Ports ----------------------
    input           GTXRXRESET_IN,
    input   [1:0]   MGTREFCLKRX_IN,
    input           PLLRXRESET_IN,
    output          RXPLLLKDET_OUT,
    output          RXRESETDONE_OUT,
    //-------------- Transmit Ports - 8b10b Encoder Control Ports --------------
    input   [1:0]   TXCHARDISPMODE_IN,
    input   [1:0]   TXCHARDISPVAL_IN,
    input   [1:0]   TXCHARISK_IN,
    //----------------------- Transmit Ports - GTX Ports -----------------------
    input   [12:0]  GTXTEST_IN,
    //---------------- Transmit Ports - TX Data Path interface -----------------
    input   [15:0]  TXDATA_IN,
    output          TXOUTCLK_OUT,
    input           TXRESET_IN,
    input           TXUSRCLK2_IN,
    //-------------- Transmit Ports - TX Driver and OOB signaling --------------
    output          TXN_OUT,
    output          TXP_OUT,
    //--------------------- Transmit Ports - TX PLL Ports ----------------------
    input           GTXTXRESET_IN,
    input   [1:0]   MGTREFCLKTX_IN,
    input           PLLTXRESET_IN,
    output          TXPLLLKDET_OUT,
    output          TXRESETDONE_OUT


);


//***************************** Wire Declarations *****************************

    // ground and vcc signals
    wire            tied_to_ground_i;
    wire    [63:0]  tied_to_ground_vec_i;
    wire            tied_to_vcc_i;
    wire    [63:0]  tied_to_vcc_vec_i;


    //RX Datapath signals
    wire    [31:0]  rxdata_i;
    wire    [1:0]   rxchariscomma_float_i;
    wire    [1:0]   rxcharisk_float_i;
    wire    [1:0]   rxdisperr_float_i;
    wire    [1:0]   rxnotintable_float_i;
    wire    [1:0]   rxrundisp_float_i;


    //TX Datapath signals
    wire    [31:0]  txdata_i;           
    wire    [1:0]   txkerr_float_i;
    wire    [1:0]   txrundisp_float_i;
        
// 
//********************************* Main Body of Code**************************
                       
    //-------------------------  Static signal Assigments ---------------------   

    assign tied_to_ground_i             = 1'b0;
    assign tied_to_ground_vec_i         = 64'h0000000000000000;
    assign tied_to_vcc_i                = 1'b1;
    assign tied_to_vcc_vec_i            = 64'hffffffffffffffff;
    
    //-------------------  GTX Datapath byte mapping  -----------------

    assign  RXDATA_OUT    =   rxdata_i[15:0];

    // The GTX transmits little endian data (TXDATA[7:0] transmitted first)     
    assign  txdata_i    =   {tied_to_ground_vec_i[15:0],TXDATA_IN};





    //------------------------- GTX Instantiations  --------------------------
        GTXE1 #
        (
            //_______________________ Simulation-Only Attributes __________________
    
            .SIM_RECEIVER_DETECT_PASS   ("TRUE"),
            
            .SIM_TX_ELEC_IDLE_LEVEL     ("X"),
    
            .SIM_GTXRESET_SPEEDUP       (GTX_SIM_GTXRESET_SPEEDUP),
            .SIM_VERSION                ("2.0"),
            .SIM_TXREFCLK_SOURCE        (3'b000),
            .SIM_RXREFCLK_SOURCE        (3'b000),
            

           //--------------------------TX PLL----------------------------
            .TX_CLK_SOURCE                          (GTX_TX_CLK_SOURCE),
            .TX_OVERSAMPLE_MODE                     ("FALSE"),
            .TXPLL_COM_CFG                          (24'h21680a),
            .TXPLL_CP_CFG                           (8'h0D),
            .TXPLL_DIVSEL_FB                        (2),
            .TXPLL_DIVSEL_OUT                       (2),
            .TXPLL_DIVSEL_REF                       (1),
            .TXPLL_DIVSEL45_FB                      (5),
            .TXPLL_LKDET_CFG                        (3'b111),
            .TX_CLK25_DIVIDER                       (5),
            .TXPLL_SATA                             (2'b00),
            .TX_TDCC_CFG                            (2'b00),
            .PMA_CAS_CLK_EN                         ("FALSE"),
            .POWER_SAVE                             (GTX_POWER_SAVE),

           //-----------------------TX Interface-------------------------
            .GEN_TXUSRCLK                           ("TRUE"),
            .TX_DATA_WIDTH                          (20),
            .TX_USRCLK_CFG                          (6'h00),
            .TXOUTCLK_CTRL                          ("TXPLLREFCLK_DIV1"),
            .TXOUTCLK_DLY                           (10'b0000000000),

           //------------TX Buffering and Phase Alignment----------------
            .TX_PMADATA_OPT                         (1'b0),
            .PMA_TX_CFG                             (20'h80082),
            .TX_BUFFER_USE                          ("TRUE"),
            .TX_BYTECLK_CFG                         (6'h00),
            .TX_EN_RATE_RESET_BUF                   ("TRUE"),
            .TX_XCLK_SEL                            ("TXOUT"),
            .TX_DLYALIGN_CTRINC                     (4'b0100),
            .TX_DLYALIGN_LPFINC                     (4'b0110),
            .TX_DLYALIGN_MONSEL                     (3'b000),
            .TX_DLYALIGN_OVRDSETTING                (8'b10000000),

           //-----------------------TX Gearbox---------------------------
            .GEARBOX_ENDEC                          (3'b000),
            .TXGEARBOX_USE                          ("FALSE"),

           //--------------TX Driver and OOB Signalling------------------
            .TX_DRIVE_MODE                          ("DIRECT"),
            .TX_IDLE_ASSERT_DELAY                   (3'b100),
            .TX_IDLE_DEASSERT_DELAY                 (3'b010),
            .TXDRIVE_LOOPBACK_HIZ                   ("FALSE"),
            .TXDRIVE_LOOPBACK_PD                    ("FALSE"),

           //------------TX Pipe Control for PCI Express/SATA------------
            .COM_BURST_VAL                          (4'b1111),

           //----------------TX Attributes for PCI Express---------------
            .TX_DEEMPH_0                            (5'b11010),
            .TX_DEEMPH_1                            (5'b10000),
            .TX_MARGIN_FULL_0                       (7'b1001110),
            .TX_MARGIN_FULL_1                       (7'b1001001),
            .TX_MARGIN_FULL_2                       (7'b1000101),
            .TX_MARGIN_FULL_3                       (7'b1000010),
            .TX_MARGIN_FULL_4                       (7'b1000000),
            .TX_MARGIN_LOW_0                        (7'b1000110),
            .TX_MARGIN_LOW_1                        (7'b1000100),
            .TX_MARGIN_LOW_2                        (7'b1000010),
            .TX_MARGIN_LOW_3                        (7'b1000000),
            .TX_MARGIN_LOW_4                        (7'b1000000),

           //--------------------------RX PLL----------------------------
            .RX_OVERSAMPLE_MODE                     ("FALSE"),
            .RXPLL_COM_CFG                          (24'h21680a),
            .RXPLL_CP_CFG                           (8'h0D),
            .RXPLL_DIVSEL_FB                        (2),
            .RXPLL_DIVSEL_OUT                       (2),
            .RXPLL_DIVSEL_REF                       (1),
            .RXPLL_DIVSEL45_FB                      (5),
            .RXPLL_LKDET_CFG                        (3'b111),
            .RX_CLK25_DIVIDER                       (5),

           //-----------------------RX Interface-------------------------
            .GEN_RXUSRCLK                           ("TRUE"),
            .RX_DATA_WIDTH                          (20),
            .RXRECCLK_CTRL                          ("RXRECCLKPMA_DIV1"),
            .RXRECCLK_DLY                           (10'b0000000000),
            .RXUSRCLK_DLY                           (16'h0000),

           //--------RX Driver,OOB signalling,Coupling and Eq.,CDR-------
            .AC_CAP_DIS                             ("TRUE"),
            .CDR_PH_ADJ_TIME                        (5'b10100),
            .OOBDETECT_THRESHOLD                    (3'b011),
            .PMA_CDR_SCAN                           (27'h640404C),
            .PMA_RX_CFG                             (25'h05ce008),
            .RCV_TERM_GND                           ("FALSE"),
            .RCV_TERM_VTTRX                         ("FALSE"),
            .RX_EN_IDLE_HOLD_CDR                    ("FALSE"),
            .RX_EN_IDLE_RESET_FR                    ("TRUE"),
            .RX_EN_IDLE_RESET_PH                    ("TRUE"),
            .TX_DETECT_RX_CFG                       (14'h1832),
            .TERMINATION_CTRL                       (5'b00000),
            .TERMINATION_OVRD                       ("FALSE"),
            .CM_TRIM                                (2'b01),
            .PMA_RXSYNC_CFG                         (7'h00),
            .PMA_CFG                                (76'h0040000040000000003),
            .BGTEST_CFG                             (2'b00),
            .BIAS_CFG                               (17'h00000),

           //------------RX Decision Feedback Equalizer(DFE)-------------
            .DFE_CAL_TIME                           (5'b01100),
            .DFE_CFG                                (8'b00011011),
            .RX_EN_IDLE_HOLD_DFE                    ("TRUE"),
            .RX_EYE_OFFSET                          (8'h4C),
            .RX_EYE_SCANMODE                        (2'b00),

           //-----------------------PRBS Detection-----------------------
            .RXPRBSERR_LOOPBACK                     (1'b0),

           //----------------Comma Detection and Alignment---------------
            .ALIGN_COMMA_WORD                       (2), // 1 == any byte.  2 == even byte.
            .COMMA_10B_ENABLE                       (10'b0001111111),
            .COMMA_DOUBLE                           ("FALSE"),
            .DEC_MCOMMA_DETECT                      ("TRUE"),
            .DEC_PCOMMA_DETECT                      ("TRUE"),
            .DEC_VALID_COMMA_ONLY                   ("FALSE"),
            .MCOMMA_10B_VALUE                       (10'b1010000011),
            .MCOMMA_DETECT                          ("TRUE"),
            .PCOMMA_10B_VALUE                       (10'b0101111100),
            .PCOMMA_DETECT                          ("TRUE"),
            .RX_DECODE_SEQ_MATCH                    ("TRUE"),
            .RX_SLIDE_AUTO_WAIT                     (5),
            .RX_SLIDE_MODE                          ("OFF"),
            .SHOW_REALIGN_COMMA                     ("FALSE"),

           //---------------RX Loss-of-sync State Machine----------------
            .RX_LOS_INVALID_INCR                    (1),
            .RX_LOS_THRESHOLD                       (4),
            .RX_LOSS_OF_SYNC_FSM                    ("FALSE"),

           //-----------------------RX Gearbox---------------------------
            .RXGEARBOX_USE                          ("FALSE"),

           //-----------RX Elastic Buffer and Phase alignment------------
            .RX_BUFFER_USE                          ("TRUE"),
            .RX_EN_IDLE_RESET_BUF                   ("TRUE"),
            .RX_EN_MODE_RESET_BUF                   ("TRUE"),
            .RX_EN_RATE_RESET_BUF                   ("TRUE"),
            .RX_EN_REALIGN_RESET_BUF                ("FALSE"),
            .RX_EN_REALIGN_RESET_BUF2               ("FALSE"),
            .RX_FIFO_ADDR_MODE                      ("FULL"),
            .RX_IDLE_HI_CNT                         (4'b1000),
            .RX_IDLE_LO_CNT                         (4'b0000),
            .RX_XCLK_SEL                            ("RXREC"),
            .RX_DLYALIGN_CTRINC                     (4'b1110),
            .RX_DLYALIGN_EDGESET                    (5'b00010),
            .RX_DLYALIGN_LPFINC                     (4'b1110),
            .RX_DLYALIGN_MONSEL                     (3'b000),
            .RX_DLYALIGN_OVRDSETTING                (8'b10000000),

           //----------------------Clock Correction----------------------
            .CLK_COR_ADJ_LEN                        (2),
            .CLK_COR_DET_LEN                        (2),
            .CLK_COR_INSERT_IDLE_FLAG               ("FALSE"),
            .CLK_COR_KEEP_IDLE                      ("FALSE"),
            .CLK_COR_MAX_LAT                        (18),
            .CLK_COR_MIN_LAT                        (14),
            .CLK_COR_PRECEDENCE                     ("TRUE"),
            .CLK_COR_REPEAT_WAIT                    (0),
            .CLK_COR_SEQ_1_1                        (10'b0110111100),
            .CLK_COR_SEQ_1_2                        (10'b0001010000),
            .CLK_COR_SEQ_1_3                        (10'b0100000000),
            .CLK_COR_SEQ_1_4                        (10'b0100000000),
            .CLK_COR_SEQ_1_ENABLE                   (4'b1111),
            .CLK_COR_SEQ_2_1                        (10'b0110111100),
            .CLK_COR_SEQ_2_2                        (10'b0010110101),
            .CLK_COR_SEQ_2_3                        (10'b0100000000),
            .CLK_COR_SEQ_2_4                        (10'b0100000000),
            .CLK_COR_SEQ_2_ENABLE                   (4'b1111),
            .CLK_COR_SEQ_2_USE                      ("TRUE"),
            .CLK_CORRECT_USE                        ("TRUE"),

           //----------------------Channel Bonding----------------------
            .CHAN_BOND_1_MAX_SKEW                   (1),
            .CHAN_BOND_2_MAX_SKEW                   (1),
            .CHAN_BOND_KEEP_ALIGN                   ("FALSE"),
            .CHAN_BOND_SEQ_1_1                      (10'b0000000000),
            .CHAN_BOND_SEQ_1_2                      (10'b0000000000),
            .CHAN_BOND_SEQ_1_3                      (10'b0000000000),
            .CHAN_BOND_SEQ_1_4                      (10'b0000000000),
            .CHAN_BOND_SEQ_1_ENABLE                 (4'b1111),
            .CHAN_BOND_SEQ_2_1                      (10'b0000000000),
            .CHAN_BOND_SEQ_2_2                      (10'b0000000000),
            .CHAN_BOND_SEQ_2_3                      (10'b0000000000),
            .CHAN_BOND_SEQ_2_4                      (10'b0000000000),
            .CHAN_BOND_SEQ_2_CFG                    (5'b00000),
            .CHAN_BOND_SEQ_2_ENABLE                 (4'b1111),
            .CHAN_BOND_SEQ_2_USE                    ("FALSE"),
            .CHAN_BOND_SEQ_LEN                      (1),
            .PCI_EXPRESS_MODE                       ("FALSE"),

           //-----------RX Attributes for PCI Express/SATA/SAS----------
            .SAS_MAX_COMSAS                         (52),
            .SAS_MIN_COMSAS                         (40),
            .SATA_BURST_VAL                         (3'b100),
            .SATA_IDLE_VAL                          (3'b100),
            .SATA_MAX_BURST                         (9),
            .SATA_MAX_INIT                          (27),
            .SATA_MAX_WAKE                          (9),
            .SATA_MIN_BURST                         (5),
            .SATA_MIN_INIT                          (15),
            .SATA_MIN_WAKE                          (5),
            .TRANS_TIME_FROM_P2                     (12'h03c),
            .TRANS_TIME_NON_P2                      (8'h19),
            .TRANS_TIME_RATE                        (8'hff),
            .TRANS_TIME_TO_P2                       (10'h064)

            
        ) 
        gtxe1_i 
        (
        
        //---------------------- Loopback and Powerdown Ports ----------------------
        .LOOPBACK                       (LOOPBACK_IN),
        .RXPOWERDOWN                    (RXPOWERDOWN_IN),
        .TXPOWERDOWN                    (2'b00),
        //------------ Receive Ports - 64b66b and 64b67b Gearbox Ports -------------
        .RXDATAVALID                    (),
        .RXGEARBOXSLIP                  (tied_to_ground_i),
        .RXHEADER                       (),
        .RXHEADERVALID                  (),
        .RXSTARTOFSEQ                   (),
        //--------------------- Receive Ports - 8b10b Decoder ----------------------
        .RXCHARISCOMMA                  ({rxchariscomma_float_i,RXCHARISCOMMA_OUT}),
        .RXCHARISK                      ({rxcharisk_float_i,RXCHARISK_OUT}),
        .RXDEC8B10BUSE                  (tied_to_vcc_i),
        .RXDISPERR                      ({rxdisperr_float_i,RXDISPERR_OUT}),
        .RXNOTINTABLE                   ({rxnotintable_float_i,RXNOTINTABLE_OUT}),
        .RXRUNDISP                      ({rxrundisp_float_i,RXRUNDISP_OUT}),
        .USRCODEERR                     (tied_to_ground_i),
        //----------------- Receive Ports - Channel Bonding Ports ------------------
        .RXCHANBONDSEQ                  (),
        .RXCHBONDI                      (tied_to_ground_vec_i[3:0]),
        .RXCHBONDLEVEL                  (tied_to_ground_vec_i[2:0]),
        .RXCHBONDMASTER                 (tied_to_ground_i),
        .RXCHBONDO                      (),
        .RXCHBONDSLAVE                  (tied_to_ground_i),
        .RXENCHANSYNC                   (tied_to_ground_i),
        //----------------- Receive Ports - Clock Correction Ports -----------------
        .RXCLKCORCNT                    (RXCLKCORCNT_OUT),
        //------------- Receive Ports - Comma Detection and Alignment --------------
        .RXBYTEISALIGNED                (),
        .RXBYTEREALIGN                  (),
        .RXCOMMADET                     (),
        .RXCOMMADETUSE                  (tied_to_vcc_i),
        .RXENMCOMMAALIGN                (RXENMCOMMAALIGN_IN),
        .RXENPCOMMAALIGN                (RXENPCOMMAALIGN_IN),
        .RXSLIDE                        (tied_to_ground_i),
        //--------------------- Receive Ports - PRBS Detection ---------------------
        .PRBSCNTRESET                   (tied_to_ground_i),
        .RXENPRBSTST                    (tied_to_ground_vec_i[2:0]),
        .RXPRBSERR                      (),
        //----------------- Receive Ports - RX Data Path interface -----------------
        .RXDATA                         (rxdata_i),
        .RXRECCLK                       (),
        .RXRECCLKPCS                    (),
        .RXRESET                        (RXRESET_IN),
        .RXUSRCLK                       (tied_to_ground_i),
        .RXUSRCLK2                      (RXUSRCLK2_IN),
        //---------- Receive Ports - RX Decision Feedback Equalizer(DFE) -----------
        .DFECLKDLYADJ                   (tied_to_ground_vec_i[5:0]),
        .DFECLKDLYADJMON                (),
        .DFEDLYOVRD                     (tied_to_vcc_i),
        .DFEEYEDACMON                   (),
        .DFESENSCAL                     (),
        .DFETAP1                        (tied_to_ground_vec_i[4:0]),
        .DFETAP1MONITOR                 (),
        .DFETAP2                        (tied_to_ground_vec_i[4:0]),
        .DFETAP2MONITOR                 (),
        .DFETAP3                        (tied_to_ground_vec_i[3:0]),
        .DFETAP3MONITOR                 (),
        .DFETAP4                        (tied_to_ground_vec_i[3:0]),
        .DFETAP4MONITOR                 (),
        .DFETAPOVRD                     (tied_to_vcc_i),
        //----- Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR ------
        .GATERXELECIDLE                 (tied_to_vcc_i),
        .IGNORESIGDET                   (tied_to_vcc_i),
        .RXCDRRESET                     (tied_to_ground_i),
        .RXELECIDLE                     (RXELECIDLE_OUT),
        .RXEQMIX                        (10'b0000000111),
        .RXN                            (RXN_IN),
        .RXP                            (RXP_IN),
        //------ Receive Ports - RX Elastic Buffer and Phase Alignment Ports -------
        .RXBUFRESET                     (RXBUFRESET_IN),
        .RXBUFSTATUS                    (RXBUFSTATUS_OUT),
        .RXCHANISALIGNED                (),
        .RXCHANREALIGN                  (),
        .RXDLYALIGNDISABLE              (tied_to_ground_i),
        .RXDLYALIGNMONENB               (tied_to_ground_i),
        .RXDLYALIGNMONITOR              (),
        .RXDLYALIGNOVERRIDE             (tied_to_vcc_i),
        .RXDLYALIGNRESET                (tied_to_ground_i),
        .RXDLYALIGNSWPPRECURB           (tied_to_vcc_i),
        .RXDLYALIGNUPDSW                (tied_to_ground_i),
        .RXENPMAPHASEALIGN              (tied_to_ground_i),
        .RXPMASETPHASE                  (tied_to_ground_i),
        .RXSTATUS                       (),
        //------------- Receive Ports - RX Loss-of-sync State Machine --------------
        .RXLOSSOFSYNC                   (RXLOSSOFSYNC_OUT),
        //-------------------- Receive Ports - RX Oversampling ---------------------
        .RXENSAMPLEALIGN                (tied_to_ground_i),
        .RXOVERSAMPLEERR                (),
        //---------------------- Receive Ports - RX PLL Ports ----------------------
        .GREFCLKRX                      (tied_to_ground_i),
        .GTXRXRESET                     (GTXRXRESET_IN),
        .MGTREFCLKRX                    (MGTREFCLKRX_IN),
        .NORTHREFCLKRX                  (tied_to_ground_vec_i[1:0]),
        .PERFCLKRX                      (tied_to_ground_i),
        .PLLRXRESET                     (PLLRXRESET_IN),
        .RXPLLLKDET                     (RXPLLLKDET_OUT),
        .RXPLLLKDETEN                   (tied_to_vcc_i),
        .RXPLLPOWERDOWN                 (tied_to_ground_i),
        .RXPLLREFSELDY                  (tied_to_ground_vec_i[2:0]),
        .RXRATE                         (tied_to_ground_vec_i[1:0]),
        .RXRATEDONE                     (),
        .RXRESETDONE                    (RXRESETDONE_OUT),
        .SOUTHREFCLKRX                  (tied_to_ground_vec_i[1:0]),
        //------------ Receive Ports - RX Pipe Control for PCI Express -------------
        .PHYSTATUS                      (),
        .RXVALID                        (),
        //--------------- Receive Ports - RX Polarity Control Ports ----------------
        .RXPOLARITY                     (tied_to_ground_i),
        //------------------- Receive Ports - RX Ports for SATA --------------------
        .COMINITDET                     (),
        .COMSASDET                      (),
        .COMWAKEDET                     (),
        //----------- Shared Ports - Dynamic Reconfiguration Port (DRP) ------------
        .DADDR                          (tied_to_ground_vec_i[7:0]),
        .DCLK                           (tied_to_ground_i),
        .DEN                            (tied_to_ground_i),
        .DI                             (tied_to_ground_vec_i[15:0]),
        .DRDY                           (),
        .DRPDO                          (),
        .DWE                            (tied_to_ground_i),
        //------------ Transmit Ports - 64b66b and 64b67b Gearbox Ports ------------
        .TXGEARBOXREADY                 (),
        .TXHEADER                       (tied_to_ground_vec_i[2:0]),
        .TXSEQUENCE                     (tied_to_ground_vec_i[6:0]),
        .TXSTARTSEQ                     (tied_to_ground_i),
        //-------------- Transmit Ports - 8b10b Encoder Control Ports --------------
        .TXBYPASS8B10B                  (tied_to_ground_vec_i[3:0]),
        .TXCHARDISPMODE                 ({tied_to_ground_vec_i[1:0],TXCHARDISPMODE_IN}),
        .TXCHARDISPVAL                  ({tied_to_ground_vec_i[1:0],TXCHARDISPVAL_IN}),
        .TXCHARISK                      ({tied_to_ground_vec_i[1:0],TXCHARISK_IN}),
        .TXENC8B10BUSE                  (tied_to_vcc_i),
        .TXKERR                         (),
        .TXRUNDISP                      (),
        //----------------------- Transmit Ports - GTX Ports -----------------------
        .GTXTEST                        (GTXTEST_IN),
        .MGTREFCLKFAB                   (),
        .TSTCLK0                        (tied_to_ground_i),
        .TSTCLK1                        (tied_to_ground_i),
        .TSTIN                          (20'b11111111111111111111),
        .TSTOUT                         (),
        //---------------- Transmit Ports - TX Data Path interface -----------------
        .TXDATA                         (txdata_i),
        .TXOUTCLK                       (TXOUTCLK_OUT),
        .TXOUTCLKPCS                    (),
        .TXRESET                        (TXRESET_IN),
        .TXUSRCLK                       (tied_to_ground_i),
        .TXUSRCLK2                      (TXUSRCLK2_IN),
        //-------------- Transmit Ports - TX Driver and OOB signaling --------------
        .TXBUFDIFFCTRL                  (3'b100),
        .TXDIFFCTRL                     (4'b0000),
        .TXINHIBIT                      (tied_to_ground_i),
        .TXN                            (TXN_OUT),
        .TXP                            (TXP_OUT),
        .TXPOSTEMPHASIS                 (5'b00000),
        //------------- Transmit Ports - TX Driver and OOB signalling --------------
        .TXPREEMPHASIS                  (4'b0000),
        //--------- Transmit Ports - TX Elastic Buffer and Phase Alignment ---------
        .TXBUFSTATUS                    (),
        //------ Transmit Ports - TX Elastic Buffer and Phase Alignment Ports ------
        .TXDLYALIGNDISABLE              (tied_to_vcc_i),
        .TXDLYALIGNMONENB               (tied_to_ground_i),
        .TXDLYALIGNMONITOR              (),
        .TXDLYALIGNOVERRIDE             (tied_to_ground_i),
        .TXDLYALIGNRESET                (tied_to_ground_i),
        .TXDLYALIGNUPDSW                (tied_to_vcc_i),
        .TXENPMAPHASEALIGN              (tied_to_ground_i),
        .TXPMASETPHASE                  (tied_to_ground_i),
        //--------------------- Transmit Ports - TX PLL Ports ----------------------
        .GREFCLKTX                      (tied_to_ground_i),
        .GTXTXRESET                     (GTXTXRESET_IN),
        .MGTREFCLKTX                    (MGTREFCLKTX_IN),
        .NORTHREFCLKTX                  (tied_to_ground_vec_i[1:0]),
        .PERFCLKTX                      (tied_to_ground_i),
        .PLLTXRESET                     (PLLTXRESET_IN),
        .SOUTHREFCLKTX                  (tied_to_ground_vec_i[1:0]),
        .TXPLLLKDET                     (TXPLLLKDET_OUT),
        .TXPLLLKDETEN                   (tied_to_vcc_i),
        .TXPLLPOWERDOWN                 (tied_to_ground_i),
        .TXPLLREFSELDY                  (tied_to_ground_vec_i[2:0]),
        .TXRATE                         (tied_to_ground_vec_i[1:0]),
        .TXRATEDONE                     (),
        .TXRESETDONE                    (TXRESETDONE_OUT),
        //------------------- Transmit Ports - TX PRBS Generator -------------------
        .TXENPRBSTST                    (tied_to_ground_vec_i[2:0]),
        .TXPRBSFORCEERR                 (tied_to_ground_i),
        //------------------ Transmit Ports - TX Polarity Control ------------------
        .TXPOLARITY                     (tied_to_ground_i),
        //--------------- Transmit Ports - TX Ports for PCI Express ----------------
        .TXDEEMPH                       (tied_to_ground_i),
        .TXDETECTRX                     (tied_to_ground_i),
        .TXELECIDLE                     (tied_to_ground_i),
        .TXMARGIN                       (tied_to_ground_vec_i[2:0]),
        .TXPDOWNASYNCH                  (tied_to_ground_i),
        .TXSWING                        (tied_to_ground_i),
        //------------------- Transmit Ports - TX Ports for SATA -------------------
        .COMFINISH                      (),
        .TXCOMINIT                      (tied_to_ground_i),
        .TXCOMSAS                       (tied_to_ground_i),
        .TXCOMWAKE                      (tied_to_ground_i)

     );
     
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
// PRBS39_TEST
//    J. Gilmore, 4/20/11
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

        prbs39 randombit(init_dat, en, out, rst, clk);
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
	      out_r <= out;
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
// PRBS39
//    J. Gilmore, 4/20/11
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
