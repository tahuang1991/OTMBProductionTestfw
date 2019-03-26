`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Create Date:    12:22 4/20/11
// Design Name: 
// Module Name:    rad_test 
//
// -Full RadTest system: GbE packet commands & data, BRAM readout, Snap12 and Translator loopbacks-
//  1: start with GbE commands for BRAM and counters readout (f3, f2, f1); build LFSRs!
//  1.2: made some fixes...still not responding to GbE commands
//  1.3: add some debug signals for cmd_code registration
//  1.4: undo debug, move cmd_code to rx_count=3 (was 10)
//  1.5: rearrange rx_count logic, use SLOW clock for lvlshft logic, initialize all lvlshft to same value
//  1.6: add bk_adr in rx_count logic, used for data_bram select; add data_bram pipe register after MUX before tx_dat;
//           initialize all lvlshft to unique values  -- GOOD result, but still see Bad State ~30%
//  1.7: add ck15125 for lvlshft test; add LED debug to test bad state after reProgram.  change some &'s into &&
//         --> see that rx_count sometimes toggles for 71 usec after Reset!  WHY???  And it freezes at xxx10000.
//  1.8: cleanup the rx_count init code...
//  1.9: fix the rxdv/rx_count>4400/rx_timeout logic; change lvlshft init values
//  1.10: use .h include file for memory init values; bring out_lvlshft[23:19] to LEDs to see them flip
//  1.11: more debug.  1.12: removed "integer" in .h file, may be worse!
//  1.13: fix f2f2 ireg 23 bug, try new param mem_init format in .h file
//  1.14: switch to 32broms .h file, configure for 33 BRAMs; add logic to handle f1f1 and f2f2 commands
//  1.15: add oob handling for bk_adr w/64 broms; removed old commented code bits
//  1.16-17: add some forced distributed ROMs, increase f3f3 to 4096 bytes (pkt_lim=2059)
//  1.18: make a DC signal level mode on LEDs
//  1.19: tune TestLEDs, re-enable Snap12 Logic:  fix tx_cadr.  old tx_clk2 -> gbe_txclk2, rx_count -> gbe_rxcount,
//    rx_dat -> gbe_rxdat, tx_dat -> gbe_txdat.  Added TX_SYNC module, call to RAMB16_S18_S18, SNAP12_T20R20_TOP
//  1.19b: fix mem_00 in the bram64.h file, bring out Snap[6] Rx data/k/dv signals to TestLEDs
//  1.20: go for 128 brams, 32 crams.  It works great!
//  1.21: go for 256 brams, 64 crams?
//  1.22: make snap12 transmit continuously after startup, show tx_dat on testLEDs
//  1.23: make snap12 ignore rxdv after the first one, assume continuous synchronous looping
//    -- still gets lost after fibers are unplugged/replugged; consider auto reset after 3 consecutive bad checks?
//
//  2: implement random data on Snap12 links; use individual random seeds
//  2.1: randmomized the cram .mem file, should use a LOT more CLBs!  Now 4% used, only 3 are memory
//    -- only 4 CRAMs for now
//  2.2: try 16 CRAMs
//  2.3: back to 64 CRAMs.  Over 51 hr compile time!
//  2.4: made fully random 256BROMs file, but back to fast/simple CRAM init file for test.
// -> still need to add & test Function4 (send regular GbE data packets...how fast?)
//  2.5: reassign 7 pins for new loopback board pins.  Works great!
//  2.6: add "Force Error" feature for 32 loopbacks (sw7 & !pb)
//  2.7: add Function4 capability to send ~100 GbE packets/sec and ignore commands from GbE (sw8 & !sw7)
//  2.8: go back to full-random CRAM file;  64 random CRAMs (already had the 256 BRAMs)
//         -- reduce CCLK program speed from 40 MHz to 22 MHz, just for extra safety margin, matches UCLA test
//  2.9: increase F4 GbE packet rate to 1215 Hz (16 times faster than v2.8) using f4loop_tc logic
//
//  //
//////////////////////////////////////////////////////////////////////////////////
module rad_test(
    input 	      ck_gben, ck_gbep,
    input 	      ck125n, ck125p,
    input 	      ck160n, ck160p,
    input 	      lhc_ckn, lhc_ckp,
    input 	      gbe_rxn, gbe_rxp,
    input  [8:7]      sw,
    input 	      pb,
    input 	      gbe_fok, qpll_lock,
    input 	      prom_d3, prom_d7, jtag_fpga3, sda0, tmb_sn, t_crit, // not used now
    input 	      alct_rx13, alct_rx19, alct_rx23, // not used now
    input [11:7]      alct_rx, // not used now
    input [28:5]      rpc_rx, // 24 inputs from TI Level Shifters
    output 	      gbe_txn, gbe_txp,
    output 	      f_sclk, rst_qpll,
    output [361:340]  io_, // outs to TI Level Shifters via Loopback PCB. don't need 360, 358
    output 	      io_286, io_298, io_306, io_310, // more outs to TI Level Shifters via loopback
    output 	      io_259, io_283, io_287, io_291, io_299, io_307, io_311, // new outs to new loopback
    output reg [7:0]  led_low,
    output reg [15:8] led_hi,
    output reg [10:1] test_led,

    input 	      t12_fault, r12_fok,
    input  [7:0]      rxn, rxp,
    output [7:0]      txn, txp,
    output 	      t12_rst, t12_sclk, r12_sclk
   );

// snap12 GTX signals
   wire [15:0] send_lim;
   wire        all_rx_ready, all_tx_ready;
   wire        snap_clk2, ck160_locked;
   wire        snap_wait;
   wire [7:0]  check_ok_snapr, check_bad_snapr;
   wire [7:0]  rxdv_snapr, rxcomma_snapr, synced_snapt;
   wire [7:0]  rxdv_diff, rxcomma_diff;
   wire [7:0]  lgood_snapr, lbad_snapr, llost_snapr;
   wire [15:0] snap_rxdat;
   wire [1:0]  snap_rxk;
   reg  [7:0]  check_okr_snapr, check_badr_snapr;
   reg  [7:0]  rxdvr_snapr, rxcommar_snapr, syncedr_snapt;
   reg [15:0]  snap_rxdat_r;
   reg  [1:0]  snap_rxk_r;
   reg [15:0]  snap_count;
   reg  [7:0]  time_r_snap;
   reg  [7:0]  time_snap;
   reg [15:0]  snap_tx_dat;
   reg  [1:0]  snap_kout;
   reg 	       snap_comma_align, snap_count_send;
   reg 	       snap_rstdone_r2, snap_rstdone_r3;
   wire [15:0] snap_bram;
   wire [1:0]  snapp_bram;

   wire  ck15125, stopped, locked, stop40, lock40;
   wire  gbe_refck; // GTXE1 ref clk, used for MGTref and DoubleReset clock
   wire  gbe_tx_outclk; // out from Tx PLL
   wire  gbe_txclk2;   // drives logic for Tx, the "internal" choice for USR clock
   wire  all_locked, all_ready, txoutclk_mmcm_lk;

   parameter SEEDBASE = 64'h8731c6ef4a5b0d29;
   parameter SEEDSTEP = 16'hc01d;
   parameter BRAM_LIM = 12'hbff; // Try 256 Brom's.  these are Dout from BRAMs
     // make sure BRAM_LIM matches rad_testXXXbroms.h file size!
   parameter CRAM_LIM = 131071;  // put all CROM pages into one big block, 2048 words each.
     // 4 is OK, now try 32.  64 pages is 131071:0, 32 is 65535:0, 16 is 32767:0, 4 is 8191:0
   parameter GBE_LIM = 16'h080b; // WORDcount limit. allow extra bytes for MAC preamble, addr's...
   reg [15:0] pkt_lim; // BytecountLimit/2. Plus extra 22 bytes for MAC preamble etc.
   reg [15:0] counter;  // ^^^ 1st data word @ counter == 12; need 11 extra word counts, 22 bytes.
   reg [5:0]  ireg;
   reg [10:0] tx_adr;
   reg [16:0] tx_cadr;
   reg [15:0] gbe_rxcount;
   reg [22:0] pkt_time, pkt_time_r;
   reg [15:0] free_count;
   reg 	      free_tc;
   reg 	      f4loop_tc, f4loop_tc_r;
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
   reg 	       l_lock40, ck160_locklost;
   reg 	       l_lock125, ck125_locklost;
   wire reset, gtx_reset;
   wire ckgbe, ck125, ck160, lhc_ck;   // 125 ref, ext 125 usr, QPLL 160, QPLL 40
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

   reg  [31:0] err_seq, err_flag;  // slow seq .1 sec shift, loads to flag on force_err  err_seq = 32'h00000001;
   reg 	       force_err;  //  <- sw7 & !pb, clears on pb high after 2nd seq. shift (debounce)
   reg 	       err_wait;   // force_err & tc & !wait -> load err_flag, set wait.  !force_err & tc & wait -> clear wait

   
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


   reg [38:0] init_lvlshft[23:0];
   reg en_levelshift;
   wire [23:0] in_lvlshft, out_lvlshft;
   wire [15:0] count_lvlshft[23:0];    // 24 16-bit wide elements, error counts for level shifting translator signals
   wire [31:0] snap_errcount[7:0];    // 8 32-bit wide elements,  error counts for snap12 tx-rx loops
   integer     i;

   assign in_lvlshft[23:0] = rpc_rx[28:5];
   assign io_[361:340] = {out_lvlshft[0],1'b0,out_lvlshft[2],1'b1,out_lvlshft[4],out_lvlshft[3],out_lvlshft[6],
		   out_lvlshft[1],out_lvlshft[8],out_lvlshft[7],out_lvlshft[10],out_lvlshft[5],out_lvlshft[12],
		   out_lvlshft[11],out_lvlshft[14],out_lvlshft[9],out_lvlshft[16],out_lvlshft[15],out_lvlshft[18],
		   out_lvlshft[13],out_lvlshft[20],out_lvlshft[19]};
   assign io_259 = out_lvlshft[9];   // new loopback output
   assign io_283 = out_lvlshft[12];  // new loopback output
   assign io_287 = out_lvlshft[16];  // new loopback output
   assign io_291 = out_lvlshft[20];  // new loopback output
   assign io_299 = out_lvlshft[13];  // new loopback output
   assign io_307 = out_lvlshft[17];  // new loopback output
   assign io_310 = out_lvlshft[17];  // was 310
   assign io_306 = out_lvlshft[21];  // was 306
   assign io_311 = out_lvlshft[21];  // new loopback output
   assign io_298 = out_lvlshft[22];  // A2, needs wire to socket on Mezz
   assign io_286 = out_lvlshft[23]; // B3, needs wire to socket on Mezz

/*  Level Shifting Translator IO map
 io286->rpc_rx28   shft_b23
 io298->rpc_rx27   shft_b22
 io306->io394 rpc_rx26  shft_b21
 io310->io390 rpc_rx22  shft_b17

 io340->io392 rpc_rx24  shft_b19
 io341->io393 rpc_rx25  shft_b20
 io342->io386 rpc_rx18  shft_b13
 io343->io391 rpc_rx23  shft_b18
 io344->io388 rpc_rx20  shft_b15
 io345->io389 rpc_rx21  shft_b16
 io346->io382 rpc_rx14  shft_b9
 io347->io387 rpc_rx19  shft_b14
 io348->io384 rpc_rx16  shft_b11
 io349->io385 rpc_rx17  shft_b12
 io350->io378 rpc_rx10  shft_b5
 io351->io383 rpc_rx15  shft_b10
 io352->io380 rpc_rx12  shft_b7
 io353->io381 rpc_rx13  shft_b8
 io354->io374 rpc_rx6  shft_b1
 io355->io379 rpc_rx11  shft_b6
 io356->io376 rpc_rx8  shft_b3
 io357->io377 rpc_rx9  shft_b4
 io359->io375 rpc_rx7  shft_b2
 io361->io373 rpc_rx5  shft_b0
 XX io358->io370 rpc_rx2
*/
   (* ram_style = "distributed" *)
   reg [15:0] cram [CRAM_LIM:0];  // put all CROM pages into one big block
/* Ben's distributed RAM inference method
        reg [23:0] ram1 [31:0];
        reg [23:0] ram2 [31:0];
        reg [23:0] ram3 [31:0];
...
        initial begin
           $readmemh ("ADC_ram1_contents", ram1, 0, 31);
           $readmemh ("ADC_ram2_contents", ram2, 0, 31);
           $readmemh ("ADC_ram3_contents", ram3, 0, 31);
        end
...
   assign mem_out1 = ram1[raddr];
   assign mem_out2 = ram2[raddr];
   assign mem_out3 = ram3[raddr];
...
//
// Infer distributed RAM for storing ADC configuration memory
//
   always @(posedge CLK) begin
           if(we[0])
                   ram0[a] <= mem_in;
        end
   always @(posedge CLK) begin
           if(we[1])
                   ram1[a] <= mem_in;
        end
   always @(posedge CLK) begin
           if(we[2])
                   ram2[a] <= mem_in;
        end
   always @(posedge CLK) begin
           if(we[3])
                   ram3[a] <= mem_in;
        end
        
The file format is just one word on each line for each memory location.  The
words can be decimal, binary, or hex depending on whether you use $readmem,
$readmemb, or $readmemh:
000000
010010
0F0000
110000
 ...
 */

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
      en_levelshift = 1'b1;
      for (i = 0; i < 24; i = i + 1) begin
	 if (i < 22) init_lvlshft[i] = 11401017*i;
	 else begin
	    init_lvlshft[22] = 39'h0a5b9c876e;
	    init_lvlshft[23] = 39'h123456789a;
	 end
      end
      gbe_rxcount = 16'h0000;
      good_rx_cmd = 1'b0;
      rx_timeout = 1'b0;
      f4loop_tc_r = 1'b0;
      loop_command = 1'b0;
      pkt_id = 8'h0000;
      $readmemh ("cram_rand64.mem", cram); // rand contents: compile takes 50+ hrs.  30% SLICEs used.
//      $readmemh ("cram_init64.mem", cram); // make sure this matches with CRAM_LIM!  fast but few SLICEs
//      $readmemh ("cram_init.mem", cram, 0, 8191); // 4 pages here; 2048 each
//      $readmemh ("cram_rand16.mem", cram); // SMALL random file contents: took 4 hours!
      err_seq = 32'h00000001;
      force_err = 0;
   end


   IBUFGDS #(.DIFF_TERM("TRUE"),.IOSTANDARD("LVDS_25"))  lhc_clock(.I(lhc_ckp) , .IB(lhc_ckn) , .O(lhc_ck));
   IBUFGDS #(.DIFF_TERM("FALSE"),.IOSTANDARD("LVDS_25"))  clock125(.I(ck125p) , .IB(ck125n) , .O(ck125));
//   IBUFDS_GTXE1 #(.DIFF_TERM("TRUE"),.IOSTANDARD("LVDS_25"))  clock_gbe(.I(ck_gbep) , .IB(ck_gben) , .O(ckgbe));
   IBUFDS_GTXE1  clock_gbe(.I(ck_gbep) , .IB(ck_gben) , .O(ckgbe), .ODIV2(), .CEB(zero));
//   IBUFDS_GTXE1  clock160(.I(ck160p) , .IB(ck160n) , .O(ck160), .ODIV2(), .CEB(zero));

   BUFG gbe_refclock(.I(ckgbe), .O(gbe_refck));
   bufg_div25clk clk5(ck125,clk,stopped,locked); // can we get 125 and 5 MHz outputs here?  Also 62.5 MHz?

//   bufg_div4clk clk40(ck160,ck40,stop40,lock40); // can we get 160 and 40 MHz outputs here?
   bufg_div4clk clk40(snap_clk2,ck40,stop40,lock40); // can we get 160 and 40 MHz outputs here?

   MGT_USRCLK_SOURCE_MMCM #
    (
        .MULT                           (8.0),
        .DIVIDE                         (1),
        .CLK_PERIOD                     (8.0),
        .OUT0_DIVIDE                    (16.0),
        .OUT1_DIVIDE                    (64.0),
        .OUT2_DIVIDE                    (1),
        .OUT3_DIVIDE                    (1)
     )
    txusrclk_mmcm
    (
        .CLK0_OUT                       (gbe_txclk2),
        .CLK1_OUT                       (ck15125),
        .CLK2_OUT                       (),
        .CLK3_OUT                       (),
        .CLK_IN                         (ck125),  // use ck125 or gbe_tx_outclk...or gbe_refck?
        .MMCM_LOCKED_OUT                (txoutclk_mmcm_lk),
        .MMCM_RESET_IN                  (!rxpll_lk)
     );


    genvar u;
    generate
       for (u=0; u<24; u=u+1) begin:prbs39gen
	  prbs39_test lvlshft_b(init_lvlshft[u], en_levelshift, in_lvlshft[u], out_lvlshft[u], count_lvlshft[u], err_wait&err_flag[u], !l_lock125, ck15125);
       end
    endgenerate

// `include "rad_test256broms.h"
`include "random_256broms.h"

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
        .INIT_00(MEM_00[v]),
        .INIT_01(MEM_01[v]),
        .INIT_02(MEM_02[v]),
        .INIT_03(MEM_03[v]),
        .INIT_04(MEM_04[v]),
        .INIT_05(MEM_05[v]),
        .INIT_06(MEM_06[v]),
        .INIT_07(MEM_07[v]),
        .INIT_08(MEM_08[v]),
        .INIT_09(MEM_09[v]),
        .INIT_0A(MEM_0A[v]),
        .INIT_0B(MEM_0B[v]),
        .INIT_0C(MEM_0C[v]),
        .INIT_0D(MEM_0D[v]),
        .INIT_0E(MEM_0E[v]),
        .INIT_0F(MEM_0F[v]),
        .INIT_10(MEM_10[v]),
        .INIT_11(MEM_11[v]),
        .INIT_12(MEM_12[v]),
        .INIT_13(MEM_13[v]),
        .INIT_14(MEM_14[v]),
        .INIT_15(MEM_15[v]),
        .INIT_16(MEM_16[v]),
        .INIT_17(MEM_17[v]),
        .INIT_18(MEM_18[v]),
        .INIT_19(MEM_19[v]),
        .INIT_1A(MEM_1A[v]),
        .INIT_1B(MEM_1B[v]),
        .INIT_1C(MEM_1C[v]),
        .INIT_1D(MEM_1D[v]),
        .INIT_1E(MEM_1E[v]),
        .INIT_1F(MEM_1F[v]),
        .INIT_20(MEM_20[v]),
        .INIT_21(MEM_21[v]),
        .INIT_22(MEM_22[v]),
        .INIT_23(MEM_23[v]),
        .INIT_24(MEM_24[v]),
        .INIT_25(MEM_25[v]),
        .INIT_26(MEM_26[v]),
        .INIT_27(MEM_27[v]),
        .INIT_28(MEM_28[v]),
        .INIT_29(MEM_29[v]),
        .INIT_2A(MEM_2A[v]),
        .INIT_2B(MEM_2B[v]),
        .INIT_2C(MEM_2C[v]),
        .INIT_2D(MEM_2D[v]),
        .INIT_2E(MEM_2E[v]),
        .INIT_2F(MEM_2F[v]),
        .INIT_30(MEM_30[v]),
        .INIT_31(MEM_31[v]),
        .INIT_32(MEM_32[v]),
        .INIT_33(MEM_33[v]),
        .INIT_34(MEM_34[v]),
        .INIT_35(MEM_35[v]),
        .INIT_36(MEM_36[v]),
        .INIT_37(MEM_37[v]),
        .INIT_38(MEM_38[v]),
        .INIT_39(MEM_39[v]),
        .INIT_3A(MEM_3A[v]),
        .INIT_3B(MEM_3B[v]),
        .INIT_3C(MEM_3C[v]),
        .INIT_3D(MEM_3D[v]),
        .INIT_3E(MEM_3E[v]),
        .INIT_3F(MEM_3F[v]),
        .INIT_40(MEM_40[v]),
        .INIT_41(MEM_41[v]),
        .INIT_42(MEM_42[v]),
        .INIT_43(MEM_43[v]),
        .INIT_44(MEM_44[v]),
        .INIT_45(MEM_45[v]),
        .INIT_46(MEM_46[v]),
        .INIT_47(MEM_47[v]),
        .INIT_48(MEM_48[v]),
        .INIT_49(MEM_49[v]),
        .INIT_4A(MEM_4A[v]),
        .INIT_4B(MEM_4B[v]),
        .INIT_4C(MEM_4C[v]),
        .INIT_4D(MEM_4D[v]),
        .INIT_4E(MEM_4E[v]),
        .INIT_4F(MEM_4F[v]),
        .INIT_50(MEM_50[v]),
        .INIT_51(MEM_51[v]),
        .INIT_52(MEM_52[v]),
        .INIT_53(MEM_53[v]),
        .INIT_54(MEM_54[v]),
        .INIT_55(MEM_55[v]),
        .INIT_56(MEM_56[v]),
        .INIT_57(MEM_57[v]),
        .INIT_58(MEM_58[v]),
        .INIT_59(MEM_59[v]),
        .INIT_5A(MEM_5A[v]),
        .INIT_5B(MEM_5B[v]),
        .INIT_5C(MEM_5C[v]),
        .INIT_5D(MEM_5D[v]),
        .INIT_5E(MEM_5E[v]),
        .INIT_5F(MEM_5F[v]),
        .INIT_60(MEM_60[v]),
        .INIT_61(MEM_61[v]),
        .INIT_62(MEM_62[v]),
        .INIT_63(MEM_63[v]),
        .INIT_64(MEM_64[v]),
        .INIT_65(MEM_65[v]),
        .INIT_66(MEM_66[v]),
        .INIT_67(MEM_67[v]),
        .INIT_68(MEM_68[v]),
        .INIT_69(MEM_69[v]),
        .INIT_6A(MEM_6A[v]),
        .INIT_6B(MEM_6B[v]),
        .INIT_6C(MEM_6C[v]),
        .INIT_6D(MEM_6D[v]),
        .INIT_6E(MEM_6E[v]),
        .INIT_6F(MEM_6F[v]),
        .INIT_70(MEM_70[v]),
        .INIT_71(MEM_71[v]),
        .INIT_72(MEM_72[v]),
        .INIT_73(MEM_73[v]),
        .INIT_74(MEM_74[v]),
        .INIT_75(MEM_75[v]),
        .INIT_76(MEM_76[v]),
        .INIT_77(MEM_77[v]),
        .INIT_78(MEM_78[v]),
        .INIT_79(MEM_79[v]),
        .INIT_7A(MEM_7A[v]),
        .INIT_7B(MEM_7B[v]),
        .INIT_7C(MEM_7C[v]),
        .INIT_7D(MEM_7D[v]),
        .INIT_7E(MEM_7E[v]),
        .INIT_7F(MEM_7F[v]),
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

   assign all_locked = locked & txoutclk_mmcm_lk;
   assign gtx_ready = tx_resetdone_r & rx_resetdone_r & txoutclk_mmcm_lk;
   assign all_ready = locked & tx_resetdone_r & rx_resetdone_r;
   assign rxdv = ~(|rxer | rx_bufstat[2]) & gbe_fok; // idle is K28.5,D16.2  =  BC,50 in time order
   assign reset = !sw[7] & !pb;
   assign gtx_reset = reset | !gtx_ready;

   always @(posedge clk or posedge reset) // everything that uses 5 MHz clock with simple Reset
     begin
	if (reset == 1) begin
	   tx_resetdone_r <= 0;
	   rx_resetdone_r <= 0;
	   free_count <= 0;
	   time_count <= 0;
	   pkt_time <= 23'h00000001;
	   free_tc <= 0;
	   f4loop_tc <= 0;
	   err_seq <= 32'h00000001;
	   err_flag <= 32'h00000000;
	   err_wait <= 0;
	   force_err <= 0;
	end
	else begin
	   tx_resetdone_r <= tx_resetdone;
	   rx_resetdone_r <= rx_resetdone;
	   free_count <= free_count + 1'b1;
	   if (free_count[11:0] == 12'hfff) f4loop_tc <= 1;  // ~1215 Hz or .0008 sec, for F4 GbE packet rate
	   else f4loop_tc <= 0;

	   if (free_count == 16'hffff) free_tc <= 1;  // ~76 Hz or .013 sec, to debounce forced error test
	   else free_tc <= 0;

	   if (free_tc) err_seq[31:0] <= {err_seq[30:0],err_seq[31]};
	   if (!force_err) begin
	      force_err <= sw[7] & !pb;
	      err_wait <= 0;
	   end
	   else begin
	      if (free_tc & !err_wait) begin
		 err_flag <= err_seq;
		 err_wait <= 1;  // use this to sync with Xmit clocks & load err_flag in their domains
	      end
	      else if (free_tc & err_wait) begin
		 force_err <= sw[7] & !pb;
	      end
	      else begin
		 err_flag <= 0;
	      end
	   end


	   if (gtx_ready && time_count < 8'hfe) time_count <= time_count + 1'b1; // use to delay startup; ends.
	   if (gtx_ready) pkt_time <= pkt_time + 1'b1; // use to space packets 1.67 sec apart; looping.
	end
     end

   always @(posedge ck40) // everything that uses ck40, no Reset
     begin
	time_40i <= time_count;
	time_40r <= time_40i;  // transfer slow clk time counter to ck40 domain
     end

   always @(posedge ck40 or posedge reset) // everything that uses ck40 w/simple Reset
     begin
	if (reset) begin
	   l_lock125 <= 0;    // use ck40(160) to monitor ck125
	   ck125_locklost <= 0;
	end
	else begin
	   if (time_40r[7] & locked) l_lock125 <= 1;
	   if (l_lock125 & (!locked)) ck125_locklost <= 1;
	end
     end

   always @(posedge gbe_txclk2) // everything that uses GbE USR clock, no Reset
     begin
	tx_out_r  <= tx_out;
	tx_kout_r <= tx_kout;
	time_r_i  <= time_count;
	data_bram_r  <= data_bram;
	data_cram_r  <= data_cram;
	if (bk_adr[11:8] == 4'hc) tx_cadr <= {bk_adr[5:0],tx_adr[10:0]};  // get 4 kB CROM pages from one big block
     end

   always @(posedge gbe_txclk2 or posedge reset) // everything using GbE USR clock w/simple Reset
     begin
	if (reset) begin
	   l_lock40 <= 0;    // use ck125 to monitor ck40(160)
	   ck160_locklost <= 0;
	   time_r <= 8'h00;
	   pkt_time_r <= 23'h00000001;
	end
	else begin
	   if (time_r[7] & lock40) l_lock40 <= 1;
	   if (l_lock40 & (!lock40)) ck160_locklost <= 1;
	   time_r <= time_r_i;  // transfer slow clk time counter to GbE USR clock domain
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
	   f4loop_tc_r <= 1'b0;
	   loop_command <= 1'b0;
	   pkt_id <= 8'h0000;
	   gbe_rxcount <= 16'h0000;
	   cmd_code <= 16'h0000;
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
	   en_levelshift <= 1'b1;
	   pkt_send <= 0; // 1st data word @counter == 12 --> 11 extra words, so subtract 22 bytes.
	   good_rx_cmd <= 1'b0;
	   rx_timeout <= 1'b0;
	end

	else begin // Not Reset case
	   if (sw[8] && !sw[7]) f4loop_tc_r <= f4loop_tc;
	   if (cmd_code == 16'h0000) loop_command <= f4loop_tc_r;

	   rx_resetdone_r2 <= rx_resetdone_r;
	   rx_resetdone_r3 <= rx_resetdone_r2;
	   if (rx_resetdone_r3) comma_align <= 1; // maybe use time_r > 8'h10 ?

	   crc_rst <= (data_state == 2'h2)? 1'b1 : 1'b0;  // needs one cycle during state 3

	   if (cmd_code == 16'hf3f3 || loop_command) begin  // load counter2 to READ registers and send it out
	      pkt_lim <= 16'd2059;   // 16'd61 special test size 100 bytes.  later will be 4 KB
	   end
	   else pkt_lim <= 16'd36; // our usual 50-byte size is default; F3F3 is the special 4 KB count

	   byte_count <= 16'hffff&((pkt_lim*2) - 16'd22); // Need BYTES; subtract preamble bytes etc.

	   if (data_state[1]) begin  // end of sent data packet
	      cmd_code <= 16'h0000;
	      bk_adr <= 12'h000;
	      pkt_send <= 1'b0;
	      counter <= 0;
	      en_levelshift <= 1'b1;
	      loop_command <= 1'b0;
	      pkt_id <= pkt_id + 1'b1;
	   end
	   else if (!sync_state[0] && pkt_send) begin
	      counter <= counter + 1'b1;  // verified that sync[0] ends OFF
	   end
 	   else if (!sync_state[0] && time_r[7] && (cmd_code > 16'h0000)) pkt_send <= 1'b1;

	   if (loop_command && (cmd_code == 16'h0000) ) cmd_code <= 16'hf4f4;

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
		 if(cmd_code[1]^cmd_code[0]) begin  // generalized for f1f1 or f2f2 ( == !f3f3 && !f4f4)
		    if (counter == 16'd12) begin
		       gbe_txdat <= cmd_code;    //  <-- first word returns the cmd_code
		       en_levelshift <= (cmd_code != 16'hf2f2);
		    end
		    else if (cmd_code == 16'hf1f1 && ireg < 8) begin  //  "en_snapshift" signal for the Snap12s
		       if (counter[0]) gbe_txdat <= snap_errcount[ireg][15:0]; // 8 32-bit wide elements in this array
		       else gbe_txdat <= snap_errcount[ireg][31:16];
		    end
		    else if (ireg < 24) gbe_txdat <= count_lvlshft[ireg]; // 24 16-bit wide elements in this array
		    else gbe_txdat <= 16'h0000; // ireg out of range
		 end
		 else if (cmd_code[2]) begin  // JGhere: Function4 is active
		   gbe_txdat <= {pkt_id[7:0],tx_adr[7:0]};  // send the sequential packet ID for each packet, plus data
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
	      if (counter_send) data_state <= 2'h1;
	      else if (data_state > 2'b0) data_state <= data_state + 2'h1;
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
		 en_levelshift <= 1'b1;
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


   always @(posedge snap_clk2) // everything that uses snap USR clock, w/o Reset
     begin
	time_r_snap <= time_count;
	time_snap <= time_r_snap;  // transfer slow clk time counter to snap USR clock domain
     end

   always @(posedge snap_clk2 or posedge reset or posedge snap_wait) // things that uses snap USR clock, w/Reset
     begin
	if (reset | snap_wait) begin
	   snap_comma_align <= 0;
	   snap_count_send <= 0;
	   snap_count <= 0;
	   snap_tx_dat <= 16'h50bc;  // <-- idle 28.5/16.2
	   snap_kout <= 2'h1;
	   snap_rstdone_r2 <= 0;
	   snap_rstdone_r3 <= 0;
	   check_okr_snapr  <= 0;
	   check_badr_snapr <= 0;
	   rxdvr_snapr    <= 0;
	   rxcommar_snapr <= 0;
	   syncedr_snapt  <= 0;
	   snap_rxdat_r   <= 0;
	   snap_rxk_r     <= 0;
/*	   for (i = 0; i < 8; i = i + 1) begin
	      snap_errcount[i] = 32'h00000000;
	   end */
	end

	else begin // Not Reset case
	   check_okr_snapr  <= check_ok_snapr;
	   check_badr_snapr <= check_bad_snapr;
	   rxdvr_snapr    <= rxdv_snapr;
	   rxcommar_snapr <= rxcomma_snapr;
	   syncedr_snapt  <= synced_snapt;
	   snap_rxdat_r   <= snap_rxdat;
	   snap_rxk_r     <= snap_rxk;

	   snap_rstdone_r2 <= all_rx_ready;
	   snap_rstdone_r3 <= snap_rstdone_r2;
	   if (snap_rstdone_r3) snap_comma_align <= 1; // maybe use time_r > 8'h10 ?
	   if (all_tx_ready & snap_rstdone_r3 & time_snap[7]) begin  // wait 3000 clocks after Reset
	      snap_count <= snap_count + 1'b1;
	   end

	   if ((snap_count > 0 || snap_count_send > 0) && time_snap[7] == 1) begin   // set tx_dat
	      snap_count_send <= 1;
	      snap_tx_dat <= snap_bram;
	      snap_kout <= 2'h0;
	   end
	   else  begin // send Idles for a while, allow time for Rx Checking
	      snap_count_send <= 0;
	      snap_tx_dat <= 16'h50bc;  // <-- idle 28.5/16.2
	      snap_kout <= 2'h1;
	   end // else: !if((snap_count > 0 || snap_count_send > 1) && time_snap[7] == 1)
        end

     end



   always @(*)
     begin
	if (sw[7] == 0) begin
	   if (sw[8] == 1) begin  // sw8 ON
	      led_hi[8] = !gbe_fok;   // always On
	      led_hi[9] = !all_ready; // always On
 	      led_hi[10] = !pkt_send; //
	      led_hi[15:11] = ~out_lvlshft[23:19];  //
	      test_led[1] = !gbe_fok; // 
	      test_led[2] = !all_ready; // 
	      test_led[3] = !time_r[7]; // 
	      test_led[4] = !lock40; // 
	      test_led[5] = ck160_locklost; // 
	      test_led[6] = !locked; // 
	      test_led[7] = ck125_locklost; // 
	      test_led[8] = gtx_reset; //
	      test_led[9] = snap_wait; // t12_fault, r12_fok,
	      test_led[10]  = 1'b1;     // sw8 High
/*	      test_led[8:1] = l_gbe_rxdat[15:8]; // 
	      test_led[9]   = kchar_r[1]; //
	      test_led[10]  = l_rxdv;     // */
	      led_low = 0;
	   end
	   else begin            // sw8 OFF      ck40,stop40,lock40
	      led_hi[8]  = ck40;     // used to check ck40 freq. from QPLL ck160  --toggles
	      led_hi[9]  = time_r[7];  // was !stop40 --zero
	      led_hi[10] = !pkt_send;  // was !lock40 --high
	      led_hi[11] = !ck160_locklost;  // QPLL ck160 lock was lost  --zero
	      led_hi[12] = lhc_ck;   // used to check ck40 freq. directly from QPLL  --toggles
//	      led_hi[13] = !qpll_lock;
	      led_hi[13] = !good_rx_cmd; // for bad state after Reset, NEVER goes true!
	      led_hi[14] = !locked;  //  --high
	      led_hi[15] = !ck125_locklost;  //  --zero
/*	   test_led[2:1]   = lbad_snapr[1:0]; //  
	   test_led[4:3]   = llost_snapr[1:0]; // 
	   test_led[6:5]   = lgood_snapr[1:0]; // 
           test_led[7]   = snap_wait;   */
	      test_led[1] = !gbe_fok; // 
	      test_led[2] = !all_ready; // 
	      test_led[3] = !time_r[7]; // 
	      test_led[4] = !lock40; // 
	      test_led[5] = ck160_locklost; // 
	      test_led[6] = !locked; // 
	      test_led[7] = ck125_locklost; //
	      test_led[8] = gtx_reset; //
	      test_led[9] = snap_wait; //
	      test_led[10]  = 1'b0;     // sw8 Low
/*
 	      test_led[8:1] = l_gbe_rxdat[7:0]; // counter[7:0];
	      test_led[9]   = kchar_r[0]; // gbe_fok;
	      test_led[10]  = l_rxdv;  // all_ready;   */
	      led_low = 8'b11111111;   // Prom D[7:0], go to TMB LEDs
	   end
	end
	else if (sw[8] == 0) begin  // *** sw7 ON, from here on
  	   led_hi[8]  = !snap_count_send;
	   led_hi[9]  = !crc_en;    // goes True at end of 2nd h55, False at end of hFE.  Never True in bad state.
	   led_hi[10] = snap_count[0];
	   led_hi[11] = !gtx_reset; // OFF
	   led_hi[13:12] = ~sync_state[1:0];    // Mezz LED hardware is inverse  ON/OFF
	   led_hi[15:14] = ~data_state[1:0];    // Mezz LED hardware is inverse  ALWAYS OFF/OFF!
	   led_low = 0;      //
	   test_led[8:1] = snap_rxdat_r[7:0]; // tx_out or l_gbe_rxdat[15:8];
	   test_led[9]   = snap_rxk_r[0];  // tx_kout_r or kchar_r[1];
	   test_led[10]  = rxdvr_snapr[6];  // counter_send
/*	      test_led[4:1] = l_gbe_rxdat[3:0]; // counter[7:0];
	      test_led[8:5] = cmd_code[3:0]; // counter[7:0];
	      test_led[9]   = good_rx_cmd; // gbe_fok;
	      test_led[10]  = l_rxdv;  // all_ready;  */
	end  // if (sw[8] == 0)
	else begin  // display counter value, when Both sw ON
	   led_hi[8]  = !snap_count_send;
	   led_hi[9]  = !crc_en;       // !mac_sync  OFF
	   led_hi[10] = snap_count[0];
	   led_hi[11] = !gtx_ready; //       ON
	   led_hi[13:12] = ~sync_state[1:0];    // Mezz LED hardware is inverse  ON/OFF
	   led_hi[15:14] = ~data_state[1:0];    // Mezz LED hardware is inverse  OFF/OFF
	   test_led[8:1] = snap_rxdat_r[15:8];  // tx_out or l_gbe_rxdat[7:0];
	   test_led[9]   = snap_rxk_r[1];   // tx_kout_r or kchar_r[0]; //
	   test_led[10]  = rxdvr_snapr[6];  // counter_send
	   led_low = 8'b11111111;  // Prom D[7:0], go to TMB LEDs
	end
     end

// jghere:
    SNAP12_T20R20_TOP snap12_gtxs (  // need to add err_wait & err_flag[31:24] here
	   .ck_160n (ck160n),
	   .ck_160p (ck160p),
	   .RXN (rxn),
	   .RXP (rxp),
	   .TXN (txn),
	   .TXP (txp),
	   .snap_clk2 (snap_clk2),
	   .ck_160_plllkdet (ck160_locked),
	   .GTXTXRESET_IN (reset),
	   .GTXRXRESET_IN (reset),
	   .SEED_STEP (SEEDSTEP),      // replace with SEEDSTEP 15-0
	   .gtx_wait (snap_wait),
	   .force_err_wait (err_wait),
           .force_err_flag (err_flag[31:24]),
	   .snap_commaalign (snap_comma_align),
	   .SEED_BASE (SEEDBASE),  // replace with SEEDBASE 64-0
	   .all_rx_ready (all_rx_ready),
	   .all_tx_ready (all_tx_ready),
	   .rxdv_snapr (rxdv_snapr),
	   .rxcomma_snapr (rxcomma_snapr),
	   .syncdone_snapt (synced_snapt),
	   .check_ok_snapr (check_ok_snapr),    // remove?
	   .check_bad_snapr (check_bad_snapr),  // remove?
	   .good_byte (lgood_snapr),  // remove?
	   .bad_byte (lbad_snapr),    // remove?
	   .lost_byte (llost_snapr),  // remove!
	   .snap_err0 (snap_errcount[0]),    // 32-bit wide error count for snap12 tx-rx loop
	   .snap_err1 (snap_errcount[1]),    // 32-bit wide error count for snap12 tx-rx loop
	   .snap_err2 (snap_errcount[2]),    // 32-bit wide error count for snap12 tx-rx loop
	   .snap_err3 (snap_errcount[3]),    // 32-bit wide error count for snap12 tx-rx loop
	   .snap_err4 (snap_errcount[4]),    // 32-bit wide error count for snap12 tx-rx loop
	   .snap_err5 (snap_errcount[5]),    // 32-bit wide error count for snap12 tx-rx loop
	   .snap_err6 (snap_errcount[6]),    // 32-bit wide error count for snap12 tx-rx loop
	   .snap_err7 (snap_errcount[7]),    // 32-bit wide error count for snap12 tx-rx loop
	   .rxdv_diff (rxdv_diff), // diff between my RxDV/Comma and Theirs
	   .rxcomma_diff (rxcomma_diff),
	   .rxdata_i (snap_rxdat),
	   .RXK_OUT (snap_rxk)
    );

   assign send_lim = 16'h0800; // h0800=dec 2048, h0300=dec 768  --now try to ignore this!
   assign snap_wait = !(&synced_snapt & all_rx_ready & ck160_locked);


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
	input [38:0] init_dat,
	input        en,    // en allows data to go out and checks on the return
	input        in,    // this is the data bit received, Tpd < 10 ns?  Fmax=50Mhz
	output       out,   // this is the data bit to send out, on pos-clk
	output reg [15:0]  count, // this is a 16-bit counter for # of errors
        input  force_error,  // this needs to trigger a  _single_  bit-flip
	input        rst,
	input        clk);  // assume 10-20 MHz operation

        reg 	    en_r, in_r, ferr_r, ferr_rr, ferr_done;

   prbs39 randombit(init_dat, en, out, rst, clk);

	always @(posedge clk, posedge rst) begin
	   if(rst) begin
	      en_r <= 0;
	      count  <= 0;
	      ferr_r <= 0;
	      ferr_rr <= 0;	      
	   end
	   else begin
	      en_r <= en;
	      if (en && en_r && in_r != out) count <= count + 1'b1;

	      ferr_r <= force_error;
	      if (!ferr_done) begin
		 ferr_rr <= ferr_r;
		 ferr_done <= ferr_rr;
	      end
	      else begin
		 ferr_rr <= 0;
		 ferr_done <= force_error;
	      end

	   end
	end

	always @(negedge clk, posedge rst) begin  // do check after neg edge, count on pos edge?
	   if(rst) begin
	      in_r <= 0;
	   end
	   else begin
	      in_r <= in^ferr_rr;
	   end
	end // always
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

	always @(posedge clk, posedge rst) begin
	   if(rst) begin
	      lfsr[39:1]  <= init_dat[38:0];
	      out <= lfsr[39];
	   end
	   else begin
	      out <= lfsr[39];
	      if (en) lfsr[39:1] <= {lfsr[38:1],lfsr[39]^lfsr[4]};
	      else lfsr  <= lfsr;
	   end
	end // always
endmodule
