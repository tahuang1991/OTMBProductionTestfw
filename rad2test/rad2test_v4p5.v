`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Create Date:    12:12 4/30/12
// Design Name: 
// Module Name:    rad2test 
//
// - New for Rad Test 2012
//  4.2: first version that should work, added F5 command for CLB Shift Register voting test
//        -- only has 8 BRAMs employed for test F3, no CRAMs, and test F3 will fail
//        -- probably F4 will fail too (only 8 BRAMs), but we need to add Rx and Tx capability anyway...
//        -- test F2 for level shifter loopback should be OK, but runs at 84 MHz...
//  4.3: add new BRAM test: use F7 to WRITE BRAMs? And F3 reads them back. Then F7 + F3 replace F4. ~2 hour compile
//  4.4: double CSR_MAX to 16383, rx_adr is returned on GbE during F7F7 writes to BRAMs.  12 hour compile!
//        -- problems: cycle4 & cmd_f7f7 don't get cleared after F7F7; need to stop writes @ rx_adr=2047
//  4.5: increase BRAMs to 256, fix cycle4 & cmd_f7f7 clearing problem, stop writes after rx-adr 2047
// 
// 
// 
//  
// -Full 2010 RadTest system: GbE packet commands & data, BRAM readout, Snap12 and Translator loopbacks-
//  2.4: made fully random 256BROMs file, but back to fast/simple CRAM init file for test.
// -> still need to add & test Function4 (send regular GbE data packets...how fast?)
//  2.5: reassign 7 pins for new loopback board pins.  Works great!
//  2.6: add "Force Error" feature for 32 loopbacks (sw7 & !pb)
//  2.7: add Function4 capability to send ~100 GbE packets/sec and ignore commands from GbE (sw8 & !sw7)
//
//  3:   start from v2.7, modify translator tests to 80 MHz for 22 channels, 2 special ch. DC 1,0
//        -- use outreg and inreg, need to double pipeline the data for checking logic
//        -- tests may need to force errors to make sure it works
//  3.1: add 90 MHz clocking for TI translator tests
//  3.2: reduce BRAMs & CRAMs for fast compiling, try to force obuf_ff use.   --> snapr[6] out
//  3.3: slow translator test to 84 MHz & use IOB FFs (no help);  fixed (!) snap12_gtx ferr_r logic,
//         bring rxdvr[7:0] to test LEDs.  --> snapr[1] out to scope
//  3.4: add explicit IOB=TRUE code but still uses SLICEs;  fixed (!) F4 packet count logic (no skip)
//         --> snapr[2] out to scope
//  3.5: fixed gtx_wait connection to GTX4/snaptx[1] and GTX7/snaptx[2] inside snap12_t20r20_custom
//         --> snapr[1] out to scope
//
//
//////////////////////////////////////////////////////////////////////////////////
module rad2test(
    input 	      ck_gben, ck_gbep,
    input 	      ck125n, ck125p,
    input 	      ck160n, ck160p,
    input 	      ck80n, ck80p,
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
 // outs to TI Level Shifters via Loopback PCB. don't need 360, 358:
    output reg [361:340]  io_ /* synthesis xc_props = "IOB=true" */,
 // 4 more outs to TI Level Shifters via loopback:
    output reg	      io_286, io_298, io_306, io_310 /* synthesis xc_props = "IOB=true" */,
 // 7 new outs to new loopback:
    output reg	      io_259, io_283, io_287, io_291, io_299, io_307, io_311 /* synthesis xc_props = "IOB=true" */,
    output reg [7:0]  led_low,
    output reg [15:8] led_hi,
    output reg [10:1] test_led,

    input 	      t12_fault, r12_fok,
    input  [7:0]      rxn, rxp,
    output [7:0]      txn, txp,
    output 	      t12_rst, t12_sclk, r12_sclk
   )/* synthesis syn_useioff = 1 */;



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

   wire  ck15125, stopped, locked, stop40, lock40, ck90, ck84;
   wire  gbe_refck; // GTXE1 ref clk, used for MGTref and DoubleReset clock
   wire  gbe_tx_outclk; // out from Tx PLL
   wire  gbe_txclk2;   // drives logic for Tx, the "internal" choice for USR clock
   wire  all_locked, all_ready, txoutclk_mmcm_lk;

   parameter SEEDBASE = 64'h8731c6ef4a5b0d29;
   parameter SEEDSTEP = 16'hc01d;
   parameter BRAM_LIM = 12'hbff; // bff: Try 256 Bram's -> bff, or 8 -> b07.
   parameter CSR_MAX = 16383;  // CLB Shift Register length, start with 8192 bits.  replaces CRAMs
   parameter GBE_LIM = 16'h080b; // WORDcount limit. allow extra bytes for MAC preamble, addr's...
   reg [15:0] pkt_lim; // BytecountLimit/2. Plus extra 22 bytes for MAC preamble etc.
   reg [15:0] counter;  // ^^^ 1st data word @ counter == 12; need 11 extra word counts, 22 bytes.
   reg [5:0]  ireg;
   reg [10:0] tx_adr, rx_adr;
   reg [16:0] tx_cadr;
   reg [15:0] gbe_rxcount;
   reg [22:0] pkt_time, pkt_time_r;
   reg [15:0] free_count;
   reg 	      free_tc, free_tc_r;
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

   reg  [15:0] data_bram, data_bram_r;  // these are the BRAM MUX bus & register
   wire [63:0] data_oram[BRAM_LIM:12'hb00];
   reg  [63:0] data_iram;
   wire [BRAM_LIM:12'hb00] sbiterr_oram,  dbiterr_oram;
   reg         cycle4, cmd_f7f7;  // use this to toggle bram WRITE every 4th GbE word during cmd=f7f7

   reg         crc_en, crc_rst;
   wire [31:0] crc_out;
   reg  [15:0] byte_count;

 // 24 registers for inputs from TI Level Shifters
   reg  [28:5] rpc_rx_r /* synthesis xc_props = "IOB=true" */;
   reg  [31:0] err_seq, err_flag;  // slow seq .1 sec shift, loads to flag on force_err  err_seq = 32'h00000001;
   reg 	       force_err;  //  <- sw7 & !pb, clears on pb high after 2nd seq. shift (debounce)
   reg 	       err_wait;   // force_err & tc & !wait -> load err_flag, set wait.  !force_err & tc & wait -> clear wait

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


   (* ram_style = "distributed" *)
   reg [CSR_MAX:0] csr1_r,csr2_r,csr3_r,csr4_r,csr5_r,csr6_r;  // the CLB Shift Register registers to replaces CRAMs
   wire 	      lfsr_bit;
   reg [38:0]  init_lvlshft[23:0], init_csrlfsr;
   reg [63:0]  csr80_count;
   reg 	       en_levelshift, en_csr, vote_a, vote_b, error_a, error_b, tri_vote_fail;
   reg  [31:0] tri_vote0_errcnt, vote0a_errcnt, vote0b_errcnt, tri_vote1_errcnt, vote1a_errcnt, vote1b_errcnt, tri_vote2_errcnt, vote2a_errcnt, vote2b_errcnt;
   wire [31:0] vote_errcnt[11:0];    // 12 32-bit wide elements,  CLB Shift Register (CSR) vote error counts
   wire [23:0] in_lvlshft, out_lvlshft;
   wire [15:0] count_lvlshft[23:0];    // 24 16-bit wide elements, error counts for level shifting translator signals
   wire [31:0] snap_errcount[7:0];    // 8 32-bit wide elements,  error counts for snap12 tx-rx loops
   integer     i;

   wire [361:340] io_i;
   wire  io_i259, io_i283, io_i287, io_i291, io_i299, io_i307, io_i310, io_i306, io_i311, io_i298, io_i286;

   assign in_lvlshft[23:0] = rpc_rx_r[28:5];
   assign io_i[361:340] = {out_lvlshft[0],1'b0,out_lvlshft[2],1'b1,out_lvlshft[4],out_lvlshft[3],out_lvlshft[6],
		   out_lvlshft[1],out_lvlshft[8],out_lvlshft[7],out_lvlshft[10],out_lvlshft[5],out_lvlshft[12],
		   out_lvlshft[11],out_lvlshft[14],out_lvlshft[9],out_lvlshft[16],out_lvlshft[15],out_lvlshft[18],
		   out_lvlshft[13],out_lvlshft[20],out_lvlshft[19]};
   assign io_i259 = out_lvlshft[9];   // new loopback output
   assign io_i283 = out_lvlshft[12];  // new loopback output
   assign io_i287 = out_lvlshft[16];  // new loopback output
   assign io_i291 = out_lvlshft[20];  // new loopback output
   assign io_i299 = out_lvlshft[13];  // new loopback output
   assign io_i307 = out_lvlshft[17];  // new loopback output
   assign io_i310 = out_lvlshft[17];  // was 310
   assign io_i306 = out_lvlshft[21];  // was 306
   assign io_i311 = out_lvlshft[21];  // new loopback output
   assign io_i298 = out_lvlshft[22];  // A2, needs wire to socket on Mezz
   assign io_i286 = out_lvlshft[23];  // B3, needs wire to socket on Mezz

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
      en_csr = 1'b1;
      csr1_r = 0; // these are the CLB Shift Registers, replace CRAMs
      csr2_r = 0; // these are the CLB Shift Registers, replace CRAMs
      csr3_r = 0; // these are the CLB Shift Registers, replace CRAMs
      csr4_r = 0; // these are the CLB Shift Registers, replace CRAMs
      csr5_r = 0; // these are the CLB Shift Registers, replace CRAMs
      csr6_r = 0; // these are the CLB Shift Registers, replace CRAMs
      for (i = 0; i < 24; i = i + 1) begin
	 if (i < 22) init_lvlshft[i] = 15 + 11401017*i;
	 else begin
//	    init_lvlshft[22] = 39'h0000000000;
//	    init_lvlshft[23] = 39'h0000000000;
	    init_lvlshft[22] = 39'h0a5b9c876e;
	    init_lvlshft[23] = 39'h123456789a;
	 end
      end
      gbe_rxcount = 16'h0000;
      good_rx_cmd = 1'b0;
      rx_timeout = 1'b0;
      free_tc_r = 1'b0;
      loop_command = 1'b0;
      pkt_id = 8'h0000;
      err_seq = 32'h00000001;
      force_err = 0;
      init_csrlfsr = 39'h0a5b9ce876;
   end


   IBUFGDS #(.DIFF_TERM("TRUE"),.IOSTANDARD("LVDS_25"))  lhc_clock(.I(lhc_ckp) , .IB(lhc_ckn) , .O(lhc_ck));
   IBUFGDS #(.DIFF_TERM("FALSE"),.IOSTANDARD("LVDS_25"))  clock125(.I(ck125p) , .IB(ck125n) , .O(ck125));
   IBUFDS_GTXE1  clock_gbe(.I(ck_gbep) , .IB(ck_gben) , .O(ckgbe), .ODIV2(), .CEB(zero));
   IBUFDS_GTXE1  clock_80(.I(ck80p) , .IB(ck80n) , .O(ck80), .ODIV2(), .CEB(1'b0));

   BUFG gbe_refclock(.I(ckgbe), .O(gbe_refck));
   bufg_div25clk clk5(ck125,clk,stopped,locked); // can we get 125 and 5 MHz outputs here?  Also 62.5 MHz?

   bufg_div4clk clk40(snap_clk2,ck40,stop40,lock40); // can we get 160 and 40 MHz outputs here?

// try out 0:3 = 62.5, 89.6, 84.0, 15.1
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


    genvar u;
    generate
       for (u=0; u<24; u=u+1) begin:prbs39gen
	  prbs39_test lvlshft_b(init_lvlshft[u], en_levelshift, in_lvlshft[u], out_lvlshft[u], count_lvlshft[u], err_wait&err_flag[u], !rxpll_lk, ck84);
       end
    endgenerate




    genvar v;
    generate
       for (v=12'hb00; v<=BRAM_LIM; v=v+1'b1) begin:bramgen
	  RAMB36E1 #(
        .DOA_REG(0),         // Optional output register (0 or 1)
        .DOB_REG(0),         // Optional output register (0 or 1)
	.EN_ECC_READ("TRUE"),
	.EN_ECC_WRITE("TRUE"),
        .RAM_MODE("SDP"),    // "SDP" or "TDP"
        .READ_WIDTH_A(72), // 0, 1, 2, 4, 9, 18, 36, or 72, 64??
//.READ_WIDTH_B(0), // 0, 1, 2, 4, 9, 18, or 36
//.WRITE_WIDTH_A(0), // 0, 1, 2, 4, 9, 18, or 36
        .WRITE_WIDTH_B(72), // 0, 1, 2, 4, 9, 18, 36, or 72, 64??
//  --WriteMode: Value on output upon a write ("WRITE_FIRST", "READ_FIRST", or "NO_CHANGE")--
	.WRITE_MODE_A("WRITE_FIRST"),
        .WRITE_MODE_B("WRITE_FIRST")
//        .WRITEMODE("WRITE_FIRST")  // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE"
    )
    block_ram_ecc
    (
        .ADDRARDADDR ({1'b1, tx_adr[10:2], 6'h3F}), // 16 bit RDADDR, but only 14:6 are used w/ECC.
        .DIADI   (data_iram[31:0]),  // DI low 32-bit
        .DIBDI   (data_iram[63:32]), // DI high 32-bit
        .DOADO   (data_oram[v][31:0]),   // DO low 32-bit
        .DOBDO   (data_oram[v][63:32]),  // DO high 32-bit
        .WEA     (4'h0),      // WEA, NA for SDP
        .ENARDEN (!cmd_f7f7 & gtx_ready),  // RDEN
        .REGCEAREGCE (1'b0),  // REGCE, NA if DO_REG=0
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),       // NA if SDP
        .RSTREGARSTREG(1'b0), // NA if SDP or DO_REG=0
        .RSTREGB(1'b0),       // NA if SDP or DO_REG=0
        .CLKARDCLK   (gbe_txclk2), //  RDCLK
// not valid if DO_REG=0:  REGCEAREGCE, REGCEB, RSTREGARSTREG, RSTREGB
// not used for SDP:  CASCADE*, REGCEB, RSTRAMB, RSTREGARSTREG, RSTREGB, WEA
        .ADDRBWRADDR ({1'b1, rx_adr[10:2], 6'h3F}), // 16 bit WRADDR, but only 14:6 are used w/ECC
        .WEBWE   (8'hFF),  // WE?  WEBWE(8 bits)
        .ENBWREN (cycle4 & (bk_adr==v)),   // WREN  Alfke: "WE off" is not sufficient to protect
        .REGCEB  (1'b0),       // REGCEB        Init data, So require stable clocks before EN is set,
        .CLKBWRCLK   (gbe_txclk2),  // WRCLK    and ensure safe setup time for ADR when EN is Enabled!
//        .RST     (reset) // err.mon: SBITERR DBITERR   
        .INJECTSBITERR (1'b0),
        .SBITERR (sbiterr_oram[v]),
        .INJECTDBITERR (1'b0),
        .DBITERR (dbiterr_oram[v])
    );
       end
    endgenerate
// JGhere: 

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
	   err_seq <= 32'h00000001;
	   err_flag <= 32'h00000000;
	   err_wait <= 0;
	   force_err <= 0;
	end
	else begin
	   tx_resetdone_r <= tx_resetdone;
	   rx_resetdone_r <= rx_resetdone;
	   free_count <= free_count + 1'b1;
	   if (free_count == 16'hffff) free_tc <= 1;  // ~76 Hz or .013 sec
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

   always @(posedge ck84) // everything that uses ck84, no Reset
     begin
 // Translator I/O registers:
	rpc_rx_r[28:5] <= rpc_rx[28:5];  // register inputs before checking logic, PUT IN I/O BLOCKS!
   // register outputs before sending to translator chip, PUT IN I/O BLOCKS!
	io_[361:340] <= io_i[361:340];
	io_259 <= io_i259;
	io_283 <= io_i283;
	io_287 <= io_i287;
	io_291 <= io_i291;
	io_299 <= io_i299;
	io_307 <= io_i307;
	io_310 <= io_i310;
	io_306 <= io_i306;
	io_311 <= io_i311;
	io_298 <= io_i298;
	io_286 <= io_i286;
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
	   rx_adr <= 0;
	   cycle4 <= 0;
	   cmd_f7f7 <= 0;
	   free_tc_r <= 1'b0;
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
	   en_csr <= 1'b1;
	   pkt_send <= 0; // 1st data word @counter == 12 --> 11 extra words, so subtract 22 bytes.
	   good_rx_cmd <= 1'b0;
	   rx_timeout <= 1'b0;
	end

	else begin // Not Reset case
	   if (sw[8] && !sw[7]) free_tc_r <= free_tc;
	   if (cmd_code == 16'h0000) loop_command <= free_tc_r;

	   rx_resetdone_r2 <= rx_resetdone_r;
	   rx_resetdone_r3 <= rx_resetdone_r2;
	   if (rx_resetdone_r3) comma_align <= 1; // maybe use time_r > 8'h10 ?

	   crc_rst <= (data_state == 2'h2)? 1'b1 : 1'b0;  // needs one cycle during state 3

	   if (cmd_code == 16'hf3f3 || loop_command) begin  // load counter2 to READ registers and send it out
	      pkt_lim <= 16'd2059;   // 16'd61 special test size 100 bytes.  later will be 4 KB
	   end
	   else if (cmd_code == 16'hf7f7) pkt_lim <= 16'd2069; // just over 4 KB for good Rx/Tx BRAM loading overlap
	   else pkt_lim <= 16'd36; // our usual 50-byte size is default; F3F3 is the special 4 KB count

	   byte_count <= 16'hffff&((pkt_lim*2) - 16'd22); // Need BYTES; subtract preamble bytes etc.

	   if (data_state[1]) begin  // end of sent data packet
	      cmd_code <= 16'h0000;
	      bk_adr <= 12'h000;
	      pkt_send <= 1'b0;
	      counter <= 0;
	      en_levelshift <= 1'b1;
	      en_csr <= 1'b1;
	      loop_command <= 1'b0;
	      if (data_state == 2'h2) pkt_id <= pkt_id + 1'b1;
	   end
	   else if (rx_timeout) begin
	      cmd_code <= 16'h0000;
	      bk_adr <= 12'h000;
	      counter <= 0;
	      pkt_send <= 1'b0;
	      en_levelshift <= 1'b1;
	      en_csr <= 1'b1;
	   end
	   else if (!sync_state[0] && pkt_send) begin
	      counter <= counter + 1'b1;  // verified that sync[0] ends OFF
	   end
 	   else if (!sync_state[0] && time_r[7] && (cmd_code > 16'h0000)) pkt_send <= 1'b1;
	   else if (!rxdv && loop_command && (cmd_code == 16'h0000) ) cmd_code <= 16'hf4f4;

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
   		 en_levelshift <= (cmd_code != 16'hf2f2);
		 en_csr <= (cmd_code != 16'hf5f5);
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
		 if (counter > 16'd12) ireg <= ireg + cmd_code[1] + (~counter[0]&cmd_code[0]); // 32b if CMD Odd, 16b if CMD Even
		   //  F5 => 0 + (cnt[0]&1), alternates ireg increments for 32-bit register readout.
		   //  F6 => 1 + (cnt[0]&0), continuous increments for 16-bit register readout.
		 if(cmd_code[1]^cmd_code[0]) begin  // generalized for f1f1 or f2f2 ( == !f3f3 && !f4f4) or f5 or f6...f9, fa, fd, fe
		    if (counter == 16'd12) begin
		       gbe_txdat <= cmd_code;    //  <-- first word returns the cmd_code
		    end
		    else if (cmd_code == 16'hf1f1 && ireg < 8) begin  //  "en_snapshift" signal for the Snap12s
		       if (counter[0]) gbe_txdat <= snap_errcount[ireg][15:0]; // 8 32-bit wide elements in this array
		       else gbe_txdat <= snap_errcount[ireg][31:16];
		    end
		    else if (cmd_code == 16'hf2f2 && ireg < 24) gbe_txdat <= count_lvlshft[ireg]; // 24 16-bit wide elements in this array
// added 3 csr vote_errcnts here, # of trials (ck80 counter w/en_csr & reset).  vote_a_err, vote_b_err, tri_vote_err
		    else if (cmd_code == 16'hf5f5 && ireg < 12) begin
		       if (counter[0]) gbe_txdat <= vote_errcnt[ireg][15:0]; // 12 32-bit wide elements in this array
		       else gbe_txdat <= vote_errcnt[ireg][31:16];
		    end
		    else gbe_txdat <= 16'h0000; // ireg out of range or CMD=f5, f6, f9, fa, fd, fe
		 end
//		 else if (cmd_code == 16'hf7f7) begin
//		    gbe_txdat <= 16'h50bc;  // <-- idle 28.5/16.2
//		    gbe_kout <= 2'h1;
//		 end
		 else if (cmd_code[2]) begin  // Function4 is active, or f7 or fc, ff
		    gbe_txdat <= {pkt_id[4:0],rx_adr[10:0]};  // send the sequential packet ID for each packet, plus data
		 end
		 else if (bk_adr <= BRAM_LIM) begin  // this is data_bram range.  for f3f3, f8f8, fbfb...
		    gbe_txdat <= data_bram_r[15:0];  // added a pipe register for data_bram, allows better timing?
		 end
		 else begin
		    gbe_txdat <= 16'h50bc;  // <-- idle 28.5/16.2
		    gbe_kout <= 2'h1;
		 end
	      end // if (counter >= 16'd12)
	   end // if (counter > 0 && counter <= GBE_LIM)
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
	      else if (data_state > 2'h0) data_state <= data_state + 2'h1;
	   end

	   l_rxdv <= rxdv;  // note, rxdv may stay True for over .5 sec after Reset!  Why...?
	   ll_kchar <= l_kchar;
	   l_kchar <= rxer[0]|rxer[1];
	   kchar_r <= rxer[1:0];
	   ll_gbe_rxdat <= l_gbe_rxdat;
	   l_gbe_rxdat <= gbe_rxdat;

	   if (time_r[7] && rxdv && (sw[7] || !sw[8]) ) begin // get command from first 1-2 data bytes of packet
	      if (gbe_rxcount == 16'd4407) begin  // tuned to handle weird .5 sec rxdv case after Reset.
		 ovfl_packet <= ovfl_packet + 1'b1; // tracks when a rx packet times-out, over jumbo GbE limit
		 rx_timeout <= 1'b1;  // only set for pretty big jumbo packet, over 8810 bytes
	      end
	      else gbe_rxcount <= gbe_rxcount + 1'b1;

	      if (rx_timeout) begin
		 tx_adr <= 0;
		 rx_adr <= 0;
		 cycle4 <= 0;
		 cmd_f7f7 <= 0;
	      end
	      else if ( (gbe_rxcount == 16'h0003) && (cmd_code == 16'h0000) && (sw[7] || !sw[8]) ) begin
		 if ( (gbe_rxdat&16'hf0f0)==16'hf0f0 && (gbe_rxdat&16'h0f0f)!=16'h0f0f ) begin
		    cmd_code <= gbe_rxdat; //   ^^^ make sure cmd code looks valid ^^^
		    good_rx_cmd <= 1'b1;
		    rx_adr <= 0;
		 end
	      end
	      else if (gbe_rxcount == 16'h0004 && (cmd_code == 16'hf3f3 || cmd_code == 16'hf7f7) ) begin
	        //     **> hex b00 = 2816 dec.   hex c00 = 3072 dec.
		 if (gbe_rxdat[11:8] == 4'hb || gbe_rxdat[11:8] == 4'hc) bk_adr <= gbe_rxdat[11:0]; // test gbe_rxdat for b & c
		 else bk_adr <= 0;
		 data_iram[15:0] <= gbe_rxdat[15:0];
		 rx_adr <= rx_adr + 1'b1;
		 cmd_f7f7 <= (cmd_code == 16'hf7f7);
	      end
	      else if (gbe_rxcount > 16'h0004 && (cmd_code == 16'hf7f7) && (bk_adr > 0) )begin
		 if (rx_adr < 11'h7ff) rx_adr <= rx_adr + 1'b1;
		 else cmd_f7f7 <= 0;
		 if (rx_adr[1:0] == 2'b00) data_iram[15:0] <= gbe_rxdat[15:0];
		 else if (rx_adr[1:0] == 2'b01) data_iram[31:16] <= gbe_rxdat[15:0];
		 else if (rx_adr[1:0] == 2'b10) data_iram[47:32] <= gbe_rxdat[15:0];
		 else data_iram[63:48] <= gbe_rxdat[15:0];
		 cycle4 <= (cmd_f7f7 & (rx_adr[1:0] == 2'b11));
	      end
	      else begin
		 good_rx_cmd <= 1'b0;
	      end // else: !if(gbe_rxcount == 3 or 4, or cmd=f7f7)
	   end  // if (time_r[7] & rxdv & correct switch state)

	   else  begin  // if (rxdv == 0) or time_r[7]==0
	      gbe_rxcount <= 16'h0000;
	      rx_timeout <= 1'b0;
	      cycle4 <= 0;
	      cmd_f7f7 <= 0;
	   end

	end // else: !if(gtx_reset)
     end

   always @(*)
     begin  // select what goes on the BRAM bus.  Later add the CLBram bus.
	if (bk_adr < 12'hb00 || bk_adr > BRAM_LIM) data_bram = 16'hba0f; // limited range of bk_adr space
	else if (cmd_code == 16'hf3f3) begin
	   case (tx_adr[1:0])
	     2'b00: data_bram = data_oram[bk_adr][15:0];
	     2'b01: data_bram = data_oram[bk_adr][31:16];
	     2'b10: data_bram = data_oram[bk_adr][47:32];
	     2'b11: data_bram = data_oram[bk_adr][63:48];
	   endcase // case (tx_adr[1:0])
//	   data_bram = data_oram[bk_adr];
	end
	else data_bram = 16'h0000;

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


   prbs39 csr_lfsr(init_csrlfsr, en_csr, lfsr_bit, reset, ck80);
   always @(posedge ck80 or posedge reset) // things that use ck80, w/Reset
     begin
	if (reset) begin
	   csr1_r[CSR_MAX:CSR_MAX-15] <= {1'b0,sw[7],14'h3770};  // init the CLB Shift Register register.  sw8 is always ZERO!
	   csr1_r[(CSR_MAX-16):0] <= 0;
	   csr2_r[CSR_MAX:(CSR_MAX-15)] <= {1'b0,sw[8]|sw[7],14'h3770};
	   csr2_r[(CSR_MAX-16):0] <= 0;
	   csr3_r[CSR_MAX:(CSR_MAX-15)] <= {sw[8],sw[8]^sw[7],14'h3770};
	   csr3_r[(CSR_MAX-16):0] <= 0;

	   csr4_r[CSR_MAX:(CSR_MAX-15)] <= {sw[8],sw[8]|sw[7],14'h3770};  // init the CLB Shift Register register, sw[8] is always ZERO!
	   csr4_r[(CSR_MAX-16):0] <= 0;
	   csr5_r[CSR_MAX:(CSR_MAX-15)] <= {sw[8]&sw[7],sw[8]^sw[7],14'h3770};
	   csr5_r[(CSR_MAX-16):0] <= 0;
	   csr6_r[CSR_MAX:(CSR_MAX-15)] <= {1'b0,sw[8]^sw[7],14'h3770};
	   csr6_r[(CSR_MAX-16):0] <= 0;
	   vote_a <= 0;
	   vote_b <= 0;
	   error_a <= 0;
	   error_b <= 0;
	   tri_vote_fail <= 0;
	   csr80_count <= 0;
//  init 3 groups of identical parallel counters; sw8 is always ZERO!
	   tri_vote0_errcnt <= 0;
	   vote0a_errcnt <= 32'hFFFFFFFF * (sw[8]);
	   vote0b_errcnt <= 32'h90000003 * (sw[7]&sw[8]);
	   tri_vote1_errcnt <= 32'hFFFFFFFF * (sw[8]);
	   vote1a_errcnt <= 32'h90000003 * (sw[8]);
	   vote1b_errcnt <= 32'hC0000009 * (sw[8]);
	   tri_vote2_errcnt <= 32'hC0000009 * (sw[8]);
	   vote2a_errcnt <= 32'hC0000009 * (sw[7]&sw[8]);
	   vote2b_errcnt <= 32'hFFFFFFFF * (sw[7]&sw[8]);
	end

	else begin
	   vote_a  <= ((csr1_r[0]&csr2_r[0]) | (csr1_r[0]&csr3_r[0])) | (csr3_r[0]&csr2_r[0]);
	   error_a <= ((csr1_r[0]^csr2_r[0]) | (csr1_r[0]^csr3_r[0])) | (csr3_r[0]^csr2_r[0]);
	   vote_b  <= ((csr4_r[0]&csr5_r[0]) | (csr4_r[0]&csr6_r[0])) | (csr6_r[0]&csr5_r[0]);
	   error_b <= ((csr4_r[0]^csr5_r[0]) | (csr4_r[0]^csr6_r[0])) | (csr6_r[0]^csr5_r[0]);
	   if (en_csr) begin
	      csr80_count <= csr80_count + 1'b1;
	      csr1_r <= {lfsr_bit,csr1_r[CSR_MAX:1]};
	      csr2_r <= {lfsr_bit,csr2_r[CSR_MAX:1]};
	      csr3_r <= {lfsr_bit,csr3_r[CSR_MAX:1]};
	      csr4_r <= {lfsr_bit,csr4_r[CSR_MAX:1]};
	      csr5_r <= {lfsr_bit,csr5_r[CSR_MAX:1]};
	      csr6_r <= {lfsr_bit,csr6_r[CSR_MAX:1]};
	      if (vote_a^vote_b) begin  // this means the 2 triple-voters disagree!  Mitigation failure, should be rare.
		 tri_vote_fail <= 1'b1; //   ...should be rare...
		 tri_vote0_errcnt <= tri_vote0_errcnt + 1'b1;
		 tri_vote1_errcnt <= tri_vote1_errcnt + 1'b1;
		 tri_vote2_errcnt <= tri_vote2_errcnt + 1'b1;
	      end
	      else tri_vote_fail <= 1'b0;
	      if (error_a) begin  // this means a single CLB bit error, very common.
		 vote0a_errcnt <= vote0a_errcnt + 1'b1;
		 vote1a_errcnt <= vote1a_errcnt + 1'b1;
		 vote2a_errcnt <= vote2a_errcnt + 1'b1;
	      end
	      if (error_b) begin  // this means a single CLB bit error, very common.
		 vote0b_errcnt <= vote0b_errcnt + 1'b1;
		 vote1b_errcnt <= vote1b_errcnt + 1'b1;
		 vote2b_errcnt <= vote2b_errcnt + 1'b1;
	      end
	   end // if (en_csr)
	   else tri_vote_fail <= 1'b0;

	end // else: !if(reset)
     end // always @ (posedge ck80 or posedge reset)

   assign vote_errcnt[0][31:0] = vote0a_errcnt[31:0];
   assign vote_errcnt[1][31:0] = vote1a_errcnt[31:0];
   assign vote_errcnt[2][31:0] = vote2a_errcnt[31:0];
   assign vote_errcnt[3][31:0] = vote0b_errcnt[31:0];
   assign vote_errcnt[4][31:0] = vote1b_errcnt[31:0];
   assign vote_errcnt[5][31:0] = vote2b_errcnt[31:0];
   assign vote_errcnt[6][31:0] = tri_vote0_errcnt[31:0];
   assign vote_errcnt[7][31:0] = tri_vote1_errcnt[31:0];
   assign vote_errcnt[8][31:0] = tri_vote2_errcnt[31:0];
   assign vote_errcnt[9][31:0] = csr80_count[31:0];
   assign vote_errcnt[10][31:0] = csr80_count[63:32];
   assign vote_errcnt[11][31:0] = csr80_count[31:0];


   always @(posedge snap_clk2) // everything that uses snap USR clock, w/o Reset
     begin
	time_r_snap <= time_count;
	time_snap <= time_r_snap;  // transfer slow clk time counter to snap USR clock domain
     end

   always @(posedge snap_clk2 or posedge reset or posedge snap_wait) // things that use snap USR clock, w/Reset
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
           test_led[7]   = snap_wait;
	      test_led[1] = !gbe_fok; // 
	      test_led[2] = !all_ready; // 
	      test_led[3] = !time_r[7]; // 
	      test_led[4] = !lock40; // 
	      test_led[5] = ck160_locklost; // 
	      test_led[6] = !locked; // 
	      test_led[7] = ck125_locklost; //
	      test_led[8] = gtx_reset; //
 */
	      test_led[8:1]   = rxdvr_snapr[7:0];
	      test_led[9] = snap_wait; //
	      test_led[10]  = 1'b0;     // sw8 Low
/*
 	      test_led[8:1] = l_gbe_rxdat[7:0]; // counter[7:0];
	      test_led[9]   = kchar_r[0]; // gbe_fok;
	      test_led[10]  = l_rxdv;  // all_ready;  
 */
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
	   test_led[10]  = rxdvr_snapr[2];  // counter_send
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
	   test_led[10]  = rxdvr_snapr[2];  // counter_send
	   led_low = 8'b11111111;  // Prom D[7:0], go to TMB LEDs
	end
     end


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


// `define DLY #1

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
//`define DLY #1

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
	output reg [15:0]  count, // this is a 16-bit counter for # of errors
        input  force_error,  // this needs to trigger a  _single_  bit-flip
	input        rst,
	input        clk);  // assume 10-20 MHz operation...NOW 40 Mhz!  later 80?

        reg 	    en_r, in_r, out_r, out_rr, out_3r, ferr_r, ferr_rr, ferr_done;

   prbs39 randombit(init_dat, en, out, rst, clk);

	always @(posedge clk, posedge rst) begin
	   if(rst) begin
	      en_r <= 0;
	      count  <= 0;
	      ferr_r <= 0;
	      ferr_rr <= 0;	      
	      in_r <= 0;
	   end

	   else begin
	      in_r <= in;
	      out_3r <= out_rr;
	      out_rr <= out_r^ferr_rr;
	      out_r <= out;
	      if (init_dat != 39'b0) en_r <= en;
	      if (en && en_r && in_r != out_3r) count <= count + 1'b1;

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
	      out <= lfsr[38];
	   end
	   else begin
	      out <= lfsr[38];
	      if (en) lfsr[39:1] <= {lfsr[38:1],lfsr[39]^lfsr[4]};
	      else lfsr  <= lfsr;
	   end
	end // always
endmodule
