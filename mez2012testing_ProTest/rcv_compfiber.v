
module rcv_compfiber(
    input  rxn, rxp,
    input  ref_clk, fabric_clk, ref_locked,
    input  reset, gtx_reset, rst_errcount, startup_ready, en_prbstest, en_fiforead,
    input  rx_pol_swap,
    output reg rx_start_r, rx_fc_r, rx_valid_r, rx_match_r, synced_snapr_r,   // bring to fabric domain!
    output reg compfifo_dav_r, compfifo_overflow_r, err_r,   // bring to fabric domain!
    output reg [15:0] err_count_r,  // bring to fabric domain!
    output reg [47:0] comp_dout,  // bring to fabric domain!
    output [47:0] compfifo_dout,
    output rx_pll_locked,
    output k_sync,
    output k_synclost,
    output reg [3:1] nzdat
     );
   
  
// rx module reqs, IN:  rxp/n[i], reset, gtx_reset, refclk=ck160, lhc_clk, ck160_locked,
//                      time_snap[7], rst_errcount, "sw[8]"=en_PRBStest?
// rx module reqs, OUT: comp_dat_out[i], err[i], err_count[i], rx_strt[i], rx_fc[i], rx_valid[i], rx_match[i]?

//  set itriad=9  if ( !sw[8] && itriad==0 && |comp_dat_r > 0 )  can we pick a few specific bits?  TRIAD FORMAT!
//      -- comp_dat is 48 bits, takes 3-step logic....pipeline some before _r and finish before _2r
//  48-bit wide, 5-deep pipeline; word0 is EN.  comp_dat_r, comp_dat_2r, .... comp_dat_5r
//      -- goes to 48-bit FIFO with  PUSH=word0&itriad>0; runs on rx_clk (160 MHz)
//      -- PUSH controls itriad-1 countdown
//  FIFO output is 16-bits wide(?) for GbE tx_dat...NOT ALLOWED!   runs on clk 
//      -- !empty triggers GbE data dump of three triads (3 * 48 bits each...54 bytes total) preceded by some empty words

   wire    rx_start, rx_fc, rx_valid, rx_match, synced_snapr;   // bring to fabric domain!
   reg     err;    // bring to fabric domain!
   reg  [15:0] err_count;  // bring to fabric domain!

   wire    rx_clk;
   wire [3:0]  word;
   wire [3:1]  nz;
   wire [47:0] comp_dat;
   reg  [47:0] comp_dat_r, comp_dat_2r, comp_dat_3r, comp_dat_4r, comp_dat_5r;
   reg  [3:0]  itriad;
   reg 	   save_triad, rst_errcount_r;
   wire    push_fifo, no_compfifo_dav, compfifo_overflow;

   comp_fiber_in dcfeb_in (
	.CMP_RX_VIO_CNTRL (), // empty or Delete from code
	.CMP_RX_LA_CNTRL (),  // empty or Delete from code
	.RST (gtx_reset),     // gtx_reset = PBrst | !RxSyncDone.  What does RST do inside?
        .CMP_SIGDET (),    //  N/A
        .CMP_RX_N (rxn),      // pick a fiber, set LOC constraint
        .CMP_RX_P (rxp),      // pick a fiber
	.CMP_TDIS (),      //  N/A
	.CMP_TX_N (),      // empty
	.CMP_TX_P (),      // empty
        .CMP_RX_REFCLK (ref_clk), // QPLL 160 via GTX Clock
        .RX_POLARITY_SWAP  (rx_pol_swap), // JGnew
        .CMP_SD (),        // from IBUF, useless output. N/A
	.CMP_RX_CLK160 (rx_clk), // Rx recovered clock out.  Use for internal Logic Fabric clock
			           //   Needed to sync all 7 CFEBs with Fabric clock!
	.STRT_MTCH (rx_start), // gets set when the Start Pattern is present, N/A for me.  To TP for debug only.  --sw8,7
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
        .LTNCY_TRIG (rx_fc),  // flags when RX sees "FC" for latency msm't.  Send raw to TP or LED
        .RX_PLL_LOCKED (rx_pll_locked),
        .RX_SYNC_DONE (synced_snapr), // inverse of this goes into GTX Reset
        .SYNCWORD (k_sync),
        .SYNCLOST (k_synclost)
	);

   always @(posedge fabric_clk) // fabric clock time domain transition
     begin
	comp_dout <= comp_dat_r; // or just use comp_dat?  need word[0] too?
	rx_start_r <= rx_start;
	rx_fc_r <= rx_fc;
	rx_valid_r <= rx_valid;
	rx_match_r <= rx_match;
	synced_snapr_r <= synced_snapr;
	compfifo_dav_r <= !no_compfifo_dav;
	compfifo_overflow_r <= compfifo_overflow;
	err_r <= err;
	err_count_r <= err_count;
	nzdat <= nz; // JG 3/19/2013
     end


   assign snap_wait = !(synced_snapr & ref_locked);  // allows pattern checks when RX is ready
   //assign snap_wait = !(synced_snapr );  // allows pattern checks when RX is ready
   assign push_fifo = ( word[0] && save_triad );
   always @(posedge rx_clk or posedge reset or posedge snap_wait) // things that use 160 MHz snap rx USR clock, w/Reset
     begin
	if (reset | snap_wait) begin
	   comp_dat_r  <= 0;
	   comp_dat_2r <= 0;
	   comp_dat_3r <= 0;
	   comp_dat_4r <= 0;
	   comp_dat_5r <= 0;
	   save_triad  <= 0;
	   itriad  <= 0;
	   rst_errcount_r <= 1'b1;
	end
	else begin // Not Reset case
	   rst_errcount_r <= rst_errcount;
//	   if (!en_prbstest && !save_triad && |nz && word[0]) begin  // 6-input logic
	   if (!save_triad && |nz && word[0]) begin  // JG 3/19/2013, this new code assumes we never get random data!
	      itriad <= 9;  // so we will save 9 words (3 triads)
	      save_triad <= 1;
	   end
	   else if (push_fifo) begin
	      itriad <= itriad - 1'b1;
	      if(itriad == 4'h1)save_triad <= 0;
	   end
	   if (word[0]) begin
	      comp_dat_r   <= comp_dat;
	      comp_dat_2r  <= comp_dat_r;
	      comp_dat_3r  <= comp_dat_2r;
	      comp_dat_4r  <= comp_dat_3r;
	   end

	   if (rst_errcount_r) begin
	      err_count <= 0;
	      err <= 0;
	   end
	   else if (en_prbstest & word[0] & !snap_wait & startup_ready) begin  // wait 3000 clocks after Reset
	      if ( !rx_match & rx_valid ) begin
		 err <= 1'b1;  // take this to testLEDs for monitoring on scope
		 err_count <= err_count + 1'b1;  // this goes to Results Reg for software monitoring
	      end
	      else err <= 0;
	   end
        end

     end

   fifo_48b_256_async comp_fifo (
	.rst    (reset),
	.wr_clk (rx_clk),
	.rd_clk (fabric_clk),
	.din    (comp_dat_4r),
	.wr_en  (push_fifo),
	.rd_en  (en_fiforead),
	.dout   (compfifo_dout),
	.full   (compfifo_overflow),
	.empty  (no_compfifo_dav)
	);

endmodule // rcv_compfiber
