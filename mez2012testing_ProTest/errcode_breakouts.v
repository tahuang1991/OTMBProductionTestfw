// ---- ERROR CODE SUMMARY RESULTS ----
   wire [27:0] err_dmbloop;
   wire [46:0] err_ratloop;
   reg [7:0]   slowloop_err, slowloop_stat;

	      err_dmbloop[27:0] = dmbloop3stat[3:0] + dmbloop2stat[11:0] + dmbloop1stat[11:0];
	          //             ^^^^ preceded by {8'h00}

	      err_ratloop[46:0] = ratloop4stat[10:0] + ratloop3stat[11:7] + (!en_fibertests)ratloop3stat[6:5] + ratloop3stat[4] + (!en_fibertests)ratloop3stat[3] + ratloop3stat[2:0] + ratloop2stat[11:0] + ratloop1stat[11:5] + (en_fibertests)ratloop1stat[4] + ratloop1stat[3:0];
 // bit 29 gets killed in en_fibertests due to SMB control issues, bits 27 & 30 due to step4 conflicts
 // bit 4 gets killed in !en_fibertests due to SMB control issues
 //             ^^^^ ratloop4stat is preceded by {1'b0}
	      
	      slowloop_err[7:0] = (!en_fibertests==HzLoop)slowloop_stat[7:5] + (en_fibertests==MHzLoop)slowloop_stat[4:0];
	          //             ^^^^ preceded by {~vstat[3],1'b0,~vstat[1:0]}



// ---- Informative sections of the code ----
    input [50:0]      _ccb_rx,  // add 42-47
// CCB can Write DMB_Reserved_Out[4:0] (to all TMBs & DMBs) on base+2a (CSRB6, bits 14:10).  ccb_rx[47-43]
// CCB can Read TMB_Reserved_In[4:0] from TMB on base+34 (CSRB11, bits 7:3).  ccb_tx[26-22]
//   --> For this test, TMB will return the value we set on DMB_Reserved_Out, back to the CCB via TMB_Reserved_In.
   assign _ccb_tx[26:22] = _ccb_rx[47:43];  // returns DMB_Reserved_Out[4:0] from CCB back to the CCB on TMB_Reserved_In[4:0]
// These tx bits are outputs for TMB_L1A_Release/Request. Create pulses using CCB_Reserved[3:2] from CCB:
// CCB can Write CCB_Reserved[3:2] (to all TMBs) on base+2a (CSRB6, bits 1:0).
   assign _ccb_tx[21:20] = tmb_l1a_relreq[1:0];  // firmware sets  ccb_rx[25-24] -> tmb_l1a_relreq -> ccb_tx[21-20]
  //   when TMB L1A Release & Request go out, CCB may send an L1A!  May be disabled on CCB.
   assign _ccb_tx[19] = alct_cfg_out;  // serial data signal path back to CCB
   assign _ccb_tx[18] = tmb_cfg_out;   // serial data signal path back to CCB
   assign _ccb_tx[17:9] = pulse_count[8:0]; // have to look at CCB LED plug for these 9 bits
// _ccb_tx[17:9] = alct_status bits, only tested via CCB FP (our LED plug)
// _ccb_tx[8:0]  = clct_status bits, only tested via CCB FP (cable)



   ~_ccb_rx[50:48];  // undefined?   set at CCB & checked in results reg 
// CCB can Write DMB_Reserved_Out[4:0] (to all TMBs & DMBs) on base+2a (CSRB6, bits 14:10).  ccb_rx[47-43]
// CCB can Read TMB_Reserved_In[4:0] from TMB on base+34 (CSRB11, bits 7:3).  ccb_tx[26-22]
//   --> For this test, TMB will return the value we set on DMB_Reserved_Out, back to the CCB via TMB_Reserved_In.
   assign _ccb_tx[26:22] = _ccb_rx[47:43]; // ccb_DMB_Reserved_Out[4:0] -> ccb_TMB_Reserved_In[4:0]
   assign ccb_unused[5]   = ~_ccb_rx[42];  // set at CCB & checked in results reg 
// CCB can Write TMB_Reserved_Out[2:0] (to all TMBs) on base+2a (CSRB6, bits 9:7).  ccb_rx[38-36]
   assign tmb_res_out[2:0] = ~_ccb_rx[38:36];  // JGhere:  where does tmb_res_out go?  just results reg?
//   assign mpc_in1 = !_ccb_rx[35];  unused, no test, skip
//   assign mpc_in0 = !_ccb_rx[34];  unused, no test, skip

// CCB can Write TMB_Reserved0 (to all TMBs) on base+2a (CSRB6, bit2).   ccb_rx28
// CCB can Write CCB_Reserved[3:2] (to all TMBs) on base+2a (CSRB6, bits 1:0).  ccb_rx[25-24]
   assign ccb_unused[4:0] = ~_ccb_rx[28:24]; // 28-27 are tested as Pulses, 26 is TMB HardReset! 25-24 are tmb_l1a_relreq bits
// ccb_reserved(1:0) are for QPLL & TTC status... just try to read them back via CCB.  ccb_rx[23-22]
   assign ccb_qpll_lck = !_ccb_rx[23];  // JGhere, check: ccb_reserved[1]?
   assign ccb_ttcrx_rdy = !_ccb_rx[22]; // JGhere, check: ccb_reserved[0]?
   assign ccb_data[7:0] = ~_ccb_rx[21:14];
   assign ccb_datstrb = !_ccb_rx[13]; // loads data register
   assign ccb_cmdstrb = !_ccb_rx[10]; // loads command register
   assign ccb_cmd[7:0] = { (~_ccb_rx[7:2]), (!_ccb_rx[8]), (!_ccb_rx[9])};
   assign ccb_l1reset = !_ccb_rx[1];  // clears pulse_count & all counters
   wire  ccb_cken; // global clock signal to use for ccb_rx[0] "clock"
// weird ones returned in results register:   results_r <= {!_ccb_rx[42],~_ccb_rx[50:48],tmb_res_out[2:0],!_ccb_rx[28],~_ccb_rx[25:22],last_cmd[7:0]}; default has a bunch of CCB signals to check



// ccb_rx1=L1rst; 7-2=CMD, 8,9=ev/bcntrst; 10=cmdstr, 11=bc0, 12=L1A, 13=datstr, 21-14=DAT
// - pulse counter (pulse is CE); send pulses & read back count via status LEDs (11 ccb_rx)
//      > 1 reset signal (L1Reset=ccb_reserved4, to clear) and triggered by 11 different pulses:
//          BC0, L1A, tmb_soft_reset=tmb_reserved1, clct/alct_external_trigger, dmb_cfeb_calibrate[2:0], 
// 	    adb_pulse sync/async, alct_hard_reset,
//         - verify that all single CMD & DATA bits work reliably (avoid CMD/DATA = 0C,0D,0E,0F; 10,11,12,13; 40,41,42,43)
//         - need to check LEDs at least one time too, to verify status bus works
// unless noted otherwise, these pulses are only 25ns long and count just one time:
//   assign bc0 = !_ccb_rx[11]; // BC0 control, CCB base+52
//   assign rst_errcnt  = !_ccb_rx[29]; // TMB_SoftRst control, CCB base+6c or 6a
// THESE CONTROL THE PULSE_COUNT TEST:
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
      //  ^^^ right now we access this with CCB base+2a (CSRB6, write a 1 then a 0 to bit2): we get a random count each time



////////////////////////////////////////////////////////////////////////////////////////////////////
// JGhere, CCB test summary:
//    checks done for ccb_rx  50-36, 33-29, 28-24, 21-0   --skip 35,34,23,22  --50-48,42,28 are results_reg only
//    checks done for ccb_tx  26-18, (17-0 all go to CCB FP only)
// ccb_rx tested via ccb_tx:  ccb_rx 25:24 --> ccb_tx21:20 // 
//                            ccb_rx 47:43 --> ccb_tx26:22 // 
// pulsecount tests 11 ccb_rx 41-39,33-29,27,12-11.... maybe also 28?
//   ** not sure about  ccb_rx[28] === tmb_reserved0, Not Really a Pulse!  read via results reg.
// these have specific CCB test cycles in software, ccb_rx 21-13,10-1
   assign ccb_data[7:0] = ~_ccb_rx[21:14];
   assign ccb_datstrb = !_ccb_rx[13]; // loads data register
   assign ccb_cmdstrb = !_ccb_rx[10]; // loads command register
   assign ccb_cmd[7:0] = { (~_ccb_rx[7:2]), (!_ccb_rx[8]), (!_ccb_rx[9])};
   assign ccb_l1reset = !_ccb_rx[1];  // clears pulse_count & all counters
//   _ccb_rx 50-48,42...   set by S/W at CCB & checked in results reg 
// ccb_rx0 should be a clock... count it to see it toggle, and send some bits via CCB.
   wire  ccb_cken; // global clock signal to use for ccb_rx[0] "clock", checked in results reg C0
// weird ones returned in results register:   results_r <= {!_ccb_rx[42],~_ccb_rx[50:48],tmb_res_out[2:0],!_ccb_rx[28],~_ccb_rx[25:22],last_cmd[7:0]}; default has a bunch of CCB signals to check, select an unused CMD to see this!  C1-7, D6-7, E6-F, F9-B
   assign tmb_res_out[2:0] = ~_ccb_rx[38:36];  // CCB write TMB_Reserved_Out[2:0] (to all TMBs) on base+2a (CSRB6, bits 9:7) == ccb_rx[38-36], goes to results reg
//   assign mpc_in1 = !_ccb_rx[35];  unused, no test, skip
//   assign mpc_in0 = !_ccb_rx[34];  unused, no test, skip
// CCB can Write TMB_Reserved0 (to all TMBs) on base+2a (CSRB6, bit2).   ccb_rx28
// CCB can Write CCB_Reserved[3:2] (to all TMBs) on base+2a (CSRB6, bits 1:0).  ccb_rx[25-24]
   assign ccb_unused[4:0] = ~_ccb_rx[28:24]; // 28-27 are tested as Pulses, 26 is TMB HardReset! 25-24 are tmb_l1a_relreq bits
// ccb_reserved(1:0) are for QPLL & TTC status... just try to read them back via CCB.  ccb_rx[23-22]
   assign ccb_qpll_lck = !_ccb_rx[23];  // JGhere, check: ccb_reserved[1]?
   assign ccb_ttcrx_rdy = !_ccb_rx[22]; // JGhere, check: ccb_reserved[0]?

// CCB can Write DMB_Reserved_Out[4:0] (to all TMBs & DMBs) on base+2a (CSRB6, bits 14:10).  ccb_rx[47-43]
// CCB can Read TMB_Reserved_In[4:0] from TMB on base+34 (CSRB11, bits 7:3).  ccb_tx[26-22]
//   --> For this test, TMB will return the value we set on DMB_Reserved_Out, back to the CCB via TMB_Reserved_In.
   assign _ccb_tx[26:22] = _ccb_rx[47:43];  // returns DMB_Reserved_Out[4:0] from CCB back to the CCB on TMB_Reserved_In[4:0]
// These tx bits are outputs for TMB_L1A_Release/Request. Create pulses using CCB_Reserved[3:2] from CCB:
// CCB can Write CCB_Reserved[3:2] (to all TMBs) on base+2a (CSRB6, bits 1:0).
   assign _ccb_tx[21:20] = tmb_l1a_relreq[1:0]; //  ==  ccb_rx[25-24] -> tmb_l1a_relreq -> ccb_tx[21-20]
  //   when TMB L1A Release & Request go out, CCB may send an L1A!  May be disabled on CCB.
   assign _ccb_tx[19] = alct_cfg_out;  // serial data signal path back to CCB
   assign _ccb_tx[18] = tmb_cfg_out;   // serial data signal path back to CCB
   assign _ccb_tx[17:9] = pulse_count[8:0]; // have to look at CCB LED plug for these 9 bits
// _ccb_tx[17:9] = alct_status bits, only tested via CCB FP (our LED plug)
// _ccb_tx[8:0]  = clct_status bits, only tested via CCB FP (cable)


////////////////////////////////////////////////////////////////////////////////////////////////////



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



//   DMB Loop:  27 pairs + one clock.    lout_dmbloop goes out, then in_dmbloop comes back from DMB-Loopback
// dmbfifo_step1ck -> dmb_rx0   This is CCB clock, 40 MHz. But NOT a CLK pin!  Div2 via Flop and send to bufg?
//    Note: step4 selects STEP mode for 3:0 = cfeb, rpc, dmb, alct.  step4 Low makes all free-running clocks.
// 0 dmb_tx33 -> dmb_rx1
//       dmb_tx33 = io_447 = pin b17 on XP2
//       dmb_rx1 = 
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
   assign rawin_ratloop[24] = alct_rx[5];  //
   assign rawin_ratloop[25] = alct_rx[7];  // alct_txoe
   assign rawin_ratloop[26] = alct_rx[11]; // alct_clock_en
   assign rawin_ratloop[27] = alct_rx[13]; // step0
   assign rawin_ratloop[28] = alct_rx[6];  // alct_rxoe
   assign rawin_ratloop[29] = alct_rx[3];  //
   assign rawin_ratloop[30] = rpc_i2rx[28];// step2
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
   assign rawin_ratloop[41] = alct_rx[28];  //
   assign rawin_ratloop[42] = rpc_i2rx[32];
   assign rawin_ratloop[43] = rpc_i2rx[33];
   assign rawin_ratloop[44] = rpc_i2rx[34];
   assign rawin_ratloop[45] = alct_rx[27];
   assign rawin_ratloop[46] = rpc_i2rx[26];  // io_394 == pin A3 on XP4, TP is ALCT_RX26 etc

   assign jtag_usr[3] = lout_slowloop[2]; // vstat[2] = slowloop2 is SLOW!!  Only ~3 HZ max from power-sense chip
   assign jtag_usr[1] = lout_slowloop[0];
   assign jtag_usr[2] = lout_slowloop[1];
   always @(*)
     begin
	if (!en_fibertests) selusr = 4'b1101;    // rpc_jtag active, 3 bits only, includes ~1 Hz Vstat2 test
	else selusr = {2'b00,lout_slowloop[4],lout_slowloop[3]}; // alct_jtag active, 5 bits under test
     end




	   dmbloop1_stat <= dmbloop1_stat | err_dmbloop[11:0];
	   dmbloop2_stat <= dmbloop2_stat | err_dmbloop[23:12];
	   dmbloop3_stat <= dmbloop3_stat | err_dmbloop[27:24];
	   if ( |err_dmbloop ) dmbloop_errcnt <= ((dmbloop_errcnt[11:0] + 1'b1) | (dmbloop_errcnt[11]<<11));



	   ratloop4_stat[10:0] <= ratloop4_stat[10:0] | err_ratloop[46:36];
	   ratloop3_stat[11:0] <= ratloop3_stat[11:0] | {err_ratloop[35:31], (!en_fibertests_r & err_ratloop[30]), (!en_fibertests_r & err_ratloop[29]), err_ratloop[28], (!en_fibertests_r & err_ratloop[27]), err_ratloop[26:24]}; // bits 27 & 30 get killed by step4/en_fibetests, similar for 29 due to SMB control issues
	   ratloop2_stat[11:0] <= ratloop2_stat[11:0] | err_ratloop[23:12];
	   ratloop1_stat[11:0] <= ratloop1_stat[11:0] | {err_ratloop[11:5], (en_fibertests_r & err_ratloop[4]), err_ratloop[3:0]}; // bit

	   if (en_fibertests) begin  // tests are disabled for step 0 & 2 (ratloop 27 & 30) so ignore
	      if ( (|err_ratloop[26:0]) || ( err_ratloop[28]) || ( |err_ratloop[46:31]) ) ratloop_errcnt <= ((ratloop_errcnt[11:0] + 1'b1) | (ratloop_errcnt[11]<<11)); // bit 29 gets killed in en_fibertests due to SMB control issues, bits 27 & 30 due to step4 conflicts
	   end
	   else begin  // note that step tests are ON when fibertests are Off.
	      fiber_stat[0] <= 1'b0; // set Hi if en_fibertests, otherwise Lo
	      if ( (|err_ratloop[46:5]) || ( |err_ratloop[3:0]) ) ratloop_errcnt <= ((ratloop_errcnt[11:0] + 1'b1) | (ratloop_errcnt[11]<<11)); // bit 4 gets killed in !en_fibertests due to SMB control issues



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


// JG, CC cmd: first pass
		 if (last_cmd[7:0]==8'hCD) results_r <= {pulse_count[11:0],last_cmd[7:0]};    // count of CCB pulsed signals
		 else if(last_cmd[7:0]==8'hCE) results_r <= {ck160_locklost,!locked160,qpll_locklost,!qpll_lock ,ccb_data_r[7:0],last_cmd[7:0]}; // CCB data bus content
		 else if(last_cmd[7:0]==8'hCF) results_r <= {pulses_fired[11:0],last_cmd[7:0]};   // observed CCB test pulses
		 else if(last_cmd[7:0]==8'hC0) results_r <= {ccbrxzero_count[11:0],last_cmd[7:0]};   // check ccb_rx0 clocking
		 else  results_r <= {!_ccb_rx[42],~_ccb_rx[50:48],tmb_res_out[2:0],!_ccb_rx[28],~_ccb_rx[25:22],last_cmd[7:0]};   // default is send last cmd along with a bunch of CCB signals to check, select an unused CMD to see this!  C1-7, D6-7, E6-F, F9-B
// JG, CC cmd: second pass and beyond
		 if ((last_cmd[7:0]&8'hf8)==8'h18) results_r <= {en_fibertests, en_cabletests, en_loopbacks,1'b0, ccb_data_r[7:0],last_cmd[7:0]}; // show "en_test" status and also CCB data bus content ---> eventually this reads "testLED switch control" setting

		 if(last_cmd[7:0]==8'hD0) results_r <= {dmbloop_errcnt[11:0],last_cmd[7:0]}; // total errors in 27 DMB loops, bit 11 = rollover
		 if(last_cmd[7:0]==8'hD1) results_r <= {dmbloop1_stat[11:0],last_cmd[7:0]};  // DMB loop1 signals with error
		 if(last_cmd[7:0]==8'hD2) results_r <= {dmbloop2_stat[11:0],last_cmd[7:0]};  // DMB loop2 signals with error
		 if(last_cmd[7:0]==8'hD3) results_r <= {dmbloop3_stat[11:0],last_cmd[7:0]};  // DMB loop3 signals with error
		    //                                   ^^^^ 28 bits here, only last 3:0 active for DMB loop tests
//		 if(last_cmd[7:0]==8'hD4) results_r <= {dmbloop4_stat[11:0],last_cmd[7:0]};  // DMB unused signals
 CCB?		 if(last_cmd[7:0]==8'hD5) results_r <= {tmb_ck1reg[3],lhc_ckreg[3],6'h1E,hard_count[3:0],last_cmd[7:0]};  // DMB unused. Hijack for HardReset check!  Should give "DEX,LastCmd" where X is the # of pulses sent since Hard Reset

		 if(last_cmd[7:0]==8'hD8) results_r <= {ratloop_errcnt[11:0],last_cmd[7:0]}; // total errors in 47 fast RPC loops, bit 11=rollover
		 if(last_cmd[7:0]==8'hD9) results_r <= {ratloop1_stat[11:0],last_cmd[7:0]};  // RPC loop1 signals with error
		 if(last_cmd[7:0]==8'hDA) results_r <= {ratloop2_stat[11:0],last_cmd[7:0]};  // RPC loop2 signals with error
		 if(last_cmd[7:0]==8'hDB) results_r <= {ratloop3_stat[11:0],last_cmd[7:0]};  // RPC loop3 signals with error
		 if(last_cmd[7:0]==8'hDC) results_r <= {ratloop4_stat[11:0],last_cmd[7:0]};  // RPC loop4 signals with error
		    //                                   ^^^^ 47 bits here, only last 10:0 active, these are fast RPC loop tests
 CCB?		 if(last_cmd[7:0]==8'hDD) results_r <= {~vstat[3],1'b0,~vstat[1:0],slowloop_stat[7:0],last_cmd[7:0]};  // RPC slowloop signals with error
		    //                                   ^^^^ 8 bits here, only last 7:0 active, these are the SLOW RPC loop tests
		 if(last_cmd[7:0]==8'hDE) results_r <= {hzloop_count[11:0],last_cmd[7:0]}; // counts VERY slow 1.5 Hz Loopback cycles
		 if(last_cmd[7:0]==8'hDF) results_r <= {slowloop_errcnt[11:0],last_cmd[7:0]}; // total errors in 8 slow RPC loops, bit 11=rollover

		 if(last_cmd[7:0]==8'hE0) results_r <= {2'b10,cfeb_start[9:0],last_cmd[7:0]};   // all skewclear cables status
		 if(last_cmd[7:0]==8'hE1) results_r <= {cfeb0errcnt[11:0],last_cmd[7:0]};  // for cable 1
		 if(last_cmd[7:0]==8'hE2) results_r <= {cfeb1errcnt[11:0],last_cmd[7:0]};  // for cable 2
		 if(last_cmd[7:0]==8'hE3) results_r <= {cfeb2errcnt[11:0],last_cmd[7:0]};  // for cable 3
		 if(last_cmd[7:0]==8'hE4) results_r <= {cfeb3errcnt[11:0],last_cmd[7:0]};  // for cable 4
		 if(last_cmd[7:0]==8'hE5) results_r <= {cfeb4errcnt[11:0],last_cmd[7:0]};  // for cable 5
		 if(last_cmd[7:0]==8'hF0) results_r <= {fiber_stat[11:0],last_cmd[7:0]};  // all fibers link + enable status
		 if(last_cmd[7:0]==8'hF1) results_r <= {error_f1count[11:0],last_cmd[7:0]};  // for fiber 1
		 if(last_cmd[7:0]==8'hF2) results_r <= {error_f2count[11:0],last_cmd[7:0]};  // for fiber 2
		 if(last_cmd[7:0]==8'hF3) results_r <= {error_f3count[11:0],last_cmd[7:0]};  // for fiber 3
		 if(last_cmd[7:0]==8'hF4) results_r <= {error_f4count[11:0],last_cmd[7:0]};  // for fiber 4
		 if(last_cmd[7:0]==8'hF5) results_r <= {error_f5count[11:0],last_cmd[7:0]};  // for fiber 5
		 if(last_cmd[7:0]==8'hF6) results_r <= {error_f6count[11:0],last_cmd[7:0]};  // for fiber 6
		 if(last_cmd[7:0]==8'hF7) results_r <= {error_f7count[11:0],last_cmd[7:0]};  // for fiber 7
		 if(last_cmd[7:0]==8'hF8) results_r <= {fiber_invalid[11:0],last_cmd[7:0]};  // all fibers invalid trips
		 if(last_cmd[7:0]==8'hFC) results_r <= {cable_count[11:0],last_cmd[7:0]};   // count of cable test cycles
		 if(last_cmd[7:0]==8'hFD) results_r <= {loop_count[31:20],last_cmd[7:0]}; // #1.05M counts @40MHz Loopback test cycles
		 if(last_cmd[7:0]==8'hFE) results_r <= {slowloop_count[26:15],last_cmd[7:0]}; // #32K counts @1.6 MHz Loopback cycles
		 if(last_cmd[7:0]==8'hFF) results_r <= {fiber_count_r[11:0],last_cmd[7:0]};   // #1.05M counts of word[0] fiber test cycles




	      