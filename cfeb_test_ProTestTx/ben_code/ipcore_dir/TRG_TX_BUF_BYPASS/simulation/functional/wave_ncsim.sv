
###############################################################################
##
## (c) Copyright 2009-2010 Xilinx, Inc. All rights reserved.
##
## This file contains confidential and proprietary information
## of Xilinx, Inc. and is protected under U.S. and
## international copyright and other intellectual property
## laws.
##
## DISCLAIMER
## This disclaimer is not a license and does not grant any
## rights to the materials distributed herewith. Except as
## otherwise provided in a valid license issued to you by
## Xilinx, and to the maximum extent permitted by applicable
## law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
## WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
## AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
## BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
## INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
## (2) Xilinx shall not be liable (whether in contract or tort,
## including negligence, or under any other theory of
## liability) for any loss or damage of any kind or nature
## related to, arising under or in connection with these
## materials, including for any direct, or any indirect,
## special, incidental, or consequential loss or damage
## (including loss of data, profits, goodwill, or any type of
## loss or damage suffered as a result of any action brought
## by a third party) even if such damage or loss was
## reasonably foreseeable or Xilinx had been advised of the
## possibility of the same.
##
## CRITICAL APPLICATIONS
## Xilinx products are not designed or intended to be fail-
## safe, or for use in any application requiring fail-safe
## performance, such as life-support or safety devices or
## systems, Class III medical devices, nuclear facilities,
## applications related to the deployment of airbags, or any
## other applications that could lead to death, personal
## injury, or severe property or environmental damage
## (individually and collectively, "Critical
## Applications"). Customer assumes the sole risk and
## liability of any use of Xilinx products in Critical
## Applications, subject only to applicable laws and
## regulations governing limitations on product liability.
## 
## THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
## PART OF THIS FILE AT ALL TIMES.

  window new WaveWindow  -name  "Waves for Virtex-6 GTX Wizard Example Design"
  waveform  using  "Waves for Virtex-6 GTX Wizard Example Design"
  
  waveform  add  -label GTX0_TRG_TX_BUF_BYPASS -comment GTX0_TRG_TX_BUF_BYPASS
  waveform  add  -label Receive_Ports_-_RX_Driver,OOB_signalling,Coupling_and_Eq.,CDR  -comment  Receive_Ports_-_RX_Driver,OOB_signalling,Coupling_and_Eq.,CDR
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:RXN_IN
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:RXP_IN
  waveform  add  -label Transmit_Ports_-_8b10b_Encoder_Control_Ports  -comment  Transmit_Ports_-_8b10b_Encoder_Control_Ports
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:TXCHARISK_IN
  waveform  add  -label Transmit_Ports_-_TX_Data_Path_interface  -comment  Transmit_Ports_-_TX_Data_Path_interface
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:TXDATA_IN
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:TXOUTCLK_OUT
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:TXUSRCLK_IN
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:TXUSRCLK2_IN
  waveform  add  -label Transmit_Ports_-_TX_Driver_and_OOB_signaling  -comment  Transmit_Ports_-_TX_Driver_and_OOB_signaling
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:TXN_OUT
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:TXP_OUT
  waveform  add  -label Transmit_Ports_-_TX_Elastic_Buffer_and_Phase_Alignment_Ports  -comment  Transmit_Ports_-_TX_Elastic_Buffer_and_Phase_Alignment_Ports
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:TXDLYALIGNDISABLE_IN
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:TXDLYALIGNMONENB_IN
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:TXDLYALIGNMONITOR_OUT
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:TXDLYALIGNRESET_IN
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:TXENPMAPHASEALIGN_IN
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:TXPMASETPHASE_IN
  waveform  add  -label Transmit_Ports_-_TX_PLL_Ports  -comment  Transmit_Ports_-_TX_PLL_Ports
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:GTXTXRESET_IN
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:MGTREFCLKTX_IN
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:PLLTXRESET_IN
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:TXPLLLKDET_OUT
  waveform  add  -signals  :trg_tx_buf_bypass_top_i:trg_tx_buf_bypass_i:gtx0_trg_tx_buf_bypass_i:TXRESETDONE_OUT

  console submit -using simulator -wait no "run 145 us"

