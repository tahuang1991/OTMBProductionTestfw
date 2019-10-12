//
// PRNG65_16
//    J. Gilmore, 4/20/11
//-----------------------------------------------------------------------------
// 65-bit PRBS module that generates a 16-bit number, refs:
//   http://www.eng.auburn.edu/~strouce/polycorrect2.pdf
//   http://www.xilinx.com/support/documentation/application_notes/xapp052.pdf
//   http://www.eecs.berkeley.edu/~newton/Classes/CS150sp97/labs/lab5/lab5.html#primPolynomials
//   http://en.wikipedia.org/wiki/Linear_feedback_shift_register
//   http://www.springerlink.com/content/u590n42r72568803/
// Note that for any primitive polynomial, the reverse polynomial is also primitive!
//-----------------------------------------------------------------------------

module prng65_16(
	input [64:0] init_dat,
	input        en,
	output reg [15:0] dout,
	input        rst,
	input        clk);

	reg  [65:1] lfsr;

	always @(posedge clk, posedge rst) begin
	   if(rst) begin
	      lfsr[65:1]  <= init_dat[64:0];
	      dout[15:0] <= 0;
	   end
	   else begin
	      dout[15:0] <= lfsr[16:1];
	      if (en) lfsr  <= {lfsr[64:1],lfsr[65]^lfsr[18]};
	      else lfsr  <= lfsr;
	   end
	end // always
endmodule
