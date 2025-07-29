//See LICENSE.iitm for license details
/*

Author : IIT Madras
Details:

This module implements the integer and floating point register files. They are currently implemented
as RegFile. The integer register file requires 2 read and 1 write ports.
The floating point registerfile however will require 3 read ports and 1 write ports

On system reset,  the register files are initialized to 0. This phase will take 32 cycles total.
Only after the initialization phase can the

compile params affecting this file:
- merged_rf: indicate if frf and xrf should be merged into a single RF. Only enabled when F/D
  support is there.
- spfpu: to indicate if floating point support is available or not.
--------------------------------------------------------------------------------------------------
*/
package registerfile;
	import ccore_types::*;
	import RegFile4r2w::*;
  import RegFile::*;
  import Vector :: *;
  `include "Logger.bsv"

	interface Ifc_registerfile;
    method ActionValue#(Bit#(`elen)) read_rs1(Bit#(5) addr `ifdef spfpu, RFType rstype `endif );
    method ActionValue#(Bit#(`elen)) read_rs2(Bit#(5) addr `ifdef spfpu, RFType rstype `endif );
    method ActionValue#(Bit#(`elen)) read_rs3(Bit#(5) addr);
    method ActionValue#(Bit#(`elen)) read_rs4(Bit#(5) addr `ifdef spfpu, RFType rstype `endif );
    method ActionValue#(Bit#(`elen)) read_rs5(Bit#(5) addr `ifdef spfpu, RFType rstype `endif );
		method Action commit_rd (Vector#(`num_issue, CommitData) c);
		//method Action commit_rd (CommitData c);
  `ifdef simulate
    method Action ma_simulate_log_start(Bit#(1) start);
  `endif
	endinterface
`ifdef registerfile_noinline
	(*synthesize*)
`endif
	module mkregisterfile#(parameter Bit#(`xlen) hartid) (Ifc_registerfile);
    String regfile ="";
`ifdef merged_rf
    RegFile4r2w#(Bit#(6), Bit#(`elen)) rf <- mkRegFile4r2wWCF(0,63);
		Reg#(Bit#(6)) rg_index <- mkReg(0);
`else
    RegFile4r2w#(Bit#(5), Bit#(`xlen)) xrf <- mkRegFile4r2wWCF(0, 31);
		Reg#(Bit#(5)) rg_index <- mkReg(0);
  `ifdef spfpu
    RegFile#(Bit#(5), Bit#(`flen)) frf <- mkRegFileWCF(0, 31);
  `endif
`endif

		Reg#(Bool) initialize <- mkReg(True);
  `ifdef simulate
    Wire#(Bit#(1)) wr_simulate_log_start <- mkDWire(0);
  `endif

    // The following rule is fired on system reset and writes all the register values to "0". This
    // rule will never fire otherwise
		rule initialize_regfile(initialize);
    `ifdef merged_rf
			rf.upd_1(rg_index,0);
    `else
      xrf.upd_1(rg_index, 0);
      `ifdef spfpu
        frf.upd(rg_index, 0);
      `endif
    `endif
			rg_index <= rg_index + 1;
			if(rg_index == `ifdef merged_rf 'd63 `else 'd31 `endif )
				initialize <= False;
        `logLevel( regfile, 1, $format("[%2d]RF : Initialization phase. Count: %d",hartid,rg_index), wr_simulate_log_start)
		endrule


    // This method will read operand1 using rs1addr from the decode stage. If there a commit in the
    // same cycle to rs1addr, then that value if bypassed else the value is read from the
    // corresponding register file.
    // Explicit Conditions : fire only when initialize is False;
    // Implicit Conditions : None
    method ActionValue#(Bit#(`elen)) read_rs1(Bit#(5) addr `ifdef spfpu, RFType rstype `endif )
                                                                                    if(!initialize);
      `ifdef merged_rf
        return zeroExtend(rf.sub({pack(rstype==FRF),addr}));
      `else
        `ifdef spfpu
          if(rstype == FRF) return zeroExtend(frf.sub(addr)); else
        `endif
        return zeroExtend(xrf.sub(addr)); // zero extend is required when `xlen<`elen*/
      `endif
    endmethod
    // This method will read operand2 using rs2addr from the decode stage. If there a commit in the
    // same cycle to rs2addr, then that value if bypassed else the value is read from the
    // corresponding register file.
    // Explicit Conditions : fire only when initialize is False;
    // Implicit Conditions : None
    method ActionValue#(Bit#(`elen)) read_rs2(Bit#(5) addr `ifdef spfpu, RFType rstype `endif )
                                                                                    if(!initialize);
      `ifdef merged_rf
        return zeroExtend(rf.sub({pack(rstype==FRF),addr}));
      `else
        `ifdef spfpu
          if(rstype == FRF) return zeroExtend(frf.sub(addr)); else
        `endif
        return zeroExtend(xrf.sub(addr)); // zero extend is required when XLEN<ELEN*/
      `endif
    endmethod

    // This method will read operand3 using rs3addr from the decode stage. If there a commit in the
    // same cycle to rs2addr, then that value if bypassed else the value is read from the
    // Floating register file. Integer RF is not looked - up for rs3 at all.
    // Explicit Conditions : fire only when initialize is False;
    // Implicit Conditions : None
    method ActionValue#(Bit#(`elen)) read_rs3(Bit#(5) addr) if(!initialize);
      /*return frf.sub(addr);*/
      `ifdef merged_rf 
         return rf.sub({1'b1,addr});
      `else 
         return frf.sub(addr);
      `endif
    endmethod

    // This method will read operand3 using rs3addr from the decode stage. If there a commit in the
    // same cycle to rs2addr, then that value if bypassed else the value is read from the
    // Floating register file. Integer RF is not looked - up for rs3 at all.
    // Explicit Conditions : fire only when initialize is False;
    // Implicit Conditions : None
    method ActionValue#(Bit#(`elen)) read_rs4(Bit#(5) addr `ifdef spfpu, RFType rstype `endif ) if(!initialize);
      `ifdef merged_rf
        return zeroExtend(rf.sub({pack(rstype==FRF),addr}));
      `else
        return zeroExtend(xrf.sub(addr)); // zero extend is required when XLEN<ELEN*/
      `endif
    endmethod

    // This method will read operand3 using rs3addr from the decode stage. If there a commit in the
    // same cycle to rs2addr, then that value if bypassed else the value is read from the
    // Floating register file. Integer RF is not looked - up for rs3 at all.
    // Explicit Conditions : fire only when initialize is False;
    // Implicit Conditions : None
    method ActionValue#(Bit#(`elen)) read_rs5(Bit#(5) addr `ifdef spfpu, RFType rstype `endif ) if(!initialize);
      `ifdef merged_rf
        return zeroExtend(rf.sub({pack(rstype==FRF),addr}));
      `else
        return zeroExtend(xrf.sub(addr)); // zero extend is required when XLEN<ELEN*/
      `endif
    endmethod

    // This method is fired when the write - back stage performs a commit and needs to update the RF.
    // The value being commited is updated in the respective register file and also bypassed to the
    // above methods for operand forwarding.
    // Explicit Conditions : fire only when initialize is False;
    // Implicit Conditions : None
		//method Action commit_rd_1 (CommitData c) if(!initialize);
    // `logLevel( regfile, 1, $format("[%2d]RF : Writing Rd: %d(%h) ",hartid,c.addr, c.data
    //                                              `ifdef spfpu, fshow(c.rdtype) `endif ), wr_simulate_log_start)
    //`ifdef merged_rf 
  	//  if (c.rdtype != IRF || c.addr != 0)
    //	  rf.upd_1({pack(c.rdtype==FRF),c.addr},truncate(c.data));
    //`else
    //  `ifdef spfpu
    //    if(c.rdtype == FRF) frf.upd(c.addr, truncate(c.data)); else
    //  `endif
    //    if(c.addr != 0) xrf.upd_1(c.addr, truncate(c.data)); // truncate is required when XLEN<ELEN
    //`endif
		//endmethod

		//method Action commit_rd_2 (CommitData c) if(!initialize);
    // `logLevel( regfile, 1, $format("[%2d]RF : Writing Rd: %d(%h) ",hartid,c.addr, c.data
    //                                              `ifdef spfpu, fshow(c.rdtype) `endif ), wr_simulate_log_start)
    //`ifdef merged_rf 
  	//  if (c.rdtype != IRF || c.addr != 0)
    //	  rf.upd_2({pack(c.rdtype==FRF),c.addr},truncate(c.data));
    //`else
    //    if(c.addr != 0) xrf.upd_2(c.addr, truncate(c.data)); // truncate is required when XLEN<ELEN
    //`endif
		//endmethod
    
    method Action commit_rd (Vector#(`num_issue, CommitData) c) if(!initialize);
      if (!c[0].unlock_only && c[0].addr != 0 && c[0].rdtype == IRF)
          xrf.upd_1(c[0].addr, truncate(c[0].data));
      if (!c[1].unlock_only && c[1].addr != 0 && c[1].rdtype == IRF)
          xrf.upd_2(c[1].addr, truncate(c[1].data));

      if (!c[0].unlock_only && c[0].rdtype == FRF)
          frf.upd(c[0].addr, truncate(c[0].data)); 
      else if (!c[1].unlock_only && c[1].rdtype == FRF)
          frf.upd(c[1].addr, truncate(c[1].data)); 

      //if(!c[0].unlock_only) 
      //  if (c[0].rdtype == FRF) 
      //  else if (c[0].addr != 0) 
      //if (!c[1].unlock_only && c[1].addr != 0) 
      //  xrf.upd_2(c[
    endmethod
  `ifdef simulate
    method Action ma_simulate_log_start(Bit#(1) start);
      wr_simulate_log_start <= start;
    endmethod
  `endif
	endmodule
endpackage
