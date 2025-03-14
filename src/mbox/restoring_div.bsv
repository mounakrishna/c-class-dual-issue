//See LICENSE.iitm for license details
/*

Author: IIT Madras
Details:

--------------------------------------------------------------------------------------------------
*/
package restoring_div;

import ccore_types    :: * ;
`include "Logger.bsv"
import Assert         :: * ;
import DReg           :: * ;
import FIFOF          :: * ;
import ConfigReg      :: * ;
`include "trap.defines"

function Tuple2#(Bit#(TAdd#(1,`xlen)), Bit#(`xlen)) fn_single_div (Bit#(TAdd#(1,`xlen)) remainder,Bit#(`xlen) quotient, Bit#(`xlen) divisor);
  for(Integer i=0; i<(valueOf(`xlen)/`DIVSTAGES); i=i+ 1)begin
    let x={remainder, quotient}<<1;
    remainder=truncateLSB(x);
    quotient=truncate(x);
    Bit#(TAdd#(1, `xlen)) sub=remainder+signExtend(~divisor+1);
    if(truncate(remainder)>=divisor)begin // if subtraction is positive
			quotient[0]=1;
			remainder=sub;
    end
  end
  return tuple2(remainder,quotient);
endfunction

function Tuple4#(Bit#(`xlen), Bit#(`xlen), Bool, Bit#(1)) fn_fix_inputs (Bit#(`xlen) in1, Bit#(`xlen) in2, Bit#(3) funct3
  `ifdef RV64 , Bool wordop `endif );

  Bool mul_div = unpack(funct3[2]);
  `ifdef RV64
    if(wordop)begin
      in1=funct3[0]==0? signExtend(in1[31:0]):zeroExtend(in1[31:0]);
      in2=funct3[0]==0? signExtend(in2[31:0]):zeroExtend(in2[31:0]);
    end
  `endif

  Bool lv_upperbits = unpack(funct3[1]);

  Bool invert_op1=False;
  Bool invert_op2=False;

  if(funct3[2]==1 && funct3[0]==0 && in1[valueOf(`xlen)-1]==1) // in case of division operations.
    invert_op1=True;

  if(funct3[2]==1 && funct3[0]==0 && in2[valueOf(`xlen)-1]==1)// in case of division operations.
    invert_op2=True;

  Bit#(`xlen) t1=signExtend(pack(invert_op1));
  Bit#(`xlen) t2=signExtend(pack(invert_op2));
  Bit#(`xlen) op1= (t1^in1)+ zeroExtend(pack(invert_op1));
  Bit#(`xlen) op2= (t2^in2)+ zeroExtend(pack(invert_op2));

  Bool lv_take_complement = False;
  if(funct3==4) // in case of DIV
	  lv_take_complement=unpack(in1[valueOf(`xlen)-1]^in2[valueOf(`xlen)-1]);
  else if(funct3==6)
    lv_take_complement=True;

  return tuple4(op1, op2, lv_take_complement, in1[valueOf(`xlen)-1]);
endfunction


interface Ifc_restoring_div;
  method Action ma_inputs(Bit#(`xlen) in1, Bit#(`xlen) in2,  Bit#(3) funct3
                                                `ifdef RV64 ,Bool wordop `endif ) ;
	method Bool mv_ready;
	method Bool mv_output_valid;
	method ActionValue#(Bit#(`xlen)) mv_output;
  `ifdef arith_trap
    method Tuple2#(Bool, Bit#(`causesize)) mv_arith_trap_out;
    method Action ma_div_arith_trap_en(Bit#(1) en);
 `endif
  `ifdef simulate
    method Action ma_simulate_log_start(Bit#(1) start);
  `endif
endinterface

`ifdef mbox_div_noinline
(*synthesize*)
(*conflict_free="single_step_div, ma_inputs"*)
(*conflict_free="single_step_div, mv_output"*)
`endif
module mkrestoring_div#(parameter Bit#(`xlen) hartid) (Ifc_restoring_div);

  String divider = "";
  staticAssert(valueOf(TExp#(TLog#(`DIVSTAGES))) == `DIVSTAGES, "DIVSTAGES is not power of 2");
  staticAssert(`DIVSTAGES<= valueOf(`xlen), "DIVSTAGES cannot be larger than `xlen");

  Reg#(Bit#(TLog#(TAdd#(`DIVSTAGES, 1)))) rg_count <-mkReg(0);

  Reg#(Bit#(TAdd#(1, TMul#(2, `xlen)))) partial<-mkReg(0);
  Reg#(Bit#(`xlen)) rg_op2 <-mkReg(0);
  Reg#(Bit#(`xlen)) rg_in1 <-mkReg(0);
  Reg#(Bool) quotient_remainder <- mkReg(False);
  Reg#(Bool) rg_upperbits <- mkReg(False);
  Reg#(Bool) rg_complement <- mkReg(False);
  Reg#(Bit#(1)) rg_sign_op1 <- mkReg(0);

  ConfigReg#(Bool)  rg_valid <- mkConfigReg(False);
  Reg#(Bit#(`xlen))  rg_result <- mkReg(0);
`ifdef RV64
  Reg#(Bool)        rg_wordop <- mkReg(False);
`endif

  `ifdef simulate
    Wire#(Bit#(1)) wr_simulate_log_start <- mkDWire(0);
  `endif
  `ifdef arith_trap
    Wire#(Bit#(1)) wr_arith_trap <- mkDWire(0);
    Reg#(Bool)  rg_trap <- mkDReg(False);
  `endif

  rule rl_display;
    `logLevel( divider, 0, $format("[%2d]DIV: RgCount:%d rg_valid:%d",hartid, rg_count, rg_valid), wr_simulate_log_start)
  endrule

  rule single_step_div(rg_count != 0 && !rg_valid);
    let {upper, lower}=fn_single_div(truncateLSB(partial),truncate(partial), rg_op2);
    partial<= {upper, lower};
    `logLevel( divider, 0, $format("[%2d]DIV: RgCount:%d partial:%h QR:%b",hartid, rg_count, partial, quotient_remainder), wr_simulate_log_start)
    if(rg_op2 == 0)begin
      `logLevel( divider, 0, $format("[%2d] DIV: Divide by zero detected. RgCount:%d",hartid, rg_count), wr_simulate_log_start)
      rg_count <= 0;
      rg_valid <= True;
      `ifdef arith_trap
        `logLevel( divider, 0, $format("[%2d] DIV: Arith_trap_EN ",hartid, wr_arith_trap), wr_simulate_log_start)
        if(wr_arith_trap==1)
          rg_trap <= True;
      `endif
      Bit#(`xlen) reslt=quotient_remainder? truncate(rg_in1):'1;
      Bit#(`xlen) product= `ifdef RV64 rg_wordop?signExtend(reslt[31:0]): `endif truncate(reslt);
      rg_result <= product;
    end
    else if(rg_count == fromInteger(`DIVSTAGES)+ 1 ) begin
      rg_count <= 0;
      rg_valid <= True;
      `ifdef arith_trap
        rg_trap <= False;
      `endif
      Bit#(`xlen) reslt=quotient_remainder?partial[valueOf(TMul#(2, `xlen))-1:valueOf(`xlen)]:  truncate(partial);
      if((rg_upperbits && rg_complement && reslt[valueOf(`xlen)-1] != rg_sign_op1)||(rg_complement && !rg_upperbits))
      reslt = ~reslt+ 1;
      Bit#(`xlen) product= `ifdef RV64 rg_wordop?signExtend(reslt[31:0]): `endif truncate(reslt);
      rg_result <= product;
      `logLevel( divider, 0, $format("[%2d] DIV: Sending output:%h",hartid,product), wr_simulate_log_start)
    end
    else
      rg_count <= rg_count +1;
  endrule: single_step_div

  method Action ma_inputs(Bit#(`xlen) in1, Bit#(`xlen) in2,  Bit#(3) funct3
                        `ifdef RV64 ,Bool wordop `endif );
    `logLevel( divider, 0, $format("[%2d]DIV: Got inputs rg_count: %d",hartid, rg_count), wr_simulate_log_start)
    let {op1, op2, complement, sign_op1} = fn_fix_inputs(in1, in2, funct3 `ifdef RV64 ,wordop `endif );
    partial<= zeroExtend(op1);
    rg_op2<= op2;
    rg_in1<= in1;
    rg_count <= rg_count + 1;
    rg_complement <= complement;
    rg_sign_op1<= sign_op1;
    quotient_remainder<= unpack(funct3[1]);
    `ifdef RV64
      rg_wordop <= wordop;
    `endif
    rg_upperbits <= unpack(funct3[1]);
  endmethod: ma_inputs

	method mv_ready = rg_count == 0 && !rg_valid ;
	method mv_output_valid = rg_valid;
	method ActionValue#(Bit#(`xlen)) mv_output;
	  rg_valid <= False;
    return rg_result;
  endmethod
  `ifdef simulate
    method Action ma_simulate_log_start(Bit#(1) start);
      wr_simulate_log_start <= start;
    endmethod
  `endif
  `ifdef arith_trap
    method mv_arith_trap_out = tuple2(rg_trap, `Int_divide_by_zero);
    method Action ma_div_arith_trap_en(Bit#(1) en);
      wr_arith_trap <= en;
    endmethod
 `endif

endmodule
endpackage
