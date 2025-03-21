// See LICENSE.iitm for license details
/*
Author: IIT Madras
Created on: Wednesday 16 June 2021 09:53:08 PM

*/
package combo ;
  import FIFOF        :: * ;
  import Vector       :: * ;
  import SpecialFIFOs :: * ;

  import signedmul      :: * ;
  import ccore_types    :: * ;

  `include "Logger.bsv"

interface Ifc_combo_mul;
	method Action ma_inputs(Bit#(`xlen) in1, Bit#(`xlen) in2, Bit#(3) funct3
		                                              `ifdef RV64, Bool wordop `endif );
	method Bool mv_ready;
	method Bool mv_output_valid;
	method ActionValue#(Bit#(`xlen)) mv_output;
  `ifdef simulate
    method Action ma_simulate_log_start(Bit#(1) start);
  `endif
endinterface

`ifdef mbox_mul_noinline
(*synthesize*)
`endif
module mkcombo_mul(Ifc_combo_mul);

  String mul = "";

  Vector#(`MULSTAGES_IN, Reg#(Bit#(TAdd#(1, `xlen))))  rg_op1  <- replicateM(mkReg(0));
  Vector#(`MULSTAGES_IN, Reg#(Bit#(TAdd#(1, `xlen))))  rg_op2  <- replicateM(mkReg(0));
  Vector#(`MULSTAGES_IN, Reg#(Bit#(3)))                rg_fn3  <- replicateM(mkReg(0));
`ifdef RV64
  Vector#(`MULSTAGES_IN, Reg#(Bool))                   rg_wordop  <- replicateM(mkReg(False));
`endif

  /*doc:wire: */
  Wire#(Bit#(TAdd#(1, `xlen))) wr_op1 <- mkWire();
  /*doc:wire: */
  Wire#(Bit#(TAdd#(1, `xlen))) wr_op2 <- mkWire();
  /*doc:wire: */
  Wire#(Bit#(3)) wr_fn3 <- mkWire();
  /*doc:wire: */
  Wire#(Bool) wr_wordop <- mkWire();

  Vector#(`MULSTAGES_IN, FIFOF#(Bool)) rg_valid_in <- replicateM(mkUGLFIFOF);
  Vector#(`MULSTAGES_OUT, FIFOF#(Bool)) rg_valid_out <- replicateM(mkUGLFIFOF);
  Vector#(`MULSTAGES_OUT, Reg#(Bit#(`xlen)))           rg_output <- replicateM(mkReg(0));

  /*doc:wire: */
  Wire#(Bit#(`xlen)) wr_output <- mkDWire(?);
  /*doc:wire: */
  Wire#(Bool) wr_valid <- mkDWire(False);

  `ifdef simulate
    Wire#(Bit#(1)) wr_simulate_log_start <- mkDWire(0);
  `endif

  Ifc_signedmul#(TAdd#(`xlen, 1), TAdd#(`xlen, 1)) signed_mul <- mksignedmul();

  /*doc:rule: */
  for (Integer i = `MULSTAGES_IN -1 ; i > 0; i = i - 1) begin
    rule rl_move_inputs(rg_valid_in[i].notFull && rg_valid_in[i-1].notEmpty);
      rg_op1[i] <= rg_op1[i - 1];
      rg_op2[i] <= rg_op2[i - 1];
      rg_fn3[i] <= rg_fn3[i-1];
    `ifdef RV64
      rg_wordop[i] <= rg_wordop[i-1];
    `endif
      rg_valid_in[i].enq(rg_valid_in[i-1].first);
      rg_valid_in[i-1].deq;
    endrule:rl_move_inputs
  end

  /*doc:rule: */
  if (`MULSTAGES_IN > 0 && `MULSTAGES_OUT > 0 ) begin
    rule rl_perform_mul_0(rg_valid_in[`MULSTAGES_IN - 1].notEmpty && rg_valid_out[0].notFull);
      rg_valid_out[0].enq(rg_valid_in[`MULSTAGES_IN - 1].first);
      rg_valid_in[`MULSTAGES_IN - 1].deq;
      signed_mul.ia(rg_op1[`MULSTAGES_IN - 1]);
      signed_mul.ib(rg_op2[`MULSTAGES_IN - 1]);
      Bool lv_upperbits ;
      lv_upperbits = unpack(|rg_fn3[`MULSTAGES_IN -1][1 : 0]);
      let out = signed_mul.oc;

      Bit#(`xlen) default_out;
      if(lv_upperbits)
        default_out = pack(out)[valueOf(TMul#(2, `xlen)) - 1 : valueOf(`xlen)];
      else
        default_out = pack(out)[valueOf(`xlen) - 1:0];
    `ifdef RV64
      if (rg_wordop[`MULSTAGES_IN - 1])
        default_out = signExtend(default_out[31 : 0]);
    `endif
      rg_output[0] <= default_out;
    endrule:rl_perform_mul_0
  end
  else if (`MULSTAGES_OUT > 0) begin
    rule rl_perform_mul_1(rg_valid_out[0].notFull);
      signed_mul.ia(wr_op1);
      signed_mul.ib(wr_op2);
      Bool lv_upperbits ;
      lv_upperbits = unpack(|wr_fn3[1 : 0]);
      let out = signed_mul.oc;

      Bit#(`xlen) default_out;
      if(lv_upperbits)
        default_out = pack(out)[valueOf(TMul#(2, `xlen)) - 1 : valueOf(`xlen)];
      else
        default_out = pack(out)[valueOf(`xlen) - 1:0];
    `ifdef RV64
      if (wr_wordop)
        default_out = signExtend(default_out[31 : 0]);
    `endif
      rg_output[0] <= default_out;
    endrule: rl_perform_mul_1
  end
  else if (`MULSTAGES_IN > 0) begin
    rule rl_perform_mul_2(rg_valid_in[`MULSTAGES_IN - 1].notEmpty);
      rg_valid_in[`MULSTAGES_IN - 1].deq;
      signed_mul.ia(rg_op1[`MULSTAGES_IN - 1]);
      signed_mul.ib(rg_op2[`MULSTAGES_IN - 1]);
      Bool lv_upperbits ;
      lv_upperbits = unpack(|rg_fn3[`MULSTAGES_IN -1][1 : 0]);
      let out = signed_mul.oc;

      Bit#(`xlen) default_out;
      if(lv_upperbits)
        default_out = pack(out)[valueOf(TMul#(2, `xlen)) - 1 : valueOf(`xlen)];
      else
        default_out = pack(out)[valueOf(`xlen) - 1:0];
    `ifdef RV64
      if (rg_wordop[`MULSTAGES_IN - 1])
        default_out = signExtend(default_out[31 : 0]);
    `endif
      wr_output <= default_out;
      wr_valid <= True;
      `logLevel( mul, 0, $format("MUL: product computed:%h",default_out), wr_simulate_log_start)
    endrule:rl_perform_mul_2
  end
  else begin
    rule rl_perform_mul_3;
      signed_mul.ia(wr_op1);
      signed_mul.ib(wr_op2);
      Bool lv_upperbits ;
      lv_upperbits = unpack(|wr_fn3[1 : 0]);
      let out = signed_mul.oc;

      Bit#(`xlen) default_out;
      if(lv_upperbits)
        default_out = pack(out)[valueOf(TMul#(2, `xlen)) - 1 : valueOf(`xlen)];
      else
        default_out = pack(out)[valueOf(`xlen) - 1:0];
    `ifdef RV64
      if (wr_wordop)
        default_out = signExtend(default_out[31 : 0]);
    `endif
      wr_output <= default_out;
      wr_valid <= True;
    endrule: rl_perform_mul_3
  end

  for (Integer i = `MULSTAGES_OUT - 1; i>0; i = i - 1) begin
    /*doc:rule: */
    rule rl_move_outputs(rg_valid_out[i].notFull && rg_valid_out[i-1].notEmpty);
      rg_output[i] <= rg_output[i-1];
      rg_valid_out[i].enq(rg_valid_out[i-1].first);
      rg_valid_out[i-1].deq;
    endrule:rl_move_outputs
  end

	method Action ma_inputs(Bit#(`xlen) in1, Bit#(`xlen) in2, Bit#(3) funct3
		`ifdef RV64, Bool wordop `endif );

    Bit#(1) sign1 = funct3[1]^funct3[0];
    Bit#(1) sign2 = pack(funct3[1 : 0] == 1);
    if(`MULSTAGES_IN > 0) begin
      rg_op1[0] <= unpack({sign1 & in1[valueOf(`xlen) - 1], in1});
      rg_op2[0] <= unpack({sign2 & in2[valueOf(`xlen) - 1], in2});
      rg_fn3[0] <= funct3;
    `ifdef RV64
      rg_wordop[0] <= wordop;
    `endif
    end
    else begin
      wr_op1 <= unpack({sign1 & in1[valueOf(`xlen) - 1], in1});
      wr_op2 <= unpack({sign2 & in2[valueOf(`xlen) - 1], in2});
      wr_fn3 <= funct3;
    `ifdef RV64
      wr_wordop <= wordop;
    `endif
    end
    if(`MULSTAGES_IN > 0)
      rg_valid_in[0].enq(True);
    else if (`MULSTAGES_OUT > 0)
      rg_valid_out[0].enq(True);

  endmethod
  method ActionValue#(Bit#(`elen)) mv_output;
    if (`MULSTAGES_OUT > 0 ) rg_valid_out[`MULSTAGES_OUT - 1].deq;
    return (`MULSTAGES_OUT> 0)?rg_output[`MULSTAGES_OUT - 1]: wr_output;
  endmethod
  method mv_ready = (`MULSTAGES_IN > 0)?rg_valid_in[0].notFull(): 
                       (`MULSTAGES_OUT> 0)?rg_valid_out[0].notFull: True;
  method mv_output_valid = (`MULSTAGES_OUT > 0)?rg_valid_out[`MULSTAGES_OUT - 1].notEmpty : wr_valid;

  `ifdef simulate
    method Action ma_simulate_log_start(Bit#(1) start);
      wr_simulate_log_start <= start;
    endmethod
  `endif

endmodule:mkcombo_mul
endpackage: combo

