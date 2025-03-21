// See LICENSE.iitm for license details
package mbox;

import ccore_types    :: *;
`include "Logger.bsv"

import combo          :: * ;
import restoring_div  :: * ;
import SpecialFIFOs   :: * ;
import FIFOF          :: * ;
import TxRx           :: * ;
import Assert         :: * ;

typedef struct{
`ifdef RV64
  Bool wordop ;
`endif
  Bit#(`xlen) in1; 
  Bit#(`xlen) in2;
  Bit#(3) funct3;
}MBoxIn deriving(Bits, FShow, Eq);

typedef struct{
  Bool mul;
  Bool div;
} MBoxRdy deriving(Bits, FShow, Eq);

interface Ifc_mbox;
	method Action ma_inputs(MBoxIn inputs);
  method MBoxRdy mv_ready;
  method TXe#(Bit#(`xlen)) tx_output;
  `ifdef arith_trap
    method Action ma_arith_trap_en(Bit#(1) en);
    method TXe#(Tuple2#(Bool, Bit#(`causesize))) tx_arith_trap_output;
  `endif
  `ifdef simulate
    method Action ma_simulate_log_start(Bit#(1) start);
  `endif
endinterface: Ifc_mbox

`ifdef mbox_noinline
(*synthesize*)
`endif
module mkmbox#(parameter Bit#(`xlen) hartid) (Ifc_mbox);
  String mbox = "";

  Ifc_combo_mul mul_ <- mkcombo_mul;
  Ifc_restoring_div div_ <- mkrestoring_div(hartid);

  FIFOF#(Bool) ff_ordering <- mkUGSizedFIFOF(max(`MULSTAGES_TOTAL,2));
  TX#(Bit#(`xlen)) tx_mbox_out <- mkTX;
  `ifdef arith_trap
    //Wire#(Bit#(1)) wr_arith_trap_en <- mkDWire(0);
    TX#(Tuple2#(Bool, Bit#(`causesize))) tx_arith_trap_out <- mkTX;
  `endif
  `ifdef simulate
    Wire#(Bit#(1)) wr_simulate_log_start <- mkDWire(0);
    rule rl_upd_log_start;
      mul_.ma_simulate_log_start(wr_simulate_log_start);
      div_.ma_simulate_log_start(wr_simulate_log_start);
    endrule
  `endif

  /*doc:rule: */
  rule rl_fifo_full(!tx_mbox_out.u.notFull());
    `logLevel( mbox, 0, $format("[%2d]MBOX: Buffer is FULL",hartid), wr_simulate_log_start)
    dynamicAssert(!mul_.mv_output_valid ,"MUL provided result when O/P FIFO is full");
    dynamicAssert(!div_.mv_output_valid ,"DIV provided result when O/P FIFO is full");
  endrule:rl_fifo_full

  /*doc:rule: */
  rule rl_capture_output(ff_ordering.notEmpty);
    if (ff_ordering.first) begin // mul operation
      if (mul_.mv_output_valid) begin
        let _x <- mul_.mv_output;
        tx_mbox_out.u.enq(_x);
        `ifdef arith_trap
          tx_arith_trap_out.u.enq(unpack(0));
        `endif
        ff_ordering.deq;
        `logLevel( mbox, 0, $format("MBOX: Collecting MUL o/p"), wr_simulate_log_start)
      end
      else
        `logLevel( mbox, 0, $format("MBOX: Waiting for Mul o/p"), wr_simulate_log_start)
    end
    else if (!ff_ordering.first) begin // div operation
      if (div_.mv_output_valid) begin
        let _x <- div_.mv_output;
        tx_mbox_out.u.enq(_x);
        `ifdef arith_trap
          tx_arith_trap_out.u.enq(div_.mv_arith_trap_out);
        `endif
        ff_ordering.deq;
        `logLevel( mbox, 0, $format("MBOX: Collecting DIV o/p"), wr_simulate_log_start)
      end
      else
        `logLevel( mbox, 0, $format("MBOX: Waiting for Div o/p"), wr_simulate_log_start)
    end
  endrule: rl_capture_output

	method Action ma_inputs(MBoxIn inputs);
  `ifdef ASSERT
    dynamicAssert(ff_ordering.notFull(), "Enquing MBOX inputs to full fifo");
  `endif
    if( inputs.funct3[2] == 0 ) begin // Multiplication ops
      `logLevel( mbox, 0, $format("MBOX: To MUL. Op1:%h Op2:%h ", inputs.in1, inputs.in2 ), wr_simulate_log_start)
      mul_.ma_inputs(inputs.in1, inputs.in2, inputs.funct3 `ifdef RV64 ,inputs.wordop `endif );
      ff_ordering.enq(True);
    end
    if (inputs.funct3[2] == 1) begin
      ff_ordering.enq(False);
      `logLevel( mbox, 0, $format("MBOX: To DIV. Op1:%h Op2:%h sign:%b", inputs.in1, inputs.in2, inputs.in1[valueOf(`xlen)-1] ), wr_simulate_log_start)
      div_.ma_inputs( inputs.in1, inputs.in2, inputs.funct3 `ifdef RV64 ,inputs.wordop `endif ) ;
    end
  endmethod
  method mv_ready= MBoxRdy{mul: mul_.mv_ready && ff_ordering.notFull, div: div_.mv_ready && ff_ordering.notFull()};

  method tx_output = tx_mbox_out.e;
  `ifdef arith_trap
    method Action ma_arith_trap_en(Bit#(1) en);
      `logLevel( mbox, 0, $format("MBOX: arith_en: %h ", en ), wr_simulate_log_start)
       div_.ma_div_arith_trap_en(en);
      //wr_arith_trap_en <= en;
    endmethod
    method tx_arith_trap_output = tx_arith_trap_out.e;
  `endif
  `ifdef simulate
    method Action ma_simulate_log_start(Bit#(1) start);
      wr_simulate_log_start <= start;
    endmethod
  `endif
endmodule
endpackage


