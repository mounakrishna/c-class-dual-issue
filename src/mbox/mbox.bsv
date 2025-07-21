// See LICENSE.iitm for license details
package mbox;

import ccore_types    :: *;
`include "Logger.bsv"

//import combo          :: * ;
import signedmul      :: * ;
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

  Reg#(Bool)     rg_fn3  <- mkReg(False);
  `ifdef RV64
    Reg#(Bool)        rg_word <- mkReg(False);
  `endif

  Ifc_signedmul#(TAdd#(XLEN, 1), TAdd#(XLEN, 1)) signed_mul <- mksignedmul();
  //Wire#(Bit#(XLEN)) wr_result <- mkWire();
  //Reg#(Bool)  rg_valid  <- mkDReg(False);

  Ifc_restoring_div div_ <- mkrestoring_div(hartid);

  FIFOF#(Bool) ff_ordering <- mkUGSizedFIFOF(max(`MULSTAGES_TOTAL,2));
  Reg#(Bit#(64)) temp_out <- mkReg(0);
  Reg#(Bool) temp_out_valid <- mkReg(False);
  TX#(Bit#(`xlen)) tx_mbox_out <- mkTX;

  `ifdef arith_trap
    //Wire#(Bit#(1)) wr_arith_trap_en <- mkDWire(0);
    TX#(Tuple2#(Bool, Bit#(`causesize))) tx_arith_trap_out <- mkTX;
  `endif
  `ifdef simulate
    Wire#(Bit#(1)) wr_simulate_log_start <- mkDWire(0);
    rule rl_upd_log_start;
      //mul_.ma_simulate_log_start(wr_simulate_log_start);
      div_.ma_simulate_log_start(wr_simulate_log_start);
    endrule
  `endif

  /*doc:rule: */
  rule rl_fifo_full(!tx_mbox_out.u.notFull());
    `logLevel( mbox, 0, $format("[%2d]MBOX: Buffer is FULL",hartid), wr_simulate_log_start)
    dynamicAssert(!signed_mul.output_valid ,"MUL provided result when O/P FIFO is full");
    dynamicAssert(!div_.mv_output_valid ,"DIV provided result when O/P FIFO is full");
  endrule:rl_fifo_full

  /*doc:rule: */
  rule rl_capture_output(ff_ordering.notEmpty);
        
    Bit#(64) _y='0;
    
    let out = signed_mul.receive;
    Bit#(XLEN) default_out;
    if(rg_fn3)
      default_out = pack(out)[valueOf(TMul#(2, XLEN)) - 1 : valueOf(XLEN)];
    else
      default_out = pack(out)[valueOf(XLEN) - 1:0];
    `ifdef RV64
      if(rg_word)
        default_out = signExtend(default_out[31 : 0]);
    `endif
    
    if(!ff_ordering.first && signed_mul.output_valid ==True) 
    begin
      temp_out <= default_out;
      temp_out_valid <= True;    
    end

    if (ff_ordering.first) // mul operation
    begin 
      if (signed_mul.output_valid == True) 
      begin
        _y = default_out;
        tx_mbox_out.u.enq(_y);
        `ifdef arith_trap
          tx_arith_trap_out.u.enq(unpack(0));
        `endif
        ff_ordering.deq;
        `logLevel( mbox, 0, $format("MBOX: Collecting recent MUL o/p"), wr_simulate_log_start)
      end
      else if (temp_out_valid) 
      begin
        _y = temp_out;
        tx_mbox_out.u.enq(_y);
        `ifdef arith_trap
          tx_arith_trap_out.u.enq(unpack(0));
        `endif
        ff_ordering.deq;
        temp_out_valid <= False;
        `logLevel( mbox, 0, $format("MBOX: Collecting old MUL o/p"), wr_simulate_log_start)
      end
      else
        `logLevel( mbox, 0, $format("MBOX: Waiting for MUL o/p"), wr_simulate_log_start)
    end
    else if (!ff_ordering.first) // div operation
    begin 
        if (div_.mv_output_valid) 
        begin
          let _x <- div_.mv_output;
          tx_mbox_out.u.enq(_x);
          `ifdef arith_trap
        //   tx_arith_trap_out.u.enq(div_.mv_arith_trap_out);
          tx_arith_trap_out.u.enq(unpack(0));
          `endif
          ff_ordering.deq;
          `logLevel( mbox, 0, $format("MBOX: Collecting DIV o/p"), wr_simulate_log_start)
        end
      else
        `logLevel( mbox, 0, $format("MBOX: Waiting for Div o/p"), wr_simulate_log_start)
    end


    //if (ff_ordering.first) begin // mul operation
    //  if (mul_.mv_output_valid) begin
    //    let _x <- mul_.mv_output;
    //    tx_mbox_out.u.enq(_x);
    //    `ifdef arith_trap
    //      tx_arith_trap_out.u.enq(unpack(0));
    //    `endif
    //    ff_ordering.deq;
    //    `logLevel( mbox, 0, $format("MBOX: Collecting MUL o/p"), wr_simulate_log_start)
    //  end
    //  else
    //    `logLevel( mbox, 0, $format("MBOX: Waiting for Mul o/p"), wr_simulate_log_start)
    //end
    //else if (!ff_ordering.first) begin // div operation
    //  if (div_.mv_output_valid) begin
    //    let _x <- div_.mv_output;
    //    tx_mbox_out.u.enq(_x);
    //    `ifdef arith_trap
    //      tx_arith_trap_out.u.enq(div_.mv_arith_trap_out);
    //    `endif
    //    ff_ordering.deq;
    //    `logLevel( mbox, 0, $format("MBOX: Collecting DIV o/p"), wr_simulate_log_start)
    //  end
    //  else
    //    `logLevel( mbox, 0, $format("MBOX: Waiting for Div o/p"), wr_simulate_log_start)
    //end
  endrule: rl_capture_output


  method Action ma_inputs(MBoxIn inputs);
  `ifdef ASSERT
    dynamicAssert(ff_ordering.notFull(), "Enquing MBOX inputs to full fifo");
  `endif
    if( inputs.funct3[2] == 0 ) // Multiplication ops
    begin 
      Bit#(1) sign1 = inputs.funct3[1]^inputs.funct3[0];
      Bit#(1) sign2 = pack(inputs.funct3[1 : 0] == 1);
      //rg_valid <= True;
      rg_fn3 <= unpack(|inputs.funct3[1 : 0]);

      `ifdef RV64
        rg_word <= inputs.wordop;
      `endif

      signed_mul.in({unpack({sign1 & inputs.in1[valueOf(XLEN) - 1], inputs.in1}),unpack({sign2 & inputs.in2[valueOf(XLEN) - 1], inputs.in2})});
      `logLevel( mbox, 0, $format("MBOX: To MUL. Op1:%h Op2:%h ", inputs.in1, inputs.in2 ), wr_simulate_log_start)
      ff_ordering.enq(True);
    end
    if (inputs.funct3[2] == 1) // Division ops
    begin
      ff_ordering.enq(False);
      `logLevel( mbox, 0, $format("MBOX: To DIV. Op1:%h Op2:%h sign:%b", inputs.in1, inputs.in2, inputs.in1[valueOf(`xlen)-1] ), wr_simulate_log_start)
       div_.ma_inputs( inputs.in1, inputs.in2, inputs.funct3 `ifdef RV64 ,inputs.wordop `endif ) ;
    end
  endmethod


	//method Action ma_inputs(MBoxIn inputs);
  //`ifdef ASSERT
  //  dynamicAssert(ff_ordering.notFull(), "Enquing MBOX inputs to full fifo");
  //`endif
  //  if( inputs.funct3[2] == 0 ) begin // Multiplication ops
  //    `logLevel( mbox, 0, $format("MBOX: To MUL. Op1:%h Op2:%h ", inputs.in1, inputs.in2 ), wr_simulate_log_start)
  //    mul_.ma_inputs(inputs.in1, inputs.in2, inputs.funct3 `ifdef RV64 ,inputs.wordop `endif );
  //    ff_ordering.enq(True);
  //  end
  //  if (inputs.funct3[2] == 1) begin
  //    ff_ordering.enq(False);
  //    `logLevel( mbox, 0, $format("MBOX: To DIV. Op1:%h Op2:%h sign:%b", inputs.in1, inputs.in2, inputs.in1[valueOf(`xlen)-1] ), wr_simulate_log_start)
  //    div_.ma_inputs( inputs.in1, inputs.in2, inputs.funct3 `ifdef RV64 ,inputs.wordop `endif ) ;
  //  end
  //endmethod
  method mv_ready= MBoxRdy{mul: ff_ordering.notFull, div: div_.mv_ready && ff_ordering.notFull()};

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


