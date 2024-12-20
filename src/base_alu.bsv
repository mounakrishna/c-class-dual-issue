//See LICENSE.iitm for license details
/*
Author: IIT Madras
Created on: Saturday 12 June 2021 01:49:20 PM

*/
/*doc:overview:
This package includes multiple functions for carrying ops from the base RISC-V ISA.

compile-macros:
- base_alu_noinline: When set, causes each function to be synthesized as a separate verilog file.
*/
package base_alu ;
  import FIFOF        :: * ;
  import Vector       :: * ;
  import SpecialFIFOs :: * ;
  import FIFOF        :: * ;
  import BUtils       :: * ;

  import ccore_types  :: * ;

  `include "Logger.bsv"
  `include "decoder.defines"

`ifdef base_alu_noinline
  (*noinline*)
`endif
  /*doc:func: This function performs the branch comparison operations using adders.*/
  function Bit#(1) fn_bru2 (Bit#(`xlen) op1, Bit#(`xlen) op2, Bit#(4) fn);
    Bit#(`xlen) inv = signExtend(fn[3]);
    let inv_op2 = op2^inv;
    let op1_xor_op2 = op1^inv_op2;
    let adder_output = op1 + inv_op2 + zeroExtend(fn[3]);
    Bit#(1) compare_out = fn[0]^(
            (fn[3] == 0) ? pack(op1_xor_op2 == 0):
            (op1[valueOf(`xlen) - 1] == op2[valueOf(`xlen) - 1]) ? adder_output[valueOf(`xlen) - 1]:
            (fn[1] == 1) ? op2[valueOf(`xlen) - 1] : op1[valueOf(`xlen) - 1]);
    return compare_out;
  endfunction: fn_bru2

`ifdef base_alu_noinline
  (*noinline*)
`endif
  /*doc:func: this function performs the branch comparison operations using the less than
  * comparator. this is preferred over the bru2*/
  function Bit#(1) fn_bru (Bit#(`xlen) op1, Bit#(`xlen) op2, Bit#(4) fn);
    Bit#(`xlen) op1_xor_op2 = op1 ^ op2;
    Bit#(1) sign = ~fn[1];
    Bit#(1) adder_z_flag = ~|op1_xor_op2;
    Int#(TAdd#(`xlen, 1)) a = unpack({sign & op1[valueOf(`xlen)-1], op1});
    Int#(TAdd#(`xlen, 1)) b = unpack({sign & op2[valueOf(`xlen)-1], op2});
    Bool less = a < b;
    case (fn)
      `FNSEQ : return adder_z_flag;
      `FNSNE : return ~adder_z_flag;
      `FNSLT, `FNSLTU : return pack(less);
      default: return pack(!less);
    endcase
  endfunction:fn_bru

`ifdef base_alu_noinline
  (*noinline*)
`endif
  /*doc:func: this function peforms the add and subtract operation*/
  function Bit#(`xlen) fn_add ( Bit#(`xlen) op1, Bit#(`xlen) op2, Bit#(1) sub);
    Bit#(TAdd#(`xlen, 1)) inv = duplicate(pack(sub));
    let inv_op2 = {op2,1'b0}^inv;
    return truncateLSB({op1,1'b1} + inv_op2);
  endfunction: fn_add

`ifdef base_alu_noinline
  (*noinline*)
`endif
  /*doc:func: This function peforms the set less operations using regular less than comparator*/
  function Bit#(1) fn_compare (Bit#(`xlen) op1, Bit#(`xlen) op2, Bit#(4) fn, Bit#(`xlen) op1_xor_op2);
    Bit#(1) sign = ~fn[1];
    Bit#(1) adder_z_flag = ~|op1_xor_op2;
    Int#(TAdd#(`xlen, 1)) a = unpack({sign & op1[valueOf(`xlen)-1], op1});
    Int#(TAdd#(`xlen, 1)) b = unpack({sign & op2[valueOf(`xlen)-1], op2});
    Bool less = a < b;
    return pack(less);
  endfunction: fn_compare

  /*doc:func: */
`ifdef base_alu_noinline
  (*noinline*)
`endif
  /*doc:func: This function performs all the shift operations of the base ISA. wordop argument is
   * available only in rv64 mode. When wordop is set to true, it indicates a 32-bit shift, so only
   * the lower 5 bits of op2 will be used to deduce the shift amount. 
   * in this module we implement only a single left-barrel-shifter. For right shifts, we reverse op1
   * before the left-shift and then reverse the output again.
  */
  function Bit#(`xlen) fn_shift (Bit#(`xlen) op1, Bit#(TLog#(`xlen)) op2, Bit#(4) fn 
                      `ifdef RV64 , Bool wordop `endif );
  `ifdef RV64
	  Bit#(6) shift_amt={((!wordop) ? op2[5] : 0), op2[4 : 0]};
		Bit#(TDiv#(`xlen, 2)) upper_bits = wordop ? signExtend(fn[3] & op1[31]) : op1[63 : 32];
		Bit#(`xlen) shift_inright={upper_bits, op1[31 : 0]};//size of 64 bit
  `else
    Bit#(5) shift_amt = op2[4 : 0];
    Bit#(`xlen) shift_inright = zeroExtend(op1[31 : 0]);//size of 32bit
  `endif
	  let shin = (fn==`FNSR || fn==`FNSRA) ? shift_inright : reverseBits(shift_inright);
	  Int#(TAdd#(`xlen, 1)) t = unpack({(fn[3] & shin[valueOf(`xlen) - 1]), shin});
	  Int#(`xlen) shift_r = unpack(pack(t>>shift_amt)[valueOf(`xlen) - 1 : 0]);//shift right by shift_amt
	  let shift_l = reverseBits(pack(shift_r));//shift left
	  case (fn)
	    `FNSR, `FNSRA: return pack(shift_r);
	    default: return pack(shift_l);
	  endcase
  endfunction: fn_shift

`ifdef base_alu_noinline
  (*noinline*)
`endif
  /*doc:func: This function performs the logic operations as defined by the base ISA spec. The XOR
  * value if provided as an argument. This is done because the same XOR operation is being used by the
  * compare operations as well.*/
  function Bit#(`xlen) fn_logic (Bit#(`xlen) op1, Bit#(`xlen) op2, Bit#(4) fn, Bit#(`xlen) op1_xor_op2);
    case (fn)
      `FNOR: return (op1 | op2);
      `FNAND: return (op1 & op2);
      default: return op1_xor_op2;
    endcase
  endfunction: fn_logic

`ifdef base_alu_noinline
  (*noinline*)
`endif
  /*doc:func: This is the top level base-alu function which calls the above individual functions
  * depending on the opcode provided as inputs*/
  function Bit#(`xlen) fn_base_alu (Bit#(`xlen) op1, Bit#(`xlen) op2, Bit#(4) fn
                          , Bit#(`xlen) pc , Bool op1pc `ifdef RV64 , Bool wordop `endif );
    let op1_xor_op2 = op1 ^ op2;
    let lv_add = fn_add(op1pc?pc:op1, op2, fn[1]);
    let less = fn_compare(op1, op2, fn, op1_xor_op2);
    Bit#(`xlen) lv_shiftout = fn_shift(op1, truncate(op2), fn `ifdef RV64 , wordop `endif );
    let lv_logic = fn_logic(op1, op2, fn, op1_xor_op2);
    Bit#(`xlen) aluout = case (fn) 
      `FNADD, `FNSUB: lv_add;
      `FNSLT, `FNSLTU: zeroExtend(pack(less));
      `FNSR, `FNSRA, `FNSL: lv_shiftout;
      default: lv_logic;
    endcase;
    return aluout;
  endfunction: fn_base_alu
endpackage: base_alu

