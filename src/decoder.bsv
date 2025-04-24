//See LICENSE.iitm for license details
/*

Author: IIT Madras
Details:

This module primarily holds the combo functions to decode the instructions and provide
various meta data to fetch operands and execute on them.

The module also contains functions to check if a particular csr access is valid or illegal

Interrupt checks are also performed in this package.

The decoder outputs minimal data required to peform operand fetch and executions in the later stage.

Following table describes what the ALU will need for some critical operations. Based on this
the next set of logic is implemented. rs1+ rs2 is a XLEN bit adder. rs3+ rs4 is `paddr bit
adder.
Now PC can be present either in rs1 or rs3. This has been done to reduce the mux to the ALU
in the next stage. There will only be a mux in the next stage to identify the PC and send it
to the next stage.

         rs1   rs2   rs3   rs4
Branch   OP1   OP2   PC    Imm
JAL      PC    'd4   PC    Imm   (rs1=0, rs2=0 since neither required)
JALR     PC    'd4   op1   Imm   (rs2=0 since not required)
LOAD     PC    op2   op1   Imm   (rs2=0 since not required) // PC needs to be sent as well
STORE    PC    op2   op1   Imm   (both required. op2 is the data)
AUIPC    PC    Imm   PC    Imm   (rs1=0, rs2=0 since neither required)
Atomic   PC    op1   op1    0

--------------------------------------------------------------------------------------------------
*/
package decoder;

  // pacakge imports from project
  import ccore_types::*;
  import BUtils::*;
 
  import csr_types :: *;
  import csrbox_decoder :: * ;
  `include "decoder.defines"
  `include "trap.defines"
  `include "csrbox.defines"

  `ifdef decoder_noinline
  (*noinline*)
  `endif
  function Bool hasCSRPermission(Bit#(12) address, Bool write,  Privilege_mode prv
  `ifdef hypervisor ,Bit#(1) v `endif );
    Bit#(12) csr_index = pack(address);
    Privilege_mode _prv = prv;
  `ifdef hypervisor
    if (_prv == Supervisor && v==0) _prv = Hypervisor;
  `endif
    return ((pack(_prv) >= csr_index[9:8]) && !(write && csr_index[11:10]==2'b11) );
  endfunction

  // if the operand is not 0 then the instruction will perform a write on the CSR.
  `ifdef decoder_noinline
  (*noinline*)
  `endif
	function Bool valid_csr_access(Bit#(12) csr_addr, Bit#(5) operand, Bit#(2) operation,
                                  Bit#(1) tvm, Privilege_mode prv
                                  `ifdef hypervisor ,Bit#(1) v `endif );
		Bool ret = hasCSRPermission(unpack(csr_addr), (operand != 0 || operation=='b01) ? True:False, prv `ifdef hypervisor ,v `endif );

    // accessing satp in supervisor mode with tvm=1 should raise an illegal exception
  `ifdef supervisor
    if ( ret && csr_addr == 'h180 && tvm == 1 && prv == Supervisor)
      ret = False;
  `endif
		return ret;
	endfunction

  `ifdef decoder_noinline
  (*noinline*)
  `endif
	function Tuple2#(Bit#(`causesize), Bool) chk_interrupt(
	                                                        Privilege_mode prv,
	                                                        Bit#(`xlen) mstatus,
                                                          Bit#(`xlen) sstatus,
                                                          Bit#(TAdd#(`max_int_cause,1)) mip,
                                                          Bit#(TAdd#(`max_int_cause,1)) mie
                                                        `ifdef non_m_traps
                                                          ,Bit#(TAdd#(`max_int_cause,1)) mideleg
                                                        `endif
                                                        `ifdef supervisor `ifdef usertraps
                                                          ,Bit#(TAdd#(`max_int_cause,1)) sideleg
                                                        `endif `endif
                                                        `ifdef debug
                                                          ,DebugStatus debug, Bool step_done
                                                        `endif 
                                                        `ifdef hypervisor
                                                          , Bit#(1) vs_bit 
                                                          , Bit#(12) hideleg
                                                        `endif );



    Bool m_enabled = (prv != Machine) || (mstatus[3]==1);
  `ifdef supervisor
    Bool s_enabled = (prv == User) || (sstatus[1]==1 && prv==Supervisor);
  `endif
  `ifdef usertraps
    Bool u_enabled = (mstatus[0]==1 && prv==User);
  `endif
   
    Bit#(TAdd#(`max_int_cause,1)) d_interrupts = 0;
    Bit#(TAdd#(`max_int_cause,1)) hs_interrupts = 0;
    Bit#(TAdd#(`max_int_cause,1)) m_interrupts = 0;
    Bit#(TAdd#(`max_int_cause,1)) s_interrupts = 0;
    Bit#(TAdd#(`max_int_cause,1)) u_interrupts = 0;


  `ifdef debug
    Bool d_enabled = debug.debugger_available && debug.core_debugenable;
    d_interrupts = { mip[16],16'd0} & signExtend(pack(d_enabled)) &
    signExtend(~pack(debug.debug_mode));
  `endif

    // truncating because in debug mode mie and mip are 14 bits. 12-halt-req 13-resume-req
    m_interrupts =                mie & mip & signExtend(pack(m_enabled))
             `ifdef non_m_traps & ~zeroExtend(mideleg) `endif
             `ifdef debug       & signExtend(pack(!debug.debug_mode)) `endif ;
  `ifdef hypervisor
    Bool hs_enabled = vs_bit == 1 || s_enabled;
    hs_interrupts = mie & mip & signExtend(pack(hs_enabled)) & 
                    (zeroExtend(mideleg) & ~zeroExtend(hideleg)) ;
  `endif
  `ifdef supervisor
    s_interrupts =              mie & mip & zeroExtend(mideleg) & signExtend(pack(s_enabled))
               `ifdef hypervisor & zeroExtend(hideleg) & signExtend(vs_bit) `endif
               `ifdef usertraps & ~zeroExtend(sideleg) `endif
               `ifdef debug     & signExtend(pack(!debug.debug_mode)) `endif ;
  `endif
  `ifdef usertraps
    u_interrupts =                mie & mip & zeroExtend(mideleg) & signExtend(pack(u_enabled))
              `ifdef supervisor & ~zeroExtend(sideleg) `endif
              `ifdef debug      & signExtend(pack(!debug.debug_mode)) `endif ;
  `endif

    Bit#(TAdd#(`max_int_cause,1)) pending_interrupts = d_interrupts | m_interrupts | s_interrupts 
                                                      | u_interrupts 
                                    `ifdef hypervisor | hs_interrupts `endif ;
		// format pendingInterrupt value to return
    Bool taketrap=unpack(|pending_interrupts) `ifdef debug ||  (step_done && !debug.debug_mode) `endif ;

    Bit#(TSub#(`causesize, 1)) int_cause='1;
    Bit#(1) mode = 1;
  `ifdef debug
    if(step_done && !debug.debug_mode) begin
      int_cause = `halt_step;
      mode = 0;
    end
    else if(pending_interrupts[16] == 1)
      int_cause = `debug_interrupt;
    else
  `endif
    if(pending_interrupts[11]==1)
      int_cause=`Machine_external_int;
    else if(pending_interrupts[3]==1)
      int_cause=`Machine_soft_int;
    else if(pending_interrupts[7]==1)
      int_cause=`Machine_timer_int;
  `ifdef perfmonitors
    else if(pending_interrupts[16] == 1)
      int_cause = `CounterInterrupt;
  `endif
  `ifdef supervisor
    else if(pending_interrupts[9]==1)
      int_cause=`Supervisor_external_int;
    else if(pending_interrupts[1]==1)
      int_cause=`Supervisor_soft_int;
    else if(pending_interrupts[5]==1)
      int_cause=`Supervisor_timer_int;
  `endif
  `ifdef hypervisor
    else if (pending_interrupts[12] == 1)
      int_cause = `Supervisor_guest_ext_int;
    else if (pending_interrupts[10] == 1)
      int_cause = `VS_ext_int;
    else if (pending_interrupts[2] == 1)
      int_cause = `VS_soft_int;
    else if (pending_interrupts[6] == 1)
      int_cause = `VS_timer_int;
  `endif
  `ifdef user
    else if(pending_interrupts[8]==1)
      int_cause=`User_external_int;
    else if(pending_interrupts[0]==1)
      int_cause=`User_soft_int;
    else if(pending_interrupts[4]==1)
      int_cause=`User_timer_int;
  `endif


		return tuple2({mode,int_cause}, taketrap);
	endfunction


  typedef enum {Q0='b00, Q1='b01, Q2='b10} Quadrant deriving(Bits,Eq,FShow);

  `ifdef decoder_noinline
  (*noinline*)
  `endif
  /*doc:func: This function decodes the rs1 addr of the instruction. It is set to 0 for instructions
  * that do not require rs1 operand*/
  function Bit#(5) fn_decode_rs1(Bit#(32) inst );
    case (inst[6:2]) matches
       `LUI_op : return 0;
       `AUIPC_op : return 0;
       `JAL_op : return 0 ;
       `SYSTEM_op : `ifdef hypervisor if (inst[14:12] == 4) return inst[19:15]; else `endif return (inst[14]==1)?0:inst[19:15];
       default: return inst[19:15] ;
     endcase
  endfunction:fn_decode_rs1
 
  `ifdef decoder_noinline
  (*noinline*)
  `endif
  /*doc:func: This function decodes the type of rs1 required by the operation. 
  * It can be one of : PC, FRF or IRF*/
  function Op1type fn_decode_rs1type(Bit#(32) inst );
    case (inst[6:2]) matches
      `JAL_op: return PC ;
      `JALR_op: return PC ;
      `AUIPC_op: return PC ;
    `ifdef spfpu
      `R4_op: return FloatingRF;
      `FN_op: if (inst[31:28] != 13 && inst[31:28] != 15 ) return FloatingRF; else return IntegerRF;
    `endif
      default: return IntegerRF ;
    endcase
  endfunction:fn_decode_rs1type
 
  `ifdef decoder_noinline
  (*noinline*)
  `endif
  /*doc:func: This function decodes the address of the rs2 operand. It is set to 0 for those
   * instructions which do not require rs2 operand at all.*/
  function Bit#(5) fn_decode_rs2(Bit#(32) inst, CSRtoDecode csrs );
		Bit#(3) funct3 = inst[14:12];
		Bit#(7) funct7 = inst[31:25];
    case (inst[6:2]) matches
      `JAL_op: return 0 ;
      `JALR_op: return 0 ;
      `LUI_op: return 0 ;
      `AUIPC_op : return 0 ;
      `LOAD_op: return 0;
      `SYSTEM_op: return `ifdef hypervisor (funct3 == 4 && inst[25]==1)?inst[24:20]: `endif 
                                (funct3!=0)?0: (funct7=='b0001001 || funct7 == 'b0010001 || funct7 == 'b0110001)?inst[24:20]: 0;
      `IMM_ARITH_op: return 0;
    `ifdef spfpu
      `FN_op: return (funct7[5]==1)?0:inst[24:20];
    `endif
       default: return inst[24:20] ;
    endcase
  endfunction:fn_decode_rs2
 
  `ifdef decoder_noinline
  (*noinline*)
  `endif
  /*doc:func: This function decodes the type of the rs2 operand. If can be one of : Constant2,
   * Constant4, Immediate, FRF or IRF*/
  function Op2type fn_decode_rs2type(Bit#(32) inst `ifdef compressed , Bool compressed `endif );
		Bit#(3) funct3 = inst[14:12];
		Bit#(7) funct7 = inst[31:25];
    case (inst[6:2]) matches
      `JAL_op: `ifdef compressed if(compressed) return Constant2; else `endif return Constant4;
      `JALR_op: `ifdef compressed if(compressed) return Constant2; else `endif return Constant4;
      `FENCE_op: `ifdef compressed if(compressed) return Constant2; else `endif return Constant4;  
      `LUI_op: return Immediate;
      `IMM_ARITH_op: return Immediate;
    `ifdef spfpu
      `R4_op: return FloatingRF;
      `FN_op: if (funct7[5]!=1)return FloatingRF; else return IntegerRF;
      `FSTORE_op: return FloatingRF;
    `endif
       default: return IntegerRF ;
    endcase
  endfunction:fn_decode_rs2type
 
  `ifdef decoder_noinline
  (*noinline*)
  `endif
  /*doc:func: This function is used to decode the destination register address from the
   * instruction. Instructions like branch and store do not update any register and thus the values
  * are assigned 0 for these instructions*/
  function Bit#(5) fn_decode_rd(Bit#(32) inst );
    case (inst[6:2]) matches
      `BRANCH_op: return 0 ;
      `STORE_op: return 0 ;
      `FSTORE_op: return 0 ;
       default: return inst[11:7] ;
    endcase
  endfunction:fn_decode_rd
 
  `ifdef decoder_noinline
  (*noinline*)
  `endif
  /*doc:func: This function is used to capture the immediate values from the instruction. In case of
  * CSR operations we capture the immediate value and the csr address in the same field. For atomics
  * we set the immediate to 0 so that we can re-use the same address generator for memory operations*/
  function Bit#(32) fn_decode_immediate(Bit#(32) inst, CSRtoDecode csrs );
    //---------------- Decoding the immediate values-------------------------------------

		Bit#(5) opcode= inst[6:2];
		Bit#(7) funct7 = inst[31:25];
		Bit#(3) funct3 = inst[14:12];
    // Identify the type of intruction first
    Bool stype= (opcode=='b01000 || (opcode=='b01001 && csrs.csr_misa[5]==1) `ifdef hypervisor || (csrs.csr_misa[7]==1 && funct7[6:3] =='b0110 && funct7[0]==1 && funct3[2:0] =='b100 && inst[11:7]==0 && opcode=='b11100) `endif );
    Bool btype= (opcode=='b11000);
    Bool utype= (opcode=='b01101 || opcode=='b00101);
    Bool jtype= (opcode=='b11011);
    Bool r4type= (opcode[4:2]=='b100);
    Bool atomictype=(opcode=='b01011);
    Bool systemtype = (opcode == `SYSTEM_op);

    // refer to section 2.3 (Immediate Encoding Variants) of the risc-v iser spec for more details
    // on the following logic.
    // The default values are chosen such that in case of FPU,  the immediate encoding will hold the
    // upper 7-bit for further decoding.
    // The default values also enable capturing the encoding for atomic operations as well.
    Bit#(1) bit0 = inst[20]; // because of I-type instructions
    `ifdef atomic
      if(atomictype) bit0=0; else
    `endif
    if(stype)
      bit0=inst[7];
    else if(btype || utype || jtype)
      bit0=0;

    Bit#(4) bit1_4=inst[24:21]; // I/J-type instructions
    `ifdef atomic
      if(atomictype) bit1_4=0; else
    `endif
    if(stype || btype) // S/B-Type
      bit1_4=inst[11:8];
    else if(utype) // U type
      bit1_4=0;

    Bit#(6) bit5_10=inst[30:25];
    `ifdef atomic
      if(atomictype) bit5_10=0; else
    `endif
    if(utype)
      bit5_10=0;

    Bit#(1) bit11 = inst[31]; // I/S type
    `ifdef atomic
      if(atomictype) bit11=0; else
    `endif
    if(btype)
      bit11=inst[7];
    else if(utype)
      bit11=0;
    else if(jtype)
      bit11=inst[20];

    Bit#(8) bit12_19=duplicate(inst[31]); // I/S/B type
    `ifdef atomic
      if(atomictype) bit12_19=0; else
    `endif
    if(utype || jtype || systemtype)
      bit12_19=inst[19:12];

    Bit#(11) bit20_30=duplicate(inst[31]); // I/B/S/J type
    `ifdef atomic
      if(atomictype) bit20_30=0; else
    `endif
    if(utype)
      bit20_30=inst[30:20];
    Bit#(1) bit31= `ifdef atomic (atomictype)?0: `endif inst[31];
    Bit#(32) immediate_value={bit31, bit20_30, bit12_19, bit11, bit5_10, bit1_4, bit0};
    return immediate_value;
    /*case (inst) matches
      `S_TYPE: return {duplicate(inst[31]), inst[30:25] ,inst[11:8] ,inst[7]} ;
      `FSTORE_TYPE: if (csrs.csr_misa[5]==1) return {duplicate(inst[31]), inst[30:25] ,inst[11:8] ,inst[7]} ; else return 0;
      `B_TYPE: return {duplicate(inst[31]), inst[7], inst[30:25], inst[11:8], 1'b0};
      `U_TYPE: return { inst[31], inst[30:20], inst[19:12], 12'b0};
      `J_TYPE: return { duplicate(inst[31]), inst[19:12], inst[20], inst[30:25], inst[24:21], 1'b0};
    `ifdef atomic
      `A_TYPE: return 32'b0;
    `endif
    `ifdef zicsr
      `CSR_INSTR: return {?,inst[19:15],inst[31:20]};
    `endif
      default: return {duplicate(inst[31]), inst[30:25] ,inst[24:21] ,inst[20]} ;
    endcase*/
  endfunction:fn_decode_immediate
 
  `ifdef decoder_noinline
  (*noinline*)
  `endif
  /*doc:func: This function if used to captuer the nature of the memory access that needs to be
  * performed. It can be one of : Load, Store, Fence, FenceI and Atomic.*/
  function Access_type fn_decode_mem_access(Bit#(32) inst);
		Bit#(3) funct3 = inst[14:12];
		Bit#(7) funct7 = inst[31:25];
    case(inst[6:2]) matches
      `STORE_op: return Store;
      `FSTORE_op: return Store;
      `FENCE_op: if (funct3[0]==1) return FenceI; else return Fence;
    `ifdef atomic
      `ATOMIC_op: return Atomic; 
    `endif
    `ifdef supervisor
      `SYSTEM_op: if (funct7=='b0001001 && funct3==0) 
										return SFence;
							  `ifdef hypervisor
									else if(funct7=='b0010001 && funct3==0) //HFENCE.VVMA
						      	return HFence_VVMA;
							    else if(funct7=='b0110001 && funct3==0) 						// HFENCE.GVMA
      							return HFence_GVMA;
      						else if ((funct7 == 'b0110111 || funct7 == 'b0110101 || funct7 == 'b0110011 || funct7 == 'b0110001) &&
                						funct3 == 'b100)
                		return Store;
						    `endif
									else return Load;
    `endif
      default: return Load;
    endcase
  endfunction:fn_decode_mem_access
     
`ifdef spfpu
  `ifdef decoder_noinline
  (*noinline*)
  `endif
  /*doc:func: This function is used to capture the type of registerfile the destination should be
   * written to. This function is only enabled when floating point support is enabled at compile
   * time*/
  function RFType fn_decode_rdtype(Bit#(32) inst );
    case (inst) matches
      `FLOAD_INSTR: return FRF ;
      `FN_INSTR: if(inst[31:28] != 10 && inst[31:28] != 12 && inst[31:28] != 14) return FRF ; else return IRF;
      `R4_TYPE: return FRF;
      default: return IRF ;
    endcase
  endfunction:fn_decode_rdtype
 
  `ifdef decoder_noinline
  (*noinline*)
  `endif
  /*doc:func: This function decodes the rs3 operand used for fused floating point ops*/
  function Bit#(5) fn_decode_rs3(Bit#(32) inst);
    case(inst) matches
      `R4_TYPE: return inst[31:27];
      default: return 0;
    endcase
  endfunction:fn_decode_rs3
 
  `ifdef decoder_noinline
  (*noinline*)
  `endif
  /*doc:func: This function decodes the registerfile type of rs3 operand*/
  function RFType fn_decode_rs3type(Bit#(32) inst);
    case(inst) matches
      `R4_TYPE: return FRF;
      default: return IRF;
    endcase
  endfunction:fn_decode_rs3type
`endif

  `ifdef decoder_noinline
  (*noinline*)
  `endif
  /*doc:func: This function decodes the type of instruction. Default the instruction type is TRAP.
   * Only when a legal instruction is encountered the type changes accordingly*/
  function Instruction_type fn_decode_insttype(Bit#(32) inst, CSRtoDecode csrs
                            `ifdef debug ,DebugStatus debug `endif );
    Bit#(1) fs = |csrs.csr_mstatus[14:13];
		Bit#(3) funct3= inst[14:12];
		Bit#(7) funct7= inst[31:25];
  `ifdef spfpu
    Bit#(3) frm = csrs.frm;
    Bool valid_rounding = (funct3=='b111)?(frm!='b101 && frm!='b110 && frm!='b111):(funct3!='b101 && funct3!='b110);
    Bool address_is_valid=address_valid(inst[31:20],csrs.csr_misa,fs);
   `else
    Bool address_is_valid=address_valid(inst[31:20],csrs.csr_misa);
   `endif
  	Bool access_is_valid=valid_csr_access(inst[31:20],inst[19:15], inst[13:12], 
																					csrs.csr_mstatus[20], csrs.prv
																				`ifdef hypervisor ,csrs.csr_vs_bit `endif );
    case (inst) matches
      `LUI_INSTR		         :return ALU;
      `AUIPC_INSTR		       :return ALU;
      `JAL_INSTR		         :return JAL;
      `JALR_INSTR		         :return JALR;
      `BRANCH_INSTR          : if (funct3 != 'b011 && funct3 != 'b010) return BRANCH; else return TRAP;
      `LOAD_INSTR            :if (funct3 != 7 `ifdef RV32 && funct3 != 3 && funct3 !=6 `endif ) return MEMORY; else return TRAP;
      `STORE_INSTR           : `ifdef RV32 if (inst[13:12] != 3) return MEMORY; else return TRAP; `else return MEMORY; `endif
      `ARITHIMM_INSTR        :if (funct3 == 1)  // SLLI
                                if (inst[31:26]==0 `ifdef RV32 && inst[25]==0 `endif ) return ALU; else return TRAP;
                              else if (funct3 == 5) // SR*I
                                if (inst[31:26] matches 'b0?0000 `ifdef RV32 &&& inst[25]==0 `endif ) return ALU; else return TRAP;
                              else return ALU;
      `ARITH_INSTR           :if (inst[30]==1)
                                if (funct3==0 || funct3==5) return ALU; else return TRAP;
                              else return ALU;
      `FENCE_INSTR		       :return MEMORY;
      `ECALL_INSTR           :return TRAP;
      `EBREAK_INSTR          :return TRAP;
    `ifdef RV64
      `ARITHIMM32_INSTR      : if(funct3==0 || (funct3==1 && inst[31:25]==0) || (funct3 == 5 && inst[31]==0 && inst[29:25] == 0 )) return ALU; else return TRAP;
      `ARITH32_INSTR         : if(funct3==0 || funct3==5 || (funct3==1 && inst[30]==0)) return ALU; else return TRAP;
    `endif
    `ifdef ifence
      `FENCEI_INSTR	         :return MEMORY;
    `endif
      `CSR_INSTR             :if (funct3==0) begin
                                if (inst[31:20]=='b000000000010 && inst[19:7]==0 && csrs.csr_misa[13]==1) // URET
                                  return SYSTEM_INSTR;
                                else if (inst[31:20]== 'b000100000010 && inst[19:7]==0 && csrs.csr_misa[18]==1) begin
                                  if (csrs.prv == Machine || (csrs.prv == Supervisor && csrs.csr_mstatus[22]==0 `ifdef hypervisor && csrs.csr_vs_bit == 0 )
                                                         || (csrs.prv == Supervisor && csrs.csr_hstatus[22] == 0 && csrs.csr_vs_bit == 1 `endif ) )
                                    return SYSTEM_INSTR;
                                  else
                                    return TRAP;
                                end
                                else if (inst[31:20] == 'b001100000010 && inst[19:7]==0 &&  csrs.prv == Machine) // MRET
                                  return SYSTEM_INSTR;
                                else if (inst[31:20] == 'b00100000101 && inst[19:7]==0 && (csrs.prv==Machine || csrs.csr_mstatus[21]==0)) //WFI
                                  return WFI;
                              `ifdef hypervisor // HFENCE
                                else if (inst[31] == 0 && inst[29:25] == 'b10001 && inst[11:7] == 0) begin
                                  if (csrs.prv == Machine || (csrs.prv == Supervisor && csrs.csr_vs_bit == 0 && csrs.csr_mstatus[20] == 0))
                                    return MEMORY;
                                  else
                                    return TRAP;
                                end
                              `endif
                              `ifdef supervisor
                                else if (inst[31:25] == 'b001001 && inst[14:7]== 0 && 
                                        (csrs.csr_mstatus[20]==0 || csrs.prv == Machine)) // SFENCE
                                  `ifdef hypervisor
                                    if (csrs.prv == Supervisor && csrs.csr_vs_bit == 1 && csrs.csr_hstatus[20] == 1) 
                                      return TRAP; 
                                    else
                                  `endif
                                  return MEMORY;
                              `endif
                              `ifdef debug
                                else if (inst[31:20] == 'b011110110010 && debug.debug_mode)// DRET 
                                  return SYSTEM_INSTR;
                              `endif
                                else
                                  return TRAP;
                              end
                            `ifdef zicsr
                              else if(funct3!=4 && address_is_valid && access_is_valid)  // CSR ops
                                return SYSTEM_INSTR;
                            `endif
                            `ifdef hypervisor
                              else if (funct3 == 4) begin // Hypevisor load/store ops
                                if ( (funct7[0] == 0 && funct7[6:3] == 'b0110 && inst[24:22] == 0 
                                      && inst[21:20] !=2 && ( ((inst[21:20]==1) ? (funct7[2:1]!=3) : (inst[21:20]==3) ? (funct7[2:1]==2 || funct7[2:1]==1) : True )) )
                                   || (funct7[0] == 1 && funct7[6:3] == 'b0110 && inst[11:7] == 0) // VStore
                                   )
                                   if (csrs.prv == Machine || (csrs.prv == Supervisor && csrs.csr_vs_bit == 0 ) || (csrs.prv== User && csrs.csr_hstatus[9] == 1))
                                     return MEMORY;
                                   else
                                     return TRAP;
                                else
                                  return TRAP;
                              end
                            `endif
                              else
                                return TRAP;
    `ifdef muldiv
      `MULDIV_INSTR          : if (csrs.csr_misa[12]==1) return MULDIV; else return TRAP;
     `ifdef RV64
      `MULDIV32_INSTR        : if (csrs.csr_misa[12]==1 && (funct3==0 || funct3>3)) return MULDIV; else return TRAP;
     `endif
    `endif
    `ifdef atomic
      `ATOMIC_INSTR        : 
        case(inst[31:27]) 
          'd0, 'd1, 'd3, 'd4, 'd8, 'd12, 'd16, 'd20, 'd24, 'd28: if (csrs.csr_misa[0] == 1) return MEMORY;else return TRAP;
          'd2: if (inst[24:20]==0 && csrs.csr_misa[0] == 1) return MEMORY; else return TRAP;
          default: return TRAP;
        endcase
    `endif
    `ifdef spfpu 
      `FLW_INSTR             : if( csrs.csr_misa[5]==1 && fs !=0) return MEMORY; else return TRAP;
      `FSW_INSTR             : if( csrs.csr_misa[5]==1 && fs !=0) return MEMORY; else return TRAP;

      `FUSED_INSTR           : if( valid_rounding && csrs.csr_misa[5]==1 `ifdef dpfpu && csrs.csr_misa[3] == 1 `endif && fs !=0) return FLOAT; else return TRAP;
      `FLOAT_ARITH           : if( valid_rounding && csrs.csr_misa[5]==1 `ifdef dpfpu && csrs.csr_misa[3] == 1 `endif && fs !=0) return FLOAT; else return TRAP;
      `FSQRTS_INSTR          : if( valid_rounding && csrs.csr_misa[5]==1 && fs !=0) return FLOAT; else return TRAP;
      `FSGN_INSTR            : if( inst[13:12]!=3 && csrs.csr_misa[5]==1 `ifdef dpfpu && csrs.csr_misa[3] == 1 `endif  && fs !=0) return FLOAT; else return TRAP;  
      `FMIN_MAX_INSTR        : if( csrs.csr_misa[5]==1 && fs !=0 `ifdef dpfpu && csrs.csr_misa[3] == 1 `endif ) return FLOAT; else return TRAP;
      `FCMP_INSTR            : if( inst[13:12]!=3 && csrs.csr_misa[5]==1 `ifdef dpfpu && csrs.csr_misa[3] == 1 `endif  && fs !=0) return FLOAT; else return TRAP;  
      `FMV_CLASS_INSTR       : if ( (funct3[0]==0 || funct7[3]==0) && csrs.csr_misa[5]==1 `ifdef dpfpu && csrs.csr_misa[3] == 1 `endif && fs !=0) return FLOAT; else return TRAP;
      `FCVT_INSTR            : if (valid_rounding && csrs.csr_misa[5]==1 && fs !=0 `ifdef dpfpu && (csrs.csr_misa[3] == 1 || funct7[0]==0) `endif  ) return FLOAT; else return TRAP;
    `endif
    `ifdef dpfpu 
      `FLD_INSTR             : if( csrs.csr_misa[3]==1 && csrs.csr_misa[5]==1 && fs !=0) return MEMORY; else return TRAP;
      `FSD_INSTR             : if( csrs.csr_misa[3]==1 && csrs.csr_misa[5]==1 && fs !=0) return MEMORY; else return TRAP;
      `FSQRTD_INSTR          : if( valid_rounding && csrs.csr_misa[5]==1 && csrs.csr_misa[3]==1&& fs !=0) return FLOAT; else return TRAP;
      `FCVT2_INSTR           : if( funct7[0]!=inst[20] && valid_rounding && csrs.csr_misa[5]==1 && csrs.csr_misa[3]==1&& fs !=0) return FLOAT; else return TRAP;
    `endif
    default: return TRAP;
      
    endcase
  endfunction:fn_decode_insttype
 
  `ifdef decoder_noinline
  (*noinline*)
  `endif
  function Bit#(4) fn_decode_fn(Bit#(32) inst, CSRtoDecode csrs);
    Bit#(3) funct3 = inst[14:12];
    Bit#(7) funct7 = inst[31:25];
	  case(inst) matches
	  `ifdef atomic
      `A_TYPE: if((inst[27]|inst[28]) == 1) return {inst[29:27],1'b1}; else return {inst[31:29],inst[27]};
    `endif
      `BRANCH_INSTR: if(inst[14]==0) return {2'b0,1,funct3[0]}; else return {1'b1,funct3};
      `ARITHIMMANY_INSTR : case(funct3)
  				'b010: return 'b1100;
	  			'b011: return 'b1110;
		  		'b101: if(funct7[5]==1) return 'b1011; else return 'b0101;
				  default: return {1'b0, funct3};
        endcase
      `ARITHANY_INSTR: case(funct3)
  				'b000:if(funct7[5]==1) return 'b1010; else  return 'b0000;
	  			'b010:return 'b1100;
		  		'b011:return 'b1110;
			  	'b101:if (funct7[5]==1) return 'b1011;else return 'b0101;
				  default:return {1'b0,funct3};
  			endcase
  	`ifdef spfpu
  		'b?????????????????????????10?????: if ((csrs.csr_misa[5]|csrs.csr_misa[3])==1) return inst[5:2]; else return `FNADD;
  	`endif
//      `R4_TYPE: if ((csrs.csr_misa[5]|csrs.csr_misa[3])==1) return inst[5:2]; else return `FNADD;
      default: return `FNADD;
    endcase
  endfunction
  
  `ifdef decoder_noinline
  (*noinline*)
  `endif
  function Bit#(`causesize) fn_decode_trapcause(Bit#(32) inst, CSRtoDecode csrs
                                    `ifdef debug ,DebugStatus debug `endif );
  `ifdef debug
    Bool ebreakm = unpack(csrs.csr_dcsr[15]) && !debug.debug_mode;
    Bool ebreaks = unpack(`ifdef supervisor csrs.csr_dcsr[13] `else 0 `endif ) && !debug.debug_mode;
    Bool ebreaku = unpack(`ifdef user csrs.csr_dcsr[12] `else 0 `endif ) && !debug.debug_mode;
  `endif
		Bit#(7) funct7 = inst[31:25];
		Bit#(3) funct3 = inst[14:12];
    case (inst) matches
      `ECALL_INSTR: return
        `ifdef user       (csrs.csr_misa[20]==1 && csrs.prv==User)?       `Ecall_from_user:  `endif
        `ifdef supervisor (csrs.csr_misa[18]==1 && csrs.prv==Supervisor)? 
        `ifdef hypervisor (csrs.csr_vs_bit == 1)? `Ecall_from_vs_supervisor: `endif 
                                                 `Ecall_from_supervisor: `endif
                                                                       `Ecall_from_machine;
      `EBREAK_INSTR: `ifdef debug
          if(                   (ebreakm && csrs.prv == Machine)
            `ifdef supervisor || (ebreaks && csrs.prv == Supervisor) `endif
            `ifdef user       || (ebreaku && csrs.prv == User)    `endif ) begin
            Bit#(`causesize) trapcause = `halt_ebreak;
            trapcause[`causesize - 1] = 0;
            return trapcause;
          end
         
          else
        `endif
          return `Breakpoint;
    `ifdef hypervisor
      `CSR_INSTR : begin
        if (funct3 == 0 && (inst[31:20] == 'h102 && inst[19:15] == 0 && inst[11:7] == 0) && 
            (csrs.prv == Supervisor && csrs.csr_vs_bit == 1 && csrs.csr_hstatus[22] == 1) ) // SRET
          return `Virt_inst ;
        else if (funct3 == 0 &&  (inst[31:25] == 'b0001001 && inst[11:7] == 0) && 
            (csrs.prv == Supervisor && csrs.csr_vs_bit == 1 && csrs.csr_hstatus[20] == 1)) // SFENCE
          return `Virt_inst ;
        else if (funct3 == 0 &&  ( inst[31] == 0 && inst[29:25] == 'b10001 && inst[11:7] == 0)  &&
                  (csrs.csr_vs_bit == 1) ) // HFENCE
          return `Virt_inst;
        else if (funct3 == 4 && inst[31:28] == 'b0110 && csrs.csr_vs_bit == 1) // HLV Load/Store
          return `Virt_inst;
        else 
          return `Illegal_inst;
      end
    `endif
      default: return `Illegal_inst ;
    endcase
  endfunction 
 
  `ifdef decoder_noinline
  (*noinline*)
  `endif
  function DecodeOut fn_decode(Bit#(32) inst, CSRtoDecode csrs
                                    `ifdef compressed , Bool compressed `endif
                                    `ifdef debug ,DebugStatus debug `endif );

		Bool word32 =False;

		Bit#(1) hlvx = 0;
		Bit#(1) hvm_loadstore = 0;

		Bit#(5) rs1 = fn_decode_rs1(inst);
		Bit#(5) rs2 = fn_decode_rs2(inst, csrs);
		Bit#(5) rd  = fn_decode_rd(inst) ;

		//operand types
		Op1type rs1type = fn_decode_rs1type(inst);
		Op2type rs2type = fn_decode_rs2type(inst `ifdef compressed , compressed `endif );

  `ifdef spfpu
    Bit#(5) rs3 = fn_decode_rs3(inst);
    RFType rs3type = fn_decode_rs3type(inst);
    RFType rdtype = fn_decode_rdtype(inst);
  `endif

    Bit#(32) immediate_value = fn_decode_immediate(inst, csrs);
    Access_type mem_access = fn_decode_mem_access(inst);
    Bit#(`causesize) trapcause = fn_decode_trapcause(inst, csrs `ifdef debug ,debug `endif );
    Instruction_type inst_type = fn_decode_insttype(inst, csrs `ifdef debug , debug `endif );

    // --------- Function for ALU -------------
    // In case of Atomic operations as well,  the immediate portion will ensure the right opcode is
    // sent to the cache for operations.
		Bit#(4) fn=fn_decode_fn(inst, csrs);
		Bit#(3) funct3= inst[14:12];
  `ifdef spfpu
    if(inst_type==FLOAT && funct3=='b111)
      funct3=csrs.frm;
  `endif

  `ifdef hypervisor
		if (inst[6:0] == 'b1110011 && inst[14:12] == 4 && inst[31:28] == 'b0110) begin
		  hvm_loadstore = 1;
		  immediate_value = 0;
		  funct3 = {inst[20]&~inst[25], inst[27:26]}; // the funct3 defines size of the hvm load/store ops
  		if ( (inst[27:25] == 'b010 || inst[27:25] == 'b100   ) && inst[24:20] == 'b00011 )
  		  hlvx = 1;
		end
	`endif

    Bit#(TMax#(`causesize, 7)) temp1 = {'d0,fn,funct3};
    if(inst_type==TRAP)
      temp1=zeroExtend(trapcause);

    Bool microtrap = mem_access==Fence || mem_access==FenceI || inst_type==SYSTEM_INSTR
                `ifdef supervisor || mem_access==SFence `endif 
								`ifdef hypervisor || mem_access == HFence_VVMA || mem_access == HFence_GVMA `endif ;
    let op_addr = OpAddr{rs1addr:rs1, rs2addr:rs2, rd:rd `ifdef spfpu ,rs3addr: rs3 `endif };
    let op_type = OpType{rs1type: rs1type, rs2type:rs2type `ifdef spfpu ,rs3type: rs3type, rdtype: rdtype `endif };
    let instr_meta = InstrMeta{inst_type: inst_type,
                              memaccess: mem_access,
                              funct_cause:temp1,
                              immediate: immediate_value,
                              microtrap: microtrap
                            `ifdef hypervisor
                              ,hlvx : hlvx
                              ,hvm_loadstore : hvm_loadstore
                            `endif };
    return DecodeOut{op_addr:op_addr, op_type:op_type, meta:instr_meta
                    `ifdef compressed , compressed:compressed `endif };

  endfunction

  `ifdef decoder_noinline
  (*noinline*)
  `endif
  function Bool decode_word32 (Bit#(32) inst, Bit#(1) misa_c);
    Bool word32=False;
    `ifdef RV64
		  Bit#(5) opcode= inst[6:2];
      Bit#(7) funct7 = inst[31:25];
		  Bit#(3) funct3= inst[14:12];
  		if(opcode==`IMM_ARITHW_op || opcode==`MULDIVW_op ||  opcode==`ARITHW_op ||
        (opcode[4:1]=='b0101 && funct3[0]==0)
      `ifdef spfpu || (opcode[4:3]=='b10 && funct7[0]==0) `endif )
      word32=True;
    `endif
    return word32;
  endfunction

  function ActionValue#(DecodeOut) decoder_func(Bit#(32) inst, Bool trap, Bit#(`causesize) cause,
                                                CSRtoDecode csrs, Bool is_microtrap_set,
                                                Bit#(`causesize) microtrap_cause
								            `ifdef compressed , Bool compressed `endif 
                            `ifdef debug , DebugStatus debug, Bool step_done `endif ) =  actionvalue

      DecodeOut result_decode = fn_decode(inst, csrs `ifdef compressed ,compressed `endif
                                                    `ifdef debug ,debug `endif );
      let {icause, takeinterrupt} = chk_interrupt( csrs.prv,
                                                   csrs.csr_mstatus,
                                                   csrs.csr_sstatus,
                                                   csrs.csr_mip,
                                                   csrs.csr_mie
                                `ifdef non_m_traps ,csrs.csr_mideleg `endif
                `ifdef supervisor `ifdef usertraps ,csrs.csr_sideleg `endif `endif
                                  `ifdef debug     ,debug, step_done `endif 
								`ifdef hypervisor  ,csrs.csr_vs_bit, csrs.csr_hideleg `endif );

      Bit#(TMax#(7,`causesize)) func_cause=result_decode.meta.funct_cause;
      Instruction_type x_inst_type = result_decode.meta.inst_type;
      Op1type x_rs1type = result_decode.op_type.rs1type;
      Op2type x_rs2type = result_decode.op_type.rs2type;
      Bit#(5) x_rs1addr = result_decode.op_addr.rs1addr;
      Bit#(5) x_rs2addr = result_decode.op_addr.rs2addr;

      if(is_microtrap_set)begin
        x_inst_type=TRAP;
        func_cause=extend(microtrap_cause);
        result_decode.meta.microtrap=False;
      end
      /* we given precedence to WFI over interrupt. As in the next cycle the interrupt will exit the
       * WFI as usual*/
      else if(takeinterrupt && x_inst_type != WFI)begin
        func_cause=zeroExtend(icause);
        x_inst_type=TRAP;
      end
      else if(trap) begin
        x_inst_type=TRAP;
        func_cause = zeroExtend(cause) ;
      end

      result_decode.meta.inst_type=x_inst_type;
      result_decode.meta.funct_cause=func_cause;
      result_decode.op_type.rs1type=x_rs1type;
      result_decode.op_type.rs2type=x_rs2type;
      result_decode.op_addr.rs1addr=x_rs1addr;
      result_decode.op_addr.rs2addr=x_rs2addr;
      return result_decode;

  endactionvalue;
endpackage
