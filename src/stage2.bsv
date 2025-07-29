//See LICENSE.iitm for license details
/*

Author : IIT Madras
Details:
1.  This module decodes the instructions fetched from the previous stage and also fetches the
    operands from the registerfile.
2.  If a csr operation is being decoded, then the next instruction is stalled untill the csr
    completes and commits the instruction.

NOTE0 : Handling flushes
  Flushes in this stage are handled by 2 epoch registers : eEpoch and wEpoch. This bits are compared
  to the epochs bits from the fetch unit (status under which they were fetched). If they do not
  match then the instruction is dropped. The reason for having 2 epoch registers is because both:
  the execute and the writeback stage can generate a flush of the pipe,  causing instructions to be
  dropped all over.

NOTE1 : Handling Traps
  By handling trap and flushing fetch to jump to the trap routine in this stage saves cycle. One
  might also consider that PC no longer needs to be sent to the subsequent stages. However,  note that
  the load / store exceptions are only captured in the next staged. Including pagefaults. So you will
  any how need to handle a trap in that stage as well.

  Additionally, if you have 2 stages handling traps,  you will have prioritize on over the other.
  Suppose you take a trap from the decode stage but there exists an instruction in the subsequent
  pipeline buffers which will generate a memory exception. While taking the trap in the decode stage
  you have corrupted the csrs and this will screw up all further exception handling.

NOTE2 : Handling WFI.
  WFI is also handled in this stage. If a wfi instruction is encountered is treated as a NOP and
  simply dropped. Simultaenously a register is set. When the instruction requests to be decoded and
  the register is set,  the instruction will only progress if an interrupt has arrived. This will
  ensure that the interrupt is taken on the next instructions as required by the spec. When this
  interrupt is taken (under stall mode) then the register is reset and normal functionality resumes.

  If there are n - continous "wfi" instructions,  then n - interrupts will have to be serviced to resume
  the core.

  If a wfi instruction is decoded and an interrupt is pending, then the WFI instruction is taken. In
  WFI mode, the interrupt is detected in the next cycle and wfi-wait ends. 

NOTE3 : When an illegal exception is taken you have to store the instruction in the mtval register.
This enables trap handlers to quickly emulate the instruction in software. To do this, in case of an
illegal exception we send the instruction as rs1 from stage2 and then pass it as the rd value after
exection stage.

--------------------------------------------------------------------------------------------------
*/
package stage2;
// -- package imports --//
import FIFOF        :: * ;
import TxRx         :: * ;
import TxRx_MIMO    :: * ;
import DReg         :: * ;
import Connectable  :: * ;
import GetPut       :: * ;
import ConfigReg    :: * ;
import OInt         :: * ;
import Vector       :: * ;
import MIMO         :: * ;

// -- project imports --//
import registerfile :: * ;        // for instantiating the registerfile
import decoder      :: * ;        // for the decode functions.
import ccore_types  :: * ;        // for pipe - line types
import pipe_ifcs    :: * ;
`ifdef compressed
  import decompress     :: * ;
`endif
`include "ccore_params.defines"   // for core parameters
`include "trap.defines"
`include "Logger.bsv"             // for logging display statements.

interface Ifc_stage2;

  interface Ifc_s2_rx rx;
  interface Ifc_s2_tx tx;
  interface Ifc_s2_rf rf;
  interface Ifc_s2_common  common;
`ifdef debug
  interface Ifc_s2_debug debug;
`endif
`ifdef perfmonitors
  interface Ifc_s2_perfmonitors perf;
`endif
  method Bool mv_wfi_detected;
endinterface : Ifc_stage2

function Fmt fstage2(Bit#(`xlen) hartid, Bit#(1) buffer_no, FwdType op1, Op1type op1type, FwdType op2, Op2type op2type, 
                        FwdType op3, Instruction_type insttype, Stage3Meta meta, Bit#(`xlen) mtval );
  Fmt result = $format("[%2d]STAGE2 : ",hartid) + $format(" BUFFER_NO: ", buffer_no);
  Fmt op1_addr = ?;
  if (op1type == IntegerRF)
    op1_addr = $format(" RS1=") + op1_addr + $format("X[%2d][%h]",op1.addr,op1.data);
  else
`ifdef spfpu
  if(op1type == FloatingRF)
    op1_addr = $format(" RS1=") + op1_addr + $format("F[%2d][%h]",op1.addr,op1.data);
  else
`endif
    op1_addr = $format(" RS1=") + op1_addr + $format("PC[%h]",meta.pc);

  Fmt op2_addr = ?; 
  if (op2type == IntegerRF)
    op2_addr = $format(" RS2=") + op2_addr + $format("X[%2d][%h]",op2.addr,op2.data);
  else
`ifdef spfpu
  if (op2type == FloatingRF)
    op2_addr = $format(" RS2=") + op2_addr + $format("F[%2d][%h]",op2.addr,op2.data);
  else
`endif
`ifdef compressed
  if (op2type == Constant2)
    op2_addr = $format(" RS2=") + op2_addr + $format("Immediate['h2]");
  else
`endif
  if (op2type == Immediate)
    op2_addr = $format(" RS2=") + op2_addr + $format("Immediate[%h]",op2.data);
  else
    op2_addr = $format(" RS2=") + op2_addr + $format(" Immediate['h4]");

  Fmt op_rd = ?; 
`ifdef spfpu 
  if(meta.rdtype == FRF)
    op_rd = $format(" RD=") + op_rd + $format("F[%2d]",meta.rd);
  else
`endif
    op_rd = $format(" RD=") + op_rd + $format("X[%2d]",meta.rd);

  Fmt op3_addr = $format(" Immediate[%h]",op3.data);
  Fmt offset = $format(" Offset[%h]",op3.data);
  Fmt csr_addr = $format(" CSRADDR[%h]",op3.data[11:0]);
  case (insttype)
    ALU: result = result + $format("ALU -") + op1_addr + op2_addr + op_rd;
    BRANCH: result = result + $format("Branch -") + op1_addr + op2_addr + offset;
    JAL: result = result + $format("JAL -") + offset + op_rd;
    JALR: result = result + $format("JAL -") + op1_addr + offset + op_rd;
    TRAP: result = result + $format("TRAP - Cause:%3d Microtrap:%b Mtval:%h",meta.funct, meta.is_microtrap,mtval);
    WFI: result = result + $format("WFI");
    SYSTEM_INSTR: result = result + $format("SYSTEM -") + op1_addr +csr_addr +op_rd;
    MEMORY: begin
      result = result + $format("MEMORY - ");
      if (meta.memaccess == Load)
        result = result + $format("LOAD") + op1_addr + op_rd + offset;
      else if (meta.memaccess == Store)
        result = result + $format("STORE") + op1_addr + op2_addr + offset;
    `ifdef atomic
      else if (meta.memaccess == Atomic)
        result = result + $format("Atomic") + op1_addr + op2_addr + op_rd;
    `endif
      else
        result = result + fshow(meta.memaccess);
    end
  `ifdef muldiv
    MULDIV: result = result + $format("MULDIV -") + op1_addr + op2_addr + op_rd;
  `endif
  `ifdef spfpu
    FLOAT: result = result + $format("FLOAT -") + op1_addr + op2_addr + op3_addr + op_rd;
  `endif
  endcase
  return result;
endfunction:fstage2

//typedef enum {CheckPrev, None} ActionType deriving(Bits, Eq);

`ifdef stage2_noinline
(*synthesize*)
`endif
module mkstage2#(parameter Bit#(`xlen) hartid) (Ifc_stage2);

  String stage2=""; // defined for logger

  // --------------------- Start instantiations ------------------------//

  /*doc:mod: instantiation of the registerfile module */
  Ifc_registerfile registerfile <- mkregisterfile(hartid);

  /*doc:mod FIFO to interface with stage0 and receive fetched instruction */
	RX_MIMO#(2, 2, `instr_queue, PIPE1) rx_pipe1 <- mkRX_MIMO;

  /*doc:mod FIFO interface to send the decoded information to the next stage.*/
  TX#(Vector#(`num_issue, Stage3Meta))   tx_meta   <- mkTX;

  /*doc:mod FIFO interface to send the bad-address information to the next stage.*/
  TX#(Vector#(`num_issue, Bit#(`xlen)))   tx_mtval   <- mkTX;

  /*doc:mod FIFO interface to send the bad-address information to the next stage.*/
  TX#(Vector#(`num_issue, Instruction_type))   tx_instrtype <- mkTX;

  /*doc:mod FIFO interface to send the operands meta data to the next stage.*/
  TX#(OpMeta)   tx_opmeta <- mkTX;
`ifdef rtldump
  // fifo interface used to transmit the trace of the instruction for rtl.dump generation
  TX#(Vector#(`num_issue, CommitLogPacket)) tx_commitlog <- mkTX;
  RX_MIMO#(2, 2, `instr_queue, CommitLogPacket) rx_commitlog <- mkRX_MIMO;
`endif

  /*doc:wire: wire to capture the latest csr values from csr-file*/
  Wire#(CSRtoDecode) wr_csrs <- mkWire();

  /*doc:reg: this register maintains the epoch value modified by the execute stage*/
	Reg#(Bit#(1)) eEpoch <- mkConfigReg(0);

  /*doc:reg: this register maintains the epoch value modified by the write-back stage*/
	Reg#(Bit#(1)) wEpoch <- mkConfigReg(0);

  /*doc:reg:
    this register is used to stall the current stage from processing any new instructions until a
    redirection from execute / write - back is received. The stall is generated when an trap is
    detected in this stage for the current instruction being processed. This prevents flooding
    the pipe with un - necessary instructions since a redirection is expected.*/
  Reg#(Bool) rg_stall <- mkReg(False);

  /*doc:reg:
    this register when True indicates the current stage is waiting for interrupts before
    sending any new info to the next stage*/
  Reg#(Bool) rg_wfi   <- mkReg(False);

  /*doc:wire: This wire indicates if any locally enabled interrupt is pending irrespective of the
  * global status of interrupt-enable or delegation. It simply carries mie&mip*/
  Wire#(Bool) wr_resume_wfi <- mkDWire(False);

  /*doc:reg:
    This register when set to true indicates that the current instruction being processed will
    have to be re - fetched and executed since the previous instruction was a CSR operation.*/
  Reg#(Bool) rg_microtrap <- mkReg(False);

  /*doc:reg: This register stores the micro-trap cause values*/
  Reg#(Bit#(`causesize)) rg_microtrap_cause <- mkReg(0);

  /*doc:wire:
    the following wires are used to ensure that rg_microtrap, rg_wfi and rg_stall are not set in the cycle a
    redirection from the exe / wb stage is received.*/
  Wire#(Bool) wr_flush_from_exe <- mkDWire(False);
  Wire#(Bool) wr_flush_from_wb  <- mkDWire(False);

`ifdef debug
  /*doc:wire: This wire will capture info about the current debug state of the core*/
  Wire#(DebugStatus) wr_debug_info <- mkWire();

  // This register indicates when an instruction passed the decode stage after a resume request is
  // received while is step is set.
  Reg#(Bool) rg_step_done <- mkReg(False);
`endif

  /*doc:reg:
    This register holds the latest value of operand1 from the RF. This will get updated
    every time a retirement to the same register occurs.*/
  Reg#(FwdType) rg_op1[2] <- mkCReg(2, unpack(0));

  /*doc:reg:
    This register holds the latest value of operand2 from the RF. This will get updated
    every time a retirement to the same register occurs.*/
  Reg#(FwdType) rg_op2[2] <- mkCReg(2, unpack(0));

  //Reg#(Op2type) rg_op2type[2] <- mkCReg(2, IntegerRF);
  Wire#(Op2type) wr_op2type <- mkDWire(IntegerRF);
  /*doc:reg:
    This register holds the latest value of operand1 from the RF. This will get updated
    every time a retirement to the same register occurs.*/
  Reg#(FwdType) rg_op4[2] <- mkCReg(2, unpack(0));

  /*doc:reg:
    This register holds the latest value of operand2 from the RF. This will get updated
    every time a retirement to the same register occurs.*/
  Reg#(FwdType) rg_op5[2] <- mkCReg(2, unpack(0));

  Wire#(Op2type) wr_op5type <- mkDWire(IntegerRF);
  /*doc:reg:
    This register holds the latest value of operand3 from the RF. This will get updated
    every time a retirement to the same register occurs.*/
  Reg#(FwdType) rg_op3[2] <- mkCReg(2, unpack(0));

  /*doc:reg:
    This register holds the offset value of branch instruction being present in the second
    pipeline buffer.
  */
  Reg#(Bit#(32)) rg_op6 <- mkReg(unpack(0));

`ifdef perfmonitors
  Wire#(Bit#(1)) wr_dual_issued <- mkDWire(0);
  Wire#(Bit#(1)) wr_raw_hazard <- mkDWire(0);
  Wire#(Bit#(1)) wr_raw_hazard_dual <- mkDWire(0);
  /*doc: wire: Indicates that only one instruction is present in the instruction queue.*/
  Wire#(Bit#(1)) wr_one_instr <- mkDWire(0);
  Wire#(Bit#(1)) wr_mul_branch_hazard <- mkDWire(0);
  Wire#(Bit#(1)) wr_mul_mem_hazard <- mkDWire(0);
  Wire#(Bit#(1)) wr_mul_float_hazard <- mkDWire(0);
  Wire#(Bit#(1)) wr_mul_mul_hazard <- mkDWire(0);
  Wire#(Bit#(1)) wr_mem_mem_hazard <- mkDWire(0);
  Wire#(Bit#(1)) wr_mem_branch_hazard <- mkDWire(0);
  Wire#(Bit#(1)) wr_mem_float_hazard <- mkDWire(0);
  Wire#(Bit#(1)) wr_float_branch_hazard <- mkDWire(0);
  Wire#(Bit#(1)) wr_float_float_hazard <- mkDWire(0);
  Wire#(Bit#(1)) wr_branch_branch_hazard <- mkDWire(0);
`endif
`ifdef simulate
  Wire#(Bit#(1)) wr_simulate_log_start <- mkDWire(0);
`endif

  /*doc:reg: This register stores the 2nd instruction for the next cycle if it was not
    issued due to dependency reasons.
  */
  //Reg#(ActionType) rg_action <- mkReg(None);

  //MIMOConfiguration cfg = defaultValue;
  //cfg.unguarded=True;
  //MIMO#(2, 2, `instr_queue, InstructionQueue) instruction_queue <- mkMIMO(cfg);



  // ---------------------- End Instatiations --------------------------//

  // ---------------------- Start Rules -------------------------------//
  //PulseWire pw_pipe1_deqReady <- mkPulseWire();
  //PulseWire pw_pipe1_not_deqReady <- mkPulseWire();
  //rule rl_read_pipe1_status;
  //  let valid <- rx_pipe1.u.deqReady(1);
  //  if (!valid)
  //    pw_pipe1_not_deqReady.send();
  //  if (valid)
  //    pw_pipe1_deqReady.send();
  //endrule

`ifdef simulate
  rule rl_polling_check(!tx_instrtype.u.notFull || rg_stall || rg_wfi);
    `logLevel( stage2, 1, $format("[%2d]STAGE2: rg_stall: %d, rg_wfi: %d, instrNotFull: %d", hartid, rg_stall, rg_wfi, tx_instrtype.u.notFull), wr_simulate_log_start)
  endrule

  rule rl_waiting_print (!rx_pipe1.u.deqReady_1);
    `logLevel( stage2, 1, $format("[%2d]STAGE2: Waiting for response from stage1", hartid), wr_simulate_log_start)
  endrule

  rule rl_upd_log_start;
    registerfile.ma_simulate_log_start(wr_simulate_log_start);
  endrule
`endif

  // RuleName : decode_and_opfetch
  // Explicit Conditions : rg_stall == False and rx_pipe1.deqReady
  // Implicit Conditions :  and all tx fifos are not full
  // Description : This rule decodes the current fetched instruction, fetches the operands from the
  // registerfile and sends the required struct to the next stage.
  rule decode_and_opfetch(!rg_stall && tx_instrtype.u.notFull && !rg_wfi && rx_pipe1.u.deqReady_1 `ifdef rtldump && rx_commitlog.u.deqReady_1 `endif );

    // --- extract the fields from the packet received from the stage1 ---- //
    Vector#(`num_issue, Bit#(32)) inst = replicate(0);
    Vector#(`num_issue, PIPE1) instr_data = replicate(unpack(?));
    Vector#(`num_issue, Bit#(`vaddr)) pc = replicate(?);
    Vector#(`num_issue, Bit#(`iesize)) epochs = replicate(?);
    Vector#(`num_issue, Bool) upper_instr = replicate(?);
    Bool trap = False;
    Bit#(`causesize) trapcause = ?;
    `ifdef compressed
      Vector#(`num_issue, Bool) highbyte_err = replicate(False);
      Vector#(`num_issue, Bool) compressed = replicate(False);
    `endif
    `ifdef bpu
      Vector#(`num_issue, BTBResponse) btbresponse = replicate(unpack(?));
    `endif
    for (Integer i=0; i<`num_issue; i=i+1) begin
      instr_data[i] = rx_pipe1.u.first[i];
      pc[i] = instr_data[i].program_counter;
      epochs[i] = instr_data[i].epochs;
    `ifdef compressed
      highbyte_err[i] = instr_data[i].upper_err;
      compressed[i] = instr_data[i].compressed;
      if (compressed[i])
        inst[i] = fn_decompress(truncate(instr_data[i].instruction));
      else
        inst[i] = instr_data[i].instruction;
    `else
      inst[i] = instr_data[i].instruction;
    `endif
    `ifdef bpu
      btbresponse[i] = instr_data[i].btbresponse;
    `endif
      //inst[i] = instr_data[i].instruction;
      upper_instr[i] = unpack(fromInteger(i));
    end

    trap = instr_data[0].trap;
    trapcause = instr_data[0].cause;


    // ---------------------------------------------------------------------------------------- //

    `logLevel( stage2, 4, $format("[%2d]STAGE2: csrs:",hartid,fshow(wr_csrs)), wr_simulate_log_start)

    // ----------------------------- perform decode ------------------------ //
    Vector#(`num_issue, DecodeOut) decoded_inst;
    Vector#(`num_issue, Bit#(32)) imm;
    Vector#(`num_issue, Bit#(TMax#(`causesize, 7))) func_cause; 
    Vector#(`num_issue, Bool) word32;
    RFType rf1type;
    RFType rf2type;
    Vector#(`num_issue, Bit#(`xlen)) mtval = replicate(0);
    Bit#(5) frf_rs3addr;
    Vector#(`num_issue, Instruction_type) instrType;
    //Vector#(`num_issue, Bit#(32)) inst;
    for (Integer i=0; i<`num_issue; i=i+1) begin
      if (!(i == 1 && !rx_pipe1.u.deqReady_2)) begin
        decoded_inst[i] <- decoder_func(inst[i], trap, 
                                    trapcause, wr_csrs,
                                    rg_microtrap, rg_microtrap_cause
                                    `ifdef compressed ,compressed[i] `endif
                                    `ifdef debug ,wr_debug_info, rg_step_done `endif );
        instrType[i] = decoded_inst[i].meta.inst_type;
      end
      else begin
        decoded_inst[i] = unpack(0);
        instrType[i] = NONE;
      end
      imm[i] = decoded_inst[i].meta.immediate;
      func_cause[i] = decoded_inst[i].meta.funct_cause;
      word32[i] = decode_word32(inst[i],wr_csrs.csr_misa[2]);
      if(func_cause[i] == `Illegal_inst )
        mtval[i] = zeroExtend(inst[i]); // for mtval
      else if(func_cause[i] == `Breakpoint )
        mtval[i] = zeroExtend(pc[i] + 2*fromInteger(i)); // for mtval
      `ifdef supervisor
        `ifdef compressed
          else if(func_cause[i] == `Inst_pagefault && highbyte_err[i])
            mtval[i] = zeroExtend(pc[i] + 2*fromInteger(i)) + 2;
        `endif
        else if(func_cause[i] == `Inst_pagefault)
          mtval[i] = zeroExtend(pc[i] + 2*fromInteger(i));
      `endif
    end

    // -------- Instruction Dependency Check ------------ //
    Bool issue_two_inst;
    for (Integer i=0; i<`num_issue; i=i+1)
      `logLevel( stage2, 0, $format("STAGE2[%2d]: Decoded Instruction %d : ", hartid, i, fshow(decoded_inst[i])), wr_simulate_log_start)

    Vector#(`num_issue, Bit#(6)) src1_addr, src2_addr, src3_addr, dest_addr;
    Vector#(`num_issue, RFType) src1_type, src2_type;
    for (Integer i=0; i<`num_issue; i=i+1) begin
      src1_type[i] = `ifdef spfpu decoded_inst[i].op_type.rs1type == FloatingRF ? FRF : `endif IRF;
      src1_addr[i] = {pack(src1_type[i]), decoded_inst[i].op_addr.rs1addr};

      src2_type[i] = `ifdef spfpu decoded_inst[i].op_type.rs2type == FloatingRF ? FRF : `endif IRF;
      src2_addr[i] = {pack(src2_type[i]), decoded_inst[i].op_addr.rs2addr};

      if (instrType[i] == FLOAT)
        src3_addr[i] = {pack(FRF), decoded_inst[i].op_addr.rs3addr};
      else
        src3_addr[i] = 0;

      //dest_type[i] = `ifdef spfpu decoded_inst[i].op_type.rdtype == FloatingRF ? FRF : `endif IRF;
      dest_addr[i] = {pack(decoded_inst[i].op_type.rdtype), decoded_inst[i].op_addr.rd};
    end
    if (instrType[1] != NONE) begin
      //if (decoded_inst[0].op_type.rs1type != FloatingRF && decoded_inst[1].op_type.rs1type != FloatingRF &&
      //  (decoded_inst[0].op_addr.rd == decoded_inst[1].op_addr.rs1addr || 
      //   decoded_inst[0].op_addr.rd == decoded_inst[1].op_addr.rs2addr  
      //   `ifdef spfpu || decoded_inst[0].op_addr.rd == decoded_inst[1].op_addr.rs3addr `endif ))
      //   issue_two_inst = False;
      //`ifndef no_wawstalls
      //// WAW hazard
      //else if (decoded_inst[0].op_type.rs1type != FloatingRF && decoded_inst[1].op_type.rs1type != FloatingRF &&
      //  (decoded_inst[1].op_addr.rd == decoded_inst[0].op_addr.rd))
      //   issue_two_inst = False;
      //// WAR Hazard
      //else if (decoded_inst[0].op_addr.rs1type != FloatingRF && decoded_inst[1].op_addr.rs1type != FloatingRF &&
      //  (decoded_inst[1].op_addr.rd == decoded_inst[0].op_addr.rs1addr || 
      //   decoded_inst[1].op_addr.rd == decoded_inst[0].op_addr.rs2addr  
      //   `ifdef spfpu || decoded_inst[1].op_addr.rd == decoded_inst[0].op_addr.rs3addr `endif ))
      //   issue_two_inst = False;
      //`endif
      // RAW hazard
      if (!(dest_addr[0] == 0) &&
          (dest_addr[0] == src1_addr[1] ||
          dest_addr[0] == src2_addr[1]
          `ifdef spfpu || dest_addr[0] == src3_addr[1] `endif )) begin
        issue_two_inst = False;
        wr_raw_hazard <= 1;
        if (instrType[0] == ALU || instrType[1] == ALU 
          || instrType[0] == BRANCH || instrType[1] == BRANCH
          || instrType[0] == JAL || instrType[1] == JAL
          || instrType[0] == JALR || instrType[1] == JALR) begin
          wr_raw_hazard_dual <= 1;
          `logLevel( stage2, rawalu, $format("RAW ALU/BRANCH Hazard PC1: %h (", pc[0], fshow(instrType[0]), ") PC2: %h (", pc[1], fshow(instrType[1]), ")"), wr_simulate_log_start)
          `logLevel( stage2, rawalu2, $format("Instr1: RS1=%d ", decoded_inst[0].op_addr.rs1addr, fshow(decoded_inst[0].op_type.rs1type), 
              " RS2=%d ", decoded_inst[0].op_addr.rs2addr, fshow(decoded_inst[0].op_type.rs2type), 
              " RS3=%d ", decoded_inst[0].op_addr.rs3addr, fshow(decoded_inst[0].op_type.rs3type),
              " RD=%d ", decoded_inst[0].op_addr.rd, fshow(decoded_inst[0].op_type.rdtype)), wr_simulate_log_start)
          `logLevel( stage2, rawalu2, $format("Instr2: RS1=%d ", decoded_inst[1].op_addr.rs1addr, fshow(decoded_inst[1].op_type.rs1type),
              " RS2=%d ", decoded_inst[1].op_addr.rs2addr, fshow(decoded_inst[1].op_type.rs2type), 
              " RS3=%d ", decoded_inst[1].op_addr.rs3addr, fshow(decoded_inst[1].op_type.rs3type),
              " RD=%d ", decoded_inst[1].op_addr.rd, fshow(decoded_inst[1].op_type.rdtype), "\n-----------------------------------------------"), wr_simulate_log_start)
        end

        `logLevel( stage2, perf, $format("[%2d]STAGE2: RAW Hazard", hartid), wr_simulate_log_start)
        `logLevel( stage2, raw, $format("RAW Hazard PC1: %h (", pc[0], fshow(instrType[0]), ") PC2: %h (", pc[1], fshow(instrType[1]), ")"), wr_simulate_log_start)
        `logLevel( stage2, raw2, $format("Instr1: RS1=%d ", decoded_inst[0].op_addr.rs1addr, fshow(decoded_inst[0].op_type.rs1type), 
            " RS2=%d ", decoded_inst[0].op_addr.rs2addr, fshow(decoded_inst[0].op_type.rs2type), 
            " RS3=%d ", decoded_inst[0].op_addr.rs3addr, fshow(decoded_inst[0].op_type.rs3type),
            " RD=%d ", decoded_inst[0].op_addr.rd, fshow(decoded_inst[0].op_type.rdtype)), wr_simulate_log_start)
        `logLevel( stage2, raw2, $format("Instr2: RS1=%d ", decoded_inst[1].op_addr.rs1addr, fshow(decoded_inst[1].op_type.rs1type),
            " RS2=%d ", decoded_inst[1].op_addr.rs2addr, fshow(decoded_inst[1].op_type.rs2type), 
            " RS3=%d ", decoded_inst[1].op_addr.rs3addr, fshow(decoded_inst[1].op_type.rs3type),
            " RD=%d ", decoded_inst[1].op_addr.rd, fshow(decoded_inst[1].op_type.rdtype), "\n-----------------------------------------------"), wr_simulate_log_start)
        `logLevel( stage2, raw, $format(""), wr_simulate_log_start)
      end
    `ifndef no_wawstalls
      // WAR Hazard
      else if (dest_addr[1] == src1_addr[0] ||
               dest_addr[1] == src2_addr[0]
               `ifdef spfpu || dest_addr[1] == src3_addr[0] `endif ) begin
        issue_two_inst = False;
        `logLevel( stage2, perf, $format("[%2d]STAGE2: WAR Hazard", hartid), wr_simulate_log_start)
      end
      //WAW Hazard
      else if (dest_addr[1] == dest_addr[0]) begin
        issue_two_inst = False;
        `logLevel( stage2, perf, $format("[%2d]STAGE2: WAW Hazard", hartid), wr_simulate_log_start)
      end
    `endif
      // Issue both only when both the instructions are ALU
      else if (instrType[0] == ALU && instrType[1] == ALU)
        issue_two_inst = True;
      else if (instrType[0] == ALU && instrType[1] == MULDIV)
        issue_two_inst = True;
      else if (instrType[0] == MULDIV && instrType[1] == ALU)
        issue_two_inst = True;
      // Removing below logic as it will not help coremarks. This will allow me
      // to just have 4 bypass lines instead of 5.
      else if (instrType[0] == ALU && instrType[1] == FLOAT)
        issue_two_inst = True;
      else if (instrType[0] == FLOAT && instrType[1] == ALU)
        issue_two_inst = True;
      else if (instrType[0] == ALU && instrType[1] == MEMORY)
        issue_two_inst = True;
      else if (instrType[0] == MEMORY && instrType[1] == ALU)
        issue_two_inst = True;
      else if (instrType[0] == ALU && (instrType[1] == BRANCH || instrType[1] == JAL || instrType[1] == JALR))
        issue_two_inst = True;
      else if ((instrType[0] == BRANCH || instrType[0] == JAL || instrType[0] == JALR) && instrType[1] == ALU)
        issue_two_inst = True;
      else if ((instrType[0] == BRANCH || instrType[0] == JAL || instrType[0] == JALR) && instrType[1] == MULDIV)
        issue_two_inst = True;
      else if (instrType[0] == MULDIV && (instrType[1] == BRANCH || instrType[1] == JAL || instrType[1] == JALR))
        issue_two_inst = True;
      else if ((instrType[0] == BRANCH || instrType[0] == JAL || instrType[0] == JALR) && instrType[1] == MEMORY)
        issue_two_inst = True;
      else if (instrType[0] == MEMORY && (instrType[1] == BRANCH || instrType[1] == JAL || instrType[1] == JALR))
        issue_two_inst = True;
      else if ((instrType[0] == BRANCH || instrType[0] == JAL || instrType[0] == JALR) && instrType[1] == FLOAT)
        issue_two_inst = True;
      else if (instrType[0] == FLOAT && (instrType[1] == BRANCH || instrType[1] == JAL || instrType[1] == JALR))
        issue_two_inst = True;
      // For all other cases issue only one instruction.
      else begin
      `ifdef perfmonitors
        if ((instrType[0] == MULDIV && (instrType[1] == BRANCH || instrType[1] == JAL || instrType[1] == JALR)) || 
          ((instrType[0] == BRANCH || instrType[0] == JAL || instrType[0] == JALR) && instrType[1] == MULDIV)) begin
          wr_mul_branch_hazard <= 1;
          `logLevel( stage2, perf, $format("[%2d]STAGE2: MUL BRANCH Hazard", hartid), wr_simulate_log_start)
        end
        else if ((instrType[0] == FLOAT && instrType[1] == ALU) || (instrType[0] == ALU && instrType[1] == FLOAT)) begin
          `logLevel( stage2, perf, $format("[%2d]STAGE2: FLOAT ALU Hazard", hartid), wr_simulate_log_start)
        end
        else if ((instrType[0] == MULDIV && instrType[1] == MEMORY) || (instrType[0] == MEMORY && instrType[1] == MULDIV)) begin
          wr_mul_mem_hazard <= 1;
          `logLevel( stage2, perf, $format("[%2d]STAGE2: MUL MEMORY Hazard", hartid), wr_simulate_log_start)
        end
        else if ((instrType[0] == MULDIV && instrType[1] == FLOAT) || (instrType[0] == FLOAT && instrType[1] == MULDIV)) begin
          wr_mul_float_hazard <= 1;
          `logLevel( stage2, perf, $format("[%2d]STAGE2: MUL FLOAT Hazard", hartid), wr_simulate_log_start)
        end
        else if (instrType[0] == MULDIV && instrType[1] == MULDIV) begin
          wr_mul_mul_hazard <= 1;
          `logLevel( stage2, perf, $format("[%2d]STAGE2: MUL MUL Hazard", hartid), wr_simulate_log_start)
        end
        else if (instrType[0] == MEMORY && instrType[1] == MEMORY) begin
          wr_mem_mem_hazard <= 1;
          `logLevel( stage2, perf, $format("[%2d]STAGE2: MEMORY MEMORY Hazard", hartid), wr_simulate_log_start)
        end
        else if ((instrType[0] == MEMORY && (instrType[1] == BRANCH || instrType[1] == JAL || instrType[1] == JALR)) || 
          ((instrType[0] == BRANCH || instrType[0] == JAL || instrType[0] == JALR) && instrType[1] == MEMORY)) begin
          wr_mem_branch_hazard <= 1;
          `logLevel( stage2, perf, $format("[%2d]STAGE2: MEMORY BRANCH Hazard", hartid), wr_simulate_log_start)
        end
        else if ((instrType[0] == MEMORY && instrType[1] == FLOAT) || (instrType[0] == FLOAT && instrType[1] == MEMORY)) begin
          wr_mem_float_hazard <= 1;
          `logLevel( stage2, perf, $format("[%2d]STAGE2: MEMORY FLOAT Hazard", hartid), wr_simulate_log_start)
        end
        else if ((instrType[0] == FLOAT && (instrType[1] == BRANCH || instrType[1] == JAL || instrType[1] == JALR)) || 
          ((instrType[0] == BRANCH || instrType[0] == JAL || instrType[0] == JALR) && instrType[1] == FLOAT)) begin
          wr_float_branch_hazard <= 1;
          `logLevel( stage2, perf, $format("[%2d]STAGE2: FLOAT BRANCH Hazard", hartid), wr_simulate_log_start)
        end
        else if (instrType[0] == FLOAT && instrType[1] == FLOAT) begin
          wr_float_float_hazard <= 1;
          `logLevel( stage2, perf, $format("[%2d]STAGE2: FLOAT FLOAT Hazard", hartid), wr_simulate_log_start)
        end
        else if ((instrType[0] == BRANCH || instrType[0] == JAL || instrType[0] == JALR) && (instrType[0] == BRANCH || instrType[0] == JAL || instrType[0] == JALR)) begin
          wr_branch_branch_hazard <= 1;
          `logLevel( stage2, perf, $format("[%2d]STAGE2: BRANCH BRANCH Hazard", hartid), wr_simulate_log_start)
        end
      `endif
        issue_two_inst = False;
      end

      //// When any of the instruction is a MULDIV.
      //else if (instrType[0] == MULDIV || instrType[1] == MULDIV)
      //  issue_two_inst = False;
      //// When any of the instruction is a FLOAT.
      //else if (instrType[0] == FLOAT || instrType[1] == FLOAT)
      //  issue_two_inst = False;
      //// When any of the instruction is a MEMORY.
      //else if (instrType[0] == MEMORY || instrType[1] == MEMORY)
      //  issue_two_inst = False;
      //// When any of the instruction is a SYSTEM.
      //else if (instrType[0] == SYSTEM_INSTR || instrType[1] == SYSTEM_INSTR)
      //  issue_two_inst = False;
      //// When any of the instructions is a BRANCH
      //else if (instrType[0] == BRANCH || instrType[1] == BRANCH ||
      //         instrType[0] == JAL || instrType[1] == JAL || 
      //         instrType[0] == JALR || instrType[1] == JALR )
      //  issue_two_inst = False;
      //// When any of the instructions is a TRAP
      //else if (instrType[0] == TRAP || instrType[1] == TRAP)
      //  issue_two_inst = False;
      //else if (instrType[0] == WFI || instrType[1] == WFI)
      //  issue_two_inst = False;
      //else
      //  issue_two_inst = True;
    end
    else
      issue_two_inst = False;
    // -------------------------------------------------- //



  `ifdef rtldump
    Vector#(`num_issue, CommitLogPacket) clogpkt;
    clogpkt = rx_commitlog.u.first;
  `endif

    Bool instr_reversed = False;

    //if (issue_two_inst && ((instrType[1] == MULDIV) || (instrType[1] == FLOAT) || instrType[1] == MEMORY)) begin
    if (issue_two_inst && (instrType[0] == ALU || (instrType[0] == BRANCH || instrType[0] == JAL || instrType[0] == JALR))) begin
      decoded_inst = reverse(decoded_inst); 
      imm = reverse(imm);
      func_cause = reverse(func_cause);
      mtval = reverse(mtval);
      word32 = reverse(word32);
      compressed = reverse(compressed);
      inst = reverse(inst);
      pc = reverse(pc);
      epochs = reverse(epochs);
    `ifdef bpu
      btbresponse = reverse(btbresponse);
    `endif
      instr_data = reverse(instr_data);
      highbyte_err = reverse(highbyte_err);
      instrType = reverse(instrType);
      upper_instr = reverse(upper_instr);
      instr_reversed = True;
      `ifdef rtldump
        clogpkt = reverse(clogpkt);
      `endif
    end
    
    frf_rs3addr = inst[0][31:27];

  `ifdef spfpu
    rf1type = `ifdef spfpu decoded_inst[0].op_type.rs1type == FloatingRF ? FRF : `endif IRF;
    rf2type = `ifdef spfpu decoded_inst[0].op_type.rs2type == FloatingRF ? FRF : `endif IRF;
  `endif
    // --------------------------------------------//

    let rs1_from_rf <- registerfile.read_rs1(decoded_inst[0].op_addr.rs1addr
                        `ifdef spfpu ,rf1type `endif );
    let rs2_from_rf <- registerfile.read_rs2(decoded_inst[0].op_addr.rs2addr
                        `ifdef spfpu ,rf2type `endif );
  `ifdef spfpu
    let rs3 <- registerfile.read_rs3(frf_rs3addr);
  `endif
    let rs4_from_rf <- registerfile.read_rs4(decoded_inst[1].op_addr.rs1addr
                        `ifdef spfpu ,IRF `endif );
    let rs5_from_rf <- registerfile.read_rs5(decoded_inst[1].op_addr.rs2addr
                        `ifdef spfpu ,IRF `endif );
    // -------------------------------------------------------------------------------------- //
    
    // ------------------------ modify operand values before enquing to next stage -----------//
    Bit#(`elen) op1_inst0 =  rs1_from_rf;
    Bit#(`elen) op2_inst0 =  (decoded_inst[0].op_type.rs2type == Constant2) ? 'd2: // constant2 only is C enabled.
                      (decoded_inst[0].op_type.rs2type == Constant4) ? 'd4:
                      (decoded_inst[0].op_type.rs2type == Immediate) ? signExtend(imm[0]) : rs2_from_rf;
  `ifdef spfpu
    Bit#(`flen) inst0_imm = (decoded_inst[0].op_type.rs3type == FRF) ? rs3 : signExtend(imm[0]);
  `else
    Bit#(`flen) inst0_imm = signExtend(imm[0]);
  `endif
    Bit#(`elen) op1_inst1 = rs4_from_rf;
    Bit#(`elen) op2_inst1 =  (decoded_inst[1].op_type.rs2type == Constant2) ? 'd2: // constant2 only is C enabled.
                      (decoded_inst[1].op_type.rs2type == Constant4) ? 'd4:
                      (decoded_inst[1].op_type.rs2type == Immediate) ? signExtend(imm[1]) : rs5_from_rf;
    Bit#(32) inst1_imm = signExtend(imm[1]);
    // -------------------------------------------------------------------------------------- //

    Vector#(`num_issue, Stage3Meta) stage3meta;
    for (Integer i=0; i<`num_issue; i=i+1) begin
      stage3meta[i] = Stage3Meta{funct : decoded_inst[i].meta.funct_cause,
                              memaccess : decoded_inst[i].meta.memaccess,
                              pc : pc[i], 
                              epochs : epochs[i],
                              rd: decoded_inst[i].op_addr.rd,
                              is_microtrap: rg_microtrap,
                              upper_instr: upper_instr[i],
                              instr_reversed: instr_reversed
           `ifdef hypervisor ,hlvx : decoded_inst[i].meta.hlvx
                             ,hvm_loadstore : decoded_inst[i].meta.hvm_loadstore
           `endif
           `ifdef spfpu      ,rdtype : decoded_inst[i].op_type.rdtype `endif
           `ifdef RV64       ,word32 : word32[i]
           `elsif dpfpu      ,word32 : word32[i], `endif
           `ifdef compressed     , compressed : compressed[i] `endif
           `ifdef bpu        ,btbresponse:  btbresponse[i] `endif 
            };
    end

    //LUInt#(`num_issue) valid_instructions = 0;

    //for (Integer i=0; i<`num_issue; i=i+1) begin
    //  if({eEpoch, wEpoch} != epochs[i])begin
    //    valid_instructions = valid_instructions + 1;
    //  end
    //end
    if({eEpoch, wEpoch} != epochs[0] && {eEpoch, wEpoch} != epochs[1] && rx_pipe1.u.deqReady_2 && issue_two_inst) begin
      rx_pipe1.u.deq(1);
      `ifdef rtldump
        rx_commitlog.u.deq(1);
      `endif
      `logLevel( stage2, 0, $format("[%2d]STAGE2 : Dropping Two Instructions due to epoch mis - match",hartid), wr_simulate_log_start)
    end
    else if ({eEpoch, wEpoch} != epochs[0]) begin
      rx_pipe1.u.deq(1);
      `ifdef rtldump
        rx_commitlog.u.deq(1);
      `endif
      `logLevel( stage2, 0, $format("[%2d]STAGE2 : Dropping Instruction due to epoch mis - match",hartid), wr_simulate_log_start)
    end
    else begin
      if (instrType[0] == WFI) begin 
        if(!wr_flush_from_exe && !wr_flush_from_wb) begin
          `logLevel( stage2, 2, $format("[%2d]STAGE2 : Encountered WFI",hartid), wr_simulate_log_start)
          rg_wfi <= True;
        end
        else
          `logLevel( stage2, 2, $format("[%2d]STAGE2 : Dropping WFI",hartid), wr_simulate_log_start)
        instrType[0] = ALU;
        instrType[1] = NONE;
      end
      // The following logic is used to ensure correct step functionality. When the core is halted
      // or free-running rg_step_done is set to false. When the step bit in dcsr is set and resume
      // request is received, the very next instruction (matching epochs) will set rg_step_done to
      // True. The core needs to halt after this instruction commit. Thus, the 2nd instruction in
      // the stream (which matches the epochs which could have changed if the first instruction is a
      // branch/jump) is tagged as a Trap with HaltStep cause code, thus causing the core to go back
      // to the halted stage. When the core is again halted then, rg_step_done is reset to False.
      `ifdef debug
        `logLevel( stage2, 4, $format("[%2d]STAGE2: step_done:%b rerun:%b",hartid,rg_step_done,rg_microtrap), wr_simulate_log_start)
        if(rg_step_done && wr_debug_info.debug_mode)
          rg_step_done<=False;
        else if(!rg_microtrap)
          rg_step_done <= !wr_debug_info.debug_mode && wr_debug_info.step_set && wr_debug_info.debugger_available;

        // When step functionality of debug mode is enabled, only issue one instruction always.
        if (!wr_debug_info.debug_mode && wr_debug_info.step_set && wr_debug_info.debugger_available)
          issue_two_inst = False;
      `endif
  
      rg_microtrap <= decoded_inst[0].meta.microtrap && !wr_flush_from_exe && !wr_flush_from_wb;
      rg_microtrap_cause <= (decoded_inst[0].meta.inst_type== SYSTEM_INSTR )? `CSR_rerun :
`ifdef hypervisor (decoded_inst[0].meta.memaccess == HFence_GVMA || decoded_inst[0].meta.memaccess== HFence_VVMA)? `Hfence_rerun: `endif
                           (decoded_inst[0].meta.memaccess == FenceI)?`FenceI_rerun : `Sfence_rerun ;
  
      // -------------------------- Enque relevant data to the next stage -------------------- //

      if(instrType[0] == TRAP)
        rg_stall <= True && !wr_flush_from_exe && !wr_flush_from_wb;

      let opmeta = OpMeta { rs1addr: decoded_inst[0].op_addr.rs1addr,
                            rs2addr: decoded_inst[0].op_addr.rs2addr,
                            rs1type: decoded_inst[0].op_type.rs1type,
                            rs2type: decoded_inst[0].op_type.rs2type
                          `ifdef spfpu
                            ,rs3type: decoded_inst[0].op_type.rs3type
                            ,rs3addr: decoded_inst[0].op_addr.rs3addr
                          `endif
                            ,rs4addr: decoded_inst[1].op_addr.rs1addr,
                            rs5addr: decoded_inst[1].op_addr.rs2addr,
                            rs4type: decoded_inst[1].op_type.rs1type,
                            rs5type: decoded_inst[1].op_type.rs2type
                          };
      tx_meta.u.enq(stage3meta);
      //instrType[1] = NONE; // TODO Need to remove this for dual issue. Just wrote temp so that single issue works properly.
      if (!issue_two_inst)
        instrType[1] = NONE;

      for (Integer i=0; i<`num_issue; i=i+1) begin
        `logLevel( stage2, 0, $format("[%2d]STAGE2 : PC:%h Instruction:%h",hartid,pc[i], inst[i]), wr_simulate_log_start)
        `logLevel( stage2, 0, $format("[%2d]STAGE2 : InstrType : ", hartid, fshow(instrType[i])), wr_simulate_log_start)
      end

      tx_instrtype.u.enq(instrType);
      tx_opmeta.u.enq(opmeta);
      tx_mtval.u.enq(mtval);
      `ifdef rtldump
        for (Integer i=0; i<`num_issue; i=i+1) begin
          if (instrType[i] != NONE) begin
            if (instrType[i] == SYSTEM_INSTR) begin
              clogpkt[i].inst_type = tagged CSR (CommitLogCSR{csr_address : truncate(imm[i]),
                  rd: stage3meta[i].rd, rdata:?, wdata:?, op:truncate(func_cause[i])} );
            end
            else if (instrType[i] == MEMORY) begin
              clogpkt[i].inst_type = tagged MEM (CommitLogMem{access: stage3meta[i].memaccess, 
                      rd: stage3meta[i].rd, 
                      size: truncate(func_cause[i]), address: ?, data: ?, commit_data:?,
                      irf: `ifdef spfpu stage3meta[i].rdtype==IRF `else True `endif });
            end
            else
              clogpkt[i].inst_type = tagged REG (CommitLogReg{wdata:?, rd: stage3meta[i].rd, 
                                  irf: `ifdef spfpu stage3meta[i].rdtype==IRF `else True `endif });
          end
          else
            clogpkt[i].inst_type = tagged None;
        end

        if (!issue_two_inst)
          clogpkt[1].inst_type = tagged None;
        tx_commitlog.u.enq(clogpkt);
      `endif
   
      let _op1 = FwdType{ valid: True, addr: decoded_inst[0].op_addr.rs1addr, data: op1_inst0, epochs: wEpoch
                        `ifdef no_wawstalls ,id: ? `endif
                        `ifdef spfpu ,rdtype: (decoded_inst[0].op_type.rs1type==FloatingRF)?FRF:IRF `endif
                        };
      let _op2 = FwdType{ valid: True, addr: decoded_inst[0].op_addr.rs2addr, data: op2_inst0, epochs: wEpoch
                        `ifdef no_wawstalls ,id: ? `endif
                        `ifdef spfpu ,rdtype: (decoded_inst[0].op_type.rs2type==FloatingRF)?FRF:IRF `endif
                        };
      let _op3 = FwdType{ valid: True, 
                          addr: `ifdef spfpu decoded_inst[0].op_addr.rs3addr `else 0 `endif ,
                          data: signExtend(inst0_imm),
                          epochs: wEpoch
                        `ifdef spfpu ,rdtype: decoded_inst[0].op_type.rs3type `endif 
                        `ifdef no_wawstalls ,id : ? `endif };
      let _op4 = FwdType{ valid: True, addr: decoded_inst[1].op_addr.rs1addr, data: op1_inst1, epochs: wEpoch
                        `ifdef no_wawstalls ,id: ? `endif
                        `ifdef spfpu ,rdtype: (decoded_inst[1].op_type.rs2type==FloatingRF)?FRF:IRF `endif
                        };
      let _op5 = FwdType{ valid: True, addr: decoded_inst[1].op_addr.rs2addr, data: op2_inst1, epochs: wEpoch
                        `ifdef no_wawstalls ,id: ? `endif
                        `ifdef spfpu ,rdtype: (decoded_inst[1].op_type.rs2type==FloatingRF)?FRF:IRF `endif
                        };
      //let _op6 = FwdType{ valid: True, 
      //                    addr: 0,
      //                    data: zeroExtend(inst1_imm),
      //                    epochs: wEpoch
      //                  `ifdef spfpu ,rdtype: unpack(0) `endif 
      //                  `ifdef no_wawstalls ,id : ? `endif };
      let _op6 = inst1_imm;
                          
      rg_op1[0] <= _op1;
      rg_op2[0] <= _op2;
      //rg_op2type[0] <= decoded_inst[0].op_type.rs2type;
      wr_op2type <= decoded_inst[0].op_type.rs2type;
      rg_op3[0] <= _op3;
      rg_op4[0] <= _op4;
      rg_op5[0] <= _op5;
      wr_op5type <= IntegerRF;
      rg_op6 <= _op6;

      `logLevel( stage2, 0, fstage2( hartid, 0, _op1, decoded_inst[0].op_type.rs1type, 
                _op2, decoded_inst[0].op_type.rs2type, _op3, instrType[0], stage3meta[0], mtval[0] ), wr_simulate_log_start)
      `logLevel( stage2, 0, fstage2( hartid, 1, _op4, decoded_inst[1].op_type.rs1type, 
                _op5, decoded_inst[1].op_type.rs2type, _op3, instrType[1], stage3meta[1], mtval[1] ), wr_simulate_log_start)
      // -------------------------------------------------------------------------------------- //
      if (issue_two_inst && rx_pipe1.u.deqReady_2()) begin
        wr_dual_issued <= pack(issue_two_inst);
        `logLevel( stage2, 0, $format("[%2d]STAGE2: Issuing two independent instructions", hartid), wr_simulate_log_start)
        rx_pipe1.u.deq(2); // TODO: Change it back
      `ifdef rtldump
        rx_commitlog.u.deq(2);
      `endif
      end
      else begin
        rx_pipe1.u.deq(1);
      `ifdef rtldump
        rx_commitlog.u.deq(1);
      `endif
      end
    end

    `ifdef perfmonitors
      if (!rx_pipe1.u.deqReady_2())
        wr_one_instr <= 1;
    `endif
  endrule

  /*doc:rule: */
  rule rl_wait_for_interrupt(rg_wfi);
    if(wr_resume_wfi || wr_flush_from_wb || wr_flush_from_exe)
      rg_wfi <= False;
  endrule


  // interface to send decoded structs to the next stage.
  interface tx = interface Ifc_s2_tx
    interface tx_meta_to_stage3   = tx_meta.e;
    interface tx_mtval_to_stage3  = tx_mtval.e;
    interface tx_instrtype_to_stage3 = tx_instrtype.e;
    interface tx_opmeta_to_stage3= tx_opmeta.e;
  `ifdef rtldump
    interface tx_commitlog= tx_commitlog.e;
  `endif
  endinterface;

  interface rx = interface Ifc_s2_rx
  `ifdef rtldump
    interface rx_commitlog = rx_commitlog.e;
  `endif
	  interface rx_from_stage1 = rx_pipe1.e;
	endinterface;

  interface common = interface Ifc_s2_common
    method Action ma_csrs (CSRtoDecode csr);
      wr_csrs <= csr;
    endmethod

    `ifdef simulate
      method Action ma_simulate_log_start(Bit#(1) start);
        wr_simulate_log_start <= start;
      endmethod
    `endif
  
    /*doc:method: This method is use to perform the commit to the registerfile. This method is also
    * used to update the operands presented by the registerfile to the subsequent stage. This update
    * could occur either in the same cycle as the operands are being read from the RF or at a later
    * stage. This in some sense mimics a bypass registerfile implementation. 
    * One might expect that the stage3 shall capture all commits that the current instruction depends
    * on. However, stage3 can be stalled and may be prevented from firing for multiple reasons. Under
    * those scenarios its necessary that the operands from this stage are updated. This allows us to
    * maintain a single register state for each operand source.
    */
  	method Action ma_commit_rd (Vector#(`num_issue, CommitData) commit);
      `logLevel( stage2, 1, $format("[%2d]STAGE2: ",hartid,fshow(commit)), wr_simulate_log_start)
      registerfile.commit_rd(commit);
      //if (!commit[0].unlock_only) 
      //  registerfile.commit_rd_1(commit[0]);
      //if (!commit[1].unlock_only) 
      //  registerfile.commit_rd_2(commit[1]);
   
      //if ( || !commit[0].unlock_only) begin
      `ifdef spfpu
        if(!commit[1].unlock_only && commit[1].addr == rg_op1[1].addr && commit[1].rdtype == rg_op1[1].rdtype) begin
          let _x = rg_op1[1];
          if(commit[1].rdtype == FRF || rg_op1[1].addr!=0)
            _x.data=commit[1].data;
          rg_op1[1] <= _x;
        end
        else if(!commit[0].unlock_only && commit[0].addr == rg_op1[1].addr && commit[0].rdtype == rg_op1[1].rdtype) begin
          let _x = rg_op1[1];
          if(commit[0].rdtype == FRF || rg_op1[1].addr!=0)
            _x.data=commit[0].data;
          rg_op1[1] <= _x;
        end
    
        if(!commit[1].unlock_only && commit[1].addr == rg_op2[1].addr && commit[1].rdtype == rg_op2[1].rdtype) begin
          let _x = rg_op2[1];
          if(commit[1].rdtype == FRF || (rg_op2[1].addr != 0 && wr_op2type == IntegerRF))
            _x.data=commit[1].data;
          rg_op2[1] <= _x;
        end
        else if(!commit[0].unlock_only && commit[0].addr == rg_op2[1].addr && commit[0].rdtype == rg_op2[1].rdtype) begin
          let _x = rg_op2[1];
          if(commit[0].rdtype == FRF || (rg_op2[1].addr != 0 && wr_op2type == IntegerRF))
            _x.data=commit[0].data;
          rg_op2[1] <= _x;
        end
   
        //TODO DUAL ISSUE: Need to consider commit output of 2nd pipe inst also when float units are replicated.
        if(rg_op3[1].addr == commit[0].addr && rg_op3[1].rdtype == FRF &&  commit[0].rdtype == FRF)
          rg_op3[1].data <= commit[0].data;
    
        if(!commit[1].unlock_only && commit[1].addr == rg_op4[1].addr && commit[1].rdtype == rg_op4[1].rdtype) begin
          let _x = rg_op4[1];
          if(commit[1].rdtype == FRF || rg_op4[1].addr!=0)
            _x.data=commit[1].data;
          rg_op4[1] <= _x;
        end
        else if(!commit[0].unlock_only && commit[0].addr == rg_op4[1].addr && commit[0].rdtype == rg_op4[1].rdtype) begin
          let _x = rg_op4[1];
          if(commit[0].rdtype == FRF || rg_op4[1].addr!=0)
            _x.data=commit[0].data;
          rg_op4[1] <= _x;
        end

        if(!commit[1].unlock_only && commit[1].addr == rg_op5[1].addr && commit[1].rdtype == rg_op5[1].rdtype) begin
          let _x = rg_op5[1];
          if(commit[1].rdtype == FRF || (rg_op5[1].addr != 0 && wr_op5type == IntegerRF))
            _x.data=commit[1].data;
          rg_op5[1] <= _x;
        end
        else if(!commit[0].unlock_only && commit[0].addr == rg_op5[1].addr && commit[0].rdtype == rg_op5[1].rdtype) begin
          let _x = rg_op5[1];
          if(commit[0].rdtype == FRF || (rg_op5[1].addr != 0 && wr_op5type == IntegerRF))
            _x.data=commit[0].data;
          rg_op5[1] <= _x;
        end

      `else
        //TODO DUAL ISSUE: Add 2nd instruction commit constructs for the time when FPU is not enabled as well
        let _x = rg_op1[1];
        let _y = rg_op2[1];
        if(rg_op1[1].addr == commit[0].addr && rg_op1[1].addr!=0) begin
          _x.data = commit[0].data;
          rg_op1[1] <= _x;
        end
    
        if(rg_op2[1].addr == commit[0].addr && rg_op2[1].addr!=0 && rg_op2type[1] == IntegerRF)
          _y.data = commit[0].data;
          rg_op2[1] <= _y;
      `endif
    //end
    endmethod

    // This method will get activated when there is a flush from the execute stage
  	method Action ma_update_eEpoch;
      wr_flush_from_exe <= True;
  		eEpoch<=~eEpoch;
  	endmethod
  
    // This method gets activated when there is a flush from the write - back stage.
  	method Action ma_update_wEpoch;
      wr_flush_from_wb <= True;
  		wEpoch<=~wEpoch;
  	endmethod
  
    method Action ma_clear_stall (Bool upd) if(rg_stall);
      if(upd) begin
        rg_stall <= False;
        rg_microtrap <= False;
      end
    endmethod
  
    /*doc:method: */
    method Action ma_resume_wfi (Bool w);
      wr_resume_wfi <= w;
    endmethod
  endinterface;

  interface rf = interface Ifc_s2_rf
    method mv_op1 = rg_op1[0];
    method mv_op2 = rg_op2[0];
    method mv_op3 = rg_op3[0];
    method mv_op4 = rg_op4[0];
    method mv_op5 = rg_op5[0];
    method mv_op6 = rg_op6;
  endinterface;
  method mv_wfi_detected = rg_wfi;
	

`ifdef debug
  interface debug = interface Ifc_s2_debug
    method Action debug_status (DebugStatus status);
      wr_debug_info <= status;
    endmethod
  endinterface;
`endif

`ifdef perfmonitors
  interface perf = interface Ifc_s2_perfmonitors
    method mv_instr_queue_empty = pack(!rx_pipe1.u.deqReady_1);
    method mv_dual_issued = wr_dual_issued;
    method mv_raw_hazard = wr_raw_hazard;
    method mv_raw_hazard_dual = wr_raw_hazard_dual;
    method mv_one_instr = wr_one_instr;
    method mv_mul_branch_hazard = wr_mul_branch_hazard;
    method mv_mul_mem_hazard = wr_mul_mem_hazard;
    method mv_mul_float_hazard = wr_mul_float_hazard;
    method mv_mul_mul_hazard = wr_mul_mul_hazard;
    method mv_mem_mem_hazard = wr_mem_mem_hazard;
    method mv_mem_branch_hazard = wr_mem_branch_hazard;
    method mv_mem_float_hazard = wr_mem_float_hazard;
    method mv_float_branch_hazard = wr_float_branch_hazard;
    method mv_float_float_hazard = wr_float_float_hazard;
    method mv_branch_branch_hazard = wr_branch_branch_hazard;
  endinterface;
`endif

endmodule
endpackage
