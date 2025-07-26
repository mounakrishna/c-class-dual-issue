// See LICENSE.iitm for license details
/*
Author: IIT Madras
Created on: Friday 11 June 2021 03:30:43 PM

*/
/*doc:overview
This module is the execution stage of the pipeline. This stage is also where the operand bypass
happens. 

Operand Bypass
^^^^^^^^^^^^^^
The module receives the operands from the registerfile (always holding the latest values as the
registerfile acts as a bypass-registerfile). The module also has access to the current scoreboard
which indicates if there exists a an instruction in the further stages of the pipeline with a
potential new value of the operand.

The sources of bypass include the head of the ISB between EXE-MEM and the head of the ISB between
MEM-WB. The third source of the bypass is the registerfile itself.

The Bypass is done for rs1 and rs2. It is also done for rs3 when the F/D extensions are enabled.

The bypass module will indicate if the respective operand is available to initiate execution or not.
When waw stalls are disabled, then checks on the bypass packets from the ISB will also include
checking if the bypass register id matches the corresponding id from the scoreboard.

Scoreboard
^^^^^^^^^^

The scoreboard is also maintained in this module. Whenever there is an instruction with a valid
destination register (i.e. rd != x0), then the corresponding bit in the scoreboard is locked by
setting the lock bit to one. If waw-stalls are disabled, then a new id is allocated to the
destination register in the scoreboard. This id will be used for comparison in the above Operand
Bypass phase.

If waw-stalls are not disabled, then the instructions whose destination register is already locked
in the scoreboard will stall untill an unlock happens.


Execution Units
^^^^^^^^^^^^^^^

The module is divided into multiple rules. Each rule is defined to handle a specific category of
instructions:
- Base-ALU: This executes basic arithmetic, logic and shift operations
- Muldiv : This will offload the mul-div operations to the mbox
- trap : this is simply transfer the instruction tagged as trap to the next stage
- branch: This will handle all the control instructions and the mispredictions if any
- memory: This will offload the memory operations to the cache/data subsystem.

the functional units are provided the inputs only when the the FUs are free and the operands are
available, else the respective rules maye fire, but may not perform any action.

compile-macros:
  - muldiv : if set, then the muldive instructions are offloaded from this module and respective
    interfaces are also instantiated
  - bpu: if set then misprediction is detected and interfaces to send the training data to the
    branch predictor are also instantiated
  - perfmonitors: when set, the performance monitoring event signals are instantiated to track
    specific events
  - stage3_noinline: when set, a separate verilog file is created for the mkstage3 module
  - rtldump: when set, logic to generate the instruction trace dump is enabled.
  - no_wawstalls: when set, no stalls are generated due to a WAW hazard. 
  - RV64: used to enable 32-bit wordops in 64-bit mode. Also detect misaligned traps
  - compressed : used to increment the pc by +2 or +4 depending on the instruction. This is required
    for jump instructinos to calculate the next logical pc value to be stored in the destination reg.
  -
*/
package stage3 ;
// -- package imports --//
import GetPut         :: * ;
import FIFOF          :: * ;
import SpecialFIFOs   :: * ;
import DReg           :: * ;
import TxRx           :: * ;
import Vector         :: * ;
import ConfigReg      :: * ;
import Probe          :: * ;      // used to retain some signals for simulation purposes

// -- project imports --//
import base_alu       :: * ;                // implements the ALU function
import bypass         :: * ;       // provides the operand bypassing logic
import ccore_types    :: * ;       // for pipe - line types
import dcache_types   :: * ;          // for dmem request types
import pipe_ifcs      :: * ;      // implements the pipeline stage interfaces
import scoreboard     :: * ;      // implements the scoreboard
`ifdef muldiv
  import mbox           :: * ;
`endif

`include "ccore_params.defines"  // for core parameters
`include "Logger.bsv"         // for logging display statements.
`include "trap.defines"           // for cause values of traps captured in this stage


interface Ifc_stage3;
  /*doc:subifc: This interface contains generic methods like epoch updates, flush from this stage,
   * csr value reads, etc*/
  interface Ifc_s3_common common;

  /*doc:subifc: Contains the decoded meta information from the previous stage*/
  interface Ifc_s3_rx rx;

  /*doc:subifc: interface to send information to the next stage*/
  interface Ifc_s3_tx tx;

  /*doc:subifc: interface to accepts the registerfile values*/
  interface Ifc_s3_rf rf;

  /*doc:subifc: Interface to send request to data cache and also another method to check if data
   * cache is free*/
  interface Ifc_s3_cache cache;

  /*doc:subifc: interface to accept the bypass signals from various downstream ISBs*/
  interface Ifc_s3_bypass bypass;
`ifdef bpu
  /*doc:subifc: interface to train the branch predictor on branch or jump instruction */
  interface Ifc_s3_bpu bpu;
`endif

`ifdef muldiv
  /*doc:subifc: interface to the multiplication and division unit*/
  interface Ifc_s3_muldiv muldiv;
`endif
`ifdef spfpu
  /*doc:subifc: interface to the multiplication and division unit*/
  interface Ifc_s3_float float;
`endif
`ifdef perfmonitors
  interface Ifc_s3_perfmonitors perfmonitors;
`endif
endinterface:Ifc_stage3

`ifdef stage3_noinline
(*synthesize*)
`endif
// the following attributes is used to detect when a structural hazard occurs. This is useful only
// when perfmonitors is enabled or simulate is enabled at compile time. If neither is implemented
// then the rule will be empty and the compiler should be removing it thereby causing no harm.
(*preempts="rl_drop_instr, rl_update_pipeline"*)
(*preempts="rl_exe_base_arith,rl_structural_stalls"*)
(*preempts="rl_drop_instr, rl_structural_stalls"*)
(*preempts="rl_system_instr, rl_structural_stalls"*)
(*preempts="rl_trap_from_prev, rl_structural_stalls"*)
(*preempts="rl_exe_base_arith, rl_structural_stalls"*)
(*preempts="rl_exe_base_memory, rl_structural_stalls"*)
(*preempts="rl_exe_base_control, rl_structural_stalls"*)
(*conflict_free="rl_exe_base_control, rl_exe_base_arith_1"*)
(*conflict_free="rl_exe_base_control, rl_exe_base_arith"*)
(*conflict_free="rl_exe_base_control, rl_exe_base_memory"*)
(*conflict_free="rl_exe_base_control, rl_2nd_instruction_invalid"*)
`ifdef muldiv
(*preempts="rl_mbox, rl_structural_stalls"*)
(*conflict_free="rl_exe_base_control, rl_mbox"*)
`endif
`ifdef spfpu
(*preempts="rl_fbox, rl_structural_stalls"*)
(*conflict_free="rl_exe_base_control, rl_fbox"*)
`endif
module mkstage3#(parameter Bit#(`xlen) hartid) (Ifc_stage3);

  String stage3=""; // defined for logger

  // --------------------- Start instantiations ------------------------//

  /*doc:submodule: instantiating the scoreboard module */
  Ifc_scoreboard sboard <- mkscoreboard(hartid);

`ifdef muldiv
  /*doc:wire: wire to drive the inputs to the mul-div unit*/
  Wire#(MBoxIn) wr_muldiv_inputs <- mkWire();
  /*doc:wire: wire to check if the multiplier is ready to accept new inputs*/
  Wire#(Bool) wr_mul_ready<- mkWire();
  /*doc:wire: wire to check if the divider is ready to accept new inputs*/
  Wire#(Bool) wr_div_ready<- mkWire();
`endif

  // rx fifos to receive the decoded information and the operands from the RF.
  RX#(Vector#(`num_issue, Stage3Meta))         rx_meta   <- mkRX;
  RX#(Vector#(`num_issue, Bit#(`xlen)))         rx_mtval   <- mkRX;
  RX#(Vector#(`num_issue, Instruction_type))   rx_instrtype   <- mkRX;
  RX#(OpMeta)             rx_opmeta   <- mkRX;

  /*doc:wire: reads operand-1 from the registerfile which was indexed in the previous cycle*/
  Wire#(FwdType) wr_rf_op1 <- mkWire();
  /*doc:wire: reads operand-2 from the registerfile which was indexed in the previous cycle*/
  Wire#(FwdType) wr_rf_op2 <- mkWire();
  /*doc:wire: reads operand-3/immediate from the registerfile which was indexed in the previous cycle*/
  Wire#(FwdType) wr_op3 <- mkWire();
  /*doc:wire: reads operand-3/immediate from the registerfile which was indexed in the previous cycle*/
  Wire#(Bit#(32)) wr_op6 <- mkWire();
  /*doc:wire: reads operand-1 from the registerfile which was indexed in the previous cycle*/
  Wire#(FwdType) wr_rf_op4 <- mkWire();
  /*doc:wire: reads operand-2 from the registerfile which was indexed in the previous cycle*/
  Wire#(FwdType) wr_rf_op5 <- mkWire();

  /*doc:wire: wire to hold the current privilege mode*/
  Wire#(Bit#(2)) wr_priv <- mkWire();
  /*doc:wire: wire to hold the current value of hstatus*/
  Wire#(Bit#(`xlen)) wr_mstatus <- mkWire();

`ifdef hypervisor
  /*doc:wire: wire to hold the current virtual mode*/
  Wire#(Bit#(1)) wr_vs_mode <- mkWire();
  /*doc:wire: wire to hold the current value of hstatus*/
  Wire#(Bit#(`xlen)) wr_hstatus <- mkWire();
`endif

  /*doc:wire: The vector of all bypass values coming from varios ISBs. The lower index indicates
  * bypass from the youngest instruction and the highest index indicates bypass from the oldest
  * instruction*/
  Wire#(Vector#(`bypass_sources, Vector#(`num_issue, FwdType))) wr_bypass <- mkWire();

`ifdef rtldump
  // rx fifo to receive the instruction sequence for rtl.dump feature.
  RX#(Vector#(`num_issue, CommitLogPacket)) rx_commitlog <- mkRX;
  // tx fifo to send the instructino sequence for rtl.dump feature.
  TX#(Vector#(`num_issue, CommitLogPacket)) tx_commitlog <- mkTX;
`endif

`ifdef bpu
  /*doc:wire: wire holding the PC value of the next instruction fetched into the pipe*/
  Wire#(Maybe#(Bit#(`vaddr))) wr_next_pc <- mkDWire(tagged Invalid);

  /*doc:reg: Register to send the training for the BTB on conditional branches.*/
  Reg#(Maybe#(Training_data)) wr_training_data <- mkDReg(tagged Invalid);
  // Wire to send the return - address on the stack.
`ifdef gshare
  // on a misprediction, this register contains the reset global history value and whethr the btb
  // was a hit or miss during prediction.
  Reg#(Maybe#(Tuple2#(Bool, Bit#(`histlen)))) wr_mispredict_ghr <- mkDReg( tagged Invalid);
`endif
`endif

  // transmit interfaces for MEM stage.
  //TX#(Vector#(`num_issue, BaseOut))        tx_baseout <- mkTX;
  //TX#(Vector#(`num_issue, TrapOut))        tx_trapout <- mkTX;
  //TX#(Vector#(`num_issue, SystemOut))      tx_systemout <- mkTX;
  //TX#(Vector#(`num_issue, MemoryOut))      tx_memoryout <- mkTX;
  TX#(Vector#(`num_issue, FUid))           tx_fuid <- mkTX;

  Vector#(`num_issue, Wire#(FUid)) wr_fuid <- replicateM(mkWire);

  Vector#(`num_issue, Wire#(CommitLogPacket)) wr_commitlog <- replicateM(mkWire);

  Vector#(`num_issue, Wire#(SBDUpd)) wr_lock <- replicateM(mkWire);

  //Wire#(Bool) wr_instr0_trap <- mkWire;

  Wire#(Vector#(`num_issue, Bit#(`wawid))) wr_id <- mkWire();

  /*doc:wire: holds value of operand1 after checking the bypass signals from downstream isbs and
  * regfile*/
  Wire#(Bit#(`xlen)) wr_fwd_op1 <- mkWire();

  /*doc:wire: holds value of operand2 after checking the bypass signals from downstream isbs and
  * regfile*/
  Wire#(Bit#(`xlen)) wr_fwd_op2 <- mkWire();

  /*doc:wire: holds value of operand1 after checking the bypass signals from downstream isbs and
  * regfile*/
  Wire#(Bit#(`xlen)) wr_fwd_op4 <- mkWire();

  /*doc:wire: holds value of operand2 after checking the bypass signals from downstream isbs and
  * regfile*/
  Wire#(Bit#(`xlen)) wr_fwd_op5 <- mkWire();
`ifdef spfpu
  /*doc:wire: holds value of operand3 after checking the bypass signals from downstream isbs and
  * regfile*/
  Wire#(Bit#(`xlen)) wr_fwd_op3 <- mkWire();
  Wire#(Input_Packet) wr_float_inputs <- mkWire();
`endif
  /*doc:wire: after checking the bypass signals from downstream ISBs, this wire indicates if the
  * latest value of operand1 is available or not. If not then we need stall on instructions waiting
  * for this value.*/
  Wire#(Bool) wr_op1_avail <- mkWire();
  Probe#(Bool) wr_op1_avail_probe <- mkProbe();


  /*doc:wire: after checking the bypass signals from downstream ISBs, this wire indicates if the
  * latest value of operand2 is available or not. If not then we need stall on instructions waiting
  * for this value.*/
  Wire#(Bool) wr_op2_avail <- mkWire();
  Probe#(Bool) wr_op2_avail_probe <- mkProbe();

  /*doc:wire: after checking the bypass signals from downstream ISBs, this wire indicates if the
  * latest value of operand1 is available or not. If not then we need stall on instructions waiting
  * for this value.*/
  Wire#(Bool) wr_op4_avail <- mkWire();
  Probe#(Bool) wr_op4_avail_probe <- mkProbe();


  /*doc:wire: after checking the bypass signals from downstream ISBs, this wire indicates if the
  * latest value of operand2 is available or not. If not then we need stall on instructions waiting
  * for this value.*/
  Wire#(Bool) wr_op5_avail <- mkWire();
  Probe#(Bool) wr_op5_avail_probe <- mkProbe();

`ifdef spfpu
  /*doc:wire: after checking the bypass signals from downstream ISBs, this wire indicates if the
  * latest value of operand3 is available or not. If not then we need stall on instructions waiting
  * for this value.*/
  Wire#(Bool) wr_op3_avail <- mkWire();
  Probe#(Bool) wr_op3_avail_probe <- mkProbe();

  Wire#(Bool) wr_fbox_ready <- mkWire();
`endif

  /*doc:wire: This wire will indicate that the operands are available for the corresponding rules
  * to fire. */
  Wire#(Bool) wr_ops_avail <- mkWire();
  Probe#(Bool) wr_ops_avail_probe <- mkProbe();

  // The following registers are use to the maintain epochs from various pipeline stages:
  // writeback and execute stage.
	Reg#(Bit#(1)) rg_eEpoch <- mkConfigReg(0);
	Reg#(Bit#(1)) rg_wEpoch <- mkReg(0);

  // Wire sending redirection indication to the previous stages.
  Reg#(Bool) wr_flush_from_exe <- mkDWire(False);

  // Wire holding the new pc to be redirected to due to branches / jumps
  Reg#(Bit#(`vaddr)) wr_redirect_pc <- mkDWire(0);

  Wire#(DMem_request#(`vaddr, `elen, 1)) wr_memory_request <- mkWire;
  Wire#(Bool) wr_cache_avail <- mkWire;

  // wire holding the compressed bit of the misa csr
  Wire#(Bit#(1)) wr_misa_c <- mkWire();

  /*doc:wire: */
  Wire#(Bool) wr_waw_stall <- mkDWire(False);

  // This variable holds the current epoch values of the pipe
  let curr_epochs = {rg_eEpoch, rg_wEpoch};

`ifdef perfmonitors
  /*doc:wire: set to one when a float operation has been offloaded for execution*/
  Wire#(Bit#(1)) wr_count_floats <- mkDWire(0);
  /*doc:wire: set to one when a muldiv operation has been offloaded for execution*/
  Wire#(Bit#(1)) wr_count_muldiv <- mkDWire(0);
  /*doc:wire: set to one when a branch operation has been executed*/
  Wire#(Bit#(1)) wr_count_branches <- mkDWire(0);
  /*doc:wire: set to one when a jump operation has been executed*/
  Wire#(Bit#(1)) wr_count_jumps <- mkDWire(0);
  /*doc:wire: set to one when a stall occurs because of RAW hazard*/
  Wire#(Bit#(1)) wr_count_rawstalls <- mkDWire(0);
  /*doc:wire: set to one when a stall occurs because of a structural hazard*/
  Wire#(Bit#(1)) wr_count_exestalls <- mkDWire(0);
  /*doc:wire: set to one when the ISB between stage3 and stage4 is FULL.*/
  Wire#(Bit#(1)) wr_isb3_isb4_full <- mkDWire(0);
  Wire#(Bit#(1)) wr_st3_not_firing <- mkDWire(0);
`endif
`ifdef simulate
  Wire#(Bit#(1)) wr_simulate_log_start <- mkDWire(0);
`endif
  // ---------------------- End Instatiations --------------------------//
  let meta = rx_meta.u.first;
  let mtval   = rx_mtval.u.first;
  let opmeta = rx_opmeta.u.first;
  let instr_type = rx_instrtype.u.first;

  // create a generic variable for the common params. The id is what will be assigned when execution
  // happens
  let default_commitlog = CommitLogPacket { mode : unpack(?),
                                            pc : ?,
                                            instruction : ?,
                                            inst_type : tagged None
                                          `ifdef hypervisor
                                            ,v : ?
                                          `endif
                                          };
  Bool epochs_match = curr_epochs == meta[0].epochs;
  //Bool epochs_match_instr1 = (instr_type[1] != NONE) ? curr_epochs == meta[1].epochs : False;
`ifdef bpu
  let btbresponse = meta[0].btbresponse;
`endif
  // ---------------------- Start local function definitions ----------------//

  // this function will deque the response received from the previous stage. Instead of replicating
  // this code all over, best to have it as an in lined function called at relevant places.
  function Action deq_rx = action
    rx_meta.u.deq;
    rx_mtval.u.deq;
    rx_instrtype.u.deq;
    rx_opmeta.u.deq;
  `ifdef rtldump
    rx_commitlog.u.deq;
  `endif
  endaction;

  /*doc:rule: This rule will only fire when there is a pending instruction to be executed by the
  * corresponding unit rule is unable to fire due to structural hazards (cache not available,
  * downstream ISB is full, mul-div unit is busy, etc.*/
  rule rl_structural_stalls(rx_meta.u.notEmpty && wr_ops_avail);
  `ifdef perfmonitors
    wr_count_exestalls <= 1;
  `endif
    `logLevel( stage3, stall, $format("[%2d]STAGE3: Structural stall in EXE", hartid), wr_simulate_log_start)
  endrule:rl_structural_stalls

  /*doc:rule: This rule will set the perfmonitor ISB full when the ISB between stage3 and 
  stage4 is full.*/
  rule rl_isb_full(!tx_fuid.u.notFull);
    `ifdef perfmonitors
      wr_isb3_isb4_full <= 1;
    `endif
    `logLevel( stage3, stall, $format("[%2d]STAGE3: ISB3-4 is FULL", hartid), wr_simulate_log_start)
  endrule

  rule rl_stage3_not_firing(!rx_meta.u.notEmpty);
    `logLevel( stage3, stall, $format("[%2d]STAGE3: Not firing", hartid), wr_simulate_log_start)
    `ifdef perfmonitors
      wr_st3_not_firing <= 1;
    `endif
  endrule

`ifdef simulate
  rule rl_upd_log_start;
    sboard.ma_simulate_log_start(wr_simulate_log_start);
  endrule
`endif

  /*doc:rule: This rule performs the operand bypass for each operand. For each operand, this rule
  * will read all the values from the downstream FIFOs, feed them to the bypass module and check if
  * the operand is available or not. 
  * If the head of downstream ISBs don't have the latest operand,
  * but the scoreboard indicates that an instruction in the pipeline will soon update the operand,
  * then a stall is created. 
  * If the scoreboard indicates that the operand is not going to be updated be previous executed
  * instructions, then the bypass module will use the values from the registerfile as is and
  * initiate execution.
  */
  rule rl_perform_fwding(rx_meta.u.notEmpty && tx_fuid.u.notFull);
    `logLevel( stage3, 0, $format("[%2d]STAGE3: PC0:%h, instrType0:",hartid, meta[0].pc,fshow(instr_type[0])), wr_simulate_log_start)
    `logLevel( stage3, 0, $format("[%2d]STAGE3: PC1:%h, instrType1:",hartid, meta[1].pc,fshow(instr_type[1])), wr_simulate_log_start)
    
    // ----------------------- check for WAW hazard ------------------------------------------- //
    `ifdef no_wawstalls
      Bool lv_waw_stall = False;
    `else
      Bit#(`num_issue) sb_lock;
      for (Integer i=0; i<`num_issue; i=i+1) begin
        sb_lock[i] = sboard.mv_board.v_id[{ `ifdef spfpu pack(meta[i].rdtype==FRF), `endif meta.rd }];
      end
      Bool lv_waw_stall = unpack(|sb_lock);
      //let sb_index_inst0 = {`ifdef spfpu pack(meta.rdtype[0]==FRF), `endif meta.rd[0] };
      //let sb_index_inst1 = {`ifdef spfpu pack(meta.rdtype[1]==FRF), `endif meta.rd[1] };
      //let sb_inst0 = sboard.mv_board.v_id(sb_index_inst0);
      //let sb_inst1 = sboard.mv_board.v_id(sb_index_inst1);
      //Bool lv_waw_stall = unpack(sb_inst0.lock) && unpack(sb_inst1.lock);
    `endif
    wr_waw_stall <= lv_waw_stall ;
    // ---------------------------------------------------------------------------------------- //
  `ifdef spfpu
    RFType rf1type = `ifdef spfpu opmeta.rs1type == FloatingRF ? FRF : `endif IRF;
    RFType rf2type = `ifdef spfpu opmeta.rs2type == FloatingRF ? FRF : `endif IRF;
    RFType rf3type = opmeta.rs3type;
    RFType rf4type = `ifdef spfpu opmeta.rs4type == FloatingRF ? FRF : `endif IRF;
    RFType rf5type = `ifdef spfpu opmeta.rs5type == FloatingRF ? FRF : `endif IRF;
  `endif

    let sb_rs1_id = sboard.mv_board.v_id[{ `ifdef spfpu pack(rf1type==FRF), `endif opmeta.rs1addr}];
    let sb_rs2_id = sboard.mv_board.v_id[{ `ifdef spfpu pack(rf2type==FRF), `endif opmeta.rs2addr}];
    let sb_rs4_id = sboard.mv_board.v_id[{ `ifdef spfpu pack(rf4type==FRF), `endif opmeta.rs4addr}];
    let sb_rs5_id = sboard.mv_board.v_id[{ `ifdef spfpu pack(rf5type==FRF), `endif opmeta.rs5addr}];
    let sb_rs1_lock = sboard.mv_board.rf_lock[{ `ifdef spfpu pack(rf1type==FRF), `endif opmeta.rs1addr}];
    let sb_rs2_lock = sboard.mv_board.rf_lock[{ `ifdef spfpu pack(rf2type==FRF), `endif opmeta.rs2addr}];
    let sb_rs4_lock = sboard.mv_board.rf_lock[{ `ifdef spfpu pack(rf4type==FRF), `endif opmeta.rs4addr}];
    let sb_rs5_lock = sboard.mv_board.rf_lock[{ `ifdef spfpu pack(rf5type==FRF), `endif opmeta.rs5addr}];

    BypassReq req_addr1 = BypassReq{rd:opmeta.rs1addr, epochs: curr_epochs[0]
                    ,sb_lock: sb_rs1_lock
                    `ifdef no_wawstalls ,id: sb_rs1_id `endif
                    `ifdef spfpu ,rdtype: rf1type `endif };
    BypassReq req_addr2 = BypassReq{rd: opmeta.rs2addr, epochs: curr_epochs[0]
                    ,sb_lock: sb_rs2_lock
                    `ifdef no_wawstalls ,id: sb_rs2_id `endif
                    `ifdef spfpu ,rdtype: rf2type `endif };
    BypassReq req_addr4 = BypassReq{rd:opmeta.rs4addr, epochs: curr_epochs[0]
                    ,sb_lock: sb_rs4_lock
                    `ifdef no_wawstalls ,id: sb_rs4_id `endif
                    `ifdef spfpu ,rdtype: rf4type `endif };
    BypassReq req_addr5 = BypassReq{rd: opmeta.rs5addr, epochs: curr_epochs[0]
                    ,sb_lock: sb_rs5_lock
                    `ifdef no_wawstalls ,id: sb_rs5_id `endif
                    `ifdef spfpu ,rdtype: rf5type `endif };
    Vector#(`bypass_sources, Vector#(`num_issue, FwdType)) byp1, byp2, byp4, byp5;
    byp1[0] = wr_bypass[0];
    byp2[0] = wr_bypass[0];
    byp1[1] = wr_bypass[1];
    byp2[1] = wr_bypass[1];
    byp4[0] = wr_bypass[0];
    byp5[0] = wr_bypass[0];
    byp4[1] = wr_bypass[1];
    byp5[1] = wr_bypass[1];
    let {_op1_avail, _fwd_op1} = fn_bypass( req_addr1, byp1, wr_rf_op1);
    let {_op2_avail, _fwd_op2} = fn_bypass( req_addr2, byp2, wr_rf_op2);
    let {_op4_avail, _fwd_op4} = fn_bypass( req_addr4, byp4, wr_rf_op4);
    let {_op5_avail, _fwd_op5} = fn_bypass( req_addr5, byp5, wr_rf_op5);

    `logLevel(stage3, 3, $format("[%2d]STAGE3: Op1: ",hartid, fshow(byp1), " req:", fshow(req_addr1), " rf_data:", fshow(wr_rf_op1)), wr_simulate_log_start)
    `logLevel(stage3, 3, $format("[%2d]STAGE3: Op2: ",hartid, fshow(byp2), " req:", fshow(req_addr2), " rf_data:", fshow(wr_rf_op2)), wr_simulate_log_start)
    `logLevel(stage3, 3, $format("[%2d]STAGE3: Op4: ",hartid, fshow(byp4), " req:", fshow(req_addr4), " rf_data:", fshow(wr_rf_op4)), wr_simulate_log_start)
    `logLevel(stage3, 3, $format("[%2d]STAGE3: Op5: ",hartid, fshow(byp5), " req:", fshow(req_addr5), " rf_data:", fshow(wr_rf_op5)), wr_simulate_log_start)

    wr_op1_avail <= _op1_avail; wr_fwd_op1 <= _fwd_op1;
    wr_op1_avail_probe <= _op1_avail;

    wr_op2_avail <= _op2_avail; wr_fwd_op2 <= _fwd_op2;
    wr_op2_avail_probe <= _op2_avail;

    wr_op4_avail <= _op4_avail; wr_fwd_op4 <= _fwd_op4;
    wr_op4_avail_probe <= _op4_avail;

    wr_op5_avail <= _op5_avail; wr_fwd_op5 <= _fwd_op5;
    wr_op5_avail_probe <= _op5_avail;

  `ifdef spfpu
    `ifdef no_wawstalls
    let sb_rs3_id = sboard.mv_board.v_id[{ `ifdef spfpu pack(rf3type==FRF), `endif opmeta.rs3addr}];
    let sb_rs3_lock = sboard.mv_board.rf_lock[{ `ifdef spfpu pack(rf3type==FRF), `endif opmeta.rs3addr}];
    `endif
    BypassReq req_addr3 = BypassReq{rd:opmeta.rs3addr, epochs: curr_epochs[0]
                    ,sb_lock: sb_rs3_lock 
                    `ifdef no_wawstalls ,id: sb_rs3_id `endif
                    `ifdef spfpu ,rdtype: rf3type `endif };
    Vector#(`bypass_sources, Vector#(`num_issue, FwdType)) byp3;
    byp3[0] = wr_bypass[0];
    byp3[1] = wr_bypass[1];

    `logLevel(stage3,3, $format("[%2d]STAGE3: Op3: ",hartid, fshow(byp3), " req:", fshow(req_addr3)), wr_simulate_log_start)
    let {_op3_avail, _fwd_op3} = fn_bypass( req_addr3, byp3, wr_op3);
    wr_op3_avail <= (rf3type==IRF || _op3_avail); wr_fwd_op3 <= _fwd_op3;
    wr_op3_avail_probe <= _op3_avail;
  `endif
    `logLevel( stage3, 3, $format("[%2d]STAGE3: ",hartid, fshow(sboard.mv_board)), wr_simulate_log_start)
    if (lv_waw_stall)begin
      `logLevel( stage3, 0, $format("[%2d]STAGE3: WAW Stall", hartid), wr_simulate_log_start)
    end
    else begin
      `logLevel( stage3, 0, $format("[%2d]STAGE3: Bypass Op1:%2d Op1Avail:%b Op1Val:%h",
          hartid, opmeta.rs1addr, _op1_avail, _fwd_op1), wr_simulate_log_start)
      `logLevel( stage3, 0, $format("[%2d]STAGE3: Bypass Op2:%2d Op2Avail:%b Op2Val:%h",
          hartid, opmeta.rs2addr, _op2_avail, _fwd_op2), wr_simulate_log_start)
    `ifdef spfpu
      `logLevel( stage3, 0, $format("[%2d]STAGE3: Bypass Op3:%2d Op3Avail:%b Op3Val:%h",
          hartid, opmeta.rs3addr, _op3_avail, _fwd_op3), wr_simulate_log_start)
      `logLevel( stage3, 0, $format("[%2d]STAGE3: imm:",hartid, fshow(wr_op3)), wr_simulate_log_start)
    `endif
      `logLevel( stage3, 0, $format("[%2d]STAGE3: Bypass Op4:%2d Op4Avail:%b Op4Val:%h",
          hartid, opmeta.rs4addr, _op4_avail, _fwd_op4), wr_simulate_log_start)
      `logLevel( stage3, 0, $format("[%2d]STAGE3: Bypass Op5:%2d Op5Avail:%b Op5Val:%h",
          hartid, opmeta.rs5addr, _op5_avail, _fwd_op5), wr_simulate_log_start)
    end
  endrule:rl_perform_fwding


  /*doc:rule: This rule will compute whether all the required operands are available for 
  * computation. The updated wire will be the firing condition for all the rules which are meant 
  * for computation.
  */
  rule rl_operands_available(epochs_match);
    Bool ops_instr1_avail;
    if (instr_type[1] == ALU || instr_type[1] == BRANCH || instr_type[1] == JAL || instr_type[1] == JALR) 
      if (wr_op4_avail && wr_op5_avail) 
        ops_instr1_avail = True;
      else 
        ops_instr1_avail = False;
    else
      ops_instr1_avail = True;

    if (instr_type[0] == TRAP) begin
      wr_ops_avail <= True;
      wr_ops_avail_probe <= True;
    end
    else if (instr_type[0] == SYSTEM_INSTR && wr_op1_avail) begin
      wr_ops_avail <= True;
      wr_ops_avail_probe <= True;
    end
    else if ((instr_type[0] == ALU || instr_type[0] == MULDIV || instr_type[0] == MEMORY 
              || instr_type[0] == JALR || instr_type[0] == JAL || instr_type[0] == BRANCH) &&
             wr_op1_avail && wr_op2_avail && ops_instr1_avail) begin
      wr_ops_avail <= True;
      wr_ops_avail_probe <= True;
    end
    else if (instr_type[0] == FLOAT && 
             wr_op1_avail && wr_op2_avail && wr_op3_avail && ops_instr1_avail) begin
      wr_ops_avail <= True;
      wr_ops_avail_probe <= True;
    end
    else begin
      `logLevel( stage3, 4, $format("[%2d]STAGE3: SBD: ",hartid,fshow(sboard.mv_board)), wr_simulate_log_start)
      `logLevel( stage3, 0, $format("[%2d]STAGE3: Waiting for operands to be available \n op1_avail: %h, op2_avail: %h, op3_avail: %h, op4_avail: %h, op5_avail: %h",hartid, wr_op1_avail, wr_op2_avail, wr_op3_avail, wr_op4_avail, wr_op5_avail), wr_simulate_log_start)
      wr_ops_avail <= False;
      wr_ops_avail_probe <= False;
      `ifdef perfmonitors
        wr_count_rawstalls <= 1;
      `endif
    end
  endrule

//`ifdef perfmonitors
//  rule rl_update_rawstalls;
//    if (instr_type[0] == ALU && instr_type[1] == ALU 
//      && epochs_match_instr0 && epochs_match_instr1 &&
//      !(wr_op1_avail && wr_op2_avail && wr_op4_avail && wr_op5_avail))
//      wr_count_rawstalls <= 1;
//    else if (instr_type[0] == ALU && epochs_match_instr0 &&
//           !(wr_op1_avail && wr_op2_avail)) 
//      wr_count_rawstalls <= 1;
//    else if (instr_type[0] == MULDIV && epochs_match_instr0 &&
//            ((meta[0].funct[2]==0 && wr_mul_ready) || (meta[0].funct[2]==1 && wr_div_ready)) && 
//           !(wr_op1_avail && wr_op2_avail)) 
//      wr_count_rawstalls <= 1;
//    else if (instr_type[0] == FLOAT && epochs_match_instr0 && wr_fbox_ready && 
//           !(wr_op1_avail && wr_op2_avail && wr_op3_avail)) 
//      wr_count_rawstalls <= 1;
//    else if (instr_type[0] == SYSTEM_INSTR && epochs_match_instr0 &&
//           !(wr_op1_avail)) 
//      wr_count_rawstalls <= 1;
//    else if (instr_type[0] == MEMORY && epochs_match_instr0 && wr_cache_avail &&
//           !(wr_op1_avail && wr_op2_avail)) 
//      wr_count_rawstalls <= 1;
//    else if ((instr_type[0] == JALR || instr_type[0] == JAL || instr_type[0] == BRANCH) 
//            && epochs_match_instr0 && wr_redirection &&
//           !(wr_op1_avail && wr_op2_avail)) 
//      wr_count_rawstalls <= 1;
//    else 
//      wr_count_rawstalls <= 0;
//  endrule
//`endif

  /*doc:rule: When epochs don't match the instruction is marked as DROP for all further pipeline
  * stages. We don't drop the instructin here itself, as it could have locked the rd in the
  * previous stage which require a release from here if dropped. Thus, we let the write-back stage
  * drop the instruction and release the destination register as well.
  * The other option of locking and performing the WAW stalls here itself has not been explored,
  * but intuition says it will add further to the critical path
  */
  rule rl_drop_instr(!(epochs_match));
    deq_rx;
    `logLevel( stage3, 0, $format("[%2d]STAGE3: NOPing instruction - epochs-mismatch",hartid), wr_simulate_log_start)
  endrule:rl_drop_instr

  /*doc:rule: This rule will fire when the epochs match the instruction has been decoded as a
  * system instruction from the previous stage. Only operand1 is required for these instructions
  * and thus a stall is created only if operand-1 is not available*/
  rule rl_system_instr(instr_type[0] == SYSTEM_INSTR && !wr_waw_stall && epochs_match && tx_fuid.u.notFull && wr_ops_avail);
    `logLevel( stage3, 0, $format("[%2d]STAGE3: System Op received.",hartid), wr_simulate_log_start)
    let systemout = SystemOut {funct3     : truncate(meta[0].funct),
                               lpc        : truncate(meta[0].pc),
                               rs1_imm    : meta[0].funct[2] == 1?zeroExtend(wr_op3.data[19 : 15]):
                                                             wr_fwd_op1,
                               csr_address : truncate(wr_op3.data) };
    //let _id <- sboard.ma_lock_rd(unpack({0, pack(SBDUpd{rd: meta[0].rd `ifdef spfpu ,rdtype: meta[0].rdtype `endif })}));
    wr_lock[0] <= SBDUpd{rd: meta[0].rd `ifdef spfpu ,rdtype: meta[0].rdtype `endif };
    //wr_instr0_trap <= False;
    let common_pkt = FUid{pc    : meta[0].pc,
                            rd    : meta[0].rd,
                            epochs : meta[0].epochs[0],
                            upper_instr: meta[0].upper_instr,
                            drop_instr: False,
                            insttype : SYSTEM,
                            instpkt: tagged SYSTEM systemout
                          `ifdef no_wawstalls 
                            ,id: ?
                          `endif
                          `ifdef spfpu
                            ,rdtype : meta[0].rdtype
                          `endif } ;

    //tx_systemout.u.enq(unpack({0, pack(systemout)}));
    //tx_fuid.u.enq(common_pkt);
    wr_fuid[0] <= common_pkt;
    //deq_rx;
    `logLevel( stage3, 0, $format("[%2d]STAGE3: System Op completed : ",hartid,fshow(systemout)), wr_simulate_log_start)
    `ifdef rtldump
      let clogpkt = rx_commitlog.u.first[0];
      //tx_commitlog.u.enq(clogpkt);
      wr_commitlog[0] <= clogpkt;
    `endif
  endrule:rl_system_instr

  /*doc:rule: This rule is fired if an instruction was tagged as trap by any of the previous
  * stages. No operand availability is required here*/
  rule rl_trap_from_prev(instr_type[0] == TRAP && epochs_match && tx_fuid.u.notFull && wr_ops_avail);
    TrapOut trapout = TrapOut {cause   : truncate(meta[0].funct),
                               mtval : mtval[0]
                             `ifdef hypervisor
                               ,mtval2 : 0
                             `endif
                             `ifdef microtrap_support
                                ,is_microtrap : meta[0].is_microtrap 
                             `endif };
    //let _id <- sboard.ma_lock_rd(unpack({0, pack(SBDUpd{rd: meta[0].rd `ifdef spfpu ,rdtype: meta[0].rdtype `endif })}));
    wr_lock[0] <= SBDUpd{rd: meta[0].rd `ifdef spfpu ,rdtype: meta[0].rdtype `endif };
    let common_pkt = FUid{pc    : meta[0].pc,
                           rd    : meta[0].rd,
                           epochs : meta[0].epochs[0],
                           upper_instr: meta[0].upper_instr,
                           drop_instr: False,
                           insttype : TRAP,
                           instpkt : tagged TRAP trapout
                         `ifdef no_wawstalls 
                           ,id: ?
                         `endif
                         `ifdef spfpu
                           ,rdtype : meta[0].rdtype
                         `endif } ;
    //common_pkt[0].insttype = TRAP;
    //tx_trapout.u.enq(unpack({0, pack(trapout)}));
    //tx_fuid.u.enq(unpack({0, pack(common_pkt)}));
    wr_fuid[0] <= common_pkt;
    //deq_rx;
    `logLevel( stage3, 0, $format("[%2d]STAGE3: Trap received and completed: ",hartid,fshow(trapout)), wr_simulate_log_start)
  `ifdef rtldump
    let clogpkt = rx_commitlog.u.first[0];
    //tx_commitlog.u.enq(clogpkt);
    wr_commitlog[0] <= clogpkt;
  `endif
  endrule: rl_trap_from_prev

  /*doc:rule: This rule is used to perform execution of base arithmetic ops. Both operands are
  * required to perform these operations. In case of 32-bit ops in RV64, the result is
  * sign-Extended version of the lower 32-bits results from the alu */
    rule rl_exe_base_arith(instr_type[0] == ALU && epochs_match && tx_fuid.u.notFull && !wr_waw_stall && wr_ops_avail);
      let alu_result = fn_base_alu(wr_fwd_op1, wr_fwd_op2, truncateLSB(meta[0].funct),
                                meta[0].pc, opmeta.rs1type==PC `ifdef RV64 ,meta[0].word32 `endif );

    //let alu_result = alu_result_two[0];
    `ifdef RV64
      if (meta[0].word32) begin
        alu_result = signExtend(alu_result[31:0]);
        //alu_result[1] = signExtend(alu_result[1][31:0]);
      end
    `endif
      let baseoutput = BaseOut{ rdvalue   : alu_result, rd: meta[0].rd , epochs: curr_epochs[0]
                          `ifdef spfpu ,fflags    : 0 , rdtype: meta[0].rdtype `endif };
      `logLevel( stage3, 0, $format("[%2d]STAGE3: Base ALU Op received in 1st pipe",hartid), wr_simulate_log_start)

      wr_lock[0] <= SBDUpd{rd: meta[0].rd `ifdef spfpu ,rdtype: meta[0].rdtype `endif };
      //let _id <- sboard.ma_lock_rd(lock);
      let common_pkt = FUid{pc    : meta[0].pc,
                           rd    : meta[0].rd,
                           epochs : meta[0].epochs[0],
                           upper_instr: meta[0].upper_instr,
                           drop_instr: False,
                           insttype : BASE,
                           instpkt: tagged BASE baseoutput
                           `ifdef no_wawstalls 
                             ,id: ?
                           `endif
                           `ifdef spfpu
                             ,rdtype : meta[0].rdtype
                           `endif } ;
      //tx_baseout.u.enq(baseoutput);
      //tx_fuid.u.enq(common_pkt);
      wr_fuid[0] <= common_pkt;
      //deq_rx;
      `logLevel( stage3, 0, $format("[%2d]STAGE3: Base ALU Op completed of 1st pipe: ",hartid,fshow(baseoutput)), wr_simulate_log_start)
      `ifdef rtldump
        let clogpkt = rx_commitlog.u.first[0];
        CommitLogReg _pkt =?;
        if (clogpkt.inst_type matches tagged REG .r)
          _pkt = r;
        _pkt.wdata = alu_result;
        clogpkt.inst_type = tagged REG _pkt;
        //tx_commitlog.u.enq(clogpkt);
        wr_commitlog[0] <= clogpkt;
      `endif
    endrule:rl_exe_base_arith

    /*doc:rule: The base ALU operations of the second instructions is done in this rule.*/
    rule rl_exe_base_arith_1(instr_type[1] == ALU && epochs_match && tx_fuid.u.notFull && !wr_waw_stall && wr_ops_avail);
      let alu_result = fn_base_alu(wr_fwd_op4, wr_fwd_op5, truncateLSB(meta[1].funct),
                                meta[1].pc, opmeta.rs4type==PC `ifdef RV64 ,meta[1].word32 `endif );

    //let alu_result = alu_result_two[0];
    `ifdef RV64
      if (meta[1].word32) begin
        alu_result = signExtend(alu_result[31:0]);
        //alu_result[1] = signExtend(alu_result[1][31:0]);
      end
    `endif
      let baseoutput = BaseOut{ rdvalue   : alu_result, rd: meta[1].rd , epochs: curr_epochs[0]
                          `ifdef spfpu ,fflags    : 0 , rdtype: meta[1].rdtype `endif };
      `logLevel( stage3, 0, $format("[%2d]STAGE3: Base ALU Op received in 2nd pipe",hartid), wr_simulate_log_start)

      wr_lock[1] <= SBDUpd{rd: meta[1].rd `ifdef spfpu ,rdtype: meta[1].rdtype `endif };
      //let _id <- sboard.ma_lock_rd(lock);
      let common_pkt = FUid{pc    : meta[1].pc,
                           rd    : meta[1].rd,
                           epochs : meta[1].epochs[0],
                           upper_instr: meta[1].upper_instr,
                           drop_instr: False,
                           insttype : BASE,
                           instpkt: tagged BASE baseoutput
                           `ifdef no_wawstalls 
                             ,id: ?
                           `endif
                           `ifdef spfpu
                             ,rdtype : meta[1].rdtype
                           `endif } ;
      //tx_baseout.u.enq(baseoutput);
      //tx_fuid.u.enq(common_pkt);
      wr_fuid[1] <= common_pkt;
      //deq_rx;
      `logLevel( stage3, 0, $format("[%2d]STAGE3: Base ALU Op completed in 2nd pipe: ",hartid,fshow(baseoutput)), wr_simulate_log_start)
      `ifdef rtldump
        let clogpkt = rx_commitlog.u.first[1];
        CommitLogReg _pkt =?;
        if (clogpkt.inst_type matches tagged REG .r)
          _pkt = r;
        _pkt.wdata = alu_result;
        clogpkt.inst_type = tagged REG _pkt;
        //tx_commitlog.u.enq(clogpkt);
        wr_commitlog[1] <= clogpkt;
      `endif
    endrule:rl_exe_base_arith_1

  /*doc:rule: This rule is fired when the epochs match and a memory operation needs to
  * be performed and when the cache is available for receiving new requests. This rule can also
  * detect load/store mis-aligned traps and can avoid issuing memory ops altogether.
  * The Valid checks and operations are offloaded only when both the operands are available.
  * In case of SFence, we do not expect an output/acknowledgement from the data subsystem, thus
  * SFence instruction henceforth will be treated as a regular nop instruction and avoiding
  * polling on the data subsystem in the subsequent pipeline stages.
  */
  rule rl_exe_base_memory(instr_type[0] == MEMORY && wr_cache_avail && epochs_match && tx_fuid.u.notFull && !wr_waw_stall && wr_ops_avail && 
    ((instr_type[1] != BRANCH && instr_type[1] != JAL && instr_type[1] != JALR) || isValid(wr_next_pc)));
    `logLevel( stage3, 0, $format("[%2d]STAGE3: Base Memory Op received",hartid), wr_simulate_log_start)
    Bit#(`vaddr) memory_address = wr_fwd_op1 + truncate(wr_op3.data);
    Bit#(3) funct3  = truncate(meta[0].funct);
    Bool trap = ((funct3[1 : 0] == 1 && memory_address[0] != 0)
                     || (funct3[1 : 0] == 2 && memory_address[1 : 0] != 0)
         `ifdef RV64 || (funct3[1 : 0] == 3 && memory_address[2 : 0] != 0) `endif );
    Bit#(`causesize) memory_cause = meta[0].memaccess == Load? `Load_addr_misaligned:
                                                           `Store_addr_misaligned ;
  `ifdef dpfpu
    Bit#(1) nanboxing = pack( funct3[1 : 0] == 2 && meta[0].rdtype == FRF);
  `endif

    // create a trap template
    TrapOut trapout = TrapOut {cause   : memory_cause, is_microtrap: False,
                               mtval : memory_address
                               `ifdef hypervisor , mtval2: ? `endif
                                };

    // craete the memory output response template
    let memoryout = MemoryOut{  memaccess   : meta[0].memaccess
       `ifdef rtldump `ifdef atomic ,atomicop    : {funct3[0], meta[0].funct[6:3]} `endif `endif
                 `ifdef dpfpu ,nanboxing   : nanboxing `endif } ;

    Bit#(1) mprv = wr_mstatus[17];
    Bit#(2) access_prv = mprv == 1?wr_mstatus[12:11]: wr_priv;
  `ifdef hypervisor
    Bool hvm_loadstore = unpack(meta[0].hvm_loadstore);
    Bit#(1) hlvx = meta[0].hlvx;
    Bit#(1) mpv = wr_mstatus[39];
    if (hvm_loadstore) access_prv = zeroExtend(wr_hstatus[8]);

    Bit#(1) access_virt = (mprv == 1 && mpv == 1 && access_prv!=3)?1: wr_vs_mode;
    if (hvm_loadstore) access_virt = 1;
  `endif
    let req = DMem_request{address      : memory_address,
                           epochs       : meta[0].epochs[0],
                           size         : funct3
                           ,fence       : meta[0].memaccess == FenceI || meta[0].memaccess == Fence
                           ,access      : truncate(pack(meta[0].memaccess))
                           ,writedata   : wr_fwd_op2
                           ,prv         : access_prv
                        `ifdef atomic ,atomic_op   : {funct3[0], meta[0].funct[6:3]} `endif
												`ifdef hypervisor
                            ,hfence     : meta[0].memaccess == HFence_VVMA || meta[0].memaccess == HFence_GVMA
                            ,virt       : access_virt
                            ,hlvx       : hlvx
                        `endif
                        `ifdef supervisor
                           ,sfence      : meta[0].memaccess == SFence
                           ,ptwalk_req  : False
                           ,ptwalk_trap : False
                        `endif } ;

    //let _id <- sboard.ma_lock_rd(unpack({0, pack(SBDUpd{rd: meta[0].rd `ifdef spfpu ,rdtype: meta[0].rdtype `endif })}));
    wr_lock[0] <= SBDUpd{rd: meta[0].rd `ifdef spfpu ,rdtype: meta[0].rdtype `endif };
    let common_pkt = FUid{pc    : meta[0].pc,
                             rd    : meta[0].rd,
                             epochs : meta[0].epochs[0],
                             upper_instr: meta[0].upper_instr,
                             drop_instr: False,
                             insttype : NONE,
                             instpkt: tagged None
                           `ifdef no_wawstalls 
                             ,id: ?
                           `endif
                           `ifdef spfpu
                             ,rdtype : meta[0].rdtype
                           `endif } ;
    // if no trap offload instruction to cache.
    if (!trap) begin
      wr_memory_request <= req;
      `logLevel( stage3, 0, $format("[%2d]STAGE3: wr_memory_request assignd : ",hartid, fshow(req)), wr_simulate_log_start)
    end
    // lock the destination register in the scoreboard

    // if trap then forward the cause
    if (trap) begin
      //tx_trapout.u.enq(unpack({0, pack(trapout)}));
      common_pkt.instpkt = tagged TRAP trapout;
      common_pkt.insttype = TRAP;
      `logLevel( stage3, 0, $format("[%2d]STAGE3: Base Memory Op created Trap: ", hartid,fshow(trapout)), wr_simulate_log_start)
    end
  `ifdef supervisor
    // convert SFence as a nop hence forth in the pipeline.
    else if (meta[0].memaccess == SFence `ifdef hypervisor || meta[0].memaccess == HFence_GVMA || meta[0].memaccess == HFence_VVMA `endif ) begin
      BaseOut baseoutput = BaseOut { rdvalue   : ?, rd: 0, epochs: curr_epochs[0]
                               `ifdef spfpu ,fflags    : 0 , rdtype: IRF `endif };
    //`ifdef no_wawstalls
    //  baseoutput.id = _id[0];
    //`endif
      //tx_baseout.u.enq(unpack({0, pack(baseoutput)}));
      common_pkt.instpkt = tagged BASE baseoutput;
      common_pkt.insttype = BASE;
      `logLevel( stage3, 0, $format("[%2d]STAGE3: SFence goes as Nop", hartid), wr_simulate_log_start)
    end
  `endif
    // tag the instruction as memory so it waits for cache response in the next stage.
    else begin
      common_pkt.insttype = MEMORY;
      common_pkt.instpkt = tagged MEMORY memoryout;
      //tx_memoryout.u.enq(unpack({0, pack(memoryout)}));
      `logLevel( stage3, 0, $format("[%2d]STAGE3: Base Memory Op initiated: ",hartid, fshow(memoryout)), wr_simulate_log_start)
    end

    //tx_fuid.u.enq(unpack({0, pack(common_pkt)}));
    wr_fuid[0] <= common_pkt;
    //deq_rx;
  `ifdef rtldump
    let clogpkt = rx_commitlog.u.first[0];
    CommitLogMem _pkt = ?;
    if (clogpkt.inst_type matches tagged MEM .p)
      _pkt = p;
    _pkt.address = memory_address;
    _pkt.data = wr_fwd_op2;
    `ifdef atomic
      _pkt.atomic_op = {funct3[0],meta[0].funct[6:3]};
    `endif
    clogpkt.inst_type = tagged MEM _pkt;
    //tx_commitlog.u.enq(clogpkt);
    wr_commitlog[0] <= clogpkt;
  `endif
  endrule:rl_exe_base_memory

  /*doc:rule: This rule executes control instructions when epochs match. Control instructions
   * include: JAL, JALR and Branch. When branch predictor is instantiated, this rule can only fire
   * when the next pc in the previous ISB is available. This pc value is required to check if the
   * prediction from the bpu was indeed correct or not.
   * This rule will also capture the address mis-aligned traps.
   * When a misprediction is detected, a flush from this pipe is initiated to all the previous
   * pipeline stages to change the epochs and start fetching from a corrected PC.
   * When the branch predictor is enabled, this rule will further send training informatino back
   * to the bpu for the control instruction.
  */
  rule rl_exe_base_control(((instr_type[0] == JALR || 
                          instr_type[0] == JAL ||
                          instr_type[0] == BRANCH ) || 
                          (instr_type[1] == JALR ||
                           instr_type[1] == JAL ||
                           instr_type[1] == BRANCH ))
                          && epochs_match && tx_fuid.u.notFull && !wr_waw_stall && wr_ops_avail
               `ifdef bpu && (isValid(wr_next_pc)) `endif );

    Bit#(TLog#(`num_issue)) inst_num;
    Instruction_type inst_type;
    Stage3Meta lv_meta;
    Bit#(TMax#(`vaddr,`flen))  offset;
    if (instr_type[0] == JALR || instr_type[0] == JAL || instr_type[0] == BRANCH) begin
      inst_type = instr_type[0];
      lv_meta = meta[0];
      inst_num = 0;
      offset = wr_op3.data;
    end
    else begin
      inst_type = instr_type[1];
      lv_meta = meta[1];
      inst_num = 1;
      offset = signExtend(wr_op6);
    end

    Bit#(`vaddr)  base;
    if (inst_num == 0)
      base = (inst_type == JALR) ? truncate(wr_fwd_op1) : lv_meta.pc;
    else
      base = (inst_type == JALR) ? truncate(wr_fwd_op4) : lv_meta.pc;
    
    `logLevel( stage3, 0, $format("[%2d]STAGE3: Base Control Op received: ",hartid,fshow(inst_type)), wr_simulate_log_start)
    `logLevel( stage3, 0, $format("[%2d]STAGE3: Base meta : ", hartid, fshow(lv_meta)), wr_simulate_log_start)

    Bit#(`vaddr) jump_address = (base + truncate(offset)) & {'1, ~(pack(inst_type==JALR))};
    Bit#(`xlen) incr = `ifdef compressed (lv_meta.compressed)?2 : `endif 4;
    Bit#(`xlen) nlogical_pc = lv_meta.pc + incr;

    Bit#(1) btaken;
    if (inst_num == 0)
      btaken = fn_bru(wr_fwd_op1, wr_fwd_op2, truncateLSB(lv_meta.funct));
    else
      btaken = fn_bru(wr_fwd_op4, wr_fwd_op5, truncateLSB(lv_meta.funct));


    Bool trap = ( jump_address[1] != 0 && wr_misa_c == 0 &&
                ( inst_type == JALR || inst_type == JAL ||
                ( inst_type == BRANCH && btaken == 1)));

    Bit#(`vaddr) redirect_pc = jump_address;
    Bool redirection = False;
  `ifndef bpu
    if((inst_type == BRANCH && btaken == 1) || inst_type == JALR || inst_type == JAL )
	  	redirection = !trap;
  `else
    Bit#(`vaddr) nextpc;
    //if (inst_num == 1 && meta.instr_reversed)
    //  nextpc = meta[1].pc;
    //else if (inst_num == 1)
    //  nextpc =  fromMaybe(?,wr_next_pc);
    //else if (instr_type[1] == NONE)
    //  nextpc =  fromMaybe(?,wr_next_pc);
    //else
    //  nextpc = meta[1].pc;

    if ((inst_num == 1 && !lv_meta.instr_reversed) || (inst_num == 0 && instr_type[1] == NONE) || (inst_num == 0 && lv_meta.instr_reversed))
      nextpc =  fromMaybe(?,wr_next_pc);
    else if(inst_num == 0)
      nextpc = meta[1].pc;
    else
      nextpc = meta[0].pc;

    `logLevel( stage3, 1, $format("[%2d]STAGE3: Base Control, nextpc: %h, jump_addr: %h", hartid, nextpc, jump_address), wr_simulate_log_start)
    //if (instr_type[1] == NONE || meta.instr_reversed)
    //  nextpc = fromMaybe(?,wr_next_pc);
    //else
    //  nextpc = meta[1].pc;
    let prediction = btbresponse.prediction;
    if(inst_type == BRANCH && btaken == 0)begin
      redirect_pc = nlogical_pc;
    end
    //if( (inst_type == BRANCH  && btaken != prediction[`statesize-1] && (btbresponse.ci_offset == meta[0].pc[2:1])) ||
    //    ( (inst_type == JALR || inst_type == JAL ) && nextpc != jump_address) )begin
    if (nextpc != redirect_pc)
	    redirection = !trap;
    //end
    let td = Training_data{pc : lv_meta.pc,
                           target : jump_address,
                           state  : ?
                        `ifdef gshare
                           ,history   : btbresponse.history
                        `endif
                        `ifdef compressed
                           ,compressed : lv_meta.compressed
                        `endif
                           ,ci         : ?
                           ,btbhit     : btbresponse.btbhit
                        };
    if((inst_type == JAL || inst_type == JALR) && lv_meta.rd ==1)
      td.ci = Call;
    else if(inst_type == JALR &&& opmeta.rs1addr matches 'b00?01)
      td.ci = Ret;
    else if(inst_type == JAL || inst_type == JALR)
      td.ci = JAL;
    else
      td.ci = Branch;

    if(inst_type == BRANCH && !trap)begin
      if(btaken==1)begin
        case(prediction)
          'b00: prediction= 'b01;
          'b01: prediction= 'b11;
          'b10: prediction= 'b11;
        endcase
      end
      else begin
        case(prediction)
          'b01:prediction= 'b00;
          'b10:prediction= 'b00;
          'b11:prediction= 'b10;
        endcase
      end
      td.state = prediction;
    end
    else begin
      td.state = 3;
    end
    if(redirection)
      `logLevel( stage3, 0, $format("[%2d]STAGE3: Misprediction. NextPC in Pipe:%h ExpectedPC:%h",hartid,nextpc,redirect_pc), wr_simulate_log_start)
  `endif
    TrapOut trapout = TrapOut {cause   : `Inst_addr_misaligned, is_microtrap: False, mtval : lv_meta.pc
                                                                                            `ifdef hypervisor ,mtval2: ?
                                                                                            `endif
                                                                                            };
    BaseOut baseoutput = BaseOut { rdvalue   : nlogical_pc, rd: lv_meta.rd, epochs: curr_epochs[0]
                                 `ifdef spfpu ,fflags    : 0 , rdtype: lv_meta.rdtype `endif };
    //let _id <- sboard.ma_lock_rd(unpack({0, pack(SBDUpd{rd: meta[0].rd `ifdef spfpu ,rdtype: meta[0].rdtype `endif })}));
    if (inst_num == 0)
      wr_lock[0] <= SBDUpd{rd: lv_meta.rd `ifdef spfpu ,rdtype: lv_meta.rdtype `endif };
    else
      wr_lock[1] <= SBDUpd{rd: lv_meta.rd `ifdef spfpu ,rdtype: lv_meta.rdtype `endif };
    let common_pkt = FUid{pc    : lv_meta.pc,
                             rd    : lv_meta.rd,
                             epochs : lv_meta.epochs[0],
                             upper_instr: lv_meta.upper_instr,
                             drop_instr: False,
                             insttype : NONE,
                             instpkt : tagged None
                            `ifdef no_wawstalls 
                              ,id: ?
                            `endif
                            `ifdef spfpu
                              ,rdtype : lv_meta.rdtype
                            `endif } ;
  //`ifdef no_wawstalls
  //  baseoutput.id = _id[0];
  //`endif
    if (!trap) begin
      //tx_baseout.u.enq(unpack({0, pack(baseoutput)}));
      common_pkt.instpkt = tagged BASE baseoutput;
      common_pkt.insttype = BASE;
      `logLevel( stage3, 0, $format("[%2d]STAGE3: Base Control Op completed: ",hartid,fshow(baseoutput)), wr_simulate_log_start)
    end
    else begin
      //tx_trapout.u.enq(unpack({0, pack(trapout)}));
      common_pkt.instpkt = tagged TRAP trapout;
      common_pkt.insttype = TRAP;
      `logLevel( stage3, 0, $format("[%2d]STAGE3: Base Control Op created trap: ",hartid,fshow(trapout)), wr_simulate_log_start)
    end
    //tx_fuid.u.enq(unpack({0, pack(common_pkt)}));
    if (inst_num == 0)
      wr_fuid[0] <= common_pkt;
    else
      wr_fuid[1] <= common_pkt;
    //deq_rx;
    rg_eEpoch         <= pack(redirection)^rg_eEpoch;
    wr_redirect_pc    <= redirect_pc;
    wr_flush_from_exe <= redirection;
  `ifdef rtldump
    if (inst_num == 0) begin
      let clogpkt = rx_commitlog.u.first[0];
      wr_commitlog[0] <= clogpkt;
    end
    else begin
      let clogpkt = rx_commitlog.u.first[1];
      wr_commitlog[1] <= clogpkt;
    end
  `endif
  `ifdef bpu 
    if (!trap && redirection)
      wr_mispredict_ghr <= tagged Valid tuple2(btbresponse.btbhit, btbresponse.history);
    wr_training_data <= tagged Valid td;
  `endif
  `ifdef perfmonitors
    if (inst_type == BRANCH)
      wr_count_branches <= 1;
    else 
      wr_count_jumps <= 1;
  `endif
  endrule:rl_exe_base_control

  //rule rl_exe_base_control_1((instr_type[1] == JALR || 
  //                        instr_type[1] == JAL ||
  //                        instr_type[1] == BRANCH )
  //                        && epochs_match && tx_fuid.u.notFull && !wr_waw_stall && wr_ops_avail
  //             `ifdef bpu && (isValid(wr_next_pc)) `endif );

  //  let inst_type = instr_type[1];
  //  Bit#(`vaddr)  base = (inst_type == JALR) ? truncate(wr_fwd_op1) : meta[1].pc;
  //  Bit#(TMax#(`vaddr,`flen))  offset = wr_op3.data;
  //  
  //  `logLevel( stage3, 0, $format("[%2d]STAGE3: Base Control Op received: ",hartid,fshow(inst_type)), wr_simulate_log_start)
  //  `logLevel( stage3, 0, $format("[%2d]STAGE3: Base meta : ", hartid, fshow(meta[1])), wr_simulate_log_start)

  //  Bit#(`vaddr) jump_address = (base + truncate(offset)) & {'1, ~(pack(inst_type==JALR))};
  //  Bit#(`xlen) incr = `ifdef compressed (meta[0].compressed)?2 : `endif 4;
  //  Bit#(`xlen) nlogical_pc = meta[1].pc + incr;

  //  let btaken = fn_bru(wr_fwd_op1, wr_fwd_op2, truncateLSB(meta[1].funct));

  //  Bool trap = ( jump_address[1] != 0 && wr_misa_c == 0 &&
  //              ( inst_type == JALR || inst_type == JAL ||
  //              ( inst_type == BRANCH && btaken == 1)));

  //  Bit#(`vaddr) redirect_pc = jump_address;
  //  Bool redirection = False;
  //`ifndef bpu
  //  if((inst_type == BRANCH && btaken == 1) || inst_type == JALR || inst_type == JAL )
	//  	redirection = !trap;
  //`else
  //  Bit#(`vaddr) nextpc;
  //  if (instr_type[1] == NONE || meta[0].instr_reversed)
  //    nextpc = fromMaybe(?,wr_next_pc);
  //  else
  //    nextpc = meta[1].pc;
  //  let prediction = btbresponse.prediction;
  //  if(inst_type == BRANCH && btaken == 0)begin
  //    redirect_pc = nlogical_pc;
  //  end
  //  //if( (inst_type == BRANCH  && btaken != prediction[`statesize-1] && (btbresponse.ci_offset == meta[0].pc[2:1])) ||
  //  //    ( (inst_type == JALR || inst_type == JAL ) && nextpc != jump_address) )begin
  //  if (nextpc != redirect_pc)
	//    redirection = !trap;
  //  //end
  //  let td = Training_data{pc : meta[1].pc,
  //                         target : jump_address,
  //                         state  : ?
  //                      `ifdef gshare
  //                         ,history   : btbresponse.history
  //                      `endif
  //                      `ifdef compressed
  //                         ,compressed : meta[1].compressed
  //                      `endif
  //                         ,ci         : ?
  //                         ,btbhit     : btbresponse.btbhit
  //                      };
  //  if((inst_type == JAL || inst_type == JALR) && meta[1].rd ==1)
  //    td.ci = Call;
  //  else if(inst_type == JALR &&& opmeta.rs1addr matches 'b00?01)
  //    td.ci = Ret;
  //  else if(inst_type == JAL || inst_type == JALR)
  //    td.ci = JAL;
  //  else
  //    td.ci = Branch;

  //  if(inst_type == BRANCH && !trap)begin
  //    if(btaken==1)begin
  //      case(prediction)
  //        'b00: prediction= 'b01;
  //        'b01: prediction= 'b11;
  //        'b10: prediction= 'b11;
  //      endcase
  //    end
  //    else begin
  //      case(prediction)
  //        'b01:prediction= 'b00;
  //        'b10:prediction= 'b00;
  //        'b11:prediction= 'b10;
  //      endcase
  //    end
  //    td.state = prediction;
  //  end
  //  else begin
  //    td.state = 3;
  //  end
  //  if(redirection)
  //    `logLevel( stage3, 0, $format("[%2d]STAGE3: Misprediction. NextPC in Pipe:%h ExpectedPC:%h",hartid,nextpc,redirect_pc), wr_simulate_log_start)
  //`endif
  //  TrapOut trapout = TrapOut {cause   : `Inst_addr_misaligned, is_microtrap: False, mtval : meta[1].pc
  //                                                                                          `ifdef hypervisor ,mtval2: ?
  //                                                                                          `endif
  //                                                                                          };
  //  BaseOut baseoutput = BaseOut { rdvalue   : nlogical_pc, rd: meta[1].rd, epochs: curr_epochs[1]
  //                               `ifdef spfpu ,fflags    : 0 , rdtype: meta[1].rdtype `endif };
  //  //let _id <- sboard.ma_lock_rd(unpack({0, pack(SBDUpd{rd: meta[0].rd `ifdef spfpu ,rdtype: meta[0].rdtype `endif })}));
  //  wr_lock[0] <= SBDUpd{rd: meta[1].rd `ifdef spfpu ,rdtype: meta[1].rdtype `endif };
  //  let common_pkt = FUid{pc    : meta[1].pc,
  //                           rd    : meta[1].rd,
  //                           epochs : meta[1].epochs[1],
  //                           upper_instr: meta[1].upper_instr,
  //                           insttype : NONE,
  //                           instpkt : tagged None
  //                          `ifdef no_wawstalls 
  //                            ,id: ?
  //                          `endif
  //                          `ifdef spfpu
  //                            ,rdtype : meta[1].rdtype
  //                          `endif } ;
  ////`ifdef no_wawstalls
  ////  baseoutput.id = _id[0];
  ////`endif
  //  if (!trap) begin
  //    //tx_baseout.u.enq(unpack({0, pack(baseoutput)}));
  //    common_pkt.instpkt = tagged BASE baseoutput;
  //    common_pkt.insttype = BASE;
  //    `logLevel( stage3, 0, $format("[%2d]STAGE3: Base Control Op completed: ",hartid,fshow(baseoutput)), wr_simulate_log_start)
  //  end
  //  else begin
  //    //tx_trapout.u.enq(unpack({0, pack(trapout)}));
  //    common_pkt.instpkt = tagged TRAP trapout;
  //    common_pkt.insttype = TRAP;
  //    `logLevel( stage3, 0, $format("[%2d]STAGE3: Base Control Op created trap: ",hartid,fshow(trapout)), wr_simulate_log_start)
  //  end
  //  //tx_fuid.u.enq(unpack({0, pack(common_pkt)}));
  //  wr_fuid[0] <= common_pkt;
  //  //deq_rx;
  //  rg_eEpoch         <= pack(redirection)^rg_eEpoch;
  //  wr_redirect_pc    <= redirect_pc;
  //  wr_flush_from_exe <= redirection;
  //`ifdef rtldump
  //  let clogpkt = rx_commitlog.u.first[0];
  //  //tx_commitlog.u.enq(clogpkt);
  //  wr_commitlog[0] <= clogpkt;
  //`endif
  //`ifdef bpu 
  //  if (!trap && redirection)
  //    wr_mispredict_ghr <= tagged Valid tuple2(btbresponse.btbhit, btbresponse.history);
  //  wr_training_data <= tagged Valid td;
  //`endif
  //`ifdef perfmonitors
  //  if (inst_type == BRANCH)
  //    wr_count_branches <= 1;
  //  else 
  //    wr_count_jumps <= 1;
  //`endif
  //endrule:rl_exe_base_control
`ifdef muldiv

  /*doc:rule: dummy rule to simply display the ready signals of the multiplication and division
    * submodules*/
  rule rl_show_mbox_rdy;
    `logLevel( mbox, 0, $format("[%2d]MBOX: MulRdy:%b DivRdy:%b",hartid, wr_mul_ready, wr_div_ready), wr_simulate_log_start)
  endrule:rl_show_mbox_rdy

  /*doc:rule: This rule will fire when the epochs match and when the multiplier/divider are
  * avaialble based on the current instruction. Both the operands are required for execution to be
  * offloaded the mbox.*/
  rule rl_mbox(instr_type[0] == MULDIV && epochs_match && tx_fuid.u.notFull && !wr_waw_stall && wr_ops_avail &&
              ( (meta[0].funct[2]==0 && wr_mul_ready) || 
                (meta[0].funct[2]==1 && wr_div_ready) ) &&
    ((instr_type[1] != BRANCH && instr_type[1] != JAL && instr_type[1] != JALR) || isValid(wr_next_pc)));
    `logLevel( stage3, 0, $format("[%2d]STAGE3: MULDIV Op received",hartid), wr_simulate_log_start)
    wr_muldiv_inputs <= MBoxIn{in1: wr_fwd_op1, in2: wr_fwd_op2, funct3: truncate(meta[0].funct)
                              `ifdef RV64 , wordop: meta[0].word32 `endif };
    `logLevel( stage3, 0, $format("[%2d]STAGE3: MULDIV op offloaded",hartid), wr_simulate_log_start)
    //deq_rx;
    //let _id <- sboard.ma_lock_rd(unpack({0, pack(SBDUpd{rd: meta[0].rd `ifdef spfpu ,rdtype: meta[0].rdtype `endif })}));
    wr_lock[0] <= SBDUpd{rd: meta[0].rd `ifdef spfpu ,rdtype: meta[0].rdtype `endif };
                                  
    let common_pkt = FUid{pc    : meta[0].pc,
                      rd    : meta[0].rd,
                      epochs : meta[0].epochs[0],
                      upper_instr: meta[0].upper_instr,
                      drop_instr: False,
                      insttype : MULDIV,
                      instpkt: tagged None
                    `ifdef no_wawstalls 
                      ,id: ?
                    `endif
                    `ifdef spfpu
                      ,rdtype : meta[0].rdtype
                    `endif } ;
    //tx_fuid.u.enq(unpack({0, pack(common_pkt)}));
    wr_fuid[0] <= common_pkt;
  `ifdef perfmonitors
    wr_count_muldiv <= 1;
  `endif
  `ifdef rtldump
    let clogpkt = rx_commitlog.u.first[0];
    //tx_commitlog.u.enq(clogpkt);
    wr_commitlog[0] <= clogpkt;
  `endif
  endrule:rl_mbox
`endif

`ifdef spfpu
  let f7 = wr_op3.data[11:5];
  let opcode = meta[0].funct[6:3];
  let f3 = truncate(meta[0].funct);
  `ifdef dpfpu
    let issp = meta[0].word32;
  `endif

  // Bool spfma_rdy = (`ifdef dpfpu issp && `endif 
  //                  (opcode[2] == 0 || (opcode[2] == 1 && (f7[6:1] == `FADDS_f7 || f7[6:1] == `FSUBS_f7 || f7[6:1] == `FMULS_f7))));
  // Bool dpfma_rdy = (wr_fbox_ready.dfma && `ifdef dpfpu !issp && `endif 
  //                  (opcode[2] == 0 || (opcode[2] == 1 && (f7[6:1] == `FADDS_f7 || f7[6:1] == `FSUBS_f7 || f7[6:1] == `FMULS_f7))));
  // Bool spdiv_rdy = (opcode[2] == 1 && `ifdef dpfpu issp && `endif (f7[6:1] == `FDIV_f7 || f7[6:1] == `FSQRT_f7) && wr_fbox_ready.sdivsqrt);
  // Bool dpdiv_rdy = (opcode[2] == 1 && `ifdef dpfpu !issp && `endif (f7[6:1] == `FDIV_f7 || f7[6:1] == `FSQRT_f7) && wr_fbox_ready.ddivsqrt);
  // Bool spcvt_rdy = (opcode[2] == 1 && `ifdef dpfpu issp && `endif (f7[6:1] == `FCVT_F2I_f7 || f7[6:1] == `FCVT_I2F_f7) && wr_fbox_ready.scvt);
  // Bool dpcvt_rdy = (opcode[2] == 1 && `ifdef dpfpu !issp && `endif (f7[6:1] == `FCVT_F2I_f7 || f7[6:1] == `FCVT_I2F_f7) && wr_fbox_ready.dcvt);
  // Bool misc_rdy =  (opcode[2] == 1 &&  (f7[6:1] == `FMV_S_X_f7 || f7[6:1] == `FMV_X_S_f7 || 
  //                                     f7[6:1] == `FSGNJN_f7 || f7[6:1] == `FCLASS_f7 || 
  //                                     f7[6:1] == `FCMP_f7 || f7[6:1] == `FMAX_f7
  //                                 `ifdef dpfpu || f7[6:1] == `FCVT_S_D_f7 `endif ) && wr_fbox_ready.singlecycle);


  /*doc:rule: This rule will fire when the epochs match and when the multiplier/divider are
  * avaialble based on the current instruction. Both the operands are required for execution to be
  * offloaded the mbox.*/
  rule rl_fbox(instr_type[0] == FLOAT && epochs_match && tx_fuid.u.notFull && !wr_waw_stall && wr_fbox_ready && wr_ops_avail &&
    ((instr_type[1] != BRANCH && instr_type[1] != JAL && instr_type[1] != JALR) || isValid(wr_next_pc)));
    `logLevel( stage3, 0, $format("[%2d]STAGE3: FLOAT Op received",hartid), wr_simulate_log_start)
    wr_float_inputs <= Input_Packet{operand1: truncate(wr_fwd_op1), operand2: truncate(wr_fwd_op2), operand3:truncate(wr_fwd_op3),
                             opcode: (meta[0].funct[6:3]), funct3: truncate(meta[0].funct), 
                             funct7: wr_op3.data[11:5], imm: wr_op3.data[1:0],issp: issp 
                            };

    // multicycle_alu.ma_inputs(fn, funct3, arg1, arg2, arg4
    //                             `ifdef RV64 ,meta.word32 `endif );
    `logLevel( stage3, 0, $format("FPU: op1:%h op2:%h op3:%h",wr_fwd_op1,wr_fwd_op2,wr_fwd_op3), wr_simulate_log_start)
    `logLevel( stage3, 0, $format("[%2d]STAGE3: FLOAT op offloaded",hartid), wr_simulate_log_start)
    //deq_rx;
    //let _id <- sboard.ma_lock_rd(unpack({0, pack(SBDUpd{rd: meta[0].rd `ifdef spfpu ,rdtype: meta[0].rdtype `endif })}));
    wr_lock[0] <= SBDUpd{rd: meta[0].rd `ifdef spfpu ,rdtype: meta[0].rdtype `endif };
    let common_pkt = FUid{pc    : meta[0].pc,
                      rd    : meta[0].rd,
                      epochs : meta[0].epochs[0],
                      upper_instr: meta[0].upper_instr,
                      drop_instr: False,
                      insttype : FLOAT,
                      instpkt : tagged None
                    `ifdef no_wawstalls 
                      ,id: ?
                    `endif
                    `ifdef spfpu
                      ,rdtype : meta[0].rdtype
                    `endif } ;
    //tx_fuid.u.enq(unpack({0, pack(common_pkt)}));
    wr_fuid[0] <= common_pkt;
    `ifdef perfmonitors
      wr_count_floats<= 1;
    `endif
    `ifdef rtldump
      let clogpkt = rx_commitlog.u.first[0];
      //tx_commitlog.u.enq(clogpkt);
      wr_commitlog[0] <= clogpkt;
    `endif
  endrule:rl_fbox
`endif

  /*doc:rule: Updating default values to 2nd instruction flowing in the pipeline if not driven*/
  rule rl_2nd_instruction_invalid(instr_type[1] == NONE);
    let default_common = FUid { pc : ?,
                                rd : ?,
                                epochs: ?,
                                upper_instr: meta[1].upper_instr,
                                drop_instr: True,
                                insttype: NONE,
                                instpkt : tagged None
                              `ifdef no_wawstalls 
                                ,id: ? 
                              `endif
                              `ifdef spfpu
                                ,rdtype : ?
                              `endif } ;
    wr_fuid[1] <= default_common;
    wr_commitlog[1] <= default_commitlog;
    wr_lock[1] <= SBDUpd {rd: 0 `ifdef spfpu ,rdtype: IRF `endif };
  endrule

  /*doc:rule: To lock the required register and get it's new rename id.*/
  rule rl_lock_sb;
    Vector#(`num_issue, SBDUpd) lock;
    for (Integer i=0; i<`num_issue; i=i+1)
      lock[i] = wr_lock[i];
    //if (!meta[0].upper_instr && (wr_fuid[0].insttype == TRAP || wr_flush_from_exe)) begin
    //  `logLevel( stage3, 0, $format("[%2d]STAGE3: Removing SBoard Lock lower instruction generated a TRAP/misprediction", hartid), wr_simulate_log_start)
    //  lock[1] = SBDUpd {rd: 0 `ifdef spfpu ,rdtype: IRF `endif };
    //end

    if (meta[0].upper_instr)
      lock = reverse(lock);

    let _id <- sboard.ma_lock_rd(lock);
    if (meta[0].upper_instr)
      wr_id <= reverse(_id);
    else 
      wr_id <= _id;

    `ifdef no_wawstalls
      `logLevel( stage3, 0, $format("[%2d]STAGE3: issuing ID:%2d",hartid,_id), wr_simulate_log_start)
    `endif
  endrule


  /*doc:rule: Updating the ISBs with the required values*/
  rule rl_update_pipeline;
    deq_rx;
    Vector#(`num_issue, FUid) fuid;
  `ifdef rtldump
    Vector#(`num_issue, CommitLogPacket) commitlog;
  `endif
    for (Integer i=0; i<`num_issue; i=i+1) begin
      fuid[i] = wr_fuid[i];
    `ifdef no_wawstalls
      fuid[i].id = wr_id[i];
    `endif
    `ifdef rtldump
      commitlog[i] = wr_commitlog[i];
    `endif
    end

    // Drop upper instruction if lower instruction generates a trap.
    if (!meta[0].upper_instr && (instr_type[0] == BRANCH || instr_type[0] == JAL || instr_type[0] == JALR)
      && (fuid[0].insttype == TRAP || wr_flush_from_exe)) begin
      `logLevel( stage3, 0, $format("[%2d]STAGE3: Dropping upper instruction as lower instruction generated a TRAP/misprediction", hartid), wr_simulate_log_start)
      fuid[1].drop_instr = True;
      //fuid[1].insttype = NONE;
      //fuid[1].instpkt = tagged None;
      //`ifdef rtldump
      //  commitlog[1] = default_commitlog;
      //`endif
    end
    else if (!meta[1].upper_instr && (instr_type[1] == BRANCH || instr_type[1] == JAL || instr_type[1] == JALR) 
      && wr_flush_from_exe) begin
      `logLevel( stage3, 0, $format("[%2d]STAGE3: Dropping upper instruction as lower instruction generated a TRAP/misprediction", hartid), wr_simulate_log_start)
      fuid[0].drop_instr = True;
      //fuid[0].insttype = NONE;
      //fuid[0].instpkt = tagged None;
      //`ifdef rtldump
      //  commitlog[0] = default_commitlog;
      //`endif
    end

    tx_fuid.u.enq(fuid);
  `ifdef rtldump
    tx_commitlog.u.enq(commitlog);
  `endif
  endrule
  //--------------- interfaces to receive the decoded info from the previous stage. ------------//
  interface rx = interface Ifc_s3_rx
    interface rx_meta_from_stage2   = rx_meta.e;
    interface rx_mtval_from_stage2  = rx_mtval.e;
    interface rx_instrtype_from_stage2 = rx_instrtype.e;
    interface rx_opmeta_from_stage2= rx_opmeta.e;
  `ifdef rtldump
    interface rx_commitlog = rx_commitlog.e;
  `endif
  endinterface;
  // -------------------------------------------------------------------------------------------//

  // ------------------ interfaces to send the executed result to the next stage --------------//
  interface tx = interface Ifc_s3_tx
  	interface tx_fuid_to_stage4 = tx_fuid.e;
  	//interface tx_baseout_to_stage4 = tx_baseout.e;
  	//interface tx_trapout_to_stage4= tx_trapout.e;
  	//interface tx_systemout_to_stage4 = tx_systemout.e;
  	//interface tx_memoryout_to_stage4 = tx_memoryout.e;
  `ifdef rtldump
    interface tx_commitlog = tx_commitlog.e;
  `endif
  endinterface;
  // -------------------------------------------------------------------------------------------//
  // ------------------ interfaces to receive RF operands from previous stage    --------------//
  interface rf = interface Ifc_s3_rf
    method Action ma_op1 (FwdType i);
      wr_rf_op1 <= i;
    endmethod
    method Action ma_op2 (FwdType i);
      wr_rf_op2 <= i;
    endmethod
    method Action ma_op3 (FwdType i);
      wr_op3 <= i;
    endmethod
    method Action ma_op4 (FwdType i);
      wr_rf_op4 <= i;
    endmethod
    method Action ma_op5 (FwdType i);
      wr_rf_op5 <= i;
    endmethod
    method Action ma_op6 (Bit#(32) i);
      wr_op6 <= i;
    endmethod
  endinterface;
  // -------------------------------------------------------------------------------------------//

  interface common = interface Ifc_s3_common
    // Description : This method fires when there is a flush from the write - back stage.
    method Action ma_update_wEpoch;
      rg_wEpoch<= ~rg_wEpoch;
    endmethod
    // Thhis is method is fired when a branch / jump redicrection is detected from this stage.
    method mv_flush = tuple2(wr_flush_from_exe, wr_redirect_pc);
    // Description : captures the current setting of the "C" bit in the misa csr.
    method Action ma_csr_misa_c (Bit#(1) m);
      wr_misa_c <= m;
    endmethod

    method Action ma_sb_release(Vector#(`num_issue, CommitData) commit);
      Vector#(`num_issue, SBDUpd) release_lock;
      for (Integer i=0; i<`num_issue; i=i+1) begin
        release_lock[i] = SBDUpd { rd: commit[i].addr
                              `ifdef no_wawstalls ,id: commit[i].id `endif 
                              `ifdef spfpu ,rdtype: commit[i].rdtype `endif };
      end
      sboard.ma_release_rd(release_lock);
    endmethod

    method Action ma_priv (Bit#(2) priv);
      wr_priv <= priv;
    endmethod

    method Action ma_mstatus (Bit#(`xlen) mstatus);
      wr_mstatus <= mstatus;
    endmethod

  `ifdef simulate
    method Action ma_simulate_log_start(Bit#(1) start);
      wr_simulate_log_start <= start;
    endmethod
  `endif

  `ifdef hypervisor
    method Action ma_vs_mode (Bit#(1) vs);
      wr_vs_mode <= vs;
    endmethod

    method Action ma_hstatus (Bit#(`xlen) hstatus);
      wr_hstatus <= hstatus;
    endmethod
 `endif
  endinterface;

  interface cache = interface Ifc_s3_cache
    // Description : captures if the dmem subsystem is available for sending requests
    method Action ma_cache_is_available(Bool avail);
      wr_cache_avail <= avail;
    endmethod

    // Description : interface to send memory requests.
    interface mv_memory_request = interface Get
      method ActionValue#(DMem_request#(`vaddr, `elen, 1)) get;
        `logLevel( stage3, 0, $format("[%2d]STAGE3: request sent",hartid), wr_simulate_log_start)
        return wr_memory_request;
      endmethod
    endinterface;
  endinterface;

  interface bypass = interface Ifc_s3_bypass
    // Description : receives the bypass values from the pipe3
    method Action ma_bypass (Vector#(`bypass_sources, Vector#(`num_issue, FwdType)) fwd);
      wr_bypass <= fwd;
    endmethod:ma_bypass
  endinterface;

`ifdef bpu
  interface bpu = interface Ifc_s3_bpu
    // Description : captures the next_pc in the pipe
    method Action ma_next_pc (Bit#(`vaddr) npc);
      wr_next_pc <= tagged Valid npc;
    endmethod
    // Description : method to train the branch predictor BTB
    method Training_data mv_train_bpu if(wr_training_data matches tagged Valid .x);
      return x;
    endmethod
  `ifdef gshare
    method Tuple2#(Bool, Bit#(`histlen)) mv_mispredict if(wr_mispredict_ghr matches tagged Valid .x);
      return x;
    endmethod
  `endif
  endinterface;
`endif
`ifdef perfmonitors
  interface perfmonitors = interface Ifc_s3_perfmonitors
    /*doc:method: */
    `ifdef spfpu
      method mv_count_floats = wr_count_floats;
    `endif
    `ifdef muldiv
      method mv_count_muldiv = wr_count_muldiv;
    `endif
    method mv_count_jumps = wr_count_jumps;
    method mv_count_branches = wr_count_branches;
    method mv_count_rawstalls = wr_count_rawstalls;
    method mv_count_exestalls = wr_count_exestalls;
    method mv_count_isb3_isb4_full = wr_isb3_isb4_full;
    method mv_count_st3_not_firing = wr_st3_not_firing;
  endinterface;
`endif
`ifdef muldiv
  interface muldiv = interface Ifc_s3_muldiv
    method mv_mbox_inputs = wr_muldiv_inputs;
  method Action ma_mbox_ready(MBoxRdy rdy);
      wr_mul_ready <= rdy.mul;
      wr_div_ready <= rdy.div;
    endmethod: ma_mbox_ready
  endinterface;
`endif
`ifdef spfpu
  interface float = interface Ifc_s3_float
    method mv_fbox_inputs = wr_float_inputs;
    method Action ma_fbox_ready(Bit#(1) rdy);
      wr_fbox_ready <= unpack(rdy);
    endmethod: ma_fbox_ready
  endinterface;
`endif
endmodule
endpackage: stage3

