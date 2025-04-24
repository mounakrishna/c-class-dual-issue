//See LICENSE.iitm for license details
/*

Author : IIT Madras
Details:
This module will interact with the memory subsystem to fetch relevant instructions from memory. The
module will also receive flushes from the write - back stage which could be because of a fence or trap
handling.
--------------------------------------------------------------------------------------------------
*/
package stage1;
  // -- package imports --//
  import FIFOF::*;
  import SpecialFIFOs::*;
  import FIFO::*;
  import GetPut::*;
  import Assert::*;
  import Vector :: *;
  import MIMO   :: *;

  // -- project imports --//
	import TxRx	          :: * ;            // for interstage buffer connection
  import TxRx_MIMO      :: * ;
  import ccore_types    :: * ;     // for pipe - line types
  import icache_types   :: * ;          // for global interface definitions
  import pipe_ifcs      :: * ;
`ifdef compressed
  import decompress     :: * ;
`endif
  `include "ccore_params.defines"// for core parameters
  `include "Logger.bsv"       // for logging display statements.

  // Enum to define the action to be taken when an instruction arrives.
  typedef enum {CheckPrev, None} ActionType deriving(Bits, Eq, FShow);

  typedef struct{
    Bit#(`vaddr) pc;
    Bit#(48) instruction;
    Bit#(2) mask;
    Bit#(2) epochs;
  `ifdef bpu
    BTBResponse btbresponse;
  `endif
  } PrevMeta deriving(Eq, Bits, FShow);

	interface Ifc_stage1;
    interface Ifc_s1_rx rx;
    interface Ifc_s1_tx tx;
    interface Ifc_s1_icache icache;
    interface Ifc_s1_common common;
  `ifdef perfmonitors
    interface Ifc_s1_perfmonitors perf;
  `endif
	endinterface:Ifc_stage1

`ifdef stage1_noinline
  (*synthesize*)
`endif
  module mkstage1#(parameter Bit#(`xlen) hartid) (Ifc_stage1);

    String stage1=""; // defined for logger

    // --------------------- Start instantiations ------------------------//

    // this wire carries the current values of certain csrs.
    Wire#(Bit#(1)) wr_csr_misa_c <- mkWire();

    // The following registers are use to the maintain epochs from various pipeline stages:
    // writeback and execute stage.
    Reg#(Bit#(1)) rg_wEpoch <- mkReg(0);
    Reg#(Bit#(1)) rg_eEpoch <- mkReg(0);

    // This register implements a simple state - machine which indicates how the instruction should
    // be extracted from the cache response.
    //Reg#(ActionType) rg_action <- mkReg(None);

    // This register indicates that the lower 16 - bits of the response from the cache need to be
    // ignored. This happens because, when there is jump to non - 4-byte aligned address the cache
    // still receives a previous 4 - byte - ailgned address from the fetch stage.
    //Vector#(`num_issue, Reg#(Bool)) vrg_receiving_upper <- replicateM(mkReg(False));

    // This register holds the 16 - bits of instruction from the previous cache response if required.
    Reg#(PrevMeta) rg_prev <- mkReg(?);

    // This FIFO receives the response from the memory subsytem (a.k.a cache)
    FIFOF#(IMem_core_response#(TMul#(`iwords, 8), `iesize)) ff_memory_response <- mkSizedBypassFIFOF(2);

    // This FIFO receives the response from the branch prediction unit (bpu or ras)
    RX#(Stage0PC#(`vaddr)) rx_fromstage0 <- mkRX;

    // FIFO to interface with the next pipeline stage
		TX_MIMO#(2, 2, `instr_queue, PIPE1) tx_tostage2 <- mkTX_MIMO;

  `ifdef rtldump
		TX_MIMO#(2, 2, `instr_queue, CommitLogPacket) tx_commitlog <- mkTX_MIMO;
  `endif

    // This variable holds the current epoch values of the pipe
    let curr_epoch = {rg_eEpoch, rg_wEpoch};
`ifdef simulate
  Wire#(Bit#(1)) wr_simulate_log_start <- mkDWire(0);
`endif

  `ifdef triggers
    Vector#(`trigger_num, Wire#(TriggerData)) v_trigger_data1 <- replicateM(mkWire());
    Vector#(`trigger_num, Wire#(Bit#(`xlen))) v_trigger_data2 <- replicateM(mkWire());
    Vector#(`trigger_num, Wire#(Bool)) v_trigger_enable <- replicateM(mkWire());
  `endif

    // ---------------------- End Instatiations --------------------------//


    // ---------------------- Start local function definitions ----------------//

    // this function will deque the response from i - mem fifo and the branch prediction fifo
    function Action deq_response = action
      ff_memory_response.deq;
      rx_fromstage0.u.deq;
    endaction;

  `ifdef triggers

    function ActionValue#(Tuple2#(Bool, Bit#(`causesize))) check_trigger (Bit#(`vaddr) pc,
                           Bit#(32) instr `ifdef compressed , Bool compressed `endif ) = actionvalue
      Bool trap = False;
      Bit#(`causesize) cause = `Breakpoint;
      Bit#(`xlen) compare_value ;
      Bool chain = False;
      for(Integer i=0; i < `trigger_num; i=i+1)begin
        `logLevel( stage1, 3, $format("[%2d]STAGE1: Trigger[%2d] Data1: ",hartid, i, 
                                                                        fshow(v_trigger_data1[i])), wr_simulate_log_start)
        `logLevel( stage1, 3, $format("[%2d]STAGE1: Trigger[%2d] Data2: ",hartid, i, 
                                                                        fshow(v_trigger_data2[i])), wr_simulate_log_start)
        `logLevel( stage1, 3, $format("[%2d]STAGE1: Trigger[%2d] Enable: ",hartid, i, 
                                                                        fshow(v_trigger_enable[i])), wr_simulate_log_start)
        if(v_trigger_enable[i] &&& v_trigger_data1[i] matches tagged MCONTROL .mc &&&
                              ((!trap && !chain) || (chain && trap)) &&& mc.execute == 1)begin
          Bit#(`xlen) trigger_compare = `ifdef compressed
                     (compressed && mc.size == 2)? zeroExtend(v_trigger_data2[i][15:0]): `endif
                                                   v_trigger_data2[i];
          if(mc.select == 0)
            compare_value = pc;
          else
            compare_value = zeroExtend(instr);

          if(mc.matched == 0)begin
            if(trigger_compare == compare_value)
              trap = True;
            else if(chain)
              trap = False;
          end
          if(mc.matched == 2)begin
            if(compare_value >= trigger_compare)
              trap = True;
            else if(chain)
              trap = False;
          end
          if(mc.matched == 3)begin
            if(compare_value < trigger_compare)
              trap = True;
            else if(chain)
              trap = False;
          end

        `ifdef debug
          if(trap && mc.action_ == 1)begin
            cause = `HaltTrigger;
            cause[`causesize - 1] = 1;
          end
        `endif
          chain = unpack(mc.chain);
        end
      end
      return tuple2(trap, cause);
    endactionvalue;
  `endif
    // ---------------------- End local function definitions ------------------//

    rule rl_instr_queue_full(!tx_tostage2.u.enqReady_1());
      `logLevel( stage1, 0, $format("[%2d]STAGE1 : Instruction queue full to enque one instruction ", hartid), wr_simulate_log_start)
    endrule
    rule rl_stage0_pipe_notEmpty(!rx_fromstage0.u.notEmpty);
      `logLevel( stage1, 0, $format("[%2d]STAGE1 : Stage0 pipe is not yet full", hartid), wr_simulate_log_start)
    endrule
    rule rl_mem_response_wait(!ff_memory_response.notEmpty);
      `logLevel( stage1, 0, $format("[%2d]STAGE1 : Waiting for Icache response", hartid), wr_simulate_log_start)
    endrule


    // RuleName : process_instruction
    // Explicit Conditions : None
    // Implicit Conditions:
    //    1. ff_memory_response.notEmpty
    //    2. wr_csr is written in the same cycle
    //    3. tostage FIFO notFull
    // Schedule Conflicts : This rule will not fire if there is flush from the write - back stage. A
    // flush from the write - back stage will cause a change in the rg_pc and rg_discard,
    // both of which are being updated in this method as well. This schedule is acceptable since
    // anyways the response from the memory currently to be handled in this rule will match epochs
    // and will be dropped.
    //
    // Details : This rule will receive the instruction from the memory subsystem and decide if the
    // instruction is compressed or not. The final instruction is then sent to the next stage.
    // To extract the instruction from the memory response a state machine is implemented.
    //
    // 1. First the epochs are compared and if a mis - match is observed then the response is dropped
    // without any other changes to the state of the module.
    // 2. if rg_discard is set and compressed is enabled then the lower 16 - bits of the
    // resposne are discarded and the upper 16 - bits are probed to check if it is a compressed
    // instruction. If so, then the instruction is sent to the next stage. However is it is not a
    // compressed instruction it means the upper 16 - bits of the response refer to the lower 16 - bits
    // of a 32 - bit instruction and thus we will have to wait for the next response from the cache to
    // form the instruction is send to the next stage. To ensure the concatenation happens in the
    // next response we set rg_action to ChecPrev and set enque_instruction to False.
    // 3. if rg_action is set to None, then we simply probe the lower 2 - bits to the response to
    // check if it is compressed. If so then the lower 16 bits form an instruction which is sent to
    // the next stage, the upper 16 - bits are stored to rg_instruction and rg_action is set to
    // CheckPrev to ensure that in the next resposne we first probe rg_instruction.
    // 4. if rg_Action is set to CheckPrev then we first probe the lower 2 - bits of the
    // rg_instruction which leads to two possibilities. Either rg_instruction could hold a
    // compressed instruction from the previous response, in which case the current memory response
    // is not dequed and rg_instruction is sent to the next stage. This can happen due to state - 3
    // mentioned above. The other possibility is that rg_instruction holds the lower 16 - bits of a
    // 32 - bit isntruction, in which case we have to concatenate the lower 16 - bits of the response with
    // rg_instruction and send to the next, and also store the upper 16 - bits of the response into
    // rg_instruction. rg_Action in this case will remain CheckPrev so that the upper bits of this
    // repsonse are probed in the next cycle.
    rule process_instruction(tx_tostage2.u.enqReady_1() && rx_fromstage0.u.notEmpty);// && tx_tostage2.u.notFull);
      let stage0pc = rx_fromstage0.u.first;
      `logLevel( stage1, 1, $format("[%2d]STAGE1 : Prediction: ",hartid, fshow(stage0pc)), wr_simulate_log_start)
    `ifdef bpu
      Vector#(`num_issue, BTBResponse) btbresponse = replicate(stage0pc.btbresponse);
      //if (btbresponse[0].hi) begin
      //  btbresponse[0].btbhit = False;
      //  btbresponse[0].prediction = 1;
      //end
    `endif

      // capture the response from the cache
      let imem_resp = ff_memory_response.first;
      Vector#(`num_issue, Bool) trap = replicate(False);

      // local variable to hold the instruction to be enqueued
      Vector#(`num_issue, Bit#(32)) final_instruction = replicate(0);
      Vector#(`num_issue, Bit#(`vaddr)) instr_pc = replicate(0);
      LUInt#(`num_issue) valid_instructions = 0;

      // local variable to indicate if the instruction being analysed is compressed or not
      Vector#(`num_issue, Bool) compressed_instr = replicate(False);

      // local variable to indicate that the received instruction is part of two memory responses.
      Vector#(`num_issue, Bool) receiving_upper = replicate(False);

      // local variable to indicate if two compressed instructions are paired and issued to decode stage.
      Bool issue = False;
      //ActionType lv_action = None;

      PrevMeta lv_prev = rg_prev;
      Bool lv_deq_response = False;
      // if epochs do not match then drop the instruction
      if(curr_epoch != imem_resp.epochs)begin
        valid_instructions = 0;
        //lv_action = None;
        lv_prev.mask = 2'b00;
        tx_tostage2.u.flush();
        `ifdef rtldump
          tx_commitlog.u.flush();
        `endif
        `logLevel( stage1, 1,$format("[%2d]STAGE1 : Dropping Instruction. ExpEpoch:%b CurrEpoch:%b",
            hartid, imem_resp.epochs, curr_epoch), wr_simulate_log_start)
        receiving_upper[0] = False;
        receiving_upper[1] = False;
        lv_deq_response = True;
        //deq_response();
      end
    `ifdef compressed
      else if(rg_prev.mask == 2'b11 && rg_prev.epochs == curr_epoch)begin
        valid_instructions = 2;
      `ifdef bpu
        btbresponse[0] = rg_prev.btbresponse;
        if(!btbresponse[1].hi) begin
          btbresponse[1].btbhit = False;
          btbresponse[1].prediction = 1;
        end
      `endif
        if (rg_prev.instruction[1:0] == 2'b11 && rg_prev.instruction[33:32] == 2'b11) begin // SS
          `logLevel( stage1, 1, $format("Case I - SS"), wr_simulate_log_start)
          final_instruction[0] = rg_prev.instruction[31:0];
          final_instruction[1] = {imem_resp.word[15:0], rg_prev.instruction[47:32]};
          instr_pc[0] = rg_prev.pc;
          instr_pc[1] = rg_prev.pc + 4;
          compressed_instr = replicate(False);
          lv_prev.instruction = imem_resp.word[63:16];
          lv_prev.pc = stage0pc.address | zeroExtend(2'b10);
        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbresponse;
        `endif
          lv_prev.mask = 2'b11;
          trap[0] = False;
          trap[1] = imem_resp.trap;
          lv_deq_response = True;
          //deq_response();
        end
        else if (rg_prev.instruction[1:0] == 2'b11 && rg_prev.instruction[33:32] != 2'b11) begin // CS
          `logLevel( stage1, 1, $format("Case I - CS"), wr_simulate_log_start)
          final_instruction[0] = rg_prev.instruction[31:0];
          final_instruction[1] = zeroExtend(rg_prev.instruction[47:32]);
          instr_pc[0] = rg_prev.pc;
          instr_pc[1] = rg_prev.pc + 4;
          compressed_instr[0] = False;
          compressed_instr[1] = True;
          lv_prev.mask = 2'b00;
          trap = replicate(False);
          lv_deq_response = False;
        end
        else if (rg_prev.instruction[1:0] != 2'b11 && rg_prev.instruction[17:16] != 2'b11) begin // CC
          `logLevel( stage1, 1, $format("Case I - CC"), wr_simulate_log_start)
          final_instruction[0] = zeroExtend(rg_prev.instruction[15:0]);
          final_instruction[1] = zeroExtend(rg_prev.instruction[31:16]);
          instr_pc[0] = rg_prev.pc;
          instr_pc[1] = rg_prev.pc + 2;
          compressed_instr = replicate(True);
          lv_prev.mask = 2'b01;
        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbresponse;
        `endif
          lv_prev.instruction = zeroExtend(rg_prev.instruction[47:32]);
          lv_prev.pc = rg_prev.pc + 4;
          trap = replicate(False);
          lv_deq_response = False;
        end
        else if (rg_prev.instruction[1:0] != 2'b11 && rg_prev.instruction[17:16] == 2'b11) begin // SC
          `logLevel( stage1, 1, $format("Case I - SC"), wr_simulate_log_start)
          final_instruction[0] = zeroExtend(rg_prev.instruction[15:0]);
          final_instruction[1] = rg_prev.instruction[47:16];
          instr_pc[0] = rg_prev.pc;
          instr_pc[1] = rg_prev.pc + 2;
          compressed_instr[0] = True;
          compressed_instr[1] = False;
          lv_prev.mask = 2'b00;
          trap = replicate(False);
          lv_deq_response = False;
        end
      end
      else if(rg_prev.mask == 2'b10 && rg_prev.epochs == curr_epoch)begin
        valid_instructions = 2;
        if (rg_prev.instruction[1:0] == 2'b11 && imem_resp.word[1:0] == 2'b11) begin
          `logLevel( stage1, 1, $format("Case II - SS"), wr_simulate_log_start)
          final_instruction[0] = rg_prev.instruction[31:0];
          final_instruction[1] = imem_resp.word[31:0];
          instr_pc[0] = rg_prev.pc;
          instr_pc[1] = stage0pc.address;
          compressed_instr = replicate(False);
          lv_prev.mask = 2'b10;
        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbresponse;
        `endif
          lv_prev.instruction = zeroExtend(imem_resp.word[63:32]);
          lv_prev.pc = stage0pc.address + 4;
          lv_deq_response = True;
          //deq_response();
          trap[0] = False;
          trap[1] = imem_resp.trap;
        end
        else if (rg_prev.instruction[1:0] == 2'b11 && imem_resp.word[1:0] != 2'b11) begin
          `logLevel( stage1, 1, $format("Case II - CS"), wr_simulate_log_start)
          final_instruction[0] = rg_prev.instruction[31:0];
          final_instruction[1] = zeroExtend(imem_resp.word[15:0]);
          instr_pc[0] = rg_prev.pc;
          instr_pc[1] = stage0pc.address;
          compressed_instr[0] = False;
          compressed_instr[1] = True;
          lv_prev.mask = 2'b11;
        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbresponse;
        `endif
          lv_prev.instruction = imem_resp.word[63:16];
          lv_prev.pc = stage0pc.address + 2;
          //deq_response();
          lv_deq_response = True;
          trap[0] = False;
          trap[1] = imem_resp.trap;
        end
        else if (rg_prev.instruction[1:0] != 2'b11 && rg_prev.instruction[17:16] != 2'b11) begin
          `logLevel( stage1, 1, $format("Case II - CC"), wr_simulate_log_start)
          final_instruction[0] = zeroExtend(rg_prev.instruction[15:0]);
          final_instruction[1] = zeroExtend(rg_prev.instruction[31:16]);
          instr_pc[0] = rg_prev.pc;
          instr_pc[1] = rg_prev.pc + 2;
          compressed_instr[0] = True;
          compressed_instr[1] = True;
          lv_prev.mask = 2'b00;
          trap = replicate(False);
          lv_deq_response = False;
        end
        else if (rg_prev.instruction[1:0] != 2'b11 && rg_prev.instruction[17:16] == 2'b11) begin
          `logLevel( stage1, 1, $format("Case II - SC"), wr_simulate_log_start)
          final_instruction[0] = zeroExtend(rg_prev.instruction[15:0]);
          final_instruction[1] = {imem_resp.word[15:0], rg_prev.instruction[31:16]};
          instr_pc[0] = rg_prev.pc;
          instr_pc[1] = rg_prev.pc + 2;
          compressed_instr[0] = True;
          compressed_instr[1] = False;
          lv_prev.mask = 2'b11;
        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbresponse;
        `endif
          lv_prev.instruction = imem_resp.word[63:16];
          lv_prev.pc = stage0pc.address | 2;
          //deq_response();
          lv_deq_response = True;
          trap[0] = False;
          trap[1] = imem_resp.trap;
        end
      end
      else if(rg_prev.mask == 2'b01 && rg_prev.epochs == curr_epoch)begin
        valid_instructions = 2;
        //deq_response();
        lv_deq_response = True;
        trap[0] = False;
        trap[1] = imem_resp.trap;
        if (rg_prev.instruction[1:0] == 2'b11 && imem_resp.word[17:16] == 2'b11) begin
          `logLevel( stage1, 1, $format("Case III - SS"), wr_simulate_log_start)
          final_instruction[0] = {imem_resp.word[15:0], rg_prev.instruction[15:0]};
          final_instruction[1] = imem_resp.word[47:16];
          instr_pc[0] = rg_prev.pc;
          instr_pc[1] = stage0pc.address + 2;
          compressed_instr = replicate(False);
          lv_prev.mask = 2'b01;
        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbresponse;
        `endif
          lv_prev.instruction = zeroExtend(imem_resp.word[63:48]);
          lv_prev.pc = stage0pc.address + 6;
        end
        else if (rg_prev.instruction[1:0] == 2'b11 && imem_resp.word[17:16] != 2'b11) begin
          `logLevel( stage1, 1, $format("Case III - CS"), wr_simulate_log_start)
          final_instruction[0] = {imem_resp.word[15:0], rg_prev.instruction[15:0]};
          final_instruction[1] = zeroExtend(imem_resp.word[32:16]);
          instr_pc[0] = rg_prev.pc;
          instr_pc[1] = stage0pc.address + 2;
          compressed_instr[0] = False;
          compressed_instr[1] = True;
          lv_prev.mask = 2'b10;
        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbresponse;
        `endif
          lv_prev.instruction = zeroExtend(imem_resp.word[63:32]);
          lv_prev.pc = stage0pc.address | 4;
        end
        else if (rg_prev.instruction[1:0] != 2'b11 && imem_resp.word[1:0] != 2'b11) begin
          `logLevel( stage1, 1, $format("Case III - CC"), wr_simulate_log_start)
          final_instruction[0] = zeroExtend(rg_prev.instruction[15:0]);
          final_instruction[1] = zeroExtend(imem_resp.word[15:0]);
          instr_pc[0] = rg_prev.pc;
          instr_pc[1] = stage0pc.address;
          compressed_instr[0] = True;
          compressed_instr[1] = True;
          lv_prev.mask = 2'b11;
        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbresponse;
        `endif
          lv_prev.instruction = imem_resp.word[63:16];
          lv_prev.pc = stage0pc.address | 2;
        end
        else if (rg_prev.instruction[1:0] != 2'b11 && imem_resp.word[1:0] == 2'b11) begin
          `logLevel( stage1, 1, $format("Case III - SC"), wr_simulate_log_start)
          final_instruction[0] = zeroExtend(rg_prev.instruction[15:0]);
          final_instruction[1] = imem_resp.word[31:0];
          instr_pc[0] = rg_prev.pc;
          instr_pc[1] = stage0pc.address;
          compressed_instr[0] = True;
          compressed_instr[1] = False;
          lv_prev.mask = 2'b10;
        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbresponse;
        `endif
          lv_prev.instruction = zeroExtend(imem_resp.word[63:32]);
          lv_prev.pc = stage0pc.address | 4;
        end
      end
      else if (rg_prev.mask == 2'b00 && stage0pc.discard == 2'b00) begin
        valid_instructions = 2;
        //deq_response();
        lv_deq_response = True;
        trap = replicate(imem_resp.trap);
        if (imem_resp.word[1:0] == 2'b11 && imem_resp.word[33:32] == 2'b11) begin
          `logLevel( stage1, 1, $format("Case IV - SS"), wr_simulate_log_start)
          final_instruction[0] = imem_resp.word[31:0];
          final_instruction[1] = imem_resp.word[63:32];
          instr_pc[0] = stage0pc.address;
          instr_pc[1] = stage0pc.address | 4;
          compressed_instr[0] = False;
          compressed_instr[1] = False;
          lv_prev.mask = 2'b00;
        end
        else if (imem_resp.word[1:0] == 2'b11 && imem_resp.word[33:32] != 2'b11) begin
          `logLevel( stage1, 1, $format("Case IV - CS"), wr_simulate_log_start)
          final_instruction[0] = imem_resp.word[31:0];
          final_instruction[1] = zeroExtend(imem_resp.word[47:32]);
          instr_pc[0] = stage0pc.address;
          instr_pc[1] = stage0pc.address | 4;
          compressed_instr[0] = False;
          compressed_instr[1] = True;
          lv_prev.mask = 2'b01;
        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbresposne;
        `endif
          lv_prev.instruction = zeroExtend(imem_resp.word[63:48]);
          lv_prev.pc = stage0pc.address | 6;
        end
        else if (imem_resp.word[1:0] != 2'b11 && imem_resp.word[17:16] != 2'b11) begin
          `logLevel( stage1, 1, $format("Case IV - CC"), wr_simulate_log_start)
          final_instruction[0] = zeroExtend(imem_resp.word[15:0]);
          final_instruction[1] = zeroExtend(imem_resp.word[31:16]);
          instr_pc[0] = stage0pc.address;
          instr_pc[1] = stage0pc.address | 2;
          compressed_instr[0] = True;
          compressed_instr[1] = True;
          lv_prev.mask = 2'b10;
        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbrespone;
        `endif
          lv_prev.instruction = zeroExtend(imem_resp.word[63:32]);
          lv_prev.pc = stage0pc.address | 4;
        end
        else if (imem_resp.word[1:0] != 2'b11 && imem_resp.word[17:16] == 2'b11) begin
          `logLevel( stage1, 1, $format("Case IV - SC"), wr_simulate_log_start)
          final_instruction[0] = zeroExtend(imem_resp.word[15:0]);
          final_instruction[1] = imem_resp.word[47:16];
          instr_pc[0] = stage0pc.address;
          instr_pc[1] = stage0pc.address | 2;
          compressed_instr[0] = True;
          compressed_instr[1] = False;
          lv_prev.mask = 2'b01;
        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbrespone;
        `endif
          lv_prev.instruction = zeroExtend(imem_resp.word[63:48]);
          lv_prev.pc = stage0pc.address | 6;
        end
      end
      else if (rg_prev.mask == 2'b00 && stage0pc.discard == 2'b01) begin
        //deq_response();
        lv_deq_response = True;
        trap = replicate(imem_resp.trap);
        if (imem_resp.word[17:16] == 2'b11 && imem_resp.word[49:48] == 2'b11) begin
          `logLevel( stage1, 1, $format("Case V - SS"), wr_simulate_log_start)
          final_instruction[0] = imem_resp.word[47:16];
          final_instruction[1] = ?;
          compressed_instr = replicate(False);
          instr_pc[0] = stage0pc.address | 2;
          instr_pc[1] = ?;
          lv_prev.mask = 2'b01;
        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbresponse;
        `endif
          lv_prev.instruction = zeroExtend(imem_resp.word[63:48]);
          lv_prev.pc = stage0pc.address | 6;
          valid_instructions = 1;
        end
        else if (imem_resp.word[17:16] == 2'b11 && imem_resp.word[49:48] != 2'b11) begin
          `logLevel( stage1, 1, $format("Case V - CS"), wr_simulate_log_start)
          final_instruction[0] = imem_resp.word[47:16];
          final_instruction[1] = zeroExtend(imem_resp.word[63:48]);
          compressed_instr[0] = False;
          compressed_instr[1] = True;
          instr_pc[0] = stage0pc.address | 2;
          instr_pc[1] = stage0pc.address | 6;
          lv_prev.mask = 2'b00;
          valid_instructions = 2;
        end
        else if (imem_resp.word[17:16] != 2'b11 && imem_resp.word[33:32] != 2'b11) begin
          `logLevel( stage1, 1, $format("Case V - CC"), wr_simulate_log_start)
          final_instruction[0] = zeroExtend(imem_resp.word[31:16]);
          final_instruction[1] = zeroExtend(imem_resp.word[47:32]);
          compressed_instr = replicate(True);
          instr_pc[0] = stage0pc.address | 2;
          instr_pc[1] = stage0pc.address | 4;
          lv_prev.mask = 2'b01;
        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbresponse;
        `endif
          lv_prev.instruction = zeroExtend(imem_resp.word[63:48]);
          lv_prev.pc = stage0pc.address | 6;
          valid_instructions = 2;
        end
        else if (imem_resp.word[17:16] != 2'b11 && imem_resp.word[33:32] == 2'b11) begin 
          `logLevel( stage1, 1, $format("Case V - SC"), wr_simulate_log_start)
          final_instruction[0] = zeroExtend(imem_resp.word[31:16]);
          final_instruction[1] = imem_resp.word[63:32];
          compressed_instr[0] = True;
          compressed_instr[1] = False;
          instr_pc[0] = stage0pc.address | 2;
          instr_pc[1] = stage0pc.address | 4;
          lv_prev.mask = 2'b00;
          valid_instructions = 2;
        end
      end
      else if (rg_prev.mask == 2'b00 && stage0pc.discard == 2'b10) begin
        //deq_response();
        lv_deq_response = True;
        trap = replicate(imem_resp.trap);
        if (imem_resp.word[33:32] == 2'b11) begin
          `logLevel( stage1, 1, $format("Case VI - S"), wr_simulate_log_start)
          final_instruction[0] = imem_resp.word[63:32];
          final_instruction[1] = ?;
          instr_pc[0] = stage0pc.address | 4;
          instr_pc[1] = ?;
          compressed_instr = replicate(False);
          lv_prev.mask = 2'b00;
          valid_instructions = 1;
        end
        else if (imem_resp.word[33:32] != 2'b11 && imem_resp.word[49:48] != 2'b11) begin
          `logLevel( stage1, 1, $format("Case VI - CC"), wr_simulate_log_start)
          final_instruction[0] = zeroExtend(imem_resp.word[47:32]);
          final_instruction[1] = zeroExtend(imem_resp.word[63:48]);
          instr_pc[0] = stage0pc.address | 4;
          instr_pc[1] = stage0pc.address | 6;
          compressed_instr = replicate(True);
          lv_prev.mask = 2'b00;
          valid_instructions = 2;
        end
        else if (imem_resp.word[33:32] != 2'b11 && imem_resp.word[49:48] == 2'b11) begin
          `logLevel( stage1, 1, $format("Case VI - SC"), wr_simulate_log_start)
          final_instruction[0] = zeroExtend(imem_resp.word[47:32]);
          final_instruction[1] = ?;
          compressed_instr[0] = True;
          compressed_instr[1] = False;
          instr_pc[0] = stage0pc.address | 4;
          instr_pc[1] = ?;
          lv_prev.mask = 2'b01;
        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbresponse;
        `endif
          lv_prev.instruction = zeroExtend(imem_resp.word[63:48]);
          lv_prev.pc = stage0pc.address | 6;
          valid_instructions = 1;
        end
      end
      else if (rg_prev.mask == 2'b00 && stage0pc.discard == 2'b11) begin
        //deq_response();
        lv_deq_response = True;
        trap = replicate(imem_resp.trap);
        if (imem_resp.word[49:48] == 2'b11) begin
          `logLevel( stage1, 1, $format("Case VII - S"), wr_simulate_log_start)
          final_instruction = replicate(?);
          compressed_instr = replicate(False);
          instr_pc[0] = ?;
          instr_pc[1] = ?;
          lv_prev.mask = 2'b01;
        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbresponse;
        `endif
          lv_prev.instruction = zeroExtend(imem_resp.word[63:48]);
          lv_prev.pc = stage0pc.address | 6;
          valid_instructions = 0;
        end
        else if (imem_resp.word[49:48] != 2'b11) begin
          `logLevel( stage1, 1, $format("Case VII - C"), wr_simulate_log_start)
          final_instruction[0] = zeroExtend(imem_resp.word[63:48]);
          final_instruction[1] = ?;
          compressed_instr[0] = True;
          compressed_instr[1] = False;
          instr_pc[0] = stage0pc.address | 6;
          instr_pc[1] = ?;
          lv_prev.mask = 2'b00;
          valid_instructions = 1;
        end
      end
      else begin
        valid_instructions = 0;
        final_instruction = replicate(?);
        compressed_instr = replicate(False);
      end

      lv_prev.epochs = curr_epoch;
      Bit#(`causesize) cause = imem_resp.cause;
    `ifdef triggers
      let {trig_trap, trig_cause} <- check_trigger(stage0pc.address, final_instruction
                                        `ifdef compressed ,compressed `endif );
      if(trig_trap)begin
        trap = True;
        cause = trig_cause;
      end
    `endif
      Vector#(`num_issue, Bit#(32)) inst;
      inst[0] = final_instruction[0];
      inst[1] = final_instruction[1];

      if (compressed_instr[0])
        final_instruction[0] = fn_decompress(truncate(final_instruction[0]));

      if (compressed_instr[1])
        final_instruction[1] = fn_decompress(truncate(final_instruction[1]));

    Vector#(`num_issue, PIPE1) pipedata;
    for (Integer i=0; i<`num_issue; i=i+1) begin
			pipedata[i] = PIPE1{program_counter : instr_pc[i],
                      instruction : final_instruction[i],
                      epochs:{rg_eEpoch, rg_wEpoch},
                      trap : trap[i]
                    `ifdef bpu
                      ,btbresponse: btbresponse[i]
                    `endif
                    `ifdef compressed
                      ,upper_err : receiving_upper[i] && imem_resp.trap
                      ,compressed: compressed_instr[i]
                    `endif
                      ,cause : cause };
    end

  `ifdef rtldump
    Vector#(`num_issue, CommitLogPacket) commit_packet;
    for (Integer i=0; i<`num_issue; i=i+1) begin
      commit_packet[i] = CommitLogPacket{instruction: inst[i], 
                                         pc: instr_pc[i], 
                                         mode: ?,
                                         inst_type: tagged None};
    end
  `endif

      `logLevel( stage1, 0,$format("[%2d]STAGE1 : PC:%h: ",hartid,stage0pc.address,
                                    fshow(ff_memory_response.first)), wr_simulate_log_start)
    `ifdef compressed
      `logLevel( stage1, 1,$format("[%2d]STAGE1 : Prev_mask: %d Prev_data: %h",hartid, rg_prev.mask, rg_prev.instruction, " discard:%d ", stage0pc.discard), wr_simulate_log_start)
    `endif
      if (valid_instructions == 2 && !tx_tostage2.u.enqReady_2()) begin
        `logLevel( stage1, 0, $format("[%2d]STAGE1 : Instruction queue full. Cannot enque two instructions ", hartid), wr_simulate_log_start)
        valid_instructions = 0;
      end
      else begin
        valid_instructions = valid_instructions;
        if (lv_deq_response)
          deq_response();
        //deq_response();
        //rg_action <= lv_action;
        rg_prev <= lv_prev;
      end
        
      if (valid_instructions != 0) begin
        `ifdef rtldump 
          tx_commitlog.u.enq(commit_packet, valid_instructions);
        `endif
          tx_tostage2.u.enq(pipedata, valid_instructions);
          if (valid_instructions == 1)
            `logLevel( stage1, 0,$format("[%2d]STAGE1 : Enquing 1 pipe data: ",hartid,fshow(pipedata[0])), wr_simulate_log_start)
          else 
            `logLevel( stage1, 0,$format("[%2d]STAGE1 : Enquing 2 pipe data: ",hartid,fshow(pipedata)), wr_simulate_log_start)

      end
    endrule

    // MethodName : inst_response_put
    // Explicit Conditions : None
    // Implicit Conditions : ff_memory_response.notFull
    // Description : This method will capture the response from the memory subsytem and enque it in
    // a FIFO. One could of think of performing all the function in the process_instruction
    // rule in this method itself. This would only work if you are not supporting compressed
    // instructions. When you support compressed, the cache can send a single response which
    // contains 2 16 - bit instruction. In such a case the process_instruction rule will fire twice
    // and deque the fifo only on the second run. Thus we need to have a fifo which will store the
    // response from the cache for an extra cycle.
    // The former approach could work with compressed as well if : we process both the instructions
    // and enqueue them simultaneously into the next stage. Not sure what other dependencies would
    // be there?
    interface icache = interface Ifc_s1_icache
  		interface inst_response = interface Put
  			method Action put (IMem_core_response#(TMul#(`iwords, 8), `iesize) resp);
          `logLevel( stage1, 3, $format("[%2d]STAGE1: ",hartid,fshow(resp)), wr_simulate_log_start)
          ff_memory_response.enq(resp);
  			endmethod
      endinterface;
    endinterface;

    // Description : This interface will capture the prediction response from the BTB module. If
    // compressed is supported the, BTB will provide 2 predictions for each of the 2byte addresses
    // that have been fetched from the I - mem. If compressed is not supported then a single
    // prediction is only provided for the entire 32 - bit instruction has been received from the
    // I - cache.
    interface rx = interface Ifc_s1_rx
      interface rx_from_stage0 = rx_fromstage0.e;
    endinterface;

    // MethodName : tx_to_stage2
    // Explicit Conditions : None
    // Implicit Conditions : tx is not empty.
    // Description : This method will transmit the instruction to the next stage.
    interface tx = interface Ifc_s1_tx
  		interface tx_to_stage2 = tx_tostage2.e;
    `ifdef rtldump
	  	interface tx_commitlog = tx_commitlog.e;
    `endif
    endinterface;

    interface common = interface Ifc_s1_common
      // MethodName : update_eEpoch
      // Explicit Conditions : None
      // Implicit Conditions : None
      method Action ma_update_eEpoch;
        rg_eEpoch<=~rg_eEpoch;
      endmethod
  
      // MethodName : update_wEpoch
      // Explicit Conditions : None
      // Implicit Conditions : None
      method Action ma_update_wEpoch;
        rg_wEpoch<=~rg_wEpoch;
      endmethod
  
      // This method captures the "c" of misa csr
      method Action ma_csr_misa_c (Bit#(1) c);
        wr_csr_misa_c <= c;
      endmethod
    `ifdef simulate
      method Action ma_simulate_log_start(Bit#(1) start);
        wr_simulate_log_start <= start;
      endmethod
    `endif
    `ifdef triggers
      method Action trigger_data1(Vector#(`trigger_num, TriggerData) t);
        for(Integer i=0; i<`trigger_num; i=i+1)
          v_trigger_data1[i] <= t[i];
      endmethod
      method Action trigger_data2(Vector#(`trigger_num, Bit#(`xlen)) t);
        for(Integer i=0; i<`trigger_num; i=i+1)
          v_trigger_data2[i] <= t[i];
      endmethod
      method Action trigger_enable(Vector#(`trigger_num, Bool) t);
        for(Integer i=0; i<`trigger_num; i=i+1)
          v_trigger_enable[i] <= t[i];
      endmethod
    `endif
    endinterface;
  `ifdef perfmonitors
    interface perf = interface Ifc_s1_perfmonitors
      method mv_instr_queue_full = pack(!tx_tostage2.u.enqReady_1);
    endinterface;
  `endif
  endmodule
endpackage

