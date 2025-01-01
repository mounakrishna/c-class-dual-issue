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
    Bit#(16) instruction;
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
    Reg#(ActionType) rg_action <- mkReg(None);

    // This register indicates that the lower 16 - bits of the response from the cache need to be
    // ignored. This happens because, when there is jump to non - 4-byte aligned address the cache
    // still receives a previous 4 - byte - ailgned address from the fetch stage.
    //Vector#(`num_issue, Reg#(Bool)) vrg_receiving_upper <- replicateM(mkReg(False));

    // This register holds the 16 - bits of instruction from the previous cache response if required.
    Reg#(PrevMeta) rg_prev <- mkReg(?);

    // This FIFO receives the response from the memory subsytem (a.k.a cache)
    FIFOF#(IMem_core_response#(32, `iesize)) ff_memory_response <- mkSizedBypassFIFOF(2);

    // This FIFO receives the response from the branch prediction unit (bpu or ras)
    RX#(Stage0PC#(`vaddr)) rx_fromstage0 <- mkRX;

    // FIFO to interface with the next pipeline stage
		TX_MIMO#(2, 2, `instr_queue, PIPE1) tx_tostage2 <- mkTX_MIMO;

  `ifdef rtldump
		TX_MIMO#(2, 2, `instr_queue, CommitLogPacket) tx_commitlog <- mkTX_MIMO;
  `endif

    // This variable holds the current epoch values of the pipe
    let curr_epoch = {rg_eEpoch, rg_wEpoch};

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
                                                                        fshow(v_trigger_data1[i])))
        `logLevel( stage1, 3, $format("[%2d]STAGE1: Trigger[%2d] Data2: ",hartid, i, 
                                                                        fshow(v_trigger_data2[i])))
        `logLevel( stage1, 3, $format("[%2d]STAGE1: Trigger[%2d] Enable: ",hartid, i, 
                                                                        fshow(v_trigger_enable[i])))
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
      `logLevel( stage1, 0, $format("[%2d]STAGE1 : Instruction queue full to enque one instruction ", hartid))
    endrule
    rule rl_stage0_pipe_notEmpty(!rx_fromstage0.u.notEmpty);
      `logLevel( stage1, 0, $format("[%2d]STAGE1 : Stage0 pipe is not yet full", hartid))
    endrule
    rule rl_mem_response_wait(!ff_memory_response.notEmpty);
      `logLevel( stage1, 0, $format("[%2d]STAGE1 : Waiting for Icache response", hartid))
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
      `logLevel( stage1, 1, $format("[%2d]STAGE1 : Prediction: ",hartid, fshow(stage0pc)))
    `ifdef bpu
      let btbresponse = stage0pc.btbresponse;
    `endif

      // capture the response from the cache
      let imem_resp = ff_memory_response.first;
      Bool trap = False;

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
      ActionType lv_action = None;

      // local variable indicating if the current instruction under analysis should be enqueued to
      // the next stage or dropped.
      //Bool enque_instruction = True;
      //Bool deq_resp = False;
      PrevMeta lv_prev = rg_prev;
      // if epochs do not match then drop the instruction
      if(curr_epoch != imem_resp.epochs)begin
        //deq_resp = True;
        valid_instructions = 0;
        //deq_response;
        lv_action = None;
        tx_tostage2.u.flush();
        `ifdef rtldump
          tx_commitlog.u.flush();
        `endif
        //enque_instruction = False;
        `logLevel( stage1, 1,$format("[%2d]STAGE1 : Dropping Instruction. ExpEpoch:%b CurrEpoch:%b",
            hartid, imem_resp.epochs, curr_epoch))
        receiving_upper[0] = False;
        receiving_upper[1] = False;
      end
    `ifdef compressed
      else if(rg_action == CheckPrev && rg_prev.epochs == curr_epoch)begin
      `ifdef bpu
        btbresponse = rg_prev.btbresponse;
        if(!btbresponse.hi)begin
          btbresponse.btbhit= False;
          btbresponse.prediction = 1;
        end
      `endif
        if(rg_prev.instruction[1 : 0] == 2'b11) begin
          `logLevel( stage1, 1, $format("Forming a 32 bit instruction from two ICACHE responses"))
          final_instruction[0] = {imem_resp.word[15:0], rg_prev.instruction};
          instr_pc[0] = rg_prev.pc;
          compressed_instr[0] = False;
          if (imem_resp.word[17:16] != 2'b11) begin
            final_instruction[1] = zeroExtend(imem_resp.word[31:16]);
            compressed_instr[1] = True;
            valid_instructions = 2;
            instr_pc[1] = stage0pc.address | zeroExtend(2'b10);
            lv_action = None;
            lv_prev = lv_prev;
          end
          else begin
            final_instruction[1] = ?;
            instr_pc[1] = ?;
            compressed_instr[1] = False;
            lv_action = CheckPrev;
            valid_instructions = 1;
            //deq_response;
            lv_prev.instruction = truncateLSB(imem_resp.word);
            lv_prev.pc = stage0pc.address | zeroExtend(2'b10);
          end
          trap = imem_resp.trap;
          //let deq_valid <- tx_tostage2.u.enqReady(valid_instructions);
          //if (deq_valid)
          //deq_resp = True;

          //stage0pc.address = rg_prev.pc | zeroExtend(2'b10);

        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbresponse;
          if(btbresponse.prediction > 1) 
            lv_action = None;
          else 
            lv_action = lv_action;
        `endif
          receiving_upper[0] = True;
          receiving_upper[1] = False;
        end
        else begin 
          compressed_instr[0] = True;
          final_instruction[0] = zeroExtend(rg_prev.instruction);
          instr_pc[0] = rg_prev.pc;
          if (imem_resp.word[1:0] != 2'b11) begin
            `logLevel( stage1, 1, $format("STAGE1: Previous stored instr is compressed and the new received instruction also has compressed, leading to dual issue and storing upper 16 as previous"))
            final_instruction[1] = zeroExtend(imem_resp.word[15:0]);
            instr_pc[1] = stage0pc.address;
            compressed_instr[1] = True;
            valid_instructions = 2;
            lv_action = CheckPrev;
            lv_prev.instruction = truncateLSB(imem_resp.word);
            lv_prev.pc = stage0pc.address | zeroExtend(2'b10);
          end
          else begin
            `logLevel( stage1, 1, $format("STAGE1: Previous stored instr is compressed and the new received instruction is not compressed. Issueing both"))
            valid_instructions = 2;
            compressed_instr[1] = False;
            final_instruction[1] = imem_resp.word;
            instr_pc[1] = stage0pc.address;
            lv_action = None;
            lv_prev = lv_prev;
          end
          receiving_upper = replicate(False);
        end
      end
      // discard the lower - 16bits of the imem - response.
      else if(stage0pc.discard && wr_csr_misa_c == 1)begin
        `logLevel( stage1, 1, $format("STAGE1: Discard the lower 16 bits as it has been issued into the pipeline"))
      `ifdef bpu
        if(!btbresponse.hi)begin
          btbresponse.btbhit= False;
          btbresponse.prediction = 1;
        end
      `endif
        if(imem_resp.word[17 : 16] == 2'b11)begin
          `logLevel( stage1, 1, $format("STAGE1: After discarding lower 16 bits, the upper 16 bits is part of a 32 bit instruction. So CheckPrev action set."))
          trap = imem_resp.trap;
          lv_action = CheckPrev;
          valid_instructions = 0;
          final_instruction = replicate(?);
          instr_pc = replicate(?);
          receiving_upper[0] = True;
          receiving_upper[1] = False;
          lv_prev.instruction = imem_resp.word[31 : 16];
          lv_prev.pc = stage0pc.address | zeroExtend(2'b10);
        `ifdef bpu
          lv_prev.btbresponse = stage0pc.btbresponse;
        `endif
        end
        else begin
          `logLevel( stage1, 1, $format("STAGE1: After discarding lower 16 bits, the upper 16 bits is a compressed instruction. So issuing it."))
          lv_action = None;
          receiving_upper = replicate(False);
          compressed_instr[0] = True;
          compressed_instr[1] = False;
          final_instruction[0] = zeroExtend(imem_resp.word[31 : 16]);
          final_instruction[1] = ?;
          valid_instructions = 1;
          lv_prev = lv_prev;
          trap = imem_resp.trap;
          instr_pc[0] = stage0pc.address | zeroExtend(2'b10);
          instr_pc[1] = ?;
        end
        //let deq_valid <- tx_tostage2.u.enqReady(valid_instructions);
        //if (deq_valid)
        //  deq_response;
        //deq_response;
        //deq_resp = True;
      end
    `endif
      else begin
        trap = imem_resp.trap;
        //deq_response;
        // if instruction is 32-bit simply pass it on
        if(imem_resp.word[1 : 0] == 'b11)begin
          receiving_upper = replicate(False);
          `logLevel( stage1, 1, $format("STAGE1: The fetched word is a 32 bit instr, just passing it"))
          final_instruction[0] = imem_resp.word;
          final_instruction[1] = ?;
          compressed_instr[0] = False;
          compressed_instr[1] = False;
          instr_pc[0] = stage0pc.address;
          instr_pc[1] = ?;
          valid_instructions = 1;
          lv_action = None;
          lv_prev = lv_prev;
        end
      `ifdef compressed
        // if instruction from cache is compressed
        else if(wr_csr_misa_c == 1) begin
          compressed_instr[0] = True;
          final_instruction[0] = zeroExtend(imem_resp.word[15 : 0]);
          instr_pc[0] = stage0pc.address;
          if (imem_resp.word[17:16] == 2'b11) begin
            receiving_upper = replicate(False);
            `logLevel( stage1, 1, $format("STAGE1: The fetched word has one 16 bit instr and the upper 16 bits is part of a 32 bit instruction"))
            final_instruction[1] = ?;
            compressed_instr[1] = False;
            valid_instructions = 1;
            lv_prev.instruction = truncateLSB(imem_resp.word);
            lv_prev.pc = stage0pc.address | zeroExtend(2'b10);
            instr_pc[1] = ?;
            `ifdef bpu
              lv_action = (!btbresponse.hi && btbresponse.prediction > 1) ? None: CheckPrev;
            `else
              lv_action = CheckPrev;
            `endif
          end
          else begin
            receiving_upper = replicate(False);
            `logLevel( stage1, 1, $format("STAGE1: The fetched word has two 16 bit instrs"))
            final_instruction[1] = zeroExtend(imem_resp.word[31:16]);
            compressed_instr[1] = True;
            valid_instructions = 2; //TODO Need to make it 2 for dual issue
            lv_prev = lv_prev;
            instr_pc[1] = stage0pc.address | zeroExtend(2'b10);
            lv_action = None;
          end
        `ifdef bpu
          // check if prediction was for lower-16 else reset btbresponse before forwarding.
          //lv_action = (!btbresponse.hi && btbresponse.prediction > 1) ? None: CheckPrev;
          //rg_receiving_upper <= !(!btbresponse.hi && btbresponse.prediction > 1);
          if(btbresponse.hi)begin
            btbresponse.btbhit= False;
            btbresponse.prediction = 1;
          end
          // store the btbresponse for next cycle
          lv_prev.btbresponse = stage0pc.btbresponse;
        `endif
        end
      `endif
      end
      lv_prev.epochs = curr_epoch;
      //rg_prev <= lv_prev;
      //Bit#(`vaddr) incr_value = (compressed_instr[0] && !compressed_instr[1]  && wr_csr_misa_c == 1) ? 2:4;
      //Bit#(`vaddr) incr_value = (compressed_instr[0] && wr_csr_misa_c == 1) ? 2:4;
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
                      trap : trap
                    `ifdef bpu
                      ,btbresponse: btbresponse
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
                                    fshow(ff_memory_response.first)))
    `ifdef compressed
      `logLevel( stage1, 1,$format("[%2d]STAGE1 : rg_action: ",hartid,fshow(rg_action),
            " misa[c]:%b discard:%b ",hartid, wr_csr_misa_c, stage0pc.discard))
    `endif
      //tx_tostage2.u.enqReady_count(valid_instructions);
      //let enq_ready <- tx_tostage2.u.enqReady(valid_instructions);
      //Bool enq_ready;
      //if (valid_instructions == 1)
      //  enq_ready = tx_tostage2.u.enqReady_1();
      //else if (valid_instructions == 2)
      //  enq_ready = tx_tostage2.u.enqReady_2();
      //else
      //  enq_ready = False;

      //if (deq_resp && enq_ready)
      //  deq_response;
      //if(enque_instruction) begin

      if (valid_instructions == 2 && !tx_tostage2.u.enqReady_2()) begin
        `logLevel( stage1, 0, $format("[%2d]STAGE1 : Instruction queue full. Cannot enque two instructions ", hartid))
        valid_instructions = 0;
      end
      else begin
        valid_instructions = valid_instructions;
        deq_response();
        rg_action <= lv_action;
        rg_prev <= lv_prev;
      end
        
      if (valid_instructions != 0) begin
        `ifdef rtldump 
          tx_commitlog.u.enq(commit_packet, valid_instructions);
          //tx_commitlog.u.enq(CommitLogPacket{instruction: inst, pc: instr_pc[0], mode: ?,
          //    inst_type: tagged None});
        `endif
          tx_tostage2.u.enq(pipedata, valid_instructions);
          if (valid_instructions == 1)
            `logLevel( stage1, 0,$format("[%2d]STAGE1 : Enquing 1 pipe data: ",hartid,fshow(pipedata[0])))
          else 
            `logLevel( stage1, 0,$format("[%2d]STAGE1 : Enquing 2 pipe data: ",hartid,fshow(pipedata)))

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
  			method Action put (IMem_core_response#(32, `iesize) resp);
          `logLevel( stage1, 3, $format("[%2d]STAGE1: ",hartid,fshow(resp)))
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
  endmodule
endpackage

