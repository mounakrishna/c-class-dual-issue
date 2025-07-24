//See LICENSE.iitm for license details
/*

Author : IIT Madras
Details: This module implements a fullt-associative GSHARE branch predictor with
Return-Address-Stack support. It basically has the prediction and training phase. The comments on
the respective methods describe their operations.

--------------------------------------------------------------------------------------------------
*/
package gshare_fa;

  // -- library imports
  import Assert :: *;
  import ConfigReg :: * ;
  import Vector :: * ;
  import OInt :: * ;
  import RegFile :: * ;

  // -- project imports
  `include "Logger.bsv"
  `include "ccore_params.defines"
  import ccore_types :: *;
`ifdef bpu_ras
  import stack :: * ;
`endif

  `define ignore 3

  // the following macro describes the number of banks the bht array is split into
  `define bhtcols 2

  /*doc:struct: This struct defines the fields of each entry in the Branch-Target-Buffer*/
  typedef struct{
    Bit#(`vaddr)  target;               // full target virtual address
    ControlInsn   ci;                   // indicate the type of entry. Branch, JAL, Call, Ret.
    Bool compressed;                    // When True indicates that the Ci is a compressed instruction.
    Bit#(1) ci_offset;                  // The below number is formed by combining ci_offset of two entries of the same tag 
                                        // 0 - Prediction for instruction at offset pc+0;
                                        // 1 - Prediction for instruction at offset pc+2;
                                        // 2 - Prediction for instruction at offset pc+4;
                                        // 3 - Prediction for instruction at offset pc+6;
  } BTBEntry deriving(Bits, Eq, FShow);

  /*doc:struct: This struct holds the tag and valid bit of each BTB entry.
  Each entry corresponds to a tag for a 4-byte aligned address. This means that this tag can be a
  hit of at-most 2 instructions when compressed is supported. To distinguish between the 2
  instructions we have provided a 'hi' field in BTBEntry, which when true indicates that the higher
  instruction within the 4-byte address is a hit/trained */
  typedef struct{
    Bit#(TSub#(`vaddr, `ignore)) tag;
    Bool valid;
  } BTBTag deriving(Bits, Eq, FShow);


  /*doc : func : function to calculate the hash address to access the branch-history-table.
  1. Here the lower 2 bits of the pc are ignored since they are always going to be 0 for a
  non-compressed supported system.
  2. We take lower (extrahist+histlen) bit of pc and XOR with the next set of (extrahist+histlen)
  bits of pc and with the GHR register.
  Note here that the ghr is right shifted i.e. the speculation is inserted at the MSB and hence the
  usage of truncateLSB.
  This has proven to be a better hash function, not so costly. It is completely empirical and better
  hash functions could exist and can replace this function with that to evaluate what works best.
  */
  /*function Bit#(TLog#(TDiv#(`bhtdepth,`bhtcols))) fn_hash (
                                      Bit#(`histlen) history, Bit#(`vaddr) pc);
    return truncate(pc >> `ignore) ^ truncate(pc >> (`ignore +
                                                     valueOf(TLog#(TDiv#(`bhtdepth,`bhtcols)))))
                                   ^ truncateLSB(history);
  endfunction*/
  function Bit#(TLog#(TDiv#(`bhtdepth,`bhtcols))) fn_hash (
                                      Bit#(`histlen) history, Bit#(`vaddr) pc);
    
    Bit#(TLog#(TDiv#(`bhtdepth,`bhtcols))) pc_hash = 
            truncate(pc >> `ignore) 
          ^ zeroExtend((pc >> (`ignore + valueOf(TLog#(TDiv#(`bhtdepth,`bhtcols)))))[2:0]);
    Bit#(`histbits) _h = truncateLSB(history);
    Bit#(TLog#(TDiv#(`bhtdepth,`bhtcols))) hist_hash = zeroExtend(_h << (valueOf(TLog#(`bhtdepth)) - `histbits));
    return pc_hash ^ hist_hash;
  endfunction

  interface Ifc_bpu;
    /*doc : method : receive the request of new pc and return the next pc in case of hit. */
    method ActionValue#(PredictionResponse) mav_prediction_response (PredictionRequest r);

    /*doc : method : method to train the BTB and BHT tables based on the evaluation from execute
    stage*/
	  method Action ma_train_bpu (Training_data td);

    /*doc : method : This method is fired when there is a misprediction.
    It received 2 fields. A boolean indicating if the instructin was a conditional branch or not.
    The second field contains the GHR value after predicting the same instruction. In case
    of a conditional branch the LSB bit of this GHR is negated and this value is restored in the
    rg_ghr register. Otherwise, the GHR directly written to the rg_ghr register. */
    method Action ma_mispredict (Tuple2#(Bool, Bit#(`histlen)) g);

    /*doc : method : This method captures if the bpu is enabled through csr or not*/
    method Action ma_bpu_enable (Bool e);
  `ifdef simulate
    method Action ma_simulate_log_start(Bit#(1) start);
  `endif
  endinterface

`ifdef bpu_noinline
  (*synthesize*)
`endif
  module mkbpu#(parameter Bit#(`xlen) hartid) (Ifc_bpu);

    String bpu = "";

  `ifdef bpu_ras
    Ifc_stack#(`vaddr, `rasdepth) ras_stack <- mkstack;
  `endif
  `ifdef simulate
    Wire#(Bit#(1)) wr_simulate_log_start <- mkDWire(0);
  `endif

    /*doc : vec : This vector of register holds the BTB entries. We use vector instead of array
    to leverage the select function provided by bluespec*/
    Vector#(`btbdepth, Reg#(Vector#(2, BTBEntry))) v_reg_btb_entry <-
                                                  replicateM(mkReg(replicate(BTBEntry{target: ?, ci : None
                                                  ,ci_offset : 0
                                                  `ifdef compressed ,compressed: False `endif })));
                                           //`ifdef compressed ,instr16: False, hi:False `endif }));

    /*doc : vec : This vector holds the BTB tags and the respecitve valid bits. This has been split
    from the BTB entries for better hw of CAM look-ups and index retrieval */
    Vector#(`btbdepth, Reg#(BTBTag)) v_reg_btb_tag <- replicateM(mkReg(unpack(0)));

    /*doc : reg : This array holds the branch history table. The bht table banked into `bhtcols
    banks. In case of compressed `bhtcols is 2 else 1. By banking it becomes easy to access the bht
    in case of compressed support since we are storing only one BTB per 4-byte align addresses.
    Each entry is `statesize-bits wide and represents a up/down saturated counter.
    The reset value is set to 1 */
    /*Reg#(Bit#(`statesize)) rg_bht_arr[`bhtcols][`bhtdepth/`bhtcols];
    for(Integer i = 0; i < `bhtcols; i =  i + 1)
      for(Integer j = 0; j < `bhtdepth/`bhtcols ; j =  j + 1)
        rg_bht_arr[i][j] <- mkReg(1);*/
    
    RegFile#(Bit#(TLog#(TDiv#(`bhtdepth,`bhtcols))), Bit#(`statesize)) rg_bht_arr[`bhtcols];
    for (Integer i = 0; i<`bhtcols; i = i + 1) begin
      rg_bht_arr[i] <- mkRegFileWCF(0,fromInteger(valueOf(TDiv#(`bhtdepth,`bhtcols))-1));
    end
    /*doc:reg: */
    Reg#(Bit#(TLog#(TDiv#(`bhtdepth, `bhtcols)))) rg_bht_index <- mkReg(0);
    /*doc : reg : This register points to the next entry in the Fully associative BTB that should
    be allocated for a new entry */
    Reg#(Bit#(TLog#(`btbdepth))) rg_allocate <- mkReg(0);

    /*doc : reg : This register holds the global history buffer. There are two methods which can
    update this register: mav_prediction_response and ma_mispredict. The former is called every
    time a new pc is generted and updates the regiser speculatively for conditional branches
    which are a hit in the BTB. The later method called when a mis-prediction occurs and restores
    the register with the non-speculative version.
    Both of these method are in conflict with each other. One way to resolve this would be create a
    preempts attribute given ma_mispredict a higher priority since it doesn't make sense to provide
    a prediction knowing the pipe has flushed. However, this solution would create a path from the
    ma_mispredict enable method to the mav_prediction_response output ready signal making it the
    critical path.
    Alternate to that is to implement this register as a CReg where the ma_mispredict value shadows
    the value updated by the mav_prediction_response method. This remove the above critical path.
    */
    Reg#(Bit#(`histlen)) rg_ghr[2] <- mkCReg(2, 0);

    /*doc : wire : This wire indicates if the predictor is enabled or disabled by the csr config*/
    Wire#(Bool) wr_bpu_enable <- mkWire();

  `ifdef ifence
    /*doc : reg : When true this register flushes all the entries and cleans up the btb*/
    ConfigReg#(Bool) rg_initialize <- mkConfigReg(False);

    /*doc : rule : This rule flushes the btb and puts it back to the initial reset state.
    This rule would be called each time a fence.i is being performed. This rule will also reset the
    ghr and rg_allocate register*/
    rule rl_initialize (rg_initialize);
      for(Integer i = 0; i < `btbdepth; i = i + 1) begin
        v_reg_btb_tag[i]<=unpack(0);
        v_reg_btb_entry[i]<=unpack(0);
      end
      for(Integer i = 0; i < `bhtcols ; i = i + 1)
        rg_bht_arr[i].upd(rg_bht_index,1);
      if (rg_bht_index == fromInteger(valueOf(TDiv#(`bhtdepth,`bhtcols))-1))
        rg_initialize <= False;
      rg_bht_index <= rg_bht_index + 1;
      rg_ghr[1] <= 0;
      rg_allocate <= 0;
    `ifdef bpu_ras
      ras_stack.clear;
    `endif
    endrule
  `endif

    /*doc:method: This method provides prediction for a requested PC.
    If a fence.i is requested, then the rg_initialize register is set to true.

    The index of the bht is obtained using the hash function above on the pc and the current value
    of GHR. This index is then used to find the entry in the BHT.

    We then perform a fully-associative look-up on the BTB. We compare the tags with the pc and
    check for the valid bit to be set. This applied to each entry and a corresponding bit is set in
    match_ variable. By nature of how training and prediction is performed, we expect match_
    variable to be a one-hot vector i.e. only one entry is a hit in the entire BTB. Multiple entries
    can't be a hit since update comes from only one source.

    Then using the match_ variable and the special select function from BSV we pick out the entry
    that is a hit. A hit is detected only is OR(match) != 0.

    Depending on the ci type the prediction variable is set either to 3 or the value in the BHT
    entry that we indexed earlier.

    In case of a BTB hit and the ci being a Branch, the ghr is left-shifted and the lsb is set to 1
    if predicted taken else 0. This ghr is also sent back out along with the prediction and target
    address.

    We also send out a boolean value indicating if the pc caused a hit in the BTB or not.

    Fence: This feature is required for self-modifying codes. Software is required to conduct an
    fence.i each time the text-section is modified by the software. When this happens we need to
    flush the branch predictor as well else non-branch instructions could be treated as predicted
    taken leading to wrong behavior

    Working of RAS: Earlier versions of the predictor included a separate method which would push
    the return address onto the RAS. This address came from the execute stage which when a Call
    instruction was detected. If the BTB was a hit for a pc and it detected a Ret type ci then the
    RAS popped. However with this architecture you could have a push happening from an execute stage
    and a ret being detected in the predictor, this return would never see this latest push and thus
    would pick the wrong address from the RAS. So essentially the RAS would work only if the
    call-ret are a few number of instructions apart, for smaller functions the RAS would fail
    consistently.

    To fix this problem, we push and pop with the predictor itself. If a pc is a btb hit and is a
    Call type ci, the pc+4 value if pushed on the Stack. If the subsequent pc was a btb hit and a
    Ret type ci, it would immediately pick up the RAS top which would be correct. Thus, an empty
    function would also benefit from this mechanism.
    */
    method ActionValue#(PredictionResponse) mav_prediction_response (PredictionRequest r)
                                                         `ifdef ifence if(!rg_initialize) `endif ;
      `logLevel( bpu, 0, $format("[%2d]BPU : Received Request: ",hartid, fshow(r), " ghr:%h",rg_ghr[0]), wr_simulate_log_start)
    `ifdef ifence
      if( r.fence && wr_bpu_enable)
        rg_initialize <= True;
    `endif
      let bht_index_ = fn_hash(rg_ghr[0], r.pc);
      Bit#(`statesize) branch_state_ [`bhtcols];
      for(Integer i = 0; i < `bhtcols ; i = i + 1)
        branch_state_[i] = rg_bht_arr[i].sub(bht_index_);

      Bit#(`statesize) prediction_ = 1;
      Bool hit = False;
      Bit#(`histlen) ghr = rg_ghr[0];


      Vector#(2, Bool) pop_ras = replicate(False);
      Vector#(2, Bool) push_ras = replicate(False);
      Vector#(2, Bit#(`vaddr)) push_pc = replicate(0);

      `ifdef compressed
        Bool edgecase = False;
      `endif
      Bit#(`vaddr) target_ = r.pc;
      Vector#(2, Bool) taken = replicate(False);
      Vector#(2, BTBResponse) btbresponse = replicate(BTBResponse{prediction: 1, btbhit: False, ci_offset : 0
                                                                 `ifdef gshare , history : ghr `endif });

      if(!r.fence && wr_bpu_enable) begin
        Bit#(`btbdepth) match_;
        // a one - hot vector to store the hits of the btb
        for(Integer i = 0; i < `btbdepth; i =  i + 1)
          match_[i] = pack(v_reg_btb_tag[i].tag == truncateLSB(r.pc) && v_reg_btb_tag[i].valid);

        `logLevel( bpu, 1, $format("[%2d]BPU : Match:%h",hartid, match_), wr_simulate_log_start)

        hit = unpack(|match_);
        let v_hit_entry = select(readVReg(v_reg_btb_entry), unpack(match_));

        if(hit) begin
          for (Integer i=0; i<2; i=i+1)
            `logLevel( bpu, 1, $format("[%2d]BPU : BTB Hit: ",hartid,fshow(v_hit_entry[i])), wr_simulate_log_start)
        end

        if (hit) begin
          if ((r.discard[1] == 1 && v_hit_entry[1].ci_offset >= r.discard[0]) || (r.discard[1] == 0)) begin
            if (v_hit_entry[1].ci == Call) begin
              Bit#(`vaddr) ras_push_offset;
              case (v_hit_entry[1].ci_offset) //TODO: Didnt include support when compressed is OFF.
                'b0: ras_push_offset = v_hit_entry[0].compressed ? 6 : 8;
                'b1: ras_push_offset = v_hit_entry[0].compressed ? 8 : 12;
                default: ras_push_offset = 0;
              endcase
              push_pc[1] = r.pc + ras_push_offset;
              push_ras[1] = True;
              //ras_stack.push(push_pc);
              target_ = target_;
              //compressed_ = v_hit_entry[1].compressed;
              taken[1] = True;
              btbresponse[1] = BTBResponse { prediction: 3, btbhit: True, ci_offset: {1'b1, v_hit_entry[1].ci_offset}
                                             `ifdef gshare , history : ghr `endif };
            end
            else if(v_hit_entry[1].ci == Ret) begin
              target_ = ras_stack.top;
              //compressed_ = v_hit_entry[1].compressed;
              pop_ras[1] = True;
              taken[1] = True;
              btbresponse[1] = BTBResponse { prediction: 3, btbhit: True, ci_offset: {1'b1, v_hit_entry[1].ci_offset}
                                             `ifdef gshare , history : ghr `endif };
              //ras_stack.pop;
            end
            else if(v_hit_entry[1].ci == JAL) begin
              taken[1] = True;
              target_ = v_hit_entry[1].target;
              //compressed_ = v_hit_entry[1].compressed;
              btbresponse[1] = BTBResponse { prediction: 3, btbhit: True, ci_offset: {1'b1, v_hit_entry[1].ci_offset}
                                             `ifdef gshare , history : ghr `endif };
            end
            else if(v_hit_entry[1].ci == Branch) begin
              prediction_ = branch_state_[1];
              taken[1] = unpack(prediction_[`statesize-1]);
              //compressed_ = v_hit_entry[1].compressed;
              if (taken[1])
                target_ = v_hit_entry[1].target;
              ghr = {prediction_[`statesize - 1], truncateLSB(rg_ghr[0])};
              btbresponse[1] = BTBResponse { prediction: prediction_, btbhit: True, ci_offset: {1'b1, v_hit_entry[1].ci_offset}
                                             `ifdef gshare , history : ghr `endif };
            end
          end

          if (r.discard[1] == 0 && v_hit_entry[0].ci_offset >= r.discard[0]) begin
            if (v_hit_entry[0].ci == Call) begin
              Bit#(`vaddr) ras_push_offset;
              case (v_hit_entry[0].ci_offset) //TODO: Didnt include support when compressed is OFF.
                'b0: ras_push_offset = v_hit_entry[0].compressed ? 2 : 4;
                'b1: ras_push_offset = v_hit_entry[0].compressed ? 4 : 6;
                default: ras_push_offset = 0;
              endcase
              push_ras[0] = True;
              push_pc[0] = r.pc + ras_push_offset;
              target_ = target_;
              //compressed_ = v_hit_entry[0].compressed;
              taken[0] = True;
              btbresponse[0] = BTBResponse { prediction: 3, btbhit: True, ci_offset: {1'b0, v_hit_entry[0].ci_offset}
                                             `ifdef gshare , history : ghr `endif };
            end
            else if (v_hit_entry[0].ci == Ret) begin
              target_ = ras_stack.top;
              pop_ras[0] = True;
              taken[0] = True;
              //compressed_ = v_hit_entry[0].compressed;
              btbresponse[0] = BTBResponse { prediction: 3, btbhit: True, ci_offset: {1'b0, v_hit_entry[0].ci_offset}
                                             `ifdef gshare , history : ghr `endif };
            end
            else if (v_hit_entry[0].ci == JAL) begin
              taken[0] = True;
              target_ = v_hit_entry[0].target;
              //compressed_ = v_hit_entry[0].compressed;
              btbresponse[0] = BTBResponse { prediction: 3, btbhit: True, ci_offset: {1'b0, v_hit_entry[0].ci_offset}
                                             `ifdef gshare , history : ghr `endif };
            end
            else if (v_hit_entry[0].ci == Branch) begin
              prediction_ = branch_state_[0];
              taken[0] = unpack(prediction_[`statesize-1]);
              //compressed_ = v_hit_entry[0].compressed;
              if (taken[0])
                target_ = v_hit_entry[0].target;
              ghr = {prediction_[`statesize - 1], truncateLSB(rg_ghr[0])};
              btbresponse[0] = BTBResponse { prediction: prediction_, btbhit: True, ci_offset: {1'b0, v_hit_entry[0].ci_offset}
                                              `ifdef gshare , history : ghr `endif };

            end
          end

          if (taken[0] && push_ras[0])
            ras_stack.push(push_pc[0]);
          else if (!taken[0] && taken[1] && push_ras[1])
            ras_stack.push(push_pc[1]);

          if (taken[0] && pop_ras[0])
            ras_stack.pop;
          else if (!taken[0] && taken[1] && pop_ras[1])
            ras_stack.pop;

          `ifdef compressed
            if (taken[1] && !taken[0] && !v_hit_entry[1].compressed && v_hit_entry[1].ci_offset == 1)
              edgecase = True;
          `endif
        end
        else begin
          taken = replicate(False);
          btbresponse = replicate(BTBResponse{prediction: 1, btbhit: False, ci_offset : 0
                                                 `ifdef gshare , history : ghr `endif });
        end

        rg_ghr[0] <= ghr;

        `ifdef ASSERT
          dynamicAssert(countOnes(match_) < 2, "Multiple Matches in BTB");
        `endif


        `logLevel( bpu, 0, $format("[%2d]BPU : BHTindex_:%d Target:%h Pred:%d",hartid,
                                                  bht_index_, target_, prediction_), wr_simulate_log_start)
      end

      return PredictionResponse{ nextpc : target_, btbresponse: btbresponse
        `ifdef compressed ,edgecase: edgecase `endif , taken: (taken[0] || taken[1])};


        // A local variable to indicate which entry in the BTB is giving the prediction.
        //if (hit) begin
        //  if (v_hit_entry[0].ci == Call && r.discard[1] == 0 && v_hit_entry[0].ci_offset >= r.discard[0]) begin
        //    Bit#(`vaddr) ras_push_offset;
        //    case (v_hit_entry[0].ci_offset) //TODO: Didnt include support when compressed is OFF.
        //      'b0: ras_push_offset = v_hit_entry[0].compressed ? 2 : 4;
        //      'b1: ras_push_offset = v_hit_entry[0].compressed ? 4 : 6;
        //      default: ras_push_offset = 0;
        //    endcase
        //    let push_pc = r.pc + ras_push_offset;
        //    ras_stack.push(r.pc);
        //    prediction_ = 3;
        //    target_ = target_;
        //    ci_off = {1'b0, v_hit_entry[0].ci_offset};
        //    compressed = v_hit_entry[0].compressed;
        //    `logLevel( bpu, 0, $format("[%2d]BPU : Pushing to RAS: %h", hartid, push_pc), wr_simulate_log_start)
        //  end
        //  else if (v_hit_entry[0].ci == Ret && r.discard[1] == 0 && v_hit_entry[0].ci_offset >= r.discard[0]) begin
        //    target_ = ras_stack.top;
        //    ras_stack.pop;
        //    ci_off = {1'b0, v_hit_entry[0].ci_offset};
        //    compressed = v_hit_entry[0].compressed;
        //    `logLevel( bpu, 1, $format("[%2d]BPU: Choosing from top RAS:%h",hartid,target_), wr_simulate_log_start)
        //    prediction_ = 3;
        //  end
        //  else if (v_hit_entry[0].ci == JAL && r.discard[1] == 0 && v_hit_entry[0].ci_offset >= r.discard[0]) begin
        //    prediction_ = 3;
        //    target_ = v_hit_entry[0].target;
        //    ci_off = {1'b0, v_hit_entry[0].ci_offset};
        //    compressed = v_hit_entry[0].compressed;
        //  end
        //  else if (v_hit_entry[0].ci == Branch  && r.discard[1] == 0 && v_hit_entry[0].ci_offset >= r.discard[0] && branch_state_[0][`statesize - 1] == 1) begin
        //    prediction_ = branch_state_[0];
        //    target_ = v_hit_entry[0].target;
        //    ghr = {prediction_[`statesize - 1], truncateLSB(rg_ghr[0])};
        //    ci_off = {1'b0, v_hit_entry[0].ci_offset};
        //    compressed = v_hit_entry[0].compressed;
        //  end
        //  else if (v_hit_entry[1].ci == Call && (r.discard[1] == 0 || (r.discard[1] == 1 && v_hit_entry[1].ci_offset >= r.discard[0]))) begin
        //    Bit#(`vaddr) ras_push_offset;
        //    case (v_hit_entry[1].ci_offset) //TODO: Didnt include support when compressed is OFF.
        //      'b0: ras_push_offset = v_hit_entry[0].compressed ? 6 : 8;
        //      'b1: ras_push_offset = v_hit_entry[0].compressed ? 8 : 12;
        //      default: ras_push_offset = 0;
        //    endcase
        //    let push_pc = r.pc + ras_push_offset;
        //    ras_stack.push(r.pc);
        //    prediction_ = 3;
        //    target_ = target_;
        //    ci_off = {1'b1, v_hit_entry[1].ci_offset};
        //    compressed = v_hit_entry[1].compressed;
        //    `logLevel( bpu, 0, $format("[%2d]BPU : Pushing to RAS: %h", hartid, push_pc), wr_simulate_log_start)
        //  end
        //  else if (v_hit_entry[1].ci == Ret && (r.discard[1] == 0 || (r.discard[1] == 1 && v_hit_entry[1].ci_offset >= r.discard[0]))) begin
        //    target_ = ras_stack.top;
        //    ras_stack.pop;
        //    ci_off = {1'b1, v_hit_entry[1].ci_offset};
        //    compressed = v_hit_entry[1].compressed;
        //    `logLevel( bpu, 1, $format("[%2d]BPU: Choosing from top RAS:%h",hartid,target_), wr_simulate_log_start)
        //    prediction_ = 3;
        //  end
        //  else if (v_hit_entry[1].ci == JAL && (r.discard[1] == 0 || (r.discard[1] == 1 && v_hit_entry[1].ci_offset >= r.discard[0]))) begin
        //    prediction_ = 3;
        //    target_ = v_hit_entry[1].target;
        //    ci_off = {1'b1, v_hit_entry[1].ci_offset};
        //    compressed = v_hit_entry[1].compressed;
        //  end
        //  else if (v_hit_entry[1].ci == Branch  && (r.discard[1] == 0 || (r.discard[1] == 1 && v_hit_entry[1].ci_offset >= r.discard[0]))) begin
        //    prediction_ = branch_state_[1];
        //    target_ = v_hit_entry[1].target;
        //    ghr = {prediction_[`statesize - 1], truncateLSB(rg_ghr[0])};
        //    ci_off = {1'b1, v_hit_entry[1].ci_offset};
        //    compressed = v_hit_entry[1].compressed;
        //  end
        //  else begin
        //    ci_off = 0;
        //    compressed = False;
        //    prediction_ = 1;
        //    target_ = target_;
        //  end
        //end
        //else begin
        //  ci_off = 0;
        //  compressed = False;
        //  prediction_ = 1;
        //  target_ = target_;
        //end

      //end
      //else begin
      //  //ci_off = 0;
      //  //compressed = False;
      //  //prediction_ = 1;
      //  target_ = target_;
      //  btbresponse = replicate(BTBResponse{prediction: 1, btbhit: False, ci_offset : 0,
      //                                      compressed: False  `ifdef gshare , history : ghr `endif };
      //end

      //let btbresponse = BTBResponse{prediction: prediction_, btbhit: hit, ci_offset : ci_off
      //                  `ifdef gshare , history : ghr `endif };

    endmethod

    /*doc:method: This method is called for all unconditional and conditional jumps.
    Using the pc of the instruction we first check if the entry already exists in the btb or not. If
    it does then entry is updated with a new/same target from the execute stage.

    If the entry does not exists then a new entry is allotted in the btb depending on rg_allocate
    value.

    Additionally in case of conditional branches, the bht is again indexed using the pc and the ghr.
    This entry is updated only if the BTB was a hit during prediction i.e. only on the second
    instance of the branch the bht gets updated.

    It was first thought to be better to send the btbindex along the pipe to reduce the additional
    look-up hw here. However, for really small loops its possible that while training for an entry
    in a cycle, the same instruction is getting predicted again. This will cause a miss in the
    prediction and the training of the second instance would lead to allocating a new entry. This
    would lead to duplicates and thus would require zapping them - another simultaneous look-up. It
    seems the current approach does to seem close on required frequencies.
    */
    method Action ma_train_bpu (Training_data d) if(wr_bpu_enable
                                                          `ifdef ifence && !rg_initialize `endif );
      `logLevel( bpu, 4, $format("[%2d]BPU : Received Training: ",hartid,fshow(d)), wr_simulate_log_start)

      function Bool fn_tag_match (BTBTag a);
        return  (a.tag == truncateLSB(d.pc) && a.valid);
      endfunction

      let hit_index_ = findIndex(fn_tag_match, readVReg(v_reg_btb_tag));

      if(hit_index_ matches tagged Valid .h) begin
        v_reg_btb_entry[h][d.pc[2]] <= BTBEntry{ target : d.target, ci : d.ci
                            ,ci_offset : d.pc[1]
                            `ifdef compressed ,compressed : d.compressed `endif };
                            //`ifdef compressed ,instr16: d.instr16, hi:unpack(d.pc[1]) `endif };
        `logLevel( bpu, 4, $format("[%2d]BPU : Training existing Entry index: %d",hartid,h), wr_simulate_log_start)
      end
      else begin
        `logLevel( bpu, 4, $format("[%2d]BPU : Allocating new index: %d",hartid,rg_allocate), wr_simulate_log_start)
        Vector#(2, BTBEntry) btb_entry = replicate(unpack(0));
        //v_reg_btb_entry[rg_allocate][d.pc[2]] <= BTBEntry{ target : d.target, ci : d.ci
        btb_entry[d.pc[2]] = BTBEntry{ target : d.target, ci : d.ci
                            ,ci_offset : d.pc[1]
                            `ifdef compressed ,compressed : d.compressed `endif };
        if(v_reg_btb_tag[rg_allocate].valid) begin
          btb_entry[~d.pc[2]] = BTBEntry{ target: ?, ci: None,
                                           ci_offset : ?
                                           `ifdef compressed ,compressed : ? `endif };
          `logLevel( bpu, 4, $format("[%2d]BPU : Conflict Detected",hartid), wr_simulate_log_start)
        end
        else begin
          btb_entry[~d.pc[2]] = v_reg_btb_entry[rg_allocate][~d.pc[2]];
        end

        v_reg_btb_entry[rg_allocate] <= btb_entry;
        v_reg_btb_tag[rg_allocate] <= BTBTag{tag: truncateLSB(d.pc), valid: True};
        rg_allocate <= rg_allocate + 1;
      end

      // we use the ghr version before the prediction to train the BHT
      `logLevel( bpu, 4, $format("[%2d]BPU : BHT Hash inputs during training : ghr : %h, pc : %h", hartid, d.history << 1, d.pc), wr_simulate_log_start)
      let bht_index_ = fn_hash(d.history<<1, d.pc);
      if(d.ci == Branch && d.btbhit) begin
        rg_bht_arr[d.pc[2]].upd(bht_index_, d.state);
        `logLevel( bpu, 4, $format("[%2d]BPU : Upd BHT entry: %d with state: %d",hartid,
                                                                              bht_index_, d.state), wr_simulate_log_start)
      end
    endmethod

    /*doc:method: This method is called each time the evaluation stage detects a mis-prediction. If
    the misprediction was due to a conditional branch then the ghr is fixed by flipping the lsb
    and then writing it to the rg_ghr.
    */
    method Action ma_mispredict (Tuple2#(Bool, Bit#(`histlen)) g)
                                                         `ifdef ifence if(!rg_initialize) `endif ;
      let {btbhit, ghr} = g;
      if(btbhit)
        ghr[`histlen-1] = ~ghr[`histlen-1];
      `logLevel( bpu, 4, $format("[%2d]BPU : Misprediction fired. Restoring ghr:%h",hartid,
                                                                                              ghr), wr_simulate_log_start)
      rg_ghr[1] <= ghr;
    endmethod
  `ifdef simulate
    method Action ma_simulate_log_start(Bit#(1) start);
      wr_simulate_log_start <= start;
    endmethod
  `endif

    method Action ma_bpu_enable (Bool e);
      wr_bpu_enable <= e;
    endmethod

  endmodule
endpackage

