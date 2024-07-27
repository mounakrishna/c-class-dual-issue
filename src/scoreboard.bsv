// See LICENSE.iitm for license details
/*
Author: IIT Madras
Created on: Friday 18 June 2021 12:34:39 PM

*/
/*doc:overview
This module implements the scoreboard which currently simply implements a single bit per register
indicating if there exists an instruction in the pipeline which has that particular register as its
destination register.

*/
package scoreboard ;
  import FIFOF        :: * ;
  import Vector       :: * ;
  import SpecialFIFOs :: * ;
  import FIFOF        :: * ;

  `include "Logger.bsv"
  import ccore_types  :: * ;

  interface Ifc_scoreboard;
    method ActionValue#(Vector#(`num_issue, Bit#(`wawid))) ma_lock_rd (Vector#(`num_issue, SBDUpd) lock);
    method Action ma_release_rd (Vector#(`num_issue, SBDUpd) rls);
    method SBD mv_board;
  endinterface: Ifc_scoreboard

  /*doc:module: */
`ifdef scoreboard_noinline
  (*synthesize*)
`endif
  module mkscoreboard#(parameter Bit#(`xlen) hartid)(Ifc_scoreboard);

  `ifdef spfpu
    Vector#(64, Array#(Reg#(SBEntry))) rg_rf_board <- replicateM(mkCReg(2,unpack(0)));
  `else
    Vector#(32, Array#(Reg#(SBEntry))) rg_rf_board <- replicateM(mkCReg(2,unpack(0)));
  `endif

  `ifdef no_wawstalls
    /*doc:reg: */
    Reg#(Bit#(`wawid)) rg_renameid <- mkReg(0);
  `endif
    /*doc:method: This method is used to lock a destination register. WAW is prevented by ensuring
    * that the rd of the new instruction is not already locked*/
    method ActionValue#(Vector#(`num_issue, Bit#(`wawid))) ma_lock_rd (Vector#(`num_issue, SBDUpd) lock);
      `logLevel( sboard, 0, $format("[%2d]SBoard Lock 0 for : ",hartid,fshow(lock[0])))
      `logLevel( sboard, 0, $format("[%2d]SBoard Lock 1 for : ",hartid,fshow(lock[1])))
      Vector#(`num_issue, Bit#(`wawid)) id;
      Vector#(`num_issue, Bit#(6)) index;
      Vector#(`num_issue, SBEntry) entry;
      for (Integer i=0; i<`num_issue; i=i+1) begin
        index[i] =  { `ifdef spfpu pack(lock[i].rdtype), `endif lock[i].rd};
        entry[i] = rg_rf_board[index[i]][0];
      `ifdef no_wawstalls
        id[i] = rg_renameid + fromInteger(i);
        entry[i].id = id[i];
      `endif
        entry[i].lock = 1;
        //if (index !=0 ) 
        //  rg_rf_board[index][0] <= entry;
      end

      if ((index[0] == index[1]) && index[0] != 0) //TODO: Fixed for dual issue. Need to think of multi-issue
        rg_rf_board[index[1]][0] <= entry[0]; //TODO: entry[1];
      else begin
        if (index[0] != 0)
          rg_rf_board[index[0]][0] <= entry[0];
        // TODO:
        //if (index[1] != 0)
        //  rg_rf_board[index[1]][0] <= entry[1];
      end
      rg_renameid <= rg_renameid + 1; //`num_issue;
    `ifdef no_wawstalls
      return id;
    `else
      return ?;
    `endif
    endmethod
    /*doc:method: This method is used to release the lock of a destination register when the
     * instruction has committed.*/
    method Action ma_release_rd (Vector#(`num_issue, SBDUpd) rls);
      Vector#(`num_issue, Bit#(6)) index;
      Vector#(`num_issue, SBEntry) entry;
      for (Integer i=0; i<`num_issue; i=i+1) begin
        index[i] =  { `ifdef spfpu pack(rls[i].rdtype), `endif rls[i].rd};
        entry[i] = rg_rf_board[index[i]][1];
        `ifdef no_wawstalls if (rls[i].id == entry[i].id) `endif entry[i].lock = 0;
        //if (index !=0  ) 
        //    rg_rf_board[index][1] <=  entry;
        `logLevel( sboard, 0, $format("[%2d]SBoard release %d for : ",hartid, i, fshow(rls[i])))
        `logLevel( sboard, 0, $format("[%2d]SBoard release %d entry : ",hartid, i, fshow(entry[i])))
      end
      if ((index[0] == index[1]) && index[0] != 0) //TODO: Fixed for dual issue. Need to think of multi-issue
        rg_rf_board[index[1]][1] <= entry[0]; //TODO: entry[1];
      else begin
        if (index[0] != 0)
          rg_rf_board[index[0]][1] <= entry[0];
        //if (index[1] != 0)
        //  rg_rf_board[index[1]][1] <= entry[1];
      end
    endmethod
    /*doc:method: This method provides a peek into the current score-board status */
    method SBD mv_board;
      Bit#(`ifdef spfpu 64 `else 32 `endif ) _rflock;
    `ifdef no_wawstalls
      Vector#(`ifdef spfpu 64 `else 32 `endif , Bit#(`wawid)) _id;

      for (Integer i = 0; i< `ifdef spfpu 64 `else 32 `endif ; i = i + 1) begin
        _rflock[i] = rg_rf_board[i][0].lock;
      `ifdef no_wawstalls
        _id[i] = rg_rf_board[i][0].id;
      `endif
      end

      return SBD{rf_lock: _rflock 
                 `ifdef no_wawstalls ,v_id: _id `endif
                };
    endmethod
  endmodule:mkscoreboard
endpackage: scoreboard

