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
  import RegFile5r2w  :: * ;

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

  //`ifdef spfpu
  //  Vector#(64, Array#(Reg#(SBEntry))) rg_rf_board <- replicateM(mkCReg(2,unpack(0)));
  //`else
  //  Vector#(32, Array#(Reg#(SBEntry))) rg_rf_board <- replicateM(mkCReg(2,unpack(0)));
  //`endif
  `ifdef spfpu
    RegFile5r2w#(Bit#(6), SBEntry) rg_rf_board <- mkRegFile5r2wWCF(0, 63);
  `else
    RegFile5r2w#(Bit#(5), SBEntry) rg_rf_board <- mkRegFile5r2wWCF(0, 31);
  `endif

  `ifdef no_wawstalls
    /*doc:reg: */
    Reg#(Bit#(`wawid)) rg_renameid <- mkReg(0);
  `endif

    /*doc:method: This method is used to lock a destination register. WAW is prevented by ensuring
    * that the rd of the new instruction is not already locked*/
    method ActionValue#(Vector#(`num_issue, Bit#(`wawid))) ma_lock_rd (Vector#(`num_issue, SBDUpd) lock);
      `logLevel( sboard, 0, $format("[%2d]SBoard Lock for : ",hartid,fshow(lock)))
      Vector#(`num_issue, Bit#(`wawid)) lv_rename_id;
      Vector#(`num_issue, SBEntry) entry;
      Vector#(`num_issue, Bit#(6)) index;
      for (Integer i = 0; i < `num_issue; i=i+1) begin
        index[i] =  { `ifdef spfpu pack(lock[i].rdtype), `endif lock[i].rd};
      `ifdef no_wawstalls
        entry[i].id = rg_renameid + i;
        lv_rename_id[i] = entry[i].id;
      `endif
        entry.lock = 1;
      end
      rg_renameid <= rg_renameid + `num_issue;
      rg_rf_board.upd_1(index[0], entry[0]);
      rg_rf_board.upd_2(index[1], entry[1]);

      `ifdef no_wawstalls
        return lv_rename_id;
      `else
        return ?;
      `endif
    endmethod
    /*doc:method: This method is used to release the lock of a destination register when the
     * instruction has committed.*/
    method Action ma_release_rd (SBDUpd rls);
      Vector#(`num_issue, SBEntry) entry;
      Vector#(`num_issue, Bit#(6)) index;
      for (Integer i=0; i<`num_issue; i=i+1) begin
        index =  { `ifdef spfpu pack(rls.rdtype), `endif rls.rd};
        entry = rg_rf_board.sub(index);
        `ifdef no_wawstalls if (rls.id == entry.id) `endif entry.lock = 0;
      end
      rg_rf_board.upd_1(index[0], entry[0]);
      rg_rf_board.upd_2(index[1], entry[1]);
      `logLevel( sboard, 0, $format("[%2d]SBoard release for : ",hartid,fshow(rls)))
      `logLevel( sboard, 0, $format("[%2d]SBoard release entry : ",hartid,fshow(entry)))
    endmethod
    /*doc:method: This method provides a peek into the current score-board status */
    method SBEntry mv_board(Bit#(6) index);
      return rg_rf_board.sub(index);
    endmethod
  endmodule:mkscoreboard
endpackage: scoreboard

