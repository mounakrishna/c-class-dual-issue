// See LICENSE.iitm for license details
/*
Author: IIT Madras
Created on: Thursday 17 June 2021 09:27:28 PM

*/
package riscv;
  import FIFOF        :: * ;
  import Vector       :: * ;
  import SpecialFIFOs :: * ;
  import FIFOF        :: * ;
  import Connectable  :: * ;
  import GetPut       :: * ;
  import DReg         :: * ;
  import MIMO_MODIFY  :: * ;
  `include "Logger.bsv"

`ifdef muldiv
  import mbox         :: * ;
`endif
`ifdef spfpu 
  import fpu         :: * ;
`endif
  import scoreboard   :: * ;
  import stage0       :: * ;
  import stage1       :: * ;
  import stage2       :: * ;
  import stage3       :: * ;
  import stage4       :: * ;
  import stage5       :: * ;
  import pipe_ifcs    :: * ;
  import ccore_types  :: * ;
`ifdef rtldump
  import csrbox       :: * ;
`endif

  interface Ifc_riscv_csrs;
    method Bit#(`xlen) mv_csr_mstatus;
    method Bit#(4) mv_cacheenable;
    method Bit#(2) mv_curr_priv;
	`ifdef supervisor
		method Bit#(`xlen) mv_csr_satp;
	`endif
  `ifdef hypervisor
  	method Bit#(`xlen) mv_csr_vsatp;
  	method Bit#(`xlen) mv_csr_hgatp;
  	method Bit#(1) mv_vs_bit;
	  method Bit#(`xlen) mv_csr_hstatus;
  	method Bit#(`xlen) mv_csr_vsstatus;				
		`ifdef RV32
			method Bit#(`xlen) mv_csr_mstatush;
		`endif
  `endif	
  `ifdef pmp
    method Vector#(`pmpentries, Bit#(8)) mv_pmp_cfg;
    method Vector#(`pmpentries, Bit#(`paddr)) mv_pmp_addr;
  `endif
  endinterface:Ifc_riscv_csrs

`ifdef perfmonitors
  interface Ifc_riscv_perfmonitors;
  `ifdef icache
    /*doc:method: */
    method Action ma_icache_counters (Bit#(5) i);
  `endif
  `ifdef dcache
    /*doc:method: */
    method Action ma_dcache_counters (Bit#(13) i);
  `endif
  `ifdef supervisor
    method Action ma_dtlb_counters (Bit#(1) i);
    method Action ma_itlb_counters (Bit#(1) i);
  `endif
  endinterface:Ifc_riscv_perfmonitors
`endif

  interface Ifc_pipe_status;
    method Vector#(5,Bool) mv_pipe_isbs_empty;
    method Bool mv_wfi_detected;
  endinterface:Ifc_pipe_status

  interface Ifc_riscv;
    interface Ifc_s0_icache s0_icache;

    interface Ifc_s1_icache s1_icache;
    
    interface Ifc_s3_cache s3_cache;
    
    interface Ifc_s4_cache s4_cache;

    interface Ifc_s5_interrupts interrupts;
    interface Ifc_s5_cache s5_cache;
    interface Ifc_riscv_csrs csrs;
    interface Ifc_pipe_status pipe_status;
  `ifdef perfmonitors
    interface Ifc_riscv_perfmonitors perfmonitors;
  `endif
  `ifdef debug
    method Action ma_debug_interrupt(Bit#(1) _int);
    method Bit#(1) mv_core_is_reset;
    method Bit#(1) mv_core_debugenable;
    (*always_enabled*)
    method Action ma_debugger_available (Bit#(1) avail);
    method Bit#(1) mv_stop_timer;
    method Bit#(1) mv_stop_count;
  `endif
  `ifdef rtldump
    method Vector#(`num_issue, Maybe#(CommitLogPacket)) commitlog;
    interface Sbread sbread;
  `endif
  `ifdef simulate
    method Bit#(1) mv_simulate_log_start;
  `endif
  endinterface: Ifc_riscv

`ifdef riscv_noinline
  (*synthesize*)
`endif
module mkriscv#(Bit#(`vaddr) resetpc, parameter Bit#(`xlen) hartid)(Ifc_riscv);
    Ifc_stage0  stage0 <- mkstage0(resetpc, hartid);
    Ifc_stage1  stage1 <- mkstage1(hartid);
    Ifc_stage2  stage2 <- mkstage2(hartid);
    Ifc_stage3  stage3 <- mkstage3(hartid);
    Ifc_stage4  stage4 <- mkstage4(hartid);
    Ifc_stage5  stage5 <- mkstage5(hartid);
    
    /*doc:reg: This register stays high once the hart has been reset */
    Reg#(Bit#(1)) rg_reset_done <- mkReg(0);
    /*doc:reg: This register sends a single cycle pulse one the hart is out of reset*/
    Reg#(Bit#(1)) rg_reset_event <- mkDReg(0);
    /*doc:reg: This register is used to set the above registers after certain amount of clock cycles
     * have passed since the deassertion of the reset*/
    Reg#(Bit#(TAdd#(1,TLog#(`reset_cycles)))) rg_reset_cycle <- mkReg(0);

  `ifdef simulate
    Bit#(1) simulate_log_start = stage5.common.mv_simulate_log_start;
  `endif

    let wbflush = stage5.common.mv_flush;
    let {exeflush, exepc} = stage3.common.mv_flush;

`ifdef perfmonitors
    /*doc:wire: */
    Reg#(Bit#(`mhpm_eventcount)) rg_total_count <- mkReg(0);
  `ifdef icache
    Wire#(Bit#(5)) wr_icache_counters <- mkDWire(0);
  `endif
  `ifdef dcache
    Wire#(Bit#(13)) wr_dcache_counters <- mkDWire(0);
  `endif
  `ifdef supervisor
    /*doc:wire: */
    Wire#(Bit#(1)) wr_dtlb_counters <- mkDWire(0);
    Wire#(Bit#(1)) wr_itlb_counters <- mkDWire(0);
  `endif
    Bit#(1) lv_count_misprediction          = `ifdef bpu pack(exeflush && !wbflush.flush) `else 0 `endif ;
    Bit#(1) lv_count_exceptions             = stage5.perf.mv_count_exceptions;
    Bit#(1) lv_count_interrupts             = stage5.perf.mv_count_interrupts;
    Bit#(1) lv_count_microtraps             = stage5.perf.mv_count_microtraps;
    Bit#(1) lv_count_csrops                 = stage5.perf.mv_count_csrops;
    Bit#(1) lv_count_jumps                  = stage3.perfmonitors.mv_count_jumps;
    Bit#(1) lv_count_branches               = stage3.perfmonitors.mv_count_branches;
    Bit#(1) lv_count_floats                 = `ifdef spfpu stage3.perfmonitors.mv_count_floats `else 0 `endif ;
    Bit#(1) lv_count_muldiv                 = `ifdef muldiv stage3.perfmonitors.mv_count_muldiv `else 0 `endif ;
    Bit#(1) lv_count_rawstalls              = stage3.perfmonitors.mv_count_rawstalls;
    Bit#(1) lv_count_exestalls               = stage3.perfmonitors.mv_count_exestalls;
    Bit#(1) lv_count_icache_access          = `ifdef icache wr_icache_counters[4] `else 0 `endif ;
    Bit#(1) lv_count_icache_miss            = `ifdef icache wr_icache_counters[2] `else 0 `endif ;
    Bit#(1) lv_count_icache_fbhit           = `ifdef icache wr_icache_counters[0] `else 0 `endif ;
    Bit#(1) lv_count_icache_ncaccess        = `ifdef icache wr_icache_counters[3] `else 0 `endif ;
    Bit#(1) lv_count_icache_fbrelease       = `ifdef icache wr_icache_counters[1] `else 0 `endif ;
    Bit#(1) lv_count_dcache_read_access		  = `ifdef dcache wr_dcache_counters[12] `else 0 `endif ;
    Bit#(1) lv_count_dcache_write_access		= `ifdef dcache wr_dcache_counters[11] `else 0 `endif ;
    Bit#(1) lv_count_dcache_atomic_access		= `ifdef dcache wr_dcache_counters[10] `else 0 `endif ;
    Bit#(1) lv_count_dcache_nc_read_access	= `ifdef dcache wr_dcache_counters[9] `else 0 `endif ;
    Bit#(1) lv_count_dcache_nc_write_access = `ifdef dcache wr_dcache_counters[8] `else 0 `endif ;
    Bit#(1) lv_count_dcache_read_miss		    = `ifdef dcache wr_dcache_counters[7] `else 0 `endif ;
    Bit#(1) lv_count_dcache_write_miss		  = `ifdef dcache wr_dcache_counters[6] `else 0 `endif ;
    Bit#(1) lv_count_dcache_atomic_miss		  = `ifdef dcache wr_dcache_counters[5] `else 0 `endif ;
    Bit#(1) lv_count_dcache_read_fb_hits		= `ifdef dcache wr_dcache_counters[4] `else 0 `endif ;
    Bit#(1) lv_count_dcache_write_fb_hits		= `ifdef dcache wr_dcache_counters[3] `else 0 `endif ;
    Bit#(1) lv_count_dcache_atomic_fb_hits	= `ifdef dcache wr_dcache_counters[2] `else 0 `endif ;
    Bit#(1) lv_count_dcache_fb_releases		  = `ifdef dcache wr_dcache_counters[1] `else 0 `endif ;
    Bit#(1) lv_count_dcache_line_evictions	= `ifdef dcache wr_dcache_counters[0] `else 0 `endif ;
    Bit#(1) lv_count_itlb_misses            = `ifdef supervisor wr_itlb_counters `else 0 `endif ;
    Bit#(1) lv_count_dtlb_misses            = `ifdef supervisor wr_dtlb_counters `else 0 `endif ;
    Bit#(1) lv_count_instr_queue_full       = stage1.perf.mv_instr_queue_full;
    Bit#(1) lv_count_instr_queue_empty      = stage2.perf.mv_instr_queue_empty;
    Bit#(1) lv_dual_issued                  = stage2.perf.mv_dual_issued;
    Bit#(1) lv_raw_hazard                   = stage2.perf.mv_raw_hazard;
    Bit#(1) lv_one_instr                    = stage2.perf.mv_one_instr;
    Bit#(1) lv_mul_branch_hazard            = stage2.perf.mv_mul_branch_hazard;
    Bit#(1) lv_mul_mem_hazard               = stage2.perf.mv_mul_mem_hazard;
    Bit#(1) lv_mul_float_hazard             = stage2.perf.mv_mul_float_hazard;
    Bit#(1) lv_mul_mul_hazard               = stage2.perf.mv_mul_mul_hazard;
    Bit#(1) lv_mem_mem_hazard               = stage2.perf.mv_mem_mem_hazard;
    Bit#(1) lv_mem_branch_hazard            = stage2.perf.mv_mem_branch_hazard;
    Bit#(1) lv_mem_float_hazard             = stage2.perf.mv_mem_float_hazard;
    Bit#(1) lv_float_branch_hazard          = stage2.perf.mv_float_branch_hazard;
    Bit#(1) lv_float_float_hazard           = stage2.perf.mv_float_float_hazard;
    Bit#(1) lv_branch_branch_hazard         = stage2.perf.mv_branch_branch_hazard;
    Bit#(1) lv_count_isb3_isb4_full         = stage3.perfmonitors.mv_count_isb3_isb4_full;
    Bit#(1) lv_count_isb3_isb4_empty        = stage4.perf.mv_count_isb3_isb4_empty;
    Bit#(1) lv_count_isb4_isb5_full         = stage4.perf.mv_count_isb4_isb5_full;
    Bit#(1) lv_count_isb4_isb5_empty        = stage5.perf.mv_count_isb4_isb5_empty;

    let lv_total_count = reverseBits({lv_count_misprediction, lv_count_exceptions, lv_count_interrupts,
      lv_count_microtraps, lv_count_csrops, lv_count_jumps, lv_count_branches, lv_count_floats, lv_count_muldiv,
      lv_count_rawstalls, lv_count_exestalls, lv_count_icache_access, lv_count_icache_miss,
      lv_count_icache_fbhit, lv_count_icache_ncaccess, lv_count_icache_fbrelease,
      lv_count_dcache_read_access		, lv_count_dcache_write_access		,
      lv_count_dcache_atomic_access		, lv_count_dcache_nc_read_access		,
      lv_count_dcache_nc_write_access, lv_count_dcache_read_miss		, lv_count_dcache_write_miss,
      lv_count_dcache_atomic_miss		, lv_count_dcache_read_fb_hits		,
      lv_count_dcache_write_fb_hits		, lv_count_dcache_atomic_fb_hits		,
      lv_count_dcache_fb_releases		, lv_count_dcache_line_evictions		, lv_count_itlb_misses,
      lv_count_dtlb_misses, lv_count_instr_queue_full, lv_count_instr_queue_empty, lv_dual_issued, lv_raw_hazard, lv_one_instr,
      lv_count_isb3_isb4_full,
      lv_count_isb3_isb4_empty,
      lv_count_isb4_isb5_full, 
      lv_count_isb4_isb5_empty,
      lv_mul_branch_hazard,
      lv_mul_mem_hazard,
      lv_mul_float_hazard,
      lv_mul_mul_hazard,      
      lv_mem_mem_hazard,
      lv_mem_branch_hazard,
      lv_mem_float_hazard,
      lv_float_branch_hazard,
      lv_float_float_hazard,
      lv_branch_branch_hazard
    });
`endif

  `ifdef muldiv
    Ifc_mbox mbox <- mkmbox(0);
    FIFOF#(Bit#(`xlen)) ff_mbox_out <- mkSizedBypassFIFOF(`isb_s3s4 );
    `ifdef arith_trap
      FIFOF#(Tuple2#(Bool, Bit#(`causesize))) ff_mbox_arith_trap_out <- mkSizedBypassFIFOF(`isb_s3s4 );
    `endif
  `endif
  `ifdef debug
    /*doc:wire: */
    Wire#(Bit#(1)) wr_debugger_available <- mkWire();
  `endif
  `ifdef spfpu
    Ifc_fpu fbox <- mkfpu();
    FIFOF#(XBoxOutput) ff_fbox_out <- mkSizedBypassFIFOF(`isb_s3s4);
  `endif

    let {pipe_s4s5_notEmpty, lv_bypass_1} <- mkPipe_s4_s5(stage4.tx, stage5.rx);
    let {pipe_s3s4_notEmpty, lv_bypass_0} <- mkPipe_s3_s4(stage3.tx, stage4.rx);
    let pipe_s2s3_notEmpty                <- mkPipe_s2_s3(stage2.tx, stage3.rx);
    let {pipe_s1s2_notEmpty, pipe1}       <- mkPipe_s1_s2(stage1.tx, stage2.rx);
    let pipe_s0s1_notEmpty                <- mkPipe_s0_s1(stage0.tx, stage1.rx);

    Vector#(5, Bool) lv_isb_empty;
    lv_isb_empty[0] = !pipe_s0s1_notEmpty;
    lv_isb_empty[1] = !pipe_s1s2_notEmpty;
    lv_isb_empty[2] = !pipe_s2s3_notEmpty;
    lv_isb_empty[3] = !pipe_s3s4_notEmpty;
    lv_isb_empty[4] = !pipe_s4s5_notEmpty;

    Vector#(`bypass_sources, Vector#(`num_issue, FwdType)) lv_bypass;
    lv_bypass[0] = lv_bypass_0;
    lv_bypass[1] = lv_bypass_1;


    mkConnection(stage1.common.ma_csr_misa_c, stage5.csrs.mv_csr_misa_c);

    mkConnection(stage2.common.ma_commit_rd, stage5.common.mv_commit_rd);
    mkConnection(stage3.common.ma_sb_release,stage5.common.mv_commit_rd);
    mkConnection(stage2.common.ma_csrs , stage5.csrs.mv_csrs_to_decode);
    mkConnection(stage2.common.ma_resume_wfi, stage5.csrs.mv_resume_wfi);
    mkConnection(stage2.rf, stage3.rf);

    mkConnection(stage3.common.ma_csr_misa_c, stage5.csrs.mv_csr_misa_c);
    mkConnection(stage3.bypass.ma_bypass, lv_bypass);
    mkConnection(stage3.common.ma_priv, stage5.csrs.mv_curr_priv);
    mkConnection(stage3.common.ma_mstatus, stage5.csrs.mv_csr_mstatus);
  `ifdef hypervisor
    mkConnection(stage3.common.ma_vs_mode, stage5.csrs.mv_vs_bit);
    mkConnection(stage3.common.ma_hstatus, stage5.csrs.mv_csr_hstatus);
  `endif
  `ifdef bpu
    rule rl_connect_next_pc_to_stage3(pipe_s1s2_notEmpty);                                        
       stage3.bpu.ma_next_pc(pipe1.first[0].program_counter);                                      
    endrule
    //mkConnection(stage3.bpu.ma_next_pc, pipe1.first[0].program_counter);
    mkConnection(stage0.s0_bpu.ma_train_bpu, stage3.bpu.mv_train_bpu);
  `ifdef gshare
    mkConnection(stage0.s0_bpu.ma_mispredict, stage3.bpu.mv_mispredict);
  `endif 

  `endif
  `ifdef muldiv
    `ifdef arith_trap
      rule rl_mbox_arith_en;
        mbox.ma_arith_trap_en(stage5.csrs.mv_cacheenable[3]);
      endrule
      mkConnection(mbox.tx_arith_trap_output, ff_mbox_arith_trap_out);
      mkConnection(ff_mbox_arith_trap_out, stage4.s4_mbox.rx_mbox_arith_trap_output);
    `endif
    mkConnection(stage3.muldiv.ma_mbox_ready,mbox.mv_ready);
    mkConnection(stage3.muldiv.mv_mbox_inputs, mbox.ma_inputs);
    mkConnection(mbox.tx_output, ff_mbox_out);
    mkConnection(ff_mbox_out, stage4.s4_mbox.rx_mbox_output);
  `endif
  `ifdef spfpu
    mkConnection(stage3.float.ma_fbox_ready,fbox.fpu_ready);
    mkConnection(stage3.float.mv_fbox_inputs, fbox._start);
    mkConnection(fbox.tx_output, ff_fbox_out);
    mkConnection(ff_fbox_out, stage4.s4_fbox.rx_fbox_output);
    `ifdef arith_trap
      rule rl_fbox_arith_en;
        fbox.rd_arith_excep_en(unpack(stage5.csrs.mv_cacheenable[3]));
      endrule
    `endif
  `endif
  `ifdef bpu
    /*doc:rule: */
    rule rl_connect_bpu_enable;
      stage0.s0_bpu.ma_bpu_enable(unpack(stage5.csrs.mv_cacheenable[2]));
    endrule:rl_connect_bpu_enable
  `endif
  `ifdef simulate
    rule rl_upd_log_start;
      fbox.ma_simulate_log_start(simulate_log_start);
      mbox.ma_simulate_log_start(simulate_log_start);
      stage0.common.ma_simulate_log_start(simulate_log_start);
      stage1.common.ma_simulate_log_start(simulate_log_start);
      stage2.common.ma_simulate_log_start(simulate_log_start);
      stage3.common.ma_simulate_log_start(simulate_log_start);
      stage4.rx.ma_simulate_log_start(simulate_log_start);
    endrule
  `endif
  `ifdef perfmonitors
    
    rule rl_connect_events;
      rg_total_count <= lv_total_count;
    endrule:rl_connect_events

    rule rl_connect_events1;
      stage5.perf.ma_events(rg_total_count);
    endrule
  `endif
    
    rule rl_flush_stage0(exeflush || wbflush.flush);
        stage0.common.ma_flush(Stage0Flush{ pc : wbflush.flush ? wbflush.newpc : exepc
                                `ifdef ifence
                                  ,fence : wbflush.fencei
                                `endif
															 	`ifdef hypervisor
                                  , hfence :  wbflush.hfence
                                `endif 
                                `ifdef supervisor
                                  , sfence :  wbflush.sfence
                                `endif });
    endrule:rl_flush_stage0
    /*doc:rule: */
    rule rl_assert_reset_done;
      if (rg_reset_cycle == `reset_cycles) begin
        rg_reset_done <= 1;
        rg_reset_event <= 1;
        rg_reset_cycle <= rg_reset_cycle + 1;
        `logLevel( riscv, 0, $format("[%2d]RISCV: Hart is out of reset sequence",hartid), simulate_log_start)
      end                                                                       
      else if (rg_reset_cycle < `reset_cycles) 
        rg_reset_cycle <= rg_reset_cycle + 1;
      stage0.common.ma_reset_done(unpack(rg_reset_done));
    endrule:rl_assert_reset_done

    rule rl_clear_stall_in_decode_stage(exeflush || wbflush.flush);
      stage2.common.ma_clear_stall(True);
    endrule:rl_clear_stall_in_decode_stage
    /*doc:rule: This is fired when execute stage generates a flush*/
    rule rl_update_eEpoch(exeflush);
      stage0.common.ma_update_eEpoch();
      stage1.common.ma_update_eEpoch();
      stage2.common.ma_update_eEpoch();
    endrule:rl_update_eEpoch

    /*doc:rule: This rule is fired when the write-back stage generates a flush */
    rule rl_update_wEpoch(wbflush.flush);
      stage0.common.ma_update_wEpoch();
      stage1.common.ma_update_wEpoch();
      stage2.common.ma_update_wEpoch();
      stage3.common.ma_update_wEpoch();
    endrule:rl_update_wEpoch

  `ifdef debug
    rule rl_connect_debug_decode;
      stage2.debug.debug_status(DebugStatus {debugger_available : wr_debugger_available==1 ,
                                       debug_mode: unpack(stage5.debug.mv_debug_mode),
                                       step_set           : unpack(stage5.debug.mv_csr_dcsr[2]),
                                       step_ie            : unpack(stage5.debug.mv_csr_dcsr[11]),
                                       core_debugenable   : unpack(stage5.debug.mv_core_debugenable)} );
    endrule
  `endif

    interface s0_icache = stage0.icache;
    interface s1_icache = stage1.icache;
    
    interface s3_cache = stage3.cache;

    interface s4_cache = stage4.cache;
    interface interrupts = stage5.interrupts;
    interface s5_cache = stage5.cache;
    interface csrs = interface Ifc_riscv_csrs
      method mv_csr_mstatus = stage5.csrs.mv_csr_mstatus;
      method mv_cacheenable = stage5.csrs.mv_cacheenable;
      method mv_curr_priv = stage5.csrs.mv_curr_priv;
  	`ifdef supervisor
  		method mv_csr_satp = stage5.csrs.mv_csr_satp;
  	`endif
    `ifdef pmp
      method mv_pmp_cfg = stage5.csrs.mv_pmp_cfg;
      method mv_pmp_addr = stage5.csrs.mv_pmp_addr;
    `endif
		`ifdef hypervisor
		  method  mv_csr_vsatp = stage5.csrs.mv_csr_vsatp;
  		method  mv_csr_hgatp = stage5.csrs.mv_csr_hgatp;
  		method  mv_vs_bit = stage5.csrs.mv_vs_bit;
	    method  mv_csr_hstatus  = stage5.csrs.mv_csr_hstatus;
	    method  mv_csr_vsstatus = stage5.csrs.mv_csr_vsstatus;				
			`ifdef RV32
				method mv_csr_mstatush = stage5.csrs.mv_csr_mstatush;
 			`endif
		`endif	
    endinterface;
    interface pipe_status = interface Ifc_pipe_status
      method mv_pipe_isbs_empty = lv_isb_empty;
      method mv_wfi_detected = stage2.mv_wfi_detected;
    endinterface;
  `ifdef debug
    method ma_debug_interrupt= stage5.debug.ma_debug_interrupt;
    method mv_core_debugenable = stage5.debug.mv_core_debugenable;
    method mv_core_is_reset = rg_reset_event;
    method Action ma_debugger_available (Bit#(1) avail);
      wr_debugger_available <= avail;
    endmethod:ma_debugger_available
    method Bit#(1) mv_stop_timer = stage5.debug.mv_stop_timer;
    method Bit#(1) mv_stop_count = stage5.debug.mv_stop_count;
  `endif
  `ifdef rtldump
    method commitlog = stage5.common.mv_commit_log;
    interface sbread = stage5.csrs.sbread;
  `endif
  `ifdef simulate
    method mv_simulate_log_start = simulate_log_start;
  `endif
  `ifdef perfmonitors
    interface perfmonitors = interface Ifc_riscv_perfmonitors
    `ifdef icache
      /*doc:method: */
      method Action ma_icache_counters (Bit#(5) i);
        wr_icache_counters <= i;
      endmethod
    `endif
    `ifdef dcache
      /*doc:method: */
      method Action ma_dcache_counters (Bit#(13) i);
        wr_dcache_counters <= i;
      endmethod
    `endif
    `ifdef supervisor
      method Action ma_dtlb_counters (Bit#(1) i);
        wr_dtlb_counters <= i;
      endmethod
      method Action ma_itlb_counters (Bit#(1) i);
        wr_itlb_counters <= i;
      endmethod
    `endif
    endinterface;
  `endif
  endmodule: mkriscv
endpackage: riscv 

