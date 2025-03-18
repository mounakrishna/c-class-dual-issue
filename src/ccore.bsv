//See LICENSE.iitm for license details
/*

Author : IIT Madras
Details:

--------------------------------------------------------------------------------------------------
*/
package ccore;

//=================== Interface and module for a ccore - master on the AXI4 fabric ============= //
// project related imports
import Semi_FIFOF:: *;
import AXI4_Types:: *;
import AXI4_Fabric:: *;
//import riscv:: * ;
import Vector :: *;
import riscv :: *;
import ccore_types:: * ;
import FIFOF::*;
import dcache_types :: *;
import icache_types :: * ;
import Assert ::*;
import imem::*;
import dmem::*;
import pipe_ifcs :: * ;
`ifdef hypervisor
  import ptwalk_hypervisor :: * ;
`elsif supervisor
  import ptwalk_merged::*;
`endif
`include "ccore_params.defines"
`include "Logger.bsv"

`define Mem_master_num 0

// package imports
import Connectable 				:: *;
import GetPut:: *;
import BUtils::*;
import csrbox :: * ;

`ifdef debug
import debug_types  :: * ;
import csr_types    :: * ;
`endif

`ifdef supervisor
typedef enum {None, IWalk, DWalk} PTWState deriving(Bits, Eq, FShow);
`endif

interface Ifc_ccore_axi4;
	interface AXI4_Master_IFC#(`paddr, `elen, USERSPACE) master_d;
	interface AXI4_Master_IFC#(`paddr, `elen, USERSPACE) master_i;
    interface Put#(Bit#(1)) sb_clint_msip;

	/*doc:method: This method should receive the machine timer interrupt from the CLINT module*/
  interface Put#(Bit#(1)) sb_clint_mtip;

	/*doc:method: This method should receive the machine timer value from the CLINT module. This will
	* be used for the pseduo op rdtime instruction in the user mode*/
  interface Put#(Bit#(64)) sb_clint_mtime;

  /*doc:method: This method should receive the machine external interrupt from the PLIC module*/
	method Action sb_plic_meip(Bit#(1) ex_i);

`ifdef supervisor
  /*doc:method: When supervisor is enabled, this method will receive the supervisor external
   * interrupt from the PLIC. Note this is a different context from the machine external interrupt*/
	method Action sb_plic_seip(Bit#(1) ex_i);
`endif
`ifdef usertraps
  /*doc:method: When user traps (N-extesion) is enabled, this method will receive the user external
   * interrupt from the PLIC. Note this is a different context from the machine,supervisor 
   * external interrupt*/
	method Action sb_plic_ueip(Bit#(1) ex_i);
`endif
`ifdef rtldump
  /*doc:sbifc: This interface is available only for simulation when intruction trace dump has been 
  enabled. This method is used to read the value of the csrs in the next cycle after csr-ops are
  done. This allows the TB to dump the correct value that is written to the csr.*/
  interface Sbread sbread;
  /*doc:method: this method carries the trace dump information of the instruction that was recently
   * committed*/
  method Vector#(`num_issue, Maybe#(CommitLogPacket)) commitlog;
`endif
`ifdef simulate
  method Bit#(1) mv_simulate_log_start;
`endif
`ifdef debug
  /*doc:method: This method is used to capture the interrupt from the debugger in case of resume or
   * halt indication*/
  method Action ma_debug_interrupt(Bit#(1) _int);
  /*doc:method: This method indicates if the core has been reset successfully*/
  method Bit#(1) mv_core_is_reset;

  /*doc:method: This method indicates to the debugger is the core is available for debugging*/
  method Bit#(1) mv_core_debugenable;

  (*always_enabled*)
  /*doc:method: This action method indicates the core that a debugger is connected and available for
   * carrying our debug operations*/
  method Action ma_debugger_available (Bit#(1) avail);
  /*doc:method: This method holds the bits from the dcsr register which indicate that the timer in
   * CLINT should stop incrementing when in debug mode*/
  method Bit#(1) mv_stop_timer;
  /*doc:method: This method holds the bits from the dcsr register which indicate that the
   * mhpmcounters in the csrs should stop incrementing when in debug mode*/
  method Bit#(1) mv_stop_count;
`endif
endinterface : Ifc_ccore_axi4

(*synthesize*)
`ifdef supervisor
(*preempts="rl_dtlb_req_to_ptwalk, rl_itlb_req_to_ptwalk"*)
  (*preempts="core_req_mkConnectionGetPut, ptwalk_req_mkConnectionGetPut"*)
`endif

`ifdef itim
  (*conflict_free="handle_itim_write_resp, handle_nc_write_resp"*)
`endif
(*mutually_exclusive ="rl_handle_io_read_response, rl_handle_io_write_resp"*)
module mkccore_axi4#(Bit#(`vaddr) resetpc, parameter Bit#(`xlen) hartid)(Ifc_ccore_axi4);
  String core = "";
  /*doc:mod: instatiate the riscv pipeline */
  Ifc_riscv riscv <- mkriscv(resetpc, hartid);

    Reg#(PTWState) rg_ptw_state <- mkReg(None);
    `ifdef hypervisor
    Ifc_ptwalk ptwalk <- mkptwalk;
  `elsif supervisor
    Ifc_ptwalk#(`asidwidth) ptwalk <- mkptwalk;
  `endif

	AXI4_Master_Xactor_IFC #(`paddr, `elen, USERSPACE) fetch_xactor <- mkAXI4_Master_Xactor;
	AXI4_Master_Xactor_IFC #(`paddr, `elen, USERSPACE) memory_xactor <- mkAXI4_Master_Xactor;
`ifdef pmp
  /*doc:var: When pmp is enabled we capture the curernt pmp configurations and addresses that will
  * be required by the TLBs*/
  let lv_pmp_cfg = riscv.csrs.mv_pmp_cfg;
  let lv_pmp_adr = riscv.csrs.mv_pmp_addr;
`endif

  /*doc:mod: instantiate the instruction memory subsystem*/
	Ifc_imem imem <- mkimem(truncate(hartid) `ifdef pmp ,lv_pmp_cfg, lv_pmp_adr `endif );

  /*doc:mod: instantiate the data memory subsystem*/
	Ifc_dmem dmem <- mkdmem(truncate(hartid) `ifdef pmp ,lv_pmp_cfg, lv_pmp_adr `endif );

`ifdef simulate
  let simulate_log_start = riscv.mv_simulate_log_start;
`endif

`ifdef dcache
  /*doc:reg: This register is used to keep track of the beats/bursts occurring during the write
  * operation of a data line to memory. A new line-write or an io-write operation can be initiated
  * only when rg_burst_count == 0, else the requests are stalled. */
  Reg#(Bit#(8)) rg_burst_count <- mkReg(0);

  /*doc:reg: While performing burst writes during line eviction, this register indicates the amount
   * the line should be shifted to send the next beat of data on the bus*/
  Reg#(Bit#(TLog#(TMul#(TMul#(`dwords, 8), `dblocks)))) rg_shift_amount <- mkReg(`dwords * 8 );
`endif
  /*doc: capture the current privilege mode under which the current transaction is being carried
  * out. TODO: This should ideally come form the caches themselves. Capturing here can have issues.
  * For example during an eviction, if the prv changes in the write-back stage then stores of the same
  * line may occur in different privilege modes*/
  let curr_priv = riscv.csrs.mv_curr_priv;

	/*doc:connect: Connect the instruction request from the core to the instruction memory subsystem*/
	mkConnection(imem.put_core_req , riscv.s0_icache.to_icache);

  /*doc:connect: Connect the instruction memoty subsystem's response to the pipeline's stage-1 which
   * will accept it and process*/
	mkConnection(imem.get_core_resp, riscv.s1_icache.inst_response); // imem integration

  /*doc:connect: Send the signal from the data memory subsystem indicating that it is available to
   * receive new instructions from the stage3. If unavailable, stage3 should stall on a possible
   * memory operation.*/
	mkConnection(dmem.mv_dmem_available, riscv.s3_cache.ma_cache_is_available);

  /*doc:connect: Connect the load/store request happening in the stage3 of the pipeline to the
   * request port of the data memory subsystem*/
  let core_req <- mkConnection(dmem.receive_core_req, riscv.s3_cache.mv_memory_request);

  /*doc:connect: Connect the data memory subsystem's response to stage4 of the pipeline*/
	let core_resp <- mkConnection(dmem.send_core_cache_resp, riscv.s4_cache.memory_response); // dmem integration

`ifdef simulate
  rule rl_upd_log_start;
    ptwalk.ma_simulate_log_start(simulate_log_start);
  endrule
`endif
  
  /*doc:rule: This rule sends out requests from the I-cache to the fabric. This rule will only fire
  * when there has been a line-miss in the instructino cache or the request is an io operation. This
  * rule also will not fire if the fetch-xactor fifo is full - which can happen due to contention in
  * the fabric*/
  rule rl_handle_imem_line_request;
		let request <- imem.get_read_mem_req.get;
		AXI4_Rd_Addr#(`paddr, 0) imem_request = AXI4_Rd_Addr {araddr : truncate(request.address),
      aruser: ?, arlen : request.burst_len, arsize : request.burst_size, arburst : 'b10,
      arid : zeroExtend(pack(request.io)), arprot:{1'b1, 1'b0, curr_priv[1]} }; // arburst : 00 - FIXED 01 - INCR 10 - WRAP
	  fetch_xactor.i_rd_addr.enq(imem_request);
		`logLevel( core, 1, $format("[%2d]CORE : IMEM Line Requesting ",hartid, fshow(imem_request)), simulate_log_start)
  endrule:rl_handle_imem_line_request

  /*doc:rule: This rule captures the response from the fetch transactor of the fabric and routes
   * the response back to the instruction memory subsystem. This rule will fire only when the
   * fetch-xactor has a response available. The instruction memory is implicitly assumed to be always
   * available to pick the responses owing to its blocking nature*/
	rule rl_handle_imem_line_resp;
	  let fab_resp <- pop_o (fetch_xactor.o_rd_data);
		Bool bus_error = !(fab_resp.rresp == AXI4_OKAY);
    imem.put_read_mem_resp.put(ICache_mem_readresp{data   : truncate(fab_resp.rdata),
                                               last   : fab_resp.rlast,
                                               err    : bus_error});
		`logLevel( core, 1, $format("[%2d]CORE : IMEM Line Response ",hartid, fshow(fab_resp)), simulate_log_start)
	endrule:rl_handle_imem_line_resp

`ifdef icache
  /*doc:rule: This rule is used connect the cache enable signal for the instruction memory
   * subsystem*/
  rule rl_imem_enable;
	  imem.ma_cache_enable(unpack(riscv.csrs.mv_cacheenable[0]));
  endrule: rl_imem_enable
`endif

`ifdef dtim
  /*doc:rule: */
  rule rl_connect_dtim_memorymap_csrs;
    dmem.ma_dtim_memory_map(truncate(riscv.mv_csr_dtim_base), truncate(riscv.mv_csr_dtim_bound));
  endrule
`endif
`ifdef itim
  /*doc:rule: */
  rule rl_connect_itim_memorymap_csrs;
    imem.ma_itim_memory_map(truncate(riscv.mv_csr_itim_base), truncate(riscv.mv_csr_itim_bound));
  endrule
`endif

	/*doc:rule: This rule will initiate an IO read or write as indicated by the WB stage of the
	* pipeline. If a burst write is on-going then this rule is stalled. Since AXI4 support narrow
  * transfers (transfers whose size is lesser than the size of the bus), for write operations 
  * we make sure to duplicate
  * the data accordingly so that respective bytes of the lane contain the correct data. This is easier
  * than to shift the data.*/
  rule rl_initiate_io `ifdef dcache (rg_burst_count == 0) `endif ;
	  // receive the request from the data memory subsystem
	  let req <- dmem.send_mem_io_req.get;
    `logLevel( core, 0, $format("CORE: Received io op: ",fshow(req)), simulate_log_start)

    // resize the data and duplicate the bytes based on the size of the transaction
    if(req.size[1:0]== 0)
      req.data = duplicate(req.data[7 : 0]);
    else if(req.size[1:0] == 1)
      req.data = duplicate(req.data[15 : 0]);
    else if(req.size[1:0] == 2)
      req.data = duplicate(req.data[31 : 0]);

    // build the write-strobe based on the size of the request.
    Bit#(TDiv#(`elen, 8)) write_strobe = req.size[1:0] == 0?'b1 :
                                        req.size[1:0] == 1?'b11 :
                                        req.size[1:0] == 2?'hf : '1;
    Bit#(TAdd#(1, TDiv#(`elen, 32))) byte_offset = truncate(req.address);
    write_strobe = write_strobe<<byte_offset;

    // if read operation send transaction on the i_rd_addr channel
	  if (!req.read_write) begin
		  AXI4_Rd_Addr#(`paddr, 0) dmem_request = AXI4_Rd_Addr {araddr : truncate(req.address), aruser: ?,
        arlen : 0, arsize : zeroExtend(req.size[1:0]), arburst : 'b00, // arburst : 00 - FIXED 01 - INCR 10 - WRAP
        arid : 1 ,arprot:{1'b0, 1'b0, curr_priv[1]} }; 
      memory_xactor.i_rd_addr.enq(dmem_request);
	  end
	  // if write operation send transaction on the i_wr_addr and i_wr_data channels
	  else begin
	    AXI4_Wr_Addr#(`paddr, 0) aw = AXI4_Wr_Addr {awaddr : truncate(req.address), awuser : 0,
        awlen : 0, awsize : zeroExtend(req.size[1 : 0]), awburst : 'b0,
        awid : 1, awprot:{1'b0, 1'b0, curr_priv[1]} }; // arburst : 00 - FIXED 01 - INCR 10 - WRAP

      let w  = AXI4_Wr_Data {wdata : truncate(req.data), wstrb : write_strobe,
                             wlast : True, 
                             wid : 1};
	    memory_xactor.i_wr_addr.enq(aw);
	    memory_xactor.i_wr_data.enq(w);
	  end
	endrule:rl_initiate_io

   /*doc:rule: This rule will receive the response generated for a previous data memory subsystem IO
  * read transaction. Because of the in-order nature of the core and that all IO transactions in the
  * WB stage can only be initiated when the previous one is over, we do not need to handle the
  * scenario where the responses can be out of order.*/
  rule rl_handle_io_read_response(memory_xactor.o_rd_data.first.rid == 1);
    let response <- pop_o(memory_xactor.o_rd_data);
  	let bus_error = !(response.rresp == AXI4_OKAY);
    dmem.receive_mem_io_resp.put(DCache_io_response{data:response.rdata, 
                                              error:bus_error});
    `logLevel( core, 1, $format("[%2d]CORE : IO Read Response ",hartid, fshow(response)), simulate_log_start)
  endrule:rl_handle_io_read_response


  /*doc:rule: This rule will receive the response generated for a previous data memory subsystem IO
  * write transaction. Because of the in-order nature of the core and that all IO transactions in the
  * WB stage can only be initiated when the previous one is over, we do not need to handle the
  * scenario where the responses can be out of order.*/
  rule rl_handle_io_write_resp (memory_xactor.o_wr_resp.first.bid == 1);
    let response <- pop_o(memory_xactor.o_wr_resp);
  	let bus_error = !(response.bresp == AXI4_OKAY);
    dmem.receive_mem_io_resp.put(DCache_io_response{data: ?, 
                                              error:bus_error});
    `logLevel( core, 1, $format("[%2d]CORE : IO Write Response ",hartid, fshow(response)), simulate_log_start)
  endrule:rl_handle_io_write_resp

`ifdef dcache

  /*doc:reg: This register when Valid, indicaets that there is pending read line operation which got
   * delayed because of a contentious write happening on the same line address*/
  Reg#(Maybe#(AXI4_Rd_Addr#(`paddr, 0))) rg_read_line_req <- mkReg(tagged Invalid);

  /*doc:reg: This register when Valid, indicates the address to which a line a request is in
   * progress. This is used to delay a read operation on the same line initiated by the cache*/
  Reg#(Maybe#(Bit#(`paddr))) wr_write_req <- mkReg(tagged Invalid);
  
  /*doc:rule: This rule is used connect the cache enable signal for the data memory
   * subsystem*/
  rule rl_map_dmem_enable;
	  dmem.ma_cache_enable(unpack(riscv.csrs.mv_cacheenable[1]));
  endrule:rl_map_dmem_enable

  /*doc:connect: This connects the store commit routine from the WB stage of the pipeline to the
   * commit store method of the data memory subsystem*/
  mkConnection(dmem.ma_commit_store, riscv.s5_cache.mv_initiate_store);

  /*doc:connect: This connects the IO commit routine from the WB stage of the pipeline to the
   * commit io method of the data memory subsystem*/
  mkConnection(dmem.ma_commit_io, riscv.s5_cache.mv_initiate_ioop);

  /*doc:connect: This connects the io response from the data memory subsystem to the WB stage of the
   * pipeline */
  mkConnection(dmem.send_core_io_resp, riscv.s5_cache.ma_io_response);

  /*doc:rule: Currently it is possible that the data cache can generate a write - request followed by a
  read - request, but the fabric (due to contention) latches the read first to the slave followed
  by the write - req. This could lead to wrong behavior. To avoid this it is necessary to ensure
  that if a write - request has been initiated no read - requests should be latched unless the
  write - response has arrived.
  The contraint is fullilled using the register wr_write_req which holds the current address of
  the line being written to the fabric on a eviction. When such a conflict is detected we store the
  popped request from the data memory subsystem into the rg_read_line_req register so that it can be
  handled once the conflict is done.
  */
  rule rl_handle_dmem_line_read_request(rg_read_line_req matches tagged Invalid );
    Bool perform_req = True;
  	let req <- dmem.send_mem_rd_req.get;
  	AXI4_Rd_Addr#(`paddr, 0) dmem_request = AXI4_Rd_Addr {araddr : truncate(req.address), aruser: ?,
      arlen : req.burst_len, arsize : req.burst_size, arburst : 'b10, // arburst : 00 - FIXED 01 - INCR 10 - WRAP
      arid : 0 ,arprot:{1'b0, 1'b0, curr_priv[1]} }; 
    if(wr_write_req matches tagged Valid .waddr) begin
      if((waddr>>(`dwords + `dblocks )) == (req.address>>(`dwords + `dblocks ) ))begin
        perform_req = False;
        rg_read_line_req <= tagged Valid dmem_request;
        `logLevel( core, 1, $format("[%2d]CORE: Delaying Request: ",hartid,fshow(req)), simulate_log_start)
      end
    end
    if(perform_req)  begin
 	    memory_xactor.i_rd_addr.enq(dmem_request);
      `logLevel( core, 1, $format("[%2d]CORE : DMEM Line Requesting ",hartid, fshow(dmem_request)), simulate_log_start)
    end
	endrule:rl_handle_dmem_line_read_request

  /*doc:rule: This rule will fire when a pending read operation is present due to a RAW conflict on
   * the line address*/
  rule rl_handle_delayed_read(rg_read_line_req matches tagged Valid .r &&& 
                                  wr_write_req matches tagged Invalid );
	  memory_xactor.i_rd_addr.enq(r);
    `logLevel( core, 1, $format("[%2d]CORE : DMEM Delayed Line Requesting ",hartid, fshow(r)), simulate_log_start)
    rg_read_line_req <= tagged Invalid;
  endrule:rl_handle_delayed_read

	rule rl_handle_dmem_line_resp(memory_xactor.o_rd_data.first.rid == 0);
    let fab_resp <- pop_o (memory_xactor.o_rd_data);
		let lv_data= fab_resp.rdata;
  	Bool bus_error = !(fab_resp.rresp == AXI4_OKAY);
    dmem.receive_mem_rd_resp.put(DCache_mem_readresp{data:truncate(lv_data),
                                               last:fab_resp.rlast,
                                               err :bus_error});
    `logLevel( core, 1, $format("[%2d]CORE : DMEM Line Response ",hartid, fshow(fab_resp)), simulate_log_start)
  endrule:rl_handle_dmem_line_resp

  /*doc:rule: This rule is fired when the data memory subsytem is requesting to write an entire line
   * back to memory. This rule will only fire if both the write address channel and the write data
  * channel are capable of taking in new transactions. In this rule we will enqueue the first beat
  * of line. Once sent, the rg_burst_count count is incremented preventing this rule from firing in
  * the susequent cycles untill all the beats of the line has been delivered.*/
  rule rl_handle_dmem_write_request (rg_burst_count == 0);
    // pop the request from the data cache.
    let req = dmem.send_mem_wr_req;

    // write strobe will always be all-ones.
	  Bit#(TDiv#(`elen, 8)) write_strobe = '1;

  	// increment burst count to indicate one beat has been sent.
      rg_burst_count <= rg_burst_count + 1;

	  AXI4_Wr_Addr#(`paddr, 0) aw = AXI4_Wr_Addr {awaddr : truncate(req.address), awuser : 0,
      awlen : req.burst_len, awsize : zeroExtend(req.burst_size[1 : 0]), awburst : 'b01,
      awid : 0, awprot:{1'b0, 1'b0, curr_priv[1]} }; // arburst : 00 - FIXED 01 - INCR 10 - WRAP

	  let w  = AXI4_Wr_Data {wdata : truncate(req.data), wstrb : write_strobe,
                           wlast : req.burst_len == 0, 
                           wid : 0};
    memory_xactor.i_wr_addr.enq(aw);
	  memory_xactor.i_wr_data.enq(w);
    `logLevel( core, 1, $format("[%2d]CORE : DMEM Line Write Addr : Request ",hartid, fshow(aw)), simulate_log_start)
    if(req.burst_len != 0 )
      wr_write_req <= tagged Valid req.address;
  endrule:rl_handle_dmem_write_request

  /*doc:rule: This rule sends the burst beats of the line write operation. On each iteration the
  * line is read from the eviction fifo of the data memory subystem and shifted by an appropriate
  * amount indicating the beat number. On the last beat, the eviction fifo is dequence and the burst
  * counter is reset to zero. We also invalidate the wr_write_req register on the last beat*/
  rule rl_dmem_burst_write_data(rg_burst_count != 0);
    // last beat is detected if the burst_counter has reached the size of the words in each line -1.
    Bool last = rg_burst_count == fromInteger(`dblocks - 1 );

    // read the eviction fifo
    let req = dmem.send_mem_wr_req;

    // shift line by the relevant the amount
    req.data = req.data >> rg_shift_amount;
	  let w  = AXI4_Wr_Data {wdata : truncate(req.data), wstrb : '1, wlast : last,
                           wid : 0};
    Bit#(TAdd#(TAdd#(TLog#(`dwords), 1), 3)) shift = {`dwords, 3'b0};

    // if last reset all state and exit this loop
    if(last) begin
      rg_burst_count <= 0;
      rg_shift_amount <= (`dwords * 8);
      wr_write_req <= tagged Invalid;
      dmem.deq_mem_wr_req;
    end
    else begin
      // generate the next shift amount and increment rg_burst_count counter.
      rg_shift_amount <= rg_shift_amount + (`dwords * 8);
      rg_burst_count <= rg_burst_count + 1;
    end
	  memory_xactor.i_wr_data.enq(w);
    `logLevel( core, 1, $format("[%2d]CORE : DMEM Write Data: %h rg_burst_count: %d last: %b \
_shift_amount:%d",hartid, req.data, rg_burst_count, last, rg_shift_amount), simulate_log_start)
  endrule:rl_dmem_burst_write_data

  /*doc:rule: This rule will simply forward the response obtained from the fabric for a previous
   * line write operation to the data memory subsystem*/
  rule handle_dmem_line_write_resp (memory_xactor.o_wr_resp.first.bid == 0);
    let response <- pop_o(memory_xactor.o_wr_resp);
  	let bus_error = !(response.bresp == AXI4_OKAY);
	  dmem.receive_mem_wr_resp.put(bus_error);
    `logLevel( core, 1, $format("[%2d]CORE : DMEM Write Line Response ",hartid, fshow(response)), simulate_log_start)
  endrule: handle_dmem_line_write_resp

`ifdef itim 
  rule handle_dmem_itim_read_response;
  	let response <- imem.get_mem_read_itim_resp.get;
  	Bool bus_error = response.err;
    dmem.put_nc_read_resp.put(DCache_mem_readresp{data:zeroExtend(response.data),
                                              last:True,
                                              err :bus_error});
    `logLevel( core, 1, $format("[%2d]CORE : DMEM ITIM Response ",hartid, fshow(response)), simulate_log_start)
  endrule
  rule handle_itim_write_resp;
  	let response <- imem.get_mem_write_itim_resp.get;
  	Bool bus_error = response;
  	riscv.s5_cache.ma_io_response(tagged Valid tuple2(pack(bus_error),?));
    `logLevel( core, 1, $format("[%2d]CORE : ITIM Memory Write Response ",hartid, fshow(response)), simulate_log_start)
  endrule
`endif
`endif

  mkConnection(imem.ma_curr_priv, curr_priv);
  mkConnection(dmem.ma_curr_priv, curr_priv);

`ifdef supervisor
  // if supervisor is implemented connect the various csrs required by the ptwalk and dtlb modules
  // here
  mkConnection(imem.ma_satp_from_csr,riscv.csrs.mv_csr_satp);
  mkConnection(dmem.ma_satp_from_csr, riscv.csrs.mv_csr_satp);
  mkConnection(dmem.ma_mstatus_from_csr, riscv.csrs.mv_csr_mstatus);
  mkConnection(ptwalk.ma_satp_from_csr,riscv.csrs.mv_csr_satp);
  mkConnection(ptwalk.ma_mstatus_from_csr, riscv.csrs.mv_csr_mstatus);
  `ifndef hypervisor
  mkConnection(ptwalk.ma_curr_priv,curr_priv);
  `endif
  `ifdef hypervisor
    mkConnection(ptwalk.ma_hgatp_from_csr,riscv.csrs.mv_csr_hgatp);
    mkConnection(ptwalk.ma_vsatp_from_csr,riscv.csrs.mv_csr_vsatp);
    mkConnection(ptwalk.ma_hstatus_from_csr, riscv.csrs.mv_csr_hstatus);
    mkConnection(ptwalk.ma_vsstatus_from_csr, riscv.csrs.mv_csr_vsstatus);
    mkConnection(dmem.ma_vsatp_from_csr,riscv.csrs.mv_csr_vsatp);
    mkConnection(dmem.ma_vsstatus_from_csr, riscv.csrs.mv_csr_vsstatus);
    mkConnection(imem.ma_vsatp_from_csr,riscv.csrs.mv_csr_vsatp);
    mkConnection(imem.ma_vs_mode, riscv.csrs.mv_vs_bit);				//vs_bit from from csr_grp1		
  `endif
  /*doc:rule: this rule forwards the tlb miss request from the itlb to the pagetable walk module.
  * This only fires if the page-table walk module is not already handling a miss*/
  rule rl_itlb_req_to_ptwalk(rg_ptw_state == None);
    let req <- imem.get_request_to_ptw.get();
    ptwalk.from_tlb.put(req);
    rg_ptw_state <= IWalk;
  endrule:rl_itlb_req_to_ptwalk

  /*doc:rule: This rule passes the response from the ptwalk-module to the itlb*/
  rule rl_ptwalk_resp_to_itlb(rg_ptw_state == IWalk);
    let resp <- ptwalk.to_tlb.get();
    imem.put_response_frm_ptw.put(resp);
    rg_ptw_state <= None;
  endrule:rl_ptwalk_resp_to_itlb

  /*doc:rule: This rule is used to connect the request from the data tlb to the ptwalk-module. Fires
   * only when the ptwalk is not already processing another tlb miss*/
  rule rl_dtlb_req_to_ptwalk(rg_ptw_state == None);
    let req <- dmem.get_req_to_ptw.get();
    ptwalk.from_tlb.put(req);
    rg_ptw_state <= DWalk;
  endrule:rl_dtlb_req_to_ptwalk

  /*doc:rule: This rule connects the ptwalk-module response to the data tlb*/
  rule rl_ptwalk_resp_to_dtlb(rg_ptw_state == DWalk);
    let resp <- ptwalk.to_tlb.get();
    dmem.put_resp_from_ptw.put(resp);
    rg_ptw_state <= None;
  endrule:rl_ptwalk_resp_to_dtlb

  /*doc:connect: this connects the ptwalk request to the data memory subsystem*/
  let ptwalk_req <- mkConnection(dmem.receive_core_req, ptwalk.request_to_cache);

  /*doc:connect: This connects the data memory response to the ptwalk-module*/
  mkConnection(dmem.get_ptw_resp, ptwalk.response_frm_cache);

  /*doc:connect: when a tlb-miss occurs in the data tlb, we need to park the original request in the
   * ptwalk-module so that it can be replayed again at the end of the walk.*/
  mkConnection(dmem.get_hold_req, ptwalk.hold_req);
`endif

`ifdef perfmonitors
  `ifdef icache
    mkConnection(riscv.perfmonitors.ma_icache_counters,imem.mv_icache_perf_counters);
  `endif
  `ifdef dcache
    mkConnection(riscv.perfmonitors.ma_dcache_counters,dmem.mv_dcache_perf_counters);
  `endif
  `ifdef supervisor
    mkConnection(riscv.perfmonitors.ma_itlb_counters,imem.mv_itlb_perf_counters);
    mkConnection(riscv.perfmonitors.ma_dtlb_counters,dmem.mv_dtlb_perf_counters);
  `endif
`endif   
  
  interface sb_clint_msip = interface Put
    method Action put(Bit#(1) intrpt);
      riscv.interrupts.ma_clint_msip(intrpt);
    endmethod
  endinterface;
  interface sb_clint_mtip = interface Put
    method Action put(Bit#(1) intrpt);
      riscv.interrupts.ma_clint_mtip(intrpt);
    endmethod
  endinterface;
  interface sb_clint_mtime = interface Put
    method Action put (Bit#(64) c_mtime);
      riscv.interrupts.ma_clint_mtime(c_mtime);
    endmethod
  endinterface;

	method sb_plic_meip  = riscv.interrupts.ma_plic_meip;
`ifdef supervisor
	method sb_plic_seip = riscv.interrupts.ma_plic_seip;
`endif
`ifdef usertraps
	method sb_plic_ueip = riscv.interrupts.ma_plic_ueip;
`endif
	interface master_i = fetch_xactor.axi_side;
	interface master_d = memory_xactor.axi_side;
`ifdef rtldump
  interface commitlog = riscv.commitlog;
  interface sbread = riscv.sbread;
`endif
`ifdef simulate
  method mv_simulate_log_start = riscv.mv_simulate_log_start;
`endif
`ifdef debug
  method ma_debug_interrupt= riscv.ma_debug_interrupt;
  method mv_core_is_reset = riscv.mv_core_is_reset;
  method mv_core_debugenable = riscv.mv_core_debugenable;
  method ma_debugger_available = riscv.ma_debugger_available;
  method mv_stop_timer = riscv.mv_stop_timer;
  method mv_stop_count = riscv.mv_stop_count;
`endif
endmodule : mkccore_axi4

endpackage:ccore
