include ./makefile.inc

# ------------------------------------- Makefile TARGETS ----------------------------------------- #
default: generate_verilog link_verilator generate_boot_files
gdb: generate_verilog link_verilator_gdb generate_boot_files
MOREDEFINES=$(addprefix -D , $(BSC_DEFINES))

%.bo:
	$(info building $@)
	@$(BSCCMD) $(MOREDEFINES) -p $(BSVINCDIR) $< || (echo "BSC COMPILE ERROR"; exit 1)

.PHONY: generate_verilog
generate_verilog: $(BSVBUILDDIR)/$(TOP_BIN)
	@cp ${BS_VERILOG_LIB}/../Verilog/RegFile.v ${VERILOGDIR}  
	@cp ${BS_VERILOG_LIB}/FIFO2.v ${VERILOGDIR}
	@cp ${BS_VERILOG_LIB}/FIFO1.v ${VERILOGDIR}
	@cp ${BS_VERILOG_LIB}/FIFO10.v ${VERILOGDIR}
	@cp ${BS_VERILOG_LIB}/FIFOL1.v ${VERILOGDIR}
	@cp ${BS_VERILOG_LIB}/Counter.v ${VERILOGDIR}
	@cp ${BS_VERILOG_LIB}/SizedFIFO.v ${VERILOGDIR}
	@cp ${BS_VERILOG_LIB}/ResetEither.v ${VERILOGDIR}
	@cp ${BS_VERILOG_LIB}/MakeReset0.v ${VERILOGDIR}
	@cp ${BS_VERILOG_LIB}/SyncReset0.v ${VERILOGDIR}
	@cp ${BS_VERILOG_LIB}/ClockInverter.v ${VERILOGDIR}
	@cp ${BS_VERILOG_LIB}/RevertReg.v ${VERILOGDIR}
	@cp ${BS_VERILOG_LIB}/SyncBit1.v ${VERILOGDIR}
	@cp ./common_verilog/bram_1rw.v ${VERILOGDIR}
	@cp ${BS_VERILOG_LIB}/FIFO20.v ${VERILOGDIR}
	@cp common_verilog/signedmul.v ${VERILOGDIR}
	@cp common_verilog/RegFile5r2w.v ${VERILOGDIR}
	@cp ${BS_VERILOG_LIB}/SyncRegister.v ${VERILOGDIR}
	@cp ${BS_VERILOG_LIB}/MakeClock.v ${VERILOGDIR}
	@cp ${BS_VERILOG_LIB}/SyncHandshake.v ${VERILOGDIR}
	@cp ${BS_VERILOG_LIB}/BRAM2.v ${VERILOGDIR}
	@echo Compilation finished


.PHONY: link_verilator
link_verilator: ## Generate simulation executable using Verilator
	@echo "Linking $(TOP_MODULE) using verilator"
	@mkdir -p $(BSVOUTDIR) obj_dir
	@echo "#define TOPMODULE V$(TOP_MODULE)" > sim_main.h
	@echo '#include "V$(TOP_MODULE).h"' >> sim_main.h
	verilator $(VERILATOR_FLAGS) --cc $(TOP_MODULE).v -y $(VERILOGDIR) \
		-y $(BS_VERILOG_LIB) -y common_verilog --exe
	@ln -f -s ../test_soc/sim_main.cpp obj_dir/sim_main.cpp
	@ln -f -s ../sim_main.h obj_dir/sim_main.h
	make $(VERILATOR_SPEED) VM_PARALLEL_BUILDS=1 -j4 -C obj_dir -f V$(TOP_MODULE).mk
	@cp obj_dir/V$(TOP_MODULE) $(BSVOUTDIR)/out

.PHONY: link_verilator_gdb
link_verilator_gdb: ## Generate simulation executable using Verilator and VPI for GDB
	@echo "Linking Verilator With the Shakti RBB Vpi"
	@mkdir -p $(BSVOUTDIR) obj_dir
	@echo "#define TOPMODULE V$(TOP_MODULE)_edited" >sim_main.h
	@echo '#include "V$(TOP_MODULE)_edited.h"' >> sim_main.h
	@sed  -f devices/jtagdtm/sed_script.txt  $(VERILOGDIR)/$(TOP_MODULE).v > tmp1.v
	@cat  devices/jtagdtm/verilator_config.vlt \
	      devices/jtagdtm/vpi_sv.v \
	      tmp1.v                         > $(VERILOGDIR)/$(TOP_MODULE)_edited.v
	@rm   -f  tmp1.v
	verilator $(VERILATOR_FLAGS) --threads-dpi all --cc $(TOP_MODULE)_edited.v --exe sim_main.cpp devices/jtagdtm/RBB_Shakti.c -y $(VERILOGDIR) -y $(BS_VERILOG_LIB) -y common_verilog 
	@ln -f -s ../test_soc/sim_main.cpp obj_dir/sim_main.cpp
	@ln -f -s ../sim_main.h obj_dir/sim_main.h
	@ln -f -s ./devices/jtagdtm/RBB_Shakti.c obj_dir/RBB_Shakti.c
	@echo "INFO: Linking verilated files"
	make $(VERILATOR_SPEED) VM_PARALLEL_BUILDS=1 -j4 -C obj_dir -f V$(TOP_MODULE)_edited.mk
	@cp obj_dir/V$(TOP_MODULE)_edited $(BSVOUTDIR)/out
	@cp test_soc/gdb_setup/code.mem$(XLEN) $(BSVOUTDIR)/code.mem
	@echo Linking finished


.PHONY: update_xlen
update_xlen:
	@echo "XLEN=$(XLEN)" > verification/dts/Makefile.inc

.PHONY: simulate
simulate: ## Simulate the 'out' executable
	@echo Simulation...
	@exec ./$(BSVOUTDIR)/out
	@echo Simulation finished

.PHONY: link_vcs
link_vcs: ## Generate simulation executable using Synopsys VCS
	@rm -rf $(BSVOUTDIR)
	@mkdir -p $(BSVOUTDIR)
	vcs -LDFLAGS -Wl,--no-as-needed -l vcs_compile.log -sverilog +vpi +v2k -lca +define+TOP=$(TOP_MODULE) $(VCS_MACROS) \
	+define+BSV_TIMESCALE=1ns/1ps +cli+4 +libext+.v +notimingcheck \
	-y $(VERILOGDIR)/ \
	${BS_VERILOG_LIB}/main.v -o out
	@mv csrc out* $(BSVOUTDIR)

.PHONY: link_ncverilog
link_ncverilog: ## Generate simulation executable using Cadence NCVerilog
	@echo "Linking $(TOP_MODULE) using ncverilog..."
	@rm -rf work include $(BSVOUTDIR)/work
	@mkdir -p $(BSVOUTDIR) work
	@echo "define work ./work" > cds.lib
	@echo "define WORK work" > hdl.var
	@ncvlog -64BIT -sv -cdslib ./cds.lib -hdlvar ./hdl.var +define+TOP=$(TOP_MODULE) $(VCS_MACROS)\
	${BS_VERILOG_LIB}/main.v \
	-y $(VERILOGDIR)/ \
	-y ${BS_VERILOG_LIB}/
	@ncelab  -cdslib ./cds.lib -hdlvar ./hdl.var work.main -timescale 1ns/1ps
	@echo 'ncsim -cdslib ./cds.lib -hdlvar ./hdl.var work.main #> /dev/null' > $(BSVOUTDIR)/out
	@mv work cds.lib hdl.var $(BSVOUTDIR)/
	@chmod +x $(BSVOUTDIR)/out
	@echo Linking finished

.PHONY: link_ncverilog_openocd
link_ncverilog_openocd: ## Generate simulation executable using Synopsys VCS with VPI for GDB
	@echo "Linking $(TOP_MODULE) using ncverilog..."
	@rm -rf work include bin/work
	@mkdir -p bin
	@mkdir work
	@echo "Building RBB VPI"
	@echo "define work ./work" > cds.lib
	@echo "define WORK work" > hdl.var
	@ncvlog -64BIT -sv -cdslib ./cds.lib -hdlvar ./hdl.var +define+TOP=$(TOP_MODULE) \
	${BS_VERILOG_LIB}/main.v \
	-y $(VERILOGDIR)/ \
	-y ${BS_VERILOG_LIB}/
	@ncelab -64BIT -cdslib ./cds.lib -hdlvar ./hdl.var work.main -loadvpi rbb_vpi.so: -timescale 1ns/1ps
	@echo 'ncsim -64BIT -cdslib ./cds.lib -hdlvar ./hdl.var -loadvpi rbb_vpi.so: work.main #> /dev/null' > $(BSVOUTDIR)/out
	@mv ./*.so $(BSVOUTDIR)/
	@mv work cds.lib hdl.var $(BSVOUTDIR)/
	@chmod +x $(BSVOUTDIR)/out
	@echo Linking finished

.PHONY: link_irun
link_irun:
	@irun -define TOP=mkTbSoC -timescale 1ns/1ps $(VERILOGDIR)/main.v \
	-y $(VERILOGDIR)/ \
	-y ${BS_VERILOG_LIB}/

.PHONY: link_msim
link_msim: ## Generate simulation executable using Mentor's ModelSim tool
	@echo "Linking $(TOP_MODULE) using modelsim..."
	@rm -rf work* $(BSVOUTDIR)/*
	@mkdir -p $(BSVOUTDIR)
	vlib work
	vlog -work work +libext+.v+.vqm -y $(VERILOGDIR) -y ${BS_VERILOG_LIB} \
		+define+TOP=$(TOP_MODULE) $(VCS_MACROS) ${BS_VERILOG_LIB}/main.v \
		./$(VERILOGDIR)/$(TOP_MODULE).v  > compile_log
	mv compile_log ./$(BSVOUTDIR)
	mv work ./$(BSVOUTDIR)
	echo 'vsim -quiet -novopt -lib work -do "run -all; quit" -c main' > $(BSVOUTDIR)/out
	@chmod +x $(BSVOUTDIR)/out
	@echo Linking finished

.PHONY: release-verilog-artifacts
release-verilog-artifacts: ## target to generate verilog artifacts
release-verilog-artifacts: generate_verilog generate_boot_files link_verilator
	@mkdir -p verilog-artifacts
	@mkdir -p verilog-artifacts/sim
	@cp -r ${VERILOGDIR} verilog-artifacts/
	@cp ${CONFIG} verilog-artifacts/
	@cp -r benchmarks verilog-artifacts/
	@cp ${BSVOUTDIR}/boot.* verilog-artifacts/sim
	@cp ${BSVOUTDIR}/out verilog-artifacts/sim/ccore
	@cp ${HWTOOLS_DIR}/IITM_LICENSE.txt verilog-artifacts/LICENSE.txt
	@mv verilog-artifacts ../

.PHONY: regress
regress: ## To run regressions on the core.
	@SHAKTI_HOME=$$PWD python3 $(SHAKTI_HOME)/verification/verif-scripts/makeRegress.py --shaktiHome=$(SHAKTI_HOME) $(opts)

.PHONY: test
test: ## To run a single riscv-test on the core.
	@SHAKTI_HOME=$$PWD python3 $(SHAKTI_HOME)/verification/verif-scripts/makeTest.py --shaktiHome=$(SHAKTI_HOME) $(opts)

.PHONY: simonly
simonly: ## To run a single riscv-test on the core.
	@SHAKTI_HOME=$$PWD CONFIG_LOG=0 perl -I$(SHAKTI_HOME)/verification/verif-scripts $(SHAKTI_HOME)/verification/verif-scripts/simOnly.pl $(opts)

.PHONY: torture
torture: ## To run riscv-tortur on the core.
	@SHAKTI_HOME=$$PWD perl -I$(SHAKTI_HOME)/verification/verif-scripts $(SHAKTI_HOME)/verification/verif-scripts/makeTorture.pl $(opts)

.PHONY: aapg
aapg: ## to generate and run aapf tests
	@SHAKTI_HOME=$$PWD perl -I$(SHAKTI_HOME)/verification/verif-scripts $(SHAKTI_HOME)/verification/verif-scripts/makeAapg.pl $(opts)

.PHONY: csmith
csmith: ## to generate and run csmith tests
	@SHAKTI_HOME=$$PWD perl -I$(SHAKTI_HOME)/verification/verif-scripts $(SHAKTI_HOME)/verification/verif-scripts/makeCSmith.pl $(opts)

.PHONY: benchmarks
benchmarks: ## to run benchmarks
	@make -C benchmarks hello
	@echo "Running hello.. \n Output:"
	@cd benchmarks/output; \
	ln -sf $(SHAKTI_HOME)/bin/* . ;\
	./out > /dev/null 2>&1 ;\
	cat app_log
	@make -C benchmarks coremarks
	@echo "Running coremarks.. \n Output:"
	@cd benchmarks/output; \
	ln -sf $(SHAKTI_HOME)/bin/* . ;\
	./out > /dev/null 2>&1 ;\
	cat app_log

.PHONY: generate_boot_files
generate_boot_files: ## to generate boot files for simulation
	@echo "XLEN=$(XLEN)" > boot/Makefile.inc
	@mkdir -p bin
	@cd boot/; make;
	@cut -c1-8 boot/boot.hex > bin/boot.MSB
	@if [ "$(XLEN)" = "64" ]; then\
	  cut -c9-16 boot/boot.hex > bin/boot.LSB;\
    else cp bin/boot.MSB bin/boot.LSB;\
  fi

.PHONY: merge_cov
merge_cov:
	cd $(SHAKTI_HOME)/verification/workdir && ln -s $(SHAKTI_HOME)/verilog verilog
	verilator_coverage --write merged.dat $(SHAKTI_HOME)/verification/workdir/*/*/*/coverage.dat	
	verilator_coverage --annotate logs merged.dat
	verilator_coverage --rank $(SHAKTI_HOME)/verification/workdir/*/*/*/coverage.dat	
	rm -rf $(SHAKTI_HOME)/verification/workdir/*/*/*/coverage.dat	

.PHONY: yml
yml:
	@SHAKTI_HOME=$$PWD python3 $(SHAKTI_HOME)/verification/verif-scripts/gen_yml.py $(opts)

.PHONY: clean
clean:
	rm -rf $(BSVBUILDDIR)/* *.log $(BSVOUTDIR)/* obj_dir $(VERILOGDIR)/*
	rm -f *.jou rm *.log *.mem log sim_main.h cds.lib hdl.var

clean_verif:
	rm -rf verification/workdir/*
	rm -rf verification/riscv-torture/output/riscv-torture
