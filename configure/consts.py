
length_check_fields=['reset_pc']

bsc_cmd = '''bsc -u -verilog -elab -vdir {0} -bdir {1} -info-dir {1} \
+RTS -K4000M -RTS -check-assert  -keep-fires \
-opt-undetermined-vals -remove-false-rules -remove-empty-rules \
-remove-starved-rules -remove-dollar -unspecified-to X -show-schedule \
-show-module-use -cross-info {2}'''

bsc_defines = ''

verilator_cmd = ''' -O3 -LDFLAGS "-static" --x-assign fast \
 --x-initial fast --noassert sim_main.cpp --bbox-sys -Wno-STMTDLY \
 -Wno-UNOPTFLAT -Wno-WIDTH -Wno-lint -Wno-COMBDLY -Wno-INITIALDLY \
 --autoflush {0} {1} --threads {2} -DBSV_RESET_FIFO_HEAD \
 -DBSV_RESET_FIFO_ARRAY --output-split 20000 \
 --output-split-ctrace 10000'''

makefile_temp='''
VERILOGDIR:={0}

BSVBUILDDIR:={1}

BSVOUTDIR:={2}

BSCCMD:={3}

BSC_DEFINES:={4}

BSVINCDIR:={5}

BS_VERILOG_LIB:={6}lib/Verilog/

TOP_MODULE:={7}

TOP_DIR:={8}

TOP_FILE:={9}

VERILATOR_FLAGS:={10}

VERILATOR_SPEED:={11}

SHAKTI_HOME:={12}

XLEN:={13}

TOP_BIN={14}

include depends.mk
'''

dependency_yaml='''
caches_mmu:
  url: https://gitlab.com/shaktiproject/uncore/caches_mmu
  checkout: 14.1.0
common_bsv:
  url: https://gitlab.com/shaktiproject/common_bsv
  checkout: master
devices:
  url: https://gitlab.com/shaktiproject/uncore/devices
  checkout: 8.0.1
fabrics:
  url: https://gitlab.com/shaktiproject/uncore/fabrics
  checkout: 1.2.0
common_verilog:
  url: https://gitlab.com/shaktiproject/common_verilog
  checkout: master
verification:
  url: git@github.com:Mindgrove-Technologies/verification.git
  checkout: master
  recursive: True
  patch:
    - [riscv-tests/env , verification/patches/riscv-tests-shakti-signature-machine.patch]
benchmarks:
  url: https://gitlab.com/shaktiproject/cores/benchmarks
  checkout: c-class-chnages
csrbox:
  url: https://gitlab.com/shaktiproject/cores/csrbox
  checkout: Floating_point
riscv-config:
  url: https://gitlab.com/shaktiproject/cores/riscv-config
  checkout: No_hyp_reg
'''
