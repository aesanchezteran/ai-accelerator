PROJECT := dragon_accel
VIVADO_DIR := build/vivado
XPR := $(VIVADO_DIR)/$(PROJECT).xpr

.PHONY: all create gui clean synth impl bitstream

all: create

create:
	vivado -mode batch -source scripts/create_project.tcl

gui:
	vivado $(XPR)

synth:
	vivado -mode batch -source scripts/synth.tcl

impl:
	vivado -mode batch -source scripts/impl.tcl

bitstream:
	vivado -mode batch -source scripts/bitstream.tcl

clean:
	rm -rf build .Xil