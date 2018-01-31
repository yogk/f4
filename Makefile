
TARGETS = ws2812
DIR = target/thumbv7em-none-eabihf/debug/examples/
ELFS = $(addprefix $(DIR), $(TARGETS))
EELFS = $(addsuffix .elf, $(ELFS))

.PHONY: all

all: xargo $(EELFS) 
	
xargo:
	xargo build --examples
		
%.elf : % 
	cp $< $@

