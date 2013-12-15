################################################################################
#
## Example Makefile
#
# For convenience, copy this file to "Makefile" and customize it to fit your
# needs.
#
# Then you can type 'make avr' or simply 'make' to build for your printer.
#
################################################################################
.PHONY: sim avr clean all default plot program

# Override variables in the stock makefiles
export F_CPU = 20000000L
export MCU_TARGET = atmega644p

default: avr

all: sim avr plot

# Build the simulator
sim:
	@echo "----[ Simulator ]----"
	@make -sf Makefile-SIM

# Build Teacup for an Atmel processor
avr:
	@echo "----[ $(MCU_TARGET) ]----"
	@make -sf Makefile-AVR

program:
	@echo "----[ $(MCU_TARGET) ]----"
	@make -sf Makefile-AVR program

clean:
	@echo "----[ Clean ]----"
	@make -sf Makefile-SIM clean
	@make -sf Makefile-AVR clean
	# Add any more cleanup steps you want here. Example,
	@rm -f *.png

%.png: gnuplot/%.gpi
	gnuplot -persist $<

CHART_SOURCES = $(wildcard gnuplot/*.gpi)
CHARTS = $(notdir $(CHART_SOURCES:.gpi=.png))

plot: $(CHARTS)
