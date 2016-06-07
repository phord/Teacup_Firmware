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
.PHONY: sim avr clean all default program regressiontests
.PHONY: plot test

# Override variables in the stock makefiles
MCU=atmega2560
AVR_OVERRIDES = F_CPU=16000000L MCU=$(MCU)

default: avr

all: sim avr plot


UPLOADER_FLAGS  = -D -c stk500v2
UPLOADER_FLAGS += -b 115200
UPLOADER_FLAGS += -p $(MCU)
UPLOADER_FLAGS += -P /dev/ttyACM0

# Build the simulator
sim:
	@echo "----[ Simulator ]----"
	@make -sf Makefile-SIM

# Build Teacup for an Atmel processor
avr:
	@echo "----[ $(MCU) ]----"
	@make -sf Makefile-AVR $(AVR_OVERRIDES) UPLOADER_FLAGS="$(UPLOADER_FLAGS)"

# Shortcut to program target in Makefile-AVR
program:
	@echo "----[ $(MCU) ]----"
	@make -sf Makefile-AVR program $(AVR_OVERRIDES)

regressiontests test:
	@make -sf Makefile-common regressiontests

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
