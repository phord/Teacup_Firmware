.PHONY: sim avr plot clean

sim:
	make -sf Makefile-SIM

avr:
	make -sf Makefile-AVR

clean:
	make -sf Makefile-SIM clean
	make -sf Makefile-AVR clean
	rm -f *.png

%.png: gnuplot/%.gpi
	gnuplot -persist $<

CHART_SOURCES = $(wildcard gnuplot/*.gpi)
CHARTS = $(notdir $(CHART_SOURCES:.gpi=.png))

plot: $(CHARTS)
