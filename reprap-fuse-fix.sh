# Use this with the USB-Tiny attached to the Gen7 to reset the fuses to normal.
# Must disconnect the ENDSTOPS for comms to work

sudo avrdude -c usbtiny -p atmega644p -B 5 -U lfuse:w:0xF7:m -U hfuse:w:0xDC:m -U efuse:w:0xFD:m
