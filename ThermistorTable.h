// default thermistor lookup table

// How many thermistor tables we have
#define NUMTABLES 2

#define THERMISTOR_EXTRUDER	0
#define THERMISTOR_BED		1

#define NUMTEMPS 102

const uint16_t temptable[NUMTABLES][NUMTEMPS][2] PROGMEM = {
{
#include "Thermistor_head.h"
},

{
#include "Thermistor_bed.h"
}
};
