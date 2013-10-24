// Thermistor lookup table for two different thermistors.

/*
   This table doesn't depend on the type of electronics, but on the type of
   thermistor(s) you use. You want one table for each thermistor type you use.
*/

// How many thermistor tables we have.
#define NUMTABLES 2

// Names for our tables, so you can use them in config.h.
// Table numbering starts at 0.
#define THERMISTOR_EXTRUDER   0
#define THERMISTOR_BED        1

/*
   You may be able to improve the accuracy of this table in various ways.

   1. Measure the actual resistance of the resistor. It's "nominally" 4.7K,
      but that's ± 5%.
   2. Measure the actual beta of your thermistor:
      http://reprap.org/wiki/MeasuringThermistorBeta
   3. Generate more table entries than you need, then trim down the ones
      in uninteresting ranges.

   In either case you'll have to regenerate this table with
   createTemperatureLookup.py, which requires python, which is difficult to
   install on windows. Since you'll have to do some testing to determine the
   correct temperature for your application anyway, you may decide that the
   effort isn't worth it. Who cares if it's reporting the "right" temperature
   as long as it's keeping the temperature steady enough to print, right?
*/

// The number of value pairs in our table.
// Must be the same for all tables.
#define NUMTEMPS 102

uint16_t const temptable[NUMTABLES][NUMTEMPS][2] PROGMEM = {

// Table for the Extruder.
// Thermistor: EPCOS B57560G104F

// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=4555 --beta=4092 --max-adc=1023
{
#include "Thermistor_head.h"
},

// Thermistor table for the Heatbed.
// The thermistor used for this table was an Epocs B57560G104F

// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=4580 --beta=4092 --max-adc=1023
{
#include "Thermistor_bed.h"
}
};
