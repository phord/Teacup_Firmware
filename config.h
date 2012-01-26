/* Notice to developers: this file is intentionally included twice. */

/** \file
	\brief Sample Configuration

	\note this sample uses AIO0 for both X_STEP and thermistor, and is intended to be an example only!
*/

/*
	CONTENTS

	1. Mechanical/Hardware
	2. Acceleration settings
	3. Pinouts
	4. Temperature sensors
	5. Heaters
	6. Communication options
	7. Miscellaneous
	8. Appendix A - PWMable pins and mappings
*/

/***************************************************************************\
*                                                                           *
* 1. MECHANICAL/HARDWARE                                                    *
*                                                                           *
\***************************************************************************/

/*
	Set your microcontroller type in Makefile! atmega168/atmega328p/atmega644p/atmega1280

	If you want to port this to a new chip, start off with arduino.h and see how you go.
*/
#ifndef __AVR_ATmega644P__
  #ifndef __AVR_ATmega644__
	  #error GEN7 has a 644 or a 644P! set your cpu type in Makefile!
  #endif
#endif

/** \def F_CPU
	CPU clock rate
*/
#ifndef	F_CPU
	#define	F_CPU	20000000UL
#endif

/** \def HOST
	This is the motherboard, as opposed to the extruder. See extruder/ directory for GEN3 extruder firmware
*/
#define	HOST

/*
	Values reflecting the gearing of your machine.
		All numbers are fixed point integers, so no more than 3 digits to the right of the decimal point, please :-)
*/

/** \def STEPS_PER_M
	steps per meter ( = steps per mm * 1000 )

	calculate these values appropriate for your machine

	for threaded rods, this is
		(steps motor per turn) / (pitch of the thread) * 1000

	for belts, this is
		(steps per motor turn) / (number of gear teeth) / (belt module) * 1000

	half-stepping doubles the number, quarter stepping requires * 4, etc.

	valid range = 20 to 4'0960'000 (0.02 to 40960 steps/mm)
*/
#define	STEPS_PER_M_X					12450 // 4000 / 6 * 100 / 53
#define	STEPS_PER_M_Y					12450 // Empirically determined half-step value
#define	STEPS_PER_M_Z					320000

/// http://blog.arcol.hu/?p=157 may help with this one
#define	STEPS_PER_M_E					75236  // Calculated from here http://reprap.org/wiki/Volumetric_Dimension_settings  (was 96271)
//#define	STEPS_PER_M_E					43000  // Empirically determined (mm of feedstock)


/*
	Values depending on the capabilities of your stepper motors and other mechanics.
		All numbers are integers, no decimals allowed.

		Units are mm/min
*/

/// used for G0 rapid moves and as a cap for all other feedrates
#define	MAXIMUM_FEEDRATE_X			14000
#define	MAXIMUM_FEEDRATE_Y			18000
#define	MAXIMUM_FEEDRATE_Z			200
#define	MAXIMUM_FEEDRATE_E			2000

/// used when searching endstops and as default feedrate
#define	SEARCH_FEEDRATE_X			50
#define	SEARCH_FEEDRATE_Y			50
#define	SEARCH_FEEDRATE_Z			50
// no SEARCH_FEEDRATE_E, as E can't be searched

/** \def SLOW_HOMING
	wether to search the home point slowly
		With some endstop configurations, like when probing for the surface of a PCB, you can't deal with overrunning the endstop. In such a case, uncomment this definition.
*/
// #define	SLOW_HOMING

/// this is how many steps to suck back the filament by when we stop. set to zero to disable
#define	E_STARTSTOP_STEPS			289

/**
	Soft axis limits, in mm.
	Define them to your machine's size relative to what your host considers to be the origin.
*/

//#define	X_MIN			0.0
//#define	X_MAX			200.0

//#define	Y_MIN			0.0
//#define	Y_MAX			200.0

//#define	Z_MIN			0.0
//#define	Z_MAX			140.0

/**	\def E_ABSOLUTE
	Some G-Code creators produce relative length commands for the extruder, others absolute ones. G-Code using absolute lengths can be recognized when there are G92 E0 commands from time to time. If you have G92 E0 in your G-Code, define this flag.
*/
// #define E_ABSOLUTE



/***************************************************************************\
*                                                                           *
* 2. ACCELERATION                                                           *
*                                                                           *
* IMPORTANT: choose only one! These algorithms choose when to step, trying  *
*            to use more than one will have undefined and probably          *
*            disastrous results!                                            *
*                                                                           *
\***************************************************************************/

/** \def ACCELERATION_REPRAP
	acceleration, reprap style.
		Each movement starts at the speed of the previous command and accelerates or decelerates linearly to reach target speed at the end of the movement.
*/
// #define ACCELERATION_REPRAP

/** \def ACCELERATION_RAMPING
	acceleration and deceleration ramping.
		Each movement starts at (almost) no speed, linearly accelerates to target speed and decelerates just in time to smoothly stop at the target. alternative to ACCELERATION_REPRAP
*/
#define ACCELERATION_RAMPING

/** \def ACCELERATION
	how fast to accelerate when using ACCELERATION_RAMPING.
		given in mm/s^2, decimal allowed, useful range 1. to 10'000. Start with 10. for milling (high precision) or 1000. for printing
*/
#define ACCELERATION 5000.

/** \def ACCELERATION_TEMPORAL
	temporal step algorithm
		This algorithm causes the timer to fire when any axis needs to step, instead of synchronising to the axis with the most steps ala bresenham.

		This algorithm is not a type of acceleration, and I haven't worked out how to integrate acceleration with it.
		However it does control step timing, so acceleration algorithms seemed appropriate

		The Bresenham algorithm is great for drawing lines, but not so good for steppers - In the case where X steps 3 times to Y's two, Y experiences massive jitter as it steps in sync with X every 2 out of 3 X steps. This is a worst-case, but the problem exists for most non-45/90 degree moves. At higher speeds, the jitter /will/ cause position loss and unnecessary vibration.
		This algorithm instead calculates when a step occurs on any axis, and sets the timer to that value.

		// TODO: figure out how to add acceleration to this algorithm
*/
// #define ACCELERATION_TEMPORAL



/***************************************************************************\
*                                                                           *
* 3. PINOUTS                                                                *
*                                                                           *
\***************************************************************************/

/*
	Machine Pin Definitions
	- make sure to avoid duplicate usage of a pin
	- comment out pins not in use, as this drops the corresponding code and makes operations faster
*/

#include	"arduino.h"

/** \def USE_INTERNAL_PULLUPS
	internal pullup resistors
		the ATmega has internal pullup resistors on it's input pins which are counterproductive with the commonly used eletronic endstops, so they should be switched off. For other endstops, like mechanical ones, you may want to uncomment this.
*/
//#define USE_INTERNAL_PULLUPS

/*
	user defined pins
	adjust to suit your electronics,
	or adjust your electronics to suit this
*/

#define	X_STEP_PIN						DIO19
#define	X_DIR_PIN							DIO18
#define	X_MIN_PIN							DIO7
//#define	X_MAX_PIN							xxxx
//#define	X_ENABLE_PIN					xxxx
//#define	X_INVERT_DIR
//#define	X_INVERT_MIN
//#define	X_INVERT_MAX
//#define	X_INVERT_ENABLE

#define	Y_STEP_PIN						DIO23
#define	Y_DIR_PIN							DIO22
#define	Y_MIN_PIN							DIO5
//#define	Y_MAX_PIN							xxxx
//#define	Y_ENABLE_PIN					xxxx
//#define	Y_INVERT_DIR
//#define	Y_INVERT_MIN
//#define	Y_INVERT_MAX
//#define	Y_INVERT_ENABLE

#define	Z_STEP_PIN						DIO26
#define	Z_DIR_PIN							DIO25
#define	Z_MIN_PIN							DIO1
//#define	Z_MAX_PIN							xxxx
//#define	Z_ENABLE_PIN					xxxx
//#define	Z_INVERT_DIR
//#define	Z_INVERT_MIN
//#define	Z_INVERT_MAX
//#define	Z_INVERT_ENABLE

#define	E_STEP_PIN						DIO28
#define	E_DIR_PIN							DIO27
//#define E_ENABLE_PIN
//#define	E_INVERT_DIR
//#define	E_INVERT_ENABLE

#define	PS_ON_PIN							DIO15
#define STEPPER_ENABLE_PIN		DIO24
#define	STEPPER_INVERT_ENABLE



/***************************************************************************\
*                                                                           *
* 4. TEMPERATURE SENSORS                                                    *
*                                                                           *
\***************************************************************************/

/**
	TEMP_HYSTERESIS: actual temperature must be target +/- hysteresis before target temperature can be achieved.
	Unit is degree Celsius.
*/
#define	TEMP_HYSTERESIS			20
/**
	TEMP_RESIDENCY_TIME: actual temperature must be close to target for this long before target is achieved

	temperature is "achieved" for purposes of M109 and friends when actual temperature is within [hysteresis] of target for [residency] seconds
*/
#define	TEMP_RESIDENCY_TIME		60

/// which temperature sensors are you using? List every type of sensor you use here once, to enable the appropriate code. Intercom is the gen3-style separate extruder board.
// #define	TEMP_MAX6675
#define	TEMP_THERMISTOR
// #define	TEMP_AD595
// #define	TEMP_PT100
// #define	TEMP_INTERCOM
// #define	TEMP_NONE

/***************************************************************************\
*                                                                           *
* Define your temperature sensors here                                      *
*                                                                           *
* for GEN3 set temp_type to TT_INTERCOM and temp_pin to 0                   *
*                                                                           *
* Types are same as TEMP_ list above- TT_MAX6675, TT_THERMISTOR, TT_AD595,  *
*   TT_PT100, TT_INTERCOM, TT_NONE. See list in temp.c.                     *
*                                                                           *
\***************************************************************************/

#ifndef DEFINE_TEMP_SENSOR
	#define DEFINE_TEMP_SENSOR(...)
#endif

//                 name       type          pin		additional
//DEFINE_TEMP_SENSOR(extruder,	TT_THERMISTOR,		0,	THERMISTOR_EXTRUDER)
// DEFINE_TEMP_SENSOR(bed,				TT_THERMISTOR,	1,	THERMISTOR_EXTRUDER)
// "noheater" is a special name for a sensor which doesn't have a heater.
// Use "M105 P#" to read it, where # is a zero-based index into this list.
// DEFINE_TEMP_SENSOR(noheater,				TT_THERMISTOR,	1,	0)

// to get both thermistors working make sure to use an appropriate 'ThermistorTable.h' file! 
// See 'ThermistorTable.gen7.h' for an example
DEFINE_TEMP_SENSOR(extruder, TT_THERMISTOR, PINA2, THERMISTOR_EXTRUDER)
DEFINE_TEMP_SENSOR(bed, TT_THERMISTOR, PINA1, THERMISTOR_BED)



/***************************************************************************\
*                                                                           *
* 5. HEATERS                                                                *
*                                                                           *
\***************************************************************************/

/** \def HEATER_SANITY_CHECK
	check if heater responds to changes in target temperature, disable and spit errors if not
	largely untested, please comment in forum if this works, or doesn't work for you!
*/
// #define	HEATER_SANITY_CHECK

/***************************************************************************\
*                                                                           *
* Define your heaters here                                                  *
*                                                                           *
* If your heater isn't on a PWM-able pin, set heater_pwm to zero and we'll  *
*   use bang-bang output. Note that PID will still be used                  *
*                                                                           *
* See Appendix 8 at the end of this file for PWMable pin mappings           *
*                                                                           *
* If a heater isn't attached to a temperature sensor above, it can still be *
*   controlled by host but otherwise is ignored by firmware                 *
*                                                                           *
* To attach a heater to a temp sensor above, simply use exactly the same    *
*   name - copy+paste is your friend                                        *
*                                                                           *
* Some common names are 'extruder', 'bed', 'fan', 'motor'                   *
*                                                                           *
* A milling spindle can also be defined as a heater. Attach it to a         *
* temperature sensor of TT_NONE, then you can control the spindle's rpm     *
* via temperature commands. M104 S1..255 for spindle on, M104 S0 for off.   *
*                                                                           *
\***************************************************************************/

#ifndef DEFINE_HEATER
	#define DEFINE_HEATER(...)
#endif

//               name      port   pin    pwm
DEFINE_HEATER(extruder,	PB4)
DEFINE_HEATER(bed, PB3)

/// and now because the c preprocessor isn't as smart as it could be,
/// uncomment the ones you've listed above and comment the rest.
/// NOTE: these are used to enable various capability-specific chunks of code, you do NOT need to create new entries unless you are adding new capabilities elsewhere in the code!
/// so if you list a bed above, uncomment HEATER_BED, but if you list a chamber you do NOT need to create HEATED_CHAMBER
/// I have searched high and low for a way to make the preprocessor do this for us, but so far I have not found a way.

#define	HEATER_EXTRUDER HEATER_extruder
#define HEATER_BED HEATER_bed



/***************************************************************************\
*                                                                           *
* 6. COMMUNICATION OPTIONS                                                  *
*                                                                           *
\***************************************************************************/

/** \def REPRAP_HOST_COMPATIBILITY
	RepRap Host changes it's communications protocol from time to time and intentionally avoids backwards compatibility. Set this to the date the source code of your Host was fetched from RepRap's repository, which is likely also the build date.
	See the discussion on the reprap-dev mailing list from 11 Oct. 2010.

	Undefine it for best human readability, set it to an old date for compatibility with hosts before August 2010
*/
// #define REPRAP_HOST_COMPATIBILITY 19750101
// #define REPRAP_HOST_COMPATIBILITY 20100806
// #define REPRAP_HOST_COMPATIBILITY 20110509
// #define REPRAP_HOST_COMPATIBILITY <date of next RepRap Host compatibility break>

/**
	Baud rate for the connection to the host. Usually 115200, other common values are 19200, 38400 or 57600.
*/
#define	BAUD	115200

/** \def XONXOFF
	Xon/Xoff flow control.
		Redundant when using RepRap Host for sending GCode, but mandatory when sending GCode files with a plain terminal emulator, like GtkTerm (Linux), CoolTerm (Mac) or HyperTerminal (Windows).
		Can also be set in Makefile
*/
// #define	XONXOFF



/***************************************************************************\
*                                                                           *
* 7. MISCELLANEOUS OPTIONS                                                  *
*                                                                           *
\***************************************************************************/

/** \def DEBUG
	DEBUG
		enables /heaps/ of extra output, and some extra M-codes.
		WARNING: this WILL break most host-side talkers that expect particular responses from firmware such as reprap host and replicatorG
		use with serial terminal or other suitable talker only.
*/
// #define	DEBUG

/** \def BANG_BANG
BANG_BANG
drops PID loop from heater control, reduces code size significantly (1300 bytes!)
may allow DEBUG on '168
*/
// #define	BANG_BANG
/** \def BANG_BANG_ON
BANG_BANG_ON
PWM value for 'on'
*/
// #define	BANG_BANG_ON	200
/** \def BANG_BANG_OFF
BANG_BANG_OFF
PWM value for 'off'
*/
// #define	BANG_BANG_OFF	45

/**
	move buffer size, in number of moves
		note that each move takes a fair chunk of ram (69 bytes as of this writing) so don't make the buffer too big - a bigger serial readbuffer may help more than increasing this unless your gcodes are more than 70 characters long on average.
		however, a larger movebuffer will probably help with lots of short consecutive moves, as each move takes a bunch of math (hence time) to set up so a longer buffer allows more of the math to be done during preceding longer moves
*/
#define	MOVEBUFFER_SIZE	8

/** \def DC_EXTRUDER
	DC extruder
		If you have a DC motor extruder, configure it as a "heater" above and define this value as the index or name. You probably also want to comment out E_STEP_PIN and E_DIR_PIN in the Pinouts section above.
*/
// #define	DC_EXTRUDER HEATER_motor
// #define	DC_EXTRUDER_PWM	180

/** \def USE_WATCHDOG
	Teacup implements a watchdog, which has to be reset every 250ms or it will reboot the controller. As rebooting (and letting the GCode sending application trying to continue the build with a then different Home point) is probably even worse than just hanging, and there is no better restore code in place, this is disabled for now.
*/
// #define USE_WATCHDOG

/**
	analog subsystem stuff
	REFERENCE - which analog reference to use. see analog.h for choices
*/
#define	REFERENCE			REFERENCE_AVCC

/** \def STEP_INTERRUPT_INTERRUPTIBLE
	this option makes the step interrupt interruptible (nested).
	this should help immensely with dropped serial characters, but may also make debugging infuriating due to the complexities arising from nested interrupts
	\note disable this option if you're using a '168 or for some reason your ram usage is above 90%. This option hugely increases likelihood of stack smashing.
*/
#define		STEP_INTERRUPT_INTERRUPTIBLE	1

/**
	temperature history count. This is how many temperature readings to keep in order to calculate derivative in PID loop
	higher values make PID derivative term more stable at the expense of reaction time
*/
#define	TH_COUNT					8

/// this is the scaling of internally stored PID values. 1024L is a good value
#define	PID_SCALE						1024L

/** \def ENDSTOP_STEPS
	number of steps to run into the endstops intentionally
		As Endstops trigger false alarm sometimes, Teacup debounces them by counting a number of consecutive positives. Valid range is 1...255. Use 4 or less for reliable endstops, 8 or even more for flaky ones.
*/
#define	ENDSTOP_STEPS	4



/***************************************************************************\
*                                                                           *
* 8. APPENDIX A - PWMABLE PINS AND MAPPINGS                                 *
*                                                                           *
*                                                                           *
* list of PWM-able pins and corresponding timers                            *
* timer1 is used for step timing so don't use OC1A/OC1B                     *
* they are omitted from this listing for that reason                        *
*                                                                           *
* For the atmega168/328, timer/pin mappings are as follows                  *
*                                                                           *
* OCR0A - PD6  - DIO6                                                       *
* OCR0B - PD5  - DIO5                                                       *
* OCR2A - PB3  - DIO11                                                      *
* OCR2B - PD3  - DIO3                                                       *
*                                                                           *
* For the atmega644, timer/pin mappings are as follows                      *
*                                                                           *
* OCR0A - PB3  - DIO3                                                       *
* OCR0B - PB4  - DIO4                                                       *
* OCR2A - PD7  - DIO15                                                      *
* OCR2B - PD6  - DIO14                                                      *
*                                                                           *
* For the atmega1280, timer/pin mappings are as follows                     *
*                                                                           *
* OCR0A  - PB7 - DIO13                                                      *
* OCR0B  - PG5 - DIO4                                                       *
* OCR2A  - PB4 - DIO10                                                      *
* OCR2B  - PH6 - DIO9                                                       *
* OCR3AL - PE3 - DIO5                                                       *
* OCR3BL - PE4 - DIO2                                                       *
* OCR3CL - PE5 - DIO3                                                       *
* OCR4AL - PH3 - DIO6                                                       *
* OCR4BL - PH4 - DIO7                                                       *
* OCR4CL - PH5 - DIO8                                                       *
* OCR5AL - PL3 - DIO46                                                      *
* OCR5BL - PL4 - DIO45                                                      *
* OCR5CL - PL5 - DIO44                                                      *
*                                                                           *
\***************************************************************************/
