#if !defined _SIMULATION_H && defined SIMULATION
#define	_SIMULATION_H

#include <stdint.h>
#include <stdbool.h>

#define PROGMEM
#define PGM_P const char *
#define PSTR(x) (x)
#define pgm_read_byte(x) (*((uint8_t *)(x)))
#define pgm_read_word(x) (*((uint16_t *)(x)))

#define	MASK(PIN) (1 << PIN)
#define ACD	7
#define OCIE1A	1


#undef X_STEP_PIN
#undef X_DIR_PIN
#undef X_MIN_PIN
#undef Y_STEP_PIN
#undef Y_DIR_PIN
#undef Y_MIN_PIN
#undef Z_STEP_PIN
#undef Z_DIR_PIN
#undef Z_MIN_PIN
#undef E_STEP_PIN
#undef E_DIR_PIN
#undef STEPPER_ENABLE_PIN
#undef HEATER_PIN
#undef FAN_PIN
#undef HEATER_PWM
#undef FAN_PWM

typedef enum {
	X_STEP_PIN,
	X_DIR_PIN,
	X_MIN_PIN,
	Y_STEP_PIN,
	Y_DIR_PIN,
	Y_MIN_PIN,
	Z_STEP_PIN,
	Z_DIR_PIN,
	Z_MIN_PIN,
	E_STEP_PIN,
	E_DIR_PIN,

	STEPPER_ENABLE_PIN,

	SCK,
	MOSI,
	MISO,
	SS,

	PIN_NB,


    // Define pins used
    WGM00 = 0,
    WGM01,
    WGM20,
    WGM21,
    CS00 = 0,
    CS02,
    CS20,
    CS21,
    CS22,
    DIO1,
    DIO18,
    DIO19,
    DIO22,
    DIO23,
    DIO24,
    DIO25,
    DIO26,
    DIO27,
    DIO28,
    DIO5,
    PB3,
    PB4,
    PINA1,
    PINA2,
} pin_t;

#undef TEMP_PIN_CHANNEL
#define TEMP_PIN_CHANNEL 0

extern uint8_t ACSR;
extern uint8_t TIMSK1;
extern volatile bool sim_interrupts;
extern uint16_t    TCCR0A,
    TCCR0B,
    TCCR2A,
    TCCR2B,
    OCR0A,
    OCR0B,
    OCR2A,
    OCR2B,
    TIMSK0,
    TIMSK2;

void WRITE(pin_t pin, bool on);
void SET_OUTPUT(pin_t pin);
void SET_INPUT(pin_t pin);

void sei(void);

#ifdef USE_WATCHDOG
#define wd_init()
#define wd_reset()
#endif

void sim_info(const char fmt[], ...);
void sim_error(const char msg[]);
void sim_assert(bool cond, const char msg[]);
inline void cli(void);
inline void cli() { }

#endif /* _SIMULATION_H */

