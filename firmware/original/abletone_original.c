/**
 * @file ABLETONE_test.c
 * @brief Digital controller firmware for analog synthesizer pitch and RGB feedback.
 *
 * This program reads note-selection inputs from PORTD, generates a 12-bit DAC
 * output using a bit-banged SPI interface for the MCP4821 DAC, and updates an
 * RGB LED according to the current key position. Key up/down buttons allow
 * chromatic or octave-based shifting depending on the active input combination.
 */

#define F_CPU 4000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

/* ---------- Pin definitions ---------- */

/**
 * @defgroup DAC_Pins MCP4821 Software SPI Pins
 * @brief Pin mappings used for the bit-banged SPI DAC interface.
 * @{
 */

/** @brief DAC SPI signals are routed through PORTC. */
#define DAC_PORT_C      PORTC

/** @brief MOSI output pin for sending serial data to the DAC. */
#define DAC_MOSI_bm     PIN0_bm

/** @brief MISO input pin. Present for SPI-style layout, but unused by MCP4821. */
#define DAC_MISO_bm     PIN1_bm

/** @brief Serial clock output pin for the DAC. */
#define DAC_SCK_bm      PIN2_bm

/** @brief DAC chip-select signal is routed through PORTB. */
#define DAC_CS_PORT     PORTB

/** @brief Chip-select pin for the MCP4821 DAC. */
#define DAC_CS_bm       PIN2_bm

/** @} */

/**
 * @defgroup Input_Pins Note and Key Input Pins
 * @brief Digital inputs used for note selection and key shifting.
 * @{
 */

/** @brief PORTD contains the note and key-shift inputs. */
#define NOTE_PORT       PORTD

/** @brief Mask for note-selection inputs on PD0-PD3. */
#define NOTE_MASK       (PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm)

/** @brief Key-up button input on PD4. */
#define KEY_UP_bm       PIN4_bm

/** @brief Key-down button input on PD5. */
#define KEY_DN_bm       PIN5_bm

/** @} */

/**
 * @defgroup RGB_Pins RGB LED Pins
 * @brief Output pins used for RGB LED PWM control.
 * @{
 */

/** @brief Red LED output on PB5. */
#define RED_bm          PIN5_bm

/** @brief Green LED output on PA3. */
#define GREEN_bm        PIN3_bm

/** @brief Blue LED output on PA2. */
#define BLUE_bm         PIN2_bm

/** @} */

/* ---------- RGB STRUCT ---------- */

/**
 * @brief Stores one 8-bit RGB color value.
 *
 * Each field represents the PWM duty value for one LED channel.
 * Values range from 0 for off to 255 for maximum brightness.
 */
typedef struct {
	uint8_t r; /**< Red channel intensity. */
	uint8_t g; /**< Green channel intensity. */
	uint8_t b; /**< Blue channel intensity. */
} RGB_t;

/** @brief Number of key positions with assigned RGB colors. */
#define NUM_KEYS 36

/**
 * @brief RGB lookup table for each key position.
 *
 * The table maps the current key index to a color gradient. As the key changes,
 * the RGB LED shifts color to provide visual feedback for the selected pitch range.
 */
static const RGB_t rgb_values[NUM_KEYS] = {
	{255,0,0},{255,32,0},{255,64,0},{255,96,0},
	{255,128,0},{255,160,0},{255,192,0},{255,224,0},
	{255,255,0},{224,255,0},{192,255,0},{160,255,0},
	{128,255,0},{96,255,0},{64,255,0},{32,255,0},
	{0,255,0},{0,255,64},{0,255,128},{0,255,192},
	{0,255,255},{0,192,255},{0,128,255},{0,64,255},
	{0,0,255},{32,0,255},{64,0,255},{96,0,255},
	{128,0,255},{160,0,255},{192,0,255},{224,0,255},
	{255,0,255},{255,0,192},{255,0,128},{255,0,64}
};

/* ---------- Globals ---------- */

/**
 * @brief Current base key index.
 *
 * This value selects the starting pitch from the DAC note table. It is adjusted
 * by the key-up and key-down buttons and is limited to the range 0-35.
 */
static volatile uint8_t key = 0;

/**
 * @brief Flag set when the key-up interrupt is triggered.
 *
 * The ISR only sets this flag. Debouncing and key handling are performed later
 * in the main loop.
 */
static volatile uint8_t pd4_event = 0;

/**
 * @brief Flag set when the key-down interrupt is triggered.
 *
 * The ISR only sets this flag. Debouncing and key handling are performed later
 * in the main loop.
 */
static volatile uint8_t pd5_event = 0;

/* ---------- DAC helpers ---------- */

/**
 * @brief Pulls the DAC chip-select line low.
 *
 * This begins an MCP4821 serial transfer.
 */
static inline void dac_cs_low(void)  { PORTB.OUTCLR = DAC_CS_bm; }

/**
 * @brief Pulls the DAC chip-select line high.
 *
 * This ends an MCP4821 serial transfer and latches the transmitted data.
 */
static inline void dac_cs_high(void) { PORTB.OUTSET = DAC_CS_bm; }

/**
 * @brief Sets the software SPI clock line low.
 */
static inline void sck_low(void)     { PORTC.OUTCLR = DAC_SCK_bm; }

/**
 * @brief Sets the software SPI clock line high.
 */
static inline void sck_high(void)    { PORTC.OUTSET = DAC_SCK_bm; }

/**
 * @brief Sets the software SPI MOSI line low.
 */
static inline void mosi_low(void)    { PORTC.OUTCLR = DAC_MOSI_bm; }

/**
 * @brief Sets the software SPI MOSI line high.
 */
static inline void mosi_high(void)   { PORTC.OUTSET = DAC_MOSI_bm; }

/* ---------- IO Init ---------- */

/**
 * @brief Initializes GPIO directions, pull-ups, and button interrupts.
 *
 * The DAC MOSI and SCK pins are configured as outputs, while the DAC MISO pin
 * is configured as an input. Note and key-shift inputs use internal pull-ups.
 * PD4 and PD5 are configured to trigger interrupts on falling edges, allowing
 * button presses to be detected without polling every cycle.
 */
static void io_init(void)
{
	PORTC.DIRSET = DAC_MOSI_bm | DAC_SCK_bm;
	PORTC.DIRCLR = DAC_MISO_bm;

	PORTB.DIRSET = DAC_CS_bm;
	dac_cs_high();

	PORTD.DIRCLR = NOTE_MASK | KEY_UP_bm | KEY_DN_bm;
	PORTD.PIN0CTRL = PORT_PULLUPEN_bm;
	PORTD.PIN1CTRL = PORT_PULLUPEN_bm;
	PORTD.PIN2CTRL = PORT_PULLUPEN_bm;
	PORTD.PIN3CTRL = PORT_PULLUPEN_bm;
	PORTD.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
	PORTD.PIN5CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
}

/* ---------- RGB PWM ---------- */

/**
 * @brief Initializes PWM hardware for RGB LED control.
 *
 * Green and blue are driven using TCA0 split mode on PA3 and PA2. Red is driven
 * separately using TCB3 on PB5. Each channel uses an 8-bit PWM value, allowing
 * brightness values from 0 to 255.
 */
static void RGB_PWM_Init(void)
{
	PORTA.DIRSET = GREEN_bm | BLUE_bm;
	PORTB.DIRSET = RED_bm;

	PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTA_gc;

	TCA0.SPLIT.CTRLA = 0;
	TCA0.SPLIT.CTRLD = TCA_SPLIT_SPLITM_bm;
	TCA0.SPLIT.CTRLB = TCA_SPLIT_LCMP2EN_bm | TCA_SPLIT_HCMP0EN_bm;

	TCA0.SPLIT.LPER = 0xFF;
	TCA0.SPLIT.HPER = 0xFF;

	TCA0.SPLIT.LCMP2 = 0; /* PA2 = BLUE */
	TCA0.SPLIT.HCMP0 = 0; /* PA3 = GREEN */

	TCA0.SPLIT.CTRLA = TCA_SPLIT_CLKSEL_DIV16_gc | TCA_SPLIT_ENABLE_bm;

	/* PB5 = RED via TCB3 */
	TCB3.CTRLA = 0;
	TCB3.CTRLB = TCB_CNTMODE_PWM8_gc | TCB_CCMPEN_bm;
	TCB3.CCMP = 0x00FF;
	TCB3.CTRLA = TCB_ENABLE_bm;
}

/**
 * @brief Sets the RGB LED color.
 *
 * @param r Red channel duty value from 0 to 255.
 * @param g Green channel duty value from 0 to 255.
 * @param b Blue channel duty value from 0 to 255.
 */
static void RGB_Set(uint8_t r, uint8_t g, uint8_t b)
{
	/* PB5 = RED */
	TCB3.CCMP = ((uint16_t)r << 8) | 0xFF;

	/* PA3 = GREEN */
	TCA0.SPLIT.HCMP0 = g;

	/* PA2 = BLUE */
	TCA0.SPLIT.LCMP2 = b;
}

/**
 * @brief Updates the RGB LED based on the current key index.
 *
 * If the key value is outside the valid RGB table range, the LED is turned off.
 */
static void RGB_Update_From_Key(void)
{
	if (key >= NUM_KEYS)
	{
		RGB_Set(0,0,0);
		return;
	}

	RGB_t c = rgb_values[key];
	RGB_Set(c.r, c.g, c.b);
}

/* ---------- DAC ---------- */

/**
 * @brief Sends a raw 12-bit value to the MCP4821 DAC.
 *
 * The MCP4821 expects a 16-bit packet. The upper control bits configure the DAC,
 * while the lower 12 bits contain the output code. This function manually clocks
 * out the packet MSB-first using software SPI.
 *
 * @param data12 Raw 12-bit DAC value. Only the lower 12 bits are used.
 */
static void mcp4821_write_raw(uint16_t data12)
{
	uint16_t packet = (0x1 << 12) | (data12 & 0x0FFF);

	dac_cs_low();

	for (int8_t i = 15; i >= 0; i--)
	{
		if (packet & (1 << i)) mosi_high();
		else mosi_low();

		sck_high();
		sck_low();
	}

	dac_cs_high();
}

/**
 * @brief Writes a bounded DAC code to the MCP4821.
 *
 * Values above the 12-bit DAC maximum are clamped to 4095 before transmission.
 *
 * @param code Desired DAC output code from 0 to 4095.
 */
static void mcp4821_set_code(uint16_t code)
{
	if (code > 4095) code = 4095;
	mcp4821_write_raw(code);
}

/* ---------- Notes ---------- */

/**
 * @brief DAC output values corresponding to available note positions.
 *
 * These values map note indices to 12-bit DAC codes. The analog synthesizer
 * interprets the resulting voltage as the pitch-control signal.
 */
static const uint16_t note_values[] = {
	/* OCTAVE 3 */
	82,164,246,328,410,492,574,656,738,820,902,984,
	/* OCTAVE 4 */
	1066,1148,1230,1312,1394,1476,1558,1640,1722,1804,1886,1968,
	/* OCTAVE 5 */
	2050,2132,2214,2296,2378,2460,2542,2624,2706,2788,2870,2952,
	/* OCTAVE 6 (necessary for scales, not for playing this octave specifically) */
	3034,3116,3198,3280,3362,3444,3526,3608,3690,3772,3854,3936
};

/**
 * @brief Converts the note input bitmask into a scale degree.
 *
 * The note inputs are active-low, so pressed inputs are detected by inverting
 * VPORTD.IN and masking the lower four bits. Specific input combinations map
 * to one of seven supported scale degrees.
 *
 * @return Scale degree from 0 to 6, or -1 if no valid note input is active.
 */
static int8_t note_from_mask(void)
{
	uint8_t raw = (~VPORTD.IN) & 0x0F;

	switch (raw)
	{
		case 0x01: return 0;
		case 0x02: return 1;
		case 0x04: return 2;
		case 0x08: return 3;
		case 0x03: return 4;
		case 0x06: return 5;
		case 0x0C: return 6;
		default: return -1;
	}
}

/**
 * @brief Outputs the DAC value for the currently selected note.
 *
 * The selected scale degree is added to the current key offset to form the final
 * note-table index. If no valid note is pressed or the calculated index is out
 * of range, the DAC output is set to zero.
 */
static void play_note(void)
{
	int8_t degree = note_from_mask();
	if (degree < 0) { mcp4821_set_code(0); return; }

	static const uint8_t offsets[7] = {0,2,4,5,7,9,11};

	if (key > 35) key = 35;

	uint8_t index = key + offsets[degree];
	if (index >= sizeof(note_values)/sizeof(note_values[0]))
	{
		mcp4821_set_code(0);
		return;
	}

	mcp4821_set_code(note_values[index]);
}

/**
 * @brief Refreshes both the pitch output and RGB feedback.
 *
 * This function keeps the DAC output and LED color synchronized with the current
 * note and key state.
 */
static void update_output(void)
{
	play_note();
	RGB_Update_From_Key();
}

/* ---------- Interrupt ---------- */

/**
 * @brief PORTD interrupt service routine.
 *
 * Detects falling-edge events on the key-up and key-down inputs. The ISR only
 * records which event occurred, then clears the interrupt flags. Button handling
 * and debouncing are performed in the main loop.
 */
ISR(PORTD_PORT_vect)
{
	uint8_t flags = PORTD.INTFLAGS;

	if (flags & KEY_UP_bm) pd4_event = 1;
	if (flags & KEY_DN_bm) pd5_event = 1;

	PORTD.INTFLAGS = flags;
}

/* ---------- Services ---------- */

/**
 * @brief Polls and debounces the note-selection inputs.
 *
 * A note is accepted only after a short debounce delay confirms that the input
 * is stable. When a note is released, the DAC and RGB LED are turned off.
 */
static void service_note_inputs(void)
{
	static int8_t last = -1;
	int8_t now = note_from_mask();

	if (now != -1 && now != last)
	{
		_delay_ms(20);
		now = note_from_mask();

		if (now != -1 && now != last)
		{
			update_output();
			last = now;
		}
	}

	if (now == -1 && last != -1)
	{
		_delay_ms(20);
		if (note_from_mask() == -1)
		{
			mcp4821_set_code(0);
			RGB_Set(0,0,0);
			last = -1;
		}
	}
}

/**
 * @brief Checks whether all four note inputs are pressed.
 *
 * This condition is used as a modifier for octave shifting. When all note inputs
 * are active, the key-up and key-down buttons shift by one octave instead of one
 * chromatic step.
 *
 * @return true if all four note inputs are active, otherwise false.
 */
static bool octave_shift_request(void)
{
	return (((~VPORTD.IN) & 0x0F) == 0x0F);
}

/**
 * @brief Handles debounced key-up and key-down button events.
 *
 * Key-up and key-down events are set by the PORTD ISR. This function debounces
 * the button press, adjusts the key value, updates the DAC and RGB outputs, and
 * waits until the button is released before accepting another press.
 */
static void service_key_interrupts(void)
{
	if (pd4_event)
	{
		pd4_event = 0;
		_delay_ms(20);

		if (!(VPORTD.IN & KEY_UP_bm))
		{
			if (octave_shift_request())
				key = (key <= 23) ? key + 12 : 35;
			else if (key < 35)
				key++;

			update_output();
			while (!(VPORTD.IN & KEY_UP_bm));
		}
	}

	if (pd5_event)
	{
		pd5_event = 0;
		_delay_ms(20);

		if (!(VPORTD.IN & KEY_DN_bm))
		{
			if (octave_shift_request())
				key = (key >= 12) ? key - 12 : 0;
			else if (key > 0)
				key--;

			update_output();
			while (!(VPORTD.IN & KEY_DN_bm));
		}
	}
}

/* ---------- Main loop ---------- */

/**
 * @brief Main program entry point.
 *
 * Initializes GPIO, RGB PWM, and global interrupts. The main loop continuously
 * services note inputs and key-shift events, updating the DAC and RGB LED as
 * needed.
 *
 * @return This function does not return during normal operation.
 */
int main(void)
{
	io_init();

	RGB_PWM_Init();

	sei();

	while (1)
	{
		service_note_inputs();
		service_key_interrupts();
		_delay_ms(5);
	}
}