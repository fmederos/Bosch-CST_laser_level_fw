/*
 * File:   main.c
 * Author: Niclas Forslund
 * Created on September 17, 2017, 10:13
 *
 *
 *	An alternative firmware to the Bosch PLL360 laser leveller.
 *
 * After flashing the PLL 360 leveller with this firmware
 * it will behave as before. The only difference in normal mode
 * is that the laser will remember the last mode used.
 *
 * By holding down the Mode button while turning on the leveller,
 * the indoor/outdoor mode is toggled. The difference is that the
 * laser is flashing with ~2.6kHz so that laser detector devices
 * can detect the laser outside. Even in bright daylight.
 *
 * It has been tested outdoor with a laserdetector from Clasohlson
 * Laserdetektor Cocraft PRO Edition D50 (40-9978).
 * At least 55 meter have been measured to work.
 *
 * The battery lifetime has not yet been tested when using this firmware.
 *
 *	Use it at your own risk.
 *
 */

#include <xc.h>
#include <stdint.h>
#include "defines.h"


#define T_VAL 0xF5	// 2.6kHz measured frequency

// PORTA-defines
#define H_LED		0	// OUTPUT  RA0, Horizontal laser indicator LED
#define V_LED		1	// OUTPUT  RA1, Vertical laser indicator LED
#define BATT_STAT	2	// INPUT   RA2/AN2, 
#define WARN_LED	5	// OUTPUT  RA5, Warning indicator LED
// PORTC-defines
#define LOCK_LED	0	// OUTPUT  RC0, Lock indicator LED
#define V_CTRL		1	// OUTPUT  RC1, Vertical laser control pin
#define H_CTRL		2	// OUTPUT  RC2, Horizontal laser control pin
#define MODE_BUTTON	3	// INPUT   RC3, Mode button
#define LOCK_BUTTON	4	// INPUT   RC4, Locked mode button
#define LEVEL_TRG	5	// INPUT   RC5, Level switch

#define DEBOUNCE	10	// Debounce relax time in milliseconds


#define bittest(D,i) (D & (0x01ULL << i))


/* EEPROM address function mapping
-At address 0x00   Initial laser status
    0x2 - Vertical
    0x4 - Horizontal
    0x6 - Both Horizontal and Vertical (default)
-At address 0x01   Lock status
    0 = Unlocked	    (default)
    1 = Locked
-At address 0x02   Indoor mode
    0 = Pulsating laser
    1 = Laser constantly on (default)
*/
__EEPROM_DATA(0x06, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);



//Variable definitions
uint8_t laserMode;	// Keeps state of H- and V-laser
uint8_t locked;		// Ignore inclination switch
uint8_t indoorMode;	// Increase intensity of laser
uint8_t inLevel;	// Inclination switch state

// Function definitions
void initSetup(void);
void setLaserMode(uint8_t);


// The interrupt function only handle Timer0 overflow
// Lasers will be toggled on and off while in outdoor mode
void interrupt swInt(void)
{
    if (INTCONbits.T0IF)
    {
	TMR0 = T_VAL;
	if (!inLevel) { // Turn OFF Laser and ON Warning
	    PORTA |= (1<<WARN_LED);
	    PORTC &= ~laserMode;
	} else {
	    PORTA &= ~(1<<WARN_LED);
	    if (!indoorMode) {
		PORTC ^= laserMode;
	    } else {
		PORTC |= laserMode;
	    }
	}
    }
    INTCONbits.T0IF = 0;
}


// Main funciton contains setup function and an eternal while loop.
void main()
{
    initSetup();

// Recall last used settings from EEPROM
    laserMode	= eeprom_read(0x00);
    locked	= eeprom_read(0x01);
    indoorMode	= eeprom_read(0x02);
    inLevel	= 0x01UL;


// Toggle indoor/outdoor mode if the Mode button is held down while powering on
    if (0 == bittest(PORTC, MODE_BUTTON)) {
	__delay_ms(DEBOUNCE);
	if (0 == bittest(PORTC, MODE_BUTTON)) {
	    indoorMode ^= 1;
	    eeprom_write(0x02, indoorMode);
	    while (0 == bittest(PORTC,MODE_BUTTON));
	    __delay_ms(DEBOUNCE);
	}
    }

// Set lasers according to EEPROM
    setLaserMode(laserMode);


// Set lock status LED according to EEPROM
    PORTC |= (locked<<LOCK_LED);


// The loop check 3 things:
// 1. Are we in level by checking inclination switch?
// 2. Have the laser mode button been pressed?
// 3. Have the lock button been pressed?
    while (1)
    {

// Check Inclination switch
	if (0 != bittest(PORTC, LEVEL_TRG)) {
	    __delay_ms(DEBOUNCE);
	    INTCONbits.T0IE = 0;
	    if (0 != bittest(PORTC, LEVEL_TRG)) {
		inLevel = locked;
	    } else {
		inLevel = 1;
	    }
	    INTCONbits.T0IE = 1;
	}


// Check if Mode Button has been pressed
	if (0 == bittest(PORTC, MODE_BUTTON)) {
	    __delay_ms(DEBOUNCE);
	    if (0 == bittest(PORTC, MODE_BUTTON)) {
		INTCONbits.T0IE = 0;

		switch (laserMode) {
		  case 0x2:	// Only H -> Only V
			laserMode = 0x4ULL;
			break;
		  case 0x4:	// Only V -> both H and V
			laserMode = 0x6ULL;
			break;
		  case 0x6:	// both H and V -> Only H
			laserMode = 0x2ULL;
			break;
		  default:	// Will never end up here
			break;
		}
		eeprom_write(0x00, laserMode);
		while (0 == bittest(PORTC,MODE_BUTTON));
		__delay_ms(DEBOUNCE);
		setLaserMode(laserMode);
	    }
	    INTCONbits.T0IE = 1;
	}


// Check if Lock Button has been pressed
	if (0 == bittest(PORTC, LOCK_BUTTON)) {
	    __delay_ms(DEBOUNCE);
	    if (0 == bittest(PORTC, LOCK_BUTTON)) {
		INTCONbits.T0IE = 0;
		PORTC &= ~(1<<LOCK_LED);
		locked ^= 1;
		eeprom_write(0x01, locked);
		PORTC |= (locked<<LOCK_LED);
		while (0 == bittest(PORTC, LOCK_BUTTON));
		__delay_ms(DEBOUNCE);
		INTCONbits.T0IE = 1;
	    }
	}
    } // End while()
} // End main()




void setLaserMode(uint8_t mode) {
// Start function with a known state
    INTCONbits.T0IE = 0;
    PORTC &= ~(1<<V_CTRL | 1<<H_CTRL);
    PORTA &= ~(1<<V_LED | 1<<H_LED);

    switch (mode) {
	case 0x2:     // Set only Vertical laser
	    PORTC |= (1<<V_CTRL);
	    PORTA |= (1<<V_LED);
	    break;
	case 0x4:     // Set only Horizontal laser
	    PORTC |= (1<<H_CTRL);
	    PORTA |= (1<<H_LED);
	    break;
	case 0x6:     // Set both V- and H-laser
	    PORTC |= (1<<H_CTRL | 1<<V_CTRL);
	    PORTA |= (1<<H_LED | 1<<V_LED);
	    break;
	default: // I should never end up here. If I do, I revert to safe mode
	    eeprom_write(0x00, 0x6); // H+V laser on
	    eeprom_write(0x01, 0x0); // Unlocked mode
	    eeprom_write(0x02, 0x1); // Indoor mode
	    while (1) { // I start to signal error until restarted
		PORTA |= (1<<H_LED | 1<<V_LED | 1<<WARN_LED);
		PORTC |= (1<<LOCK_LED);
		__delay_ms(200);
		PORTA &= ~(1<<H_LED | 1<<V_LED | 1<<WARN_LED);
		PORTC &= ~(1<<LOCK_LED);
		__delay_ms(200);
	    }
    }
    INTCONbits.T0IE = 1;
}


void initSetup(void) {
    CMCON   = 0x07;		// Disable comparator

// When I follow the traces on the PCB I can see that battery voltage is
// connected to RA2/AN2. Probably in order to monitor battery status.
// The register is setup but I never use it.
    ANSEL   = (1<<BATT_STAT);
    TRISA   = (1<<BATT_STAT);
    TRISC   = (1<<MODE_BUTTON | 1<<LOCK_BUTTON | 1<<LEVEL_TRG);

    WPUA    = 0x00;	// Resistor pullups exists on PCB
    PORTA   = 0x00;	// LED- and laser-status is read from EEPROM
    PORTC   = 0x00;

    OPTION_REGbits.nRAPU  = 1;	// Disable weak pull-ups
    OPTION_REGbits.T0CS   = 0;	// Int. instr. cycle clock (FOSC/4)
    OPTION_REGbits.PSA    = 0;	// Prescaler assigned to Timer0
    OPTION_REGbits.PS = 0b011;	// Prescaler 1:16
 
    TMR0	    = 0;	// Timer0 interrupts are used to
    INTCONbits.T0IF = 0;	// toggle laser on/off
    INTCONbits.GIE  = 1;
}

