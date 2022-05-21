/*
 * File:   main.c
 * Author: Niclas Forslund
 * Created on September 17, 2017, 10:13
 * 
 * Add laser pulse frequency user-programmable function for compatibility
 * with virtually any laser detector.
 * Tested on CST/Berger CL10 wich is same hardware.
 * Fernando Mederos april 2021
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

// Notes by Fernando Mederos adding variable pulse frequency
//
// For user-programmable pulse frequency we try to cover widest range possible
// We control both TMR0 start value and prescaler ratio to adjust pulse frequency
// Lowest freq: Prescaler 1:256, TMR0 starts counting from 0 -> pulses at 7Hz (aprox. 458PPM)
// Highest freq: Prescaler 1:1, TMR0 starts from 205 -> pulses at 10KHz
// We need to keep two more values in EEPROM: Count and Prescale.
// To increase pulse freq. we decrease Count value, to decrease freq. we increase Count value.
// If Count value exceeds 256 we increase Prescale to
// next step (double it's value) AND also divide Count value by two.
// If Count value is less than 128 we decrease Prescale (double value) AND
// also multiply Count value by two.
// To reach 10KHz pulse repetition freq. we need to let Count value go down
// to 50 with Prescale ratio of 1.
// 
// Algorithm to calculate prescaler value and Tmr0 start value:
// Prescaler value is most-significant to determine pulse period.
// Try to obtain desired freq. with tmr0 starting between 0 and 128
// Period = 1/Freq/2 this is toggling period
// Ratio = Period / 128 / 1uS this is used to select prescale value
// if Ratio >= 256 set PS2:0 = 7 and P=256
// if Ratio >= 128 set PS2:0 = 6 and P=128
// if Ratio >= 64 set PS2:0 = 5 and P=64
// ...
// if Ratio >= 2 set PS2:0 = 0 and P=2
// If Ratio < 2 disable prescaler (PSA=1) and P=1
// Count = Period / (1uS * P)
// TMR0 start value = 256 - Count



#include <xc.h>
#include <stdint.h>
#include "defines.h"


//#define T_VAL 0xF5	// 2.6kHz measured frequency


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

#define MODE_V      0x2U
#define MODE_H      0x4U
#define MODE_HV     0x6U

//#define bittest(D,i) (D & (0x01ULL << i))
#define bittest(D,i) (D & (0x01UL << i))



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
-At address 0x03   Count-H value for TMR0 initialization (default 50)
-At address 0x04   Count-L value for TMR0 initialization (default 50)
-At address 0x05   Prescale value for PS2:0 setting +1, (default 0), if value=0 Prescaler off
*/
#define EEPROM_COUNT_H    0x03
#define EEPROM_COUNT_L    0x04
#define EEPROM_PRESCALE 0x05
__EEPROM_DATA(0x06, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);



//Variable definitions
uint8_t laserMode;	// Keeps state of H- and V-laser
uint8_t locked;		// Ignore inclination switch
uint8_t indoorMode;	// Increase intensity of laser
uint8_t inLevel;	// Inclination switch state
uint8_t countH;    // Duration of High-state pulse out
uint8_t countL;    // Duration of Low-state pulse out
uint8_t tmrPresetH;  // Contains pre-calculated 256 - count to save some microseconds at the ISR
uint8_t tmrPresetL; // low-time pulse correction count
uint8_t prescale;   // Prescale index 0..8 0=1(off), 1=2,... 8=256
uint16_t indDelay;
uint16_t prIndDelay; // Preset value for flash indication delay. To show laser
                    // is pulsating.
uint8_t butDelay;   // button delay used to write eeprom seconds after button release

#define BRK_COUNT_LOW   140  // Low-level and high-level have different minimum count under
                            // this break point when prescaling is 1:1 (highest freqs).
                            // This tries to compensate for laser diode response
                            // delay observed.
                            // horizontal laser needs longer low-state (LED-On) 
                            // period to keep luminosity at higher freqs.
#define MIN_COUNT_LOW  50  // 
#define MIN_COUNT_HIGH  13

// Function definitions
void initSetup(void);
void setLaserMode(uint8_t);
void setLaserFreq(uint8_t, uint8_t, uint8_t);



// The interrupt function only handle Timer0 overflow
// Lasers will be toggled on and off while in outdoor mode
// Try to do as little as possible during this ISR to let it return as quick
// as possible, leave all the work to be done at main loop.
void __interrupt() swInt(void)
{
    if (INTCONbits.T0IF){
        PORTC ^= laserMode;     // toggle laser
        if((PORTC & MODE_HV) != 0){
            // make low pulse different length (there is an inverter before the laser)
            TMR0 = tmrPresetL;
        }
        else TMR0 = tmrPresetH;
        // Decrement indication delay for flashing H&V LEDs in main loop
        if(indDelay) indDelay--;     
        
        INTCONbits.T0IF = 0;
    }
}


// Main funciton contains setup function and an eternal while loop.
void main()
{
    initSetup();

// Recall last used settings from EEPROM
    laserMode	= eeprom_read(0x00);
    locked	= eeprom_read(0x01);
    indoorMode	= eeprom_read(0x02);
    countH = eeprom_read(EEPROM_COUNT_H);
    if(countH < MIN_COUNT_HIGH){
        countH = MIN_COUNT_HIGH;
        eeprom_write(EEPROM_COUNT_H, MIN_COUNT_HIGH);
    }
    countL = eeprom_read(EEPROM_COUNT_L);
    if(countL < MIN_COUNT_LOW){
        countL = MIN_COUNT_LOW;
        eeprom_write(EEPROM_COUNT_L, MIN_COUNT_LOW);
    }
    prescale = eeprom_read(EEPROM_PRESCALE);
    if(prescale > 8){
        prescale = 8;
        eeprom_write(EEPROM_PRESCALE, 8);
    }
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
    setLaserFreq(prescale, countH, countL);

// IF Lock button is pressed while power-on then enter pulse frequency setting mode
    if (0 == bittest(PORTC, LOCK_BUTTON)) {
	__delay_ms(DEBOUNCE);
	if (0 == bittest(PORTC, LOCK_BUTTON)) {
	    eeprom_write(0x02, indoorMode);
        // wait for lock button release
	    while (0 == bittest(PORTC,LOCK_BUTTON));
	    __delay_ms(DEBOUNCE);

        indoorMode = 0;     // set outdoor mode temporarily
        INTCONbits.T0IE = 1;

        while(1){

            if(!indDelay){  // toggle LED indicators at a scaled-down freq.
                PORTA ^= (laserMode & MODE_V);
                PORTA ^= (laserMode & MODE_H)>>2;
                indDelay=prIndDelay;
            }

            // check button presses
            if (0 == bittest(PORTC, LOCK_BUTTON)) {
            __delay_ms(DEBOUNCE);
            if (0 == bittest(PORTC, LOCK_BUTTON)) {
                // acknowledge button press
                PORTC |= (1<<LOCK_LED);
                PORTA &= ~(1<<WARN_LED);    // turn waring off
                if(prescale){
                    // prescale not 1:1 keep counts between 127 and 255
                    if(countL <= 127){
                        // we can decrease prescale we do and double count
                        countH=255;
                        countL=255;
                        prescale--;
                    }
                    else{
                        // decrease both counts
                        countL--;
                        countH=countL;
                    }
                }
//                 Cannot decrease prescaler->
//                 Due to a special behaviour of Horizontal LED we need
//                 to independently shorten on and off periods. Otherwise
//                 LED significantly reduces brightness at around 3KHz.
//                 By independently controlling on and off we can achieve a much
//                 higher pulse frequency IF we place a pull-down resistor at
//                 its cathode in order not to let diode current drop to zero.
                else if(countL > BRK_COUNT_LOW){
                    // decrease both counts
                    countL--;
                    countH=countL;
                }
                else if(countH > MIN_COUNT_HIGH){
                    // decrease only countH
                    countH--;                   
                }
                else if(countL > MIN_COUNT_LOW){
                    // decrease only countL
                    countL--;
                }
                else{
                    // reached limit
                    PORTA |= (1<<WARN_LED);
                }
                setLaserFreq(prescale, countH, countL);
                butDelay = 255;     // start delay to write new nalue to eeprom
                __delay_ms(DEBOUNCE);
            }
            }

            if (0 == bittest(PORTC, MODE_BUTTON)) {
            __delay_ms(DEBOUNCE);
            if (0 == bittest(PORTC, MODE_BUTTON)) {
                // acknowledge button press by pulsing lock led
                PORTC |= (1<<LOCK_LED);
                PORTA &= ~(1<<WARN_LED);    // turn waring off
                // decrease pulse frequency (increase count)
                if(prescale){
                    // prescale not 1:1 keep counts between 128 and 255
                    if(countL == 255){
                        if(prescale < 8){
                            prescale++;
                            countH=128;
                            countL=128;
                        }
                        else{
                            // reached limit
                            PORTA |= (1<<WARN_LED);
                        }
                    }
                    else{
                        countL++;
                        countH=countL;
                    }
                }
                // we are at 1:1 prescale, on/off periods receive special treatment
                else if(countL == 255){
                    // reached top without prescaling...
                    prescale++;
                    countH=128;
                    countL=128;
                }
                else if(countH > BRK_COUNT_LOW){
                    // both periods increase
                    countL++;
                    countH=countL;
                }
                else if(countL >= BRK_COUNT_LOW){
                    // only high period increase
                    countH++;
                }
                else{
                    // only low period increase
                    countL++;
                }
                
                setLaserFreq(prescale, countH, countL);
                butDelay = 255;     // start delay to write new nalue to eeprom
                __delay_ms(DEBOUNCE);
            }
            }

            __delay_ms(DEBOUNCE);
            if(butDelay){
                if(--butDelay==0){
                    PORTA ^= (1<<WARN_LED);     // toggle warning led to signal write to eeprom
                    __delay_ms(DEBOUNCE);
                    eeprom_write(EEPROM_COUNT_H, countH);
                    eeprom_write(EEPROM_COUNT_L, countL);
                    eeprom_write(EEPROM_PRESCALE, prescale);
                    __delay_ms(DEBOUNCE);
                    PORTA ^= (1<<WARN_LED);
                }
            }
            PORTC &= ~(1<<LOCK_LED);
        }
	}
    }

// Set lock status LED according to EEPROM
    PORTC |= (locked<<LOCK_LED);

// *******************************************************************
// MAIN LOOP
// *******************************************************************
// The loop does 4 things:
// 1. Check if we are in level by checking inclination switch?
// 2. If in level or locked activate lasers
// 3. Have the laser mode button been pressed?
// 4. Have the lock button been pressed?
    while (1)
    {

// Check Inclination switch
	if (0 != bittest(PORTC, LEVEL_TRG)) {
	    __delay_ms(DEBOUNCE);
	    if (0 != bittest(PORTC, LEVEL_TRG)) {
            // Level is tilted
    		inLevel = locked;       // if user locked the laser inLevel will still be 1
            if (!inLevel) { // Turn OFF Laser and ON Warning
        	    INTCONbits.T0IE = 0;        // Disable toggling
                PORTA |= (1<<WARN_LED);
                PORTC &= ~(1<<V_CTRL | 1<<H_CTRL);      // OFF Lasers
                PORTA &= ~(1<<V_LED | 1<<H_LED);        // OFF indicators
                // leave interrupts desabled
            }
	    }
        else {
    		inLevel = 1;
        }
    }
    
// Activate lasers if leveled or locked-on  
    if(inLevel){
        PORTA &= ~(1<<WARN_LED);
        if(indoorMode){
            INTCONbits.T0IE = 0;    // disable laser toggling
            // just turn-on lasers
            PORTC |= laserMode;
            PORTA |= (laserMode & MODE_V);
            PORTA |= (laserMode & MODE_H)>>2;
        }
        else{
            // in Outdoor mode enable interrupts for toggling laser
            if(!INTCONbits.T0IE) INTCONbits.T0IE = 1;
            if(!indDelay){  // toggle LED indicators at a scaled-down freq.
                PORTA ^= (laserMode & MODE_V);
                PORTA ^= (laserMode & MODE_H)>>2;
                indDelay=prIndDelay;
            }
        }
    }


// Check if Mode Button has been pressed
	if (0 == bittest(PORTC, MODE_BUTTON)) {
	    __delay_ms(DEBOUNCE);
	    if (0 == bittest(PORTC, MODE_BUTTON)) {
		INTCONbits.T0IE = 0;

		switch (laserMode) {
		  case 0x2:	// Only H -> Only V
			laserMode = 0x4UL;
			break;
		  case 0x4:	// Only V -> both H and V
			laserMode = 0x6UL;
			break;
		  case 0x6:	// both H and V -> Only H
			laserMode = 0x2UL;
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


void setLaserFreq(uint8_t ps, uint8_t cntH, uint8_t cntL){
    INTCONbits.T0IE = 0;
    // limit count and prescale values
    if(cntH < MIN_COUNT_HIGH) cntH = MIN_COUNT_HIGH;
    if(cntL < MIN_COUNT_LOW) cntL = MIN_COUNT_LOW;
    if(ps > 8) ps = 8;
    if(ps==0){
//        TMR0=0;
        OPTION_REGbits.PSA    = 1;	// No prescaling for Timer0
        OPTION_REGbits.PS = 7;
//        TMR0=0;
    }
    else{
        OPTION_REGbits.PSA    = 0;	// Prescaler assigned to Timer0
        OPTION_REGbits.PS = ps-1;
    }
    tmrPresetH = 255 - cntH;      // Precalculate Timer0 preset value to make it
    // rollover after cnt counts.
    tmrPresetL = 255 - cntL;
    
    // keep minumum duration for low pulse to maintain LED brightness
//    if(cnt < MIN_COUNT_LOW) tmrPresetL = 255 - MIN_COUNT_LOW;
//    else tmrPresetL = tmrPreset;

    // Indication delay is used to flash H & V leds at a visible frequency
    // related to laser pulse frequency. This is done using prescaler rate
    // assignment.
    // Max laser pulse freq 10KHz makes leds flash 576 times slower or 17Hz
    // Min laser pulse freq 7Hz makes leds flash 64 times slower or 0.1Hz
    prIndDelay = (9U-ps) << 6;  // Preset Indication delay counter to scale-down
                                // flashing frequency up to a visible range
    INTCONbits.T0IE = 1;
}


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
 
//    TMR0	    = 0;	// Timer0 interrupts are used to
    INTCONbits.T0IF = 0;	// toggle laser on/off
    INTCONbits.GIE  = 1;
}

