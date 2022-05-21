#ifndef DEFINES_H
#define	DEFINES_H

// Check, https://gputils.sourceforge.io/html-help/PIC16F676-conf.html
// for configuration defined pragmas
/* Config Register CONFIGL @ 0x8007 */
#pragma config FOSC = INTRCIO	// INTRCIO oscillator: I/O function on RA4 and RA5
#pragma config WDTE = OFF	// Watchdog is disabled
#pragma config PWRTE = ON	// Power-up Timer Enable bit
#pragma config MCLRE = OFF	// MCLR/VPP pin function is MCLR
#pragma config BOREN = OFF	// No Brown-out Reset
#pragma config CPD = OFF	// No Data memory code protection
#pragma config CP = OFF		// No Program memory code protection

// Preprocessor define for __delay_ms() and __delay_us()
#define _XTAL_FREQ 4000000

#endif	/* DEFINES_H */

