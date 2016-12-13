/* Copyright (C) 2014-2016 by Jacob Alexander
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// IMPORTANT NOTE from DMA
/*
 * Don't even try to use DMA with ADC software activation.
 * becaise it will fuck up results transfer.
 * You see, there's _supposed_ to be an ADC results buffer in K20,
 * but there's none.
 * So conversion start immediately fills ADCx_Rn with garbage.
 * You need to wait 3 full microseconds after conversion ends
 * before you can write to SC1n.
 * And DMA request is generated when? You guessed right, when COCO is asserted.
 * So you only scoop garbage from ADC_Rn.
 *
 * Chaining another DMA channel to write to ADCxSC1n AN4590-style works.
 * The only problem is you only get garbage instead of results.
 * Oh, and AN4590 itself has TONS of errors.
 * I didn't try to compile the sample project,
 * but if you paste texts from PDF - you're up to an unpleasant surprise.)
 */
// ----- Includes -----

// Compiler Includes
#include <Lib/ScanLib.h>

// Project Includes
#include <cli.h>
#include <led.h>
#include <print.h>
#include <matrix_scan.h>
#include <macro.h>

// Local Includes
#include "scan_loop.h"

// ----- Function Declarations -----

// CLI Functions
void cliFunc_ADCPrint( char* args);
void cliFunc_T( char* args);
void cliFunc_ADCCal( char* args);


// ----- Variables -----

// Scan Module command dictionary
CLIDict_Entry( ADCPrint,        "Prints contents of the ADC buffer." );
CLIDict_Entry( T,        "Start conversion." );
CLIDict_Entry( ADCCal,        "Calibrate, print results" );

CLIDict_Def( scanCLIDict, "Scan Module Commands" ) = {
	CLIDict_Item( ADCPrint ),
	CLIDict_Item( T ),
	CLIDict_Item( ADCCal ),
	{ 0, 0, 0 } // Null entry for dictionary end
};

// Number of scans since the last USB send
uint16_t Scan_scanCount = 0;

volatile uint8_t CoCo1 = 0;
volatile uint8_t CoCo2 = 0;

// ADC-related stuff
#define ADC0_CHAN_COUNT 16
#define ADC1_CHAN_COUNT 16
uint8_t adc1_sequencer[ADC0_CHAN_COUNT] = {
	 0, 30, 1, 30,   2, 30,  3, 30,
	4, 30, 5, 30,  6, 30,  7, 30
//	 4, 30, 15, 30,   7, 30,  6, 30,
//	12, 30, 13, 30,  14, 30,  5, 30
};
uint8_t adc0_sequencer[ADC1_CHAN_COUNT] = {
	30,  0, 30,  2,  30,  3, 30,  4,
	30,  4, 30,  5,  30,  6, 30,  7,
//	30,  6, 30,  7,  30,  0 ,30,  3
//	30,  6, 30,  7,  30,  0 ,30,  3
};

volatile uint16_t adc0_results[8][ADC0_CHAN_COUNT];
volatile uint16_t adc0_results_prev[8][ADC0_CHAN_COUNT];
volatile uint16_t adc1_results[8][ADC0_CHAN_COUNT];
volatile uint16_t adc1_results_prev[8][ADC0_CHAN_COUNT];
// ----- Functions -----

/*
 * To calibrate: SC2 = SW trigger no DMA, alt Vref
 * CFG2 = MUX b channels, 24ADCLK sample (24/16/10/6) if ADLSMP 
 * plus gain must be calculated and set by hand because calibration doesn't.
 */
#define ADC_CALIBRATE(idx)\
	ADC##idx##_SC2 = 0x0; \
	ADC##idx##_CFG1	= ADC_CFG1_ADIV(8) | ADC_CFG1_MODE(2) \
			| ADC_CFG1_ADLSMP | ADC_CFG1_ADICLK(3); \
\
	ADC##idx##_CFG2	= ADC_CFG2_MUXSEL | ADC_CFG2_ADACKEN \
			| ADC_CFG2_ADHSC | ADC_CFG2_ADLSTS(0); \
\
	ADC##idx##_SC3 = ADC_SC3_AVGE | ADC_SC3_AVGS(3); /* 32x */ \
\
	print( "Cal Start.." ); \
	ADC##idx##_SC3 |= ADC_SC3_CAL; \
	uint32_t t##idx = 100000; \
	while (t##idx && 0 == (ADC##idx##_SC1A & ADC_SC1_COCO)) { t##idx--; };\
	print( ".end. Cycles remaining: " );\
	printInt32( t##idx );\
	print( NL );\
\
	if (ADC##idx##_SC3 & ADC_SC3_CALF) return 255; /* failed */ \
	ADC##idx##_PG = (ADC##idx##_CLP0 + ADC##idx##_CLP1 + ADC##idx##_CLP2 \
 	+ ADC##idx##_CLP3 + ADC##idx##_CLP4 + ADC##idx##_CLPS) / 2 | 0x8000u; \


/* To setup:
 * CFG2 = MUX b channels, ADACKEN, ADHSC, 24ADCLK sample (24/16/10/6) if ADLSMP
 * SC3 = Single shot, no averaging
 * SC2 = internal Vref (=1)
*/
#define ADC_SETUP(mod)\
	ADC##mod##_CFG1 = ADC_CFG1_ADIV(1) | ADC_CFG1_MODE(2) | ADC_CFG1_ADICLK(1) /*| ADC_CFG1_ADLSMP*/ ;\
	ADC##mod##_CFG2 	= ADC_CFG2_MUXSEL | ADC_CFG2_ADACKEN \
			| ADC_CFG2_ADHSC | ADC_CFG2_ADLSTS(2); \
	ADC##mod##_SC3 = 0u; \
	ADC##mod##_SC2 = 0x01u; \

#define SENSOR_SETUP(idx) \
	ADC_CALIBRATE(idx) \
	ADC_SETUP(idx) \

// Calibration macro returns 255 on calibration fail.
// Dunno, may be remove that and integrate into Scan_setup
uint8_t ADC_Setup()
{
	// Power
	SIM_SCGC3 |= SIM_SCGC3_ADC1;
	SIM_SCGC6 |= SIM_SCGC6_ADC0;

	ADC_CALIBRATE(0)
	ADC_SETUP(0)

	ADC_CALIBRATE(1)
	ADC_SETUP(1)
	return 0;
}

// Setup
inline void Scan_setup()
{
	// Register Scan CLI dictionary
	CLI_registerDictionary( scanCLIDict, scanCLIDictName );

	// Setup GPIO pins for matrix scanning
	Matrix_setup();

	// Reset scan count
	Scan_scanCount = 0;

	SIM_SCGC4 |= SIM_SCGC4_VREF;

	// Vref: VREFEN + REGEN + ICOMPEN + MODE_LV=1 (high power, 2 is low)
	VREF_SC = 0x80u + 0x40u + 0x20u + 0x01u;

	//SIM_SCGC5 |= SIM_SCGC5_PORTA | SIM_SCGC5_PORTB | SIM_SCGC5_PORTC | SIM_SCGC5_PORTD | SIM_SCGC5_PORTE;

	ADC_Setup();
}

// Main Detection Loop
inline uint8_t Scan_loop()
{
	for (uint8_t i = 0; i < 8; i++)
	{
		Matrix_sense(1);
		Matrix_strobe(i, 1);
		//printInt32( SYST_CVR );
		for (uint8_t k = 0; k < ADC0_CHAN_COUNT; k++)
		{
			ADC0_SC1A = adc0_sequencer[k];
			ADC1_SC1A = adc1_sequencer[k];
			while (!(ADC0_SC1A && ADC_SC1_COCO)){};
			adc0_results[i][k] = (uint16_t)ADC0_RA >> 1;
			while (!(ADC1_SC1A && ADC_SC1_COCO)){};
			adc1_results[i][k] = (uint16_t)ADC1_RA >> 1;
			delayMicroseconds(3);
		}
		Matrix_sense(0);
		Matrix_strobe(i, 0);
		delayMicroseconds(3); // Give it time to settle down
	}
	// Not ready for prime time.
	//Matrix_scan( Scan_scanCount++ );
	return 0;
}


// Signal from Macro Module that all keys have been processed (that it knows about)
inline void Scan_finishedWithMacro( uint8_t sentKeys )
{
}


// Signal from Output Module that all keys have been processed (that it knows about)
inline void Scan_finishedWithOutput( uint8_t sentKeys )
{
	// Reset scan loop indicator (resets each key debounce state)
	// TODO should this occur after USB send or Macro processing?
	Scan_scanCount = 0;
}



// ----- Capabilities -----

// Custom capability examples
// Refer to kll.h in Macros/PartialMap for state and stateType information
void CustomAction_action1_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
	// Display capability name
	// XXX This is required for debug cli to give you a list of capabilities
	if ( stateType == 0xFF && state == 0xFF )
	{
		print("CustomAction_action1_capability()");
		return;
	}

	// Prints Action1 info message to the debug cli
	info_print("Action1");
}

uint8_t CustomAction_blockHold_storage = 0;
void CustomAction_blockHold_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
	// Display capability name
	if ( stateType == 0xFF && state == 0xFF )
	{
		print("CustomAction_blockHold_capability(usbCode)");
		return;
	}

	// Retrieve 8-bit argument
	uint8_t key = args[0];

	// We only care about normal keys
	if ( stateType == 0x00 )
	{
		// Block given key if we're in the "Press" or "Hold" state
		if ( ( state == 0x01 || state == 0x02 )
			&& CustomAction_blockHold_storage == 0 )
		{
			CustomAction_blockHold_storage = key;
			info_msg("Blocking Key: ");
			printHex( key );
			print( NL );
		}
		// Release if in the "Off" or "Release" state and we're blocking
		else if ( ( state == 0x00 || state == 0x03 )
			&& key == CustomAction_blockHold_storage )
		{
			info_msg("Unblocking Key: ");
			printHex( CustomAction_blockHold_storage );
			print( NL );
			CustomAction_blockHold_storage = 0;
		}
	}
}

void CustomAction_blockKey_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
	// Display capability name
	if ( stateType == 0xFF && state == 0xFF )
	{
		print("CustomAction_blockKey_capability(usbCode)");
		return;
	}

	// Retrieve 8-bit argument
	uint8_t key = args[0];

	// If key is not blocked, process
	if ( key != CustomAction_blockHold_storage )
	{
		extern void Output_usbCodeSend_capability( uint8_t state, uint8_t stateType, uint8_t *args );
		Output_usbCodeSend_capability( state, stateType, &key );
	}
}


// Signal from the Output Module that the available current has changed
// current - mA
void Scan_currentChange( unsigned int current )
{
	// Indicate to all submodules current change
	Matrix_currentChange( current );
}



// ----- CLI Command Functions -----
void cliFunc_T( char* args )
{
	// Test the scan loop (at max speed or general debugging)
	print( NL );
	for (uint32_t j = 0; j < 100000; j++)
	{
		for (uint8_t i = 0; i < 8; i++)
		{
			Matrix_sense(1);
			Matrix_strobe(i, 1);
			for (uint8_t k = 0; k < ADC0_CHAN_COUNT; k++)
			{
				ADC0_SC1A = adc0_sequencer[k];
				ADC1_SC1A = adc1_sequencer[k];
				while (!(ADC0_SC1A && ADC_SC1_COCO)){};
				adc0_results[i][k] = (uint16_t)ADC0_RA >> 1;
				while (!(ADC1_SC1A && ADC_SC1_COCO)){};
				adc1_results[i][k] = (uint16_t)ADC1_RA >> 1;
				delayMicroseconds(3);
			}
			Matrix_sense(0);
			Matrix_strobe(i, 0);
			delayMicroseconds(3); // Give it time to settle down
		}
		//cliFunc_ADCPrint("");
		if (j % 1000 == 0)
		{
			print ( ".");
		}
	}

}

void cliFunc_ADCPrint( char* args )
{
	print( NL );
	for (uint8_t j=0; j < 8; j++)
	{
		for (uint8_t i=0; i < ADC0_CHAN_COUNT; i+=2)
		{
			if (adc0_results[j][i] - adc0_results_prev[j][i] < 0)
				print("!");
			else
				printInt16(adc0_results[j][i] - adc0_results_prev[j][i]);
			adc0_results_prev[j][i] = adc0_results[j][i];
			print (" ");
		}
		print (" | ");
		for (uint8_t i=1; i < ADC1_CHAN_COUNT; i+=2)
		{
			if (adc1_results[j][i] - adc1_results_prev[j][i] < 0)
				print("!");
			else
				printInt16(adc1_results[j][i] - adc1_results_prev[j][i]);
			adc1_results_prev[j][i] = adc1_results[j][i];
			print (" ");
		}
		print( NL );
	}
}

inline void printADCCalData( void )
{
	printHex(ADC0_PG);
	dPrint (" ");
	printHex(ADC0_CLP0);
	dPrint (" ");
	printHex(ADC0_CLP1);
	dPrint (" ");
	printHex(ADC0_CLP2);
	dPrint (" ");
	printHex(ADC0_CLP3);
	dPrint (" ");
	printHex(ADC0_CLP4);
	dPrint (" ");
	printHex(ADC0_CLPS);
	dPrint (" ");
	printHex(ADC0_CLPD);
	dPrint ( NL );
}

void cliFunc_ADCCal( char* args )
{
	print( NL );
	print ( "Old: " );
	printADCCalData();
	ADC_Setup();
	print ( "New: ");
	printADCCalData();
}

// vim:ts=8:sts=8:sw=8:noet:
