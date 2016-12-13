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
uint8_t adc0_sequencer[ADC0_CHAN_COUNT] = {
	 4, 19|ADC_SC1_AIEN, 15|ADC_SC1_AIEN, 19|ADC_SC1_AIEN,   7, 19,  6, 19,
	12, 19, 13, 19,  14, 19,  5, 19
//	 4, 19, 15, 19,   7, 19,  6, 19,
//	12, 19, 13, 19,  14, 19,  5, 19
};
uint8_t adc1_sequencer[ADC1_CHAN_COUNT] = {
	19, 19, 19, 19,  19, 19, 19, 19,
	19, 19, 19, 19,  19, 19, 19, 19
//	19,  9, 19,  8,  19,  5, 19,  4,
//	19,  6, 19,  7,  19,  0 ,19,  3
};

volatile uint8_t adc0_results[ADC0_CHAN_COUNT] = {1, 2, 3, 4, 5, 6, 7, 8};
volatile uint8_t adc1_results[ADC1_CHAN_COUNT] = {3, 3, 3, 3, 4, 5, 6, 7};
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
	ADC##mod##_CFG1 = ADC_CFG1_ADIV(8) | ADC_CFG1_MODE(1) | ADC_CFG1_ADICLK(3) /*| ADC_CFG1_ADLSMP*/;\
	ADC##mod##_CFG2 	= ADC_CFG2_MUXSEL | ADC_CFG2_ADACKEN \
			| ADC_CFG2_ADHSC | ADC_CFG2_ADLSTS(2); \
	ADC##mod##_SC3 = 0u; \
	ADC##mod##_SC2	= ADC_SC2_DMAEN | 0x01u; \

/* Command sequencer
 * MUX source doesn't matter, we don't enable requests from it.
 */
#define DMA_SEQUENCER(adcidx, bufsz, tcd) \
	DMAMUX0_CHCFG##tcd	= DMAMUX_ENABLE | DMAMUX_SOURCE_ALWAYS##tcd; \
	DMA_TCD##tcd##_SADDR	= &adc##adcidx##_sequencer; \
	DMA_TCD##tcd##_SOFF	= 1; \
	DMA_TCD##tcd##_SLAST	= -(bufsz); \
	DMA_TCD##tcd##_DADDR	= &ADC##adcidx##_SC1A; \
	DMA_TCD##tcd##_DOFF 	= 0; \
	DMA_TCD##tcd##_DLASTSGA = 0; \
	DMA_TCD##tcd##_NBYTES_MLNO = 1; \
	DMA_TCD##tcd##_BITER_ELINKNO = ((bufsz) & DMA_TCD_BITER_MASK); \
	DMA_TCD##tcd##_CITER_ELINKNO = DMA_TCD##tcd##_BITER_ELINKNO; /* Current = start */ \
	DMA_TCD##tcd##_ATTR	= DMA_TCD_ATTR_SSIZE(DMA_TCD_ATTR_SIZE_8BIT) \
			| DMA_TCD_ATTR_DSIZE(DMA_TCD_ATTR_SIZE_8BIT); \
	DMA_TCD##tcd##_CSR	= 0; \

/*
 * Results transporter. CAN ONLY BE ONE OF THE LOWER 4 TCDs on mk20dx128!
 * No link at the end of the loop - we started sequencer manually.
 * So it's at initial position at the end of our major loop.
 */
#define DMA_RESULT_GATHERER(adcidx, bufsz, tcd, seqc) \
	DMAMUX0_CHCFG##tcd	= DMAMUX_ENABLE | DMAMUX_SOURCE_ADC##adcidx; \
	DMA_TCD##tcd##_SADDR	= &ADC##adcidx##_RA; \
	DMA_TCD##tcd##_SOFF	= 0; \
	DMA_TCD##tcd##_SLAST	= 0; \
	DMA_TCD##tcd##_DADDR	= &adc##adcidx##_results; \
	DMA_TCD##tcd##_DOFF	= 1; \
	DMA_TCD##tcd##_DLASTSGA = -(bufsz); \
	DMA_TCD##tcd##_NBYTES_MLNO = 1; \
	DMA_TCD##tcd##_BITER_ELINKYES = DMA_TCD_BITER_ELINK \
				| ((seqc << 9) & 0x1E00u) \
				| ((bufsz) & DMA_TCD_BITER_MASK); \
	DMA_TCD##tcd##_CITER_ELINKYES = DMA_TCD##tcd##_BITER_ELINKYES; /*Current = start */ \
	DMA_TCD##tcd##_ATTR	= DMA_TCD_ATTR_SSIZE(DMA_TCD_ATTR_SIZE_8BIT) \
			| DMA_TCD_ATTR_DSIZE(DMA_TCD_ATTR_SIZE_8BIT); \
	DMA_TCD##tcd##_CSR	= DMA_TCD_CSR_INTMAJOR; \
	NVIC_ENABLE_IRQ(IRQ_DMA_CH##tcd); \
	DMA_ERQ		|= DMA_ERQ_ERQ##tcd;

#define START_DMA(idx) \
	DMA_TCD##idx##_CITER_ELINKNO = DMA_TCD##idx##_BITER_ELINKNO; \
	DMA_TCD##idx##_CSR |= DMA_TCD_CSR_START; \

void dma_ch1_isr(void)
{
	cli();
	DMA_CINT |= DMA_CINT_CINT(1);
	CoCo1 = 1;
	sei();
}

void dma_ch3_isr(void)
{
	cli();
	DMA_CINT |= DMA_CINT_CINT(3);
	CoCo2 = 1;
	sei();
}

#define SENSOR_SETUP(idx, ch_count, seq_tcd, res_tcd) \
	ADC_CALIBRATE(idx) \
	ADC_SETUP(idx) \
	DMA_SEQUENCER(idx, ch_count, seq_tcd) \
	DMA_RESULT_GATHERER(idx, ch_count, res_tcd, seq_tcd) \

uint8_t ADC_Setup()
{
	// Power
	SIM_SCGC3 |= SIM_SCGC3_ADC1;
	SIM_SCGC6 |= SIM_SCGC6_DMAMUX | SIM_SCGC6_ADC0;
	SIM_SCGC7 |= SIM_SCGC7_DMA;
	// ADC0 - DMA channels 0(seq) and 1(results)
	SENSOR_SETUP(0, ADC0_CHAN_COUNT, 0, 1)

	// ADC1 - DMA channels 2(seq) and 3(results)
	//SENSOR_SETUP(1, ADC1_CHAN_COUNT, 2, 3)
	CoCo1 = 0; CoCo2 = 0;
	return 0;
}

inline void PIN_Setup()
{
	SIM_SCGC5 |= SIM_SCGC5_PORTA | SIM_SCGC5_PORTB | SIM_SCGC5_PORTC | SIM_SCGC5_PORTD | SIM_SCGC5_PORTE;
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
	ADC_Setup();
	PIN_Setup();
}

// Main Detection Loop
inline uint8_t Scan_loop()
{
	/*
	sei(); // Give it back, main thread!
	for (uint8_t i=0; i < 8; i++)
	{
		Matrix_sense(1);
		Matrix_strobe(i, 1);
		delay(5);
		START_DMA(0)
		delay(25);
		START_DMA(2)
		delay(25);
		while (CoCo1 + CoCo2 < 2) {};
		CoCo1 = CoCo2 = 0;
		delay(200);
		Matrix_sense(0);
		Matrix_strobe(i, 0);
	}
	Matrix_scan( Scan_scanCount++ );
	*/
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
	print( NL );
	for (uint32_t j = 0; j < 1; j++)
	{
	for (uint8_t i = 0; i < 8; i++)
	{
		Matrix_sense(1);
		Matrix_strobe(i, 1);
		//delayMicroseconds(100);
		CoCo1 = 0; CoCo2 = 0;
		uint32_t c1 = 1000;
		uint32_t c2 = 1000;
		//START_DMA(2)
		START_DMA(0)
		//while (c > 0 && CoCo1 + CoCo2 < 2) {c--;};
		while (c1 > 0 && CoCo1 == 0) {c1--;};
		//while (c2 > 0 && CoCo2 == 0) {c2--;};
		if (c1 == 0 || c2 == 0)
		{
			print( "\n\n\n\n!!!!\n" );
			printHex( DMA_TCD0_CITER_ELINKNO);
			print( " " );
			printHex( DMA_TCD1_CITER_ELINKNO);
			print( " " );
			printHex( DMA_TCD2_CITER_ELINKNO);
			print( " " );
			printHex( DMA_TCD3_CITER_ELINKNO);
			print( NL );
			printInt32( j );
			print( " " );
			printInt32( i );
			print( " " );
			printInt32(c1);
			print( " " );
			printInt32(c2);
			print( "\n\n\n");
		}
		Matrix_sense(0);
		Matrix_strobe(i, 0);
		cliFunc_ADCPrint("");
		delayMicroseconds(100);
	}
	if (j % 1000 == 0)
	{
		print ( ".");
	}
	//delay(200);
	}

}

void cliFunc_ADCPrint( char* args )
{
	print( NL );
	printHex( DMA_TCD0_CITER_ELINKNO);
	print( " " );
	printHex( DMA_TCD1_CITER_ELINKNO);
	print( " " );
	printHex( DMA_TCD2_CITER_ELINKNO);
	print( " " );
	printHex( DMA_TCD3_CITER_ELINKNO);
	print( " " );
	for (uint8_t i=0; i < ADC0_CHAN_COUNT; i++)
	{
		printInt8(adc0_results[i]);
		adc0_results[i] = i;
		print (" ");
	}
	print ("| ");
	for (uint8_t i=0; i < ADC1_CHAN_COUNT; i++)
	{
		printInt8(adc1_results[i]);
		adc1_results[i] = i;
		print (" ");
	}
	print( NL );
	printHex(ADC0_SC1A);
	print( NL );
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
