//************************************************************************
//*	pic32bootloader.c
//*
//*		This bootloader impliments AVR stk500 v2 protocol
//*		for use with avrdude program which is cross platform
//*
//*		(C) 2010, 2011 by Mark Sproul
//************************************************************************
//*	this code is based on code by Peter Fleury <pfleury@gmx.ch>   http://jump.to/fleury
//*
//*	This library is free software; you can redistribute it and/or
//*	modify it under the terms of the GNU Lesser General Public
//*	License as published by the Free Software Foundation; either
//*	version 2.1 of the License, or (at your option) any later version.
//*
//*	This library is distributed in the hope that it will be useful,
//*	but WITHOUT ANY WARRANTY; without even the implied warranty of
//*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.//*	See the GNU
//*	Lesser General Public License for more details.
//*
//*	You should have received a copy of the GNU Lesser General
//*	Public License along with this library; if not, write to the
//*	Free Software Foundation, Inc., 59 Temple Place, Suite 330,
//*	Boston, MA	02111-1307	USA
//*
//*
//************************************************************************
//*	Edit History
//************************************************************************
//*	Oct 20,	2010	<MLS> Started on bootloader
//*	Feb  5,	2011	<MLS> added LED blink while waiting for bootloader to start
//*	Mar  1,	2011	Jason Kajita of Microchip helped with the linker scripts
//*	Apr  7,	2011	<MLS> Bootloader working, it really worked all along, just had to get the hex files linked properly
//*	Apr 11,	2011	<MLS> Mega working prefectly, Still having problems with the UNO board
//*	Apr 12,	2011	<MLS> Now working on uno, stack ptr was not being set right.
//*	Apr 14,	2011	<ML>S Updated config bits as per Gene
//*	Apr 24,	2011	<MLS> Working on reducing memory size
//*	Apr 24,	2011	<MLS> Added Version number, Version 1.0
//*	Apr 24,	2011	<MLS> Added chip erase, only erases blocks that have non-0xff data
//*	Apr 24,	2011	<MLS> Released version 1.0 to Digilent
//*	May 26,	2011	<MLS> Added options for various Microchip Starter kits
//*	Jun  7,	2011	<MLS> Added support for Digilent Cerebot board
//*	Jun 14,	2011	<MLS> Added support for MikroElectronica boards (http://www.mikroe.com/)
//*	Jun 15,	2011	<MLS> Added support for Explorer16, it uses Uart2, the DB9 connector
//*	Jun 15,	2011	<MLS> Added 2nd LED to indicate in process of downloading
//*	Jun 15,	2011	<MLS> Version 1.1
//************************************************************************

#define	_VERSION_MAJOR_		1
#define	_VERSION_MINOR_		1
#define	_VERSION_STRING_	"V1.1"

#define _USE_RELEASE_OPTIONS_

//************************************************************************
//*	NOTE!!!!!
//*		Some of these #defines are defined in the MPLAB-X option configurations
//*			Project Properties
//*				pic32-gcc
//*					Preprossor macros
//************************************************************************

//*	the board definitions are set in MPLAB-X properties -> pic32-gcc -> Preprocessor macros

//************************************************************************
#if defined(_BOARD_DIGILENT_UNO_)

	//*	portG bit 6
 	#define	_LED_ON_PORT_G
 	#define	_LED_BIT_	BIT_6

	#define	_BOARD_HAS_2ND_LED_
 	#define	_LED2_ON_PORT_F
 	#define	_LED2_BIT_	BIT_0


//************************************************************************
#elif defined(_BOARD_DIGILENT_MEGA_)

	//*	portA bit 3
 	#define	_LED_ON_PORT_A
 	#define	_LED_BIT_	BIT_3

//************************************************************************
#elif defined(_BOARD_PIC32_STARTER_KIT_) && defined(__32MX360F512L__)

	//*	portD bit 1
 	#define	_LED_ON_PORT_D
 	#define	_LED_BIT_	BIT_1

	#define	_BOARD_HAS_2ND_LED_
 	#define	_LED2_ON_PORT_D
 	#define	_LED2_BIT_	BIT_2

//************************************************************************
#elif defined(_BOARD_ETHERNET_STARTER_KIT_) && defined(__32MX795F512L__)

	//*	portD bit 1
 	#define	_LED_ON_PORT_D
 	#define	_LED_BIT_	BIT_1

//************************************************************************
#elif defined(_BOARD_PIC32_EXPLORER16_)

	#warning EXPLORER16 uses UART2 for bootloader
	
	#define	_USE_UART2_FOR_BOOTLOADER_
	
 	#define	_LED_ON_PORT_A
 	#define	_LED_BIT_	BIT_2

	#define	_BOARD_HAS_2ND_LED_
 	#define	_LED2_ON_PORT_A
 	#define	_LED2_BIT_	BIT_3


//************************************************************************
#elif defined(_BOARD_CEREBOT_32MX4_)
 
 	#define	_LED_ON_PORT_B
 	#define	_LED_BIT_	BIT_10

//************************************************************************
#elif defined(_BOARD_MIKROE_MULTIMEDIA_)
 
 	#define	_LED_ON_PORT_A
 	#define	_LED_BIT_	BIT_3

	#define	_BOARD_HAS_2ND_LED_
 	#define	_LED2_ON_PORT_A
 	#define	_LED2_BIT_	BIT_2

	#define	_LEDS_INVERTED_
	
	#define _USE_WORD_WRITE_
//************************************************************************
#elif defined(__32MX320F064H__) || defined(__32MX320F128H__)
	#define	_BOARD_DIGILENT_UNO_

//************************************************************************
#elif defined(__32MX795F512L__)
	//*	mega board ready for release
	#define	_BOARD_DIGILENT_MEGA_

//************************************************************************
#elif defined(__32MX360F512L__)
	#define	_BOARD_PIC32_STARTER_KIT_
#else
	#error	Board/CPU combination not defined
#endif




#ifdef _USE_RELEASE_OPTIONS_
	#define	_ENABLE_MONITOR_
//	#define	_ENABLE_PORT_LIST_
//	#define _ENABLE_PORT_BLINK_
#else
	#warning NOT-FOR-RELEASE-VERSION

	//#define	_ENABLE_MONITOR_

	#if defined(__32MX795F512L__) && !defined(_ENABLE_MONITOR_)
		#define	DEBUG_VIA_SERIAL_AUX
	#endif
	//#define	_DEBUG_SERIAL_
	//#define	_DEBUG_WITH_LEDS_


	#ifdef DEBUG_VIA_SERIAL_AUX
		//#define	_USE_UART2_FOR_DEBUG_
		#define		_USE_UART3A_FOR_DEBUG_
		#define BAUDRATE_SERIAL2 230400
	#endif
#endif

#if defined(_USE_UART2_FOR_BOOTLOADER_) && defined(_USE_UART2_FOR_DEBUG_)
	#error	Cannot use UART2 for bootloader AND DEBUG at the same time
#endif

#define		_ENABLE_HELP_MESSAGES_


#include <plib.h>
#include <p32xxxx.h>


#include	"cpudefs.h"

//************************************************************************
//** CONFIGURATION
//************************************************************************
//#pragma config POSCMOD=XT, FNOSC=PRIPLL
//#pragma config FPLLIDIV=DIV_2, FPLLMUL=MUL_20, FPLLODIV=DIV_1
//#pragma config FPBDIV=DIV_2, FWDTEN=OFF, CP=OFF, BWP=OFF
#if !defined(__PIC32MX__)
	#error "This project was designed for PIC32MX family devices. Please select the appropriate project for your target device family."
#endif

	//************************************************************************
	//*	These config bits are common to all CPUS

	//*	Oscillator Settings
	#pragma config	FNOSC		=	PRIPLL		// Oscillator selection
	#pragma config	POSCMOD		=	XT			// Primary oscillator mode
	#pragma config	FPLLIDIV 	=	DIV_2		// PLL input divider
	#pragma config	FPLLMUL		=	MUL_20		// PLL multiplier
	#pragma config	FPLLODIV 	=	DIV_1		// PLL output divider
	#pragma config	FPBDIV		=	DIV_8		// Peripheral bus clock divider
//	#pragma config	FSOSCEN		=	ON			// Secondary oscillator enable

	//*	Clock control settings
	#pragma config	IESO		=	OFF			// Internal/external clock switchover
	#pragma config	FCKSM		=	CSDCMD		// Clock switching (CSx)/Clock monitor (CMx)
	#pragma config	OSCIOFNC	=	OFF			// Clock output on OSCO pin enable

	//*	Other Peripheral Device settings
	#pragma config	FWDTEN		=	OFF			// Watchdog timer enable
	#pragma config	WDTPS		=	PS1024		// Watchdog timer postscaler

	//*	Code Protection settings
	#pragma config	CP			=	OFF			// Code protection
	#pragma config	BWP			=	OFF			// Boot flash write protect
	#pragma config	PWP			=	OFF			// Program flash write protect

	//*	Debug settings
	#pragma config	ICESEL		=	ICS_PGx2	// ICE pin selection
//	#pragma config	DEBUG		=	OFF			// Debug mode select

#if defined(__32MX320F064H__) ||  defined(__32MX320F128H__) || defined(__32MX360F512L__) || defined(__32MX460F512L__)
	//************************************************************************
	//*		PIC32MX3XX Configuration Settings
	//************************************************************************

	//*	Oscillator Settings
	#pragma config	FSOSCEN		=	ON			// Secondary oscillator enable


	//*	Other Peripheral Device settings

	//************************************************************************

#elif defined(__32MX795F512L__)
	//************************************************************************
	//*		PIC32MX7XX Configuration Settings
	//************************************************************************

	//*	Oscillator Settings
	#pragma config	FSOSCEN		=	OFF			// Secondary oscillator enable


	//*	Other Peripheral Device settings
//	#pragma config	FSRSSEL		=	PRIORITY_0	// SRS interrupt priority
	#pragma config	FSRSSEL		=	PRIORITY_7	// SRS interrupt priority
	#pragma config	FCANIO		=	OFF			// Standard/alternate CAN pin select (OFF=Alt)
	#pragma config	FETHIO		=	ON			// Standard/alternate ETH pin select (OFF=Alt)
	#pragma config	FMIIEN		=	OFF			// MII/RMII select (OFF=RMII)


	//*	USB Settings
	#pragma config	UPLLEN		=	ON			// USB PLL enable
	#pragma config	UPLLIDIV	=	DIV_2		// USB PLL input divider
	#pragma config	FVBUSONIO	=	OFF			// VBUS pin control
	#pragma config	FUSBIDIO	=	OFF			// USBID pin control

	//************************************************************************
#else
	#warning CPU config bits not set for this CPU
#endif

/*

	#pragma config	FPLLODIV	=	DIV_1		//	PLL Output Divider
#if defined(__32MX795F512L__)
	#pragma config	UPLLEN		=	ON			//	USB PLL Enabled
	#pragma config	UPLLIDIV	=	DIV_2		//	USB PLL Input Divider
#endif
	#pragma config	FPLLMUL		=	MUL_20		//	PLL Multiplier
	#pragma config	FPLLIDIV	=	DIV_2		//	PLL Input Divider
	#pragma config	FWDTEN		=	OFF			//	Watchdog Timer
	#pragma config	FPBDIV		=	DIV_1		//	Peripheral Clock divisor
	#pragma config	WDTPS		=	PS1			//	Watchdog Timer Postscale
	#pragma config	FCKSM		=	CSECME		//	Clock Switching & Fail Safe Clock Monitor enabled
	#pragma config	OSCIOFNC	=	OFF			//	CLKO Enable
	#pragma config	POSCMOD		=	HS			//	Primary Oscillator
	#pragma config	IESO		=	ON			//	Internal/External Switch-over
	#pragma config	FSOSCEN		=	OFF			//	Secondary Oscillator Enable (KLO was off)
	#pragma config	FNOSC		=	PRIPLL		//	Oscillator Selection
	#pragma config	CP			=	OFF			//	Code Protect
	#pragma config	BWP			=	OFF			//	Boot Flash Write Protect
	#pragma config	PWP			=	OFF			//	Program Flash Write Protect
	#pragma config	ICESEL		=	ICS_PGx1	//	ICE/ICD Comm Channel Select
//	#pragma config	ICESEL		=	ICS_PGx2	//	ICE/ICD Comm Channel Select
	//#pragma config DEBUG		=	OFF			//	Background Debugger Enable
*/

#include	<inttypes.h>
#include	<stdlib.h>
#include	"command.h"


#define	true	1
#define	false	0



/*
 * Comment the following lines to save code space
 */
//#define	INCLUDE_PROGRAM_LOCK_BIT_SUPPORT		// enable program lock bits





/*
 * define CPU frequency in Mhz here if not defined in Makefile
 */
#ifndef F_CPU
	#define F_CPU 80000000UL
#endif

#define	_BLINK_LOOP_COUNT_	(F_CPU / 5000)
/*
 * UART Baudrate, AVRStudio AVRISP only accepts 115200 bps
 */

#ifndef BAUDRATE
	#define BAUDRATE 115200
#endif


/*
 * HW and SW version, reported to AVRISP, must match version of AVRStudio
 */
#define CONFIG_PARAM_BUILD_NUMBER_LOW	0
#define CONFIG_PARAM_BUILD_NUMBER_HIGH	0
#define CONFIG_PARAM_HW_VER				0x0F
#define CONFIG_PARAM_SW_MAJOR			2
#define CONFIG_PARAM_SW_MINOR			0x0A



//*
//*	Signature bytes are not available in avr-gcc io_xxx.h
//*
//#define SIGNATURE_BYTES 0x1E9801
//						"PIC"
#define SIGNATURE_BYTES 0x504943

//*	function prototypes
static	void	RunMonitor(void);
static	void	delay_microSeconds(unsigned long microSeconds);
static	void	delay_ms(unsigned long ms);
static	void	BootLED_Toggle();
static	void	PrintDecInt(int theNumber, int digitCnt);
static	void	JumpToApp();
static void 	DownloadLED_Off();


//?#define BL_CSUM_ADDR 		0x9D0017F4
//?#define BL_PROGSZ_ADDR 		0x9D0017F0
#define FLASH_PROG_BASE		0x9D000000
#define USER_APP_ADDR 		0x9D001000
//?#define FLASH_PAGE_SIZE		4096

unsigned char	*gFlashPtr;
unsigned char	*gEBASEptr;

//************************************************************************
// Let compile time pre-processor calculate the CORE_TICK_PERIOD
//	clock rate is 80000000ull
#define TOGGLES_PER_SEC			1000
#define CORE_TICK_RATE			(F_CPU / 2 / TOGGLES_PER_SEC)

//************************************************************************
//*	globals
unsigned long			__PIC32_pbClk;


/*
 * States used in the receive state machine
 */
#define	ST_START		0
#define	ST_GET_SEQ_NUM	1
#define ST_MSG_SIZE_1	2
#define ST_MSG_SIZE_2	3
#define ST_GET_TOKEN	4
#define ST_GET_DATA		5
#define	ST_GET_CHECK	6
#define	ST_PROCESS		7




//************************************************************************
static int 	Serial_Available(void)
{
#ifdef _USE_UART2_FOR_BOOTLOADER_
	return(U2STA & 1);	// is there any data
#else
	return(U1STA & 1);	// is there any data
#endif
}


//*****************************************************************************
//*	 Read single byte from USART, block if no data available
//*****************************************************************************
static unsigned char  Serial_read(void)
{
#ifdef _USE_UART2_FOR_BOOTLOADER_
	while (!(U2STA & 1))
	{
		// wait for data
	}
	return (U2RXREG);
#else
	while (!(U1STA & 1))
	{
		// wait for data
	}
	return (U1RXREG);
#endif
}

#define	MAX_TIME_COUNT	(F_CPU >> 1)
//*****************************************************************************
static unsigned char  Serial_read_timeout(void)
{
uint32_t count = 0;

#ifdef _USE_UART2_FOR_BOOTLOADER_
	if((U2STA & 0x000E) != 0)
	{
	BYTE	dummy;

		dummy			=	U2RXREG; 	//dummy read to clear FERR/PERR
		U2STAbits.OERR	=	0;			//clear OERR to keep receiving
	}

	while (!(U2STA & 1))
	{
		// wait for data
		count++;
		if (count > MAX_TIME_COUNT)
		{
			//*	jump to user program
			JumpToApp();
		}
	}
	return (U2RXREG);
#else
	if((U1STA & 0x000E) != 0)
	{
	BYTE	dummy;

		dummy			=	U1RXREG; 	//dummy read to clear FERR/PERR
		U1STAbits.OERR	=	0;			//clear OERR to keep receiving
	}

	while (!(U1STA & 1))
	{
		// wait for data
		count++;
		if (count > MAX_TIME_COUNT)
		{
			//*	jump to user program
			JumpToApp();
		}
	}
	return (U1RXREG);
#endif
}

//*****************************************************************************
//*	send single byte to USART, wait until transmission is completed
//*****************************************************************************
static void   Serial_write(char theChar)
{
#ifdef _USE_UART2_FOR_BOOTLOADER_
	while (! U2STAbits.TRMT)
	{
		//*	wait for the buffer to be clear
	}
	U2TXREG	=	theChar;
#else
	while (! U1STAbits.TRMT)
	{
		//*	wait for the buffer to be clear
	}
	U1TXREG	=	theChar;
#endif
}

//************************************************************************
static void 	Serial_print(const char *textString)
{
char			theChar;
unsigned int	ii;

	theChar		=	1;
	ii			=	0;
	while (theChar != 0)
	{
		theChar	=	textString[ii];
		if (theChar != 0)
		{
			Serial_write(theChar);
		}
		ii++;
	}
}

//************************************************************************
static void 	Serial_println(void)
{
	Serial_print("\x0d\x0a");
}

#ifdef DEBUG_VIA_SERIAL_AUX


//*****************************************************************************
static void  Serial2_begin(long buadRate)
{
#ifdef _USE_UART2_FOR_DEBUG_
	U2MODE				=	(UART_EN);
	U2STA				=	(UART_RX_ENABLE | UART_TX_ENABLE);
	U2BRG				=	(__PIC32_pbClk / 16 / (buadRate - 1));	// calculate actual BAUD generate value.
	U2MODEbits.UARTEN	=	0x01;
	U2STAbits.UTXEN		=	0x01;
#elif defined _USE_UART3A_FOR_DEBUG_
	U3AMODE				=	(UART_EN);
	U3ASTA				=	(UART_RX_ENABLE | UART_TX_ENABLE);
	U3ABRG				=	(__PIC32_pbClk / 16 / (buadRate - 1));	// calculate actual BAUD generate value.
	U3AMODEbits.UARTEN	=	0x01;
	U3ASTAbits.UTXEN		=	0x01;
#endif
}

//*****************************************************************************
static void  Serial2_write(char theChar)
{
#ifdef _USE_UART2_FOR_DEBUG_
	while (! U2STAbits.TRMT)
	{
		//*	wait for the buffer to be clear
	}
	U2TXREG	=	theChar;
#elif defined _USE_UART3A_FOR_DEBUG_
	while (! U3ASTAbits.TRMT)
	{
		//*	wait for the buffer to be clear
	}
	U3ATXREG	=	theChar;
#else
	#error AUX serial port not defined
#endif
}

//************************************************************************
static void 	Serial2_PrintHexByte(unsigned char theByte)
{
char	theChar;

	theChar	=	0x30 + ((theByte >> 4) & 0x0f);
	if (theChar > 0x39)
	{
		theChar	+=	7;
	}
	Serial2_write(theChar );

	theChar	=	0x30 + (theByte & 0x0f);
	if (theChar > 0x39)
	{
		theChar	+=	7;
	}
	Serial2_write(theChar );
}

//************************************************************************
static void 	Serial2_PrintLongWordHex(uint32_t longWord)
{
	Serial2_PrintHexByte((longWord >> 24) & 0x0ff);
	Serial2_PrintHexByte((longWord >> 16) & 0x0ff);
	Serial2_PrintHexByte((longWord >> 8) & 0x0ff);
	Serial2_PrintHexByte(longWord & 0x0ff);
}


//*****************************************************************************
static void  Serial2_print(char *textString)
{
char			theChar;
unsigned int	ii;

	theChar		=	1;
	ii			=	0;
	while (theChar != 0)
	{
		theChar	=	textString[ii];
		if (theChar != 0)
		{
			Serial2_write(theChar);
		}
		ii++;
	}
}
	
//*****************************************************************************
static void  Serial2_println(void)
{
	Serial2_write(0x0d);
	Serial2_write(0x0a);
}
#endif		//	DEBUG_VIA_SERIAL_AUX

#pragma mark -

//*****************************************************************************
//*	this routine was taken from the Microchip bootloader
static void  JumpToApp()
{
void			(*fptr)(void);
unsigned long	firstLongWord;

#ifdef _BOARD_HAS_2ND_LED_
	DownloadLED_Off();
#endif


#ifdef DEBUG_VIA_SERIAL_AUX
	Serial2_print("JumpToApp");
	Serial2_println();
#endif

#if defined(_DEBUG_WITH_LEDS_)
unsigned int ii;

	for (ii=0; ii<=12; ii++)
	{
		BootLED_Toggle();
		delay_ms(50);
	}
#endif


	fptr			=	(void (*)(void))USER_APP_ADDR;
	firstLongWord	=	(unsigned long)*((unsigned long *)USER_APP_ADDR);
	


#ifdef DEBUG_VIA_SERIAL_AUX
	Serial2_PrintLongWordHex(firstLongWord);
#endif

	if (firstLongWord != (unsigned long)0xFFFFFFFF)
	{
	#ifdef DEBUG_VIA_SERIAL_AUX
		Serial2_print("Jumping");
		Serial2_println();
	#endif
		fptr();
	}
#ifdef DEBUG_VIA_SERIAL_AUX
	else
	{
		Serial2_print("Not Jumping");
		Serial2_println();
	}
#endif
}



//************************************************************************
static	void  delay_microSeconds(unsigned long microSeconds)
{
volatile unsigned long	tickCount;

//	tickCount	=	(microSeconds * (F_CPU / 1000000)) / 5;
	tickCount	=	(microSeconds * (F_CPU / 1000000)) / 9;

	while (tickCount > 0)
	{
		__asm__ volatile ("nop");
		tickCount--;
	}
}

//************************************************************************
static	void  delay_ms(unsigned long ms)
{
unsigned long	ii;

	for (ii = 0; ii< ms; ii++)
	{
		delay_microSeconds(1000);
	}
}


//*****************************************************************************
//*	if a particular board does NOT have an LED, these routines should just do nothing
//*****************************************************************************
static void  	BootLED_Init()
{
#if defined(_LED_ON_PORT_A) && defined(_LED_BIT_)
	mPORTASetPinsDigitalOut(_LED_BIT_);
	#ifdef _LEDS_INVERTED_
		mPORTAClearBits(_LED_BIT_);				//*	set pins as output
	#else
		mPORTASetBits(_LED_BIT_);				//*	set pins as output
	#endif
#elif defined(_LED_ON_PORT_B) && defined(_LED_BIT_)
	mPORTBClearBits(_LED_BIT_);				//*	set pins as output
	mPORTBSetPinsDigitalOut(_LED_BIT_);

#elif defined(_LED_ON_PORT_C) && defined(_LED_BIT_)
	mPORTCClearBits(_LED_BIT_);				//*	set pins as output
	mPORTCSetPinsDigitalOut(_LED_BIT_);

#elif defined(_LED_ON_PORT_D) && defined(_LED_BIT_)
	mPORTDClearBits(_LED_BIT_);				//*	set pins as output
	mPORTDSetPinsDigitalOut(_LED_BIT_);

#elif defined(_LED_ON_PORT_E) && defined(_LED_BIT_)
	mPORTEClearBits(_LED_BIT_);				//*	set pins as output
	mPORTESetPinsDigitalOut(_LED_BIT_);

#elif defined(_LED_ON_PORT_F) && defined(_LED_BIT_)
	mPORTFClearBits(_LED_BIT_);				//*	set pins as output
	mPORTFSetPinsDigitalOut(_LED_BIT_);

#elif defined(_LED_ON_PORT_G) && defined(_LED_BIT_)
	mPORTGClearBits(_LED_BIT_);				//*	set pins as output
	mPORTGSetPinsDigitalOut(_LED_BIT_);

#elif defined(_BOARD_DIGILENT_UNO_)
	//*	portG bit 6
	mPORTGSetPinsDigitalOut(BIT_6);			//*	set pins as output
	mPORTGClearBits(BIT_6);
#elif defined(_BOARD_DIGILENT_MEGA_)
	//*	portA bit 3
	mPORTAClearBits(BIT_3);
	mPORTASetPinsDigitalOut(BIT_3);			//*	set pins as output
#elif defined(_BOARD_PIC32_STARTER_KIT_) || defined(_BOARD_ETHERNET_STARTER_KIT_)
	//*	portD bit 1
	mPORTDClearBits(BIT_1);
	mPORTDSetPinsDigitalOut(BIT_1);
#elif defined(_BOARD_CEREBOT_32MX4_)
	//*	portB bit 10
	mPORTBClearBits(BIT_10);
	mPORTBSetPinsDigitalOut(BIT_10);
#else
	#warning board not defined
	mPORTDClearBits(BIT_0 | BIT_1 | BIT_2);
	mPORTDSetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2);		//*	set pins as output
#endif
}

//*****************************************************************************
static void	 BootLED_Toggle()
{
#if defined(_LED_ON_PORT_A) && defined(_LED_BIT_)
	mPORTAToggleBits(_LED_BIT_);

#elif defined(_LED_ON_PORT_B) && defined(_LED_BIT_)
	mPORTBToggleBits(_LED_BIT_);

#elif defined(_LED_ON_PORT_C) && defined(_LED_BIT_)
	mPORTCToggleBits(_LED_BIT_);

#elif defined(_LED_ON_PORT_D) && defined(_LED_BIT_)
	mPORTDToggleBits(_LED_BIT_);

#elif defined(_LED_ON_PORT_E) && defined(_LED_BIT_)
	mPORTEToggleBits(_LED_BIT_);

#elif defined(_LED_ON_PORT_F) && defined(_LED_BIT_)
	mPORTFToggleBits(_LED_BIT_);

#elif defined(_LED_ON_PORT_G) && defined(_LED_BIT_)
	mPORTGToggleBits(_LED_BIT_);

#elif defined(_BOARD_DIGILENT_UNO_)
//#error foo2
	mPORTGToggleBits(BIT_6);
#elif defined(_BOARD_DIGILENT_MEGA_)
	mPORTAToggleBits(BIT_3);
#elif defined(_BOARD_PIC32_STARTER_KIT_) || defined(_BOARD_ETHERNET_STARTER_KIT_)
	//*	portD bit 1
	mPORTDToggleBits(BIT_1);
#elif defined(_BOARD_CEREBOT_32MX4_)
	//*	portB bit 10
	mPORTBToggleBits(BIT_10);
#else
	mPORTDToggleBits(BIT_0);
#endif
}

//*****************************************************************************
static void 	BootLED_Off()
{
#if defined(_LED_ON_PORT_A) && defined(_LED_BIT_)
	#ifdef _LEDS_INVERTED_
		mPORTASetBits(_LED_BIT_);
	#else
		mPORTAClearBits(_LED_BIT_);
	#endif
	
#elif defined(_LED_ON_PORT_B) && defined(_LED_BIT_)
	mPORTBClearBits(_LED_BIT_);

#elif defined(_LED_ON_PORT_C) && defined(_LED_BIT_)
	mPORTCClearBits(_LED_BIT_);

#elif defined(_LED_ON_PORT_D) && defined(_LED_BIT_)
	mPORTDClearBits(_LED_BIT_);

#elif defined(_LED_ON_PORT_E) && defined(_LED_BIT_)
	mPORTEClearBits(_LED_BIT_);

#elif defined(_LED_ON_PORT_F) && defined(_LED_BIT_)
	mPORTFClearBits(_LED_BIT_);

#elif defined(_LED_ON_PORT_G) && defined(_LED_BIT_)
	mPORTGClearBits(_LED_BIT_);

#elif defined(_BOARD_DIGILENT_UNO_)
//#error foo3
	mPORTGClearBits(BIT_6);
#elif defined(_BOARD_DIGILENT_MEGA_)
	mPORTAClearBits(BIT_3);
#elif defined(_BOARD_PIC32_STARTER_KIT_) || defined(_BOARD_ETHERNET_STARTER_KIT_)
	//*	portD bit 1
	mPORTDClearBits(BIT_1);
#elif defined(_BOARD_CEREBOT_32MX4_)
	//*	portB bit 10
	mPORTBClearBits(BIT_10);
#else
	#warning board not defined
	mPORTDClearBits(BIT_0 | BIT_1 | BIT_2);
#endif
}


#ifdef _BOARD_HAS_2ND_LED_
//*****************************************************************************
static void  	DownloadLED_Init()
{
#if defined(_LED2_ON_PORT_A) && defined(_LED2_BIT_)
	mPORTASetPinsDigitalOut(_LED2_BIT_);			//*	set pins as output

	#ifdef _LEDS_INVERTED_
		mPORTASetBits(_LED2_BIT_);					
	#else
		mPORTAClearBits(_LED2_BIT_);				
	#endif

#elif defined(_LED2_ON_PORT_B) && defined(_LED2_BIT_)
	mPORTBClearBits(_LED2_BIT_);				
	mPORTBSetPinsDigitalOut(_LED2_BIT_);		//*	set pins as output

#elif defined(_LED2_ON_PORT_C) && defined(_LED2_BIT_)
	mPORTCClearBits(_LED2_BIT_);				
	mPORTCSetPinsDigitalOut(_LED2_BIT_);		//*	set pins as output

#elif defined(_LED2_ON_PORT_D) && defined(_LED2_BIT_)
	mPORTDClearBits(_LED2_BIT_);				
	mPORTDSetPinsDigitalOut(_LED2_BIT_);		//*	set pins as output

#elif defined(_LED2_ON_PORT_E) && defined(_LED2_BIT_)
	mPORTEClearBits(_LED2_BIT_);				
	mPORTESetPinsDigitalOut(_LED2_BIT_);		//*	set pins as output

#elif defined(_LED2_ON_PORT_F) && defined(_LED2_BIT_)
	mPORTFClearBits(_LED2_BIT_);				
	mPORTFSetPinsDigitalOut(_LED2_BIT_);		//*	set pins as output

#elif defined(_LED2_ON_PORT_G) && defined(_LED2_BIT_)
	mPORTGClearBits(_LED2_BIT_);				
	mPORTGSetPinsDigitalOut(_LED2_BIT_);		//*	set pins as output

#endif
}

//*****************************************************************************
static void 	DownloadLED_On()
{
#if defined(_LED2_ON_PORT_A) && defined(_LED2_BIT_)
	#ifdef _LEDS_INVERTED_
		mPORTAClearBits(_LED2_BIT_);
	#else
		mPORTASetBits(_LED2_BIT_);
	#endif

#elif defined(_LED2_ON_PORT_B) && defined(_LED2_BIT_)
	mPORTBSetBits(_LED2_BIT_);

#elif defined(_LED2_ON_PORT_C) && defined(_LED2_BIT_)
	mPORTCSetBits(_LED2_BIT_);

#elif defined(_LED2_ON_PORT_D) && defined(_LED2_BIT_)
	mPORTDSetBits(_LED2_BIT_);

#elif defined(_LED2_ON_PORT_E) && defined(_LED2_BIT_)
	mPORTESetBits(_LED2_BIT_);

#elif defined(_LED2_ON_PORT_F) && defined(_LED2_BIT_)
	mPORTFSetBits(_LED2_BIT_);

#elif defined(_LED2_ON_PORT_G) && defined(_LED2_BIT_)
	mPORTGSetBits(_LED2_BIT_);

#endif
}


//*****************************************************************************
static void 	DownloadLED_Off()
{
#if defined(_LED2_ON_PORT_A) && defined(_LED2_BIT_)
	#ifdef _LEDS_INVERTED_
		mPORTASetBits(_LED2_BIT_);
	#else
		mPORTAClearBits(_LED2_BIT_);
	#endif

#elif defined(_LED2_ON_PORT_B) && defined(_LED2_BIT_)
	mPORTBClearBits(_LED2_BIT_);

#elif defined(_LED2_ON_PORT_C) && defined(_LED2_BIT_)
	mPORTCClearBits(_LED2_BIT_);

#elif defined(_LED2_ON_PORT_D) && defined(_LED2_BIT_)
	mPORTDClearBits(_LED2_BIT_);

#elif defined(_LED2_ON_PORT_E) && defined(_LED2_BIT_)
	mPORTEClearBits(_LED2_BIT_);

#elif defined(_LED2_ON_PORT_F) && defined(_LED2_BIT_)
	mPORTFClearBits(_LED2_BIT_);

#elif defined(_LED2_ON_PORT_G) && defined(_LED2_BIT_)
	mPORTGClearBits(_LED2_BIT_);

#endif
}


#endif

#pragma mark -

//*****************************************************************************
//*	this is for AVR compatiblility
#define	kFlashBufferSize	512
char			gFlashDataBuffer[kFlashBufferSize];
char			gPageBuffer[1024];

//*****************************************************************************
static void InitFlashBuffer(void)
{
unsigned int	ii;

	for (ii=0; ii<kFlashBufferSize; ii++)
	{
		gFlashDataBuffer[ii]	=	0xFF;
	}
}


//*****************************************************************************
//*	write 256 bytes of date
static void boot_page_write(long address)
{
void			*actualFlashAddress;
unsigned int	returnCode;

	actualFlashAddress	=	(void *)FLASH_PROG_BASE;
	actualFlashAddress	+=	address;
	
#ifdef _USE_WORD_WRITE_
	unsigned int ii;
	for (ii=0; ii<256; ii+=4)
	{
	unsigned long	theLongWord;
	
		theLongWord	=	(gFlashDataBuffer[ii + 3] & 0x0ff) << 24;
		theLongWord	+=	(gFlashDataBuffer[ii + 2] & 0x0ff) << 16;
		theLongWord	+=	(gFlashDataBuffer[ii + 1] & 0x0ff) << 8;
		theLongWord	+=	(gFlashDataBuffer[ii + 0] & 0x0ff);
	
		NVMWriteWord(actualFlashAddress, theLongWord);

		
		actualFlashAddress	+=	4;
	}
#else
	// Write 128 words 
//	NVMWriteRow((void*)actualFlashAddress, (void *)gFlashDataBuffer);
//unsigned int NVMProgram(void * address, const void * data, unsigned int size, void* pagebuff);
	returnCode	=	NVMProgram(	actualFlashAddress,				//	Destination address to start writing from.
								(void *)gFlashDataBuffer,		//	Location of data to write.
								256,							//	Number of bytes to write.
								(void *)gPageBuffer);
#endif

#ifdef DEBUG_VIA_SERIAL_AUX
	Serial2_println();
	Serial2_print("boot_page_write addr=");
	Serial2_PrintLongWordHex((long)actualFlashAddress);
	Serial2_print(" NVMProgram rc=");
	Serial2_PrintHexByte(returnCode);
	Serial2_println();
#endif
}


//*****************************************************************************
static void boot_page_fill(long address, int data)
{
int				bufferOffset;

	bufferOffset	=	address % 256;
	
	gFlashDataBuffer[bufferOffset]		=	data & 0x0ff;
	gFlashDataBuffer[bufferOffset + 1]	=	(data >> 8) & 0x0ff;
}

#define	PIC32_PAGE_SIZE	4096
#define	EEPROM_SIZE		4096
//*****************************************************************************
//*	Erase the entire chip
//*	this looks at each 4096 byte block, if it is already all 0xff, then it is left alone,
//*	if even one byte is non-0xff, then the entire block gets erasased
//*****************************************************************************
static void boot_chip_erase(void)
{
unsigned char	*actualFlashAddress;
unsigned int	returnCode;
unsigned int	ii;
unsigned char	erasePage;

#ifdef DEBUG_VIA_SERIAL_AUX
long			erasedPageCnt;
long			notErasedCnt;

	erasedPageCnt		=	0;
	notErasedCnt		=	0;

	Serial2_println();
	Serial2_print("boot_chip_erase");
#endif

	actualFlashAddress	=	(unsigned char *)FLASH_PROG_BASE;
	while ((long)actualFlashAddress < (FLASH_PROG_BASE + FLASHEND - EEPROM_SIZE ))
	{
		//*	look through the page and make sure it needs to be erased
		erasePage	=	false;
		for (ii=0; ii<PIC32_PAGE_SIZE; ii++)
		{
			if (actualFlashAddress[ii] != 0x0ff)
			{
				erasePage	=	true;
				break;
			}
		}
		if (erasePage)
		{
			returnCode	=	NVMErasePage(actualFlashAddress);
#ifdef DEBUG_VIA_SERIAL_AUX
			erasedPageCnt++;
#endif
		}
#ifdef DEBUG_VIA_SERIAL_AUX
		else
		{
			notErasedCnt++;
		}
#endif
	
	
		actualFlashAddress	+=	PIC32_PAGE_SIZE;
	}
#ifdef DEBUG_VIA_SERIAL_AUX
	Serial2_println();
	Serial2_print("erased page count=");
	Serial2_PrintLongWordHex(erasedPageCnt);
	Serial2_println();
	Serial2_print("not erased page count=");
	Serial2_PrintLongWordHex(notErasedCnt);
	Serial2_println();
	Serial2_print("total memory erased=");
	Serial2_PrintLongWordHex(erasedPageCnt * PIC32_PAGE_SIZE);
	Serial2_println();
#endif
}


//*****************************************************************************
static unsigned int	pgm_read_word_far(unsigned long argAddress)
{
unsigned int	theData;
unsigned char	*memoryPtr;

	memoryPtr	=	(char *)FLASH_PROG_BASE;

	theData	=	(memoryPtr[argAddress + 1] & 0x00ff) << 8;
	theData	+=	(memoryPtr[argAddress] & 0x00ff);
	

	return(theData);
}




//*****************************************************************************
int   main(void)
{
uint32_t		address			=	0;
uint32_t		eraseAddress	=	0;
unsigned char	msgParseState;
unsigned long	ii				=	0;
unsigned char	checksum		=	0;
unsigned char	seqNum			=	0;
unsigned int	msgLength		=	0;
unsigned char	msgBuffer[285];
unsigned char	theChar;
unsigned char	*dataPtr;
unsigned char   keepGoing;

unsigned long	boot_timeout;
unsigned long	boot_timer;
unsigned int	boot_state;
uint32_t		tempaddress;
unsigned int	data;
unsigned int	size;
unsigned char	highByte, lowByte;
unsigned long	rcvCharCounter;
unsigned char	chipNeedsToBeErased;

#ifdef _ENABLE_MONITOR_
	unsigned int	exPointCntr	=	0;
#endif
#ifdef DEBUG_VIA_SERIAL_AUX
	char			asciiBuff[32];
	int				asciiIdx;
	int				clmCounter;
	unsigned int	jj;
#endif

	__PIC32_pbClk		=	SYSTEMConfigPerformance(F_CPU);


	gFlashPtr			=	(unsigned char *)FLASH_PROG_BASE;
	gEBASEptr			=	(char *)_CP0_GET_EBASE();
	chipNeedsToBeErased	=	true;
	
	OpenCoreTimer(CORE_TICK_RATE);

	BootLED_Init();
#ifdef _BOARD_HAS_2ND_LED_
	DownloadLED_Init();
#endif

#ifdef _DEBUG_WITH_LEDS_
	for (ii=0; ii<=12; ii++)
	{	
		BootLED_Toggle();
		delay_ms(75);
	}
	BootLED_Off();
#endif


#if defined(_BOARD_DIGILENT_UNO_) || defined(_BOARD_DIGILENT_MEGA_)
	//*	boot_timeout is in 10 microsecond increments
	boot_timeout	=	4 * 100000;		//*	should be about 4 seconds
#else
	//*	longer timeout for boards without auto reset
	boot_timeout	=	10 * 100000;		//*	should be about 10 seconds
#endif


	//*
	//*	Init UART
	//*	set baudrate and enable USART receiver and transmiter without interrupts
	#ifdef _USE_UART2_FOR_BOOTLOADER_
		U2MODE				=	(UART_EN);
		U2STA				=	(UART_RX_ENABLE | UART_TX_ENABLE);
		U2BRG				=	(__PIC32_pbClk / 16 / (BAUDRATE - 1));	// calculate actual BAUD generate value.
		U2MODEbits.UARTEN	=	0x01;
		U2STAbits.UTXEN		=	0x01;

		mU2ClearAllIntFlags();
	#else
		U1MODE				=	(UART_EN);
		U1STA				=	(UART_RX_ENABLE | UART_TX_ENABLE);
		U1BRG				=	(__PIC32_pbClk / 16 / (BAUDRATE - 1));	// calculate actual BAUD generate value.
		U1MODEbits.UARTEN	=	0x01;
		U1STAbits.UTXEN		=	0x01;

		mU1ClearAllIntFlags();
	#endif

#ifdef DEBUG_VIA_SERIAL_AUX
	//*
	//*	Init UART
	//*	set baudrate and enable USART receiver and transmiter without interrupts
	Serial2_begin(BAUDRATE_SERIAL2);

	Serial2_println();
	Serial2_print("testing via port 2");
	Serial2_println();
#endif


#ifdef _DEBUG_SERIAL_
//	delay_ms(500);

	Serial_write('s');
	Serial_write('t');
	Serial_write('k');
	Serial_write('5');
	Serial_write('0');
	Serial_write('0');
	Serial_write('v');
	Serial_write('2');
	Serial_write(0x0d);
	Serial_write(0x0a);

//	delay_ms(100);
#endif

	//*	ok, the serial port is initialized, clear any data that may be there
	while (Serial_Available())
	{
		Serial_read();
	}


	//*	this is in case there is no code in the flash memory
	//*	and JumpToApp actually returns
	while (1)
	{
			
	#ifdef DEBUG_VIA_SERIAL_AUX
		Serial2_write(7);
		Serial2_print("boot_state == 0");
		Serial2_println();
	#endif

		boot_timer		=	0;
		boot_state		=	0;
		rcvCharCounter	=	0;

		
		InitFlashBuffer();

		while (boot_state == 0)
		{
			while ((!(Serial_Available())) && (boot_state == 0))		// wait for data
			{
				delay_microSeconds(10);
				boot_timer++;
				if (boot_timer > boot_timeout)
				{
					boot_state	=	1; // (after ++ -> boot_state=2 bootloader timeout, jump to main 0x00000 )
				#ifdef DEBUG_VIA_SERIAL_AUX
					Serial2_print("boot_timer > boot_timeout");
					Serial2_println();

					Serial2_PrintLongWordHex(boot_timer);
					Serial2_println();
				#endif
				}


				//*	this blinks the LED during waiting
				if ((boot_timer % _BLINK_LOOP_COUNT_) == 0)
				{
					BootLED_Toggle();	//*	toggle the LED
				#ifdef DEBUG_VIA_SERIAL_AUX
					Serial2_write('.');
				#endif
				}
			}
			boot_state++; // ( if boot_state=1 bootloader received byte from UART, enter bootloader mode)
		}

	#ifdef DEBUG_VIA_SERIAL_AUX
		Serial2_println();
		Serial2_write(7);
		Serial2_print("boot_state = ");
		Serial2_PrintHexByte(boot_state);
		Serial2_println();
	#endif

		if (boot_state == 1)
		{
			//*	main loop
			ii			=	0;
			keepGoing	=	true;
			while (keepGoing)
			{
				/*
				 * Collect received bytes to a complete message
				 */
				msgParseState	=	ST_START;
				while ( msgParseState != ST_PROCESS )
				{
					if (boot_state == 1)
					{
						boot_state	=	0;
						theChar		=	Serial_read_timeout();
					}
					else
					{
						theChar		=	Serial_read_timeout();
					}
					
					rcvCharCounter++;

				#ifdef _ENABLE_MONITOR_
					if ((theChar == '!') && (rcvCharCounter < 10))
					{
						exPointCntr++;
						if (exPointCntr == 3)
						{
							RunMonitor();
							exPointCntr		=	0;	//	reset back to zero so we dont get in an endless loop
							keepGoing		=	false;
							msgParseState	=	99;	//*	we dont want it do anything
							break;
						}
					}
					else
					{
						exPointCntr	=	0;
					}
				#endif

					switch (msgParseState)
					{
						case ST_START:
						#ifdef _BOARD_HAS_2ND_LED_
							DownloadLED_On();
						#endif
							if ( theChar == MESSAGE_START )
							{
								msgParseState	=	ST_GET_SEQ_NUM;
								checksum		=	MESSAGE_START ^ 0;
							}
							break;

						case ST_GET_SEQ_NUM:
							if ( (theChar == 1) || (theChar == seqNum) )
							{
								seqNum			=	theChar;
								msgParseState	=	ST_MSG_SIZE_1;
								checksum		^=	theChar;
							}
							else
							{
								msgParseState	=	ST_START;
							}
							break;

						case ST_MSG_SIZE_1:
							msgLength		=	theChar << 8;
							msgParseState	=	ST_MSG_SIZE_2;
							checksum		^=	theChar;
							break;

						case ST_MSG_SIZE_2:
							msgLength		|=	theChar;
							msgParseState	=	ST_GET_TOKEN;
							checksum		^=	theChar;
							break;

						case ST_GET_TOKEN:
							if ( theChar == TOKEN )
							{
								msgParseState	=	ST_GET_DATA;
								checksum		^=	theChar;
								ii				=	0;
							}
							else
							{
								msgParseState	=	ST_START;
							}
							break;

						case ST_GET_DATA:
							msgBuffer[ii++]	=	theChar;
							checksum		^=	theChar;
							if (ii == msgLength )
							{
								msgParseState	=	ST_GET_CHECK;
							}
							break;

						case ST_GET_CHECK:
							if ( theChar == checksum )
							{
								msgParseState	=	ST_PROCESS;
							}
							else
							{
								msgParseState	=	ST_START;
							#ifdef DEBUG_VIA_SERIAL_AUX
								Serial2_print("Checksum error ");
								Serial2_println();
							#endif
							}
							break;
					}	//	switch
				}	//	while(msgParseState)

				//*
				//*	Now process the STK500 commands, see Atmel Appnote AVR068
				//*

			#ifdef DEBUG_VIA_SERIAL_AUX
				Serial2_print("switch (msgBuffer[0])  ");
				Serial2_PrintHexByte(msgBuffer[0]);
				Serial2_println();
			#endif
				switch (msgBuffer[0])
				{
					case CMD_SPI_MULTI:
					#ifdef DEBUG_VIA_SERIAL_AUX
						Serial2_print("CMD_SPI_MULTI msgBuffer=");
						for (jj=0; jj<16; jj++)
						{
							Serial2_PrintHexByte(msgBuffer[jj]);
							Serial2_write(0x20);
						}
						Serial2_println();
					#endif
						{
							unsigned char answerByte;
							unsigned char flag=0;


							if ( msgBuffer[4] == 0x30 )
							{
								unsigned char signatureIndex	=	msgBuffer[6];

								if ( signatureIndex == 0 )
								{
									answerByte	=	(SIGNATURE_BYTES >>16) & 0x000000FF;
								}
								else if ( signatureIndex == 1 )
								{
									answerByte	=	(SIGNATURE_BYTES >> 8) & 0x000000FF;
								}
								else
								{
									answerByte	=	SIGNATURE_BYTES & 0x000000FF;
								}
							}
							else if ((msgBuffer[4] == 0x20) || (msgBuffer[4] == 0x28))
							{
								//*	read one byte from flash
								//*	0x20 is read odd byte
								//*	0x28 is read even byte
							
								//*	read the even address
								address		=	msgBuffer[5] << 8;
								address		+=	msgBuffer[6];
								//*	the address is in 16 bit words
								address		=	address << 1;
								if (msgBuffer[4] == 0x20)
								{
									answerByte	=	pgm_read_word_far(address) & 0x0ff;
								}
								else
								{
									answerByte	=	pgm_read_word_far(address) >> 8;
								}
								
							}
							else if ( msgBuffer[4] & 0x50 )
							{
								answerByte	=	0; //read fuse/lock bits not implemented, return dummy value
							}
							else
							{
	//							answerByte	=	0; // for all others command are not implemented, return dummy value for AVRDUDE happy <Worapoht>
								answerByte	=	0xBB;
		//						flag	=	1; // Remark this line for AVRDUDE <Worapoht>
							}
							if ( !flag )
							{
								msgLength		=	7;
								msgBuffer[1]	=	STATUS_CMD_OK;
								msgBuffer[2]	=	0;
								msgBuffer[3]	=	msgBuffer[4];
								msgBuffer[4]	=	0;
								msgBuffer[5]	=	answerByte;
								msgBuffer[6]	=	STATUS_CMD_OK;
							}
						}
						break;

					case CMD_SIGN_ON:
					#ifdef DEBUG_VIA_SERIAL_AUX
						Serial2_print("CMD_SIGN_ON");
						Serial2_println();
					#endif
						msgLength		=	11;
						msgBuffer[1] 	=	STATUS_CMD_OK;
						msgBuffer[2] 	=	8;
						msgBuffer[3] 	=	'A';
						msgBuffer[4] 	=	'V';
						msgBuffer[5] 	=	'R';
						msgBuffer[6] 	=	'I';
						msgBuffer[7] 	=	'S';
						msgBuffer[8] 	=	'P';
						msgBuffer[9] 	=	'_';
						msgBuffer[10]	=	'2';
						break;

					case CMD_GET_PARAMETER:
					#ifdef DEBUG_VIA_SERIAL_AUX
						Serial2_print("CMD_GET_PARAMETER switch(msgBuffer[1])=");
						Serial2_PrintHexByte(msgBuffer[1]);
						Serial2_println();
					#endif
						{
							unsigned char value;

							switch(msgBuffer[1])
							{
							case PARAM_BUILD_NUMBER_LOW:
								value	=	CONFIG_PARAM_BUILD_NUMBER_LOW;
								break;
							case PARAM_BUILD_NUMBER_HIGH:
								value	=	CONFIG_PARAM_BUILD_NUMBER_HIGH;
								break;
							case PARAM_HW_VER:
								value	=	CONFIG_PARAM_HW_VER;
								break;
							case PARAM_SW_MAJOR:
								value	=	CONFIG_PARAM_SW_MAJOR;
								break;
							case PARAM_SW_MINOR:
								value	=	CONFIG_PARAM_SW_MINOR;
								break;
							default:
								value	=	0;
								break;
							}
							msgLength		=	3;
							msgBuffer[1]	=	STATUS_CMD_OK;
							msgBuffer[2]	=	value;
						}
						break;

					case CMD_LEAVE_PROGMODE_ISP:
					#ifdef DEBUG_VIA_SERIAL_AUX
						Serial2_print("CMD_LEAVE_PROGMODE_ISP");
						Serial2_println();
					#endif
						keepGoing		=	false;
						msgLength		=	2;
						msgBuffer[1]	=	STATUS_CMD_OK;
						break;

					case CMD_SET_PARAMETER:
					case CMD_ENTER_PROGMODE_ISP:
					#ifdef DEBUG_VIA_SERIAL_AUX
						Serial2_print("CMD_ENTER_PROGMODE_ISP");
						Serial2_println();
					#endif
						msgLength		=	2;
						msgBuffer[1]	=	STATUS_CMD_OK;
						break;

					case CMD_READ_SIGNATURE_ISP:
					#ifdef DEBUG_VIA_SERIAL_AUX
						Serial2_print("CMD_READ_SIGNATURE_ISP");
						Serial2_println();
					#endif
						{
							unsigned char signatureIndex	=	msgBuffer[4];
							unsigned char signature;

							if ( signatureIndex == 0 )
								signature	=	(SIGNATURE_BYTES >>16) & 0x000000FF;
							else if ( signatureIndex == 1 )
								signature	=	(SIGNATURE_BYTES >> 8) & 0x000000FF;
							else
								signature	=	SIGNATURE_BYTES & 0x000000FF;

							msgLength		=	4;
							msgBuffer[1]	=	STATUS_CMD_OK;
							msgBuffer[2]	=	signature;
							msgBuffer[3]	=	STATUS_CMD_OK;
						}
						break;

					case CMD_READ_LOCK_ISP:
						msgLength		=	4;
						msgBuffer[1]	=	STATUS_CMD_OK;
	//+					msgBuffer[2]	=	boot_lock_fuse_bits_get( GET_LOCK_BITS );
						msgBuffer[3]	=	STATUS_CMD_OK;
						break;

					case CMD_READ_FUSE_ISP:
						{
							unsigned char fuseBits;

							if ( msgBuffer[2] == 0x50 )
							{
								if ( msgBuffer[3] == 0x08 )
								{
	//+								fuseBits	=	boot_lock_fuse_bits_get( GET_EXTENDED_FUSE_BITS );
									fuseBits	=	'C';
								}
								else
								{
	//+								fuseBits	=	boot_lock_fuse_bits_get( GET_LOW_FUSE_BITS );
									fuseBits	=	'P';
								}
							}
							else
							{
	//+							fuseBits	=	boot_lock_fuse_bits_get( GET_HIGH_FUSE_BITS );
								fuseBits	=	'I';
							}
							msgLength		=	4;
							msgBuffer[1]	=	STATUS_CMD_OK;
							msgBuffer[2]	=	fuseBits;
							msgBuffer[3]	=	STATUS_CMD_OK;
						}
						break;

		#ifdef INCLUDE_PROGRAM_LOCK_BIT_SUPPORT
					case CMD_PROGRAM_LOCK_ISP:
						{
							unsigned char lockBits	=	msgBuffer[4];

							lockBits	=	(~lockBits) & 0x3C;	// mask BLBxx bits
	//+						boot_lock_bits_set(lockBits);		// and program it
	//+						boot_spm_busy_wait();

							msgLength		=	3;
							msgBuffer[1]	=	STATUS_CMD_OK;
							msgBuffer[2]	=	STATUS_CMD_OK;
						}
						break;
		#endif
					case CMD_CHIP_ERASE_ISP:
					#ifdef DEBUG_VIA_SERIAL_AUX
						Serial2_print("CMD_CHIP_ERASE_ISP ");
						Serial2_println();
					#endif

						eraseAddress	=	0;
						msgLength		=	2;
						msgBuffer[1]	=	STATUS_CMD_OK;
						break;

					case CMD_LOAD_ADDRESS:
						address			=	(	((unsigned long)(msgBuffer[1]) << 24) |
												((unsigned long)(msgBuffer[2])<<16) |
												((unsigned long)(msgBuffer[3])<<8) |
												(msgBuffer[4]) ) << 1;
						msgLength		=	2;
						msgBuffer[1]	=	STATUS_CMD_OK;

					#ifdef DEBUG_VIA_SERIAL_AUX
						Serial2_print("CMD_LOAD_ADDRESS ");
						Serial2_PrintLongWordHex(address);

						Serial2_println();
					#endif
						break;

					case CMD_PROGRAM_FLASH_ISP:
						{
							size		=	((msgBuffer[1]) << 8) | msgBuffer[2];
							dataPtr		=	msgBuffer + 10;
							tempaddress	=	address;

						#ifdef DEBUG_VIA_SERIAL_AUX
							Serial2_print("CMD_PROGRAM_FLASH_ISP");
							Serial2_println();

							Serial2_print("size=");
							Serial2_PrintHexByte(size >> 8);
							Serial2_PrintHexByte(size & 0x0ff);
							Serial2_println();
							Serial2_print("address=");
							Serial2_PrintLongWordHex(address);
							Serial2_println();
						#endif

							if ( msgBuffer[0] == CMD_PROGRAM_FLASH_ISP )
							{
								if (chipNeedsToBeErased)
								{
									//*	this only gets executed ONCE
									boot_chip_erase();
									chipNeedsToBeErased	=	false;
								}

							#ifdef DEBUG_VIA_SERIAL_AUX
								asciiIdx		=	0;
								asciiBuff[0]	=	0;
							#endif
								/* Write FLASH */
								do
								{
									lowByte		=	*dataPtr++;
									highByte 	=	*dataPtr++;

									data		=	(highByte << 8) | lowByte;
									boot_page_fill(address, data);

							#ifdef DEBUG_VIA_SERIAL_AUX
									if ((address % 16) == 0)
									{
										if (strlen(asciiBuff) > 0)
										{
											Serial2_print("  ");
											Serial2_print(asciiBuff);
										}
										Serial2_println();
										Serial2_PrintLongWordHex(address);
										Serial2_print(" - ");
										asciiIdx	=	0;
									}
									Serial2_PrintHexByte(lowByte);
									Serial2_write(' ');
									Serial2_PrintHexByte(highByte);
									Serial2_write(' ');

									asciiBuff[asciiIdx++]	=	(((lowByte >= 0x20) && (lowByte < 0x7f)) ? lowByte : '.');
									asciiBuff[asciiIdx++]	=	(((highByte >= 0x20) && (highByte < 0x7f)) ? highByte : '.');;
									asciiBuff[asciiIdx]		=	0;
							#endif

									address	=	address + 2;	// Select next word in memory
									size	-=	2;				// Reduce number of bytes to write by two
								} while (size > 0);				// Loop until all bytes written

							#ifdef DEBUG_VIA_SERIAL_AUX
								if (strlen(asciiBuff) > 0)
								{
									Serial2_print("  ");
									Serial2_print(asciiBuff);
								}
							#endif

								boot_page_write(tempaddress);
	//+							boot_spm_busy_wait();
	//+							boot_rww_enable();				// Re-enable the RWW section
							}
							msgLength		=	2;
							msgBuffer[1]	=	STATUS_CMD_OK;
						#ifdef DEBUG_VIA_SERIAL_AUX
							Serial2_println();
						#endif
						}
						break;

	#if 0
	//*	not implemented (Mar 13, 2011)
					case CMD_PROGRAM_EEPROM_ISP:
						{
							dataPtr		=	msgBuffer + 10;
							size		=	((msgBuffer[1]) << 8) | msgBuffer[2];
							tempaddress	=	address;
						#if 0
							/* write EEPROM */
							do {
								EEARL	=	address;			// Setup EEPROM address
								EEARH	=	(address >> 8);
								address++;						// Select next EEPROM byte

								EEDR	=	*dataPtr++;			// get byte from buffer
								EECR	|=	(1<<EEMWE);			// Write data into EEPROM
								EECR	|=	(1<<EEWE);

								while (EECR & (1<<EEWE));		// Wait for write operation to finish
								size--;							// Decrease number of bytes to write
							} while (size);						// Loop until all bytes written
						#endif
							msgLength		=	2;
							msgBuffer[1]	=	STATUS_CMD_OK;
						}
						break;

					case CMD_READ_EEPROM_ISP:
						#ifdef DEBUG_VIA_SERIAL_AUX
							Serial2_print("CMD_READ_EEPROM_ISP");
							Serial2_println();
						#endif
	#endif
					case CMD_READ_FLASH_ISP:
						{

							BootLED_Toggle();

							size		=	((msgBuffer[1]) << 8) | msgBuffer[2];
							dataPtr		=	msgBuffer + 1;
							msgLength	=	size + 3;

						#ifdef DEBUG_VIA_SERIAL_AUX
							Serial2_print("CMD_READ_FLASH_ISP");
							Serial2_println();
							Serial2_print("size=");
							Serial2_PrintHexByte(size >> 8);
							Serial2_PrintHexByte(size & 0x0ff);
							Serial2_println();
							Serial2_print("address=");
							Serial2_PrintLongWordHex(address);
							Serial2_println();
							
							clmCounter	=	0;
						#endif

							*dataPtr++	=	STATUS_CMD_OK;
							if (msgBuffer[0] == CMD_READ_FLASH_ISP )
							{

								// Read FLASH
								do
								{
								
									data		=	pgm_read_word_far(address);
									*dataPtr++	=	(unsigned char)data;		//LSB
									*dataPtr++	=	(unsigned char)(data >> 8);	//MSB

									address	+=	2;							// Select next word in memory
									size	-=	2;
								#ifdef DEBUG_VIA_SERIAL_AUX
									Serial2_PrintHexByte(data & 0x0ff);
									Serial2_write(0x20);
									Serial2_PrintHexByte(data >> 8);
									Serial2_write(0x20);
									clmCounter++;
									if (clmCounter >= 8)
									{
										Serial2_println();
										clmCounter	=	0;
									}
								#endif
								} while (size > 0);
							#ifdef DEBUG_VIA_SERIAL_AUX
								Serial2_println();
							#endif
							}
							else
							{
								/* Read EEPROM */
								do
								{
	//+								EEARL		=	address;			// Setup EEPROM address
	//+								EEARH		=	((address >> 8));
									address++;					// Select next EEPROM byte
	//+								EECR		|=	(1<<EERE);			// Read EEPROM
	//+								*dataPtr++	=	EEDR;				// Send EEPROM data
									size--;
								} while (size);
							}
							*dataPtr++	=	STATUS_CMD_OK;
						}
						break;

					default:
						msgLength		=	2;
						msgBuffer[1]	=	STATUS_CMD_FAILED;
						break;
				}

				//*
				//*	Now send answer message back
				//*
				Serial_write(MESSAGE_START);
				checksum	=	MESSAGE_START^0;

				Serial_write(seqNum);
				checksum	^=	seqNum;

				theChar		=	((msgLength>>8)&0xFF);
				Serial_write(theChar);
				checksum	^=	theChar;

				theChar		=	msgLength&0x00FF;
				Serial_write(theChar);
				checksum	^=	theChar;

				Serial_write(TOKEN);
				checksum	^=	TOKEN;

				dataPtr	=	msgBuffer;
				while ( msgLength )
				{
					theChar	=	*dataPtr++;
					Serial_write(theChar);
					checksum	^=	theChar;
					msgLength--;
				}
				Serial_write(checksum);
				seqNum++;

				//*	<MLS>	toggle the LED
				BootLED_Toggle();

			}
		}


	#ifdef _DEBUG_SERIAL_
		Serial_write('j');
	//	Serial_write('u');
	//	Serial_write('m');
	//	Serial_write('p');
	//	Serial_write(' ');
	//	Serial_write('u');
	//	Serial_write('s');
	//	Serial_write('r');
		Serial_write(0x0d);
		Serial_write(0x0a);
		delay_ms(100);
	#endif



		//*
		//*	Now leave bootloader
		//*

		JumpToApp();
		
	}
	 /*
	 * Never return to stop GCC to generate exit return code
	 * Actually we will never reach this point, but the compiler doesn't
	 * understand this
	 */
	for(;;);
}





#pragma mark -
#pragma mark Monitor code

//************************************************************************
#ifdef _ENABLE_MONITOR_
#include	<math.h>


unsigned char	*gRamPtr;
unsigned long	gRamIndex;
unsigned long	gFlashIndex;



enum
{
	kDUMP_FLASH	=	0,
	kDUMP_EEPROM,
	kDUMP_RAM
};


#define	prog_char	const char
#define	PROGMEM


#ifdef _CPU_NAME_
	prog_char	gTextMsg_CPU_Name[]			PROGMEM	=	_CPU_NAME_;
#else
	prog_char	gTextMsg_CPU_Name[]			PROGMEM	=	"UNKNOWN";
#endif

	prog_char	gTextMsg_Explorer[]			PROGMEM	=	"Explorer stk500V2 by MLS " _VERSION_STRING_;
	prog_char	gTextMsg_Prompt[]			PROGMEM	=	"Bootloader>";
	prog_char	gTextMsg_HUH[]				PROGMEM	=	"Huh?";
	prog_char	gTextMsg_COMPILED_ON[]		PROGMEM	=	"Compiled on = ";
	prog_char	gTextMsg_GCC_DATE_STR[]		PROGMEM	=	__DATE__;
	prog_char	gTextMsg_CPU_Type[]			PROGMEM	=	"CPU Type    = ";
	prog_char	gTextMsg_DEVID[]			PROGMEM	=	"DEVID       = ";
	prog_char	gTextMsg_GCC_VERSION[]		PROGMEM	=	"GCC Version = ";
	prog_char	gTextMsg_GCC_VERSION_STR[]	PROGMEM	=	__VERSION__;

//************************************************************************
//*	Help messages
	prog_char	gTextMsg_HELP_MSG_0[]		PROGMEM	=	"0=Zero address ctrs";
	prog_char	gTextMsg_HELP_MSG_QM[]		PROGMEM	=	"?=CPU stats";
	prog_char	gTextMsg_HELP_MSG_B[]		PROGMEM	=	"B=Blink LED";
	prog_char	gTextMsg_HELP_MSG_F[]		PROGMEM	=	"F=Dump FLASH";
	prog_char	gTextMsg_HELP_MSG_H[]		PROGMEM	=	"H=Help";
#ifdef _ENABLE_PORT_LIST_
	prog_char	gTextMsg_HELP_MSG_L[]		PROGMEM	=	"L=List I/O Ports";
#endif
	prog_char	gTextMsg_HELP_MSG_Q[]		PROGMEM	=	"Q=Quit & jump to user pgm";
	prog_char	gTextMsg_HELP_MSG_R[]		PROGMEM	=	"R=Dump RAM";
#ifdef _ENABLE_PORT_BLINK_
	prog_char	gTextMsg_HELP_MSG_Y[]		PROGMEM	=	"Y=Port blink";
#endif




//************************************************************************
static void 	PrintFromPROGMEM(const char *dataPtr, unsigned char offset)
{
	Serial_print(&dataPtr[offset]);
}


//************************************************************************
static void 	PrintFromPROGMEMln(const char *dataPtr, uint8_t offset)
{
	PrintFromPROGMEM(dataPtr, offset);

	Serial_println();
}


//************************************************************************
static void 	PrintHexNyble(unsigned char theNyble)
{
char	theChar;

	theChar	=	0x30 + theNyble;
	if (theChar > 0x39)
	{
		theChar	+=	7;
	}
	Serial_write(theChar );

}

//************************************************************************
static void 	PrintHexByte(unsigned char theByte)
{
	PrintHexNyble(theByte >> 4);
	PrintHexNyble(theByte & 0x0f);
}

//************************************************************************
static void 	PrintHexLong(unsigned long theLongWord)
{
	PrintHexByte((theLongWord >> 24) & 0x0ff);
	PrintHexByte((theLongWord >> 16) & 0x0ff);
	PrintHexByte((theLongWord >> 8) & 0x0ff);
	PrintHexByte((theLongWord) & 0x0ff);
}



//************************************************************************
static  void	PrintCPUstats(void)
{
unsigned long	ebase;
int				myIntCtl;
int				myVectorSpacing;

	PrintFromPROGMEMln(gTextMsg_Explorer, 0);

	PrintFromPROGMEM(gTextMsg_COMPILED_ON, 0);
	PrintFromPROGMEMln(gTextMsg_GCC_DATE_STR, 0);

	PrintFromPROGMEM(gTextMsg_CPU_Type, 0);
	PrintFromPROGMEMln(gTextMsg_CPU_Name, 0);


	PrintFromPROGMEM(gTextMsg_GCC_VERSION, 0);
	PrintFromPROGMEMln(gTextMsg_GCC_VERSION_STR, 0);

	//*	get the DEVID
	PrintFromPROGMEM(gTextMsg_DEVID, 0);
	PrintHexLong(DEVID);
	Serial_println();

#if 0
	//*	print out the start of the bootloader code

	Serial_print("Bootloader start=");
	PrintHexLong((unsigned long)&JumpToApp);
	Serial_println();

	Serial_print("BMXDRMSZ        =");
	PrintHexLong((unsigned long)BMXDRMSZ);
	Serial_println();

	Serial_print("BMXPFMSZ        =");
	PrintHexLong((unsigned long)BMXPFMSZ);
	Serial_println();

	Serial_print("BMXBOOTSZ       =");
	PrintHexLong((unsigned long)BMXBOOTSZ);
	Serial_println();

	//*	print the EBASE value
	ebase	=	_CP0_GET_EBASE();
	Serial_print("EBASE           =");
	PrintHexLong(ebase);
	Serial_println();


	//*	get the vector spacing (IntCtl<9:5)
	myIntCtl	=	_CP0_GET_INTCTL();
	Serial_print("IntCtl          =");
	PrintHexLong(myIntCtl);
	Serial_println();

	myVectorSpacing	=	(myIntCtl >> 5) & 0x01F;
	Serial_print("VectorSpacing   =");
	PrintHexLong(myVectorSpacing);
	Serial_println();

#endif
}



//************************************************************************
static void  BlinkLED(void)
{
	while (!Serial_Available())
	{
		BootLED_Toggle();
		delay_microSeconds(100 * 1000);
	}
	Serial_read();	//	get the char out of the buffer

	BootLED_Off();
}


//************************************************************************
static void 	DumpHex(unsigned char dumpWhat, unsigned long startAddress, unsigned char numRows)
{
unsigned long	myAddressPointer;
unsigned long	baseAddress;
uint8_t			ii;
unsigned char	theValue;
char			asciiDump[18];
unsigned char	*ramPtr;


	ramPtr				=	0;
	theValue			=	0;
	myAddressPointer	=	startAddress;
	while (numRows > 0)
	{
		//*	print the base address
		switch(dumpWhat)
		{
			case kDUMP_FLASH:
				baseAddress	=	(unsigned long)gFlashPtr;
				break;

			case kDUMP_EEPROM:
				baseAddress	=	(unsigned long)gEBASEptr;
				break;

			case kDUMP_RAM:
				baseAddress	=	(unsigned long)gRamPtr;
				break;

		}
		PrintHexLong(baseAddress);
		Serial_print(" + ");

		if (myAddressPointer > 0x10000)
		{
			PrintHexByte((myAddressPointer >> 16) & 0x00ff);
		}
		PrintHexByte((myAddressPointer >> 8) & 0x00ff);
		PrintHexByte(myAddressPointer & 0x00ff);
		Serial_write(0x20);
		Serial_write('-');
		Serial_write(0x20);

		asciiDump[0]		=	0;
		for (ii=0; ii<16; ii++)
		{
			switch(dumpWhat)
			{
				case kDUMP_FLASH:
					theValue	=	gFlashPtr[myAddressPointer];
					break;

				case kDUMP_EEPROM:
//+					theValue	=	eeprom_read_byte((void *)myAddressPointer);
					theValue	=	gEBASEptr[myAddressPointer];
					break;

				case kDUMP_RAM:
					theValue	=	gRamPtr[myAddressPointer];
					break;

			}
			PrintHexByte(theValue);
			Serial_write(0x20);
			if ((theValue >= 0x20) && (theValue < 0x7f))
			{
				asciiDump[ii % 16]	=	theValue;
			}
			else
			{
				asciiDump[ii % 16]	=	'.';
			}

			myAddressPointer++;
		}
		asciiDump[16]	=	0;
		Serial_print(asciiDump);
		Serial_println();

		numRows--;
	}
}

#ifdef _ENABLE_PORT_LIST_
//************************************************************************
static void	 PrintAvailablePort(char thePortLetter, uint16_t dataFromPort)
{
uint8_t				theData;
uint8_t				ii;
volatile uint8_t	*thePort;
char				theChar;

	Serial_print("PORT");
	Serial_write(thePortLetter);
	Serial_print("  =");
	PrintHexByte(dataFromPort / 256);
	PrintHexByte(dataFromPort % 256);

	Serial_println();
}

//************************************************************************
static void 	ListAvailablePorts(void)
{

#ifdef _PORTA
	PrintAvailablePort('A', PORTA);
#endif

#ifdef _PORTB
	PrintAvailablePort('B', PORTB);
#endif

#ifdef _PORTC
	PrintAvailablePort('C', PORTC);
#endif

#ifdef _PORTD
	PrintAvailablePort('D', PORTD);
#endif

#ifdef _PORTE
	PrintAvailablePort('E', PORTE);
#endif

#ifdef _PORTF
	PrintAvailablePort('F', PORTF);
#endif

#ifdef _PORTG
	PrintAvailablePort('G', PORTG);
#endif


}
#endif

#ifdef _ENABLE_PORT_BLINK_
#define	kBlinkDelayValue	200

//************************************************************************
static void	 BlinkEntirePort(void)
{
char				portLetter;
char				validPortFlag;
volatile uint32_t	*tris_reg;
volatile uint32_t	*port_reg;
uint8_t				port;

	Serial_print("What port:");

	portLetter	=	Serial_read();
	portLetter	=	portLetter & 0x5f;
	Serial_write(portLetter);
	Serial_println();

	if ((portLetter >= 'A') && (portLetter <= 'Z'))
	{
		validPortFlag	=	true;
		switch(portLetter)
		{
		#ifdef _PORTA
			case 'A':	
				tris_reg	=	&TRISA;
				port_reg	=	&PORTA;
				break;
		#endif

		#ifdef _PORTB
			case 'B':
				tris_reg	=	&TRISB;
				port_reg	=	&PORTB;
				break;
		#endif

		#ifdef _PORTC
			case 'C':
				tris_reg	=	&TRISC;
				port_reg	=	&PORTC;
				break;
		#endif

		#ifdef _PORTD
			case 'D':
				tris_reg	=	&TRISD;
				port_reg	=	&PORTD;
				break;
		#endif

		#ifdef _PORTE
			case 'E':
				tris_reg	=	&TRISE;
				port_reg	=	&PORTE;
				break;
		#endif

		#ifdef _PORTF
			case 'F':
				tris_reg	=	&TRISF;
				port_reg	=	&PORTF;
				break;
		#endif

		#ifdef _PORTG
			case 'G':
				tris_reg	=	&TRISG;
				port_reg	=	&PORTG;
				break;
		#endif

		#ifdef _PORTH
			case 'H':
				tris_reg	=	&TRISH;
				port_reg	=	&PORTH;
				break;
		#endif

		#ifdef _PORTI
			case 'I':
				tris_reg	=	&TRISI;
				port_reg	=	&PORTI;
				break;
		#endif

		#ifdef _PORTJ
			case 'J':
				tris_reg	=	&TRISJ;
				port_reg	=	&PORTJ;
				break;
		#endif

		#ifdef _PORTK
			case 'K':
				tris_reg	=	&TRISK;
				port_reg	=	&PORTK;
				break;
		#endif

		#ifdef _PORTL
			case 'L':
				tris_reg	=	&TRISL;
				port_reg	=	&PORTL;
				break;
		#endif

			default:
				validPortFlag	=	false;
				Serial_print("Port not supported");
				Serial_println();
				break;
		}
		if (validPortFlag)
		{
			*tris_reg	=	0;	//*	set port to output
			while (!Serial_Available())
			{
				*port_reg	=	0xffff;
				delay_ms(kBlinkDelayValue);

				*port_reg	=	0;
				delay_ms(kBlinkDelayValue);
			}
			*port_reg	=	0;
			Serial_read();
		}
	}
	else
	{
		Serial_print("Must be a letter");
		Serial_println();
	}
}
#endif


//*******************************************************************
static void PrintHelp(void)
{
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_0, 0);
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_QM, 0);
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_B, 0);

	PrintFromPROGMEMln(gTextMsg_HELP_MSG_F, 0);
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_H, 0);

#ifdef _ENABLE_PORT_LIST_
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_L, 0);
#endif
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_Q, 0);
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_R, 0);
#ifdef _ENABLE_PORT_BLINK_
	PrintFromPROGMEMln(gTextMsg_HELP_MSG_Y, 0);
#endif
}

#pragma mark -


//************************************************************************
static void	 RunMonitor(void)
{
char			keepGoing;
unsigned char	theChar;
unsigned int	ii, jj;

#ifdef _BOARD_HAS_2ND_LED_
	DownloadLED_Off();
#endif

	for (ii=0; ii<5; ii++)
	{
		for (jj=0; jj<25; jj++)
		{
			Serial_write('!');
		}
		Serial_println();
	}

	gRamIndex			=	0;
	gFlashIndex			=	0;
	gRamPtr			=	(unsigned char *)0x80000000;
	gRamIndex		=	0;
	gFlashPtr		=	(unsigned char *)FLASH_PROG_BASE;
	gFlashIndex		=	0;

	PrintFromPROGMEMln(gTextMsg_Explorer, 0);

	keepGoing	=	1;
	while (keepGoing)
	{
		PrintFromPROGMEM(gTextMsg_Prompt, 0);
		theChar	=	Serial_read();
		if (theChar >= 0x60)
		{
			theChar	=	theChar & 0x5F;
		}
	#if defined( _CEREBOTPLUS_BOARD_ )
		if (theChar == 0x5F)
		{

		}
		else
	#endif
		if (theChar >= 0x20)
		{
			Serial_write(theChar);
			Serial_write(0x20);
		}

		switch(theChar)
		{
			case '0':
				PrintFromPROGMEMln(gTextMsg_HELP_MSG_0, 2);
				gFlashIndex		=	0;
				gRamIndex		=	0;
				break;

			case '?':
				PrintFromPROGMEMln(gTextMsg_HELP_MSG_QM, 2);
				PrintCPUstats();
				break;

			case 'B':
				PrintFromPROGMEMln(gTextMsg_HELP_MSG_B, 2);
				BlinkLED();
				break;
		
			case 'F':
				PrintFromPROGMEMln(gTextMsg_HELP_MSG_F, 2);
				DumpHex(kDUMP_FLASH, gFlashIndex, 16);
				gFlashIndex	+=	256;
				break;

			case 'H':
				PrintFromPROGMEMln(gTextMsg_HELP_MSG_H, 2);
				PrintHelp();
				break;
		
		#ifdef _ENABLE_PORT_LIST_
			case 'L':
				PrintFromPROGMEMln(gTextMsg_HELP_MSG_L, 2);
				ListAvailablePorts();
				break;
		#endif

			case 'Q':
				PrintFromPROGMEMln(gTextMsg_HELP_MSG_Q, 2);
				keepGoing	=	false;
				break;

			case 'R':
				PrintFromPROGMEMln(gTextMsg_HELP_MSG_R, 2);
				DumpHex(kDUMP_RAM, gRamIndex, 16);
				gRamIndex	+=	256;
				break;


		#ifdef _ENABLE_PORT_BLINK_
			case 'Y':
				PrintFromPROGMEMln(gTextMsg_HELP_MSG_Y, 2);
				BlinkEntirePort();
				break;
		#endif
		
		#ifdef _ENABLE_TIME_TEST_
			case 'T':
				Serial_println();
				Serial_print("TimerStart");
				for (ii=0; ii<10; ii++)
				{
					delay_ms(1000);
					Serial_write(0x30 + ii);

				}
				Serial_print("Done 10 seconds");
				Serial_println();
				break;
		#endif
				
				
			default:
				PrintFromPROGMEMln(gTextMsg_HUH, 0);
				break;
		}
	}
}

#endif	//	_ENABLE_MONITOR_
