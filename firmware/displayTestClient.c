/************************************************************************************
* Gemini Interface Control Board -- Seven Segment Display Test Client
* Property of Super Props Inc., all rights reserved
*
* Client file to test all 7-segment displays and LEDs present on the Gemini Interface
* Control Board. This test client first does a single SPI write to all displays at
* the same time, clearing them to OFF. Within the main loop, a test is run to count
* from 0x0 to 0xF on all displays at the same time starting OFF and ending OFF.
* Then, each display counts from 0x0 to 0xF individually to ensure each chip select
* pin is working properly.Two more SPI writes are then done to all displays at once,
* flashing all segments ON (including the decimal points) and then OFF again.
* Finally, a loop of SPI writes are done to the LED shift register, turning each
* LED on one at a time as if they were being shifted through.
* The loop is then repeated. The delay between each count or flash is controlled by
* the symbolic constant COUNT_DELAY_CYCLES, which is about 0.25 seconds at the time
* of writing.
*
* NOTE: Be sure to set WAIT_FOR_PUTCHAR to 1 in the SPI module to ensure all 8 bits
* are sent to the desired shift registers before continuing; the clocks and code
* are fast/efficient enough such that the chip select pin can be disabled before
* the SPI write is done. If WAIT_FOR_PUTCHAR is not 1, SMCLK must be at least as
* fast as MCLK to have a chance at sending all data properly -- even then, it is
* not guaranteed for every application.
*
* Author:       Mason Kury
* Created:      November 10, 2022
* Modified:     November 20, 2022
************************************************************************************/


//########## DEPENDENCIES ##########//
#include <msp430.h>
#include "spi.h"
#include "sevenSeg.h"


//########## SYMBOLIC CONSTANTS ##########//
#define DISPS_CSDIR         P3DIR
#define DISPS_CSOUT         P3OUT
#define DISP0               BIT0
#define DISP1               BIT1
#define DISP2               BIT2
#define DISP3               BIT3
#define DISP4               BIT4
#define DISP5               BIT5
#define DISP6               BIT6
#define DISP7               BIT7
#define TOP_DISPS           0x0F
#define BOT_DISPS           0xF0
#define ALL_DISPS           0xFF

#define LEDSR_CSDIR         P1DIR
#define LEDSR_CSOUT         P1OUT
#define LEDSR               BIT6
#define LED0                BIT0
#define LED1                BIT1
#define LED2                BIT2
#define LED3                BIT3
#define LED4                BIT4
#define LED5                BIT5
#define LED6                BIT6
#define LED7                BIT7
#define ALL_LEDS            0xFF

#define COUNT_DELAY_CYCLES  262000  // number of CPU cycles to delay between counting during tests
#define NUM_DISPS           8       /* Do not set to be greater than 8, as byte-size shifting is done for display index manipulation.
                                       If a port wider than 1 byte is used and properly defined for use with the writeHexToSevSeg function,
                                       you can set NUM_DISPS up to the width of port.  */

//########## PREPROCESSOR MACROS ##########//

// macros to activate chip select pins
#define SEL_DISP0       DISPS_CSOUT |= DISP0
#define SEL_DISP1       DISPS_CSOUT |= DISP1
#define SEL_DISP2       DISPS_CSOUT |= DISP2
#define SEL_DISP3       DISPS_CSOUT |= DISP3
#define SEL_DISP4       DISPS_CSOUT |= DISP4
#define SEL_DISP5       DISPS_CSOUT |= DISP5
#define SEL_DISP6       DISPS_CSOUT |= DISP6
#define SEL_DISP7       DISPS_CSOUT |= DISP7
#define SEL_TOP_DISPS   DISPS_CSOUT |= TOP_DISPS
#define SEL_BOT_DISPS   DISPS_CSOUT |= BOT_DISPS
#define SEL_ALL_DISPS   DISPS_CSOUT |= ALL_DISPS
#define SEL_LEDSR       LEDSR_CSOUT |= LEDSR

// macros to deactivate chip select pins
#define DSEL_DISP0      DISPS_CSOUT &= ~DISP0
#define DSEL_DISP1      DISPS_CSOUT &= ~DISP1
#define DSEL_DISP2      DISPS_CSOUT &= ~DISP2
#define DSEL_DISP3      DISPS_CSOUT &= ~DISP3
#define DSEL_DISP4      DISPS_CSOUT &= ~DISP4
#define DSEL_DISP5      DISPS_CSOUT &= ~DISP5
#define DSEL_DISP6      DISPS_CSOUT &= ~DISP6
#define DSEL_DISP7      DISPS_CSOUT &= ~DISP7
#define DSEL_TOP_DISPS  DISPS_CSOUT &= ~TOP_DISPS
#define DSEL_BOT_DISPS  DISPS_CSOUT &= ~BOT_DISPS
#define DSEL_ALL_DISPS  DISPS_CSOUT &= ~ALL_DISPS
#define DSEL_LEDSR      LEDSR_CSOUT &= ~LEDSR


//########## GLOBALS ##########//



//########## FUNCTION PROTOTYPES ##########//
unsigned char writeHexToSevSeg(const USCIXNSPI* usciXN, SEVEN_SEG_DISP* display, unsigned char hexCode, unsigned char csPortIndex);


//########## MAIN FUNCTION ##########//
int main(void)
{
    // define registers for USCI_A0 on PORT1, with only SOMI and SCLK; this peripheral will run with loopback, as no SOMI is needed
    const USCIXNSPI USCIA0SPI = {&P1SEL, &P1SEL2, 0x0, BIT2, 0x0, BIT4, &UCA0CTL0, &UCA0CTL1, &UCA0BR0, &UCA0BR1, &UCA0STAT, &UCA0TXBUF, &UCA0RXBUF, &IFG2, UCA0TXIFG, UCA0RXIFG};

    SEVEN_SEG_DISP sevSegDispArr[NUM_DISPS];    // an array of seven segment displays, representing the 8 on the gemini interface
    unsigned char dispIndex;                    // used to index displays within the array
    unsigned char hexCount;                     // used to count in hex for writing to the displays

    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    // set display and LED SR chip select ports to output, initializing chip selects as inactive
    DISPS_CSDIR |= ALL_DISPS;
    LEDSR_CSDIR |= LEDSR;
    DSEL_ALL_DISPS;
    DSEL_LEDSR;

    // init USCI_A0 in master mode, sclkdiv of 1, sclk active high with change on first edge, 8-bit mode, MSB first, with loopback
    usciXNSpiInit(&USCIA0SPI, SPI_MST, 1, (~UCCKPH & ~UCCKPL), SPI_DAT8BIT, SPI_MSB, SPI_LOOPBACK);

    // initialize all displays as active high, and to represent an OFF state
    for (dispIndex = 0; dispIndex < NUM_DISPS; dispIndex++)
        sevSegDispArr[dispIndex] = (SEVEN_SEG_DISP){0, OFF_CODE, 0, 0x0, 0x0};

    // turn off all LEDs
    SEL_LEDSR;
    usciXNSpiPutChar(&USCIA0SPI, ~ALL_LEDS);
    DSEL_LEDSR;

    // test SPI bus write to all display SRs at the same time, actually writing the OFF state to them
    SEL_ALL_DISPS;
    usciXNSpiPutChar(&USCIA0SPI, 0x00);
    DSEL_ALL_DISPS;

    // MAIN LOOP
    while(1)
    {
        // count from 0x0-0xF on all displays at the same time
        for (hexCount = 0; hexCount <= 0xF; hexCount++)
        {
            for (dispIndex = 0; dispIndex < NUM_DISPS; dispIndex++)
                writeHexToSevSeg(&USCIA0SPI, &sevSegDispArr[dispIndex], hexCount, dispIndex);

            __delay_cycles(COUNT_DELAY_CYCLES);
        }

        // turn all displays off before the next test sequence
        for (dispIndex = 0; dispIndex < NUM_DISPS; dispIndex++)
            writeHexToSevSeg(&USCIA0SPI, &sevSegDispArr[dispIndex], OFF_CODE, dispIndex);

        // count on each display individually, one at a time, from OFF to 0x0-0xF to OFF again
        for (dispIndex = 0; dispIndex < NUM_DISPS; dispIndex++)
        {
            for (hexCount = 0; hexCount <= 0xF; hexCount++)
            {
                // write the current count to the current display and convert to binary segment code
                writeHexToSevSeg(&USCIA0SPI, &sevSegDispArr[dispIndex], hexCount, dispIndex);
                __delay_cycles(COUNT_DELAY_CYCLES);
            }

            writeHexToSevSeg(&USCIA0SPI, &sevSegDispArr[dispIndex], OFF_CODE, dispIndex);
        }

        // flash all segments once with a single SPI bus write, to see if there's a speed difference
        SEL_ALL_DISPS;
        usciXNSpiPutChar(&USCIA0SPI, 0xFF);
        __delay_cycles(COUNT_DELAY_CYCLES);
        usciXNSpiPutChar(&USCIA0SPI, 0x00);
        DSEL_ALL_DISPS;
        __delay_cycles(COUNT_DELAY_CYCLES);

        // shift through all LEDs (there will obviously be no action for any LEDs not connected to the control board)
        SEL_LEDSR;
        for (hexCount = 0; hexCount < 0x8; hexCount++)
        {
            usciXNSpiPutChar(&USCIA0SPI, BIT0 << hexCount);
            __delay_cycles(COUNT_DELAY_CYCLES);
        }
        usciXNSpiPutChar(&USCIA0SPI, 0x0);
        DSEL_LEDSR;
        __delay_cycles(COUNT_DELAY_CYCLES);
    }

    return 0;
}


//########## CLIENT FUNCTIONS ##########//

/************************************************************************************
* Function: writeHexToSevSeg
*
* Description:
*   Converts and writes the given hexCode to the specified display object over the
*   desired SPI peripheral interface. The user must also provide the pin index of
*   the chip select line to the display shift register within its output port; this
*   allows for the most convenient way to address the display.
*
*   This function also checks that the display object is in an expected state to
*   ensure nothing has gone wrong; nextBinSegCode and currBinSegCode should always
*   be matching to represent a successful hexCode write to the display, and a steady
*   state of displaying information. These two members could therefore be used to
*   detect when a hexCode conversion has taken place, but the code has not yet been
*   written to the display. Since this is all handled in this function, the two
*   members should always match upon entry to this function; if this is not the case,
*   the inconsistency is detected and the hexCode is not written to the display.
*   A nonzero value is then returned from this function.
*
* Arguments:
*   *usciXN     -   pointer to the the USCI peripheral object
*   *display    -   pointer to the 7seg display object
*   hexCode     -   8-bit hexadecimal code to convert and write to the display
*   csPortIndex -   the display SR's chip select pin within the chip select port
*                   defined as a symbolic constant above (DISPS_CSOUT)
*                   (example: P3.2 would have csPortIndex == 2)
*
* Returns:
*   char inconsistency; 0 if the state of the display object was normal, nonzero if
*   the current and next binary segment codes were out of sync.
*
* Author:       Mason Kury
* Created:      November 10, 2022
* Modified:     November 10, 2022
************************************************************************************/
unsigned char writeHexToSevSeg(const USCIXNSPI* usciXN, SEVEN_SEG_DISP* display, unsigned char hexCode, unsigned char csPortIndex)
{
    unsigned char inconsistency = 0;

    // only update display if a new code was passed; check that the display is in an expected state as well
    if ((hexCode != display->hexDigit) && (display->currBinSegCode == display->nextBinSegCode))
    {
        // write the hex code to the display, and convert to binary segment code
        display->hexDigit = hexCode;
        hexToSevSeg(display);

        // activate display's chip select, write the binary segment code over SPI, then deactivate chip select
        DISPS_CSOUT |= (DISP0 << csPortIndex);
        usciXNSpiPutChar(usciXN, (display->nextBinSegCode));
        DISPS_CSOUT &= ~(DISP0 << csPortIndex);

        // what was previously nextBinSegCode is now currBinSegCode
        display->currBinSegCode = display->nextBinSegCode;
    }
    // if the current and next codes didn't match before updating the hex code, there is something wrong
    else if (display->nextBinSegCode != display->currBinSegCode)
        inconsistency = 1;

    return inconsistency;
}


//########## INTERRUPT SERVICE ROUTINES ##########//

