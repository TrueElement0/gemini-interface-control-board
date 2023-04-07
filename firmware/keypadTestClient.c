/************************************************************************************
* Gemini Interface Control Board -- Matrix Keypad Test Client
* Property of Super Props Inc., all rights reserved
*
* Client file to test Gemini keypad operation. Each normal key on the keypad
* triggers an interrupt when pressed, initiating the coordinate search process.
* Upon a second interrupt, when a key is released, the key coordinate is stored
* if a coordinate was successfully found. These coordinates are defined in the enum
* within this file, corresponding to the actual buttons on a typical Gemini unit.
* In the main loop, these coordinates are decoded and display a hex value from
* 0x0 to 0xC on display 0 to verify functionality.
*
* The power button is connected via two separate pins on the same keypad connector.
* These pins are connected to a separate port on the MSP430 with its own ISR. Upon
* a button press, the ON/OFF state of the displays is toggled through the ISR to
* verify its functionality as well.
*
* Author:       Mason Kury
* Created:      November 12, 2022
* Modified:     November 20, 2022
************************************************************************************/


//########## DEPENDENCIES ##########//
#include <msp430.h>
#include "spi.h"
#include "sevenSeg.h"
#include "mtrxKeypad.h"


//########## SYMBOLIC CONSTANTS ##########//

// display registers and chip select pins
#define DISPS_CSDIR             P3DIR
#define DISPS_CSOUT             P3OUT
#define DISP0                   BIT0
#define DISP1                   BIT1
#define DISP2                   BIT2
#define DISP3                   BIT3
#define DISP4                   BIT4
#define DISP5                   BIT5
#define DISP6                   BIT6
#define DISP7                   BIT7
#define TOP_DISPS               0x0F
#define BOT_DISPS               0xF0
#define ALL_DISPS               0xFF

// LED registers and chip select pins
#define LEDSR_CSDIR             P1DIR
#define LEDSR_CSOUT             P1OUT
#define LEDSR                   BIT6
#define LED0                    BIT0
#define LED1                    BIT1
#define LED2                    BIT2
#define LED3                    BIT3
#define LED4                    BIT4
#define LED5                    BIT5
#define LED6                    BIT6
#define LED7                    BIT7
#define ALL_LEDS                0xFF

// keypad power button registers and pin (separate from the rest of the keypad)
#define KEYPAD_PWR_DIR          P1DIR
#define KEYPAD_PWR_REN          P1REN
#define KEYPAD_PWR_OUT          P1OUT
#define KEYPAD_PWR_IN           P1IN
#define KEYPAD_PWR_IE           P1IE
#define KEYPAD_PWR_IES          P1IES
#define KEYPAD_PWR_IFG          P1IFG
#define KEYPAD_PWR_BTN          BIT0

// change back to PORT2 for MSP430G2253
#define PWRBTN_ISR_VECTOR       PORT1_VECTOR    // keypad power button port vector for interrupts
#define KEYPAD_ISR_VECTOR       PORT2_VECTOR    // keypad port vector for interrupts (should be interrupt vector for row port)

#define FLAG_KEYPAD_PRESS       BIT0            // a bit within sysState representing a keypress event
#define FLAG_PWR_OFF            BIT1            // a bit within sysState representing a power-off state

#define PWR_BTN_PRESS_DELAY     100000          // number of MCLK cycles to delay when the power button is pressed
#define PWR_BTN_RELEASE_DELAY   300000          // number of MCLK cycles to delay when the power button is released

#define NUM_DISPS               8               /* Do not set to be greater than 8, as byte-size shifting is done for display index manipulation.
                                                 * If a port wider than 1 byte is used and properly defined for use with the writeHexToSevSeg function,
                                                 * you can set NUM_DISPS up to the width of port.  */

// SEE "mtrxKeypad.h" FOR COORDINATE FORMAT EXPLANATION
enum GEMINI_KEYS
{
    CC_MONITOR      = 0x40,
    PAUSE_STOP      = 0x60,
    RATE            = 0x51,
    VTBI            = 0x61,
    START           = 0x71,
    HUNDRED         = 0x42,
    TEN             = 0x52,
    ONE             = 0x62,
    TENTH           = 0x72,
    CLEAR_SILENCE   = 0x43,
    PC_MODE         = 0x53,
    SEC_PIGGY_BACK  = 0x63,
    VOLUME_INFUSED  = 0x73
};


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

/* These global variables keep track of system flags set off by ISRs; each bit is as defined by the symbolic constant above.
 * All ISRs and functions dealing with a sysState variable should update both previous and current, allowing for easy detection
 * of system state changes. */
unsigned char currSysState = 0x00;
unsigned char prevSysState = 0x00;

// define registers and pin masks for a matrix keypad with row pins PORT2<3:0> and column pins PORT2<7:4>
static MATRIX_KEYPAD geminiKeypad = {&P2IN, &P2OUT, &P2DIR, &P2SEL, &P2REN, &P2IE, &P2IES, &P2IFG, &P2OUT, &P2DIR, &P2SEL, 0x0F, 0xF0};


//########## FUNCTION PROTOTYPES ##########//
unsigned char writeHexToSevSeg(const USCIXNSPI* usciXN, SEVEN_SEG_DISP* display, unsigned char hexCode, unsigned char csPortIndex);
__inline void initKeypadDelayTimer();
__inline void initPwrBtn();


//########## MAIN FUNCTION ##########//
void main(void)
{
    // define registers for USCI_A0 on PORT1, with only SOMI and SCLK; this peripheral will run with loopback, as no SOMI is needed
    const USCIXNSPI USCIA0SPI = {&P1SEL, &P1SEL2, 0x0, BIT2, 0x0, BIT4, &UCA0CTL0, &UCA0CTL1, &UCA0BR0, &UCA0BR1, &UCA0STAT, &UCA0TXBUF, &UCA0RXBUF, &IFG2, UCA0TXIFG, UCA0RXIFG};

    SEVEN_SEG_DISP sevSegDispArr[NUM_DISPS];    // an array of seven segment displays, representing the 8 on the gemini interface
    unsigned char dispIndex;                    // used to index displays within the array
    unsigned char hexCode;                      // used to store a hex code to write to a display; saves memory in button decode switch statement
    unsigned char invalidKeyPress;              // used to store the validity of a keypress scan

    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    // set display and LED SR chip select ports to output, initializing chip selects as inactive
    DISPS_CSDIR |= ALL_DISPS;
    LEDSR_CSDIR |= LEDSR;
    DSEL_ALL_DISPS;
    DSEL_LEDSR;

    // initialize all displays as active high, and to represent an OFF state
    for (dispIndex = 0; dispIndex < NUM_DISPS; dispIndex++)
        sevSegDispArr[dispIndex] = (SEVEN_SEG_DISP){0, OFF_CODE, 0, 0x0, 0x0};

    // init USCI_A0 in master mode, sclkdiv of 1, sclk active high with capture on first edge, 8-bit mode, MSB first, with loopback
    usciXNSpiInit(&USCIA0SPI, SPI_MST, 1, (~UCCKPH & ~UCCKPL), SPI_DAT8BIT, SPI_MSB, SPI_LOOPBACK);
    mtrxKeypadInit(&geminiKeypad);
    initPwrBtn();
    initKeypadDelayTimer();

    // turn off all LEDs
    SEL_LEDSR;
    usciXNSpiPutChar(&USCIA0SPI, ~ALL_LEDS);
    DSEL_LEDSR;

    // clear all displays with one SPI write
    SEL_ALL_DISPS;
    usciXNSpiPutChar(&USCIA0SPI, ~ALL_DISPS);
    DSEL_ALL_DISPS;

    writeHexToSevSeg(&USCIA0SPI, &sevSegDispArr[0], 0x0, 0);

    // MAIN LOOP
    while(1)
    {
        // keypad event occurred; button was either pressed or released
        if ((currSysState ^ prevSysState) & FLAG_KEYPAD_PRESS)
        {
            // key is being pressed, so scan to determine the coordinates
            if (currSysState & FLAG_KEYPAD_PRESS)
            {
                invalidKeyPress = scanForKeyPress(&geminiKeypad);

                // if the key press was invalid, immediately clear the keypad event flag
                // this should ensure an invalid key press is never saved, and no unexpected actions occur because of it
                if (invalidKeyPress)
                    currSysState &= ~FLAG_KEYPAD_PRESS;
            }

            // key was released and a valid key was scanned, so save the key data and perform the appropriate action
            else
            {
                saveKeyPress(&geminiKeypad);

                switch (geminiKeypad.currKeyCoord)
                {
                case CC_MONITOR:
                    dispIndex = 0;
                    hexCode = 0x1;
                    break;
                case PAUSE_STOP:
                    dispIndex = 0;
                    hexCode = 0x2;
                    break;
                case RATE:
                    dispIndex = 0;
                    hexCode = 0x3;
                    break;
                case VTBI:
                    dispIndex = 0;
                    hexCode = 0x4;
                    break;
                case START:
                    dispIndex = 0;
                    hexCode = 0x5;
                    break;
                case HUNDRED:
                    dispIndex = 0;
                    hexCode = 0x6;
                    break;
                case TEN:
                    dispIndex = 0;
                    hexCode = 0x7;
                    break;
                case ONE:
                    dispIndex = 0;
                    hexCode = 0x8;
                    break;
                case TENTH:
                    dispIndex = 0;
                    hexCode = 0x9;
                    break;
                case CLEAR_SILENCE:
                    dispIndex = 0;
                    hexCode = 0xA;
                    break;
                case PC_MODE:
                    dispIndex = 0;
                    hexCode = 0xB;
                    break;
                case SEC_PIGGY_BACK:
                    dispIndex = 0;
                    hexCode = 0xC;
                    break;
                case VOLUME_INFUSED:
                    dispIndex = 0;
                    hexCode = 0xD;
                    break;
                default:
                    dispIndex = 0;
                    hexCode = OFF_CODE;
                    break;
                }
                writeHexToSevSeg(&USCIA0SPI, &sevSegDispArr[dispIndex], hexCode, dispIndex);
            }

            // the keypress button flag should match in previous and current system states now that it has been handled
            prevSysState ^= ((prevSysState ^ currSysState) & FLAG_KEYPAD_PRESS);
        }
        // power button was pressed
        if ((currSysState ^ prevSysState) & FLAG_PWR_OFF)
        {
            // debounce press, wait for power button release, then debounce again
            __delay_cycles(PWR_BTN_PRESS_DELAY);
            while(KEYPAD_PWR_IN & KEYPAD_PWR_BTN);
            __delay_cycles(PWR_BTN_RELEASE_DELAY);

            // enter power OFF state
            if (currSysState & FLAG_PWR_OFF)
            {
                writeHexToSevSeg(&USCIA0SPI, &sevSegDispArr[0], OFF_CODE, 0);
            }
            // enter power ON state
            else
            {
                writeHexToSevSeg(&USCIA0SPI, &sevSegDispArr[0], 0x0, 0);
                *(geminiKeypad.ROW_IFG) &= ~(geminiKeypad.ROW_PINS);        // clear any keypad flags
                *(geminiKeypad.ROW_IE) |= (geminiKeypad.ROW_PINS);          // enable keypad interrupts
            }

            // the power button flag should match in previous and current system states now that it has been handled
            prevSysState ^= ((prevSysState ^ currSysState) & FLAG_PWR_OFF);

            KEYPAD_PWR_IFG &= ~KEYPAD_PWR_BTN;  // clear any unwanted power button flags
            KEYPAD_PWR_IE |= KEYPAD_PWR_BTN;    // power button interrupts can now be enabled again
        }
    }
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

__inline void initKeypadDelayTimer()
{
    BCSCTL1 &= ~(BIT4 | BIT5 | XTS);            // ensure no ACLK division, and low-frequency mode for LFXT1 to allow for VLOCLK selection
    BCSCTL3 |= LFXT1S_2;                        // set ACLK source to VLOCLK (12kHz)

    TA0CTL = TASSEL_1 | MC_1;                   // set source as ACLK, no clock division, count up mode
    TA0CCTL0 |= CCIE;                           // enable timer interruptss
    TA0CCTL0 &= ~CCIFG;                         // clear timer interrupt flags
    __enable_interrupt();                       // enable global interrupts
}

__inline void initPwrBtn()
{
    // set power button pin as an input pulled HIGH
    KEYPAD_PWR_DIR &= ~KEYPAD_PWR_BTN;
    KEYPAD_PWR_REN |= KEYPAD_PWR_BTN;
    KEYPAD_PWR_OUT |= KEYPAD_PWR_BTN;

    // enable power button interrupts
    KEYPAD_PWR_IES |= KEYPAD_PWR_BTN;   // edge select H -> L
    KEYPAD_PWR_IFG &= ~KEYPAD_PWR_BTN;  // clear unwanted flags
    KEYPAD_PWR_IE |= KEYPAD_PWR_BTN;    // enable interrupts
    __enable_interrupt();               // ensure global interrupts are enabled
}


//########## INTERRUPT SERVICE ROUTINES ##########//
#pragma vector = KEYPAD_ISR_VECTOR
__interrupt void keypadPressISR(void)
{
    // ensure one of the row pins triggered the interrupt
    if (*(geminiKeypad.ROW_IFG) & (geminiKeypad.ROW_PINS))
    {
        // turn keypad interrupts off until press is handled by a scanKeyPress call
        *(geminiKeypad.ROW_IE) &= ~(geminiKeypad.ROW_PINS);
        *(geminiKeypad.ROW_IFG) &= ~(geminiKeypad.ROW_PINS);

        /* Before registering the button press, if the button was pressed, set the timer to wait for a release debounce delay.
         * Otherwise, have the timer wait for the press debounce delay. These are configured in "mtrxKeypad.h" */
        TA0CCR0 = (currSysState & FLAG_KEYPAD_PRESS) ? RELEASE_DBNC_DELAY : PRESS_DBNC_DELAY;
    }
    else
    {
        // if it wasn't a row pin that triggered the interrupt, shut off non-row interrupts and clear flags
        *(geminiKeypad.ROW_IE) &= (geminiKeypad.ROW_PINS);
        *(geminiKeypad.ROW_IFG) &= (geminiKeypad.ROW_PINS);
    }
}

#pragma vector = PWRBTN_ISR_VECTOR
__interrupt void pwrbtnPressISR(void)
{
    // ensure the power button triggered the interrupt
    if (KEYPAD_PWR_IFG & KEYPAD_PWR_BTN)
    {
        TA0CCR0 = 0;                                            // stop debouncing
        prevSysState &= ~FLAG_KEYPAD_PRESS;                     // clear any kind of pending button press
        currSysState &= ~FLAG_KEYPAD_PRESS;
        *(geminiKeypad.ROW_IE) &= ~(geminiKeypad.ROW_PINS);     // shut off keypad interrupts
        *(geminiKeypad.ROW_IES) &= ~(geminiKeypad.ROW_PINS);    // reset the edge select to wait for a press (L -> H)
        *(geminiKeypad.ROW_IFG) &= ~(geminiKeypad.ROW_PINS);    // clear any keypad flags

        currSysState ^= FLAG_PWR_OFF;                           // toggle the system power state
        KEYPAD_PWR_IE &= ~KEYPAD_PWR_BTN;                       // turn off power button interrupts until debounced
        KEYPAD_PWR_IFG &= ~KEYPAD_PWR_BTN;                      // and clear the power button interrupt
    }
    else
    {
        // if it wasn't the power button that triggered the interrupt, shut off non-power button interrupts and clear flags
        KEYPAD_PWR_IE &= KEYPAD_PWR_BTN;
        KEYPAD_PWR_IFG &= KEYPAD_PWR_BTN;
    }
}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void timer0A0ISR(void)
{
    prevSysState ^= ((prevSysState ^ currSysState) & FLAG_KEYPAD_PRESS);    // toggle previous key state to match current if it differs from the current
    currSysState ^= FLAG_KEYPAD_PRESS;                                      // officially register a keypad event, now that debouncing is complete
    TA0CCR0 = 0;                                                            // stop the timer, now that the debounce delay is complete
}
