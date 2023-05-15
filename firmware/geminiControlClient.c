/************************************************************************************
* Gemini Interface Control Board -- Seven Segment Display Test Client
* Property of Super Props Inc., all rights reserved
*
* Contains all functionality necessary to emulate a Gemini IV pump user interface
* using 7-segment displays and the existing matrix keypad on the unit.
*
* There are several states associated with operation:
*   ON:
*       Upon first entry into this state, all displays will show a "-", and all LEDs
*       will be off. "100", "10", "1", and, "0.1" buttons are not functional in this
*       state. The blank button on the top left of the keypad can be pressed to cycle
*       through LEDs that light up "COMPUTER CONTROL", the button itself, and "MONITOR".
*       The "P/C MODE" button can be pressed to cycle through the "CONTROLLER" and
*       "PUMP" LEDs being lit. The "SEC PIGGY BACK" button can be pressed to toggle
*       the LED under that button on or off. The "VOLUME INFUSED" button can be pressed
*       to cycle through the battery indicator and power plug indicator LEDs. The
*       "CLEAR/SILENCE" button can be pressed to turn off all LEDs and reset the
*       displays to their initial "-" values.
*
*       While in the ON state, the RATE button enters the "RATE EDIT" state, the
*       VTBI button enters the "VTBI EDIT" state, the START button enters the
*       "PUMP ACTIVE" state, and the "POWER ON" button enters the OFF state.
*
*   OFF:
*       All displays and LEDs are turned off, except for the plug power LED;
*       however, their previous values are usually retained so they can be restored
*       when powering back ON.
*
*       While in the OFF state, the "POWER ON" button enters the ON state.
*
*   RATE EDIT:
*       Upon first entry into this state, the top (RATE) row of displays will change
*       to "[][]0[]". Upon any entry into this state, the RATE row will flash twice.
*       The displays will also reset to this value any time the "CLEAR/SILENCE" button
*       is pressed while in this state. Pressing the "100", "10", "1", and "0.1" buttons
*       will increment the displayed number as described in detail within the incDispRow
*       function header below. None of the LED control buttons are functional in this
*       state.
*
*       While in the "RATE EDIT" state, both the "RATE" and "PAUSE/STOP" buttons enter
*       the ON state, the VTBI button enters the "VTBI EDIT" state, the START button
*       enters the "PUMP ACTIVE" state, and the "POWER ON" button enters the OFF state.
*
*       Upon exit from this state by pressing the RATE button, the VTBI row will flash
*       once. Upon exit by the "PAUSE/STOP" button, both rows will flash once.
*
*   VTBI EDIT:
*       Upon first entry into this state, the bottom (VTBI) row of displays will change
*       to "[][]0[]". Upon any entry into this state, the VTBI row will flash twice.
*       The displays will also reset to this value any time the "CLEAR/SILENCE" button
*       is pressed while in this state. Pressing the "100", "10", "1", and "0.1" buttons
*       will increment the displayed number as described in detail within the incDispRow
*       function header below. None of the LED control buttons are functional in this
*       state.
*
*       While in the "VTBI EDIT" state, both the "VTBI" and "PAUSE/STOP" buttons enter
*       the ON state, the RATE button enters the "RATE EDIT" state, the START button
*       enters the "PUMP ACTIVE" state, and the "POWER ON" button enters the OFF state.
*
*       Upon exit from this state by pressing the VTBI button, the VTBI row will flash
*       once. Upon exit by the "PAUSE/STOP" button, both rows will flash once.
*
*   PUMP ACTIVE:
*       Upon any entry into this state, both display rows will flash twice.
*       In this state, no buttons are functional except the "PAUSE/STOP" button, the
*       "POWER ON" button, and the START button (though it just re-enters this state.)
*       This state basically puts the device into a locked mode, safe from unintended
*       value changes on set.
*
*       While in the "PUMP ACTIVE" state, the "PAUSE/STOP" button enters the ON state,
*       and the "POWER ON" button enters the OFF state. Pressing START again does not
*       change states, but will still flash both rows twice.
*
*       Upon exit from this state by pressing the "PAUSE/STOP" button, both rows will
*       flash once.
*
* Author:       Mason Kury
* Created:      November 30, 2022
* Modified:     March 27, 2023
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
#define LED_CONTROLLER          BIT0
#define LED_PUMP                BIT1
#define LED_CC                  BIT2
#define LED_BLANKBUTTON         BIT3
#define LED_MONITOR             BIT4
#define LED_SECPIGGYBACK        BIT5
#define LED_BATTPWR             BIT6
#define LED_PLUGPWR             BIT7
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

#define PWRBTN_ISR_VECTOR       PORT1_VECTOR    // keypad power button port vector for interrupts
#define KEYPAD_ISR_VECTOR       PORT2_VECTOR    // keypad port vector for interrupts (should be interrupt vector for row port)

// sysState bit definitions
#define FLAG_KEYPAD_PRESS       BIT0            // indicates a keypress event
#define FLAG_PWR_OFF            BIT1            // represents a power-off state
#define FLAG_RATE_EDIT          BIT2            // represents a RATE value editing state on the interface
#define FLAG_VTBI_EDIT          BIT3            // represents a VTBI value editing state on the interface
#define FLAG_PUMP_ACTIVE        BIT4            // represents an active "pump"
#define FLAG_RATE_VALUE         BIT5            // represents a user-defined RATE value, rather than the default "----"
#define FLAG_VTBI_VALUE         BIT6            // represents a user-defined VTBI value, rather than the default "----"

#define PWR_BTN_PRESS_DELAY     100000          // number of MCLK cycles to delay when the power button is pressed
#define PWR_BTN_RELEASE_DELAY   300000          // number of MCLK cycles to delay when the power button is released
#define DISP_FLASH_DELAY        262000          // number of MCLK cycles to delay when flashing displays as an indication
#define STARTUP_DELAY           1048000         // number of MCLK cycles to delay on boot before initializing the main subsystems

#define TOP_ROW                 0               // represents a write to the top row of displays; used to make calls to writeToDispRow easier to read
#define BOT_ROW                 1               // represents a write to the bottom row of displays
#define ALL_ROWS                2               // represents access to both rows of displays (only used for calls to flashDispRow and blinkDispRow)

#define HUNDREDS_PLACE          0               // used to increment the hundreds place within a display row
#define TENS_PLACE              1               // used to increment the tens place within a display row
#define ONES_PLACE              2               // used to increment the ones place within a display row
#define TENTHS_PLACE            3               // used to increment the tenths place within a display rowt

#define NUM_DISPS               8               /* Do not set to be greater than 8, as byte-size shifting is done for display index manipulation.
                                                 * If a port wider than 1 byte is used and properly defined for use with the writeHexToSevSeg function,
                                                 * you can set NUM_DISPS up to the width of port.  */

// SEE "mtrxKeypad.h" FOR COORDINATE FORMAT EXPLANATION
enum GEMINI_KEYS
{
    CC_MONITOR      = 0x40,
    PAUSE_STOP_DOWN = 0x60,     // Most keypads have pause/stop on (col2, row0), but some variants have a down arrow there instead.
    PAUSE_STOP_ALT  = 0x41,     // On keypads with the up and down arrows, pause/stop is located at (col0, row1)
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
#define SEL_EVERYTHING  DISPS_CSOUT |= ALL_DISPS; LEDSR_CSOUT |= LEDSR

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
#define DSEL_EVERYTHING DISPS_CSOUT &= ~ALL_DISPS; LEDSR_CSOUT &= ~LEDSR



//########## GLOBALS ##########//

/* These global variables keep track of system flags set off by ISRs; each bit is as defined by the symbolic constant above.
 * All ISRs and functions dealing with a sysState variable should update both previous and current, allowing for easy detection
 * of system state changes. */
volatile unsigned char currSysState = 0x00;
volatile unsigned char prevSysState = 0x00;

// define registers and pin masks for a matrix keypad with row pins PORT2<3:0> and column pins PORT2<7:4>
static MATRIX_KEYPAD geminiKeypad = {&P2IN, &P2OUT, &P2DIR, &P2SEL, &P2REN, &P2IE, &P2IES, &P2IFG, &P2OUT, &P2DIR, &P2SEL, 0x0F, 0xF0};


//########## FUNCTION PROTOTYPES ##########//
static unsigned char writeHexToSevSeg(const USCIXNSPI* usciXN, SEVEN_SEG_DISP *const display, const unsigned char hexCode, const unsigned char csPortIndex);
static unsigned char writeToDispRow(const USCIXNSPI* usciXN, SEVEN_SEG_DISP *const displayArr, const unsigned char (*rowDataArr)[2], unsigned char botRow);
static unsigned char incDispRow(const USCIXNSPI* usciXN, SEVEN_SEG_DISP *const displayArr, const unsigned char digitPos, const unsigned char botRow);
static void refreshAllDisps(const USCIXNSPI* usciXN, SEVEN_SEG_DISP *const displayArr);
static void writeToRowBuff(unsigned char (*const rowBuff)[2], const unsigned char dat0, const unsigned char dp0, const unsigned char dat1, const unsigned char dp1, const unsigned char dat2, const unsigned char dp2, const unsigned char dat3, const unsigned char dp3);
static void flashDispRow(const USCIXNSPI* usciXN, SEVEN_SEG_DISP *const displayArr, const unsigned char rowSel, const unsigned char numFlashes);
static void criticalFaultHandler(const USCIXNSPI* usciXN, SEVEN_SEG_DISP *const displayArr, unsigned char (*const rowBuff)[2]);
static void disableKeypad();
__inline static void enableKeypad();
__inline static void initKeypadDelayTimer();
__inline static void initPwrBtn();


//########## MAIN FUNCTION ##########//
void main(void)
{
    // define registers for USCI_A0 on PORT1, with only SOMI and SCLK; this peripheral will run with loopback, as no SOMI is needed
    const USCIXNSPI USCIA0SPI = {&P1SEL, &P1SEL2, 0x0, BIT2, 0x0, BIT4, &UCA0CTL0, &UCA0CTL1, &UCA0BR0, &UCA0BR1, &UCA0STAT, &UCA0TXBUF, &UCA0RXBUF, &IFG2, UCA0TXIFG, UCA0RXIFG};

    SEVEN_SEG_DISP sevSegDispArr[NUM_DISPS];    // an array of seven segment displays, representing the 8 on the gemini interface
    unsigned char nextLedSRState;               // next state of the LED shift register; compared with currnextLedSRState to only do SPI writes when a change occurs
    unsigned char currLedSRState;               // current state of the LED shift register

    unsigned char rowDataBuff[4][2];            // 4x2 array used to store row data before passing by reference to writeToDispRow; see function header for format definition

    unsigned char dispIndex;                    // used to index displays within the array (usually within a loop)
    unsigned char hexCode;                      // used to store various hex digits when counting on displays
    unsigned char invalidKeyPress;              // used to store the validity of a keypress scan

    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    // set display and LED SR chip select ports to output, initializing chip selects as inactive
    DISPS_CSDIR |= ALL_DISPS;
    LEDSR_CSDIR |= LEDSR;
    DSEL_ALL_DISPS;
    DSEL_LEDSR;

    __delay_cycles(STARTUP_DELAY);  // delay before initializing keypad and other subsystems to avoid interference from AC power transients

    // initialize all displays as active high, representing "----" being displayed on each row
    // this allows the interface to boot up in an OFF state, able to resume to these values upon turning ON
    for (dispIndex = 0; dispIndex < NUM_DISPS; dispIndex++)
    {
        sevSegDispArr[dispIndex] = (SEVEN_SEG_DISP){0, DASH_CODE, 0, 0x0, 0x0};
        hexToSevSeg(&sevSegDispArr[dispIndex]);
        sevSegDispArr[dispIndex].currBinSegCode = sevSegDispArr[dispIndex].nextBinSegCode;
    }

    // initialize the LED shift register to have the pump, computer control, and plug power LEDs lit upon power on
    currLedSRState = LED_PUMP | LED_CC | LED_PLUGPWR;
    nextLedSRState = currLedSRState;

    // init USCI_A0 in master mode, sclkdiv of 1, sclk active high with capture on first edge, 8-bit mode, MSB first, with loopback
    usciXNSpiInit(&USCIA0SPI, SPI_MST, 1, (~UCCKPH & ~UCCKPL), SPI_DAT8BIT, SPI_MSB, SPI_LOOPBACK);
    mtrxKeypadInit(&geminiKeypad);
    initPwrBtn();
    initKeypadDelayTimer();

    // turn off all displays
    SEL_ALL_DISPS;
    usciXNSpiPutChar(&USCIA0SPI, 0x0);
    DSEL_ALL_DISPS;

    // set the plug power LED on and all others off
    SEL_LEDSR;
    usciXNSpiPutChar(&USCIA0SPI, LED_PLUGPWR);
    DSEL_LEDSR;

    // ensure the system starts up in an OFF state
    disableKeypad();
    currSysState |= FLAG_PWR_OFF;

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

                /* if the key press was invalid, immediately clear the keypad event flag and ensure previous and current states match;
                   this should ensure an invalid key press is never saved, and no unexpected actions occur because of it */
                if (invalidKeyPress)
                {
                    currSysState &= ~FLAG_KEYPAD_PRESS;
                    prevSysState &= ~FLAG_KEYPAD_PRESS;
                }
                /* as far as the next loop is concerned, the keypad was previously pressed -- there is no need to match previous and current states;
                 * this should ensure that a release event can occur smoothly even during this press check branch,
                 * and still trigger a release handling in the next loop*/
                else
                    prevSysState |= FLAG_KEYPAD_PRESS;
            }

            // key was released and a valid key was scanned, so save the key data and perform the appropriate action
            else
            {
                saveKeyPress(&geminiKeypad);

                switch (geminiKeypad.currKeyCoord)
                {
                case CC_MONITOR:
                    if (!(currSysState & FLAG_PUMP_ACTIVE))         // disable functionality while pump is active
                    {
                        if (!(((currLedSRState & LED_CC) ^ (currLedSRState & LED_BLANKBUTTON)) \
                                || ((currLedSRState & LED_BLANKBUTTON) ^ (currLedSRState & LED_MONITOR)) \
                                || ((currLedSRState & LED_CC) ^ (currLedSRState & LED_MONITOR))))
                        {
                            // if none or multiple of the CC, BLANKBUTTON, or MONITOR LEDs are active, set only the CC LED active
                            nextLedSRState &= ~(LED_BLANKBUTTON | LED_MONITOR);
                            nextLedSRState |= LED_CC;
                        }
                        else if (currLedSRState & LED_CC)           // cycle from CC LED to BLANKBUTTON LED
                        {
                            nextLedSRState &= ~LED_CC;
                            nextLedSRState |= LED_BLANKBUTTON;
                        }
                        else if (currLedSRState & LED_BLANKBUTTON)  // cycle from BLANKBUTTON LED to MONITOR LED
                        {
                            nextLedSRState &= ~LED_BLANKBUTTON;
                            nextLedSRState |= LED_MONITOR;
                        }
                        else if (currLedSRState & LED_MONITOR)      // cycle from MONITOR LED back to all 3 LEDs OFF
                        {
                            nextLedSRState &= ~LED_MONITOR;
                        }
                    }
                    break;

                case PAUSE_STOP_DOWN:   // scanning either coordinate on either variant should have the same result
                case PAUSE_STOP_ALT:
                    currSysState &= ~(FLAG_RATE_EDIT | FLAG_VTBI_EDIT);     // exit any ongoing value edits
                    currSysState &= ~FLAG_PUMP_ACTIVE;                      // the "pump" is no longer active
                    flashDispRow(&USCIA0SPI, sevSegDispArr, ALL_ROWS, 1);   // flash RATE and VTBI rows once at the same time
                    break;

                case RATE:
                    if (!(currSysState & FLAG_PUMP_ACTIVE))                         // do not allow user to edit RATE value while pump is active
                    {
                        *(geminiKeypad.ROW_IE) &= ~(geminiKeypad.ROW_PINS);         // shut off keypad interrupts

                        currSysState &= ~FLAG_VTBI_EDIT;                            // exit any ongoing edit of the VTBI value
                        currSysState ^= FLAG_RATE_EDIT;                             // either enter or exit a RATE value edit state

                        if (currSysState & FLAG_RATE_EDIT)                          // entering the RATE edit state
                        {
                            if (!(currSysState & FLAG_RATE_VALUE))                  // if there is no user-defined RATE value, set it to "[][]0[]"
                            {
                                writeToRowBuff(rowDataBuff, OFF_CODE, 0, OFF_CODE, 0, 0x0, 0, OFF_CODE, 0);
                                writeToDispRow(&USCIA0SPI, sevSegDispArr, rowDataBuff, TOP_ROW);
                                currSysState |= FLAG_RATE_VALUE;
                            }
                            flashDispRow(&USCIA0SPI, sevSegDispArr, TOP_ROW, 2);    // blink the RATE display row twice
                        }

                        else                                                        // exiting the RATE edit state
                            flashDispRow(&USCIA0SPI, sevSegDispArr, TOP_ROW, 1);    // flash the RATE display row once

                        *(geminiKeypad.ROW_IFG) &= ~(geminiKeypad.ROW_PINS);        // clear any keypad flags
                        *(geminiKeypad.ROW_IE) |= (geminiKeypad.ROW_PINS);          // re-enable keypad interrupts
                    }
                    break;

                case VTBI:
                    if (!(currSysState & FLAG_PUMP_ACTIVE))                         // do not allow user to edit VTBI value while pump is active
                    {
                        *(geminiKeypad.ROW_IE) &= ~(geminiKeypad.ROW_PINS);         // shut off keypad interrupts

                        currSysState &= ~FLAG_RATE_EDIT;                            // exit any ongoing edit of the RATE value
                        currSysState ^= FLAG_VTBI_EDIT;                             // either enter or exit a VTBI value edit state

                        if (currSysState & FLAG_VTBI_EDIT)                          // entering the VTBI edit state
                        {
                            if (!(currSysState & FLAG_VTBI_VALUE))                  // if there is no user-defined VTBI value, set it to "[][]0[]"
                            {
                                writeToRowBuff(rowDataBuff, OFF_CODE, 0, OFF_CODE, 0, 0x0, 0, OFF_CODE, 0);
                                writeToDispRow(&USCIA0SPI, sevSegDispArr, rowDataBuff, BOT_ROW);
                                currSysState |= FLAG_VTBI_VALUE;
                            }
                            flashDispRow(&USCIA0SPI, sevSegDispArr, BOT_ROW, 2);    // blink the VTBI display row twice
                        }

                        else                                                        // exiting the VTBI edit state
                            flashDispRow(&USCIA0SPI, sevSegDispArr, BOT_ROW, 1);    // flash the VTBI display row once

                        *(geminiKeypad.ROW_IFG) &= ~(geminiKeypad.ROW_PINS);        // clear any keypad flags
                        *(geminiKeypad.ROW_IE) |= (geminiKeypad.ROW_PINS);          // re-enable keypad interrupts
                    }
                    break;

                case START:
                    currSysState &= ~(FLAG_RATE_EDIT | FLAG_VTBI_EDIT);     // exit any ongoing value edits
                    currSysState |= FLAG_PUMP_ACTIVE;                       // the "pump" is now activated
                    flashDispRow(&USCIA0SPI, sevSegDispArr, ALL_ROWS, 2);   // blink RATE and VTBI rows twice at the same time
                    break;

                case HUNDRED:
                    if (currSysState & FLAG_RATE_EDIT)
                        incDispRow(&USCIA0SPI, sevSegDispArr, HUNDREDS_PLACE, TOP_ROW);
                    else if (currSysState & FLAG_VTBI_EDIT)
                        incDispRow(&USCIA0SPI, sevSegDispArr, HUNDREDS_PLACE, BOT_ROW);
                    break;

                case TEN:
                    if (currSysState & FLAG_RATE_EDIT)
                        incDispRow(&USCIA0SPI, sevSegDispArr, TENS_PLACE, TOP_ROW);
                    else if (currSysState & FLAG_VTBI_EDIT)
                        incDispRow(&USCIA0SPI, sevSegDispArr, TENS_PLACE, BOT_ROW);
                    break;

                case ONE:
                    if (currSysState & FLAG_RATE_EDIT)
                        incDispRow(&USCIA0SPI, sevSegDispArr, ONES_PLACE, TOP_ROW);
                    else if (currSysState & FLAG_VTBI_EDIT)
                        incDispRow(&USCIA0SPI, sevSegDispArr, ONES_PLACE, BOT_ROW);
                    break;

                case TENTH:
                    if (currSysState & FLAG_RATE_EDIT)
                        incDispRow(&USCIA0SPI, sevSegDispArr, TENTHS_PLACE, TOP_ROW);
                    else if (currSysState & FLAG_VTBI_EDIT)
                        incDispRow(&USCIA0SPI, sevSegDispArr, TENTHS_PLACE, BOT_ROW);
                    break;

                case CLEAR_SILENCE:
                    if (!(currSysState & FLAG_PUMP_ACTIVE)) // only allow a clear then the pump is not active
                    {
                        // reset only the top row to its default value "[][]0[]" if editing the RATE value
                        if (currSysState & FLAG_RATE_EDIT)
                        {
                            writeToRowBuff(rowDataBuff, OFF_CODE, 0, OFF_CODE, 0, 0x0, 0, OFF_CODE, 0);
                            writeToDispRow(&USCIA0SPI, sevSegDispArr, rowDataBuff, TOP_ROW);
                        }

                        // reset only the bottom row to its default value "[][]0[]" if editing the VTBI value
                        else if (currSysState & FLAG_VTBI_EDIT)
                        {
                            writeToRowBuff(rowDataBuff, OFF_CODE, 0, OFF_CODE, 0, 0x0, 0, OFF_CODE, 0);
                            writeToDispRow(&USCIA0SPI, sevSegDispArr, rowDataBuff, BOT_ROW);
                        }

                        // if user isn't editing anything specific, reset all displays (both rows) to "----" and mark LEDs to reset to pump, cc, and plug power
                        else
                        {
                            writeToRowBuff(rowDataBuff, DASH_CODE, 0, DASH_CODE, 0, DASH_CODE, 0, DASH_CODE, 0);
                            writeToDispRow(&USCIA0SPI, sevSegDispArr, rowDataBuff, TOP_ROW);
                            writeToDispRow(&USCIA0SPI, sevSegDispArr, rowDataBuff, BOT_ROW);
                            currSysState &= ~(FLAG_RATE_VALUE | FLAG_VTBI_VALUE);
                            nextLedSRState = LED_PUMP | LED_CC | LED_PLUGPWR;
                        }
                    }
                    break;

                case PC_MODE:
                    if (!(currSysState & FLAG_PUMP_ACTIVE))                 // disable functionality while pump is active
                    {
                        if (!((currLedSRState & LED_CONTROLLER) ^ (currLedSRState & LED_PUMP)))
                        {                                                   // if neither the PUMP or CONTROLLER LED is ON, or both are ON
                            nextLedSRState &= ~LED_PUMP;                    // ensure the PUMP LED is cleared OFF
                            nextLedSRState |= LED_CONTROLLER;               // and only the CONTROLLER LED is set ON
                        }
                        else if (currLedSRState & LED_CONTROLLER)           // cycle from PUMP LED to CONTROLLER LED
                        {
                            nextLedSRState &= ~LED_CONTROLLER;
                            nextLedSRState |= LED_PUMP;
                        }
                        else if (currLedSRState & LED_PUMP)                 // cycle from CONTROLLER LED to both LEDs OFF
                        {
                            nextLedSRState &= ~LED_PUMP;
                        }
                    }
                    break;

                case SEC_PIGGY_BACK:
                    if (!(currSysState & FLAG_PUMP_ACTIVE)) // disable functionality while pump is active
                    {
                        nextLedSRState ^= LED_SECPIGGYBACK; // simply toggle the LED on the SEC_PIGGY_BACK button whenever it is pressed
                    }
                    break;

                case VOLUME_INFUSED:
                    if (!(currSysState & FLAG_PUMP_ACTIVE))                 // disable functionality while pump is active
                    {
                        if (!((currLedSRState & LED_BATTPWR) ^ (currLedSRState & LED_PLUGPWR)))
                        {                                                   // if neither the BATTPWR or PLUGPWR LED is ON, or both are ON
                            nextLedSRState &= ~LED_PLUGPWR;                 // ensure the PLUGPWR LED is cleared OFF
                            nextLedSRState |= LED_BATTPWR;                  // and only the BATTPWR LED is set ON
                        }
                        else if (currLedSRState & LED_BATTPWR)              // cycle from BATTPWR LED to PLUGPWR LED
                        {
                            nextLedSRState &= ~LED_BATTPWR;
                            nextLedSRState |= LED_PLUGPWR;
                        }
                        else if (currLedSRState & LED_PLUGPWR)              // cycle from PLUGPWR LED to both LEDs OFF
                        {
                            nextLedSRState &= ~LED_PLUGPWR;
                        }
                    }
                    break;
                }

                // clear the keypress button flag, so a press event can potentially register in the next loop, even if it happened during a screen flash delay
                prevSysState &= ~FLAG_KEYPAD_PRESS;
            }

            // if the next LED state differs from the current state, write the new state -- only do this once per button press to avoid SR flicker from constant refreshing
            if (nextLedSRState ^ currLedSRState)
            {
                SEL_LEDSR;
                usciXNSpiPutChar(&USCIA0SPI, nextLedSRState);
                DSEL_LEDSR;

                currLedSRState = nextLedSRState;
            }
        }

        // power button was pressed
        if ((currSysState ^ prevSysState) & FLAG_PWR_OFF)
        {
            // debounce press, wait for power button release, then debounce the release
            __delay_cycles(PWR_BTN_PRESS_DELAY);
            while(!(KEYPAD_PWR_IN & KEYPAD_PWR_BTN));
            __delay_cycles(PWR_BTN_RELEASE_DELAY);

            // enter power OFF state, without resetting any stored display/LED states
            if (currSysState & FLAG_PWR_OFF)
            {
                currSysState &= ~(FLAG_RATE_EDIT | FLAG_VTBI_EDIT | FLAG_PUMP_ACTIVE);

                SEL_ALL_DISPS;
                usciXNSpiPutChar(&USCIA0SPI, 0x00);
                DSEL_ALL_DISPS;

                SEL_LEDSR;
                usciXNSpiPutChar(&USCIA0SPI, LED_PLUGPWR);
                DSEL_LEDSR;
            }
            // enter power ON state, restoring previous display/LED states
            else
            {
                refreshAllDisps(&USCIA0SPI, sevSegDispArr);

                SEL_LEDSR;
                usciXNSpiPutChar(&USCIA0SPI, currLedSRState);
                DSEL_LEDSR;

                enableKeypad();
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
*   If the code is detected as invalid, a 1 is returned from this function
*
*   This function also checks that the display object is in an expected state to
*   ensure nothing has gone wrong; nextBinSegCode and currBinSegCode should always
*   be matching to represent a successful hexCode write to the display, and a steady
*   state of displaying information. These two members could therefore be used to
*   detect when a hexCode conversion has taken place, but the code has not yet been
*   written to the display. Since this is all handled in this function, the two
*   members should always match upon entry to this function; if this is not the case,
*   the inconsistency is detected and the hexCode is not written to the display.
*   A 2 is then returned from this function.
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
*   char errorCode; 0 if the write was successful and display state was normal,'
*   1 if the given hexCode was invalid, 2 if the current and next binary segment
*   codes were out of sync.
*
* Author:       Mason Kury
* Created:      November 10, 2022
* Modified:     December 5, 2022
************************************************************************************/
static unsigned char writeHexToSevSeg(const USCIXNSPI *const usciXN, SEVEN_SEG_DISP *const display, const unsigned char hexCode, const unsigned char csPortIndex)
{
    unsigned char errorCode = 0;

    // check that the display is in an expected state
    if (display->currBinSegCode == display->nextBinSegCode)
    {
        // write the hex code to the display, and convert to binary segment code
        display->hexDigit = hexCode;
        if (!hexToSevSeg(display))
        {
            // activate display's chip select, write the binary segment code over SPI, then deactivate chip select
            DISPS_CSOUT |= (DISP0 << csPortIndex);
            usciXNSpiPutChar(usciXN, (display->nextBinSegCode));
            DISPS_CSOUT &= ~(DISP0 << csPortIndex);

            // what was previously nextBinSegCode is now currBinSegCode
            display->currBinSegCode = display->nextBinSegCode;
        }
        else
            errorCode = 1;
    }
    // if the current and next codes didn't match before updating the hex code, there is something wrong
    else if (display->nextBinSegCode != display->currBinSegCode)
        errorCode = 2;

    return errorCode;
}

/************************************************************************************
* Function: writeToDispRow
*
* Description:
*   Writes given 4-digit hex data to the 4-display top/bottom row of the gemini
*   interface, using calls to the writeHexToSevSeg function defined above.
*   A true/false value representing the decimal ON/OFF state for each display in
*   the row is present rowDataArr[n][1], while the hex data itself is located in
*   rowDataArr[n][0]. This is important, as the decimal state must be set within
*   the display object before calling writeHexToSevSeg -- the actual decoding function
*   hexToSevSeg looks for this value within the object, which is passed by reference.
*
*   PLEASE NOTE: Though 'displayArr' is defined in the same way as 'display' in
*   the writeHexToSevSeg function, it represents an ARRAY OF STRUCTURES, rather
*   than just a pointer referencing a single structure (likely from the array.)
*   This is more a requirement of conceptual understanding, as the function will
*   effectively still be called the same way (by passing the address of the first
*   element in the array, compared to the address of an individual struct object.)
*
*   EXAMPLE: If you wanted to write "102.7" to the top row, you would pass the array
*   {{1, 0}, {0, 0}, {2, 1}, {7, 0}} with parameter 'botRow' == 0
*
* Arguments:
*   *usciXN         -   pointer to the the USCI peripheral object
*   *displayArr     -   pointer to the array of 7seg display objects
*   *rowDataArr     -   pointer to the 2D array of hex codes and decimal states
*   botRow          -   0 to write to top disp row, nonzero to write to bottom disp row
*
* Returns:
*   char errorCode; 0 if the write was successful and display state was normal,'
*   1 if a given hexCode was invalid, 2 if the current and next binary segment
*   codes were out of sync.
*
*   (NOTE: errorCode is based off of the return from writeHexToSevSeg; see function
*   header above for further detail)
*
* Author:       Mason Kury
* Created:      November 30, 2022
* Modified:     November 30, 2022
************************************************************************************/
static unsigned char writeToDispRow(const USCIXNSPI *const usciXN, SEVEN_SEG_DISP *const displayArr, const unsigned char (*rowDataArr)[2], unsigned char botRow)
{
    unsigned char errorCode = 0;
    unsigned char dispRowIndex;

    // ensure botRow can only be 4 or 0
    if (botRow) botRow = 4;

    for (dispRowIndex = 0; dispRowIndex < 4; dispRowIndex ++)
    {
        // the decimal state of each display is on each odd number of dispRowArr
        if (rowDataArr[dispRowIndex][1])
            (displayArr[dispRowIndex + botRow]).dp = 1;
        else
            (displayArr[dispRowIndex + botRow]).dp = 0;

        // write data at the current index in rowDataArr to the corresponding display within the specified top or bottom row
        errorCode = writeHexToSevSeg(usciXN, &displayArr[dispRowIndex + botRow], rowDataArr[dispRowIndex][0], (dispRowIndex + botRow));

        if (errorCode)
            break;
    }

    return errorCode;
}

/************************************************************************************
* Function: incDispRow
*
* Description:
*   **Please note: this description assumes the function is being used for its
*   original application in combination with the Gemini interface keypad. Because
*   of this, the function's logic is described with reference to the actual keys, as
*   in the main code the 100 button would call this function with digitPos == 0.
*   Though this function doesn't actually involve the buttons, the logic is easier
*   to understand with reference to them.**
*
*   Increments the value at a given display row digit position to emulate a real
*   Gemini IV pump unit. Incrementing is done by pressing the 100, 10, 1, or 0.1
*   button on the Gemini keypad when in the edit state of the desired row. Normally,
*   when pressing the 100 button, the first digit is incremented, the second is
*   incremented by the 10 button, the third by the 1 button, and the fourth by the
*   0.1 button. When a digit is incremented past 9, it rolls back to 0 -- this is
*   always displayed as a 0 in the ones place, and in the tens place when the hundreds
*   place contains a value; otherwise, the tens place remains blank until incremented
*   to 1. The tenth place also increments back to an OFF state past 9. When the tenth
*   place contains a value, the decimal point on the third display in the row is enabled;
*   it is disabled when the tenth place rolls back to an OFF state.
*
*   The hundred place has slightly different behaviour: when it rolls past 9, the first
*   display is reset back to 1, and the 100, 10, and 1 buttons then address the next
*   display over, meaning the tenth place becomes the ones place, and so on. Values
*   are retained in the tens and ones place as they are shifted to the right. Upon
*   rolling over 9 again, the thousands place is incremented to 2. This behaviour
*   remains until a value >= 9900 is reached. Upon pressing 100 again, the display
*   shifts back to the left, so the 100 button addresses the first display, the
*   10 button addresses the second display, etc; once again, tens and ones values
*   are retained during the shift. When resetting back to 0, the hundreds place
*   is reset to OFF, as well as the tens place if it was also 0. This behaviour
*   does not occur for the top (RATE) row, as it is not allowed to exceed 999.9;
*   upon incrementing the hundreds place past 900, it simply resets to 0 as if
*   a value of 9900 was incremented as described above.
*
*   When in the thousands state previously described, a press of the 0.1 button
*   can instantly exit this state; ones, tens, and hundreds values are shifted to
*   the left, and the decimal point is enabled. The hundreds place must then be
*   incremented normally to get back up to the thousands.
*
*   Examples:
*       "[][]2 []"  --> 0.1 --> "[][]2. 1" on either row
*       "[][]2. 9"  --> 0.1 --> "[][]2 []" on either row
*       "1 0 0 0"   --> 0.1 --> "[][]0. 1" (only possible on bottom row)
*       "1 0 2 3"   --> 0.1 --> "[]2 3. 1" (only possible on bottom row)
*       "1 2 0 3"   --> 0.1 --> "2 0 3. 1" (only possible on bottom row)
*       "[][]2 []"  --> 10  --> "[]1 2 []" on either row
*       "[]9 2 []"  --> 10  --> "[][]2 []" on either row
*       "2 9 3 []"  --> 10  --> "2 0 3 []" on either row
*       "4 2 3 []"  --> 100 --> "5 2 3 []" on either row
*       "9 0 2. 3"  --> 100 --> "[][]2. 3" on top row
*       "9 0 2. 3   --> 100 --> "1 0 0 2" on bottom row
*       "9 9 2 3"   --> 100 --> "[]2 3 []" (only possible on bottom row)
*
* Arguments:
*   *usciXN         -   pointer to the the USCI peripheral object
*   *displayArr     -   pointer to the array of 7seg display objects
*   digitPos        -   a value from 0 to 3 representing the hundreds to tenths place respectively
*   botRow          -   0 to write to top disp row, nonzero to write to bottom disp row
*
* Returns:
*   char errorCode; 0 if the increment was successful and display state was normal,'
*   1 if a given hexCode was invalid, 2 if the current and next binary segment
*   codes were out of sync, 3 if the given digitPos was invalid.
*
*   (NOTE: errorCode is based off of the return from writeHexToSevSeg; see function
*   header above for further detail)
*
* Author:       Mason Kury
* Created:      December 2, 2022
* Modified:     December 5, 2022
************************************************************************************/
static unsigned char incDispRow(const USCIXNSPI *const usciXN, SEVEN_SEG_DISP *const displayArr, const unsigned char digitPos, const unsigned char botRow)
{
    unsigned char errorCode;
    unsigned char dispRowIndex;     // holds the index of the target digit with displayArr, offset to address the appropriate row based on the value of botRow
    unsigned char nextHexCode;      // stores the target digit's hexDigit + 1, or the resulting code after rolling over from 0x9
    unsigned char thousandState;    // used to change the behaviour of an increment when a display row value is >= 1000

    // ensure digitPos is valid, else throw errorCode 3
    if (digitPos <= TENTHS_PLACE)
    {
        dispRowIndex = (botRow) ? (digitPos + 4) : digitPos;    // offset by an index of 4 when addressing the bottom row

        // a display value >= 1000 can be detected if there is no decimal place present, and there is a digit being displayed in the normal tenth place
        // in the case of a thousand state, shift the display index to the right by 1 place
        if (botRow)
            thousandState = (((displayArr[6]).dp) == 0 && ((displayArr[7]).hexDigit) != OFF_CODE) ? 1 : 0;
        else
            thousandState = (((displayArr[2]).dp) == 0 && ((displayArr[3]).hexDigit) != OFF_CODE) ? 1 : 0;

        dispRowIndex += thousandState;
        nextHexCode = (displayArr[dispRowIndex]).hexDigit + 1;  // store and increment the target digit's hex code

        switch (digitPos)
        {
        case HUNDREDS_PLACE:
            if (nextHexCode > 0x9 && (nextHexCode - 1) != OFF_CODE)
            {
                // rolling back out of hundreds on the top row (from >= 900 to >= 0)
                if (!botRow)
                {
                    // ensure the tens place is turned off as well as the hundreds place if the tens value was 0
                    if ((displayArr[(unsigned int)(dispRowIndex + 1)]).hexDigit == 0x0)
                        (displayArr[(unsigned int)(dispRowIndex + 1)]).hexDigit = OFF_CODE;
                    nextHexCode = OFF_CODE;
                }
                // rolling over into thousands (from >= 900 to >= 1000)
                else if (!thousandState)
                {
                    (displayArr[(unsigned int)(dispRowIndex + 3)]).hexDigit = (displayArr[(unsigned int)(dispRowIndex + 2)]).hexDigit;      // copy the previous ones value to the new ones place
                    (displayArr[(unsigned int)(dispRowIndex + 2)]).hexDigit = (displayArr[(unsigned int)(dispRowIndex + 1)]).hexDigit;      // copy the previous tens value to the new tens place
                    (displayArr[(unsigned int)(dispRowIndex + 2)]).dp = 0;                                                                  // clear any previous decimal point in the new tens place
                    (displayArr[(unsigned int)(dispRowIndex + 1)]).hexDigit = 0x0;                                                          // set the new hundreds place to 0
                    nextHexCode = 0x1;                                                                                                      // set the new thousands place to 1
                }
                // rolling out of thousands state (from >= 9900 to >= 0)
                else if (thousandState && (displayArr[(unsigned int)(dispRowIndex - 1)]).hexDigit == 0x9)
                {
                    if ((displayArr[(unsigned int)(dispRowIndex + 1)]).hexDigit == 0x0)
                        (displayArr[(unsigned int)(dispRowIndex)]).hexDigit = OFF_CODE;                                                     // turn off the tens place if it was 0
                    else
                        (displayArr[(unsigned int)(dispRowIndex)]).hexDigit = (displayArr[(unsigned int)(dispRowIndex + 1)]).hexDigit;      // otherwise copy the previous tens value to the new tens place
                    (displayArr[(unsigned int)(dispRowIndex + 1)]).hexDigit = (displayArr[(unsigned int)(dispRowIndex + 2)]).hexDigit;      // copy the previous ones value to the new ones place
                    (displayArr[(unsigned int)(dispRowIndex + 2)]).hexDigit = OFF_CODE;                                                     // clear the current ones place so it can be the tenths place again
                    dispRowIndex -= 1;                                                                                                      // officially exit the thousand state
                    nextHexCode = OFF_CODE;                                                                                                 // ensure the new hundreds place is cleared
                }
                // rolling over into the next thousand
                else
                {
                    (displayArr[(unsigned int)(dispRowIndex - 1)]).hexDigit++;  // increment the thousands place
                    nextHexCode = 0x0;                                          // reset the hundreds place to 0
                }
            }
            else
            {
                if ((nextHexCode - 1) == OFF_CODE)
                    nextHexCode = 0x1;

                // the tens and ones place must always be at least 0 when the hundreds or thousands place has a value
                if ((displayArr[(unsigned int)(dispRowIndex + 1)]).hexDigit == OFF_CODE)
                    (displayArr[(unsigned int)(dispRowIndex + 1)]).hexDigit = 0x0;
                if ((displayArr[(unsigned int)(dispRowIndex + 2)]).hexDigit == OFF_CODE)
                    (displayArr[(unsigned int)(dispRowIndex + 2)]).hexDigit = 0x0;
            }
            break;

        case TENS_PLACE:
            if ((nextHexCode - 1) == OFF_CODE)                                              // if previously OFF, increment straight to 0x1
                nextHexCode = 0x1;
            else if (nextHexCode > 0x9)
                if ((displayArr[(unsigned int)(dispRowIndex - 1)]).hexDigit == OFF_CODE)    // if rolling over from 9 and the hundreds place is OFF, turn OFF as well
                    nextHexCode = OFF_CODE;
                else                                                                        // if rolling over from 9 and the hundreds place has a value, roll back to 0
                    nextHexCode = 0x0;
            break;

        case ONES_PLACE:
            if (nextHexCode > 0x9)
                nextHexCode = 0x0;
            break;

        case TENTHS_PLACE:
            if (thousandState)
            {
                dispRowIndex -= 1;                                                                                                      // leave the thousands state upon a tenth button press

                if ((displayArr[(unsigned int)(dispRowIndex - 2)]).hexDigit == 0x0)
                    (displayArr[(unsigned int)(dispRowIndex - 3)]).hexDigit = OFF_CODE;                                                 // turn off the hundreds place if it was 0
                else
                    (displayArr[(unsigned int)(dispRowIndex - 3)]).hexDigit = (displayArr[(unsigned int)(dispRowIndex - 2)]).hexDigit;  // otherwise copy the previous hundreds value to the new hundreds place

                if ((displayArr[(unsigned int)(dispRowIndex - 1)]).hexDigit == 0x0)
                    if ((displayArr[(unsigned int)(dispRowIndex - 3)]).hexDigit == OFF_CODE)
                        (displayArr[(unsigned int)(dispRowIndex - 2)]).hexDigit = OFF_CODE;                                             // turn off the tens place if both it and the hundreds place are 0
                    else
                        (displayArr[(unsigned int)(dispRowIndex - 2)]).hexDigit = 0x0;                                                  // otherwise display 0 if the hundreds place has a value
                else
                    (displayArr[(unsigned int)(dispRowIndex - 2)]).hexDigit = (displayArr[(unsigned int)(dispRowIndex - 1)]).hexDigit;  // otherwise copy the previous tens value to the new tens place

                (displayArr[(unsigned int)(dispRowIndex - 1)]).hexDigit = (displayArr[(unsigned int)(dispRowIndex)]).hexDigit;          // copy the previous ones value to the new ones place
            }

            if ((nextHexCode - 1) == OFF_CODE || thousandState)
            {
                nextHexCode = 0x1;
                (displayArr[(unsigned int)(dispRowIndex - 1)]).dp = 1;
            }

            else if (nextHexCode > 0x9)
            {
                nextHexCode = OFF_CODE;
                (displayArr[(unsigned int)(dispRowIndex - 1)]).dp = 0;
            }
            break;
        }

        // write the incremented hex value to the indexed display, and ensure any other modified hexDigits are refreshed
        errorCode = writeHexToSevSeg(usciXN, &displayArr[dispRowIndex], nextHexCode, dispRowIndex);
        refreshAllDisps(usciXN, displayArr);
    }
    else
        errorCode = 3;  // invalid digitPos

    return errorCode;
}

/************************************************************************************
* Function: refreshAllDisps
*
* Description:
*   Simply rewrites the contents of the currBinSegCode member of a SEVEN_SEG_DISP
*   object within the displayArr to the appropriate seven segment display.
*   This is intended to allow you to turn off various displays with a manual SPI
*   bus write, but not clear the previously displayed contents of the interface;
*   upon turning back on, calling this function easily restores the previous contents.
*
*   For each display, the hexDigit member is re-converted into binary segment code,
*   and the currentBinSegCode is ensured to match the nextBinSegCode for consistency.
*   In addition to the functionality described above, this makes it possible to write
*   hex codes to multiple display objects and call this function once to update them.
*
*   Please note that this function does not check the hexDigit member value before
*   attempting a segment code conversion, so it may be harder to debug than
*   writeHexToSevSeg; additionally, because this function updates all displays
*   even if they are already displaying their stored values, it may be less efficient
*   than multiple calls to writeHexToSevSeg.
*
* Arguments:
*   *usciXN         -   pointer to the the USCI peripheral object
*   *displayArr     -   pointer to the array of 7seg display objects
*
* Returns:
*   (none)
*
* Author:       Mason Kury
* Created:      November 30, 2022
* Modified:     December 1, 2022
************************************************************************************/
static void refreshAllDisps(const USCIXNSPI *const usciXN, SEVEN_SEG_DISP *const displayArr)
{
    unsigned char dispIndex;

    for (dispIndex = 0; dispIndex < NUM_DISPS; dispIndex++)
    {
        hexToSevSeg(&displayArr[dispIndex]);
        (displayArr[dispIndex]).currBinSegCode = (displayArr[dispIndex]).nextBinSegCode;
        DISPS_CSOUT |= (DISP0 << dispIndex);
        usciXNSpiPutChar(usciXN, ((displayArr[dispIndex]).currBinSegCode));
        DISPS_CSOUT &= ~(DISP0 << dispIndex);
    }
}

/************************************************************************************
* Function: writeToRowBuff
*
* Description:
*   A simple, very specific function to completely reassign all values of a 4x2 array
*   at once, intended to have the same format as described in the writeToDispRow
*   function header. The main purpose of this function is to keep code organized
*   leading up to a writeToDispRow call, which also exists to make code more readable.
*
* Arguments:
*   *rowBuff        -   pointer to the 2D array
    dat0...dat3     -   hex codes to write to the respective displays
    dp0...dp3       -   0 for no decimal, nonzero for an active decimal point

    PLEASE SEE writeToDispRow HEADER FOR FURTHER PARAMETER/ARRAY ELEMENT CLARIFICATION!
*
* Returns:
*   (none)
*
* Author:       Mason Kury
* Created:      November 30, 2022
* Modified:     December 1, 2022
************************************************************************************/
static void writeToRowBuff(unsigned char (*const rowBuff)[2], const unsigned char dat0, const unsigned char dp0, const unsigned char dat1, const unsigned char dp1,
                    const unsigned char dat2, const unsigned char dp2, const unsigned char dat3, const unsigned char dp3)
{
    rowBuff[0][0] = dat0;
    rowBuff[0][1] = dp0;
    rowBuff[1][0] = dat1;
    rowBuff[1][1] = dp1;
    rowBuff[2][0] = dat2;
    rowBuff[2][1] = dp2;
    rowBuff[3][0] = dat3;
    rowBuff[3][1] = dp3;
}

/************************************************************************************
* Function: flashDispRow
*
* Description:
*   Flashes the top/bottom row(s) of displays a given number of times based on the
*   DISP_FLASH_DELAY interval defined at the top of this client. Flashing is
*   accomplished by clearing the specified row of displays with a manual SPI bus write,
*   then later calling the refreshAllDisps function to restore their previous contents.
*   No additional delay interval is added after restoring the row contents for the last
*   time in the loop.
*
* Arguments:
*   *usciXN         -   pointer to the the USCI peripheral object
*   *displayArr     -   pointer to the array of 7seg display objects
    rowSel          -   0 for top row, 1 for bottom row, else for both rows
    numFlashes      -   number of times to flash the row (must be > 0)
*
* Returns:
*   (none)
*
* Author:       Mason Kury
* Created:      November 31, 2022
* Modified:     December 1, 2022
************************************************************************************/
static void flashDispRow(const USCIXNSPI *const usciXN, SEVEN_SEG_DISP *const displayArr, const unsigned char rowSel, const unsigned char numFlashes)
{
    unsigned char row;
    unsigned char flashIndex;

    if (rowSel == TOP_ROW)
        row = TOP_DISPS;

    else if (rowSel == BOT_ROW)
        row = BOT_DISPS;

    else
        row = ALL_DISPS;

    for (flashIndex = 0; flashIndex < numFlashes; flashIndex++)
    {
        DISPS_CSOUT |= row;
        usciXNSpiPutChar(usciXN, 0x00);
        DISPS_CSOUT &= ~row;
        __delay_cycles(DISP_FLASH_DELAY);
        refreshAllDisps(usciXN, displayArr);

        // no need to add a delay after refreshing the displays for the final time
        if (flashIndex < (numFlashes - 1))
            __delay_cycles(DISP_FLASH_DELAY);
    }
}


/************************************************************************************
* Function: criticalFaultHandler
*
* Description:
*   An emergency panic function for the event of an impossibly unexpected situation.
*   All maskable interrupts are disabled, LEDs are turned off, the (col,row)
*   coordinates of the last known key press are displayed, and a message flashes
*   letting the user know they should power off the device. A while loop polls
*   for the press of the power button while the message flashes. Once the button is
*   pressed, it is deboucned, and an invalid write to the WDTCTL register is done
*   on purpose to initiate a PUC reset. An infinite loop is placed below the WDTCTL
*   write as a final fail-safe to prevent further code from being run if the PUC
*   doesn't occur for some reason.
*
* Arguments:
*   *usciXN         -   pointer to the the USCI peripheral object
*   *displayArr     -   pointer to the array of 7seg display objects
*   *rowBuff        -   pointer to a 4x2 buffer array
*
* Returns:
*   (none; DOES NOT RETURN)
*
* Author:       Mason Kury
* Created:      November 31, 2022
* Modified:     March 27, 2022
************************************************************************************/
static void criticalFaultHandler(const USCIXNSPI *const usciXN, SEVEN_SEG_DISP *const displayArr, unsigned char (*const rowBuff)[2])
{
    // disable interrupts, as the power button is polled
    __disable_interrupt();

    // clear all LEDs
    SEL_LEDSR;
    usciXNSpiPutChar(usciXN, 0x00);
    DSEL_LEDSR;

    // write to displays:
    //  [E.]    [C.]    [(col)]  [(row)]
    //  [(OFF)] [0]     [F]     [F]
    // meaning "error corrdinate: (col,row); press power off
    // 0FF flashes to suggest the power button is required to be pressed
    writeToRowBuff(rowBuff, 0xE, 1, 0xC, 1, ((geminiKeypad.currKeyCoord & 0xF0) >> 4), 0, (geminiKeypad.currKeyCoord & 0x0F), 0);
    writeToDispRow(usciXN, displayArr, rowBuff, TOP_ROW);
    writeToRowBuff(rowBuff, OFF_CODE, 0, 0x0, 0, 0xF, 0, 0xF, 0);
    writeToDispRow(usciXN, displayArr, rowBuff, BOT_ROW);

    // flash "0FF" while waiting for power button press
    while(KEYPAD_PWR_IN & KEYPAD_PWR_BTN)
    {
        __delay_cycles(DISP_FLASH_DELAY);
        SEL_BOT_DISPS;
        usciXNSpiPutChar(usciXN, 0x00);
        DSEL_BOT_DISPS;
        __delay_cycles(DISP_FLASH_DELAY);
        refreshAllDisps(usciXN, displayArr);
    }
    // debounce press, wait for power button release, then debounce the release
    __delay_cycles(PWR_BTN_PRESS_DELAY);
    while(!(KEYPAD_PWR_IN & KEYPAD_PWR_BTN));
    __delay_cycles(PWR_BTN_RELEASE_DELAY);

    WDTCTL = 0xDEAD;    // write to the watchdog register with an invalid password, generating a PUC
    while(1);           // if something goes wrong with the watchdog reset, ensure no other code is executed
}

// disables the keypad interrupts, debounce timer, and stops any pending keypresses
// (this function could possibly be inline, but I've left that up to the compiler to decide)
static void disableKeypad()
{
    TA0CCR0 = 0;                                            // stop debouncing
    prevSysState &= ~FLAG_KEYPAD_PRESS;                     // clear any kind of pending button press
    currSysState &= ~FLAG_KEYPAD_PRESS;
    *(geminiKeypad.ROW_IE) &= ~(geminiKeypad.ROW_PINS);     // shut off keypad interrupts
    *(geminiKeypad.ROW_IES) &= ~(geminiKeypad.ROW_PINS);    // reset the edge select to wait for a press (L -> H)
    *(geminiKeypad.ROW_IFG) &= ~(geminiKeypad.ROW_PINS);    // clear any keypad flags
}

// re-enables keypad interrupts
__inline static void enableKeypad()
{

    *(geminiKeypad.ROW_IFG) &= ~(geminiKeypad.ROW_PINS);        // clear any keypad flags
    *(geminiKeypad.ROW_IE) |= (geminiKeypad.ROW_PINS);          // enable keypad interrupts
}

// initializes all necessary registers for timerA0 interrupt functionality; the timer must actually be started by later loading a TA0CCR0 value
__inline static void initKeypadDelayTimer()
{
    BCSCTL1 &= ~(BIT4 | BIT5 | XTS);            // ensure no ACLK division, and low-frequency mode for LFXT1 to allow for VLOCLK selection
    BCSCTL3 |= LFXT1S_2;                        // set ACLK source to VLOCLK (12kHz)

    TA0CTL = TASSEL_1 | MC_1;                   // set source as ACLK, no clock division, count up mode
    TA0CCTL0 |= CCIE;                           // enable timer interruptss
    TA0CCTL0 &= ~CCIFG;                         // clear timer interrupt flags
    __enable_interrupt();                       // enable global interrupts
}

// initializes all necessary registers for power button interrupt functionality
__inline static void initPwrBtn()
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
        disableKeypad();

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
