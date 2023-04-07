/************************************************************************************
* See header file for general module documentation
************************************************************************************/


//########## DEPENDENCIES ##########//
#include <msp430.h>
#include "mtrxKeypad.h"


//########## PRIVATE GLOBALS ##########//
static unsigned char pendingKeyCoord = 0x00;    // stores the latest keypad coordinate; copied to keypad object in saveKeyPress() after key release

//########## FUNCTION DEFINITIONS ##########//

/************************************************************************************
* Function: mtrxKeypadInit
*
* Description:
*   Initializes a given matrix keypad object with initial values and sets the
*   appropriate register values at the addresses within the keypad's members.
*   Row pins are set as INPUTS with PULLDOWN resistors, and column pins are set as
*   OUTPUTS initialliy set HIGH for interrupt capability. All row pins are configured
*   for interrupts on a RISING EDGE transition. GIE is also set in the SR.
*
* Arguments:
*   *keypad     -   pointer to the keypad object
*
* Returns:
*   (none)
*
* Author:       Mason Kury
* Created:      November 12, 2022
* Modified:     December 4, 2022
************************************************************************************/
void mtrxKeypadInit(MATRIX_KEYPAD *const keypad)
{
    // give initial values to keypad members
    keypad->currKeyCoord = 0x00;

    // set row pins as inputs pulled LOW
    *(keypad->ROW_SEL) &= ~(keypad->ROW_PINS);
    *(keypad->ROW_DIR) &= ~(keypad->ROW_PINS);
    *(keypad->ROW_REN) |= (keypad->ROW_PINS);
    *(keypad->ROW_OUT) &= ~(keypad->ROW_PINS);

    // set column pins as outputs, initially HIGH so all keypad interrupts can work
    *(keypad->COL_SEL) &= ~(keypad->COL_PINS);
    *(keypad->COL_DIR) |= (keypad->COL_PINS);
    *(keypad->COL_OUT) |= (keypad->COL_PINS);

    // set up interrupts on row pins
    *(keypad->ROW_IES) &= ~(keypad->ROW_PINS);  // set edge select to L->H on row pins (start by detecting button presses, as a press must happen first to scan the keypad)
    *(keypad->ROW_IFG) &= ~(keypad->ROW_PINS);  // clear any flags on row pins
    *(keypad->ROW_IE) |= (keypad->ROW_PINS);    // enable interrupts on row pins
    __enable_interrupt();                       // ensure global interrupts are enabled
}

/************************************************************************************
* Function: scanForKeyPress
*
* Description:
*   Shifts through the column pins of a given matrix keypad object one by one in an
*   attempt to find the active row pin. As soon as an active row pin is found due to
*   an active column pin, an 8-bit hex coordinate is stored in the private global
*   variable pendingKeyCoord. This coordinate can be saved to the keypad object
*   (usually upon key release) by calling saveKeyPress.
*
*   At the end of the function, all interrupts are re-enabled, and the column pins
*   are set HIGH regardless of whether coordinates were found or not; a typical
*   application would leave interrupts disabled after an interrupt was detected,
*   relying on this function to re-enable them upon completion. If coordinates were
*   found, all row pin bits are set within their interrupt edge select register
*   in addition to interrupts being re-enabled; this results in the next interrupt
*   occurring for the release of the pressed key, allowing for easy debouncing.
*
*   The 8-bit coordinate is formed based on the (column,row) pins that are active
*   within their given register. Example: column 1 on P4.2 is set HIGH, pulling
*   row 3 on P1.6 HIGH as well. This means the coordinates are (C1, R3) from a user
*   standpoint, but are represented by 0x26 -- effectively (2, 6) -- based solely
*   on the pins within the row and column registers.
*
* Arguments:
*   *keypad     -   pointer to the keypad object
*
* Returns:
*   unsigned char scanError; 0 if a keypress was successfully scanned, nonzero if
*   a pressed key was not found.
*
* Author:       Mason Kury
* Created:      November 12, 2022
* Modified:     April 3, 2023
************************************************************************************/
unsigned char scanForKeyPress(MATRIX_KEYPAD *const keypad)
{
    // ensure the keypad can't trigger an interrupt during scanning
    *(keypad->ROW_IE) &= ~(keypad->ROW_PINS);

    unsigned char scanError = 1;

    // used to shift through the row and column registers to scan for a press
    unsigned char colPinIndex;
    unsigned char rowPinIndex;

    // shift through each column pin on its respective register in search of a matching row pin
    for (colPinIndex = 0; colPinIndex < 8; colPinIndex++)
    {
        // if the current pin in the column register isn't a column pin, skip it
        if (!((BIT0 << colPinIndex) & (keypad->COL_PINS)))
            continue;

        // clear the columns, then set the current column pin HIGH
        *(keypad->COL_OUT) &= ~(keypad->COL_PINS);
        *(keypad->COL_OUT) |= (BIT0 << colPinIndex);

        // if any of the row pins are high, shift through each one until a matching HIGH signal is found
        if (*(keypad->ROW_IN) & (keypad->ROW_PINS))
        {
            for (rowPinIndex = 0; rowPinIndex < 8; rowPinIndex++)
            {
                if ((BIT0 << rowPinIndex) & (keypad->ROW_PINS) & *(keypad->ROW_IN))
                {
                    pendingKeyCoord = (colPinIndex << 4) | (rowPinIndex);   // create the 8-bit key coordinate, but don't write it to the keypad object within this function
                    *(keypad->ROW_IES) |= (keypad->ROW_PINS);               // set row pin edge select to H->L, as a release is required before saving keypress data
                    scanError = 0;                                          // there is no scan error, since a key coordinate was found
                    goto SCAN_FINISH;
                }
            }
        }
    }

SCAN_FINISH:

    *(keypad->ROW_IFG) &= ~(keypad->ROW_PINS);  // clear any keypad interrupts that occurred during scanning
    *(keypad->COL_OUT) |= (keypad->COL_PINS);   // make sure column pins are set back to HIGH to allow for future interrupts
    *(keypad->ROW_IE) |= (keypad->ROW_PINS);    // start listening to keypad interrupts again

    return scanError;
}

/************************************************************************************
* Function: saveKeyPress
*
* Description:
*   Copies private global pendingKeyCoord (which should contain the most recently
*   scanned key press coordinate) to the currKeyCoord member of the given
*   keypad object.
*
*   Keypad interrupts are disabled upon function entry, and re-enabled upon exit;
*   the GIE bit in the SR is not affected.
*
*   At the end of the function, before interrupts are re-enabled, row pin bits
*   are cleared in the interrupt edge select register. This ensures the next
*   interrupt will occur on a key press now that the current press-release cycle
*   has been handled.
*
* Arguments:
*   *keypad     -   pointer to the keypad object
*
* Returns:
*   (none)
*
* Author:       Mason Kury
* Created:      November 12, 2022
* Modified:     April 3, 2023
************************************************************************************/
void saveKeyPress(MATRIX_KEYPAD *const keypad)
{
    // ensure the keypad can't trigger an interrupt during saving
    *(keypad->ROW_IE) &= ~(keypad->ROW_PINS);

    // actually store the current key coordinate
    (keypad->currKeyCoord) = pendingKeyCoord;

    *(keypad->ROW_IES) &= ~(keypad->ROW_PINS);  // set edge select back to L->H, as we are now waiting for a press again
    *(keypad->ROW_IFG) &= ~(keypad->ROW_PINS);  // clear any keypad interrupts that occurred during saving
    *(keypad->ROW_IE) |= (keypad->ROW_PINS);    // start listening to keypad interrupts again
}
