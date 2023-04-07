/************************************************************************************
* Matrix Keypad Module
*
* A module for interrupt-based scanning and decoding of matrix keypad coordinates.
* This module is built around the use of MATRIX_KEYPAD typedef structs, which store
* all necessary register addresses, pin masks, and the most recently decoded keypad
* coordinate. A single 8-bit register is used for row pins, and another is used for
* column pins, allowing for interaction with a keypad up to 8x8 in size. Since this
* module is based on keypad struct objects, any number of keypads can be defined in
* your application.
*
* The register addresses required in a keypad object include PxIN, PxOUT, PxDIR, PxSEL,
* PxREN, PxIE, PxIES, and PxIFG for the single I/O register used for up to 8 row pins,
* as well as the PxOUT, PxDIR, and PxSEL addresses of the single I/O register used for
* up to 8 column pins. The row pins are configured as INPUTS pulled LOW, and the column
* pins are configured as OUTPUTS initially driven HIGH, allowing for interrupts as soon
* as a mtrxKeypadInit() call is returned. Additionally, 8-bit masks for the row and
* column pin registers corresponding to the connected pins are required; masked off pins
* are not initialized, and can therefore be used elsewhere in your application.
*
* For use in your project, create a keypad object with the appropriate register addresses,
* set up an ISR for the row I/O register, and set up an ISR for a timer CCR0 vector.
* The keypad ISR should simply shut off keypad row interrupts and load the timer's CCR0
* value with the PRESS_DBNC_DELAY or RELEASE_DBNC_DELAY depending on whether the interrupt
* was a press or release. The timer ISR should then signal a need to call either
* scanForKeyPress() or saveKeyPress(), also depending on the interrupt direction,
* and reset it's own CCR0 back to 0. If scanForKeyPress() was successful, the interrupt
* edge selection will automatically change to H->L so the next interrupt will be for the
* release; otherwise, the edge will not change and it will still wait for a press.
* At the end of saveKeyPress(), edge selection is always set to L->H in order to wait
* for a keypress, as it is assumed the most recent one was successfully scanned and stored.
* This should eliminate any chance for the edge select to be left in the wrong state, as
* long as saveKeyPress() is called following a successful scan, or a scan was simply invalid.
*
* After a saveKeyPress() call, the coordinate should be readable from the currKeyCoord
* member of the keypad object; please see the comment above that member in the definition
* below for details about the coordinate format.
*
* Author:       Mason Kury
* Created:      November 12, 2022
* Modified:     April 3, 2023
************************************************************************************/

#ifndef MATRIXKEYPAD_MODULE_MTRXKEYPAD_H_
#define MATRIXKEYPAD_MODULE_MTRXKEYPAD_H_


//########## SYMBOLIC CONSTANTS ##########//
#define PRESS_DBNC_DELAY    300     // number of VLOCLK->ACLK (12kHz) cycles to delay when debouncing a key press
#define RELEASE_DBNC_DELAY  700     // number of VLOCLK->ACLK (12kHz) cycles to delay when debouncing a key release


//########## STRUCTURES ##########//

// NOTE: if RAM is scarce, make a const struct for all register pointers and masks, and keep the
// coordinate, count, buffer, and decoded key members in a separate variable struct.
// Functions in this module would need to be altered to accommodate this change.
typedef struct MATRIX_KEYPAD
{
    // register pointers for setting up keypad pin directions, pullups, and interrupts
    volatile unsigned char *const ROW_IN;
    volatile unsigned char *const ROW_OUT;
    volatile unsigned char *const ROW_DIR;
    volatile unsigned char *const ROW_SEL;
    volatile unsigned char *const ROW_REN;
    volatile unsigned char *const ROW_IE;
    volatile unsigned char *const ROW_IES;
    volatile unsigned char *const ROW_IFG;
    volatile unsigned char *const COL_OUT;
    volatile unsigned char *const COL_DIR;
    volatile unsigned char *const COL_SEL;

    // 8-bit masks respective to ROW and COL IO registers used to select the correct keypad pins
    const unsigned char ROW_PINS;
    const unsigned char COL_PINS;

    /* The (column, row) coordinate of the pressed key; the coordinates are stored in an 8-bit format
     * where the high nibble represents the column, and the low nibble represents the row. Each nibble
     * reflects the state of that row/column's port directly, so the client file should handle any decoding.
     * (Example: column 1 on P4.2 is set HIGH, pulling row 3 on P1.6 HIGH as well. This means the coordinates
     * are (C1, R3) from a user standpoint, but are represented by 0x26 -- effectively (2, 6) -- based
     * solely on the pins within the row and column registers. This allows for greater wiring flexibility.) */
    unsigned char currKeyCoord;
}
MATRIX_KEYPAD;


//########## FUNCTION PROTOTYPES ##########//

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
void mtrxKeypadInit(MATRIX_KEYPAD *const keypad);

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
unsigned char scanForKeyPress(MATRIX_KEYPAD *const keypad);

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
void saveKeyPress(MATRIX_KEYPAD *const keypad);


#endif /* MATRIXKEYPAD_MODULE_MTRXKEYPAD_H_ */
