/************************************************************************************
* See header file for general module documentation
************************************************************************************/


//########## DEPENDENCIES ##########//
#include <msp430.h>
#include "sevenSeg.h"


//########## FUNCTION DEFINITIONS ##########//

/************************************************************************************
* Function: hexToSevSeg
*
* Description:
*   Converts the hexadecimal value currently stored in a 7seg display's hexDigit
*   member into the appropriate binary; the binary code is placed into the 7seg's
*   nextBinSegCode member in the format <BIT7:BIT0> {dp, G, F, E, D, C, B, A}.
*   The binary code is automatically inverted if the display is indicated as being
*   active-low by its activeLow member.
*
* Arguments:
*   *display    -   pointer to the 7seg display object
*
* Returns:
*   char convFail; 0 if hex to 7seg code was successful, nonzero if the contents of
*   display->hexDigit were not a valid hexadecimal number.
*
* Author:       Mason Kury
* Date:         November 9, 2022
* Modified:     December 2, 2022
************************************************************************************/
unsigned char hexToSevSeg(SEVEN_SEG_DISP *const display)
{
    // a 1-dimensional lookup table for binary segment codes based on a 0x0 to 0xF hex digit input, or a 0x10 OFF_CODE
    static const unsigned char segCodeTable[0x12] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71, 0x00, 0x40};

    unsigned char convFail = 0;

    // begin by storing the decimal place bit in the segment code if the dp member is set
    (display->nextBinSegCode) = (display->dp) ? BIT7 : 0x00;

    // if hexDigit is valid, append the segment code for the hex digit
    if ((display->hexDigit) < 0x12)
        (display->nextBinSegCode) |= segCodeTable[display->hexDigit];
    else
        convFail = 1;

    // automatically invert binary code for an active-low common anode display
    if (display->activeLow)
        display->nextBinSegCode = ~(display->nextBinSegCode);

    return convFail;
}


/************************************************************************************
* Function: sevSegToHex
*
* Description:
*   Attempts to convert the binary code currently displayed on a 7seg display back
*   into a hexadecimal digit (or the OFF_CODE). If successful, the hex digit is
*   stored in the display object's hexDigit member, and the decimal point state is
*   stored in the dp member.
*
* Arguments:
*   *display    -   pointer to the 7seg display object
*
* Returns:
*   char convFail; 0 if 7seg to hex code was successful, nonzero if a valid hex digit
*   or code was not recognized from the display through display->currBinSegCode
*
* Author:       Mason Kury
* Date:         November 9, 2022
* Modified:     December 2, 2022
************************************************************************************/
unsigned char SevSegToHex(SEVEN_SEG_DISP *const display)
{
    unsigned char convFail = 0;
    unsigned char sevSegCode; // code for current 7seg state excluding decimal, and inverted for 1=ON/0=OFF within code

    // store only bits corresponding to segments (no dp) and invert code if display is common anode/active-low
    sevSegCode = (display->currBinSegCode) & 0x7F;
    sevSegCode = (display->activeLow) ? ~sevSegCode : sevSegCode;

    // attempt to find a hex digit (or OFF code) match to the currently output binary code
    switch (sevSegCode)
    {
    case 0x3F: // 0011 1111
        display->hexDigit = 0x0;
        break;
    case 0x06: // 0000 0110
        display->hexDigit = 0x1;
        break;
    case 0x5B: // 0101 1011
        display->hexDigit = 0x2;
        break;
    case 0x4F: // 0100 1111
        display->hexDigit = 0x3;
        break;
    case 0x66: // 0110 0110
        display->hexDigit = 0x4;
        break;
    case 0x6D: // 0110 1101
        display->hexDigit = 0x5;
        break;
    case 0x7D: // 0111 1101
        display->hexDigit = 0x6;
        break;
    case 0x07: // 0000 0111
        display->hexDigit = 0x7;
        break;
    case 0x7F: // 0111 1111
        display->hexDigit = 0x8;
        break;
    case 0x6F: // 0110 1111
        display->hexDigit = 0x9;
        break;
    case 0x77: // 0111 0111
        display->hexDigit = 0XA;
        break;
    case 0x7C: // 0111 1100
        display->hexDigit = 0xB;
        break;
    case 0x39: // 0011 1001
        display->hexDigit = 0xC;
        break;
    case 0x5E: // 0101 1110
        display->hexDigit = 0xD;
        break;
    case 0x79: // 0111 1001
        display->hexDigit = 0xE;
        break;
    case 0x71: // 0111 0001
        display->hexDigit = 0xF;
        break;
    case 0x00: // OFF
        display->hexDigit = OFF_CODE;
        break;
    case 0x40: // DASH
        display->hexDigit = DASH_CODE;
    default:
        convFail = 1;
        break;
    }

    // only store the decimal place state if a hex digit or code was successfully recognized from the display above
    if (!convFail)
        display->dp = ((display->currBinSegCode) & BIT7) ? 1 : 0;

    return convFail;
}
