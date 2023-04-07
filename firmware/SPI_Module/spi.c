/************************************************************************************
* See header file for general module documentation
************************************************************************************/


//########## DEPENDENCIES ##########//
#include <msp430.h>
#include "spi.h"


//########## FUNCTION DEFINITIONS ##########//

/************************************************************************************
* Function: usciXNSpiInit
*
* Description:
*   Initializes the SPI peripheral specified by addresses stored in the usciXN object.
*   The peripheral is set up based on user arguments for master/slave mode, clock
*   division, clock polarity, 7/8-bit mode, MSB/LSB shifted first, and loopback enable.
*
*   Please see usciXNSpiClkDiv function header for preferred sclkDiv values.
*
*   NOTE: This function does not currently support configuration for 4-wire SPI mode.
*
* Arguments:
*   *usciXN     -   pointer to the USCI peripheral object
*   spiMST      -   0 to configure USCIXN as slave, else configure as master
*   sclkDiv     -   16 bit clock divisor to divide SMCLK
*   sclkPol     -   2-bit code for UCCKPH and UCCKPL; sclkMode<1> = UCCKPH, sclkMode<0> = UCCKPL
*   dat7Bit     -   1 to set 7-bit data mode, 0 for 8-bit data
*   spiMSB      -   1 to shift in order of MSB first, 0 to shift LSB first
*   spiLoopback -   1 to set UCLISTEN, 0 to clear; setting UCLISTEN connects SIMO -> SOMI
*
* Returns:
*   (none)
*
* Author:       Mason Kury
* Created:      January 26, 2022
* Modified:     November 13, 2022
************************************************************************************/
void usciXNSpiInit(const USCIXNSPI *const usciXN, const unsigned char spiMST, const unsigned int sclkDiv, const unsigned char sclkPol, const unsigned char dat7Bit, const unsigned char spiMSB, const unsigned char spiLoopBack)
{
    // set up clock division
    usciXNSpiClkDiv(usciXN, sclkDiv);

    *(usciXN->UCXNCTL1) |= UCSWRST; // put usciXN state machine in reset while initializing

	// configure the control registers using the input arguments
	*(usciXN->UCXNCTL0) |= (sclkPol << UCCKP_SHIFT) | (spiMSB << UCMSB_SHIFT) | (dat7Bit << UC7BIT_SHIFT) | (spiMST << UCMST_SHIFT) | UCSYNC;
	*(usciXN->UCXNCTL1) |= UCSSEL_2;
	*(usciXN->UCXNSTAT) |= (spiLoopBack << UCLISTEN_SHIFT);
	
	// configure the SPI pins with PxSEL register (3-wire SPI mode)
	*(usciXN->UCXNSEL) |= (usciXN->UCXNCLK) | (usciXN->UCXNSIMO);
#if SECONDARY_UCXNSEL
    *(usciXN->UCXNSEL2) |= (usciXN->UCXNCLK) | (usciXN->UCXNSIMO);
#endif

    // SOMI is only required when loopback is disabled; otherwise, the user may want to use that pin for something else
	if (!spiLoopBack)
	{
	    *(usciXN->UCXNSEL) |= (usciXN->UCXNSOMI);
#if SECONDARY_UCXNSEL
	    *(usciXN->UCXNSEL2) |= (usciXN->UCXNSOMI);
#endif
	}

	*(usciXN->UCXNCTL1) &= ~UCSWRST; // take usciXN state machine out of reset
}

/************************************************************************************
* Function: usciXNSpiClkDiv
*
* Description:
*   Sets up UCBRx dividor/prescaler value by setting low and high byte registers.
*   The selected clock source can then be divided by UCBRx to get the SPI clock:
*   fBitClock = fBRCLK/UCBRx; even values for sclkDiv allow for even divisions,
*   creating a 50% duty cycle clock, and are therefore preferred.
*
* Arguments:
*   *usciXN     -   pointer to the USCI peripheral object
*   sclkDiv     -   16 bit clock divisor to divide SMCLK
*
* Returns:
*   (none)
*
* Author:       Mason Kury
* Created:      January 26, 2022
* Modified:     November 13, 2022
************************************************************************************/
void usciXNSpiClkDiv(const USCIXNSPI *const usciXN, const unsigned int sclkDiv)
{
    *(usciXN->UCXNCTL1) |= UCSWRST;         // put usciXN state machine in reset while initializing

    *(usciXN->UCXNBR0) = (sclkDiv & 0xFF);  // configure low byte of UCBRx
    *(usciXN->UCXNBR1) = (sclkDiv >> 8);    // configure high byte of UCBRx

    *(usciXN->UCXNCTL1) &= ~UCSWRST;        // take usciXN state machine out of reset
}

/************************************************************************************
* Function: usciXNSpiPutChar
*
* Description:
*   Waits for TXBUF to become available before loading it with the given byte
*   and having it shifted out over SPI. If WAIT_FOR_PUTCHAR is defined as 1 in
*   the header, this function will always wait for transmission to finish before
*   returning. Usually this is only required when transmitting multiple chars
*   in a row, but it may help ensure chip select management doesn't cut off data
*   early in a transmission if the clock is slow. When calling usciXNSpiTxBuffer,
*   the wait for RX_BUF is handled regardless of WAIT_FOR_PUTCHAR.
*
* Arguments:
*   usciXN      -   pointer to the USCI peripheral object
*   txByte      -   the byte to be shifted out
*
* Returns:
*   (none)
*
* Author:       Mason Kury
* Created:      January 26, 2022
* Modified:     November 13, 2022
************************************************************************************/
void usciXNSpiPutChar(const USCIXNSPI *const usciXN, const unsigned char txByte)
{
	WAIT_FOR_TX;
	*(usciXN->UCXNTXBUF) = txByte;
#if (WAIT_FOR_PUTCHAR)
    WAIT_FOR_RX;
#endif
}

/************************************************************************************
* Function: usciXNSpiTxBuffer
*
* Description:
*   Sends an array of bytes over SPI using usciXNSpiPutChar function, given
*   a pointer to the buffer and the number of bytes to send from the buffer.
*
* Arguments:
*   *usciXN     -   pointer to the USCI peripheral object
*   *buffer     -   pointer to the first element of the buffer
*   buffLen     -   number of bytes to send from the buffer (usually length of buffer)
*
* Returns:
*   char txFail; 0 if transmission was successful, nonzero if buffLen was greater than
*   the length of the SPI buffer defined in the header.
*
* Author:       Mason Kury
* Created:      January 26, 2022
* Modified:     November 13, 2022
************************************************************************************/
unsigned char usciXNSpiTxBuffer(const USCIXNSPI *const usciXN, const unsigned char *const buffer, const int buffLen)
{
    unsigned char txFail = 0;
    unsigned int currentChar;   // counter for txloop

    // ensure buffLen is valid based on header file definition
    if (buffLen <= SPI_BUF_SZ)
    {
        // transmit each integer in the buffer
        for (currentChar = 0; currentChar < buffLen; currentChar++)
        {
            usciXNSpiPutChar(usciXN, buffer[currentChar]);
#if (!WAIT_FOR_PUTCHAR)
            WAIT_FOR_RX;
#endif
        }
    }
    else
        txFail = 1;

    return txFail;
}
