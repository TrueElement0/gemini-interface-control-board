/************************************************************************************
* SPI Module
*
* Universal SPI module for use with any MSP430 device, with functionality for any
* valid USCI peripheral. This module is built around creating a constant usciXN
* object, based on the typedef struct below, with all register and pin addresses
* required for the specified USCI SPI peripheral to operate. This allows for
* this one module to be used, rather than defining a separate module for each USCI
* peripheral used on the device (USCI_A0, USCI_B0, etc.)
*
* Please note: this module assumes each USCI peripheral present on your device has
* the same register layout in terms of bit organization. Please ensure this is
* true for all USCI_Ax, USCI_Bx, etc. peripherals on your device.
*
* !!! THIS MODULE DOES NOT CONTAIN FUNCTIONALITY FOR MANAGING SLAVE SELECT PINS !!!
* Slave select functionality should be managed externally by the client, as this
* makes it possible to perform a wider variety of slave management in master mode.
* For example, you may want to talk to multiple slaves at one time with loopback on.
* This could save a lot of SPI bus bandwidth under certain circumstances, and would
* not be possible if slave select was managed by this module.
*
* THIS MODULE CURRENTLY ONLY SUPPORTS 3-WIRE MASTER MODE
*
* Author:       Mason Kury
* Created:      November 10, 2022
* Modified:     November 18, 2022
************************************************************************************/

#ifndef SPI_MODULE_SPI_H_
#define SPI_MODULE_SPI_H_


//########## SYMBOLIC CONSTANTS ##########//

// helpful constants for SPI processing
#define UCMST_SHIFT         3       // number of bits to shift master/slave config value to be in UCXNCTL0<3>
#define UC7BIT_SHIFT        4       // number of bits to shift 7/8 bit data mode dat7Bit<0> to be in UCXNCTL0<4>
#define UCMSB_SHIFT         5       // number of bits to shift msb<0> to be in UCXNCTL0<5>
#define UCCKP_SHIFT         6       // number of bits to shift sclkMode<1:0> (UCCKPH and UCCKPL) to be in UCXNCTL0<7:6>
#define UCLISTEN_SHIFT      7       // number of bits to shift loopBack enable to be in UCXNSTAT<7>

// SPI device-specific configuration values for passing to init function (should apply to all usci peripherals on device)
#define SPI_MST             1
#define SPI_SLV             0
#define SPI_CKPH            BIT1
#define SPI_CKPL            BIT0
#define SPI_MSB             1
#define SPI_LSB             0
#define SPI_DAT7BIT         1
#define SPI_DAT8BIT         0
#define SPI_LOOPBACK        1

// user-defined constants
#define WAIT_FOR_PUTCHAR    1       // set to 1 if usciXNSpiPutChar() should wait for transmission to finish before returning; 0 to not wait
#define SPI_BUF_SZ          100     // the maximum size of an SPI buffer to transmit
#define SECONDARY_UCXNSEL   1       // set to 1 if your device has a secondary function select register for peripheral devices


//########## PREPROCESSOR MACROS ##########//

// waits for TXIFG to determine when TXBUF is available to load and transmit
#define WAIT_FOR_TX     while (!(*(usciXN->UCXNIFG) & (usciXN->UCXNTXIFG)))

// waits for RXIFG to determine when transmission is complete; used within functions where a UCXNSPI object is passed as *ucsiXN
#define WAIT_FOR_RX     while (!(*(usciXN->UCXNIFG) & usciXN->UCXNRXIFG)); *(usciXN->UCXNIFG) &= ~(usciXN->UCXNRXIFG)


//########## STRUCTURES ##########//
typedef struct USCIXNSPI
{
    // port/pin configuration
    volatile unsigned char *const UCXNSEL;  // address of SPI peripheral's port select register (ex: &P1SEL for PORT1)
    volatile unsigned char *const UCXNSEL2; // (optional/device-dependant) address of SPI peripheral secondary port function select register (ex: &P1SEL2 for PORT1)
    const unsigned char UCXNSTE;            // bit representing pin for STE functionality if applicable (ex: BIT0 for P1.0)
    const unsigned char UCXNSIMO;           // bit representing pin for SIMO functionality within peripheral (ex: BIT1 for P1.1)
    const unsigned char UCXNSOMI;           // bit representing pin for SOMI functionality within peripheral (ex: BIT2 for P1.2)
    const unsigned char UCXNCLK;            // bit representing pin for CLK functionality within peripheral (ex: BIT3 for P1.3)

    // register configuration (ex: UCXNCTL0 = UCA0CTL for using UCSI_A0 in SPI mode)
    volatile unsigned char *const UCXNCTL0;
    volatile unsigned char *const UCXNCTL1;
    volatile unsigned char *const UCXNBR0;
    volatile unsigned char *const UCXNBR1;
    volatile unsigned char *const UCXNSTAT;
    volatile unsigned char *const UCXNTXBUF;
    volatile unsigned char *const UCXNRXBUF;
    volatile unsigned char *const UCXNIFG;
    const unsigned char UCXNTXIFG;
    const unsigned char UCXNRXIFG;
}
USCIXNSPI;


//########## FUNCTION PROTOTYPES ##########//

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
void usciXNSpiInit(const USCIXNSPI *const usciXN, const unsigned char spiMST, const unsigned int sclkDiv, const unsigned char sclkPol, const unsigned char dat7Bit, const unsigned char msb, const unsigned char spiLoopBack);

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
void usciXNSpiClkDiv(const USCIXNSPI *const usciXN, const unsigned int sclkDiv);

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
void usciXNSpiPutChar(const USCIXNSPI *const usciXN, const unsigned char txByte);

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
unsigned char usciXNSpiTxBuffer(const USCIXNSPI *const usciXN, const unsigned char *const buffer, const int buffLen);


#endif /* SPI_MODULE_SPI_H_ */
