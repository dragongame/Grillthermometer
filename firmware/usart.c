#include "usart.h"

bool USART_TXBuffer_FreeSpace(USART_data_t * usart_data);
void USART_RXComplete(USART_data_t * usart_data);
void USART_DataRegEmpty(USART_data_t * usart_data);
void USART_InterruptDriver_Initialize(USART_data_t * usart_data, USART_t * usart, USART_DREINTLVL_t dreIntLevel );
void USART_InterruptDriver_DreInterruptLevel_Set(USART_data_t * usart_data, USART_DREINTLVL_t dreIntLevel);

volatile uint8_t g_commandReceived;

USART_data_t USART_data; /* USART data struct */

//##########################################################################################
// Purpose:  initialize UART and set baudrate
// Input:    baudrate using macro UART_BAUD_SELECT()
// Returns:  none
//##########################################################################################
// Purpose:  transmit string to UART
// Input:    string to be transmitted
// Returns:  none          
void usart_puts(USART_data_t * usart_data, const char *str )
{
    while (*str) 
		usart_putc(usart_data, *str++);
}
//########################################################################################## uart_puts_p
// Purpose:  transmit string from program memory to UART
// Input:    program memory string to be transmitted
// Returns:  none
void usart_puts_p(USART_data_t * usart_data, const char *progmem_s )
{
	register char data;
    while ( (data = pgm_read_byte(progmem_s++)) ) 
		usart_putc(usart_data, data);
}
//##########################################################################################
/*! \brief Initializes buffer and selects what USART module to use.
 *
 *  Initializes receive and transmit buffer and selects what USART module to use,
 *  and stores the data register empty interrupt level.
 *
 *  \param usart_data           The USART_data_t struct instance.
 *  \param usart                The USART module.
 *  \param dreIntLevel          Data register empty interrupt level.
 */
void USART_InterruptDriver_Initialize(USART_data_t * usart_data, USART_t * usart, USART_DREINTLVL_t dreIntLevel)
{
	usart_data->usart = usart;
	usart_data->dreIntLevel = dreIntLevel;

	usart_data->buffer.RX_Tail = 0;
	usart_data->buffer.RX_Head = 0;
	usart_data->buffer.TX_Tail = 0;
	usart_data->buffer.TX_Head = 0;
}
//##########################################################################################
/*! \brief Set USART DRE interrupt level.
 *
 *  Set the interrupt level on Data Register interrupt.
 *
 *  \note Changing the DRE interrupt level in the interrupt driver while it is
 *        running will not change the DRE interrupt level in the USART before the
 *        DRE interrupt have been disabled and enabled again.
 *
 *  \param usart_data         The USART_data_t struct instance
 *  \param dreIntLevel        Interrupt level of the DRE interrupt.
 */
void USART_InterruptDriver_DreInterruptLevel_Set(USART_data_t * usart_data, USART_DREINTLVL_t dreIntLevel)
{
	usart_data->dreIntLevel = dreIntLevel;
}
//########################################################################################## USART_TXBuffer_FreeSpace
/*! \brief Test if there is data in the transmitter software buffer.
 *
 *  This function can be used to test if there is free space in the transmitter
 *  software buffer.
 *
 *  \param usart_data The USART_data_t struct instance.
 *
 *  \retval true      There is data in the receive buffer.
 *  \retval false     The receive buffer is empty.
 */
bool USART_TXBuffer_FreeSpace(USART_data_t * usart_data)
{
	/* Make copies to make sure that volatile access is specified. */
	uint8_t tempHead = (usart_data->buffer.TX_Head + 1) & USART_TX_BUFFER_MASK;
	uint8_t tempTail = usart_data->buffer.TX_Tail;

	/* There are data left in the buffer unless Head and Tail are equal. */
	return (tempHead != tempTail);
}
//########################################################################################## usart_putc
/*! \brief Put data (5-8 bit character).
 *
 *  Stores data byte in TX software buffer and enables DRE interrupt if there
 *  is free space in the TX software buffer.
 *
 *  \param usart_data The USART_data_t struct instance.
 *  \param data       The data to send.
 */
void usart_putc(USART_data_t * usart_data, char data)
{
	while (!USART_TXBuffer_FreeSpace(usart_data)) {};

	uint8_t tempCTRLA;
	uint8_t tempTX_Head;
	USART_Buffer_t * TXbufPtr = &usart_data->buffer;
	
	//cli();
  	tempTX_Head = TXbufPtr->TX_Head;
  	TXbufPtr->TX[tempTX_Head]= data;
	/* Advance buffer head. */
	TXbufPtr->TX_Head = (tempTX_Head + 1) & USART_TX_BUFFER_MASK;
	//sei();

	/* Enable DRE interrupt. */
	tempCTRLA = usart_data->usart->CTRLA;
	tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | usart_data->dreIntLevel;
	usart_data->usart->CTRLA = tempCTRLA;
}
//########################################################################################## USART_RXBufferData_Available
/*! \brief Test if there is data in the receive software buffer.
 *
 *  This function can be used to test if there is data in the receive software buffer.
 *
 *  \param usart_data         The USART_data_t struct instance
 *
 *  \retval true      There is data in the receive buffer.
 *  \retval false     The receive buffer is empty.
 */
bool USART_RXBufferData_Available(USART_data_t * usart_data)
{
	/* Make copies to make sure that volatile access is specified. */
	uint8_t tempHead = usart_data->buffer.RX_Head;
	uint8_t tempTail = usart_data->buffer.RX_Tail;

	/* There are data left in the buffer unless Head and Tail are equal. */
	return (tempHead != tempTail);
}
//########################################################################################## usart_getc
/*! \brief Get received data (5-8 bit character).
 *
 *  The function returns the next character from RX software buffer or EOF,
 *  if buffer is empty.
 *
 *  \param usart_data       The USART_data_t struct instance.
 *
 *  \return         Received data.
 */
uint8_t usart_getc(USART_data_t * usart_data)
{
    uint8_t c = EOF;
    if (USART_RXBufferData_Available(usart_data)) {
        
        USART_Buffer_t * bufPtr;
        
        bufPtr = &usart_data->buffer;
        c = (bufPtr->RX[bufPtr->RX_Tail]);
        
        /* Advance buffer tail. */
        bufPtr->RX_Tail = (bufPtr->RX_Tail + 1) & USART_RX_BUFFER_MASK;
        
    }
    return c;
}

//########################################################################################## USART_RXComplete
/*! \brief RX Complete Interrupt Service Routine.
 *
 *  RX Complete Interrupt Service Routine.
 *  Stores received data in RX software buffer.
 *
 *  \param usart_data      The USART_data_t struct instance.
 */
void USART_RXComplete(USART_data_t * usart_data) {
    USART_Buffer_t * bufPtr;
    uint8_t data = usart_data->usart->DATA;
    
    // always store most recently received character
    usart_data->buffer.RX[usart_data->buffer.RX_Head] = data;
    
    bufPtr = &usart_data->buffer;
    /* Advance buffer head. */
    uint8_t tempRX_Head = (bufPtr->RX_Head + 1) & USART_RX_BUFFER_MASK;
    
    /* Check for overflow. */
    uint8_t tempRX_Tail = bufPtr->RX_Tail;
    
    if (tempRX_Head != tempRX_Tail) {
        usart_data->buffer.RX_Head = tempRX_Head;
    }
    if (data == 0x0d || data == 0x0a) {
        g_commandReceived += 1;
    }
}
//########################################################################################## USART_DataRegEmpty
/*! \brief Data Register Empty Interrupt Service Routine.
 *
 *  Data Register Empty Interrupt Service Routine.
 *  Transmits one byte from TX software buffer. Disables DRE interrupt if buffer
 *  is empty. Argument is pointer to USART (USART_data_t).
 *
 *  \param usart_data      The USART_data_t struct instance.
 */
void USART_DataRegEmpty(USART_data_t * usart_data)
{
	USART_Buffer_t * bufPtr;
	bufPtr = &usart_data->buffer;

	/* Check if all data is transmitted. */
	uint8_t tempTX_Tail = usart_data->buffer.TX_Tail;
	if (bufPtr->TX_Head == tempTX_Tail){
	    /* Disable DRE interrupts. */
		uint8_t tempCTRLA = usart_data->usart->CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		usart_data->usart->CTRLA = tempCTRLA;

	}else{
		/* Start transmitting. */
		uint8_t data = bufPtr->TX[usart_data->buffer.TX_Tail];
		usart_data->usart->DATA = data;

		/* Advance buffer tail. */
		bufPtr->TX_Tail = (bufPtr->TX_Tail + 1) & USART_TX_BUFFER_MASK;
	}
}

/* print binary value to USART */
void usart_putBin(USART_data_t *usart, int val)
{
  uint8_t cnt;
  unsigned int bm = 0x8000;
  for (cnt=0; cnt<16; cnt++) {
    if (bm & val) {
      usart_putc(usart, '1');
    } else {
      usart_putc(usart, '0');
    }
    bm >>= 1;
  }
}

void usart_putByteHex(USART_data_t *usart, uint8_t val) {
    const unsigned char chrs[] = "0123456789abcdef";
    usart_putc(usart, chrs[val>>4]);
    usart_putc(usart, chrs[val & 0x0f]);
}

/* print decimal value to USART */
void usart_putDec(USART_data_t *usart, long val) {
  unsigned char buf[8];
  unsigned char *ptr = buf;

  if (val < 0) {
    usart_putc(usart, '-');
    val = -val;
  }
  if (val == 0) {
    usart_putc(usart, '0');
    return;
  }
  while (val!=0) {
    uint8_t digit = val%10;
    *ptr++ = '0'+digit;
    val /= 10;
  }
  ptr--;
  while (ptr >= buf) {
    usart_putc(usart, *ptr--);
  }
}


//####################################################### init_usart
void usart_init( void )
{
  PORTC.OUTSET = 0x08;
  PORTC.DIRSET = 0x08; /* PC3 (TXD0) as output. */
  PORTC.DIRCLR = 0x04; /* PC2 (RXD0) as input. */
 
  g_commandReceived = 0;

 /* Use USARTC0 and initialize buffers. */
 USART_InterruptDriver_Initialize(&USART_data,&USART,USART_DREINTLVL_LO_gc);
 /* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
 USART_Format_Set(USART_data.usart,USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc,false); 
 /* Enable RXC interrupt. */
 USART_RxdInterruptLevel_Set(USART_data.usart, USART_RXCINTLVL_LO_gc); 
 
 /* calculate Baudrate = (1/(16*(((I/O clock frequency)/Baudrate)-1) = 12 */
 /* BSEL[11:0]-Wert bei 32MHz Takt und BSCALE[3:0]==0: */
 /* 207 : 9600 */
 /* 103 : 19200 */
 /* 51 : 38400 */
 /* 34 : 57600 */
 /* 16 : 115200 */ 
 //#ifdef USE32MHZ
 // USART_Baudrate_Set(&USART, 207, false);  //#else
 USART_Baudrate_Set(&USART, 75, 0x0a);

 USART_Rx_Enable(USART_data.usart);
 USART_Tx_Enable(USART_data.usart);
 
 /* Enable PMIC interrupt level low. */
 PMIC.CTRL |= PMIC_LOLVLEX_bm;
}

bool USART_TXBufferEmpty(USART_data_t *usart)
{
  return (usart->buffer.TX_Head == usart->buffer.TX_Tail);
}

//####################################################### IRQ RXD Port C
ISR(USARTC0_RXC_vect) // Interrupt Daten empfangen
{
    USART_RXComplete(&USART_data);
}

//####################################################### IRQ TXD Port C
ISR(USARTC0_DRE_vect) // Interrupt Daten senden
{
    USART_DataRegEmpty(&USART_data);
}
