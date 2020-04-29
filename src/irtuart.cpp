/*
 * irtuart.cpp
 *
 * The low level UART code for ESP8266 to read and write to the EMS bus via uart
 * Paul Derbyshire - https://github.com/proddy/EMS-ESP
 */

#include "irtuart.h"
#include "irt.h"
#include <user_interface.h>
//#include "ems.h"

#include "MyESP.h"
// MyESP class for logging to telnet and serial
#define myDebug(...) myESP.myDebug(__VA_ARGS__)
#define myDebug_P(...) myESP.myDebug_P(__VA_ARGS__)

extern _EMS_Sys_Status  EMS_Sys_Status;
extern _IRT_Sys_Status IRT_Sys_Status; // iRT Status

_IRTRxBuf * pIRTRxBuf;
_IRTRxBuf * paIRTRxBuf[IRT_MAXBUFFERS];
uint8_t     irtRxBufIdx  = 0;
_IRTTxBuf * pIRTTxBuf;
uint8_t		_disable_rxtx = 0;

os_event_t irtRecvTaskQueue[IRTUART_recvTaskQueueLen]; // our Rx queue

extern _EMSESP_Settings EMSESP_Settings;
//
// Main interrupt handler
// Important: do not use ICACHE_FLASH_ATTR !
//
/*
uint8_t add_debug_to_buf(uint8_t *buffer, uint8_t in_length, const char *text)
{
	return in_length;
	uint8_t i = in_length;
	size_t j, len;
	len = strlen(text);

	if (i<IRT_MAXBUFFERSIZE) buffer[i++] = 0x55;
	for (j=0; j<len; j++) {
		if (i<IRT_MAXBUFFERSIZE) buffer[i++] = text[j];
	}
	if (i<IRT_MAXBUFFERSIZE) buffer[i++] = 0xAA;

	return i;
}
*/
static uint8_t irtuart_tx_received_byte(_IRTTxBuf *pTx, uint8_t rx, uint8_t *buffer, uint8_t in_length)
{
	/* for each incoming byte, test if we need to send anything */

	uint8_t out_length = in_length;

	if (pTx == NULL) return out_length;
	if (pTx->valid != 1) return out_length;

	if (pTx->state > 90) return out_length;

	// check if all data has been send
	if (pTx->pos >= pTx->len) {
		pTx->valid = 2; /* done */
		pTx->state = 98;
		return out_length;
	}

	if (pTx->state == 1) {
		/* check for the 'break' 0 byte or the address */
		if ((rx == 0) || (rx == pTx->address)) {
			pTx->state = 2; // break byte
			if (rx == pTx->address) pTx->state = 3;
		} else {
			// not our time slot, wait for break
			pTx->state = 0;
		}
	} else if (pTx->state == 2) {
		if (rx == pTx->address) {
			pTx->state = 3;
		} else {
			// not our slot
			pTx->state = 0;
		}
	} else if (pTx->state == 4) {
		/* we just send our address, we should receive the echo */
		if (rx == pTx->address) {
			pTx->state = 5;
		} else {
			// something went wrong, abort
			pTx->valid = 3; // error
			pTx->state = 99;
		}
	} else if (pTx->state == 5) {
		/* check for inverse address from boiler */
		if (rx == (0xFF - pTx->address)) {
			pTx->state = 10; // start tx
		} else {
			// something went wrong, abort
			pTx->valid = 3; // error
			pTx->state = 99;
		}
	}

	if (pTx->state == 14) { // receiving bytes
		pTx->buffer[pTx->pos++] = rx;
		if (pTx->rx_bytes > 0) pTx->rx_bytes--;
		if (pTx->rx_bytes) {
			// check for rx bytes
			pTx->state = 14;
		} else {
			// msg done, check for next
			pTx->state = 10;
		}
	}

	if (pTx->state == 13) { // echo from boiler
		if (rx == pTx->buffer[pTx->pos]) {
			pTx->pos++;
			if (pTx->tx_bytes > 0) pTx->tx_bytes--;
			if (pTx->tx_bytes > 0) {
				pTx->state = 11;
			} else if (pTx->rx_bytes > 0) {
				// check for rx bytes
				pTx->state = 14;
			} else {
				// msg done, check for next
				pTx->state = 10;
			}
		} else {
			// something went wrong, abort
			pTx->valid = 3; // error
			pTx->state = 99;
		}
	}

	if (pTx->state == 10) {
		/* start first msg byte */
		if (pTx->pos < pTx->len) {
			if (pTx->buffer[pTx->pos] & 0x80) {
				pTx->tx_bytes = 4;
				pTx->rx_bytes = 2; /* response plus inverted response */
			} else {
				pTx->tx_bytes = 4;
				pTx->rx_bytes = 0;
			}
			pTx->state = 11;
		} else {
			pTx->valid = 2; /* done */
			pTx->state = 20; /* done */
		}
	}

	if (pTx->state == 12) { // own echo
		if (rx == pTx->buffer[pTx->pos]) {
			pTx->state = 13;
		} else {
			// something went wrong, abort
			pTx->valid = 3; // error
			pTx->state = 99;
		}
	}
	if (pTx->state == 11) {
		/* transmit byte */
		USF(IRTUART_UART) = pTx->buffer[pTx->pos];
		pTx->state = 12;
	}

	return out_length;
}

static void irtuart_rx_intr_handler(void * para) {
	static uint8_t length;
	static uint8_t uart_buffer[IRT_MAXBUFFERSIZE + 2];

	unsigned long now_millis = millis();

	// is a new buffer? if so init the thing for a new telegram
	if (IRT_Sys_Status.irtRxStatus == IRT_RX_STATUS_IDLE) {
		IRT_Sys_Status.irtRxStatus = IRT_RX_STATUS_BUSY; // status set to busy
		length                     = 0;
	}

	/* If we have valid transmit buffer, detect break before processing data */
	if ((pIRTTxBuf) && (pIRTTxBuf->valid == 1)) {
		if ( (pIRTTxBuf->state < 20) && ((now_millis - pIRTTxBuf->start_time) >= IRTUART_TX_MSG_TIMEOUT_MS) ) {
			pIRTTxBuf->state = 97; // abort on timeout
			pIRTTxBuf->valid = 3; // exit
		}
		if (pIRTTxBuf->state == 0) {
			if ( USIS(IRTUART_UART) & (1 << UIBD) ) {
				// we have a break, go to next state
				pIRTTxBuf->state = 1;
			}
		}
	}

    // fill IRQ buffer, by emptying Rx FIFO
    uint8_t rx_cnt = 0;
    if (USIS(IRTUART_UART) & ((1 << UIFF) | (1 << UITO) | (1 << UIBD))) {
        while (((USS(IRTUART_UART) >> USRXC) & 0xFF) && (rx_cnt < 100)) {
				rx_cnt ++;
            uint8_t rx = USF(IRTUART_UART);
            if (length < IRT_MAXBUFFERSIZE) uart_buffer[length++] = rx;

				length = irtuart_tx_received_byte(pIRTTxBuf, rx, uart_buffer, length);
        }

        // clear Rx FIFO full and Rx FIFO timeout interrupts
        USIC(IRTUART_UART) = (1 << UIFF) | (1 << UITO);
	}
	if ((pIRTTxBuf) && (pIRTTxBuf->valid == 1)) {
		if (pIRTTxBuf->state == 3) {

			/* send own address, to let the boiler know we want to transmit */
			USF(IRTUART_UART) = pIRTTxBuf->address;
			pIRTTxBuf->state = 4;
		}
	}


    // BREAK detection = End of IRT data block or overflow
    if ((USIS(IRTUART_UART) & ((1 << UIBD))) || (length >= IRT_MAX_TELEGRAM_LENGTH)) {
        ETS_UART_INTR_DISABLE();          // disable all interrupts and clear them
        USIC(IRTUART_UART) = (1 << UIBD); // INT clear the BREAK detect interrupt

        pIRTRxBuf->length = (length > IRT_MAXBUFFERSIZE) ? IRT_MAXBUFFERSIZE : length;
        os_memcpy((void *)pIRTRxBuf->buffer, (void *)&uart_buffer, pIRTRxBuf->length); // copy data into transfer buffer, including the BRK 0x00 at the end
        length                     = 0;
        IRT_Sys_Status.irtRxStatus = IRT_RX_STATUS_IDLE; // set the status flag stating BRK has been received and we can start a new package
        ETS_UART_INTR_ENABLE();                          // re-enable UART interrupts

        system_os_post(IRTUART_recvTaskPrio, 0, 0); // call irtuart_recvTask() at next opportunity
    }

}

/*
 * system task triggered on BRK interrupt
 * incoming received messages are always asynchronous
 * The full buffer is sent to the irt_parseTelegram() function in irt.cpp.
 */
static void ICACHE_FLASH_ATTR irtuart_recvTask(os_event_t * events) {
    _IRTRxBuf * pCurrent = pIRTRxBuf;
    pIRTRxBuf            = paIRTRxBuf[++irtRxBufIdx % IRT_MAXBUFFERS]; // next free IRT Receive buffer
    uint8_t length       = pCurrent->length;                           // number of bytes including the BRK at the end
    pCurrent->length     = 0;

    // parse mesg
    irt_parseTelegram((uint8_t *)pCurrent->buffer, length - 1 + 1); // decode IRT buffer, excluding the BRK
}

/*
 * flush everything left over in buffer, this clears both rx and tx FIFOs
 */
static inline void ICACHE_FLASH_ATTR irtuart_flush_fifos() {
    uint32_t tmp = ((1 << UCRXRST) | (1 << UCTXRST)); // bit mask
    USC0(IRTUART_UART) |= (tmp);                      // set bits
    USC0(IRTUART_UART) &= ~(tmp);                     // clear bits
}

/*
 * init UART0 driver
 */
void ICACHE_FLASH_ATTR irtuart_set_registers() {

    // pin settings
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0RXD_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD);

    // set 4800, 8 bits, no parity check, 1 stop bit
    USD(IRTUART_UART)  = (UART_CLK_FREQ / IRTUART_BAUD);
    if ((EMSESP_Settings.tx_mode == 5) || (EMSESP_Settings.tx_mode == 6) ) {
		USC0(IRTUART_UART) = IRTUART_CONFIG_ACTIVE; // 8N1
    } else {
		USC0(IRTUART_UART) = IRTUART_CONFIG_PASSIF; // 8N1
    }
    irtuart_flush_fifos();

    // conf1 params
    // UCTOE = RX TimeOut enable (default is 1)
    // UCTOT = RX TimeOut Threshold (7 bit) = want this when no more data after 1 characters (default is 2)
    // UCFFT = RX FIFO Full Threshold (7 bit) = want this to be 31 for 32 bytes of buffer (default was 127)
    // see https://www.espressif.com/sites/default/files/documentation/esp8266-technical_reference_en.pdf
    //
    // change: we set UCFFT to 1 to get an immediate indicator about incoming traffic.
    //         Otherwise, we're only noticed by UCTOT or RxBRK!
    USC1(IRTUART_UART) = 0;                                                // reset config first
    USC1(IRTUART_UART) = (0x01 << UCFFT) | (0x01 << UCTOT) | (0 << UCTOE); // enable interupts

    // set interrupts for triggers
    USIC(IRTUART_UART) = 0xFFFF; // clear all interupts
    USIE(IRTUART_UART) = 0;      // disable all interrupts

    // enable rx break, fifo full and timeout.
    // but not frame error UIFR (because they are too frequent) or overflow UIOF because our buffer is only max 32 bytes
    // change: we don't care about Rx Timeout - it may lead to wrong readouts
    USIE(IRTUART_UART) = (1 << UIBD) | (1 << UIFF) | (0 << UITO);

}


/*
 * init UART0 driver
 */
void ICACHE_FLASH_ATTR irtuart_init() {
    ETS_UART_INTR_DISABLE();
    ETS_UART_INTR_ATTACH(nullptr, nullptr);

    // allocate and preset IRT Receive buffers
    for (int i = 0; i < IRT_MAXBUFFERS; i++) {
        _IRTRxBuf * p = (_IRTRxBuf *)malloc(sizeof(_IRTRxBuf));
        paIRTRxBuf[i] = p;
    }
    pIRTRxBuf = paIRTRxBuf[0]; // preset IRT Rx Buffer

    // Create a transmit buffer
    pIRTTxBuf = (_IRTTxBuf *)malloc(sizeof(_IRTTxBuf));
    pIRTTxBuf->valid = 0;
    pIRTTxBuf->state = 0;

    irtuart_set_registers();

    // set up interrupt callbacks for Rx
    system_os_task(irtuart_recvTask, IRTUART_recvTaskPrio, irtRecvTaskQueue, IRTUART_recvTaskQueueLen);

    // disable esp debug which will go to Tx and mess up the line - see https://github.com/espruino/Espruino/issues/655
    system_set_os_print(0);

    // swap Rx and Tx pins to use GPIO13 (D7) and GPIO15 (D8) respectively
    system_uart_swap();

    ETS_UART_INTR_ATTACH(irtuart_rx_intr_handler, nullptr);
    ETS_UART_INTR_ENABLE();
}


/*
 * init UART0 driver
 */
void ICACHE_FLASH_ATTR irtuart_setup() {
    ETS_UART_INTR_DISABLE();

    irtuart_set_registers();

    ETS_UART_INTR_ENABLE();
}

/*
 * stop UART0 driver
 * This is called prior to an OTA upload and also before a save to SPIFFS to prevent conflicts
 */
void ICACHE_FLASH_ATTR irtuart_stop() {
    ETS_UART_INTR_DISABLE();
    _disable_rxtx = 1;
}

/*
 * re-start UART0 driver
 */
void ICACHE_FLASH_ATTR irtuart_start() {
    ETS_UART_INTR_ENABLE();
    _disable_rxtx = 0;
}

/*
 * Send a BRK signal
 * Which is a 11-bit set of zero's (11 cycles)
 */
void ICACHE_FLASH_ATTR irtuart_tx_brk() {
    uint32_t tmp;

    // must make sure Tx FIFO is empty
    while (((USS(IRTUART_UART) >> USTXC) & 0xFF) != 0)
        ;

    tmp = ((1 << UCRXRST) | (1 << UCTXRST)); // bit mask
    USC0(IRTUART_UART) |= (tmp);             // set bits
    USC0(IRTUART_UART) &= ~(tmp);            // clear bits

    // To create a 11-bit <BRK> we set TXD_BRK bit so the break signal will
    // automatically be sent when the tx fifo is empty
    tmp = (1 << UCBRK);
    USC0(IRTUART_UART) |= (tmp); // set bit

    delayMicroseconds(IRTUART_TX_BRK_WAIT);

    USC0(IRTUART_UART) &= ~(tmp); // clear bit
}

/*
 * check if we are not transmitting, busy transmitting
 * or have a complete buffer
 */

uint16_t ICACHE_FLASH_ATTR irtuart_check_tx(uint8_t reset_if_done)
{
	uint8_t tx_valid;
	uint8_t tx_state;
	uint16_t tx_ret;

	unsigned long now_millis = millis();

	if (_disable_rxtx) return 0x0100;

	ETS_UART_INTR_DISABLE(); // disable rx interrupt
	tx_valid = 0;
	tx_state = 0;
	if (pIRTTxBuf) {
		// check for timeout of message
		if (pIRTTxBuf->valid == 1) {
			if ((now_millis - pIRTTxBuf->start_time) >= IRTUART_TX_MSG_TIMEOUT_MS) {
				pIRTTxBuf->state = 97; // abort on timeout
				pIRTTxBuf->valid = 3; // exit
			}
		}
		// copy current state with irq's off
		tx_valid = pIRTTxBuf->valid;
		tx_state = pIRTTxBuf->state;
		if (pIRTTxBuf->valid > 1) {
			pIRTTxBuf->valid = 0;
			pIRTTxBuf->state = 0;
		}
	}
	ETS_UART_INTR_ENABLE(); // receive anything from FIFO...

	// combine valid and state as single entry
	tx_ret = tx_valid;
	tx_ret = tx_ret << 8;
	tx_ret |= tx_state;

	return tx_ret;
}
uint16_t ICACHE_FLASH_ATTR irtuart_send_tx_buffer(uint8_t address, uint8_t *telegram, uint8_t len)
{
	uint16_t status;

	if ((pIRTTxBuf == NULL) || (len >= IRT_MAXTXBUFFERSIZE)) return 0xFF00;

	if (_disable_rxtx) return 0xFF00;

	status = irtuart_check_tx(1);
	// if we are still processing or there is a finished buffer
	// return status, only if empty add buffer
	if ((len < 1) || (status >= 0x0100)) return status;

	// the irq will only pick-up a buffer if valid == 1
	// we just checked it is  not, so no need to disable irq

	pIRTTxBuf->state = 0;
	pIRTTxBuf->start_time = millis();
	pIRTTxBuf->valid = 0;
	pIRTTxBuf->address = address;

	memcpy(pIRTTxBuf->buffer, telegram, len);

	pIRTTxBuf->tx_bytes = 0;
	pIRTTxBuf->rx_bytes = 0;

	pIRTTxBuf->pos = 0;
	pIRTTxBuf->len = len;

	pIRTTxBuf->state = 0;
	// The last instruction puts the buffer to valid
	// after this it may be picked up by the irq
	pIRTTxBuf->valid = 1;

	return 0x0100;
}
