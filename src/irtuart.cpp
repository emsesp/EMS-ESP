/*
 * irtuart.cpp
 *
 * The low level UART code for ESP8266 to read and write to the EMS bus via uart
 * Paul Derbyshire - https://github.com/proddy/EMS-ESP
 */

#include "irtuart.h"
#include "irt.h"
#include <user_interface.h>
#include "ems.h"

#include "MyESP.h"
// MyESP class for logging to telnet and serial
#define myDebug(...) myESP.myDebug(__VA_ARGS__)
#define myDebug_P(...) myESP.myDebug_P(__VA_ARGS__)


_IRTRxBuf * pIRTRxBuf;
_IRTRxBuf * paIRTRxBuf[IRT_MAXBUFFERS];
uint8_t     irtRxBufIdx  = 0;
_IRTTxBuf * pIRTTxBuf;

os_event_t irtRecvTaskQueue[IRTUART_recvTaskQueueLen]; // our Rx queue

extern _EMSESP_Settings EMSESP_Settings;
//
// Main interrupt handler
// Important: do not use ICACHE_FLASH_ATTR !
//

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

static uint8_t irtuart_tx_received_byte(_IRTTxBuf *pTx, uint8_t rx, uint8_t *buffer, uint8_t in_length)
{
	/* for each incoming byte, test if we need to send anything */

	uint8_t out_length = in_length;

	if (pTx == NULL) return out_length;
	if (pTx->valid != 1) return out_length;

	if (pTx->state > 90) return out_length;

	if (pTx->pos >= pTx->len) {
		out_length = add_debug_to_buf(buffer, out_length, "98");
		pTx->valid = 2; /* done */
		pTx->state = 98;
		return out_length;
	}

	if (pTx->state == 1) {
		/* check for the 'break' 0 byte or the address */
		if ((rx == 0) || (rx == pTx->address)) {
			pTx->state = 2; // break byte
//			length = add_debug_to_buf(uart_buffer, length, "2");
			if (rx == pTx->address) pTx->state = 3;
		} else {
			// not our time slot
//			length = add_debug_to_buf(uart_buffer, length, "3");
			pTx->state = 0;
//			length = add_debug_to_buf(uart_buffer, length, "0");
		}
	} else if (pTx->state == 2) {
//		length = add_debug_to_buf(uart_buffer, length, "4");
		if (rx == pTx->address) {
//			out_length = add_debug_to_buf(buffer, out_length, "5");
			pTx->state = 3;
		} else {
			// not our slot
			pTx->state = 0;
		}
	} else if (pTx->state == 4) {
		/* we just send our address, we should receive the echo */
//		out_length = add_debug_to_buf(buffer, out_length, "7");
		if (rx == pTx->address) {
//			out_length = add_debug_to_buf(buffer, out_length, "8");
			pTx->state = 5;
		} else {
			// something went wrong, abort
			out_length = add_debug_to_buf(buffer, out_length, "99");
			pTx->valid = 3; // error
			pTx->state = 99;
		}
	} else if (pTx->state == 5) {
		/* check for inverse from boiler */
//		out_length = add_debug_to_buf(buffer, out_length, "9");
		if (rx == (0xFF - pTx->address)) {
//			out_length = add_debug_to_buf(buffer, out_length, "10");
			pTx->state = 10; // start tx
		} else {
			// something went wrong, abort
			out_length = add_debug_to_buf(buffer, out_length, "99");
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
			out_length = add_debug_to_buf(buffer, out_length, "99");
			pTx->valid = 3; // error
			pTx->state = 99;
		}
	}

	if (pTx->state == 10) {
//		out_length = add_debug_to_buf(buffer, out_length, "20");
		/* start first msg byte */
		if (pTx->pos < pTx->len) {
			if (pTx->buffer[pTx->pos] & 0x80) {
				pTx->tx_bytes = 4;
				pTx->rx_bytes = 2; /* response plus inverted response */
			} else {
				pTx->tx_bytes = 4;
				pTx->rx_bytes = 0;
			}
//			out_length = add_debug_to_buf(buffer, out_length, "21");
			pTx->state = 11;
		} else {
			out_length = add_debug_to_buf(buffer, out_length, "30");
			pTx->valid = 2; /* done */
			pTx->state = 20; /* done */
		}
	}

	if (pTx->state == 12) { // own echo
		if (rx == pTx->buffer[pTx->pos]) {
			pTx->state = 13;
		} else {
			// something went wrong, abort
			out_length = add_debug_to_buf(buffer, out_length, "99");
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
	if (EMS_Sys_Status.emsRxStatus == EMS_RX_STATUS_IDLE) {
		EMS_Sys_Status.emsRxStatus = EMS_RX_STATUS_BUSY; // status set to busy
		length                     = 0;
	}

	/* If we have valid transmit buffer, detect break before processing data */
	if ((pIRTTxBuf) && (pIRTTxBuf->valid == 1)) {
		if ( (pIRTTxBuf->state < 20) && ((now_millis - pIRTTxBuf->start_time) >= IRTUART_TX_MSG_TIMEOUT_MS) ) {
			length = add_debug_to_buf(uart_buffer, length, "97");
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
//			length = add_debug_to_buf(uart_buffer, length, "6");

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
        EMS_Sys_Status.emsRxStatus = EMS_RX_STATUS_IDLE; // set the status flag stating BRK has been received and we can start a new package
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

    // pin settings
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0RXD_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD);

    // set 4800, 8 bits, no parity check, 1 stop bit
    USD(IRTUART_UART)  = (UART_CLK_FREQ / IRTUART_BAUD);
    if (EMSESP_Settings.tx_mode == 5) {
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
 * stop UART0 driver
 * This is called prior to an OTA upload and also before a save to SPIFFS to prevent conflicts
 */
void ICACHE_FLASH_ATTR irtuart_stop() {
    ETS_UART_INTR_DISABLE();
}

/*
 * re-start UART0 driver
 */
void ICACHE_FLASH_ATTR irtuart_start() {
    ETS_UART_INTR_ENABLE();
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
//    GPIO_H(TX_MARK_MASK);
    USC0(IRTUART_UART) |= (tmp); // set bit

    if (EMS_Sys_Status.emsTxMode == IRT_TXMODE_IRTPLUS) { // IRT+ mode
        delayMicroseconds(IRTUART_TX_BRK_WAIT);
    } else if (EMS_Sys_Status.emsTxMode == IRT_TXMODE_HT3) {     // junkers mode
        delayMicroseconds(IRTUART_TX_WAIT_BRK - IRTUART_TX_LAG); // 1144 (11 Bits)
    }

    USC0(IRTUART_UART) &= ~(tmp); // clear bit
//    GPIO_L(TX_MARK_MASK);
}

/*
 * Send to Tx, ending with a <BRK>
 */
_EMS_TX_STATUS ICACHE_FLASH_ATTR irtuart_tx_buffer(uint8_t * buf, uint8_t len) {
    _EMS_TX_STATUS result = EMS_TX_STATUS_OK;

    if (EMS_Sys_Status.emsLogging == EMS_SYS_LOGGING_JABBER) {
        ems_dumpBuffer("irtuart_tx_buffer: ", buf, len); // validate and transmit the IRT buffer, excluding the BRK
    }

    if (len) {
//        LA_PULSE(50);

        if (EMS_Sys_Status.emsTxMode == IRT_TXMODE_IRTPLUS) { // With extra tx delay for IRT+
            for (uint8_t i = 0; i < len; i++) {
//                TX_PULSE(IRTUART_BIT_TIME / 4);
                USF(IRTUART_UART) = buf[i];
                delayMicroseconds(IRTUART_TX_BRK_WAIT); // https://github.com/proddy/EMS-ESP/issues/23#
            }
            irtuart_tx_brk();                       // send <BRK>
        } else if (EMS_Sys_Status.emsTxMode == IRT_TXMODE_HT3) { // Junkers logic by @philrich
            for (uint8_t i = 0; i < len; i++) {
//                TX_PULSE(IRTUART_BIT_TIME / 4);
                USF(IRTUART_UART) = buf[i];

                // just to be safe wait for tx fifo empty (needed?)
                while (((USS(IRTUART_UART) >> USTXC) & 0xff) != 0)
                    ;

                // wait until bits are sent on wire
                delayMicroseconds(IRTUART_TX_WAIT_BYTE - IRTUART_TX_LAG + IRTUART_TX_WAIT_GAP);
            }
            irtuart_tx_brk(); // send <BRK>
        } else if (EMS_Sys_Status.emsTxMode == IRT_TXMODE_DEFAULT) {
            /*
        * based on code from https://github.com/proddy/EMS-ESP/issues/103 by @susisstrolch
        * we emit the whole telegram, with Rx interrupt disabled, collecting busmaster response in FIFO.
        * after sending the last char we poll the Rx status until either
        * - size(Rx FIFO) == size(Tx-Telegram)
        * - <BRK> is detected
        * At end of receive we re-enable Rx-INT and send a Tx-BRK in loopback mode.
        *
        * IRT-Bus error handling
        * 1. Busmaster stops echoing on Tx w/o permission
        * 2. Busmaster cancel telegram by sending a BRK
        *
        * Case 1. is handled by a watchdog counter which is reset on each
        * Tx attempt. The timeout should be 20x IRTUART_BIT_TIME plus
        * some smart guess for processing time on targeted IRT device.
        * We set EMS_Sys_Status.irtTxStatus to IRT_TX_WTD_TIMEOUT and return
        *
        * Case 2. is handled via a BRK chk during transmission.
        * We set EMS_Sys_Status.irtTxStatus to IRT_TX_BRK_DETECT and return
        *
        */

// shorter busy poll...
#define IRTUART_BUSY_WAIT (IRTUART_BIT_TIME / 8)
#define IRT_TX_TO_CHARS (2 + 20)
#define IRT_TX_TO_COUNT ((IRT_TX_TO_CHARS)*10 * 8)
            uint16_t wdc = IRT_TX_TO_COUNT;
            ETS_UART_INTR_DISABLE(); // disable rx interrupt

            // clear Rx status register
            USC0(IRTUART_UART) |= (1 << UCRXRST); // reset uart rx fifo
            irtuart_flush_fifos();

            // throw out the telegram...
            for (uint8_t i = 0; i < len && result == EMS_TX_STATUS_OK;) {
//                GPIO_H(TX_MARK_MASK);

                wdc                     = IRT_TX_TO_COUNT;
                volatile uint8_t _usrxc = (USS(IRTUART_UART) >> USRXC) & 0xFF;
                USF(IRTUART_UART)       = buf[i++]; // send each Tx byte
                // wait for echo from busmaster
//                GPIO_L(TX_MARK_MASK);
                while (((USS(IRTUART_UART) >> USRXC) & 0xFF) == _usrxc) {
                    delayMicroseconds(IRTUART_BUSY_WAIT); // burn CPU cycles...
                    if (--wdc == 0) {
                        EMS_Sys_Status.emsTxStatus = result = EMS_TX_WTD_TIMEOUT;
                        break;
                    }
                    if (USIR(IRTUART_UART) & (1 << UIBD)) {
                        USIC(IRTUART_UART)         = (1 << UIBD); // clear BRK detect IRQ
                        EMS_Sys_Status.emsTxStatus = result = EMS_TX_BRK_DETECT;
                    }
                }
            }

            // we got the whole telegram in the Rx buffer
            // on Rx-BRK (bus collision), we simply enable Rx and leave it
            // otherwise we send the final Tx-BRK in the loopback and re=enable Rx-INT.
            // worst case, we'll see an additional Rx-BRK...
            if (result != EMS_TX_STATUS_OK) {
//                LA_PULSE(200); // mark Tx error
            } else {
                // neither bus collision nor timeout - send terminating BRK signal
//                GPIO_H(TX_MARK_MASK);
                if (!(USIS(IRTUART_UART) & (1 << UIBD))) {
                    // no bus collision - send terminating BRK signal
                    USC0(IRTUART_UART) |= (1 << UCLBE) | (1 << UCBRK); // enable loopback & set <BRK>

                    // wait until BRK detected...
                    while (!(USIR(IRTUART_UART) & (1 << UIBD))) {
                        // delayMicroseconds(IRTUART_BUSY_WAIT);
                        delayMicroseconds(IRTUART_BIT_TIME);
                    }

                    USC0(IRTUART_UART) &= ~((1 << UCBRK) | (1 << UCLBE)); // disable loopback & clear <BRK>
                    USIC(IRTUART_UART) = (1 << UIBD);                     // clear BRK detect IRQ
                }
//                GPIO_L(TX_MARK_MASK);
            }
            ETS_UART_INTR_ENABLE(); // receive anything from FIFO...
        }
    }
    return result;
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

void _irtRunTest(uint8_t wc, const char *setting, const char *value)
{
	int sel_test = 0;
	myDebug("Irt test, wc %d\n", wc);

	if ((wc > 0) && (setting != NULL)) {
		sel_test = atoi(setting);
	}
	if (pIRTTxBuf == NULL) {
		myDebug("Irt test, wc %d, test %d\n", wc, sel_test);
	} else {
		myDebug("Irt test, wc %d, test %d, state %d pos %d len %d\n", wc, sel_test, pIRTTxBuf->state, pIRTTxBuf->pos, pIRTTxBuf->len);
	}
// (00:24:24.892) ems_parseTelegram: 00 05: 01 01 FE 90 90 00 00 31 31 54 54 CF 30 82 82 63 63 16 16 1C 1C 00 FF A3 A3 7A 7A 96 96 3D 3D 06 F9 A4 A4 2D 2D 0C 0C CE CE 1C E3 8A 8A D1 D1 A4 A4 93 93 FE 01 00

	if (sel_test == 1) {
		myDebug("Irt test, buffer %x\n", (int)pIRTTxBuf);
		if (pIRTTxBuf == NULL) {
			myDebug("Irt test, no valid transmit buffer");
		} else {
			myDebug("Irt test, start test 1");
//			pIRTTxBuf->state = 0;
//			pIRTTxBuf->start_time = millis();
//			pIRTTxBuf->valid = 0;
//			pIRTTxBuf->address = 2;

			size_t i,j,len = 0;
			char temp_buf[4];
			uint8_t out_buf[IRT_MAXTXBUFFERSIZE];
			if ((wc > 1) && (value)) len = strlen(value);
			if (len > 0) len--;
			j = 0;
			for (i=0; i<len; i+=2) {
				temp_buf[0] = value[i];
				temp_buf[1] = value[i+1];
				temp_buf[2] = 0;
				if (j < IRT_MAXTXBUFFERSIZE) {
					out_buf[j++] = (uint8_t)strtol(temp_buf, 0, 16);
				}
			}
			uint16_t status;

			status = irtuart_send_tx_buffer(2, out_buf, j);
			myDebug("Irt test, status: %04x", status);

//			uint8_t test_msg[] = {0xA4, 0x5D, 0x17, 0x00};
//			uint8_t test_msg[] = {0x90, 0x00, 0x31, 0x54, 0x00, 0x00, 0x90, 0x00, 0x31, 0x54, 0x00, 0x00};
//			uint8_t test_msg[] = {0x90, 0x00, 0x31, 0x54, 0x00, 0x00, 0x07, 0xCF, 0xD0, 0x6C};
//			memcpy(pIRTTxBuf->buffer, test_msg, sizeof(test_msg));

//			pIRTTxBuf->tx_bytes = 0;
//			pIRTTxBuf->rx_bytes = 0;

//			pIRTTxBuf->pos = 0;
//			pIRTTxBuf->len = sizeof(test_msg);
//			pIRTTxBuf->len = j;

//			pIRTTxBuf->start_time = millis();
//			pIRTTxBuf->state = 0;
//			if (len > 0) pIRTTxBuf->valid = 1;

			myDebug("Irt test, start %d test 1 pos %d len %d", pIRTTxBuf->valid, pIRTTxBuf->pos, pIRTTxBuf->len);

		}
	}


//	myDebug("Irt test, state %d\n", pIRTTxBuf->state);
/*
	if (wc == 1) {
		pIRTTxBuf->state = 0;

		pIRTTxBuf->valid = 1;
		pIRTTxBuf->address = 2;
	}*/
}