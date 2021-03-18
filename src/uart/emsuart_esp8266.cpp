/*
 * EMS-ESP - https://github.com/emsesp/EMS-ESP
 * Copyright 2020  Paul Derbyshire
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(ESP8266)

#include "uart/emsuart_esp8266.h"

#include "emsesp.h"

namespace emsesp {

typedef struct {
    uint8_t length;
    uint8_t buffer[EMS_MAXBUFFERSIZE];
} EMSRxBuf_t;

os_event_t   recvTaskQueue[EMSUART_recvTaskQueueLen]; // our Rx queue
EMSRxBuf_t   aEMSRxBuf[EMS_MAXBUFFERS];
EMSRxBuf_t * pEMSRxBuf     = &aEMSRxBuf[0];
EMSRxBuf_t * pCurrent      = pEMSRxBuf;
uint8_t      emsRxBufIdx_  = 0;
uint8_t      tx_mode_      = 0xFF;
bool         drop_next_rx_ = true;

//
// Main interrupt handler
// Important: must not use ICACHE_FLASH_ATTR
//
void ICACHE_RAM_ATTR EMSuart::emsuart_rx_intr_handler(void * para) {

    if (USIR(EMSUART_UART) & (1 << UIBD)) {  // BREAK detection = End of EMS data block
        USC0(EMSUART_UART) &= ~(1 << UCBRK); // reset tx-brk
        USIC(EMSUART_UART) = (1 << UIBD);    // INT clear the BREAK detect interrupt
        pEMSRxBuf->length = 0;
        while ((USS(EMSUART_UART) >> USRXC) & 0x0FF) { // read fifo into buffer
            uint8_t rx = USF(EMSUART_UART);
            if (pEMSRxBuf->length < EMS_MAXBUFFERSIZE) {
                if (pEMSRxBuf->length || rx) { // skip a leading zero
                    pEMSRxBuf->buffer[pEMSRxBuf->length++] = rx;
                }
            } else {
                drop_next_rx_ = true;
            }
        }
        if (pEMSRxBuf->buffer[pEMSRxBuf->length - 1]) { // check if last byte is break
            pEMSRxBuf->length++;
        }
        // Ignore telegrams with no data value, then transmit EMS buffer, excluding the BRK
        if (!drop_next_rx_ && (pEMSRxBuf->length > 4 || pEMSRxBuf->length == 2)) {
            pCurrent  = pEMSRxBuf;                                   // current buffer to receive task
            pEMSRxBuf = &aEMSRxBuf[++emsRxBufIdx_ % EMS_MAXBUFFERS]; // next free EMS Receive buffer
            system_os_post(EMSUART_recvTaskPrio, 0, 0);              // call emsuart_recvTask() at next opportunity
        }
        drop_next_rx_ = false;
    }
}

/*
 * system task triggered on BRK interrupt
 * incoming received messages are always asynchronous
 * The full buffer is sent to EMSESP::incoming_telegram()
 */
void ICACHE_FLASH_ATTR EMSuart::emsuart_recvTask(os_event_t * events) {

    EMSESP::incoming_telegram((uint8_t *)pCurrent->buffer, pCurrent->length - 1);
}

/*
 * init UART0 driver
 */
void ICACHE_FLASH_ATTR EMSuart::start(const uint8_t tx_mode, const uint8_t rx_gpio, const uint8_t tx_gpio) {
    if (tx_mode_ != 0xFF) { // it's a restart no need to configure uart
        tx_mode_ = tx_mode;
        restart();
        return;
    }
    tx_mode_ = tx_mode;

    ETS_UART_INTR_ATTACH(nullptr, nullptr);

    // pin settings
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0RXD_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD);

    // set 9600, 8 bits, no parity check, 1 stop bit
    USD(EMSUART_UART)  = (UART_CLK_FREQ / EMSUART_BAUD);
    USC0(EMSUART_UART) = EMSUART_CONFIG; // 8N1
    USC0(EMSUART_UART) |= ((1 << UCRXRST) | (1 << UCTXRST));  // set clear fifo bits
    USC0(EMSUART_UART) &= ~((1 << UCRXRST) | (1 << UCTXRST)); // clear bits

    // conf1 params
    // UCTOE = RX TimeOut enable (default is 1)
    // UCTOT = RX TimeOut Threshold (7 bit) = want this when no more data after 1 characters (default is 2)
    // UCFFT = RX FIFO Full Threshold (7 bit) = want this to be 31 for 32 bytes of buffer (default was 127)
    // see https://www.espressif.com/sites/default/files/documentation/esp8266-technical_reference_en.pdf
    //
    // change: don't care, we do not use these interrupts
    // USC1(EMSUART_UART) = (0x7F << UCFFT) | (0x01 << UCTOT) | (1 << UCTOE); // enable interupts

    // set interrupts for triggers
    USIC(EMSUART_UART) = 0xFFFF; // clear all interupts
    USIE(EMSUART_UART) = 0;      // disable all interrupts

    // set up interrupt callbacks for Rx
    system_os_task(emsuart_recvTask, EMSUART_recvTaskPrio, recvTaskQueue, EMSUART_recvTaskQueueLen);

    // disable esp debug which will go to Tx and mess up the line - see https://github.com/espruino/Espruino/issues/655
    system_set_os_print(0);

    if (tx_gpio == 1 && rx_gpio == 3) {
        system_uart_de_swap();
    } else if (tx_gpio == 15 && rx_gpio == 13) {
        system_uart_swap(); // swap Rx and Tx pins to use GPIO13 (D7) and GPIO15 (D8) respectively
    }

    ETS_UART_INTR_ATTACH(emsuart_rx_intr_handler, nullptr);
    drop_next_rx_ = true;

    restart();
}

/*
 * stop UART0 driver
 * This is called prior to an OTA upload and also before a save to the filesystem to prevent conflicts
 */
void ICACHE_FLASH_ATTR EMSuart::stop() {
    USIE(EMSUART_UART) = 0;              // disable receive interrupts
    USC0(EMSUART_UART) &= ~(1 << UCBRK); // clear Tx-BRK bit
}

/*
 * re-start UART0 driver
 */
void ICACHE_FLASH_ATTR EMSuart::restart() {
    if (USIR(EMSUART_UART) & ((1 << UIBD))) {
        USIC(EMSUART_UART) = (1 << UIBD); // INT clear the <brk> detect interrupt
        drop_next_rx_      = true;
    }
    USIE(EMSUART_UART) = (1 << UIBD); // enable brk interrupt
}

/*
 * Sends a 1-byte poll, ending with a <BRK>
 */
void ICACHE_FLASH_ATTR EMSuart::send_poll(uint8_t data) {
    transmit(&data, 1);
}

/*
 * Send data to Tx line, ending with a <BRK>
 * buf contains the CRC and len is #bytes including the CRC
 * returns code, 0=success, 1=brk error, 2=watchdog timeout
 */
uint16_t ICACHE_FLASH_ATTR EMSuart::transmit(uint8_t * buf, uint8_t len) {
    if (len == 0 || len >= EMS_MAXBUFFERSIZE) {
        return EMS_TX_STATUS_ERR; // nothing or to much to send
    }
    if (tx_mode_ == 0) {
        return EMS_TX_STATUS_OK;
    }

    // See https://github.com/emsesp/EMS-ESP/issues/380
    if (tx_mode_ == EMS_TXMODE_HW) { // tx_mode 4
        for (uint8_t i = 0; i < len; i++) {
            USF(EMSUART_UART) = buf[i];
        }
        USC0(EMSUART_UART) |= (1 << UCBRK); // send <BRK> at the end, clear by interrupt
        return EMS_TX_STATUS_OK;
    }

    // EMS+ https://github.com/emsesp/EMS-ESP/issues/23#
    if (tx_mode_ == EMS_TXMODE_EMSPLUS) { // tx_mode 2, With extra tx delay for EMS+
        for (uint8_t i = 0; i < len; i++) {
            USF(EMSUART_UART) = buf[i];
            delayMicroseconds(EMSUART_TX_WAIT_PLUS); // 2070
        }
        USC0(EMSUART_UART) |= (1 << UCTXI); // set break
        delayMicroseconds(EMSUART_TX_BRK_PLUS);
        USC0(EMSUART_UART) &= ~(1 << UCTXI);
        return EMS_TX_STATUS_OK;
    }

    // Junkers logic by @philrich, tx_mode 3
    if (tx_mode_ == EMS_TXMODE_HT3) {
        for (uint8_t i = 0; i < len; i++) {
            USF(EMSUART_UART) = buf[i];
            // just to be safe wait for tx fifo empty (still needed?)
            while (((USS(EMSUART_UART) >> USTXC) & 0xff)) {
            }
            // wait until bits are sent on wire
            delayMicroseconds(EMSUART_TX_WAIT_HT3);
        }
        USC0(EMSUART_UART) |= (1 << UCTXI); // set break bit
        delayMicroseconds(EMSUART_TX_BRK_HT3);
        USC0(EMSUART_UART) &= ~(1 << UCTXI);
        return EMS_TX_STATUS_OK;
    }

    /*
     * Logic for tx_mode of 1
     * based on code from https://github.com/emsesp/EMS-ESP/issues/103 by @susisstrolch
     * 
     * Logic (modified by @MichaelDvP):
     * wait after each byte for the master echo
     * after last byte echo send a fixed break and leave.
     * The master echo will trigger the interrupt.
     */

    // send the bytes along the serial line
    for (uint8_t i = 0; i < len; i++) {
        volatile uint8_t _usrxc     = (USS(EMSUART_UART) >> USRXC) & 0xFF;
        uint16_t         timeoutcnt = EMSUART_TX_TIMEOUT;
        USF(EMSUART_UART)           = buf[i]; // send each Tx byte
        // wait for echo
        while ((((USS(EMSUART_UART) >> USRXC) & 0xFF) == _usrxc) && (--timeoutcnt > 0)) {
            delayMicroseconds(EMSUART_TX_BUSY_WAIT); // burn CPU cycles...
        }
    }
    USC0(EMSUART_UART) |= (1 << UCTXI); // snd break
    delayMicroseconds(EMSUART_TX_BRK_EMS);
    USC0(EMSUART_UART) &= ~(1 << UCTXI);
    return EMS_TX_STATUS_OK; // send the Tx ok status back
}

} // namespace emsesp

#endif
