/*
 * irtuart.h
 *
 * Header file for irtuart.cpp
 *
 * Paul Derbyshire - https://github.com/proddy/EMS-ESP
 */
#pragma once

#include <irt.h>

#define IRTUART_UART 0      // UART 0
#define IRTUART_CONFIG_PASSIF (0x1C | (1 << UCRXI) ) // 8N1 (8 bits, no stop bits, 1 parity), invert Rxd
#define IRTUART_CONFIG_ACTIVE (0x1C | (1 << UCRXI) | (1 << UCTXI)) // 8N1 (8 bits, no stop bits, 1 parity), invert Rxd, Invert TX


#define IRTUART_BAUD 4800   // uart baud rate for the IRT circuit
//#define IRTUART_BAUD 9600   // uart baud rate for the IRT circuit

#define IRT_MAXBUFFERS 5                                // buffers for circular filling to avoid collisions
#define IRT_MAXBUFFERSIZE (IRT_MAX_TELEGRAM_LENGTH + 2) // max size of the buffer. IRT packets are max 32 bytes, plus extra 2 for BRKs

#define IRTUART_BIT_TIME 208 // bit time @4800 baud

#define IRTUART_TX_BRK_WAIT 2070 // the BRK from Boiler master is roughly 1.039ms, so accounting for hardware lag using around 2078 (for half-duplex) - 8 (lag)
#define IRTUART_TX_WAIT_BYTE IRTUART_BIT_TIME * 10 // Time to send one Byte (8 Bits, 1 Start Bit, 1 Stop Bit)
#define IRTUART_TX_WAIT_BRK IRTUART_BIT_TIME * 11  // Time to send a BRK Signal (11 Bit)
#define IRTUART_TX_WAIT_GAP IRTUART_BIT_TIME * 7   // Gap between to Bytes
#define IRTUART_TX_LAG 8

#define IRTUART_recvTaskPrio 1
#define IRTUART_recvTaskQueueLen 64

typedef struct {
    uint8_t length;
    uint8_t buffer[IRT_MAXBUFFERSIZE];
} _IRTRxBuf;

void ICACHE_FLASH_ATTR irtuart_init();
void ICACHE_FLASH_ATTR irtuart_stop();
void ICACHE_FLASH_ATTR irtuart_start();
_EMS_TX_STATUS ICACHE_FLASH_ATTR irtuart_tx_buffer(uint8_t * buf, uint8_t len);
