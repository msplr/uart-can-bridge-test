#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <serial-can-bridge/serial_can_bridge.h>
#include "hal.h"

void serial_interface_write(void *arg, const void *data, size_t len)
{
    if (len > 0) {
        // chSequentialStreamWrite((BaseSequentialStream*)arg, (const uint8_t*)data, len);
        sdWrite((SerialDriver *)arg, data, len);
    }
}

size_t serial_interface_read(void *arg, void *buf, size_t max)
{
    // return chSequentialStreamRead((BaseSequentialStream*)arg, buf, max);
    return sdReadTimeout((SerialDriver *)arg, buf, max, 10);
}

void can_interface_send(bool ext, uint32_t id, void *data, uint8_t len)
{
    canmbx_t mailbox = 0;
    systime_t timeout = TIME_INFINITE;

    CANTxFrame ctf;

    ctf.DLC = len;
    ctf.RTR = 0;

    if (ext) {
        ctf.IDE = 1;
        ctf.EID = id;
    } else {
        ctf.IDE = 0;
        ctf.SID = id;
    }

    memcpy(ctf.data8, data, len);

    canTransmit(&CAND2, mailbox, &ctf, timeout);
}

void can_interface_receive(bool *ext, uint32_t *id, void *data, uint8_t *len)
{
    CANRxFrame crf;
    canmbx_t mailbox = 0;
    systime_t timeout = TIME_INFINITE;

    canReceive(&CAND2, mailbox, &crf, timeout);

    // extended frame
    if (crf.IDE == 1) {
        *ext = true;
    } else {
        *ext = false;
    }

    // id
    if (crf.IDE == 1) {
        *id = crf.SID;
    } else {
        *id = crf.EID;
    }

    // data
    memcpy(data, crf.data8, crf.DLC);

    // length
    *len = crf.DLC;

}

static THD_WORKING_AREA(bridge_rx_wa, 512);
static THD_FUNCTION(bridge_rx, arg)
{
    serial_rx_main(arg);
    return 0;
}

static THD_WORKING_AREA(bridge_tx_wa, 512);
static THD_FUNCTION(bridge_tx, arg)
{
    serial_tx_main(arg);
    return 0;
}

static CANConfig can_config = {
    .mcr = 0,
    .btr = (1 << 0) | (11 << 16) | (7 << 20) | (0 << 24)
};

void run_bridge(void)
{
    BaseSequentialStream *serial = (BaseSequentialStream *) &UART_CONN2;
    sdStart(&UART_CONN2, NULL);

    canStart(&CAND2, &can_config);

    chThdCreateStatic(bridge_rx_wa, sizeof(bridge_rx_wa), NORMALPRIO, bridge_rx, serial);
    chThdCreateStatic(bridge_tx_wa, sizeof(bridge_tx_wa), NORMALPRIO, bridge_tx, serial);
}
