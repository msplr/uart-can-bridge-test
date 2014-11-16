#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <serial-can-bridge/serial_can_bridge.h>
#include <hal.h>
#include "board.h"
#include <serial-can-bridge/can_frame_cmp.h>
#include <serial-datagram/serial_datagram.h>
#include <serializer/serialization.h>


extern void debug(const char *fmt, ...);

void mock_master_sd_rcv_cb(const void *send_data, size_t send_len)
{
    bool ext;
    uint32_t id;
    uint8_t data[8];
    uint8_t len;

    serializer_t ser;
    cmp_ctx_t ctx;
    serializer_init(&ser, (char *)send_data, send_len);
    serializer_cmp_ctx_factory(&ctx, &ser);
    if (!can_frame_cmp_read(&ctx, &ext, &id, data, &len)) {
        debug("can_frame_cmp_read error\n");
    }

    debug("can rcv: ext %u, id %x, len %u, \"", ext, id, len);
    int i;
    for (i = 0; i < len; i++) {
        debug("%c", data[i]);
    }
    debug("\"\n");
}

static bool first = true;
void mock_master_receiver(const void *data, size_t len)
{
    static serial_datagram_rcv_handler_t rcv;
    static uint8_t mock_serial_write_buf[32];

    if (first) {
        first = false;
        serial_datagram_rcv_handler_init(
            &rcv,
            mock_serial_write_buf,
            sizeof(mock_serial_write_buf),
            mock_master_sd_rcv_cb);
    }

    if (serial_datagram_receive(&rcv, data, len) != SERIAL_DATAGRAM_RCV_NO_ERROR) {
        debug("serial_datagram_receive error\n");
    }
}

void serial_interface_write(void *arg, const void *data, size_t len)
{
    (void)arg;
    if (len > 0) {
        // // chSequentialStreamWrite((BaseSequentialStream*)arg, (const uint8_t*)data, len);
        // sdWrite((SerialDriver *)arg, data, len);
        mock_master_receiver(data, len);
    }
}

size_t mock_write_count;

void mock_write(void *arg, const void *data, size_t len)
{
    uint8_t *dest = (uint8_t *)arg + mock_write_count;
    uint8_t *src = (uint8_t *)data;
    mock_write_count += len;
    while (len--) {
        *dest++ = *src++;
    }
}

size_t serial_master_mock(void *buf)
{
    static char frame_buf[32];
    chThdSleepMilliseconds(500);
    serializer_t ser;
    cmp_ctx_t ctx;
    serializer_init(&ser, frame_buf, sizeof(frame_buf));
    serializer_cmp_ctx_factory(&ctx, &ser);
    cmp_write_uint(&ctx, 0);
    bool w = can_frame_cmp_write(&ctx, false, 42, "hi world", 8);
    if (!w) {
        debug("can frame write err\n");
    }

    size_t fb_len = serializer_written_bytes_count(&ser);

    mock_write_count = 0;
    serial_datagram_send(frame_buf, fb_len, mock_write, buf);

    return mock_write_count;
}

size_t serial_interface_read(void *arg, void *buf, size_t max)
{
    (void)arg;
    return serial_master_mock(buf);

    // // return chSequentialStreamRead((BaseSequentialStream*)arg, buf, max);

    // return sdReadTimeout((SerialDriver *)arg, buf, max, 10);
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

static const CANConfig can1_config = {
    .mcr = (1 << 6) | (1 << 2),

    // loopback, silent
    .btr = (1 << 0) | (11 << 16) | (7 << 20) | (0 << 24)  | (1 << 30) | (1 << 31)
};

static const CANConfig can2_config = {
    .mcr = (1 << 6) | (1 << 2),
    .btr = (1 << 0) | (11 << 16) | (7 << 20) | (0 << 24)
};

void run_bridge(void)
{
    BaseSequentialStream *serial = (BaseSequentialStream *) &UART_CONN2;
    sdStart(&UART_CONN2, NULL);

    canStart(&CAND1, &can1_config);
    canStart(&CAND2, &can2_config);

    board_can_standby(false);

    chThdCreateStatic(bridge_rx_wa, sizeof(bridge_rx_wa), NORMALPRIO, bridge_rx, serial);
    chThdCreateStatic(bridge_tx_wa, sizeof(bridge_tx_wa), NORMALPRIO, bridge_tx, serial);
}
