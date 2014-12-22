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

void serial_send_to_master_mock(const uint8_t *data, size_t len)
{
    struct can_bridge_frame frame;
    serializer_t ser;
    cmp_ctx_t ctx;
    serializer_init(&ser, (char *)data, len);
    serializer_cmp_ctx_factory(&ctx, &ser);
    if (!can_frame_cmp_read(&ctx, &frame)) {
        debug("can_frame_cmp_read error\n");
    }

    debug("can rcv: ext %u, id %x, len %u, %08x, %08x\n", frame.ext,
            frame.ext ? frame.id.ext : frame.id.std, frame.dlc,
            frame.data.u32[0], frame.data.u32[1]);
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

size_t serial_receive_from_master_mock(void *buf)
{
    static uint8_t id = 0;
    id++;
    static char frame_buf[32];
    chThdSleepMilliseconds(500);
    serializer_t ser;
    cmp_ctx_t ctx;
    serializer_init(&ser, frame_buf, sizeof(frame_buf));
    serializer_cmp_ctx_factory(&ctx, &ser);
    cmp_write_uint(&ctx, 0);
    struct can_bridge_frame frame = {
        .ext = 0, .rtr = 0, .id.std = id, .data.u8 = {'x','k','c','d'}, .dlc = 4
    };
    bool w = can_frame_cmp_write(&ctx, &frame);
    if (!w) {
        debug("can frame write err\n");
    }

    size_t fb_len = serializer_written_bytes_count(&ser);

    mock_write_count = 0;
    serial_datagram_send(frame_buf, fb_len, mock_write, buf);

    return mock_write_count;
}

void can_interface_send(bool ext, bool rtr, uint32_t id, void *data, uint8_t len)
{
    canmbx_t mailbox = 0;
    systime_t timeout = TIME_INFINITE;

    CANTxFrame ctf;

    ctf.DLC = len;

    if (rtr) {
        ctf.RTR = 1;
    } else {
        ctf.RTR = 0;
    }

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

void can_interface_receive(struct can_bridge_frame *frame)
{
    CANRxFrame crf;
    canmbx_t mailbox = 0;
    systime_t timeout = TIME_INFINITE;

    canReceive(&CAND2, mailbox, &crf, timeout);

    frame->rtr = crf.RTR;
    frame->ext = crf.IDE;
    frame->dlc = crf.DLC;

    if (crf.IDE == 1) {
        frame->id.ext = crf.EID;
    } else {
        frame->id.std = crf.SID;
    }

    frame->data.u32[0] = crf.data32[0];
    frame->data.u32[1] = crf.data32[1];
}

static THD_WORKING_AREA(bridge_rx_wa, 512);
static THD_FUNCTION(bridge_rx, arg)
{
    static char datagram_buf[32];
    static char serial_buf[42];
    serial_datagram_rcv_handler_t rcv;

    SerialDriver *sd = (SerialDriver *)arg;

    serial_datagram_rcv_handler_init(
        &rcv,
        datagram_buf,
        sizeof(datagram_buf),
        can_bridge_datagram_rcv_cb);

    while (1) {
        int err;
        size_t len;

        len = sdReadTimeout(sd, (uint8_t *) serial_buf, sizeof(serial_buf), 10);

        if (len > 0) {
            err = serial_datagram_receive(&rcv, serial_buf, len);

            if (err != SERIAL_DATAGRAM_RCV_NO_ERROR) {
                debug("serial datagram error: %d", err);
            }
        }
    }
    return 0;
}

void serial_write(void *arg, const void *p, size_t len)
{
    SerialDriver *sd = (SerialDriver *)arg;
    // chSequentialStreamWrite((BaseSequentialStream*)arg, (const uint8_t*)data, len);
    if (len != 0) {
        sdWrite(sd, p, len);
    }
}

static THD_WORKING_AREA(bridge_tx_wa, 512);
static THD_FUNCTION(bridge_tx, arg)
{

    struct can_bridge_frame frame;

    static uint8_t outbuf[32];
    size_t outlen;

    while (1) {
        can_interface_receive(&frame);

        outlen = sizeof(outbuf);
        if (can_bridge_frame_write(&frame, outbuf, &outlen)) {
            serial_datagram_send(outbuf, outlen, serial_write, arg);
        } else {
            debug("failed to encode received CAN frame\n");
        }
    }

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
    debug("bridge\n");
    BaseSequentialStream *serial = (BaseSequentialStream *) &UART_CONN2;
    sdStart(&UART_CONN2, NULL);

    canStart(&CAND1, &can1_config);
    canStart(&CAND2, &can2_config);

    board_can_standby(false);

    debug("can initialized\n");

    chThdCreateStatic(bridge_rx_wa, sizeof(bridge_rx_wa), NORMALPRIO, bridge_rx, serial);
    chThdCreateStatic(bridge_tx_wa, sizeof(bridge_tx_wa), NORMALPRIO, bridge_tx, serial);
}
