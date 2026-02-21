#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <pru_cfg.h>
#include <pru_intc.h>
#include <pru_rpmsg.h>
#include <rsc_types.h>

#include "../common/rpmsg_protocol.h"

#ifndef VIRTIO_ID_RPMSG
#define VIRTIO_ID_RPMSG 7
#endif

#ifndef VIRTIO_RPMSG_F_NS
#define VIRTIO_RPMSG_F_NS 0
#endif

#ifndef VIRTIO_CONFIG_S_DRIVER_OK
#define VIRTIO_CONFIG_S_DRIVER_OK 4
#endif

#if !defined(BBB_RPMSG_CHANNEL_WITH_DESC) && !defined(BBB_RPMSG_CHANNEL_NO_DESC)
#if defined(__TI_COMPILER_VERSION__)
#define BBB_RPMSG_CHANNEL_NO_DESC 1
#else
#define BBB_RPMSG_CHANNEL_WITH_DESC 1
#endif
#endif

volatile register uint32_t __R31;

#define CHAN_NAME "rpmsg-pru"
#define CHAN_DESC "bbb-pru0-enc"
#define CHAN_PORT 30

#define HOST_INT ((uint32_t)1 << 30)
#define TO_ARM_HOST 16
#define FROM_ARM_HOST 17

#define PRU_CLOCK_HZ 200000000u
#define DEFAULT_LOOP_PERIOD_US 20u

#define RPMSG_VRING_SIZE 16
#define RPMSG_NOTIFYID_VDEV 0u
#define RPMSG_NOTIFYID_VRING0 1u
#define RPMSG_NOTIFYID_VRING1 2u

struct my_resource_table {
    struct resource_table base;
    uint32_t offset[1];
    struct fw_rsc_vdev rpmsg_vdev;
    struct fw_rsc_vdev_vring rpmsg_vring0;
    struct fw_rsc_vdev_vring rpmsg_vring1;
};

#pragma DATA_SECTION(resourceTable, ".resource_table")
#pragma RETAIN(resourceTable)
struct my_resource_table resourceTable = {
    .base = {
        .ver = 1,
        .num = 1,
        .reserved = {0, 0},
    },
    .offset = {
        offsetof(struct my_resource_table, rpmsg_vdev),
    },
    .rpmsg_vdev = {
        .type = TYPE_VDEV,
        .id = VIRTIO_ID_RPMSG,
        .notifyid = RPMSG_NOTIFYID_VDEV,
        .dfeatures = 1 << VIRTIO_RPMSG_F_NS,
        .gfeatures = 0,
        .config_len = 0,
        .status = 0,
        .num_of_vrings = 2,
        .reserved = {0, 0},
    },
    .rpmsg_vring0 = {
        .da = FW_RSC_ADDR_ANY,
        .align = 16,
        .num = RPMSG_VRING_SIZE,
        .notifyid = RPMSG_NOTIFYID_VRING0,
        .reserved = 0,
    },
    .rpmsg_vring1 = {
        .da = FW_RSC_ADDR_ANY,
        .align = 16,
        .num = RPMSG_VRING_SIZE,
        .notifyid = RPMSG_NOTIFYID_VRING1,
        .reserved = 0,
    },
};

static const int8_t k_quad_delta[16] = {
    0, 1, -1, 0,
    -1, 0, 0, 1,
    1, 0, 0, -1,
    0, -1, 1, 0,
};

static struct pru_rpmsg_transport g_transport;
static uint16_t g_last_src;
static uint16_t g_last_dst;
static uint8_t g_has_host_peer;
typedef uint16_t rpmsg_len_t;

static int32_t g_counts[4];
static int32_t g_velocity_tps[4];
static int32_t g_prev_vel_counts[4];
static uint8_t g_prev_state[4];

static uint32_t g_now_us;
static uint32_t g_last_velocity_us;
static uint32_t g_last_stream_us;
static uint32_t g_loop_period_us = DEFAULT_LOOP_PERIOD_US;
static uint32_t g_loop_delay_cycles =
    (PRU_CLOCK_HZ / 1000000u) * DEFAULT_LOOP_PERIOD_US;
static uint32_t g_velocity_interval_us = 20000u;
static uint16_t g_stream_hz = 0u;

static uint16_t g_sequence;

static int16_t rpmsg_create_channel(struct pru_rpmsg_transport *transport)
{
#if defined(BBB_RPMSG_CHANNEL_WITH_DESC)
    return pru_rpmsg_channel(RPMSG_NS_CREATE,
                             transport,
                             (char *)CHAN_NAME,
                             (char *)CHAN_DESC,
                             (int32_t)CHAN_PORT);
#else
    return pru_rpmsg_channel(RPMSG_NS_CREATE,
                             transport,
                             (char *)CHAN_NAME,
                             (int32_t)CHAN_PORT);
#endif
}

static void delay_cycles(uint32_t cycles)
{
    while (cycles-- > 0u) {
        __asm__(" NOP");
    }
}

static uint32_t hz_to_period_us(uint16_t hz)
{
    if (hz == 0u) {
        return 0u;
    }

    return (uint32_t)(1000000u / hz);
}

static void fill_snapshot(bbb_enc_snapshot_t *snapshot)
{
    uint8_t i;
    for (i = 0; i < 4; ++i) {
        snapshot->counts[i] = g_counts[i];
        snapshot->velocity_tps[i] = g_velocity_tps[i];
    }
    snapshot->timestamp_us = g_now_us;
    snapshot->stream_hz = g_stream_hz;
    snapshot->flags = 0u;
}

static void send_message(const bbb_msg_t *msg)
{
    if (!g_has_host_peer) {
        return;
    }

    (void)pru_rpmsg_send(&g_transport,
                         g_last_dst,
                         g_last_src,
                         (void *)msg,
                         (uint16_t)BBB_MSG_WIRE_SIZE);
}

static void send_ack(uint16_t request_sequence, int16_t status, uint16_t detail)
{
    bbb_msg_t out;
    bbb_ack_payload_t ack;

    bbb_init_message(&out,
                     BBB_MODULE_PRU0,
                     BBB_MSG_TYPE_RESPONSE,
                     BBB_CMD_ACK,
                     request_sequence);

    ack.status = status;
    ack.detail = detail;
    ack.timestamp_us = g_now_us;

    memcpy(out.payload, &ack, sizeof(ack));
    out.payload_len = sizeof(ack);
    bbb_finalize_message(&out);
    send_message(&out);
}

static void send_nack(uint16_t request_sequence, uint16_t detail)
{
    bbb_msg_t out;
    bbb_ack_payload_t nack;

    bbb_init_message(&out,
                     BBB_MODULE_PRU0,
                     BBB_MSG_TYPE_RESPONSE,
                     BBB_CMD_NACK,
                     request_sequence);

    nack.status = -1;
    nack.detail = detail;
    nack.timestamp_us = g_now_us;

    memcpy(out.payload, &nack, sizeof(nack));
    out.payload_len = sizeof(nack);
    bbb_finalize_message(&out);
    send_message(&out);
}

static void send_snapshot(uint8_t type, uint16_t sequence)
{
    bbb_msg_t out;
    bbb_enc_snapshot_t snapshot;

    fill_snapshot(&snapshot);
    bbb_init_message(&out,
                     BBB_MODULE_PRU0,
                     type,
                     BBB_CMD_GET_SNAPSHOT,
                     sequence);

    memcpy(out.payload, &snapshot, sizeof(snapshot));
    out.payload_len = sizeof(snapshot);
    bbb_finalize_message(&out);
    send_message(&out);
}

static void reset_counts(void)
{
    uint8_t i;
    for (i = 0; i < 4; ++i) {
        g_counts[i] = 0;
        g_velocity_tps[i] = 0;
        g_prev_vel_counts[i] = 0;
    }
    g_last_velocity_us = g_now_us;
}

static void handle_message(const bbb_msg_t *msg)
{
    if (!bbb_validate_message(msg)) {
        send_nack(msg->sequence, 1u);
        return;
    }

    if (msg->type != BBB_MSG_TYPE_COMMAND) {
        send_nack(msg->sequence, 2u);
        return;
    }

    switch (msg->command) {
    case BBB_CMD_RESET_COUNTS:
        reset_counts();
        send_ack(msg->sequence, 0, 0u);
        break;

    case BBB_CMD_GET_SNAPSHOT:
        send_snapshot(BBB_MSG_TYPE_RESPONSE, msg->sequence);
        break;

    case BBB_CMD_SET_ENC_PARAMS:
        if (msg->payload_len < sizeof(bbb_enc_params_t)) {
            send_nack(msg->sequence, 3u);
            break;
        }

        {
            bbb_enc_params_t params;
            memcpy(&params, msg->payload, sizeof(params));

            if (params.velocity_interval_ms == 0u) {
                params.velocity_interval_ms = 1u;
            }

            if (params.velocity_interval_ms > 1000u) {
                params.velocity_interval_ms = 1000u;
            }

            g_velocity_interval_us =
                (uint32_t)params.velocity_interval_ms * 1000u;
            g_stream_hz = params.stream_hz;
            if (params.sample_period_us < 5u) {
                params.sample_period_us = 5u;
            }
            if (params.sample_period_us > 2000u) {
                params.sample_period_us = 2000u;
            }
            g_loop_period_us = params.sample_period_us;
            g_loop_delay_cycles =
                (PRU_CLOCK_HZ / 1000000u) * g_loop_period_us;
            g_last_velocity_us = g_now_us;
            send_ack(msg->sequence, 0, 0u);
        }
        break;

    case BBB_CMD_SET_STREAM:
        if (msg->payload_len < sizeof(uint16_t)) {
            send_nack(msg->sequence, 4u);
            break;
        }

        memcpy(&g_stream_hz, msg->payload, sizeof(uint16_t));
        g_last_stream_us = g_now_us;
        send_ack(msg->sequence, 0, 0u);
        break;

    case BBB_CMD_PING:
        send_ack(msg->sequence, 0, 0u);
        break;

    default:
        send_nack(msg->sequence, 5u);
        break;
    }
}

static void process_rpmsg(void)
{
    uint16_t src;
    uint16_t dst;
    rpmsg_len_t len = 0u;
    uint8_t payload[BBB_MSG_WIRE_SIZE];

    while (pru_rpmsg_receive(&g_transport,
                             &src,
                             &dst,
                             payload,
                             &len) == PRU_RPMSG_SUCCESS) {
        bbb_msg_t msg;

        if (len < (rpmsg_len_t)BBB_MSG_WIRE_SIZE) {
            continue;
        }

        memset(&msg, 0, sizeof(msg));
        memcpy(&msg, payload, BBB_MSG_WIRE_SIZE);
        g_last_src = src;
        g_last_dst = dst;
        g_has_host_peer = 1u;
        handle_message(&msg);
    }
}

static void update_quadrature(uint8_t pins)
{
    uint8_t wheel;
    for (wheel = 0u; wheel < 4u; ++wheel) {
        uint8_t state = (uint8_t)((pins >> (wheel * 2u)) & 0x03u);
        uint8_t index = (uint8_t)((g_prev_state[wheel] << 2u) | state);
        int8_t delta = k_quad_delta[index];

        if (delta != 0) {
            g_counts[wheel] += (int32_t)delta;
        }

        g_prev_state[wheel] = state;
    }
}

static void update_velocity_if_due(void)
{
    uint32_t elapsed_us = g_now_us - g_last_velocity_us;

    if (elapsed_us < g_velocity_interval_us) {
        return;
    }

    {
        uint8_t wheel;
        for (wheel = 0u; wheel < 4u; ++wheel) {
            int32_t delta = g_counts[wheel] - g_prev_vel_counts[wheel];
            int64_t scaled = ((int64_t)delta * 1000000LL) / (int64_t)elapsed_us;
            g_velocity_tps[wheel] = (int32_t)scaled;
            g_prev_vel_counts[wheel] = g_counts[wheel];
        }
    }

    g_last_velocity_us = g_now_us;
}

static void stream_snapshot_if_due(void)
{
    uint32_t stream_period_us;

    if (g_stream_hz == 0u || !g_has_host_peer) {
        return;
    }

    stream_period_us = hz_to_period_us(g_stream_hz);
    if (stream_period_us == 0u) {
        return;
    }

    if ((g_now_us - g_last_stream_us) < stream_period_us) {
        return;
    }

    g_last_stream_us = g_now_us;
    send_snapshot(BBB_MSG_TYPE_STREAM, g_sequence++);
}

void main(void)
{
    volatile uint8_t *status;

    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

    {
        uint8_t wheel;
        uint8_t initial_pins = (uint8_t)(__R31 & 0xFFu);
        for (wheel = 0u; wheel < 4u; ++wheel) {
            g_prev_state[wheel] =
                (uint8_t)((initial_pins >> (wheel * 2u)) & 0x03u);
        }
    }

    status = &resourceTable.rpmsg_vdev.status;
    while (((*status) & VIRTIO_CONFIG_S_DRIVER_OK) == 0u) {
    }

    pru_rpmsg_init(&g_transport,
                   &resourceTable.rpmsg_vring0,
                   &resourceTable.rpmsg_vring1,
                   TO_ARM_HOST,
                   FROM_ARM_HOST);

    while (rpmsg_create_channel(&g_transport) != PRU_RPMSG_SUCCESS) {
    }

    while (1) {
        update_quadrature((uint8_t)(__R31 & 0xFFu));
        update_velocity_if_due();
        stream_snapshot_if_due();

        if ((__R31 & HOST_INT) != 0u) {
            process_rpmsg();
            CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;
        }

        delay_cycles(g_loop_delay_cycles);
        g_now_us += g_loop_period_us;
    }
}
