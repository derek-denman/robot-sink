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

volatile register uint32_t __R30;
volatile register uint32_t __R31;

#define CHAN_NAME "rpmsg-pru"
#define CHAN_DESC "bbb-pru1-sabertooth"
#define CHAN_PORT 31

#define HOST_INT ((uint32_t)1 << 30)
#define TO_ARM_HOST 18
#define FROM_ARM_HOST 19

#define PRU_CLOCK_HZ 200000000u
#define MAIN_LOOP_US 1000u
#define MAIN_LOOP_DELAY_CYCLES ((PRU_CLOCK_HZ / 1000000u) * MAIN_LOOP_US)

#define MOTOR_CMD_MAX 1000
#define DEFAULT_BAUD 38400u
#define DEFAULT_ADDR 128u
#define DEFAULT_TX_BIT 0u
#define DEFAULT_WATCHDOG_MS 200u
#define DEFAULT_SAFE_STOP_MS 250u
#define DEFAULT_RAMP_STEP 40u
#define DEFAULT_CONTROL_PERIOD_US 10000u

#define RPMSG_VRING_SIZE 16
#define RPMSG_NOTIFYID_VDEV 0u
#define RPMSG_NOTIFYID_VRING0 ((uint32_t)TO_ARM_HOST)
#define RPMSG_NOTIFYID_VRING1 ((uint32_t)FROM_ARM_HOST)

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

static struct pru_rpmsg_transport g_transport;
static uint16_t g_last_src;
static uint16_t g_last_dst;
static uint8_t g_has_host_peer;
typedef uint16_t rpmsg_len_t;

static uint32_t g_now_us;
static uint32_t g_last_control_us;
static uint32_t g_last_motor_cmd_us;
static uint32_t g_stop_hold_until_us;

static uint32_t g_baud = DEFAULT_BAUD;
static uint8_t g_mode = BBB_SABER_MODE_PACKETIZED;
static uint8_t g_address = DEFAULT_ADDR;
static uint8_t g_tx_bit = DEFAULT_TX_BIT;
static uint16_t g_watchdog_ms = DEFAULT_WATCHDOG_MS;
static uint16_t g_safe_stop_ms = DEFAULT_SAFE_STOP_MS;
static uint16_t g_ramp_step = DEFAULT_RAMP_STEP;
static uint16_t g_control_period_us = DEFAULT_CONTROL_PERIOD_US;
static uint32_t g_uart_bit_cycles;

static int16_t g_target_left;
static int16_t g_target_right;
static int16_t g_current_left;
static int16_t g_current_right;
static uint8_t g_estop_asserted = 1u;

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

static int16_t clamp_motor(int32_t value)
{
    if (value > MOTOR_CMD_MAX) {
        return MOTOR_CMD_MAX;
    }

    if (value < -MOTOR_CMD_MAX) {
        return -MOTOR_CMD_MAX;
    }

    return (int16_t)value;
}

static void recompute_uart_timing(void)
{
    if (g_baud < 1200u) {
        g_baud = 1200u;
    }

    if (g_baud > 115200u) {
        g_baud = 115200u;
    }

    g_uart_bit_cycles = PRU_CLOCK_HZ / g_baud;
    if (g_uart_bit_cycles < 50u) {
        g_uart_bit_cycles = 50u;
    }
}

static void tx_set_level(uint8_t high)
{
    uint32_t mask = (uint32_t)1u << g_tx_bit;

    if (high != 0u) {
        __R30 |= mask;
    } else {
        __R30 &= ~mask;
    }
}

static void tx_send_byte(uint8_t value)
{
    uint8_t bit;

    tx_set_level(0u);
    delay_cycles(g_uart_bit_cycles);

    for (bit = 0u; bit < 8u; ++bit) {
        tx_set_level((uint8_t)(value & 0x01u));
        delay_cycles(g_uart_bit_cycles);
        value >>= 1u;
    }

    tx_set_level(1u);
    delay_cycles(g_uart_bit_cycles);
}

static uint8_t to_7bit_mag(int16_t command)
{
    int32_t mag = command;
    if (mag < 0) {
        mag = -mag;
    }

    return (uint8_t)((mag * 127) / MOTOR_CMD_MAX);
}

static void send_packetized_motor(uint8_t cmd_forward,
                                  uint8_t cmd_reverse,
                                  int16_t command)
{
    uint8_t cmd;
    uint8_t data = to_7bit_mag(command);
    uint8_t checksum;

    cmd = (command >= 0) ? cmd_forward : cmd_reverse;
    checksum = (uint8_t)((g_address + cmd + data) & 0x7Fu);

    tx_send_byte(g_address);
    tx_send_byte(cmd);
    tx_send_byte(data);
    tx_send_byte(checksum);
}

static uint8_t map_simple_m1(int16_t command)
{
    int32_t value = 64 + ((int32_t)command * 63) / MOTOR_CMD_MAX;

    if (value < 1) {
        value = 1;
    }

    if (value > 127) {
        value = 127;
    }

    return (uint8_t)value;
}

static uint8_t map_simple_m2(int16_t command)
{
    int32_t value = 192 + ((int32_t)command * 63) / MOTOR_CMD_MAX;

    if (value < 128) {
        value = 128;
    }

    if (value > 255) {
        value = 255;
    }

    return (uint8_t)value;
}

static void send_drive_command(int16_t left, int16_t right)
{
    if (g_mode == BBB_SABER_MODE_SIMPLIFIED) {
        tx_send_byte(map_simple_m1(left));
        tx_send_byte(map_simple_m2(right));
        return;
    }

    send_packetized_motor(0u, 1u, left);
    send_packetized_motor(4u, 5u, right);
}

static void send_neutral(void)
{
    send_drive_command(0, 0);
}

static void apply_ramp(void)
{
    if (g_ramp_step == 0u) {
        g_current_left = g_target_left;
        g_current_right = g_target_right;
        return;
    }

    if (g_current_left < (g_target_left - (int16_t)g_ramp_step)) {
        g_current_left += (int16_t)g_ramp_step;
    } else if (g_current_left > (g_target_left + (int16_t)g_ramp_step)) {
        g_current_left -= (int16_t)g_ramp_step;
    } else {
        g_current_left = g_target_left;
    }

    if (g_current_right < (g_target_right - (int16_t)g_ramp_step)) {
        g_current_right += (int16_t)g_ramp_step;
    } else if (g_current_right > (g_target_right + (int16_t)g_ramp_step)) {
        g_current_right -= (int16_t)g_ramp_step;
    } else {
        g_current_right = g_target_right;
    }
}

static void enter_estop(void)
{
    g_estop_asserted = 1u;
    g_target_left = 0;
    g_target_right = 0;
    g_current_left = 0;
    g_current_right = 0;
    g_stop_hold_until_us = g_now_us + ((uint32_t)g_safe_stop_ms * 1000u);
    send_neutral();
}

static void try_release_estop(uint8_t *released)
{
    *released = 0u;
    if (g_now_us < g_stop_hold_until_us) {
        return;
    }

    g_estop_asserted = 0u;
    g_last_motor_cmd_us = g_now_us;
    *released = 1u;
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
                     BBB_MODULE_PRU1,
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
                     BBB_MODULE_PRU1,
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
    case BBB_CMD_SET_MOTOR:
        if (msg->payload_len < sizeof(bbb_motor_cmd_t)) {
            send_nack(msg->sequence, 3u);
            break;
        }

        if (g_estop_asserted != 0u) {
            send_nack(msg->sequence, 4u);
            break;
        }

        {
            bbb_motor_cmd_t cmd;
            memcpy(&cmd, msg->payload, sizeof(cmd));
            g_target_left = clamp_motor(cmd.left);
            g_target_right = clamp_motor(cmd.right);
            g_last_motor_cmd_us = g_now_us;
            send_ack(msg->sequence, 0, 0u);
        }
        break;

    case BBB_CMD_SET_ESTOP:
        if (msg->payload_len < sizeof(bbb_estop_cmd_t)) {
            send_nack(msg->sequence, 5u);
            break;
        }

        {
            bbb_estop_cmd_t estop;
            uint8_t released;
            memcpy(&estop, msg->payload, sizeof(estop));

            if (estop.asserted != 0u) {
                enter_estop();
                send_ack(msg->sequence, 0, 1u);
                break;
            }

            try_release_estop(&released);
            if (released == 0u) {
                send_nack(msg->sequence, 6u);
            } else {
                send_ack(msg->sequence, 0, 0u);
            }
        }
        break;

    case BBB_CMD_SET_SABER_PARAMS:
        if (msg->payload_len < sizeof(bbb_saber_params_t)) {
            send_nack(msg->sequence, 7u);
            break;
        }

        {
            bbb_saber_params_t params;
            memcpy(&params, msg->payload, sizeof(params));

            g_baud = params.baud;
            g_mode = (params.mode == BBB_SABER_MODE_SIMPLIFIED)
                         ? BBB_SABER_MODE_SIMPLIFIED
                         : BBB_SABER_MODE_PACKETIZED;
            g_address = params.address;
            g_tx_bit = params.tx_r30_bit & 0x1Fu;
            g_watchdog_ms = params.watchdog_ms;
            g_safe_stop_ms = params.safe_stop_hold_ms;
            g_ramp_step = params.ramp_step;
            g_control_period_us = params.control_period_us;

            if (g_watchdog_ms == 0u) {
                g_watchdog_ms = DEFAULT_WATCHDOG_MS;
            }

            if (g_safe_stop_ms == 0u) {
                g_safe_stop_ms = DEFAULT_SAFE_STOP_MS;
            }

            if (g_control_period_us < 2000u) {
                g_control_period_us = 2000u;
            }

            recompute_uart_timing();
            tx_set_level(1u);
            send_ack(msg->sequence, 0, 0u);
        }
        break;

    case BBB_CMD_PING:
        send_ack(msg->sequence, 0, 0u);
        break;

    default:
        send_nack(msg->sequence, 8u);
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

static void control_step(void)
{
    uint32_t watchdog_us = (uint32_t)g_watchdog_ms * 1000u;

    if (g_estop_asserted != 0u) {
        send_neutral();
        return;
    }

    if ((g_now_us - g_last_motor_cmd_us) > watchdog_us) {
        enter_estop();
        return;
    }

    apply_ramp();
    send_drive_command(g_current_left, g_current_right);
}

void main(void)
{
    volatile uint8_t *status;

    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

    recompute_uart_timing();
    tx_set_level(1u);

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

    enter_estop();

    while (1) {
        if ((__R31 & HOST_INT) != 0u) {
            process_rpmsg();
            CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;
        }

        if ((g_now_us - g_last_control_us) >= g_control_period_us) {
            g_last_control_us = g_now_us;
            control_step();
        }

        __delay_cycles(MAIN_LOOP_DELAY_CYCLES);
        g_now_us += MAIN_LOOP_US;
    }
}
