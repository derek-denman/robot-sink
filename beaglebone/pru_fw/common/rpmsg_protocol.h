#ifndef BBB_RPMSG_PROTOCOL_H_
#define BBB_RPMSG_PROTOCOL_H_

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BBB_MSG_MAGIC 0x42424747u
#define BBB_MSG_VERSION 1u
#define BBB_MSG_PAYLOAD_MAX 48u
#define BBB_MSG_WIRE_SIZE 62u

#define BBB_MODULE_HOST 0u
#define BBB_MODULE_PRU0 1u
#define BBB_MODULE_PRU1 2u

#define BBB_MSG_TYPE_COMMAND 1u
#define BBB_MSG_TYPE_RESPONSE 2u
#define BBB_MSG_TYPE_STREAM 3u
#define BBB_MSG_TYPE_HEARTBEAT 4u

#define BBB_CMD_RESET_COUNTS 0x10u
#define BBB_CMD_GET_SNAPSHOT 0x11u
#define BBB_CMD_SET_ENC_PARAMS 0x12u
#define BBB_CMD_SET_STREAM 0x13u

#define BBB_CMD_SET_MOTOR 0x20u
#define BBB_CMD_SET_ESTOP 0x21u
#define BBB_CMD_PING 0x22u
#define BBB_CMD_SET_SABER_PARAMS 0x23u

#define BBB_CMD_ACK 0x7Eu
#define BBB_CMD_NACK 0x7Fu

enum {
    BBB_SABER_MODE_SIMPLIFIED = 0,
    BBB_SABER_MODE_PACKETIZED = 1,
};

#ifdef __TI_COMPILER_VERSION__
#define BBB_PACKED
#else
#define BBB_PACKED __attribute__((packed))
#endif

typedef struct BBB_PACKED {
    uint32_t magic;
    uint8_t version;
    uint8_t module;
    uint8_t type;
    uint8_t command;
    uint16_t sequence;
    uint16_t payload_len;
    uint8_t payload[BBB_MSG_PAYLOAD_MAX];
    uint16_t checksum;
} bbb_msg_t;

typedef struct BBB_PACKED {
    int32_t counts[4];
    int32_t velocity_tps[4];
    uint32_t timestamp_us;
    uint16_t stream_hz;
    uint16_t flags;
} bbb_enc_snapshot_t;

typedef struct BBB_PACKED {
    uint16_t sample_period_us;
    uint16_t velocity_interval_ms;
    uint16_t stream_hz;
    uint16_t reserved0;
} bbb_enc_params_t;

typedef struct BBB_PACKED {
    int16_t left;
    int16_t right;
} bbb_motor_cmd_t;

typedef struct BBB_PACKED {
    uint8_t asserted;
    uint8_t reserved[3];
} bbb_estop_cmd_t;

typedef struct BBB_PACKED {
    uint32_t baud;
    uint8_t mode;
    uint8_t address;
    uint8_t tx_r30_bit;
    uint8_t reserved0;
    uint16_t watchdog_ms;
    uint16_t safe_stop_hold_ms;
    uint16_t ramp_step;
    uint16_t control_period_us;
} bbb_saber_params_t;

typedef struct BBB_PACKED {
    int16_t status;
    uint16_t detail;
    uint32_t timestamp_us;
} bbb_ack_payload_t;

static inline uint16_t bbb_checksum16_bytes(const uint8_t *bytes, uint16_t len)
{
    uint32_t sum = 0;
    uint16_t i;

    for (i = 0; i < len; ++i) {
        sum += bytes[i];
    }

    sum = (sum & 0xFFFFu) + (sum >> 16);
    sum = (sum & 0xFFFFu) + (sum >> 16);
    return (uint16_t)~sum;
}

static inline uint16_t bbb_message_checksum(const bbb_msg_t *msg)
{
    return bbb_checksum16_bytes((const uint8_t *)msg,
                                (uint16_t)(BBB_MSG_WIRE_SIZE - sizeof(uint16_t)));
}

static inline void bbb_init_message(bbb_msg_t *msg,
                                    uint8_t module,
                                    uint8_t type,
                                    uint8_t command,
                                    uint16_t sequence)
{
    memset(msg, 0, sizeof(*msg));
    msg->magic = BBB_MSG_MAGIC;
    msg->version = BBB_MSG_VERSION;
    msg->module = module;
    msg->type = type;
    msg->command = command;
    msg->sequence = sequence;
}

static inline void bbb_finalize_message(bbb_msg_t *msg)
{
    msg->checksum = bbb_message_checksum(msg);
}

static inline uint8_t bbb_validate_message(const bbb_msg_t *msg)
{
    if (msg->magic != BBB_MSG_MAGIC || msg->version != BBB_MSG_VERSION) {
        return 0u;
    }

    if (msg->payload_len > BBB_MSG_PAYLOAD_MAX) {
        return 0u;
    }

    return (bbb_message_checksum(msg) == msg->checksum) ? 1u : 0u;
}

#undef BBB_PACKED

#ifdef __cplusplus
}
#endif

#endif
