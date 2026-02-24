#ifndef BBB_UIO_SHARED_LAYOUT_H_
#define BBB_UIO_SHARED_LAYOUT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BBB_UIO_LAYOUT_MAGIC 0x42425549u
#define BBB_UIO_LAYOUT_VERSION 1u
#define BBB_UIO_SHARED_OFFSET 0x00010000u

#define BBB_UIO_HOST_STATUS_LAYOUT_OK (1u << 0)
#define BBB_UIO_HOST_STATUS_ARMED (1u << 1)
#define BBB_UIO_HOST_STATUS_ESTOP (1u << 2)

#define BBB_UIO_PRU0_STATUS_READY (1u << 0)
#define BBB_UIO_PRU0_STATUS_ENCODER_ACTIVE (1u << 1)

#define BBB_UIO_PRU1_STATUS_READY (1u << 0)
#define BBB_UIO_PRU1_STATUS_MOTOR_ACTIVE (1u << 1)

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t size;

    uint32_t host_status_bits;
    uint32_t pru0_status_bits;
    uint32_t pru1_status_bits;

    uint32_t host_heartbeat;
    uint32_t pru0_heartbeat;
    uint32_t pru1_heartbeat;

    uint32_t host_time_us;
    uint32_t pru0_time_us;
    uint32_t pru1_time_us;

    uint32_t encoder_timestamp_us;
    int32_t encoder_counts[4];
    int32_t encoder_velocity_tps[4];

    uint32_t encoder_sample_period_us;
    uint32_t encoder_velocity_interval_us;
    uint32_t encoder_stream_hz;
    uint32_t encoder_reset_token;

    int16_t motor_command_left;
    int16_t motor_command_right;
    int16_t motor_applied_left;
    int16_t motor_applied_right;

    uint32_t motor_command_seq;
    uint32_t motor_command_time_us;
    uint32_t motor_watchdog_ms;
    uint16_t motor_safe_stop_ms;
    uint16_t motor_ramp_step;
    uint32_t motor_control_period_us;

    uint32_t sabertooth_baud;
    uint8_t sabertooth_mode;
    uint8_t sabertooth_address;
    uint8_t sabertooth_tx_bit;
    uint8_t estop_asserted;
    uint8_t armed;
    uint8_t reserved_u8[3];

    uint32_t reserved_u32[16];
} bbb_uio_shared_layout_t;

#ifdef __cplusplus
}
#endif

#endif
