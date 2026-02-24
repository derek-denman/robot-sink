#include <stdint.h>

#include <pru_cfg.h>
#include <pru_ctrl.h>

#include "../common/uio_shared_layout.h"

volatile register uint32_t __R31;

#define PRU_CLOCK_HZ 200000000u
#define DEFAULT_LOOP_PERIOD_US 20u
#define DEFAULT_VELOCITY_INTERVAL_US 20000u

#ifndef PRU_SRAM
#define PRU_SRAM __far __attribute__((cregister("PRU_SHAREDMEM", near)))
#endif

PRU_SRAM volatile bbb_uio_shared_layout_t g_shared;

static const int8_t k_quad_delta[16] = {
    0, 1, -1, 0,
    -1, 0, 0, 1,
    1, 0, 0, -1,
    0, -1, 1, 0,
};

static int32_t g_counts[4];
static int32_t g_velocity_tps[4];
static int32_t g_prev_vel_counts[4];
static uint8_t g_prev_state[4];

static uint32_t g_now_us;
static uint32_t g_last_velocity_us;
static uint32_t g_loop_period_us = DEFAULT_LOOP_PERIOD_US;
static uint32_t g_loop_delay_cycles =
    (PRU_CLOCK_HZ / 1000000u) * DEFAULT_LOOP_PERIOD_US;
static uint32_t g_velocity_interval_us = DEFAULT_VELOCITY_INTERVAL_US;
static uint32_t g_last_reset_token;

static void delay_cycles(uint32_t cycles)
{
    while (cycles-- > 0u) {
        __asm__(" NOP");
    }
}

static uint8_t layout_valid(void)
{
    return (g_shared.magic == BBB_UIO_LAYOUT_MAGIC) &&
           (g_shared.version == BBB_UIO_LAYOUT_VERSION) &&
           (g_shared.size == (uint16_t)sizeof(bbb_uio_shared_layout_t));
}

static void reset_counts(void)
{
    uint8_t i;
    for (i = 0u; i < 4u; ++i) {
        g_counts[i] = 0;
        g_velocity_tps[i] = 0;
        g_prev_vel_counts[i] = 0;
    }
    g_last_velocity_us = g_now_us;
}

static void sync_host_encoder_config(void)
{
    uint32_t sample_period_us = g_shared.encoder_sample_period_us;
    uint32_t velocity_interval_us = g_shared.encoder_velocity_interval_us;
    uint32_t reset_token = g_shared.encoder_reset_token;

    if (sample_period_us < 5u) {
        sample_period_us = 5u;
    }
    if (sample_period_us > 2000u) {
        sample_period_us = 2000u;
    }

    if (sample_period_us != g_loop_period_us) {
        g_loop_period_us = sample_period_us;
        g_loop_delay_cycles = (PRU_CLOCK_HZ / 1000000u) * g_loop_period_us;
    }

    if (velocity_interval_us < 1000u) {
        velocity_interval_us = 1000u;
    }
    if (velocity_interval_us > 1000000u) {
        velocity_interval_us = 1000000u;
    }
    g_velocity_interval_us = velocity_interval_us;

    if (reset_token != g_last_reset_token) {
        g_last_reset_token = reset_token;
        reset_counts();
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

static void publish_snapshot(void)
{
    uint8_t i;

    g_shared.pru0_heartbeat = g_shared.pru0_heartbeat + 1u;
    g_shared.pru0_time_us = g_now_us;

    g_shared.encoder_timestamp_us = g_now_us;
    for (i = 0u; i < 4u; ++i) {
        g_shared.encoder_counts[i] = g_counts[i];
        g_shared.encoder_velocity_tps[i] = g_velocity_tps[i];
    }

    g_shared.pru0_status_bits =
        BBB_UIO_PRU0_STATUS_READY | BBB_UIO_PRU0_STATUS_ENCODER_ACTIVE;
}

void main(void)
{
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;
    PRU0_CTRL.CTPPR0_bit.C28_BLK_POINTER = 0x0100;

    {
        uint8_t wheel;
        uint8_t initial_pins = (uint8_t)(__R31 & 0xFFu);
        for (wheel = 0u; wheel < 4u; ++wheel) {
            g_prev_state[wheel] =
                (uint8_t)((initial_pins >> (wheel * 2u)) & 0x03u);
        }
    }

    while (1) {
        update_quadrature((uint8_t)(__R31 & 0xFFu));

        if (layout_valid() != 0u) {
            sync_host_encoder_config();
            update_velocity_if_due();
            publish_snapshot();
        }

        delay_cycles(g_loop_delay_cycles);
        g_now_us += g_loop_period_us;
    }
}
