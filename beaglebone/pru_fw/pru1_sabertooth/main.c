#include <stdint.h>

#include <pru_cfg.h>
#include <pru_ctrl.h>

#include "../common/uio_shared_layout.h"

volatile register uint32_t __R30;

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

#ifndef PRU_SRAM
#define PRU_SRAM __far __attribute__((cregister("PRU_SHAREDMEM", near)))
#endif

PRU_SRAM volatile bbb_uio_shared_layout_t g_shared;

static uint32_t g_now_us;
static uint32_t g_last_control_us;
static uint32_t g_last_motor_cmd_us;
static uint32_t g_stop_hold_until_us;

static uint32_t g_baud = DEFAULT_BAUD;
static uint8_t g_mode = 1u;
static uint8_t g_address = DEFAULT_ADDR;
static uint8_t g_tx_bit = DEFAULT_TX_BIT;
static uint16_t g_watchdog_ms = DEFAULT_WATCHDOG_MS;
static uint16_t g_safe_stop_ms = DEFAULT_SAFE_STOP_MS;
static uint16_t g_ramp_step = DEFAULT_RAMP_STEP;
static uint32_t g_control_period_us = DEFAULT_CONTROL_PERIOD_US;
static uint32_t g_uart_bit_cycles;

static int16_t g_target_left;
static int16_t g_target_right;
static int16_t g_current_left;
static int16_t g_current_right;
static uint8_t g_estop_asserted = 1u;
static uint32_t g_last_command_seq;

static uint8_t layout_valid(void)
{
    return (g_shared.magic == BBB_UIO_LAYOUT_MAGIC) &&
           (g_shared.version == BBB_UIO_LAYOUT_VERSION) &&
           (g_shared.size == (uint16_t)sizeof(bbb_uio_shared_layout_t));
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
    if (g_mode == 0u) {
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

static uint8_t try_release_estop(void)
{
    if (g_now_us < g_stop_hold_until_us) {
        return 0u;
    }

    g_estop_asserted = 0u;
    g_last_motor_cmd_us = g_now_us;
    return 1u;
}

static void sync_host_config(void)
{
    uint32_t baud = g_shared.sabertooth_baud;
    uint8_t mode = g_shared.sabertooth_mode;
    uint8_t address = g_shared.sabertooth_address;
    uint8_t tx_bit = g_shared.sabertooth_tx_bit;
    uint32_t watchdog_ms = g_shared.motor_watchdog_ms;
    uint16_t safe_stop_ms = g_shared.motor_safe_stop_ms;
    uint16_t ramp_step = g_shared.motor_ramp_step;
    uint32_t control_period_us = g_shared.motor_control_period_us;

    if (baud == 0u) {
        baud = DEFAULT_BAUD;
    }
    if (watchdog_ms == 0u) {
        watchdog_ms = DEFAULT_WATCHDOG_MS;
    }
    if (safe_stop_ms == 0u) {
        safe_stop_ms = DEFAULT_SAFE_STOP_MS;
    }
    if (control_period_us < 2000u) {
        control_period_us = 2000u;
    }

    g_baud = baud;
    g_mode = (mode == 0u) ? 0u : 1u;
    g_address = address;
    g_tx_bit = tx_bit & 0x1Fu;
    g_watchdog_ms = (uint16_t)watchdog_ms;
    g_safe_stop_ms = safe_stop_ms;
    g_ramp_step = ramp_step;
    g_control_period_us = control_period_us;

    recompute_uart_timing();
}

static void consume_host_command(void)
{
    uint32_t command_seq;

    if ((g_shared.estop_asserted != 0u) || (g_shared.armed == 0u)) {
        if (g_estop_asserted == 0u) {
            enter_estop();
        }
        return;
    }

    if (g_estop_asserted != 0u) {
        if (try_release_estop() == 0u) {
            return;
        }
    }

    command_seq = g_shared.motor_command_seq;
    if (command_seq == g_last_command_seq) {
        return;
    }

    g_last_command_seq = command_seq;
    g_target_left = clamp_motor((int32_t)g_shared.motor_command_left);
    g_target_right = clamp_motor((int32_t)g_shared.motor_command_right);
    g_last_motor_cmd_us = g_now_us;
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

static void publish_status(void)
{
    uint32_t status = BBB_UIO_PRU1_STATUS_READY;

    if (g_estop_asserted == 0u) {
        status |= BBB_UIO_PRU1_STATUS_MOTOR_ACTIVE;
    }

    g_shared.motor_applied_left = g_current_left;
    g_shared.motor_applied_right = g_current_right;
    g_shared.pru1_status_bits = status;
    g_shared.pru1_time_us = g_now_us;
    g_shared.pru1_heartbeat = g_shared.pru1_heartbeat + 1u;
}

void main(void)
{
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;
    PRU1_CTRL.CTPPR0_bit.C28_BLK_POINTER = 0x0100;

    recompute_uart_timing();
    tx_set_level(1u);
    enter_estop();

    while (1) {
        if (layout_valid() != 0u) {
            sync_host_config();
            consume_host_command();

            if ((g_now_us - g_last_control_us) >= g_control_period_us) {
                g_last_control_us = g_now_us;
                control_step();
            }

            publish_status();
        } else {
            send_neutral();
        }

        delay_cycles(MAIN_LOOP_DELAY_CYCLES);
        g_now_us += MAIN_LOOP_US;
    }
}
