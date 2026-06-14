/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <dk_buttons_and_leds.h>
#include <esb.h>
#include <nrf.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#if defined(CONFIG_CLOCK_CONTROL_NRF2)
#include <hal/nrf_lrcconf.h>
#endif
#include <nrf_erratas.h>
#if NRF54L_ERRATA_20_PRESENT
#include <hal/nrf_power.h>
#endif /* NRF54L_ERRATA_20_PRESENT */
#if defined(NRF54LM20A_ENGA_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54LM20A_ENGA_XXAA) */

LOG_MODULE_REGISTER(esb_ptx, CONFIG_ESB_PTX_APP_LOG_LEVEL);

#define NUM_SAMPLES 16U
typedef struct {
    uint16_t left;
    uint16_t right;
} sample_t;

typedef struct {
    uint16_t sequence_number;
    sample_t samples[NUM_SAMPLES];
} audio_message_t;

static struct esb_payload tx_payload = {0};
static void sample_handler(struct k_timer* timer);
K_TIMER_DEFINE(sample_timer, sample_handler, NULL);
static K_SEM_DEFINE(tx_sem, CONFIG_ESB_TX_FIFO_SIZE, CONFIG_ESB_TX_FIFO_SIZE);
static int packets_send = 0;

void event_handler(struct esb_evt const* event);

#define _RADIO_SHORTS_COMMON                                                                            \
    (RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk | \
     RADIO_SHORTS_DISABLED_RSSISTOP_Msk)

#if defined(CONFIG_CLOCK_CONTROL_NRF)
int clocks_start(void) {
    int err;
    int res;
    struct onoff_manager* clk_mgr;
    struct onoff_client clk_cli;

    clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (!clk_mgr) {
        LOG_ERR("Unable to get the Clock manager");
        return -ENXIO;
    }

    sys_notify_init_spinwait(&clk_cli.notify);

    err = onoff_request(clk_mgr, &clk_cli);
    if (err < 0) {
        LOG_ERR("Clock request failed: %d", err);
        return err;
    }

    do {
        err = sys_notify_fetch_result(&clk_cli.notify, &res);
        if (!err && res) {
            LOG_ERR("Clock could not be started: %d", res);
            return res;
        }
    } while (err);

#if NRF54L_ERRATA_20_PRESENT
    if (nrf54l_errata_20()) {
        nrf_power_task_trigger(NRF_POWER, NRF_POWER_TASK_CONSTLAT);
    }
#endif /* NRF54L_ERRATA_20_PRESENT */

#if defined(NRF54LM20A_ENGA_XXAA)
    /* MLTPAN-39 */
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif

    LOG_DBG("HF clock started");
    return 0;
}

#elif defined(CONFIG_CLOCK_CONTROL_NRF2)

int clocks_start(void) {
    int err;
    int res;
    const struct device* radio_clk_dev = DEVICE_DT_GET_OR_NULL(DT_CLOCKS_CTLR(DT_NODELABEL(radio)));
    struct onoff_client radio_cli;

    /** Keep radio domain powered all the time to reduce latency. */
    nrf_lrcconf_poweron_force_set(NRF_LRCCONF010, NRF_LRCCONF_POWER_DOMAIN_1, true);

    sys_notify_init_spinwait(&radio_cli.notify);

    err = nrf_clock_control_request(radio_clk_dev, NULL, &radio_cli);

    do {
        err = sys_notify_fetch_result(&radio_cli.notify, &res);
        if (!err && res) {
            LOG_ERR("Clock could not be started: %d", res);
            return res;
        }
    } while (err == -EAGAIN);

    nrf_lrcconf_clock_always_run_force_set(NRF_LRCCONF000, 0, true);
    nrf_lrcconf_task_trigger(NRF_LRCCONF000, NRF_LRCCONF_TASK_CLKSTART_0);

    LOG_DBG("HF clock started");
    return 0;
}

#else
BUILD_ASSERT(false, "No Clock Control driver");
#endif /* defined(CONFIG_CLOCK_CONTROL_NRF2) */

static int esb_initialize(void) {
    int rc;
    /* These are arbitrary default addresses. In end user products
     * different addresses should be used for each set of devices.
     */
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};
#if 1
    struct esb_config config = {.protocol = ESB_PROTOCOL_ESB_DPL,
                                .mode = ESB_MODE_PTX,
                                .event_handler = event_handler,
                                .bitrate = ESB_BITRATE_4MBPS,
                                .crc = ESB_CRC_16BIT,
                                .tx_output_power = 0,
                                .retransmit_delay = 0,
                                .retransmit_count = 0,
                                .tx_mode = ESB_TXMODE_AUTO,  // ESB_TXMODE_MANUAL,
                                .payload_length = CONFIG_ESB_MAX_PAYLOAD_LENGTH,
                                .selective_auto_ack = true,
                                .use_fast_ramp_up = true};
#else
    struct esb_config config = ESB_DEFAULT_CONFIG;

    config.protocol = ESB_PROTOCOL_ESB_DPL;
    config.retransmit_delay = 0;
    config.retransmit_count = 0;
    config.bitrate = ESB_BITRATE_4MBPS;
    config.event_handler = event_handler;
    config.mode = ESB_MODE_PTX;
    config.selective_auto_ack = true;
    if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING)) {
        config.use_fast_ramp_up = true;
    }

#endif
    rc = esb_init(&config);

    if (rc) {
        return rc;
    }

    rc = esb_set_base_address_0(base_addr_0);
    if (rc) {
        return rc;
    }

    rc = esb_set_base_address_1(base_addr_1);
    if (rc) {
        return rc;
    }

    rc = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
    if (rc) {
        return rc;
    }

    esb_flush_tx();

    return 0;
}

void event_handler(struct esb_evt const* event) {
    dk_set_led(0, 0);

    switch (event->evt_id) {
        case ESB_EVENT_TX_SUCCESS:
            packets_send++;
            k_sem_give(&tx_sem);
            break;
        case ESB_EVENT_TX_FAILED:
            LOG_DBG("TX FAILED EVENT");
            break;
        case ESB_EVENT_RX_RECEIVED:
            LOG_DBG("Packet received");
            break;
    }
}

static void sample_handler(struct k_timer* timer) {
    dk_set_led(0, 1);
    int rc = esb_start_tx();
    if (rc != 0) {
        LOG_ERR("esb_start_tx returned %d", rc);
    }
}

int main(void) {
    int err;
    bool timer_started = false;
    audio_message_t message = {0};

    LOG_INF("Enhanced ShockBurst ptx sample. sample message size: %d bytes", sizeof(audio_message_t));

    err = clocks_start();
    if (err) {
        return 0;
    }

    err = dk_leds_init();
    if (err) {
        LOG_ERR("LEDs initialization failed, err %d", err);
        return 0;
    }
    dk_set_led(0, 0);

    err = esb_initialize();
    if (err) {
        LOG_ERR("ESB initialization failed, err %d", err);
        return 0;
    }

    tx_payload.pipe = 0;
    tx_payload.length = sizeof(audio_message_t);
    tx_payload.noack = true;

    LOG_INF("Initialization complete");
    LOG_INF("Sending test packet");

    while (1) {
        k_sem_take(&tx_sem, K_FOREVER);
        for (int i = 0; i < NUM_SAMPLES; i++) {
            message.samples[i].left = i;
            message.samples[i].right = NUM_SAMPLES - i;
        }

        memcpy(tx_payload.data, &message, sizeof(audio_message_t));
        int rc = esb_write_payload(&tx_payload);
        if (rc) {
            LOG_ERR("Payload write failed, rc %d", rc);
        }
        dk_set_led(0, 1);

        message.sequence_number++;

        if (0 && timer_started == false) {
            k_timer_start(&sample_timer, K_USEC(333), K_USEC(333));
            timer_started = true;
        }
    }
}
