/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <dk_buttons_and_leds.h>
#include <esb.h>
#include <nrf.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2s.h>
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

LOG_MODULE_REGISTER(esb_prx, CONFIG_ESB_PRX_APP_LOG_LEVEL);

#define PACKET_RECEIVED 0U
#define AUDIO_QUEUE_OK 1U
#define I2S_WRITE_OK 2U
#define I2S_ACTIVE 3U

#define ESB_RF_CHANNEL 10U

static struct esb_payload rx_payload = {0};
#define NUM_SAMPLES 16U
typedef struct {
    uint16_t left;
    uint16_t right;
} sample_t;

typedef struct {
    uint16_t sequence_number;
    sample_t samples[NUM_SAMPLES];
} audio_message_t;

static uint16_t last_sequence_number = 0;
#define SAMPLE_BIT_WIDTH 16U
#define NUMBER_OF_CHANNELS 2U
#define BLOCK_DURATION_MS 100U

#define SAMPLES_PER_BLOCK (NUMBER_OF_CHANNELS * NUM_SAMPLES)
#define BYTES_PER_SAMPLE (SAMPLE_BIT_WIDTH / 8U)
#define BLOCK_SIZE (SAMPLES_PER_BLOCK * BYTES_PER_SAMPLE)
#define BLOCK_COUNT 4U

/* mem slab */
K_MEM_SLAB_DEFINE(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);
K_MSGQ_DEFINE(audio_msgq, sizeof(void*), 8, 4);

const struct device* i2s_dev = DEVICE_DT_GET(DT_ALIAS(i2s_codec_tx));

void event_handler(struct esb_evt const* event) {
    switch (event->evt_id) {
        case ESB_EVENT_TX_SUCCESS:
            LOG_DBG("TX SUCCESS EVENT");
            break;
        case ESB_EVENT_TX_FAILED:
            LOG_DBG("TX FAILED EVENT");
            break;
        case ESB_EVENT_RX_RECEIVED:
            if (esb_read_rx_payload(&rx_payload) == 0) {
                audio_message_t* msg = (audio_message_t*)&rx_payload.data;
                if ((last_sequence_number + 1) == msg->sequence_number) {
                    void* mem_block;
                    if (k_mem_slab_alloc(&mem_slab, &mem_block, K_NO_WAIT) == 0) {
                        memcpy(mem_block, msg->samples, BLOCK_SIZE);
                        int rc = k_msgq_put(&audio_msgq, mem_block, K_NO_WAIT);
                        if (rc == 0) {
                            dk_set_led(AUDIO_QUEUE_OK, 1);
                        } else {
                            dk_set_led(AUDIO_QUEUE_OK, 0);
                        }
                    }
                    dk_set_led(PACKET_RECEIVED, 1);
                } else {
                    dk_set_led(PACKET_RECEIVED, 0);
                }
                last_sequence_number = msg->sequence_number;
            } else {
                LOG_ERR("Error while reading rx packet");
            }
            break;
    }
}

#if defined(CONFIG_CLOCK_CONTROL_NRF)
int clocks_start(void) {
    int rc;
    int res;
    struct onoff_manager* clk_mgr;
    struct onoff_client clk_cli;

    clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (!clk_mgr) {
        LOG_ERR("Unable to get the Clock manager");
        return -ENXIO;
    }

    sys_notify_init_spinwait(&clk_cli.notify);

    rc = onoff_request(clk_mgr, &clk_cli);
    if (rc < 0) {
        LOG_ERR("Clock request failed: %d", rc);
        return rc;
    }

    do {
        rc = sys_notify_fetch_result(&clk_cli.notify, &res);
        if (!rc && res) {
            LOG_ERR("Clock could not be started: %d", res);
            return res;
        }
    } while (rc);

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
    int rc;
    int res;
    const struct device* radio_clk_dev = DEVICE_DT_GET_OR_NULL(DT_CLOCKS_CTLR(DT_NODELABEL(radio)));
    struct onoff_client radio_cli;

    /** Keep radio domain powered all the time to reduce latency. */
    nrf_lrcconf_poweron_force_set(NRF_LRCCONF010, NRF_LRCCONF_POWER_DOMAIN_1, true);

    sys_notify_init_spinwait(&radio_cli.notify);

    rc = nrf_clock_control_request(radio_clk_dev, NULL, &radio_cli);

    do {
        rc = sys_notify_fetch_result(&radio_cli.notify, &res);
        if (!rc && res) {
            LOG_ERR("Clock could not be started: %d", res);
            return res;
        }
    } while (rc == -EAGAIN);

    nrf_lrcconf_clock_always_run_force_set(NRF_LRCCONF000, 0, true);
    nrf_lrcconf_task_trigger(NRF_LRCCONF000, NRF_LRCCONF_TASK_CLKSTART_0);

    LOG_DBG("HF clock started");

    return 0;
}

#else
BUILD_ASSERT(false, "No Clock Control driver");
#endif /* defined(CONFIG_CLOCK_CONTROL_NRF2) */

int esb_initialize(void) {
    int rc;
    /* These are arbitrary default addresses. In end user products
     * different addresses should be used for each set of devices.
     */
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

    struct esb_config config = ESB_DEFAULT_CONFIG;

    config.protocol = ESB_PROTOCOL_ESB_DPL;
    config.mode = ESB_MODE_PRX;
    config.bitrate = ESB_BITRATE_4MBPS;
    config.crc = ESB_CRC_OFF;
    config.use_fast_ramp_up = true;

    /* No Ack, no retransmit. */
    config.retransmit_count = 0;
    config.retransmit_delay = 0;
    config.selective_auto_ack = true;

    config.event_handler = event_handler;

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

    rc = esb_set_rf_channel(ESB_RF_CHANNEL);
    if (rc) {
        return rc;
    }

    return 0;
}

int main(void) {
    int rc;
    int blocks_received = 0;
    LOG_INF("Enhanced ShockBurst prx sample");

    if (!device_is_ready(i2s_dev)) {
        printk("I2S device not ready\n");
        return -1;
    }

    struct i2s_config cfg = {0};
    cfg.word_size = SAMPLE_BIT_WIDTH;
    cfg.channels = NUMBER_OF_CHANNELS;
    cfg.format = I2S_FMT_DATA_FORMAT_I2S;
#ifdef CONFIG_USE_CODEC_CLOCK
    cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
#else
    cfg.options = I2S_OPT_FRAME_CLK_SLAVE | I2S_OPT_BIT_CLK_SLAVE;
#endif
    cfg.frame_clk_freq = CONFIG_SAMPLE_FREQ;
    cfg.mem_slab = &mem_slab;
    cfg.block_size = BLOCK_SIZE;
    cfg.timeout = 2000U;

    if (i2s_configure(i2s_dev, I2S_DIR_TX, &cfg) < 0) {
        printk("I2S configure failed\n");
        return -1;
    }

    rc = clocks_start();
    if (rc) {
        return 0;
    }

    rc = dk_leds_init();
    if (rc) {
        LOG_ERR("LEDs initialization failed, rc %d", rc);
        return 0;
    }

    dk_set_led(PACKET_RECEIVED, 0);
    dk_set_led(AUDIO_QUEUE_OK, 0);
    dk_set_led(I2S_WRITE_OK, 0);
    dk_set_led(I2S_ACTIVE, 0);

    rc = esb_initialize();
    if (rc) {
        LOG_ERR("ESB initialization failed, rc %d", rc);
        return 0;
    }

    LOG_INF("Initialization complete");

    LOG_INF("Setting up for packet receiption");

    rc = esb_start_rx();
    if (rc) {
        LOG_ERR("RX setup failed, rc %d", rc);
        return 0;
    }

    while (true) {
        void* audio_buffer;
        rc = k_msgq_get(&audio_msgq, &audio_buffer, K_FOREVER);
        if (rc == 0) {
            if (i2s_write(i2s_dev, audio_buffer, BLOCK_SIZE) < 0) {
                dk_set_led(I2S_WRITE_OK, 1);
            }
            k_mem_slab_free(&mem_slab, audio_buffer);
            if (++blocks_received == BLOCK_COUNT) {
                if (i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START) == 0) {
                    dk_set_led(I2S_ACTIVE, 1);
                } else {
                    dk_set_led(I2S_ACTIVE, 0);
                }
            }
        }
    }

    return 0;
}
