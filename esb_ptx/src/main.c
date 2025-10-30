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

#define NUM_SAMPLES 126U

static bool ready = true;
static struct esb_payload tx_payload = {0};
static void sample_handler(struct k_timer* timer);
K_TIMER_DEFINE(sample_timer, sample_handler, NULL);

static uint16_t audio_samples[NUM_SAMPLES] = {0};
static int packets_send = 0;

#define _RADIO_SHORTS_COMMON                                     \
  (RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | \
   RADIO_SHORTS_ADDRESS_RSSISTART_Msk | RADIO_SHORTS_DISABLED_RSSISTOP_Msk)

void event_handler(struct esb_evt const* event) {
  ready = true;

  switch (event->evt_id) {
    case ESB_EVENT_TX_SUCCESS:
      LOG_DBG("TX SUCCESS EVENT");
      break;
    case ESB_EVENT_TX_FAILED:
      LOG_DBG("TX FAILED EVENT");
      break;
    case ESB_EVENT_RX_RECEIVED:
      LOG_DBG("Packet received");
      break;
  }
}

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
  const struct device* radio_clk_dev =
      DEVICE_DT_GET_OR_NULL(DT_CLOCKS_CTLR(DT_NODELABEL(radio)));
  struct onoff_client radio_cli;

  /** Keep radio domain powered all the time to reduce latency. */
  nrf_lrcconf_poweron_force_set(NRF_LRCCONF010, NRF_LRCCONF_POWER_DOMAIN_1,
                                true);

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

int esb_initialize(void) {
  int err;
  /* These are arbitrary default addresses. In end user products
   * different addresses should be used for each set of devices.
   */
  uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
  uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
  uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

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

  err = esb_init(&config);

  if (err) {
    return err;
  }

  err = esb_set_base_address_0(base_addr_0);
  if (err) {
    return err;
  }

  err = esb_set_base_address_1(base_addr_1);
  if (err) {
    return err;
  }

  err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
  if (err) {
    return err;
  }

  return 0;
}

static void leds_update(uint8_t value) {
  uint32_t leds_mask = (!(value % 8 > 0 && value % 8 <= 4) ? DK_LED1_MSK : 0) |
                       (!(value % 8 > 1 && value % 8 <= 5) ? DK_LED2_MSK : 0) |
                       (!(value % 8 > 2 && value % 8 <= 6) ? DK_LED3_MSK : 0) |
                       (!(value % 8 > 3) ? DK_LED4_MSK : 0);

  dk_set_leds(leds_mask);
}

static void sample_handler(struct k_timer* timer) {
  if (ready) {
    ready = false;
    esb_flush_tx();
    leds_update(tx_payload.data[1]);

    memcpy(tx_payload.data, audio_samples, sizeof(audio_samples));
    int err = esb_write_payload(&tx_payload);
    if (err) {
      LOG_ERR("Payload write failed, err %d", err);
    } else {
      packets_send++;
    }

    for (int i = 0; i < NUM_SAMPLES; i++) {
      audio_samples[i] += 1;
    }
  } else {
    LOG_ERR("NOT READY!");
  }
}

int main(void) {
  int err;

  LOG_INF("Enhanced ShockBurst ptx sample");

  err = clocks_start();
  if (err) {
    return 0;
  }

  err = dk_leds_init();
  if (err) {
    LOG_ERR("LEDs initialization failed, err %d", err);
    return 0;
  }

  err = esb_initialize();
  if (err) {
    LOG_ERR("ESB initialization failed, err %d", err);
    return 0;
  }

  tx_payload.pipe = 0;
  tx_payload.length = sizeof(audio_samples);
  tx_payload.noack = true;

  LOG_INF("Initialization complete");
  LOG_INF("Sending test packet");

  k_timer_start(&sample_timer, K_USEC(2000), K_USEC(2000));

  while (1) {
    k_sleep(K_SECONDS(10));
  }
}
