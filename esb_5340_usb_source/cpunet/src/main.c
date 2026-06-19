/*
 * cpunet/src/main.c
 *
 * IPC (von cpuapp) → ESB PTX ohne ACK
 *
 * Ablauf:
 *  1. IPC Service initialisieren (Empfänger)
 *  2. ESB im PTX-Modus ohne ACK konfigurieren
 *  3. Eingehende IPC-Pakete direkt als ESB-Payload senden
 */

#include <esb.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/ipc/ipc_service.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cpunet, LOG_LEVEL_INF);

/* ── Konstanten ────────────────────────────────────────────────────────────── */

#define AUDIO_FRAME_BYTES 192
#define IPC_PACKET_SIZE (AUDIO_FRAME_BYTES + 2) /* seq(2) + audio(192) */

/* ESB RF-Kanal (0-100, entspricht 2400 + N MHz) */
#define ESB_RF_CHANNEL 10

/* ESB Pipe 0 Adresse */
static uint8_t base_addr_0[] = {0xE7, 0xE7, 0xE7, 0xE7};
static uint8_t base_addr_1[] = {0xC2, 0xC2, 0xC2, 0xC2};
static uint8_t addr_prefix[] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

/* ── ESB TX-Queue ──────────────────────────────────────────────────────────── */

/* Einfache Queue für ESB-Pakete (IPC-Callback → Sender-Thread) */
#define ESB_QUEUE_DEPTH 4

static struct esb_payload esb_queue[ESB_QUEUE_DEPTH];
static uint32_t esb_q_write = 0;
static uint32_t esb_q_read = 0;
K_SEM_DEFINE(sem_esb_ready, 0, ESB_QUEUE_DEPTH);

/* ── ESB Callbacks ─────────────────────────────────────────────────────────── */

/* Wird aufgerufen wenn ESB ein Paket gesendet hat (kein ACK → immer "TX done") */
void event_handler(struct esb_evt const* event) {
    switch (event->evt_id) {
        case ESB_EVENT_TX_SUCCESS:
            /* Gut, Paket gesendet */
            break;
        case ESB_EVENT_TX_FAILED:
            /* Kann ohne ACK nicht passieren */
            break;
        case ESB_EVENT_RX_RECEIVED:
            /* Wir empfangen nichts */
            break;
    }
}

/* ── ESB Initialisierung ───────────────────────────────────────────────────── */

static int esb_initialize(void) {
    int ret;

    struct esb_config esb_config = ESB_DEFAULT_CONFIG;

    /* PTX Modus: nur senden */
    esb_config.protocol = ESB_PROTOCOL_ESB_DPL;
    esb_config.mode = ESB_MODE_PTX;
    esb_config.bitrate = ESB_BITRATE_2MBPS;
    esb_config.crc = ESB_CRC_16BIT;

    /* Kein ACK, keine Retransmits → maximaler Durchsatz */
    esb_config.retransmit_count = 0;
    esb_config.retransmit_delay = 0;
    esb_config.selective_auto_ack = true; /* ACK per Paket steuerbar */

    /* TX FIFO sofort leeren */
    esb_config.tx_mode = ESB_TXMODE_AUTO;

    esb_config.event_handler = event_handler;

    ret = esb_init(&esb_config);
    if (ret) {
        LOG_ERR("esb_init: %d", ret);
        return ret;
    }

    ret = esb_set_base_address_0(base_addr_0);
    if (ret) {
        return ret;
    }

    ret = esb_set_base_address_1(base_addr_1);
    if (ret) {
        return ret;
    }

    ret = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
    if (ret) {
        return ret;
    }

    ret = esb_set_rf_channel(ESB_RF_CHANNEL);
    if (ret) {
        return ret;
    }

    LOG_INF("ESB initialisiert – Kanal %d (%.3d MHz), 2Mbit/s, kein ACK", ESB_RF_CHANNEL, 2400 + ESB_RF_CHANNEL);

    return 0;
}

/* ── IPC ───────────────────────────────────────────────────────────────────── */

static struct ipc_ept ept;
static K_SEM_DEFINE(sem_ipc_bound, 0, 1);

static void ipc_bound_cb(void* priv) {
    LOG_INF("IPC endpoint bound – cpuapp verbunden");
    k_sem_give(&sem_ipc_bound);
}

/*
 * Wird aufgerufen wenn ein Audio-Frame von cpuapp ankommt.
 * Läuft im IPC-ISR-Kontext → nur in Queue schreiben!
 */
static void ipc_received_cb(const void* data, size_t len, void* priv) {
    ARG_UNUSED(priv);

    if (len != IPC_PACKET_SIZE) {
        LOG_WRN("Unerwartete IPC-Paketgröße: %zu (erwartet %d)", len, IPC_PACKET_SIZE);
        return;
    }

    /* Queue voll? Ältestes Paket verwerfen */
    uint32_t next_write = (esb_q_write + 1) % ESB_QUEUE_DEPTH;
    if (next_write == esb_q_read) {
        LOG_WRN("ESB-Queue voll, Paket verworfen");
        esb_q_read = (esb_q_read + 1) % ESB_QUEUE_DEPTH;
        k_sem_take(&sem_esb_ready, K_NO_WAIT);
    }

    /* Paket aufbauen */
    struct esb_payload* pkt = &esb_queue[esb_q_write];
    pkt->pipe = 0;
    pkt->length = (uint8_t)len;
    pkt->noack = true; /* Kein ACK anfordern */
    memcpy(pkt->data, data, len);

    esb_q_write = next_write;
    k_sem_give(&sem_esb_ready);
}

static struct ipc_ept_cfg ept_cfg = {
    .name = "audio_esb",
    .cb =
        {
            .bound = ipc_bound_cb,
            .received = ipc_received_cb,
        },
};

/* ── ESB Sender Thread ─────────────────────────────────────────────────────── */

static void esb_sender_thread_fn(void* a, void* b, void* c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    /* Warten bis IPC und ESB bereit */
    k_sem_take(&sem_ipc_bound, K_FOREVER);
    LOG_INF("Starte ESB-Übertragung");

    while (true) {
        k_sem_take(&sem_esb_ready, K_FOREVER);

        struct esb_payload* pkt = &esb_queue[esb_q_read];
        esb_q_read = (esb_q_read + 1) % ESB_QUEUE_DEPTH;

        int ret = esb_write_payload(pkt);
        if (ret) {
            LOG_ERR("esb_write_payload: %d", ret);
        }
    }
}

K_THREAD_DEFINE(esb_sender_thread, 1024, esb_sender_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(4), 0, 0);

/* ── Main ──────────────────────────────────────────────────────────────────── */

int main(void) {
    int ret;

    LOG_INF("nRF5340 Audio ESB – cpunet startet");

    /* ESB initialisieren */
    ret = esb_initialize();
    if (ret) {
        LOG_ERR("ESB Init fehlgeschlagen: %d", ret);
        return ret;
    }

    /* IPC initialisieren */
    const struct device* ipc_dev = DEVICE_DT_GET(DT_NODELABEL(ipc0));
    if (!device_is_ready(ipc_dev)) {
        LOG_ERR("IPC Device nicht bereit");
        return -ENODEV;
    }

    ret = ipc_service_open_instance(ipc_dev);
    if (ret < 0 && ret != -EALREADY) {
        LOG_ERR("ipc_service_open_instance: %d", ret);
        return ret;
    }

    ret = ipc_service_register_endpoint(ipc_dev, &ept, &ept_cfg);
    if (ret < 0) {
        LOG_ERR("ipc_service_register_endpoint: %d", ret);
        return ret;
    }

    LOG_INF("Warte auf Audio-Daten von cpuapp...");

    /* Ab hier übernehmen die Threads */
    return 0;
}
