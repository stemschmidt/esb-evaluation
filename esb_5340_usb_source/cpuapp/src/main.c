/*
 * cpuapp/src/main.c
 *
 * USB Audio (UAC2 Explicit Feedback) → IPC → cpunet (ESB)
 *
 * Feedback-Mechanismus:
 *   Der USB-Host erwartet alle 2^(10-P) SOF-Intervalle einen Feedback-Wert
 *   im 10.14 Fixed-Point Format (USB Audio Class 2 Spec, Abschnitt 5.12.4.2).
 *
 *   Wir messen den Füllstand des IPC-Ringpuffers und passen den Feedback-Wert
 *   entsprechend an:
 *     - Puffer > Ziel → Host sendet zu viel → Feedback < Nominal  (bremsen)
 *     - Puffer < Ziel → Host sendet zu wenig → Feedback > Nominal (beschleunigen)
 *     - Puffer = Ziel → Feedback = Nominal (48.000 in 10.14)
 *
 *   Nominaler Feedback-Wert für 48kHz:
 *     48000 Hz / 1000 SOF/s = 48 Samples/SOF
 *     48 << 14 = 786432 = 0x0C0000
 */

#include <stdint.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/ipc/ipc_service.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/class/usbd_uac2.h>
#include <zephyr/usb/usbd.h>

#include "feedback.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define SAMPLE_FREQUENCY (SAMPLES_PER_SOF * 1000)
#define SAMPLE_BIT_WIDTH 16
#define NUMBER_OF_CHANNELS 2
#define BYTES_PER_SAMPLE DIV_ROUND_UP(SAMPLE_BIT_WIDTH, 8)
#define BYTES_PER_SLOT (BYTES_PER_SAMPLE * NUMBER_OF_CHANNELS)
#define MIN_BLOCK_SIZE ((SAMPLES_PER_SOF - 1) * BYTES_PER_SLOT)
#define BLOCK_SIZE (SAMPLES_PER_SOF * BYTES_PER_SLOT)
#define MAX_BLOCK_SIZE ((SAMPLES_PER_SOF + 1) * BYTES_PER_SLOT)

#define HEADPHONES_OUT_TERMINAL_ID UAC2_ENTITY_ID(DT_NODELABEL(out_terminal))

/* 48kHz × 2ch × 2Byte × 1ms = 192 Byte pro Frame */
#define AUDIO_FRAME_BYTES 192
/* Sequenznummer (2 Byte) + Payload (192 Byte) */
#define IPC_PACKET_SIZE (AUDIO_FRAME_BYTES + 2)

/* Anzahl gepufferter Frames (Jitter-Puffer) */
#define AUDIO_BUF_FRAMES 8
/* Ziel-Füllstand: halb voll = minimale Latenz bei maximalem Spielraum */
#define AUDIO_BUF_TARGET (AUDIO_BUF_FRAMES / 2)

/* ── Feedback Konstanten (USB Audio Class 2, 10.14 Fixed-Point) ────────────── */

/* 48 Samples/SOF << 14 = 0x0C0000 */
#define FEEDBACK_NOMINAL ((48U << 14) + 1)
/* Maximale Abweichung: ±1 Sample/SOF (= ±16384 in 10.14) */
#define FEEDBACK_STEP (1U << 14)
/* Totband: erst reagieren wenn Puffer ±1 Frame vom Ziel abweicht */
#define FEEDBACK_DEADBAND 1

/* Feedback wird alle N ms neu berechnet (nicht bei jedem Frame nötig) */
#define FEEDBACK_INTERVAL_MS 10

/* ── Puffer ────────────────────────────────────────────────────────────────── */

/* Ringpuffer für Audio-Frames */
static uint8_t audio_ring[AUDIO_BUF_FRAMES][AUDIO_FRAME_BYTES];
static volatile uint32_t ring_write = 0;
static volatile uint32_t ring_read = 0;
K_SEM_DEFINE(sem_frame_ready, 0, AUDIO_BUF_FRAMES);

/* IPC Send-Puffer */
static uint8_t ipc_buf[IPC_PACKET_SIZE];
static uint16_t seq_num = 0;

/* Aktueller Feedback-Wert (atomar gelesen/geschrieben) */
static atomic_t current_feedback_value = ATOMIC_INIT(FEEDBACK_NOMINAL);

static uint8_t dummy[MAX_BLOCK_SIZE];
/* ── Hilfsfunktionen ───────────────────────────────────────────────────────── */

/* Füllstand des Ringpuffers (0 .. AUDIO_BUF_FRAMES-1) */
static inline int buf_fill_level(void) {
    int fill = (int)ring_write - (int)ring_read;
    if (fill < 0) {
        fill += AUDIO_BUF_FRAMES;
    }
    return fill;
}

/* ── Feedback Berechnung ───────────────────────────────────────────────────── */

/*
 * Berechnet den neuen Feedback-Wert anhand des Pufferfüllstands.
 * Wird alle FEEDBACK_INTERVAL_MS ms aufgerufen.
 *
 * Regelstrategie (einfacher P-Regler mit Totband):
 *   error = fill_level - TARGET
 *   if |error| <= DEADBAND → Nominal
 *   if error > 0           → Nominal - STEP  (Host bremsen)
 *   if error < 0           → Nominal + STEP  (Host beschleunigen)
 */
static void feedback_update(void) {
    int fill = buf_fill_level();
    int error = fill - AUDIO_BUF_TARGET;
    uint32_t new_fb;

    if (error > FEEDBACK_DEADBAND) {
        /* Puffer läuft voll → Host bremsen */
        new_fb = FEEDBACK_NOMINAL - FEEDBACK_STEP;
    } else if (error < -FEEDBACK_DEADBAND) {
        /* Puffer läuft leer → Host beschleunigen */
        new_fb = FEEDBACK_NOMINAL + FEEDBACK_STEP;
    } else {
        /* Im Totband → Nominalwert */
        new_fb = FEEDBACK_NOMINAL;
    }

    atomic_set(&current_feedback_value, (atomic_val_t)new_fb);

    LOG_DBG("Feedback: fill=%d error=%d fb=0x%06X (%.3f Hz)", fill, error, new_fb,
            (double)new_fb / (double)(1 << 14) * 1000.0);
}

/* ── USB Audio Callbacks ───────────────────────────────────────────────────── */

/*
 * Wird vom USB-Stack aufgerufen wenn ein Audio-Paket vom Host angekommen ist.
 * Läuft im USB-ISR-Kontext → nur in Puffer schreiben, kein blocking!
 */
static void usb_audio_data_received(const struct device* dev, struct net_buf* buffer, size_t size) {
    ARG_UNUSED(dev);

    if (!buffer || size == 0) {
        return;
    }

    /* Puffer voll? Ältesten Frame überschreiben (Drop-oldest Strategie) */
    uint32_t next_write = (ring_write + 1) % AUDIO_BUF_FRAMES;
    if (next_write == ring_read) {
        LOG_WRN("Audio-Ringpuffer voll, Frame verworfen");
        ring_read = (ring_read + 1) % AUDIO_BUF_FRAMES;
        k_sem_take(&sem_frame_ready, K_NO_WAIT);
    }

    /* Daten in Ringpuffer kopieren */
    size_t copy_len = MIN(size, AUDIO_FRAME_BYTES);
    memcpy(audio_ring[ring_write], buffer->data, copy_len);

    /* Bei kürzeren Paketen auffüllen (z.B. 191 Byte bei Feedback-Korrektur) */
    if (copy_len < AUDIO_FRAME_BYTES) {
        memset(audio_ring[ring_write] + copy_len, 0, AUDIO_FRAME_BYTES - copy_len);
    }

    ring_write = next_write;
    k_sem_give(&sem_frame_ready);
}

/*
 * Wird vom USB-Stack aufgerufen wenn der Host den Feedback-Wert anfordert.
 * Läuft im USB-ISR-Kontext.
 *
 * USB Audio Class 2: Feedback als 3-Byte Little-Endian 10.14 Fixed-Point.
 */
static void usb_audio_feedback_cb(const struct device* dev, struct net_buf* buffer, size_t* size) {
    ARG_UNUSED(dev);

    uint32_t fb = (uint32_t)atomic_get(&current_feedback_value);

    /* 3 Byte Little-Endian: Bits [21:0] des 10.14 Werts */
    buffer->data[0] = (fb) & 0xFF;
    buffer->data[1] = (fb >> 8) & 0xFF;
    buffer->data[2] = (fb >> 16) & 0xFF;
    *size = 3;
}

/* ── IPC ───────────────────────────────────────────────────────────────────── */

static struct ipc_ept ept;
static K_SEM_DEFINE(sem_ipc_bound, 0, 1);

static void ipc_bound_cb(void* priv) {
    LOG_INF("IPC endpoint bound – cpunet bereit");
    k_sem_give(&sem_ipc_bound);
}

static void ipc_received_cb(const void* data, size_t len, void* priv) {
    /* cpunet sendet nichts zurück – ignorieren */
    ARG_UNUSED(data);
    ARG_UNUSED(len);
    ARG_UNUSED(priv);
}

static struct ipc_ept_cfg ept_cfg = {
    .name = "audio_esb",
    .cb =
        {
            .bound = ipc_bound_cb,
            .received = ipc_received_cb,
        },
};

/* ── Sender Thread ─────────────────────────────────────────────────────────── */

/*
 * Wartet auf fertige Audio-Frames, aktualisiert periodisch den Feedback-Wert
 * und sendet Frames via IPC an cpunet.
 */
static void sender_thread_fn(void* a, void* b, void* c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    /* Warten bis IPC bereit */
    LOG_INF("Warte auf IPC-Verbindung zu cpunet...");
    k_sem_take(&sem_ipc_bound, K_FOREVER);
    LOG_INF("IPC bereit – starte Audio-Übertragung");

    uint32_t last_feedback_update = k_uptime_get_32();

    while (true) {
        /* Auf nächsten Frame warten (max. 2ms, damit Feedback-Update nicht blockiert) */
        if (k_sem_take(&sem_frame_ready, K_MSEC(2)) == 0) {
            /* Sequenznummer eintragen (Big Endian) */
            ipc_buf[0] = (seq_num >> 8) & 0xFF;
            ipc_buf[1] = seq_num & 0xFF;
            seq_num++;

            /* Audio-Daten kopieren und Ringpuffer vorrücken */
            memcpy(&ipc_buf[2], audio_ring[ring_read], AUDIO_FRAME_BYTES);
            ring_read = (ring_read + 1) % AUDIO_BUF_FRAMES;

            /* Via IPC senden */
            int ret = ipc_service_send(&ept, ipc_buf, IPC_PACKET_SIZE);
            if (ret < 0) {
                LOG_ERR("IPC send fehlgeschlagen: %d", ret);
            }
        }

        /* Feedback periodisch neu berechnen */
        uint32_t now = k_uptime_get_32();
        if ((now - last_feedback_update) >= FEEDBACK_INTERVAL_MS) {
            feedback_update();
            last_feedback_update = now;
        }
    }
}

K_THREAD_DEFINE(sender_thread, 2048, sender_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, 0);

/* Absolute minimum is 5 buffers (1 actively consumed by I2S, 2nd queued as next
 * buffer, 3rd acquired by USB stack to receive data to, and 2 to handle SOF/I2S
 * offset errors), but add 2 additional buffers to prevent out of memory errors
 * when USB host decides to perform rapid terminal enable/disable cycles.
 */
#define I2S_BUFFERS_COUNT 7
K_MEM_SLAB_DEFINE_STATIC(i2s_tx_slab, ROUND_UP(MAX_BLOCK_SIZE, UDC_BUF_GRANULARITY), I2S_BUFFERS_COUNT, UDC_BUF_ALIGN);

struct usb_i2s_ctx {
    const struct device* i2s_dev;
    bool terminal_enabled;
    bool i2s_started;
    /* Number of blocks written, used to determine when to start I2S.
     * Overflows are not a problem becuse this variable is not necessary
     * after I2S is started.
     */
    uint8_t i2s_blocks_written;
    struct feedback_ctx* fb;
};

static void uac2_terminal_update_cb(const struct device* dev, uint8_t terminal, bool enabled, bool microframes,
                                    void* user_data) {
    struct usb_i2s_ctx* ctx = user_data;

    /* This sample has only one terminal therefore the callback can simply
     * ignore the terminal variable.
     */
    __ASSERT_NO_MSG(terminal == HEADPHONES_OUT_TERMINAL_ID);
    /* This sample is for Full-Speed only devices. */
    __ASSERT_NO_MSG(microframes == false);

    ctx->terminal_enabled = enabled;
    if (ctx->i2s_started && !enabled) {
        i2s_trigger(ctx->i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
        ctx->i2s_started = false;
        ctx->i2s_blocks_written = 0;
        feedback_reset_ctx(ctx->fb);
    }
}

static void* uac2_get_recv_buf(const struct device* dev, uint8_t terminal, uint16_t size, void* user_data) {
    ARG_UNUSED(dev);
    static uint32_t count = 0;

    count++;

    if (count % 100 == 0) {
        LOG_INF("dummy has been requested %d times", count);
    }

    return dummy;
}

static void uac2_data_recv_cb(const struct device* dev, uint8_t terminal, void* buf, uint16_t size, void* user_data) {
    struct usb_i2s_ctx* ctx = user_data;
    int ret;

    static uint32_t received = 0;

    received++;
    if (received % 100 == 0) {
        LOG_INF("%d bytes of data received", received * size);
    }
    LOG_DBG("Received %d data to input terminal %d", size, terminal);
}

static void uac2_buf_release_cb(const struct device* dev, uint8_t terminal, void* buf, void* user_data) {
    /* This sample does not send audio data so this won't be called */
}

/* Variables for debug use to facilitate simple how feedback value affects
 * audio data rate experiments. These debug variables can also be used to
 * determine how well the feedback regulator deals with errors. The values
 * are supposed to be modified by debugger.
 *
 * Setting use_hardcoded_feedback to true, essentially bypasses the feedback
 * regulator and makes host send hardcoded_feedback samples every 16384 SOFs
 * (when operating at Full-Speed).
 *
 * The feedback at Full-Speed is Q10.14 value. For 48 kHz audio sample rate,
 * there are nominally 48 samples every SOF. The corresponding value is thus
 * 48 << 14. Such feedback value would result in host sending always 48 samples.
 * Now, if we want to receive more samples (because 1 ms according to audio
 * sink is shorter than 1 ms according to USB Host 500 ppm SOF timer), then
 * the feedback value has to be increased. The fractional part is 14-bit wide
 * and therefore increment by 1 means 1 additional sample every 2**14 SOFs.
 * (48 << 14) + 1 therefore results in host sending 48 samples 16383 times and
 * 49 samples 1 time during every 16384 SOFs.
 *
 * Similarly, if we want to receive less samples (because 1 ms according to
 * audio signk is longer than 1 ms according to USB Host), then the feedback
 * value has to be decreased. (48 << 14) - 1 therefore results in host sending
 * 48 samples 16383 times and 47 samples 1 time during every 16384 SOFs.
 *
 * If the feedback value differs by more than 1 (i.e. LSB), then the +1/-1
 * samples packets are generally evenly distributed. For example feedback value
 * (48 << 14) + (1 << 5) results in 48 samples 511 times and 49 samples 1 time
 * during every 512 SOFs.
 *
 * For High-Speed above changes slightly, because the feedback format is Q16.16
 * and microframes are used. The 48 kHz audio sample rate is achieved by sending
 * 6 samples every SOF (microframe). The nominal value is the average number of
 * samples to send every microframe and therefore for 48 kHz the nominal value
 * is (6 << 16).
 */
static volatile bool use_hardcoded_feedback;
static volatile uint32_t hardcoded_feedback = (48 << 14) + 1;

static uint32_t uac2_feedback_cb(const struct device* dev, uint8_t terminal, void* user_data) {
    /* Sample has only one UAC2 instance with one terminal so both can be
     * ignored here.
     */
    ARG_UNUSED(dev);
    ARG_UNUSED(terminal);

    return (uint32_t)atomic_get(&current_feedback_value);
}

static void uac2_sof(const struct device* dev, void* user_data) {
    ARG_UNUSED(dev);
    struct usb_i2s_ctx* ctx = user_data;

    if (ctx->i2s_started) {
        feedback_process(ctx->fb);
    }

    /* We want to maintain 3 SOFs delay, i.e. samples received during SOF n
     * should be on I2S during SOF n+3. This provides enough wiggle room
     * for software scheduling that effectively eliminates "buffers not
     * provided in time" problem.
     *
     * ">= 2" translates into 3 SOFs delay because the timeline is:
     * USB SOF n
     *   OUT DATA0 n received from host
     * USB SOF n+1
     *   DATA0 n is available to UDC driver (See Universal Serial Bus
     *   Specification Revision 2.0 5.12.5 Data Prebuffering) and copied
     *   to I2S buffer before SOF n+2; i2s_blocks_written = 1
     *   OUT DATA0 n+1 received from host
     * USB SOF n+2
     *   DATA0 n+1 is copied; i2s_block_written = 2
     *   OUT DATA0 n+2 received from host
     * USB SOF n+3
     *   This function triggers I2S start
     *   DATA0 n+2 is copied; i2s_block_written is no longer relevant
     *   OUT DATA0 n+3 received from host
     */
    if (!ctx->i2s_started && ctx->terminal_enabled && ctx->i2s_blocks_written >= 2) {
        i2s_trigger(ctx->i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
        ctx->i2s_started = true;
        feedback_start(ctx->fb, ctx->i2s_blocks_written);
    }
}

static struct uac2_ops usb_audio_ops = {
    .sof_cb = uac2_sof,
    .terminal_update_cb = uac2_terminal_update_cb,
    .get_recv_buf = uac2_get_recv_buf,
    .data_recv_cb = uac2_data_recv_cb,
    .buf_release_cb = uac2_buf_release_cb,
    .feedback_cb = uac2_feedback_cb,
};

static struct usb_i2s_ctx main_ctx;

int main(void) {
    const struct device* dev = DEVICE_DT_GET(DT_NODELABEL(uac2_headphones));
    struct usbd_context* sample_usbd;
    struct i2s_config config;
    int ret;

    LOG_INF("Start cpuapp...");

    main_ctx.fb = feedback_init();

    usbd_uac2_set_ops(dev, &usb_audio_ops, &main_ctx);

    sample_usbd = sample_usbd_init_device(NULL);
    if (sample_usbd == NULL) {
        return -ENODEV;
    }

    ret = usbd_enable(sample_usbd);
    if (ret) {
        return ret;
    }

    return 0;
}
