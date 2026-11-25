// M5Dial -> QMX CAT controller, v3
//   Screens: VFO / PAN / CW / AUDIO / SETUP
//   PAN: IQ spectrum +/-24 kHz (QMX IQ mode on while the screen is open)
//   (swipe left/right, or long-press the knob)
//   CW screen: speed, CW center (QMX moves offset + sidetone with it), filter width
//              (filter choices = filters enabled in "Choose filters")
//   VFO taps: mode button = next mode, VFO button = A/B/SPLIT,
//             band button = next band, frequency digit = step,
//             RIT button = on/off (long-press: knob adjusts offset; long-press again: zero)
//   Band stack, radio profiles saved in NVS, step-grid tuning
// ESP-IDF v6, components: m5stack/m5unified, espressif/usb_host_cdc_acm (>=2.2.0)
// sdkconfig: CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE=2048

#include <M5Unified.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "freertos/queue.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/uac_host.h"
#include <math.h>

// ---------------- settings ----------------
#define QMX_CDC_IF  5               // fallback only; the CDC interface is auto-detected
#define ENC_A       GPIO_NUM_40
#define ENC_B       GPIO_NUM_41

static uint32_t ms() { return (uint32_t)(esp_timer_get_time() / 1000); }

// ---------------- radio profiles ----------------
struct radio_t {
    const char *name;
    uint32_t fmin, fmax;            // bands outside this range are hidden on the BAND screen
    const char *modes;              // tap-to-cycle mode list, Kenwood MD codes
};
static const radio_t radios[] = {
    {"QMX",   14000000, 29700000, "3216"},   // high-band QMX: 20/17/15/12/10m
    {"QMX+",   1800000, 54000000, "3216"},
    {"QDX",    3500000, 30000000, "6"},
    {"TS-480", 1800000, 54000000, "3214"},
};
static constexpr int NRADIOS = sizeof(radios) / sizeof(radios[0]);
static int radio_idx = 0;
static const radio_t &R() { return radios[radio_idx]; }

// ---------------- bands ----------------
struct band_t { const char *name; uint32_t lo, hi, def; };
static const band_t bands[] = {          // def = QRP CW calling frequencies; add 60m if you use it
    {"160m",  1800000,  2000000,  1836000},
    {"80m",   3500000,  4000000,  3560000},
    {"40m",   7000000,  7300000,  7030000},
    {"30m",  10100000, 10150000, 10116000},
    {"20m",  14000000, 14350000, 14060000},
    {"17m",  18068000, 18168000, 18086000},
    {"15m",  21000000, 21450000, 21060000},
    {"12m",  24890000, 24990000, 24906000},
    {"10m",  28000000, 29700000, 28060000},
    {"6m",   50000000, 54000000, 50060000},
};
static constexpr int NBANDS = sizeof(bands) / sizeof(bands[0]);
static uint32_t band_last[NBANDS];       // band stack: last frequency used on each band

static int band_of(uint32_t f)
{
    for (int i = 0; i < NBANDS; i++)
        if (f >= bands[i].lo && f <= bands[i].hi) return i;
    return -1;
}
static bool band_visible(int i) { return bands[i].lo >= R().fmin && bands[i].hi <= R().fmax; }

// ---------------- settings storage (NVS) ----------------
static void settings_load()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    nvs_handle_t h;
    if (nvs_open("qmxctl", NVS_READONLY, &h) == ESP_OK) {
        uint8_t r = 0;
        if (nvs_get_u8(h, "radio", &r) == ESP_OK && r < NRADIOS) radio_idx = r;
        nvs_close(h);
    }
}

static void settings_save()
{
    nvs_handle_t h;
    if (nvs_open("qmxctl", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "radio", (uint8_t)radio_idx);
        nvs_commit(h);
        nvs_close(h);
    }
}

// ---------------- encoder ----------------
static pcnt_unit_handle_t enc;

static void encoder_init()
{
    pcnt_unit_config_t ucfg = {};
    ucfg.low_limit  = -10000;
    ucfg.high_limit =  10000;
    ucfg.flags.accum_count = 1;
    ESP_ERROR_CHECK(pcnt_new_unit(&ucfg, &enc));

    pcnt_glitch_filter_config_t flt = {};
    flt.max_glitch_ns = 1000;
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(enc, &flt));

    pcnt_chan_config_t a = {}; a.edge_gpio_num = ENC_A; a.level_gpio_num = ENC_B;
    pcnt_chan_config_t b = {}; b.edge_gpio_num = ENC_B; b.level_gpio_num = ENC_A;
    pcnt_channel_handle_t ca, cb;
    ESP_ERROR_CHECK(pcnt_new_channel(enc, &a, &ca));
    ESP_ERROR_CHECK(pcnt_new_channel(enc, &b, &cb));
    pcnt_channel_set_edge_action(ca, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(ca, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(cb, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(cb, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_unit_add_watch_point(enc, ucfg.low_limit);
    pcnt_unit_add_watch_point(enc, ucfg.high_limit);

    gpio_set_pull_mode(ENC_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENC_B, GPIO_PULLUP_ONLY);

    ESP_ERROR_CHECK(pcnt_unit_enable(enc));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(enc));
    ESP_ERROR_CHECK(pcnt_unit_start(enc));
}

static int detents()
{
    int c = 0;
    pcnt_unit_get_count(enc, &c);
    return (c >= 0) ? c / 4 : (c - 3) / 4;
}

// ---------------- USB host / CAT ----------------
typedef struct { char s[96]; } reply_t;        // one complete CAT reply

static QueueHandle_t rxq;
static cdc_acm_dev_hdl_t qmx = nullptr;
static volatile bool qmx_gone = true;
static volatile bool connected = false;

static volatile uint16_t seen_vid = 0, seen_pid = 0;
static volatile int seen_ifaces = 0;
static volatile int cdc_if = -1;
static volatile int open_if = -1;             // interface actually opened

static void usb_lib_task(void *)
{
    while (true) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();
    }
}

static void on_new_dev(usb_device_handle_t h)
{
    const usb_device_desc_t *dd;
    if (usb_host_get_device_descriptor(h, &dd) == ESP_OK) {
        seen_vid = dd->idVendor;
        seen_pid = dd->idProduct;
    }
    const usb_config_desc_t *cfg;
    if (usb_host_get_active_config_descriptor(h, &cfg) == ESP_OK) {
        const uint8_t *p = (const uint8_t *)cfg;
        int off = 0, n = 0, first_cdc = -1;
        while (off + 1 < cfg->wTotalLength) {
            uint8_t len = p[off], type = p[off + 1];
            if (len == 0) break;
            if (type == 0x04 && p[off + 3] == 0) {            // interface descriptor, alt 0
                n++;
                if (p[off + 5] == 0x02 && first_cdc < 0)      // CDC Communications class
                    first_cdc = p[off + 2];
            }
            off += len;
        }
        seen_ifaces = n;
        cdc_if = first_cdc;
    }
}

static bool on_rx(const uint8_t *data, size_t len, void *)
{
    static reply_t cur;
    static size_t n = 0;
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\r' || c == '\n') continue;
        if (n < sizeof(cur.s) - 1) cur.s[n++] = c;
        if (c == ';') {
            cur.s[n] = 0;
            xQueueSend(rxq, &cur, 0);
            n = 0;
        }
    }
    return true;
}

static void on_event(const cdc_acm_host_dev_event_data_t *ev, void *)
{
    if (ev->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
        qmx_gone = true;
        connected = false;
        cdc_acm_host_close(ev->data.cdc_hdl);
    }
}

static bool qmx_open()
{
    cdc_acm_host_device_config_t dc = {};
    dc.connection_timeout_ms = 1000;
    dc.out_buffer_size = 128;
    dc.in_buffer_size = 128;
    dc.event_cb = on_event;
    dc.data_cb = on_rx;
    dc.user_arg = nullptr;
    int iface = (cdc_if >= 0) ? cdc_if : QMX_CDC_IF;
    cdc_acm_dev_hdl_t h = nullptr;
    if (cdc_acm_host_open(CDC_HOST_ANY_VID, CDC_HOST_ANY_PID, iface, &dc, &h) != ESP_OK) return false;
    qmx = h;
    open_if = iface;
    qmx_gone = false;
    connected = true;
    return true;
}

// Keeps (re)connecting in the background so the UI never blocks
static void usb_connect_task(void *)
{
    while (true) {
        if (!connected || qmx_gone) {
            qmx = nullptr;
            connected = false;
            if (!qmx_open()) vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

static void usb_init()
{
    usb_host_config_t hc = {};
    hc.skip_phy_setup = false;
    hc.intr_flags = ESP_INTR_FLAG_LEVEL1;
    ESP_ERROR_CHECK(usb_host_install(&hc));
    xTaskCreate(usb_lib_task, "usb_lib", 4096, nullptr, 10, nullptr);

    cdc_acm_host_driver_config_t drv = {};
    drv.driver_task_stack_size = 4096;
    drv.driver_task_priority = 10;
    drv.xCoreID = 0;
    drv.new_dev_cb = on_new_dev;
    ESP_ERROR_CHECK(cdc_acm_host_install(&drv));

    xTaskCreate(usb_connect_task, "usb_conn", 4096, nullptr, 5, nullptr);
}
static void pan_audio_init();

// Never send CR/LF: a carriage return drops the QMX port into its menu
static void cat_send(const char *cmd)
{
    cdc_acm_dev_hdl_t h = qmx;
    if (!h || !connected || qmx_gone) return;
    cdc_acm_host_data_tx_blocking(h, (const uint8_t *)cmd, strlen(cmd), 200);
}


// ---------------- PAN: USB audio (IQ) + FFT ----------------
// The QMX's sound card streams 48 kHz, 24-bit stereo. In IQ mode (Q91;) left/right are I/Q,
// so an FFT of them is the spectrum +/-24 kHz around the receiver's LO.
#define PAN_N        512                    // FFT size -> 93.75 Hz per bin
#define PAN_AVG      6                      // FFTs averaged per display update (~15 fps)
#define PAN_W        240                    // display columns
#define PAN_SWAP_IQ  0                      // set 1 if the spectrum is mirrored
#define PAN_LO_OFS   12000                  // Hz: dial freq minus IQ centre (QMX uses a ~12 kHz low IF)

struct uac_evt_t { bool driver; int event; uint8_t addr, iface; uac_host_device_handle_t h; };
static QueueHandle_t uac_q;
static volatile bool pan_active = false;    // PAN screen showing -> run the FFT
static volatile bool pan_audio_ok = false;  // audio stream running
static volatile bool pan_ready = false;     // new pan_px[] available for the UI
static float pan_px[PAN_W];                 // dB per display column (published)
static volatile int  pan_rate = 48000;

static float fft_re[PAN_N], fft_im[PAN_N], pan_win[PAN_N], pan_acc[PAN_N];
static float tw_c[PAN_N / 2], tw_s[PAN_N / 2];
static uint16_t bitrev[PAN_N];

static void fft_init()
{
    int bits = 0;
    while ((1 << bits) < PAN_N) bits++;
    for (int i = 0; i < PAN_N; i++) {
        int r = 0;
        for (int b = 0; b < bits; b++) if (i & (1 << b)) r |= 1 << (bits - 1 - b);
        bitrev[i] = r;
        pan_win[i] = 0.5f - 0.5f * cosf(2 * (float)M_PI * i / (PAN_N - 1));   // Hann
    }
    for (int i = 0; i < PAN_N / 2; i++) {
        tw_c[i] = cosf(2 * (float)M_PI * i / PAN_N);
        tw_s[i] = -sinf(2 * (float)M_PI * i / PAN_N);
    }
}

static void fft_run()                       // in place, radix-2
{
    for (int i = 0; i < PAN_N; i++) {
        int j = bitrev[i];
        if (j > i) {
            float t = fft_re[i]; fft_re[i] = fft_re[j]; fft_re[j] = t;
            t = fft_im[i]; fft_im[i] = fft_im[j]; fft_im[j] = t;
        }
    }
    for (int len = 2; len <= PAN_N; len <<= 1) {
        int half = len / 2, step = PAN_N / len;
        for (int i = 0; i < PAN_N; i += len) {
            for (int k = 0; k < half; k++) {
                float wr = tw_c[k * step], wi = tw_s[k * step];
                int a = i + k, b = a + half;
                float xr = fft_re[b] * wr - fft_im[b] * wi;
                float xi = fft_re[b] * wi + fft_im[b] * wr;
                fft_re[b] = fft_re[a] - xr; fft_im[b] = fft_im[a] - xi;
                fft_re[a] += xr;            fft_im[a] += xi;
            }
        }
    }
}

static void uac_driver_cb(uint8_t addr, uint8_t iface, const uac_host_driver_event_t ev, void *)
{
    uac_evt_t e = {true, (int)ev, addr, iface, nullptr};
    xQueueSend(uac_q, &e, 0);
}
static void uac_device_cb(uac_host_device_handle_t h, const uac_host_device_event_t ev, void *)
{
    uac_evt_t e = {false, (int)ev, 0, 0, h};
    xQueueSend(uac_q, &e, 0);
}

static inline int32_t pcm24(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] << 8 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 24);
}

static void uac_task(void *)
{
    uac_host_device_handle_t mic = nullptr;
    static uint8_t buf[4096];
    uint8_t carry[8];
    int carry_n = 0, sub = 3, ch = 2, n = 0, nacc = 0;

    while (true) {
        uac_evt_t e;
        if (xQueueReceive(uac_q, &e, pdMS_TO_TICKS(100)) != pdTRUE) continue;

        if (e.driver && e.event == UAC_HOST_DRIVER_EVENT_RX_CONNECTED && !mic) {
            uac_host_device_config_t dc = {};
            dc.addr = e.addr;
            dc.iface_num = e.iface;
            dc.buffer_size = 19200;
            dc.buffer_threshold = 2880;            // ~10 ms
            dc.callback = uac_device_cb;
            if (uac_host_device_open(&dc, &mic) != ESP_OK) { mic = nullptr; continue; }
            uac_host_dev_info_t info;
            uac_host_get_device_info(mic, &info);
            uac_host_dev_alt_param_t best = {};
            bool found = false;
            for (int alt = 1; alt <= info.iface_alt_num && !found; alt++) {
                uac_host_dev_alt_param_t p;
                if (uac_host_get_device_alt_param(mic, alt, &p) != ESP_OK) continue;
                if (p.channels != 2 || p.subframe_size != 3) continue;
                if (p.sample_freq_type > 0) {
                    for (int k = 0; k < p.sample_freq_type && k < UAC_FREQ_NUM_MAX; k++)
                        if (p.sample_freq[k] == 48000) found = true;
                } else if (48000 >= p.sample_freq_lower && 48000 <= p.sample_freq_upper) {
                    found = true;
                }
                if (found) best = p;
            }
            uac_host_stream_config_t sc = {};
            sc.channels = 2;
            sc.bit_resolution = best.bit_resolution;
            sc.sample_freq = 48000;
            if (!found || uac_host_device_start(mic, &sc) != ESP_OK) {
                uac_host_device_close(mic);
                mic = nullptr;
                continue;
            }
            sub = 3; ch = 2; carry_n = 0; n = 0; nacc = 0;
            pan_rate = 48000;
            pan_audio_ok = true;
        } else if (!e.driver && e.event == UAC_HOST_DRIVER_EVENT_DISCONNECTED) {
            uac_host_device_close(e.h);
            if (e.h == mic) { mic = nullptr; pan_audio_ok = false; }
        } else if (!e.driver && e.event == UAC_HOST_DEVICE_EVENT_RX_DONE && mic && e.h == mic) {
            int fb = sub * ch;
            uint32_t got = 0;
            while (uac_host_device_read(mic, buf + carry_n, sizeof(buf) - carry_n, &got, 0) == ESP_OK
                   && got > 0) {
                if (carry_n) memcpy(buf, carry, carry_n);
                int total = carry_n + (int)got;
                int nfr = total / fb;
                if (pan_active) {
                    for (int f = 0; f < nfr; f++) {
                        const uint8_t *q = buf + f * fb;
                        float i = pcm24(q) * (1.0f / 2147483648.0f);
                        float v = pcm24(q + sub) * (1.0f / 2147483648.0f);
#if PAN_SWAP_IQ
                        float t = i; i = v; v = t;
#endif
                        fft_re[n] = i * pan_win[n];
                        fft_im[n] = v * pan_win[n];
                        if (++n < PAN_N) continue;
                        n = 0;
                        fft_run();
                        for (int k = 0; k < PAN_N; k++)
                            pan_acc[k] += fft_re[k] * fft_re[k] + fft_im[k] * fft_im[k];
                        if (++nacc < PAN_AVG) continue;
                        nacc = 0;
                        if (!pan_ready) {
                            // fftshift + map 512 bins -> 240 columns (max of the bins per column)
                            for (int x = 0; x < PAN_W; x++) {
                                int b0 = x * PAN_N / PAN_W, b1 = (x + 1) * PAN_N / PAN_W;
                                float m = 0;
                                for (int b = b0; b < b1; b++) {
                                    float pw = pan_acc[(b + PAN_N / 2) % PAN_N];
                                    if (pw > m) m = pw;
                                }
                                pan_px[x] = 10 * log10f(m / PAN_AVG + 1e-20f);
                            }
                            pan_ready = true;
                        }
                        memset(pan_acc, 0, sizeof(pan_acc));
                    }
                }
                carry_n = total - nfr * fb;
                if (carry_n) memcpy(carry, buf + nfr * fb, carry_n);
            }
        }
    }
}

static void pan_audio_init()
{
    fft_init();
    uac_q = xQueueCreate(32, sizeof(uac_evt_t));
    uac_host_driver_config_t ud = {};
    ud.create_background_task = true;
    ud.task_priority = 9;
    ud.stack_size = 4096;
    ud.core_id = 0;
    ud.callback = uac_driver_cb;
    ESP_ERROR_CHECK(uac_host_install(&ud));
    xTaskCreatePinnedToCore(uac_task, "uac", 6144, nullptr, 8, nullptr, 1);
}

// ---------------- radio state ----------------
static const uint32_t steps[] = {10, 100, 1000, 10000};   // knob-press presets
static constexpr int NSTEPS = sizeof(steps) / sizeof(steps[0]);
static uint32_t step = 100;                               // any power of 10 via digit tap
#define ROUND_MAX_STEP 10000                              // don't round when picking 100 kHz+ steps

static uint32_t freq = 0;           // 0 = unknown until the radio reports it
static char mode = 0;               // Kenwood MD code, 0 = unknown
static bool split = false, tx = false;
static char vfo_rx = '0';           // receive VFO from IF: '0' = A, '1' = B
static uint32_t txfreq = 0;         // VFO B (transmit) frequency while in split
static bool expect_fb = false;      // an FB; query is outstanding
static bool tune_tx = false;        // in split: knob tunes TX (VFO B) instead of RX (VFO A)

static bool tx_target() { return split && tune_tx; }
static uint32_t last_tune = 0;      // last time WE changed something (holds off poll updates)

static const char *mode_name(char m)
{
    switch (m) {
        case '1': return "LSB";
        case '2': return "USB";
        case '3': return "CW";
        case '4': return "FM";
        case '5': return "AM";
        case '6': return "DIGI";
        case '7': return "CW-R";
        case '9': return "DIGI-R";
        default:  return "";
    }
}

// Sets the VFO we're listening on: B when in VFO B mode, otherwise A (also A in split)
static void send_freq()
{
    char cmd[20];
    bool use_b = (vfo_rx == '1' && !split);
    snprintf(cmd, sizeof(cmd), "%s%011lu;", use_b ? "FB" : "FA", (unsigned long)freq);
    cat_send(cmd);
    last_tune = ms();
    int b = band_of(freq);
    if (b >= 0) band_last[b] = freq;
}

// Split: set the transmit VFO (B)
static void send_tx()
{
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "FB%011lu;", (unsigned long)txfreq);
    cat_send(cmd);
    last_tune = ms();
}

// ---------------- display helpers ----------------
enum { SCR_VFO, SCR_PAN, SCR_CW, SCR_AUDIO, SCR_SETUP, NSCR };
static int screen = SCR_VFO;
static int radio_sel = 0;           // SETUP screen cursor

// Centred text at (centre + dx, centre + dy), clearing 'pad' pixels of width behind it
static void text_at(const char *s, int dx, int dy, const lgfx::IFont *font, uint16_t fg,
                    int pad, uint16_t bg = TFT_BLACK)
{
    auto &d = M5.Display;
    d.setTextDatum(middle_center);
    d.setTextColor(fg, bg);
    d.setFont(font);
    d.setTextPadding(pad);
    d.drawString(s, d.width() / 2 + dx, d.height() / 2 + dy);
    d.setTextPadding(0);
}

static void line(const char *s, int dy, const lgfx::IFont *font, uint16_t fg, int pad = 200)
{
    text_at(s, 0, dy, font, fg, pad);
}

static void draw_tx_ring()
{
    auto &d = M5.Display;
    int cx = d.width() / 2, cy = d.height() / 2;
    int r = (d.width() < d.height() ? d.width() : d.height()) / 2 - 1;
    d.fillArc(cx, cy, r, r - 6, 0, 360, tx ? TFT_RED : TFT_BLACK);
}

static void draw_page_dots()
{
    auto &d = M5.Display;
    int cx = d.width() / 2, y = d.height() - 22;
    for (int i = 0; i < NSCR; i++) {
        int x = cx + i * 14 - (NSCR - 1) * 7;
        d.fillCircle(x, y, 4, TFT_BLACK);
        if (i == screen) d.fillCircle(x, y, 4, TFT_LIGHTGREY);
        else             d.drawCircle(x, y, 4, TFT_DARKGREY);
    }
}

static void fmt_freq(char *buf, size_t n, uint32_t f)
{
    if (f == 0) snprintf(buf, n, "--.---.---");
    else snprintf(buf, n, "%lu.%03lu.%03lu",
                  (unsigned long)(f / 1000000), (unsigned long)(f / 1000 % 1000),
                  (unsigned long)(f % 1000));
}

// ----- VFO screen -----
static void vfo_status()
{
    char s[40];
    if (connected && split) {
        // Show whichever split frequency the big readout ISN'T showing
        char f[20];
        if (tune_tx) {
            fmt_freq(f, sizeof(f), freq);
            snprintf(s, sizeof(s), "RX %s", f);
            line(s, -72, &fonts::FreeSans9pt7b, TFT_GREEN, 160);
        } else {
            fmt_freq(f, sizeof(f), txfreq);
            snprintf(s, sizeof(s), "TX %s", f);
            line(s, -72, &fonts::FreeSans9pt7b, TFT_YELLOW, 160);
        }
    } else if (connected) {
        line(R().name, -72, &fonts::FreeSans9pt7b, TFT_GREEN, 160);
    } else if (seen_vid) {
        snprintf(s, sizeof(s), "%04X:%04X no CDC", seen_vid, seen_pid);
        line(s, -72, &fonts::FreeSans9pt7b, TFT_YELLOW, 160);
    } else {
        line("waiting for radio", -72, &fonts::FreeSans9pt7b, TFT_ORANGE, 160);
    }
}
// Top row: mode button (left) and VFO A/B/SPLIT button (right)
#define BTN_FILL      0x18E3        // dark grey
#define TOP_BTN_DY    (-40)
#define TOP_BTN_W     74
#define TOP_BTN_H     30
#define MODE_BTN_DX   (-42)
#define VFO_BTN_DX    42

static void draw_button(int dx, int dy, int w, int h, const char *label, uint16_t fg)
{
    auto &d = M5.Display;
    int x = d.width() / 2 + dx - w / 2, y = d.height() / 2 + dy - h / 2;
    d.fillRoundRect(x, y, w, h, 8, BTN_FILL);
    d.drawRoundRect(x, y, w, h, 8, TFT_DARKGREY);
    // long labels (e.g. DIGI-R, SPLIT) drop to the smaller font so they fit
    d.setFont(&fonts::FreeSansBold12pt7b);
    const lgfx::IFont *f = (d.textWidth(label) > w - 8)
        ? (const lgfx::IFont *)&fonts::FreeSansBold9pt7b
        : (const lgfx::IFont *)&fonts::FreeSansBold12pt7b;
    text_at(label, dx, dy, f, fg, 0, BTN_FILL);
}

static bool button_hit(int x, int y, int dx, int dy, int w, int h)
{
    auto &d = M5.Display;
    int bx = d.width() / 2 + dx, by = d.height() / 2 + dy;
    return abs(x - bx) <= w / 2 + 6 && abs(y - by) <= h / 2 + 8;
}

static void vfo_mode()
{
    draw_button(MODE_BTN_DX, TOP_BTN_DY, TOP_BTN_W, TOP_BTN_H,
                mode ? mode_name(mode) : "--", TFT_CYAN);
}
static bool mode_tap(int x, int y)
{
    return button_hit(x, y, MODE_BTN_DX, TOP_BTN_DY, TOP_BTN_W, TOP_BTN_H);
}

static void vfo_ab()
{
    const char *label = split ? "SPLIT" : (vfo_rx == '1' ? "VFO B" : "VFO A");
    uint16_t fg = split ? TFT_YELLOW : (vfo_rx == '1' ? TFT_ORANGE : TFT_WHITE);
    draw_button(VFO_BTN_DX, TOP_BTN_DY, TOP_BTN_W, TOP_BTN_H, label, fg);
}
static bool vfo_tap(int x, int y)
{
    return button_hit(x, y, VFO_BTN_DX, TOP_BTN_DY, TOP_BTN_W, TOP_BTN_H);
}
// Screen position of each digit of the frequency readout, for tap-to-select-step
static int dig_x0[12], dig_x1[12];
static uint32_t dig_val[12];
static int ndig = 0;

static void vfo_freq()
{
    auto &d = M5.Display;
    int cx = d.width() / 2, cy = d.height() / 2;
    // Big readout = whatever the knob tunes: RX normally, TX (yellow) in split TX-tune
    uint32_t shown = tx_target() ? txfreq : freq;
    uint16_t col = tx_target() ? TFT_YELLOW : TFT_GREEN;
    char s[20];
    fmt_freq(s, sizeof(s), shown);
    line(s, 0, &fonts::FreeSansBold18pt7b, col, 216);

    // Work out where each digit landed (proportional font, so measure prefixes)
    d.setFont(&fonts::FreeSansBold18pt7b);
    int left = cx - d.textWidth(s) / 2;
    int len = strlen(s);
    char pre[20];
    uint32_t place = 1;
    ndig = 0;
    for (int i = len - 1; i >= 0 && ndig < 12; i--) {
        if (s[i] < '0' || s[i] > '9') continue;
        memcpy(pre, s, i);     pre[i] = 0;     int x0 = left + d.textWidth(pre);
        memcpy(pre, s, i + 1); pre[i + 1] = 0; int x1 = left + d.textWidth(pre);
        dig_x0[ndig] = x0; dig_x1[ndig] = x1; dig_val[ndig] = place;
        ndig++;
        place *= 10;
    }

    // Underline the digit that matches the current step
    int uy = cy + 17;
    d.fillRect(cx - 110, uy, 220, 3, TFT_BLACK);
    for (int i = 0; i < ndig; i++)
        if (dig_val[i] == step)
            d.fillRect(dig_x0[i] + 1, uy, dig_x1[i] - dig_x0[i] - 2, 3, col);
}
// Bottom row: band button (left) and RIT button (right)
#define BOT_BTN_DY    42
#define BOT_BTN_W     68
#define BOT_BTN_H     30
#define BAND_BTN_DX   (-42)
#define RIT_BTN_DX    42

static void vfo_info()
{
    int b = band_of(freq);
    draw_button(BAND_BTN_DX, BOT_BTN_DY, BOT_BTN_W, BOT_BTN_H,
                b >= 0 ? bands[b].name : "--", TFT_WHITE);
}
static bool band_tap(int x, int y)
{
    return button_hit(x, y, BAND_BTN_DX, BOT_BTN_DY, BOT_BTN_W, BOT_BTN_H);
}

// RIT: off -> grey "RIT"; on -> orange offset, e.g. "R+120"; editing -> yellow outline
static bool rit_on = false;
static int  rit_ofs = 0;            // Hz
static bool rit_edit = false;       // knob is adjusting the RIT offset
static uint32_t rit_edit_t = 0;     // last activity while editing (auto-exit after 5 s)
#define RIT_STEP      10            // Hz per knob click
#define RIT_MAX       9990

static void vfo_rit()
{
    char s[24];
    uint16_t fg;
    if (rit_on || rit_edit) { snprintf(s, sizeof(s), "R%+d", rit_ofs); fg = TFT_ORANGE; }
    else                    { snprintf(s, sizeof(s), "RIT");           fg = TFT_DARKGREY; }
    draw_button(RIT_BTN_DX, BOT_BTN_DY, BOT_BTN_W, BOT_BTN_H, s, fg);
    if (rit_edit) {
        auto &d = M5.Display;
        int x = d.width() / 2 + RIT_BTN_DX - BOT_BTN_W / 2;
        int y = d.height() / 2 + BOT_BTN_DY - BOT_BTN_H / 2;
        d.drawRoundRect(x, y, BOT_BTN_W, BOT_BTN_H, 8, TFT_YELLOW);
        d.drawRoundRect(x + 1, y + 1, BOT_BTN_W - 2, BOT_BTN_H - 2, 7, TFT_YELLOW);
    }
}
static bool rit_tap(int x, int y)
{
    return button_hit(x, y, RIT_BTN_DX, BOT_BTN_DY, BOT_BTN_W, BOT_BTN_H);
}

// ----- SETUP screen -----
static void setup_conn()
{
    char s[40];
    if (connected) line("connected", -48, &fonts::FreeSans9pt7b, TFT_GREEN, 170);
    else           line("no radio", -48, &fonts::FreeSans9pt7b, TFT_ORANGE, 170);
    if (seen_vid) snprintf(s, sizeof(s), "USB %04X:%04X  if %d", seen_vid, seen_pid,
                           connected ? open_if : cdc_if);
    else          snprintf(s, sizeof(s), "USB --");
    line(s, -26, &fonts::FreeSans9pt7b, TFT_DARKGREY, 190);
}
static void setup_radio()
{
    char s[32];
    snprintf(s, sizeof(s), "<  %s  >", radios[radio_sel].name);
    line(s, 18, &fonts::FreeSansBold12pt7b,
         radio_sel == radio_idx ? TFT_GREEN : TFT_YELLOW, 190);
    line(radio_sel == radio_idx ? "active" : "press to select", 46,
         &fonts::FreeSans9pt7b, TFT_DARKGREY, 170);
}

// ---------------- CW screen ----------------
// Keyer speed: KS (standard CAT, not stored in the QMX's EEPROM).
// Keyer mode, CW center and passband are QMX menu items, read/written with MM.
// Setting CW center makes the QMX move the CW offset and sidetone to match (Auto-offset/tone).
#define WPM_MIN     5
#define WPM_MAX     50
#define CTR_STEP   50
#define CTR_MIN    500
#define CTR_MAX    900

static int  cw_wpm = 0;                     // 0 = unknown
static int  cw_ctr = 0;                    // Hz, 0 = unknown
static char cw_keyer[24] = "";
static uint32_t wpm_t = 0, ctr_set_t = 0;  // when WE last changed them (ignore stale replies)
static bool ctr_dirty = false;             // center changed here, not yet sent
static uint32_t ctr_dirty_t = 0;
static uint32_t cw_poll_t = 0;

static const char *path_keyer = "CW|CW Keyer|Keyer mode";
static char path_ctr[64] = "";             // found by the CW menu scan, e.g. "CW|CW center"

// CW filters. "CW|Choose filters" has one item per filter width, addressed by index:
//   MMCW|Choose filters|i?;  -> "MM7|6|50;"   (name = width in Hz)
//   MMCW|Choose filters|i;   -> "MMENABLED;" / "MMDISABLED;"
// The item that selects the active filter is found by scanning the CW menu for a name
// containing "passband" or "bandwidth".
#define MAX_FILT 16
static int  all_w[MAX_FILT], all_n = 0;     // every filter width the radio offers
static int  filt_w[MAX_FILT], filt_n = 0;   // the enabled ones = choices on the CW screen
static char path_bw[64] = "";               // e.g. "CW|CW passband"; empty = not found
enum { FS_START, FS_NAMES, FS_ENABLED, FS_BWPATH, FS_DONE };
static int  fscan = FS_START, fscan_i = 0;
static int  cw_bw = 0;                      // active filter width, 0 = unknown
static bool bw_dirty = false;
static uint32_t bw_dirty_t = 0, bw_set_t = 0;
static int  fscan_retry = 0;                // retries of the current scan step
static int  fscan_items = 0;                // CW menu items seen (diagnostics)
static uint32_t conn_t = 0;                 // when the radio connected (scan starts later)
static int  cw_poll_n = 0;                  // rotates center / keyer / filter reads

// One MM request in flight at a time, so we know what each reply belongs to
enum { MM_IDLE, MM_GET_KEYER, MM_GET_CTR, MM_GET_COMP, MM_GET_STV, MM_GET_BW,
       MM_FILT_NAME, MM_FILT_EN, MM_CW_ITEM, MM_SET };
static int mm_state = MM_IDLE;
static uint32_t mm_t = 0;

static void mm_send(int state, const char *cmd)
{
    cat_send(cmd);
    mm_state = state;
    mm_t = ms();
}

static bool icontains(const char *h, const char *n)
{
    size_t ln = strlen(n);
    for (; *h; h++) {
        size_t i = 0;
        while (i < ln && h[i] && tolower((unsigned char)h[i]) == tolower((unsigned char)n[i])) i++;
        if (i == ln) return true;
    }
    return false;
}

// Text after the last '|' of an MM reply, without the ';'
static void mm_name(const char *r, char *out, size_t n)
{
    const char *p = strrchr(r, '|');
    p = p ? p + 1 : r + 2;
    strncpy(out, p, n - 1);
    out[n - 1] = 0;
    char *semi = strchr(out, ';');
    if (semi) *semi = 0;
}

// ----- CW screen drawing -----
#define CW_BTN_W    44
#define CW_BTN_H    32
#define CW_BTN_DX   72
#define CW_SPD_DY   (-44)
#define CW_CTR_DY  (-4)
#define CW_BW_DY    36

static void cw_speed()
{
    char s[24];
    if (cw_wpm) snprintf(s, sizeof(s), "%d wpm", cw_wpm);
    else        snprintf(s, sizeof(s), "-- wpm");
    text_at(s, 0, CW_SPD_DY, &fonts::FreeSansBold9pt7b, TFT_WHITE, 96);
}
static void cw_ctr_draw()
{
    char s[24];
    if (fscan == FS_DONE && !path_ctr[0]) snprintf(s, sizeof(s), "Center n/a");
    else if (cw_ctr) snprintf(s, sizeof(s), "Center %d", cw_ctr);
    else             snprintf(s, sizeof(s), "Center --");
    text_at(s, 0, CW_CTR_DY, &fonts::FreeSansBold9pt7b,
            ctr_dirty ? TFT_YELLOW : TFT_WHITE, 96);
}
static void cw_keyer_draw()
{
    char s[40];
    snprintf(s, sizeof(s), "Keyer: %s", cw_keyer[0] ? cw_keyer : "--");
    text_at(s, 0, 68, &fonts::FreeSans9pt7b, TFT_CYAN, 180);
}
static void cw_bw_draw()
{
    char s[24];
    if (fscan != FS_DONE)            snprintf(s, sizeof(s), "Filter ...");
    else if (!path_bw[0] || !filt_n) snprintf(s, sizeof(s), "Filter n/a");
    else if (cw_bw)                  snprintf(s, sizeof(s), "Filter %d", cw_bw);
    else                             snprintf(s, sizeof(s), "Filter --");
    text_at(s, 0, CW_BW_DY, &fonts::FreeSansBold9pt7b,
            bw_dirty ? TFT_YELLOW : TFT_WHITE, 96);

    // Diagnostics while something is missing: filters found/enabled, CW menu items seen
    char dg[40] = "";
    if (fscan == FS_DONE && (!path_bw[0] || !path_ctr[0] || !filt_n))
        snprintf(dg, sizeof(dg), "f%d/%d m%d %c%c", all_n, filt_n, fscan_items,
                 path_bw[0] ? 'P' : '-', path_ctr[0] ? 'C' : '-');
    text_at(dg, 0, 86, &fonts::FreeSans9pt7b, TFT_DARKGREY, 150);
}
static void cw_draw()
{
    line("CW", -80, &fonts::FreeSans9pt7b, TFT_DARKGREY, 60);
    const int rows[] = {CW_SPD_DY, CW_CTR_DY, CW_BW_DY};
    for (int i = 0; i < 3; i++) {
        draw_button(-CW_BTN_DX, rows[i], CW_BTN_W, CW_BTN_H, "-", TFT_WHITE);
        draw_button( CW_BTN_DX, rows[i], CW_BTN_W, CW_BTN_H, "+", TFT_WHITE);
    }
    cw_speed();
    cw_ctr_draw();
    cw_bw_draw();
    cw_keyer_draw();
}

// ----- CW actions -----
static void cw_speed_adj(int d)
{
    if (cw_wpm == 0 || d == 0) return;
    cw_wpm += d;
    if (cw_wpm < WPM_MIN) cw_wpm = WPM_MIN;
    if (cw_wpm > WPM_MAX) cw_wpm = WPM_MAX;
    char c[24];
    snprintf(c, sizeof(c), "KS%03d;", cw_wpm);
    cat_send(c);
    wpm_t = ms();
    cw_speed();
}

// CW center is a menu item stored in the QMX's EEPROM, so send it only once you stop pressing
static void cw_ctr_adj(int d)
{
    if (cw_ctr == 0 || !path_ctr[0]) return;
    cw_ctr = ((cw_ctr + CTR_STEP / 2) / CTR_STEP) * CTR_STEP;   // onto the 50 Hz grid
    cw_ctr += d * CTR_STEP;
    if (cw_ctr < CTR_MIN) cw_ctr = CTR_MIN;
    if (cw_ctr > CTR_MAX) cw_ctr = CTR_MAX;
    ctr_dirty = true;
    ctr_dirty_t = ms();
    cw_ctr_draw();
}

// Step through the enabled filter widths; written to the radio once you stop pressing
static void cw_bw_adj(int d)
{
    if (fscan != FS_DONE || !path_bw[0] || filt_n == 0) return;
    int i = 0;
    while (i < filt_n && filt_w[i] != cw_bw) i++;
    if (i == filt_n) i = (d > 0) ? -1 : filt_n;      // current width not in list
    i += d;
    if (i < 0)       i = 0;
    if (i >= filt_n) i = filt_n - 1;
    if (filt_w[i] == cw_bw) return;
    cw_bw = filt_w[i];
    bw_dirty = true;
    bw_dirty_t = ms();
    cw_bw_draw();
}

static void cw_tap(int x, int y)
{
    if      (button_hit(x, y, -CW_BTN_DX, CW_SPD_DY,  CW_BTN_W, CW_BTN_H)) cw_speed_adj(-1);
    else if (button_hit(x, y,  CW_BTN_DX, CW_SPD_DY,  CW_BTN_W, CW_BTN_H)) cw_speed_adj(+1);
    else if (button_hit(x, y, -CW_BTN_DX, CW_CTR_DY, CW_BTN_W, CW_BTN_H)) cw_ctr_adj(-1);
    else if (button_hit(x, y,  CW_BTN_DX, CW_CTR_DY, CW_BTN_W, CW_BTN_H)) cw_ctr_adj(+1);
    else if (button_hit(x, y, -CW_BTN_DX, CW_BW_DY,   CW_BTN_W, CW_BTN_H)) cw_bw_adj(-1);
    else if (button_hit(x, y,  CW_BTN_DX, CW_BW_DY,   CW_BTN_W, CW_BTN_H)) cw_bw_adj(+1);
}

// ---------------- AUDIO screen ----------------
// Volume: AG (AF gain, 0.25 dB units, 0-799).  RF gain: RG (dB, applies to the current band).
// Compression: QMX menu item SSB|Microphone|Compression (dB), via MM; SSB transmit only.
// Sidetone volume: QMX menu item CW|Sidetone volume, via MM.
#define AF_STEP     4               // 1 dB per click
#define AF_MAX      799
#define RF_MAX      99              // range not documented
#define COMP_MAX    30              // manual allows more, but calls very high values silly

static int  af_gain = -1, rf_gain = -1, comp = -1;     // -1 = unknown
static bool comp_na = false;                           // radio rejected the menu path
static uint32_t af_t = 0, rf_t = 0, comp_set_t = 0;    // when WE last changed them
static bool comp_dirty = false;                        // changed here, not yet sent
static uint32_t comp_dirty_t = 0;
static uint32_t au_poll_t = 0;
static const char *path_comp = "SSB|Microphone|Compression";

// Sidetone volume: QMX menu item CW|Sidetone volume (default 70; range not documented)
#define STV_STEP    5
#define STV_MAX     100
static int  stv = -1;                                  // -1 = unknown
static bool stv_na = false;
static bool stv_dirty = false;
static uint32_t stv_dirty_t = 0, stv_set_t = 0;
static int  au_poll_n = 0;                             // alternates compression / sidetone reads
static const char *path_stv = "CW|Sidetone volume";

#define AU_BTN_W    44
#define AU_BTN_H    30
#define AU_BTN_DX   68
#define AU_VOL_DY   (-54)
#define AU_RF_DY    (-18)
#define AU_COMP_DY  18
#define AU_STV_DY   54

static void au_af()
{
    char s[24];
    if (af_gain < 0) snprintf(s, sizeof(s), "Vol --");
    else             snprintf(s, sizeof(s), "Vol %d dB", (af_gain + 2) / 4);
    text_at(s, 0, AU_VOL_DY, &fonts::FreeSansBold9pt7b, TFT_WHITE, 96);
}
static void au_rf()
{
    char s[24];
    if (rf_gain < 0) snprintf(s, sizeof(s), "RF --");
    else             snprintf(s, sizeof(s), "RF %d dB", rf_gain);
    text_at(s, 0, AU_RF_DY, &fonts::FreeSansBold9pt7b, TFT_WHITE, 96);
}
static void au_comp()
{
    char s[24];
    if (comp_na)       snprintf(s, sizeof(s), "Comp n/a");
    else if (comp < 0) snprintf(s, sizeof(s), "Comp --");
    else               snprintf(s, sizeof(s), "Comp %d dB", comp);
    text_at(s, 0, AU_COMP_DY, &fonts::FreeSansBold9pt7b,
            comp_dirty ? TFT_YELLOW : TFT_WHITE, 96);
}
static void au_stv()
{
    char s[24];
    if (stv_na)       snprintf(s, sizeof(s), "Sidetone n/a");
    else if (stv < 0) snprintf(s, sizeof(s), "Sidetone --");
    else              snprintf(s, sizeof(s), "Sidetone %d", stv);
    text_at(s, 0, AU_STV_DY, &fonts::FreeSansBold9pt7b,
            stv_dirty ? TFT_YELLOW : TFT_WHITE, 104);
}
static void au_draw()
{
    line("AUDIO", -88, &fonts::FreeSans9pt7b, TFT_DARKGREY, 70);
    const int rows[] = {AU_VOL_DY, AU_RF_DY, AU_COMP_DY, AU_STV_DY};
    for (int i = 0; i < 4; i++) {
        draw_button(-AU_BTN_DX, rows[i], AU_BTN_W, AU_BTN_H, "-", TFT_WHITE);
        draw_button( AU_BTN_DX, rows[i], AU_BTN_W, AU_BTN_H, "+", TFT_WHITE);
    }
    au_af();
    au_rf();
    au_comp();
    au_stv();
}

static void af_adj(int d)
{
    if (af_gain < 0 || d == 0) return;
    af_gain += d * AF_STEP;
    if (af_gain < 0)      af_gain = 0;
    if (af_gain > AF_MAX) af_gain = AF_MAX;
    char c[24];
    snprintf(c, sizeof(c), "AG0%03d;", af_gain);
    cat_send(c);
    af_t = ms();
    au_af();
}
static void rf_adj(int d)
{
    if (rf_gain < 0 || d == 0) return;
    rf_gain += d;
    if (rf_gain < 0)      rf_gain = 0;
    if (rf_gain > RF_MAX) rf_gain = RF_MAX;
    char c[24];
    snprintf(c, sizeof(c), "RG%d;", rf_gain);
    cat_send(c);
    rf_t = ms();
    au_rf();
}
// Menu item: written once you stop pressing (see cw_service)
static void comp_adj(int d)
{
    if (comp < 0 || d == 0) return;
    comp += d;
    if (comp < 0)        comp = 0;
    if (comp > COMP_MAX) comp = COMP_MAX;
    comp_dirty = true;
    comp_dirty_t = ms();
    au_comp();
}
// Menu item: written once you stop pressing (see cw_service)
static void stv_adj(int d)
{
    if (stv < 0 || d == 0) return;
    stv += d * STV_STEP;
    if (stv < 0)       stv = 0;
    if (stv > STV_MAX) stv = STV_MAX;
    stv_dirty = true;
    stv_dirty_t = ms();
    au_stv();
}

static void au_tap(int x, int y)
{
    if      (button_hit(x, y, -AU_BTN_DX, AU_VOL_DY,  AU_BTN_W, AU_BTN_H)) af_adj(-1);
    else if (button_hit(x, y,  AU_BTN_DX, AU_VOL_DY,  AU_BTN_W, AU_BTN_H)) af_adj(+1);
    else if (button_hit(x, y, -AU_BTN_DX, AU_RF_DY,   AU_BTN_W, AU_BTN_H)) rf_adj(-1);
    else if (button_hit(x, y,  AU_BTN_DX, AU_RF_DY,   AU_BTN_W, AU_BTN_H)) rf_adj(+1);
    else if (button_hit(x, y, -AU_BTN_DX, AU_COMP_DY, AU_BTN_W, AU_BTN_H)) comp_adj(-1);
    else if (button_hit(x, y,  AU_BTN_DX, AU_COMP_DY, AU_BTN_W, AU_BTN_H)) comp_adj(+1);
    else if (button_hit(x, y, -AU_BTN_DX, AU_STV_DY,  AU_BTN_W, AU_BTN_H)) stv_adj(-1);
    else if (button_hit(x, y,  AU_BTN_DX, AU_STV_DY,  AU_BTN_W, AU_BTN_H)) stv_adj(+1);
}

// Filter scan: called when a level ends ("?;" reply or no answer)
static void fscan_end()
{
    fscan_i = 0;
    if (fscan == FS_NAMES)        fscan = all_n ? FS_ENABLED : FS_BWPATH;
    else if (fscan == FS_ENABLED) fscan = FS_BWPATH;
    else                          fscan = FS_DONE;
    if (fscan == FS_DONE && screen == SCR_CW) { cw_bw_draw(); cw_ctr_draw(); }
}

static void fscan_step()
{
    char cmd[64];
    switch (fscan) {
        case FS_NAMES:
            snprintf(cmd, sizeof(cmd), "MMCW|Choose filters|%d?;", fscan_i);
            mm_send(MM_FILT_NAME, cmd);
            break;
        case FS_ENABLED:
            if (fscan_i >= all_n) { fscan_end(); return; }
            snprintf(cmd, sizeof(cmd), "MMCW|Choose filters|%d;", fscan_i);
            mm_send(MM_FILT_EN, cmd);
            break;
        case FS_BWPATH:
            if (fscan_i > 40) { fscan_end(); return; }
            snprintf(cmd, sizeof(cmd), "MMCW|%d?;", fscan_i);
            mm_send(MM_CW_ITEM, cmd);
            break;
    }
}

// MM replies, routed here while a request is outstanding
static void mm_reply(const char *r)
{
    int st = mm_state;
    fscan_retry = 0;
    mm_state = MM_IDLE;
    bool ok = strncmp(r, "MM", 2) == 0;
    switch (st) {
        case MM_GET_KEYER:
            if (ok) {
                strncpy(cw_keyer, r + 2, sizeof(cw_keyer) - 1);
                cw_keyer[sizeof(cw_keyer) - 1] = 0;
                char *semi = strchr(cw_keyer, ';');
                if (semi) *semi = 0;
                if (screen == SCR_CW) cw_keyer_draw();
            }
            break;
        case MM_GET_CTR:
            if (ok && !ctr_dirty && ms() - ctr_set_t > 1500) {
                int t = atoi(r + 2);
                if (t > 0 && t != cw_ctr) {
                    cw_ctr = t;
                    if (screen == SCR_CW) cw_ctr_draw();
                }
            }
            break;
        case MM_GET_COMP:
            if (!ok) {
                if (!comp_na) { comp_na = true; if (screen == SCR_AUDIO) au_comp(); }
            } else if (!comp_dirty && ms() - comp_set_t > 1500) {
                int v = atoi(r + 2);
                comp_na = false;
                if (v != comp) {
                    comp = v;
                    if (screen == SCR_AUDIO) au_comp();
                }
            }
            break;
        case MM_GET_STV:
            if (!ok) {
                if (!stv_na) { stv_na = true; if (screen == SCR_AUDIO) au_stv(); }
            } else if (!stv_dirty && ms() - stv_set_t > 1500) {
                int v = atoi(r + 2);
                stv_na = false;
                if (v != stv) {
                    stv = v;
                    if (screen == SCR_AUDIO) au_stv();
                }
            }
            break;
        case MM_FILT_NAME:                  // "MM7|6|50;" -> width 50
            if (!ok || all_n >= MAX_FILT) { fscan_end(); break; }
            {
                char nm[16];
                mm_name(r, nm, sizeof(nm));
                all_w[all_n++] = atoi(nm);
                fscan_i++;
            }
            break;
        case MM_FILT_EN:                    // "MMENABLED;" / "MMDISABLED;"
            if (ok && strstr(r, "ENABLED") && !strstr(r, "DISABLED") && filt_n < MAX_FILT)
                filt_w[filt_n++] = all_w[fscan_i];
            fscan_i++;
            break;
        case MM_CW_ITEM:                    // looking for "CW passband" and "CW center"
            if (!ok) { fscan_end(); cw_poll_t = 0; break; }
            {
                char nm[40];
                mm_name(r, nm, sizeof(nm));
                fscan_items++;
                if (!icontains(nm, "choose") &&
                    (icontains(nm, "passband") || icontains(nm, "bandwidth")))
                    snprintf(path_bw, sizeof(path_bw), "CW|%s", nm);
                else if (icontains(nm, "center") || icontains(nm, "centre")) {
                    // QMX reports this one as "CW center(": drop anything from '('
                    // and trailing spaces; "CW|CW center" is the path that works
                    char *paren = strchr(nm, '(');
                    if (paren) *paren = 0;
                    size_t L = strlen(nm);
                    while (L && nm[L - 1] == ' ') nm[--L] = 0;
                    snprintf(path_ctr, sizeof(path_ctr), "CW|%s", nm);
                }
                fscan_i++;
                if (path_bw[0] && path_ctr[0]) { fscan_end(); cw_poll_t = 0; }   // found both
            }
            break;
        case MM_GET_BW:
            if (ok && !bw_dirty && ms() - bw_set_t > 1500) {
                int w = atoi(r + 2);
                if (w > 0 && w != cw_bw) {
                    cw_bw = w;
                    if (screen == SCR_CW) cw_bw_draw();
                }
            }
            break;
        default:
            break;                          // MM_SET: nothing to do with the reply
    }
}

// Runs every loop: deferred menu writes, and polling while the CW or AUDIO screen is up
static void cw_service(uint32_t now)
{
    if (!connected) {
        mm_state = MM_IDLE;
        if (fscan != FS_START) {            // rescan filters on the next connection
            fscan = FS_START;
            all_n = filt_n = 0;
            path_bw[0] = 0;
            path_ctr[0] = 0;
            cw_bw = 0;
            cw_ctr = 0;
        }
        return;
    }
    if (mm_state != MM_IDLE) {              // waiting for a reply
        if (now - mm_t > 1000) {
            int st = mm_state;
            mm_state = MM_IDLE;
            bool scan = (st == MM_FILT_NAME || st == MM_FILT_EN || st == MM_CW_ITEM);
            if (scan && ++fscan_retry > 2) {     // no answer 3 times: skip on
                fscan_retry = 0;
                if (st == MM_FILT_EN) fscan_i++;
                else                  fscan_end();
            }
            // otherwise fscan_step() repeats the same request on the next pass
        }
        return;
    }

    if (bw_dirty && now - bw_dirty_t > 800) {
        char cmd[96];
        snprintf(cmd, sizeof(cmd), "MM%s=%d;", path_bw, cw_bw);
        mm_send(MM_SET, cmd);
        cat_send("MU;");
        bw_dirty = false;
        bw_set_t = now;
        if (screen == SCR_CW) cw_bw_draw();
        return;
    }

    if (fscan == FS_START) {                // read the filter list once per connection
        if (now - conn_t < 1500) return;    // let the radio settle after connecting
        fscan = FS_NAMES;
        fscan_retry = fscan_items = 0;
        fscan_i = 0;
        all_n = filt_n = 0;
        path_bw[0] = 0;
        path_ctr[0] = 0;
        if (screen == SCR_CW) { cw_bw_draw(); cw_ctr_draw(); }
    }
    if (fscan != FS_DONE) {
        fscan_step();
        return;
    }

    if (ctr_dirty && now - ctr_dirty_t > 800) {
        char cmd[160];
        snprintf(cmd, sizeof(cmd), "MM%s=%d;", path_ctr, cw_ctr);
        mm_send(MM_SET, cmd);
        cat_send("MU;");                    // make the menu change take effect now
        ctr_dirty = false;
        ctr_set_t = now;
        if (screen == SCR_CW) cw_ctr_draw();
        return;
    }

    if (comp_dirty && now - comp_dirty_t > 800) {
        char cmd[96];
        snprintf(cmd, sizeof(cmd), "MM%s=%d;", path_comp, comp);
        mm_send(MM_SET, cmd);
        cat_send("MU;");
        comp_dirty = false;
        comp_set_t = now;
        if (screen == SCR_AUDIO) au_comp();
        return;
    }

    if (stv_dirty && now - stv_dirty_t > 800) {
        char cmd[96];
        snprintf(cmd, sizeof(cmd), "MM%s=%d;", path_stv, stv);
        mm_send(MM_SET, cmd);
        cat_send("MU;");
        stv_dirty = false;
        stv_set_t = now;
        if (screen == SCR_AUDIO) au_stv();
        return;
    }

    if (screen == SCR_AUDIO && now - au_poll_t > 750) {
        au_poll_t = now;
        char cmd[96];
        if (au_poll_n++ % 2 == 0) {
            if (now - af_t > 800) cat_send("AG0;");
            if (now - rf_t > 800) cat_send("RG;");
            snprintf(cmd, sizeof(cmd), "MM%s;", path_comp);
            mm_send(MM_GET_COMP, cmd);
        } else {
            snprintf(cmd, sizeof(cmd), "MM%s;", path_stv);
            mm_send(MM_GET_STV, cmd);
        }
        return;
    }

    if (screen == SCR_CW && now - cw_poll_t > 1500) {
        cw_poll_t = now;
        if (now - wpm_t > 800) cat_send("KS;");
        char cmd[160];
        int which = cw_poll_n++ % 3;
        if (which == 2 && !path_bw[0])  which = 1;
        if (which == 0 && !path_ctr[0]) which = 1;
        if (which == 0) {
            snprintf(cmd, sizeof(cmd), "MM%s;", path_ctr);
            mm_send(MM_GET_CTR, cmd);
        } else if (which == 1) {
            snprintf(cmd, sizeof(cmd), "MM%s;", path_keyer);
            mm_send(MM_GET_KEYER, cmd);
        } else {
            snprintf(cmd, sizeof(cmd), "MM%s;", path_bw);
            mm_send(MM_GET_BW, cmd);
        }
    }
}

// ---------------- PAN screen ----------------
#define PAN_Y      74                       // spectrum area top (screen y)
#define PAN_H      84                       // spectrum area height
#define PAN_RANGE  50.0f                    // dB shown top to bottom
static M5Canvas *pan_spr = nullptr;
static float pan_floor = -120;              // auto-tracked noise floor (dB)

static float pan_hz_per_px() { return (float)pan_rate / PAN_W; }
static uint32_t step_freq(uint32_t f0, int delta);

static void pan_header()
{
    char s[32], f[20];
    fmt_freq(f, sizeof(f), freq);
    text_at(f, 0, -78, &fonts::FreeSansBold12pt7b, TFT_GREEN, 200);
    char st[16];
    if (step >= 1000) snprintf(st, sizeof(st), "%lu kHz", (unsigned long)(step / 1000));
    else              snprintf(st, sizeof(st), "%lu Hz", (unsigned long)step);
    snprintf(s, sizeof(s), "%s   step %s", mode_name(mode), st);
    text_at(s, 0, -56, &fonts::FreeSans9pt7b, TFT_CYAN, 160);
}

// Edge labels relative to the dial frequency, e.g. "-36k" ... "+12k"
static void pan_scale()
{
    int lo_k = (int)lroundf((-pan_rate / 2 - PAN_LO_OFS) / 1000.0f);
    int hi_k = (int)lroundf(( pan_rate / 2 - PAN_LO_OFS) / 1000.0f);
    char s[12];
    snprintf(s, sizeof(s), "%+dk", lo_k);
    text_at(s, -78, 48, &fonts::FreeSans9pt7b, TFT_DARKGREY, 44);
    snprintf(s, sizeof(s), "%+dk", hi_k);
    text_at(s,  78, 48, &fonts::FreeSans9pt7b, TFT_DARKGREY, 44);
    text_at(pan_audio_ok ? "" : "no audio", 0, 48, &fonts::FreeSans9pt7b, TFT_ORANGE, 90);
}

// Column of the dial frequency: right of the IQ centre by PAN_LO_OFS
static int pan_marker_x() { return PAN_W / 2 + (int)lroundf(PAN_LO_OFS / pan_hz_per_px()); }

static void pan_draw_spectrum()
{
    if (!pan_spr) return;
    float px[PAN_W];
    memcpy(px, pan_px, sizeof(px));
    pan_ready = false;

    // Noise floor: slowly track the 20th-percentile column level
    float sorted[PAN_W];
    memcpy(sorted, px, sizeof(sorted));
    int k = PAN_W / 5;
    for (int i = 0; i <= k; i++)                     // partial selection sort
        for (int j = i + 1; j < PAN_W; j++)
            if (sorted[j] < sorted[i]) { float t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t; }
    float nf = sorted[k];
    pan_floor = (pan_floor < -119) ? nf : pan_floor * 0.9f + nf * 0.1f;
    float bottom = pan_floor - 5;

    auto &c = *pan_spr;
    c.fillSprite(TFT_BLACK);
    for (int g = 1; g < 5; g++) c.drawFastHLine(0, g * PAN_H / 5, PAN_W, 0x2104);   // 10 dB grid
    int mx = pan_marker_x();
    c.drawFastVLine(mx, 0, PAN_H, 0x7800);                                           // tuned freq
    int prev_y = -1;
    for (int x = 0; x < PAN_W; x++) {
        float v = (px[x] - bottom) / PAN_RANGE;
        if (v < 0) v = 0;
        if (v > 1) v = 1;
        int y = PAN_H - 1 - (int)(v * (PAN_H - 1));
        c.drawFastVLine(x, y, PAN_H - y, 0x0320);                                    // fill
        if (prev_y >= 0) c.drawLine(x - 1, prev_y, x, y, TFT_GREEN);                 // trace
        prev_y = y;
    }
    c.pushSprite(0, PAN_Y);
}

static void pan_draw()
{
    line("PAN", -98, &fonts::FreeSans9pt7b, TFT_DARKGREY, 60);
    pan_header();
    pan_scale();
    pan_ready = false;
}

static void pan_enter()
{
    if (!pan_spr) {
        pan_spr = new M5Canvas(&M5.Display);
        if (!pan_spr->createSprite(PAN_W, PAN_H)) { delete pan_spr; pan_spr = nullptr; }
    }
    pan_floor = -120;
    pan_active = true;
    cat_send("Q91;");                         // IQ mode on (not saved by the QMX)
}

static void pan_leave()
{
    pan_active = false;
    cat_send("Q90;");                         // back to normal audio
    if (pan_spr) { pan_spr->deleteSprite(); delete pan_spr; pan_spr = nullptr; }
}

static void pan_tune(int d)
{
    if (freq == 0 || d == 0) return;
    freq = step_freq(freq, d);
    send_freq();
    pan_header();
}

static void pan_next_step()
{
    int i = 0;
    while (i < NSTEPS && steps[i] <= step) i++;
    step = (i < NSTEPS) ? steps[i] : steps[0];
    pan_header();
}

// Tap on the spectrum: tune there (rounded to the step)
static void pan_tap(int x, int y)
{
    if (freq == 0 || y < PAN_Y || y >= PAN_Y + PAN_H) return;
    int64_t f = (int64_t)freq + (int64_t)lroundf((x - pan_marker_x()) * pan_hz_per_px());
    int64_t st = step;
    f = ((f + st / 2) / st) * st;
    if (f < 100000) return;
    freq = (uint32_t)f;
    send_freq();
    pan_header();
}

static void draw_screen()
{
    M5.Display.fillScreen(TFT_BLACK);
    switch (screen) {
        case SCR_VFO:
            vfo_status(); vfo_mode(); vfo_ab(); vfo_freq(); vfo_info(); vfo_rit();
            break;
        case SCR_PAN:
            pan_draw();
            break;
        case SCR_CW:
            cw_draw();
            break;
        case SCR_AUDIO:
            au_draw();
            break;
        case SCR_SETUP:
            line("SETUP", -72, &fonts::FreeSans9pt7b, TFT_DARKGREY, 100);
            setup_conn();
            line("Radio", -2, &fonts::FreeSans9pt7b, TFT_LIGHTGREY, 100);
            setup_radio();
            break;
    }
    draw_page_dots();
    draw_tx_ring();
}

static void goto_screen(int s)
{
    int prev = screen;
    screen = (s + NSCR) % NSCR;
    if (prev == SCR_PAN && screen != SCR_PAN) pan_leave();
    if (prev != SCR_PAN && screen == SCR_PAN) pan_enter();
    rit_edit = false;
    if (screen == SCR_SETUP) radio_sel = radio_idx;
    if (screen == SCR_CW) {
        cw_poll_t = 0;                          // refresh values straight away
        if (fscan == FS_DONE && (!path_bw[0] || !path_ctr[0] || !filt_n))
            fscan = FS_START;                   // last scan came up short: try again
    }
    if (screen == SCR_AUDIO) au_poll_t = 0;
    draw_screen();
}

// ---------------- actions ----------------
// Tune on the step grid: the first click from an off-grid frequency lands on the grid
static uint32_t step_freq(uint32_t f0, int delta)
{
    int64_t st = step;
    int64_t f = f0;
    int64_t base = (delta > 0) ? (f / st) * st : ((f + st - 1) / st) * st;
    f = base + (int64_t)delta * st;
    if (f < 100000)   f = 100000;
    if (f > 60000000) f = 60000000;
    return (uint32_t)f;
}

static void tune(int delta)
{
    if (delta == 0) return;
    if (tx_target()) {                              // split: tuning the TX frequency
        if (txfreq == 0) return;
        txfreq = step_freq(txfreq, delta);
        send_tx();
        vfo_freq();
        return;
    }
    if (freq == 0) return;
    int old_band = band_of(freq);
    freq = step_freq(freq, delta);
    send_freq();
    vfo_freq();
    if (band_of(freq) != old_band) vfo_info();
    if (split) vfo_status();
}

// Set the step; when it gets bigger (up to ROUND_MAX_STEP), round the frequency to it
static void set_step(uint32_t st)
{
    uint32_t old = step;
    step = st;
    if (st > old && st <= ROUND_MAX_STEP) {
        if (tx_target()) {
            if (txfreq != 0) {
                uint32_t f = ((txfreq + st / 2) / st) * st;
                if (f != txfreq) { txfreq = f; send_tx(); }
            }
        } else if (freq != 0) {
            uint32_t f = ((freq + st / 2) / st) * st;
            if (f != freq) { freq = f; send_freq(); }
        }
    }
    vfo_freq();        // also moves the underline
    vfo_info();
}

// Knob press: next preset above the current step, wrapping to the smallest
static void next_step()
{
    for (int i = 0; i < NSTEPS; i++)
        if (steps[i] > step) { set_step(steps[i]); return; }
    set_step(steps[0]);
}

// Tap on the frequency readout: the nearest digit becomes the step. Returns false if
// the tap wasn't on the readout.
static bool freq_tap(int x, int y)
{
    int cy = M5.Display.height() / 2;
    if (ndig == 0 || y < cy - 24 || y > cy + 26) return false;
    int best = -1, bd = 1 << 30;
    for (int i = 0; i < ndig; i++) {
        int dd = abs(x - (dig_x0[i] + dig_x1[i]) / 2);
        if (dd < bd) { bd = dd; best = i; }
    }
    if (best < 0 || bd > 30) return false;
    set_step(dig_val[best]);
    return true;
}

static void cycle_mode()
{
    const char *list = R().modes;
    int n = strlen(list);
    if (n < 2 || mode == 0) return;
    const char *p = strchr(list, mode);
    char next = p ? list[((p - list) + 1) % n] : list[0];
    char cmd[8];
    snprintf(cmd, sizeof(cmd), "MD%c;", next);
    cat_send(cmd);
    mode = next;
    last_tune = ms();
    vfo_mode();
}

// VFO button: A -> B -> SPLIT -> A   (QMX: FR0 / FR1 / FR2)
static void cycle_vfo()
{
    int state = split ? 2 : (vfo_rx == '1' ? 1 : 0);
    int next = (state + 1) % 3;
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "FR%d;", next);
    cat_send(cmd);
    vfo_rx = (next == 1) ? '1' : '0';
    split = (next == 2);
    txfreq = 0;
    tune_tx = split;                    // entering split: knob tunes TX
    last_tune = ms();
    if (split) {                        // fetch the TX frequency right away
        expect_fb = true;
        cat_send("FB;");
    }
    vfo_ab();
    vfo_status();
    vfo_freq();
}

// Knob press in split: swap the knob between TX and RX
static void toggle_tune_target()
{
    tune_tx = !tune_tx;
    vfo_freq();
    vfo_status();
}

// ----- RIT -----
// Needs QMX System Config "CAT RU and RD" = Absolute, so RUnnn;/RDnnn; set the exact offset
static void send_rit_ofs()
{
    char c[24];
    if (rit_ofs >= 0) snprintf(c, sizeof(c), "RU%d;", rit_ofs);
    else              snprintf(c, sizeof(c), "RD%d;", -rit_ofs);
    cat_send(c);
    last_tune = ms();
}

static void rit_toggle()                 // tap: RIT on/off, offset kept
{
    rit_on = !rit_on;
    cat_send(rit_on ? "RT1;" : "RT0;");
    last_tune = ms();
    vfo_rit();
}

static void rit_edit_begin()             // long-press: knob adjusts the offset
{
    rit_edit = true;
    rit_edit_t = ms();
    if (!rit_on) {
        rit_on = true;
        send_rit_ofs();
        cat_send("RT1;");
    }
    vfo_rit();
}

static void rit_edit_end()
{
    rit_edit = false;
    vfo_rit();
}

static void rit_adjust(int delta)
{
    rit_ofs += delta * RIT_STEP;
    if (rit_ofs >  RIT_MAX) rit_ofs =  RIT_MAX;
    if (rit_ofs < -RIT_MAX) rit_ofs = -RIT_MAX;
    send_rit_ofs();
    rit_edit_t = ms();
    vfo_rit();
}

static void rit_zero()                   // long-press while editing
{
    rit_ofs = 0;
    send_rit_ofs();
    rit_edit_t = ms();
    vfo_rit();
}

// Band button: jump to the next band up (wrapping), restoring its last-used frequency
static void next_band()
{
    if (freq == 0) return;
    int b = band_of(freq);
    if (b < 0) {                                    // between bands: next band above us
        b = NBANDS - 1;
        for (int i = 0; i < NBANDS; i++)
            if (bands[i].lo > freq) { b = (i + NBANDS - 1) % NBANDS; break; }
    }
    int nb = b;
    do { nb = (nb + 1) % NBANDS; } while (!band_visible(nb) && nb != b);
    freq = band_last[nb];
    send_freq();
    if (split) {                        // keep TX on the same band: start it on the RX frequency
        txfreq = freq;
        send_tx();
        vfo_status();
    }
    vfo_freq();
    vfo_info();
}

static void radio_move(int delta)
{
    radio_sel = ((radio_sel + delta) % NRADIOS + NRADIOS) % NRADIOS;
    setup_radio();
}

static void radio_select()
{
    radio_idx = radio_sel;
    settings_save();
    setup_conn();
    setup_radio();
}

// ---------------- CAT replies ----------------
static void handle_reply(const char *r)
{
    size_t n = strlen(r);

    // Menu (MM) replies, and "?;" errors, belong to the outstanding MM request
    if (mm_state != MM_IDLE && (strncmp(r, "MM", 2) == 0 || strcmp(r, "?;") == 0)) {
        mm_reply(r);
        return;
    }
    if (strncmp(r, "AG", 2) == 0 && n >= 5) {          // e.g. "AG0091;" = 22.75 dB
        int v = atoi(r + 3);
        if (ms() - af_t > 800 && v != af_gain) {
            af_gain = v;
            if (screen == SCR_AUDIO) au_af();
        }
        return;
    }
    if (strncmp(r, "RG", 2) == 0 && n >= 4) {          // e.g. "RG063;" = 63 dB
        int v = atoi(r + 2);
        if (ms() - rf_t > 800 && v != rf_gain) {
            rf_gain = v;
            if (screen == SCR_AUDIO) au_rf();
        }
        return;
    }
    if (strncmp(r, "KS", 2) == 0 && n >= 4) {
        int w = atoi(r + 2);
        if (w > 0 && ms() - wpm_t > 800 && w != cw_wpm) {
            cw_wpm = w;
            if (screen == SCR_CW) cw_speed();
        }
        return;
    }
    uint32_t now = ms();
    bool hold = (now - last_tune) < 500;   // ignore stale echoes right after we changed something

    if (strncmp(r, "IF", 2) == 0 && n >= 34) {
        char fbuf[12];
        memcpy(fbuf, r + 2, 11);
        fbuf[11] = 0;
        uint32_t f = strtoul(fbuf, nullptr, 10);
        bool t  = (r[28] == '1');
        char m  = r[29];
        char v  = r[30];
        bool sp = (r[32] == '1');
        char rb[6];
        memcpy(rb, r + 18, 5);             // RIT offset, e.g. "+0120"
        rb[5] = 0;
        int  ro  = atoi(rb);
        bool ron = (r[23] == '1');

        if (!hold && !rit_edit && (ro != rit_ofs || ron != rit_on)) {
            rit_ofs = ro;
            rit_on = ron;
            if (screen == SCR_VFO) vfo_rit();
        }

        if (t != tx) { tx = t; draw_tx_ring(); }
        if (!hold) {
            if (f != freq) {
                int old_band = band_of(freq);
                freq = f;
                int b = band_of(freq);
                if (b >= 0) band_last[b] = freq;
                if (screen == SCR_PAN) pan_header();
                if (screen == SCR_VFO) {
                    if (tx_target()) vfo_status(); else vfo_freq();   // RX is on the small line
                    if (b != old_band) vfo_info();
                }
            }
            if (m != mode) {
                mode = m;
                if (screen == SCR_PAN) pan_header();
                if (screen == SCR_VFO) vfo_mode();
            }
            if (v != vfo_rx || sp != split) {
                bool entering = sp && !split;
                vfo_rx = v; split = sp;
                if (entering) tune_tx = true;           // split set on the radio itself
                if (!split) { txfreq = 0; tune_tx = false; }
                if (screen == SCR_VFO) { vfo_ab(); vfo_status(); vfo_freq(); }
            }
        }
    } else if ((strncmp(r, "FB", 2) == 0 || (expect_fb && strncmp(r, "FA", 2) == 0)) && n >= 14) {
        // VFO B reply (the manual shows FB; answering with an "FA" prefix, so accept both)
        expect_fb = false;
        uint32_t f = strtoul(r + 2, nullptr, 10);
        bool stale = hold && tx_target() && txfreq != 0;   // we just tuned TX ourselves
        if (split && !stale && f != txfreq) {
            txfreq = f;
            if (screen == SCR_VFO) { if (tune_tx) vfo_freq(); else vfo_status(); }
        }
    }
}

// ---------------- main ----------------
extern "C" void app_main(void)
{
    auto cfg = M5.config();
    M5.begin(cfg);

    settings_load();
    for (int i = 0; i < NBANDS; i++) band_last[i] = bands[i].def;

    rxq = xQueueCreate(8, sizeof(reply_t));
    encoder_init();
    usb_init();
    pan_audio_init();

    int last_det = detents();
    uint32_t last_poll = 0;
    bool shown_conn = false;
    uint16_t shown_vid = 0;

    draw_screen();

    while (true) {
        M5.update();
        uint32_t now = ms();

        // Connection state changed -> refresh status lines
        if (connected != shown_conn || seen_vid != shown_vid) {
            shown_conn = connected;
            shown_vid = seen_vid;
            if (connected) {
                cat_send("IF;"); cat_send("KS;"); conn_t = now;
                if (screen == SCR_PAN) cat_send("Q91;");
            }
            if (screen == SCR_VFO)   vfo_status();
            if (screen == SCR_SETUP) setup_conn();
        }

        // ---- input ----
        int d = detents() - last_det;
        last_det += d;

        auto t = M5.Touch.getDetail();
        bool tap = false, longtap = false;
        if (t.wasFlicked()) {
            int dx = t.distanceX();
            if (dx < -30)      goto_screen(screen + 1);
            else if (dx > 30)  goto_screen(screen - 1);
        } else if (t.wasHold()) {
            longtap = true;
        } else if (t.wasClicked()) {
            tap = true;
        }

        bool click = M5.BtnA.wasClicked();
        if (M5.BtnA.wasHold()) goto_screen(screen + 1);

        switch (screen) {
            case SCR_VFO:
                if (rit_edit && now - rit_edit_t > 5000) rit_edit_end();   // idle timeout

                if (d) {
                    if (rit_edit) rit_adjust(d);
                    else          tune(d);
                }
                if (click) {
                    if (rit_edit)   rit_edit_end();          // done editing RIT
                    else if (split) toggle_tune_target();    // split: swap knob between TX and RX
                    else            next_step();             // otherwise: next step preset
                }
                if (longtap && rit_tap(t.x, t.y)) {
                    if (rit_edit) rit_zero();                // long-press again: zero the offset
                    else          rit_edit_begin();
                }
                if (tap) {
                    if (rit_tap(t.x, t.y)) {
                        if (rit_edit) rit_edit_end();
                        else          rit_toggle();
                    }
                    else if (band_tap(t.x, t.y))  next_band();
                    else if (mode_tap(t.x, t.y))  cycle_mode();
                    else if (vfo_tap(t.x, t.y))   cycle_vfo();
                    else                          freq_tap(t.x, t.y);   // digit = step; elsewhere ignored
                }
                break;
            case SCR_PAN:
                if (d)     pan_tune(d);                // knob = tune
                if (click) pan_next_step();            // knob press = step
                if (tap)   pan_tap(t.x, t.y);          // tap spectrum = tune there
                if (pan_ready) pan_draw_spectrum();
                {
                    static bool shown_audio = false;
                    if (pan_audio_ok != shown_audio) { shown_audio = pan_audio_ok; pan_scale(); }
                }
                break;
            case SCR_CW:
                if (d)   cw_speed_adj(d);              // knob = keyer speed
                if (tap) cw_tap(t.x, t.y);
                break;
            case SCR_AUDIO:
                if (d)   af_adj(d);                    // knob = volume
                if (tap) au_tap(t.x, t.y);
                break;
            case SCR_SETUP:
                if (d)             radio_move(d);
                if (click || tap)  radio_select();
                break;
        }

        cw_service(now);

        // ---- poll the radio ----
        if (connected && now - last_poll > 300 && now - last_tune > 500) {
            cat_send("IF;");
            if (split) {                       // also fetch the TX (VFO B) frequency
                expect_fb = true;
                cat_send("FB;");
            }
            last_poll = now;
        }

        reply_t r;
        while (xQueueReceive(rxq, &r, 0) == pdTRUE) handle_reply(r.s);

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}