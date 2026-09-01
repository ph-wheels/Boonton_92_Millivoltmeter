/*
* Boonton 92BD V3 firmware
* 
* This firmware is designed for the Boonton 92BD device to replace the original mechanical chopper
8 with a modern solid-state implementation as a input-follower logic with interlock, 
* ADC monitoring, and a serial menu interface for configuration & diagnostic
* this code is based on the ESP-IDF framework and uses FreeRTOS for task management.
* majority of this code is based extensive support of Claude AI field tested by meself.
* on my personally owned equipment and as a result of reverse enginering mechanical chopper
* and creating a modern replacement for it using the ESP32 platform.
* this to prevent an excellent piece of test equipment from being scrapped 
* due to the unavailability of the original mechanical chopper.
* current FW 3.0.4 replaces 1.6b1 and is based on HW PCB 2.7
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* ================= Pin configuration ================= */
#define OUT_PIN_Ph_A      GPIO_NUM_19   // phase A output may need to be swapped with phase B for syncronous chopper operation
#define OUT_PIN_Ph_B      GPIO_NUM_18   // phase B output may need to be swapped with phase A for syncronous chopper operation
#define OUT_PIN_1mV       GPIO_NUM_22
#define OUT_PIN_3mV       GPIO_NUM_32
#define OUT_PIN_10mV      GPIO_NUM_33
#define OUT_PIN_30mV      GPIO_NUM_25
#define OUT_PIN_100mV     GPIO_NUM_26
#define OUT_PIN_300mV     GPIO_NUM_27
#define OUT_PIN_1000mV    GPIO_NUM_14  // <== true for Rev2.2 PCB, rev 2.7 requires GPIO_NUM_23
#define OUT_PIN_3000mV    GPIO_NUM_21

#define IN_PIN_C_2        GPIO_NUM_16   // inputs from the 92BD's internal chooper connector
#define IN_PIN_C_1        GPIO_NUM_17   // inputs from the 92BD's internal chooper connector
#define IN_PIN_Auto       GPIO_NUM_5    // enable auto ranging system
#define IN_PIN_CMD        GPIO_NUM_2    // active-low: LOW = enable ESP_LOG output, HIGH = silence it

#define OUTPUT_ACTIVE_LOW  1   // outputs: ON = pin driven low
#define INPUT_ACTIVE_LOW   1   // inputs:  active = pin read low

#define ADC_level         GPIO_NUM_35   // feedback ADC input from the 92BD's internal analog source, used for auto-ranging decisions
#define ADC_UNIT          ADC_UNIT_1
#define ADC_CHANNEL       ADC_CHANNEL_7   // GPIO35 on ESP32 = ADC1_CH7
#define ADC_ATTEN         ADC_ATTEN_DB_12 // ~0-3.1V full scale at the pin (post-divider)
#define ADC_BITWIDTH      ADC_BITWIDTH_DEFAULT
#define ADC_SAMPLE_COUNT  8               // averaging
#define ADC_TASK_PERIOD_MS 100

/* ================= Timing configuration ================= */

#define SCAN_PERIOD_US       10600U   // 94 Hz +/- 1% scan/chopper rate
#define DEAD_TIME_DEFAULT_US 120U     // was 1060U -- capped at 500us max per requirement
#define DEAD_TIME_MIN_US     40U
#define DEAD_TIME_MAX_US     200U     // was 5000U -- hard ceiling now matches the default cap

#define VERSION "3.0.4"      // 3.0.0 initial freeRTOS implementation
                             // 3.0.1 adjusted selectable range of various parameter for NVS
                             // 3.0.2cleanup of menu wording / layot
                             // 3.0.3 dropped usage of input CMD & M42
                             // 3.0.4 added log gate functionality

#define NVS_KEY_DEADTIME     "dead_time_us"
#define NVS_KEY_ADC_LOW      "adc_low_mv"
#define NVS_KEY_ADC_HIGH     "adc_high_mv"
#define NVS_KEY_RANGE_SETTLE "range_settle_ms"

#define ADC_LOW_MIN_MV       0
#define ADC_LOW_MAX_MV       600
#define ADC_HIGH_MIN_MV      0
#define ADC_HIGH_MAX_MV      2000

#define RANGE_SETTLE_MIN_MS  10
#define RANGE_SETTLE_MAX_MS  500

#define UART_PORT_NUM    UART_NUM_0
#define UART_BUF_SIZE    256

#define NVS_NAMESPACE     "settings"
#define NVS_KEY_DEADTIME  "dead_time_us"

#define RANGE_COUNT              8
#define RANGE_SWITCH_DEAD_US     100    // break-before-make gap between range pins

#define NVS_KEY_RANGE_HYST   "range_hyst_mv"
#define RANGE_HYST_MIN_MV    0
#define RANGE_HYST_MAX_MV    500


static const gpio_num_t s_range_pins[RANGE_COUNT] = {
    OUT_PIN_1mV, OUT_PIN_3mV, OUT_PIN_10mV, OUT_PIN_30mV,
    OUT_PIN_100mV, OUT_PIN_300mV, OUT_PIN_1000mV, OUT_PIN_3000mV
};

static int64_t s_last_range_switch_us = 0;

static const char *TAG = "92BD";

/* ================= Shared runtime-adjustable state ================= */
static volatile uint32_t s_dead_time_us   = DEAD_TIME_DEFAULT_US;
static volatile int32_t  s_adc_low_mv     = 200;    // range floor, mV
static volatile int32_t  s_adc_high_mv    = 1800;   // range ceiling, mV
static volatile int32_t  s_adc_last_mv    = 0;      // last calibrated reading
static volatile int8_t   s_adc_range_stat = 0;      // -1 / 0 / +1
static volatile bool     s_input_fault    = false;  // both inputs active simultaneously
static volatile int32_t  s_range_hyst_mv  = 50;     // extra margin beyond al/ah before switching, mV
static volatile uint32_t s_range_settle_ms = 150;   // default, overridden by NVS if present
static bool s_auto_was_active = false;              // tracks Auto's previous state, for edge detection
static volatile uint8_t s_adc_range_index = 0;
static bool s_log_enabled = false;                  // mirrors IN_PIN_CMD; starts silent until first poll

static TaskHandle_t s_io_task_handle = NULL;

#include "freertos/queue.h"

typedef struct {
    gpio_num_t pin;
    bool active;      // true = just went active (falling edge on active-low input)
    bool heartbeat;   // true = this is just a WDT heartbeat tick, ignore pin/active
} phase_event_t;

static QueueHandle_t s_phase_evt_queue;

/* ================= Output / input helpers ================= */
static inline void output_set(gpio_num_t pin, bool on)
{
#if OUTPUT_ACTIVE_LOW
    gpio_set_level(pin, on ? 0 : 1);
#else
    gpio_set_level(pin, on ? 1 : 0);
#endif
}

static inline bool input_is_active(gpio_num_t pin)
{
    int level = gpio_get_level(pin);
#if INPUT_ACTIVE_LOW
    return level == 0;
#else
    return level == 1;
#endif
}

/* ================= Range switching helpers ================= */
static void range_apply(uint8_t new_idx)
{
    if (new_idx >= RANGE_COUNT) return;
    if (new_idx == s_adc_range_index) return;

    output_set(s_range_pins[s_adc_range_index], false);  // old range off
    esp_rom_delay_us(RANGE_SWITCH_DEAD_US);               // guaranteed gap
    output_set(s_range_pins[new_idx], true);              // new range on

    s_adc_range_index = new_idx;
    s_last_range_switch_us = esp_timer_get_time();
    ESP_LOGI(TAG, "Auto-range switched to index %u (%s)", new_idx,
             new_idx == 0 ? "1mV" : new_idx == 7 ? "3000mV" : "intermediate");
}

static void range_all_off(void)
{
    for (int i = 0; i < RANGE_COUNT; i++) {
        output_set(s_range_pins[i], false);
    }
}

/* Called exactly once, on the transition into Auto-active */
static void range_enter(uint8_t start_idx)
{
    range_all_off();                        // guarantee known state regardless of history
    esp_rom_delay_us(RANGE_SWITCH_DEAD_US);  // break-before-make even from the "all off" state
    output_set(s_range_pins[start_idx], true);
    s_adc_range_index = start_idx;
    s_last_range_switch_us = esp_timer_get_time();
    ESP_LOGI(TAG, "Auto-ranging engaged, starting at range index %u", start_idx);
}

/* Called exactly once, on the transition out of Auto-active */
static void range_exit(void)
{
    range_all_off();
    ESP_LOGI(TAG, "Auto-ranging disengaged, all ranges off");
}

static void auto_range_update(int8_t status)
{
    bool active = input_is_active(IN_PIN_Auto);

    if (active && !s_auto_was_active) {
        range_enter(0);
        s_auto_was_active = true;
        return;
    }
    if (!active && s_auto_was_active) {
        range_exit();
        s_auto_was_active = false;
        return;
    }
    if (!active) return;

    int64_t now = esp_timer_get_time();
    if (now - s_last_range_switch_us < ((int64_t)s_range_settle_ms * 1000)) return;

    int32_t mv = s_adc_last_mv;                 // use the raw reading directly, not just the -1/0/+1 status
    int32_t hyst = s_range_hyst_mv;

    if (mv > (s_adc_high_mv + hyst) && s_adc_range_index < RANGE_COUNT - 1) {
        range_apply(s_adc_range_index + 1);
    } else if (mv < (s_adc_low_mv - hyst) && s_adc_range_index > 0) {
        range_apply(s_adc_range_index - 1);
    }
}

static void log_gate_update(void)
{
    bool want_enabled = input_is_active(IN_PIN_CMD);   // active-low: LOW = enabled
    if (want_enabled != s_log_enabled) {
        s_log_enabled = want_enabled;
        esp_log_level_set("*", want_enabled ? ESP_LOG_INFO : ESP_LOG_NONE);
        // Note: this line itself only prints if logs are already enabled at the moment it runs
        if (want_enabled) {
            ESP_LOGI(TAG, "ESP_LOG output enabled via IN_PIN_CMD");
        }
    }
}

/* ================= NVS: generic int32 load/save ================= */
static bool nvs_load_i32(const char *key, int32_t min, int32_t max, int32_t *out_val)
{
    nvs_handle_t h;
    bool ok = false;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        int32_t val = 0;
        if (nvs_get_i32(h, key, &val) == ESP_OK && val >= min && val <= max) {
            *out_val = val;
            ok = true;
            ESP_LOGI(TAG, "Loaded %s=%ld from NVS", key, (long)val);
        }
        nvs_close(h);
    }
    return ok;
}

static bool nvs_save_i32(const char *key, int32_t val)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t err = nvs_set_i32(h, key, val);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS save failed for %s: %s", key, esp_err_to_name(err));
    }
    return err == ESP_OK;
}

/* ================= Periodic scan tick (esp_timer -> notify) ================= */
static void IRAM_ATTR c_input_isr_handler(void *arg)
{
    gpio_num_t pin = (gpio_num_t)(uint32_t)arg;
    int level = gpio_get_level(pin);
    bool active = INPUT_ACTIVE_LOW ? (level == 0) : (level == 1);
    phase_event_t evt = { .pin = pin, .active = active, .heartbeat = false };
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(s_phase_evt_queue, &evt, &woken);
    if (woken) portYIELD_FROM_ISR();
}

static void IRAM_ATTR heartbeat_timer_cb(void *arg)
{
    phase_event_t evt = { .heartbeat = true };
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(s_phase_evt_queue, &evt, &woken);
    if (woken) portYIELD_FROM_ISR();
}

/* ================= Task 1 (highest priority): input-follower with interlock ================= */
typedef enum { OUT_NONE = 0, OUT_ACTIVE_1, OUT_ACTIVE_2 } out_state_t;

static void io_task(void *pvParameters)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    s_phase_evt_queue = xQueueCreate(16, sizeof(phase_event_t));
    configASSERT(s_phase_evt_queue != NULL);

    // Heartbeat: guarantees a periodic WDT feed independent of whether
    // C_1/C_2 are actually toggling (see note above if you'd prefer otherwise).
    const esp_timer_create_args_t hb_args = { .callback = &heartbeat_timer_cb, .name = "hb_timer" };
    esp_timer_handle_t hb_timer;
    ESP_ERROR_CHECK(esp_timer_create(&hb_args, &hb_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(hb_timer, SCAN_PERIOD_US)); // 10.6ms

    output_set(OUT_PIN_Ph_A, false);
    output_set(OUT_PIN_Ph_B, false);

    static bool c1_active = false, c2_active = false;
    phase_event_t evt;

    for (;;) {
        if (xQueueReceive(s_phase_evt_queue, &evt, pdMS_TO_TICKS(50)) == pdTRUE && !evt.heartbeat) {
            uint32_t dead_us = s_dead_time_us; // snapshot, runtime-adjustable

            if (evt.pin == IN_PIN_C_1) {
                c1_active = evt.active;
                if (evt.active) {
                    output_set(OUT_PIN_Ph_B, false);   // force-off the other side first
                    esp_rom_delay_us(dead_us);         // guaranteed gap before energizing
                    output_set(OUT_PIN_Ph_A, true);
                } else {
                    output_set(OUT_PIN_Ph_A, false);   // deactivate immediately, no delay
                }
            } else if (evt.pin == IN_PIN_C_2) {
                c2_active = evt.active;
                if (evt.active) {
                    output_set(OUT_PIN_Ph_A, false);
                    esp_rom_delay_us(dead_us);
                    output_set(OUT_PIN_Ph_B, true);
                } else {
                    output_set(OUT_PIN_Ph_B, false);
                }
            }

            s_input_fault = c1_active && c2_active; // both active simultaneously = fault flag
        }

        // Runs every loop pass (real event, heartbeat, or 50ms timeout fallback) -
        // WDT is fed at least every ~10.6ms in normal operation.
        esp_task_wdt_reset();
    }
}

/* ================= ADC calibration + classification ================= */
static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_adc_cali_handle = NULL;
static bool s_adc_cali_ok = false;

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten,
                                  adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id = unit, .chan = channel, .atten = atten, .bitwidth = ADC_BITWIDTH,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cfg, &handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cfg = {
        .unit_id = unit, .atten = atten, .bitwidth = ADC_BITWIDTH,
#if CONFIG_IDF_TARGET_ESP32
        .default_vref = 1100,   // fallback mV used only if no eFuse Vref/TP calibration is burned
#endif
    };
    ret = adc_cali_create_scheme_line_fitting(&cfg, &handle);
#endif
    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration scheme created OK (using %s)",
                 ret == ESP_OK ? "eFuse or default_vref fallback" : "n/a");
    }
    return ret == ESP_OK;
}

/* Returns -1 below range, 0 within range, +1 above range */
static int8_t adc_classify(int32_t mv)
{
    if (mv < s_adc_low_mv)  return -1;
    if (mv > s_adc_high_mv) return  1;
    return 0;
}

/* ================= Task: ADC monitor ================= */
static void adc_task(void *pvParameters)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = { .atten = ADC_ATTEN, .bitwidth = ADC_BITWIDTH };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, ADC_CHANNEL, &chan_cfg));

    s_adc_cali_ok = adc_calibration_init(ADC_UNIT, ADC_CHANNEL, ADC_ATTEN, &s_adc_cali_handle);
    if (!s_adc_cali_ok) {
        ESP_LOGW(TAG, "ADC calibration not available on this chip/eFuse - readings will be raw-only");
    }

    for (;;) {
        int32_t sum_raw = 0;
        for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
            int raw = 0;
            adc_oneshot_read(s_adc_handle, ADC_CHANNEL, &raw);
            sum_raw += raw;
            esp_rom_delay_us(50); // small gap between samples
        }
        int raw_avg = sum_raw / ADC_SAMPLE_COUNT;

        int mv = 0;
        if (s_adc_cali_ok) {
            adc_cali_raw_to_voltage(s_adc_cali_handle, raw_avg, &mv);
        } else {
            mv = raw_avg; // fallback: unscaled, treat thresholds as raw counts in this case
        }

        s_adc_last_mv    = mv;
        s_adc_range_stat = adc_classify(mv);

        auto_range_update(s_adc_range_stat);   // auto-range switching if enabled and needed

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(ADC_TASK_PERIOD_MS));
    }
}

/* ================= Task: serial menu ================= */
static void print_menu(void)
{
    printf("\r\n--- Menu --- Boonton %s --- Version %s\r\n", TAG, VERSION);
    printf("s                Show status\r\n");
    printf("d                Show dead_time_us\r\n");
    printf("d  <us>  (120)   Set dead_time_us (%u-%u), saved to NVS\r\n", DEAD_TIME_MIN_US, DEAD_TIME_MAX_US);
    printf("al <mv>  (310)   Set ADC low threshold (mV), saved to NVS\r\n");
    printf("ah <mv>  (1670)  Set ADC high threshold (mV), saved to NVS\r\n");
    printf("a                Show ADC reading + range status\r\n");
    printf("rs               Show range_settle_ms\r\n");
    printf("rs <ms>  (150)   Set range_settle_ms (%u-%u), saved to NVS\r\n", RANGE_SETTLE_MIN_MS, RANGE_SETTLE_MAX_MS);
    printf("rh <mv>  (10)    Set range hysteresis (mV), saved to NVS\r\n");
    printf("rng <0-7>        Manually force range index (bypasses Auto)\r\n");
    printf("                 Advised  value's in (), changing dead_time requires calibration !\r\n> ");
    printf("h                Help.\r\n> ");
    fflush(stdout);
}

static void handle_command(char *line)
{
    line[strcspn(line, "\r\n")] = 0;

    if (line[0] == '\0') {
        return;
    } else if (strcmp(line, "s") == 0) {
        printf("\r\ndead_time_us=%lu  input_fault=%d  adc_mv=%ld  adc_status=%d  range_idx=%u  auto_active=%d\r\n",
               (unsigned long)s_dead_time_us, s_input_fault,
               (long)s_adc_last_mv, s_adc_range_stat, s_adc_range_index,
               input_is_active(IN_PIN_Auto));
    } else if (line[0] == 'd' && line[1] == '\0') {
        printf("\r\nCurrent dead_time_us = %lu\r\n", (unsigned long)s_dead_time_us);
    } else if (line[0] == 'd' && line[1] == ' ') {
        uint32_t val;
        if (sscanf(line + 2, "%lu", (unsigned long *)&val) == 1 &&
            val >= DEAD_TIME_MIN_US && val <= DEAD_TIME_MAX_US) {
            s_dead_time_us = val;
            bool ok = nvs_save_i32(NVS_KEY_DEADTIME, (int32_t)val);
            printf("\r\ndead_time_us set to %lu (%s)\r\n", (unsigned long)val, ok ? "saved" : "SAVE FAILED");
        } else {
            printf("\r\nInvalid value (range %u-%u)\r\n", DEAD_TIME_MIN_US, DEAD_TIME_MAX_US);
        }
    } else if (strncmp(line, "al ", 3) == 0) {
        int32_t val;
        if (sscanf(line + 3, "%ld", (long *)&val) == 1 &&
            val >= ADC_LOW_MIN_MV && val <= ADC_LOW_MAX_MV) {
            s_adc_low_mv = val;
            bool ok = nvs_save_i32(NVS_KEY_ADC_LOW, val);
            printf("\r\nadc_low_mv set to %ld (%s)\r\n", (long)val, ok ? "saved" : "SAVE FAILED");
        } else {
            printf("\r\nInvalid value (range %d-%d)\r\n", ADC_LOW_MIN_MV, ADC_LOW_MAX_MV);
        }
    } else if (strncmp(line, "ah ", 3) == 0) {
        int32_t val;
        if (sscanf(line + 3, "%ld", (long *)&val) == 1 &&
            val >= ADC_HIGH_MIN_MV && val <= ADC_HIGH_MAX_MV) {
            s_adc_high_mv = val;
            bool ok = nvs_save_i32(NVS_KEY_ADC_HIGH, val);
            printf("\r\nadc_high_mv set to %ld (%s)\r\n", (long)val, ok ? "saved" : "SAVE FAILED");
        } else {
            printf("\r\nInvalid value (range %d-%d)\r\n", ADC_HIGH_MIN_MV, ADC_HIGH_MAX_MV);
        }
    } else if (strcmp(line, "a") == 0) {
        printf("\r\nADC = %ld mV, range=[%ld,%ld,%ld], status=%d (-1 below / 0 within / +1 above)\r\n",
               (long)s_adc_last_mv, (long)s_adc_low_mv, (long)s_adc_high_mv, (long)s_adc_range_index, s_adc_range_stat);
    } else if (line[0] == 'r' && line[1] == 's' && line[2] == ' ') {
        uint32_t val;
        if (sscanf(line + 3, "%lu", (unsigned long *)&val) == 1 &&
            val >= RANGE_SETTLE_MIN_MS && val <= RANGE_SETTLE_MAX_MS) {
            s_range_settle_ms = val;
            bool ok = nvs_save_i32(NVS_KEY_RANGE_SETTLE, (int32_t)val);
            printf("\r\nrange_settle_ms set to %lu (%s)\r\n", (unsigned long)val, ok ? "saved" : "SAVE FAILED");
        } else {
            printf("\r\nInvalid value (range %u-%u)\r\n", RANGE_SETTLE_MIN_MS, RANGE_SETTLE_MAX_MS);
        }
    } else if (line[0] == 'r' && line[1] == 'h' && line[2] == ' ') {
        int32_t val;
        if (sscanf(line + 3, "%ld", (long *)&val) == 1 &&
            val >= RANGE_HYST_MIN_MV && val <= RANGE_HYST_MAX_MV) {
            s_range_hyst_mv = val;
            bool ok = nvs_save_i32(NVS_KEY_RANGE_HYST, val);
            printf("\r\nrange_hyst_mv set to %ld (%s)\r\n", (long)val, ok ? "saved" : "SAVE FAILED");
        } else {
            printf("\r\nInvalid value (range %d-%d)\r\n", RANGE_HYST_MIN_MV, RANGE_HYST_MAX_MV);
        }
    } else if (strncmp(line, "rng ", 4) == 0) {
        int val;
        if (sscanf(line + 4, "%d", &val) == 1 && val >= 0 && val < RANGE_COUNT) {
            range_all_off();
            esp_rom_delay_us(RANGE_SWITCH_DEAD_US);
            output_set(s_range_pins[val], true);
            s_adc_range_index = val;
            printf("\r\nManually forced range index %d (bypasses Auto)\r\n", val);
        } else {
            printf("\r\nInvalid range index (0-%d)\r\n", RANGE_COUNT - 1);
        }
    } else if (line[0] == 'r' && line[1] == 'h' && line[2] == '\0') {
        printf("\r\nCurrent range_hyst_mv = %ld\r\n", (long)s_range_hyst_mv);
    } else if (line[0] == 'r' && line[1] == 's' && line[2] == '\0') {
        printf("\r\nCurrent range_settle_ms = %lu\r\n", (unsigned long)s_range_settle_ms);       
    } else if (strcmp(line, "h") == 0) {
        // falls through to menu
    } else {
        printf("\r\nUnknown command\r\n");
    }
    print_menu();
}

static void menu_task(void *pvParameters)
{
    static char line_buf[64];
    static size_t line_len = 0;
    uint8_t data[UART_BUF_SIZE];

    print_menu();

    for (;;) {
        int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data), pdMS_TO_TICKS(100));
        for (int i = 0; i < len; i++) {
            char c = (char)data[i];
            putchar(c);
            if (c == '\n' || c == '\r') {
                if (line_len > 0) {
                    line_buf[line_len] = '\0';
                    handle_command(line_buf);
                    line_len = 0;
                }
            } else if (line_len < sizeof(line_buf) - 1) {
                line_buf[line_len++] = c;
            }
        }
        fflush(stdout);
        log_gate_update();
    }
}

/* ================= app_main ================= */
void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    
     int32_t val;
    if (nvs_load_i32(NVS_KEY_DEADTIME, DEAD_TIME_MIN_US, DEAD_TIME_MAX_US, &val)) {
        s_dead_time_us = (uint32_t)val;
    }
    if (nvs_load_i32(NVS_KEY_ADC_LOW, ADC_LOW_MIN_MV, ADC_LOW_MAX_MV, &val)) {
        s_adc_low_mv = val;
    }
    if (nvs_load_i32(NVS_KEY_ADC_HIGH, ADC_HIGH_MIN_MV, ADC_HIGH_MAX_MV, &val)) {
        s_adc_high_mv = val;
    }
    if (nvs_load_i32(NVS_KEY_RANGE_SETTLE, RANGE_SETTLE_MIN_MS, RANGE_SETTLE_MAX_MS, &val)) {
        s_range_settle_ms = (uint32_t)val;
    }
    if (nvs_load_i32(NVS_KEY_RANGE_HYST, RANGE_HYST_MIN_MV, RANGE_HYST_MAX_MV, &val)) {
        s_range_hyst_mv = val;
    }

    // Outputs
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL << OUT_PIN_Ph_A) | (1ULL << OUT_PIN_Ph_B) | \
        (1ULL << OUT_PIN_1mV) | (1ULL << OUT_PIN_3mV) | (1ULL << OUT_PIN_10mV) | \
        (1ULL << OUT_PIN_30mV) | (1ULL << OUT_PIN_100mV) | (1ULL << OUT_PIN_300mV) | \
        (1ULL << OUT_PIN_1000mV) | (1ULL << OUT_PIN_3000mV),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&out_conf);
    output_set(OUT_PIN_Ph_A, false);
    output_set(OUT_PIN_Ph_B, false);
    output_set(OUT_PIN_1mV, false);
    output_set(OUT_PIN_3mV, false);
    output_set(OUT_PIN_10mV, false);
    output_set(OUT_PIN_30mV, false);
    output_set(OUT_PIN_100mV, false);
    output_set(OUT_PIN_300mV, false);
    output_set(OUT_PIN_1000mV, false);
    output_set(OUT_PIN_3000mV, false);

    /* ========= make sure all range outputs are disabled ========= */
    for (int i = 0; i < RANGE_COUNT; i++) {
        output_set(s_range_pins[i], false);
    }
    s_adc_range_index = 0;        // not meaningful until Auto engages
    s_last_range_switch_us = 0;

    // Inputs (active-low -> internal pull-up)
    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << IN_PIN_C_2) | (1ULL << IN_PIN_C_1) | (1ULL << IN_PIN_Auto) | (1ULL << IN_PIN_CMD),
        .mode = GPIO_MODE_INPUT,
    #if INPUT_ACTIVE_LOW
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    #else
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    #endif
        .intr_type = GPIO_INTR_DISABLE,   // default; C_1/C_2 overridden below
    };
    gpio_config(&in_conf);

    // Edge-triggered interrupts specifically for the two phase inputs
    ESP_ERROR_CHECK(gpio_set_intr_type(IN_PIN_C_1, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(IN_PIN_C_2, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(IN_PIN_C_1, c_input_isr_handler, (void *)IN_PIN_C_1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(IN_PIN_C_2, c_input_isr_handler, (void *)IN_PIN_C_2));

    // UART for menu
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);

    s_log_enabled = input_is_active(IN_PIN_CMD);
    esp_log_level_set("*", s_log_enabled ? ESP_LOG_INFO : ESP_LOG_NONE);

    // Task watchdog
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 1000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    esp_task_wdt_reconfigure(&twdt_config);

    xTaskCreatePinnedToCore(io_task, "io_task", 4096, NULL,
                             configMAX_PRIORITIES - 1, &s_io_task_handle, 1);
    xTaskCreatePinnedToCore(adc_task, "adc_task", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(menu_task, "menu_task", 4096, NULL, 5, NULL, 0);
}
