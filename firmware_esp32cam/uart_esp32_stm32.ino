/*******************************************************
 * ESP32-CAM AI Thinker + Edge Impulse + ThingsBoard
 * ENV SENSOR VERSION (SHT3x + Soil Moisture)
 *
 * ✔ AI Inference (96x96 RGB)
 * ✔ Snapshot upload
 * ✔ UART ENV from STM32 ($ENV)
 * ✔ Terminal realtime log (FIFO)
 * ✔ ENV Telemetry (chart + log)
 *******************************************************/

#include <WiFi.h>
#include <HTTPClient.h>
#include "esp_camera.h"
#include "base64.h"

/* ================= UART ENV ================= */
#define UART_ENV    Serial2
#define ENV_RX_PIN  3        // STM32 TX -> ESP32 RX

/* ================= WIFI ================= */
const char* ssid     = "KDT";
const char* password = "dkdt1813";

/* ================= THINGSBOARD ============== */
String TB_TOKEN = "kqx3BKLRbOTY0HhbeVg4";
String TB_URL   = "http://thingsboard.cloud/api/v1/";

/* ================= EDGE IMPULSE ================= */
#include <plant_bean_classification_custom_mt_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

/* ================= CAMERA MODEL ================= */
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

/* ================= EI INPUT SIZE ================= */
#define EI_COLS 96
#define EI_ROWS 96
uint8_t* ei_buf;
uint8_t* resize_buf;

/* ================= ENV DATA ================= */
struct ENV_Data {
    float temperature;
    float humidity;
    float soil_percent;
    uint32_t soil_adc;
    unsigned long lastUpdate;
};

ENV_Data env;


float    env_temp = 0.0f;
float    env_humi = 0.0f;
float    env_soil = 0.0f;
uint32_t env_adc  = 0;
float    env_uv   = 0.0f;
int      env_aqi  = 0;
float    env_leaf_wet = 0.0f; 
unsigned long lastSnapshot = 0;
/* ================= PERFORMANCE ================= */
unsigned long lastFrame = 0;
float fps = 0;

/* ================= LOG FIFO ================= */
#define LOG_FIFO_SIZE 50
String logFIFO[LOG_FIFO_SIZE];
int fifo_head = 0;
int fifo_tail = 0;

bool fifo_empty() { return fifo_head == fifo_tail; }
bool fifo_full()  { return (fifo_head + 1) % LOG_FIFO_SIZE == fifo_tail; }

void fifo_push(String msg) {
    if (fifo_full()) fifo_tail = (fifo_tail + 1) % LOG_FIFO_SIZE;
    fifo_head = (fifo_head + 1) % LOG_FIFO_SIZE;
    logFIFO[fifo_head] = msg;
}

String fifo_pop() {
    if (fifo_empty()) return "";
    fifo_tail = (fifo_tail + 1) % LOG_FIFO_SIZE;
    return logFIFO[fifo_tail];
}

/* ================= LOG FORMAT ================= */
String timeTag() {
    unsigned long t = millis() / 1000;
    char buf[20];
    sprintf(buf, "[%02lu:%02lu:%02lu]", t/3600, (t%3600)/60, t%60);
    return String(buf);
}

void LOG_INFO(String msg) {
    fifo_push(timeTag() + " [INFO] " + msg);
    Serial.println("[INFO] " + msg);
}

void LOG_ERR(String msg) {
    fifo_push(timeTag() + " [ERR] " + msg);
    Serial.println("[ERR] " + msg);
}

/* ================= SEND LOG ================= */
void sendLogFIFO() {
    if (WiFi.status() != WL_CONNECTED || fifo_empty()) return;

    HTTPClient http;
    http.begin(TB_URL + TB_TOKEN + "/telemetry");
    http.addHeader("Content-Type", "application/json");

    for (int i = 0; i < 5 && !fifo_empty(); i++) {
        String line = fifo_pop();
        line.replace("\"", "\\\"");
        http.POST("{\"log\":\"" + line + "\"}");
    }
    http.end();
}

/* ================= PARSE ENV UART ================= */
bool parseEnvLine(String line)
{
    if (!line.startsWith("$ENV")) return false;

    float temp, humi, soil, uv;
    uint32_t adc;
    int aqi;

    int ret = sscanf(line.c_str(),
        "$ENV,%f,%f,%f,%lu,%f,%d",
        &temp, &humi, &soil, &adc, &uv, &aqi);

    if (ret != 6) return false;

    env_temp = temp;
    env_humi = humi;
    env_soil = soil;
    env_adc  = adc;
    env_uv   = uv;
    env_aqi  = aqi;

    return true;
}

/* ================= SEND ENV TELEMETRY ================= */
void sendEnvTelemetry() {
    if (WiFi.status() != WL_CONNECTED) return;

    HTTPClient http;
    http.begin(TB_URL + TB_TOKEN + "/telemetry");
    http.addHeader("Content-Type", "application/json");

    String payload = "{";
    payload += "\"leaf_wet\":" + String(env_leaf_wet,1) + ",";
    payload += "\"temperature\":" + String(env_temp,2) + ",";
    payload += "\"humidity\":"    + String(env_humi,2) + ",";
    payload += "\"soil_moist\":"  + String(env_soil,1) + ",";
    payload += "\"soil_adc\":"    + String(env_adc) + ",";
    payload += "\"uv_index\":"    + String(env_uv,1) + ",";
    payload += "\"aqi\":"         + String(env_aqi);
    payload += "}";

    http.POST(payload);

    // http.POST(json);
    http.end();
}

void mockLeafWetness()
{
    static float wet = 20.0;
    static int dir = 1;

    // Base on humidity
    wet = env_humi * 0.9;

    // Add slow variation
    wet += dir * 2.0;

    if (wet > 90.0) dir = -1;
    if (wet < 20.0) dir = 1;

    // Clamp
    if (wet < 0) wet = 0;
    if (wet > 100) wet = 100;

    env_leaf_wet = wet;
}


void checkEnvThresholds()
{
    static unsigned long lastWarn = 0;
    if (millis() - lastWarn < 5000) return;   // rate limit 5s
    lastWarn = millis();

    if (env_aqi > 150)
        LOG_ERR("[CRITICAL][AQI] AQI=" + String(env_aqi) + " harmful for apple orchard");
    else if (env_aqi > 100)
        LOG_ERR("[WARN][AQI] AQI=" + String(env_aqi) + " may stress apple trees");

    if (env_uv >= 8.0)
        LOG_ERR("[CRITICAL][UV] UV=" + String(env_uv,1) + " severe leaf stress");
    else if (env_uv >= 6.0)
        LOG_ERR("[WARN][UV] UV=" + String(env_uv,1) + " high UV exposure");

    if (env_temp > 35.0 || env_temp < 5.0)
        LOG_ERR("[CRITICAL][TEMP] T=" + String(env_temp,1) + "C extreme");
    else if (env_temp > 30.0 || env_temp < 10.0)
        LOG_ERR("[WARN][TEMP] T=" + String(env_temp,1) + "C suboptimal");

    if (env_humi > 85.0)
        LOG_ERR("[WARN][HUMI] H=" + String(env_humi,1) + "% fungal risk");
    else if (env_humi < 40.0)
        LOG_ERR("[WARN][HUMI] H=" + String(env_humi,1) + "% drought risk");

    if (env_leaf_wet > 80.0) {
    LOG_ERR("[CRITICAL][LEAF] Leaf wetness=" +
            String(env_leaf_wet,1) +
            "% High fungal disease risk");
    }
    else if (env_leaf_wet > 60.0) {
        LOG_ERR("[WARN][LEAF] Leaf wetness=" +
                String(env_leaf_wet,1) +
                "% Fungal infection likely");
    }
}


/* ================= CAMERA INIT ================= */
bool initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;

    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;

    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;

    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_PSRAM;

    return esp_camera_init(&config) == ESP_OK;
}

/* ================= EI CALLBACK ================= */
int ei_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t ix = offset * 3;
    while (length--) {
        *out_ptr++ =
            (resize_buf[ix + 2] << 16) |
            (resize_buf[ix + 1] << 8) |
             resize_buf[ix];
        ix += 3;
    }
    return 0;
}

/* ================= CAPTURE EI FRAME ================= */
bool capture_ei_frame() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) return false;

    bool ok = fmt2rgb888(fb->buf, fb->len, fb->format, ei_buf);
    esp_camera_fb_return(fb);
    if (!ok) return false;

    ei::image::processing::resize_image(
        ei_buf, 320, 240,
        resize_buf, EI_COLS, EI_ROWS,
        3
    );
    return true;
}

void sendAITelemetry(String ai_class, float conf) {
    if (WiFi.status() != WL_CONNECTED) return;

    HTTPClient http;
    http.begin(TB_URL + TB_TOKEN + "/telemetry");
    http.addHeader("Content-Type", "application/json");

    String payload = "{";
    payload += "\"ai_class\":\"" + ai_class + "\",";
    payload += "\"ai_conf\":" + String(conf,3);
    payload += "}";

    http.POST(payload);
    http.end();
}

/* ================= SEND SNAPSHOT ================= */
void send_snapshot_ai(String b64, String ai_class, float conf) {
    HTTPClient http;
    http.begin(TB_URL + TB_TOKEN + "/telemetry");
    http.addHeader("Content-Type", "application/json");

    String json = "{";
    json += "\"image\":\"data:image/jpeg;base64," + b64 + "\",";
    // json += "\"ai_class\":\"" + ai_class + "\",";
    // json += "\"ai_conf\":" + String(conf,3);
    json += "}";

    http.POST(json);
    http.end();
}

/* ================= SETUP ================= */
void setup() {
    Serial.begin(115200);
    UART_ENV.begin(115200, SERIAL_8N1, ENV_RX_PIN, -1);
    fifo_head = fifo_tail = 0;   // clear FIFO

    LOG_INFO("=== FW: ENV+LEAF+ALERT v3 (2026-01-08 100%) ===");
    LOG_INFO("ENV UART started");

    if (!initCamera()) {
        LOG_ERR("Camera init FAILED");
        while (1);
    }

    ei_buf     = (uint8_t*)malloc(320 * 240 * 3);
    resize_buf = (uint8_t*)malloc(EI_COLS * EI_ROWS * 3);

    WiFi.begin(ssid, password);
    LOG_INFO("Connecting WiFi...");

    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
        Serial.print(".");
    }
    LOG_INFO("WiFi OK, IP=" + WiFi.localIP().toString());
}

/* ================= LOOP ================= */
void loop() {

    /* ===== UART ENV ===== */
    while (UART_ENV.available()) {
        String line = UART_ENV.readStringUntil('\n');
        line.trim();

        if (parseEnvLine(line)) {
            mockLeafWetness();
            LOG_INFO(
            "ENV T=" + String(env_temp,1) +
            " H=" + String(env_humi,1) +
            " Soil=" + String(env_soil,1) +
            " UV=" + String(env_uv,1) +
            " AQI=" + String(env_aqi)
            );
            sendEnvTelemetry();
            checkEnvThresholds();
            sendLogFIFO();
        }
    }

    /* ===== AI CAMERA ===== */
    unsigned long now = millis();
    fps = 1000.0 / (now - lastFrame);
    lastFrame = now;

    if (!capture_ei_frame()) {
        LOG_ERR("EI capture failed");
        sendLogFIFO();
        delay(100);
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_COLS * EI_ROWS;
    signal.get_data = &ei_get_data;

    ei_impulse_result_t result;
    unsigned long t_ai = millis();
    run_classifier(&signal, &result, false);
    t_ai = millis() - t_ai;

    int best = 0;
    float best_conf = 0;
    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > best_conf) {
            best_conf = result.classification[i].value;
            best = i;
        }
    }
    sendAITelemetry(ei_classifier_inferencing_categories[best], best_conf);

    LOG_INFO(
        "AI " + String(ei_classifier_inferencing_categories[best]) +
        " conf=" + String(best_conf,3) +
        " infer=" + String(t_ai) + "ms"
    );

    /* ================= SNAPSHOT (ĐỊNH KỲ) ================= */
    static unsigned long lastSnapshot = 0;
    if (millis() - lastSnapshot > 8000) {   // 1 ảnh / 8 giây
        lastSnapshot = millis();

        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
            String b64 = base64::encode(fb->buf, fb->len);
            esp_camera_fb_return(fb);

            send_snapshot_ai(
                b64,
                ei_classifier_inferencing_categories[best],
                best_conf
            );
        }
    }

    /* ================= SEND LOG FIFO ================= */
    sendLogFIFO();

    delay(200);   // giữ hệ ổn định
    // camera_fb_t* fb = esp_camera_fb_get();
    // if (!fb) {
    //     LOG_ERR("JPEG capture failed");
    //     return;
    // }

    // String b64 = base64::encode(fb->buf, fb->len);
    // esp_camera_fb_return(fb);

    // send_snapshot_ai(b64, ei_classifier_inferencing_categories[best], best_conf);

    // LOG_INFO("AI " + String(ei_classifier_inferencing_categories[best]) +
    //          " conf=" + String(best_conf,3) +
    //          " infer=" + String(t_ai) + "ms");

    // sendLogFIFO();
    // delay(200);
}
