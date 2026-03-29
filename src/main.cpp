/*
 * DỰ ÁN: KEYWORD SPOTTING 
 * Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "edge-impulse-sdk/classifier/ei_run_dsp.h"
#include <do_an_he_thong_nhung_inferencing.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_task_wdt.h"
#include "HardwareSerial.h"
#include <I2S.h>
#include <math.h>
#include <driver/i2s.h>
#include <algorithm>
using namespace std;

// ==================== CẤU HÌNH CƠ BẢN ====================
#define SAMPLE_RATE 16000U
#define SAMPLE_BITS 16
#define WDT_TIMEOUT_SEC 10

// ==================== CẤU HÌNH XỬ LÝ ÂM THANH ====================
#define VAD_THRESHOLD_MULTIPLIER 1.1
#define NOISE_PROFILE_SIZE 50
#define ENERGY_HISTORY_SIZE 5
#define MIN_VOICE_ENERGY 40
#define AUTO_GAIN_MAX 15.0
#define AUTO_GAIN_TARGET 15000
#define SILENCE_FRAMES_THRESHOLD 2

// ==================== CẤU HÌNH NHẬN DIỆN ====================
#define MIN_CONFIDENCE 0.40
#define HIGH_CONFIDENCE 0.70
#define DEBOUNCE_TIME_MS 400

// ==================== CẤU HÌNH WAKE-UP ====================
#define WAKEUP_TIMEOUT_MS 3000  // 3 giây timeout
#define WAKEUP_LED_BLINK_MS 200  // 200ms blink interval
#define WAKEUP_GPIO 6

// ==================== ĐỊNH NGHĨA CHÂN ====================
#define LED_NOISE 2      
#define LED_LIGHT 3       
#define LED_DOOR 4        
#define LED_BUILTIN 21     

// ==================== EVENT GROUPS ====================
EventGroupHandle_t ledEventGroup;
#define LED_LIGHT_ON_BIT (1 << 1)
#define LED_LIGHT_OFF_BIT (1 << 2)
#define LED_DOOR_ON_BIT  (1 << 3)
#define LED_DOOR_OFF_BIT (1 << 4)

// ==================== CẤU TRÚC DỮ LIỆU ====================
typedef struct {
    signed short *buffers[2];
    unsigned char buf_select;
    unsigned char buf_ready;
    unsigned int buf_count;
    unsigned int n_samples;
} inference_t;

typedef struct {
    float noise_floor;
    float noise_profile[NOISE_PROFILE_SIZE];
    int profile_index;
    float min_noise;
    float max_noise;
    int silence_counter;
} noise_manager_t;

typedef struct {
    int16_t energy_history[ENERGY_HISTORY_SIZE];
    int history_index;
    float avg_energy;
    float peak_energy;
    int voice_frames;
    bool is_speech;
    float auto_gain;
} audio_processor_t;

// ==================== BIẾN GLOBAL ====================
static inference_t inference;
static const uint32_t sample_buffer_size = 2048;
static signed short sampleBuffer[sample_buffer_size];
static bool debug_nn = false;
static bool record_status = true;
static TaskHandle_t mainTaskHandle = NULL;

static noise_manager_t noise_mgr;
static audio_processor_t audio_proc;
static bool light_state = false, door_state = false;
static bool system_awake = false;  // Trạng thái hệ thống có được đánh thức hay không
static unsigned long last_command_time = 0;  // Thời gian lệnh cuối cùng
static bool wakeup_led_state = false;  // Trạng thái LED wakeup
static unsigned long last_blink_time = 0;  // Thời gian blink cuối

HardwareSerial uart1(1);

// ==================== PROTOTYPES ====================
static void audio_inference_callback(uint32_t n_bytes);
static void capture_samples(void *arg);
static bool microphone_inference_start(uint32_t n_samples);
static bool microphone_inference_record(void);
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr);
static void controll_task(void *arg);
static void process_audio_advanced(int16_t* samples, size_t len);
static bool detect_speech_advanced(int16_t* samples, size_t len);
static void update_noise_profile(float current_level);
static float calculate_confidence_threshold(void);
static void execute_command(const char* command, float confidence);
static void print_simple_status(const char* keyword, float conf);
static void update_leds(void);
static void handle_wakeup_system(const char* command, float confidence);
static void update_wakeup_led(void);

// ==================== KHỞI TẠO ====================
void setup_watchdog() {
    esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
    mainTaskHandle = xTaskGetCurrentTaskHandle();
    esp_task_wdt_add(mainTaskHandle);
}

void feed_watchdog() {
    esp_task_wdt_reset();
}

bool init_i2s_with_retry() {
    int retry_count = 0;
    while (retry_count < 3) {
        I2S.setAllPins(-1, 42, 41, -1, -1);
        if (I2S.begin(PDM_MONO_MODE, SAMPLE_RATE, SAMPLE_BITS)) {
            return true;
        }
        retry_count++;
        delay(100);
    }
    return false;
}

void boot_indicator() {
    for (int i = 0; i < 2; i++) {
        digitalWrite(LED_BUILTIN, LOW);   // Bật LED (active LOW)
        delay(200);
        digitalWrite(LED_BUILTIN, HIGH);  // Tắt LED
        delay(200);
    }
}

void init_audio_processing() {
    noise_mgr.noise_floor = 30.0;
    noise_mgr.min_noise = 30.0;
    noise_mgr.max_noise = 30.0;
    noise_mgr.silence_counter = 0;
    noise_mgr.profile_index = 0;
    
    for(int i = 0; i < NOISE_PROFILE_SIZE; i++) {
        noise_mgr.noise_profile[i] = 30.0;
    }
    
    audio_proc.avg_energy = 0;
    audio_proc.peak_energy = 0;
    audio_proc.is_speech = false;
    audio_proc.voice_frames = 0;
    audio_proc.auto_gain = 2.0;
    audio_proc.history_index = 0;
    
    for(int i = 0; i < ENERGY_HISTORY_SIZE; i++) {
        audio_proc.energy_history[i] = 0;
    }
}

// ==================== XỬ LÝ ÂM THANH SIÊU NHẠY ====================
void process_audio_advanced(int16_t* samples, size_t len) {
    int32_t sum = 0;
    for(size_t i = 0; i < len; i++) {
        sum += samples[i];
    }
    int32_t dc_offset = sum / len;
    
    static int16_t prev_sample = 0;
    for(size_t i = 0; i < len; i++) {
        int16_t current = samples[i] - dc_offset;
        samples[i] = current - 0.95 * prev_sample;
        prev_sample = current;
    }
    
    int16_t peak = 0;
    float avg_abs = 0;
    for(size_t i = 0; i < len; i++) {
        int16_t abs_val = abs(samples[i]);
        avg_abs += abs_val;
        if(abs_val > peak) peak = abs_val;
    }
    avg_abs /= len;
    
    if(peak > 0) {
        float target_gain = (float)AUTO_GAIN_TARGET / max(peak, (int16_t)100);
        float avg_gain = (float)AUTO_GAIN_TARGET / max(avg_abs, 10.0f);
        
        float final_gain = max(target_gain, avg_gain);
        if(final_gain > AUTO_GAIN_MAX) final_gain = AUTO_GAIN_MAX;
        if(final_gain < 2.0) final_gain = 2.0;
        
        audio_proc.auto_gain = audio_proc.auto_gain * 0.5 + final_gain * 0.5;
        
        for(size_t i = 0; i < len; i++) {
            int32_t amplified = samples[i] * audio_proc.auto_gain;
            
            if(amplified > 30000) amplified = 30000 + (amplified - 30000) / 2;
            if(amplified < -30000) amplified = -30000 + (amplified + 30000) / 2;
            if(amplified > 32767) amplified = 32767;
            if(amplified < -32768) amplified = -32768;
            
            samples[i] = amplified;
        }
    }
}

bool detect_speech_advanced(int16_t* samples, size_t len) {
    float frame_energy = 0;
    float frame_peak = 0;
    float weighted_energy = 0;
    
    for(size_t i = 0; i < len; i++) {
        float abs_val = abs(samples[i]);
        frame_energy += abs_val;
        if(abs_val > frame_peak) frame_peak = abs_val;
        weighted_energy += abs_val * abs_val / 1000.0;
    }
    
    frame_energy /= len;
    weighted_energy /= len;
    
    float combined_energy = frame_energy * 0.4 + weighted_energy * 0.3 + frame_peak * 0.3;
    
    audio_proc.energy_history[audio_proc.history_index] = combined_energy;
    audio_proc.history_index = (audio_proc.history_index + 1) % ENERGY_HISTORY_SIZE;
    
    float avg_energy = 0;
    for(int i = 0; i < ENERGY_HISTORY_SIZE; i++) {
        avg_energy += audio_proc.energy_history[i];
    }
    avg_energy /= ENERGY_HISTORY_SIZE;
    audio_proc.avg_energy = avg_energy;
    
    if(combined_energy > audio_proc.peak_energy) {
        audio_proc.peak_energy = combined_energy;
    } else {
        audio_proc.peak_energy *= 0.98;
    }
    
    if(!audio_proc.is_speech && combined_energy < audio_proc.peak_energy * 0.3) {
        update_noise_profile(combined_energy);
    }
    
    float vad_threshold = noise_mgr.noise_floor * VAD_THRESHOLD_MULTIPLIER;
    if(vad_threshold < MIN_VOICE_ENERGY) vad_threshold = MIN_VOICE_ENERGY;
    
    bool is_speech_now = (
        (combined_energy > vad_threshold) || 
        (frame_peak > vad_threshold * 3) ||
        (combined_energy > audio_proc.avg_energy * 1.5 && audio_proc.avg_energy > 0)
    );
    
    if(is_speech_now) {
        audio_proc.voice_frames = min(audio_proc.voice_frames + 1, 10);
        noise_mgr.silence_counter = 0;
    } else {
        noise_mgr.silence_counter++;
        if(noise_mgr.silence_counter > SILENCE_FRAMES_THRESHOLD) {
            audio_proc.voice_frames = 0;
        }
    }
    
    audio_proc.is_speech = (audio_proc.voice_frames > 1);
    
    return audio_proc.is_speech;
}

void update_noise_profile(float current_level) {
    noise_mgr.noise_profile[noise_mgr.profile_index] = current_level;
    noise_mgr.profile_index = (noise_mgr.profile_index + 1) % NOISE_PROFILE_SIZE;
    
    vector<float> sorted(noise_mgr.noise_profile, noise_mgr.noise_profile + NOISE_PROFILE_SIZE);
    sort(sorted.begin(), sorted.end());
    
    noise_mgr.min_noise = sorted[NOISE_PROFILE_SIZE / 4];
    noise_mgr.max_noise = sorted[NOISE_PROFILE_SIZE * 3 / 4];
    
    noise_mgr.noise_floor = noise_mgr.noise_floor * 0.7 + noise_mgr.min_noise * 0.3;
    if(noise_mgr.noise_floor < 20) noise_mgr.noise_floor = 20;
}

float calculate_confidence_threshold(void) {
    float threshold = MIN_CONFIDENCE;
    
    if(noise_mgr.noise_floor < 150) {
        threshold = 0.40;
    } else if(noise_mgr.noise_floor < 400) {
        threshold = 0.45;
    } else if(noise_mgr.noise_floor < 800) {
        threshold = 0.50;
    } else {
        threshold = 0.60;
    }
    
    return threshold;
}

// ==================== XỬ LÝ LED ====================
void update_leds() {
    // LED hệ thống LUÔN SÁNG (active LOW)
    digitalWrite(LED_BUILTIN, LOW);
    
    // LED điều khiển
    digitalWrite(LED_LIGHT, light_state ? HIGH : LOW);
    digitalWrite(LED_DOOR, door_state ? HIGH : LOW);
    
    // LED NOISE: Sáng khi cả 2 LED kia TẮT
    if(!light_state && !door_state) {
        digitalWrite(LED_NOISE, HIGH);
    } else {
        digitalWrite(LED_NOISE, LOW);
    }
}

// ==================== XỬ LÝ WAKE-UP LED ====================
void update_wakeup_led() {
    if(system_awake) {
        // Hệ thống được đánh thức: LED wakeup nháy liên tục với delay 300ms
        unsigned long current_time = millis();
        if(current_time - last_blink_time >= WAKEUP_LED_BLINK_MS) {
            wakeup_led_state = !wakeup_led_state;
            digitalWrite(WAKEUP_GPIO, wakeup_led_state ? HIGH : LOW);
            last_blink_time = current_time;
        }
    } else {
        // Hệ thống chưa được đánh thức: LED wakeup tắt
        digitalWrite(WAKEUP_GPIO, LOW);
        wakeup_led_state = false;
    }
}

// ==================== XỬ LÝ HỆ THỐNG WAKE-UP ====================
void handle_wakeup_system(const char* command, float confidence) {
    // Kiểm tra nếu là lệnh "xin chào" (wake-up)
    if(strcmp(command, "xin chào") == 0) {
        system_awake = true;
        last_command_time = millis();
        Serial.printf("=== HỆ THỐNG ĐƯỢC ĐÁNH THỨC ===\n");
    }
    // Nếu hệ thống đã được đánh thức và có lệnh điều khiển
    else if(system_awake) {
        last_command_time = millis();  // Cập nhật thời gian hoạt động
        
        // Xử lý các lệnh điều khiển
        if (strcmp(command, "bật đèn") == 0) {
            light_state = true;
        }
        else if (strcmp(command, "tắt đèn") == 0) {
            light_state = false;
        }
        else if (strcmp(command, "mở cửa") == 0) {
            door_state = true;
        }
        else if (strcmp(command, "đóng cửa") == 0) {
            door_state = false;
        }
        
        update_leds();  // Cập nhật LED dựa trên trạng thái mới
        print_simple_status(command, confidence);
        
        // LED báo nhận lệnh nhấp nháy
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
    }
}

// FIX: Sửa lỗi multi-character character constant
void print_simple_status(const char* keyword, float conf) {
    static int counter = 0;
    counter++;
    
    if(conf > HIGH_CONFIDENCE) {
        // Sử dụng dấu ✓ (check mark) đúng cách
        Serial.printf("✓ [%d] %s (%d%%)\n", counter, keyword, (int)(conf * 100));
    } else {
        Serial.printf("  [%d] %s (%d%%)\n", counter, keyword, (int)(conf * 100));
    }
}

// ==================== AUDIO INFERENCE ====================
static void audio_inference_callback(uint32_t n_bytes) {
    feed_watchdog();
    for (int i = 0; i < n_bytes >> 1; i++) {
        inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];
        if (inference.buf_count >= inference.n_samples) {
            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

static void capture_samples(void *arg) {
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());
    const int32_t i2s_bytes_to_read = (uint32_t)arg;
    
    while (record_status) {
        esp_task_wdt_reset();
        size_t bytes_read = i2s_bytes_to_read;
        
        esp_i2s::i2s_read(esp_i2s::I2S_NUM_0, (void *)sampleBuffer, 
                          i2s_bytes_to_read, &bytes_read, 100);

        if (bytes_read > 0) {
            process_audio_advanced(sampleBuffer, i2s_bytes_to_read / 2);
            detect_speech_advanced(sampleBuffer, i2s_bytes_to_read / 2);
            
            if (record_status) {
                audio_inference_callback(i2s_bytes_to_read);
            }
        }
        taskYIELD();
    }
    
    esp_task_wdt_delete(xTaskGetCurrentTaskHandle());
    vTaskDelete(NULL);
}

static bool microphone_inference_start(uint32_t n_samples) {
    inference.buffers[0] = (signed short *)malloc(n_samples * sizeof(signed short));
    if (!inference.buffers[0]) return false;
    
    inference.buffers[1] = (signed short *)malloc(n_samples * sizeof(signed short));
    if (!inference.buffers[1]) {
        free(inference.buffers[0]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;
    
    record_status = true;
    
    xTaskCreatePinnedToCore(controll_task, "Controll", 4096, NULL, 8, NULL, 1);
    xTaskCreatePinnedToCore(capture_samples, "CaptureSamples", 8192, 
                           (void *)sample_buffer_size, 10, NULL, 0);
    return true;
}

static bool microphone_inference_record(void) {
    int timeout = 0;
    while (!inference.buf_ready && timeout < 500) {
        delay(1);
        timeout++;
    }
    if (inference.buf_ready) {
        inference.buf_ready = 0;
        return true;
    }
    return false;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
    return 0;
}

// ==================== TASK ĐIỀU KHIỂN ====================
static void controll_task(void *arg) {
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());
    
    while (1) {
        esp_task_wdt_reset();
        
        // Kiểm tra timeout của hệ thống
        if(system_awake) {
            unsigned long current_time = millis();
            if(current_time - last_command_time >= WAKEUP_TIMEOUT_MS) {
                system_awake = false;
                Serial.printf("=== HỆ THỐNG NGỦ (TIMEOUT 5s) ===\n");
            }
        }
        
        update_leds();
        update_wakeup_led();  // Cập nhật LED wakeup
        taskYIELD();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ==================== SETUP & LOOP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== KWS SIÊU NHẠY VỚI WAKE-UP ===");
    
    setup_watchdog();
    
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LED_NOISE, OUTPUT);
    pinMode(LED_LIGHT, OUTPUT);
    pinMode(LED_DOOR, OUTPUT);
    pinMode(WAKEUP_GPIO, OUTPUT);  // Cấu hình GPIO6 cho wake-up LED
    
    // LED Builtin luôn sáng ngay từ đầu
    digitalWrite(LED_BUILTIN, LOW);  // Bật LED (active LOW)
    
    // Khởi tạo trạng thái LED
    light_state = false;
    door_state = false;
    system_awake = false;  // Hệ thống bắt đầu ở trạng thái ngủ
    last_command_time = 0;
    
    update_leds();
    update_wakeup_led();
    
    boot_indicator();
    
    if (!init_i2s_with_retry()) {
        Serial.println("I2S FAIL");
    }
    
    ledEventGroup = xEventGroupCreate();
    init_audio_processing();
    
    run_classifier_init();
    
    delay(500);
    microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT);
    
    Serial.println("READY! Nói 'xin chào' để đánh thức hệ thống!\n");
}

void loop() {
    feed_watchdog();
    
    bool m = microphone_inference_record();
    if (!m) {
        delay(5);
        return;
    }
    
    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = {0};

    if (run_classifier_continuous(&signal, &result, debug_nn) != EI_IMPULSE_OK) {
        return;
    }
    
    float best_value = 0;
    int best_idx = -1;
    
    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > best_value) {
            best_value = result.classification[i].value;
            best_idx = i;
        }
    }
    
    float threshold = calculate_confidence_threshold();
    if (best_idx >= 0 && best_value >= threshold) {
        const char* detected = result.classification[best_idx].label;
        
        if (strcmp(detected, "noise") != 0 && 
            strcmp(detected, "_unknown") != 0 &&
            strcmp(detected, "unknown") != 0) {
            
            handle_wakeup_system(detected, best_value);
        }
    }
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif