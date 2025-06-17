#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_pdm.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "u8g2.h"
#include "kiss_fft.h"

// Configuration
#define SAMPLE_RATE      16000
#define I2S_MIC_DIN      GPIO_NUM_4   // PDM Data pin
#define I2S_MIC_CLK      GPIO_NUM_0   // PDM Clock pin
#define BUFFER_SIZE      1024         // Must be power of 2 for FFT
#define FFT_SIZE         512          // FFT size (half of buffer for real FFT)

// OLED I2C pins
#define OLED_SDA         GPIO_NUM_21
#define OLED_SCL         GPIO_NUM_22
#define OLED_ADDRESS     0x3C

// Display parameters
#define DISPLAY_WIDTH    128
#define DISPLAY_HEIGHT   64
#define SPECTRUM_HEIGHT  40
#define SPECTRUM_BARS    32           // Number of frequency bars to display

// Audio processing parameters
#define NOISE_GATE_THRESHOLD    100
#define VOICE_THRESHOLD         700
#define SMOOTHING_FACTOR        0.3f  // For spectrum smoothing
#define CALIBRATION_SAMPLES     50

static const char *TAG = "PDM_FFT_VISUALIZER";

// OLED Display
u8g2_t u8g2;

// Audio processing variables
static float noise_floor = 0.0f;
static float smoothed_amplitude = 0.0f;
static bool is_calibrated = false;
static int calibration_counter = 0;

// FFT variables
static kiss_fft_cfg fft_cfg;
static kiss_fft_cpx *fft_in;
static kiss_fft_cpx *fft_out;
static float *spectrum_data;
static float *prev_spectrum;  // For smoothing
static float *window_func;    // Hanning window

// Frequency bin mapping for display bars
static int freq_bins[SPECTRUM_BARS];

// GPIO and Delay callbacks for u8g2
extern "C" {
uint8_t u8x8_gpio_and_delay_esp32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            gpio_set_direction(OLED_SDA, GPIO_MODE_OUTPUT_OD);
            gpio_set_direction(OLED_SCL, GPIO_MODE_OUTPUT_OD);
            gpio_set_pull_mode(OLED_SDA, GPIO_PULLUP_ONLY);
            gpio_set_pull_mode(OLED_SCL, GPIO_PULLUP_ONLY);
            break;
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(pdMS_TO_TICKS(arg_int));
            break;
        case U8X8_MSG_DELAY_10MICRO:
            esp_rom_delay_us(10);
            break;
        case U8X8_MSG_DELAY_100NANO:
            esp_rom_delay_us(1);
            break;
        case U8X8_MSG_GPIO_I2C_CLOCK:
            gpio_set_level(OLED_SCL, arg_int);
            break;
        case U8X8_MSG_GPIO_I2C_DATA:
            gpio_set_level(OLED_SDA, arg_int);
            break;
        default:
            break;
    }
    return 1;
}
}

// Initialize FFT components
void init_fft() {
    ESP_LOGI(TAG, "Initializing FFT...");
    
    // Allocate FFT structures
    fft_cfg = kiss_fft_alloc(FFT_SIZE, 0, NULL, NULL);
    fft_in = (kiss_fft_cpx*)malloc(FFT_SIZE * sizeof(kiss_fft_cpx));
    fft_out = (kiss_fft_cpx*)malloc(FFT_SIZE * sizeof(kiss_fft_cpx));
    spectrum_data = (float*)malloc(FFT_SIZE/2 * sizeof(float));
    prev_spectrum = (float*)malloc(FFT_SIZE/2 * sizeof(float));
    window_func = (float*)malloc(FFT_SIZE * sizeof(float));
    
    // Initialize spectrum arrays
    memset(prev_spectrum, 0, FFT_SIZE/2 * sizeof(float));
    
    // Generate Hanning window function
    for (int i = 0; i < FFT_SIZE; i++) {
        window_func[i] = 0.5 * (1.0 - cos(2.0 * M_PI * i / (FFT_SIZE - 1)));
    }
    
    // Initialize frequency bin mapping for logarithmic distribution
    float freq_per_bin = (float)SAMPLE_RATE / (2.0 * FFT_SIZE);
    for (int i = 0; i < SPECTRUM_BARS; i++) {
        // Logarithmic frequency mapping
        float freq = 50.0 * pow(10.0, (float)i / SPECTRUM_BARS * 2.5); // 50Hz to ~8kHz
        freq_bins[i] = (int)(freq / freq_per_bin);
        if (freq_bins[i] >= FFT_SIZE/2) freq_bins[i] = FFT_SIZE/2 - 1;
    }
    
    ESP_LOGI(TAG, "FFT initialized successfully");
}

// OLED Setup
void setup_display() {
    ESP_LOGI(TAG, "Setting up OLED display...");
    
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
        &u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_esp32);
    
    u8g2_SetI2CAddress(&u8g2, OLED_ADDRESS << 1);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    
    ESP_LOGI(TAG, "OLED display initialized");
}

// Perform FFT analysis
void perform_fft_analysis(int16_t *audio_buffer, int samples) {
    // Apply windowing and prepare FFT input
    for (int i = 0; i < FFT_SIZE && i < samples; i++) {
        fft_in[i].r = (float)audio_buffer[i] * window_func[i];
        fft_in[i].i = 0.0;
    }
    
    // Zero pad if necessary
    for (int i = samples; i < FFT_SIZE; i++) {
        fft_in[i].r = 0.0;
        fft_in[i].i = 0.0;
    }
    
    // Perform FFT
    kiss_fft(fft_cfg, fft_in, fft_out);
    
    // Calculate magnitude spectrum
    for (int i = 0; i < FFT_SIZE/2; i++) {
        float magnitude = sqrt(fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i);
        
        // Apply smoothing
        spectrum_data[i] = SMOOTHING_FACTOR * magnitude + (1.0f - SMOOTHING_FACTOR) * prev_spectrum[i];
        prev_spectrum[i] = spectrum_data[i];
    }
}

// Display spectrum visualization
void display_spectrum_visualization(float raw_amplitude, bool voice_detected, bool calibrating) {
    u8g2_ClearBuffer(&u8g2);
    
    // Title
    u8g2_SetFont(&u8g2, u8g2_font_4x6_tr);
    u8g2_DrawStr(&u8g2, 2, 8, "ESP32 FFT Audio Visualizer");
    
    // Status line
    if (calibrating) {
        char cal_str[32];
        snprintf(cal_str, sizeof(cal_str), "Cal: %d%%", (calibration_counter * 100) / CALIBRATION_SAMPLES);
        u8g2_DrawStr(&u8g2, 2, 16, cal_str);
    } else {
        u8g2_DrawStr(&u8g2, 2, 16, voice_detected ? "VOICE" : "Listen");
    }
    
    // Amplitude indicator
    char amp_str[16];
    snprintf(amp_str, sizeof(amp_str), "%.0f", raw_amplitude);
    u8g2_DrawStr(&u8g2, 100, 16, amp_str);
    
    if (!calibrating && is_calibrated) {
        // Draw spectrum bars
        int bar_width = DISPLAY_WIDTH / SPECTRUM_BARS;
        int spectrum_start_y = 20;
        
        for (int i = 0; i < SPECTRUM_BARS; i++) {
            int bin_start = (i == 0) ? 1 : freq_bins[i-1]; // Skip DC component
            int bin_end = freq_bins[i];
            
            // Average magnitude across frequency bins
            float avg_magnitude = 0.0;
            int bin_count = 0;
            for (int j = bin_start; j <= bin_end && j < FFT_SIZE/2; j++) {
                avg_magnitude += spectrum_data[j];
                bin_count++;
            }
            if (bin_count > 0) avg_magnitude /= bin_count;
            
            // Scale and limit bar height
            int bar_height = (int)(avg_magnitude * SPECTRUM_HEIGHT / 5000.0);
            if (bar_height > SPECTRUM_HEIGHT) bar_height = SPECTRUM_HEIGHT;
            if (bar_height < 1 && avg_magnitude > 10) bar_height = 1;
            
            // Draw bar
            int x = i * bar_width;
            int y = spectrum_start_y + SPECTRUM_HEIGHT - bar_height;
            
            if (bar_height > 0) {
                u8g2_DrawBox(&u8g2, x, y, bar_width - 1, bar_height);
            }
        }
        
        // Draw spectrum frame
        u8g2_DrawFrame(&u8g2, 0, spectrum_start_y, DISPLAY_WIDTH, SPECTRUM_HEIGHT);
        
        // Frequency labels
        u8g2_SetFont(&u8g2, u8g2_font_4x6_tr);
        u8g2_DrawStr(&u8g2, 2, 63, "50Hz");
        u8g2_DrawStr(&u8g2, 50, 63, "500Hz");
        u8g2_DrawStr(&u8g2, 100, 63, "5kHz");
    } else {
        // Show calibration progress
        u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
        u8g2_DrawStr(&u8g2, 20, 35, "Calibrating...");
        
        // Progress bar
        int progress_width = (calibration_counter * 80) / CALIBRATION_SAMPLES;
        u8g2_DrawFrame(&u8g2, 24, 40, 80, 8);
        if (progress_width > 0) {
            u8g2_DrawBox(&u8g2, 25, 41, progress_width, 6);
        }
    }
    
    u8g2_SendBuffer(&u8g2);
}

// Initialize PDM I2S
i2s_chan_handle_t init_pdm_microphone() {
    ESP_LOGI(TAG, "Initializing PDM microphone...");
    
    i2s_chan_handle_t rx_handle;
    
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));
    
    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = I2S_MIC_CLK,
            .din = I2S_MIC_DIN,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };
    
    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_rx_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
    
    ESP_LOGI(TAG, "PDM microphone initialized successfully");
    return rx_handle;
}

// Remove DC offset
void remove_dc_offset(int16_t *buffer, int samples) {
    int32_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += buffer[i];
    }
    int16_t dc_offset = sum / samples;
    
    for (int i = 0; i < samples; i++) {
        buffer[i] -= dc_offset;
    }
}

// High-pass filter
void high_pass_filter(int16_t *buffer, int samples) {
    static int16_t prev_input = 0;
    static int16_t prev_output = 0;
    const float alpha = 0.95f;
    
    for (int i = 0; i < samples; i++) {
        int16_t current_input = buffer[i];
        int16_t current_output = (int16_t)(alpha * (prev_output + current_input - prev_input));
        buffer[i] = current_output;
        prev_input = current_input;
        prev_output = current_output;
    }
}

// Calculate RMS amplitude
float calculate_rms_amplitude(int16_t *buffer, int samples) {
    int64_t sum_squares = 0;
    for (int i = 0; i < samples; i++) {
        sum_squares += (int64_t)buffer[i] * buffer[i];
    }
    return sqrt((float)sum_squares / samples);
}

// Calibrate noise floor
void calibrate_noise_floor(float amplitude) {
    if (calibration_counter < CALIBRATION_SAMPLES) {
        noise_floor = (noise_floor * calibration_counter + amplitude) / (calibration_counter + 1);
        calibration_counter++;
        if (calibration_counter >= CALIBRATION_SAMPLES) {
            is_calibrated = true;
            noise_floor *= 1.2f;
            ESP_LOGI(TAG, "Calibration complete. Noise floor: %.2f", noise_floor);
        }
    }
}

// Enhanced voice detection
bool detect_voice(float amplitude) {
    if (!is_calibrated) return false;
    
    smoothed_amplitude = SMOOTHING_FACTOR * amplitude + (1.0f - SMOOTHING_FACTOR) * smoothed_amplitude;
    float signal_above_noise = smoothed_amplitude - noise_floor;
    
    return (signal_above_noise > NOISE_GATE_THRESHOLD) && (smoothed_amplitude > VOICE_THRESHOLD);
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "Starting ESP32 FFT Audio Visualizer...");
    
    // Initialize display
    setup_display();
    
    // Show initialization message
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(&u8g2, 10, 30, "Initializing...");
    u8g2_SendBuffer(&u8g2);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Initialize FFT
    init_fft();
    
    // Initialize PDM microphone
    i2s_chan_handle_t rx_handle = init_pdm_microphone();
    
    // Allocate audio buffer
    int16_t *audio_buffer = (int16_t *)malloc(BUFFER_SIZE * sizeof(int16_t));
    if (!audio_buffer) {
        ESP_LOGE(TAG, "Failed to allocate audio buffer");
        return;
    }
    
    size_t bytes_read;
    ESP_LOGI(TAG, "Starting FFT audio visualization...");
    
    while (1) {
        // Read audio data
        esp_err_t ret = i2s_channel_read(rx_handle, audio_buffer, 
                                        BUFFER_SIZE * sizeof(int16_t), 
                                        &bytes_read, pdMS_TO_TICKS(1000));
        
        if (ret == ESP_OK && bytes_read > 0) {
            int samples = bytes_read / sizeof(int16_t);
            
            // Calculate raw amplitude
            float raw_amplitude = calculate_rms_amplitude(audio_buffer, samples);
            
            // Apply signal processing
            remove_dc_offset(audio_buffer, samples);
            high_pass_filter(audio_buffer, samples);
            
            // Calculate processed amplitude
            float processed_amplitude = calculate_rms_amplitude(audio_buffer, samples);
            
            // Calibrate noise floor
            if (!is_calibrated) {
                calibrate_noise_floor(processed_amplitude);
            }
            
            // Perform FFT analysis
            if (is_calibrated && samples >= FFT_SIZE) {
                perform_fft_analysis(audio_buffer, samples);
            }
            
            // Detect voice
            bool voice_detected = detect_voice(processed_amplitude);
            
            // Update display with spectrum visualization
            display_spectrum_visualization(raw_amplitude, voice_detected, !is_calibrated);
            
            // Periodic logging
            static int log_counter = 0;
            if (++log_counter >= 50) {
                ESP_LOGI(TAG, "Amplitude: %.0f, Voice: %s, FFT: %s", 
                        raw_amplitude, voice_detected ? "YES" : "NO",
                        is_calibrated ? "ACTIVE" : "CALIBRATING");
                log_counter = 0;
            }
        } else {
            ESP_LOGE(TAG, "I2S read error: %s", esp_err_to_name(ret));
            
            u8g2_ClearBuffer(&u8g2);
            u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
            u8g2_DrawStr(&u8g2, 10, 30, "I2S Error!");
            u8g2_SendBuffer(&u8g2);
        }
        
        vTaskDelay(pdMS_TO_TICKS(is_calibrated ? 100 : 200));
    }
    
    // Cleanup
    free(audio_buffer);
    free(fft_in);
    free(fft_out);
    free(spectrum_data);
    free(prev_spectrum);
    free(window_func);
    kiss_fft_free(fft_cfg);
    i2s_channel_disable(rx_handle);
    i2s_del_channel(rx_handle);
}